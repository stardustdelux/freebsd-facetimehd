/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * PCI attachment, interrupts and module glue.  Ported from the Linux
 * facetimehd driver:
 *   Copyright (C) 2014 Patrik Jakobsson (patrik.r.jakobsson@gmail.com)
 *   Copyright (C) 2015 Sven Schnelle <svens@stackframe.org>
 *
 * Brings up the S2 DDR, boots the ISP firmware, queries the sensor and
 * captures frames through /dev/fthdN.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "fthd_var.h"

MALLOC_DEFINE(M_FTHD, "fthd", "FacetimeHD camera driver");

int fthd_debug = 0;
static SYSCTL_NODE(_hw, OID_AUTO, fthd, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "FacetimeHD camera driver");
SYSCTL_INT(_hw_fthd, OID_AUTO, debug, CTLFLAG_RWTUN, &fthd_debug, 0,
    "Enable debug messages");

#define FTHD_VENDOR	0x14e4
#define FTHD_DEVICE	0x1570

/*
 * Copy to and from S2 memory.  Uses 32-bit accesses for the aligned bulk
 * of the copy (the firmware image is over a megabyte).
 */
int
fthd_s2_memcpy_toio(struct fthd_softc *sc, uint32_t off, const void *buf,
    size_t len)
{
	const uint8_t *p = buf;
	size_t words;

	if (off > sc->s2_mem_len || len > sc->s2_mem_len - off) {
		device_printf(sc->dev, "S2 MEM copy out of range (%x+%zx)\n",
		    off, len);
		return (EFAULT);
	}

	if ((off & 3) == 0 && ((uintptr_t)p & 3) == 0) {
		words = len / 4;
		bus_write_region_4(sc->s2_mem, off, (const uint32_t *)p, words);
		off += words * 4;
		p += words * 4;
		len -= words * 4;
	}
	if (len > 0)
		bus_write_region_1(sc->s2_mem, off, p, len);
	return (0);
}

int
fthd_s2_memcpy_fromio(struct fthd_softc *sc, void *buf, uint32_t off,
    size_t len)
{
	uint8_t *p = buf;
	size_t words;

	if (off > sc->s2_mem_len || len > sc->s2_mem_len - off) {
		device_printf(sc->dev, "S2 MEM copy out of range (%x+%zx)\n",
		    off, len);
		memset(buf, 0, len);
		return (EFAULT);
	}

	if ((off & 3) == 0 && ((uintptr_t)p & 3) == 0) {
		words = len / 4;
		bus_read_region_4(sc->s2_mem, off, (uint32_t *)p, words);
		off += words * 4;
		p += words * 4;
		len -= words * 4;
	}
	if (len > 0)
		bus_read_region_1(sc->s2_mem, off, p, len);
	return (0);
}

/*
 * Config offsets 0x90/0x94 are written around interrupt handling by the
 * Linux driver (and by Apple's).  Their meaning is unknown; we copy the
 * sequence as is.
 */
void
fthd_irq_enable(struct fthd_softc *sc)
{
	fthd_isp_reg_write(sc, 0xf8, ISP_IRQ_ENABLE);
	pci_write_config(sc->dev, 0x94, 0x200, 4);
}

void
fthd_irq_disable(struct fthd_softc *sc)
{
	fthd_isp_reg_write(sc, 0, ISP_IRQ_ENABLE);
	pci_write_config(sc->dev, 0x94, 0, 4);
}

static int
fthd_intr_filter(void *arg)
{
	struct fthd_softc *sc = arg;

	if (!(bus_read_4(sc->isp_io, ISP_IRQ_STATUS) & 0xf0))
		return (FILTER_STRAY);

	taskqueue_enqueue(sc->tq, &sc->irq_task);
	return (FILTER_HANDLED);
}

static void
fthd_intr_task(void *arg, int pending __unused)
{
	struct fthd_softc *sc = arg;
	struct fw_channel *chan;
	uint32_t status;
	int i, loops;

	for (loops = 0; loops < 500; loops++) {
		status = fthd_isp_reg_read(sc, ISP_IRQ_STATUS);
		if (!(status & 0xf0))
			break;

		pci_write_config(sc->dev, 0x94, 0, 4);
		mtx_lock(&sc->mtx);
		fthd_isp_reg_write(sc, status, ISP_IRQ_CLEAR);
		mtx_unlock(&sc->mtx);
		pci_write_config(sc->dev, 0x90, 0x200, 4);

		for (i = 0; i < sc->num_channels; i++) {
			chan = &sc->channels[i];
			if ((0x10 << chan->source) & status)
				fthd_handle_channel(sc, chan);
		}
	}

	if (loops >= 500) {
		device_printf(sc->dev, "Interrupt stuck, disabling\n");
		fthd_irq_disable(sc);
		return;
	}
	pci_write_config(sc->dev, 0x94, 0x200, 4);
}

/*
 * ASPM must be off or the device hangs while streaming.  The link is
 * controlled from both ends, so clear it on the upstream bridge too.
 */
static void
fthd_disable_aspm(device_t dev)
{
	device_t bridge;
	uint32_t mask = PCIEM_LINK_CTL_ASPMC | PCIEM_LINK_CTL_ECPM;

	pcie_adjust_config(dev, PCIER_LINK_CTL, mask, 0, 2);

	bridge = device_get_parent(device_get_parent(dev));
	if (bridge != NULL && pci_find_cap(bridge, PCIY_EXPRESS, NULL) == 0)
		pcie_adjust_config(bridge, PCIER_LINK_CTL, PCIEM_LINK_CTL_ASPMC,
		    0, 2);
}

static struct resource *
fthd_alloc_bar(struct fthd_softc *sc, int rid, uint32_t *lenp)
{
	struct resource *res;

	res = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (res == NULL) {
		device_printf(sc->dev, "Failed to map BAR at 0x%x\n", rid);
		return (NULL);
	}
	*lenp = rman_get_size(res);
	return (res);
}

static int
fthd_probe(device_t dev)
{
	if (pci_get_vendor(dev) != FTHD_VENDOR ||
	    pci_get_device(dev) != FTHD_DEVICE)
		return (ENXIO);

	device_set_desc(dev, "FacetimeHD camera (Broadcom 1570)");
	return (BUS_PROBE_DEFAULT);
}

static int fthd_detach(device_t dev);

static int
fthd_attach(device_t dev)
{
	struct fthd_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	int count, error;

	sc->dev = dev;
	sc->ddr_model = 4;
	sc->ddr_speed = 450;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	mtx_init(&sc->buf_mtx, "fthd buffers", NULL, MTX_DEF);
	sx_init(&sc->cmd_sx, "fthd cmd");
	sx_init(&sc->dev_sx, "fthd dev");
	TAILQ_INIT(&sc->shared_mallocs);
	TASK_INIT(&sc->irq_task, 0, fthd_intr_task, sc);

	sc->tq = taskqueue_create("fthd", M_WAITOK, taskqueue_thread_enqueue,
	    &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_NET, "%s taskq",
	    device_get_nameunit(dev));

	pci_enable_busmaster(dev);
	fthd_disable_aspm(dev);

	sc->s2_io = fthd_alloc_bar(sc, FTHD_PCI_S2_IO, &sc->s2_io_len);
	sc->s2_mem = fthd_alloc_bar(sc, FTHD_PCI_S2_MEM, &sc->s2_mem_len);
	sc->isp_io = fthd_alloc_bar(sc, FTHD_PCI_ISP_IO, &sc->isp_io_len);
	if (sc->s2_io == NULL || sc->s2_mem == NULL || sc->isp_io == NULL) {
		error = ENXIO;
		goto fail;
	}
	device_printf(dev, "S2 IO %uKB, S2 MEM %uMB, ISP IO %uKB\n",
	    sc->s2_io_len / 1024, sc->s2_mem_len / (1024 * 1024),
	    sc->isp_io_len / 1024);

	if (sc->s2_mem_len <= FTHD_MEM_FW_SIZE || sc->isp_io_len < 0xc4000) {
		device_printf(dev, "Unexpected BAR sizes\n");
		error = ENXIO;
		goto fail;
	}

	count = 1;
	if (pci_alloc_msi(dev, &count) == 0) {
		sc->msi = 1;
		sc->irq_rid = 1;
	} else {
		device_printf(dev, "MSI unavailable, using INTx\n");
		sc->irq_rid = 0;
	}
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE | (sc->msi ? 0 : RF_SHAREABLE));
	if (sc->irq == NULL) {
		device_printf(dev, "Failed to allocate interrupt\n");
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_AV | INTR_MPSAFE,
	    fthd_intr_filter, NULL, sc, &sc->irq_cookie);
	if (error) {
		device_printf(dev, "Failed to set up interrupt\n");
		goto fail;
	}

	/* Clears the camera IOMMU, which must happen before DDR bring-up */
	error = fthd_buffer_init(sc);
	if (error)
		goto fail;

	error = fthd_hw_init(sc);
	if (error)
		goto fail;

	error = fthd_firmware_start(sc);
	if (error) {
		device_printf(dev, "Firmware start failed (%d)\n", error);
		goto fail;
	}

	fthd_format_default(sc);

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "sensor_id0",
	    CTLFLAG_RD, &sc->sensor_id0, 0, "Sensor ID 0");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "sensor_id1",
	    CTLFLAG_RD, &sc->sensor_id1, 0, "Sensor ID 1");
	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "width",
	    CTLFLAG_RD, &sc->sensor_width, 0, "Sensor native width");
	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "height",
	    CTLFLAG_RD, &sc->sensor_height, 0, "Sensor native height");
	SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "frames",
	    CTLFLAG_RD, &sc->frames_total, 0, "Frames captured");
	SYSCTL_ADD_U64(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "drops",
	    CTLFLAG_RD, &sc->drops_total, 0, "Frames dropped (reader too slow)");

	error = fthd_cdev_create(sc);
	if (error) {
		device_printf(dev, "Failed to create /dev/fthd%d\n",
		    device_get_unit(dev));
		goto fail;
	}

	device_printf(dev, "Camera is up\n");
	return (0);

fail:
	fthd_detach(dev);
	return (error);
}

static int
fthd_detach(device_t dev)
{
	struct fthd_softc *sc = device_get_softc(dev);

	fthd_cdev_destroy(sc);

	if (sc->isp_io != NULL) {
		fthd_firmware_stop(sc);
		fthd_hw_deinit(sc);
	}

	if (sc->irq_cookie != NULL) {
		bus_teardown_intr(dev, sc->irq, sc->irq_cookie);
		sc->irq_cookie = NULL;
	}
	if (sc->tq != NULL) {
		taskqueue_drain(sc->tq, &sc->irq_task);
		taskqueue_free(sc->tq);
		sc->tq = NULL;
	}

	fthd_buffer_uninit(sc);
	if (sc->isp_io != NULL)
		isp_uninit(sc);

	if (sc->irq != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
		sc->irq = NULL;
	}
	if (sc->msi) {
		pci_release_msi(dev);
		sc->msi = 0;
	}

	if (sc->isp_io != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, FTHD_PCI_ISP_IO,
		    sc->isp_io);
	if (sc->s2_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, FTHD_PCI_S2_MEM,
		    sc->s2_mem);
	if (sc->s2_io != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, FTHD_PCI_S2_IO,
		    sc->s2_io);
	sc->isp_io = sc->s2_mem = sc->s2_io = NULL;

	pci_disable_busmaster(dev);
	sx_destroy(&sc->dev_sx);
	sx_destroy(&sc->cmd_sx);
	mtx_destroy(&sc->buf_mtx);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t fthd_methods[] = {
	DEVMETHOD(device_probe,		fthd_probe),
	DEVMETHOD(device_attach,	fthd_attach),
	DEVMETHOD(device_detach,	fthd_detach),
	DEVMETHOD_END
};

static driver_t fthd_driver = {
	"fthd",
	fthd_methods,
	sizeof(struct fthd_softc)
};

DRIVER_MODULE(fthd, pci, fthd_driver, NULL, NULL);
MODULE_DEPEND(fthd, pci, 1, 1, 1);
MODULE_DEPEND(fthd, firmware, 1, 1, 1);
MODULE_VERSION(fthd, 1);
