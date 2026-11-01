/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * ISP firmware loading, boot and command interface, from the Linux
 * facetimehd driver:
 *   Copyright (C) 2014 Patrik Jakobsson (patrik.r.jakobsson@gmail.com)
 *   Copyright (C) 2015 Sven Schnelle <svens@stackframe.org>
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/sx.h>
#include <sys/vmem.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/pci/pcivar.h>

#include "fthd_var.h"
#include "fthd_isp.h"

MALLOC_DECLARE(M_FTHD);

#define FTHD_FIRMWARE		"facetimehd/firmware.bin"

/* Sleep for ms milliseconds; only used where sleeping is allowed */
static void
fthd_msleep(int ms)
{
	pause_sbt("fthd", SBT_1MS * ms, 0, 0);
}

/*
 * S2 memory management.  The firmware is pinned at offset 0 (the ISP
 * executes it from there); everything else comes from a vmem arena that
 * covers the rest of the BAR.
 */
static int
isp_mem_init(struct fthd_softc *sc)
{
	sc->mem = vmem_create("fthd s2", FTHD_MEM_FW_SIZE,
	    sc->s2_mem_len - FTHD_MEM_FW_SIZE, PAGE_SIZE, 0, M_WAITOK);
	if (sc->mem == NULL)
		return (ENOMEM);

	sc->firmware = malloc(sizeof(*sc->firmware), M_FTHD, M_WAITOK | M_ZERO);
	sc->firmware->type = FTHD_MEM_FIRMWARE;
	sc->firmware->offset = 0;
	sc->firmware->size = FTHD_MEM_FW_SIZE;
	sc->firmware->size_aligned = FTHD_MEM_FW_SIZE;
	return (0);
}

struct isp_mem_obj *
isp_mem_create(struct fthd_softc *sc, int type, uint32_t size, int flags)
{
	struct isp_mem_obj *obj;
	vmem_addr_t addr;
	uint32_t aligned;

	if (size == 0 || size > sc->s2_mem_len)
		return (NULL);

	obj = malloc(sizeof(*obj), M_FTHD, flags | M_ZERO);
	if (obj == NULL)
		return (NULL);

	aligned = roundup2(size, PAGE_SIZE);
	if (vmem_alloc(sc->mem, aligned, M_BESTFIT | flags, &addr) != 0) {
		device_printf(sc->dev,
		    "Failed to allocate %u bytes of S2 memory\n", size);
		free(obj, M_FTHD);
		return (NULL);
	}

	obj->type = type;
	obj->offset = addr;
	obj->size = size;
	obj->size_aligned = aligned;
	return (obj);
}

void
isp_mem_destroy(struct fthd_softc *sc, struct isp_mem_obj *obj)
{
	if (obj == NULL)
		return;
	if (obj->type != FTHD_MEM_FIRMWARE)
		vmem_free(sc->mem, obj->offset, obj->size_aligned);
	free(obj, M_FTHD);
}

/* Power the sensor through the CMPE method on the device's ACPI node */
static int
isp_acpi_set_power(struct fthd_softc *sc, int power)
{
	ACPI_HANDLE handle;
	ACPI_OBJECT arg, *res;
	ACPI_OBJECT_LIST args;
	ACPI_BUFFER buf;
	ACPI_STATUS status;
	int error = 0;

	handle = acpi_get_handle(sc->dev);
	if (handle == NULL) {
		device_printf(sc->dev, "No ACPI handle for the camera\n");
		return (ENODEV);
	}

	arg.Type = ACPI_TYPE_INTEGER;
	arg.Integer.Value = power;
	args.Count = 1;
	args.Pointer = &arg;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	buf.Pointer = NULL;

	status = AcpiEvaluateObject(handle, "CMPE", &args, &buf);
	if (ACPI_FAILURE(status)) {
		device_printf(sc->dev, "Failed to execute CMPE: %s\n",
		    AcpiFormatException(status));
		return (ENODEV);
	}

	res = buf.Pointer;
	if (res == NULL || res->Type != ACPI_TYPE_INTEGER ||
	    res->Integer.Value != 0) {
		device_printf(sc->dev, "Unexpected CMPE result\n");
		error = EINVAL;
	}
	if (buf.Pointer != NULL)
		AcpiOsFree(buf.Pointer);
	return (error);
}

static void
isp_enable_sensor(struct fthd_softc *sc)
{
	int error;

	/* Power on sensor CMOS via SMC; some systems may not have CMPE */
	error = isp_acpi_set_power(sc, 1);
	if (error)
		device_printf(sc->dev,
		    "ACPI sensor power-on failed (%d), continuing\n", error);

	fthd_msleep(100);	/* let the sensor power rail stabilize */
}

static int
isp_load_firmware(struct fthd_softc *sc)
{
	const struct firmware *fw;
	int error;

	fw = firmware_get(FTHD_FIRMWARE);
	if (fw == NULL) {
		device_printf(sc->dev,
		    "Could not load /boot/firmware/%s\n", FTHD_FIRMWARE);
		return (ENOENT);
	}

	if (fw->datasize > FTHD_MEM_FW_SIZE) {
		device_printf(sc->dev, "Firmware too big (%zu bytes)\n",
		    fw->datasize);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (EFBIG);
	}

	error = fthd_s2_memcpy_toio(sc, sc->firmware->offset, fw->data,
	    fw->datasize);
	if (error == 0)
		device_printf(sc->dev, "Loaded firmware, size: %zukb\n",
		    fw->datasize / 1024);

	firmware_put(fw, FIRMWARE_UNLOAD);
	return (error);
}

static struct fw_channel *
isp_get_chan(struct fthd_softc *sc, const char *name)
{
	int i;

	for (i = 0; i < sc->num_channels; i++)
		if (strcasecmp(sc->channels[i].name, name) == 0)
			return (&sc->channels[i]);
	return (NULL);
}

static int
isp_fill_channel_info(struct fthd_softc *sc, uint32_t offset, int num)
{
	struct isp_channel_info info;
	struct fw_channel *chan;
	int i;

	if (num == 0)
		return (EINVAL);

	sc->channels = malloc(num * sizeof(*sc->channels), M_FTHD,
	    M_WAITOK | M_ZERO);
	sc->num_channels = num;

	for (i = 0; i < num; i++) {
		fthd_s2_memcpy_fromio(sc, &info, offset + i * 256, sizeof(info));
		chan = &sc->channels[i];

		strlcpy(chan->name, info.name, sizeof(chan->name));
		chan->type = info.type;
		chan->source = info.source;
		chan->size = info.size;
		chan->offset = info.offset;

		device_printf(sc->dev,
		    "Channel %d: %s, type %u, source %u, size %u, offset %x\n",
		    i, chan->name, chan->type, chan->source, chan->size,
		    chan->offset);

		/* Values we index or shift with must be sane */
		if (chan->source > 3 || chan->size == 0 || chan->size > 1024 ||
		    chan->offset > sc->s2_mem_len -
		    chan->size * FTHD_RINGBUF_ENTRY_SIZE) {
			device_printf(sc->dev, "Bogus channel description\n");
			return (EIO);
		}
	}

	sc->channel_terminal = isp_get_chan(sc, "TERMINAL");
	sc->channel_debug = isp_get_chan(sc, "DEBUG");
	sc->channel_shared_malloc = isp_get_chan(sc, "SHAREDMALLOC");
	sc->channel_io = isp_get_chan(sc, "IO");
	sc->channel_buf_h2t = isp_get_chan(sc, "BUF_H2T");
	sc->channel_buf_t2h = isp_get_chan(sc, "BUF_T2H");
	sc->channel_io_t2h = isp_get_chan(sc, "IO_T2H");

	if (sc->channel_terminal == NULL || sc->channel_debug == NULL ||
	    sc->channel_shared_malloc == NULL || sc->channel_io == NULL ||
	    sc->channel_buf_h2t == NULL || sc->channel_buf_t2h == NULL ||
	    sc->channel_io_t2h == NULL) {
		device_printf(sc->dev,
		    "Did not find all of the required channels\n");
		return (EIO);
	}
	return (0);
}

/* Wait up to 10s for one of the channel interrupt bits to come up */
static int
isp_wait_irq(struct fthd_softc *sc, const char *what)
{
	int retries;

	for (retries = 0; retries < 1000; retries++) {
		if (fthd_isp_reg_read(sc, ISP_IRQ_STATUS) & 0xf0) {
			device_printf(sc->dev, "ISP %s after %dms\n", what,
			    retries * 10);
			return (0);
		}
		fthd_msleep(10);
	}
	device_printf(sc->dev, "ISP init failed: no %s\n", what);
	return (EIO);
}

int
isp_init(struct fthd_softc *sc)
{
	struct isp_fw_args fw_args;
	uint32_t num_channels, queue_size, heap_size, offset, reg;
	int error, i, retries;

	error = isp_mem_init(sc);
	if (error)
		return (error);

	error = isp_load_firmware(sc);
	if (error)
		return (error);

	pci_set_powerstate(sc->dev, PCI_POWERSTATE_D0);
	fthd_msleep(10);

	isp_enable_sensor(sc);

	fthd_isp_reg_write(sc, 0, ISP_FW_CHAN_CTRL);
	fthd_isp_reg_write(sc, 0, ISP_FW_QUEUE_CTRL);
	fthd_isp_reg_write(sc, 0, ISP_FW_SIZE);
	fthd_isp_reg_write(sc, 0, ISP_FW_HEAP_SIZE);
	fthd_isp_reg_write(sc, 0, ISP_FW_HEAP_ADDR);
	fthd_isp_reg_write(sc, 0, ISP_FW_HEAP_SIZE2);
	fthd_isp_reg_write(sc, 0, ISP_REG_C3018);
	fthd_isp_reg_write(sc, 0, ISP_REG_C301C);

	fthd_isp_reg_write(sc, 0xffffffff, ISP_IRQ_CLEAR);

	/* Probably the IPC queue */
	for (i = ISP_FW_CHAN_START; i <= ISP_FW_CHAN_END; i += 8) {
		fthd_isp_reg_write(sc, 0xffffffff, i);
		fthd_isp_reg_write(sc, 0, i + 4);
	}

	/* Start the ISP CPU */
	fthd_isp_reg_write(sc, 0x80000000, ISP_REG_40008);
	fthd_isp_reg_write(sc, 0x1, ISP_REG_40004);
	sc->isp_booted = true;

	error = isp_wait_irq(sc, "wake signal");
	if (error)
		return (error);

	fthd_isp_reg_write(sc, 0xffffffff, ISP_IRQ_CLEAR);

	num_channels = fthd_isp_reg_read(sc, ISP_FW_CHAN_CTRL);
	queue_size = fthd_isp_reg_read(sc, ISP_FW_QUEUE_CTRL) + 1;
	device_printf(sc->dev, "Number of IPC channels: %u, queue size: %u\n",
	    num_channels, queue_size);

	if (num_channels == 0 || num_channels > 32) {
		device_printf(sc->dev, "Bad number of IPC channels: %u\n",
		    num_channels);
		return (EIO);
	}

	sc->fw_queue = isp_mem_create(sc, FTHD_MEM_FW_QUEUE, queue_size,
	    M_WAITOK);
	if (sc->fw_queue == NULL)
		return (ENOMEM);

	heap_size = fthd_isp_reg_read(sc, ISP_FW_HEAP_SIZE);

	if (heap_size == 0) {
		/*
		 * Not seen in practice.  The Linux driver writes its resource
		 * length here, which is one less than the region size.
		 */
		fthd_isp_reg_write(sc, 0, ISP_FW_CHAN_CTRL);
		fthd_isp_reg_write(sc, sc->fw_queue->offset, ISP_FW_QUEUE_CTRL);
		fthd_isp_reg_write(sc, FTHD_MEM_FW_SIZE - 1, ISP_FW_SIZE);
		fthd_isp_reg_write(sc, 0x10000000 - (FTHD_MEM_FW_SIZE - 1),
		    ISP_FW_HEAP_SIZE);
		fthd_isp_reg_write(sc, 0, ISP_FW_HEAP_ADDR);
		fthd_isp_reg_write(sc, 0, ISP_FW_HEAP_SIZE2);
		device_printf(sc->dev,
		    "Firmware asked for no heap; this path is untested\n");
		return (EIO);
	}

	/* Must be at least 0x1000 bytes, at most 4MB */
	heap_size = MAX(heap_size, 0x1000);
	if (heap_size > 0x400000) {
		device_printf(sc->dev,
		    "Firmware heap request too big (%ukb)\n", heap_size / 1024);
		return (ENOMEM);
	}
	device_printf(sc->dev, "Firmware requested heap size: %ukb\n",
	    heap_size / 1024);

	sc->heap = isp_mem_create(sc, FTHD_MEM_HEAP, heap_size, M_WAITOK);
	if (sc->heap == NULL)
		return (ENOMEM);

	fthd_isp_reg_write(sc, 0, ISP_FW_CHAN_CTRL);
	fthd_isp_reg_write(sc, sc->fw_queue->offset, ISP_FW_QUEUE_CTRL);
	fthd_isp_reg_write(sc, FTHD_MEM_FW_SIZE, ISP_FW_SIZE);
	fthd_isp_reg_write(sc, 0x10000000 - FTHD_MEM_FW_SIZE, ISP_FW_HEAP_SIZE);
	fthd_isp_reg_write(sc, sc->heap->offset, ISP_FW_HEAP_ADDR);
	fthd_isp_reg_write(sc, sc->heap->size, ISP_FW_HEAP_SIZE2);

	sc->fw_args = isp_mem_create(sc, FTHD_MEM_FW_ARGS, sizeof(fw_args),
	    M_WAITOK);
	if (sc->fw_args == NULL)
		return (ENOMEM);

	fw_args.__unknown = 2;
	fw_args.fw_arg = 0;
	fw_args.full_stats_mode = 0;
	fthd_s2_memcpy_toio(sc, sc->fw_args->offset, &fw_args, sizeof(fw_args));

	fthd_isp_reg_write(sc, sc->fw_args->offset, ISP_REG_C301C);
	fthd_isp_reg_write(sc, 0x10, ISP_REG_41020);

	error = isp_wait_irq(sc, "second interrupt");
	if (error)
		return (error);

	offset = fthd_isp_reg_read(sc, ISP_FW_CHAN_CTRL);
	device_printf(sc->dev, "Channel description table at %08x\n", offset);
	if (offset > sc->s2_mem_len - num_channels * 256) {
		device_printf(sc->dev, "Channel table out of range\n");
		return (EIO);
	}

	error = isp_fill_channel_info(sc, offset, num_channels);
	if (error)
		return (error);

	fthd_ringbuf_init(sc, sc->channel_terminal);
	fthd_ringbuf_init(sc, sc->channel_io);
	fthd_ringbuf_init(sc, sc->channel_debug);
	fthd_ringbuf_init(sc, sc->channel_buf_h2t);
	fthd_ringbuf_init(sc, sc->channel_buf_t2h);
	fthd_ringbuf_init(sc, sc->channel_shared_malloc);
	fthd_ringbuf_init(sc, sc->channel_io_t2h);

	fthd_isp_reg_write(sc, 0x8042006, ISP_FW_HEAP_SIZE);

	for (retries = 0; retries < 1000; retries++) {
		reg = fthd_isp_reg_read(sc, ISP_FW_HEAP_SIZE);
		if (reg == 0)
			break;
		fthd_msleep(10);
	}
	if (retries >= 1000) {
		device_printf(sc->dev, "ISP init failed: no magic value\n");
		return (EIO);
	}
	device_printf(sc->dev, "ISP firmware is up after %d ms\n",
	    retries * 10);

	return (0);
}

/* Undo isp_init.  Safe to call on a partially initialized device. */
void
isp_uninit(struct fthd_softc *sc)
{
	struct isp_mem_obj *obj;

	if (sc->isp_booted) {
		fthd_isp_reg_write(sc, 0x00000000, 0x40004);
		fthd_isp_reg_write(sc, 0x00000000, ISP_IRQ_ENABLE);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc0008);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc000c);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc0010);
		fthd_isp_reg_write(sc, 0x00000000, 0xc1004);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc100c);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc1014);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc101c);
		fthd_isp_reg_write(sc, 0xffffffff, 0xc1024);
		DELAY(1000);

		for (int off = 0xc0000; off <= 0xc0024; off += 4)
			fthd_isp_reg_write(sc, 0, off);

		fthd_isp_reg_write(sc, 0xffffffff, ISP_IRQ_CLEAR);
		sc->isp_booted = false;
	}

	free(sc->channels, M_FTHD);
	sc->channels = NULL;
	sc->num_channels = 0;
	sc->channel_terminal = sc->channel_debug = NULL;
	sc->channel_shared_malloc = sc->channel_io = NULL;
	sc->channel_buf_h2t = sc->channel_buf_t2h = NULL;
	sc->channel_io_t2h = NULL;

	while ((obj = TAILQ_FIRST(&sc->shared_mallocs)) != NULL) {
		TAILQ_REMOVE(&sc->shared_mallocs, obj, link);
		isp_mem_destroy(sc, obj);
	}

	isp_mem_destroy(sc, sc->set_file);
	isp_mem_destroy(sc, sc->fw_args);
	isp_mem_destroy(sc, sc->heap);
	isp_mem_destroy(sc, sc->fw_queue);
	isp_mem_destroy(sc, sc->firmware);
	sc->set_file = sc->fw_args = sc->heap = NULL;
	sc->fw_queue = sc->firmware = NULL;

	if (sc->mem != NULL) {
		vmem_destroy(sc->mem);
		sc->mem = NULL;
	}
}

/*
 * Send a command on the IO channel and wait for the reply.  buf holds the
 * request payload and receives *response_len bytes of reply.
 */
static int
fthd_isp_cmd(struct fthd_softc *sc, enum fthd_isp_cmds command, void *buf,
    int request_len, int *response_len)
{
	struct isp_mem_obj *request;
	struct isp_cmd_hdr cmd;
	uint32_t address, entry;
	int error, len;

	if (sc->channel_io == NULL)
		return (ENXIO);

	len = request_len;
	if (response_len != NULL)
		len = MAX(len, *response_len);
	len += sizeof(cmd);

	FTHD_DBG(sc, "sending cmd 0x%x to firmware\n", command);

	sx_xlock(&sc->cmd_sx);
	request = isp_mem_create(sc, FTHD_MEM_CMD, len, M_WAITOK);
	if (request == NULL) {
		sx_xunlock(&sc->cmd_sx);
		return (ENOMEM);
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = command;
	fthd_s2_memcpy_toio(sc, request->offset, &cmd, sizeof(cmd));
	if (request_len)
		fthd_s2_memcpy_toio(sc, request->offset + sizeof(cmd), buf,
		    request_len);

	error = fthd_ringbuf_send(sc, sc->channel_io, request->offset,
	    request_len + 8, (response_len ? *response_len : 0) + 8, &entry);
	if (error)
		goto out;

	/* Power down doesn't generate a response */
	if (command == CISP_CMD_POWER_DOWN)
		goto out;

	error = fthd_ringbuf_wait_ready(sc, sc->channel_io, entry, 2000);
	if (error) {
		if (response_len)
			*response_len = 0;
		goto out;
	}

	fthd_s2_memcpy_fromio(sc, &cmd, request->offset, sizeof(cmd));
	address = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS) & ~3;

	if (response_len && *response_len)
		fthd_s2_memcpy_fromio(sc, buf, address + sizeof(cmd),
		    *response_len);

	FTHD_DBG(sc, "cmd 0x%x status %04x\n", command, cmd.status);
	error = cmd.status ? EIO : 0;
out:
	/*
	 * After a timeout the firmware may still write the reply.  Leak the
	 * buffer rather than handing the memory to someone else.
	 */
	if (error != ETIMEDOUT)
		isp_mem_destroy(sc, request);
	sx_xunlock(&sc->cmd_sx);
	return (error);
}

static int
fthd_isp_cmd_print_enable(struct fthd_softc *sc, int enable)
{
	struct isp_cmd_print_enable cmd = { .enable = enable };

	return (fthd_isp_cmd(sc, CISP_CMD_PRINT_ENABLE, &cmd, sizeof(cmd),
	    NULL));
}

static int
fthd_isp_cmd_camera_config(struct fthd_softc *sc)
{
	struct isp_cmd_config cmd;
	int error, len;

	memset(&cmd, 0, sizeof(cmd));
	len = sizeof(cmd);
	error = fthd_isp_cmd(sc, CISP_CMD_CONFIG_GET, &cmd, sizeof(cmd), &len);
	if (error == 0)
		device_printf(sc->dev, "Camera config: %08x %08x %08x %08x "
		    "%08x %08x %08x %08x\n", cmd.field[0], cmd.field[1],
		    cmd.field[2], cmd.field[3], cmd.field[4], cmd.field[5],
		    cmd.field[6], cmd.field[7]);
	return (error);
}

static int
fthd_isp_cmd_channel_info(struct fthd_softc *sc)
{
	struct isp_cmd_channel_info cmd;
	char serial[sizeof(cmd.camera_module_serial_number) + 1];
	int error, len;

	memset(&cmd, 0, sizeof(cmd));
	len = sizeof(cmd);
	error = fthd_isp_cmd(sc, CISP_CMD_CH_INFO_GET, &cmd, sizeof(cmd), &len);
	if (error)
		return (error);

	memcpy(serial, cmd.camera_module_serial_number, sizeof(serial) - 1);
	serial[sizeof(serial) - 1] = '\0';

	sc->sensor_id0 = cmd.sensorid0;
	sc->sensor_id1 = cmd.sensorid1;
	sc->sensor_count = cmd.sensor_count;

	device_printf(sc->dev, "Sensor id %04x %04x, count %d, module %s\n",
	    sc->sensor_id0, sc->sensor_id1, sc->sensor_count, serial);
	return (0);
}

static int
fthd_isp_cmd_channel_camera_config(struct fthd_softc *sc)
{
	struct isp_cmd_channel_camera_config cmd;
	unsigned int w, h;
	int error = 0, i, len;

	for (i = 0; i < sc->sensor_count && i < 4; i++) {
		memset(&cmd, 0, sizeof(cmd));
		cmd.channel = i;
		len = sizeof(cmd);
		error = fthd_isp_cmd(sc, CISP_CMD_CH_CAMERA_CONFIG_GET, &cmd,
		    sizeof(cmd), &len);
		if (error)
			break;

		/* The config starts with the sensor's native width/height */
		w = cmd.data[0] | (cmd.data[1] << 8);
		h = cmd.data[2] | (cmd.data[3] << 8);
		device_printf(sc->dev, "Sensor %d native resolution %ux%u\n",
		    i, w, h);
		if (i == 0 && w != 0 && h != 0) {
			sc->sensor_width = w;
			sc->sensor_height = h;
		}
	}
	return (error);
}

static const char *
fthd_set_file_name(struct fthd_softc *sc)
{
	char *maker, *product;
	bool is_air = false;

	switch (sc->sensor_id1) {
	case 0x164:
		return ("facetimehd/8221_01XX.dat");
	case 0x190:
		return ("facetimehd/1222_01XX.dat");
	case 0x248:
		return ((sc->sensor_id0 >> 4) == 5 ?
		    "facetimehd/1575_01XX.dat" : "facetimehd/1571_01XX.dat");
	case 0x8830:
		return ("facetimehd/9112_01XX.dat");
	case 0x9770:
		maker = kern_getenv("smbios.system.maker");
		product = kern_getenv("smbios.system.product");
		if (maker != NULL && product != NULL &&
		    strcmp(maker, "Apple Inc.") == 0 &&
		    strncmp(product, "MacBookAir", 10) == 0)
			is_air = true;
		freeenv(maker);
		freeenv(product);
		if (is_air)
			return ("facetimehd/1771_01XX.dat");
		return (sc->sensor_id0 == 4 ?
		    "facetimehd/1874_01XX.dat" : "facetimehd/1871_01XX.dat");
	case 0x9774:
		switch (sc->sensor_id0) {
		case 4:
			return ("facetimehd/1674_01XX.dat");
		case 5:
			return ("facetimehd/1675_01XX.dat");
		default:
			return ("facetimehd/1671_01XX.dat");
		}
	default:
		return (NULL);
	}
}

/* Upload the sensor calibration file.  Missing calibration is not fatal. */
static int
fthd_isp_cmd_set_loadfile(struct fthd_softc *sc)
{
	struct isp_cmd_set_loadfile cmd;
	const struct firmware *fw;
	const char *name;
	int error;

	name = fthd_set_file_name(sc);
	if (name == NULL) {
		device_printf(sc->dev, "No set file for sensor %04x %04x, "
		    "continuing without calibration\n",
		    sc->sensor_id0, sc->sensor_id1);
		return (0);
	}

	fw = firmware_get_flags(name, FIRMWARE_GET_NOWARN);
	if (fw == NULL) {
		device_printf(sc->dev, "Set file /boot/firmware/%s is missing, "
		    "continuing without calibration\n", name);
		return (0);
	}

	if (sc->set_file == NULL)
		sc->set_file = isp_mem_create(sc, FTHD_MEM_SET_FILE,
		    fw->datasize, M_WAITOK);
	if (sc->set_file == NULL) {
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (ENOMEM);
	}
	fthd_s2_memcpy_toio(sc, sc->set_file->offset, fw->data, fw->datasize);
	firmware_put(fw, FIRMWARE_UNLOAD);

	memset(&cmd, 0, sizeof(cmd));
	cmd.addr = sc->set_file->offset;
	cmd.length = sc->set_file->size;
	error = fthd_isp_cmd(sc, CISP_CMD_CH_SET_FILE_LOAD, &cmd, sizeof(cmd),
	    NULL);
	if (error)
		device_printf(sc->dev, "Set file load failed (%d), "
		    "continuing without calibration\n", error);
	else
		device_printf(sc->dev, "Loaded set file %s\n", name);
	return (0);
}

int
fthd_firmware_start(struct fthd_softc *sc)
{
	int error;

	error = fthd_isp_cmd(sc, CISP_CMD_START, NULL, 0, NULL);
	if (error)
		return (error);
	sc->fw_started = true;

	error = fthd_isp_cmd_print_enable(sc, 1);
	if (error)
		return (error);

	error = fthd_isp_cmd_camera_config(sc);
	if (error)
		return (error);

	error = fthd_isp_cmd_channel_info(sc);
	if (error)
		return (error);

	/* Non-fatal: only used to report the sensor size */
	fthd_isp_cmd_channel_camera_config(sc);

	return (fthd_isp_cmd_set_loadfile(sc));
}

int
isp_powerdown(struct fthd_softc *sc)
{
	int retries;

	fthd_isp_reg_write(sc, 0xf7fbdff9, 0xc3000);
	fthd_isp_cmd(sc, CISP_CMD_POWER_DOWN, NULL, 0, NULL);

	for (retries = 0; retries < 100; retries++) {
		if (fthd_isp_reg_read(sc, 0xc3000) == 0x8042006)
			return (0);
		fthd_msleep(10);
	}
	device_printf(sc->dev, "ISP power down failed\n");
	return (EIO);
}

void
fthd_firmware_stop(struct fthd_softc *sc)
{
	if (sc->fw_started) {
		fthd_isp_cmd(sc, CISP_CMD_STOP, NULL, 0, NULL);
		sc->fw_started = false;
	}
	if (sc->channel_io != NULL)
		isp_powerdown(sc);
}

/*
 * Channel commands.  All of them echo the request back as the response, so
 * the response length is the request length.
 */
static int
fthd_isp_cmd_echo(struct fthd_softc *sc, enum fthd_isp_cmds command,
    void *cmd, int size)
{
	int error, len = size;

	error = fthd_isp_cmd(sc, command, cmd, size, &len);
	if (error)
		device_printf(sc->dev, "cmd 0x%x failed: %d\n", command, error);
	return (error);
}

static int
fthd_isp_cmd_ch(struct fthd_softc *sc, enum fthd_isp_cmds command)
{
	struct isp_cmd_channel cmd = { .channel = 0 };

	return (fthd_isp_cmd_echo(sc, command, &cmd, sizeof(cmd)));
}

static int
fthd_isp_cmd_ch_u16(struct fthd_softc *sc, enum fthd_isp_cmds command,
    uint16_t value)
{
	struct isp_cmd_channel_u16 cmd = { .channel = 0, .value = value };

	return (fthd_isp_cmd_echo(sc, command, &cmd, sizeof(cmd)));
}

static int
fthd_isp_cmd_ch_u32(struct fthd_softc *sc, enum fthd_isp_cmds command,
    uint32_t value)
{
	struct isp_cmd_channel_u32 cmd = { .channel = 0, .value = value };

	return (fthd_isp_cmd_echo(sc, command, &cmd, sizeof(cmd)));
}

static int
fthd_isp_cmd_ch_u16x2(struct fthd_softc *sc, enum fthd_isp_cmds command,
    uint16_t param1, uint16_t param2)
{
	struct isp_cmd_channel_u16x2 cmd = {
		.channel = 0, .param1 = param1, .param2 = param2
	};

	return (fthd_isp_cmd_echo(sc, command, &cmd, sizeof(cmd)));
}

/* Start and stop don't send a response */
static int
fthd_isp_cmd_ch_noresp(struct fthd_softc *sc, enum fthd_isp_cmds command)
{
	struct isp_cmd_channel cmd = { .channel = 0 };
	int error;

	error = fthd_isp_cmd(sc, command, &cmd, sizeof(cmd), NULL);
	if (error)
		device_printf(sc->dev, "cmd 0x%x failed: %d\n", command, error);
	return (error);
}

/*
 * Crop the largest centred window with the output's aspect ratio and let
 * the ISP scale it down.  An odd width at an odd offset makes the sensor
 * interface fault on every frame, so the width is kept even.
 */
static int
fthd_isp_cmd_crop(struct fthd_softc *sc)
{
	struct isp_cmd_channel_set_crop cmd;
	unsigned int sw, sh, ow, oh, cw, ch;

	ow = sc->fmt.width;
	oh = sc->fmt.height;
	sw = sc->sensor_width ? sc->sensor_width : ow;
	sh = sc->sensor_height ? sc->sensor_height : oh;

	if (ow * sh >= sw * oh) {
		cw = sw;
		ch = sw * oh / ow;
	} else {
		ch = sh;
		cw = sh * ow / oh;
	}
	cw = MIN(cw, sw) & ~1u;
	ch = MIN(ch, sh);

	memset(&cmd, 0, sizeof(cmd));
	cmd.channel = 0;
	cmd.x = (sw - cw) / 2;
	cmd.y = (sh - ch) / 2;
	cmd.width = cw;
	cmd.height = ch;
	FTHD_DBG(sc, "crop %ux%u at %u,%u\n", cw, ch, cmd.x, cmd.y);
	return (fthd_isp_cmd_echo(sc, CISP_CMD_CH_CROP_SET, &cmd, sizeof(cmd)));
}

static int
fthd_isp_cmd_output_config(struct fthd_softc *sc)
{
	struct isp_cmd_channel_output_config cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.channel = 0;
	cmd.x1 = sc->fmt.width;
	cmd.x2 = sc->fmt.width * 2;
	cmd.x3 = sc->fmt.width;
	cmd.y1 = sc->fmt.height;
	cmd.pixelformat = 1;		/* YUYV, the only format we offer */
	cmd.unknown5 = 0x7ff;
	return (fthd_isp_cmd_echo(sc, CISP_CMD_CH_OUTPUT_CONFIG_SET, &cmd,
	    sizeof(cmd)));
}

/* Same sequence as the Linux driver, which copied it from Apple's */
int
fthd_start_channel(struct fthd_softc *sc)
{
	struct isp_cmd_channel_u32 select = { .channel = 0, .value = 0 };
	uint16_t rate = sc->fmt.fps * FTHD_FRAME_RATE_SCALE;
	int error;

	if ((error = fthd_isp_cmd_channel_camera_config(sc)) != 0 ||
	    (error = fthd_isp_cmd_echo(sc, CISP_CMD_CH_CAMERA_CONFIG_SELECT,
	    &select, sizeof(select))) != 0 ||
	    (error = fthd_isp_cmd_crop(sc)) != 0 ||
	    (error = fthd_isp_cmd_output_config(sc)) != 0 ||
	    (error = fthd_isp_cmd_ch_u32(sc,
	    CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET, 1)) != 0 ||
	    (error = fthd_isp_cmd_ch_u32(sc,
	    CISP_CMD_CH_BUFFER_RECYCLE_START, 0)) != 0 ||
	    (error = fthd_isp_cmd_ch_u32(sc,
	    CISP_CMD_APPLE_CH_AE_METERING_MODE_SET, 3)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc, CISP_CMD_CH_DRC_START)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_APPLE_CH_TONE_CURVE_ADAPTATION_START)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16(sc, CISP_CMD_CH_AE_SPEED_SET,
	    60)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16(sc, CISP_CMD_CH_AE_STABILITY_SET,
	    75)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16(sc,
	    CISP_CMD_CH_AE_STABILITY_TO_STABLE_SET, 8)) != 0 ||
	    /* param1/param2 are two u8s followed by two zero bytes */
	    (error = fthd_isp_cmd_ch_u16x2(sc,
	    CISP_CMD_CH_SIF_PIXEL_FORMAT_SET, 1 | (1 << 8), 0)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16x2(sc,
	    CISP_CMD_CH_CAMERA_ERR_HANDLE_CONFIG, 2, 1)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_CH_FACE_DETECTION_ENABLE)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_CH_FACE_DETECTION_START)) != 0 ||
	    /* Constant frame rate: AE may not slow the sensor down */
	    (error = fthd_isp_cmd_ch_u16(sc, CISP_CMD_CH_AE_FRAME_RATE_MAX_SET,
	    rate)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16(sc, CISP_CMD_CH_AE_FRAME_RATE_MIN_SET,
	    rate)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_APPLE_CH_TEMPORAL_FILTER_START)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_APPLE_CH_MOTION_HISTORY_START)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc,
	    CISP_CMD_APPLE_CH_TEMPORAL_FILTER_ENABLE)) != 0 ||
	    (error = fthd_isp_cmd_ch_u16(sc,
	    CISP_CMD_APPLE_CH_STREAMING_MODE_SET, 0)) != 0 ||
	    (error = fthd_isp_cmd_ch_noresp(sc, CISP_CMD_CH_START)) != 0)
		return (error);

	fthd_msleep(200);	/* let auto exposure settle */
	return (0);
}

/* Makes the firmware hand back every buffer it holds on BUF_T2H */
int
fthd_stop_channel(struct fthd_softc *sc)
{
	int error;

	if ((error = fthd_isp_cmd_ch_noresp(sc, CISP_CMD_CH_STOP)) != 0 ||
	    (error = fthd_isp_cmd_ch(sc, CISP_CMD_CH_BUFFER_RETURN)) != 0)
		return (error);

	/* Feature teardown failing doesn't matter once the buffers are back */
	fthd_isp_cmd_ch(sc, CISP_CMD_CH_FACE_DETECTION_STOP);
	fthd_isp_cmd_ch(sc, CISP_CMD_CH_FACE_DETECTION_DISABLE);
	fthd_isp_cmd_ch(sc, CISP_CMD_APPLE_CH_TEMPORAL_FILTER_DISABLE);
	fthd_isp_cmd_ch(sc, CISP_CMD_APPLE_CH_MOTION_HISTORY_STOP);
	fthd_isp_cmd_ch(sc, CISP_CMD_APPLE_CH_TEMPORAL_FILTER_STOP);
	return (0);
}

/*
 * Firmware-to-host message handlers, run from the interrupt task.
 */

static void
sharedmalloc_handler(struct fthd_softc *sc, struct fw_channel *chan,
    uint32_t entry)
{
	struct isp_mem_obj *obj;
	uint32_t request_size, response_size, address;
	int error;

	request_size = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_REQUEST_SIZE);
	response_size = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_RESPONSE_SIZE);
	address = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS) & ~3;

	if (address != 0) {
		/*
		 * Free.  The Linux driver keeps a kernel pointer in the 64
		 * bytes before the block; we look the block up instead, so
		 * nothing the device can write is ever dereferenced.
		 */
		FTHD_DBG(sc, "Firmware frees memory at %08x\n", address);
		mtx_lock(&sc->mtx);
		TAILQ_FOREACH(obj, &sc->shared_mallocs, link)
			if (obj->offset + 64 == address)
				break;
		if (obj != NULL)
			TAILQ_REMOVE(&sc->shared_mallocs, obj, link);
		mtx_unlock(&sc->mtx);

		if (obj != NULL)
			isp_mem_destroy(sc, obj);
		else
			device_printf(sc->dev,
			    "Firmware freed unknown block %08x\n", address);

		error = fthd_ringbuf_send(sc, chan, 0, 0, 0, NULL);
	} else {
		if (request_size == 0)
			return;
		obj = isp_mem_create(sc, FTHD_MEM_SHAREDMALLOC,
		    request_size + 64, M_NOWAIT);
		if (obj == NULL)
			return;

		FTHD_DBG(sc, "Firmware allocated %u bytes at %08x "
		    "(tag %c%c%c%c)\n", request_size, obj->offset,
		    response_size >> 24, response_size >> 16,
		    response_size >> 8, response_size);

		mtx_lock(&sc->mtx);
		TAILQ_INSERT_TAIL(&sc->shared_mallocs, obj, link);
		mtx_unlock(&sc->mtx);

		error = fthd_ringbuf_send(sc, chan, obj->offset + 64, 0, 0,
		    NULL);
	}
	if (error)
		device_printf(sc->dev, "sharedmalloc reply failed: %d\n",
		    error);
}

static void
terminal_handler(struct fthd_softc *sc, uint32_t entry)
{
	uint32_t request_size, address;
	char buf[512];
	int i;

	request_size = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_REQUEST_SIZE);
	address = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS) & ~3;

	if (address == 0 || request_size == 0)
		return;

	request_size = MIN(request_size, sizeof(buf));
	if (fthd_s2_memcpy_fromio(sc, buf, address, request_size) != 0)
		return;

	/* Strip anything that isn't printable before it reaches dmesg */
	for (i = 0; i < request_size; i++)
		if (buf[i] == '\0')
			break;
		else if ((buf[i] < 0x20 || buf[i] > 0x7e) && buf[i] != '\n')
			buf[i] = '.';
	if (i > 0 && buf[i - 1] == '\n')
		i--;

	device_printf(sc->dev, "FWMSG: %.*s\n", i, buf);
}

static void
buf_t2h_handler(struct fthd_softc *sc, struct fw_channel *chan,
    uint32_t entry)
{
	uint32_t response_size, address;
	int error;

	response_size = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_RESPONSE_SIZE);
	address = fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS);

	if (address & 1)
		return;

	fthd_buffer_return_handler(sc, address & ~3);
	error = fthd_ringbuf_send(sc, chan,
	    (response_size & 0x10000000) ? address : 0, 0, 0x80000000, NULL);
	if (error)
		device_printf(sc->dev, "buf_t2h reply failed: %d\n", error);
}

void
fthd_handle_channel(struct fthd_softc *sc, struct fw_channel *chan)
{
	uint32_t entry;
	int error, n;

	/* Host-to-target channels: wake whoever waits for a completion */
	if (chan == sc->channel_io || chan == sc->channel_buf_h2t ||
	    chan == sc->channel_debug) {
		mtx_lock(&sc->mtx);
		wakeup(chan);
		mtx_unlock(&sc->mtx);
		return;
	}

	/*
	 * On target-to-host rings only our reply advances the ring, so a
	 * handler that bails out without replying would see the same entry
	 * again.  Never process more than one ring's worth per interrupt.
	 */
	for (n = 0; n < chan->size &&
	    (entry = fthd_ringbuf_receive(sc, chan)) != (uint32_t)-1; n++) {
		if (chan == sc->channel_shared_malloc) {
			sharedmalloc_handler(sc, chan, entry);
		} else if (chan == sc->channel_terminal) {
			terminal_handler(sc, entry);
			error = fthd_ringbuf_send(sc, chan, 0, 0, 0, NULL);
			if (error)
				device_printf(sc->dev,
				    "terminal reply failed: %d\n", error);
		} else if (chan == sc->channel_buf_t2h) {
			buf_t2h_handler(sc, chan, entry);
		} else if (chan == sc->channel_io_t2h) {
			error = fthd_ringbuf_send(sc, chan, 0, 0, 0, NULL);
			if (error)
				device_printf(sc->dev,
				    "io_t2h reply failed: %d\n", error);
		} else {
			/* Unknown channel: don't spin on it */
			break;
		}
	}
}
