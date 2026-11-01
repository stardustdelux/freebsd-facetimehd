/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * Ported from the Linux facetimehd driver:
 *   Copyright (C) 2014 Patrik Jakobsson (patrik.r.jakobsson@gmail.com)
 *   Copyright (C) 2015 Sven Schnelle <svens@stackframe.org>
 */

#ifndef _FTHD_VAR_H
#define _FTHD_VAR_H

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/taskqueue.h>
#include <sys/vmem.h>

#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "fthd_reg.h"
#include "fthd_ioctl.h"

#define FTHD_PCI_S2_IO		PCIR_BAR(0)	/* 64KB S2 registers */
#define FTHD_PCI_S2_MEM		PCIR_BAR(2)	/* 256MB S2 DDR */
#define FTHD_PCI_ISP_IO		PCIR_BAR(4)	/* 1MB ISP registers */

/* ISP memory object types */
#define FTHD_MEM_FIRMWARE	1
#define FTHD_MEM_HEAP		2
#define FTHD_MEM_FW_QUEUE	3
#define FTHD_MEM_FW_ARGS	4
#define FTHD_MEM_CMD		5
#define FTHD_MEM_SHAREDMALLOC	6
#define FTHD_MEM_SET_FILE	7
#define FTHD_MEM_BUFFER		8

#define FTHD_MEM_FW_SIZE	0x800000	/* 8MB reserved at offset 0 */

#define FTHD_RINGBUF_ENTRY_SIZE		64
#define FTHD_RINGBUF_ADDRESS_FLAGS	0
#define FTHD_RINGBUF_REQUEST_SIZE	4
#define FTHD_RINGBUF_RESPONSE_SIZE	8

enum fw_chan_type {
	FW_CHAN_TYPE_OUT = 0,		/* host to target */
	FW_CHAN_TYPE_IN = 1,		/* target to host */
	FW_CHAN_TYPE_UNI_IN = 2,
};

struct isp_mem_obj {
	TAILQ_ENTRY(isp_mem_obj) link;	/* only used for SHAREDMALLOC */
	int		type;
	uint32_t	offset;		/* offset into S2 MEM */
	uint32_t	size;		/* requested size */
	uint32_t	size_aligned;	/* size rounded to page */
};

struct fw_channel {
	char		name[64];
	uint32_t	offset;
	uint32_t	size;
	uint32_t	source;
	uint32_t	type;
	int		idx;		/* ring buffer position */
};

/*
 * Capture buffers.  The camera reaches host memory through its own IOMMU
 * (the "DART"): 4096 32-bit entries at S2 IO 0x9000, each holding a host
 * page frame number, which the firmware sees at 0xc0000000 + index * 4KB.
 */
#define FTHD_IOMMU_REG		0x9000
#define FTHD_IOMMU_PAGES	0x1000
#define FTHD_IOMMU_DEVADDR	0xc0000000u

#define FTHD_NBUF		4
#define FTHD_BUF_TAG_MAGIC	0x66746864u	/* "fthd" */

enum fthd_buf_state {
	FTHD_BUF_IDLE,		/* owned by the driver */
	FTHD_BUF_HW,		/* handed to the firmware */
	FTHD_BUF_DONE,		/* holds a frame, on the done queue */
	FTHD_BUF_READING,	/* being copied out by read() */
};

struct fthd_buf {
	enum fthd_buf_state	state;
	int			idx;
	void			*vaddr;
	bus_dmamap_t		map;
	uint32_t		iommu_base;	/* first IOMMU entry */
	uint32_t		iommu_pages;
	struct isp_mem_obj	*desc;		/* descriptor list in S2 MEM */
	uint32_t		sequence;
	struct timespec		timestamp;
	STAILQ_ENTRY(fthd_buf)	link;		/* done queue */
};

struct fthd_softc {
	device_t		dev;
	struct mtx		mtx;	/* rings, ISP doorbell, sleep channel */
	struct sx		cmd_sx;	/* one firmware command at a time */

	struct resource		*s2_io;
	struct resource		*s2_mem;
	struct resource		*isp_io;
	uint32_t		s2_io_len;
	uint32_t		s2_mem_len;
	uint32_t		isp_io_len;

	struct resource		*irq;
	int			irq_rid;
	void			*irq_cookie;
	int			msi;
	struct task		irq_task;
	struct taskqueue	*tq;

	/* Hardware info */
	uint32_t		ddr_model;
	uint32_t		ddr_speed;
	uint32_t		vdl_step_size;
	uint32_t		ddr_phy_regs[DDR_PHY_NUM_REG];

	/* S2 memory allocator (everything after the firmware) */
	vmem_t			*mem;
	struct isp_mem_obj	*firmware;
	struct isp_mem_obj	*fw_queue;
	struct isp_mem_obj	*heap;
	struct isp_mem_obj	*fw_args;
	struct isp_mem_obj	*set_file;
	TAILQ_HEAD(, isp_mem_obj) shared_mallocs;

	/* Firmware channels */
	int			num_channels;
	struct fw_channel	*channels;
	struct fw_channel	*channel_terminal;
	struct fw_channel	*channel_io;
	struct fw_channel	*channel_debug;
	struct fw_channel	*channel_buf_h2t;
	struct fw_channel	*channel_buf_t2h;
	struct fw_channel	*channel_shared_malloc;
	struct fw_channel	*channel_io_t2h;

	bool			isp_booted;
	bool			fw_started;

	/* Camera config */
	int			sensor_count;
	int			sensor_id0;
	int			sensor_id1;
	unsigned int		sensor_width;
	unsigned int		sensor_height;

	/* Capture.  Lock order: dev_sx, then buf_mtx, then mtx. */
	struct sx		dev_sx;		/* format, stream start/stop */
	struct mtx		buf_mtx;	/* buffer states, done queue */
	vmem_t			*iommu;
	bus_dma_tag_t		buf_dtag;
	struct fthd_buf		bufs[FTHD_NBUF];
	int			nbufs;		/* allocated for fmt */
	STAILQ_HEAD(, fthd_buf)	done_q;
	struct fthd_format	fmt;
	bool			streaming;
	uint32_t		sequence;
	uint32_t		dropped;
	struct fthd_frame_info	last_frame;	/* of the last read() */
	uint64_t		frames_total;	/* sysctl statistics */
	uint64_t		drops_total;

	/* /dev/fthdN */
	struct cdev		*cdev;
	struct selinfo		rsel;
	bool			is_open;
	bool			dying;
};

extern int fthd_debug;

#define FTHD_DBG(sc, ...) do {					\
	if (fthd_debug)						\
		device_printf((sc)->dev, __VA_ARGS__);		\
} while (0)

/*
 * Register and memory accessors.  Writes to the register BARs are followed
 * by a config space write, which the Linux driver uses to flush posted
 * writes (the vendor ID register is read-only, so the value is ignored).
 */
static inline void
fthd_pci_post(struct fthd_softc *sc)
{
	pci_write_config(sc->dev, 0, 0x12345678, 4);
}

static inline uint32_t
fthd_s2_reg_read(struct fthd_softc *sc, uint32_t off)
{
	if (off >= sc->s2_io_len) {
		device_printf(sc->dev, "S2 IO read out of range at 0x%x\n", off);
		return (0);
	}
	return (bus_read_4(sc->s2_io, off));
}

static inline void
fthd_s2_reg_write(struct fthd_softc *sc, uint32_t val, uint32_t off)
{
	if (off >= sc->s2_io_len) {
		device_printf(sc->dev, "S2 IO write out of range at 0x%x\n", off);
		return;
	}
	bus_write_4(sc->s2_io, off, val);
	fthd_pci_post(sc);
}

static inline uint32_t
fthd_s2_mem_read(struct fthd_softc *sc, uint32_t off)
{
	if (off > sc->s2_mem_len - 4) {
		device_printf(sc->dev, "S2 MEM read out of range at 0x%x\n", off);
		return (0);
	}
	return (bus_read_4(sc->s2_mem, off));
}

static inline void
fthd_s2_mem_write(struct fthd_softc *sc, uint32_t val, uint32_t off)
{
	if (off > sc->s2_mem_len - 4) {
		device_printf(sc->dev, "S2 MEM write out of range at 0x%x\n", off);
		return;
	}
	bus_write_4(sc->s2_mem, off, val);
}

static inline uint32_t
fthd_isp_reg_read(struct fthd_softc *sc, uint32_t off)
{
	if (off >= sc->isp_io_len) {
		device_printf(sc->dev, "ISP IO read out of range at 0x%x\n", off);
		return (0);
	}
	return (bus_read_4(sc->isp_io, off));
}

static inline void
fthd_isp_reg_write(struct fthd_softc *sc, uint32_t val, uint32_t off)
{
	if (off >= sc->isp_io_len) {
		device_printf(sc->dev, "ISP IO write out of range at 0x%x\n", off);
		return;
	}
	bus_write_4(sc->isp_io, off, val);
	fthd_pci_post(sc);
}

int	fthd_s2_memcpy_toio(struct fthd_softc *sc, uint32_t off,
	    const void *buf, size_t len);
int	fthd_s2_memcpy_fromio(struct fthd_softc *sc, void *buf, uint32_t off,
	    size_t len);

/* fthd_pci.c */
void	fthd_irq_enable(struct fthd_softc *sc);
void	fthd_irq_disable(struct fthd_softc *sc);

/* fthd_hw.c */
int	fthd_hw_init(struct fthd_softc *sc);
void	fthd_hw_deinit(struct fthd_softc *sc);
int	fthd_ddr_verify_mem(struct fthd_softc *sc, uint32_t base, int count);

/* fthd_ringbuf.c */
uint32_t fthd_ringbuf_entry_addr(struct fw_channel *chan, int num);
void	fthd_ringbuf_init(struct fthd_softc *sc, struct fw_channel *chan);
int	fthd_ringbuf_send(struct fthd_softc *sc, struct fw_channel *chan,
	    uint32_t data_offset, uint32_t request_size,
	    uint32_t response_size, uint32_t *entryp);
uint32_t fthd_ringbuf_receive(struct fthd_softc *sc, struct fw_channel *chan);
int	fthd_ringbuf_wait_ready(struct fthd_softc *sc, struct fw_channel *chan,
	    uint32_t entry, int timeout_ms);
void	fthd_ringbuf_dump(struct fthd_softc *sc, struct fw_channel *chan);

/* fthd_isp.c */
struct isp_mem_obj *isp_mem_create(struct fthd_softc *sc, int type,
	    uint32_t size, int flags);
void	isp_mem_destroy(struct fthd_softc *sc, struct isp_mem_obj *obj);
int	isp_init(struct fthd_softc *sc);
void	isp_uninit(struct fthd_softc *sc);
int	isp_powerdown(struct fthd_softc *sc);
int	fthd_firmware_start(struct fthd_softc *sc);
void	fthd_firmware_stop(struct fthd_softc *sc);
void	fthd_handle_channel(struct fthd_softc *sc, struct fw_channel *chan);
int	fthd_start_channel(struct fthd_softc *sc);
int	fthd_stop_channel(struct fthd_softc *sc);

/* fthd_buffer.c */
int	fthd_buffer_init(struct fthd_softc *sc);
void	fthd_buffer_uninit(struct fthd_softc *sc);
void	fthd_format_default(struct fthd_softc *sc);
void	fthd_format_adjust(struct fthd_softc *sc, struct fthd_format *fmt);
int	fthd_buffers_alloc(struct fthd_softc *sc);
void	fthd_buffers_free(struct fthd_softc *sc);
int	fthd_stream_start(struct fthd_softc *sc);
void	fthd_stream_stop(struct fthd_softc *sc);
void	fthd_buffer_requeue(struct fthd_softc *sc, struct fthd_buf *b);
void	fthd_buffer_return_handler(struct fthd_softc *sc, uint32_t offset);

/* fthd_cdev.c */
int	fthd_cdev_create(struct fthd_softc *sc);
void	fthd_cdev_destroy(struct fthd_softc *sc);

#endif /* _FTHD_VAR_H */
