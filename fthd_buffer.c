/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * Capture buffers and streaming, from the Linux facetimehd driver:
 *   Copyright (C) 2015 Sven Schnelle <svens@stackframe.org>
 *
 * Frames land directly in host memory.  Each buffer is mapped page by page
 * into the camera's IOMMU and described to the firmware by a descriptor
 * list in S2 memory, sent on BUF_H2T.  Filled buffers come back on BUF_T2H
 * with the same descriptor, which carries a tag we chose.  Unlike Linux we
 * don't put a kernel pointer in the tag: it holds a magic and the buffer
 * index, and is checked before use.
 *
 * A buffer cycles IDLE -> HW -> DONE -> READING -> HW ...  When the reader
 * falls behind, the oldest DONE frame goes back to the firmware, so read()
 * always returns the newest frame and the camera never runs dry.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/vmem.h>

#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "fthd_var.h"
#include "fthd_isp.h"

/*
 * vmem doesn't hand out address 0, so the IOMMU arena is offset by its own
 * size; index = vmem address - FTHD_IOMMU_PAGES.
 */
#define IOMMU_ARENA_BASE	FTHD_IOMMU_PAGES

/* Page frame numbers are written to 32-bit IOMMU entries */
#define FTHD_DMA_LOWADDR	((1ULL << (32 + PAGE_SHIFT)) - 1)

#define FTHD_DEFAULT_FPS	FTHD_MAX_FPS

static void
fthd_iommu_clear(struct fthd_softc *sc, uint32_t base, uint32_t pages)
{
	uint32_t i;

	for (i = base; i < base + pages; i++)
		fthd_s2_reg_write(sc, 0, FTHD_IOMMU_REG + i * 4);
}

int
fthd_buffer_init(struct fthd_softc *sc)
{
	fthd_iommu_clear(sc, 0, FTHD_IOMMU_PAGES);

	sc->iommu = vmem_create("fthd iommu", IOMMU_ARENA_BASE,
	    FTHD_IOMMU_PAGES, 1, 0, M_WAITOK);
	if (sc->iommu == NULL)
		return (ENOMEM);

	STAILQ_INIT(&sc->done_q);
	return (0);
}

/* Full sensor resolution; needs the sensor size from the firmware */
void
fthd_format_default(struct fthd_softc *sc)
{
	sc->fmt.width = sc->sensor_width ? sc->sensor_width : 1280;
	sc->fmt.height = sc->sensor_height ? sc->sensor_height : 720;
	sc->fmt.pixelformat = FTHD_PIX_FMT_YUYV;
	sc->fmt.fps = FTHD_DEFAULT_FPS;
	fthd_format_adjust(sc, &sc->fmt);
}

void
fthd_buffer_uninit(struct fthd_softc *sc)
{
	if (sc->iommu != NULL) {
		vmem_destroy(sc->iommu);
		sc->iommu = NULL;
	}
}

void
fthd_format_adjust(struct fthd_softc *sc, struct fthd_format *fmt)
{
	uint32_t max_w = sc->sensor_width ? sc->sensor_width : 1280;
	uint32_t max_h = sc->sensor_height ? sc->sensor_height : 720;

	fmt->pixelformat = FTHD_PIX_FMT_YUYV;
	fmt->width = MAX(MIN(fmt->width, max_w), FTHD_MIN_WIDTH) & ~7u;
	fmt->height = MAX(MIN(fmt->height, max_h), FTHD_MIN_HEIGHT);
	fmt->fps = MAX(MIN(fmt->fps, FTHD_MAX_FPS), 1);
	fmt->bytesperline = fmt->width * 2;
	fmt->sizeimage = fmt->bytesperline * fmt->height;
}

struct fthd_load_ctx {
	struct fthd_softc	*sc;
	struct fthd_buf		*b;
	int			error;
};

/* Write each page of the buffer into its IOMMU entry */
static void
fthd_buf_load_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct fthd_load_ctx *ctx = arg;
	struct fthd_buf *b = ctx->b;
	bus_addr_t pa, end;
	uint32_t pos;
	int i;

	ctx->error = error;
	if (error)
		return;

	pos = b->iommu_base;
	for (i = 0; i < nseg; i++) {
		end = segs[i].ds_addr + segs[i].ds_len;
		if ((segs[i].ds_addr & PAGE_MASK) != 0 ||
		    (i < nseg - 1 && (end & PAGE_MASK) != 0)) {
			ctx->error = EINVAL;
			return;
		}
		for (pa = segs[i].ds_addr; pa < end; pa += PAGE_SIZE) {
			if (pos >= b->iommu_base + b->iommu_pages) {
				ctx->error = EFBIG;
				return;
			}
			fthd_s2_reg_write(ctx->sc, (uint32_t)(pa >> PAGE_SHIFT),
			    FTHD_IOMMU_REG + pos * 4);
			pos++;
		}
	}
}

static void
fthd_buf_free_one(struct fthd_softc *sc, struct fthd_buf *b)
{
	if (b->iommu_pages != 0) {
		fthd_iommu_clear(sc, b->iommu_base, b->iommu_pages);
		bus_dmamap_unload(sc->buf_dtag, b->map);
		vmem_free(sc->iommu, b->iommu_base + IOMMU_ARENA_BASE,
		    b->iommu_pages);
		b->iommu_pages = 0;
	}
	if (b->vaddr != NULL) {
		bus_dmamem_free(sc->buf_dtag, b->vaddr, b->map);
		b->vaddr = NULL;
	}
	isp_mem_destroy(sc, b->desc);
	b->desc = NULL;
}

static int
fthd_buf_alloc_one(struct fthd_softc *sc, struct fthd_buf *b, int idx)
{
	struct dma_descriptor_list list;
	struct fthd_load_ctx ctx;
	vmem_addr_t addr;
	bus_size_t size;
	int error;

	memset(b, 0, sizeof(*b));
	b->idx = idx;
	b->state = FTHD_BUF_IDLE;
	size = round_page(sc->fmt.sizeimage);

	error = bus_dmamem_alloc(sc->buf_dtag, &b->vaddr,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO, &b->map);
	if (error) {
		device_printf(sc->dev, "Failed to allocate frame buffer\n");
		return (error);
	}

	error = vmem_alloc(sc->iommu, atop(size), M_BESTFIT | M_WAITOK, &addr);
	if (error) {
		device_printf(sc->dev, "Out of IOMMU entries\n");
		goto fail;
	}
	b->iommu_base = addr - IOMMU_ARENA_BASE;
	b->iommu_pages = atop(size);

	ctx.sc = sc;
	ctx.b = b;
	ctx.error = 0;
	error = bus_dmamap_load(sc->buf_dtag, b->map, b->vaddr, size,
	    fthd_buf_load_cb, &ctx, BUS_DMA_NOWAIT);
	if (error == 0)
		error = ctx.error;
	if (error) {
		device_printf(sc->dev, "Failed to map frame buffer: %d\n",
		    error);
		goto fail;
	}

	b->desc = isp_mem_create(sc, FTHD_MEM_BUFFER, sizeof(list), M_WAITOK);
	if (b->desc == NULL) {
		error = ENOMEM;
		goto fail;
	}

	memset(&list, 0, sizeof(list));
	list.field0 = 1;
	list.count = 1;
	list.desc[0].count = 1;
	list.desc[0].pool = 0x02;
	list.desc[0].addr0 = FTHD_IOMMU_DEVADDR | (b->iommu_base << PAGE_SHIFT);
	list.desc[0].tag = ((uint64_t)FTHD_BUF_TAG_MAGIC << 32) | idx;
	fthd_s2_memcpy_toio(sc, b->desc->offset, &list, sizeof(list));
	return (0);

fail:
	fthd_buf_free_one(sc, b);
	return (error);
}

int
fthd_buffers_alloc(struct fthd_softc *sc)
{
	bus_size_t size;
	int error, i;

	sx_assert(&sc->dev_sx, SA_XLOCKED);
	if (sc->nbufs != 0)
		return (0);

	size = round_page(sc->fmt.sizeimage);
	if (atop(size) * FTHD_NBUF > FTHD_IOMMU_PAGES)
		return (EFBIG);

	/* One segment per page: the IOMMU maps pages, not ranges */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), PAGE_SIZE, 0,
	    FTHD_DMA_LOWADDR, BUS_SPACE_MAXADDR, NULL, NULL, size, atop(size),
	    PAGE_SIZE, 0, NULL, NULL, &sc->buf_dtag);
	if (error) {
		device_printf(sc->dev, "Failed to create DMA tag: %d\n", error);
		return (error);
	}

	for (i = 0; i < FTHD_NBUF; i++) {
		error = fthd_buf_alloc_one(sc, &sc->bufs[i], i);
		if (error)
			break;
		sc->nbufs++;
	}
	if (error) {
		fthd_buffers_free(sc);
		return (error);
	}

	FTHD_DBG(sc, "%d buffers of %u bytes\n", sc->nbufs, sc->fmt.sizeimage);
	return (0);
}

void
fthd_buffers_free(struct fthd_softc *sc)
{
	int i;

	sx_assert(&sc->dev_sx, SA_XLOCKED);
	KASSERT(!sc->streaming, ("freeing buffers while streaming"));

	for (i = 0; i < sc->nbufs; i++)
		fthd_buf_free_one(sc, &sc->bufs[i]);
	sc->nbufs = 0;
	STAILQ_INIT(&sc->done_q);

	if (sc->buf_dtag != NULL) {
		bus_dma_tag_destroy(sc->buf_dtag);
		sc->buf_dtag = NULL;
	}
}

/*
 * Hand a buffer to the firmware.  Doesn't wait for the firmware to pick
 * up the descriptor, so it can run from the interrupt task.
 */
static int
fthd_buf_submit(struct fthd_softc *sc, struct fthd_buf *b, uint32_t *entryp)
{
	int error;

	mtx_assert(&sc->buf_mtx, MA_OWNED);

	bus_dmamap_sync(sc->buf_dtag, b->map, BUS_DMASYNC_PREREAD);
	error = fthd_ringbuf_send(sc, sc->channel_buf_h2t, b->desc->offset,
	    sizeof(struct dma_descriptor_list), 0x30000000, entryp);
	if (error) {
		device_printf(sc->dev, "Failed to queue buffer %d: %d\n",
		    b->idx, error);
		b->state = FTHD_BUF_IDLE;
		return (error);
	}
	b->state = FTHD_BUF_HW;
	return (0);
}

/* Give a buffer back after read() copied it out */
void
fthd_buffer_requeue(struct fthd_softc *sc, struct fthd_buf *b)
{
	mtx_lock(&sc->buf_mtx);
	if (sc->streaming)
		fthd_buf_submit(sc, b, NULL);
	else
		b->state = FTHD_BUF_IDLE;
	mtx_unlock(&sc->buf_mtx);
}

/* Called from the interrupt task with a descriptor list from BUF_T2H */
void
fthd_buffer_return_handler(struct fthd_softc *sc, uint32_t offset)
{
	struct dma_descriptor_list list;
	struct fthd_buf *b, *old;
	uint64_t tag;
	uint32_t i, idx;
	bool wake = false;

	if (offset == 0)
		return;
	if (fthd_s2_memcpy_fromio(sc, &list, offset, sizeof(list)) != 0)
		return;

	mtx_lock(&sc->buf_mtx);
	for (i = 0; i < MIN(list.count, nitems(list.desc)); i++) {
		tag = list.desc[i].tag;
		idx = (uint32_t)tag;
		if ((tag >> 32) != FTHD_BUF_TAG_MAGIC || idx >= sc->nbufs) {
			device_printf(sc->dev,
			    "Firmware returned unknown buffer tag %016jx\n",
			    (uintmax_t)tag);
			continue;
		}
		b = &sc->bufs[idx];
		if (b->state != FTHD_BUF_HW) {
			device_printf(sc->dev,
			    "Firmware returned buffer %u in state %d\n",
			    idx, b->state);
			continue;
		}

		bus_dmamap_sync(sc->buf_dtag, b->map, BUS_DMASYNC_POSTREAD);

		if (!sc->streaming) {
			/* Returned by CH_BUFFER_RETURN while stopping */
			b->state = FTHD_BUF_IDLE;
			wake = true;
			continue;
		}

		b->sequence = sc->sequence++;
		nanouptime(&b->timestamp);
		b->state = FTHD_BUF_DONE;
		STAILQ_INSERT_TAIL(&sc->done_q, b, link);
		sc->frames_total++;
		wake = true;

		/* Keep only the newest frame waiting; recycle the rest */
		while ((old = STAILQ_FIRST(&sc->done_q)) != b) {
			STAILQ_REMOVE_HEAD(&sc->done_q, link);
			sc->dropped++;
			sc->drops_total++;
			fthd_buf_submit(sc, old, NULL);
		}
	}
	if (wake) {
		wakeup(&sc->done_q);
		selwakeup(&sc->rsel);
	}
	mtx_unlock(&sc->buf_mtx);
}

int
fthd_stream_start(struct fthd_softc *sc)
{
	uint32_t entry = 0;
	int error, i, queued = 0;

	sx_assert(&sc->dev_sx, SA_XLOCKED);
	if (sc->streaming)
		return (0);

	error = fthd_buffers_alloc(sc);
	if (error)
		return (error);

	device_printf(sc->dev, "Starting capture: %ux%u YUYV at %u fps\n",
	    sc->fmt.width, sc->fmt.height, sc->fmt.fps);

	error = fthd_start_channel(sc);
	if (error) {
		device_printf(sc->dev, "Channel start failed: %d\n", error);
		/* The channel may be half set up; stopping it is harmless */
		fthd_stop_channel(sc);
		return (error);
	}

	mtx_lock(&sc->buf_mtx);
	sc->sequence = 0;
	sc->dropped = 0;
	sc->streaming = true;
	mtx_unlock(&sc->buf_mtx);

	/* Like Linux, wait for the firmware to take each descriptor */
	for (i = 0; i < sc->nbufs; i++) {
		mtx_lock(&sc->buf_mtx);
		error = fthd_buf_submit(sc, &sc->bufs[i], &entry);
		mtx_unlock(&sc->buf_mtx);
		if (error == 0)
			error = fthd_ringbuf_wait_ready(sc,
			    sc->channel_buf_h2t, entry, 2000);
		if (error == 0)
			queued++;
	}

	if (queued == 0) {
		device_printf(sc->dev, "Firmware took no buffers\n");
		fthd_stream_stop(sc);
		return (EIO);
	}
	return (0);
}

/*
 * Stop the channel and get every buffer back.  If the firmware keeps some,
 * they are leaked, not freed: it could still DMA into them.
 */
void
fthd_stream_stop(struct fthd_softc *sc)
{
	sbintime_t deadline;
	int error, i, held;

	sx_assert(&sc->dev_sx, SA_XLOCKED);

	mtx_lock(&sc->buf_mtx);
	if (!sc->streaming) {
		mtx_unlock(&sc->buf_mtx);
		return;
	}
	sc->streaming = false;
	/* Frames nobody read yet won't be */
	while (!STAILQ_EMPTY(&sc->done_q)) {
		STAILQ_FIRST(&sc->done_q)->state = FTHD_BUF_IDLE;
		STAILQ_REMOVE_HEAD(&sc->done_q, link);
	}
	mtx_unlock(&sc->buf_mtx);

	error = fthd_stop_channel(sc);
	if (error)
		device_printf(sc->dev, "Channel stop failed: %d\n", error);

	deadline = getsbinuptime() + SBT_1S;
	mtx_lock(&sc->buf_mtx);
	for (;;) {
		held = 0;
		for (i = 0; i < sc->nbufs; i++)
			if (sc->bufs[i].state == FTHD_BUF_HW)
				held++;
		if (held == 0 || getsbinuptime() >= deadline)
			break;
		mtx_sleep(&sc->done_q, &sc->buf_mtx, 0, "fthdst", hz / 20 + 1);
	}
	mtx_unlock(&sc->buf_mtx);

	if (held != 0) {
		device_printf(sc->dev, "Firmware kept %d buffers; leaking "
		    "them\n", held);
		/* Forget the buffers without freeing them */
		sc->nbufs = 0;
		sc->buf_dtag = NULL;
		STAILQ_INIT(&sc->done_q);
	}

	device_printf(sc->dev, "Capture stopped: %u frames, %u dropped\n",
	    sc->sequence, sc->dropped);
}
