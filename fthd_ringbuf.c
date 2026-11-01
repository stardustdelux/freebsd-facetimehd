/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * Firmware IPC ring buffers, from the Linux facetimehd driver:
 *   Copyright (C) 2015 Sven Schnelle <svens@stackframe.org>
 *
 * Each channel is a ring of 64-byte entries in S2 memory.  Bit 0 of the
 * address word says who owns an entry; which value means "ours" depends on
 * the channel direction.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>

#include <dev/pci/pcivar.h>

#include "fthd_var.h"

/* True when the entry has been handed back to the host side */
static inline bool
fthd_entry_ready(struct fthd_softc *sc, struct fw_channel *chan,
    uint32_t entry)
{
	return (((fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS) & 1) ^
	    (chan->type != 0)) != 0);
}

uint32_t
fthd_ringbuf_entry_addr(struct fw_channel *chan, int num)
{
	return (chan->offset + num * FTHD_RINGBUF_ENTRY_SIZE);
}

void
fthd_ringbuf_dump(struct fthd_softc *sc, struct fw_channel *chan)
{
	uint32_t entry;
	int i;

	for (i = 0; i < chan->size; i++) {
		entry = fthd_ringbuf_entry_addr(chan, i);
		device_printf(sc->dev,
		    "%s: %c%3d: ADDRESS %08x REQUEST_SIZE %08x RESPONSE_SIZE %08x\n",
		    chan->name, chan->idx == i ? '*' : ' ', i,
		    fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_ADDRESS_FLAGS),
		    fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_REQUEST_SIZE),
		    fthd_s2_mem_read(sc, entry + FTHD_RINGBUF_RESPONSE_SIZE));
	}
}

void
fthd_ringbuf_init(struct fthd_softc *sc, struct fw_channel *chan)
{
	uint32_t entry;
	int i;

	mtx_lock(&sc->mtx);
	chan->idx = 0;
	if (chan->type == FW_CHAN_TYPE_OUT) {
		FTHD_DBG(sc, "clearing ringbuf %s at %08x (size %u)\n",
		    chan->name, chan->offset, chan->size);
		for (i = 0; i < chan->size; i++) {
			entry = fthd_ringbuf_entry_addr(chan, i);
			fthd_s2_mem_write(sc, 1, entry + FTHD_RINGBUF_ADDRESS_FLAGS);
			fthd_s2_mem_write(sc, 0, entry + FTHD_RINGBUF_REQUEST_SIZE);
			fthd_s2_mem_write(sc, 0, entry + FTHD_RINGBUF_RESPONSE_SIZE);
		}
	}
	mtx_unlock(&sc->mtx);
}

int
fthd_ringbuf_send(struct fthd_softc *sc, struct fw_channel *chan,
    uint32_t data_offset, uint32_t request_size, uint32_t response_size,
    uint32_t *entryp)
{
	uint32_t entry;

	FTHD_DBG(sc, "%s: send %08x\n", chan->name, data_offset);

	mtx_lock(&sc->mtx);
	entry = fthd_ringbuf_entry_addr(chan, chan->idx);
	if (++chan->idx >= chan->size)
		chan->idx = 0;

	if (!fthd_entry_ready(sc, chan, entry)) {
		mtx_unlock(&sc->mtx);
		return (EAGAIN);
	}

	fthd_s2_mem_write(sc, request_size, entry + FTHD_RINGBUF_REQUEST_SIZE);
	fthd_s2_mem_write(sc, response_size, entry + FTHD_RINGBUF_RESPONSE_SIZE);
	wmb();
	fthd_s2_mem_write(sc, data_offset | (chan->type == 0 ? 0 : 1),
	    entry + FTHD_RINGBUF_ADDRESS_FLAGS);

	/* Ring the doorbell for this channel's source */
	fthd_isp_reg_write(sc, 0x10 << chan->source, ISP_REG_41020);
	mtx_unlock(&sc->mtx);

	if (entryp != NULL)
		*entryp = entry;
	return (0);
}

/* Returns the next entry the target has filled in, or (uint32_t)-1 */
uint32_t
fthd_ringbuf_receive(struct fthd_softc *sc, struct fw_channel *chan)
{
	uint32_t entry, ret = (uint32_t)-1;

	mtx_lock(&sc->mtx);
	entry = fthd_ringbuf_entry_addr(chan, chan->idx);
	if (fthd_entry_ready(sc, chan, entry)) {
		ret = entry;
		if (chan->type == FW_CHAN_TYPE_OUT && ++chan->idx >= chan->size)
			chan->idx = 0;
	}
	mtx_unlock(&sc->mtx);
	return (ret);
}

/*
 * Sleep until the firmware completes the given entry.  The interrupt task
 * calls wakeup(chan) with sc->mtx held, so checking the condition under the
 * same lock cannot miss a completion.
 */
int
fthd_ringbuf_wait_ready(struct fthd_softc *sc, struct fw_channel *chan,
    uint32_t entry, int timeout_ms)
{
	sbintime_t deadline;
	int error = 0;

	deadline = getsbinuptime() + SBT_1MS * timeout_ms;

	mtx_lock(&sc->mtx);
	while (!fthd_entry_ready(sc, chan, entry)) {
		if (getsbinuptime() >= deadline) {
			error = ETIMEDOUT;
			break;
		}
		/* Poll every 10ms in case an interrupt goes missing */
		mtx_sleep(chan, &sc->mtx, 0, "fthdch", hz / 100 + 1);
	}
	mtx_unlock(&sc->mtx);

	if (error) {
		device_printf(sc->dev, "%s: timeout\n", chan->name);
		fthd_ringbuf_dump(sc, chan);
	}
	return (error);
}
