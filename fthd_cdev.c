/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * /dev/fthdN: one opener at a time; every read() returns one frame (the
 * newest one), truncated to the caller's buffer.  See fthd_ioctl.h.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/poll.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/uio.h>

#include <dev/pci/pcivar.h>

#include "fthd_var.h"

static d_open_t		fthd_open;
static d_close_t	fthd_close;
static d_read_t		fthd_read;
static d_ioctl_t	fthd_ioctl;
static d_poll_t		fthd_poll;

static struct cdevsw fthd_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	D_TRACKCLOSE,
	.d_open =	fthd_open,
	.d_close =	fthd_close,
	.d_read =	fthd_read,
	.d_ioctl =	fthd_ioctl,
	.d_poll =	fthd_poll,
	.d_name =	"fthd",
};

static int
fthd_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct fthd_softc *sc = dev->si_drv1;
	int error = 0;

	sx_xlock(&sc->dev_sx);
	if (sc->dying)
		error = ENXIO;
	else if (sc->is_open)
		error = EBUSY;
	else
		sc->is_open = true;
	sx_xunlock(&sc->dev_sx);
	return (error);
}

static int
fthd_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct fthd_softc *sc = dev->si_drv1;

	sx_xlock(&sc->dev_sx);
	fthd_stream_stop(sc);
	fthd_buffers_free(sc);
	sc->is_open = false;
	sx_xunlock(&sc->dev_sx);
	return (0);
}

static int
fthd_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct fthd_softc *sc = dev->si_drv1;
	struct fthd_buf *b;
	int error;

	sx_xlock(&sc->dev_sx);
	error = fthd_stream_start(sc);
	if (error)
		goto out;

	mtx_lock(&sc->buf_mtx);
	while ((b = STAILQ_FIRST(&sc->done_q)) == NULL) {
		if (sc->dying || !sc->streaming) {
			error = ENXIO;
			break;
		}
		if (ioflag & O_NONBLOCK) {
			error = EWOULDBLOCK;
			break;
		}
		error = mtx_sleep(&sc->done_q, &sc->buf_mtx, PCATCH, "fthdrd",
		    2 * hz);
		if (error == EWOULDBLOCK) {
			device_printf(sc->dev, "No frame for 2 seconds\n");
			error = EIO;
		}
		if (error)
			break;
	}
	if (b != NULL) {
		STAILQ_REMOVE_HEAD(&sc->done_q, link);
		b->state = FTHD_BUF_READING;
		sc->last_frame.sequence = b->sequence;
		sc->last_frame.dropped = sc->dropped;
		sc->last_frame.timestamp = b->timestamp;
	}
	mtx_unlock(&sc->buf_mtx);
	if (b == NULL)
		goto out;

	error = uiomove(b->vaddr, MIN(uio->uio_resid, sc->fmt.sizeimage), uio);
	fthd_buffer_requeue(sc, b);
out:
	sx_xunlock(&sc->dev_sx);
	return (error);
}

static int
fthd_poll(struct cdev *dev, int events, struct thread *td)
{
	struct fthd_softc *sc = dev->si_drv1;
	int revents = 0;

	if ((events & (POLLIN | POLLRDNORM)) == 0)
		return (0);

	sx_xlock(&sc->dev_sx);
	/* Like V4L2 read I/O, polling starts the stream */
	if (fthd_stream_start(sc) != 0) {
		sx_xunlock(&sc->dev_sx);
		return (POLLERR);
	}

	mtx_lock(&sc->buf_mtx);
	if (!STAILQ_EMPTY(&sc->done_q))
		revents = events & (POLLIN | POLLRDNORM);
	else
		selrecord(td, &sc->rsel);
	mtx_unlock(&sc->buf_mtx);
	sx_xunlock(&sc->dev_sx);
	return (revents);
}

static int
fthd_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct fthd_softc *sc = dev->si_drv1;
	struct fthd_format *fmt;
	int error = 0;

	sx_xlock(&sc->dev_sx);
	switch (cmd) {
	case FTHDIOC_G_FMT:
		*(struct fthd_format *)data = sc->fmt;
		break;
	case FTHDIOC_S_FMT:
		fmt = (struct fthd_format *)data;
		fthd_format_adjust(sc, fmt);
		if (sc->streaming) {
			if (memcmp(fmt, &sc->fmt, sizeof(*fmt)) != 0)
				error = EBUSY;
			break;
		}
		if (memcmp(fmt, &sc->fmt, sizeof(*fmt)) != 0) {
			fthd_buffers_free(sc);
			sc->fmt = *fmt;
		}
		break;
	case FTHDIOC_STREAMON:
		error = fthd_stream_start(sc);
		break;
	case FTHDIOC_STREAMOFF:
		fthd_stream_stop(sc);
		break;
	case FTHDIOC_G_FRAME:
		*(struct fthd_frame_info *)data = sc->last_frame;
		break;
	default:
		error = ENOTTY;
		break;
	}
	sx_xunlock(&sc->dev_sx);
	return (error);
}

int
fthd_cdev_create(struct fthd_softc *sc)
{
	struct make_dev_args args;

	make_dev_args_init(&args);
	args.mda_devsw = &fthd_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_VIDEO;
	args.mda_mode = 0660;
	args.mda_si_drv1 = sc;
	return (make_dev_s(&args, &sc->cdev, "fthd%d",
	    device_get_unit(sc->dev)));
}

/*
 * Wake sleeping readers so destroy_dev() can drain them.  After it returns
 * no thread is inside the cdev and close() will never be called, so the
 * stream is torn down here.
 */
void
fthd_cdev_destroy(struct fthd_softc *sc)
{
	if (sc->cdev == NULL)
		return;

	mtx_lock(&sc->buf_mtx);
	sc->dying = true;
	wakeup(&sc->done_q);
	selwakeup(&sc->rsel);
	mtx_unlock(&sc->buf_mtx);

	destroy_dev(sc->cdev);
	sc->cdev = NULL;

	sx_xlock(&sc->dev_sx);
	fthd_stream_stop(sc);
	fthd_buffers_free(sc);
	sc->is_open = false;
	sx_xunlock(&sc->dev_sx);
	seldrain(&sc->rsel);
}
