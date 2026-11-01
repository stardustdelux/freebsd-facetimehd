/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * fthd-v4l2d: expose the FacetimeHD camera (/dev/fthdN) as a V4L2 capture
 * device (/dev/videoN) through cuse(3), so that applications written for
 * V4L2 (browsers, OBS, ffmpeg, PipeWire) can use it.
 *
 * Supports the YUYV format at a list of discrete sizes, mmap streaming and
 * read().  Frames are read from the kernel driver straight into the buffer
 * the application mmapped, so there is one copy per frame.
 *
 * Unsupported ioctls return EINVAL (cuse has no way to return ENOTTY);
 * V4L2 applications treat both as "not supported".
 */

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/queue.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <cuse.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "../fthd_ioctl.h"

#define NWORKERS	8
#define MAX_BUFFERS	8
#define DQBUF_POLL_MS	100

enum buf_state {
	B_USER,			/* dequeued, owned by the application */
	B_QUEUED,		/* waiting to be filled */
	B_CAPTURING,		/* being filled by the capture thread */
	B_DONE,			/* filled, waiting for DQBUF */
};

struct vbuf {
	enum buf_state		state;
	uint32_t		index;
	void			*mem;		/* cuse_vmalloc'ed, mmappable */
	uint32_t		length;
	uint32_t		bytesused;
	uint32_t		sequence;
	struct timeval		timestamp;
	TAILQ_ENTRY(vbuf)	link;		/* queued or done list */
};
TAILQ_HEAD(vbuf_list, vbuf);

enum fh_mode {
	FH_NONE,
	FH_MMAP,		/* owns the buffers through REQBUFS */
	FH_READ,		/* owns the buffers through read() */
};

struct fh {
	enum fh_mode		mode;
};

/* One camera, one set of buffers; everything below is under mtx */
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv_queued = PTHREAD_COND_INITIALIZER;
static pthread_cond_t cv_done = PTHREAD_COND_INITIALIZER;

static const char *cam_path = "/dev/fthd0";
static int cam_fd = -1;
static struct fthd_format fmt;
static uint32_t max_width = 1280, max_height = 720;

static struct vbuf bufs[MAX_BUFFERS];
static uint32_t nbufs;
static struct vbuf_list queued = TAILQ_HEAD_INITIALIZER(queued);
static struct vbuf_list done = TAILQ_HEAD_INITIALIZER(done);
static struct fh *owner;
static bool streaming, capture_error;
static pthread_t capture_thread;

static bool foreground;

static void
logmsg(int prio, const char *f, ...)
{
	va_list ap;

	va_start(ap, f);
	if (foreground) {
		vfprintf(stderr, f, ap);
		fputc('\n', stderr);
	} else {
		vsyslog(prio, f, ap);
	}
	va_end(ap);
}

/*
 * Sizes we offer.  The ISP crops the largest centred window with the
 * requested aspect ratio and scales it, so any of these works.
 */
static const struct {
	uint32_t w, h;
} sizes[] = {
	{ 1280, 720 }, { 1024, 576 }, { 960, 540 }, { 848, 480 },
	{ 640, 480 }, { 640, 360 }, { 424, 240 }, { 320, 240 },
};

static const uint32_t rates[] = { 30, 25, 20, 15, 10, 5 };

static bool
size_ok(uint32_t w, uint32_t h)
{
	return (w <= max_width && h <= max_height);
}

/* Nearest offered size that is not larger than the request, else smallest */
static void
pick_size(uint32_t *w, uint32_t *h)
{
	size_t i, best = nitems(sizes);

	for (i = 0; i < nitems(sizes); i++) {
		if (!size_ok(sizes[i].w, sizes[i].h))
			continue;
		if (sizes[i].w <= *w && sizes[i].h <= *h) {
			best = i;
			break;
		}
	}
	if (best == nitems(sizes)) {
		for (i = nitems(sizes); i-- > 0;)
			if (size_ok(sizes[i].w, sizes[i].h)) {
				best = i;
				break;
			}
	}
	if (best == nitems(sizes)) {
		/* Sensor smaller than all our sizes: use it as is */
		*w = max_width & ~7u;
		*h = max_height;
		return;
	}
	*w = sizes[best].w;
	*h = sizes[best].h;
}

static void
fill_pix(struct v4l2_pix_format *pix, const struct fthd_format *f)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = f->width;
	pix->height = f->height;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = f->width * 2;
	pix->sizeimage = f->width * 2 * f->height;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
}

static void
adjust_format(struct fthd_format *f, uint32_t w, uint32_t h)
{
	pick_size(&w, &h);
	f->width = w;
	f->height = h;
	f->pixelformat = FTHD_PIX_FMT_YUYV;
	f->bytesperline = w * 2;
	f->sizeimage = w * 2 * h;
}

static void
buffers_free(void)
{
	uint32_t i;

	for (i = 0; i < nbufs; i++) {
		cuse_vmfree(bufs[i].mem);
		bufs[i].mem = NULL;
	}
	nbufs = 0;
	TAILQ_INIT(&queued);
	TAILQ_INIT(&done);
}

static int
buffers_alloc(uint32_t count)
{
	uint32_t i, len;

	len = roundup2(fmt.sizeimage, getpagesize());
	for (i = 0; i < count; i++) {
		memset(&bufs[i], 0, sizeof(bufs[i]));
		bufs[i].mem = cuse_vmalloc(len);
		if (bufs[i].mem == NULL)
			break;
		bufs[i].index = i;
		bufs[i].length = fmt.sizeimage;
		bufs[i].state = B_USER;
		nbufs++;
	}
	if (nbufs < 2) {
		buffers_free();
		return (CUSE_ERR_NO_MEMORY);
	}
	return (0);
}

/* Fill queued buffers from the camera until streaming stops */
static void *
capture_main(void *arg __unused)
{
	struct fthd_frame_info info;
	struct vbuf *b;
	ssize_t n;

	pthread_mutex_lock(&mtx);
	while (streaming) {
		b = TAILQ_FIRST(&queued);
		if (b == NULL) {
			pthread_cond_wait(&cv_queued, &mtx);
			continue;
		}
		TAILQ_REMOVE(&queued, b, link);
		b->state = B_CAPTURING;
		pthread_mutex_unlock(&mtx);

		n = read(cam_fd, b->mem, b->length);
		if (n >= 0 && ioctl(cam_fd, FTHDIOC_G_FRAME, &info) < 0)
			n = -1;

		pthread_mutex_lock(&mtx);
		if (n < 0 || (size_t)n != b->length) {
			logmsg(LOG_ERR, "camera read failed: %s",
			    n < 0 ? strerror(errno) : "short frame");
			b->state = B_QUEUED;
			TAILQ_INSERT_HEAD(&queued, b, link);
			capture_error = true;
			pthread_cond_broadcast(&cv_done);
			break;
		}
		if (b->state != B_CAPTURING)	/* reset by stream_off */
			continue;
		b->bytesused = n;
		b->sequence = info.sequence;
		TIMESPEC_TO_TIMEVAL(&b->timestamp, &info.timestamp);
		b->state = B_DONE;
		TAILQ_INSERT_TAIL(&done, b, link);
		pthread_cond_broadcast(&cv_done);
		pthread_mutex_unlock(&mtx);
		cuse_poll_wakeup();
		pthread_mutex_lock(&mtx);
	}
	pthread_mutex_unlock(&mtx);
	cuse_poll_wakeup();
	return (NULL);
}

/* Called with mtx held */
static int
stream_on(void)
{
	struct fthd_format kf;

	if (streaming)
		return (0);
	if (nbufs == 0)
		return (CUSE_ERR_INVALID);

	cam_fd = open(cam_path, O_RDONLY);
	if (cam_fd < 0) {
		logmsg(LOG_ERR, "%s: %s", cam_path, strerror(errno));
		return (errno == EBUSY ? CUSE_ERR_BUSY : CUSE_ERR_NO_DEVICE);
	}
	kf = fmt;
	if (ioctl(cam_fd, FTHDIOC_S_FMT, &kf) < 0 ||
	    kf.width != fmt.width || kf.height != fmt.height) {
		logmsg(LOG_ERR, "camera refused %ux%u", fmt.width, fmt.height);
		close(cam_fd);
		cam_fd = -1;
		return (CUSE_ERR_INVALID);
	}
	if (ioctl(cam_fd, FTHDIOC_STREAMON) < 0) {
		logmsg(LOG_ERR, "camera stream start: %s", strerror(errno));
		close(cam_fd);
		cam_fd = -1;
		return (CUSE_ERR_OTHER);
	}

	streaming = true;
	capture_error = false;
	if (pthread_create(&capture_thread, NULL, capture_main, NULL) != 0) {
		streaming = false;
		close(cam_fd);
		cam_fd = -1;
		return (CUSE_ERR_NO_MEMORY);
	}
	logmsg(LOG_INFO, "streaming %ux%u at %u fps", fmt.width, fmt.height,
	    fmt.fps);
	return (0);
}

/* Called with mtx held; returns every buffer to the application */
static void
stream_off(void)
{
	uint32_t i;

	if (streaming) {
		streaming = false;
		pthread_cond_broadcast(&cv_queued);
		pthread_cond_broadcast(&cv_done);
		/* The capture thread needs mtx to finish */
		pthread_mutex_unlock(&mtx);
		pthread_join(capture_thread, NULL);
		pthread_mutex_lock(&mtx);
		close(cam_fd);
		cam_fd = -1;
		logmsg(LOG_INFO, "streaming stopped");
	}
	TAILQ_INIT(&queued);
	TAILQ_INIT(&done);
	for (i = 0; i < nbufs; i++)
		bufs[i].state = B_USER;
}

/* Release everything a file handle owns; called with mtx held */
static void
release_owner(struct fh *fh)
{
	if (owner != fh)
		return;
	stream_off();
	buffers_free();
	owner = NULL;
	fh->mode = FH_NONE;
}

/* Wait for a filled buffer; called with mtx held */
static int
wait_done(int fflags, struct vbuf **bp)
{
	struct timespec ts;

	while (TAILQ_EMPTY(&done)) {
		if (!streaming)
			return (CUSE_ERR_INVALID);
		if (capture_error)
			return (CUSE_ERR_OTHER);
		if (fflags & CUSE_FFLAG_NONBLOCK)
			return (CUSE_ERR_WOULDBLOCK);
		if (cuse_got_peer_signal() == 0)
			return (CUSE_ERR_SIGNAL);
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_nsec += DQBUF_POLL_MS * 1000000L;
		if (ts.tv_nsec >= 1000000000L) {
			ts.tv_sec++;
			ts.tv_nsec -= 1000000000L;
		}
		pthread_cond_timedwait(&cv_done, &mtx, &ts);
	}
	*bp = TAILQ_FIRST(&done);
	TAILQ_REMOVE(&done, *bp, link);
	(*bp)->state = B_USER;
	return (0);
}

static void
fill_vbuf(struct v4l2_buffer *vb, const struct vbuf *b)
{
	uint32_t type = vb->type;

	memset(vb, 0, sizeof(*vb));
	vb->type = type;
	vb->index = b->index;
	vb->memory = V4L2_MEMORY_MMAP;
	vb->m.offset = cuse_vmoffset(b->mem);
	vb->length = b->length;
	vb->field = V4L2_FIELD_NONE;
	vb->flags = V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
	    V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	if (b->state == B_QUEUED || b->state == B_CAPTURING)
		vb->flags |= V4L2_BUF_FLAG_QUEUED;
	else if (b->state == B_DONE)
		vb->flags |= V4L2_BUF_FLAG_DONE;
	vb->bytesused = b->bytesused;
	vb->sequence = b->sequence;
	vb->timestamp = b->timestamp;
}

static int
v4l2_open(struct cuse_dev *cdev, int fflags __unused)
{
	struct fh *fh;

	fh = calloc(1, sizeof(*fh));
	if (fh == NULL)
		return (CUSE_ERR_NO_MEMORY);
	cuse_dev_set_per_file_handle(cdev, fh);
	return (0);
}

static int
v4l2_close(struct cuse_dev *cdev, int fflags __unused)
{
	struct fh *fh = cuse_dev_get_per_file_handle(cdev);

	pthread_mutex_lock(&mtx);
	release_owner(fh);
	pthread_mutex_unlock(&mtx);
	free(fh);
	return (0);
}

/* read() I/O: the reader owns a private set of buffers */
static int
v4l2_read(struct cuse_dev *cdev, int fflags, void *user_ptr, int len)
{
	struct fh *fh = cuse_dev_get_per_file_handle(cdev);
	struct vbuf *b;
	uint32_t i;
	int error, n;

	pthread_mutex_lock(&mtx);
	if (owner != NULL && owner != fh) {
		error = CUSE_ERR_BUSY;
		goto out;
	}
	if (fh->mode == FH_MMAP) {
		error = CUSE_ERR_BUSY;
		goto out;
	}
	if (fh->mode == FH_NONE) {
		error = buffers_alloc(2);
		if (error)
			goto out;
		owner = fh;
		fh->mode = FH_READ;
		for (i = 0; i < nbufs; i++) {
			bufs[i].state = B_QUEUED;
			TAILQ_INSERT_TAIL(&queued, &bufs[i], link);
		}
		error = stream_on();
		if (error) {
			release_owner(fh);
			goto out;
		}
	}

	error = wait_done(fflags, &b);
	if (error)
		goto out;
	n = MIN((uint32_t)len, b->bytesused);
	/* Copying out doesn't need the lock, but b must stay ours */
	pthread_mutex_unlock(&mtx);
	error = cuse_copy_out(b->mem, user_ptr, n);
	pthread_mutex_lock(&mtx);
	if (streaming && owner == fh) {
		b->state = B_QUEUED;
		TAILQ_INSERT_TAIL(&queued, b, link);
		pthread_cond_signal(&cv_queued);
	}
	if (error == 0)
		error = n;
out:
	pthread_mutex_unlock(&mtx);
	return (error);
}

static int
v4l2_poll(struct cuse_dev *cdev, int fflags __unused, int events)
{
	struct fh *fh = cuse_dev_get_per_file_handle(cdev);
	int revents = CUSE_POLL_NONE;

	pthread_mutex_lock(&mtx);
	if (owner != fh || fh->mode == FH_NONE) {
		/* read() starts the stream; let the application call it */
		if (owner == NULL)
			revents = events & CUSE_POLL_READ;
	} else if (!TAILQ_EMPTY(&done)) {
		revents = events & CUSE_POLL_READ;
	} else if (!streaming || capture_error) {
		revents = CUSE_POLL_ERROR;
	}
	pthread_mutex_unlock(&mtx);
	return (revents);
}

static int
ioctl_reqbufs(struct fh *fh, struct v4l2_requestbuffers *rb)
{
	int error;

	if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return (CUSE_ERR_INVALID);
	if (rb->memory != V4L2_MEMORY_MMAP)
		return (CUSE_ERR_INVALID);
	if (owner != NULL && owner != fh)
		return (CUSE_ERR_BUSY);
	if (fh->mode == FH_READ)
		return (CUSE_ERR_BUSY);

	release_owner(fh);
	rb->flags = 0;
	memset(rb->reserved, 0, sizeof(rb->reserved));
	rb->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;
	if (rb->count == 0)
		return (0);

	error = buffers_alloc(MIN(MAX(rb->count, 2), MAX_BUFFERS));
	if (error)
		return (error);
	owner = fh;
	fh->mode = FH_MMAP;
	rb->count = nbufs;
	return (0);
}

static int
v4l2_ioctl(struct cuse_dev *cdev, int fflags, unsigned long cmd,
    void *user_data)
{
	struct fh *fh = cuse_dev_get_per_file_handle(cdev);
	union {
		struct v4l2_capability		cap;
		struct v4l2_fmtdesc		fmtdesc;
		struct v4l2_format		format;
		struct v4l2_frmsizeenum		frmsize;
		struct v4l2_frmivalenum		frmival;
		struct v4l2_streamparm		parm;
		struct v4l2_input		input;
		struct v4l2_requestbuffers	reqbufs;
		struct v4l2_buffer		buf;
		int				i;
		uint8_t				raw[IOCPARM_MAX];
	} u;
	struct fthd_format tf;
	struct v4l2_fract *tpf;
	struct vbuf *b;
	size_t len = IOCPARM_LEN(cmd);
	uint32_t fps;
	int error = 0;

	if (fflags & CUSE_FFLAG_COMPAT32)
		return (CUSE_ERR_INVALID);	/* layouts differ */
	if (len > sizeof(u))
		return (CUSE_ERR_INVALID);

	memset(&u, 0, sizeof(u));
	if ((cmd & IOC_IN) && len > 0) {
		error = cuse_copy_in(user_data, &u, len);
		if (error)
			return (error);
	}

	pthread_mutex_lock(&mtx);
	switch (cmd) {
	case VIDIOC_QUERYCAP:
		memset(&u.cap, 0, sizeof(u.cap));
		strlcpy((char *)u.cap.driver, "fthd", sizeof(u.cap.driver));
		strlcpy((char *)u.cap.card, "FaceTime HD Camera",
		    sizeof(u.cap.card));
		strlcpy((char *)u.cap.bus_info, "platform:fthd0",
		    sizeof(u.cap.bus_info));
		u.cap.version = 0x00060000;	/* compliance wants >= 3.0 */
		u.cap.device_caps = V4L2_CAP_VIDEO_CAPTURE |
		    V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
		u.cap.capabilities = u.cap.device_caps | V4L2_CAP_DEVICE_CAPS;
		break;

	case VIDIOC_ENUM_FMT:
		if (u.fmtdesc.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		    u.fmtdesc.index != 0) {
			error = CUSE_ERR_INVALID;
			break;
		}
		memset(&u.fmtdesc.flags, 0,
		    sizeof(u.fmtdesc) - offsetof(struct v4l2_fmtdesc, flags));
		u.fmtdesc.pixelformat = V4L2_PIX_FMT_YUYV;
		strlcpy((char *)u.fmtdesc.description, "YUYV 4:2:2",
		    sizeof(u.fmtdesc.description));
		break;

	case VIDIOC_G_FMT:
		if (u.format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		fill_pix(&u.format.fmt.pix, &fmt);
		break;

	case VIDIOC_TRY_FMT:
	case VIDIOC_S_FMT:
		if (u.format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		tf = fmt;
		adjust_format(&tf, u.format.fmt.pix.width,
		    u.format.fmt.pix.height);
		fill_pix(&u.format.fmt.pix, &tf);
		if (cmd == VIDIOC_TRY_FMT)
			break;
		if (tf.width == fmt.width && tf.height == fmt.height)
			break;
		/* Buffers are sized for the current format */
		if (nbufs != 0) {
			error = CUSE_ERR_BUSY;
			break;
		}
		fmt = tf;
		break;

	case VIDIOC_ENUM_FRAMESIZES: {
		uint32_t i, n = 0;

		if (u.frmsize.pixel_format != V4L2_PIX_FMT_YUYV) {
			error = CUSE_ERR_INVALID;
			break;
		}
		for (i = 0; i < nitems(sizes); i++) {
			if (!size_ok(sizes[i].w, sizes[i].h))
				continue;
			if (n++ == u.frmsize.index)
				break;
		}
		if (i == nitems(sizes)) {
			error = CUSE_ERR_INVALID;
			break;
		}
		memset(u.frmsize.reserved, 0, sizeof(u.frmsize.reserved));
		u.frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		u.frmsize.discrete.width = sizes[i].w;
		u.frmsize.discrete.height = sizes[i].h;
		break;
	}

	case VIDIOC_ENUM_FRAMEINTERVALS: {
		uint32_t w = u.frmival.width, h = u.frmival.height;

		pick_size(&w, &h);
		if (u.frmival.pixel_format != V4L2_PIX_FMT_YUYV ||
		    w != u.frmival.width || h != u.frmival.height ||
		    u.frmival.index >= nitems(rates)) {
			error = CUSE_ERR_INVALID;
			break;
		}
		memset(u.frmival.reserved, 0, sizeof(u.frmival.reserved));
		u.frmival.type = V4L2_FRMIVAL_TYPE_DISCRETE;
		u.frmival.discrete.numerator = 1;
		u.frmival.discrete.denominator = rates[u.frmival.index];
		break;
	}

	case VIDIOC_G_PARM:
	case VIDIOC_S_PARM:
		if (u.parm.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		tpf = &u.parm.parm.capture.timeperframe;
		if (cmd == VIDIOC_S_PARM && tpf->numerator != 0 &&
		    tpf->denominator != 0) {
			fps = (tpf->denominator + tpf->numerator / 2) /
			    tpf->numerator;
			fps = MAX(MIN(fps, FTHD_MAX_FPS), 1);
			if (fps != fmt.fps && streaming) {
				error = CUSE_ERR_BUSY;
				break;
			}
			fmt.fps = fps;
		}
		memset(&u.parm.parm, 0, sizeof(u.parm.parm));
		u.parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		u.parm.parm.capture.readbuffers = 2;
		tpf->numerator = 1;
		tpf->denominator = fmt.fps;
		break;

	case VIDIOC_ENUMINPUT:
		if (u.input.index != 0) {
			error = CUSE_ERR_INVALID;
			break;
		}
		memset(&u.input.name, 0,
		    sizeof(u.input) - offsetof(struct v4l2_input, name));
		strlcpy((char *)u.input.name, "Camera", sizeof(u.input.name));
		u.input.type = V4L2_INPUT_TYPE_CAMERA;
		break;

	case VIDIOC_G_INPUT:
		u.i = 0;
		break;

	case VIDIOC_S_INPUT:
		if (u.i != 0)
			error = CUSE_ERR_INVALID;
		break;

	case VIDIOC_REQBUFS:
		error = ioctl_reqbufs(fh, &u.reqbufs);
		break;

	case VIDIOC_QUERYBUF:
		if (u.buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		    u.buf.index >= nbufs) {
			error = CUSE_ERR_INVALID;
			break;
		}
		fill_vbuf(&u.buf, &bufs[u.buf.index]);
		break;

	case VIDIOC_QBUF:
		if (owner != fh || fh->mode != FH_MMAP ||
		    u.buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		    u.buf.memory != V4L2_MEMORY_MMAP ||
		    u.buf.index >= nbufs ||
		    bufs[u.buf.index].state != B_USER) {
			error = CUSE_ERR_INVALID;
			break;
		}
		b = &bufs[u.buf.index];
		b->state = B_QUEUED;
		b->bytesused = 0;
		TAILQ_INSERT_TAIL(&queued, b, link);
		pthread_cond_signal(&cv_queued);
		fill_vbuf(&u.buf, b);
		break;

	case VIDIOC_DQBUF:
		if (owner != fh || fh->mode != FH_MMAP ||
		    u.buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		error = wait_done(fflags, &b);
		if (error == 0)
			fill_vbuf(&u.buf, b);
		break;

	case VIDIOC_STREAMON:
		if (owner != fh || fh->mode != FH_MMAP ||
		    u.i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		error = stream_on();
		break;

	case VIDIOC_STREAMOFF:
		if (owner != fh || fh->mode != FH_MMAP ||
		    u.i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			error = CUSE_ERR_INVALID;
			break;
		}
		stream_off();
		break;

	default:
		error = CUSE_ERR_INVALID;
		break;
	}
	pthread_mutex_unlock(&mtx);

	if (error == 0 && (cmd & IOC_OUT) && len > 0)
		error = cuse_copy_out(&u, user_data, len);
	return (error);
}

static const struct cuse_methods v4l2_methods = {
	.cm_open = v4l2_open,
	.cm_close = v4l2_close,
	.cm_read = v4l2_read,
	.cm_poll = v4l2_poll,
	.cm_ioctl = v4l2_ioctl,
};

static void *
worker_main(void *arg __unused)
{
	for (;;) {
		if (cuse_wait_and_process() != 0)
			break;
	}
	return (NULL);
}

/* Learn the sensor size; the camera is exclusive, so don't keep it open */
static void
probe_camera(void)
{
	struct fthd_format kf;
	int fd;

	fd = open(cam_path, O_RDONLY);
	if (fd < 0)
		err(1, "%s", cam_path);
	if (ioctl(fd, FTHDIOC_G_FMT, &kf) < 0)
		err(1, "%s: FTHDIOC_G_FMT", cam_path);
	close(fd);

	/* The kernel driver's default format is the full sensor */
	max_width = kf.width;
	max_height = kf.height;
	fmt = kf;
	adjust_format(&fmt, kf.width, kf.height);
	fmt.fps = kf.fps;
}

static void
usage(void)
{
	fprintf(stderr,
	    "usage: fthd-v4l2d [-f] [-d camera] [-u unit] [-g group]\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	struct cuse_dev *cdev;
	struct group *gr;
	pthread_t workers[NWORKERS];
	sigset_t set;
	const char *group = "video";
	char path[32];
	gid_t gid;
	int ch, i, sig, unit = -1;

	while ((ch = getopt(argc, argv, "fd:u:g:")) != -1) {
		switch (ch) {
		case 'f':
			foreground = true;
			break;
		case 'd':
			cam_path = optarg;
			break;
		case 'u':
			unit = atoi(optarg);
			break;
		case 'g':
			group = optarg;
			break;
		default:
			usage();
		}
	}

	if ((gr = getgrnam(group)) == NULL)
		errx(1, "unknown group %s", group);
	gid = gr->gr_gid;

	probe_camera();

	/* First free /dev/videoN unless told otherwise */
	if (unit < 0) {
		for (unit = 0; unit < 64; unit++) {
			snprintf(path, sizeof(path), "/dev/video%d", unit);
			if (access(path, F_OK) != 0)
				break;
		}
	}

	if (cuse_init() != 0)
		errx(1, "cuse_init failed; is cuse.ko loaded?");

	if (!foreground) {
		if (daemon(0, 0) != 0)
			err(1, "daemon");
		openlog("fthd-v4l2d", LOG_PID, LOG_DAEMON);
	}

	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGTERM);
	sigaddset(&set, SIGHUP);
	pthread_sigmask(SIG_BLOCK, &set, NULL);

	cdev = cuse_dev_create(&v4l2_methods, NULL, NULL, 0, gid, 0660,
	    "video%d", unit);
	if (cdev == NULL) {
		logmsg(LOG_ERR, "could not create /dev/video%d", unit);
		return (1);
	}
	logmsg(LOG_INFO, "/dev/video%d -> %s, up to %ux%u", unit, cam_path,
	    max_width, max_height);

	for (i = 0; i < NWORKERS; i++)
		if (pthread_create(&workers[i], NULL, worker_main, NULL) != 0) {
			logmsg(LOG_ERR, "pthread_create failed");
			return (1);
		}

	sigwait(&set, &sig);
	logmsg(LOG_INFO, "exiting on signal %d", sig);

	pthread_mutex_lock(&mtx);
	if (owner != NULL)
		release_owner(owner);
	pthread_mutex_unlock(&mtx);
	cuse_dev_destroy(cdev);
	return (0);
}
