/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * Userland interface of /dev/fthdN.  Each read() returns one frame in the
 * current format; streaming starts on the first read() or poll() and stops
 * on close.  Frames are packed YUYV (4:2:2), bytesperline = width * 2.
 */

#ifndef _FTHD_IOCTL_H
#define _FTHD_IOCTL_H

#include <sys/types.h>
#include <sys/ioccom.h>
#include <sys/time.h>

#define FTHD_FOURCC(a, b, c, d)						\
	((uint32_t)(a) | ((uint32_t)(b) << 8) |				\
	((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define FTHD_PIX_FMT_YUYV	FTHD_FOURCC('Y', 'U', 'Y', 'V')

#define FTHD_MIN_WIDTH		320
#define FTHD_MIN_HEIGHT		240
#define FTHD_MAX_FPS		30

struct fthd_format {
	uint32_t	width;		/* multiple of 8 */
	uint32_t	height;
	uint32_t	pixelformat;	/* FTHD_PIX_FMT_* */
	uint32_t	bytesperline;	/* filled in by the driver */
	uint32_t	sizeimage;	/* filled in by the driver */
	uint32_t	fps;		/* 1 .. FTHD_MAX_FPS */
};

/* Describes the frame returned by the most recent read() */
struct fthd_frame_info {
	uint32_t	sequence;	/* frames captured since stream start */
	uint32_t	dropped;	/* frames dropped since stream start */
	struct timespec	timestamp;	/* CLOCK_MONOTONIC, at completion */
};

/* Get the format; S_FMT adjusts the request to what the camera can do */
#define FTHDIOC_G_FMT		_IOR('F', 1, struct fthd_format)
#define FTHDIOC_S_FMT		_IOWR('F', 2, struct fthd_format)
#define FTHDIOC_STREAMON	_IO('F', 3)
#define FTHDIOC_STREAMOFF	_IO('F', 4)
#define FTHDIOC_G_FRAME		_IOR('F', 5, struct fthd_frame_info)

#endif /* _FTHD_IOCTL_H */
