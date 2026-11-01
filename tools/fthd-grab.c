/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * fthd-grab: capture frames from /dev/fthdN, report the frame rate and
 * save the last frame as a PPM image.
 *
 *   fthd-grab [-d device] [-s WxH] [-f fps] [-n frames] [-o file.ppm]
 */

#include <sys/ioctl.h>
#include <sys/time.h>

#include <err.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../fthd_ioctl.h"

static void
usage(void)
{
	fprintf(stderr, "usage: fthd-grab [-d device] [-s WxH] [-f fps] "
	    "[-n frames] [-o file.ppm]\n");
	exit(1);
}

static uint8_t
clamp(int v)
{
	return (v < 0 ? 0 : v > 255 ? 255 : v);
}

/* BT.601 limited range YUYV to RGB */
static void
write_ppm(const char *path, const uint8_t *yuyv, const struct fthd_format *f)
{
	FILE *fp;
	uint8_t *row, *p;
	uint32_t x, y;
	int c, d, e, i;

	if ((fp = fopen(path, "wb")) == NULL)
		err(1, "%s", path);
	if ((row = malloc(f->width * 3)) == NULL)
		err(1, "malloc");

	fprintf(fp, "P6\n%u %u\n255\n", f->width, f->height);
	for (y = 0; y < f->height; y++) {
		const uint8_t *s = yuyv + y * f->bytesperline;

		p = row;
		for (x = 0; x < f->width; x += 2, s += 4) {
			d = s[1] - 128;
			e = s[3] - 128;
			for (i = 0; i < 2; i++) {
				c = (s[i * 2] - 16) * 298;
				*p++ = clamp((c + 409 * e + 128) >> 8);
				*p++ = clamp((c - 100 * d - 208 * e + 128) >> 8);
				*p++ = clamp((c + 516 * d + 128) >> 8);
			}
		}
		if (fwrite(row, 3, f->width, fp) != f->width)
			err(1, "%s", path);
	}
	free(row);
	if (fclose(fp) != 0)
		err(1, "%s", path);
}

static double
ts_diff(const struct timespec *a, const struct timespec *b)
{
	return ((b->tv_sec - a->tv_sec) + (b->tv_nsec - a->tv_nsec) / 1e9);
}

int
main(int argc, char **argv)
{
	const char *dev = "/dev/fthd0", *out = "frame.ppm";
	struct fthd_format fmt;
	struct fthd_frame_info first, info;
	uint8_t *buf;
	ssize_t n;
	int ch, fd, i, nframes = 60, w = 0, h = 0, fps = 0;

	while ((ch = getopt(argc, argv, "d:s:f:n:o:")) != -1) {
		switch (ch) {
		case 'd':
			dev = optarg;
			break;
		case 's':
			if (sscanf(optarg, "%dx%d", &w, &h) != 2)
				usage();
			break;
		case 'f':
			fps = atoi(optarg);
			break;
		case 'n':
			nframes = atoi(optarg);
			break;
		case 'o':
			out = optarg;
			break;
		default:
			usage();
		}
	}
	if (nframes < 1)
		usage();

	if ((fd = open(dev, O_RDONLY)) < 0)
		err(1, "%s", dev);

	if (ioctl(fd, FTHDIOC_G_FMT, &fmt) < 0)
		err(1, "FTHDIOC_G_FMT");
	if (w > 0)
		fmt.width = w;
	if (h > 0)
		fmt.height = h;
	if (fps > 0)
		fmt.fps = fps;
	if (ioctl(fd, FTHDIOC_S_FMT, &fmt) < 0)
		err(1, "FTHDIOC_S_FMT");
	printf("format: %ux%u YUYV, %u bytes/frame, %u fps\n", fmt.width,
	    fmt.height, fmt.sizeimage, fmt.fps);

	if ((buf = malloc(fmt.sizeimage)) == NULL)
		err(1, "malloc");

	memset(&first, 0, sizeof(first));
	for (i = 0; i < nframes; i++) {
		n = read(fd, buf, fmt.sizeimage);
		if (n < 0)
			err(1, "read frame %d", i);
		if ((size_t)n != fmt.sizeimage)
			errx(1, "short frame %d: %zd bytes", i, n);
		if (ioctl(fd, FTHDIOC_G_FRAME, &info) < 0)
			err(1, "FTHDIOC_G_FRAME");
		if (i == 0)
			first = info;
	}

	printf("%d frames read, sequence %u..%u, %u dropped by driver\n",
	    nframes, first.sequence, info.sequence, info.dropped);
	if (nframes > 1 && info.sequence > first.sequence)
		printf("camera rate: %.1f fps\n",
		    (info.sequence - first.sequence) /
		    ts_diff(&first.timestamp, &info.timestamp));

	write_ppm(out, buf, &fmt);
	printf("last frame written to %s\n", out);

	close(fd);
	free(buf);
	return (0);
}
