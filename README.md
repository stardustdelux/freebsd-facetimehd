# fthd — FacetimeHD camera driver for FreeBSD

Driver for the Broadcom 1570 "FaceTime HD" PCIe camera found in 2013–2017
MacBooks, ported from the Linux [facetimehd](https://github.com/patjak/facetimehd)
driver. GPL-2.0, like the original.

Tested on a MacBookPro12,1 (early 2015, 13") running FreeBSD 15.1-RELEASE:
720p capture at 30 fps with no dropped frames, and the camera shows up as
`/dev/video0` for V4L2 applications.

## How it works

The camera isn't a USB/UVC device like most webcams. It is a PCIe device
with its own ARM-based image signal processor (ISP). The host has to boot
that ISP with Apple's firmware before it will do anything. The pieces:

```
 V4L2 apps (browser, ffmpeg, OBS, PipeWire)
        │  /dev/video0
 fthd-v4l2d  ── userspace daemon, cuse(3)
        │  /dev/fthd0  (read() = newest YUYV frame, ioctls for format)
 fthd.ko     ── kernel driver
        │  PCIe: S2 bridge, DDR, ISP mailbox, IOMMU (DART)
 FaceTime HD ISP + sensor
```

**Kernel module (`fthd.ko`)**

- `fthd_pci.c`: PCI attach/detach, BARs, interrupts, sysctls.
- `fthd_hw.c`: S2 bridge bring-up: PLL, DDR memory controller
  init and calibration (the ISP has its own RAM).
- `fthd_isp.c`: uploads `firmware.bin` via firmware(9), boots the ISP,
  sets up the IPC channels and command interface, detects the sensor and
  uploads the matching calibration ("set") file `NNNN_01XX.dat`.
- `fthd_ringbuf.c`: shared-memory ring buffers used by the IPC channels.
- `fthd_buffer.c`: maps capture buffers into the camera's own IOMMU and
  hands them to the firmware. Frames come back tagged with a "fthd" magic plus
  buffer index, never with a kernel pointer.
- `fthd_cdev.c`: the `/dev/fthd0` character device (root:video, 0660).

**V4L2 bridge (`v4l2/fthd-v4l2d`)**

FreeBSD has no in-kernel V4L2. The usual approach, as with webcamd, is
a userspace daemon that creates `/dev/videoN` through cuse(3) and answers the
V4L2 ioctls. `fthd-v4l2d` does that for the camera. It supports YUYV at a
list of discrete sizes (1280x720 down to 320x240) at 5–30 fps, with
`read()` and mmap streaming.

Known limitation: cuse can't return `ENOTTY`, so unsupported ioctls fail
with `EINVAL`. `v4l2-compliance` flags this, but real applications don't
care.

## Requirements

- FreeBSD 15.x, amd64, with kernel sources in `/usr/src`
- Packages: `v4l_compat` (V4L2 headers), `v4l-utils` (optional, for testing)
- `cuse.ko` (in base)

## Firmware

The firmware belongs to Apple and is **not** included here. `get-firmware.sh`
downloads only the needed byte ranges (~25 MB) of Apple's macOS 10.12.6
combo update. It then extracts them with
[facetimehd-firmware](https://github.com/patjak/facetimehd-firmware)'s
script, which checks SHA-256 hashes:

    ./get-firmware.sh
    sudo install -d /boot/firmware/facetimehd
    sudo install -m 444 fw/firmware.bin fw/*.dat /boot/firmware/facetimehd/

## Build and install

    make
    (cd v4l2 && make)
    (cd tools && make)

    sudo install -m 555 fthd.ko /boot/modules/
    sudo kldxref /boot/modules
    sudo install -m 555 v4l2/fthd-v4l2d /usr/local/sbin/
    sudo install -m 555 rc.d/fthd_v4l2d /usr/local/etc/rc.d/

## Try it

    sudo kldload fthd cuse
    dmesg | grep fthd          # ends with "fthd0: Camera is up"
    sudo fthd-v4l2d -f         # foreground; Ctrl-C to stop
    v4l2-ctl -d /dev/video0 --list-formats-ext

Options for `fthd-v4l2d`: `-f` foreground, `-d` camera device (default
`/dev/fthd0`), `-u` video unit number, `-g` group for `/dev/videoN` (default
`video`). Add your user to the `video` group: `sudo pw groupmod video -m $USER`.

## Load at boot

In `/etc/rc.conf`:

    kld_list="fthd cuse"
    fthd_v4l2d_enable="YES"
    #fthd_v4l2d_args="-g video"

Note: the driver registers its module as `pci/fthd`, not `fthd`. That's
why the rc.d script uses `required_modules="fthd:pci/fthd cuse"`. With a
plain `fthd`, rc.subr fails to see the loaded module, tries to load it again,
gets an error and refuses to start the daemon.

## Debugging and testing

- `tools/fthd-grab -n 60 -o /tmp/frame.ppm` captures straight from
  `/dev/fthd0` (no V4L2 involved) and saves the last frame as PPM.
- `sysctl dev.fthd.0` shows sensor IDs, resolution, and the `frames` /
  `drops` counters.
- `hw.fthd.debug=1` (sysctl or loader tunable) prints verbose output,
  including the firmware's own log messages.
- Unload with `sudo service fthd_v4l2d stop && sudo kldunload fthd`.

## Status

- [x] S2 PLL / DDR bring-up, ISP firmware upload and boot
- [x] IPC channels, command interface, sensor detection, set file upload
- [x] Frame capture through the camera IOMMU, `/dev/fthd0`
- [x] V4L2 bridge (`/dev/video0` via cuse), rc.d script, load at boot
- [ ] Testing with more applications (browsers, ffmpeg, OBS)
- [ ] Camera controls (brightness, contrast, …), suspend/resume

## Credits

Based on the Linux facetimehd driver by Patrik Jakobsson, Sven Schnelle and
contributors, which is in turn based on reverse engineering of Apple's
driver. Firmware extraction by the
[facetimehd-firmware](https://github.com/patjak/facetimehd-firmware) project.
