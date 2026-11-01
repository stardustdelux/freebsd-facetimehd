# FacetimeHD (Broadcom 1570) camera driver for FreeBSD

KMOD=	fthd
SRCS=	fthd_pci.c fthd_hw.c fthd_isp.c fthd_ringbuf.c fthd_buffer.c fthd_cdev.c
SRCS+=	device_if.h bus_if.h pci_if.h acpi_if.h opt_acpi.h

SYSDIR?=	/usr/src/sys

.include <bsd.kmod.mk>
