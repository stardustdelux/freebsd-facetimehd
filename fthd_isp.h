/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FacetimeHD (Broadcom 1570) camera driver for FreeBSD
 *
 * Firmware command interface, from the Linux facetimehd driver:
 *   Copyright (C) 2014 Patrik Jakobsson (patrik.r.jakobsson@gmail.com)
 */

#ifndef _FTHD_ISP_H
#define _FTHD_ISP_H

/* Only the commands this driver uses so far */
enum fthd_isp_cmds {
	CISP_CMD_START = 0x0,
	CISP_CMD_STOP = 0x1,
	CISP_CMD_CONFIG_GET = 0x3,
	CISP_CMD_PRINT_ENABLE = 0x4,
	CISP_CMD_POWER_DOWN = 0xa,
	CISP_CMD_CH_START = 0x100,
	CISP_CMD_CH_STOP = 0x101,
	CISP_CMD_CH_BUFFER_RETURN = 0x104,
	CISP_CMD_CH_CAMERA_CONFIG_GET = 0x106,
	CISP_CMD_CH_CAMERA_CONFIG_SELECT = 0x107,
	CISP_CMD_CH_INFO_GET = 0x10d,
	CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET = 0x10e,
	CISP_CMD_CH_BUFFER_RECYCLE_START = 0x10f,
	CISP_CMD_CH_SET_FILE_LOAD = 0x111,
	CISP_CMD_CH_SIF_PIXEL_FORMAT_SET = 0x115,
	CISP_CMD_CH_CAMERA_ERR_HANDLE_CONFIG = 0x124,
	CISP_CMD_CH_AE_FRAME_RATE_MAX_SET = 0x208,
	CISP_CMD_CH_AE_FRAME_RATE_MIN_SET = 0x20a,
	CISP_CMD_CH_AE_SPEED_SET = 0x218,
	CISP_CMD_CH_AE_STABILITY_SET = 0x21a,
	CISP_CMD_CH_AE_STABILITY_TO_STABLE_SET = 0x229,
	CISP_CMD_CH_CROP_SET = 0x801,
	CISP_CMD_CH_OUTPUT_CONFIG_SET = 0xb01,
	CISP_CMD_CH_DRC_START = 0xc00,
	CISP_CMD_CH_FACE_DETECTION_START = 0xd00,
	CISP_CMD_CH_FACE_DETECTION_STOP = 0xd01,
	CISP_CMD_CH_FACE_DETECTION_DISABLE = 0xd04,
	CISP_CMD_CH_FACE_DETECTION_ENABLE = 0xd05,
	CISP_CMD_APPLE_CH_STREAMING_MODE_SET = 0x8106,
	CISP_CMD_APPLE_CH_AE_METERING_MODE_SET = 0x8206,
	CISP_CMD_APPLE_CH_TEMPORAL_FILTER_START = 0xc100,
	CISP_CMD_APPLE_CH_TEMPORAL_FILTER_STOP = 0xc101,
	CISP_CMD_APPLE_CH_MOTION_HISTORY_START = 0xc102,
	CISP_CMD_APPLE_CH_MOTION_HISTORY_STOP = 0xc103,
	CISP_CMD_APPLE_CH_TEMPORAL_FILTER_ENABLE = 0xc113,
	CISP_CMD_APPLE_CH_TEMPORAL_FILTER_DISABLE = 0xc114,
	CISP_CMD_APPLE_CH_TONE_CURVE_ADAPTATION_START = 0xc300,
};

/* Frame rates are in 1/256 fps */
#define FTHD_FRAME_RATE_SCALE	256

struct isp_fw_args {
	uint32_t __unknown;
	uint32_t fw_arg;
	uint32_t full_stats_mode;
} __packed;

struct isp_channel_info {
	char	 name[64];
	uint32_t type;
	uint32_t source;
	uint32_t size;
	uint32_t offset;
} __packed;

struct isp_cmd_hdr {
	uint32_t unknown0;
	uint16_t opcode;
	uint16_t status;
} __packed;

struct isp_cmd_print_enable {
	uint32_t enable;
} __packed;

struct isp_cmd_config {
	uint32_t field[8];
} __packed;

struct isp_cmd_set_loadfile {
	uint32_t unknown;
	uint32_t addr;
	uint32_t length;
} __packed;

struct isp_cmd_channel_info {
	uint32_t field_0;
	uint32_t field_4;
	uint32_t field_8;
	uint32_t field_c;
	uint16_t sensorid0;
	uint16_t field_12;
	uint32_t field_14;
	uint16_t sensorid1;
	uint16_t field_1a;
	uint32_t field_1c;
	uint32_t field_20;
	uint8_t	 unknown[52];
	uint32_t sensor_count;
	uint8_t	 unknown2[40];
	uint8_t	 sensor_serial_number[8];
	uint8_t	 camera_module_serial_number[18];
} __packed;

struct isp_cmd_channel_camera_config {
	uint32_t unknown;
	uint32_t channel;
	uint8_t	 data[88];
} __packed;

/*
 * Channel commands.  The Linux structs are not packed, so a u16 argument
 * after the channel is followed by two bytes of padding; spell it out.
 */
struct isp_cmd_channel {
	uint32_t channel;
} __packed;

struct isp_cmd_channel_u32 {
	uint32_t channel;
	uint32_t value;
} __packed;

struct isp_cmd_channel_u16 {
	uint32_t channel;
	uint16_t value;
	uint16_t pad;
} __packed;

struct isp_cmd_channel_u16x2 {		/* also u8 x4 for SIF format */
	uint32_t channel;
	uint16_t param1;
	uint16_t param2;
} __packed;

struct isp_cmd_channel_set_crop {
	uint32_t channel;
	uint32_t x;
	uint32_t y;
	uint32_t width;
	uint32_t height;
} __packed;

struct isp_cmd_channel_output_config {
	uint32_t channel;
	uint32_t x1;
	uint32_t y1;
	uint32_t unknown3;
	uint32_t pixelformat;	/* 0 NV16, 1 YUYV, 2 YVYU */
	uint32_t x2;
	uint32_t x3;
	uint32_t unknown5;
} __packed;

/* Frame buffer descriptors passed on the BUF_H2T / BUF_T2H channels */
struct dma_descriptor {
	uint32_t addr0;		/* device address of plane 0 */
	uint32_t addr1;
	uint32_t addr2;
	uint32_t field_c;
	uint32_t field_10;
	uint32_t field_14;
	uint32_t count;
	uint32_t pool;
	uint64_t tag;		/* opaque to the firmware, returned as is */
} __packed;

struct dma_descriptor_list {
	uint32_t field0;
	uint32_t count;
	struct dma_descriptor desc[4];
	uint8_t	 unknown[216];
} __packed;

_Static_assert(sizeof(struct dma_descriptor_list) == 0x180,
    "dma_descriptor_list must be 0x180 bytes");

#endif /* _FTHD_ISP_H */
