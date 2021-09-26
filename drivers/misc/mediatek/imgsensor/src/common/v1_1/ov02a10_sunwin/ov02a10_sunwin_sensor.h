/*
 * ov02a10_sunwin_sensor.h
 *
 * ov02a10_sunwin image sensor config settings
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifndef _OV02A10_SUNWIN_H
#define _OV02A10_SUNWIN_H

#include "imgsensor_sensor_common.h"
#include "kd_imgsensor.h"

static struct imgsensor_i2c_reg stream_on[] = {
	{ 0xfd, 0x01, 0x00 },
	{ 0x01, 0x01, 0x00 },
	{ 0x18, 0x00, 0x00 },
	{ 0x18, 0x01, 0x00 },
	/* MIPI_EN_BUF Bit[0] MIPI enbale */
	{ 0xac, 0x01, 0x00 },
};

static struct imgsensor_i2c_reg stream_off[] = {
	{ 0xfd, 0x01, 0x00 },
	{ 0xac, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg init_setting[] = {
	{ 0xfd, 0x00, 0x00 },
	{ 0x2f, 0x29, 0x00 },
	{ 0x34, 0x00, 0x00 },
	{ 0x35, 0x21, 0x00 },
	{ 0x30, 0x15, 0x00 },
	{ 0x33, 0x01, 0x00 },
	{ 0xfd, 0x01, 0x00 },
	{ 0x44, 0x00, 0x00 },
	{ 0x2a, 0x4c, 0x00 },
	{ 0x2b, 0x1e, 0x00 },
	{ 0x2c, 0x60, 0x00 },
	{ 0x25, 0x11, 0x00 },
	{ 0x03, 0x01, 0x00 },
	{ 0x04, 0xae, 0x00 },
	{ 0x09, 0x00, 0x00 },
	{ 0x0a, 0x02, 0x00 },
	{ 0x06, 0x9a, 0x00 },
	{ 0x31, 0x00, 0x00 },
	{ 0x24, 0x40, 0x00 },
	{ 0x01, 0x01, 0x00 },
	{ 0xfb, 0x73, 0x00 },
	{ 0xfd, 0x01, 0x00 },
	{ 0x16, 0x04, 0x00 },
	{ 0x1c, 0x09, 0x00 },
	{ 0x21, 0x42, 0x00 },
	{ 0x12, 0x04, 0x00 },
	{ 0x13, 0x10, 0x00 },
	{ 0x11, 0x40, 0x00 },
	{ 0x33, 0x81, 0x00 },
	{ 0xd0, 0x00, 0x00 },
	{ 0xd1, 0x01, 0x00 },
	{ 0xd2, 0x00, 0x00 },
	{ 0x50, 0x10, 0x00 },
	{ 0x51, 0x23, 0x00 },
	{ 0x52, 0x20, 0x00 },
	{ 0x53, 0x10, 0x00 },
	{ 0x54, 0x02, 0x00 },
	{ 0x55, 0x20, 0x00 },
	{ 0x56, 0x02, 0x00 },
	{ 0x58, 0x48, 0x00 },
	{ 0x5d, 0x15, 0x00 },
	{ 0x5e, 0x05, 0x00 },
	{ 0x66, 0x66, 0x00 },
	{ 0x68, 0x68, 0x00 },
	{ 0x6b, 0x00, 0x00 },
	{ 0x6c, 0x00, 0x00 },
	{ 0x6f, 0x40, 0x00 },
	{ 0x70, 0x40, 0x00 },
	{ 0x71, 0x0a, 0x00 },
	{ 0x72, 0xf0, 0x00 },
	{ 0x73, 0x10, 0x00 },
	{ 0x75, 0x80, 0x00 },
	{ 0x76, 0x10, 0x00 },
	{ 0x84, 0x00, 0x00 },
	{ 0x85, 0x10, 0x00 },
	{ 0x86, 0x10, 0x00 },
	{ 0x87, 0x00, 0x00 },
	{ 0x8a, 0x22, 0x00 },
	{ 0x8b, 0x22, 0x00 },
	{ 0x19, 0xf1, 0x00 },
	{ 0x29, 0x01, 0x00 },
	{ 0xfd, 0x01, 0x00 },
	{ 0x9d, 0x16, 0x00 },
	{ 0xa0, 0x29, 0x00 },
	{ 0xa1, 0x04, 0x00 },
	{ 0xad, 0x62, 0x00 },
	{ 0xae, 0x00, 0x00 },
	{ 0xaf, 0x85, 0x00 },
	{ 0xb1, 0x01, 0x00 },
	{ 0x8e, 0x06, 0x00 },
	{ 0x8f, 0x40, 0x00 },
	{ 0x90, 0x04, 0x00 },
	{ 0x91, 0xb0, 0x00 },
	{ 0x45, 0x01, 0x00 },
	{ 0x46, 0x00, 0x00 },
	{ 0x47, 0x6c, 0x00 },
	{ 0x48, 0x03, 0x00 },
	{ 0x49, 0x8b, 0x00 },
	{ 0x4a, 0x00, 0x00 },
	{ 0x4b, 0x07, 0x00 },
	{ 0x4c, 0x04, 0x00 },
	{ 0x4d, 0xb7, 0x00 },
	{ 0xf0, 0x40, 0x00 },
	{ 0xf1, 0x40, 0x00 },
	{ 0xf2, 0x40, 0x00 },
	{ 0xf3, 0x40, 0x00 },
	{ 0xac, 0x01, 0x00 },
	{ 0xfd, 0x01, 0x00 },
};

static struct imgsensor_i2c_reg preview_setting[] = {
	{ 0xfd, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg capture_setting[] = {
	{ 0xfd, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg video_setting[] = {
	{ 0xfd, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg_table dump_setting[] = {
	{ 0x03, 0x00, IMGSENSOR_I2C_BYTE_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x04, 0x00, IMGSENSOR_I2C_BYTE_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x05, 0x00, IMGSENSOR_I2C_BYTE_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x06, 0x00, IMGSENSOR_I2C_BYTE_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0xac, 0x00, IMGSENSOR_I2C_BYTE_DATA, IMGSENSOR_I2C_READ, 0 },
};
static struct imgsensor_info_t imgsensor_info = {
	.sensor_id_reg = 0x02,
	.sensor_id = OV02A10_SUNWIN_SENSOR_ID,
	.checksum_value = 0x38ebe79e,
	.pre = {
		.pclk = 39000000,
		.linelength = 934,
		.framelength = 1378,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 303,
		.mipi_pixel_rate = 78000000,
	},
	.cap = {
		.pclk = 39000000,
		.linelength = 934,
		.framelength = 1378,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 303,
		.mipi_pixel_rate = 78000000,
	},
	.normal_video = {
		.pclk = 39000000,
		.linelength = 934,
		.framelength = 1378,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 303,
		.mipi_pixel_rate = 78000000,
	},
	.init_setting = {
		.setting = init_setting,
		.size = IMGSENSOR_ARRAY_SIZE(init_setting),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.pre_setting = {
		.setting = preview_setting,
		.size = IMGSENSOR_ARRAY_SIZE(preview_setting),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.cap_setting = {
		.setting = capture_setting,
		.size = IMGSENSOR_ARRAY_SIZE(capture_setting),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.normal_video_setting = {
		.setting = video_setting,
		.size = IMGSENSOR_ARRAY_SIZE(video_setting),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.streamon_setting = {
		.setting = stream_on,
		.size = IMGSENSOR_ARRAY_SIZE(stream_on),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 5,
	},
	.streamoff_setting = {
		.setting = stream_off,
		.size = IMGSENSOR_ARRAY_SIZE(stream_off),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.dump_info = {
		.setting = dump_setting,
		.size = IMGSENSOR_ARRAY_SIZE(dump_setting),
	},

	.margin = 24,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,
	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.i2c_addr_table = { 0x7a, 0xff },
	.i2c_speed = 400,
	.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
};

static struct imgsensor_t imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x14d,
	.gain = 0x80,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 303,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = KAL_FALSE,
	.i2c_write_id = 0x7a,
	.i2c_speed = 400,
	.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	/* preview */
	{ 1600, 1200, 0, 0, 1600, 1200, 1600, 1200,
		0, 0, 1600, 1200, 0, 0, 1600, 1200 },
	/* capture */
	{ 1600, 1200, 0, 0, 1600, 1200, 1600, 1200,
		0, 0, 1600, 1200, 0, 0, 1600, 1200 },
	/* video */
	{ 1600, 1200, 0, 0, 1600, 1200, 1600, 1200,
		0, 0, 1600, 1200, 0, 0, 1600, 1200 },
};
#endif
