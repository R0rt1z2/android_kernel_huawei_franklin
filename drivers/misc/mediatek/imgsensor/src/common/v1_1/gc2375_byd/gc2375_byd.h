/*
 * gc2375_byd_sensor.h
 *
 * gc2375_byd image sensor config settings
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
 */
#ifndef _GC2375_BYD_H
#define _GC2375_BYD_H
#include "imgsensor_sensor_common.h"
#include "kd_imgsensor.h"

static struct imgsensor_i2c_reg stream_on[] = {
	{ 0xfe, 0x00, 0x00 },
	{ 0xef, 0x90, 0x00 },
};

static struct imgsensor_i2c_reg stream_off[] = {
	{ 0xfe, 0x00, 0x00 },
	{ 0xef, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg init_setting[] = {
	{ 0xfe, 0x00, 0x00 },
	{ 0xfe, 0x00, 0x00 },
	{ 0xfe, 0x00, 0x00 },
	{ 0xf7, 0x01, 0x00 },
	{ 0xf8, 0x0e, 0x00 },
	{ 0xf9, 0x42, 0x00 },
	{ 0xfa, 0x88, 0x00 },
	{ 0xfc, 0x8e, 0x00 },
	{ 0xfe, 0x00, 0x00 },
	{ 0x88, 0x03, 0x00 },
	{ 0x03, 0x04, 0x00 },
	{ 0x04, 0x65, 0x00 },
	{ 0x05, 0x02, 0x00 },
	{ 0x06, 0x64, 0x00 },
	{ 0x07, 0x00, 0x00 },
	{ 0x08, 0xbe, 0x00 },
	{ 0x09, 0x00, 0x00 },
	{ 0x0a, 0x04, 0x00 },
	{ 0x0b, 0x00, 0x00 },
	{ 0x0c, 0x14, 0x00 },
	{ 0x0d, 0x04, 0x00 },
	{ 0x0e, 0xb8, 0x00 },
	{ 0x0f, 0x06, 0x00 },
	{ 0x10, 0x48, 0x00 },
	{ 0x17, 0xd4, 0x00 },
	{ 0x1c, 0x10, 0x00 },
	{ 0x1d, 0x13, 0x00 },
	{ 0x20, 0x0b, 0x00 },
	{ 0x21, 0x6d, 0x00 },
	{ 0x22, 0x0c, 0x00 },
	{ 0x25, 0xc1, 0x00 },
	{ 0x26, 0x0e, 0x00 },
	{ 0x27, 0x22, 0x00 },
	{ 0x29, 0x5f, 0x00 },
	{ 0x2b, 0x88, 0x00 },
	{ 0x2f, 0x12, 0x00 },
	{ 0x38, 0x86, 0x00 },
	{ 0x3d, 0x00, 0x00 },
	{ 0xcd, 0xa3, 0x00 },
	{ 0xce, 0x57, 0x00 },
	{ 0xd0, 0x09, 0x00 },
	{ 0xd1, 0xca, 0x00 },
	{ 0xd2, 0x34, 0x00 },
	{ 0xd3, 0xbb, 0x00 },
	{ 0xd8, 0x60, 0x00 },
	{ 0xe0, 0x08, 0x00 },
	{ 0xe1, 0x1f, 0x00 },
	{ 0xe4, 0xf8, 0x00 },
	{ 0xe5, 0x0c, 0x00 },
	{ 0xe6, 0x10, 0x00 },
	{ 0xe7, 0xcc, 0x00 },
	{ 0xe8, 0x02, 0x00 },
	{ 0xe9, 0x01, 0x00 },
	{ 0xea, 0x02, 0x00 },
	{ 0xeb, 0x01, 0x00 },
	{ 0x90, 0x01, 0x00 },
	{ 0x92, 0x04, 0x00 },
	{ 0x94, 0x04, 0x00 },
	{ 0x95, 0x04, 0x00 },
	{ 0x96, 0xb0, 0x00 },
	{ 0x97, 0x06, 0x00 },
	{ 0x98, 0x40, 0x00 },
	{ 0x18, 0x02, 0x00 },
	{ 0x1a, 0x18, 0x00 },
	{ 0x28, 0x00, 0x00 },
	{ 0x3f, 0x40, 0x00 },
	{ 0x40, 0x26, 0x00 },
	{ 0x41, 0x00, 0x00 },
	{ 0x43, 0x03, 0x00 },
	{ 0x4a, 0x00, 0x00 },
	{ 0x4e, 0x00, 0x00 },
	{ 0x4f, 0x3c, 0x00 },
	{ 0x66, 0x00, 0x00 },
	{ 0x67, 0x03, 0x00 },
	{ 0x68, 0x00, 0x00 },
	{ 0xb0, 0x58, 0x00 },
	{ 0xb1, 0x01, 0x00 },
	{ 0xb2, 0x00, 0x00 },
	{ 0xb6, 0x00, 0x00 },
	{ 0xef, 0x00, 0x00 },
	{ 0xfe, 0x03, 0x00 },
	{ 0x01, 0x03, 0x00 },
	{ 0x02, 0x33, 0x00 },
	{ 0x03, 0x90, 0x00 },
	{ 0x04, 0x04, 0x00 },
	{ 0x05, 0x00, 0x00 },
	{ 0x06, 0x80, 0x00 },
	{ 0x11, 0x2b, 0x00 },
	{ 0x12, 0xd0, 0x00 },
	{ 0x13, 0x07, 0x00 },
	{ 0x15, 0x00, 0x00 },
	{ 0x21, 0x0a, 0x00 },
	{ 0x22, 0x06, 0x00 },
	{ 0x23, 0x14, 0x00 },
	{ 0x24, 0x03, 0x00 },
	{ 0x25, 0x16, 0x00 },
	{ 0x26, 0x07, 0x00 },
	{ 0x29, 0x07, 0x00 },
	{ 0x2a, 0x09, 0x00 },
	{ 0x2b, 0x08, 0x00 },
	{ 0xfe, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg preview_setting[] = {
	{ 0xfe, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg capture_setting[] = {
	{ 0xfe, 0x00, 0x00 },
};

static struct imgsensor_i2c_reg video_setting[] = {
	{ 0xfe, 0x00, 0x00 },
};

static struct imgsensor_info_t imgsensor_info = {
	.sensor_id_reg = 0xf0,
	.sensor_id = GC2375_BYD_SENSOR_ID,
	.checksum_value = 0x38ebe79e, /* checksum value for Camera Auto Test */

	.pre = {
		.pclk = 45000000,  // record different mode's pclk
		.linelength = 1050,  // record different mode's linelength
		.framelength = 1414,  // record different mode's framelength
		.startx = 0,  // record different mode's startx of grabwindow
		.starty = 0,  // record different mode's starty of grabwindow
		.grabwindow_width = 1600,  // record different mode's width of grabwindow
		.grabwindow_height = 1200,  // record different mode's height of grabwindow
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 303,
		.mipi_pixel_rate = 72000000,
	},
	.cap = {
		.pclk = 45000000,  // record different mode's pclk
		.linelength = 1050,  // record different mode's linelength
		.framelength = 1414,  // record different mode's framelength
		.startx = 0,  // record different mode's startx of grabwindow
		.starty = 0,  // record different mode's starty of grabwindow
		.grabwindow_width = 1600,  // record different mode's width of grabwindow
		.grabwindow_height = 1200,  // record different mode's height of grabwindow
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 303,
		.mipi_pixel_rate = 72000000,
	},
	.normal_video = {
		.pclk = 45000000,  // record different mode's pclk
		.linelength = 1050,  // record different mode's linelength
		.framelength = 1414,  // record different mode's framelength
		.startx = 0,  // record different mode's startx of grabwindow
		.starty = 0,  // record different mode's starty of grabwindow
		.grabwindow_width = 1600,  // record different mode's width of grabwindow
		.grabwindow_height = 1200,  // record different mode's height of grabwindow
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.max_framerate = 303,
		.mipi_pixel_rate = 72000000,
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
		.delay = 10,
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
		.delay = 10,
	},

	.streamoff_setting = {
		.setting = stream_off,
		.size = IMGSENSOR_ARRAY_SIZE(stream_off),
		.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 10,
	},

	.margin = 4, /* sensor framelength & shutter margin */
	.min_shutter = 1, /* 1,          //min shutter */
	/* max framelength by sensor register's limitation */
	.max_frame_length = 16383,
	.ae_shut_delay_frame = 0,
	/* shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	.ae_sensor_gain_delay_frame = 0,
	/* sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	.ae_ispGain_delay_frame = 2, /* isp gain delay frame for AE cycle */
	//.frame_time_delay_frame = 2,
	.ihdr_support = 1, /* 1, support; 0,not support */
	.ihdr_le_firstline = 0, /* 1,le first ; 0, se first */
	.sensor_mode_num = 3, /* support sensor mode num */
	.cap_delay_frame = 2, /* enter capture delay frame num */
	.pre_delay_frame = 2, /* enter preview delay frame num */
	.video_delay_frame = 2, /* enter video delay frame num */
	.hs_video_delay_frame = 2, /* enter high speed video  delay frame num */
	.slim_video_delay_frame = 2, /* enter slim video delay frame num */
	.isp_driving_current = ISP_DRIVING_4MA, /* mclk driving current */
	/* sensor_interface_type */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	/* sensor output first pixel color */
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_1_LANE, /* mipi lane num */
	.i2c_addr_table = { 0x2e, 0xff },
	.i2c_speed = 400,  // i2c read/write speed
	.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
};

static struct imgsensor_t imgsensor = {
	.mirror = IMAGE_NORMAL, /* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x14d, /* current shutter */
	.gain = 0xe000, /* current gain */
	.dummy_pixel = 0, /* current dummypixel */
	.dummy_line = 0, /* current dummyline */
	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.current_fps = 303,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	/* current scenario id */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = KAL_FALSE,  // sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x2e, /* record current sensor's i2c write id */
	.i2c_speed = 400,  // i2c read/write speed
	.addr_type = IMGSENSOR_I2C_BYTE_ADDR,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ /* preview */
		.full_w = 1600,
		.full_h = 1200,
		.x0_offset = 0,
		.y0_offset = 0,
		.w0_size = 1600,
		.h0_size = 1200,
		.scale_w = 1600,
		.scale_h = 1200,
		.x1_offset = 0,
		.y1_offset = 0,
		.w1_size = 1600,
		.h1_size = 1200,
		.x2_tg_offset = 0,
		.y2_tg_offset = 0,
		.w2_tg_size = 1600,
		.h2_tg_size = 1200,
	}, { /* capture */
		.full_w = 1600,
		.full_h = 1200,
		.x0_offset = 0,
		.y0_offset = 0,
		.w0_size = 1600,
		.h0_size = 1200,
		.scale_w = 1600,
		.scale_h = 1200,
		.x1_offset = 0,
		.y1_offset = 0,
		.w1_size = 1600,
		.h1_size = 1200,
		.x2_tg_offset = 0,
		.y2_tg_offset = 0,
		.w2_tg_size = 1600,
		.h2_tg_size = 1200,
	}, { /* video */
		.full_w = 1600,
		.full_h = 1200,
		.x0_offset = 0,
		.y0_offset = 0,
		.w0_size = 1600,
		.h0_size = 1200,
		.scale_w = 1600,
		.scale_h = 1200,
		.x1_offset = 0,
		.y1_offset = 0,
		.w1_size = 1600,
		.h1_size = 1200,
		.x2_tg_offset = 0,
		.y2_tg_offset = 0,
		.w2_tg_size = 1600,
		.h2_tg_size = 1200,
	},
};
#endif
