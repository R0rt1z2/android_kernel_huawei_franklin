/*
 * ov8856_foxconn_sensor.h
 *
 * Copyright (c) 2018-2019 Huawei Technologies Co., Ltd.
 *
 * ov8856_foxconn image sensor config settings
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

#ifndef _OV8856_FOXCONN_SENSOR_H
#define _OV8856_FOXCONN_SENSOR_H

#include "imgsensor_sensor_common.h"
#include "kd_imgsensor.h"

#define REG_GAIN_1X 0x80
#define REG_MAX_AGAIN 0x7c0

#define SENSOR_HTS_REG_H 0x380c
#define SENSOR_HTS_REG_L 0x380d
#define SENSOR_VTS_REG_H 0x3540
#define SENSOR_VTS_REG_M 0x380e
#define SENSOR_VTS_REG_L 0x380f
#define SENSOR_EXPO_REG_H 0x3500
#define SENSOR_EXPO_REG_M 0x3501
#define SENSOR_EXPO_REG_L 0x3502
#define SENSOR_AGAIN_REG_H 0x3508
#define SENSOR_AGAIN_REG_L 0x3509

static struct imgsensor_i2c_reg stream_on[] = {
	{ 0x0100, 0x01, 0x00 },
};

static struct imgsensor_i2c_reg stream_off[] = {
	{ 0x0100, 0x00, 0x00 },
};

// OV8855_Initial_MirrorOnFlipOn_BGGR_26FPS_2LANE_1272Mbps_08152016
static struct imgsensor_i2c_reg init_setting[] = {
	{ 0x0103, 0x01, 0x00 },
	{ 0x303f, 0x01, 0x00 },
	{ 0x3012, 0x6c, 0x00 },
	{ 0x0100, 0x00, 0x00 },
	{ 0x0302, 0x35, 0x00 },
	{ 0x0303, 0x00, 0x00 },
	{ 0x031e, 0x0c, 0x00 },
	{ 0x3000, 0x00, 0x00 },
	{ 0x300e, 0x00, 0x00 },
	{ 0x3010, 0x00, 0x00 },
	{ 0x3015, 0x84, 0x00 },
	{ 0x3018, 0x32, 0x00 },
	{ 0x3021, 0x23, 0x00 },
	{ 0x3032, 0x80, 0x00 },
	{ 0x3033, 0x24, 0x00 },
	{ 0x3500, 0x00, 0x00 },
	{ 0x3501, 0x9a, 0x00 },
	{ 0x3502, 0x20, 0x00 },
	{ 0x3503, 0x08, 0x00 },
	{ 0x3505, 0x83, 0x00 },
	{ 0x3508, 0x01, 0x00 },
	{ 0x3509, 0x80, 0x00 },
	{ 0x350c, 0x00, 0x00 },
	{ 0x350d, 0x80, 0x00 },
	{ 0x350e, 0x04, 0x00 },
	{ 0x350f, 0x00, 0x00 },
	{ 0x3510, 0x00, 0x00 },
	{ 0x3511, 0x02, 0x00 },
	{ 0x3512, 0x00, 0x00 },
	{ 0x3600, 0x72, 0x00 },
	{ 0x3601, 0x40, 0x00 },
	{ 0x3602, 0x30, 0x00 },
	{ 0x3610, 0xc5, 0x00 },
	{ 0x3611, 0x58, 0x00 },
	{ 0x3612, 0x5c, 0x00 },
	{ 0x3613, 0xca, 0x00 },
	{ 0x3614, 0x60, 0x00 },
	{ 0x3628, 0xff, 0x00 },
	{ 0x3629, 0xff, 0x00 },
	{ 0x362a, 0xff, 0x00 },
	{ 0x3633, 0x10, 0x00 },
	{ 0x3634, 0x10, 0x00 },
	{ 0x3635, 0x10, 0x00 },
	{ 0x3636, 0x10, 0x00 },
	{ 0x364a, 0x23, 0x00 },
	{ 0x3663, 0x08, 0x00 },
	{ 0x3669, 0x34, 0x00 },
	{ 0x366e, 0x10, 0x00 },
	{ 0x3706, 0x86, 0x00 },
	{ 0x370b, 0x7e, 0x00 },
	{ 0x3714, 0x23, 0x00 },
	{ 0x3730, 0x12, 0x00 },
	{ 0x3733, 0x10, 0x00 },
	{ 0x3764, 0x00, 0x00 },
	{ 0x3765, 0x00, 0x00 },
	{ 0x3769, 0x62, 0x00 },
	{ 0x376a, 0x2a, 0x00 },
	{ 0x376b, 0x36, 0x00 },
	{ 0x3780, 0x00, 0x00 },
	{ 0x3781, 0x24, 0x00 },
	{ 0x3782, 0x00, 0x00 },
	{ 0x3783, 0x23, 0x00 },
	{ 0x3798, 0x2f, 0x00 },
	{ 0x37a1, 0x60, 0x00 },
	{ 0x37a8, 0x6a, 0x00 },
	{ 0x37ab, 0x3f, 0x00 },
	{ 0x37c2, 0x04, 0x00 },
	{ 0x37c3, 0xf1, 0x00 },
	{ 0x37c9, 0x80, 0x00 },
	{ 0x37cb, 0x16, 0x00 },
	{ 0x37cc, 0x16, 0x00 },
	{ 0x37cd, 0x16, 0x00 },
	{ 0x37ce, 0x16, 0x00 },
	{ 0x3800, 0x00, 0x00 },
	{ 0x3801, 0x00, 0x00 },
	{ 0x3802, 0x00, 0x00 },
	{ 0x3803, 0x0c, 0x00 },
	{ 0x3804, 0x0c, 0x00 },
	{ 0x3805, 0xdf, 0x00 },
	{ 0x3806, 0x09, 0x00 },
	{ 0x3807, 0xa3, 0x00 },
	{ 0x3808, 0x0c, 0x00 },
	{ 0x3809, 0xc0, 0x00 },
	{ 0x380a, 0x09, 0x00 },
	{ 0x380b, 0x90, 0x00 },
	{ 0x380c, 0x07, 0x00 },
	{ 0x380d, 0x8c, 0x00 },
	{ 0x380e, 0x0b, 0x00 },
	{ 0x380f, 0x32, 0x00 },
	{ 0x3810, 0x00, 0x00 },
	{ 0x3811, 0x10, 0x00 },
	{ 0x3812, 0x00, 0x00 },
	{ 0x3813, 0x04, 0x00 },
	{ 0x3814, 0x01, 0x00 },
	{ 0x3815, 0x01, 0x00 },
	{ 0x3816, 0x00, 0x00 },
	{ 0x3817, 0x00, 0x00 },
	{ 0x3818, 0x00, 0x00 },
	{ 0x3819, 0x00, 0x00 },
	{ 0x3820, 0xc6, 0x00 },
	{ 0x3821, 0x00, 0x00 },
	{ 0x382a, 0x01, 0x00 },
	{ 0x382b, 0x01, 0x00 },
	{ 0x3830, 0x06, 0x00 },
	{ 0x3836, 0x02, 0x00 },
	{ 0x3862, 0x04, 0x00 },
	{ 0x3863, 0x08, 0x00 },
	{ 0x3cc0, 0x33, 0x00 },
	{ 0x3d85, 0x17, 0x00 },
	{ 0x3d8c, 0x73, 0x00 },
	{ 0x3d8d, 0xde, 0x00 },
	{ 0x4001, 0xe0, 0x00 },
	{ 0x4003, 0x40, 0x00 },
	{ 0x4008, 0x00, 0x00 },
	{ 0x4009, 0x0b, 0x00 },
	{ 0x400a, 0x00, 0x00 },
	{ 0x400b, 0x84, 0x00 },
	{ 0x400f, 0x80, 0x00 },
	{ 0x4010, 0xf0, 0x00 },
	{ 0x4011, 0xff, 0x00 },
	{ 0x4012, 0x02, 0x00 },
	{ 0x4013, 0x01, 0x00 },
	{ 0x4014, 0x01, 0x00 },
	{ 0x4015, 0x01, 0x00 },
	{ 0x4042, 0x00, 0x00 },
	{ 0x4043, 0x80, 0x00 },
	{ 0x4044, 0x00, 0x00 },
	{ 0x4045, 0x80, 0x00 },
	{ 0x4046, 0x00, 0x00 },
	{ 0x4047, 0x80, 0x00 },
	{ 0x4048, 0x00, 0x00 },
	{ 0x4049, 0x80, 0x00 },
	{ 0x4041, 0x03, 0x00 },
	{ 0x404c, 0x20, 0x00 },
	{ 0x404d, 0x00, 0x00 },
	{ 0x404e, 0x20, 0x00 },
	{ 0x4203, 0x80, 0x00 },
	{ 0x4307, 0x30, 0x00 },
	{ 0x4317, 0x00, 0x00 },
	{ 0x4503, 0x08, 0x00 },
	{ 0x4601, 0x80, 0x00 },
	{ 0x4800, 0x44, 0x00 },
	{ 0x4816, 0x53, 0x00 },
	{ 0x481b, 0x58, 0x00 },
	{ 0x481f, 0x27, 0x00 },
	{ 0x4837, 0x0c, 0x00 },
	{ 0x483c, 0x0f, 0x00 },
	{ 0x484b, 0x05, 0x00 },
	{ 0x5000, 0x77, 0x00 },
	{ 0x5001, 0x0e, 0x00 },
	{ 0x5004, 0x02, 0x00 },
	{ 0x502e, 0x00, 0x00 },
	{ 0x5030, 0x41, 0x00 },
	{ 0x5795, 0x02, 0x00 },
	{ 0x5796, 0x20, 0x00 },
	{ 0x5797, 0x20, 0x00 },
	{ 0x5798, 0xd5, 0x00 },
	{ 0x5799, 0xd5, 0x00 },
	{ 0x579a, 0x00, 0x00 },
	{ 0x579b, 0x50, 0x00 },
	{ 0x579c, 0x00, 0x00 },
	{ 0x579d, 0x2c, 0x00 },
	{ 0x579e, 0x0c, 0x00 },
	{ 0x579f, 0x40, 0x00 },
	{ 0x57a0, 0x09, 0x00 },
	{ 0x57a1, 0x40, 0x00 },
	{ 0x5780, 0x14, 0x00 },
	{ 0x5781, 0x0f, 0x00 },
	{ 0x5782, 0x44, 0x00 },
	{ 0x5783, 0x02, 0x00 },
	{ 0x5784, 0x01, 0x00 },
	{ 0x5785, 0x01, 0x00 },
	{ 0x5786, 0x00, 0x00 },
	{ 0x5787, 0x04, 0x00 },
	{ 0x5788, 0x02, 0x00 },
	{ 0x5789, 0x0f, 0x00 },
	{ 0x578a, 0xfd, 0x00 },
	{ 0x578b, 0xf5, 0x00 },
	{ 0x578c, 0xf5, 0x00 },
	{ 0x578d, 0x03, 0x00 },
	{ 0x578e, 0x08, 0x00 },
	{ 0x578f, 0x0c, 0x00 },
	{ 0x5790, 0x08, 0x00 },
	{ 0x5791, 0x04, 0x00 },
	{ 0x5792, 0x00, 0x00 },
	{ 0x5793, 0x52, 0x00 },
	{ 0x5794, 0xa3, 0x00 },
	{ 0x59f8, 0x3d, 0x00 },
	{ 0x5a08, 0x02, 0x00 },
	{ 0x5b00, 0x02, 0x00 },
	{ 0x5b01, 0x10, 0x00 },
	{ 0x5b02, 0x03, 0x00 },
	{ 0x5b03, 0xcf, 0x00 },
	{ 0x5b05, 0x6c, 0x00 },
	{ 0x5e00, 0x00, 0x00 },
};

// OV8856_3264x2448_26FPS_2lane_1272Mbps_08152016
static struct imgsensor_i2c_reg preview_setting[] = {
	{ 0x3802, 0x00, 0x00 },
	{ 0x3803, 0x0c, 0x00 },
	{ 0x3806, 0x09, 0x00 },
	{ 0x3807, 0xa3, 0x00 },
	{ 0x380a, 0x09, 0x00 },
	{ 0x380b, 0x90, 0x00 },
	{ 0x380e, 0x0b, 0x00 },
	{ 0x380f, 0x32, 0x00 },
	{ 0x579d, 0x2c, 0x00 },
	{ 0x57a0, 0x09, 0x00 },
};

// OV8856_3264x2448_26FPS_2lane_1272Mbps_08152016
static struct imgsensor_i2c_reg capture_setting[] = {
	{ 0x3802, 0x00, 0x00 },
	{ 0x3803, 0x0c, 0x00 },
	{ 0x3806, 0x09, 0x00 },
	{ 0x3807, 0xa3, 0x00 },
	{ 0x380a, 0x09, 0x00 },
	{ 0x380b, 0x90, 0x00 },
	{ 0x380e, 0x0b, 0x00 },
	{ 0x380f, 0x32, 0x00 },
	{ 0x579d, 0x2c, 0x00 },
	{ 0x57a0, 0x09, 0x00 },
};

// OV8856_3264x2448_26FPS_2lane_1272Mbps_08152016
static struct imgsensor_i2c_reg video_setting[] = {
	{ 0x3802, 0x00, 0x00 },
	{ 0x3803, 0x0c, 0x00 },
	{ 0x3806, 0x09, 0x00 },
	{ 0x3807, 0xa3, 0x00 },
	{ 0x380a, 0x09, 0x00 },
	{ 0x380b, 0x90, 0x00 },
	{ 0x380e, 0x0b, 0x00 },
	{ 0x380f, 0x32, 0x00 },
	{ 0x579d, 0x2c, 0x00 },
	{ 0x57a0, 0x09, 0x00 },
};

static struct imgsensor_i2c_reg_table dump_setting[] = {
	 /* Stream on/off */
	{ 0x0100, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	/* Line counter */
	{ 0x3870, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x3871, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x3872, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	/* output width */
	{ 0x3808, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x3809, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	/* output height */
	{ 0x380a, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x380b, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	/* HTS */
	{ 0x380c, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x380d, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	/* VTS */
	{ 0x380e, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x380f, 0x00, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
};

static struct imgsensor_info_t imgsensor_info = {
	.sensor_id_reg = 0x300B,
	.sensor_id = OV8856_FOXCONN_SENSOR_ID,
	.checksum_value = 0xdf4593fd,
	.gain_type = 0,
	.gain_step = 1,
	.min_gain = 64,
	.min_gain_iso = 100,
	.max_gain = 1024,

	.pre = {
		.pclk = 287930000,
		.linelength = 3864,
		.framelength = 2866,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 260,
		.mipi_pixel_rate = 254400000,
	},
	.cap = {
		.pclk = 287930000,
		.linelength = 3864,
		.framelength = 2866,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 254400000,
	},
	.normal_video = {
		.pclk = 287930000,
		.linelength = 3864,
		.framelength = 2866,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 260,
		.mipi_pixel_rate = 254400000,
	},
	.init_setting = {
		.setting = init_setting,
		.size = IMGSENSOR_ARRAY_SIZE(init_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 5,
	},
	.pre_setting = {
		.setting = preview_setting,
		.size = IMGSENSOR_ARRAY_SIZE(preview_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.cap_setting = {
		.setting = capture_setting,
		.size = IMGSENSOR_ARRAY_SIZE(capture_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.normal_video_setting = {
		.setting = video_setting,
		.size = IMGSENSOR_ARRAY_SIZE(video_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.streamon_setting = {
		.setting = stream_on,
		.size = IMGSENSOR_ARRAY_SIZE(stream_on),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.streamoff_setting = {
		.setting = stream_off,
		.size = IMGSENSOR_ARRAY_SIZE(stream_off),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_BYTE_DATA,
		.delay = 0,
	},
	.dump_info = {
		.setting = dump_setting,
		.size = IMGSENSOR_ARRAY_SIZE(dump_setting),
	},

	.margin = 12,
	.min_shutter = 2,
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
	.hs_video_delay_frame = 0,
	.slim_video_delay_frame = 0,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = { 0x6c, 0xff },
	.addr_type = IMGSENSOR_I2C_WORD_ADDR,
};

static struct imgsensor_t imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 260,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = KAL_FALSE,
	.i2c_write_id = 0x6c,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[3] = {
	/* preview */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448 },
	/* capture */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448 },
	/* video */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448 },
};

#endif