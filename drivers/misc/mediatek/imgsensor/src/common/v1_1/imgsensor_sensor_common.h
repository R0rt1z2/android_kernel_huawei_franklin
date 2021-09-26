/*
 * imgsensor_sensor_common.h
 *
 * Copyright (c) 2018-2019 Huawei Technologies Co., Ltd.
 *
 * define image senor common structures and interface
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

#ifndef _IMGSENSOR_SENSOR_COMMON_H
#define _IMGSENSOR_SENSOR_COMMON_H

#include "kd_camera_typedef.h"
#include "kd_imgsensor_define.h"
#include "kd_camkit_define.h"

#define IMGSENSOR_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define IMGSENSOR_LINGLENGTH_GAP 80


struct imgsensor_i2c_reg {
	kal_uint16 addr;
	kal_uint16 data;
	kal_uint16 delay;
};

enum imgsensor_i2c_addr_type {
	IMGSENSOR_I2C_BYTE_ADDR = 1,
	IMGSENSOR_I2C_WORD_ADDR,
	IMGSENSOR_I2C_ADDR_MAX,
};

enum imgsensor_i2c_data_type {
	IMGSENSOR_I2C_BYTE_DATA = 1,
	IMGSENSOR_I2C_WORD_DATA,
	IMGSENSOR_I2C_DATA_MAX,
};

enum imgsensor_i2c_operation {
	IMGSENSOR_I2C_WRITE,
	IMGSENSOR_I2C_READ,
	IMGSENSOR_I2C_POLL,
};

struct imgsensor_i2c_reg_table {
	kal_uint16 addr;
	kal_uint16 data;
	enum imgsensor_i2c_data_type data_type;
	enum imgsensor_i2c_operation i2c_operation;
	kal_uint16 delay;
};

struct imgsensor_i2c_reg_table_array {
	struct imgsensor_i2c_reg_table *setting;
	kal_uint16 size;
};

enum imgsensor_common_mode {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
	IMGSENSOR_MODE_CUSTOM1,
	IMGSENSOR_MODE_CUSTOM2,
	IMGSENSOR_MODE_CUSTOM3,
	IMGSENSOR_MODE_CUSTOM4,
	IMGSENSOR_MODE_CUSTOM5,
};

struct imgsensor_mode_info {
	kal_uint32 pclk;
	kal_uint32 linelength;
	kal_uint32 framelength;

	kal_uint8 startx;
	kal_uint8 starty;

	kal_uint16 grabwindow_width;
	kal_uint16 grabwindow_height;

	/*
	 * following for MIPIDataLowPwr2HighSpeedSettleDelayCount
	 * by different scenario
	 */
	kal_uint8 mipi_data_lp2hs_settle_dc;

	/* following for GetDefaultFramerateByScenario() */
	kal_uint16 max_framerate;
	kal_uint32 mipi_pixel_rate;
	kal_uint32 mipi_trail_val;
};

struct imgsensor_i2c_reg_setting {
	struct imgsensor_i2c_reg *setting;
	kal_uint16 size;
	enum imgsensor_i2c_addr_type addr_type;
	enum imgsensor_i2c_data_type data_type;
	kal_uint16 delay;
};

/* SENSOR PRIVATE STRUCT FOR CURRENT VARIABLES */
struct imgsensor_t {
	kal_uint8 mirror;
	kal_uint8 sensor_mode;
	kal_uint32 shutter;
	kal_uint16 gain;
	kal_uint32 pclk;
	kal_uint32 frame_length;
	kal_uint32 line_length;
	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel;
	kal_uint16 dummy_line;
	kal_uint16 current_fps;
	kal_bool   autoflicker_en;
	kal_bool test_pattern;
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;
	kal_bool  ihdr_en;
	kal_uint8 i2c_write_id;
	kal_uint32  i2c_speed;
	enum imgsensor_i2c_addr_type addr_type;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct imgsensor_info_t {
	/* sensor chip id reg addr */
	kal_uint16 sensor_id_reg;
	kal_uint16 gain_type;
	kal_uint16 gain_step;
	kal_uint16 min_gain;
	kal_uint16 min_gain_iso;
	kal_uint16 max_gain;
	/* record sensor id defined in Kd_imgsensor.h */
	kal_uint32 sensor_id;
	/* checksum value for Camera Auto Test */
	kal_uint32 checksum_value;
	struct imgsensor_mode_info pre;
	struct imgsensor_mode_info cap;
	/* capture for PIP relative info */
	struct imgsensor_mode_info cap1;
	struct imgsensor_mode_info normal_video;
	struct imgsensor_mode_info hs_video;
	/* slim video for VT scenario relative info */
	struct imgsensor_mode_info slim_video;
	struct imgsensor_mode_info custom1;
	struct imgsensor_mode_info custom2;
	struct imgsensor_mode_info custom3;
	struct imgsensor_mode_info custom4;
	struct imgsensor_mode_info custom5;
	struct imgsensor_i2c_reg_setting init_setting;
	struct imgsensor_i2c_reg_setting pre_setting;
	struct imgsensor_i2c_reg_setting cap_setting;
	struct imgsensor_i2c_reg_setting cap1_setting;
	struct imgsensor_i2c_reg_setting normal_video_setting;
	struct imgsensor_i2c_reg_setting hs_video_setting;
	struct imgsensor_i2c_reg_setting slim_setting;
	struct imgsensor_i2c_reg_setting custom1_setting;
	struct imgsensor_i2c_reg_setting custom2_setting;
	struct imgsensor_i2c_reg_setting custom3_setting;
	struct imgsensor_i2c_reg_setting custom4_setting;
	struct imgsensor_i2c_reg_setting custom5_setting;
	struct imgsensor_i2c_reg_setting streamon_setting;
	struct imgsensor_i2c_reg_setting streamoff_setting;
	struct imgsensor_i2c_reg_setting test_pattern_on_setting;
	struct imgsensor_i2c_reg_setting test_pattern_off_setting;
	struct imgsensor_i2c_reg_table_array dump_info;
	/*
	 * ae_shut_delay_frame: shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	kal_uint8  ae_shut_delay_frame;
	/*
	 * ae_sensor_gain_delay_frame:sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	kal_uint8  ae_sensor_gain_delay_frame;
	/* isp gain delay frame for AE cycle */
	kal_uint8  ae_ispGain_delay_frame;
	kal_uint8  ihdr_support;        /* 1, support; 0,not support */
	kal_uint8  ihdr_le_firstline;   /* 1,le first ; 0, se first */
	kal_uint8  sensor_mode_num;     /* support sensor mode num */
	kal_uint8  cap_delay_frame;
	kal_uint8  pre_delay_frame;
	kal_uint8  video_delay_frame;
	kal_uint8  hs_video_delay_frame;
	kal_uint8  slim_video_delay_frame;
	kal_uint8  custom1_delay_frame;
	kal_uint8  custom2_delay_frame;
	kal_uint8  custom3_delay_frame;
	kal_uint8  custom4_delay_frame;
	kal_uint8  custom5_delay_frame;
	/* sensor framelength & shutter margin */
	kal_uint8  margin;
	kal_uint32 min_shutter;
	/* max framelength by sensor register's limit */
	kal_uint32 max_frame_length;
	/* mclk driving current */
	kal_uint8  isp_driving_current;
	/* sensor_interface_type */
	kal_uint8  sensor_interface_type;
	/*
	 * mipi_sensor_type:
	 * 0,MIPI_OPHY_NCSI2
	 * 1,MIPI_OPHY_CSI2
	 * default is NCSI2
	 */
	kal_uint8  mipi_sensor_type;
	/*
	 * mipi_settle_delay_mode:
	 * 0, high speed signal auto detect;
	 * 1, use settle delay,unit is ns
	 * default is auto detect
	 * don't modify this para
	 */
	kal_uint8  mipi_settle_delay_mode;
	/* sensor output first pixel color */
	kal_uint8  sensor_output_dataformat;
	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	kal_uint8  mclk;
	kal_uint8  mipi_lane_num;
	/*
	 * record sensor support write id addrs,
	 * only supprt 4must end with 0xff
	 */
	kal_uint8  i2c_addr_table[5];
	kal_uint32  i2c_speed;
	enum imgsensor_i2c_addr_type addr_type;
};

MUINT32 imgsensor_convert_sensor_id(MUINT32 imgsensor_sensor_id,
					MUINT32 sensor_chip_id,
					MUINT32 vendor_id_addr,
					MUINT32 product_id);

MUINT32 get_driver_name(MUINT32 sensor_id,
	unsigned char *drv_name,
	MUINT32 len);
#endif
