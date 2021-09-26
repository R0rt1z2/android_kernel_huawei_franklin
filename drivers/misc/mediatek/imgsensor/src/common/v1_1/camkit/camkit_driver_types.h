/*
 * camkit_driver_types.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * define common structures and interface for image sensor driver
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

#ifndef CAMKIT_DRIVER_TYPES_H
#define CAMKIT_DRIVER_TYPES_H

#include "kd_imgsensor.h"
#include "imgsensor_cfg_table.h"
#include "kd_camkit_define.h"

#define MAX_I2C_REG_NUM 64

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct aec_ctrl_cfg {
	uint32 again;      // record the register value of again
	bool again_valid;
	uint32 dgain;      // record the register value of dgain
	bool dgain_valid;
	uint32 line_count;
	bool lc_valid;
	uint32 frame_length;
	bool fl_valid;
	uint32 group_hold;
	bool group_hold_valid;
	uint32 shift;
	bool shift_valid;
	uint32 long_line_count;
	bool llc_valid;
	uint32 awb_gain_gr;
	uint32 awb_gain_r;
	uint32 awb_gain_b;
	uint32 awb_gain_gb;
};

struct sensor_i2c_cfg {
	uint16 addr;
	uint16 val;
	uint16 addr_type;
	uint16 data_type;
	enum camkit_i2c_mode i2c_mode;
	uint16 delay;
};

struct sensor_setting_cfg {
	struct sensor_i2c_cfg i2c_setting[MAX_I2C_REG_NUM];
	uint16 size;
	uint16 delay;
};

struct sensor_kit_ops {
	uint32 (*sensor_open)(struct camkit_params *params);
	uint32 (*sensor_get_info)(struct camkit_params *params,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
	    MSDK_SENSOR_INFO_STRUCT *sensor_info,
	    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);

	uint32 (*sensor_get_resolution)(struct camkit_params *params,
	    MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution);

	uint32 (*sensor_feature_control)(struct camkit_params *params,
		MSDK_SENSOR_FEATURE_ENUM feature_id,
	    MUINT8 *feature_param,
	    uint32 *feature_param_len);

	uint32 (*sensor_control)(struct camkit_params *params,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
	    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);

	uint32 (*sensor_close)(struct camkit_params *params);
};

enum sensor_shutter_mode {
	SHUTTER_INVALID = -1,
	SHUTTER_DEFAULT,
	SHUTTER_NORMAL2LONG,
	SHUTTER_LONG2NORMAL,
};

#endif // CAMKIT_DRIVER_TYPES_H
