/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __IMGSENSOR_SENSOR_H__
#define __IMGSENSOR_SENSOR_H__

#include "kd_camera_feature.h"
#include "kd_imgsensor_define.h"
#include "imgsensor_i2c.h"
#include "camkit_driver_types.h"

enum IMGSENSOR_STATE {
	IMGSENSOR_STATE_CLOSE,
	IMGSENSOR_STATE_OPEN
};

struct IMGSENSOR_SENSOR_STATUS {
	u32 reserved:28;
	u32 arch:4;
};

struct IMGSENSOR_SENSOR_INST {
	enum IMGSENSOR_STATE state;
	enum IMGSENSOR_SENSOR_IDX sensor_idx;
	struct IMGSENSOR_I2C_CFG i2c_cfg;
	struct IMGSENSOR_SENSOR_STATUS status;
	struct IMGSENSOR_SENSOR_LIST *psensor_list;
	struct mutex sensor_mutex;
	struct timeval profile_time;
	struct delayed_work dump_frame_count_worker;
	int dump_frame_count;
	bool is_work_init;
};

struct IMGSENSOR_SENSOR {
	struct IMGSENSOR_SENSOR_INST  inst;
	struct SENSOR_FUNCTION_STRUCT *pfunc;

	/* Customized sensor kit parameters */
	struct sensor_kit_ops *sensor_ops;
	struct camkit_params *kit_params;
};

extern const char *g_product_name;
#endif

