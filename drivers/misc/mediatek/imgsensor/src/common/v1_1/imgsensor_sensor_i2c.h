/*
 * imgsensor_sensor_i2c.h
 *
 * Copyright (c) 2018-2019 Huawei Technologies Co., Ltd.
 *
 * image sensor i2c interface
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

#ifndef _IMGSENSOR_SENSOR_I2C_H
#define _IMGSENSOR_SENSOR_I2C_H

#include "imgsensor_sensor_common.h"

kal_int32 imgsensor_sensor_i2c_read(struct imgsensor_t *sensor,
	kal_uint32 addr, kal_uint16 *data,
	enum imgsensor_i2c_data_type data_type);

kal_int32 imgsensor_sensor_i2c_write(struct imgsensor_t *sensor,
	kal_uint32 addr, kal_uint16 data,
	enum imgsensor_i2c_data_type data_type);

kal_int32 imgsensor_sensor_i2c_poll(struct imgsensor_t *sensor,
	kal_uint32 addr, kal_uint16 data,
	enum imgsensor_i2c_data_type data_type, kal_uint16 delay);

kal_int32 imgsensor_sensor_write_table(struct imgsensor_t *sensor,
	struct imgsensor_i2c_reg *setting, kal_uint32 size,
	enum imgsensor_i2c_data_type data_type);

kal_int32 imgsensor_sensor_write_setting(struct imgsensor_t *sensor,
	struct imgsensor_i2c_reg_setting *settings);

kal_int32 imgsensor_sensor_i2c_process(struct imgsensor_t *sensor,
	struct imgsensor_i2c_reg_table_array *settings);

#endif
