/*
 * camkit_sensor_i2c.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: image sensor i2c interface
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

#ifndef CAMKIT_SENSOR_I2C_H
#define CAMKIT_SENSOR_I2C_H

#include "kd_camkit_define.h"

int32 camkit_sensor_i2c_read(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 *data,
	enum camkit_i2c_data_type data_type);

int32 camkit_sensor_i2c_write(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	enum camkit_i2c_data_type data_type);

int32 camkit_sensor_i2c_poll(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	enum camkit_i2c_data_type data_type, uint16 delay);

int32 camkit_sensor_write_table(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting, uint32 size,
	enum camkit_i2c_data_type data_type);

int32 camkit_sensor_write_setting(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_setting *settings);

int32 camkit_sensor_i2c_process(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_table_array *settings);

int32 camkit_i2c_write_table(struct camkit_sensor_ctrl_t *sensor,
	uint16 *setting, uint32 size, enum camkit_i2c_data_type data_type);

int32 camkit_i2c_read(uint8 i2c_addr, uint32 addr,
	enum camkit_i2c_addr_type addr_type, uint16 *data,
	enum camkit_i2c_data_type data_type);

int32 camkit_read_eeprom(uint8 i2c_addr, uint16 addr, uint8 *data);

// Note: Register address must be continuous
int32 camkit_i2c_write_block(uint8 i2c_addr, uint32 i2c_speed,
	uint32 start_addr, enum camkit_i2c_addr_type addr_type,
	uint8 *data, uint32 data_len);

int32 camkit_sensor_write_block(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting, uint32 size,
	enum camkit_i2c_data_type data_type);

int32 camkit_i2c_write_setting(uint32 i2c_speed, uint8 i2c_write_id,
	struct camkit_i2c_reg_setting *settings);

#endif // CAMKIT_SENSOR_I2C_H
