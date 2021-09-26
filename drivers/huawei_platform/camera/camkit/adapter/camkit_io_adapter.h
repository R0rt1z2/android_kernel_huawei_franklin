/*
 * camkit_io_adapter.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: camkit io interface adapted on the platform
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

#ifndef CAMKIT_IO_ADAPTER_H
#define CAMKIT_IO_ADAPTER_H

#include "kd_camkit_define.h"

#define CAMKIT_I2C_CMD_LENGTH_MAX 255

int32 adopt_sensor_power_sequence(uint32 sensor_idx,
	enum   camkit_power_status pwr_status,
	struct camkit_hw_power_info_t *ppower_info);

int32 adopt_i2c_read(uint8 *send_data, uint16 send_data_len,
	uint8 *recv_data, uint16 recv_data_len, uint16 i2c_addr);

int32 adopt_i2c_write(uint8 *data, uint16 len, uint16 i2c_addr);

int32 adopt_i2c_set_speed(uint16 i2c_speed);

int32 adopt_i2c_burst_write(uint8 *data, uint32 len,
	uint16 i2c_addr, uint16 len_per_cycle, uint16 i2c_speed);

#endif // CAMKIT_IO_ADAPTER_H

