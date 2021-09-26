/*
 * imgsensor_legacy.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * image sensor legacy interface
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

#ifndef _IMGSENSOR_LEGACY_H
#define _IMGSENSOR_LEGACY_H

#include "imgsensor_i2c.h"

int iReadRegI2C(u8 *send_data, u16 size_send_data,
	u8 *recv_data, u16 size_recv_data,
	u16 i2c_id);

int iReadRegI2CTiming(u8 *send_data, u16 size_send_data, u8 *recv_data,
	u16 size_recv_data, u16 i2c_id, u16 timing);

int iWriteRegI2C(u8 *send_data, u16 size_send_data, u16 i2c_id);

int iWriteRegI2CTiming(u8 *send_data, u16 size_send_data,
	u16 i2c_id, u16 timing);

int iBurstWriteReg(u8 *data, u32 bytes, u16 i2c_id);

int iBurstWriteReg_multi(u8 *data, u32 bytes, u16 i2c_id,
	u16 transfer_length, u16 timing);

int burst_write_reg_multi(
	struct IMGSENSOR_I2C_CFG *i2c_cfg,
	u8 *data, u32 bytes, u16 i2c_id,
	u16 transfer_length, u16 timing);

#endif
