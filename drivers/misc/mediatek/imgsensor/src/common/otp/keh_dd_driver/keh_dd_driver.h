/*
 * keh_dd_driver.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera keh_dd_driver
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

#ifndef KEH_DD_DRIVER_H
#define KEH_DD_DRIVER_H

#include "kd_camera_typedef.h"

/* SENSOR DD INFO */
/* Please do not enable the following micro definition, if you are not debuging otp function */
#define KEH_OTP_ID_SIZE                  9
#define KEH_OTP_ID_DATA_OFFSET           0x0020
#define KEH_OTP_DPC_FLAG_OFFSET          0x0068
#define KEH_OTP_DPC_TOTAL_NUM_OFFSET     0x0070
#define KEH_OTP_DPC_ERROR_NUM_OFFSET     0x0078
#define KEH_OTP_DATA_LENGTH              1024
#define KEH_OTP_FLAG_EMPTY               0x00
#define KEH_OTP_FLAG_VALID               0x01
#define KEH_OTP_FLAG_INVALID             0x02
#define KEH_OTP_GET_2BIT_FLAG(flag, bit) ((flag >> bit) & 0x03)

struct keh_dpc_t {
	kal_uint8 flag;
	kal_uint16 total_num;
};

struct keh_otp_t {
	kal_uint8 otp_id[KEH_OTP_ID_SIZE];
	struct keh_dpc_t dpc;
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
	u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
#endif
