/*
 * gc8034_dd_driver.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera gc8034_dd_driver
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

#ifndef GC8034_DD_DRIVER_H
#define GC8034_DD_DRIVER_H

#include "kd_camera_typedef.h"

/* SENSOR DD INFO */
/* Please do not enable the following micro definition, if you are not debuging otp function*/
#define GC8034OTP_DEBUG              0

#define DD_WIDTH                     3284
#define DD_HEIGHT                    2464
#define GC8034_OTP_ID_SIZE           9
#define GC8034_OTP_ID_DATA_OFFSET    0x02
#define DD_FLAG 0x0b
#define DD_TOTAL_NUMBER 0x0c
#define DD_ERROR_NUMBER 0x0d
#define DD_ROM_START 0x0e
#define REG_INFO_FLAG 0x4e
#define REG_INFO_PAGE 0x4f
#define REG_INFO_ADDR 0X50
#define MAX_DD_NUM 80
#define REGS_GROUP 5
#define REGS_NUM 2

struct gc8034_dd_t {
	kal_uint16 x;
	kal_uint16 y;
	kal_uint16 t;
};

struct gc8034_otp_t {
	kal_uint8 otp_id[GC8034_OTP_ID_SIZE];
	kal_uint8  dd_cnt;
	kal_uint8  dd_flag;
	struct gc8034_dd_t dd_param[2 * MAX_DD_NUM];
	kal_uint8  reg_flag;
	kal_uint8  reg_num;
	kal_uint8  reg_page[REGS_GROUP * REGS_NUM];
	kal_uint8  reg_addr[REGS_GROUP * REGS_NUM];
	kal_uint8  reg_value[REGS_GROUP * REGS_NUM];
};

enum {
	OTP_PAGE0 = 0,
	OTP_PAGE1,
	OTP_PAGE2,
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
	u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
#endif
