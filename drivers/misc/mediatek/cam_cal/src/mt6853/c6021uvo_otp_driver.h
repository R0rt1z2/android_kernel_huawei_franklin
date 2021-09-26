/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: c6021uvo otp driver
 * Author: mazhaung
 * Create: 2020-08-01
 */

#ifndef __C6021UVO_OTP_DRIVER__
#define __C6021UVO_OTP_DRIVER__
#include <linux/delay.h>
#include <linux/i2c.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include <securec.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "eeprom_hw_map.h"

#define C6021UVO_OTP_REG_CMD			0x0702
#define C6021UVO_OTP_REG_WDATA			0x0706
#define C6021UVO_OTP_REG_RDATA			0x0708
#define C6021UVO_OTP_REG_ADDRH			0x070A
#define C6021UVO_OTP_REG_ADDRL			0x070B

#define C6021UVO_OTP_CMD_NORMAL			0x00
#define C6021UVO_OTP_CMD_CONTINUOUS_READ	0x01
#define C6021UVO_OTP_CMD_CONTINUOUS_WRITE	0x02

#define C6021UVO_OTP_OFFSET_MODULE_FLAG		0x0201
#define C6021UVO_OTP_OFFSET_SN_FLAG		0x021A
#define C6021UVO_OTP_OFFSET_AWB_FLAG		0x023F
#define C6021UVO_OTP_OFFSET_LSC_FLAG		0x0273

#define C6021UVO_OTP_OFFSET_MODULE1		0x0202
#define C6021UVO_OTP_DIFFERENCE_MODULE		0x8
#define C6021UVO_OTP_OFFSET_SN1			0x021B
#define C6021UVO_OTP_DIFFERENCE_SN		0xC
#define C6021UVO_OTP_OFFSET_AWB1		0x0240
#define C6021UVO_OTP_DIFFERENCE_AWB		0x11
#define C6021UVO_OTP_OFFSET_LSC1		0x0274
#define C6021UVO_OTP_DIFFERENCE_LSC		0x74D

#define C6021UVO_OTP_SIZE_MODULE		8
#define C6021UVO_OTP_SIZE_INFO_DATA		7
#define C6021UVO_OTP_SIZE_SN			12
#define C6021UVO_OTP_SIZE_SN_DATA		4
#define C6021UVO_OTP_SIZE_AWB			17
#define C6021UVO_OTP_SIZE_AWB_DATA		16
#define C6021UVO_OTP_SIZE_LSC			1869
#define C6021UVO_OTP_SIZE_LSC_DATA		1868
#define C6021UVO_OTP_READ_SIZE			1024

#define C6021UVO_GROUP1_FLAG			0x01
#define C6021UVO_DIFFERENCE_GROUP2_FLAG		0x12
#define C6021UVO_GROUP_FLAG_SIZE		3

typedef unsigned char kal_uint8;
typedef unsigned short kal_uint16;
typedef unsigned int kal_uint32;

struct c6021uvo_otp_t {
	kal_uint8 flag_module;
	kal_uint8 flag_sn;
	kal_uint8 flag_awb;
	kal_uint8 flag_lsc;
	kal_uint8 module[C6021UVO_OTP_SIZE_MODULE];
	kal_uint8 sn[C6021UVO_OTP_SIZE_SN];
	kal_uint8 awb[C6021UVO_OTP_SIZE_AWB];
	kal_uint8 lsc[C6021UVO_OTP_SIZE_LSC];
	kal_uint32 addr_flag[C6021UVO_GROUP_FLAG_SIZE];
	kal_uint32 addr_module[C6021UVO_GROUP_FLAG_SIZE];
	kal_uint32 addr_sn[C6021UVO_GROUP_FLAG_SIZE];
	kal_uint32 addr_awb[C6021UVO_GROUP_FLAG_SIZE];
	kal_uint32 addr_lsc[C6021UVO_GROUP_FLAG_SIZE];
};

struct c6021uvo_otp_addr {
	kal_uint32 addr_flag;
	kal_uint32 addr_flag_module;
	kal_uint32 addr_flag_sn;
	kal_uint32 addr_flag_awb;
	kal_uint32 addr_flag_lsc;
};

int c6021uvo_read_buf_region(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif
