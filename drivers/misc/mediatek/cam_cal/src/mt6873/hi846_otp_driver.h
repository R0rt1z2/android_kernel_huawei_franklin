/*
 * hi846_otp_driver.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera hi846_otp_drive
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


#ifndef __HI846_OTP_DRIVER__
#define __HI846_OTP_DRIVER__

#define HI846_OTP_REG_CMD	0x0702
#define HI846_OTP_REG_WDATA	0x0706
#define HI846_OTP_REG_RDATA	0x0708
#define HI846_OTP_REG_ADDRH	0x070A
#define HI846_OTP_REG_ADDRL	0x070B

#define HI846_OTP_CMD_NORMAL			0x00
#define HI846_OTP_CMD_CONTINUOUS_READ	0x01
#define HI846_OTP_CMD_CONTINUOUS_WRITE	0x02

#define HI846_OTP_OFFSET_MODULE_FLAG	0x0201//0x0CC0
#define HI846_OTP_OFFSET_SN_FLAG		0x021A
#define HI846_OTP_OFFSET_AWB_FLAG		0x023F//0x0CD1
#define HI846_OTP_OFFSET_LSC_FLAG		0x0273//0x0CDD

#define HI846_OTP_OFFSET_MODULE1		0x0202//0x0CC1
#define HI846_OTP_OFFSET_MODULE2		0x020A//0x1430
#define HI846_OTP_OFFSET_MODULE3        0x0212
#define HI846_OTP_OFFSET_SN1		    0x021B
#define HI846_OTP_OFFSET_SN2		    0x0227
#define HI846_OTP_OFFSET_SN3            0x0233
#define HI846_OTP_OFFSET_AWB1			0x0240
#define HI846_OTP_OFFSET_AWB2			0x0251
#define HI846_OTP_OFFSET_AWB3			0x0262
#define HI846_OTP_OFFSET_LSC1			0x0274
#define HI846_OTP_OFFSET_LSC2			0x09C1
#define HI846_OTP_OFFSET_LSC3			0x110E

#define HI846_OTP_SIZE_MODULE	        8
#define HI846_OTP_SIZE_INFO_DATA	    7
#define HI846_OTP_SIZE_SN	            12
#define HI846_OTP_SIZE_SN_DATA	        4
#define HI846_OTP_SIZE_AWB	            17
#define HI846_OTP_SIZE_AWB_DATA	        16
#define HI846_OTP_SIZE_LSC	            1869
#define HI846_OTP_SIZE_LSC_DATA	        1868

#define HI846_GROUP1_FLAG               0x01
#define HI846_GROUP2_FLAG               0x13
#define HI846_GROUP3_FLAG               0x37

typedef unsigned char kal_uint8;
typedef unsigned short kal_uint16;
typedef unsigned int kal_uint32;

struct hi846_otp_t {
	kal_uint8 flag_module;
	kal_uint8 flag_sn;
	kal_uint8 flag_awb;
	kal_uint8 flag_lsc;
	kal_uint8 module[HI846_OTP_SIZE_MODULE];
	kal_uint8 sn[HI846_OTP_SIZE_SN];
	kal_uint8 awb[HI846_OTP_SIZE_AWB];
	kal_uint8 lsc[HI846_OTP_SIZE_LSC];
};

int hi846_read_buf_region(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif //__HI846_OTP_DRIVER__
