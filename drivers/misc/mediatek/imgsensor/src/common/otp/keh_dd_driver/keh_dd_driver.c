/*
 * keh_dd_driver.c
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include "securec.h"
#include "keh_dd_driver.h"

#define DEBUG_KEH_DD 0
#define PFX "keh_dd"
#define log_debug(fmt, args...) \
	do { \
		if (DEBUG_KEH_DD) \
			pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args); \
	} while (0)
#define log_inf(fmt, args...) \
	pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define log_err(fmt, args...) \
	pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)
static struct keh_otp_t keh_otp_data;
static const kal_uint8 i2c_write_id = 0x7e;

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xff), (char)(para & 0xff) };

	iWriteRegI2C(pu_send_cmd, 2, i2c_write_id);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, i2c_write_id);
	return get_byte;
}

static kal_uint8 keh_otp_read_byte(kal_uint16 addr)
{
	write_cmos_sensor(0xfe, 0x02); // reset related
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20); // otp mode new

	return read_cmos_sensor(0x6c);
}

static void keh_otp_read_group(kal_uint16 addr,
	kal_uint8 *data, kal_uint16 length)
{
	kal_uint16 i;
	if ((((addr & 0x1fff) >> 3) + length) > KEH_OTP_DATA_LENGTH) {
		log_err("out of range, start addr: 0x%.4x, length = %d\n",
			addr & 0x1fff, length);
		return;
	}

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf3, 0x12);

	for (i = 0; i < length; i++)
		data[i] = read_cmos_sensor(0x6c);

	write_cmos_sensor(0xf3, 0x00);
}

static void keh_gcore_read_dpc(void)
{
	kal_uint8 flagdd;
	struct keh_dpc_t *pdpc = &keh_otp_data.dpc;

	flagdd = keh_otp_read_byte(KEH_OTP_DPC_FLAG_OFFSET);
	log_inf("dpc flag = 0x%x\n", flagdd);

	switch (KEH_OTP_GET_2BIT_FLAG(flagdd, 0)) {
	case KEH_OTP_FLAG_EMPTY:
		log_err("dpc info is empty!!\n");
		pdpc->flag = KEH_OTP_FLAG_EMPTY;
		break;
	case KEH_OTP_FLAG_VALID:
		log_inf("dpc info is valid!\n");
		pdpc->total_num = keh_otp_read_byte(KEH_OTP_DPC_TOTAL_NUM_OFFSET) +
			keh_otp_read_byte(KEH_OTP_DPC_ERROR_NUM_OFFSET);
		pdpc->flag = KEH_OTP_FLAG_VALID;
		log_inf("total_num = %d\n", pdpc->total_num);
		break;
	default:
		pdpc->flag = KEH_OTP_FLAG_INVALID;
		break;
	}
}

static void keh_otp_update_dd(void)
{
	kal_uint8 state;
	kal_uint8 n = 0;
	struct keh_dpc_t *pdpc = &keh_otp_data.dpc;

	if (pdpc->flag == KEH_OTP_FLAG_VALID) {
		log_inf("DD auto load start!\n");
		write_cmos_sensor(0xfe, 0x02);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xa9, 0x01);
		write_cmos_sensor(0x09, 0x33); // row start
		write_cmos_sensor(0x01, (pdpc->total_num >> 8) & 0x07); // DPHY analog mode1
		write_cmos_sensor(0x02, pdpc->total_num & 0xff); // DPHY analog mode2
		write_cmos_sensor(0x03, 0x00); // DPHY analog mode3
		write_cmos_sensor(0x04, 0x80); // FIFO prog full leve
		write_cmos_sensor(0x95, 0x0a); // out win height
		write_cmos_sensor(0x96, 0x30); // out win height
		write_cmos_sensor(0x97, 0x0a); // out win width
		write_cmos_sensor(0x98, 0x32); // out win width
		write_cmos_sensor(0x99, 0x07);
		write_cmos_sensor(0x9a, 0xa9);
		write_cmos_sensor(0xf3, 0x80);
		while (n < 3) {
			state = read_cmos_sensor(0x06);
			if ((state | 0xfe) == 0xff)
				mdelay(10);
			else
				n = 3;
			n++;
		}
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0x09, 0x00);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x80, 0x02); // long exp position
		write_cmos_sensor(0xfe, 0x00);
	}
}

void keh_otp_identify(void)
{
	log_debug("enter dpc identify\n");
	write_cmos_sensor(0xfc, 0x01); // cm mode
	write_cmos_sensor(0xf4, 0x40); // reduce powermode
	write_cmos_sensor(0xf5, 0xe9); // cp clk mode
	write_cmos_sensor(0xf6, 0x14); // wpllclk div refmp div
	write_cmos_sensor(0xf8, 0x49); // pll mode
	write_cmos_sensor(0xf9, 0x82); // rpllclk div pllmp prediv
	write_cmos_sensor(0xfa, 0x00); // clk div mode
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01); // clkp drv
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xf7, 0x01); // pll mode1
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0xfa, 0x10);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0xc0);
	write_cmos_sensor(0x59, 0x3f); // colgain offset
	write_cmos_sensor(0x55, 0x80); // colgain offset
	write_cmos_sensor(0x65, 0x80);
	write_cmos_sensor(0x66, 0x03);
	write_cmos_sensor(0xfe, 0x00);
	keh_otp_read_group(KEH_OTP_ID_DATA_OFFSET,
		&keh_otp_data.otp_id[0], KEH_OTP_ID_SIZE);
	keh_gcore_read_dpc();
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfa, 0x00);
}

void keh_otp_function(void)
{
	kal_uint8 i;
	kal_uint8 flag = 0;
	kal_uint8 otp_id[KEH_OTP_ID_SIZE];

	log_debug("enter dpc function\n");
	(void)memset_s(&otp_id, KEH_OTP_ID_SIZE, 0, KEH_OTP_ID_SIZE);
	write_cmos_sensor(0xfa, 0x10);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0xc0);
	write_cmos_sensor(0x59, 0x3f);
	write_cmos_sensor(0x55, 0x80);
	write_cmos_sensor(0x65, 0x80);
	write_cmos_sensor(0x66, 0x03);
	write_cmos_sensor(0xfe, 0x00);
	keh_otp_read_group(KEH_OTP_ID_DATA_OFFSET, &otp_id[0], KEH_OTP_ID_SIZE);
	for (i = 0; i < KEH_OTP_ID_SIZE; i++) {
		log_debug("%s,line=%d keh_otp_data.otp_id=%d otp_id[i]=%d",
			__func__, __LINE__, keh_otp_data.otp_id[i], otp_id[i]);
		if (otp_id[i] != keh_otp_data.otp_id[i]) {
			flag = 1;
			break;
		}
	}
	if (flag == 1) {
		log_debug("otp id mismatch, read again");
		(void)memset_s(&keh_otp_data, sizeof(keh_otp_data), 0, sizeof(keh_otp_data));
		for (i = 0; i < KEH_OTP_ID_SIZE; i++)
			keh_otp_data.otp_id[i] = otp_id[i];
		keh_gcore_read_dpc();
	}
	keh_otp_update_dd();
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfa, 0x00);
}
