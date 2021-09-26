/*
 * gc8034_dd_driver.c
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
#include "gc8034_dd_driver.h"

#define DEBUG_GC8034_DD 0
#define PFX "gc8034_dd"
#define log_debug(fmt, args...) \
	do { \
		if (DEBUG_GC8034_DD) \
			pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args); \
	} while (0)
#define log_inf(fmt, args...) \
	pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define log_err(fmt, args...) \
	pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

static struct gc8034_otp_t gc8034_otp_info;
static kal_uint8 i2c_write_id = 0x6e;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xff)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xff), (char)(para & 0xff)};

	iWriteRegI2C(pu_send_cmd, 2, i2c_write_id);
}

static void gc8034_read_otp_kgroup(kal_uint8 page, kal_uint8 addr, kal_uint8 *buff, int size)
{
	kal_uint16 i;
	kal_uint8 regf4 = read_cmos_sensor(0xf4);

	write_cmos_sensor(0xd4, ((page << 2) & 0x3c) + ((addr >> 5) & 0x03));
	write_cmos_sensor(0xd5, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf4, regf4 | 0x02);
	write_cmos_sensor(0xf3, 0x80);
	for (i = 0; i < size; i++) {
		if (((addr + i) % 0x80) == 0) {
			write_cmos_sensor(0xf3, 0x00);
			write_cmos_sensor(0xf4, regf4 & 0xfd);
			write_cmos_sensor(0xd4, ((++page << 2) & 0x3c));
			write_cmos_sensor(0xd5, 0x00);
			mdelay(1);
			write_cmos_sensor(0xf3, 0x20);
			write_cmos_sensor(0xf4, regf4 | 0x02);
			write_cmos_sensor(0xf3, 0x80);
		}
		buff[i] = read_cmos_sensor(0xd7);
	}
	write_cmos_sensor(0xf3, 0x00);
	write_cmos_sensor(0xf4, regf4 & 0xfd);
}

static kal_uint8 gc8034_read_otp(kal_uint8 page, kal_uint8 addr)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, ((page << 2) & 0x3c) + ((addr >> 5) & 0x03));
	write_cmos_sensor(0xd5, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);
	return  read_cmos_sensor(0xd7);
}

static void gc8034_gcore_set_otp_ddparam(void)
{
	kal_uint8 i;
	kal_uint8 total_number = 0;
	kal_uint8 cnt = 0;
	kal_uint8 ddtempbuff[4 * MAX_DD_NUM] = { 0 };

	total_number = gc8034_read_otp(OTP_PAGE0, DD_TOTAL_NUMBER) + gc8034_read_otp(OTP_PAGE0, DD_ERROR_NUMBER);
	if (total_number > MAX_DD_NUM) {
		log_inf("GC8034_OTP : total_number = %d exceeds MAX_DD_NUM.\n", total_number);
		total_number = MAX_DD_NUM;
	}
	gc8034_read_otp_kgroup(OTP_PAGE0, DD_ROM_START, ddtempbuff, 4 * total_number);

	for (i = 0; i < total_number; i++) {
		if ((ddtempbuff[4 * i + 3] & 0x80) == 0x80) {
			if ((ddtempbuff[4 * i + 3] & 0x03) == 0x03) {
				gc8034_otp_info.dd_param[cnt].x = ((ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
				gc8034_otp_info.dd_param[cnt].y = (ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
				gc8034_otp_info.dd_param[cnt++].t = 2;
				gc8034_otp_info.dd_param[cnt].x = ((ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
				gc8034_otp_info.dd_param[cnt].y = (ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + 1;
				gc8034_otp_info.dd_param[cnt++].t = 2;
			} else {
				gc8034_otp_info.dd_param[cnt].x = ((ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
				gc8034_otp_info.dd_param[cnt].y = (ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
				gc8034_otp_info.dd_param[cnt++].t = ddtempbuff[4 * i + 3] & 0x03;
			}
		}
	}
	gc8034_otp_info.dd_cnt = cnt;
	log_debug("GC8034_OTP : total_number = %d\n", gc8034_otp_info.dd_cnt);
}

static void gc8034_gcore_set_otp_reginfo(void)
{
	kal_uint16 i, j;
	kal_uint8 temp = 0;
#if GC8034OTP_DEBUG
	kal_uint8  debug[128] = { 0 };
#endif
	gc8034_otp_info.reg_num = 0;

	log_debug("ENTER.\n");
	gc8034_otp_info.reg_flag = gc8034_read_otp(OTP_PAGE2, REG_INFO_FLAG);

	if ((gc8034_otp_info.reg_flag & 0x03) != 0x01) {
		log_inf("GC8034_OTP : reg_flag = %d\n", gc8034_otp_info.reg_flag);
		return;
	}

	for (i = 0; i < REGS_GROUP; i++) {
		temp = gc8034_read_otp(OTP_PAGE2, REG_INFO_PAGE + REGS_GROUP * i);
		for (j = 0; j < REGS_NUM; j++) {
			if (((temp >> (4 * j + 3)) & 0x01) == 0x01) {
				gc8034_otp_info.reg_page[gc8034_otp_info.reg_num] = (temp >> (4 * j)) & 0x03;
				gc8034_otp_info.reg_addr[gc8034_otp_info.reg_num] =
					gc8034_read_otp(OTP_PAGE2, REG_INFO_ADDR + REGS_GROUP * i + REGS_NUM * j);
				gc8034_otp_info.reg_value[gc8034_otp_info.reg_num] =
					gc8034_read_otp(OTP_PAGE2, REG_INFO_ADDR + REGS_GROUP * i + REGS_NUM * j + 1);
				gc8034_otp_info.reg_num++;
			}
		}
	}
#if GC8034OTP_DEBUG
	for (i = 0; i < 10; i++) {
		gc8034_read_otp_group(i, OTP_PAGE0, &debug[0], 128);
		for (j = 0; j < 128; j++)
			log_inf("debug: Page%d: addr = 0x%x, value = 0x%x\n", i, j, debug[j]);
	}
#endif
}

static void gc8034_read_otp_group(kal_uint8 page, kal_uint8 addr, kal_uint8 *buff, int size)
{
	kal_uint8 i;
	kal_uint8 regf4 = read_cmos_sensor(0xf4);

	write_cmos_sensor(0xd4, ((page << 2) & 0x3c) + ((addr >> 5) & 0x03));
	write_cmos_sensor(0xd5, (addr << 3) & 0xff);
	mdelay(1);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf4, regf4 | 0x02);
	write_cmos_sensor(0xf3, 0x80);
	for (i = 0; i < size; i++) {
		buff[i] = read_cmos_sensor(0xd7);
		log_debug("buff[i]= 0x%x\n", buff[i]);
	}
	write_cmos_sensor(0xf3, 0x00);
	write_cmos_sensor(0xf4, regf4 & 0xfd);
}

static void gc8034_gcore_read_otp_info(void)
{
	kal_uint8 flagdd = 0;

	if (memset_s(&gc8034_otp_info, sizeof(struct gc8034_otp_t), 0, sizeof(struct gc8034_otp_t)) != 0) {
		log_err("%s memset gc8034_otp_info fail\n", __func__);
		return;
	}
	/* Static Defective Pixel */
	flagdd = gc8034_read_otp(OTP_PAGE0, DD_FLAG);
	log_debug("GC8034_OTP_DD : flag_dd = 0x%x\n", flagdd);

	switch (flagdd & 0x03) {
	case 0x00:
		log_inf("GC8034_OTP_DD is Empty !!\n");
		gc8034_otp_info.dd_flag = 0x00;
		break;
	case 0x01:
		log_inf("GC8034_OTP_DD is Valid!!\n");
		gc8034_otp_info.dd_flag = 0x01;
		gc8034_gcore_set_otp_ddparam();
		break;
	default:
		log_inf("GC8034_OTP_DD is Invalid(%x) !!\n", flagdd & 0x03);
		gc8034_otp_info.dd_flag = 0x02;
		break;
	}

	/* chip regs */
	gc8034_gcore_set_otp_reginfo();
	/*only read once*/
	gc8034_read_otp_group(OTP_PAGE0, GC8034_OTP_ID_DATA_OFFSET, &gc8034_otp_info.otp_id[0], GC8034_OTP_ID_SIZE);
}

static void gc8034_gcore_update_dd(void)
{
	kal_uint8 i = 0, j = 0;
	kal_uint8 temp_val0 = 0, temp_val1 = 0, temp_val2 = 0;
	struct gc8034_dd_t dd_temp = {0, 0, 0};

	if (gc8034_otp_info.dd_flag == 0x01) {
		log_inf("auto load start!\n");
		for (i = 0; i < gc8034_otp_info.dd_cnt - 1; i++) {
			for (j = i + 1; j < gc8034_otp_info.dd_cnt; j++) {
				if (gc8034_otp_info.dd_param[i].y * DD_WIDTH + gc8034_otp_info.dd_param[i].x
					> gc8034_otp_info.dd_param[j].y * DD_WIDTH + gc8034_otp_info.dd_param[j].x) {
					dd_temp.x = gc8034_otp_info.dd_param[i].x;
					dd_temp.y = gc8034_otp_info.dd_param[i].y;
					dd_temp.t = gc8034_otp_info.dd_param[i].t;
					gc8034_otp_info.dd_param[i].x = gc8034_otp_info.dd_param[j].x;
					gc8034_otp_info.dd_param[i].y = gc8034_otp_info.dd_param[j].y;
					gc8034_otp_info.dd_param[i].t = gc8034_otp_info.dd_param[j].t;
					gc8034_otp_info.dd_param[j].x = dd_temp.x;
					gc8034_otp_info.dd_param[j].y = dd_temp.y;
					gc8034_otp_info.dd_param[j].t = dd_temp.t;
				}
			}
		}
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xa9, 0x01);
		for (i = 0; i < gc8034_otp_info.dd_cnt; i++) {
			temp_val0 = gc8034_otp_info.dd_param[i].x & 0x00ff;
			temp_val1 = ((gc8034_otp_info.dd_param[i].y & 0x000f) << 4)
				+ ((gc8034_otp_info.dd_param[i].x & 0x0f00)>>8);
			temp_val2 = (gc8034_otp_info.dd_param[i].y & 0x0ff0) >> 4;
			write_cmos_sensor(0xaa, i);
			write_cmos_sensor(0xac, temp_val0);
			write_cmos_sensor(0xac, temp_val1);
			write_cmos_sensor(0xac, temp_val2);
			write_cmos_sensor(0xac, gc8034_otp_info.dd_param[i].t);
			log_debug("val0 = 0x%x , val1 = 0x%x , val2 = 0x%x gc8034_otp_info.dd_param[i].t=%x\n",
				temp_val0, temp_val1, temp_val2, gc8034_otp_info.dd_param[i].t);
			log_debug("x = %d, y = %d\n",
				((temp_val1 & 0x0f) << 8) + temp_val0, (temp_val2 << 4) + ((temp_val1 & 0xf0) >> 4));
		}
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0xfe, 0x00);
	}
}

void gc8034_otp_identify(void)
{
	log_debug("enter\n");
	write_cmos_sensor(0xf2, 0x01);
	write_cmos_sensor(0xf4, 0x88);
	write_cmos_sensor(0xf5, 0x19);
	write_cmos_sensor(0xf6, 0x44);
	write_cmos_sensor(0xf8, 0x63);
	write_cmos_sensor(0xfa, 0x45);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xf7, 0x97);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0xea);
	write_cmos_sensor(0xfc, 0xee);
	gc8034_gcore_read_otp_info();
	write_cmos_sensor(0xf2, 0x00);
	write_cmos_sensor(0xf4, 0x80);
	write_cmos_sensor(0xf5, 0x19);
	write_cmos_sensor(0xf6, 0x44);
	write_cmos_sensor(0xf8, 0x63);
	write_cmos_sensor(0xfa, 0x45);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xf7, 0x95);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0xea);
	write_cmos_sensor(0xfc, 0xee);
}

void gc8034_otp_function(void)
{
	kal_uint8 i = 0;
	kal_uint8 flag = 0;
	kal_uint8 otp_id[GC8034_OTP_ID_SIZE];

	log_debug("enter\n");
	if (memset_s(&otp_id, GC8034_OTP_ID_SIZE, 0, GC8034_OTP_ID_SIZE) != 0) {
		log_err("%s memset gc8034_otp_info fail\n", __func__);
		return;
	}
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xf7, 0x97);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0xee);
	write_cmos_sensor(0xf2, 0x01);
	write_cmos_sensor(0xf4, 0x88);
	gc8034_read_otp_group(OTP_PAGE0, GC8034_OTP_ID_DATA_OFFSET, &otp_id[0], GC8034_OTP_ID_SIZE);
	for (i = 0; i < GC8034_OTP_ID_SIZE; i++) {
		log_debug("%s,line=%d gc8034_otp_info.otp_id=%d otp_id[i]=%d",
			__func__, __LINE__, gc8034_otp_info.otp_id[i], otp_id[i]);
		if (otp_id[i] != gc8034_otp_info.otp_id[i]) {
			flag = 1;
			break;
		}
	}
	if (flag == 1) {
		log_debug("otp id mismatch, read again");
		gc8034_gcore_read_otp_info();
	}
	gc8034_gcore_update_dd();
	write_cmos_sensor(0xf2, 0x00);
	write_cmos_sensor(0xf4, 0x80);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xf7, 0x95);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xfc, 0xee);
}
