/*
 * dw9800af.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * dw9800 af driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <securec.h>
#include "lens_info.h"

#define AF_DRVNAME "DW9800AF_DRV"
#define AF_I2C_SLAVE_ADDR  (0x18 >> 1)

#define DW9800_VCM_IC_INFO_REG  0x00
#define DW9800_VCM_CONTROL_REG  0x02
#define DW9800_VCM_H_CODE_REG   0x03
#define DW9800_VCM_L_CODE_REG   0x04
#define DW9800_VCM_ACTIVE_VAL   0x0
#define DW9800_VCM_STANDBY_VAL  0x1
#define DW9800_VCM_CODE_OFFSET  8
#define DW9800_VCM_H_CODE_BIT   0x3
#define DW9800_VCM_L_CODE_BIT   0xFF
#define DW9800_MODE_REG_DELAY   2
#define DW9800_VCM_MODE_REG     0x06
#define DW9800_VCM_RESON_REG    0x07
#define DW9800_VCM_RING_VAL     0x2
#define DW9800_VCM_SAC3_VAL     0x40
#define DW9800_VCM_TVIB_VAL     0x06
#define DW9800_VCM_ACT_DELAY    1
#define DW9800_VCM_HALL_DELAY   10
#define DW9800_CODE_MAX         1023
#define SHIFT_8BIT              8
#define WRITE_SIZE              2
#define READ_SIZE               1

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023; /* af macro code */
static unsigned long g_af_current_position;
static const char *g_ic_name = "DW9800";
static const char *g_module_name = "DW";

static int af_read_reg(u8 addr, u8 *result)
{
	char buff = 0;
	char send_data[READ_SIZE] = { addr };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_data, READ_SIZE));
	loge_if_ret(i2c_master_recv(g_af_i2c_client, &buff, READ_SIZE));
	*result = buff;

	return 0;
}

static int af_write_reg(u8 addr, u8 data)
{
	char send_data[WRITE_SIZE] = { addr, data };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_data, WRITE_SIZE));

	return 0;
}

static inline int get_af_info(__user struct stAF_MotorInfo *p_motor_info)
{
	struct stAF_MotorInfo motor_info;

	return_error_on_null(p_motor_info);
	motor_info.u4MacroPosition = g_af_macro;
	motor_info.u4InfPosition = g_af_inf;
	motor_info.u4CurrentPosition = g_af_current_position;
	motor_info.bIsSupportSR = 1; /* support */
	motor_info.bIsMotorMoving = 1; /* moving */

	if (*g_af_opened >= VCM_INIT_STATUS)
		motor_info.bIsMotorOpen = 1; /* open */
	else
		motor_info.bIsMotorOpen = 0; /* close */

	if (copy_to_user(p_motor_info, &motor_info,
			 sizeof(struct stAF_MotorInfo)))
		log_err("copy to user fail when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int init_af(void)
{
	unsigned char ver = 0;
	const struct i2c_cfg_t dw9800_i2c_cfg[] = {
		{
			/* Power down mode operating */
			.setting = { DW9800_VCM_CONTROL_REG, 0x01 }, /* addr, value */
			.delay = 0,
		}, {
			/* Normal operating */
			.setting = { DW9800_VCM_CONTROL_REG, 0x00 },
			.delay = 1,
		}, {
			/* Ring control select */
			.setting = { DW9800_VCM_CONTROL_REG, 0x02 },
			.delay = 0,
		}, {
			 /* SAC3 mode */
			.setting = { DW9800_VCM_MODE_REG, 0x40 },
			.delay = 0,
		}, {
			/* Tvib = 13.8ms */
			.setting = { DW9800_VCM_RESON_REG, 0x06 },
			.delay = 0,
		},
	};
	log_info("+\n");

	if (*g_af_opened == VCM_INIT_STATUS) {
		/* read version */
		loge_if_ret(af_read_reg(DW9800_VCM_IC_INFO_REG, &ver));
		log_info("ver = %u", ver);

		/* start vcm */
		g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
		if (lens_set_setting(g_af_i2c_client,
			dw9800_i2c_cfg, array_size(dw9800_i2c_cfg)) == -EIO) {
			lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
				g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
			log_err("i2c send fail\n");
			return -1;
		}

		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_OPEN_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("-\n");
	return 0;
}

static inline int set_vcm_pos(unsigned long position)
{
	/* DW9800 max position 1023: 0x0011 1111 1111 */
	unsigned char msb = (position >> SHIFT_8BIT) & DW9800_VCM_H_CODE_BIT;
	unsigned char lsb = (position & DW9800_VCM_L_CODE_BIT);

	loge_if_ret(af_write_reg(DW9800_VCM_H_CODE_REG, msb));
	loge_if_ret(af_write_reg(DW9800_VCM_L_CODE_REG, lsb));

	return 0;
}

/* moveAF only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	if (position > DW9800_CODE_MAX) {
		log_err("position %u is invalid, should be 0~1023", position);
		return -1;
	}
	log_info("position = %u", position);

	if (set_vcm_pos(position) < 0) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_MOVE_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("set I2C failed when moving the motor\n");
		return -1;
	}

	g_af_current_position = position;
	return 0;
}

static inline int set_af_inf(unsigned long position)
{
	spin_lock(g_af_spin_lock);
	g_af_inf = position;
	spin_unlock(g_af_spin_lock);
	return 0;
}

static inline int set_af_macro(unsigned long position)
{
	spin_lock(g_af_spin_lock);
	g_af_macro = position;
	spin_unlock(g_af_spin_lock);
	return 0;
}

long dw9800af_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param)
{
	long ret = 0;

	switch (a_command) {
	case AFIOC_G_MOTORINFO:
		ret = get_af_info((__user struct stAF_MotorInfo *)(a_param));
		break;
	case AFIOC_T_MOVETO:
		ret = move_af(a_param);
		break;
	case AFIOC_T_SETINFPOS:
		ret = set_af_inf(a_param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret = set_af_macro(a_param);
		break;
	default:
		log_info("No CMD\n");
		ret = -EPERM;
		break;
	}

	return ret;
}

int dw9800af_release(struct inode *a_inode, struct file *a_file)
{
	unsigned char ctrl = 0;

	log_info("Start\n");

	if (*g_af_opened) {
		log_info("Free\n");
		loge_if_ret(af_read_reg(DW9800_VCM_CONTROL_REG, &ctrl));
		/* set PD Mode */
		ctrl = ctrl | 0x1;
		loge_if_ret(af_write_reg(DW9800_VCM_CONTROL_REG, ctrl));

		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("End\n");

	return 0;
}

int dw9800af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spin_lock = af_spin_lock;
	g_af_opened = af_opened;

	return init_af();
}

int dw9800af_get_file_name(unsigned char *file_name)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[256]; /* file name size */
	char *file_string = NULL;
	errno_t ret = EOK;

	return_error_on_null(file_name);

	if (sprintf_s(file_path, sizeof(file_path), "%s", __FILE__) < 0) {
		log_err("sprintf_s fail");
		return ret;
	}

	file_string = strrchr(file_path, '/');
	if (file_string == NULL) {
		log_err("file_string NULL!");
		return -1;
	}
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	ret = strcpy_s(file_name, AF_MOTOR_NAME, file_string);
	if (ret != EOK) {
		log_err("strcpy_s fail");
		return ret;
	}
	log_info("FileName : %s\n", file_name);
#else
	file_name[0] = '\0';
#endif
	return ret;
}
