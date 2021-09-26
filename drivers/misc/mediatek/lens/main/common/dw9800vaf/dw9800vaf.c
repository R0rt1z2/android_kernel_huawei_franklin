/*
 * Copyright (C) 2020-2020 Huawei Technologies Co., Ltd.
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
#include <securec.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "DW9800VAF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18
#define DW9800V_CONTROL_REG 0x02
#define DW9800V_STANDBY_VAL 0x1

#define AF_DEBUG
#ifdef AF_DEBUG
#define log_inf(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define log_inf(format, args...)
#endif

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023;
static unsigned long g_curr_position;
static const char *g_ic_name = "DW9800VAF";
static const char *g_module_name = "DW";

static int af_write_reg(u16 af_data)
{
	int ret_value;
	char send_cmd1[2] = {0x03, ((af_data >> 8) & 0x3)};
	char send_cmd2[2] = {0x04, (af_data & 0xFF)};

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;
	ret_value = i2c_master_send(g_af_i2c_client, send_cmd1, 2);
	if (ret_value < 0) {
		log_inf("I2C send failed!!\n");
		return -1;
	}
	ret_value = i2c_master_send(g_af_i2c_client, send_cmd2, 2);
	if (ret_value < 0) {
		log_inf("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static int set_suspend(void)
{
	int ret;
	char read_ctrl[2] = { DW9800V_CONTROL_REG, DW9800V_STANDBY_VAL };
	char ctrl = DW9800V_CONTROL_REG;

	if (!g_af_i2c_client) {
		log_inf("i2c client is NULL\n");
		return -1;
	}
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;

	ret = i2c_master_send(g_af_i2c_client, &ctrl, 1);
	if (ret < 0) {
		log_inf("I2C i2c_master_send failed!!\n");
		return -1;
	}
	ret = i2c_master_recv(g_af_i2c_client, &read_ctrl[1], 1);
	if (ret < 0) {
		log_inf("I2C i2c_master_recv failed!!\n");
		return -1;
	}
	read_ctrl[1] = (unsigned char)read_ctrl[1] | DW9800V_STANDBY_VAL;
	ret = i2c_master_send(g_af_i2c_client, read_ctrl, 2);
	if (ret < 0) {
		log_inf("I2C i2c_master_send failed!!\n");
		return -1;
	}

	return 0;
}

static inline int get_af_info(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;
	return_error_on_null(pstMotorInfo);
	return_error_on_null(g_af_opened);
	stMotorInfo.u4MacroPosition = g_af_macro;
	stMotorInfo.u4InfPosition = g_af_inf;
	stMotorInfo.u4CurrentPosition = g_curr_position;
	stMotorInfo.bIsSupportSR = 1;
	stMotorInfo.bIsMotorMoving = 1;

	if (*g_af_opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;
	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		log_inf("copy to user failed when getting motor information\n");

	return 0;
}

/* init_af include driver initialization and standby mode */
static int init_af(void)
{
	/* register sequence table */
	const struct i2c_cfg_t i2c_cfg[] = {
		{
			.setting = { 0x02, 0x01 }, /* addr, value */
			.delay = 0,
		}, {
			.setting = { 0x02, 0x00 },
			.delay = 0,
		}, {
			.setting = { 0x02, 0x02 },
			.delay = 0,
		}, {
			.setting = { 0x06, 0x40 },
			.delay = 0,
		}, {
			.setting = { 0x07, 0x0A },
			.delay = 0,
		}, {
			.setting = { 0x10, 0x01 },
			.delay = 0,
		}
	};
	if (*g_af_opened == 1) {
		spin_lock(g_af_spin_lock);
		*g_af_opened = 2;
		spin_unlock(g_af_spin_lock);
	}
	if (lens_set_setting(g_af_i2c_client,
		i2c_cfg, array_size(i2c_cfg)) == -EIO) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("i2c send fail\n");
		return -1;
	}
	log_inf("exit\n");
	return 0;
}

/* move_af only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	if (af_write_reg((unsigned short)position) == 0) {
		g_curr_position = position;
		return 0;
	}
	lens_hiview_handle(DSM_CAMERA_ACTUATOR_MOVE_FAIL,
		g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
	log_err("set I2C failed when moving the motor\n");

	return -1;
}

static inline int set_af_inf(unsigned long af_position)
{
	spin_lock(g_af_spin_lock);
	g_af_inf = af_position;
	spin_unlock(g_af_spin_lock);
	return 0;
}

static inline int set_af_macro(unsigned long af_position)
{
	spin_lock(g_af_spin_lock);
	g_af_macro = af_position;
	spin_unlock(g_af_spin_lock);
	return 0;
}

long dw9800vaf_ioctl(struct file *af_file, unsigned int af_command,
		    unsigned long af_param)
{
	long ret_value = 0;
	return_error_on_null(af_file);

	switch (af_command) {
	case AFIOC_G_MOTORINFO:
		ret_value =
			get_af_info((__user struct stAF_MotorInfo *)(af_param));
		break;
	case AFIOC_T_MOVETO:
		ret_value = move_af(af_param);
		break;
	case AFIOC_T_SETINFPOS:
		ret_value = set_af_inf(af_param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret_value = set_af_macro(af_param);
		break;
	default:
		log_inf("No CMD\n");
		ret_value = -EPERM;
		break;
	}

	return ret_value;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int dw9800vaf_release(struct inode *af_inode, struct file *af_file)
{
	int ret;
	log_inf("Start\n");

	if (*g_af_opened == 2) {
		log_inf("Wait\n");
		af_write_reg(512);
		msleep(12);
	}

	if (*g_af_opened) {
		log_inf("Free\n");

		spin_lock(g_af_spin_lock);
		*g_af_opened = 0;
		spin_unlock(g_af_spin_lock);
	}

	ret = set_suspend();
	if (ret < 0)
		log_inf("DW9800VAF_Release set_suspend failed\n");

	log_inf("End\n");

	return 0;
}

int dw9800vaf_set_i2c_client(struct i2c_client *pst_af_i2c_client,
			  spinlock_t *p_af_spin_lock, int *p_af_opened)
{
	return_error_on_null(pst_af_i2c_client);
	return_error_on_null(p_af_spin_lock);
	return_error_on_null(p_af_opened);
	g_af_i2c_client = pst_af_i2c_client;
	g_af_spin_lock = p_af_spin_lock;
	g_af_opened = p_af_opened;

	init_af();

	return 1;
}

int dw9800vaf_get_file_name(unsigned char *p_file_name)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[256];
	char *file_string;
	errno_t rc;
	return_error_on_null(p_file_name);

	rc = sprintf_s(file_path, sizeof(file_path), "%s", __FILE__);
	if (rc < 0)
		log_inf("sprintf_s fail");
	file_string = strrchr(file_path, '/');
	if (!file_string) {
		log_inf("file_string NULL!");
		return -1;
	}
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	rc = strncpy_s(p_file_name, STRUCT_MOTOR_NAME, file_string, AF_MOTOR_NAME);
	if (rc != EOK)
		log_inf("strncpy_s fail, rc = %d\n", rc);
	log_inf("FileName : %s\n", p_file_name);
#else
	p_file_name[0] = '\0';
#endif
	return 0;
}
