/*
 * FP5516AF.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * fp5516 af driver
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

#define AF_DRVNAME "FP5516AF_DRV"
#define AF_I2C_SLAVE_ADDR (0x18 >> 1)
#define FP5516_CODE_H_REG 0x03
#define FP5516_CODE_L_REG 0x04
#define FP5516_CONTROL_REG 0x02
#define FP5516_MODE_REG 0x06
#define FP5516_AT_REG 0x07
#define FP5516_ACTIVE_VAL 0x0
#define FP5516_STANDBY_VAL 0x1
#define FP5516_RING_VAL 0x2
#define FP5516_AT_VAL 0x06
#define FP5516_SAC3_VAL 0x40
#define VCM_RELEASE_CODE 512
#define VCM_WAIT_RELEASE_TIME 12
#define REG_16BITS_LEN 2
#define VCM_CLOSE_STATUS 0
#define VCM_INIT_STATUS 1
#define VCM_OPEN_STATUS 2
#define STRUCT_MOTOR_LEN 32
#define MAX_LEN 256

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spinlock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023; // af macro code
static unsigned long g_cur_position;
static const char *g_ic_name = "FP5516";
static const char *g_module_name = "FP";

static int set_code(u16 dest_code)
{
	/* get high 2 bits */
	char code_reg_val_h[] = { FP5516_CODE_H_REG, ((dest_code >> 8) & 0x3) };
	/* get low 8 bits */
	char code_reg_val_l[] = { FP5516_CODE_L_REG, (dest_code & 0xFF) };

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;

	loge_if_ret(i2c_master_send(g_af_i2c_client,
		code_reg_val_h, sizeof(code_reg_val_h)));
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		code_reg_val_l, sizeof(code_reg_val_l)));
	log_info("dest_code = %u", dest_code);

	return 0;
}

static int set_suspend(void)
{
	char read_ctrl[2] = { FP5516_CONTROL_REG, FP5516_STANDBY_VAL };
	char ctrl = FP5516_CONTROL_REG;

	if (!g_af_i2c_client) {
		log_err("i2c client is NULL\n");
		return -1;
	}

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;

	loge_if_ret(i2c_master_send(g_af_i2c_client, &ctrl, 1)); /* 1 byte */
	loge_if_ret(i2c_master_recv(g_af_i2c_client, &read_ctrl[1], 1)); /* 1 byte */

	read_ctrl[1] = read_ctrl[1] | FP5516_STANDBY_VAL;
	loge_if_ret(i2c_master_send(g_af_i2c_client, read_ctrl, 2)); /* 2 bytes */

	return 0;
}

static int get_af_info(__user struct stAF_MotorInfo *af_motor_info)
{
	struct stAF_MotorInfo motor_info;
	return_error_on_null(af_motor_info);
	return_error_on_null(g_af_opened);
	motor_info.u4MacroPosition = g_af_macro;
	motor_info.u4InfPosition = g_af_inf;
	motor_info.u4CurrentPosition = g_cur_position;
	motor_info.bIsSupportSR = 1;
	motor_info.bIsMotorMoving = 1;

	if (*g_af_opened >= VCM_INIT_STATUS)
		motor_info.bIsMotorOpen = 1;
	else
		motor_info.bIsMotorOpen = 0;

	if (copy_to_user(af_motor_info, &motor_info,
		sizeof(struct stAF_MotorInfo)))
		log_err("copy to user failed when getting motor information");

	return 0;
}

/* init_af include driver initialization and standby mode */
static int init_af(void)
{
	/* register sequence table */
	const struct i2c_cfg_t i2c_cfg[] = {
		{
			.setting = { FP5516_CONTROL_REG, FP5516_STANDBY_VAL },
			.delay = 0,
		}, {
			.setting = { FP5516_CONTROL_REG, FP5516_ACTIVE_VAL },
			.delay = 5,
		}, {
			.setting = { FP5516_CONTROL_REG, FP5516_RING_VAL },
			.delay = 0,
		}, {
			.setting = { FP5516_MODE_REG, FP5516_SAC3_VAL },
			.delay = 0,
		}, {
			.setting = { FP5516_AT_REG, FP5516_AT_VAL },
			.delay = 0,
		}
	};

	if (*g_af_opened == VCM_INIT_STATUS) {
		spin_lock(g_af_spinlock);
		*g_af_opened = VCM_OPEN_STATUS;
		spin_unlock(g_af_spinlock);
	}

	if (lens_set_setting(g_af_i2c_client,
		i2c_cfg, array_size(i2c_cfg)) == -EIO) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("i2c send fail\n");
		return -1;
	}

	log_info("Exit");
	return 0;
}

/* move_af only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	if (set_code((unsigned short)position)) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_MOVE_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("set I2C failed when moving the motor\n");
		return -1;
	}
	g_cur_position = position;

	return 0;
}

static inline int set_af_inf(unsigned long position)
{
	spin_lock(g_af_spinlock);
	g_af_inf = position;
	spin_unlock(g_af_spinlock);
	return 0;
}

static inline int set_af_macro(unsigned long position)
{
	spin_lock(g_af_spinlock);
	g_af_macro = position;
	spin_unlock(g_af_spinlock);
	return 0;
}

long fp5516af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param)
{
	int ret = 0;
	return_error_on_null(af_file);

	switch (command) {
	case AFIOC_G_MOTORINFO:
		ret = get_af_info((__user struct stAF_MotorInfo *)(param));
		break;
	case AFIOC_T_MOVETO:
		ret = move_af(param);
		break;
	case AFIOC_T_SETINFPOS:
		ret = set_af_inf(param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret = set_af_macro(param);
		break;
	default:
		log_info("No CMD");
		ret = -EPERM;
		break;
	}
	return ret;
}

/*
 * Main jobs
 * 1.Deallocate anything that "open" allocated in private_data.
 * 2.Shut down the device on last close.
 * 3.Only called once on last time.
 * Q1: Try release multiple times.
 */
int fp5516af_release(struct inode *af_inode, struct file *af_file)
{
	int ret;
	log_info("Start");

	return_error_on_null(g_af_opened);
	if (*g_af_opened == VCM_OPEN_STATUS) {
		log_info("Wait");
		set_code(VCM_RELEASE_CODE);
		mdelay(VCM_WAIT_RELEASE_TIME);
	}

	if (*g_af_opened != VCM_CLOSE_STATUS) {
		log_info("Free");
		spin_lock(g_af_spinlock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spinlock);
	}

	ret = set_suspend();
	if (ret < 0)
		log_err("set_suspend failed\n");
	log_info("End");

	return 0;
}

int fp5516af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened)
{
	return_error_on_null(af_i2c_client);
	return_error_on_null(af_spinlock);
	return_error_on_null(af_opened);
	g_af_i2c_client = af_i2c_client;
	g_af_spinlock = af_spinlock;
	g_af_opened = af_opened;

	init_af();

	return 1;
}

int fp5516af_get_file_name(unsigned char *af_file_name)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[MAX_LEN] = { 0 };
	char *file_string = NULL;
	errno_t rc;
	return_error_on_null(af_file_name);

	if (sprintf_s(file_path, sizeof(file_path), "%s", __FILE__) < 0) {
		log_err("sprintf_s fail");
		return -1;
	}

	file_string = strrchr(file_path, '/');
	return_error_on_null(file_string);
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	rc = strcpy_s(af_file_name, STRUCT_MOTOR_LEN, file_string);
	if (rc != EOK) {
		log_err("strcpy_s fail, rc = %d", rc);
		return -1;
	}

	log_info("FileName : %s", af_file_name);
#else
	af_file_name[0] = '\0';
#endif
	return 0;
}
