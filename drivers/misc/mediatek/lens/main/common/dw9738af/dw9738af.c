/*
 * DW9738AF.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * dw9738 af driver
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

#define LOG_INF(fmt, args...) pr_info("%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) pr_err("%s %d " fmt, __func__, __LINE__, ##args)

#define AF_DRVNAME "DW9738AF_DRV"
#define AF_I2C_SLAVE_ADDR (0x18 >> 1)
#define DW9738_CODE_H_REG 0x03
#define DW9738_CODE_L_REG 0x04
#define DW9738_CONTROL_REG 0x02
#define DW9738_MODE_REG 0x06
#define DW9738_RESON_REG 0x07
#define DW9738_TIMING_REG 0x08
#define DW9738_ACTIVE_VAL 0x0
#define DW9738_STANDBY_VAL 0x1
#define DW9738_RESON_VAL 0x01
#define DW9738_SAC3_VAL 0x0A
#define DW9738_TVIB_VAL 0x49
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

static int set_code(u16 dest_code)
{
	/* get high 2 bits */
	char code_reg_val_h[] = { DW9738_CODE_H_REG, ((dest_code >> 8) & 0x3) };
	/* get low 8 bits */
	char code_reg_val_l[] = { DW9738_CODE_L_REG, (dest_code & 0xFF) };

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;

	loge_if_ret(i2c_master_send(g_af_i2c_client,
		code_reg_val_h, sizeof(code_reg_val_h)));
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		code_reg_val_l, sizeof(code_reg_val_l)));
	LOG_INF("dest_code = %u", dest_code);

	return 0;
}

static int set_suspend(void)
{
	char read_ctrl[] = { DW9738_CONTROL_REG, DW9738_STANDBY_VAL };
	char ctrl = DW9738_CONTROL_REG;

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;

	loge_if_ret(i2c_master_send(g_af_i2c_client, &ctrl, 1)); /* 1 byte */
	loge_if_ret(i2c_master_recv(g_af_i2c_client,
		&read_ctrl[1], 1)); /* 1 byte */

	read_ctrl[1] |= DW9738_STANDBY_VAL;
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		read_ctrl, sizeof(read_ctrl)));

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
		LOG_ERR("copy to user failed when getting motor information");

	return 0;
}

/* init_af include driver initialization and standby mode */
static int init_af(void)
{
	char ctrl_standby[] = { DW9738_CONTROL_REG, DW9738_STANDBY_VAL };
	char ctrl_active[] = { DW9738_CONTROL_REG, DW9738_ACTIVE_VAL };
	char mode_sac3[] = { DW9738_MODE_REG, DW9738_SAC3_VAL };
	char reson_x2[] = { DW9738_RESON_REG, DW9738_RESON_VAL };
	char timing_tvib[] = { DW9738_TIMING_REG, DW9738_TVIB_VAL };

	LOG_INF("Enter");

	if (*g_af_opened == VCM_INIT_STATUS) {
		spin_lock(g_af_spinlock);
		*g_af_opened = VCM_OPEN_STATUS;
		spin_unlock(g_af_spinlock);
	}

	loge_if_ret(i2c_master_send(g_af_i2c_client,
		ctrl_standby, sizeof(ctrl_standby)));
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		ctrl_active, sizeof(ctrl_active)));
	mdelay(5); /* delay 5ms for steady */
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		mode_sac3, sizeof(mode_sac3)));
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		reson_x2, sizeof(reson_x2)));
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		timing_tvib, sizeof(timing_tvib)));

	LOG_INF("Exit");
	return 0;
}

/* move_af only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	if (set_code((unsigned short)position)) {
		LOG_ERR("set I2C failed when moving the motor\n");
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

long dw9738af_ioctl(struct file *af_file, unsigned int command,
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
		LOG_INF("No CMD");
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
int dw9738af_release(struct inode *af_inode, struct file *af_file)
{
	LOG_INF("Start");

	return_error_on_null(g_af_opened);
	if (*g_af_opened == VCM_OPEN_STATUS) {
		LOG_INF("Wait");
		set_code(VCM_RELEASE_CODE);
		mdelay(VCM_WAIT_RELEASE_TIME);
	}

	if (*g_af_opened != VCM_CLOSE_STATUS) {
		LOG_INF("Free");
		spin_lock(g_af_spinlock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spinlock);
	}

	if (set_suspend() < 0)
		LOG_ERR("set_suspend failed\n");

	LOG_INF("End");

	return 0;
}

int dw9738af_set_i2c_client(struct i2c_client *af_i2c_client,
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

int dw9738af_get_file_name(unsigned char *af_file_name)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[MAX_LEN] = { 0 };
	char *file_string = NULL;
	errno_t rc;
	return_error_on_null(af_file_name);

	if (sprintf_s(file_path, sizeof(file_path), "%s", __FILE__) < 0) {
		LOG_ERR("sprintf_s fail");
		return -1;
	}

	file_string = strrchr(file_path, '/');
	return_error_on_null(file_string);
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	rc = strcpy_s(af_file_name, STRUCT_MOTOR_LEN, file_string);
	if (rc != EOK) {
		LOG_ERR("strcpy_s fail, rc = %d", rc);
		return -1;
	}

	LOG_INF("FileName : %s", af_file_name);
#else
	af_file_name[0] = '\0';
#endif
	return 0;
}
