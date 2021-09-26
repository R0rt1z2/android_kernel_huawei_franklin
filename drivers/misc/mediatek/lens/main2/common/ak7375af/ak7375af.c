/*
 * ak7375af.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * ak7375 af driver
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

#define AF_DRVNAME "AK7375AF_DRV"
#define AF_I2C_SLAVE_ADDR       (0x18 >> 1)
#define VCM_SET_POS_REG_H       0x00
#define VCM_SET_POS_REG_L       0x01
#define VCM_MODE_SETTING        0x02
#define VCM_CHIPID_REG          0x03
#define VCM_INIT_CODE_H         0x32
#define VCM_INIT_CODE_L         0xC0
#define VCM_REAL_POS_REG_H      0x84
#define VCM_REAL_POS_REG_L      0x85
#define VCM_WAKE_UP             0xF
#define DEST_CODE_MAX           1023
#define DELAY_TIME              200
#define VCM_CHIPID_MASK         0x1F
#define AK7375_MODE_SET_REG     0x02
#define AK7375_STANDBY_SET_VAL  0x40
#define AK7375_ACTIVE_SET_VAL   0x0
#define VCM_SET_POS_REG_OFFSET         4
#define VCM_SET_POS_REG_L_SET_BIT      0xF
#define VCM_SET_POS_REG_DELAY          2
#define VCM_POS_REG_OFFSET      4
#define VCM_SET_BIT             0xF
#define VCM_POS_SHIFT_2         2

#define VCM_CLOSE_STATUS         0
#define VCM_INIT_STATUS          1
#define VCM_OPEN_STATUS          2
#define WRITE_SIZE               2
#define READ_SIZE                1

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023; /* af macro code */
static unsigned long g_current_position;
static const char *g_ic_name = "AK7375";
static const char *g_module_name = "AK";

static int af_read_reg(u8 addr, u8 *result)
{
	int ret = 0;
	char buff = 0;
	char send_data[READ_SIZE] = { addr };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	ret = i2c_master_send(g_af_i2c_client, send_data, READ_SIZE);
	if (ret < 0) {
		log_err("I2C read - send failed\n");
		return -1;
	}

	ret = i2c_master_recv(g_af_i2c_client, &buff, READ_SIZE);
	if (ret < 0) {
		log_err("I2C read - recv failed\n");
		return -1;
	}
	*result = buff;

	return 0;
}

static int af_write_reg(u8 addr, u8 data)
{
	int ret = 0;
	char send_data[2] = { addr, data };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	ret = i2c_master_send(g_af_i2c_client, send_data, WRITE_SIZE);
	if (ret < 0) {
		log_err("I2C write failed\n");
		return -1;
	}

	return 0;
}

static unsigned int conv_code(unsigned int code)
{
	/* code value (0 ~ 1023) to (0 ~ 4096) */
	if (code > DEST_CODE_MAX)
		return (DEST_CODE_MAX << 2);
	else
		return (code << 2);
}

static inline int get_af_info(__user struct stAF_MotorInfo *p_motor_info)
{
	struct stAF_MotorInfo motor_info;

	return_error_on_null(p_motor_info);
	motor_info.u4MacroPosition = g_af_macro;
	motor_info.u4InfPosition = g_af_inf;
	motor_info.u4CurrentPosition = g_current_position;
	motor_info.bIsSupportSR = 1; /* support */
	motor_info.bIsMotorMoving = 1; /* moving */

	if (*g_af_opened >= VCM_INIT_STATUS)
		motor_info.bIsMotorOpen = 1; /* open */
	else
		motor_info.bIsMotorOpen = 0; /* close */

	if (copy_to_user(p_motor_info, &motor_info,
			 sizeof(struct stAF_MotorInfo)))
		log_info("copy to user fail when getting motor information\n");

	return 0;
}

/* init AF include driver initialization and standby mode */
static int init_af(void)
{
	unsigned char data = 0;
	const struct i2c_cfg_t ak7375_i2c_cfg[] = {
		{
			.setting = { VCM_SET_POS_REG_H, VCM_INIT_CODE_H }, /* addr, value */
			.delay = 0,
		}, {
			.setting = { VCM_SET_POS_REG_L, VCM_INIT_CODE_L },
			.delay = 1,
		}, {
			/* 0:active mode */
			.setting = { VCM_MODE_SETTING, 0 },
			.delay = 0,
		},
	};

	log_info("+\n");
	if (*g_af_opened == VCM_INIT_STATUS) {
		/* Device IC check */
		loge_if_ret(af_read_reg(VCM_CHIPID_REG, &data));
		data = data & VCM_CHIPID_MASK;
		if (data != VCM_WAKE_UP) {
			log_info("ak7375 check ic error data = %u", data);
			return -1;
		}
		/* start ak7375 */
		g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
		if (lens_set_setting(g_af_i2c_client,
			ak7375_i2c_cfg, array_size(ak7375_i2c_cfg)) == -EIO) {
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
	unsigned short af_code;
	unsigned short af_conv_code;

	af_code = (unsigned short)(position & 0xFFFF);
	af_conv_code = conv_code(af_code);
	log_info("dest_code = %u, conv_code = %u", position, af_conv_code);

	loge_if_ret(af_write_reg(VCM_SET_POS_REG_H,
		(af_conv_code >> VCM_SET_POS_REG_OFFSET)));
	loge_if_ret(af_write_reg(VCM_SET_POS_REG_L,
		((af_conv_code & VCM_SET_POS_REG_L_SET_BIT) <<
			VCM_SET_POS_REG_OFFSET)));

	return 0;
}

/* moveAF only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	if (position > DEST_CODE_MAX) {
		log_info("position %lu is invalid, should be 0~1023", position);
		return -1;
	}
	log_info("position = %lu", position);

	if (set_vcm_pos(position) != 0) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_MOVE_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("set I2C failed when moving the motor\n");
		return -1;
	}

	g_current_position = position;
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

long ak7375afa_ioctl(struct file *a_file, unsigned int a_command,
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

int ak7375afa_release(struct inode *a_inode, struct file *a_file)
{
	log_info("Start\n");

	if (*g_af_opened == VCM_OPEN_STATUS) {
		log_info("Wait\n");
		/* add release code */
		af_write_reg(VCM_MODE_SETTING, 0x01); /* 1:sleep mode */
	}

	if (*g_af_opened) {
		log_info("Free\n");
		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("End\n");

	return 0;
}

int ak7375afa_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spin_lock = af_spin_lock;
	g_af_opened = af_opened;

	return init_af();
}

int ak7375afa_get_file_name(unsigned char *file_name)
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
