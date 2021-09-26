/*
 * dw9781baf.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * dw9781b af driver
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

#define AF_DRVNAME "DW9781BAF_DRV"
#define AF_I2C_SLAVE_ADDR  (0x54 >> 1)

#define DW9781B_CHIP_SECOND_ID         0x0020
#define DW9781B_CHIP_SECOND_ID_ADDRESS 0xD060
#define FW_VER_CURR                    0x7001
#define DW9781B_CODE_MAX               1023
#define DW9781B_CODE_MIN               0
#define VCM_SET_CODE_REG               0xD013
#define SHIFT_8BIT                     8
#define WRITE_SIZE                     4
#define READ_SIZE                      2

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023; /* af macro code */
static unsigned long g_af_current_position;
static const char *g_ic_name = "DW9781";
static const char *g_module_name = "DW";

static int af_read_reg(u16 addr, u16 *data)
{
	u8 value[READ_SIZE];
	u8 send_cmd[READ_SIZE] = { (u8)(addr >> SHIFT_8BIT),
				(u8)(addr & 0xFF) };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_cmd, READ_SIZE));
	loge_if_ret(i2c_master_recv(g_af_i2c_client, value, READ_SIZE));
	*data = value[0] << SHIFT_8BIT | value[1];

	return 0;
}

static int af_write_reg(u16 addr, u16 data)
{
	u8 send_cmd[WRITE_SIZE] = { (u8)(addr >> SHIFT_8BIT),
				(u8)(addr & 0xFF),
				(u8)(data >> SHIFT_8BIT),
				(u8)(data & 0xFF) };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_cmd, WRITE_SIZE));

	return 0;
}

static unsigned short conv_vcode(unsigned short code)
{
	/* code value (0 ~ 1023) to (0 ~ 2048) */
	if (code > DW9781B_CODE_MAX)
		return (DW9781B_CODE_MAX * 2);
	else
		return (code * 2);
}

static int vcm_reset(void)
{
	/* Logic reset */
	loge_if_ret(af_write_reg(0xD002, 0x0001));
	mdelay(4); /* reset delay 4ms */
	/* All protection 1 */
	loge_if_ret(af_write_reg(0xFAFA, 0x98AC));
	/* All protection 2 */
	loge_if_ret(af_write_reg(0xF053, 0x70BD));
	log_info("vcm Reset succ\n");
	return 0;
}

static unsigned char dw_ois_reset(void)
{
	log_info("dw9781b ois Reset start");
	loge_if_ret(af_write_reg(0xD002, 0x0001)); /* logic reset */
	mdelay(4); /* reset delay 4ms */
	loge_if_ret(af_write_reg(0xD001, 0x0001)); /* Active mode (DSP ON) */
	mdelay(25); /* ST gyro - over wait 25ms, default Servo On */
	loge_if_ret(af_write_reg(0xEBF1, 0x56FA)); /* User protect release */
	log_info("dw9781b ois reset finish");

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
	unsigned short chip_second_id = 0;
	unsigned short fw_version_current = 0;
	log_info("+\n");

	if (*g_af_opened == VCM_INIT_STATUS) {
		vcm_reset();
		loge_if_ret(af_read_reg(DW9781B_CHIP_SECOND_ID_ADDRESS,
			&chip_second_id));
		log_info("ver_value = 0x%x", chip_second_id);

		if (chip_second_id != DW9781B_CHIP_SECOND_ID) {
			log_err("ic_check error, chip_second_id = 0x%x",
				chip_second_id);
			return -1;
		}
		if (dw_ois_reset() < 0) {
			lens_hiview_handle(DSM_CAMERA_OIS_INIT_FAIL,
				g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
			log_err("ois i2c send fail\n");
			return -1;
		}
		loge_if_ret(af_read_reg(FW_VER_CURR, &fw_version_current));
		log_info("fw_version_current = 0x%x", fw_version_current);
		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_OPEN_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("-\n");
	return 0;
}

/* moveAF only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	unsigned short af_code;
	unsigned short af_conv_code;

	if (position > DW9781B_CODE_MAX) {
		log_err("position %u is invalid, should be 0~1023", position);
		return -1;
	}
	af_code = (unsigned short)(position & 0xFFFF);  /* 16bit valid */
	af_conv_code = conv_vcode(af_code);
	log_info("dest_code = %u, conv_code = %u", position, af_conv_code);

	if (af_write_reg(VCM_SET_CODE_REG, af_conv_code) != 0) {
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

long dw9781baf_ioctl(struct file *a_file, unsigned int a_command,
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

int dw9781baf_release(struct inode *a_inode, struct file *a_file)
{
	log_info("Start\n");

	if (*g_af_opened) {
		log_info("Free\n");
		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("End\n");

	return 0;
}

int dw9781baf_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spin_lock = af_spin_lock;
	g_af_opened = af_opened;

	return init_af();
}

int dw9781baf_get_file_name(unsigned char *file_name)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[256];  /* file name size */
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
