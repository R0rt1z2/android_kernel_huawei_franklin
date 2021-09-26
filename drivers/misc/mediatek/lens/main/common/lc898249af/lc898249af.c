/*
 * lc898249af.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * lc898249 AF driver
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

#define AF_DRVNAME                 "LC898249AF_DRV"
#define AF_I2C_SLAVE_ADDR          (0xE4 >> 1)

#define LC898249_CHIP_ID           0xA5
#define LC898249_CHIP_ID_REG       0xF0
/* download */
#define LC898249_DWN_REG           0xE0
#define LC898249_DWN_START         0x1
#define LC898249_DWN_WAKE_UP       0x0
#define LC898249_DWN_WAKE_UP_REG   0xB3
#define LC898249_DWN_WAKE_UP_MASK  0xE0
#define LC898249_DWN_WAIT_MS       0x1
#define LC898249_DWN_TRY_TIMES     100
/* set code */
#define LC898249_CODE_MAX          1023
#define LC898249_CODE_MIN          0
#define LC898249_CODE_H_REG        0x84
#define LC898249_CODE_L_REG        0x85
/* hall */
#define LC898249_HALL_REG             0x0A
#define LC898249_HALL_OPEN_LOOP       0x8000
#define LC898249_HALL_INVAL           0
#define LC898249_HALL_CLOSE_LOOP_MASK 0x3FFF
#define LC898249_HALL_CLOSE_LOOP_MAX  0x3FF0
#define LC898249_HALL_CODE_MASK       0x03FF

/* scaling function */
#define SCALING_FUNCTION_OFF          0x0
#define SCALING_FUNCTION_MASK         0x3
#define LC898249_SCALING_FUNCTION_REG 0x81
#define LC898249_I2C_STATUS_REG       0xF0


/* vcm standby mode */
#define LC898249_MODE_SET_REG          0x96
#define LC898249_STANDBY_SET_VAL       0x80
#define LC898249_STANDBY_READ_REG      0x81
#define LC898249_STANDBY_READ_VAL      0x80
#define LC898249_WAKEUP_SET_VAL        0x1
#define LC898249_INIT_DOWNLOAD_REG     0xE0
#define LC898249_INIT_DOWNLOAD_VAL     0x1
#define LC898249_WAKEUP_SUCC_REG       0xB3
#define LC898249_WAKEUP_SUCC_VAL       0xE0
#define VCM_INIT_DELAY                 0x6
#define VCM_READ_DELAY                 0x1
#define VCM_COUNT_TRY_TIMES            20
#define LC898249_CODE_MID              650
#define LC898249_POSITIVE_INTERVAL     150
#define LC898249_NEGATIVE_INTERVAL     0xFF69
#define LC898249_SET_POS_REG_DELAY     2
#define LC898249_REAL_CODE_TRY_TIMES   10
#define LC898249_COMPARE_REG           0x0C

#define VCM_CLOSE_STATUS               0
#define VCM_INIT_STATUS                1
#define VCM_OPEN_STATUS                2
#define REG_ADDR_BYTE                  1
#define REG_VALUE_BYTE                 1
#define REG_VALUE_WORD                 2
#define SHIFT_8BIT                     8

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023;
static unsigned long g_af_current_position;
static const char *g_ic_name = "LC898249";
static const char *g_module_name = "LC";

static int af_read_reg(u8 addr, u8 *data)
{
	u8 value[REG_VALUE_BYTE];
	u8 send_cmd[REG_ADDR_BYTE] = { addr };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		send_cmd, REG_ADDR_BYTE));
	loge_if_ret(i2c_master_recv(g_af_i2c_client,
		value, REG_VALUE_BYTE));
	*data = value[0];

	return 0;
}

static int af_write_reg(u8 addr, u8 data)
{
	u8 size = REG_ADDR_BYTE + REG_VALUE_BYTE;
	u8 send_cmd[REG_ADDR_BYTE + REG_VALUE_BYTE] = { addr, data };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_cmd, size));

	return 0;
}

static int af_write_reg_16bit_data(u8 addr, u16 data)
{
	u8 size = REG_ADDR_BYTE + REG_VALUE_WORD;
	u8 send_cmd[REG_ADDR_BYTE + REG_VALUE_WORD] = {
		addr, (u8)(data >> SHIFT_8BIT), (u8)(data & 0xFF) };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client, send_cmd, size));

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
	unsigned char data = 0;
	unsigned short cnt = 0;
	unsigned char scaling_func = SCALING_FUNCTION_OFF;

	log_info("+\n");

	/* Driver IC check */
	loge_if_ret(af_read_reg(LC898249_CHIP_ID_REG, &data));
	if (data != LC898249_CHIP_ID) {
		log_err("lc898249 check error data = 0x%x", data);
		return -1;
	}

	/* Init data download */
	if (af_write_reg(LC898249_DWN_REG, LC898249_DWN_START) != 0) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("i2c send fail\n");
		return -1;
	}
	do {
		mdelay(LC898249_DWN_WAIT_MS);
		loge_if_ret(af_read_reg(LC898249_DWN_WAKE_UP_REG, &data));
	} while ((data & LC898249_DWN_WAKE_UP_MASK) != LC898249_DWN_WAKE_UP &&
		cnt++ < LC898249_DWN_TRY_TIMES);

	if (cnt > LC898249_DWN_TRY_TIMES) {
		log_err("lc898249 download failed");
		return -1;
	}

	/*
	 * Bugfix:
	 * MTM use 16bit code setting,but cablication in OTP use 10bit setting
	 * if that, set 0x81h[1:0] = 0x3 as scaling function
	 * ON(Hall max &Hall min);
	 */
	loge_if_ret(af_read_reg(LC898249_SCALING_FUNCTION_REG, &scaling_func));
	log_info("E2PROM scaling function = 0x%x", scaling_func);
	if ((scaling_func & SCALING_FUNCTION_MASK) == SCALING_FUNCTION_OFF) {
		scaling_func |= SCALING_FUNCTION_MASK;
		if (af_write_reg(LC898249_SCALING_FUNCTION_REG, scaling_func) != 0) {
			lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
				g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
			log_err("i2c send fail\n");
			return -1;
		}
	}

	log_info("-\n");
	return 0;
}

/* moveAF only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	unsigned short dest_code;

	if (position > LC898249_CODE_MAX) {
		log_err("position %u is invalid, should be 0~1023", position);
		return -1;
	}
	dest_code = (unsigned short)(position & 0xFFFF); /* 16bit valid */

	log_info("dest_code = %u", position);

	if (af_write_reg_16bit_data(LC898249_CODE_H_REG, dest_code) != 0) {
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

long lc898249af_ioctl(struct file *a_file, unsigned int a_command,
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

int lc898249af_release(struct inode *a_inode, struct file *a_file)
{
	log_info("Start\n");

	if (*g_af_opened) {
		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("End\n");

	return 0;
}

int lc898249af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spin_lock = af_spin_lock;
	g_af_opened = af_opened;

	return init_af();
}

int lc898249af_get_file_name(unsigned char *file_name)
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
