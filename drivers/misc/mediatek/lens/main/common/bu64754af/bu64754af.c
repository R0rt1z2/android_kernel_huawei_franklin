/*
 * bu64754af.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * bu64754 AF driver
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

#define AF_DRVNAME                    "BU64754AF_DRV"
#define AF_I2C_SLAVE_ADDR              (0xEC >> 1)

/* set code */
#define BU64754_CODE_MAX              1023
#define BU64754_CODE_MIN              0
#define BU64754_CODE_H_REG            0x45
#define BU64754_CODE_L_REG            0x46

#define BU64754_POWER_ON_OFF_REG      0x07
#define BU64754_POWER_OFF             0x00
#define BU64754_POWER_ON              0x80
#define BU64754_CHIP_ID_REG           0x28
#define BU64754_CHIP_ID               0x4000
#define BU64754_CHECKSUM              0xF7
#define BU64754_POWER_COMMAND_ADDR    0x40
#define BU64754_POWER_COMMAND_VALUE   0x1111
#define BU64754_HALL_REG              0xF4
#define BU64754_HALL_OPEN_LOOP        0x8000
#define BU64754_HALL_OPEN_LOOP2       0xC010
#define BU64754_HALL_INVAL            0
#define BU64754_HALL_CLOSE_LOOP_MASK  0x3FFF
#define BU64754_HALL_CLOSE_LOOP_MAX   0x3FF0
#define BU64754_HALL_CODE_MASK        0x03FF
#define COEFF                         0.03128  /* 32/1023 */

/* nvm data rewrite */
#define BU64754_CHECKSUM_VALUE        0x04
#define BU64754_AF_NVM_SIZE           96
#define BU64754_TRY_TIMEOUT           0x2
#define BU64754_NVM_START_REG         0x52
#define BU64754_NVM_WRITE_ADDR_REG    0x50
#define BU64754_NVM_WRITE_DATA_REG    0x51
#define BU64754_NVM_START_ENABLE      0x8000
#define BU64754_NVM_START_DISABLE     0x00
#define BU64754_SHIFT_8BIT            8

/* vcm standby mode */
#define BU64754_ACTIVE_STANDBY_REG     0x07
#define BU64754_ACTIVE_VAL             0x0080
#define BU64754_STANDBY_VAL            0x0000
#define BU64754_SET_POS_REG_DELAY      2

#define BU64754_REAL_CODE_TRY_TIMES    10
#define BU64754_CODE_MID               650
#define BU64754_MIN_INTERVAL           500
#define BU64754_MAX_INTERVAL           750

#define VCM_CLOSE_STATUS               0
#define VCM_INIT_STATUS                1
#define VCM_OPEN_STATUS                2
#define REG_ADDR_BYTE                  1
#define REG_VALUE_WORD                 2
#define SHIFT_8BIT                     8

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spin_lock;

static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023;
static unsigned long g_af_current_position;
static const char *g_ic_name = "BU64754";
static const char *g_module_name = "BU";

static int af_read_reg(u8 addr, u16 *data)
{
	u8 value[REG_VALUE_WORD] = {0};
	u8 send_cmd[REG_ADDR_BYTE] = { addr };

	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	loge_if_ret(i2c_master_send(g_af_i2c_client,
		send_cmd, REG_ADDR_BYTE));
	loge_if_ret(i2c_master_recv(g_af_i2c_client,
		value, REG_VALUE_WORD));
	*data = value[0] << SHIFT_8BIT | value[1];

	return 0;
}

static int af_write_reg(u8 addr, u16 data)
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
	unsigned short ic_version = 0;
	unsigned short checksum_value = 0;

	log_info("+\n");

	/* ic check */
	loge_if_ret(af_read_reg(BU64754_CHIP_ID_REG, &ic_version));
	if (ic_version != BU64754_CHIP_ID) {
		log_err("BU64754 check ic version is 0x%x", ic_version);
		return -1;
	}

	mdelay(1); /* delay 1ms */

	/* read checksum value */
	loge_if_ret(af_read_reg(BU64754_CHECKSUM, &checksum_value));
	log_info("ic check succ, ic_version:0x%x,checksum_value:0x%x",
		ic_version, checksum_value);

	/* start command */
	if (af_write_reg(BU64754_POWER_COMMAND_ADDR, BU64754_POWER_COMMAND_VALUE) != 0) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
			g_ic_name, g_module_name, VCM_WRITE_I2C_FAIL);
		log_err("i2c send fail\n");
		return -1;
	}

	log_info("-\n");
	return 0;
}

/* moveAF only use to control moving the motor */
static inline int move_af(unsigned long position)
{
	unsigned short dest_code;

	if (position > BU64754_CODE_MAX) {
		log_err("position %u is invalid, should be 0~1023", position);
		return -1;
	}
	dest_code = (unsigned short)(position & 0xFFFF); /* 16bit valid */

	log_info("dest_code = %u", position);

	if (af_write_reg(BU64754_CODE_H_REG, dest_code) != 0) {
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

long bu64754af_ioctl(struct file *a_file, unsigned int a_command,
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

int bu64754af_release(struct inode *a_inode, struct file *a_file)
{
	log_info("Start\n");

	if (*g_af_opened) {
		log_info("vcm-bu64754 power off start");
		/* power off:reset ic */
		loge_if_ret(af_write_reg(BU64754_POWER_ON_OFF_REG,
			BU64754_POWER_OFF));

		spin_lock(g_af_spin_lock);
		*g_af_opened = VCM_CLOSE_STATUS;
		spin_unlock(g_af_spin_lock);
	}

	log_info("End\n");

	return 0;
}

int bu64754af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spin_lock = af_spin_lock;
	g_af_opened = af_opened;

	return init_af();
}

int bu64754af_get_file_name(unsigned char *file_name)
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
		log_err("file_string NULL");
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

