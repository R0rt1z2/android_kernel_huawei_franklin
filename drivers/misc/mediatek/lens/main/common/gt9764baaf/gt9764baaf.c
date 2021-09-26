/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: CN3938AF voice coil motor driver
 * Author: mazhaung
 * Create: 2020-08-10
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <securec.h>
#include "lens_info.h"
#define AF_DRVNAME "CN3938AF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18
#define AF_DEBUG
#ifdef AF_DEBUG
#define log_inf(format, args...) \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#define log_error(format, args...) \
	pr_err(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define log_inf(format, args...)
#define log_error(format, args...)
#endif

static struct i2c_client *af_i2c_client;
static int *af_opened;
static spinlock_t *af_spinlock;
static unsigned long af_inf;
static unsigned long af_macro = 1023;
static unsigned long cur_position;
static const char *ic_name = "GT9764BA";
static const char *module_name = "GT";
#define MIN_POS 0
#define MAX_POS 1023
#define AF_INIT_POS 512
static int af_write_reg(u16 a_u2data)
{
	int ret = 0;
	char cmd1[2] = { 0x03, ((a_u2data >> 8) & 0x3) };
	char cmd2[2] = { 0x04, (a_u2data & 0xFF) };

	return_error_on_null(af_i2c_client);
	af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	af_i2c_client->addr = af_i2c_client->addr >> 1;
	ret = i2c_master_send(af_i2c_client, cmd1, 2);
	if (ret < 0) {
		log_inf("I2C send cmd1 failed!!\n");
		return -1;
	}
	ret = i2c_master_send(af_i2c_client, cmd2, 2);
	if (ret < 0) {
		log_inf("I2C send cmd2 failed!!\n");
		return -1;
	}
	return 0;
}

static inline int get_af_info(__user struct stAF_MotorInfo *pmotor_info)
{
	struct stAF_MotorInfo motor_info;
	return_error_on_null(pmotor_info);
	return_error_on_null(af_opened);
	motor_info.u4MacroPosition = af_macro;
	motor_info.u4InfPosition = af_inf;
	motor_info.u4CurrentPosition = cur_position;
	motor_info.bIsSupportSR = 1;
	motor_info.bIsMotorMoving = 1;
	if (*af_opened >= 1)
		motor_info.bIsMotorOpen = 1;
	else
		motor_info.bIsMotorOpen = 0;
	if (copy_to_user(pmotor_info, &motor_info,
		sizeof(struct stAF_MotorInfo)))
		log_error("copy to user failed when getting motor information\n");
	return 0;
}

static int init_af(void)
{
	/* register sequence table */
	const struct i2c_cfg_t i2c_cfg[] = {
		{
			.setting = { 0x02, 0x02 }, /* addr, value */
			.delay = 0,
		}, {
			.setting = { 0x06, 0x40 },
			.delay = 0,
		}, {
			.setting = { 0x07, 0x0b },
			.delay = 0,
		}
	};
	if (*af_opened == 1) {
		spin_lock(af_spinlock);
		*af_opened = 2;
		spin_unlock(af_spinlock);
	}
	if (lens_set_setting(af_i2c_client,
		i2c_cfg, array_size(i2c_cfg)) == -EIO) {
		lens_hiview_handle(DSM_CAMERA_ACTUATOR_INIT_FAIL,
			ic_name, module_name, VCM_WRITE_I2C_FAIL);
		log_error("i2c send fail\n");
		return -1;
	}
	log_inf("exit\n");
	return 0;
}

static inline int move_af(unsigned long a_u4position)
{
	if (af_write_reg((unsigned short)a_u4position) == 0) {
		cur_position = a_u4position;
		return 0;
	}
	lens_hiview_handle(DSM_CAMERA_ACTUATOR_MOVE_FAIL,
		ic_name, module_name, VCM_WRITE_I2C_FAIL);
	log_error("set I2c failed when moving the motor\n");
	return -1;
}

static inline int set_af_inf(unsigned long a_u4position)
{
	spin_lock(af_spinlock);
	af_inf = a_u4position;
	spin_unlock(af_spinlock);
	return 0;
}

static inline int set_af_macro(unsigned long a_u4position)
{
	spin_lock(af_spinlock);
	af_macro = a_u4position;
	spin_unlock(af_spinlock);
	return 0;
}

long gt9764baaf_ioctl(struct file *af_file, unsigned int command,
	unsigned long param)
{
	long ret_value = 0;
	return_error_on_null(af_file);
	switch (command) {
	case AFIOC_G_MOTORINFO:
		ret_value = get_af_info((__user struct stAF_MotorInfo *)(param));
		break;
	case AFIOC_T_MOVETO:
		ret_value = move_af(param);
		break;
	case AFIOC_T_SETINFPOS:
		ret_value = set_af_inf(param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret_value = set_af_macro(param);
		break;
	default:
		log_inf("No CMD\n");
		ret_value = -EPERM;
		break;
	}
	return ret_value;
}

int gt9764baaf_release(struct inode *af_inode, struct file *af_file)
{
	log_inf("Start\n");
	return_error_on_null(af_opened);
	if (*af_opened == 2) {
		log_inf("Wait\n");
		af_write_reg(AF_INIT_POS);
		msleep(12);
	}
	if (*af_opened) {
		log_inf("Free\n");

		spin_lock(af_spinlock);
		*af_opened = 0;
		spin_unlock(af_spinlock);
	}
	log_inf("End\n");
	return 0;
}

int gt9764baaf_set_i2c_client(struct i2c_client *i2c_client,
	spinlock_t *spinlock, int *opened)
{
	return_error_on_null(i2c_client);
	return_error_on_null(spinlock);
	return_error_on_null(opened);
	af_i2c_client = i2c_client;
	af_spinlock = spinlock;
	af_opened = opened;
	init_af();
	return 1;
}

int gt9764baaf_get_file_name(unsigned char *af_file_name)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[256];
	char *file_string = NULL;
	errno_t rc;
	return_error_on_null(af_file_name);
	rc = sprintf_s(file_path, sizeof(file_path), "%s", __FILE__);
	if (rc < 0)
		log_error("sprintf_s fail");
	file_string = strrchr(file_path, '/');
	if (file_string == NULL) {
		log_error("file_string NULL!");
		return -1;
	}
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	rc = strncpy_s(af_file_name, STRUCT_MOTOR_NAME, file_string, AF_MOTOR_NAME);
	if (rc != EOK)
		log_inf("strncpy_s fail, rc = %d\n", rc);
	log_inf("FileName : %s\n", af_file_name);
	#else
	af_file_name[0] = '\0';
	#endif
	return 1;
}
