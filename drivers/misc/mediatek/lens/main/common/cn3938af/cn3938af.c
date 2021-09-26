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
#define LOG_INF(format, args...) \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#define LOG_ERR(format, args...) \
	pr_err(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#define LOG_ERR(format, args...)
#endif

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spinlock;
static unsigned long g_af_inf;
static unsigned long g_af_macro = 1023;
static unsigned long g_target_position;
static unsigned long g_cur_position;

static int i2c_read(u8 a_u2addr, u8 *a_pubuff)
{
	char pureadcmd[1] = { (char)(a_u2addr) };
	int ret_value = i2c_master_send(g_af_i2c_client, pureadcmd, 1);
	if (ret_value != 2) {
		LOG_ERR("I2C write failed!!\n");
		return -1;
	}
	ret_value = i2c_master_recv(g_af_i2c_client, (char *)a_pubuff, 1);
	if (ret_value != 1) {
		LOG_ERR("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;
	i2c_read(addr, &get_byte);
	return get_byte;
}

static int cn3938af_read_reg(unsigned short *a_pu2result)
{
	*a_pu2result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);
	return 0;
}

static int af_write_reg(u16 a_u2data)
{
	int ret_value;
	char pusendcmd[3] = { 0x03, (char)(a_u2data >> 8),
		(char)(a_u2data & 0xFF) };
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;
	ret_value = i2c_master_send(g_af_i2c_client, pusendcmd, 3);
	if (ret_value < 0) {
		LOG_ERR("I2C send failed!\n");
		return -1;
	}
	return 0;
}

static inline int get_af_info(__user struct stAF_MotorInfo *pmotor_info)
{
	struct stAF_MotorInfo motor_info;
	motor_info.u4MacroPosition = g_af_macro;
	motor_info.u4InfPosition = g_af_inf;
	motor_info.u4CurrentPosition = g_cur_position;
	motor_info.bIsSupportSR = 1;
	motor_info.bIsMotorMoving = 1;
	if (*g_af_opened >= 1)
		motor_info.bIsMotorOpen = 1;
	else
		motor_info.bIsMotorOpen = 0;
	if (copy_to_user(pmotor_info, &motor_info,
		sizeof(struct stAF_MotorInfo)))
		LOG_ERR("copy to user failed when getting motor information\n");
	return 0;
}

static int init_drv(void)
{
	int ret_value = 0;
	char pusendcmdarray[7][2] = {    // send cmd array
	{ 0x02, 0x01 }, { 0x02, 0x00 }, { 0xFE, 0xFE },
	{ 0x02, 0x02 }, { 0x06, 0x40 }, { 0x07, 0x60 }, { 0xFE, 0xFE },
	};
	unsigned char cmd_number;
	LOG_INF("InitDrv[1] %p, %p\n", &(pusendcmdarray[1][0]), pusendcmdarray[1]);
	LOG_INF("InitDrv[2] %p, %p\n", &(pusendcmdarray[2][0]), pusendcmdarray[2]);
	for (cmd_number = 0; cmd_number < 7; cmd_number++) {
		if (pusendcmdarray[cmd_number][0] != 0xFE) {
			ret_value = i2c_master_send(g_af_i2c_client, pusendcmdarray[cmd_number], 2);
			if (ret_value < 0)
				return -1;
		} else {
			udelay(100);
		}
	}
	return ret_value;
}

static inline int move_af(unsigned long a_u4position)
{
	int ret = 0;
	if ((a_u4position > g_af_macro) || (a_u4position < g_af_inf)) {
		LOG_ERR("out of range\n");
		return -EINVAL;
	}
	if (*g_af_opened == 1) {
		unsigned short initpos;
		init_drv();
		ret = cn3938af_read_reg(&initpos);
		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", initpos);
			spin_lock(g_af_spinlock);
			g_cur_position = (unsigned long)initpos;
			spin_unlock(g_af_spinlock);
		} else {
			spin_lock(g_af_spinlock);
			g_cur_position = 0;
			spin_unlock(g_af_spinlock);
		}
		spin_lock(g_af_spinlock);
		*g_af_opened = 2;
		spin_unlock(g_af_spinlock);
	}
	if (g_cur_position == a_u4position)
		return 0;
	spin_lock(g_af_spinlock);
	g_target_position = a_u4position;
	spin_unlock(g_af_spinlock);
	if (af_write_reg((unsigned short)g_target_position) == 0) {
		spin_lock(g_af_spinlock);
		g_cur_position = (unsigned long)g_target_position;
		spin_unlock(g_af_spinlock);
	} else {
		LOG_ERR("set I2C failed when moving the motor\n");
		ret = -1;
	}
	return ret;
}

static inline int set_af_inf(unsigned long a_u4position)
{
	spin_lock(g_af_spinlock);
	g_af_inf = a_u4position;
	spin_unlock(g_af_spinlock);
	return 0;
}

static inline int set_af_macro(unsigned long a_u4position)
{
	spin_lock(g_af_spinlock);
	g_af_macro = a_u4position;
	spin_unlock(g_af_spinlock);
	return 0;
}

long cn3938af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param)
{
	long ret_value = 0;
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
		LOG_INF("No CMD\n");
		ret_value = -EPERM;
		break;
	}
	return ret_value;
}

int cn3938af_release(struct inode *af_inode, struct file *af_file)
{
	LOG_INF("Start\n");
	if (*g_af_opened == 2)
		LOG_INF("Wait\n");
	if (*g_af_opened) {
		LOG_INF("Free\n");
		spin_lock(g_af_spinlock);
		*g_af_opened = 0;
		spin_unlock(g_af_spinlock);
	}
	LOG_INF("End\n");
	return 0;
}

int cn3938af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened)
{
	g_af_i2c_client = af_i2c_client;
	g_af_spinlock = af_spinlock;
	g_af_opened = af_opened;
	return 1;
}

int cn3938af_get_file_name(unsigned char *af_file_name)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char file_path[256];
	char *file_string = NULL;
	errno_t rc;
	rc = sprintf_s(file_path, sizeof(file_path), "%s", __FILE__);
	if (rc < 0)
		LOG_ERR("sprintf_s fail");
	file_string = strrchr(file_path, '/');
	if (file_string == NULL) {
		LOG_ERR("file_string NULL!");
		return -1;
	}
	*file_string = '\0';
	file_string = (strrchr(file_path, '/') + 1);
	rc = strncpy_s(af_file_name, STRUCT_MOTOR_NAME, file_string, AF_MOTOR_NAME);
	if (rc != EOK)
		LOG_INF("strncpy_s fail, rc = %d\n", rc);
	LOG_INF("FileName : %s\n", af_file_name);
	#else
	af_file_name[0] = '\0';
	#endif
	return 1;
}
