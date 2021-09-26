/*
 * cn3927af.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * Motor driver for cn3927af
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
#include "cn3927af.h"

#define AF_DRVNAME "CN3927AF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18
#define AF_REINIT_ESD_CHECK 10  /* Check times */
#define AF_CN3927AF_FILE_NAME "CN3927AF"
#define AF_DEBUG
#ifdef AF_DEBUG
#define log_inf(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define log_inf(format, args...)
#endif

static struct i2c_client *g_af_i2c_client;
static int *g_af_opened;
static spinlock_t *g_af_spinlock;

static unsigned long g_u4af_inf;
static unsigned long g_u4af_macro = 1023;
static unsigned long g_u4currposition;

static const struct af_i2c_reg af_reset_setting[] = {
	{ 0x80, 0x00, 0x00 }, /* reset */
	{ 0x00, 0x00, 0x01 }, /* close soft pd */
};

static const struct af_i2c_reg af_init_setting[] = {
	{ 0xEC, 0xA3, 0x00 },
	{ 0xA1, 0x14, 0x00 },
	{ 0xF2, 0x20, 0x00 },
	{ 0xDC, 0x51, 0x00 },
};

static const struct af_i2c_reg power_off_setting[] = {
	{ 0xEC, 0xA3, 0x00 },
	{ 0xA1, 0x14, 0x00 },
	{ 0xF2, 0x20, 0x00 },
	{ 0xDC, 0x51, 0x00 },
	{ 0x12, 0xC0, 0x0E },
	{ 0xEC, 0xA3, 0x00 },
	{ 0xA1, 0x00, 0x00 },
	{ 0xF2, 0x00, 0x00 },
	{ 0xDC, 0x51, 0x00 },
	{ 0x00, 0x06, 0x64 },
	{ 0x80, 0x00, 0x00 }, /* open soft pd */
};

static const struct af_i2c_reg esd_check_setting[] = {
	 /* {reg_addr, expect reg_value, deley} */
	{ 0xA1, 0x14, 0x00 },
	{ 0xF2, 0x20, 0x00 },
	{ 0x05, 0x00, 0x00 },
	{ 0x06, 0x00, 0x00 },
};

/* read Control Register */
static int s4af_readreg_directly(u16 *a_u2data)
{
	int i4retvalue;
	char pbuff[2] = {0};

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;
	i4retvalue = i2c_master_recv(g_af_i2c_client, pbuff, ARRAY_SIZE(pbuff));

	if (i4retvalue < 0) {
		log_inf("I2C read failed!!\n");
		return -1;
	}

	*a_u2data = ((u16)pbuff[0] << 8) + (u16)pbuff[1]; /* Control Register MSB + LSB */

	return 0;
}

/* write dac data */
static int s4af_writereg(u16 a_u2data)
{
	int i4retvalue;
	/* DAC DATA: Control Register MSB bit[0-3] + LSB bit[4-7] */
	char pusendcmd[2] = { (char)(a_u2data >> 4), (char)((a_u2data & 0xF) << 4) };

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;
	i4retvalue =
		i2c_master_send(g_af_i2c_client, pusendcmd, ARRAY_SIZE(pusendcmd));

	if (i4retvalue < 0) {
		log_inf("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static int s4af_readreg_cn3927af(u8 *a_psend_data, u16 a_size_send_data,
	u8 *a_precv_data, u16 a_size_recv_data)
{
	int i4retvalue;
	struct i2c_msg msg[2];

	return_error_on_null(g_af_i2c_client);
	spin_lock(g_af_spinlock);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR >> 1;
	spin_unlock(g_af_spinlock);

	msg[0].addr = g_af_i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = a_size_send_data;
	msg[0].buf = a_psend_data;

	msg[1].addr = g_af_i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = a_size_recv_data;
	msg[1].buf = a_precv_data;

	i4retvalue =
		i2c_transfer(g_af_i2c_client->adapter, msg, ARRAY_SIZE(msg));

	if (i4retvalue != ARRAY_SIZE(msg)) {
		log_inf("I2C Read failed!!\n");
		return -1;
	}

	return 0;
}

static int s4af_writereg_directly(u16 a_u2data)
{
	int i4retvalue;
	/* pusendcmd[0]:reg_addr, pusendcmd[1]:reg_value */
	char pusendcmd[2] = { (char)(a_u2data >> 8), (char)(a_u2data & 0xFF) };

	return_error_on_null(g_af_i2c_client);
	g_af_i2c_client->addr = AF_I2C_SLAVE_ADDR;
	g_af_i2c_client->addr = g_af_i2c_client->addr >> 1;
	i4retvalue =
		i2c_master_send(g_af_i2c_client, pusendcmd, ARRAY_SIZE(pusendcmd));

	if (i4retvalue < 0) {
		log_inf("I2C send failed!!\n");
		return -1;
	}
	return 0;
}

static int af_write_i2c_table(
		const struct af_i2c_reg *setting, int32_t size)
{
	u16 addr;
	u16 data;
	u16 delay;
	u16 data_value;
	int i;

	return_error_on_null(setting);
	for (i = 0; i < size; i++) {
		addr = setting[i].addr;
		data = setting[i].data;
		delay = setting[i].delay;
		data_value = (addr << 8) + data;
		if (s4af_writereg_directly(data_value) < 0) {
			log_inf("I2C send failed!!\n");
			return -1;
		}
		if (delay > 0)
			mdelay(delay);
	}
	log_inf("Exit. size:%d\n", size);
	return 0;
}
static int get_af_info(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo = {0};

	return_error_on_null(pstMotorInfo);
	stMotorInfo.u4MacroPosition = g_u4af_macro;
	stMotorInfo.u4InfPosition = g_u4af_inf;
	stMotorInfo.u4CurrentPosition = g_u4currposition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_af_opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		log_inf("copy to user failed when getting motor information\n");

	return 0;
}

/* initaf include driver initialization and standby mode */
static int initaf(void)
{
	log_inf("+\n");

	return_error_on_null(g_af_opened);
	if (*g_af_opened == 1) {
		if (af_write_i2c_table(af_reset_setting, ARRAY_SIZE(af_reset_setting)) < 0) {
			log_inf("I2C reset failed!!\n");
			return -1;
		}

		if (af_write_i2c_table(af_init_setting, ARRAY_SIZE(af_init_setting)) < 0) {
			log_inf("I2C init failed!!\n");
			return -1;
		}

		spin_lock(g_af_spinlock);
		*g_af_opened = 2;
		spin_unlock(g_af_spinlock);
	}

	log_inf("-\n");

	return 0;
}

static int esd_check(void)
{
	int i;
	u8 addr;
	u8 data;
	u16 delay;
	u8 reg_value = 0;
	u16 value = 0;

	s4af_readreg_directly(&value);
	if ((value >> 14) != 0) /* Control Register MSB bit[6-7] always 0 */
		return 1;
	for (i = 0; i < ARRAY_SIZE(esd_check_setting); i++) {
		addr = esd_check_setting[i].addr;
		data = esd_check_setting[i].data;
		delay = esd_check_setting[i].delay;

		if (s4af_readreg_cn3927af(&addr, 1, &reg_value, 1) < 0) {
			log_inf("I2C read failed!!\n");
			return -1;
		}
		if (delay > 0)
			mdelay(delay);
		if (reg_value != data) {
			log_inf("reg(0x%x) expct value:0x%x, real value:0x%x\n",
				addr, data, reg_value);
			return 1;
		}
	}
	return 0;
}
/* moveaf only use to control moving the motor */
static int moveaf(unsigned long a_u4position)
{
	int ret = 0;
	static int move_af_times = 0;

	if (*g_af_opened == 1) {
		if (initaf() < 0)
			log_inf("I2C init failed!!\n");
	}

	move_af_times++;
	if ((move_af_times % AF_REINIT_ESD_CHECK) == 0) {
		if (esd_check() == 1) {
			spin_lock(g_af_spinlock);
			*g_af_opened = 1;
			spin_unlock(g_af_spinlock);
			log_inf("esd error, need reinit!!");
			if (initaf() < 0)
				log_inf("I2C init failed!!\n");
		}
		move_af_times = 0;
	}

	if (s4af_writereg((u16)a_u4position) == 0) {
		g_u4currposition = a_u4position;
		ret = 0;
	} else {
		log_inf("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static int set_af_inf(unsigned long a_u4position)
{
	spin_lock(g_af_spinlock);
	g_u4af_inf = a_u4position;
	spin_unlock(g_af_spinlock);
	return 0;
}

static int set_af_macro(unsigned long a_u4position)
{
	spin_lock(g_af_spinlock);
	g_u4af_macro = a_u4position;
	spin_unlock(g_af_spinlock);
	return 0;
}

long cn3927af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param)
{
	long i4retvalue;

	switch (command) {
	case AFIOC_G_MOTORINFO:
		i4retvalue =
			get_af_info((__user struct stAF_MotorInfo *)(param));
		break;

	case AFIOC_T_MOVETO:
		i4retvalue = moveaf(param);
		break;

	case AFIOC_T_SETINFPOS:
		i4retvalue = set_af_inf(param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4retvalue = set_af_macro(param);
		break;

	default:
		log_inf("No CMD\n");
		i4retvalue = -EPERM;
		break;
	}

	return i4retvalue;
}

static int s4af_edlc_mode_power_off(void)
{
	if (af_write_i2c_table(power_off_setting, ARRAY_SIZE(power_off_setting)) < 0) {
		log_inf("power off failed!!\n");
		return -1;
	}

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int cn3927af_release(struct inode *af_inode, struct file *af_file)
{
	log_inf("Start\n");

	return_error_on_null(g_af_opened);
	if (*g_af_opened == 2) {
		log_inf("Wait\n");
		s4af_edlc_mode_power_off();
	}

	if (*g_af_opened) {
		log_inf("Free\n");
		spin_lock(g_af_spinlock);
		*g_af_opened = 0;
		spin_unlock(g_af_spinlock);
	}

	log_inf("End\n");

	return 0;
}

int cn3927af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened)
{
	return_error_on_null(af_i2c_client);
	return_error_on_null(af_spinlock);
	return_error_on_null(af_opened);
	g_af_i2c_client = af_i2c_client;
	g_af_spinlock = af_spinlock;
	g_af_opened = af_opened;

	initaf();

	return 1;
}

int cn3927af_get_file_name(unsigned char *af_file_name)
{
	int ret;

	return_error_on_null(af_file_name);
	ret = strncpy_s(af_file_name, AF_MOTOR_NAME + 1,
		AF_CN3927AF_FILE_NAME, AF_MOTOR_NAME);
	if (ret == EOK)
		log_inf("FileName:%s\n", af_file_name);
	else
		log_inf("strncpy_s fail");
	return 0;
}
