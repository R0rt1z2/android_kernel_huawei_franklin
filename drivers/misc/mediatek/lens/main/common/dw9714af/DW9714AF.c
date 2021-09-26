/*
 * Copyright (C) 2015 MediaTek Inc.
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

/*
 * DW9714AF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"
#include "dw9714af.h"

#define AF_DRVNAME "DW9714AF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define LOG_ERR(format, args...)                                               \
	pr_err(AF_DRVNAME " [%s] " format, __func__, ##args)

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4CurrPosition;

static struct af_i2c_reg af_reset_setting[] = {
	{ 0x80, 0x00, 0x00 },
	{ 0x00, 0x01, 0x01 },
};

static struct af_i2c_reg af_init_setting[] = {
	{ 0xED, 0xAB, 0x00 },
	{ 0x02, 0x01, 0x00 },
	{ 0x02, 0x00, 0x01 },
	{ 0x06, 0x84, 0x00 },
	{ 0x07, 0x01, 0x00 },
	{ 0x08, 0x4B, 0x00 },
};

static struct af_i2c_reg power_off_setting[] = {
	{ 0x02, 0x01, 0x00 },
};

static struct af_move_reg af_exit_setting[] = {
	{ 300, 12 },
	{ 180, 12 },
	{ 80, 12 },
};

static int s4AF_ReadReg(u16 addr, u16 *data)
{
	u8 u8data=0;
	u8 pu_send_cmd[2] = {(u8)(addr & 0xFF),(u8)(addr >> 8)};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	if (i2c_master_send(g_pstAF_I2Cclient, pu_send_cmd, 1) < 0) {
		LOG_INF("read I2C send failed!!\n");
		return -1;
	}
	if (i2c_master_recv(g_pstAF_I2Cclient, &u8data, 1) < 0) {
		LOG_INF("AF_ReadReg failed!!\n");
		return -1;
	}
	*data = u8data;
	LOG_INF("actuator 0x%x, 0x%x\n", addr, *data);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;
	// 0x03 is VCM MSB addr, 0x04 is VCM LSB addr
	char puSendCmd1[2] = { 0x03, ((a_u2Data >> 8) & 0x3) };
	char puSendCmd2[2] = { 0x04, (a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static int32_t af_write_i2c_table(
		struct af_i2c_reg *setting, int32_t size)
{
	int32_t  i = 0;
	char puSendCmd[2] = {0};
	unsigned short addr = 0;
	unsigned short data = 0;
	unsigned short delay = 0;

	if (setting == NULL) {
		LOG_ERR("there are some wrong of the setting!\n");
		return -1;
	}
	for (i = 0; i < size; i++) {
		addr = setting[i].addr;
		data = setting[i].data;
		delay = setting[i].delay;
		puSendCmd[0] = (int8_t)addr;
		puSendCmd[1] = (int8_t)data;
		g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
		g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2) < 0) {
			LOG_ERR("I2C send failed!!\n");
			return -1;
		};
		LOG_INF("puSendCmd0=0x%2x,puSendCmd1=0x%2x\n",
			puSendCmd[0],puSendCmd[1]);
		if (delay > 0) {
			mdelay(delay);
		}
	}
	LOG_INF("Exit. size:%d\n", size);
	return 0;
}

static int32_t af_write_move_table(
		struct af_move_reg *setting, int32_t size)
{
	int32_t  i = 0;
	unsigned short  data = 0;
	unsigned short  delay = 0;

	if(setting == NULL){
		LOG_ERR("there are some wrong of exit setting!\n");
		return -1;
	}
	for( i = 0; i < size; i++ ) {
		data = setting[i].data;
		delay = setting[i].delay;
		if (s4AF_WriteReg(data) == 0) {
			if (delay > 0) {
				mdelay(delay);
			}
		}
		else {
			LOG_ERR("s4AF_WriteReg failed!!\n");
			return -1;
		}
	}
	LOG_INF("Exit. size:%d\n", size);
	return 0;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int initAF(void)
{
	int i4RetValue = 0;
	int table_size = 0;

	LOG_INF("+\n");

	if (*g_pAF_Opened == 1) {
		table_size = sizeof(af_init_setting) / sizeof(struct af_i2c_reg);
		i4RetValue = af_write_i2c_table(af_init_setting, table_size);
		if (i4RetValue < 0) {
			LOG_ERR("I2C init failed!!\n");
			return -1;
		}
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("-\n");

	return 0;
}

/* moveAF only use to control moving the motor */
static int move_af_times = 0;

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	int table_size = 0;
	unsigned short Reg_0x05 = 0;
	unsigned short Reg_0x00 = 0;
	unsigned short Reg_0x06 = 0;

	LOG_INF("Start\n");

	if (*g_pAF_Opened == 1) {
		if (initAF() < 0)
			LOG_ERR("I2C init failed!!\n");
	}
	move_af_times++;
	if ((move_af_times % AF_REINIT_ESD_CHECK) == 0) {
		s4AF_ReadReg(DW9714V_STATUS_REG, &Reg_0x05);
		s4AF_ReadReg(DW9714V_VCM_CFG_REG, &Reg_0x06);
		s4AF_ReadReg(DW9714V_VCM_IC_INFO_REG, &Reg_0x00);
		if (((Reg_0x05 & DW9714V_BUSY_STATE_MASK) != 0) ||
			(Reg_0x00 != DW9714V_VCM_IC_INFO_DEFAULT) ||
			(Reg_0x06 == 0)) {
			LOG_ERR("AF_REINIT_ESD: Reg_0x05=0x%x\n", Reg_0x05);
			LOG_ERR("AF_REINIT_ESD: Reg_0x06=0x%x\n", Reg_0x06);
			LOG_ERR("AF_REINIT_ESD: Reg_0x00=0x%x\n", Reg_0x00);
			table_size = sizeof(af_reset_setting) /
					sizeof(struct af_i2c_reg);
			if (af_write_i2c_table(af_reset_setting, table_size) < 0)
				LOG_ERR("AF reset  failed!!\n");
			spin_lock(g_pAF_SpinLock);
			*g_pAF_Opened = 1;
			spin_unlock(g_pAF_SpinLock);
			if (initAF() < 0)
				LOG_ERR("I2C init failed!!\n");
		}
		move_af_times = 0;
	}

	if (s4AF_WriteReg((unsigned short)a_u4Position) == 0) {
		g_u4CurrPosition = a_u4Position;
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	LOG_INF("End\n");

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		    unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
			getAFInfo((__user struct stAF_MotorInfo *)(a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	int table_size = 0;;

	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		// Move AF in three steps to reduce abnormal noise
		LOG_INF("AF_CurrPosition is %d\n", g_u4CurrPosition);
		if (g_u4CurrPosition) {
			table_size = sizeof(af_exit_setting) /
					sizeof(struct af_move_reg);
			if (af_write_move_table(af_exit_setting, table_size) < 0)
				LOG_ERR("AF exit failed!\n");
		}
		table_size = sizeof(power_off_setting)/sizeof(struct af_i2c_reg);
		if (af_write_i2c_table(power_off_setting, table_size) < 0)
			LOG_ERR("AF power_off  failed!!\n");
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			  spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initAF();

	return 1;
}

int DW9714AF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	if (FileString == NULL) {
		LOG_ERR("FileString NULL!");
		return -1;
	}
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
