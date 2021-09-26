/*
 * CN3928AF.c
 * bring up for actuator
 * Copyright (C) Huawei Technology Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
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

#define AF_DRVNAME "CN3928AF_DRV"
#define AF_I2C_SLAVE_ADDR 0x18
#define AF_CN3928AF_FILE_NAME "CN3928AF"
#define AF_DEBUG
#ifdef AF_DEBUG
#define log_inf(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define log_inf(format, args...)
#endif

#define AF_INIT_POS 512

static struct i2c_client *g_pstaf_i2cclient;
static int *g_paf_opened;
static spinlock_t *g_paf_spinlock;


static unsigned long g_u4af_inf;
static unsigned long g_u4af_macro = 1023;
static unsigned long g_u4targetposition;
static unsigned long g_u4currposition;

static int g_SR = 3;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int  i4retvalue = 0;
	char puReadCmd[1] = {(char)(a_u2Addr)};

	i4retvalue = i2c_master_send(g_pstaf_i2cclient, puReadCmd, 1);
	log_inf("[CN3928AF]i2c_master_send i4retvalue - %x\n\n", i4retvalue);
	if (i4retvalue != 1) {

		log_inf("[CN3928AF] I2C write failed!!\n");
		return -1;
	}
	i4retvalue = i2c_master_recv(g_pstaf_i2cclient, (char *)a_puBuff, 1);
	log_inf("[CN3928AF]i2c_master_recv i4retvalue - %x\n\n", i4retvalue);
	if (i4retvalue != 1) {

		log_inf("[CN3928AF] I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;

	i2c_read(addr, &get_byte);

	log_inf("[CN3928AF] get_byte %d\n", get_byte);
	return get_byte;
}

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{

	g_pstaf_i2cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstaf_i2cclient->addr = g_pstaf_i2cclient->addr >> 1;

	*a_pu2Result = ((unsigned short)(read_data(0x03)&0x03) << 8) +
		(read_data(0x04)&0xff);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2data)
{
	int i4retvalue = 0;

	char pusendcmd[3] = {0x03, (char)(a_u2data >> 8),
		(char)(a_u2data & 0xff)};

	g_pstaf_i2cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstaf_i2cclient->addr = g_pstaf_i2cclient->addr >> 1;

	i4retvalue = i2c_master_send(g_pstaf_i2cclient, pusendcmd, 3);

	if (i4retvalue < 0) {
		log_inf("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static void s4AF_WriteReg_Directly(u16 data)
{
	int i4retvalue = 0;
	char pusendcmd[2] = { (char)(data >> 8), (char)(data & 0xFF) };

	g_pstaf_i2cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstaf_i2cclient->addr = g_pstaf_i2cclient->addr >> 1;
	i4retvalue = i2c_master_send(g_pstaf_i2cclient, pusendcmd, 2);
	if (i4retvalue < 0)
		log_inf("I2C send failed!!\n");
}

static void s4AF_edlc_mode()
{
	s4AF_WriteReg_Directly(0x0201); /* af dumpping setting */
	s4AF_WriteReg_Directly(0x0200);
	s4AF_WriteReg_Directly(0x0202);
	s4AF_WriteReg_Directly(0x0640);
	s4AF_WriteReg_Directly(0x0776);
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstmotorinfo)
{
	struct stAF_MotorInfo stmotorinfo;

	stmotorinfo.u4MacroPosition = g_u4af_macro;
	stmotorinfo.u4InfPosition = g_u4af_inf;
	stmotorinfo.u4CurrentPosition = g_u4currposition;
	stmotorinfo.bIsSupportSR = 1;

	stmotorinfo.bIsMotorMoving = 1;

	if (*g_paf_opened >= 1)
		stmotorinfo.bIsMotorOpen = 1;
	else
		stmotorinfo.bIsMotorOpen = 0;

	if (copy_to_user(pstmotorinfo,
		&stmotorinfo,
		sizeof(struct stAF_MotorInfo)))
		log_inf("copy to user failed when getting motor information\n");

	return 0;
}


static inline int moveAF(unsigned long a_u4position)
{
	int ret = 0;

	if ((a_u4position > g_u4af_macro) || (a_u4position < g_u4af_inf)) {
		log_inf("out of range\n");
		return -EINVAL;
	}

	if (*g_paf_opened == 1) {
		unsigned short initpos;

		spin_lock(g_paf_spinlock);
		*g_paf_opened = 2;
		spin_unlock(g_paf_spinlock);

		ret = s4AF_ReadReg(&initpos);

		if (ret == 0) {
			log_inf("Init Pos %6d\n", initpos);

			spin_lock(g_paf_spinlock);
			g_u4currposition = (unsigned long)initpos;
			spin_unlock(g_paf_spinlock);

		} else {
			spin_lock(g_paf_spinlock);
			g_u4currposition = 0;
			spin_unlock(g_paf_spinlock);
		}

		s4AF_edlc_mode();
	}

	if (g_u4currposition == a_u4position)
		return 0;

	spin_lock(g_paf_spinlock);
	g_u4targetposition = a_u4position;
	g_SR = 3;
	spin_unlock(g_paf_spinlock);

	if (s4AF_WriteReg((unsigned short)g_u4targetposition) == 0) {
		spin_lock(g_paf_spinlock);
		g_u4currposition = (unsigned long)g_u4targetposition;
		spin_unlock(g_paf_spinlock);
		log_inf("[zp],cn3928af moveaf TargetPosition = %ld\n",
			g_u4targetposition);
	} else {
		log_inf("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4position)
{
	spin_lock(g_paf_spinlock);
	g_u4af_inf = a_u4position;
	spin_unlock(g_paf_spinlock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4position)
{
	spin_lock(g_paf_spinlock);
	g_u4af_macro = a_u4position;
	spin_unlock(g_paf_spinlock);
	return 0;
}

long CN3928AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
	unsigned long a_u4Param)
{
	long i4retvalue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4retvalue = getAFInfo(
			(__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4retvalue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4retvalue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4retvalue = setAFMacro(a_u4Param);
		break;

	default:
		log_inf("No CMD\n");
		i4retvalue = -EPERM;
		break;
	}

	return i4retvalue;
}

int CN3928AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	log_inf("Start current pos: %d\n", g_u4currposition);

	if (*g_paf_opened == 2) {
		log_inf("Wait\n");
		(void)s4AF_WriteReg(AF_INIT_POS);
		msleep(12);
	}

	if (*g_paf_opened) {
		log_inf("Free\n");

		spin_lock(g_paf_spinlock);
		*g_paf_opened = 0;
		spin_unlock(g_paf_spinlock);
	}

	log_inf("End\n");

	return 0;
}

int CN3928AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
	spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstaf_i2cclient = pstAF_I2Cclient;
	g_paf_spinlock = pAF_SpinLock;
	g_paf_opened = pAF_Opened;

	return 1;
}

int CN3928AF_GetFileName(unsigned char *pFileName)
{
	if (pFileName == NULL) {
		log_inf("pFileName is NULL error!\n");
		return -1;
	}
	strncpy_s(pFileName, AF_MOTOR_NAME + 1,
		AF_CN3928AF_FILE_NAME, AF_MOTOR_NAME);
	log_inf("FileName:%s\n", pFileName);
	return 1;
}
