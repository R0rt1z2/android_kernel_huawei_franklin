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

#ifndef _LENS_LIST_H

#define _LENS_LIST_H

#if 1
#define AK7371AF_SetI2Cclient AK7371AF_SetI2Cclient_Main3
#define AK7371AF_Ioctl AK7371AF_Ioctl_Main3
#define AK7371AF_Release AK7371AF_Release_Main3
#define AK7371AF_PowerDown AK7371AF_PowerDown_Main3
#define AK7371AF_GetFileName AK7371AF_GetFileName_Main3
extern int AK7371AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long AK7371AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int AK7371AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int AK7371AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int AK7371AF_GetFileName(unsigned char *pFileName);

#define BU24253AF_SetI2Cclient BU24253AF_SetI2Cclient_Main3
#define BU24253AF_Ioctl BU24253AF_Ioctl_Main3
#define BU24253AF_Release BU24253AF_Release_Main3
#define BU24253AF_PowerDown BU24253AF_PowerDown_Main3
#define BU24253AF_GetFileName BU24253AF_GetFileName_Main3
extern int BU24253AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU24253AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU24253AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU24253AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int BU24253AF_GetFileName(unsigned char *pFileName);
#endif

#define GT9772AF_SetI2Cclient GT9772AF_SetI2Cclient_Main3
#define GT9772AF_Ioctl GT9772AF_Ioctl_Main3
#define GT9772AF_Release GT9772AF_Release_Main3
#define GT9772AF_PowerDown GT9772AF_PowerDown_Main3
#define GT9772AF_GetFileName GT9772AF_GetFileName_Main3
extern int GT9772AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9772AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9772AF_GetFileName(unsigned char *pFileName);

extern int dw9781baf_set_i2c_client(struct i2c_client *af_i2c_client,
				spinlock_t *af_spin_lock, int *af_opened);
extern long dw9781baf_ioctl(struct file *a_file, unsigned int a_command,
				unsigned long a_param);
extern int dw9781baf_release(struct inode *a_inode, struct file *a_file);
extern int dw9781baf_get_file_name(unsigned char *file_name);

extern int ak7375afd_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_Lock, int *af_opened);
extern long ak7375afd_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param);
extern int ak7375afd_release(struct inode *a_inode, struct file *a_file);
extern int ak7375afd_get_file_name(unsigned char *file_name);

extern int lc898219af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_lock, int *af_opened);
extern long lc898219af_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param);
extern int lc898219af_release(struct inode *a_inode, struct file *a_file);
extern int lc898219af_get_file_name(unsigned char *file_name);

extern void AFRegulatorCtrl(int Stage);
#endif
