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
#define DW9800AF_SetI2Cclient DW9800AF_SetI2Cclient_Main
#define DW9800AF_Ioctl DW9800AF_Ioctl_Main
#define DW9800AF_Release DW9800AF_Release_Main
#define DW9800AF_GetFileName DW9800AF_GetFileName_Main
extern int DW9800AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9800AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9800AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9800AF_GetFileName(unsigned char *pFileName);

#define DW9714AF_SetI2Cclient DW9714AF_SetI2Cclient_Main
#define DW9714AF_Ioctl DW9714AF_Ioctl_Main
#define DW9714AF_Release DW9714AF_Release_Main
#define DW9714AF_GetFileName DW9714AF_GetFileName_Main
extern int DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9714AF_GetFileName(unsigned char *pFileName);

extern struct regulator *regulator_get_regVCAMAF(void);
#define CN3928AF_SetI2Cclient CN3928AF_SetI2Cclient_Main
#define CN3928AF_Ioctl CN3928AF_Ioctl_Main
#define CN3928AF_Release CN3928AF_Release_Main
#define CN3928AF_GetFileName CN3928AF_GetFileName_Main
extern int CN3928AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long CN3928AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int CN3928AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int CN3928AF_GetFileName(unsigned char *pFileName);

#define GT9764AF_SetI2Cclient GT9764AF_SetI2Cclient_Main
#define GT9764AF_Ioctl GT9764AF_Ioctl_Main
#define GT9764AF_Release GT9764AF_Release_Main
#define GT9764AF_GetFileName GT9764AF_GetFileName_Main
extern int GT9764AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9764AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int GT9764AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9764AF_GetFileName(unsigned char *pFileName);

extern int fp5516af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened);
extern long fp5516af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param);
extern int fp5516af_release(struct inode *af_inode, struct file *af_file);
extern int fp5516af_get_file_name(unsigned char *af_file_name);

extern int dw9738af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened);
extern long dw9738af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param);
extern int dw9738af_release(struct inode *af_inode, struct file *af_file);
extern int dw9738af_get_file_name(unsigned char *af_file_name);
extern int ak7375af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_Lock, int *af_opened);
extern long ak7375af_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param);
extern int ak7375af_release(struct inode *a_inode, struct file *a_file);
extern int ak7375af_get_file_name(unsigned char *file_name);

extern int cn3938af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened);
extern long cn3938af_ioctl(struct file *af_file, unsigned int command,
	unsigned long param);
extern int cn3938af_release(struct inode *af_inode, struct file *af_file);
extern int cn3938af_get_file_name(unsigned char *af_file_name);

extern int gt9764baaf_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened);
extern long gt9764baaf_ioctl(struct file *af_file, unsigned int command,
	unsigned long param);
extern int gt9764baaf_release(struct inode *af_inode, struct file *af_file);
extern int gt9764baaf_get_file_name(unsigned char *af_file_name);

extern int bu64754af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_Lock, int *af_opened);
extern long bu64754af_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param);
extern int bu64754af_release(struct inode *a_inode, struct file *a_file);
extern int bu64754af_get_file_name(unsigned char *file_name);

extern int lc898249af_set_i2c_client(struct i2c_client *af_i2c_client,
			spinlock_t *af_spin_Lock, int *af_opened);
extern long lc898249af_ioctl(struct file *a_file, unsigned int a_command,
			unsigned long a_param);
extern int lc898249af_release(struct inode *a_inode, struct file *a_file);
extern int lc898249af_get_file_name(unsigned char *file_name);

extern int dw9800vaf_set_i2c_client(struct i2c_client *pst_af_i2c_client,
	spinlock_t *p_af_spin_lock, int *p_af_opened);
extern long dw9800vaf_ioctl(struct file *af_file, unsigned int af_command,
	unsigned long af_param);
extern int dw9800vaf_release(struct inode *af_inode, struct file *af_file);
extern int dw9800vaf_get_file_name(unsigned char *p_file_name);

extern int cn3927af_set_i2c_client(struct i2c_client *af_i2c_client,
	spinlock_t *af_spinlock, int *af_opened);
extern long cn3927af_ioctl(struct file *a_pstFile, unsigned int command,
	unsigned long param);
extern int cn3927af_release(struct inode *af_inode, struct file *af_file);
extern int cn3927af_get_file_name(unsigned char *af_file_name);
#endif

