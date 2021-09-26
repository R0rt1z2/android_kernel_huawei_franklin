/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2020. All rights reserved.
 * Description: the rootagent_check.c is for debug mode checking.
 * Create: 2019-5-22
 */

#include "rootagent_check.h"

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG
static int tee_kcode_proc_show(struct seq_file *m, void *v)
{
	int ret = tee_status_check(KERNELCODEBIT);

	if (ret != 0)
		antiroot_error("tee kcode check failed, ret = %d\n", ret);
	else
		antiroot_debug(ROOTAGENT_DEBUG_AGENT, "tee kcode check ok\n");
	return ret;
}

static int tee_sehooks_proc_show(struct seq_file *m, void *v)
{
	int ret = tee_status_check(SEHOOKBIT);

	if (ret != 0)
		antiroot_error("tee sehooks check failed, ret = %d\n", ret);
	else
		antiroot_debug(ROOTAGENT_DEBUG_AGENT, "tee sehooks check ok\n");
	return ret;
}

static int tee_syscall_proc_show(struct seq_file *m, void *v)
{
	int ret = tee_status_check(SYSTEMCALLBIT);

	if (ret != 0)
		antiroot_error("tee syscall check failed, ret = %d\n", ret);
	else
		antiroot_debug(ROOTAGENT_DEBUG_AGENT, "tee syscall check ok\n");
	return ret;
}

static int tee_kenerl_address_proc_show(struct seq_file *m, void *v)
{
	int ret = tee_status_check(CHECKFAILBIT);

	if (ret != 0)
		antiroot_error("tee no kenerl addr check failed , ret = %d\n",
			ret);
	else
		antiroot_debug(ROOTAGENT_DEBUG_AGENT,
			"tee no kenerl addr check ok\n");
	return ret;
}

static int tee_kcode_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tee_kcode_proc_show, NULL);
}

static int tee_sehooks_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tee_sehooks_proc_show, NULL);
}

static int tee_syscall_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tee_syscall_proc_show, NULL);
}

static int tee_kenerl_address_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tee_kenerl_address_proc_show, NULL);
}
#endif
#endif

#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG
static int tee_rodata_proc_show(struct seq_file *m, void *v)
{
	int ret = tee_status_check(RODATABIT);

	if (ret != 0)
		antiroot_error("tee rodata check failed, ret = %d\n", ret);
	else
		antiroot_info("tee rodata check ok\n");
	return ret;
}

static int tee_rodata_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tee_rodata_proc_show, NULL);
}
#endif
#endif

void handle_proc_create(void)
{
#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG
#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
	static const struct file_operations tsehooks_fops = {
		.owner  = THIS_MODULE,
		.open   = tee_sehooks_proc_open,
		.read   = seq_read,
		.llseek = seq_lseek,
	};
	static const struct file_operations tkcode_fops = {
		.owner  = THIS_MODULE,
		.open   = tee_kcode_proc_open,
		.read   = seq_read,
		.llseek = seq_lseek,
	};
	static const struct file_operations tsyscall_fops = {
		.owner  = THIS_MODULE,
		.open   = tee_syscall_proc_open,
		.read   = seq_read,
		.llseek = seq_lseek,
	};
	static const struct file_operations tkaddr_fops = {
		.owner  = THIS_MODULE,
		.open   = tee_kenerl_address_proc_open,
		.read   = seq_read,
		.llseek = seq_lseek,
	};
#endif

#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
	static const struct file_operations trodata_fops = {
		.owner  = THIS_MODULE,
		.open   = tee_rodata_proc_open,
		.read   = seq_read,
		.llseek = seq_lseek,
	};
	/*
	 * 0440 means the file owner and group only have read permission
	 * ts_tm_rodata proc means check tee measure at read only data section
	 * trodata_fops means the check rodata proc's file operations,
	 * such as open, read
	 */
	proc_create("ts_tm_rodata", 0440, NULL, &trodata_fops);
#endif

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
	/* 0440 is mean only user and user group have read premission */
	proc_create("ts_tm_kcode", 0440, NULL, &tkcode_fops);
	proc_create("ts_tm_syscall", 0440, NULL, &tsyscall_fops);
	proc_create("ts_tm_sehooks", 0440, NULL, &tsehooks_fops);
	proc_create("ts_tm_kaddr", 0440, NULL, &tkaddr_fops);
	antiroot_debug(ROOTAGENT_DEBUG_AGENT, "tee test proc creat\n");
#endif
#endif
}

