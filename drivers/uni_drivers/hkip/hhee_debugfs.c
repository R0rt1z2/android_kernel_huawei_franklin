/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2019. All rights reserved.
 * Description:  HHEE general communication and test driver
 * Creator: security-ap
 * Date: 2017/2/1
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <asm/compiler.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "hhee.h"

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
#include <linux/uaccess.h>
#endif

static struct dentry *hkip_dir = 0;

/*following functions are used for log read*/
static int hhee_log_open(struct inode *inode, struct file *pfile)
{
	pfile->private_data = inode->i_private;
	return 0;
}

static ssize_t hhee_log_read(struct file *file, char __user *buf,
	size_t count, loff_t *offp)
{
	int logtype =  (int)(uintptr_t)file->private_data;

	if (logtype == CRASH_LOG)
		return 0;

	pr_info("hhee log read, logtype is %d\n", logtype);
	return hhee_copy_logs(buf, count, offp, logtype);
}

#define CONST_INFO_LEN    (18)
static ssize_t hhee_log_write(struct file *file, const char __user *buf,
	size_t count, loff_t *offp)
{
	int logtype = (int)(uintptr_t)file->private_data;
	char info[CONST_INFO_LEN];
	char *s = "kernel_crash_test";
	const int len = strlen(s);

	if(!buf)
		return -EINVAL;

	/* if then count cannot be matched with CONST_INFO_LEN,
	 * the input string cannot be kernel_crash_test either*/
	if (count != CONST_INFO_LEN)
		return -1;

	if (CRASH_LOG == logtype) {
		memset(info, 0x0, CONST_INFO_LEN); /* unsafe_function_ignore: memset*/
		if(copy_from_user(info, buf, CONST_INFO_LEN - 1))
			return -1;
		info[CONST_INFO_LEN - 1] = '\0';
		if(!strncmp(info, s, len)){
			pr_err("call AP crash from HHEE for test.\n");
			(void)hhee_fn_hvc((unsigned long)HHEE_HVC_NOTIFY, 0ul, 0ul, 0ul);
		}
	}

	return count;
}

const struct file_operations tzdbg_fops = {
	.owner   = THIS_MODULE,
	.read    = hhee_log_read,
	.write   = hhee_log_write,
	.open    = hhee_log_open,
};

int hhee_init_debugfs(void)
{
	struct dentry *junk = NULL;
	int ret;

	hkip_dir = debugfs_create_dir("hhee", NULL);
	if (!hkip_dir) {
		printk(KERN_ALERT "HHEE: failed to create /sys/kernel/debug/hhee\n");
		return -1;
	}

	junk = debugfs_create_file(
			"crashlog",
			0220,
			hkip_dir,
			(void *)CRASH_LOG,
			&tzdbg_fops);
	if (!junk) {
		pr_err("HHEE: failed to create /sys/kernel/debug/hhee/crashlog\n");
		ret = -1;
		goto out;
	}

	junk = debugfs_create_file(
			"pmflog",
			0220,
			hkip_dir,
			(void *)PMF_LOG,
			&tzdbg_fops);
	if (!junk) {
		pr_err("HHEE: failed to create /sys/kernel/debug/hhee/pmflog\n");
		ret = -1;
		goto out;
	}

	return 0;
out:
	debugfs_remove_recursive(hkip_dir);
	return ret;

}

/* This is called when the module is removed */
void hhee_cleanup_debugfs(void)
{
	debugfs_remove_recursive(hkip_dir);
}
