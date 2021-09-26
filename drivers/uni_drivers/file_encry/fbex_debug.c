/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex for "ioctrl cmd" source file
 * Author : security-ap
 * Create : 2020/07/16
 */

#include "fbex_debug.h"
#include "fbex_driver.h"

#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <uni/fbe/fbe_ctrl.h>

struct fbex_debug {
	u32 ret;
	u32 cmd;
	u32 index;
	u8 pubkey[PUBKEY_LEN];
	u32 key_len;
	u8 metadata[METADATA_LEN];
	u32 iv_len;
};

static DEFINE_MUTEX(g_fbex_debug_mutex);
static struct completion g_fbex_debug_comp;

/* global fbex debug queue needed */
static struct fbex_debug g_fbex_debug;
static struct work_struct g_fbex_debug_work;
static struct workqueue_struct *g_fbex_debug_wq;

static void huawei_fbex_debug_fn(struct work_struct *work)
{
	u32 ret = 0;
	u32 cmd = g_fbex_debug.cmd;

	switch (cmd) {
	case SEC_FILE_ENCRY_CMD_ID_KEY_RESTORE:
		ret = huawei_fbex_restore_key();
		break;
	default:
		break;
	}

	g_fbex_debug.ret = ret;
	complete(&g_fbex_debug_comp);
}

static u32 huawei_fbex_debug_dispatch(void __user *argp)
{
	u32 ret = 0;

	if (!argp) {
		pr_err("%s, user buff is NULL\n", __func__);
		return FBE2_ERROR_BUFFER_NULL;
	}
	ret = copy_from_user(&g_fbex_debug, argp, sizeof(g_fbex_debug));
	if (ret) {
		pr_err("%s, copy to user fail\n", __func__);
		return FBE2_ERROR_COPY_FAIL;
	}

	queue_work(g_fbex_debug_wq, &g_fbex_debug_work);
	if (!wait_for_completion_timeout(&g_fbex_debug_comp,
			msecs_to_jiffies(TEST_TIMEOUT))) {
		pr_err("fbex debug: timeout!\n");
		return FBE2_ERROR_DEBUG_TIMEER_OUT;
	}
	ret = copy_to_user(argp, &g_fbex_debug, sizeof(g_fbex_debug));
	if (ret) {
		pr_err("%s, copy from user fail\n", __func__);
		return FBE2_ERROR_COPY_FAIL;
	}
	return g_fbex_debug.ret;
}

static long fbex_debug_unlocked_ioctl(struct file *file, unsigned int cmd,
				      unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)(uintptr_t)arg;

	mutex_lock(&g_fbex_debug_mutex);

	ret = huawei_fbex_debug_dispatch(argp);

	mutex_unlock(&g_fbex_debug_mutex);
	return ret;
}

static int fbex_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static const struct file_operations g_fbex_debug_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fbex_debug_unlocked_ioctl,
	.open = fbex_open,
};

static struct miscdevice g_fbex_dev_debug = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fbex_debug",
	.fops = &g_fbex_debug_fops,
	.mode = 0600, /* permission */
};

int fbex_init_debugfs(void)
{
	int ret;

	init_completion(&g_fbex_debug_comp);
	ret = misc_register(&g_fbex_dev_debug);
	if (ret != 0) {
		pr_err("failed to register fbex debug\n");
		return ret;
	}

	g_fbex_debug_wq = create_singlethread_workqueue("fbex_debug");
	if (!g_fbex_debug_wq) {
		pr_err("Create fbex debug work failed\n");
		return -1;
	}
	INIT_WORK(&g_fbex_debug_work, huawei_fbex_debug_fn);
	return 0;
}

/* This is called when the module is removed */
void fbex_cleanup_debugfs(void)
{
	if (g_fbex_dev_debug.list.prev)
		misc_deregister(&g_fbex_dev_debug);
}
