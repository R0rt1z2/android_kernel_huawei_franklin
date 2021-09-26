/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: the eima_fs.c for EMUI Integrity Measurement Architecture(EIMA)
 *     eima kernel module file system function
 * Create: 2017-12-20
 */

#include "eima_fs.h"
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/parser.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>
#include <linux/security.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include "eima_agent_api.h"
#include "eima_queue.h"
#include "eima_utils.h"

static struct dentry *g_eima_dir;
static struct dentry *g_runtime_measurements;

static const integrity_targets_t g_measure_targets_list[] = {
	{ EIMA_POLICY_PATH_TEECD, EIMA_DYNAMIC, 0, 0, {0}, 0 },
	{ EIMA_POLICY_PATH_SYSTEM_TEECD, EIMA_DYNAMIC, 0, 0, {0}, 0 },
	{ EIMA_POLICY_PATH_DEBUGGERD, EIMA_STATIC, 0, 0, {0}, 0 },
	{ EIMA_POLICY_PATH_TOMBSTONED, EIMA_STATIC, 0, 0, {0}, 0 },
	{ EIMA_POLICY_PATH_TOMBSTONED, EIMA_DYNAMIC, 0, 0, {0}, 0 },
#ifdef CONFIG_HUAWEI_ENG_EIMA
	{ EIMA_POLICY_PATH_HELLO, EIMA_STATIC, 0, 0, {0}, 0 },
	{ EIMA_POLICY_PATH_HELLO, EIMA_DYNAMIC, 0, 0, {0}, 0 }
#endif
};

struct file *eima_file_open(const char *path, int flag, umode_t mode)
{
	struct file *filp = NULL;
	long err;

	if (path == NULL) {
		eima_error("file path is empty");
		return NULL;
	}

	filp = filp_open(path, flag, mode);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		eima_error("open file %s failed with error code %ld",
			path, err);
		return NULL;
	}

	return filp;
}

void eima_file_close(struct file *fp)
{
	if (fp == NULL) {
		eima_warning("file pointer is invalid");
		return;
	}
	filp_close(fp, NULL);
}

#ifndef EIMA_LOAD_PIF
static void eima_add_default_policy(void)
{
	int s_ret;
	uint32_t i;

	g_eima_pif.policy_count = 1;
	s_ret = strcpy_s(g_eima_pif.policy[0].usecase_name,
		EIMA_NAME_STRING_LEN + 1, EIMA_POLICY_USECASE_NAME);
	if (s_ret != EOK) {
		eima_error("strcpy_s failed, s_ret = %d", s_ret);
		return;
	}

	for (i = 0; (i < ARRAY_SIZE(g_measure_targets_list)) &&
		(i < EIMA_MAX_MEASURE_OBJ_CNT); i++) {
		s_ret = strcpy_s(g_eima_pif.policy[0].targets[i].file,
			EIMA_NAME_STRING_LEN + 1, g_measure_targets_list[i].file);
		if (s_ret != EOK) {
			eima_error("strcpy_s failed, s_ret = %d", s_ret);
			return;
		}
		g_eima_pif.policy[0].targets[i].target_pid = g_measure_targets_list[i].target_pid;
		g_eima_pif.policy[0].targets[i].type = g_measure_targets_list[i].type;
		g_eima_pif.policy[0].target_count++;
	}
}
#endif

static int eima_load_policy_config(void)
{
	int j;
	int i;

#ifndef EIMA_LOAD_PIF
	eima_add_default_policy();
#endif

	eima_debug("The total number of policy is %u", g_eima_pif.policy_count);
	for (i = 0; i < g_eima_pif.policy_count; i++) {
		eima_debug("  policy : %s", g_eima_pif.policy[i].usecase_name);
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++)
			eima_debug("target's filename : %s, type : %u",
				g_eima_pif.policy[i].targets[j].file,
				g_eima_pif.policy[i].targets[j].type);
	}

	return 0;
}

static int eima_ascii_measurements_show(struct seq_file *file, void *iter)
{
	struct eima_queue_entry *qe = NULL;
	struct eima_template_entry *e = NULL;
	int pcr_idx;
	u32 i;

	rcu_read_lock();
	list_for_each_entry_rcu(qe, &g_eima_measurements_list, later) {
		e = &qe->entry;

		pcr_idx = (e->type == EIMA_STATIC) ?
			EIMA_STATIC_MEASURE_PCR_IDX : EIMA_DYNAMIC_MEASURE_PCR_IDX;
		seq_printf(file, "%2d ", pcr_idx);

		for (i = 0; i < EIMA_HASH_DIGEST_SIZE; i++)
			seq_printf(file, "%02x", e->digest[i]);

		seq_printf(file, " %s", e->fn);

		seq_puts(file, "\n");
	}
	rcu_read_unlock();

	(void)iter;

	return 0;
}

static int eima_ascii_measurements_open(struct inode *inode, struct file *file)
{
	return single_open(file, eima_ascii_measurements_show, inode->i_private);
}

static const struct file_operations g_eima_ascii_measurements_ops = {
	.owner = THIS_MODULE,
	.open = eima_ascii_measurements_open,
	.read = seq_read,
	.release = single_release,
};

int __init eima_fs_init(void)
{
	int ret;

	ret = eima_load_policy_config();
	if (ret != 0) {
		eima_error("eima load policy config failed, ret is %d", ret);
		return ERROR_CODE;
	}

#ifdef CONFIG_SECURITYFS
	g_eima_dir = securityfs_create_dir("eima", NULL);
	if (IS_ERR(g_eima_dir)) {
		eima_error("securityfs create dir failed");
		return ERROR_CODE;
	}
	g_runtime_measurements = securityfs_create_file(
		"runtime_measurements", S_IRUSR | S_IRGRP, g_eima_dir, NULL,
		&g_eima_ascii_measurements_ops);
	if (IS_ERR(g_runtime_measurements)) {
		eima_error("runtime measurements file create failed");
		securityfs_remove(g_eima_dir);
		return ERROR_CODE;
	}
#endif

	return 0;
}

void eima_fs_exit(void)
{
#ifdef CONFIG_SECURITYFS
	if (g_runtime_measurements != NULL) {
		securityfs_remove(g_runtime_measurements);
		g_runtime_measurements = NULL;
	}

	if (g_eima_dir != NULL) {
		securityfs_remove(g_eima_dir);
		g_eima_dir = NULL;
	}
#endif
}
