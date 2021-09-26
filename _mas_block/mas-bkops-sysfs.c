/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2019. All rights reserved.
 * Description: mas bkops debug sysfs
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/mas-bkops-core.h>

#include "mas-bkops-sysfs-interface.h"

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
#define MAX_TIME_STR_LEN 32
#define TIME_STR_RESOLUTION 10
#define TIME_MULTIPLE 1000UL
static const char *const time_unit[] = { "s", "ms", "us", "ns" };
static char *nice_time(u64 time_ns, char *time_str_buf, int len)
{
	u64 multiplier = NSEC_PER_SEC;
	u32 i;

	if (!time_str_buf || len < MAX_TIME_STR_LEN) {
		pr_err("time_str_buf is Invalid!\n");
		return NULL;
	}

	if (time_ns <= TIME_STR_RESOLUTION) {
		snprintf(time_str_buf, len, "%llu ns", time_ns);
	} else {
		for (i = 0; i < ARRAY_SIZE(time_unit);
			i++, multiplier /= TIME_MULTIPLE) {
			if (time_ns < (multiplier * TIME_STR_RESOLUTION))
				continue;

			snprintf(time_str_buf, len, "%llu %s",
				time_ns / multiplier, time_unit[i]);
			break;
		}
	}
	time_str_buf[len - 1] = '\0';
	return time_str_buf;
}

static int mas_bkops_print_action_cnt(
	char *buf, size_t len, struct bkops_stats *bkops_stats)
{
	int offset = 0;

	offset += snprintf(buf, len, "bkops retry count: %u\n",
		bkops_stats->bkops_retry_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops started count: %u\n", bkops_stats->bkops_start_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops stopped count: %u\n", bkops_stats->bkops_stop_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops core stop count: %u\n",
		bkops_stats->bkops_core_stop_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops abort count: %u\n", bkops_stats->bkops_abort_count);

	offset += snprintf(buf + offset, len - offset, "\n");

	return offset;
}

static int mas_bkops_print_bkops_status_cnt(
	char *buf, size_t len, const struct bkops_stats *bkops_stats)
{
	unsigned int i;
	int offset = 0;

	for (i = 0; i < bkops_stats->bkops_status_max; i++)
		offset += snprintf(buf + offset, len - offset, "%s count: %u\n",
			bkops_stats->bkops_status_str[i],
			bkops_stats->bkops_status[i]);

	offset += snprintf(buf + offset, len - offset, "\n");

	return offset;
}

static int mas_bkops_print_query_cnt(
	char *buf, size_t len, const struct bkops_stats *bkops_stats)
{
	int offset = 0;

	offset += snprintf(buf, len, "bkops actual query count: %u\n",
		bkops_stats->bkops_actual_query_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops idle work canceled count: %u\n",
		bkops_stats->bkops_idle_work_canceled_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops idle work waited count: %u\n",
		bkops_stats->bkops_idle_work_waited_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops time based query count: %u\n",
		bkops_stats->bkops_time_query_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops write based query count: %u\n",
		bkops_stats->bkops_write_query_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops discard based query count: %u\n",
		bkops_stats->bkops_discard_query_count);

	offset += snprintf(buf + offset, len - offset, "\n");

	return offset;
}

static int mas_bkops_print_fail_cnt(
	char *buf, size_t len, const struct bkops_stats *bkops_stats)
{
	int offset = 0;

	offset += snprintf(buf, len, "bkops query fail count: %u\n",
		bkops_stats->bkops_query_fail_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops start fail count: %u\n",
		bkops_stats->bkops_start_fail_count);

	offset += snprintf(buf + offset, len - offset,
		"bkops stop fail count: %u\n",
		bkops_stats->bkops_stop_fail_count);

	offset += snprintf(buf + offset, len - offset, "\n");

	return offset;
}


static int mas_bkops_print_action_time(
	char *buf, size_t len, const struct bkops_stats *bkops_stats)
{
	char time_str_buf[MAX_TIME_STR_LEN];
	int offset = 0;

	offset += snprintf(buf, len, "bkops max query time: %s\n",
		nice_time(bkops_stats->bkops_max_query_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops average query time: %s\n",
		nice_time(bkops_stats->bkops_avrg_query_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops max start time: %s\n",
		nice_time(bkops_stats->bkops_max_start_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops average start time: %s\n",
		nice_time(bkops_stats->bkops_avrg_start_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops max stop time: %s\n",
		nice_time(bkops_stats->bkops_max_stop_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops average stop time: %s\n",
		nice_time(bkops_stats->bkops_avrg_stop_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset, "\n");

	return offset;
}

static int mas_bkops_print_dur(
	char *buf, size_t len, const struct bkops_stats *bkops_stats)
{
	char time_str_buf[MAX_TIME_STR_LEN];
	int offset = 0;

	offset += snprintf(buf, len, "max_bkops_duration: %s\n",
		nice_time(bkops_stats->max_bkops_duration, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"average_bkops_duration: %s\n",
		nice_time(bkops_stats->bkops_avrg_exe_time, time_str_buf,
			MAX_TIME_STR_LEN));

	offset += snprintf(buf + offset, len - offset,
		"bkops duration less than 100ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_100MS]);

	offset += snprintf(buf + offset, len - offset,
		"bkops duration less than 500ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_500MS]);

	offset += snprintf(buf + offset, len - offset,
		"bkops duration less than 1000ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_1000MS]);

	offset += snprintf(buf + offset, len - offset,
		"bkops duration less than 2000ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_2000MS]);

	offset += snprintf(buf + offset, len - offset,
		"bkops duration less than 5000ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_5000MS]);

	offset += snprintf(buf + offset, len - offset,
		"bkops duration great than 5000ms: %u\n",
		bkops_stats->bkops_dur[BKOPS_DUR_IDX_FOR_AGES]);

	return offset;
}

/* this is for debug purpose and PAGE_SIZE is long enough for now */
int mas_bkops_stat_open(const struct inode *inode, struct file *filp)
{
	struct mas_bkops *bkops = inode->i_private;
	struct bkops_stats *bkops_stats = &(bkops->bkops_stats);
	char *buf = NULL;
	int offset;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* buf of PAGE_SIZE is enough for our test purpose sysfs node */
	offset = mas_bkops_print_action_cnt(buf, PAGE_SIZE, bkops_stats);
	offset += mas_bkops_print_bkops_status_cnt(
		buf + offset, PAGE_SIZE - offset, bkops_stats);
	offset += mas_bkops_print_query_cnt(
			buf + offset, PAGE_SIZE - offset, bkops_stats);
	offset += mas_bkops_print_fail_cnt(
			buf + offset, PAGE_SIZE - offset, bkops_stats);
	offset += mas_bkops_print_action_time(
			buf + offset, PAGE_SIZE - offset, bkops_stats);
	offset += mas_bkops_print_dur(
			buf + offset, PAGE_SIZE - offset, bkops_stats);
	buf[PAGE_SIZE - 1] = 0;
	filp->private_data = buf;

	return 0;
}

ssize_t mas_bkops_stat_read(
	const struct file *filp, char __user *ubuf, size_t cnt,
	const loff_t *ppos)
{
	char *buf = filp->private_data;

	if (!buf)
		return (ssize_t)0;

	return simple_read_from_buffer(
		ubuf, cnt, (loff_t *)ppos, buf, strlen(buf));
}

int mas_bkops_stat_release(const struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	file->private_data = NULL;

	return 0;
}
static const struct file_operations mas_bkops_stat_fops = {
	.open = __cfi_mas_bkops_stat_open,
	.read = __cfi_mas_bkops_stat_read,
	.release = __cfi_mas_bkops_stat_release,
};

int mas_bkops_force_query_open(const struct inode *inode, struct file *filp)
{
	struct mas_bkops *bkops = inode->i_private;

	if (!bkops || !bkops->bkops_ops ||
		!bkops->bkops_ops->bkops_status_query) {
		pr_err("Invalid bkops params!\n");
		return -ENODEV;
	}

	filp->private_data = bkops;

	return 0;
}

ssize_t mas_bkops_force_query_read(
	const struct file *filp, char __user *ubuf, size_t cnt,
	const loff_t *ppos)
{
	int bkops_status = 0;
	ssize_t ret;
	char *buf = NULL;
	struct mas_bkops *bkops = (struct mas_bkops *)(filp->private_data);

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (bkops->bkops_ops->bkops_status_query(
		    bkops->bkops_data, &bkops_status)) {
		pr_err("Failed to query bkops status!\n");
		snprintf(buf, (size_t)PAGE_SIZE,
			"Failed to query bkops status!\n");
	} else {
		pr_err("bkops_status: %d\n", bkops_status);
		snprintf(buf, (size_t)PAGE_SIZE, "bkops_status: %d\n",
			bkops_status);
	}

	ret = simple_read_from_buffer(
		ubuf, cnt, (loff_t *)ppos, buf, strlen(buf));

	kfree(buf);
	return ret;
}

int mas_bkops_force_query_release(
	const struct inode *inode, const struct file *file)
{
	return 0;
}

static const struct file_operations mas_bkops_force_query_fops = {
	.open = __cfi_mas_bkops_force_query_open,
	.read = __cfi_mas_bkops_force_query_read,
	.release = __cfi_mas_bkops_force_query_release,
};

static int mas_bkops_add_test_debugfs(
	struct mas_bkops *bkops, const struct dentry *parent_dir)
{
	struct dentry *bkops_test_root = NULL;

	bkops_test_root = debugfs_create_dir(
		"bkops_test_root", (struct dentry *)parent_dir);
	if (IS_ERR(bkops_test_root))
		return -ENODEV;

	if (!debugfs_create_bool("sim_bkops_start_fail", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_bkops_start_fail)))
		goto err;
	if (!debugfs_create_bool("sim_bkops_stop_fail", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_bkops_stop_fail)))
		goto err;
	if (!debugfs_create_bool("sim_bkops_query_fail", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_bkops_query_fail)))
		goto err;
	if (!debugfs_create_bool("sim_critical_bkops", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_critical_bkops)))
		goto err;
	if (!debugfs_create_bool("sim_bkops_abort", 0600,
		    bkops_test_root, &(bkops->bkops_debug_ops.sim_bkops_abort)))
		goto err;
	if (!debugfs_create_u32("sim_bkops_stop_delay", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_bkops_stop_delay)))
		goto err;
	if (!debugfs_create_u32("sim_bkops_query_delay", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.sim_bkops_query_delay)))
		goto err;
	if (!debugfs_create_bool("skip_bkops_stop", 0600,
		    bkops_test_root, &(bkops->bkops_debug_ops.skip_bkops_stop)))
		goto err;
	if (!debugfs_create_bool("disable_bkops", 0600,
		    bkops_test_root, &(bkops->bkops_debug_ops.disable_bkops)))
		goto err;
	if (!debugfs_create_bool("bkops_force_query", 0600,
		    bkops_test_root,
		    &(bkops->bkops_debug_ops.bkops_force_query)))
		goto err;

	return 0;

err:
	debugfs_remove_recursive(bkops_test_root);
	return -ENODEV;
}

int mas_bkops_add_debugfs(
	struct mas_bkops *bkops, const struct dentry *parent_dir)
{
	struct dentry *bkops_root = NULL;

	if (IS_ERR(parent_dir) || !bkops)
		return -ENODEV;

	bkops_root =
		debugfs_create_dir("bkops_root", (struct dentry *)parent_dir);
	if (IS_ERR(bkops_root))
		return -ENODEV;

	if (!debugfs_create_file("bkops_stat", 0600, bkops_root,
		    bkops, &mas_bkops_stat_fops))
		goto err;
	if (!debugfs_create_file("force_bkops_query", 0600,
		    bkops_root, bkops, &mas_bkops_force_query_fops))
		goto err;
	if (!debugfs_create_ulong("bkops_idle_delay", 0600,
		    bkops_root, &(bkops->bkops_idle_delay_ms)))
		goto err;
	if (!debugfs_create_ulong("bkops_check_interval", 0600,
		    bkops_root, &(bkops->bkops_check_interval)))
		goto err;
	if (!debugfs_create_ulong("bkops_check_discard_len", 0600,
		    bkops_root, &(bkops->bkops_check_discard_len)))
		goto err;
	if (!debugfs_create_ulong("bkops_check_write_len", 0600,
		    bkops_root, &(bkops->bkops_check_write_len)))
		goto err;
	if (!debugfs_create_u32("bkops_retry_en", 0600, bkops_root,
		    &(bkops->en_bkops_retry)))
		goto err;

	if (mas_bkops_add_test_debugfs(bkops, bkops_root))
		goto err;

	bkops->bkops_root = bkops_root;
	return 0;

err:
	debugfs_remove_recursive(bkops_root);
	return -ENODEV;
}
#else
int mas_bkops_add_debugfs(
	struct mas_bkops *bkops, const struct dentry *parent_dir)
{
	return 0;
};
#endif /* CONFIG_MAS_DEBUG_FS */
