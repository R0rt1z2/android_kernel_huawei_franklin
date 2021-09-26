/*
 * Huawei Sched Cluster Debug File
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "./sched_cluster.h"

#include <linux/string.h>
#include <linux/cpumask.h>

static int sched_cluster_debug_show(struct seq_file *file, void *param)
{
	struct sched_cluster *cluster = NULL;

	seq_printf(file, "min_id:%d, max_id:%d\n",
		min_cap_cluster()->id,
		max_cap_cluster()->id);

	for_each_sched_cluster(cluster) {
		seq_printf(file, "id:%d, cpumask:%d(%*pbl)\n",
			   cluster->id,
			   cpumask_first(&cluster->cpus),
			   cpumask_pr_args(&cluster->cpus));
	}

	return 0;
}

static int sched_cluster_debug_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sched_cluster_debug_show, NULL);
}

static const struct file_operations sched_cluster_fops = {
	.open		= sched_cluster_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init init_sched_cluster_debug_procfs(void)
{
	struct proc_dir_entry *pe = NULL;

	pe = proc_create("sched_cluster",
		0444, NULL, &sched_cluster_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}
late_initcall(init_sched_cluster_debug_procfs);
