/*
 * rtg_debug.c
 *
 * related thread grop debug info implementation
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

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <../kernel/sched/sched.h>

#include "include/rtg.h"
#include "include/rtg_sched.h"
#ifdef CONFIG_HW_RTG_FRAME
#include "include/frame_info.h"
#endif
#ifdef CONFIG_HW_RTG_AUX
#include "include/aux_info.h"
#endif
#ifdef CONFIG_HW_RTG_FRAME_USE_NORMALIZED_UTIL
#include "../hw_cluster/sched_cluster.h"
#endif

/*
 * This allows printing both to /proc/sched_debug and to the console
 */
#define seq_printf_rtg(m, x...) \
do { \
	if (m) \
		seq_printf(m, x); \
	else \
		printk(x); \
} while (0)

static void print_rtg_info(struct seq_file *file,
	const struct related_thread_group *grp)
{
	seq_printf_rtg(file, "RTG_ID          : %d\n", grp->id);
	seq_printf_rtg(file, "RTG_LOAD_MODE   : %s/%s\n",
		grp->mode.util_enabled ? "true" : "false",
		grp->mode.freq_enabled ? "true" : "false");
	seq_printf_rtg(file, "RTG_INTERVAL    : UPDATE:%lums#INVALID:%lums\n",
		grp->freq_update_interval / NSEC_PER_MSEC,
		grp->util_invalid_interval / NSEC_PER_MSEC);
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	seq_printf_rtg(file, "RTG_CLUSTER     : %d\n",
		grp->preferred_cluster ? grp->preferred_cluster->id : -1);
#endif
}

#ifdef CONFIG_HW_RTG_FRAME
static void print_frame_info(struct seq_file *file,
	const struct related_thread_group *grp)
{
	struct frame_info *frame_info = (struct frame_info *)grp->private_data;

	u64 now = ktime_get_ns();
	u64 frame_end = grp->window_start;

	seq_printf_rtg(file, "RTG_ID          : %d(%s)\n", grp->id, "FRAME");
	seq_printf_rtg(file, "FRAME_INFO      : QOS:%u#MARGIN:%dSTATE:%lu\n",
		frame_info->qos_frame,
		frame_info->vload_margin,
		frame_info->status);
	seq_printf_rtg(file, "FRAME_CLAMP     : PREV:%u/%u#UTIL:%u/%u\n",
		frame_info->prev_min_util, frame_info->prev_max_util,
		frame_info->frame_min_util, frame_info->frame_max_util);
	seq_printf_rtg(file, "FRAME_LOAD_MODE : %s/%s\n",
		grp->mode.util_enabled ? "true" : "false",
		grp->mode.freq_enabled ? "true" : "false");
	seq_printf_rtg(file, "FRAME_INTERVAL  : UPDATE:%lums#INVALID:%lums\n",
		grp->freq_update_interval / NSEC_PER_MSEC,
		grp->util_invalid_interval / NSEC_PER_MSEC);
	seq_printf_rtg(file,
		"FRAME_TIMESTAMP : timestamp:%llu#now:%llu#delta:%llu\n",
		frame_end, now, now - frame_end);
	seq_printf_rtg(file, "FRAME_LAST_TIME : %llu/%llu\n",
		(unsigned long long)frame_info->prev_frame_exec,
		(unsigned long long)frame_info->prev_frame_time);
#ifdef CONFIG_HW_RTG_FRAME_USE_NORMALIZED_UTIL
	seq_printf_rtg(file, "FRAME_CLUSTER   : %d\n",
		grp->preferred_cluster ? grp->preferred_cluster->id : -1);
#endif
}
#else
static void print_frame_info(struct seq_file *file,
	const struct related_thread_group *grp)
{
}
#endif

#ifdef CONFIG_HW_RTG_AUX
static void print_aux_info(struct seq_file *file,
	const struct related_thread_group *grp)
{
	struct aux_info *aux_info = (struct aux_info *) grp->private_data;

	seq_printf_rtg(file, "RTG_ID          : %d(%s)\n", grp->id, "AUX");
	seq_printf_rtg(file, "AUX_INFO        : MIN_UTIL:%d##BOOST_UTIL:%d##PRIO:%d\n",
		aux_info->min_util,
		aux_info->boost_util,
		aux_info->prio);
	seq_printf_rtg(file, "AUX_LOAD_MODE   : %s/%s\n",
		grp->mode.util_enabled ? "true" : "false",
		grp->mode.freq_enabled ? "true" : "false");
	seq_printf_rtg(file, "AUX_INTERVAL    : UPDATE:%lums#INVALID:%lums\n",
		grp->freq_update_interval / NSEC_PER_MSEC,
		grp->util_invalid_interval / NSEC_PER_MSEC);
#ifdef CONFIG_HW_RTG_FRAME_USE_NORMALIZED_UTIL
	seq_printf_rtg(file, "AUX_CLUSTER     : %d\n",
		grp->preferred_cluster ? grp->preferred_cluster->id : -1);
#endif
}
#else
static void print_aux_info(struct seq_file *file,
	const struct related_thread_group *grp)
{
}
#endif

static char rtg_task_state_to_char(const struct task_struct *tsk)
{
	static const char state_char[] = "RSDTtXZPI";
	unsigned int tsk_state = READ_ONCE(tsk->state);
	unsigned int state = (tsk_state | tsk->exit_state) & TASK_REPORT;

	BUILD_BUG_ON_NOT_POWER_OF_2(TASK_REPORT_MAX);
	BUILD_BUG_ON(1 + ilog2(TASK_REPORT_MAX) != sizeof(state_char) - 1);

	if (tsk_state == TASK_IDLE)
		state = TASK_REPORT_IDLE;
	return state_char[fls(state)];
}

static inline void print_rtg_task_header(struct seq_file *file,
	const char *header, int run, int nr)
{
	seq_printf_rtg(file,
		"%s   : %d/%d\n"
		"STATE		COMM	   PID	PRIO	CPU\n"
		"---------------------------------------------------------\n",
		header, run, nr);
}

static inline void print_rtg_task(struct seq_file *file,
	const struct task_struct *tsk)
{
	seq_printf_rtg(file, "%5c %15s %5d %5d %5d(%*pbl)\n",
		rtg_task_state_to_char(tsk), tsk->comm, tsk->pid, tsk->prio,
		task_cpu(tsk), cpumask_pr_args(&tsk->cpus_allowed));
}

static void print_rtg_threads(struct seq_file *file,
	const struct related_thread_group *grp)
{
	struct task_struct *tsk = NULL;
	int nr_thread = 0;

	list_for_each_entry(tsk, &grp->tasks, grp_list) {
		nr_thread++;
	}

	if (!nr_thread)
		return;

	print_rtg_task_header(file, "FRAME_THREADS",
		grp->nr_running, nr_thread);
	list_for_each_entry(tsk, &grp->tasks, grp_list) {
		if (unlikely(!tsk))
			continue;
		get_task_struct(tsk);
		print_rtg_task(file, tsk);
		put_task_struct(tsk);
	}
	seq_printf_rtg(file, "---------------------------------------------------------\n");
}

static int sched_rtg_debug_show(struct seq_file *file, void *param)
{
	struct related_thread_group *grp = NULL;
	unsigned long flags;
	bool have_task = false;

	for_each_related_thread_group(grp) {
		if (unlikely(!grp)) {
			seq_printf_rtg(file, "IPROVISION RTG none\n");
			return 0;
		}

		raw_spin_lock_irqsave(&grp->lock, flags);
		if (list_empty(&grp->tasks)) {
			raw_spin_unlock_irqrestore(&grp->lock, flags);
			continue;
		}

		if (!have_task)
			have_task = true;

		seq_printf_rtg(file, "\n\n");
		if (is_frame_rtg(grp->id))
			print_frame_info(file, grp);
		else if (grp->id == DEFAULT_AUX_ID)
			print_aux_info(file, grp);
		else
			print_rtg_info(file, grp);
		print_rtg_threads(file, grp);
		raw_spin_unlock_irqrestore(&grp->lock, flags);
	}

	if (!have_task)
		seq_printf_rtg(file, "RTG tasklist empty\n");

	return 0;
}

static int sched_rtg_debug_release(struct inode *inode, struct file *file)
{
	seq_release(inode, file);
	return 0;
}

static int sched_rtg_debug_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sched_rtg_debug_show, NULL);
}

static const struct file_operations sched_rtg_debug_fops = {
	.open = sched_rtg_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = sched_rtg_debug_release,
};

static int __init init_sched_rtg_debug_procfs(void)
{
	struct proc_dir_entry *pe = NULL;

	pe = proc_create("sched_rtg_debug",
		0444, NULL, &sched_rtg_debug_fops);
	if (unlikely(!pe))
		return -ENOMEM;
	return 0;
}
late_initcall(init_sched_rtg_debug_procfs);
