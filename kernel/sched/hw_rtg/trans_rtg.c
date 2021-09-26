/*
 * trans_rtg.c
 *
 * trans rtg thread
 *
 * Copyright (c) 2019-2021 Huawei Technologies Co., Ltd.
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

#include "include/trans_rtg.h"

#include <linux/sched.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>
#include <../kernel/sched/sched.h>
#include <trace/events/sched.h>

#include "include/iaware_rtg.h"
#include "include/set_rtg.h"
#include "include/rtg_sched.h"
#include "include/frame.h"

#ifdef CONFIG_VIP_SCHED_OPT
#include <chipset_common/hwcfs/hwcfs_common.h>
#endif

enum SCHED_CLASS {
	RT_SCHED,
	FAIR_SCHED,
	INVALID_SCHED,
};

enum OPRATION_TYPE {
	OPERATION_RTG_ENQUEUE,
	OPERATION_RTG_DEQUEUE,
};

static int g_trans_depth = DEFAULT_TRANS_DEPTH;
static int g_max_thread_num = DEFAULT_MAX_THREADS;
static atomic_t g_rtg_thread_num = ATOMIC_INIT(0);

void set_trans_config(int depth, int max_threads)
{
	if ((depth != INVALID_VALUE) && (depth >= 0))
		g_trans_depth = depth;
	if ((max_threads != INVALID_VALUE) && (max_threads >= 0))
		g_max_thread_num = max_threads;
}

static bool is_depth_valid(const struct task_struct *from)
{
	if (g_trans_depth != DEFAULT_TRANS_DEPTH && g_trans_depth <= 0)
		return false;

	if ((g_trans_depth > 0) && (from->rtg_depth != STATIC_RTG_DEPTH) &&
		(from->rtg_depth >= g_trans_depth))
		return false;

	// if from is not in rtg, invalid
	if (from->rtg_depth == 0)
		return false;

	return true;
}

static void trans_vip_and_min_util(struct task_struct *task,
	struct task_struct *from)
{
#ifdef CONFIG_VIP_SCHED_OPT
	task->restore_params.vip_prio = task->vip_prio;
	if (from->vip_prio > task->vip_prio)
		set_thread_vip_prio(task, from->vip_prio);
#endif

#if defined(CONFIG_HW_RTG_FRAME_USE_MIN_UTIL) && !defined(CONFIG_HUAWEI_SCHED_VIP) && \
    defined(CONFIG_SCHED_TASK_UTIL_CLAMP)
	task->restore_params.min_util = task->util_req.min_util;
	if (from->util_req.min_util > task->util_req.min_util)
		set_task_min_util(task, from->util_req.min_util);
#endif
}

void trans_rtg_sched_enqueue(struct task_struct *task,
	struct task_struct *from, unsigned int type)
{
	int ret;
	unsigned int grpid;
	int prio;

#ifdef CONFIG_HW_QOS_THREAD
	if (!task || !from || type != DYNAMIC_QOS_BINDER)
		return;
#endif

	if (!is_depth_valid(from))
		return;

	// if task is already rtg, return
	if (task->rtg_depth == STATIC_RTG_DEPTH || task->rtg_depth > 0)
		return;

	if (from->rtg_depth == STATIC_RTG_DEPTH)
		task->rtg_depth = 1;
	else
		task->rtg_depth = from->rtg_depth + 1;

	if (get_enable_type() == TRANS_ENABLE)
		return;

	grpid = sched_get_group_id(from);
	if (is_frame_rtg(grpid)) {
		if (g_max_thread_num != DEFAULT_MAX_THREADS &&
			atomic_read(&g_rtg_thread_num) >= g_max_thread_num) {
			task->rtg_depth = 0;
			return;
		}
		prio = get_frame_prio_by_id(grpid);
	} else if (grpid == DEFAULT_AUX_ID) {
		prio = (from->prio < MAX_RT_PRIO ? from->prio : NOT_RT_PRIO);
	} else {
		task->rtg_depth = 0;
		return;
	}

	get_task_struct(task);
	ret = set_rtg_sched(task, true, grpid, prio, true);
	if (ret < 0) {
		task->rtg_depth = 0;
		put_task_struct(task);
		return;
	}

	trans_vip_and_min_util(task, from);
	if (is_frame_rtg(grpid))
		atomic_inc(&g_rtg_thread_num);

	put_task_struct(task);

	trace_sched_rtg(task, from, type, OPERATION_RTG_ENQUEUE);
}

void trans_rtg_sched_dequeue(struct task_struct *task, unsigned int type)
{
	int ret;
	unsigned int grpid;
	int depth;

#ifdef CONFIG_HW_QOS_THREAD
	if (!task || type != DYNAMIC_QOS_BINDER)
		return;
#endif

	// if task is not rtg or task is orig task, return
	if (task->rtg_depth == 0 || task->rtg_depth == STATIC_RTG_DEPTH)
		return;

	depth = task->rtg_depth;
	task->rtg_depth = 0;

	if (get_enable_type() == TRANS_ENABLE)
		return;

	get_task_struct(task);
	ret = set_rtg_sched(task, false, 0, DEFAULT_RT_PRIO, true);
	if (ret < 0) {
		task->rtg_depth = depth;
		put_task_struct(task);
		return;
	}

#ifdef CONFIG_VIP_SCHED_OPT
	set_thread_vip_prio(task, task->restore_params.vip_prio);
	task->restore_params.vip_prio = 0;
#endif
#if defined(CONFIG_HW_RTG_FRAME_USE_MIN_UTIL) && !defined(CONFIG_HUAWEI_SCHED_VIP) && \
    defined(CONFIG_SCHED_TASK_UTIL_CLAMP)
	set_task_min_util(task, task->restore_params.min_util);

	task->restore_params.min_util = 0;
#endif

	grpid = sched_get_group_id(task);
	if (is_frame_rtg(grpid) &&
		(atomic_read(&g_rtg_thread_num) > 0))
		atomic_dec(&g_rtg_thread_num);

	put_task_struct(task);

	trace_sched_rtg(task, NULL, type, OPERATION_RTG_DEQUEUE);
}
