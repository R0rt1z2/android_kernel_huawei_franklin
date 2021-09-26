/*
 * sched_perf_ctrl.h
 *
 * sched perf ctrl header file

 * Copyright (c) 2020, Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SCHED_PERF_CTRL_H
#define __SCHED_PERF_CTRL_H

struct sched_stat {
	pid_t pid;
	unsigned long long sum_exec_runtime;
	unsigned long long run_delay;
	unsigned long pcount;
};

struct sched_task_config {
	pid_t pid;
	unsigned int value;
};

#if defined(CONFIG_SCHED_INFO) && defined(CONFIG_SCHED_PERF_CTRL)
int perf_ctrl_get_sched_stat(void __user *uarg);
#else
static inline int perf_ctrl_get_sched_stat(void __user *uarg)
{
	return -EFAULT;
}
#endif

static inline int perf_ctrl_set_task_util(void __user *uarg)
{
	return 0;
}

#ifdef CONFIG_SCHED_PERF_CTRL
int perf_ctrl_get_related_tid(void __user *uarg);
#else
static inline int perf_ctrl_get_related_tid(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HUAWEI_SCHED_VIP) && defined(CONFIG_SCHED_PERF_CTRL)
int perf_ctrl_set_vip_prio(void __user *uarg);
#else
static inline int perf_ctrl_set_vip_prio(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HW_EAS_SCHED) && defined(CONFIG_SCHED_PERF_CTRL)
int perf_ctrl_set_favor_small_cap(void __user *uarg);
#else
static inline int perf_ctrl_set_favor_small_cap(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_SCHED_TASK_UTIL_CLAMP) && defined(CONFIG_SCHED_PERF_CTRL)
int perf_ctrl_set_task_min_util(void __user *uarg);
int perf_ctrl_set_task_max_util(void __user *uarg);
#else
static inline int perf_ctrl_set_task_min_util(void __user *uarg)
{
	return -EFAULT;
}

static inline int perf_ctrl_set_task_max_util(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HUAWEI_SCHED_STAT_YIELD) && defined(CONFIG_SCHED_PERF_CTRL)
int perf_ctrl_get_task_yield_time(void __user *uarg);
#else
static inline int perf_ctrl_get_task_yield_time(void __user *uarg)
{
	return -EFAULT;
}
#endif

#endif
