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

struct task_config {
	pid_t pid;
	unsigned int value;
};

#ifdef CONFIG_SCHED_INFO
int perf_ctrl_get_sched_stat(void __user *uarg);
#else
static inline int perf_ctrl_get_sched_stat(void __user *uarg)
{
	return -EFAULT;
}
#endif

#ifdef CONFIG_HW_UCLAMP_MIN_UTIL
int perf_ctrl_set_task_min_util(void __user *uarg);
#else
static inline int perf_ctrl_set_task_min_util(void __user *uarg)
{
	return -EFAULT;
}
#endif

#ifdef CONFIG_HW_UCLAMP_MAX_UTIL
int perf_ctrl_set_task_max_util(void __user *uarg);
#else
static inline int perf_ctrl_set_task_max_util(void __user *uarg)
{
	return -EFAULT;
}
#endif

#ifdef CONFIG_VIP_SCHED_OPT
int perf_ctrl_set_vip_prio(void __user *uarg);
#else
static inline int perf_ctrl_set_vip_prio(void __user *uarg)
{
	return -EFAULT;
}
#endif
#endif

