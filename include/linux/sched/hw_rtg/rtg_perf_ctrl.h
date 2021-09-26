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

#ifndef __RTG_PERF_CTRL_H__
#define __RTG_PERF_CTRL_H__

struct rtg_group_task {
	pid_t pid;
	unsigned int grp_id;
	bool pmu_sample_enabled;
};

struct rtg_cpus {
	unsigned int grp_id;
	int cluster_id;
};

struct rtg_freq {
	unsigned int grp_id;
	unsigned int freq;
};

struct rtg_interval {
	unsigned int grp_id;
	unsigned int interval;
};

/* rtg:load_mode */
struct rtg_load_mode {
	unsigned int grp_id;
	unsigned int freq_enabled;
	unsigned int util_enabled;
};

/* rtg: ed_params */
struct rtg_ed_params {
	unsigned int grp_id;
	unsigned int enabled;
	unsigned int running_ns;
	unsigned int waiting_ns;
	unsigned int nt_running_ns;
};


#if defined(CONFIG_HW_RTG) && defined(CONFIG_HW_RTG_PERF_CTRL)
int perf_ctrl_set_task_rtg(void __user *uarg);
int perf_ctrl_set_rtg_freq_update_interval(void __user *uarg);
int perf_ctrl_set_rtg_util_invalid_interval(void __user *uarg);
int perf_ctrl_set_rtg_load_mode(void __user *uarg);
#else
static inline int perf_ctrl_set_task_rtg(void __user *uarg)
{
	return -EFAULT;
}

static inline int perf_ctrl_set_rtg_freq_update_interval(void __user *uarg)
{
	return -EFAULT;
}
static inline int perf_ctrl_set_rtg_util_invalid_interval(void __user *uarg)
{
	return -EFAULT;
}
static inline int perf_ctrl_set_rtg_load_mode(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HW_RTG_NORMALIZED_UTIL) && defined(CONFIG_HW_RTG_PERF_CTRL)
int perf_ctrl_set_rtg_cpus(void __user *uarg);
int perf_ctrl_set_rtg_freq(void __user *uarg);
#else
static inline int perf_ctrl_set_rtg_cpus(void __user *uarg)
{
	return -EFAULT;
}
static inline int perf_ctrl_set_rtg_freq(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HW_RTG_NORMALIZED_UTIL) && defined(CONFIG_HW_RTG_PERF_CTRL)
int perf_ctrl_set_task_rtg_min_freq(void __user *uarg);
#else
static inline int perf_ctrl_set_task_rtg_min_freq(void __user *uarg)
{
	return -EFAULT;
}
#endif


#if defined(CONFIG_HW_ED_TASK) && defined(CONFIG_HW_RTG_PERF_CTRL)
int perf_ctrl_set_rtg_ed_params(void __user *uarg);
#else
static inline int perf_ctrl_set_rtg_ed_params(void __user *uarg)
{
	return -EFAULT;
}
#endif

#if defined(CONFIG_HW_RTG_FRAME) && defined(CONFIG_HW_RTG_PERF_CTRL)
int perf_ctrl_set_frame_rate(void __user *uarg);
int perf_ctrl_set_frame_margin(void __user *uarg);
int perf_ctrl_set_frame_status(void __user *uarg);
#else
static inline int perf_ctrl_set_frame_rate(void __user *uarg)
{
	return -EFAULT;
}
static inline int perf_ctrl_set_frame_margin(void __user *uarg)
{
	return -EFAULT;
}
static inline int perf_ctrl_set_frame_status(void __user *uarg)
{
	return -EFAULT;
}
#endif

#endif	/* __RTG_PERF_CTRL_H__ */
