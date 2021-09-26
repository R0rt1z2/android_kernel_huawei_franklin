/*
 * perf_ctrl.h
 *
 * header file for perf_ctrl
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __PERF_CTRL_H__
#define __PERF_CTRL_H__

#include <linux/types.h>
#include <linux/hw/render_rt.h>
#include <linux/hw/sched_perf_ctrl.h>
#include <linux/sched/hw_rtg/rtg_perf_ctrl.h>
#include <linux/hw/hw_cpufreq_perf_ctrl.h>

#define PERF_CTRL_MAGIC 'x'

enum {
	GET_SCHED_STAT = 1,
	GET_RELATED_TID = 5,
	/* RTG frame */
	SET_FRAME_RATE = 8,
	SET_FRAME_MARGIN,
	SET_FRAME_STATUS,
	/* RTG sched */
	SET_TASK_RTG,
	SET_RTG_CPUS,
	SET_RTG_FREQ,
	SET_RTG_FREQ_UPDATE_INTERVAL,
	SET_RTG_UTIL_INVALID_INTERVAL,
	INIT_RENDER_PID = 17,
	GET_RENDER_RT,
	STOP_RENDER_RT,
	GET_RENDER_HT,
	DESTROY_RENDER_RT,

	SET_RTG_LOAD_MODE,
	SET_RTG_ED_PARAMS,
	GET_DEV_CAP = 24,
	SET_VIP_PRIO,
	SET_FAVOR_SMALL_CAP,
	/* uclamp util */
	SET_TASK_RTG_MIN_FREQ = 27,
	SET_TASK_MIN_UTIL,

	GET_CPU_BUSY_TIME = 34,
	/* ADD BELOW */

	PERF_CTRL_MAX_NR,
};

/* use cap to track the difference of the ioctl cmd */
enum {
	CAP_AI_SCHED_COMM_CMD = 0,
	CAP_RTG_CMD,
	CAP_RENDER_RT_CMD,
};

#define PERF_CTRL_SET_FAVOR_SMALL_CAP \
	_IOW(PERF_CTRL_MAGIC, SET_FAVOR_SMALL_CAP, struct task_config)

#define PERF_CTRL_GET_SCHED_STAT \
	_IOR(PERF_CTRL_MAGIC, GET_SCHED_STAT, struct sched_stat)

#define PERF_CTRL_GET_RELATED_TID \
	_IOWR(PERF_CTRL_MAGIC, GET_RELATED_TID, struct related_tid_info)

#define PERF_CTRL_GET_DEV_CAP \
	_IOR(PERF_CTRL_MAGIC, GET_DEV_CAP, unsigned long)

#define PERF_CTRL_SET_VIP_PRIO \
	_IOW(PERF_CTRL_MAGIC, SET_VIP_PRIO, struct task_config)

/* render related threads */
#define PERF_CTRL_INIT_RENDER_PID \
	_IOW(PERF_CTRL_MAGIC, INIT_RENDER_PID, struct render_init_paras)
#define PERF_CTRL_DESTROY_RENDER_RT \
	_IOW(PERF_CTRL_MAGIC, DESTROY_RENDER_RT, pid_t)
#define PERF_CTRL_STOP_RENDER_RT \
	_IOW(PERF_CTRL_MAGIC, STOP_RENDER_RT, struct render_stop)
#define PERF_CTRL_GET_RENDER_RT \
	_IOWR(PERF_CTRL_MAGIC, GET_RENDER_RT, struct render_rt)
#define PERF_CTRL_GET_RENDER_HT \
	_IOWR(PERF_CTRL_MAGIC, GET_RENDER_HT, struct render_ht)

#define PERF_CTRL_SET_TASK_RTG \
	_IOWR(PERF_CTRL_MAGIC, SET_TASK_RTG, struct rtg_group_task)
#define PERF_CTRL_SET_RTG_CPUS \
	_IOWR(PERF_CTRL_MAGIC, SET_RTG_CPUS, struct rtg_cpus)
#define PERF_CTRL_SET_RTG_FREQ \
	_IOWR(PERF_CTRL_MAGIC, SET_RTG_FREQ, struct rtg_freq)
#define PERF_CTRL_SET_RTG_FREQ_UPDATE_INTERVAL \
	_IOWR(PERF_CTRL_MAGIC, SET_RTG_FREQ_UPDATE_INTERVAL, struct rtg_interval)
#define PERF_CTRL_SET_RTG_UTIL_INVALID_INTERVAL \
	_IOWR(PERF_CTRL_MAGIC, SET_RTG_UTIL_INVALID_INTERVAL, struct rtg_interval)
#define PERF_CTRL_SET_RTG_LOAD_MODE \
	_IOW(PERF_CTRL_MAGIC, SET_RTG_LOAD_MODE, struct rtg_load_mode)
#define PERF_CTRL_SET_RTG_ED_PARAMS \
	_IOW(PERF_CTRL_MAGIC, SET_RTG_ED_PARAMS, struct rtg_ed_params)
#define PERF_CTRL_SET_TASK_RTG_MIN_FREQ \
	_IOW(PERF_CTRL_MAGIC, SET_TASK_RTG_MIN_FREQ, struct task_config)

#define PERF_CTRL_SET_FRAME_RATE \
	_IOWR(PERF_CTRL_MAGIC, SET_FRAME_RATE, int*)
#define PERF_CTRL_SET_FRAME_MARGIN \
	_IOWR(PERF_CTRL_MAGIC, SET_FRAME_MARGIN, int*)
#define PERF_CTRL_SET_FRAME_STATUS \
	_IOWR(PERF_CTRL_MAGIC, SET_FRAME_STATUS, int*)

#define PERF_CTRL_SET_TASK_MIN_UTIL \
	_IOW(PERF_CTRL_MAGIC, SET_TASK_MIN_UTIL, struct task_config)
#define PERF_CTRL_GET_CPU_BUSY_TIME \
	_IOW(PERF_CTRL_MAGIC, GET_CPU_BUSY_TIME, struct cpu_busy_time)

#endif /*__PERF_CTRL_H__*/
