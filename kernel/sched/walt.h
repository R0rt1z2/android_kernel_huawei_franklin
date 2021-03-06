/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef __WALT_H
#define __WALT_H

#ifdef CONFIG_SCHED_WALT

#include "hw_walt/walt_hw.h"

#define WINDOW_STATS_RECENT		0
#define WINDOW_STATS_MAX		1
#define WINDOW_STATS_MAX_RECENT_AVG	2
#define WINDOW_STATS_AVG		3
#define WINDOW_STATS_INVALID_POLICY	4

void walt_update_task_ravg(struct task_struct *p, struct rq *rq, int event,
		u64 wallclock, u64 irqtime);
void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);
void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);

void walt_fixup_busy_time(struct task_struct *p, int new_cpu);
void walt_init_new_task_load(struct task_struct *p);
void walt_mark_task_starting(struct task_struct *p);
void walt_set_window_start(struct rq *rq, struct rq_flags *rf);
void walt_migrate_sync_cpu(int cpu);
u64 walt_ktime_clock(void);
void walt_account_irqtime(int cpu, struct task_struct *curr, u64 delta,
                                  u64 wallclock);

u64 walt_irqload(int cpu);
int walt_cpu_high_irqload(int cpu);

#define NEW_TASK_WINDOWS 5
static inline bool is_new_task(struct task_struct *p)
{
	return p->ravg.active_windows < NEW_TASK_WINDOWS;
}

extern unsigned long capacity_curr_of(int cpu);
/*
 * Translate absolute delta time accounted on a CPU
 * to a scale where 1024 is the capacity of the most
 * capable CPU running at FMAX
 */
static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	unsigned long capcurr = capacity_curr_of(cpu_of(rq));

	return (delta * capcurr) >> SCHED_CAPACITY_SHIFT;;
}

static inline u64 scale_exec_time_limit(u64 delta, struct rq *rq, s32 limit)
{
	return min_t(u64, scale_exec_time(delta, rq), limit);
}

extern unsigned int sysctl_sched_walt_init_task_load_pct;

#else /* CONFIG_SCHED_WALT */

static inline void walt_update_task_ravg(struct task_struct *p, struct rq *rq,
		int event, u64 wallclock, u64 irqtime) { }
static inline void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_fixup_busy_time(struct task_struct *p, int new_cpu) { }
static inline void walt_init_new_task_load(struct task_struct *p) { }
static inline void walt_mark_task_starting(struct task_struct *p) { }
static inline void walt_set_window_start(struct rq *rq, struct rq_flags *rf) { }
static inline void walt_migrate_sync_cpu(int cpu) { }
static inline u64 walt_ktime_clock(void) { return 0; }

#define walt_cpu_high_irqload(cpu) false
#define is_new_task(p) false

#endif /* CONFIG_SCHED_WALT */

#if defined(CONFIG_CFS_BANDWIDTH) && defined(CONFIG_SCHED_WALT)
void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
#else
static inline void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
static inline void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
#endif

extern bool walt_disabled;

#endif
