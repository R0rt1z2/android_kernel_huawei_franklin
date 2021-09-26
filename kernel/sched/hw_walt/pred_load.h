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

#ifndef __PRED_LOAD_H__
#define __PRED_LOAD_H__

#ifdef CONFIG_HW_SCHED_PRED_LOAD
#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
extern bool sched_predl;
#define predl_enable sched_predl
unsigned long predict_util(struct rq *rq);
unsigned long task_pred_util(struct task_struct *p);
static inline unsigned long max_pred_ls(struct rq *rq) { return 0; }
static inline unsigned long cpu_util_pred_min(struct rq *rq) { return 0; }
#else
bool use_pred_load(int cpu);
unsigned long predict_util(struct rq *rq);
unsigned long task_pred_util(struct task_struct *p);
unsigned long max_pred_ls(struct rq *rq);
unsigned long cpu_util_pred_min(struct rq *rq);
extern unsigned int predl_jump_load;
extern unsigned int predl_do_predict;
extern unsigned int predl_window_size;
extern unsigned int predl_enable;

extern void update_task_predl(struct task_struct *p,
			      struct rq *rq, int event, u64 wallclock);
static void fixup_pred_load(struct rq *rq, s64 pred_load_delta);
extern void update_predl_window_start(struct rq *rq, u64 wallclock);
#endif /*CONFIG_HAVE_QCOM_SCHED_WALT*/
#else
static inline bool use_pred_load(int cpu) { return false; }
static inline unsigned long predict_util(struct rq *rq) { return 0; }
static inline unsigned long task_pred_util(struct task_struct *p) { return 0; }
static inline unsigned long max_pred_ls(struct rq *rq) { return 0; }
static inline unsigned long cpu_util_pred_min(struct rq *rq) { return 0; }
#endif /*HW_SCHED_PRED_LOAD*/
#endif
