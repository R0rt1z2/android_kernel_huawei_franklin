/*
 * util_clamp.h
 *
 * task util limit header file
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#ifndef __UTIL_CLAMP_H
#define __UTIL_CLAMP_H

struct rq;

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
extern bool update_rq_min_util_req(struct task_struct *p,
				   struct rq *rq, unsigned int new_util);
extern unsigned int get_min_util(struct rq *rq);
extern void add_freq_request(struct rq *rq, struct task_struct *p);
extern void del_freq_request(struct task_struct *p);
extern int set_task_min_util(struct task_struct *p, unsigned int new_util);
#else
static inline bool update_rq_min_util_req(struct task_struct *p,
					  struct rq *rq, unsigned int new_util)
{
	return false;
}

static inline int get_min_util(struct rq *rq)
{
	return 0;
}

static inline void add_freq_request(struct rq *rq, struct task_struct *p) {}
static inline void del_freq_request(struct task_struct *p) {}
static inline int set_task_min_util(struct task_struct *p, unsigned int new_util)
{
	return 0;
}
#endif

#endif
