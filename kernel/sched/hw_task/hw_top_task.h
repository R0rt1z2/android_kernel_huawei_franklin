/*
 * hw_top_task.h
 *
 * huawei top task header file
 *
 * Copyright (c) 2019, Huawei Technologies Co., Ltd.
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

#ifndef __HW_TOP_TASK_H__
#define __HW_TOP_TASK_H__

#include <linux/sched.h>
#include "sched.h"
#include "walt.h"

#if defined(CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL) || defined(CONFIG_HW_MIGRATION_NOTIFY)
/* Is frequency of two cpus synchronized with each other? */
static inline int same_freq_domain(int src_cpu, int dst_cpu)
{
	struct rq *rq = cpu_rq(src_cpu);

	if (src_cpu == dst_cpu)
		return 1;

	return cpumask_test_cpu(dst_cpu, &rq->freq_domain_cpumask);
}
#else
static inline int same_freq_domain(int src_cpu, int dst_cpu) { return 0; }
#endif

#ifdef CONFIG_HW_TOP_TASK
#define DEFAULT_TOP_TASK_HIST_SIZE		RAVG_HIST_SIZE_MAX
#define DEFAULT_TOP_TASK_STATS_POLICY		WINDOW_STATS_RECENT
#define DEFAULT_TOP_TASK_STATS_EMPTY_WINDOW	false

unsigned long top_task_util(struct rq *rq);
void top_task_exit(struct task_struct *p, struct rq *rq);

struct top_task_entry {
	u8 count, preferidle_count;
};

#else /* !CONFIG_HW_TOP_TASK */
static inline unsigned long top_task_util(struct rq *rq) { return 0; }
static inline void top_task_exit(struct task_struct *p, struct rq *rq) {}
#endif /* CONFIG_HW_TOP_TASK */

#endif
