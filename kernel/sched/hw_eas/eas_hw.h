/*
 * Huawei Energy Aware Scheduling File
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
#ifndef __SCHED_HW_EAS_SCHED_H__
#define __SCHED_HW_EAS_SCHED_H__

#include "boost/boost_hw.h"
#include "multi_margin/multi_margin_hw.h"
#include "load_balance/load_balance_hw.h"
#include "favor_small/favor_small_hw.h"
#include "spread/spread_hw.h"

#ifdef CONFIG_HW_EAS_SCHED
extern unsigned int up_migration_util_filter;

bool cpu_overutilized(int cpu);
bool cpu_overutilized_for_lb(int cpu);
bool hw_favor_smaller_capacity(struct task_struct *p, int cpu);
void check_prev_cpu_in_candidates(struct energy_env *eenv);
int find_slow_cpu(struct task_struct *p);
int find_boost_cpu(struct cpumask *group_cpus, struct task_struct *p, int this_cpu);
int wake_to_idle(struct task_struct *p);
unsigned long capacity_min_of(int cpu);
void mark_reserved(int cpu);
void clear_reserved(int cpu);

static __always_inline
unsigned long arch_scale_min_freq_capacity(struct sched_domain *sd, int cpu)
{
	/*
	 * Multiplied with any capacity value, this scale factor will return
	 * 0, which represents an un-capped state
	 */
	return 0;
}

#endif
#endif
