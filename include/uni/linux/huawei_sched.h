/*
 * huawei_sched.h
 *
 * huawei sched related interfaces
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
#ifndef __HUAWEI_SCHED_H_
#define __HUAWEI_SCHED_H_

#ifdef CONFIG_HUAWEI_SCHED_VIP
int set_vip_prio(struct task_struct *p, unsigned int prio);
#endif

#ifdef CONFIG_HUAWEI_SCHED
int cpumask_test_fast_cpu(int cpu);
void cpumask_get_fast_cpus(struct cpumask *cpumask);
int cpumask_test_slow_cpu(int cpu);
void cpumask_get_slow_cpus(struct cpumask *cpumask);
#endif

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
int set_task_min_util(struct task_struct *p, unsigned int min_util);
unsigned int get_task_min_util(struct task_struct *p);

int set_task_max_util(struct task_struct *p, unsigned int max_util);
unsigned int get_task_max_util(struct task_struct *p);
#endif

#endif
