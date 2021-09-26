/*
 * Huawei Energy Aware Scheduling Extern File
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
#ifndef __HW_EAS_SCHED_H__
#define __HW_EAS_SCHED_H__

#ifdef CONFIG_HW_EAS_SCHED
extern unsigned int sysctl_sched_force_upmigrate_duration;
extern void print_hung_task_sched_info(struct task_struct *p);
extern void print_cpu_rq_info(void);
extern bool task_fits_max(struct task_struct *p, int cpu);
extern void vip_balance_set_overutilized(int cpu);
extern void reset_balance_interval(int cpu);
#else
static inline void vip_balance_set_overutilized(int cpu) { }
static inline void reset_balance_interval(int cpu) { }
#endif

#endif
