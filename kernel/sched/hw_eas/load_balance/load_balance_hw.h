/*
 * Huawei Load Balance File
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
#ifndef __SCHED_EAS_LOAD_BALANCE_H__
#define __SCHED_EAS_LOAD_BALANCE_H__

#ifdef CONFIG_HW_EAS_SCHED
int hw_find_new_ilb(void);
int find_max_idle_cpu(struct task_struct *p);
struct task_struct *hw_get_heaviest_task(struct task_struct *p, int cpu);
void check_force_upmigrate(struct task_struct *p, struct rq *rq);
#endif

#endif
