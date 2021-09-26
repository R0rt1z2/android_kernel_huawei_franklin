/*
 * Huawei Boost Declaration
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
#ifndef __SCHED_EAS_BOOST_HW_H__
#define __SCHED_EAS_BOOST_HW_H__

#ifdef CONFIG_HW_BOOST
#include <linux/sched.h>

extern unsigned int task_boost_limit;
bool hw_boost_state(bool boosetd);

#ifdef CONFIG_HW_BOOT_BOOST
extern int boot_boost;
#endif

#ifdef CONFIG_HW_GLOBAL_BOOST
extern int global_boost_enable;
#endif

#ifdef CONFIG_HW_FORK_BOOST
bool task_should_forkboost(struct task_struct *p);
#endif

#endif

#endif
