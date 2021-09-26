/*
 * Huawei Boost File
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
#ifdef CONFIG_HW_BOOST

#include "boost_hw.h"

unsigned int task_boost_limit = SCHED_CAPACITY_SCALE; /* default no limit */

#ifdef CONFIG_HW_BOOT_BOOST
int boot_boost = 1;
#endif

#ifdef CONFIG_HW_GLOBAL_BOOST
int global_boost_enable = 1;
#endif

bool hw_boost_state(bool boosted)
{
	bool state = boosted;

#ifdef CONFIG_HW_GLOBAL_BOOST
	state = state && global_boost_enable;
#endif

#ifdef CONFIG_HW_BOOT_BOOST
	state = state || boot_boost;
#endif

	return state;
}

#ifdef CONFIG_HW_FORK_BOOST
bool task_should_forkboost(struct task_struct *p)
{
	if (p->parent && p->parent->pid <= 2) {
		return false;
	}

	return true;
}
#endif

#endif
