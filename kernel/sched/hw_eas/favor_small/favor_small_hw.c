/*
 * Huawei EAS System Control File
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
#ifdef CONFIG_HW_FAVOR_SMALL_CAP
#include <chipset_common/hwqos/iaware_qos.h>

extern unsigned long boosted_task_util(struct task_struct *task);

bool hw_favor_smaller_capacity(struct task_struct *p, int cpu)
{
	if (test_tsk_thread_flag(p, TIF_FAVOR_SMALL_CAP))
		return true;

	return (boosted_task_util(p) * 100) <
		(capacity_orig_of(cpu) * up_migration_util_filter);
}

#endif
