/*
 * cpuidle_notifier.c
 *
 * cpuidle enter/exit notify
 *
 * Copyright (c) 2015-2020 Huawei Technologies Co., Ltd.
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

#include <linux/init.h>
#include <linux/notifier.h>

struct atomic_notifier_head idle_status_notifier_list;
static bool init_idle_status_notifier_list_called;

int idle_status_notifier_register(struct notifier_block *nb)
{
	WARN_ON(!init_idle_status_notifier_list_called);
	return atomic_notifier_chain_register(&idle_status_notifier_list,
					      nb);
}

int idle_status_notifier_unregister(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&idle_status_notifier_list,
						nb);
}

static int __init init_idle_status_notifier_list(void)
{
	ATOMIC_INIT_NOTIFIER_HEAD(&idle_status_notifier_list);
	init_idle_status_notifier_list_called = true;
	return 0;
}
core_initcall(init_idle_status_notifier_list);
