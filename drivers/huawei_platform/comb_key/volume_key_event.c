/*
 * volume_key_event.c
 *
 * voleme key driver
 *
 * Copyright (c) 2020 Huawei Technologies Co., Ltd.
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
#include <linux/module.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <huawei_platform/comb_key/volume_key_event.h>

static ATOMIC_NOTIFIER_HEAD(volume_key_notifier_list);
static struct delayed_work volume_key_work;
static unsigned long volume_key_status;

int volume_key_register_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&volume_key_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(volume_key_register_notifier);

int volume_key_unregister_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&volume_key_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(volume_key_unregister_notifier);

int volume_key_call_notifiers(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&volume_key_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(volume_key_call_notifiers);

void volume_key_status_distinguish(unsigned long pressed)
{
	volume_key_status = pressed;
	pr_info("%s status = %lu\n", __func__, volume_key_status);
	queue_delayed_work(system_power_efficient_wq, &volume_key_work, 0);
}

static void volume_key_delayed_work(struct work_struct *work)
{
	(void)work;
	volume_key_call_notifiers(volume_key_status, NULL);
}

int volume_key_nb_init(void)
{
	pr_info("init\n");
	INIT_DELAYED_WORK(&volume_key_work, volume_key_delayed_work);
	return 0;
}
