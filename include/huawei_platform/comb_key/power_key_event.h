/*
 * power_key_event driver
 *
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

#ifndef _POWER_KEY_EVENT_H
#define _POWER_KEY_EVENT_H
#include  <linux/notifier.h>

enum power_key_press_staus {
	POWER_KEY_PRESS_RELEASE,
	POWER_KEY_PRESS_DOWN,
	POWER_KEY_MAX_STATUS
};

int power_key_register_notifier(struct notifier_block *nb);
int power_key_unregister_notifier(struct notifier_block *nb);
int power_key_call_notifiers(unsigned long val, void *v);
void power_key_status_distinguish(unsigned long pressed);
int power_key_nb_init(void);

#endif
