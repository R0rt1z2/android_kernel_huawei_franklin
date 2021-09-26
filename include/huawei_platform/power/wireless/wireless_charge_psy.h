/*
 * wireless_charge_psy.h
 *
 * wireless charger power supply module
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#ifndef _WIRELESS_CHARGE_PSY_H_
#define _WIRELESS_CHARGE_PSY_H_

#include <linux/slab.h>

#ifdef CONFIG_WIRELESS_CHARGER
void wlc_handle_sink_event(bool sink_flag);
int wlc_power_supply_register(struct platform_device *pdev);
#else
static inline void wlc_handle_sink_event(bool sink_flag) { }
static inline int wlc_power_supply_register(struct platform_device *pdev)
{
	return -1;
}
#endif /* CONFIG_WIRELESS_CHARGER */

#endif /* _WIRELESS_CHARGE_PSY_H_ */
