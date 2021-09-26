/*
 * wireless_charge_ictrl.h
 *
 * wireless charger power iout control module
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

#ifndef _WIRELESS_CHARGE_ICTRL_H_
#define _WIRELESS_CHARGE_ICTRL_H_

#define CHARGE_CURRENT_DELAY 100

#ifdef CONFIG_WIRELESS_CHARGER
void wlc_ictrl_set_input_current(int iin);
void wlc_set_rx_iout_limit(int ilim);
void wlc_set_iin_prop(int iin_step, int iin_delay);
int wlc_get_charger_iinlim_regval(void);
#else
static inline void wlc_ictrl_set_input_current(int iin) { }
static inline void wlc_set_rx_iout_limit(int ilim) { }
static inline void wlc_set_iin_prop(int iin_step, int iin_delay) { }
static inline int wlc_get_charger_iinlim_regval(void)
{
	return 0;
}
#endif /* CONFIG_WIRELESS_CHARGER */

#endif /* _WIRELESS_CHARGE_ICTRL_H_ */
