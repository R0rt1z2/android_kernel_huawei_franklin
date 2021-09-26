/*
 * dp_aux_switch.h
 *
 * dp aux switch driver
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

#ifndef __DP_AUX_SWITCH_H__
#define __DP_AUX_SWITCH_H__

#include <linux/types.h>

#ifdef CONFIG_DP_AUX_SWITCH
void dp_aux_switch_op(uint32_t value);
void dp_aux_uart_switch_enable(void);
void dp_aux_uart_switch_disable(void);
bool is_need_define_notifier_call(void);
#else
static inline void dp_aux_switch_op(uint32_t value) {}
static inline void dp_aux_uart_switch_enable(void) {}
static inline void dp_aux_uart_switch_disable(void) {}
static inline bool is_need_define_notifier_call(void)
{
	return false;
}
#endif

#endif // __DP_AUX_SWITCH_H__
