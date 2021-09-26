/*
 * huawei_charge_time_debug.h
 *
 * The remaining time of charging debug.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#ifndef __HUAWEI_CHARGE_TIME_DEBUG_H__
#define __HUAWEI_CHARGE_TIME_DEBUG_H__

#include "huawei_charge_time.h"

#ifdef CONFIG_HW_CHARGE_TIME_DEBUG
int huawei_chg_time_debug_init(struct huawei_chg_time_device *di);
int huawei_chg_time_test_soc(void);
void huawei_chg_time_output_set(int valid, int sec);
#else
static inline int huawei_chg_time_debug_init(struct huawei_chg_time_device *di)
{
	return 0;
}

static inline int huawei_chg_time_test_soc(void)
{
	return 0;
}

static inline void huawei_chg_time_output_set(int valid, int sec)
{
}

#endif
#endif
