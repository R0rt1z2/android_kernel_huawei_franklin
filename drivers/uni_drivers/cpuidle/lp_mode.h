/*
 * lp_mode.h
 *
 * lowpower mode changing cpuidle parameter
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
#ifndef __LP_MODE_H
#define __LP_MODE_H

int get_lp_mode(void);
void cpuidle_switch_to_lp_mode(int enabled);

#endif
