/*
 * aux.h
 *
 * aux grp header
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

#ifndef AUX_EXTERN_H
#define AUX_EXTERN_H

#include "aux_info.h"

#ifdef CONFIG_HW_RTG
int sched_rtg_aux(int tid, int enable, const struct aux_info *info);
int set_aux_boost_util(int util);
#else
static inline
int sched_rtg_aux(int tid, int enable, const struct aux_info *info)
{
	return 0;
}

static inline int set_aux_boost_util(int util)
{
	return 0;
}
#endif

#endif
