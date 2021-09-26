/*
 * Huawei Spread Task File
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
#ifndef __SCHED_EAS_SPREAD_TASK_H__
#define __SCHED_EAS_SPREAD_TASK_H__
#include <linux/sched/topology.h>

bool need_spread_task(struct sched_domain *sd);
#endif
