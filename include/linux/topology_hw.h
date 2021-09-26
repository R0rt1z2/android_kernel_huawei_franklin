/*
 * Huawei CPU Topology
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
#ifndef __TOPOLOGY_HW_H__
#define __TOPOLOGY_HW_H__

#ifdef CONFIG_ARCH_HW

#include <linux/cpumask.h>

extern struct cpumask slow_cpu_mask;
extern struct cpumask fast_cpu_mask;

int hw_test_fast_cpu(int cpu);
void hw_get_fast_cpus(struct cpumask *cpumask);
int hw_test_slow_cpu(int cpu);
void hw_get_slow_cpus(struct cpumask *cpumask);
void __init arch_get_fast_and_slow_cpus(struct cpumask *fast,
					struct cpumask *slow);

#endif

#endif
