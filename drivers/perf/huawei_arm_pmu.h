/*
 * huawei_arm_pmu.h
 *
 * huawei arm pmu process
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#ifndef __HUAWEI_ARM_PMU_H
#define __HUAWEI_ARM_PMU_H

#define USE_CPUHP_STATE CPUHP_AP_PERF_ARM_STARTING
#define USE_CPUHP_STR "AP_PERF_ARM_STARTING"

DECLARE_PER_CPU(int, pmu_irq);

struct cpu_pm_pmu_args {
	struct arm_pmu	*armpmu;
	unsigned long	cmd;
	int		cpu;
	int		ret;
};

static void cpu_pm_pmu_common(void *info);
static int cpu_pm_pmu_notify(struct notifier_block *b, unsigned long cmd,
			     void *v);
static int arm_perf_starting_cpu(unsigned int cpu, struct hlist_node *node);
static int arm_perf_stopping_cpu(unsigned int cpu, struct hlist_node *node);
#endif
