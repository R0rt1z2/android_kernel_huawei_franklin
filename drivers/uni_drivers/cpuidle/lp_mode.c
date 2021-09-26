/*
 * lp_mode.c
 *
 * low power mode changing cpuidle parameter
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
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpuidle.h>

#include "cpuidle.h"

static int lp_mode_enabled = 0;

int get_lp_mode(void)
{
	return lp_mode_enabled;
}

void cpuidle_switch_to_lp_mode(int enabled)
{
	int cpu, i;
	struct cpuidle_device *dev = NULL;
	struct cpuidle_driver *drv = NULL;

	cpuidle_pause();

	spin_lock(&cpuidle_driver_lock);

	for_each_possible_cpu(cpu) {
		dev = per_cpu(cpuidle_devices, cpu);
		drv = cpuidle_get_cpu_driver(dev);
		if (!drv)
			continue;

		if (cpumask_first(drv->cpumask) != cpu)
			continue;

		/* state0 will be ignored */
		for (i = 1; i < drv->state_count; i++) {
			if (enabled && drv->states[i].lp_exit_latency >
			    drv->states[i].exit_latency)
				continue;

			if (!enabled && drv->states[i].lp_exit_latency <
			    drv->states[i].exit_latency)
				continue;

			swap(drv->states[i].lp_exit_latency, drv->states[i].exit_latency);
			swap(drv->states[i].lp_target_residency, drv->states[i].target_residency);
		}
	}
	lp_mode_enabled = enabled;
	spin_unlock(&cpuidle_driver_lock);

	cpuidle_resume();
}
