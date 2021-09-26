/*
 * cpu_cooling_hw.c
 *
 * hw cpu cooling enhance
 *
 * Copyright (C) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifdef CONFIG_HW_IPA_THERMAL
#include <trace/events/thermal_power_allocator.h>
#ifdef CONFIG_HW_THERMAL_SPM
extern unsigned int get_powerhal_profile(int actor);
extern unsigned int get_minfreq_profile(int actor);
extern bool is_spm_mode_enabled(void);

u32 profile_freq[CAPACITY_OF_ARRAY];
int hw_calc_static_power(const struct cpumask *cpumask, int temp,
			 unsigned long u_volt, u32 * static_power);
#endif
extern unsigned int g_ipa_freq_limit[];
extern unsigned int g_ipa_soc_freq_limit[];
extern unsigned int g_ipa_board_freq_limit[];
extern unsigned int g_ipa_board_state[];
extern unsigned int g_ipa_soc_state[];
#endif

#ifdef CONFIG_HW_IPA_THERMAL
static void ipa_calc_static_power(struct cpufreq_cooling_device *cpufreq_cdev,
				  u32 voltage_mv, u32 freq_mhz, u64 freq_power)
{
	u32 static_power;
	int nr_cpus;

	nr_cpus = (int)cpumask_weight(cpufreq_cdev->policy->related_cpus);
	if (nr_cpus == 0)
		nr_cpus = 1;
	cpufreq_cdev->plat_get_static_power(cpufreq_cdev->policy->related_cpus,
					    0,
					    (unsigned long)(voltage_mv * 1000),
					    &static_power);

	/* hw static_power givern in cluster */
	static_power = static_power / (u32) nr_cpus;

	pr_err("  %u MHz @ %u mV :  %u + %u = %u mW\n",
	       freq_mhz, voltage_mv, freq_power, static_power,
	       freq_power + static_power);
}

static bool ipa_check_dev_offline(struct device *dev, u32 * power)
{
	if (dev == NULL) {
		*power = 0;
		return true;
	}

	device_lock(dev);
	if (dev->offline == true) {
		*power = 0;
		device_unlock(dev);
		return true;
	}
	device_unlock(dev);
	return false;
}

static unsigned long ipa_cpufreq_set_cur_state(struct cpufreq_cooling_device
					       *cpufreq_cdev,
					       unsigned long state)
{
	unsigned int cpu = cpumask_any(cpufreq_cdev->policy->related_cpus);
	unsigned int cur_cluster;
	unsigned long limit_state;

	cur_cluster = (unsigned int)topology_physical_package_id(cpu);

	if (g_ipa_soc_state[cur_cluster] <= cpufreq_cdev->max_level)
		g_ipa_soc_freq_limit[cur_cluster] =
		    cpufreq_cdev->freq_table[g_ipa_soc_state[cur_cluster]].
		    frequency;

	if (g_ipa_board_state[cur_cluster] <= cpufreq_cdev->max_level)
		g_ipa_board_freq_limit[cur_cluster] =
		    cpufreq_cdev->freq_table[g_ipa_board_state[cur_cluster]].
		    frequency;

	limit_state =
	    max(g_ipa_soc_state[cur_cluster], g_ipa_board_state[cur_cluster]);

	/* only change new state when limit_state less than max_level */
	if (!WARN_ON(limit_state > cpufreq_cdev->max_level))
		state = max(state, limit_state);

	return state;
}

static void ipa_set_freq_limit(struct cpufreq_cooling_device *cpufreq_cdev,
			       unsigned int clip_freq)
{
	unsigned int cpu = cpumask_any(cpufreq_cdev->policy->related_cpus);
	unsigned int cur_cluster =
	    (unsigned int)topology_physical_package_id(cpu);

	g_ipa_freq_limit[cur_cluster] = clip_freq;
}
#endif

#ifdef CONFIG_HW_THERMAL_SPM
int cpufreq_update_policies(void)
{
	struct cpufreq_cooling_device *cpufreq_cdev = NULL;
	unsigned int cpus[g_cluster_num];
	int i;
	int num = 0;

	mutex_lock(&cooling_cpufreq_lock);
	list_for_each_entry(cpufreq_cdev, &cpufreq_cdev_list, node) {
		if (num >= (int)g_cluster_num)
			break;
		cpus[num] = cpumask_any(cpufreq_cdev->policy->related_cpus);
		num++;
	}
	mutex_unlock(&cooling_cpufreq_lock);

	for (i = 0; i < num; i++)
		cpufreq_update_policy(cpus[i]);

	return 0;
}
EXPORT_SYMBOL(cpufreq_update_policies);
#endif
