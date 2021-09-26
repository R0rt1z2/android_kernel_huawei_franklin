/*
 * devfreq_cooling_hw.c
 *
 * hw devfreq cooling enhance
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
extern unsigned int g_ipa_freq_limit[];
extern unsigned int g_ipa_soc_freq_limit[];
extern unsigned int g_ipa_board_freq_limit[];
extern unsigned int g_ipa_board_state[];
extern unsigned int g_ipa_soc_state[];
extern int update_devfreq(struct devfreq *devfreq);
#endif

#ifdef CONFIG_HW_EXTERNAL_SENSOR
extern void hw_gpufreq_thermal_protect(unsigned int limit_idx);
#endif

#ifdef CONFIG_HW_IPA_THERMAL
extern int update_devfreq(struct devfreq *devfreq);
static int devfreq_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long state)
{
	struct devfreq_cooling_device *dfc = cdev->devdata;
	struct devfreq *df = dfc->devfreq;
	struct device *dev = df->dev.parent;
	unsigned long freq;
	unsigned long limit_state;
	int gpu_id = -1;
	int ret;

	gpu_id = ipa_get_actor_id("gpu");
	if (gpu_id < 0)
		return -EINVAL;
	if (g_ipa_soc_state[gpu_id] < dfc->freq_table_size)
		g_ipa_soc_freq_limit[gpu_id] =
		    dfc->freq_table[g_ipa_soc_state[gpu_id]];

	if (g_ipa_board_state[gpu_id] < dfc->freq_table_size)
		g_ipa_board_freq_limit[gpu_id] =
		    dfc->freq_table[g_ipa_board_state[gpu_id]];

	limit_state = max(g_ipa_soc_state[gpu_id], g_ipa_board_state[gpu_id]);
	if (limit_state < dfc->freq_table_size)
		state = max(state, limit_state);

	if (state == dfc->cooling_state)
		return 0;

	dev_dbg(dev, "Setting cooling state %lu\n", state);

	if (state == THERMAL_NO_LIMIT) {
		freq = 0;
	} else {
		if (state >= dfc->freq_table_size)
			return -EINVAL;

		freq = dfc->freq_table[state];
	}

	g_ipa_freq_limit[gpu_id] = freq;
	trace_IPA_actor_gpu_cooling(freq / 1000, state);

#ifdef CONFIG_HW_EXTERNAL_SENSOR
	df->profile->get_dev_status(df->dev.parent, &df->last_status);
	hw_gpufreq_thermal_protect((unsigned int)state);
#else
	if (df->max_freq != freq) {
		/* NOTE use devfreq_qos_set_max,because gpufreq not support VOTE */
		mutex_lock(&df->lock);
		ret = update_devfreq(df);
		mutex_unlock(&df->lock);
		if (ret)
			dev_dbg(dev, "update devfreq fail %d\n", ret);
	}
#endif
	dfc->cooling_state = state;

	return 0;
}

static void ipa_set_current_load_freq(struct thermal_cooling_device *cdev,
				      struct thermal_zone_device *tz,
				      u32 dyn_power, u32 static_power,
				      u32 * power)
{
	struct devfreq_cooling_device *dfc = cdev->devdata;
	struct devfreq *df = dfc->devfreq;
	struct devfreq_dev_status *status = &df->last_status;
	unsigned long load = 0;
	unsigned long freq = status->current_frequency;

	if (status->total_time)
		load = 100 * status->busy_time / status->total_time;
	if (tz->is_soc_thermal)
		trace_IPA_actor_gpu_get_power((freq / 1000), load, dyn_power,
					      static_power, *power);

	cdev->current_load = load;
	cdev->current_freq = freq;
}

static unsigned long get_static_power(struct devfreq_cooling_device *dfc,
				      unsigned long freq);
static unsigned long get_dynamic_power(struct devfreq_cooling_device *dfc,
				       unsigned long freq,
				       unsigned long voltage);
static void ipa_power_debug_print(struct devfreq_cooling_device *dfc,
				  unsigned long freq, unsigned long voltage)
{
	unsigned long power_static;
	unsigned long power_dyn;

	power_static = get_static_power(dfc, freq);
	power_dyn = get_dynamic_power(dfc, freq, voltage);
	pr_debug("%lu MHz @ %lu mV: %lu + %lu = %lu mW\n",
		 freq / 1000000, voltage,
		 power_dyn, power_static, power_dyn + power_static);
}
#endif
