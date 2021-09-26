/*
 * hw_ipa_gpu.c
 *
 * hw ipa for gpufreq driver
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

#ifdef CONFIG_DEVFREQ_THERMAL
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/thermal.h>
#include <ged_dvfs.h>

extern int tscpu_curr_gpu_temp;

static unsigned long
hw_model_static_power(struct devfreq *devfreq __maybe_unused,
	unsigned long voltage)
{
	/* refer to __mt_gpufreq_calculate_power */
	int p_leakage = 0;

#if MT_GPUFREQ_STATIC_PWR_READY2USE
	p_leakage = mt_spower_get_leakage(MTK_SPOWER_GPU, (unsigned int)voltage,
			tscpu_curr_gpu_temp / 1000);
	if (!g_buck_on || p_leakage < 0)
		p_leakage = 0;
#else
	p_leakage = 71;
#endif

	/* mW */
	return (unsigned long)p_leakage;
}

static unsigned long
hw_model_dynamic_power(struct devfreq *devfreq __maybe_unused,
	unsigned long freq, unsigned long voltage)
{
	/* refer to __mt_gpufreq_calculate_power */
	unsigned long p_dynamic = 0;
	unsigned long ref_freq = 0;
	unsigned long ref_vgpu = 0;
	unsigned long volt = voltage * 100; /* 100mv */

	p_dynamic = GPU_ACT_REF_POWER;
	ref_freq = GPU_ACT_REF_FREQ;
	ref_vgpu = GPU_ACT_REF_VOLT;
	freq = freq / 1000; /* hz -> khz */

	p_dynamic = p_dynamic *
		((freq * 100) / ref_freq) *
		((volt * 100) / ref_vgpu) *
		((volt * 100) / ref_vgpu) /
		(100 * 100 * 100);

	/* mW */
	return p_dynamic;
}

static struct devfreq_cooling_power hw_model_ops = {
	.get_static_power = hw_model_static_power,
	.get_dynamic_power = hw_model_dynamic_power,
	.get_real_power = NULL,
};

static int
hw_devfreq_get_dev_status(struct device *dev, struct devfreq_dev_status *stat)
{
	int ret, i, freq_num;
	struct GED_DVFS_OPP_STAT *stat_data = NULL;
	unsigned long busy_time = 0;
	unsigned long total_time = 0;

	freq_num = (int)mt_gpufreq_get_dvfs_table_num();

	stat_data = kzalloc(freq_num * sizeof(struct GED_DVFS_OPP_STAT), GFP_KERNEL);
	if (!stat_data) {
		pr_err("%s: Failed to alloc\n", __func__);
		return -ENOMEM;
	}

	ret = ged_dvfs_query_opp_cost(stat_data, freq_num, false);
	if (ret != 0) {
		pr_err("%s: Failed to get gpu stats\n", __func__);
		kfree(stat_data);
		return ret;
	}

	for (i = 0; i < freq_num; i++) {
		busy_time += stat_data[i].ui64Active;
		total_time += stat_data[i].ui64Active + stat_data[i].ui64Idle;
	}
	kfree(stat_data);

	stat->busy_time = busy_time;
	stat->total_time = total_time;
	stat->current_frequency = __mt_gpufreq_get_cur_freq() * 1000;
	stat->private_data = NULL;

	return 0;
}

static int hw_devfreq_target(struct device *dev, unsigned long *t_freq,
			     u32 flags)
{
	unsigned long freq = 0;
#ifdef CONFIG_HW_IPA_THERMAL
	unsigned long freq_limit;
	int gpu_id;

	gpu_id = ipa_get_actor_id("gpu");
	if (gpu_id < 0) {
		pr_err("[mali]Failed to get ipa actor id for gpu.\n");
		return -ENODEV;
	}
	freq_limit = ipa_freq_limit(gpu_id, freq);
	freq = freq_limit;
#endif

	/*
	 * M doesn't use mali devfreq driver, so we cannot
	 * use kbase_devfreq_target here.
	 */
	__mt_devfreq_target(freq);
	return 0;
}

static struct devfreq_dev_profile hw_devfreq_profile = {
	.polling_ms = 0, // STOP_POLLING,
	.target = hw_devfreq_target,
	.get_dev_status = hw_devfreq_get_dev_status,
};

static void hw_gpu_devfreq_init(struct device *dev)
{
	int ret;
	struct devfreq *devfreq = NULL;
	struct thermal_cooling_device *devfreq_cooling = NULL;

	/* setup devfreq */
	devfreq = devfreq_add_device(dev, &hw_devfreq_profile,
				"simple_ondemand", NULL);
	if (IS_ERR_OR_NULL(devfreq)) {
		ret = PTR_ERR(devfreq);
		gpufreq_pr_info("@%s: add devfreq failed: %d\n", __func__, ret);
		return;
	}

	/* setup devfreq_cooling */
	devfreq_cooling = of_devfreq_cooling_register_power(dev->of_node,
				devfreq, &hw_model_ops);
	if (IS_ERR_OR_NULL(devfreq_cooling)) {
		ret = PTR_ERR(devfreq_cooling);
		gpufreq_pr_info("@%s: register cooling device failed: %d\n", __func__,  ret);
		return;
	}
}
#else
static void hw_gpu_devfreq_init(struct device *dev)
{
}
#endif
