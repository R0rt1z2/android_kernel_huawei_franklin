/*
 * thermal_core_hw.c
 *
 * hw thermal enhance
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

#ifdef CONFIG_HW_PERF_CTRL
#include <linux/hw/thermal_perf_ctrl.h>
#endif

#ifdef CONFIG_HW_IPA_THERMAL
#define MODE_LEN 10
int g_ipa_freq_limit_debug = 0;

unsigned int g_ipa_freq_limit[CAPACITY_OF_ARRAY] = {
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX,
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX
};
unsigned int g_ipa_soc_freq_limit[CAPACITY_OF_ARRAY] = {
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX,
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX
};
unsigned int g_ipa_board_freq_limit[CAPACITY_OF_ARRAY] = {
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX,
	IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX, IPA_FREQ_MAX
};
unsigned int g_ipa_soc_state[CAPACITY_OF_ARRAY] = {0};
unsigned int g_ipa_board_state[CAPACITY_OF_ARRAY] = {0};

unsigned int g_ipa_gpu_boost_weights[CAPACITY_OF_ARRAY] = {0};
unsigned int g_ipa_normal_weights[CAPACITY_OF_ARRAY] = {0};

static DEFINE_MUTEX(thermal_boost_lock);

#ifdef CONFIG_HW_THERMAL_SPM
extern bool is_spm_mode_enabled(void);
extern unsigned int get_powerhal_profile(int actor);
extern unsigned int get_minfreq_profile(int actor);
int of_thermal_get_num_tbps(struct thermal_zone_device *tz);
int of_thermal_get_actor_weight(struct thermal_zone_device *tz, int i,
				int *actor, unsigned int *weight);
#endif

bool thermal_of_get_cdev_type(struct thermal_zone_device *tzd,
			struct thermal_cooling_device *cdev);

void ipa_freq_debug(int onoff)
{
	g_ipa_freq_limit_debug = onoff;
}

void ipa_freq_limit_init(void)
{
	int i;
	for (i = 0; i < (int)g_ipa_actor_num; i++)
		g_ipa_freq_limit[i] = IPA_FREQ_MAX;
}

void ipa_freq_limit_reset(struct thermal_zone_device *tz)
{
	int i;

	struct thermal_instance *instance = NULL;

	if (tz->is_board_thermal) {
		for (i = 0; i < (int)g_ipa_actor_num; i++) {
			g_ipa_board_freq_limit[i] = IPA_FREQ_MAX;
			g_ipa_board_state[i] = 0;
		}
	} else {
		for (i = 0; i < (int)g_ipa_actor_num; i++) {
			g_ipa_soc_freq_limit[i] = IPA_FREQ_MAX;
			g_ipa_soc_state[i] = 0;
		}
	}

	for (i = 0; i < (int)g_ipa_actor_num; i++) {
		if (g_ipa_soc_freq_limit[i] != 0 && g_ipa_board_freq_limit[i] != 0)
			g_ipa_freq_limit[i] = min(g_ipa_soc_freq_limit[i], g_ipa_board_freq_limit[i]);
		else if (g_ipa_soc_freq_limit[i] == 0)
			g_ipa_freq_limit[i] = g_ipa_board_freq_limit[i];
		else if (g_ipa_board_freq_limit[i] == 0)
			g_ipa_freq_limit[i] = g_ipa_soc_freq_limit[i];
		else
			g_ipa_freq_limit[i] = IPA_FREQ_MAX;
	}

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		instance->target = 0;
		instance->cdev->ops->set_cur_state(instance->cdev, 0UL);
		mutex_lock(&instance->cdev->lock);
		instance->cdev->updated = true;
		mutex_unlock(&instance->cdev->lock);
	}
}

unsigned int ipa_freq_limit(int actor, unsigned int target_freq)
{
#ifdef CONFIG_HW_THERMAL_SPM
	unsigned int freq, min_freq;
#endif
	int i = 0;

	if (actor >= (int)g_ipa_actor_num)
		return target_freq;

	if (g_ipa_freq_limit_debug) {
		pr_err("actor[%d]target_freq[%u]IPA:", actor, target_freq);
		for (i = 0; i < (int)g_ipa_actor_num; i++)
			pr_err("[%u]", g_ipa_freq_limit[i]);
		pr_err("min[%u]\n", min(target_freq, g_ipa_freq_limit[actor]));
	}

#ifdef CONFIG_HW_THERMAL_SPM
	if (is_spm_mode_enabled()) {
		freq = get_powerhal_profile(actor);
		min_freq = get_minfreq_profile(actor);

		if (target_freq >= freq)
			target_freq = freq;

		if (target_freq <= min_freq)
			target_freq = min_freq;

		return target_freq;
	}
#endif

	return min(target_freq, g_ipa_freq_limit[actor]);
}
EXPORT_SYMBOL(ipa_freq_limit);

#ifdef CONFIG_HW_PERF_CTRL
int get_ipa_status(struct ipa_stat *status)
{
	status->cluster0 = g_ipa_freq_limit[0];
	status->cluster1 = g_ipa_freq_limit[1];
	if (ipa_get_actor_id("gpu") == 3) {
		status->cluster2 = g_ipa_freq_limit[2];
		status->gpu = g_ipa_freq_limit[3];
	} else {
		status->cluster2 = 0;
		status->gpu = g_ipa_freq_limit[2];
	}

	return 0;
}
#endif
#endif

#ifdef CONFIG_HW_IPA_THERMAL
int nametoactor(const char *type)
{
	int actor_id = -1;

	if (!strncmp(type, CDEV_GPU_NAME, sizeof(CDEV_GPU_NAME) - 1))
			actor_id = ipa_get_actor_id("gpu");
	else if (!strncmp(type, CDEV_CPU_CLUSTER0_NAME, sizeof(CDEV_CPU_CLUSTER0_NAME) - 1))
			actor_id = ipa_get_actor_id("cluster0");
	else if (!strncmp(type, CDEV_CPU_CLUSTER1_NAME, sizeof(CDEV_CPU_CLUSTER1_NAME) - 1))
			actor_id = ipa_get_actor_id("cluster1");
	else if (!strncmp(type, CDEV_CPU_CLUSTER2_NAME, sizeof(CDEV_CPU_CLUSTER2_NAME) - 1))
			actor_id = ipa_get_actor_id("cluster2");
	else
		actor_id = -1;

	return actor_id;
}

void restore_actor_weights(struct thermal_zone_device *tz)
{
	struct thermal_instance *pos = NULL;
	int actor_id = -1;

	if (!is_ipa_thermal(tz))
		return;

	list_for_each_entry(pos, &tz->thermal_instances, tz_node) {
		actor_id = nametoactor(pos->cdev->type);
		if (actor_id != -1)
			pos->weight = g_ipa_normal_weights[actor_id];
	}
}
EXPORT_SYMBOL_GPL(restore_actor_weights);

void update_actor_weights(struct thermal_zone_device *tz)
{
	struct thermal_instance *pos = NULL;
	bool bGPUBounded = false;
	struct thermal_cooling_device *cdev = NULL;
	int actor_id = -1;

	if (!is_ipa_thermal(tz))
		return;

	list_for_each_entry(pos, &tz->thermal_instances, tz_node) {
		cdev = pos->cdev;
		if (!strncmp(cdev->type, CDEV_GPU_NAME, sizeof(CDEV_GPU_NAME) - 1) && cdev->bound_event) {
			bGPUBounded = true;
			break;
		}
	}

	list_for_each_entry(pos, &tz->thermal_instances, tz_node) {
		actor_id = nametoactor(pos->cdev->type);

		if (bGPUBounded) {
			if (actor_id != -1)
				pos->weight = g_ipa_gpu_boost_weights[actor_id];
		} else {
			if (actor_id != -1)
				pos->weight = g_ipa_normal_weights[actor_id];
		}
	}
}
EXPORT_SYMBOL_GPL(update_actor_weights);
#endif

#ifdef CONFIG_HW_IPA_THERMAL
ssize_t
boost_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	enum thermal_boost_mode mode;

	if (!tz->tzp)
		return -EIO;

	mutex_lock(&tz->lock);
	mode = tz->tzp->boost;
	mutex_unlock(&tz->lock);
	return snprintf_s(buf, MODE_LEN, MODE_LEN - 1, "%s\n", mode == THERMAL_BOOST_ENABLED ? "enabled" : "disabled");
}
EXPORT_SYMBOL_GPL(boost_show);

ssize_t
boost_store(struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int result = 0;

	if (!tz->tzp)
		return -EIO;

	mutex_lock(&thermal_boost_lock);

	if (!strncmp(buf, "enabled", sizeof("enabled") - 1)) {
		mutex_lock(&tz->lock);
		if (THERMAL_BOOST_ENABLED == tz->tzp->boost)
			pr_info("perfhub boost high frequency timeout[%d]\n", tz->tzp->boost_timeout);

		tz->tzp->boost = THERMAL_BOOST_ENABLED;
		if (tz->tzp->boost_timeout == 0)
			tz->tzp->boost_timeout = 100;

		thermal_zone_device_set_polling(tz, tz->tzp->boost_timeout);
		ipa_freq_limit_reset(tz);
		mutex_unlock(&tz->lock);
	} else if (!strncmp(buf, "disabled", sizeof("disabled") - 1)) {
		tz->tzp->boost = THERMAL_BOOST_DISABLED;
		thermal_zone_device_update(tz, THERMAL_EVENT_UNSPECIFIED);
	} else {
		result = -EINVAL;
	}

	mutex_unlock(&thermal_boost_lock);

	if (result)
		return result;

	trace_IPA_boost(current->pid, tz, tz->tzp->boost,
			tz->tzp->boost_timeout);

	return count;
}
EXPORT_SYMBOL_GPL(boost_store);
#endif

#ifdef CONFIG_HW_IPA_THERMAL
static inline void set_tz_type(struct thermal_zone_device *tz)
{
	if (!strncmp(tz->type, BOARD_THERMAL_NAME, sizeof(BOARD_THERMAL_NAME) - 1))
		tz->is_board_thermal = true;
	else if (!strncmp(tz->type, SOC_THERMAL_NAME, sizeof(SOC_THERMAL_NAME) - 1))
		tz->is_soc_thermal = true;
}
#endif
