/*
 * hw power allocator to manage temperature
 *
 * Copyright (C) 2020 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_HW_IPA_THERMAL
#define MIN_POWER_DIFF 50
#define BOARDIPA_PID_RESET_TEMP 2000
extern unsigned int g_ipa_board_state[];
extern unsigned int g_ipa_soc_state[];
#define MAX_S48  0x7FFFFFFFFFFF

static void get_cur_power(struct thermal_zone_device *tz);
static void reset_pid_controller(struct power_allocator_params *params);
static void allow_maximum_power(struct thermal_zone_device *tz);
static int allocate_power(struct thermal_zone_device *tz,
			int control_temp,
			int switch_temp);

static void ipa_adjust_err_integral(struct power_allocator_params *params)
{
	if (params->err_integral > MAX_S48)
		params->err_integral = MAX_S48;
	else if (params->err_integral < -MAX_S48)
		params->err_integral = -MAX_S48;
}

static int power_actor_set_powers(struct thermal_zone_device *tz,
				struct thermal_instance *instance, u32 *soc_sustainable_power,
				u32 granted_power)
{
	unsigned long state = 0;
	int ret;
	int actor_id; // 0:Little, 1:Big, 2:GPU,

	if (!cdev_is_power_actor(instance->cdev))
		return -EINVAL;

	ret = instance->cdev->ops->power2state(instance->cdev, instance->tz, granted_power, &state);
	if (ret)
		return ret;

	actor_id = nametoactor(instance->cdev->type);
	if (actor_id < 0) {
		instance->target = state;
	} else if (tz->is_board_thermal) {
		g_ipa_board_state[actor_id] = state;
		instance->target = max(g_ipa_board_state[actor_id], g_ipa_soc_state[actor_id]);
	} else {
		g_ipa_soc_state[actor_id] = state;
		instance->target = max(g_ipa_board_state[actor_id], g_ipa_soc_state[actor_id]);
	}

	instance->cdev->updated = false;
	thermal_cdev_update(instance->cdev);

	return 0;
}

int thermal_zone_cdev_get_power(const char *thermal_zone_name, const char *cdev_name, unsigned int *power)
{
	struct thermal_instance *instance = NULL;
	int ret = 0;
	struct thermal_zone_device *tz = NULL;
	int find = 0;

	tz = thermal_zone_get_zone_by_name(thermal_zone_name);
	if (IS_ERR(tz)) {
		return -ENODEV;
	}

	mutex_lock(&tz->lock);
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		struct thermal_cooling_device *cdev = instance->cdev;

		if (!cdev_is_power_actor(cdev))
			continue;

		if (strncasecmp(instance->cdev->type, cdev_name, THERMAL_NAME_LENGTH))
			continue;

		if (!cdev->ops->get_requested_power(cdev, tz, power)) {
			find = 1;
			break;
		}
	}

	if (!find)
		ret = -ENODEV;

	mutex_unlock(&tz->lock);
	return ret;
}
EXPORT_SYMBOL(thermal_zone_cdev_get_power);

static void get_cur_power(struct thermal_zone_device *tz)
{
	struct thermal_instance *instance = NULL;
	struct power_allocator_params *params = NULL;
	u32 total_req_power;
	int trip_max_desired_temperature;

	if (tz->tzp->cur_enable == 0)
		return;

	mutex_lock(&tz->lock);
	if (tz->governor_data == NULL) {
		mutex_unlock(&tz->lock);
		return;
	}

	params = tz->governor_data;
	trip_max_desired_temperature = params->trip_max_desired_temperature;
	total_req_power = 0;

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		struct thermal_cooling_device *cdev = instance->cdev;
		u32 req_power;

		if (instance->trip != trip_max_desired_temperature)
			continue;

		if (!cdev_is_power_actor(cdev))
			continue;

		if (cdev->ops->get_requested_power(cdev, tz, &req_power))
			continue;

		total_req_power += req_power;

		cdev->cdev_cur_power += req_power;
	}

	tz->tzp->cur_ipa_total_power += total_req_power;
	tz->tzp->check_cnt++;

	mutex_unlock(&tz->lock);
}

static int power_allocator_throttle(struct thermal_zone_device *tz, int trip)
{
	int ret;
	int switch_on_temp, control_temp;
	struct power_allocator_params *params = NULL;
	struct power_allocator_params bak;
	enum thermal_device_mode mode;

	ret = tz->ops->get_mode(tz, &mode);
	if (ret || THERMAL_DEVICE_DISABLED == mode) {
		return ret;
	}
	mutex_lock(&tz->lock);
	params = tz->governor_data;
	if (params == NULL) {
		mutex_unlock(&tz->lock);
		return 0;
	}
	memcpy(&bak, params, sizeof(struct power_allocator_params));
	mutex_unlock(&tz->lock);

	/*
	 * We get called for every trip point but we only need to do
	 * our calculations once
	 */
	if (trip != bak.trip_max_desired_temperature)
		return 0;


	ret = tz->ops->get_trip_temp(tz, bak.trip_switch_on,
				     &switch_on_temp);

	if (!ret && (tz->temperature < switch_on_temp)) {
		tz->passive = 0;
		mutex_lock(&tz->lock);
		ipa_freq_limit_reset(tz);
		if (tz->governor_data != NULL)
			reset_pid_controller(tz->governor_data);
		mutex_unlock(&tz->lock);
		allow_maximum_power(tz);
		get_cur_power(tz);
		return 0;
	}

	tz->passive = 1;

	ret = tz->ops->get_trip_temp(tz, bak.trip_max_desired_temperature,
				&control_temp);
	if (ret) {
		dev_warn(&tz->device,
			 "Failed to get the maximum desired temperature: %d\n",
			 ret);
		return ret;
	}

	if (!ret && (tz->temperature <= (control_temp - BOARDIPA_PID_RESET_TEMP)) && tz->is_board_thermal) {
		mutex_lock(&tz->lock);
		if (tz->governor_data != NULL)
			reset_pid_controller(tz->governor_data);
		mutex_unlock(&tz->lock);
	}

	return allocate_power(tz, control_temp, switch_on_temp);
}

void update_pid_value(struct thermal_zone_device *tz)
{
	int ret;
	int control_temp;
	u32 sustainable_power = 0;
	struct power_allocator_params *params = tz->governor_data;

	mutex_lock(&tz->lock);
	if (!strncmp(tz->governor->name, "power_allocator", strlen("power_allocator"))) {
		ret = tz->ops->get_trip_temp(tz, params->trip_max_desired_temperature,
								&control_temp);
		if (ret) {
			dev_dbg(&tz->device,
			"Update PID failed to get control_temp: %d\n", ret);
		} else {
			if (tz->tzp->sustainable_power) {
				sustainable_power = tz->tzp->sustainable_power;
			} else {
				sustainable_power = estimate_sustainable_power(tz);
			}
			if (tz->is_soc_thermal)
				estimate_pid_constants(tz, sustainable_power,
					params->trip_switch_on,
					control_temp, (bool)true);
		}
	}
	mutex_unlock(&tz->lock);
}
#endif
