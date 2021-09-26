/*
 * thermal_sys_hw.c
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

#ifdef CONFIG_HW_IPA_THERMAL
#define MAX_TEMP 145000
#define MIN_TEMP -40000
s32 thermal_zone_temp_check(s32 temperature)
{
	if (temperature > MAX_TEMP) {
		temperature = MAX_TEMP;
	} else if (temperature < MIN_TEMP) {
		temperature = MIN_TEMP;
	}
	return temperature;
}
#endif

#ifdef CONFIG_HW_IPA_THERMAL
extern void restore_actor_weights(struct thermal_zone_device *tz);
extern void update_actor_weights(struct thermal_zone_device *tz);
extern ssize_t boost_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t boost_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(boost, 0644, boost_show, boost_store);
#endif

#ifdef CONFIG_HW_THERMAL_TSENSOR
static ssize_t
trip_point_type_activate(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	int trip = 0;
	int result;

	if (!tz->ops->activate_trip_type)
		return -EPERM;

	if (!sscanf(attr->attr.name, "trip_point_%d_type", &trip))
		return -EINVAL;

	if (!strncmp(buf, "enabled", strlen("enabled"))) {
		result = tz->ops->activate_trip_type(tz, trip,
					THERMAL_TRIP_ACTIVATION_ENABLED);
	} else if (!strncmp(buf, "disabled", strlen("disabled"))) {
		result = tz->ops->activate_trip_type(tz, trip,
					THERMAL_TRIP_ACTIVATION_DISABLED);
	} else {
		result = -EINVAL;
	}
	if (result)
		return result;
	return count;
}
#endif

static ssize_t cur_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_zone_device *tz = NULL;
	struct thermal_instance *instance = NULL;
	ssize_t buf_len;
	ssize_t size = PAGE_SIZE;
	if (dev == NULL || attr == NULL || buf == NULL) {
		return -EIO;
	}

	tz = to_thermal_zone(dev);
	if (tz == NULL || tz->tzp == NULL) {
		return -EIO;
	}

	mutex_lock(&tz->lock);
	buf_len = scnprintf(buf, size, "%llu,%u,", tz->tzp->cur_ipa_total_power, tz->tzp->check_cnt);
	if (buf_len < 0)
		goto unlock;
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		struct thermal_cooling_device *cdev = instance->cdev;
		ssize_t ret;
		size = PAGE_SIZE - buf_len;
		if (size < 0 || size > (ssize_t)PAGE_SIZE)
			break;
		ret = scnprintf(buf + buf_len, size, "%llu,", cdev->cdev_cur_power);
		if (ret < 0)
			break;
		buf_len += ret;
	}
unlock:
	mutex_unlock(&tz->lock);

	if (buf_len > 0)
		buf[buf_len - 1] = '\0'; /* set string end with '\0' */
	return buf_len;
}

static DEVICE_ATTR(cur_power, S_IRUGO, cur_power_show, NULL);

static ssize_t
cur_enable_show(struct device *dev, struct device_attribute *devattr,
		       char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);

	if (tz->tzp)
		return scnprintf(buf, PAGE_SIZE, "%u\n", tz->tzp->cur_enable);
	else
		return -EIO;
}

static ssize_t
cur_enable_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	u32 cur_enable;

	if (!tz->tzp)
		return -EIO;

	if (kstrtou32(buf, 10, &cur_enable))
		return -EINVAL;

	tz->tzp->cur_enable = (cur_enable > 0);

	return count;
}

static DEVICE_ATTR(cur_enable, S_IWUSR | S_IRUGO, cur_enable_show, cur_enable_store);


#ifdef CONFIG_HW_IPA_THERMAL
static ssize_t
boost_timeout_show(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);

	if (tz->tzp)
		return snprintf(buf, PAGE_SIZE, "%u\n", tz->tzp->boost_timeout);
	else
		return -EIO;
}

#define BOOST_TIMEOUT_THRES	5000
static ssize_t
boost_timeout_store(struct device *dev, struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	u32 boost_timeout;

	if (!tz->tzp)
		return -EIO;

	if (kstrtou32(buf, 10, &boost_timeout))
		return -EINVAL;

	if (boost_timeout > BOOST_TIMEOUT_THRES)
		return -EINVAL;

	if (boost_timeout)
		tz->tzp->boost_timeout = boost_timeout;

	trace_IPA_boost(current->pid, tz, tz->tzp->boost,
			tz->tzp->boost_timeout);
	return count;
}
static DEVICE_ATTR(boost_timeout, (S_IWUSR | S_IRUGO), boost_timeout_show,
		boost_timeout_store);

static int ipa_thermal_cooling_device_weight_store(struct device *dev, struct thermal_instance *instance, int weight)
{
	struct thermal_zone_device *tz = to_thermal_zone(dev);
	enum thermal_device_mode mode;
	int result;
	int actor_id = -1;

	if (!tz->ops->get_mode)
		return -EPERM;

	result = tz->ops->get_mode(tz, &mode);
	if (result)
		return result;

	actor_id = nametoactor(instance->cdev->type);
	if (actor_id != -1)
		g_ipa_normal_weights[actor_id] = weight;

	return 0;
}
#endif
