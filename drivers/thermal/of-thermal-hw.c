/*
 * of-thermal-hw.c
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
bool thermal_of_get_cdev_type(struct thermal_zone_device *tzd,
			struct thermal_cooling_device *cdev)
{
	struct __thermal_zone *tz = NULL;
	int i;
	struct __thermal_bind_params *tbp = NULL;

	if (!tzd)
		return false;

	tz = (struct __thermal_zone *)tzd->devdata;
	if (IS_ERR_OR_NULL(tz))
		return false;

	for (i = 0; i < tz->num_tbps; i++) {
		tbp = tz->tbps + i;

		if (IS_ERR_OR_NULL(tbp)) {
			pr_err("tbp is null\n");
			return false;
		}

		if (IS_ERR_OR_NULL(tbp->cooling_device)) {
			pr_err("tbp->cooling_device is null\n");
			return false;
		}

		if (IS_ERR_OR_NULL(cdev->np)) {
			pr_err("cdev->np is null\n");
			return false;
		}

		if (tbp->cooling_device == cdev->np)
			return tbp->is_soc_cdev;
	}

	return false;
}
EXPORT_SYMBOL(thermal_of_get_cdev_type);
#endif

#ifdef CONFIG_HW_THERMAL_SPM
int of_thermal_get_num_tbps(struct thermal_zone_device *tz)
{
	struct __thermal_zone *data = tz->devdata;

	return data->num_tbps;
}
EXPORT_SYMBOL(of_thermal_get_num_tbps);

int of_thermal_get_actor_weight(struct thermal_zone_device *tz, int i,
				int *actor, unsigned int *weight)
{
	struct __thermal_zone *data = tz->devdata;

	if (i >= data->num_tbps || i < 0)
		return -EDOM;

	*actor = ipa_get_actor_id(data->tbps[i].cooling_device->name);
	if (*actor < 0)
		*actor = ipa_get_actor_id("gpu");

	*weight = data->tbps[i].usage;
	pr_err("IPA: matched actor: %d, weight: %d\n", *actor, *weight);

	return 0;
}
EXPORT_SYMBOL(of_thermal_get_actor_weight);
#endif

