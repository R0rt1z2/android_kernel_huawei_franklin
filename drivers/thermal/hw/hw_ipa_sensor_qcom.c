/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: sensors for ipa thermal
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

#include <linux/thermal.h>
#include <securec.h>

enum hw_peripheral_temp_chanel {
	DETECT_SYSTEM_H_CHANEL = 0,
};

char *hw_peripheral_chanel[] = {
	[DETECT_SYSTEM_H_CHANEL] = "system_h",
};

enum hw_ipa_tsens {
	IPA_CLUSTER_0 = 0,
	IPA_CLUSTER_1,
	IPA_GPU,
};

int ipa_get_tsensor_id(const char *name)
{
	/*
	 * This is only used by tsens_max, and
	 * get_sensor_id_by_name will handle this error correctly.
	 */
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(ipa_get_tsensor_id);

int ipa_get_sensor_value(u32 sensor, int *val)
{
	*val = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(ipa_get_sensor_value);

int ipa_get_periph_id(const char *name)
{
	int ret = -ENODEV;
	u32 id;
	u32 sensor_num = sizeof(hw_peripheral_chanel) / sizeof(char *);

	if (name == NULL)
		return ret;

	pr_debug("IPA periph sensor_num = %d\n", sensor_num);
	for (id = 0; id < sensor_num; id++) {
		pr_debug("IPA: sensor_name = %s, hw_tsensor_name %d = %s\n",
		         name, id, hw_peripheral_chanel[id]);

		if (strlen(name) == strlen(hw_peripheral_chanel[id]) &&
		    strncmp(name, hw_peripheral_chanel[id], strlen(name)) == 0) {
			ret = (int)id;
			pr_debug("sensor_id = %d\n", ret);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipa_get_periph_id);

int ipa_get_periph_value(u32 sensor, int *val)
{
	/*
	 * Currently, we directly call mtk btsAP sensor (system_h in mtk)
	 * to get temp here to make IPA board_thermal work.
	 */
	if (sensor != DETECT_SYSTEM_H_CHANEL)
		return -ENODEV;

	*val = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(ipa_get_periph_value);

int hw_battery_temperature(void)
{
	return 1;
}
EXPORT_SYMBOL_GPL(hw_battery_temperature);
