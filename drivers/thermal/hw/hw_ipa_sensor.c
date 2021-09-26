/*
 * hw_ipa_sensor.c
 *
 * sensors for ipa thermal
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

#include <linux/thermal.h>
#include <securec.h>

#include "../../../drivers/misc/mediatek/base/power/include/mtk_upower.h"
#include "../../../drivers/misc/mediatek/base/power/include/mtk_common_static_power.h"

#define DYNAMIC_TABLE2REAL_PERCENTAGE 58

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

/* board thermal sensor in mtk */
extern int mtkts_bts_get_hw_temp(void);
/* ts thermal sensor in mtk */
extern int tscpu_get_curr_temp(void);
extern int tscpu_curr_cpu_temp;
extern int tscpu_curr_gpu_temp;

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
	/* update temp info for the first sensor */
	if (sensor == IPA_CLUSTER_0)
		(void)tscpu_get_curr_temp();

	if (sensor == IPA_GPU)
		*val = tscpu_curr_gpu_temp;
	else
		*val = tscpu_curr_cpu_temp;

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
	*val = mtkts_bts_get_hw_temp();
	return 0;
}
EXPORT_SYMBOL_GPL(ipa_get_periph_value);

extern signed int battery_get_bat_temperature(void);
int hw_battery_temperature(void)
{
	return battery_get_bat_temperature();
}
EXPORT_SYMBOL_GPL(hw_battery_temperature);

void ipa_wait_dyn_power_init(void)
{
	upower_wait_init_done();
}

unsigned int ipa_get_dyn_power(int cid, unsigned int opp)
{
	unsigned int power;

	power = upower_get_power((enum upower_bank)cid, opp, UPOWER_DYN) / 1000;

	power = (power * 100 + DYNAMIC_TABLE2REAL_PERCENTAGE - 1) /
		DYNAMIC_TABLE2REAL_PERCENTAGE;

	return power;
}

extern int get_immediate_cpuL_wrap(void);
extern int get_immediate_cpuB_wrap(void);
extern unsigned int mt_cpufreq_get_cur_volt(int cid);

static unsigned int ipa_get_cluster_temp(int cid)
{
	unsigned int temp = 85;

	switch (cid) {
	case IPA_CLUSTER_0:
		temp = get_immediate_cpuL_wrap() / 1000;
		break;
	case IPA_CLUSTER_1:
		temp = get_immediate_cpuB_wrap() / 1000;
		break;
	default:
		pr_warn("%s: invalid cluster id=%d\n", __func__, cid);
		break;
	}

	return temp;
}

unsigned int ipa_get_static_power(int cid)
{
	unsigned int power;
	unsigned int temp;
	int volt;
	int core_num;
	struct cpumask cluster_cpus, online_cpus;

	temp = ipa_get_cluster_temp(cid);

	volt = mt_cpufreq_get_cur_volt(cid) / 100;

	power = mt_spower_get_leakage(0, volt, temp);

	arch_get_cluster_cpus(&cluster_cpus, cid);
	cpumask_and(&online_cpus, &cluster_cpus, cpu_online_mask);
	core_num = cpumask_weight(&online_cpus);

	power = power * core_num;

	return power;
}
