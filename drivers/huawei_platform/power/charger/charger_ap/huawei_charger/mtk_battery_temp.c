/*
 * mtk_battery_temp.c
 *
 * mtk fuel gauge get battery temperature
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <chipset_common/hwpower/common_module/power_dts.h>
#include <chipset_common/hwpower/common_module/power_thermalzone.h>
#include <chipset_common/hwpower/coul/coul_interface.h>
#include <mt-plat/mtk_auxadc_intf.h>
#include <chipset_common/hwpower/common_module/power_sysfs.h>
#include <huawei_platform/log/hw_log.h>

#ifdef HWLOG_TAG
#undef HWLOG_TAG
#endif
#define HWLOG_TAG mtk_battery_temp
HWLOG_REGIST();

struct mtk_battery_temp_info {
	struct device *dev;
	u32 v_pullup;
	u32 r_pullup;
	u32 com_r_fg_value;
	u32 no_bat_temp_compensate;
	u32 com_fg_meter_resistance;
};

static void mtk_battery_temp_parse_dt(struct mtk_battery_temp_info *info,
	struct device_node *node)
{
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"v_pullup", &info->v_pullup, 0);
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"r_pullup", &info->r_pullup, 0);
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"com_r_fg_value", &info->com_r_fg_value, 0);
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"no_bat_temp_compensate", &info->no_bat_temp_compensate, 0);
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"com_fg_meter_resistance", &info->no_bat_temp_compensate, 0);
}

#ifdef CONFIG_HLTHERM_RUNTEST
static int mtk_battery_temp_get_raw(int adc_chnnel, long *data, void *dev_data)
{
	if (!data)
		return -1;

	*data = 10000; /* HLTHERM_RUNTEST set temperature to 25, r_ntc 10k */

	return 0;
}
#else
static int mtk_battery_temp_get_batt_temp(struct mtk_battery_temp_info *info,
	int adc_chnnel, long *data)
{
	int v_adc;
	int batt_curr;
	int com_fg_meter_resistance = info->com_fg_meter_resistance;
	int fg_r_value = info->com_r_fg_value;
	long long v_comp;

	if (!info->r_pullup)
		return -1;

	v_adc = pmic_get_auxadc_value(adc_chnnel);
	if (!v_adc)
		return -1;

	if (info->no_bat_temp_compensate)
		com_fg_meter_resistance = 0;

	batt_curr = coul_interface_get_battery_current(COUL_TYPE_MAIN);
	v_comp = (long long)batt_curr * ((long long)com_fg_meter_resistance +
		(long long)fg_r_value) / 1000;

	if ((info->v_pullup - v_adc) == 0)
		return -1;
	/* (v_pullup - v_adc) / r_pullup = (vadc - v_comp) / r_ntc */
	*data = (long long)(v_adc - v_comp) * info->r_pullup / (info->v_pullup - v_adc);

	return 0;
}

static int mtk_battery_temp_get_raw(int adc_chnnel, long *data, void *dev_data)
{
	struct mtk_battery_temp_info *info = dev_data;

	if (!data || !info)
		return -1;

	if (mtk_battery_temp_get_batt_temp(info, adc_chnnel, data)) {
		hwlog_err("get battery temp fail\n");
		return -1;
	}

	return 0;
}
#endif /* CONFIG_HLTHERM_RUNTEST */

static struct power_tz_ops mtk_battery_temp_tz_ops = {
	.get_raw_data = mtk_battery_temp_get_raw,
};

static int mtk_battery_temp_probe(struct platform_device *pdev)
{
	struct mtk_battery_temp_info *info = NULL;
	struct device_node *node = NULL;

	if (!pdev || !pdev->dev.of_node)
		return -ENODEV;

	node = pdev->dev.of_node;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mtk_battery_temp_parse_dt(info, node);

	mtk_battery_temp_tz_ops.dev_data = info;
	if (power_tz_ops_register(&mtk_battery_temp_tz_ops, "mtk_batt_temp"))
		goto fail_free_mem;

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	return 0;

fail_free_mem:
	devm_kfree(&pdev->dev, info);
	return -1;
}

static int mtk_battery_temp_remove(struct platform_device *pdev)
{
	struct mtk_battery_temp_info *info = platform_get_drvdata(pdev);

	if (!info)
		return -ENODEV;

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, info);

	return 0;
}

static const struct of_device_id battery_temp_match_table[] = {
	{
		.compatible = "huawei,mtk_battery_temp",
		.data = NULL,
	},
	{
	},
};

static struct platform_driver mtk_battery_temp_driver = {
	.probe = mtk_battery_temp_probe,
	.remove = mtk_battery_temp_remove,
	.driver = {
		.name = "mtk_battery_temp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(battery_temp_match_table),
	},
};

static int __init mtk_battery_temp_init(void)
{
	return platform_driver_register(&mtk_battery_temp_driver);
}

static void __exit mtk_battery_temp_exit(void)
{
	platform_driver_unregister(&mtk_battery_temp_driver);
}

module_init(mtk_battery_temp_init);
module_exit(mtk_battery_temp_exit);

MODULE_DESCRIPTION("mtk battery temp driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
