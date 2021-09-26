/*
 * mtk_usb_temp.c
 *
 * mtk get usb temperature
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <chipset_common/hwpower/common_module/power_dts.h>
#include <chipset_common/hwpower/common_module/power_thermalzone.h>
#include <huawei_platform/log/hw_log.h>
#include "mtk_auxadc.h"

#ifdef HWLOG_TAG
#undef HWLOG_TAG
#endif
#define HWLOG_TAG mtk_usb_temp
HWLOG_REGIST();

#define DEFAULT_ADC_CHANNEL        3
#define ADC_SAMPLE_RETRY_MAX       3
#define ADC_VOL_INIT               (-1)

struct mtk_usb_temp_info {
	struct device *dev;
	int adc_channel;
	int using_mtcharger_adc;
	struct iio_channel *channel_raw;
};

static void mtk_usb_temp_parse_dt(struct mtk_usb_temp_info *info,
	struct device_node *node)
{
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"uscp_adc", (u32 *)&info->adc_channel,
		DEFAULT_ADC_CHANNEL);
	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"using_mtcharger_adc", (u32 *)&info->using_mtcharger_adc,
		0);
}

#ifdef CONFIG_MT6873_USCP_ADC
static int mtk_usb_temp_get_raw_data(int adc_channel, long *data, void *dev_data)
{
	int i;
	int ret;
	int t_sample;
	struct mtk_usb_temp_info *info = dev_data;

	if (!data || !info || IS_ERR(info->channel_raw))
		return -1;

	for (i = 0; i < ADC_SAMPLE_RETRY_MAX; ++i) {
		t_sample = ADC_VOL_INIT;
		ret = iio_read_channel_processed(info->channel_raw, &t_sample);
		if (info->using_mtcharger_adc) {
			/* 1000 : trans to mv */
			t_sample = t_sample / 1000;
			/* scale : 4096mv /1800mv */
			t_sample = t_sample * 4096 / 1800;
		} else {
			/* scale : 1500mv /1800mv */
			t_sample = t_sample * 1500 / 1800;
		}
		if (ret < 0)
			hwlog_err("iio adc read fail\n");
		else
			break;
	}

	*data = t_sample;
	return 0;
}
#else
static int mtk_usb_temp_get_raw_data(int adc_channel, long *data, void *dev_data)
{
	int i;
	int ret;
	int t_sample;
	struct mtk_usb_temp_info *info = dev_data;

	if (!data || !info)
		return -1;

	for (i = 0; i < ADC_SAMPLE_RETRY_MAX; ++i) {
		t_sample = ADC_VOL_INIT;
		ret = IMM_GetOneChannelValue_Cali(info->adc_channel, &t_sample);
#ifdef CONFIG_MACH_MT6765
		/* 1000 : trans to mv */
		t_sample = t_sample / 1000;
		/* scale : 4096mv /1800mv */
		t_sample = t_sample * 4096 / 1800;
#endif
		if (ret < 0)
			hwlog_err("imm adc read fail\n");
		else
			break;
	}

	*data = t_sample;
	return 0;
}
#endif /* CONFIG_MT6873_USCP_ADC */

static struct power_tz_ops mtk_usb_temp_tz_ops = {
	.get_raw_data = mtk_usb_temp_get_raw_data,
};

static int mtk_usb_temp_probe(struct platform_device *pdev)
{
	struct mtk_usb_temp_info *info = NULL;
	struct device_node *node = NULL;
	int ret;

	if (!pdev || !pdev->dev.of_node)
		return -ENODEV;

	node = pdev->dev.of_node;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	mtk_usb_temp_parse_dt(info, node);

#ifdef CONFIG_MT6873_USCP_ADC
	info->channel_raw = iio_channel_get(&pdev->dev, "uscp_channel");
	ret = IS_ERR(info->channel_raw);
	if (ret) {
		hwlog_err("fail to usb adc auxadc iio: %d\n", ret);
		goto fail_free_mem;
	}
#endif /* CONFIG_MT6873_USCP_ADC */

	mtk_usb_temp_tz_ops.dev_data = info;
	ret = power_tz_ops_register(&mtk_usb_temp_tz_ops, "uscp");
	if (ret)
		goto fail_free_mem;

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	return 0;

fail_free_mem:
	devm_kfree(&pdev->dev, info);
	return ret;
}

static int mtk_usb_temp_remove(struct platform_device *pdev)
{
	struct mtk_usb_temp_info *info = platform_get_drvdata(pdev);

	if (!info)
		return -ENODEV;

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, info);

	return 0;
}

static const struct of_device_id battery_temp_match_table[] = {
	{
		.compatible = "huawei,mtk_usb_temp",
		.data = NULL,
	},
	{},
};

static struct platform_driver mtk_usb_temp_driver = {
	.probe = mtk_usb_temp_probe,
	.remove = mtk_usb_temp_remove,
	.driver = {
		.name = "mtk_usb_temp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(battery_temp_match_table),
	},
};

static int __init mtk_usb_temp_init(void)
{
	return platform_driver_register(&mtk_usb_temp_driver);
}

static void __exit mtk_usb_temp_exit(void)
{
	platform_driver_unregister(&mtk_usb_temp_driver);
}

device_initcall(mtk_usb_temp_init);
module_exit(mtk_usb_temp_exit);

MODULE_DESCRIPTION("mtk usb temp driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
