/*
 * antenna_boardid_adc_match.c
 *
 * Check the antenna board match status.
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#include "antenna_boardid_adc_match.h"
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#define antenna_sysfs_field(_name, n, m, store) {	\
	.attr = __ATTR(_name, m, antenna_show, store),	\
	.name = ANTENNA_##n,	\
}

#define antenna_sysfs_field_ro(_name, n)	\
		antenna_sysfs_field(_name, n, 0444, NULL)

#define ADC_MATCH_RANGE 2
#define ADC_MATCH_RANGE_MIN 820
#define ADC_MATCH_RANGE_MAX 975

struct iio_channel *adc_channel = NULL;

static int adc_match_range[ADC_MATCH_RANGE] = {
	ADC_MATCH_RANGE_MIN,
	ADC_MATCH_RANGE_MAX,
};

static ssize_t antenna_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct antenna_sysfs_field_info {
	struct device_attribute attr;
	u8 name;
};

static struct antenna_sysfs_field_info antenna_tb[] = {
	antenna_sysfs_field_ro(antenna_board_match,   BOARD_ADC_MATCH),
	antenna_sysfs_field_ro(antenna_board_voltage, BOARD_ADC_VOLTAGE),
};

static struct attribute *antenna_sysfs_attrs[ARRAY_SIZE(antenna_tb) + 1];

static const struct attribute_group antenna_sysfs_attr_group = {
	.attrs = antenna_sysfs_attrs,
};

static void antenna_sysfs_init_attrs(void)
{
	int i;
	int limit = ARRAY_SIZE(antenna_tb);

	for (i = 0; i < limit; i++)
		antenna_sysfs_attrs[i] = &antenna_tb[i].attr.attr;
	antenna_sysfs_attrs[limit] = NULL;
}

static struct antenna_sysfs_field_info *antenna_board_lookup(const char *name)
{
	int i;
	int limit = ARRAY_SIZE(antenna_tb);

	for (i = 0; i < limit; i++) {
		if (!strncmp(name, antenna_tb[i].attr.attr.name, strlen(name)))
			break;
	}

	if (i >= limit)
		return NULL;

	return &antenna_tb[i];
}

static int antenna_match_sysfs_create_group(struct antenna_device_info *di)
{
	antenna_sysfs_init_attrs();
	return sysfs_create_group(&di->dev->kobj, &antenna_sysfs_attr_group);
}

static int get_antenna_boardid_adc_voltage(void)
{
	int ret;
	int voltage = -1;

	ret = iio_read_channel_processed(adc_channel, &voltage);
	if (ret < 0 || voltage < 0) {
		pr_err("antenna board adc read fail, ret = %d, voltage = %d\n", ret, voltage);
		return 0;
	}

	/* voltage * 1500 / 4096 */
	voltage = ((unsigned int)voltage * 1500) >> 12;
	pr_err("antenna board adc voltage = %d\n", voltage);
	return voltage;
}

static int get_antenna_boardid_adc_match(void)
{
	int voltage = get_antenna_boardid_adc_voltage();

	if (voltage >= adc_match_range[0] && voltage <= adc_match_range[1])
		return 1;

	pr_err("adc voltage isn't in range\n");
	return 0;
}

static ssize_t antenna_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct antenna_sysfs_field_info *info = NULL;

	info = antenna_board_lookup(attr->attr.name);
	if (!info)
		return -EINVAL;

	switch (info->name) {
	case ANTENNA_BOARD_ADC_MATCH:
		ret = get_antenna_boardid_adc_match();
		break;
	case ANTENNA_BOARD_ADC_VOLTAGE:
		ret = get_antenna_boardid_adc_voltage();
		break;
	default:
		pr_err("%s: HAVE NO THIS NODE:%d\n", __func__, info->name);
		break;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static int parse_adc_channel_and_range(struct antenna_device_info *di)
{
	struct device_node *np = di->dev->of_node;

	(void)of_property_read_u32_array(np, "antenna_board_match_range",
					adc_match_range, ADC_MATCH_RANGE);
	pr_err("adc_match_range: min = %d, max = %d\n",
		adc_match_range[0], adc_match_range[1]);

	/* adc channel */
	adc_channel = devm_kzalloc(di->dev, sizeof(*adc_channel), GFP_KERNEL);
	if (!adc_channel)
		return -ENOMEM;

	adc_channel = iio_channel_get(di->dev, "antenna_channel");
	if (IS_ERR(adc_channel)) {
		pr_err("get antenna channel failed!\n");
		return -1;
	}
	pr_err("get adc channel = %d\n", adc_channel->channel->channel);
	return 0;
}

static int antenna_board_match_probe(struct platform_device *pdev)
{
	int ret;
	struct device *ant_dev = NULL;
	struct antenna_device_info *di = NULL;
	struct class *ant_class = NULL;
	pr_info("%s: function start\n", __func__);

	if (!pdev) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &pdev->dev;
	platform_set_drvdata(pdev, di);

	if (parse_adc_channel_and_range(di) != 0)
		goto free_di;

	ant_class = class_create(THIS_MODULE, "hw_antenna");
	if (IS_ERR(ant_class)) {
		pr_err("hw_antenna class create fail");
		goto free_di;
	}

	ant_dev = device_create(ant_class, NULL, 0, NULL, "antenna_board");
	if (IS_ERR(ant_dev)) {
		pr_err("create ant_dev failed!\n");
		goto free_di;
	}

	ret = sysfs_create_link(&ant_dev->kobj,
		&di->dev->kobj, "antenna_board_data");
	if (ret) {
		pr_err("create link to board_match fail\n");
		goto free_di;
	}

	ret = antenna_match_sysfs_create_group(di);
	if (ret) {
		pr_err("can't create antenna_detect sysfs entries\n");
		goto free_di;
	}
	return 0;

free_di:
	platform_set_drvdata(pdev, NULL);
	kfree(di);
	return -1;
}

static int antenna_board_match_remove(struct platform_device *pdev)
{
	struct antenna_device_info *di = platform_get_drvdata(pdev);

	if (!di) {
		pr_err("[%s]di is NULL!\n", __func__);
		return -ENODEV;
	}

	kfree(di);
	return 0;
}

static const struct of_device_id antenna_board_table[] = {
	{
		.compatible = "huawei,antenna_board_match",
		.data = NULL,
	},
	{},
};

static struct platform_driver antenna_board_match_driver = {
	.probe = antenna_board_match_probe,
	.remove = antenna_board_match_remove,
	.driver = {
		.name = "huawei,antenna_board_match",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(antenna_board_table),
	},
};

static int __init antenna_board_match_init(void)
{
	return platform_driver_register(&antenna_board_match_driver);
}

static void __exit antenna_board_match_exit(void)
{
	platform_driver_unregister(&antenna_board_match_driver);
}

module_init(antenna_board_match_init);
module_exit(antenna_board_match_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("huawei antenna board match driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");