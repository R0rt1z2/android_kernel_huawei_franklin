/*
 * camkit_check_cable.c
 *
 * Check the camera btb cable status.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/device.h>

#include "securec.h"
#include "camkit_check_cable.h"
#include "dsm_camera.h"
#include "kd_camera_feature.h"

#define MAX_BTB_CHECK_GPIO_PER_CAM 4
#define MAX_BTB_CHECK_DSMINFO_LENTH 128
#define GPIO_INIT_VALUE 0xFFFF
#define GPIO_HIGH 1
#define GPIO_LOW 0

struct btb_check_gpio_info {
	unsigned int gpio[MAX_BTB_CHECK_GPIO_PER_CAM];
	const char *dsm_notify_info;
};

static struct btb_check_gpio_info	\
	btb_check_gpio_info[IMGSENSOR_SENSOR_IDX_MAX_NUM];

static void get_btb_check_gpio_info_from_dts()
{
	struct device_node *root = NULL;
	struct device_node *child = NULL;
	int gpio_count;
	int gpio_index;
	int ret;
	int i;
	int j;
	static int dts_get_flag;

	if (dts_get_flag)
		return;

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++)
		for (j = 0; j < MAX_BTB_CHECK_GPIO_PER_CAM; j++)
			btb_check_gpio_info[i].gpio[j] = GPIO_INIT_VALUE;

	root = of_find_compatible_node(NULL, NULL, "mediatek, camera_btb_check");
	if (!root) {
		pr_err("get camera_btb_check root node fail!");
		return;
	}

	for_each_child_of_node(root, child) {
		ret = of_property_read_u32_array(child, "index", &gpio_index, 1);
		if (ret < 0) {
			pr_err("get child index fail!");
			continue;
		}

		gpio_count = of_property_count_elems_of_size(child,
			"gpio", sizeof(u32));
		if (gpio_count > MAX_BTB_CHECK_GPIO_PER_CAM) {
			pr_err("get child cam %d gpio_count max!", gpio_index);
			gpio_count = MAX_BTB_CHECK_GPIO_PER_CAM;
		}
		ret = of_property_read_u32_array(child, "gpio",
			(u32 *)&(btb_check_gpio_info[gpio_index].gpio), gpio_count);
		if (ret < 0) {
			pr_err("get child cam %d gpio fail!", gpio_index);
			continue;
		}

		ret = of_property_read_string(child, "dsminfo",
			&btb_check_gpio_info[gpio_index].dsm_notify_info);
		if (ret)
			pr_err("get child cam %d dsminfo fail!", gpio_index);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++)
		pr_debug("imgsensor %d gpio0 %u gpio1 %u dsminfo %s", i,
			btb_check_gpio_info[i].gpio[0], btb_check_gpio_info[i].gpio[1],
			btb_check_gpio_info[i].dsm_notify_info);

	dts_get_flag = 1;

	return;
}

static int check_cable(unsigned int cable_gpio)
{
	int ret;

	ret = gpio_request(cable_gpio, NULL);
	if (ret) {
		pr_err("cable gpio gpio_request fail");
		return 0;
	}
	ret = gpio_get_value(cable_gpio);

	gpio_free(cable_gpio);

	pr_info("cable gpio %u ret=%d", cable_gpio, ret);

	return ret;
}

static int check_gpio_info(struct btb_check_gpio_info *gpio_info)
{
	unsigned int *gpio_ptr = gpio_info->gpio;
	int ret = 0;
	int i = 0;

	while (i < MAX_BTB_CHECK_GPIO_PER_CAM &&
		*(gpio_ptr + i) != GPIO_INIT_VALUE) {
		pr_debug("enter cable gpio %u", *(gpio_ptr + i));
		ret = check_cable(*(gpio_ptr + i));
		if (ret) {
			pr_err("cable gpio %u ret=%d",
				*(gpio_ptr + i), ret);
			break;
		}
		i++;
	}

	if (ret)
		return *(gpio_ptr + i);

	return ret;
}

void check_camera_btb_gpio_info(int sensor_index)
{
	struct btb_check_gpio_info *cur_node_ptr = NULL;
	int ret;

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
		sensor_index < IMGSENSOR_SENSOR_IDX_MIN_NUM)
		return;

	get_btb_check_gpio_info_from_dts();

	cur_node_ptr = &btb_check_gpio_info[sensor_index];
	if (cur_node_ptr->gpio[0] == GPIO_INIT_VALUE) { // not init.
		pr_err("cable gpio not init sensor idx %d", sensor_index);
		return;
	}

	ret = check_gpio_info(cur_node_ptr);
	pr_debug("cable gpio sensor idx %d, gpio %u, ret %d",
		sensor_index, cur_node_ptr->gpio[0], ret);
	if (ret)
		(void)camera_dsm_report_info(DSM_CAMERA_I2C_ERR,
			"btb gpio (%d) check faild: %s\n",
			ret, cur_node_ptr->dsm_notify_info);
}
