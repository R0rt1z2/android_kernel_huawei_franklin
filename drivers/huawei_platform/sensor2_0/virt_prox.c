/*
 * virt_prox.c
 *
 * code for virt_prox
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#include "sensor_scp.h"
#include "securec.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#ifdef CONFIG_HUAWEI_THP
#include "huawei_thp_attr.h"
#endif

#define pr_fmt(fmt) "[virt_prox] " fmt
#define VIRT_PROX_FAR             5
struct virt_prox_t {
	struct hf_device hf_dev;
	// here add other paras if needed
};
static struct virt_prox_t *virt_prox;
static struct sensor_info virt_prox_info;
static int report_data;

static void virt_prox_exit(void)
{
}

static int virt_prox_batch(struct hf_device *hfdev, int sensor_type,
	int64_t delay, int64_t latency)
{
	if (!hfdev) {
		pr_err("hfdev para error\n");
		return -1;
	}
	return 0;
}

static int virt_prox_flush(struct hf_device *hfdev, int sensor_type)
{
	int ret;
	struct hf_manager_event event;
	struct hf_manager *manager = NULL;

	if (!virt_prox)
		return 0;
	if (report_data != 0)
		report_data = VIRT_PROX_FAR;

	manager = virt_prox->hf_dev.manager;
	if (!manager)
		return 0;
	ret = memset_s(&event, sizeof(struct hf_manager_event), 0,
		sizeof(struct hf_manager_event));
	if (ret != EOK)
		return 0;

	event.timestamp = ktime_get_boot_ns();
	event.sensor_type = SENSOR_TYPE_PROXIMITY;
	event.accurancy = SENSOR_ACCURANCY_HIGH;
	event.action = FLUSH_ACTION;
	event.word[0] = report_data;
	pr_info("%s data = %d\n", __func__, report_data);
	if (manager->report)
		manager->report(manager, &event);
	return 0;
}

static int virt_prox_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	if (!hfdev) {
		pr_err("hfdev para error\n");
		return -1;
	}
	pr_info("%s cmd = %d\n", __func__, en);

#ifdef CONFIG_HUAWEI_THP
	pr_info("%s thp cmd = %d\n", __func__, en);
	if (en)
		return thp_set_prox_switch_status(true);
	else
		return thp_set_prox_switch_status(false);
#else
	return 0;
#endif
}

static int virt_prox_data_update(int prox)
{
	int ret;
	struct hf_manager_event event;
	struct hf_manager *manager = NULL;

	if (!virt_prox)
		return 0;
	report_data = prox;
	if (prox != 0)
		prox = VIRT_PROX_FAR;

	manager = virt_prox->hf_dev.manager;
	if (!manager)
		return 0;
	ret = memset_s(&event, sizeof(struct hf_manager_event), 0,
		sizeof(struct hf_manager_event));
	if (ret != EOK)
		return 0;

	event.timestamp = ktime_get_boot_ns();
	event.sensor_type = SENSOR_TYPE_PROXIMITY;
	event.accurancy = SENSOR_ACCURANCY_HIGH;
	event.action = DATA_ACTION;
	event.word[0] = prox;
	pr_info("%s data = %d\n", __func__, prox);
	if (manager->report)
		manager->report(manager, &event);
	return 0;
}

int thp_prox_event_report(const int value[], int length)
{
	if (length <= 0)
		return 0;

	return virt_prox_data_update(value[0]);
}

static int virt_prox_sup_check(void)
{
	struct device_node *vp_node = NULL;
	unsigned int vp_support = 0;
	int ret;

	pr_info("%s\n", __func__);
	vp_node = of_find_compatible_node(NULL, NULL,
		"huawei,huawei_sensor_info");
	if (!vp_node) {
		pr_info("%s has no huawei_scp_info node\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(vp_node, "huawei_virt_prox",
		&vp_support);
	if (!ret) {
		pr_info("find huawei_virt_prox = %u\n", vp_support);
	} else {
		pr_info("not support virt prox feature\n");
		return -EINVAL;
	}

	if (vp_support != 1) { // only 1 means support v prox sensor
		pr_info("vp_support value not support virt prox feature\n");
		return -EINVAL;
	}
	return vp_support;
}

static int virt_prox_manager_init(void)
{
	int err;

	virt_prox = kzalloc(sizeof(struct virt_prox_t), GFP_KERNEL);
	if (!virt_prox) {
		err = -ENOMEM;
		return err;
	}
	virt_prox_info.sensor_type = SENSOR_TYPE_PROXIMITY;
	virt_prox_info.gain = 1;
	strlcpy(virt_prox_info.name, "proximity-tp", sizeof(virt_prox_info.name) - 1);
	strlcpy(virt_prox_info.vendor, "proximity-tp", sizeof(virt_prox_info.vendor) - 1);

	virt_prox->hf_dev.dev_name = "virt_prox_mg";
	virt_prox->hf_dev.device_poll = HF_DEVICE_IO_INTERRUPT;
	virt_prox->hf_dev.device_bus = HF_DEVICE_IO_ASYNC;
	virt_prox->hf_dev.support_list = &virt_prox_info;
	virt_prox->hf_dev.support_size = 1; // only 1 sensor
	virt_prox->hf_dev.enable = virt_prox_enable;
	virt_prox->hf_dev.batch = virt_prox_batch;
	virt_prox->hf_dev.flush = virt_prox_flush;
	return 0;
}

static int __init virt_prox_init(void)
{
	int err;

	pr_info("%s\n", __func__);
	if (virt_prox_sup_check() < 0)
		return -EINVAL;

	err = virt_prox_manager_init();
	if (err < 0) {
		pr_err("virt_prox_manager_init init fail\n");
		return err;
	}
	err = hf_manager_create(&virt_prox->hf_dev);
	if (err < 0) {
		pr_err("%s hf_manager_create fail\n", __func__);
		kfree(virt_prox);
		virt_prox = NULL;
		err = -1;
		return err;
	}
	pr_info("virt_prox_init success\n");
	return 0;
}

module_init(virt_prox_init);
module_exit(virt_prox_exit);

MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_DESCRIPTION("vp prox driver");
MODULE_LICENSE("GPL v2");
