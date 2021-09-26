/*
 * hw_dp.c
 *
 * hw DP module
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

#include <hw_dp_def.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <huawei_platform/dp/dp_factory.h>

#define HWLOG_TAG hw_dp
HWLOG_REGIST();

#define DP_KB_STATE_MAX 2

struct dp_device_type {
	const char *name;
	struct device *dev;
	int index;
};

static struct dp_device_type hw_dp_device[] = {
	{
		.name  = "source",
		.dev   = NULL,
		.index = 0,
	},
};

#define DP_DEVICE_NUM ARRAY_SIZE(hw_dp_device)

struct hw_dp_priv {
	struct class *class;
	bool parse_dts_flag;
	enum dp_source_mode source_mode;
};

static struct hw_dp_priv g_dp_priv;

__attribute__ ((weak)) int dptx_switch_source(int source)
{
	UNUSED(source);
	HW_DP_INFO("use weak\n");
	return 0;
}

static ssize_t dp_source_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	UNUSED(dev);
	UNUSED(attr);
	if (buf == NULL) {
		HW_DP_ERR("buf is NULL\n");
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", g_dp_priv.source_mode);
}

static ssize_t dp_source_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int last_mode;
	int source_mode;
	int ret;

	UNUSED(dev);
	UNUSED(attr);
	if (buf == NULL) {
		HW_DP_ERR("buf is NULL\n");
		return -EINVAL;
	}

	last_mode = g_dp_priv.source_mode;
	ret = kstrtoint(buf, 0, &source_mode);
	if (ret != 0) {
		HW_DP_ERR("store error\n");
		return -EINVAL;
	}

	if (source_mode != SAME_SOURCE && source_mode != DIFF_SOURCE) {
		HW_DP_INFO("invalid source_mode %d\n", source_mode);
		return -EINVAL;
	}

	g_dp_priv.source_mode = source_mode;
	if (last_mode == g_dp_priv.source_mode) {
		HW_DP_INFO("sync framework source mode state %d\n",
			g_dp_priv.source_mode);
	} else {
		HW_DP_INFO("store %d success\n", g_dp_priv.source_mode);
		dptx_switch_source(source_mode);
	}

	return count;
}

static struct device_attribute hw_dp_device_attr[] = {
	__ATTR(source_mode, 0644, dp_source_mode_show, dp_source_mode_store),
};

void dp_state_event_report(const char *event)
{
	char event_buf[DP_LINK_EVENT_BUF_MAX] = { 0 };
	char *envp[DP_KB_STATE_MAX] = { event_buf, NULL };
	int ret;

	if (event == NULL) {
		HW_DP_ERR("input is NULL\n");
		return;
	}

	ret = snprintf_s(event_buf, sizeof(event_buf), strlen(event), event);
	if (ret < 0) {
		HW_DP_ERR("snprintf_s ret err:%d\n", ret);
		return;
	}

	ret = kobject_uevent_env(&hw_dp_device[0].dev->kobj, KOBJ_CHANGE, envp);
	if (ret < 0)
		HW_DP_ERR("send uevent failed %d\n", ret);
	else
		HW_DP_INFO("send uevent %s success\n", envp[0]);
}
EXPORT_SYMBOL_GPL(dp_state_event_report);

static void dp_source_mode_parse_dts(void)
{
	struct device_node *np = NULL;
	int value = 0;
	int ret;

	HW_DP_INFO("enter\n");
	if (g_dp_priv.parse_dts_flag) {
		HW_DP_INFO("source mode has already parsed\n");
		return;
	}
	g_dp_priv.parse_dts_flag = true;

	np = of_find_compatible_node(NULL, NULL, "huawei,display_port");
	if (np == NULL) {
		HW_DP_INFO("not found dts node\n");
		g_dp_priv.source_mode = SAME_SOURCE;
		return;
	}

	if (!of_device_is_available(np)) {
		HW_DP_ERR("dts %s not available\n", np->name);
		g_dp_priv.source_mode = SAME_SOURCE;
		return;
	}

	ret = of_property_read_u32(np, "dp_default_source_mode", &value);
	if (ret) {
		HW_DP_INFO("get default source_mode failed %d\n", ret);
		g_dp_priv.source_mode = SAME_SOURCE;
		return;
	}

	if (value == 0)
		g_dp_priv.source_mode = DIFF_SOURCE;
	else
		g_dp_priv.source_mode = SAME_SOURCE;

	if (dp_factory_mode_enable()) {
		g_dp_priv.source_mode = SAME_SOURCE;
		HW_DP_INFO("only support same source in factory version\n");
	}

	HW_DP_INFO("get source mode %d success\n", g_dp_priv.source_mode);
}

int dp_get_current_source_mode(void)
{
	if (!g_dp_priv.parse_dts_flag)
		dp_source_mode_parse_dts();

	return g_dp_priv.source_mode;
}
EXPORT_SYMBOL_GPL(dp_get_current_source_mode);

static void dp_dev_distroy(struct class *class)
{
	int i;

	for (i = 0; i < DP_DEVICE_NUM; i++) {
		if (!IS_ERR_OR_NULL(hw_dp_device[i].dev)) {
			device_destroy(class, hw_dp_device[i].dev->devt);
			hw_dp_device[i].dev = NULL;
		}
	}
}

static int dp_dev_create(struct class *class)
{
	int i;
	int ret = 0;

	for (i = 0; i < DP_DEVICE_NUM; i++) {
		hw_dp_device[i].dev = device_create(class, NULL, 0, NULL,
			hw_dp_device[i].name);
		if (IS_ERR_OR_NULL(hw_dp_device[i].dev)) {
			ret = PTR_ERR(hw_dp_device[i].dev);
			goto exit_device_destroy;
		}
	}

	return ret;

exit_device_destroy:
	dp_dev_distroy(class);
	return ret;
}

static int dp_create_file(void)
{
	int ret;
	int created_num;
	int i;

	for (i = 0; i < DP_DEVICE_NUM; i++) {
		ret = device_create_file(hw_dp_device[i].dev,
			&hw_dp_device_attr[i]);
		if (ret != 0) {
			HW_DP_ERR("%s create failed\n", hw_dp_device[i].name);
			created_num = i;
			goto err_out;
		}
	}

	return 0;

err_out:
	for (i = 0; i < created_num; i++)
		device_remove_file(hw_dp_device[i].dev, &hw_dp_device_attr[i]);
	return ret;
}

static int __init hw_dp_init(void)
{
	struct class *dp_class = NULL;
	int ret;

	HW_DP_INFO("enter\n");
	ret = memset_s(&g_dp_priv, sizeof(g_dp_priv), 0, sizeof(g_dp_priv));
	if (ret != 0)
		HW_DP_INFO("create dp source class failed\n");

	g_dp_priv.parse_dts_flag = false;
	g_dp_priv.source_mode = SAME_SOURCE;
	dp_source_mode_parse_dts();

	dp_class = class_create(THIS_MODULE, "dp");
	if (IS_ERR(dp_class)) {
		HW_DP_ERR("create dp source class failed\n");
		return PTR_ERR(dp_class);
	}
	g_dp_priv.class = dp_class;

	ret = dp_dev_create(g_dp_priv.class);
	if (ret < 0)
		goto exit_class_destroy;

	ret = dp_create_file();
	if (ret < 0)
		goto exit_device_destroy;

	HW_DP_INFO("init success\n");
	return 0;

exit_device_destroy:
	dp_dev_distroy(g_dp_priv.class);
exit_class_destroy:
	class_destroy(g_dp_priv.class);
	return ret;
}

static void __exit hw_dp_exit(void)
{
	int i;

	hwlog_info("%s: enter\n", __func__);
	for (i = 0; i < DP_DEVICE_NUM; i++) {
		if (!IS_ERR_OR_NULL(hw_dp_device[i].dev)) {
			device_remove_file(hw_dp_device[i].dev,
				&hw_dp_device_attr[i]);
			device_destroy(g_dp_priv.class,
				hw_dp_device[i].dev->devt);
			hw_dp_device[i].dev = NULL;
		}
	}

	class_destroy(g_dp_priv.class);
	g_dp_priv.class = NULL;
}

module_init(hw_dp_init);
module_exit(hw_dp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei dp factory driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

