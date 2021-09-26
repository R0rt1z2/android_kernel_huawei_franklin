/*
 * dp_debug.c
 *
 * hw dp debug
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <hw_dp_def.h>
#include <huawei_platform/dp/dp_factory.h>

#define HWLOG_TAG dp_debug
HWLOG_REGIST();

#define DP_DEBUG_BUF_SIZE (PAGE_SIZE)

uint8_t g_dp_msg_level = 4;
bool g_enable_send_event = true;

#ifdef DP_DEBUG_ENABLE
static int dp_factory_event_set(const char *val, const struct kernel_param *kp)
{
	int link_state;
	int ret;

	UNUSED(kp);
	if (val == NULL) {
		HW_DP_ERR("input is NULL\n");
		return -EINVAL;
	}

	ret = kstrtoint(val, 0, &link_state);
	if (ret != 0 || link_state < 0 ||
		link_state >= DP_MANUFACTURE_LINK_STATE_MAX) {
		HW_DP_ERR("invalid params num %d\n", ret);
		return -EINVAL;
	}

	dp_factory_send_event(link_state);
	return 0;
}

static int dp_debug_level_get(char *buf, const struct kernel_param *kp)
{
	UNUSED(kp);
	if (buf == NULL) {
		HW_DP_ERR("buf is NULL\n");
		return -EINVAL;
	}

	return snprintf_s(buf, DP_DEBUG_BUF_SIZE, DP_DEBUG_BUF_SIZE - 1,
		"dp_debug_level:%d\n", g_dp_msg_level);
}

static int dp_debug_level_set(const char *val, const struct kernel_param *kp)
{
	int level;
	int ret;

	UNUSED(kp);
	if (val == NULL) {
		HW_DP_ERR("input is NULL\n");
		return -EINVAL;
	}

	ret = kstrtoint(val, 0, &level);
	if (ret != 0) {
		HW_DP_ERR("invalid params num %d\n", ret);
		return -EINVAL;
	}

	g_dp_msg_level = level;
	return 0;
}

static int dp_enable_send_event_get(char *buf, const struct kernel_param *kp)
{
	UNUSED(kp);
	if (buf == NULL) {
		HW_DP_ERR("buf is NULL\n");
		return -EINVAL;
	}

	return snprintf_s(buf, DP_DEBUG_BUF_SIZE, DP_DEBUG_BUF_SIZE - 1,
		"send_event status:%d\n", (char)g_enable_send_event);
}

static int dp_enable_send_event_set(const char *val,
	const struct kernel_param *kp)
{
	int enable;
	int ret;

	UNUSED(kp);
	if (val == NULL) {
		HW_DP_ERR("input is NULL\n");
		return -EINVAL;
	}

	ret = kstrtoint(val, 0, &enable);
	if (ret != 0) {
		HW_DP_ERR("invalid params num %d\n", ret);
		return -EINVAL;
	}

	g_enable_send_event = (bool)enable;
	return 0;
}


static struct kernel_param_ops dp_factory_event_ops = {
	.get = NULL,
	.set = dp_factory_event_set,
};

static struct kernel_param_ops dp_debug_level_ops = {
	.get = dp_debug_level_get,
	.set = dp_debug_level_set,
};

static struct kernel_param_ops dp_enable_send_event_ops = {
	.get = dp_enable_send_event_get,
	.set = dp_enable_send_event_set,
};

module_param_cb(dp_factory_event, &dp_factory_event_ops, NULL, 0644);
module_param_cb(dp_debug_level, &dp_debug_level_ops, NULL, 0644);
module_param_cb(dp_enable_send_event, &dp_enable_send_event_ops, NULL, 0644);

static int __init dp_debug_init(void)
{
	HW_DP_INFO("enter\n");
	g_dp_msg_level = 4;
	g_enable_send_event = true;
	HW_DP_INFO("init success\n");
	return 0;
}

static void __exit dp_debug_exit(void)
{
}

module_init(dp_debug_init);
module_exit(dp_debug_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei dp factory driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

