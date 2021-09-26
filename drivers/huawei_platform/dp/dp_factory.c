/*
 * dp_factory.c
 *
 * factory test for DP module
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

#include <huawei_platform/dp/dp_factory.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <hw_dp_def.h>

#define HWLOG_TAG dp_factory
HWLOG_REGIST();

#ifndef DP_FACTORY_MODE_ENABLE
bool dp_factory_mode_enable(void)
{
	return false;
}
EXPORT_SYMBOL_GPL(dp_factory_mode_enable);

void dp_factory_send_event(enum dp_manufacture_link_state event)
{
}

void dp_factory_get_lane_rate(uint8_t rate, uint8_t lanes,
	uint8_t max_rate, uint8_t max_lanes)
{
}

void dp_factory_get_h_v_active(uint16_t hac, uint16_t vac, uint8_t fps)
{
}

#else // DP_FACTORY_MODE_ENABLE defined

struct dp_factory_priv {
	// actual link rate and lanes
	uint16_t h_active;
	uint16_t v_active;
	uint8_t link_rate;
	uint8_t link_lanes;
	uint8_t fps;
	// according to typec cable: dock or dongle
	uint8_t max_rate;
	uint8_t max_lanes;

	uint8_t check_max_lanes; // platform support max lanes cap
	uint8_t check_max_rate; // platform support max rate cap
	bool check_display_4k;
	bool check_display_60fps;
	bool need_report_event; // report event or not for MMIE
	bool check_lanes_rate;
};

struct dp_factory_link_state_to_event {
	enum dp_manufacture_link_state state;
	char *event;
};

static struct dp_factory_link_state_to_event g_dp_factory_link_event[] = {
	// for factory version: MMIE test
	{ DP_MANUFACTURE_LINK_CABLE_IN,
		"MANUFACTURE_DP_LINK_EVENT=CABLE_IN" },
	{ DP_MANUFACTURE_LINK_CABLE_OUT,
		"MANUFACTURE_DP_LINK_EVENT=CABLE_OUT" },
	{ DP_MANUFACTURE_LINK_AUX_FAILED,
		"MANUFACTURE_DP_LINK_EVENT=AUX_FAILED" },
	{ DP_MANUFACTURE_LINK_SAFE_MODE,
		"MANUFACTURE_DP_LINK_EVENT=SAFE_MODE" },
	{ DP_MANUFACTURE_LINK_EDID_FAILED,
		"MANUFACTURE_DP_LINK_EVENT=EDID_FAILED" },
	{ DP_MANUFACTURE_LINK_LINK_FAILED,
		"MANUFACTURE_DP_LINK_EVENT=LINK_FAILED" },
	{ DP_MANUFACTURE_LINK_HPD_NOT_EXISTED,
		"MANUFACTURE_DP_LINK_EVENT=HPD_NOT_EXISTED" },
	{ DP_MANUFACTURE_LINK_REDUCE_RATE,
		"MANUFACTURE_DP_LINK_EVENT=LINK_REDUCE_RATE" },
	{ DP_MANUFACTURE_LINK_INVALID_COMBINATIONS,
		"MANUFACTURE_DP_LINK_EVENT=INVALID_COMBINATIONS" },
};
#define DP_MANUFACTURE_LINK_EVENT_CNT ARRAY_SIZE(g_dp_factory_link_event)

static struct dp_factory_priv *g_dp_factory_priv;

bool dp_factory_mode_enable(void)
{
	return true;
}

void dp_factory_send_event(enum dp_manufacture_link_state event)
{
	int i;
	struct dp_factory_priv *priv = g_dp_factory_priv;

	if (priv == NULL) {
		HW_DP_INFO("need not report event\n");
		return;
	}

	if (!g_enable_send_event) {
		HW_DP_DEBUG("report event is disabled\n");
		return;
	}

	for (i = 0; i < DP_MANUFACTURE_LINK_EVENT_CNT; i++) {
		if (event == g_dp_factory_link_event[i].state) {
			dp_state_event_report(g_dp_factory_link_event[i].event);
			return;
		}
	}
}

void dp_factory_get_lane_rate(uint8_t rate, uint8_t lanes,
	uint8_t max_rate, uint8_t max_lanes)
{
	struct dp_factory_priv *priv = g_dp_factory_priv;

	if (priv == NULL) {
		HW_DP_ERR("priv is NULL\n");
		return;
	}
	HW_DP_INFO("support max cap:rate:%d,lanes:%d,link training:rate:%d,lanes:%d\n",
		max_rate, max_lanes, rate, lanes);

	priv->link_rate = rate;
	priv->link_lanes = lanes;
	priv->max_rate = max_rate;
	priv->max_lanes = max_lanes;
	if (priv->check_lanes_rate && (priv->link_rate < priv->check_max_rate ||
		priv->link_lanes < priv->check_max_lanes)) {
		HW_DP_ERR("not support reduce lanes or rate in factory mode\n");
		dp_factory_send_event(DP_MANUFACTURE_LINK_REDUCE_RATE);
	}
}

void dp_factory_get_h_v_active(uint16_t hac, uint16_t vac, uint8_t fps)
{
	struct dp_factory_priv *priv = g_dp_factory_priv;

	if (priv == NULL) {
		HW_DP_ERR("priv is NULL\n");
		return;
	}
	HW_DP_INFO("link training h_act:%d, v_act:%d, fps:%d\n", hac, vac, fps);

	priv->h_active = hac;
	priv->v_active = vac;
	priv->fps = fps;
	if (priv->check_display_4k && (priv->h_active < DP_FACTORY_H_ACTIVE ||
		priv->v_active < DP_FACTORY_V_ACTIVE)) {
		HW_DP_ERR("not 4K display, INVALID_COMBINATIONS\n");
		dp_factory_send_event(DP_MANUFACTURE_LINK_INVALID_COMBINATIONS);
	}
}

static void dp_factory_parse_dts(struct dp_factory_priv *priv)
{
	struct device_node *np = NULL;

	np = of_find_compatible_node(NULL, NULL, "huawei,display_port");
	if (!of_device_is_available(np)) {
		HW_DP_INFO("dts display_port not available\n");
		return;
	}

	DP_GET_BOOL_DTS_PROP(priv->check_lanes_rate, np, check_lanes_rate);
	DP_GET_BOOL_DTS_PROP(priv->check_display_4k, np, check_display_4k);
	DP_GET_BOOL_DTS_PROP(priv->check_display_60fps, np, check_display_60fps);
	DP_GET_BOOL_DTS_PROP(priv->need_report_event, np, need_report_event);
	DP_GET_U32_DTS_PROP(priv->check_max_lanes, np, check_max_lanes);
	DP_GET_U32_DTS_PROP(priv->check_max_rate, np, check_max_rate);
}

static int __init dp_factory_init(void)
{
	struct dp_factory_priv *priv = NULL;

	HW_DP_INFO("enter\n");
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	dp_factory_parse_dts(priv);
	g_dp_factory_priv = priv;
	HW_DP_INFO(" init success\n", __func__);
	return 0;
}

static void __exit dp_factory_exit(void)
{
	HW_DP_INFO("enter\n");
	if (g_dp_factory_priv != NULL)
		kfree(g_dp_factory_priv);
	g_dp_factory_priv = NULL;
}

module_init(dp_factory_init);
module_exit(dp_factory_exit);

#endif // DP_FACTORY_MODE_ENABLE end.

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei dp factory driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

