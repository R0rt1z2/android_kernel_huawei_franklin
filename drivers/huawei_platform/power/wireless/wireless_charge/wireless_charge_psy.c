/*
 * wireless_charge_psy.c
 *
 * wireless charge driver, function as power supply
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

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/charger/huawei_charger.h>
#include <huawei_platform/power/huawei_charger_common.h>
#include <chipset_common/hwpower/common_module/power_ui_ne.h>
#include <chipset_common/hwpower/common_module/power_event_ne.h>
#include <chipset_common/hwpower/wireless_charge/wireless_rx_status.h>
#include <mt-plat/charger_type.h>
#include <huawei_platform/power/wireless/wireless_charger.h>

#define HWLOG_TAG wireless_charge_psy
HWLOG_REGIST();

static int wlc_psy_online_changed(bool online)
{
	static struct power_supply *psy = NULL;
	union power_supply_propval pval;

	if (!psy) {
		psy = power_supply_get_by_name("charger");
		if (!psy) {
			hwlog_err("%s: get power supply failed\n", __func__);
			return -EINVAL;
		}
	}

	pval.intval = online;

	return power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &pval);
}

static int wlc_psy_chg_type_changed(bool online)
{
	static struct power_supply *psy = NULL;
	union power_supply_propval pval;
	enum charger_type chg_type;

	if (!psy) {
		psy = power_supply_get_by_name("charger");
		if (!psy) {
			hwlog_err("%s: get power supply failed\n", __func__);
			return -EINVAL;
		}
	}

	chg_type = mt_get_charger_type();
	if (!online && (chg_type != WIRELESS_CHARGER)) {
		hwlog_err("%s: charger_type=%d\n", __func__, chg_type);
		return 0;
	}

	if (online) {
		pval.intval = WIRELESS_CHARGER;
		(void)charge_enable_powerpath(true);
	} else {
		pval.intval = CHARGER_UNKNOWN;
		(void)charge_enable_powerpath(false);
	}

	return power_supply_set_property(psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &pval);
}

void wlc_handle_sink_event(bool sink_flag)
{
	if (sink_flag) {
		power_event_bnc_notify(POWER_BNT_CONNECT, POWER_NE_WIRELESS_CONNECT, NULL);
	} else {
		power_event_bnc_notify(POWER_BNT_CONNECT, POWER_NE_WIRELESS_DISCONNECT, NULL);
		if (wlrx_get_wired_channel_state() != WIRED_CHANNEL_ON)
			charge_send_icon_uevent(ICON_TYPE_INVALID);
	}

	if (wlc_psy_online_changed(sink_flag) < 0)
		hwlog_err("%s: report psy online failed\n", __func__);

	if (wlc_psy_chg_type_changed(sink_flag) < 0)
		hwlog_err("%s: report psy chg_type failed\n", __func__);
}

static int wireless_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	enum charger_type chg_type;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		chg_type = mt_get_charger_type();
		if (chg_type == WIRELESS_CHARGER)
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property g_wireless_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc g_wireless_desc = {
	.name = "Wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = g_wireless_properties,
	.num_properties = ARRAY_SIZE(g_wireless_properties),
	.get_property = wireless_get_property,
};

int wlc_power_supply_register(struct platform_device *pdev)
{
	struct power_supply *psy = NULL;

	psy = power_supply_register(&pdev->dev, &g_wireless_desc, NULL);
	if (IS_ERR(psy)) {
		hwlog_err("power_supply_register failed\n");
		return PTR_ERR(psy);
	}

	return 0;
}
