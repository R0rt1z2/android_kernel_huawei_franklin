/*
 * wireless_tx_pwr_src.c
 *
 * power source for wireless reverse charging
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
#include <linux/delay.h>
#include <huawei_platform/log/hw_log.h>
#include <chipset_common/hwpower/hardware_ic/boost_5v.h>
#include <chipset_common/hwpower/hardware_ic/charge_pump.h>
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <chipset_common/hwpower/direct_charge/direct_charge_ic_manager.h>
#include <huawei_platform/power/wireless/wireless_tx_pwr_src.h>
#include <chipset_common/hwpower/wireless_charge/wireless_power_supply.h>
#include <chipset_common/hwpower/wireless_charge/wireless_trx_ic_intf.h>
#include <huawei_platform/power/wireless/wireless_transmitter.h>

#define HWLOG_TAG wireless_tx_pwr_src
HWLOG_REGIST();

static struct {
	enum wltx_pwr_src src;
	const char *name;
} const g_pwr_src[] = {
	{ PWR_SRC_NULL, "PWR_SRC_NULL" },
	{ PWR_SRC_VBUS, "PWR_SRC_VBUS" },
	{ PWR_SRC_OTG, "PWR_SRC_OTG" },
	{ PWR_SRC_5VBST, "PWR_SRC_5VBST" },
	{ PWR_SRC_SPBST, "PWR_SRC_SPBST" },
	{ PWR_SRC_VBUS_CP, "PWR_SRC_VBUS_CP" },
	{ PWR_SRC_OTG_CP, "PWR_SRC_OTG_CP" },
	{ PWR_SRC_BP2CP, "PWR_SRC_BP2CP" },
	{ PWR_SRC_NA, "PWR_SRC_NA" },
};

const char *wltx_get_pwr_src_name(enum wltx_pwr_src src)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_pwr_src); i++) {
		if (src == g_pwr_src[i].src)
			return g_pwr_src[i].name;
	}
	return "NA";
}

static enum wltx_pwr_src wltx_set_vbus_output(bool enable)
{
	return PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_otg_output(bool enable)
{
	return PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_5vbst_output(bool enable)
{
	wlps_control(WLTRX_IC_MAIN, WLPS_TX_SW, enable ? true : false);
	usleep_range(1000, 1050); /* 1ms */
	boost_5v_enable(enable, BOOST_CTRL_WLTX);
	return enable ? PWR_SRC_5VBST : PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_spbst_output(bool enable)
{
	wlps_control(WLTRX_IC_MAIN, WLPS_TX_PWR_SW, enable ? true : false);
	if (enable)
		(void)charge_pump_reverse_chip_init(CP_TYPE_MAIN);
	return enable ? PWR_SRC_SPBST : PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_bp2cp_output(bool enable)
{
	if (enable)
		(void)charge_pump_set_reverse_bp2cp_mode(CP_TYPE_MAIN);

	return enable ? PWR_SRC_BP2CP : PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_adaptor_output(int vset, int iset)
{
	return PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_reset_adaptor_output(void)
{
	return PWR_SRC_NULL;
}

static enum wltx_pwr_src wltx_set_vbus_cp_output(bool enable)
{
	int ret;
	enum wltx_pwr_src src;
	struct wltx_dev_info *di = wltx_get_dev_info();

	if (!di)
		return PWR_SRC_NULL;

	if (!enable)
		return wltx_reset_adaptor_output();

	if (di->ps_need_ext_pwr)
		wlps_control(WLTRX_IC_MAIN, WLPS_RX_EXT_PWR, true);

	src = wltx_set_adaptor_output(WLTX_SC_ADAP_VSET, WLTX_SC_ADAP_ISET);
	if (src != PWR_SRC_VBUS)
		goto fail;

	ret = charge_pump_reverse_cp_chip_init(CP_TYPE_MAIN);
	if (ret)
		hwlog_err("set_vbus_cp_output: set cp failed\n");
	else
		src = PWR_SRC_VBUS_CP;

fail:
	if (di->ps_need_ext_pwr)
		wlps_control(WLTRX_IC_MAIN, WLPS_RX_EXT_PWR, false);
	return src;
}

static enum wltx_pwr_src wltx_set_otg_cp_output(bool enable)
{
	int ret;
	enum wltx_pwr_src src;
	struct wltx_dev_info *di = wltx_get_dev_info();

	if (!di)
		return PWR_SRC_NULL;

	if (!enable)
		return wltx_set_otg_output(false);

	if (di->ps_need_ext_pwr)
		wlps_control(WLTRX_IC_MAIN, WLPS_RX_EXT_PWR, true);

	src = wltx_set_otg_output(true);
	if (src != PWR_SRC_OTG)
		goto fail;

	ret = charge_pump_reverse_cp_chip_init(CP_TYPE_MAIN);
	if (ret)
		hwlog_err("set_otg_cp_output: set cp failed\n");
	else
		src = PWR_SRC_OTG_CP;

fail:
	if (di->ps_need_ext_pwr)
		wlps_control(WLTRX_IC_MAIN, WLPS_RX_EXT_PWR, false);
	return src;
}

enum wltx_pwr_src wltx_set_pwr_src_output(bool enable, enum wltx_pwr_src src)
{
	hwlog_info("[set_pwr_src_output] src:%s\n", wltx_get_pwr_src_name(src));
	switch (src) {
	case PWR_SRC_VBUS:
		return wltx_set_vbus_output(enable);
	case PWR_SRC_OTG:
		return wltx_set_otg_output(enable);
	case PWR_SRC_5VBST:
		return wltx_set_5vbst_output(enable);
	case PWR_SRC_SPBST:
		return wltx_set_spbst_output(enable);
	case PWR_SRC_VBUS_CP:
		return wltx_set_vbus_cp_output(enable);
	case PWR_SRC_OTG_CP:
		return wltx_set_otg_cp_output(enable);
	case PWR_SRC_BP2CP:
		return wltx_set_bp2cp_output(enable);
	case PWR_SRC_NULL:
		return PWR_SRC_NULL;
	default:
		return PWR_SRC_NA;
	}
}

enum wltx_pwr_src wltx_get_cur_pwr_src(void)
{
	struct wltx_dev_info *di = wltx_get_dev_info();

	if (!di)
		return PWR_SRC_NA;

	return di->cur_pwr_src;
}
