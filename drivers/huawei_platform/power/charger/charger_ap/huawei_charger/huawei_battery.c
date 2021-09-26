/*
 * huawei_battery.c
 *
 * huawei battery driver
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/power/huawei_charger.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/power/huawei_mtk_charger.h>

#include <mt-plat/mtk_battery.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_class.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/charger_type.h>
#include <pmic.h>
#include <mtk_charger_intf.h>
#include <huawei_platform/power/huawei_charger_common.h>
#include <chipset_common/hwpower/common_module/power_cmdline.h>
#include <huawei_platform/log/hw_log.h>
#include <chipset_common/hwpower/common_module/power_dts.h>
#include <chipset_common/hwpower/common_module/power_supply_interface.h>
#include <chipset_common/hwpower/common_module/power_common.h>

#ifdef HWLOG_TAG
#undef HWLOG_TAG
#endif
#define HWLOG_TAG huawei_battery
HWLOG_REGIST();

static int g_factory_diag;
static int g_hz_onoff;
static int g_charge_enable;
static int g_input_current = -1;
static int g_wdt_disable;
static struct charger_consumer *g_pconsumer;
static int g_battery_has_removed_flag;
#define LOW_BATT_TEMP                   2
#define LOW_BATT_VOL                    3700
#define BAT_DIVI                        10
#define UA_PER_MA                       1000
#define MIN_INPUT_CURRENT               100000
#define DEFAULT_FASTCHG_MAX_VOLTAGE     4400000
#define DEFAULT_USB_ICL_CURRENT         2000
#define MV_TO_UV                        1000
#define DEFAULT_BATTERY_TEMP            250
#define DEFAULT_BATT_CAPACITY           50
#define DUMP_REG_LEN                    3900
#define IS_IN_HIZ_MODE                  1
#define LIMIT_CURRENT_TOO_LOW           1

struct huawei_battery_info {
	struct device *dev;
	struct power_supply *batt_psy;
	enum power_supply_property *batt_props;
	int num_props;
	struct power_supply *bk_batt_psy;
	struct power_supply *usb_psy;
	struct power_supply *ac_psy;
	struct power_supply *hwbatt_psy;
	struct power_supply_desc hwbatt_desc;
	struct power_supply_config hwbatt_cfg;
	struct power_supply *extra_batt_psy;
	struct power_supply_desc batt_extra_psy_desc;
	struct power_supply_config batt_cfg;
	int charger_batt_capacity_mah;
	bool power_path;
	int icl_current;
	int voltage_max_design;
	int use_third_coul;
};

struct huawei_battery_info *g_info;

int get_input_current(void)
{
	return g_input_current;
}

static int __init battery_has_removed_parse_cmd(char *p)
{
	int ret;

	if (!p)
		return 0;

	ret = kstrtoint(p, POWER_BASE_DEC, &g_battery_has_removed_flag);
	pr_info("[%s] parse_cmd ret = %d\n", __func__, ret);

	return 0;
}
early_param("has_battery_removed", battery_has_removed_parse_cmd);

int power_get_battery_has_removed_flag(void)
{
	pr_info("[%s] g_battery_has_removed_flag = %d\n", __func__,
		g_battery_has_removed_flag);
	return g_battery_has_removed_flag;
}

struct charger_device *hw_get_charger_device(
	struct charger_consumer *consumer, int idx)
{
	struct charger_device *pdev = NULL;
	struct charger_manager *info = NULL;

	info = consumer->cm;
	if (!info)
		return NULL;

	if (idx == MAIN_CHARGER)
		pdev = info->chg1_dev;
	else if (idx == SLAVE_CHARGER)
		pdev = info->chg2_dev;
	else
		pdev = NULL;

	return pdev;
}

struct charger_device *get_charger_dev(void)
{
	struct charger_device *pdev = NULL;

	if (!g_pconsumer)
		return NULL;

	pdev = hw_get_charger_device(g_pconsumer, 0);
	return pdev;
}

static struct charger_data *huawei_get_charger_data(
	struct charger_consumer *consumer, int idx)
{
	struct charger_data *pdata = NULL;
	struct charger_manager *info = NULL;

	info = consumer->cm;
	if (!info)
		return NULL;
	if (idx == MAIN_CHARGER)
		pdata = &info->chg1_data;
	else if (idx == SLAVE_CHARGER)
		pdata = &info->chg2_data;
	else
		pdata = NULL;

	return pdata;
}

static int huawei_get_charging_enable(void)
{
	return g_charge_enable;
}

static int huawei_set_charging_enable(struct charger_consumer *consumer,
	int chg_en)
{
	g_charge_enable = chg_en;
	charger_enable(consumer, chg_en);
	pr_info("%s g_charge_enable %d\n", __func__, g_charge_enable);
	return 0;
}

static int huawei_get_hz_enable(void)
{
	return g_hz_onoff;
}

static int huawei_set_hz_enable(struct charger_consumer *consumer, int idx,
	int hz_en)
{
	struct charger_device *pdev = NULL;

	pdev = hw_get_charger_device(consumer, idx);
	if (!pdev)
		return -EINVAL;

	g_hz_onoff = hz_en;
	charger_dev_enable_hz(pdev, hz_en);
	pr_info("%s set hz_enable to %d\n", __func__, g_hz_onoff);

	return 0;
}

static int huawei_get_wdt_disable(void)
{
	return g_wdt_disable;
}

static int huawei_set_wdt_disable(struct charger_consumer *consumer, int idx,
	int wdt_diable)
{
	struct charger_device *pdev = NULL;

	pdev = hw_get_charger_device(consumer, idx);
	if (!pdev)
		return -EINVAL;

	g_wdt_disable = wdt_diable;
	charger_dev_set_watchdog_timer(pdev, wdt_diable);
	pr_info("%s set wdt_disable to %d\n", __func__, g_wdt_disable);

	return 0;
}

static int hw_get_ibus(struct charger_consumer *consumer, int idx)
{
	struct charger_device *pdev = NULL;
	u32 ibus = 0;
	int ret;

	pdev = hw_get_charger_device(consumer, idx);
	if (!pdev)
		return -EINVAL;

	ret = charger_dev_get_ibus(pdev, &ibus);
	if (ret < 0)
		ibus = 0;
	if (ibus >= UA_PER_MA)
		ibus /= UA_PER_MA;
	return ibus;
}

static int set_input_curr_limit(struct huawei_battery_info *info,
	struct charger_consumer *consumer, int idx, int input_current)
{
	struct charger_manager *chg_info = NULL;
	struct charger_data *pdata = NULL;
	struct charger_device *pdev = NULL;
	enum charger_type chr_type;
	int temp;
	int vbat;

	temp = battery_get_bat_temperature();
	vbat = battery_get_bat_voltage();
	pr_info("%s:temp:%d vbat:%d input_current %d\n", __func__, temp,
		vbat, input_current);
	chr_type = mt_get_charger_type();
	pdev = hw_get_charger_device(consumer, idx);
	pdata = huawei_get_charger_data(consumer, idx);
	chg_info = consumer->cm;
	if (!pdev || !pdata || !chg_info)
		return -EINVAL;

#ifdef CONFIG_HLTHERM_RUNTEST
	if (temp < LOW_BATT_TEMP || vbat < LOW_BATT_VOL) {
		g_input_current = info->icl_current * MV_TO_UV;
		pr_info("DOUBLE_85,low temp or low vbat set ibus:%d\n",
			g_input_current);
	} else {
		g_input_current = min(input_current,
			info->icl_current * MV_TO_UV);
	}
#else
	g_input_current = min(input_current, info->icl_current * MV_TO_UV);
#endif
	pdata->input_current_limit = g_input_current;
	pr_info("%s set iin_current to %d\n", __func__, g_input_current);

	if ((chr_type != CHARGER_UNKNOWN) && (g_hz_onoff != IS_IN_HIZ_MODE))
		return chg_info->change_current_setting(chg_info);

	return 0;
}

static int get_thermal_input_curr_limit(struct charger_consumer *consumer,
	int idx)
{
	struct charger_data *pdata = NULL;

	pdata = huawei_get_charger_data(consumer, idx);
	if (pdata != NULL)
		return pdata->thermal_input_current_limit;

	return 0;
}

#ifdef CONFIG_HLTHERM_RUNTEST
static int set_thermal_input_curr_limit(struct huawei_battery_info *info,
	struct charger_consumer *consumer, int idx, int input_current)
{
	return 0;
}
#else
static int set_thermal_input_curr_limit(struct huawei_battery_info *info,
	struct charger_consumer *consumer, int idx, int input_current)
{
	struct charger_manager *chg_info = NULL;
	struct charger_data *pdata = NULL;
	enum charger_type chg_type;

	chg_type = mt_get_charger_type();
	pdata = huawei_get_charger_data(consumer, idx);
	chg_info = consumer->cm;
	if (!pdata || !chg_info)
		return -EINVAL;
	pr_info("%s: idx:%d set input current:%d\n", __func__,
		idx, input_current);
	/* if thermal limit set input current l,set input current to 2000 */
	if (input_current <= LIMIT_CURRENT_TOO_LOW)
		pdata->thermal_input_current_limit = info->icl_current;
	else
		pdata->thermal_input_current_limit =
			min(input_current, info->icl_current);

	pdata->thermal_input_current_limit *= UA_PER_MA;
	/* not in hiz mode */
	if ((chg_type != CHARGER_UNKNOWN) && (g_hz_onoff != IS_IN_HIZ_MODE))
		return chg_info->change_current_setting(chg_info);

	return 0;
}
#endif

static int get_sdp_input_curr_limit(struct charger_consumer *consumer, int idx)
{
	struct charger_manager *chg_info = NULL;
	struct charger_data *pdata = NULL;

	pdata = huawei_get_charger_data(consumer, idx);
	chg_info = consumer->cm;

	if (!pdata || !chg_info)
		return -EINVAL;

	return pdata->sdp_charging_current;
}

static int set_sdp_input_curr_limit(struct huawei_battery_info *info,
	struct charger_consumer *consumer, int idx, int input_current)
{
	struct charger_manager *chg_info = NULL;
	struct charger_data *pdata = NULL;

	if (!power_cmdline_is_factory_mode())
		return 0;

	pdata = huawei_get_charger_data(consumer, idx);
	chg_info = consumer->cm;
	if (!pdata || !chg_info)
		return -EINVAL;

	pdata->sdp_charging_current = input_current;

	return chg_info->change_current_setting(chg_info);
}

static int huawei_get_vbat_max_uv(void)
{
	union power_supply_propval val;

	if (!g_info)
		return DEFAULT_FASTCHG_MAX_VOLTAGE;

	if (g_info->use_third_coul) {
		if (power_supply_get_property_value("battery",
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &val))
			return DEFAULT_FASTCHG_MAX_VOLTAGE;
		return val.intval;
	}

	return g_info->voltage_max_design;
}

int huawei_get_vbat_max(void)
{
	return huawei_get_vbat_max_uv() / MV_TO_UV;
}

static enum power_supply_property hwbatt_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SDP_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_IN_THERMAL,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_HIZ_MODE,
	POWER_SUPPLY_PROP_WDT_DISABLE,
	POWER_SUPPLY_PROP_FACTORY_DIAG,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_RESET_LEARNED_CC,
};

static int hwbatt_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;
	struct huawei_battery_info *info = NULL;

	if (!psy || !val)
		return -EINVAL;
	info = power_supply_get_drvdata(psy);
	if (!info || !g_pconsumer)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		rc = power_supply_get_property_value_with_psy(info->bk_batt_psy,
			POWER_SUPPLY_PROP_ONLINE, val);
		if (rc < 0)
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = battery_get_bat_voltage();
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = mt_get_charger_type();
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE_NOW:
		val->intval = battery_get_vbus();
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = huawei_get_charging_enable();
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT_NOW:
		val->intval = hw_get_ibus(g_pconsumer, MAIN_CHARGER);
		break;
	case POWER_SUPPLY_PROP_SDP_CURRENT_LIMIT:
		val->intval = get_sdp_input_curr_limit(g_pconsumer, MAIN_CHARGER);
		break;
	case POWER_SUPPLY_PROP_IN_THERMAL:
		val->intval = (get_thermal_input_curr_limit(g_pconsumer,
			MAIN_CHARGER) / UA_PER_MA); /* mA to uA */
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = info->icl_current;
		break;
	case POWER_SUPPLY_PROP_HIZ_MODE:
		val->intval = huawei_get_hz_enable();
		break;
	case POWER_SUPPLY_PROP_WDT_DISABLE:
		val->intval = huawei_get_wdt_disable();
		break;
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
		val->intval = !g_factory_diag;
		break;
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		val->strval = huawei_get_battery_type();
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval =
			charger_manager_const_charge_current_max(g_pconsumer);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = charger_manager_get_voltage_max(g_pconsumer);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = huawei_get_vbat_max_uv();
		break;
	case POWER_SUPPLY_PROP_RESET_LEARNED_CC:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int hwbatt_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	int ret;
	struct huawei_battery_info *info = NULL;

	if (!psy || !val)
		return -EINVAL;

	info = power_supply_get_drvdata(psy);
	if (!info || !g_pconsumer)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = huawei_set_charging_enable(g_pconsumer, val->intval);
		return ret;
	case POWER_SUPPLY_PROP_IN_THERMAL:
		ret = set_thermal_input_curr_limit(info, g_pconsumer,
			MAIN_CHARGER, val->intval);
		return ret;
	case POWER_SUPPLY_PROP_SDP_CURRENT_LIMIT:
		return set_sdp_input_curr_limit(info, g_pconsumer,
			MAIN_CHARGER, val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		ret = set_input_curr_limit(info, g_pconsumer,
			MAIN_CHARGER, val->intval * UA_PER_MA);
		return ret;
	case POWER_SUPPLY_PROP_HIZ_MODE:
		ret = huawei_set_hz_enable(g_pconsumer, MAIN_CHARGER,
			val->intval);
		return ret;
	case POWER_SUPPLY_PROP_WDT_DISABLE:
		ret = huawei_set_wdt_disable(g_pconsumer, MAIN_CHARGER,
			val->intval);
		return ret;
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
		g_factory_diag = !val->intval;
		if (info->power_path) {
			ret = huawei_set_charging_enable(g_pconsumer,
				val->intval);
			return ret;
		}
		if (g_factory_diag)
			ret = set_input_curr_limit(info, g_pconsumer,
				MAIN_CHARGER, MIN_INPUT_CURRENT);
		else
			ret = set_input_curr_limit(info, g_pconsumer,
				MAIN_CHARGER, 0);
		return ret;
	case POWER_SUPPLY_PROP_RESET_LEARNED_CC:
		ret = charger_manager_reset_learned_cc(g_pconsumer,
			val->intval);
		return ret;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = charger_manager_set_voltage_max(g_pconsumer, val->intval);
		return ret;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = charger_manager_set_current_max(g_pconsumer, val->intval);
		return ret;
	default:
		return -EINVAL;
	}

	return 0;
}

static int hwbatt_property_is_writeable(struct power_supply *psy,
	enum power_supply_property prop)
{
	int rc;

	if (!psy) {
		pr_err("%s: Invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	/* pre process */
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SDP_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_IN_THERMAL:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_HIZ_MODE:
	case POWER_SUPPLY_PROP_FACTORY_DIAG:
	case POWER_SUPPLY_PROP_RESET_LEARNED_CC:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}
static enum power_supply_property hw_extra_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
#ifdef CONFIG_HUAWEI_CHARGER
	POWER_SUPPLY_PROP_BRAND,
#endif /* CONFIG_HUAWEI_CHARGER */
};

static int hw_extra_batt_get_prop(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc;
	struct huawei_battery_info *info = NULL;

	if (!psy || !val)
		return -EINVAL;

	info = power_supply_get_drvdata(psy);
	if (!info)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			POWER_SUPPLY_PROP_TEMP, val);
		if (rc < 0) {
			pr_err("%s: get temp failed\n", __func__);
			val->intval = DEFAULT_BATTERY_TEMP;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, val);
		if (rc < 0) {
			pr_err("%s: get capacity failed\n", __func__);
			val->intval = DEFAULT_BATT_CAPACITY;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			POWER_SUPPLY_PROP_STATUS, val);
		if (rc < 0)
			pr_err("%s: get battery status failed\n", __func__);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, val);
		if (rc < 0)
			pr_err("%s: get now current failed\n", __func__);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
		if (rc < 0)
			pr_err("%s: get now voltage failed\n", __func__);
		break;
	default:
		rc = power_supply_get_property_value_with_psy(info->batt_psy,
			psp, val);
	}
	return 0;
}

int huawei_battery_capacity(void)
{
	int rc;
	union power_supply_propval val = { 0, };
	int cap = 50; /* default capa is 50 */

	if (!g_info || !g_info->batt_psy)
		return cap;
	rc = power_supply_get_property_value_with_psy(g_info->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc < 0)
		return cap;
	cap = val.intval;
	return cap;
}

int huawei_get_ac_online(void)
{
	int rc;
	union power_supply_propval val = { 0, };
	int online = 0;

	if (!g_info || !g_info->ac_psy)
		return online;
	rc = power_supply_get_property_value_with_psy(g_info->ac_psy,
		POWER_SUPPLY_PROP_ONLINE, &val);
	if (rc < 0)
		return online;
	online = val.intval;
	return online;
}

static int hwbatt_power_supply_register(struct platform_device *pdev,
	struct huawei_battery_info *info)
{
	int ret = 0;

	info->hwbatt_desc.name = "hwbatt";
	info->hwbatt_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->hwbatt_desc.properties = hwbatt_properties;
	info->hwbatt_desc.num_properties = ARRAY_SIZE(hwbatt_properties);
	info->hwbatt_desc.set_property = hwbatt_set_property;
	info->hwbatt_desc.get_property = hwbatt_get_property;
	info->hwbatt_desc.property_is_writeable = hwbatt_property_is_writeable;
	info->hwbatt_cfg.drv_data = info;
	info->hwbatt_cfg.of_node = info->dev->of_node;
	info->hwbatt_psy = power_supply_register(&pdev->dev, &info->hwbatt_desc,
		&info->hwbatt_cfg);
	if (IS_ERR(info->hwbatt_psy)) {
		dev_err(&pdev->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(info->hwbatt_psy));
		ret = -EPROBE_DEFER;
	}
	return ret;
}

static int hwbatt_init_psy(struct huawei_battery_info *info)
{
	int ret = 0;

	info->batt_psy = power_supply_get_by_name("battery");
	if (!info->batt_psy) {
		dev_err(info->dev, "cannot get batery psy\n");
		ret = -EPROBE_DEFER;
	}

	info->bk_batt_psy = power_supply_get_by_name("charger");
	if (!info->bk_batt_psy) {
		dev_err(info->dev, "cannot get bk_batery psy\n");
		ret = -EPROBE_DEFER;
	}

	info->usb_psy = power_supply_get_by_name("usb");
	if (!info->usb_psy) {
		dev_err(info->dev, "cannot get usb psy\n");
		ret = -EPROBE_DEFER;
	}

	info->ac_psy = power_supply_get_by_name("ac");
	if (!info->ac_psy) {
		dev_err(info->dev, "cannot get ac psy\n");
		ret = -EPROBE_DEFER;
	}

	return ret;
}

static int batt_extra_power_supply_register(struct platform_device *pdev,
	struct huawei_battery_info *info)
{
	int ret = 0;

	if (info->use_third_coul)
		return ret;

	info->batt_extra_psy_desc.name = "Battery";
	info->batt_extra_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->batt_extra_psy_desc.properties = hw_extra_batt_props;
	info->batt_extra_psy_desc.num_properties =
		ARRAY_SIZE(hw_extra_batt_props);
	info->batt_extra_psy_desc.set_property = NULL;
	info->batt_extra_psy_desc.get_property = hw_extra_batt_get_prop;
	info->batt_extra_psy_desc.property_is_writeable = NULL;
	info->batt_cfg.drv_data = info;
	info->batt_cfg.of_node = info->dev->of_node;
	info->extra_batt_psy = power_supply_register(&pdev->dev,
		&info->batt_extra_psy_desc, &info->batt_cfg);
	if (IS_ERR(info->extra_batt_psy)) {
		dev_err(&pdev->dev, "failed to register power supply: %ld\n",
			PTR_ERR(info->extra_batt_psy));
		ret = -EPROBE_DEFER;
	}
	return ret;
}

static void huawei_battery_parse_dt(struct huawei_battery_info *info,
	struct device_node *node)
{
	int ret;
	int icl_current = 0;
	int voltage_max = 0;

	info->power_path = of_property_read_bool(node, "support_powerpath");
	ret = of_property_read_u32(node, "huawei,icl_current", &icl_current);
	if (ret < 0)
		info->icl_current = DEFAULT_USB_ICL_CURRENT;
	else
		info->icl_current = icl_current;

	ret = of_property_read_u32(node, "voltage_max_design", &voltage_max);
	if (ret < 0)
		info->voltage_max_design = DEFAULT_FASTCHG_MAX_VOLTAGE;
	else
		info->voltage_max_design = voltage_max;

	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), node,
		"use_third_coul", &info->use_third_coul, 0);

	pr_info("power_path: %d icl=%d voltage_max_design:%d\n",
		info->power_path, info->icl_current, info->voltage_max_design);
}

static int huawei_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct huawei_battery_info *info = NULL;
	struct device_node *node = NULL;

	if (!pdev) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}
	node = pdev->dev.of_node;
	if (!node) {
		pr_err("%s: no device node\n", __func__);
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	g_pconsumer = charger_manager_get_by_name(&pdev->dev, "charger");
	huawei_battery_parse_dt(info, node);

	ret = hwbatt_power_supply_register(pdev, info);
	if (ret)
		goto psy_reg_fail;

	ret = hwbatt_init_psy(info);
	if (ret)
		goto psy_get_fail;

	ret = batt_extra_power_supply_register(pdev, info);
	if (ret)
		goto psy_get_fail;
	g_info = info;

	pr_info("%s ok\n", __func__);
	return 0;

psy_get_fail:
	power_supply_unregister(info->hwbatt_psy);
psy_reg_fail:
	devm_kfree(&pdev->dev, info);
	info = NULL;
	return ret;
}

static int huawei_battery_remove(struct platform_device *pdev)
{
	struct huawei_battery_info *info = NULL;

	if (!pdev) {
		pr_err("%s: Invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	info = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(info->hwbatt_psy);
	devm_kfree(&pdev->dev, info);

	return 0;
}

static const struct of_device_id battery_match_table[] = {
	{
		.compatible = "huawei,battery",
		.data = NULL,
	},
	{
	},
};

static struct platform_driver huawei_battery_driver = {
	.probe = huawei_battery_probe,
	.remove = huawei_battery_remove,
	.driver = {
		.name = "huawei,battery",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(battery_match_table),
	},
};

static int __init huawei_battery_init(void)
{
	return platform_driver_register(&huawei_battery_driver);
}
static void __exit huawei_battery_exit(void)
{
	platform_driver_unregister(&huawei_battery_driver);
}
module_init(huawei_battery_init);
module_exit(huawei_battery_exit);

MODULE_DESCRIPTION("huawei battery driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:huawei-battery");
