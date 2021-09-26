/*
 * huawei_charger.c
 *
 * huawei charger driver
 *
 * Copyright (C) 2019-2019 Huawei Technologies Co., Ltd.
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

#include <linux/power/huawei_charger.h>
#include <linux/power/huawei_mtk_charger.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/string.h>
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
#include <linux/spmi.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <chipset_common/hwpower/common_module/power_interface.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_class.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot.h>
#include <pmic.h>
#include <mtk_charger_intf.h>
#include <chipset_common/hwpower/common_module/power_sysfs.h>
#include <chipset_common/hwpower/common_module/power_log.h>
#include <chipset_common/hwpower/common_module/power_algorithm.h>

#ifdef CONFIG_DIRECT_CHARGER
#if !defined(CONFIG_MACH_MT6768) && !defined(CONFIG_MACH_MT6765)
#include "mtk_intf.h"
#endif
#endif

#include <huawei_platform/log/hw_log.h>
#include <chipset_common/hwpower/common_module/power_cmdline.h>
#include <chipset_common/hwpower/common_module/power_supply_interface.h>

#define HWLOG_TAG huawei_charger
HWLOG_REGIST();

#define DEFAULT_ITERM            200
#define DEFAULT_VTERM            4400000
#define DEFAULT_FCP_TEST_DELAY   6000
#define DEFAULT_IIN_CURRENT      1000
#define MAX_CURRENT              3000
#define MIN_CURRENT              100
#define DEC_BASE                 10
#define INPUT_EVENT_ID_MIN       0
#define INPUT_EVENT_ID_MAX       3000
#define CHARGE_INFO_BUF_SIZE     30
#define DEFAULT_IIN_THL          2300
#define DEFAULT_IIN_RT           1
#define DEFAULT_HIZ_MODE         0
#define DEFAULT_CHRG_CONFIG      1
#define DEFAULT_FACTORY_DIAG     1
#define UPDATE_VOLT              1
#define EN_HIZ                   1
#define DIS_HIZ                  0
#define EN_CHG                   1
#define DIS_CHG                  0
#define EN_WDT                   1
#define DIS_WDT                  0
#define INVALID_LIMIT_CUR        0
#define NAME                     "charger"
#define NO_CHG_TEMP_LOW          0
#define NO_CHG_TEMP_HIGH         50
#define BATT_EXIST_TEMP_LOW     (-40)
#define DEFAULT_NORMAL_TEMP      25

enum param_type {
	ONLINE = 0,
	TYPE,
	CHG_EN,
	CHG_STS,
	BAT_STS,
	BAT_ON,
	BAT_TEMP,
	BAT_VOL,
	BAT_CUR,
	PCT,
	BAT_CYCLE,
	BAT_FCC,
	USB_IBUS,
	USB_VOL,
	PARAM_MAX
};

/* mtk platform type to common charge type */
struct convert_data mtk_charger_type[] = {
	{ CHARGER_UNKNOWN, CHARGER_REMOVED },
	{ STANDARD_HOST, CHARGER_TYPE_USB },
	{ CHARGING_HOST, CHARGER_TYPE_BC_USB },
	{ NONSTANDARD_CHARGER, CHARGER_TYPE_NON_STANDARD },
	{ STANDARD_CHARGER, CHARGER_TYPE_STANDARD },
	{ WIRELESS_CHARGER, CHARGER_TYPE_WIRELESS },
	{ FCP_CHARGER, CHARGER_TYPE_FCP },
	{ APPLE_2_1A_CHARGER, CHARGER_TYPE_APPLE_2_1A },
	{ APPLE_1_0A_CHARGER, CHARGER_TYPE_APPLE_1_0A },
	{ APPLE_0_5A_CHARGER, CHARGER_TYPE_APPLE_0_5A },
};

struct device *g_charge_dev;
static struct charge_device_info *g_charger_device_para;
struct kobject *g_sysfs_poll;
int g_basp_learned_fcc = -1;
static int first_check;

int get_eoc_max_delay_count(void)
{
	return g_charger_device_para->en_eoc_max_delay;
}

int get_first_insert(void)
{
	return first_check;
}

void set_first_insert(int flag)
{
	pr_info("set insert flag %d\n", flag);
	first_check = flag;
}

void charge_send_uevent(int input_events)
{
}

void charge_request_charge_monitor(void)
{
}

void charge_send_icon_uevent(int icon_type)
{
	hwlog_info("icon_type=%d\n", icon_type);
	power_ui_event_notify(POWER_UI_NE_ICON_TYPE, &icon_type);
	power_supply_sync_changed("battery");
	power_supply_sync_changed("Battery");
}

#ifdef CONFIG_DIRECT_CHARGER
void direct_charger_disconnect_update_charger_type(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		pr_err("[%s]di is NULL\n", __func__);
		return;
	}
	di->charger_type = CHARGER_REMOVED;
	di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
}
void ignore_pluggin_and_pluggout_interrupt(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		pr_err("[%s]di is NULL\n", __func__);
		return;
	}
	di->ignore_pluggin_and_plugout_flag = 1;
}
void restore_pluggin_pluggout_interrupt(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		pr_err("[%s]di is NULL\n", __func__);
		return;
	}
	di->ignore_pluggin_and_plugout_flag = 0;
}
int get_direct_charge_flag(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di)
		return 0;
	return di->ignore_pluggin_and_plugout_flag;
}
extern struct charger_manager *huawei_get_pinfo(void);
extern int fcp_support_show(void);
void wired_connect_send_icon_uevent(int icon_type)
{
	struct charger_manager *pinfo = huawei_get_pinfo();

	if (!pinfo) {
		chr_err("charger is not rdy, skip1\n");
		return;
	}

	if (pinfo->init_done != true) {
		chr_err("charger is not rdy, skip2\n");
		return;
	}

	if (!pinfo->charger_thread_polling) {
		pr_info("charge already stop\n");
		return;
	}

	if (mt_get_charger_type() == CHARGER_UNKNOWN) {
		pr_info("charge already plugged out\n");
		return;
	}

	charge_send_icon_uevent(icon_type);
	charger_manager_notifier(pinfo, CHARGER_NOTIFY_START_CHARGING);
}

void wired_disconnect_send_icon_uevent(void)
{
	struct charger_manager *pinfo = huawei_get_pinfo();

	if (!pinfo)
		return;

	charge_send_icon_uevent(ICON_TYPE_INVALID);
	charger_manager_notifier(pinfo, CHARGER_NOTIFY_STOP_CHARGING);
}

void wireless_connect_send_icon_uevent(int icon_type)
{
	struct charger_manager *pinfo = huawei_get_pinfo();

	if (!pinfo) {
		chr_err("charger is not rdy, skip\n");
		return;
	}

	if (mt_get_charger_type() != WIRELESS_CHARGER) {
		pr_info("charger already plugged out\n");
		return;
	}

	charge_send_icon_uevent(icon_type);
	charger_manager_notifier(pinfo, CHARGER_NOTIFY_START_CHARGING);
}

#if defined(CONFIG_MACH_MT6768) || defined(CONFIG_MACH_MT6765)
int charger_set_constant_voltage(u32 uv)
{
	struct charger_manager *pinfo = huawei_get_pinfo();

	if (!pinfo) {
		chr_err("charger is null\n");
		return -EINVAL;
	}
	return charger_dev_set_constant_voltage(pinfo->chg1_dev, uv);
}
#endif
#endif

enum charge_done_type get_charge_done_type(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di)
		return CHARGE_DONE_NON;
	return di->sysfs_data.charge_done_status;
}

static int get_property_from_psy(struct power_supply *psy,
	enum power_supply_property prop)
{
	int rc;
	int val;
	union power_supply_propval ret = { 0, };

	if (!psy) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	rc = psy->desc->get_property(psy, prop, &ret);
	if (rc) {
		pr_err("psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}
	val = ret.intval;
	return val;
}

static ssize_t get_poll_charge_start_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (!dev || !attr || !buf) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	if (chip)
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->input_event);
	else
		return 0;
}

static ssize_t set_poll_charge_event(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct charge_device_info *chip = g_charger_device_para;
	long val = 0;

	if (!dev || !attr || !buf) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	if (chip) {
		if ((kstrtol(buf, DEC_BASE, &val) < 0) ||
			(val < INPUT_EVENT_ID_MIN) ||
			(val > INPUT_EVENT_ID_MAX)) {
			return -EINVAL;
		}
		chip->input_event = val;
		power_event_notify_sysfs(g_sysfs_poll, NULL, "poll_charge_start_event");
	}
	return count;
}
static DEVICE_ATTR(poll_charge_start_event, 0644,
	get_poll_charge_start_event, set_poll_charge_event);

static ssize_t get_poll_charge_done_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!dev || !attr || !buf) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", g_basp_learned_fcc);
}
static DEVICE_ATTR(poll_charge_done_event, 0644,
	get_poll_charge_done_event, NULL);

static ssize_t get_poll_fcc_learn_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!dev || !attr || !buf) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", g_basp_learned_fcc);
}
static DEVICE_ATTR(poll_fcc_learn_event, 0644,
	get_poll_fcc_learn_event, NULL);

static int charge_event_poll_register(struct device *dev)
{
	int ret;

	if (!dev) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	ret = sysfs_create_file(&dev->kobj,
		&dev_attr_poll_charge_start_event.attr);
	if (ret) {
		pr_err("fail to create poll node for %s\n", dev->kobj.name);
		return ret;
	}
	ret = sysfs_create_file(&dev->kobj,
		&dev_attr_poll_charge_done_event.attr);
	if (ret) {
		pr_err("fail to create poll node for %s\n", dev->kobj.name);
		return ret;
	}
	ret = sysfs_create_file(&dev->kobj,
		&dev_attr_poll_fcc_learn_event.attr);
	if (ret) {
		pr_err("fail to create poll node for %s\n", dev->kobj.name);
		return ret;
	}

	g_sysfs_poll = &dev->kobj;
	return ret;
}

void cap_learning_event_done_notify(void)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (!chip) {
		pr_info("charger device is not init, do nothing!\n");
		return;
	}

	power_event_notify_sysfs(g_sysfs_poll, NULL, "basp_learned_fcc");
}

void cap_charge_done_event_notify(void)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (!chip) {
		pr_info("charger device is not init, do nothing\n");
		return;
	}

	power_event_notify_sysfs(g_sysfs_poll, NULL, "poll_basp_done_event");
}

void charge_event_notify(unsigned int event)
{
	struct charge_device_info *chip = g_charger_device_para;

	if (!chip) {
		pr_info("smb device is not init, do nothing!\n");
		return;
	}
	/* avoid notify charge stop event,
	 * continuously without charger inserted
	 */
	if ((chip->input_event != event) || (event == SMB_START_CHARGING)) {
		chip->input_event = event;
		power_event_notify_sysfs(g_sysfs_poll, NULL, "poll_charge_start_event");
	}
}

static int huawei_charger_set_runningtest(struct charge_device_info *di,
	int val)
{
	int iin_rt;

	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, !!val);
	iin_rt = get_property_from_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED);
	di->sysfs_data.iin_rt = iin_rt;

	return 0;
}

static int huawei_charger_enable_charge(struct charge_device_info *di, int val)
{
	if (!di || !(di->batt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, !!val);
	di->sysfs_data.charge_enable = get_property_from_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED);
	return 0;
}

#ifndef CONFIG_HLTHERM_RUNTEST
static bool is_batt_temp_normal(int temp)
{
	if (((temp > BATT_EXIST_TEMP_LOW) && (temp <= NO_CHG_TEMP_LOW)) ||
		(temp >= NO_CHG_TEMP_HIGH))
		return false;
	return true;
}
#endif

static void huawei_set_enable_charge(struct charge_device_info *di, int val)
{
	int batt_temp;

	if (!di) {
		hwlog_err("di is null\n");
		return;
	}

	batt_temp = battery_get_bat_temperature();
#ifndef CONFIG_HLTHERM_RUNTEST
	if ((val == EN_CHG) && !is_batt_temp_normal(batt_temp)) {
		hwlog_err("battery temp is %d, abandon enable_charge\n",
			batt_temp);
		return;
	}
#endif

	huawei_charger_enable_charge(di, val);
	return;
}

static int huawei_charger_set_in_thermal(struct charge_device_info *di,
	int val)
{
	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_IN_THERMAL, val);
	di->sysfs_data.iin_thl = get_property_from_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_IN_THERMAL);

	return 0;
}

static int huawei_charger_set_rt_current(struct charge_device_info *di, int val)
{
	int iin_rt_curr;

	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}
	/* 0&1:open limit 100:limit to 100 */
	if ((val == 0) || (val == 1)) {
		iin_rt_curr = get_property_from_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_MAX);
		pr_info("%s rt_current =%d\n", __func__, iin_rt_curr);
	} else if ((val <= MIN_CURRENT) && (val > 1)) {
		iin_rt_curr = MIN_CURRENT;
	} else {
		iin_rt_curr = val;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, iin_rt_curr);

	di->sysfs_data.iin_rt_curr = iin_rt_curr;
	return 0;
}

static int huawei_charger_set_hz_mode(struct charge_device_info *di, int val)
{
	int hiz_mode;

	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}
	hiz_mode = !!val;
	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_HIZ_MODE, hiz_mode);
	di->sysfs_data.hiz_mode = hiz_mode;
	return 0;
}

static int huawei_charger_set_wdt_disable(struct charge_device_info *di,
	int val)
{
	int wdt_disable;

	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}
	wdt_disable = val;
	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_WDT_DISABLE, wdt_disable);
	di->sysfs_data.wdt_disable = wdt_disable;
	return 0;
}

#define charge_sysfs_field(_name, n, m, store) { \
	.attr = __ATTR(_name, m, charge_sysfs_show, store), \
	.name = CHARGE_SYSFS_##n, \
}

#define charge_sysfs_field_rw(_name, n) \
	charge_sysfs_field(_name, n, 0644, charge_sysfs_store)

#define charge_sysfs_field_ro(_name, n) \
	charge_sysfs_field(_name, n, 0444, NULL)

static ssize_t charge_sysfs_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t charge_sysfs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);

struct charge_sysfs_field_info {
	char name;
	struct device_attribute attr;
};

static struct charge_sysfs_field_info g_chg_sysfs_field_tbl[] = {
	charge_sysfs_field_rw(iin_thermal, IIN_THERMAL),
	charge_sysfs_field_rw(iin_runningtest, IIN_RUNNINGTEST),
	charge_sysfs_field_rw(iin_rt_current, IIN_RT_CURRENT),
	charge_sysfs_field_rw(enable_hiz, HIZ),
	charge_sysfs_field_rw(shutdown_watchdog, WATCHDOG_DISABLE),
	charge_sysfs_field_rw(enable_charger, ENABLE_CHARGER),
	charge_sysfs_field_rw(factory_diag, FACTORY_DIAG_CHARGER),
	charge_sysfs_field_rw(update_volt_now, UPDATE_VOLT_NOW),
	charge_sysfs_field_ro(ibus, IBUS),
	charge_sysfs_field_ro(vbus, VBUS),
	charge_sysfs_field_ro(chargerType, CHARGE_TYPE),
	charge_sysfs_field_ro(charge_term_volt_design,
		CHARGE_TERM_VOLT_DESIGN),
	charge_sysfs_field_ro(charge_term_curr_design,
		CHARGE_TERM_CURR_DESIGN),
	charge_sysfs_field_ro(charge_term_volt_setting,
		CHARGE_TERM_VOLT_SETTING),
	charge_sysfs_field_ro(charge_term_curr_setting,
		CHARGE_TERM_CURR_SETTING),
	charge_sysfs_field_rw(regulation_voltage, REGULATION_VOLTAGE),
	charge_sysfs_field_ro(fcp_support, FCP_SUPPORT),
	charge_sysfs_field_rw(adaptor_voltage, ADAPTOR_VOLTAGE),
	charge_sysfs_field_rw(thermal_reason, THERMAL_REASON),
	charge_sysfs_field_rw(vterm_dec, VTERM_DEC),
	charge_sysfs_field_rw(ichg_ratio, ICHG_RATIO),
	charge_sysfs_field_ro(charger_online, CHARGER_ONLINE),
};

static struct attribute *g_chg_sysfs_attrs[ARRAY_SIZE(g_chg_sysfs_field_tbl) +
	1];

static const struct attribute_group charge_sysfs_attr_group = {
	.attrs = g_chg_sysfs_attrs,
};

/* initialize g_chg_sysfs_attrs[] for charge attribute */
static void charge_sysfs_init_attrs(void)
{
	int i;
	const int limit = ARRAY_SIZE(g_chg_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		g_chg_sysfs_attrs[i] = &g_chg_sysfs_field_tbl[i].attr.attr;

	g_chg_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

static struct charge_sysfs_field_info *charge_sysfs_field_lookup(
	const char *name)
{
	int i;
	const int limit = ARRAY_SIZE(g_chg_sysfs_field_tbl);

	if (!name) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return NULL;
	}

	for (i = 0; i < limit; i++) {
		if (!strcmp(name, g_chg_sysfs_field_tbl[i].attr.attr.name))
			break;
	}

	if (i >= limit)
		return NULL;

	return &g_chg_sysfs_field_tbl[i];
}

static void conver_usbtype(int val, char *p_str)
{
	if (!p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case CHARGER_UNKNOWN:
		strncpy(p_str, "UNKNOWN", sizeof("UNKNOWN"));
		break;
	case STANDARD_HOST:
		strncpy(p_str, "USB", sizeof("USB"));
		break;
	case CHARGING_HOST:
		strncpy(p_str, "CDP", sizeof("CDP"));
		break;
	case NONSTANDARD_CHARGER:
		strncpy(p_str, "NONSTD", sizeof("NONSTD"));
		break;
	case STANDARD_CHARGER:
		strncpy(p_str, "DC", sizeof("DC"));
		break;
	case FCP_CHARGER:
		strncpy(p_str, "FCP", sizeof("FCP"));
		break;
	case APPLE_2_1A_CHARGER:
		strncpy(p_str, "APPLE_2_1A", sizeof("APPLE_2_1A"));
		break;
	case APPLE_1_0A_CHARGER:
		strncpy(p_str, "APPLE_1_0A", sizeof("APPLE_1_0A"));
		break;
	case APPLE_0_5A_CHARGER:
		strncpy(p_str, "APPLE_0_5A", sizeof("APPLE_0_5A"));
		break;
	case WIRELESS_CHARGER:
		strncpy(p_str, "WIRELESS", sizeof("WIRELESS"));
		break;
	default:
		strncpy(p_str, "UNSTANDARD", sizeof("UNSTANDARD"));
		break;
	}
}

static void conver_charging_status(int val, char *p_str)
{
	if (!p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case POWER_SUPPLY_STATUS_UNKNOWN:
		strncpy(p_str, "UNKNOWN", sizeof("UNKNOWN"));
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		strncpy(p_str, "CHARGING", sizeof("CHARGING"));
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		strncpy(p_str, "DISCHARGING", sizeof("DISCHARGING"));
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		strncpy(p_str, "NOTCHARGING", sizeof("NOTCHARGING"));
		break;
	case POWER_SUPPLY_STATUS_FULL:
		strncpy(p_str, "FULL", sizeof("FULL"));
		break;
	default:
		break;
	}
}

static void conver_charger_health(int val, char *p_str)
{
	if (!p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}

	switch (val) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		strncpy(p_str, "OVERHEAT", sizeof("OVERHEAT"));
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		strncpy(p_str, "COLD", sizeof("COLD"));
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		strncpy(p_str, "WARM", sizeof("WARM"));
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		strncpy(p_str, "COOL", sizeof("COOL"));
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
		strncpy(p_str, "GOOD", sizeof("GOOD"));
		break;
	default:
		break;
	}
}

static unsigned int converse_usb_type(unsigned int type)
{
	unsigned int new_type = CHARGER_REMOVED;

	power_convert_value(mtk_charger_type, ARRAY_SIZE(mtk_charger_type),
		type, &new_type);

	return new_type;
}

unsigned int huawei_get_charger_type(void)
{
	struct charge_device_info *di = g_charger_device_para;
	unsigned int type = mt_get_charger_type();

	if (!di) {
		pr_err("di is NULL\n");
		return CHARGER_REMOVED;
	}

	di->charger_type = converse_usb_type(type);

	return di->charger_type;
}

static void get_charger_shutdown_flag(bool flag, char *p_str)
{
	if (!p_str) {
		pr_err("the p_str is NULL\n");
		return;
	}
	if (flag)
		strncpy(p_str, "OFF", sizeof("OFF"));
	else
		strncpy(p_str, "ON", sizeof("ON"));
}

static int huawei_charger_value_dump(char *buffer, int size, void *dev_data)
{
	struct charge_device_info *di = dev_data;
	char c_type[CHARGE_INFO_BUF_SIZE] = {0};
	char c_status[CHARGE_INFO_BUF_SIZE] = {0};
	char c_health[CHARGE_INFO_BUF_SIZE] = {0};
	char c_on[CHARGE_INFO_BUF_SIZE] = {0};
	int prm[PARAM_MAX];
	int fsoc;
	int msoc;

	if (!buffer || !di)
		return -1;

	prm[ONLINE] = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_ONLINE);
	prm[TYPE] = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE);
	conver_usbtype(prm[TYPE], c_type);
	prm[USB_VOL] = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGE_VOLTAGE_NOW);
	prm[CHG_EN] = di->sysfs_data.charge_enable;
	prm[CHG_STS] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_STATUS);
	conver_charging_status(prm[CHG_STS], c_status);
	prm[BAT_STS] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_HEALTH);
	conver_charger_health(prm[BAT_STS], c_health);
	prm[BAT_ON] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_PRESENT);
	prm[BAT_TEMP] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_TEMP);
	prm[BAT_VOL] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	prm[BAT_CUR] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW);
	prm[PCT] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY);
	prm[BAT_CYCLE] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_CYCLE_COUNT);
	prm[BAT_FCC] = power_supply_return_int_property_value_with_psy(di->batt_psy,
		POWER_SUPPLY_PROP_CHARGE_FULL);
	prm[USB_IBUS] = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGE_CURRENT_NOW);
	get_charger_shutdown_flag(power_cmdline_is_powerdown_charging_mode(), c_on);

	msoc = huawei_capacity_get_daemon_uisoc();
	fsoc = huawei_capacity_get_filter_soc();
	(void)snprintf(buffer, size,
		" %-8d %-11s %-11d %-11d %-7d "
		"%-14s %-9s %-8d %-7d %-9d "
		"%-7d %-7d %-6d %-6d %-7d %-7d %-9d %4s  ",
		prm[ONLINE], c_type, prm[USB_VOL], di->sysfs_data.iin_thl, prm[CHG_EN],
		c_status, c_health, prm[BAT_ON], prm[BAT_TEMP], prm[BAT_VOL],
		prm[BAT_CUR], prm[USB_IBUS], msoc, fsoc, prm[PCT], prm[BAT_CYCLE],
		prm[BAT_FCC], c_on);
	return 0;
}

static int huawei_charger_get_log_head(char *buf, int size, void *dev_data)
{
	struct charge_device_info *di = dev_data;

	if (!buf || !di)
		return -1;

	(void)snprintf(buf, size, " online   in_type     usb_vol"
		"     iin_thl     ch_en   status         health    "
		"bat_on   temp    vol       cur     ibus    "
		"msoc   fsoc   uisoc   cycle   fcc       Mode  ");
	return 0;
}

int charge_get_charger_online(void)
{
	struct power_supply *chg_psy = NULL;
	int online;

	chg_psy = power_supply_get_by_name("charger");
	if (!chg_psy) {
		pr_err("charger supply not found\n");
		return 0;
	}
	online = power_supply_return_int_property_value_with_psy(chg_psy,
		POWER_SUPPLY_PROP_ONLINE);
	return online;
}

#ifdef CONFIG_DIRECT_CHARGER
void charge_set_adapter_voltage(int val, enum reset_adapter_source_type type,
	unsigned int delay_time)
{
	struct charge_device_info *di = g_charger_device_para;
	struct charger_manager *pinfo = huawei_get_pinfo();
	int batt_temp;

	if (!pinfo) {
		chr_err("charger is not rdy, skip1\n");
		return;
	}

	if (!di) {
		hwlog_err("[%s] di is NULL\n", __func__);
		return;
	}
	if ((type < 0) || (type >= RESET_ADAPTER_TOTAL)) {
		hwlog_err("invalid type = %d\n", type);
		return;
	}

	if (val == ADAPTER_5V) {
		di->reset_adapter = di->reset_adapter | (1 << type);
		hwlog_info("Reset adaptor to 5V, reset_adapter = 0x%x\n", di->reset_adapter);
	} else {
		di->reset_adapter = di->reset_adapter & (~(1 << type));
		hwlog_info("Restore adaptor to 9V, reset_adapter = 0x%x\n", di->reset_adapter);
	}

	batt_temp = battery_get_bat_temperature();
	if (!pinfo->can_charging && !pinfo->sw_jeita.charging) {
		hwlog_err("charge has been stopped, no need reset. batt_temp = %d\n", batt_temp);
		return;
	}

	/* fcp adapter reset */
	if ((pinfo->chr_type == FCP_CHARGER) || (pinfo->chr_type == STANDARD_CHARGER))
		_wake_up_charger(pinfo);
}
#endif

static ssize_t charge_sysfs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;
	struct charge_sysfs_field_info *info = NULL;
	struct charge_device_info *di = NULL;
	int type;
	int conver_type;
	int vol;
	int ibus;
#ifdef CONFIG_DIRECT_CHARGER
	unsigned int val;
#endif

	if (!dev || !attr || !buf) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	di = dev_get_drvdata(dev);
	info = charge_sysfs_field_lookup(attr->attr.name);
	if (!di || !info)
		return -EINVAL;

	switch (info->name) {
	case CHARGE_SYSFS_IIN_THERMAL:
		ret = snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.iin_thl);
		break;
	case CHARGE_SYSFS_IIN_RUNNINGTEST:
		ret = snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.iin_rt);
		break;
	case CHARGE_SYSFS_IIN_RT_CURRENT:
		ret = snprintf(buf, MAX_SIZE, "%u\n",
			di->sysfs_data.iin_rt_curr);
		break;
	case CHARGE_SYSFS_ENABLE_CHARGER:
		ret = snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.charge_enable);
		break;
	case CHARGE_SYSFS_IBUS:
		ibus = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_CHARGE_CURRENT_NOW);
		ret = snprintf(buf, MAX_SIZE, "%d\n", ibus);
		break;
	case CHARGE_SYSFS_VBUS:
		vol = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_CHARGE_VOLTAGE_NOW);
		ret = snprintf(buf, MAX_SIZE, "%d\n", vol);
		break;
	case CHARGE_SYSFS_CHARGE_TYPE:
		type = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE);
		conver_type = converse_usb_type(type);
		ret = snprintf(buf, MAX_SIZE, "%d\n", conver_type);
		break;
	case CHARGE_SYSFS_WATCHDOG_DISABLE:
		ret = snprintf(buf, MAX_SIZE, "%u\n",
			di->sysfs_data.wdt_disable);
		break;
	case CHARGE_SYSFS_HIZ:
		ret = snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.hiz_mode);
		break;
	case CHARGE_SYSFS_CHARGE_TERM_VOLT_DESIGN:
		ret = snprintf(buf, PAGE_SIZE, "%d\n", di->sysfs_data.vterm);
		break;
	case CHARGE_SYSFS_CHARGE_TERM_CURR_DESIGN:
		ret = snprintf(buf, PAGE_SIZE, "%d\n", di->sysfs_data.iterm);
		break;
	case CHARGE_SYSFS_CHARGE_TERM_VOLT_SETTING:
		vol = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX);
		ret = snprintf(buf, PAGE_SIZE, "%d\n",
			((vol < di->sysfs_data.vterm) ? vol :
			di->sysfs_data.vterm));
		break;
	case CHARGE_SYSFS_CHARGE_TERM_CURR_SETTING:
		ret = snprintf(buf, PAGE_SIZE, "%d\n", di->sysfs_data.iterm);
		break;
#ifdef CONFIG_DIRECT_CHARGER
	case CHARGE_SYSFS_VTERM_DEC:
		val = huawei_get_basp_vterm_dec();
		ret = snprintf(buf, PAGE_SIZE, "%d\n", val);
		break;
	case CHARGE_SYSFS_ICHG_RATIO:
		val = huawei_get_basp_ichg_ratio();
		ret = snprintf(buf, PAGE_SIZE, "%d\n", val);
		break;
#endif
	case CHARGE_SYSFS_REGULATION_VOLTAGE:
		return snprintf(buf, PAGE_SIZE, "%u\n", di->sysfs_data.vterm_rt);
#ifdef CONFIG_FCP_CHARGER
	case CHARGE_SYSFS_FCP_SUPPORT:
		di->sysfs_data.fcp_support = fcp_support_show();
		return snprintf(buf, PAGE_SIZE, "%d\n", di->sysfs_data.fcp_support);
#endif
	case CHARGE_SYSFS_ADAPTOR_VOLTAGE:
		if (di->reset_adapter)
			return snprintf(buf, PAGE_SIZE, "%d\n", ADAPTER_5V);
		else
			return snprintf(buf, PAGE_SIZE, "%d\n", ADAPTER_9V);
	case CHARGE_SYSFS_CHARGER_ONLINE:
		return snprintf(buf, PAGE_SIZE, "%d\n", charge_get_charger_online());
	default:
		break;
	}

	return ret;
}

static ssize_t charge_sysfs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val = 0;
	struct charge_sysfs_field_info *info = NULL;
	struct charge_device_info *di = NULL;
#ifdef CONFIG_DIRECT_CHARGER
	int ret;
#endif

	if (!dev || !attr || !buf)
		return -EINVAL;

	di = dev_get_drvdata(dev);
	info = charge_sysfs_field_lookup(attr->attr.name);
	if (!info || !di)
		return -EINVAL;

	if (kstrtol(buf, DEC_BASE, &val) < 0)
		return -EINVAL;

	switch (info->name) {
	case CHARGE_SYSFS_IIN_THERMAL:
		if ((val < INVALID_LIMIT_CUR) || (val > MAX_CURRENT))
			return -EINVAL;
		pr_info("set input current = %ld\n", val);
		huawei_charger_set_in_thermal(di, val);
		break;
	case CHARGE_SYSFS_IIN_RUNNINGTEST:
		if ((val < INVALID_LIMIT_CUR) || (val > MAX_CURRENT))
			return -EINVAL;
		pr_info("set running test val = %ld\n", val);
		huawei_charger_set_runningtest(di, val);
		break;
	case CHARGE_SYSFS_IIN_RT_CURRENT:
		if ((val < INVALID_LIMIT_CUR) || (val > MAX_CURRENT))
			return -EINVAL;
		pr_info("set rt test val = %ld\n", val);
		huawei_charger_set_rt_current(di, val);
		break;
	case CHARGE_SYSFS_ENABLE_CHARGER:
		if ((val < DIS_CHG) || (val > EN_CHG))
			return -EINVAL;
		pr_info("set enable charger val = %ld\n", val);
		huawei_set_enable_charge(di, val);
		break;
	case CHARGE_SYSFS_WATCHDOG_DISABLE:
		if ((val < DIS_WDT) || (val > EN_WDT))
			return -EINVAL;
		pr_info("set watchdog disable val = %ld\n", val);
		huawei_charger_set_wdt_disable(di, val);
		break;
	case CHARGE_SYSFS_HIZ:
		if ((val < DIS_HIZ) || (val > EN_HIZ))
			return -EINVAL;
		pr_info("set hz mode val = %ld\n", val);
		huawei_charger_set_hz_mode(di, val);
		break;
#ifdef CONFIG_DIRECT_CHARGER
	case CHARGE_SYSFS_REGULATION_VOLTAGE:
		if ((strict_strtol(buf, DEC_BASE, &val) < 0) || (val < 3200) || (val > 4400))
			return -EINVAL;
		di->sysfs_data.vterm_rt = val;
		ret = charger_set_constant_voltage(di->sysfs_data.vterm_rt * 1000);
		hwlog_info("RUNNINGTEST set terminal voltage = %d\n",
			   di->sysfs_data.vterm_rt);
		break;
	case CHARGE_SYSFS_ADAPTOR_VOLTAGE:
		if ((strict_strtol(buf, DEC_BASE, &val) < 0) || (val < 0))
			return -EINVAL;
		charge_set_adapter_voltage(val, RESET_ADAPTER_SYSFS,
			PD_VOLTAGE_CHANGE_WORK_TIMEOUT);
		break;
	case CHARGE_SYSFS_VTERM_DEC:
		huawei_set_basp_vterm_dec(val);
		break;
	case CHARGE_SYSFS_ICHG_RATIO:
		huawei_set_basp_ichg_ratio(val);
		break;
#endif
	default:
		break;
	}
	return count;
}

int get_reset_adapter(void)
{
	return g_charger_device_para->reset_adapter;
}

static int dcp_charger_enable_charge(struct charge_device_info *di, int val)
{
	if (!di || !(di->batt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, !!val);
	di->sysfs_data.charge_enable = get_property_from_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED);
	return 0;
}

static int dcp_charger_set_rt_current(struct charge_device_info *di, int val)
{
	int iin_rt_curr;

	if (!di || !(di->hwbatt_psy)) {
		pr_err("charge_device is not ready\n");
		return -EINVAL;
	}
	/* 0&1:open limit 100:limit to 100 */
	if ((val == 0) || (val == 1)) {
		iin_rt_curr = get_property_from_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_MAX);
		pr_info("%s rt_current =%d\n", __func__, iin_rt_curr);
	} else if ((val <= MIN_CURRENT) && (val > 1)) {
		iin_rt_curr = MIN_CURRENT;
	} else {
		iin_rt_curr = val;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, iin_rt_curr);

	di->sysfs_data.iin_rt_curr = iin_rt_curr;
	return 0;
}


static int dcp_set_enable_charger(unsigned int val)
{
	struct charge_device_info *di = g_charger_device_para;
	int ret;
	int batt_temp;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	if ((val < DIS_CHG) || (val > EN_CHG))
		return -1;

	hwlog_info("dcp: RUNNINGTEST chg stat = %d, set charge enable = %d\n",
		di->sysfs_data.charge_enable, val);

	batt_temp = battery_get_bat_temperature();
#ifndef CONFIG_HLTHERM_RUNTEST
	if ((val == EN_CHG) && !is_batt_temp_normal(batt_temp)) {
		hwlog_err("dcp: battery temp is %d, abandon enable_charge\n",
			batt_temp);
		return -1;
	}
#endif

	ret = dcp_charger_enable_charge(di, val);
	return ret;
}

static int dcp_get_enable_charger(unsigned int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	*val = di->sysfs_data.charge_enable;
	return 0;
}

static int sdp_set_iin_limit(unsigned int val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	/* sdp current limit > 450mA */
	if (power_cmdline_is_factory_mode() && (val >= 450)) {
		power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
			POWER_SUPPLY_PROP_SDP_CURRENT_LIMIT, val);
		hwlog_info("set sdp ibus current is: %u\n", val);
	}
	return 0;
}

static int dcp_set_iin_limit(unsigned int val)
{
	struct charge_device_info *di = g_charger_device_para;
	int ret;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	ret = dcp_charger_set_rt_current(di, val);

	hwlog_info("set input current is:%d\n", val);

	return ret;
}

static int dcp_get_iin_limit(unsigned int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	*val = di->sysfs_data.iin_rt_curr;

	return 0;
}

static int get_iin_thermal_charge_type(void)
{
	int vol;
	int type;
	int conver_type;
	struct charge_device_info *di = g_charger_device_para;

	if (!di)
		return IIN_THERMAL_WCURRENT_5V;

	vol = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGE_VOLTAGE_NOW);
	type = power_supply_return_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE);
	conver_type = converse_usb_type(type);
	if (conver_type == CHARGER_TYPE_WIRELESS)
		return (vol / MV_PER_VOLT < ADAPTER_7V) ?
			IIN_THERMAL_WLCURRENT_5V : IIN_THERMAL_WLCURRENT_9V;

	return (vol / MV_PER_VOLT < ADAPTER_7V) ?
		IIN_THERMAL_WCURRENT_5V : IIN_THERMAL_WCURRENT_9V;
}

#ifndef CONFIG_HLTHERM_RUNTEST
static int dcp_set_iin_limit_array(unsigned int idx, unsigned int val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	/*
	 * set default iin when value is 0 or 1
	 * set 100mA for iin when value is between 2 and 100
	 */
	if ((val == 0) || (val == 1))
		di->sysfs_data.iin_thl_array[idx] = DEFAULT_IIN_THL;
	else if ((val > 1) && (val <= MIN_CURRENT))
		di->sysfs_data.iin_thl_array[idx] = MIN_CURRENT;
	else
		di->sysfs_data.iin_thl_array[idx] = val;

	hwlog_info("thermal send input current = %d, type: %u\n",
		di->sysfs_data.iin_thl_array[idx], idx);

	if (idx != get_iin_thermal_charge_type())
		return 0;

	di->sysfs_data.iin_thl = di->sysfs_data.iin_thl_array[idx];
	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_IN_THERMAL, di->sysfs_data.iin_thl);
	hwlog_info("thermal set input current = %d\n", di->sysfs_data.iin_thl);
	return 0;
}
#else
static int dcp_set_iin_limit_array(unsigned int idx, unsigned int val)
{
	return 0;
}
#endif /* CONFIG_HLTHERM_RUNTEST */

static int dcp_set_iin_thermal(unsigned int index, unsigned int iin_thermal_value)
{
	if (index >= IIN_THERMAL_CHARGE_TYPE_END) {
		hwlog_err("error index: %u, out of boundary\n", index);
		return -EINVAL;
	}
	return dcp_set_iin_limit_array(index, iin_thermal_value);
}

static int dcp_set_iin_thermal_all(unsigned int value)
{
	int i;

	for (i = IIN_THERMAL_CHARGE_TYPE_BEGIN; i < IIN_THERMAL_CHARGE_TYPE_END; i++) {
		if (dcp_set_iin_limit_array(i, value))
			return -EINVAL;
	}
	return 0;
}

void update_iin_thermal(void)
{
	struct charge_device_info *di = g_charger_device_para;
	unsigned int idx;

	if (!di) {
		hwlog_err("di is null\n");
		return;
	}
	idx = get_iin_thermal_charge_type();
	di->sysfs_data.iin_thl = di->sysfs_data.iin_thl_array[idx];
	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_IN_THERMAL, di->sysfs_data.iin_thl);
	hwlog_info("update iin_thermal current: %d, type: %u",
		di->sysfs_data.iin_thl, idx);
}

static int dcp_set_ichg_limit(unsigned int val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di or core_data is null\n");
		return -EINVAL;
	}

	/* 1000: mA convert to uA */
	return power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, val * 1000);
}

static void huawei_battery_set_vbat_max(int max)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di || !(di->batt_psy)) {
		pr_err("charge_device is not ready\n");
		return;
	}

	power_supply_set_int_property_value_with_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX, max);
}

static int dcp_set_vterm_dec(unsigned int val)
{
	int vterm_basp;
	struct charge_device_info *di = g_charger_device_para;
	struct charger_manager *info = huawei_get_pinfo();

	if (!di) {
		hwlog_err("di or core_data is null\n");
		return -EINVAL;
	}

	if (!info) {
		hwlog_err("charger info is null\n");
		return -EINVAL;
	}

	val *= 1000;
	vterm_basp = info->data.battery_cv - val;
	if (vterm_basp < 0) {
		hwlog_err("vterm_basp is invalid, val:%u\n", val);
		return -EINVAL;
	}

	huawei_battery_set_vbat_max(vterm_basp);
	hwlog_info("set charger terminal voltage is:%d, dec:%u\n",
		vterm_basp, val);
	return 0;
}

static int dcp_get_hota_iin_limit(unsigned int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	*val = di->hota_iin_limit;
	return 0;
}

static int dcp_get_startup_iin_limit(unsigned int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	*val = di->startup_iin_limit;
	return 0;
}

#ifdef CONFIG_FCP_CHARGER
static int fcp_get_rt_test_time(unsigned int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	*val = di->rt_test_time;
	return 0;
}
extern enum fcp_check_stage_type fcp_get_stage(void);

u32 get_rt_curr_th(void)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return 0;
	}
	return di->rt_curr_th;
}

void set_rt_test_result(bool flag)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!di) {
		hwlog_err("di is null\n");
		return;
	}
	di->rt_test_succ = flag;
}

static int fcp_get_rt_test_result(unsigned int *val)
{
	int tbatt;
	int ibat;
	struct charge_device_info *di = g_charger_device_para;
	enum fcp_check_stage_type fcp_charge_flag = fcp_get_stage();
	struct charger_data *pdata = NULL;
	struct charger_manager *info = huawei_get_pinfo();

	if (!info) {
		chr_err("charger is not rdy ,skip1\n");
		return -EINVAL;
	}

	pdata = &info->chg1_data;

	if (!di) {
		hwlog_err("di is null\n");
		return -EINVAL;
	}

	tbatt = battery_get_bat_temperature();
	ibat = battery_get_bat_current() / 10;
	if ((pdata->charging_current_limit == CHARGE_CURRENT_0000_MA) ||
		(di->sysfs_data.charge_enable == 0) ||
		(tbatt >= RT_TEST_TEMP_TH) ||
		di->rt_test_succ ||
		(fcp_charge_flag &&
		(ibat >= (int)di->rt_curr_th)))
		*val = 0; /* 0: succ */
	else
		*val = 1; /* 1: fail */

	di->rt_test_succ = false;
	return 0;
}
#endif

static int set_adapter_voltage(int val)
{
	if (val < 0)
		return -EINVAL;
#ifdef CONFIG_DIRECT_CHARGER
	charge_set_adapter_voltage(val, RESET_ADAPTER_SYSFS,
		PD_VOLTAGE_CHANGE_WORK_TIMEOUT);
#endif
	return 0;
}

static int get_adapter_voltage(int *val)
{
	struct charge_device_info *di = g_charger_device_para;

	if (!val || !di)
		return -EINVAL;
	if (di->reset_adapter)
		*val = ADAPTER_5V;
	else
		*val = ADAPTER_9V;
	return 0;
}

static struct power_if_ops sdp_if_ops = {
	.set_iin_limit = sdp_set_iin_limit,
	.type_name = "sdp",
};

static struct power_if_ops dcp_if_ops = {
	.set_enable_charger = dcp_set_enable_charger,
	.get_enable_charger = dcp_get_enable_charger,
	.set_iin_limit = dcp_set_iin_limit,
	.get_iin_limit = dcp_get_iin_limit,
	.set_ichg_limit = dcp_set_ichg_limit,
	.set_vterm_dec = dcp_set_vterm_dec,
	.get_hota_iin_limit = dcp_get_hota_iin_limit,
	.get_startup_iin_limit = dcp_get_startup_iin_limit,
	.set_iin_thermal = dcp_set_iin_thermal,
	.set_iin_thermal_all = dcp_set_iin_thermal_all,
	.set_adap_volt = set_adapter_voltage,
	.get_adap_volt = get_adapter_voltage,
	.type_name = "dcp",
};

#ifdef CONFIG_FCP_CHARGER
static struct power_if_ops fcp_if_ops = {
	.get_rt_test_time = fcp_get_rt_test_time,
	.get_rt_test_result = fcp_get_rt_test_result,
	.type_name = "hvc",
};
#endif

static struct charge_device_info *charge_device_info_alloc(void)
{
	struct charge_device_info *di = NULL;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return NULL;

	return di;
}

static void charge_device_info_free(struct charge_device_info *di)
{
	if (!di)
		return;

	kfree(di);
}

static int hw_psy_init(struct charge_device_info *di)
{
	struct power_supply *usb_psy = NULL;
	struct power_supply *batt_psy = NULL;
	struct power_supply *hwbatt_psy = NULL;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("batt supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	hwbatt_psy = power_supply_get_by_name("hwbatt");
	if (!batt_psy) {
		pr_err("batt supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	di->usb_psy = usb_psy;
	di->batt_psy = batt_psy;
	di->hwbatt_psy = hwbatt_psy;
	return 0;
}

static void parameter_init(struct charge_device_info *di)
{
	if (!di)
		return;

	di->sysfs_data.iin_thl = DEFAULT_IIN_THL;
	di->sysfs_data.iin_rt = DEFAULT_IIN_RT;
	di->sysfs_data.iin_rt_curr = DEFAULT_IIN_CURRENT;
	di->sysfs_data.iin_thl_array[IIN_THERMAL_WCURRENT_5V] = DEFAULT_IIN_THL;
	di->sysfs_data.iin_thl_array[IIN_THERMAL_WCURRENT_9V] = DEFAULT_IIN_THL;
	di->sysfs_data.iin_thl_array[IIN_THERMAL_WLCURRENT_5V] = DEFAULT_IIN_THL;
	di->sysfs_data.iin_thl_array[IIN_THERMAL_WLCURRENT_9V] = DEFAULT_IIN_THL;
	di->sysfs_data.hiz_mode = DEFAULT_HIZ_MODE;
	di->chrg_config = DEFAULT_CHRG_CONFIG;
	di->factory_diag = DEFAULT_FACTORY_DIAG;
	di->sysfs_data.charge_enable = DEFAULT_CHRG_CONFIG;
	di->sysfs_data.vterm_rt = get_property_from_psy(di->hwbatt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN);
}

static void prase_charge_dts(struct platform_device *pdev)
{
	int ret;
	struct charge_device_info *di = NULL;
	int vterm = 0;
	int iterm = 0;

	di = dev_get_drvdata(&pdev->dev);
	if (!di)
		return;

	parameter_init(di);

	ret = of_property_read_u32(pdev->dev.of_node, "huawei,vterm", &vterm);
	if (ret) {
		pr_info("There is no huawei,vterm,use default\n");
		di->sysfs_data.vterm = DEFAULT_VTERM;
	} else {
		di->sysfs_data.vterm = vterm;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "huawei,iterm", &iterm);
	if (ret) {
		pr_info("There is no huawei,iterm,use default\n");
		di->sysfs_data.iterm = DEFAULT_ITERM;
	} else {
		di->sysfs_data.iterm = iterm;
	}
	ret = of_property_read_u32(pdev->dev.of_node,
		"huawei,fcp-test-delay", &di->fcp_test_delay);
	if (ret) {
		pr_info("There is no fcp test delay setting, use default\n");
		di->fcp_test_delay = DEFAULT_FCP_TEST_DELAY;
	}
	ret = of_property_read_u32(pdev->dev.of_node,
		"startup_iin_limit", &di->startup_iin_limit);
	if (ret) {
		pr_info("There is no startup_iin_limit setting, use default\n");
		di->startup_iin_limit = 0;
	}
	ret = of_property_read_u32(pdev->dev.of_node,
		"hota_iin_limit", &di->hota_iin_limit);
	if (ret) {
		pr_info("There is no hota_iin_limit setting, use default\n");
		di->hota_iin_limit = 0;
	}
	ret = of_property_read_u32(pdev->dev.of_node,
		"en_eoc_max_delay", &di->en_eoc_max_delay);
	if (ret) {
		pr_info("There is no en_eoc_max_delay setting, use default\n");
		di->en_eoc_max_delay = 0;
	}
}

static void charge_sysfs_create_group(struct device *dev)
{
	charge_sysfs_init_attrs();
	power_sysfs_create_link_group("hw_power", "charger", "charge_data",
		dev, &charge_sysfs_attr_group);
}

static void charge_sysfs_remove_group(struct device *dev)
{
	power_sysfs_remove_link_group("hw_power", "charger", "charge_data",
		dev, &charge_sysfs_attr_group);
}

static struct power_log_ops huawei_charger_log_ops = {
	.dev_name = "batt_info",
	.dump_log_head = huawei_charger_get_log_head,
	.dump_log_content = huawei_charger_value_dump,
};

static int charge_probe(struct platform_device *pdev)
{
	int ret;
	struct charge_device_info *di = NULL;

	if (!pdev)
		return -EPROBE_DEFER;

	di = charge_device_info_alloc();
	if (!di)
		return -ENOMEM;

	ret = hw_psy_init(di);
	if (ret < 0)
		goto psy_get_fail;

	di->dev = &(pdev->dev);
	dev_set_drvdata(&(pdev->dev), di);
	prase_charge_dts(pdev);
	charge_sysfs_create_group(di->dev);

	g_charger_device_para = di;
	huawei_kick_otg_wdt();
	g_charge_dev = di->dev;
	ret = charge_event_poll_register(g_charge_dev);
	if (ret)
		pr_err("poll register fail\n");
	power_if_ops_register(&sdp_if_ops);
	power_if_ops_register(&dcp_if_ops);
#ifdef CONFIG_ADAPTER_PROTOCOL_FCP
	power_if_ops_register(&fcp_if_ops);
#endif
	huawei_charger_log_ops.dev_data = (void *)di;
	ret = power_log_ops_register(&huawei_charger_log_ops);
	if (ret)
		pr_err("power log reg fail\n");
	pr_info("huawei charger probe ok!\n");
	return ret;

psy_get_fail:
	charge_device_info_free(di);
	g_charger_device_para = NULL;
	return ret;
}

static void charge_event_poll_unregister(struct device *dev)
{
	if (!dev) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return;
	}

	sysfs_remove_file(&dev->kobj, &dev_attr_poll_charge_start_event.attr);
	g_sysfs_poll = NULL;
}

static int charge_remove(struct platform_device *pdev)
{
	struct charge_device_info *di = NULL;

	if (!pdev) {
		pr_err("%s: invalid param, fatal error\n", __func__);
		return -EINVAL;
	}

	di = dev_get_drvdata(&pdev->dev);
	if (!di) {
		pr_err("%s: Cannot get charge device info,fatal error\n",
			__func__);
		return -EINVAL;
	}

	charge_event_poll_unregister(g_charge_dev);
	charge_sysfs_remove_group(di->dev);
	charge_device_info_free(di);
	g_charger_device_para = NULL;
	dev_set_drvdata(&(pdev->dev), NULL);

	return 0;
}

static void charge_shutdown(struct platform_device  *pdev)
{
	if (!pdev)
		pr_err("%s: invalid param\n", __func__);
}

#ifdef CONFIG_PM
static int charge_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (!pdev)
		pr_err("%s: invalid param\n", __func__);
	return 0;
}

static int charge_resume(struct platform_device *pdev)
{
	if (!pdev)
		pr_err("%s: invalid param\n", __func__);
	return 0;
}
#endif

static const struct of_device_id charge_match_table[] = {
	{
		.compatible = "huawei,charger",
		.data = NULL,
	},
	{
	},
};

static struct platform_driver g_charge_driver = {
	.probe = charge_probe,
	.remove = charge_remove,
#ifdef CONFIG_PM
	.suspend = charge_suspend,
	.resume = charge_resume,
#endif
	.shutdown = charge_shutdown,
	.driver = {
		.name = "huawei,charger",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(charge_match_table),
	},
};

static int __init charge_init(void)
{
	int ret;

	ret = platform_driver_register(&g_charge_driver);
	if (ret) {
		pr_info("register platform_driver_register failed!\n");
		return ret;
	}
	pr_info("%s: %d\n", __func__, __LINE__);
	return 0;
}

static void __exit charge_exit(void)
{
	platform_driver_unregister(&g_charge_driver);
}

late_initcall(charge_init);
module_exit(charge_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("huawei charger module driver");
MODULE_AUTHOR("HUAWEI Inc");
