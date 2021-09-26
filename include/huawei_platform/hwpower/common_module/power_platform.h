/*
 * power_platform.h
 *
 * differentiated interface related to chip platform
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

#ifndef _POWER_PLATFORM_H_
#define _POWER_PLATFORM_H_

#include <huawei_platform/power/wireless/wireless_tx_pwr_ctrl.h>
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <huawei_platform/power/huawei_charger_common.h>
#include <huawei_platform/hwpower/power_proxy.h>
#include <chipset_common/hwpower/direct_charge/direct_charge_ic_manager.h>
#include <chipset_common/hwpower/common_module/power_supply_interface.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/charger_type.h>
#ifdef CONFIG_HW_COMB_KEY
#include <huawei_platform/comb_key/power_key_event.h>
#endif

#define POWER_PLATFOR_SOC_UI_OFFSET    50
#define MT63XX_ADC_CHANNEL_USBID       1001

/* charger_algorithm notify charger_dev */
enum {
	PLAT_EVENT_EOC,
	PLAT_EVENT_RECHARGE,
};

/* charger_dev notify charger_manager */
enum {
	PLAT_CHARGER_DEV_NOTIFY_VBUS_OVP,
	PLAT_CHARGER_DEV_NOTIFY_BAT_OVP,
	PLAT_CHARGER_DEV_NOTIFY_EOC,
	PLAT_CHARGER_DEV_NOTIFY_RECHG,
	PLAT_CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT,
};

extern int mt635x_auxadc_get_sample(int channel);
extern int mt635x_auxadc_get_voltage(int channel);
int power_get_battery_has_removed_flag(void);

static inline int power_platform_get_filter_soc(int base)
{
	return power_proxy_get_filter_sum(base);
}

static inline void power_platform_sync_filter_soc(int rep_soc,
	int round_soc, int base)
{
	power_proxy_sync_filter_soc(rep_soc, round_soc, base);
}

static inline void power_platform_cancle_capacity_work(void)
{
	power_proxy_cancle_capacity_work();
}

static inline void power_platform_restart_capacity_work(void)
{
	power_proxy_restart_capacity_work();
}

static inline int power_platform_get_adc_sample(int adc_channel)
{
	return mt635x_auxadc_get_sample(adc_channel);
}

static inline int power_platform_get_adc_voltage(int adc_channel)
{
	if (adc_channel == MT63XX_ADC_CHANNEL_USBID)
		return get_charger_bat_id_vol();
	return mt635x_auxadc_get_voltage(adc_channel);
}

static inline int power_platform_get_battery_id_voltage(void)
{
	return 0;
}

static inline int power_platform_get_battery_capacity(void)
{
	return battery_get_soc();
}

static inline int power_platform_get_battery_ui_capacity(void)
{
	return battery_get_uisoc();
}

static inline int power_platform_get_battery_temperature(void)
{
	return battery_get_bat_temperature();
}

static inline int power_platform_get_rt_battery_temperature(void)
{
	return battery_get_bat_temperature();
}

static inline char *power_platform_get_battery_brand(void)
{
	return huawei_get_battery_type();
}

static inline int power_platform_get_battery_voltage(void)
{
	return battery_get_bat_voltage();
}

static inline int power_platform_get_battery_current(void)
{
	return battery_get_bat_current();
}

static inline int power_platform_get_battery_current_avg(void)
{
	return battery_get_bat_avg_current();
}

static inline int power_platform_is_battery_removed(void)
{
	return power_get_battery_has_removed_flag();
}

static inline int power_platform_is_battery_exit(void)
{
	int batt_present = true;

	if (power_supply_get_int_property_value("battery",
		POWER_SUPPLY_PROP_PRESENT, &batt_present))
		batt_present = false;

	return batt_present;
}

static inline unsigned int power_platform_get_charger_type(void)
{
	return huawei_get_charger_type();
}

static inline int power_platform_get_vbus_status(void)
{
	return -1;
}

static inline int power_platform_pmic_enable_boost(int value)
{
	return 0;
}

static inline int power_platform_get_vusb_status(int *value)
{
	return -1;
}

static inline bool power_platform_usb_state_is_host(void)
{
	return false;
}

static inline bool power_platform_pogopin_is_support(void)
{
	return false;
}

static inline bool power_platform_pogopin_otg_from_buckboost(void)
{
	return false;
}

#ifdef CONFIG_HW_COMB_KEY
static inline int power_platform_powerkey_register_notifier(struct notifier_block *nb)
{
	return power_key_register_notifier(nb);
}

static inline int power_platform_powerkey_unregister_notifier(struct notifier_block *nb)
{
	return power_key_unregister_notifier(nb);
}

static inline bool power_platform_is_powerkey_up(unsigned long event)
{
	return event == POWER_KEY_PRESS_RELEASE;
}
#else
static inline int power_platform_powerkey_register_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int power_platform_powerkey_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline bool power_platform_is_powerkey_up(unsigned long event)
{
	return false;
}
#endif /* CONFIG_HW_COMB_KEY */

static inline bool power_platform_get_cancel_work_flag(void)
{
	return false;
}

static inline bool power_platform_get_sysfs_wdt_disable_flag(void)
{
	return false;
}

static inline void power_platform_charge_stop_sys_wdt(void)
{
}

static inline void power_platform_charge_feed_sys_wdt(unsigned int timeout)
{
}

static inline int power_platform_set_max_input_current(void)
{
	return -1;
}

static inline void power_platform_start_acr_calc(void)
{
}

static inline int power_platform_get_acr_resistance(int *acr_value)
{
	return -1;
}

#ifdef CONFIG_DIRECT_CHARGER
static inline bool power_platform_in_dc_charging_stage(void)
{
	return direct_charge_in_charging_stage() == DC_IN_CHARGING_STAGE;
}
#else
static inline bool power_platform_in_dc_charging_stage(void)
{
	return false;
}
#endif /* CONFIG_DIRECT_CHARGER */

static inline void power_platform_set_charge_batfet(int val)
{
}

#ifdef CONFIG_FCP_CHARGER
static inline void power_platform_set_charge_hiz(int enable)
{
	charge_enable_hz(enable);
}
#else
static inline void power_platform_set_charge_hiz(int enable)
{
}
#endif /* CONFIG_FCP_CHARGER */

#endif /* _POWER_PLATFORM_H_ */
