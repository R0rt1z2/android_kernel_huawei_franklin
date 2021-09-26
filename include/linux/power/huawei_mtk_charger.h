/*
 * huawei_mtk_charger.h
 *
 * charge solution of huawei
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

#ifndef HUAWEI_MTK_CHARGER_H
#define HUAWEI_MTK_CHARGER_H

#include <linux/platform_device.h>
#include <chipset_common/hwpower/common_module/power_dsm.h>
#include "mtk_charger_intf.h"

#define HLTHERM_HIGH_TEMP   65
#define HLTHERM_LOW_TEMP    (-30)
#define HLTHERM_HIGH_TEMP_HYSTERESIS  63

#define MAX_CONFIRM_CNT      3

struct charge_vdpm_data {
	int cap_min;
	int cap_max;
	int vin_dpm;
	int cap_back;
};

enum vdpm_para_info {
	VDPM_PARA_CAP_MIN = 0,
	VDPM_PARA_CAP_MAX,
	VDPM_PARA_DPM,
	VDPM_PARA_CAP_BACK,
	VDPM_PARA_TOTAL,
};

void dsm_batt_full_early_report(void);
void bat_check_error_func(void);
void reset_batt_dsm_notify_en(void);
void set_jeita_ibus_limit(struct charger_manager *info);
void set_jeita_curr(struct sw_jeita_data *sw_jeita,
	struct charger_manager *info);
void charger_event_notify(struct charger_manager *info);
void charger_parse_dt(struct device_node *np, struct charger_manager *info);
void set_vdpm_by_vol(struct charger_manager *info, int vbat);
int charger_dev_set_dpm(struct charger_device *charger_dev, int vbat_dpm);
void reset_vdpm_first_run(void);
unsigned int get_charging_time(void);
void set_charging_time(unsigned int chg_time);
void dsm_learning_event_report(int fcc);
void otg_kick_wdt(struct charger_device *chg_dev, bool en);
void huawei_kick_otg_wdt(void);
void charger_enable(struct charger_consumer *consumer, int chg_en);
int charger_manager_enable_charging(struct charger_consumer *consumer, int idx,
	bool en);
int charger_manager_reset_learned_cc(struct charger_consumer *consumer,
	int val);
int charger_manager_set_voltage_max(struct charger_consumer *consumer,
	int val);
int charger_manager_set_current_max(struct charger_consumer *consumer,
	int val);
int get_input_current(void);
void huawei_dyn_cv_proc(struct charger_manager *info, u32 cv_value);
void huawei_dyn_iterm_proc(struct charger_manager *info, u32 iterm_value);
void huawei_eoc_re_enable_chg(struct charger_manager *info);
#endif
