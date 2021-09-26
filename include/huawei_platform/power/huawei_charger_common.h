/*
 * huawei_charger_common.h
 *
 * common interface for huawei charger driver
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
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

#ifndef _HUAWEI_CHARGER_COMMON_H_
#define _HUAWEI_CHARGER_COMMON_H_

#include <linux/types.h>

struct charge_extra_ops {
	bool (*check_ts)(void);
	bool (*check_otg_state)(void);
	enum fcp_check_stage_type (*get_stage)(void);
	unsigned int (*get_charger_type)(void);
	int (*set_state)(int state);
	int (*get_charge_current)(void);
	int (*get_batt_by_usb_id)(void);
};

int charge_extra_ops_register(struct charge_extra_ops *ops);
int get_charger_vbus_vol(void);
int get_charger_ibus_curr(void);
int charge_check_input_dpm_state(void);
int charge_check_charger_plugged(void);
int get_charger_bat_id_vol(void);
int charge_enable_hz(bool hz_enable);
int charge_enable_force_sleep(bool enable);
int charge_enable_eoc(bool eoc_enable);
bool huawei_charge_done(void);
int mt_get_charger_status(void);

bool charge_check_charger_ts(void);
bool charge_check_charger_otg_state(void);
enum fcp_check_stage_type fcp_get_stage_status(void);
unsigned int huawei_get_charger_type(void);
int charge_set_charge_state(int state);
int get_charge_current_max(void);
int huawei_get_ac_online(void);
int huawei_get_vbat_max(void);
void reset_cur_delay_eoc_count(void);
void clear_cur_delay_eoc_count(void);
void huawei_check_delay_en_eoc(void);
void huawei_charging_full_check(void);
int charge_enable_powerpath(bool path_enable);

#endif /* _HUAWEI_CHARGER_COMMON_H_ */
