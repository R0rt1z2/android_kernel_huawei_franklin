/*
 * huawei_mtk_battery.h
 *
 * battery solution of huawei
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

#ifndef HUAWEI_MTK_BATTERY_H
#define HUAWEI_MTK_BATTERY_H

#include <linux/types.h>
#include <linux/platform_device.h>

#define MAH_TO_UAH     100
#define MAX_DESIGN_UAH 5000000
#define MIN_DESIGN_UAH 3000000
#define FCC_MAX_DESIGN 50000
#define HW_BATT_ID_EN  1
#define NTC_PARA_LEVEL 10

enum ntc_temp_compensation_para_info {
	NTC_PARA_ICHG = 0,
	NTC_PARA_VALUE,
	NTC_PARA_TOTAL,
};

void set_ui_batt_temp_hltherm(int batt_temp, int *ui_temp);
void set_low_capacity_hltherm(int batt_capacity, int *set_capacity);
void set_power_supply_health(int batt_temp, int *health_status);
int fg_get_soc_by_ocv(int ocv);
int fg_get_fcc(void);
void huawei_mtk_battery_dts_parse(struct device_node *np);
bool is_set_aging_fatcor(void);
void huawei_set_aging_factor(void);
void battid_parse_para(struct device_node *np);
void hw_get_profile_id(void);
char *huawei_get_battery_type(void);
int battery_get_fcc_design(void);
void batt_ntc_parse_para(struct device_node *np);
int set_ntc_compensation_temp(int temp_val, bool cur_state, int cur_temp);

#endif /* HUAWEI_MTK_BATTERY_H */
