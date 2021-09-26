/*
 * huawei_mtk_battery.c
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

#include <linux/power/huawei_mtk_battery.h>
#include <linux/power/huawei_mtk_charger.h>
#include <linux/power/huawei_charger.h>
#include <huawei_platform/power/huawei_charger_common.h>
#include <linux/of.h>
#include "mtk_battery_internal.h"
#include "mtk_charger_intf.h"
#include "../../../auxadc/mtk_auxadc.h"
#ifdef CONFIG_DIRECT_CHARGER
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#endif
#include <chipset_common/hwpower/common_module/power_supply_interface.h>
#include <chipset_common/hwpower/battery/battery_model_public.h>
#include <chipset_common/hwpower/common_module/power_algorithm.h>
#include <chipset_common/hwpower/common_module/power_cmdline.h>

#define HLTHERM_BATTERY_TEMP_UI_MAX    250
#define FACTORY_LOW_BATT_CAPACITY      10
#define HLTHERM_LOW_BATT_CAPACITY      30
#define DELAY_FOR_QRY_SOC              5
#define BATT_HEALTH_TEMP_HOT           60
#define BATT_HEALTH_TEMP_WARM          48
#define BATT_HEALTH_TEMP_COOL          10
#define BATT_HEALTH_TEMP_COLD          (-20)
#define AGING_FACTOR_SKIP_TIMES        2
#define TEMP_UNIT_CONVERT              10
#define VOLT_GROUP_MAX                 8
#define BAT_BRAND_LEN_MAX              16
#define BAT_ID_CHANNEL                 4
#define BAT_VENDOR_NAME_LEN            32
#define DES_BASE                       10
#define DEFAULT_ID_VOL_LOW             250000
#define DEFAULT_ID_VOL_HIGH            375000
#define MAX_ID_VOL                     1800000
#define GET_BATT_ID_INDEX_MAX          2
#define COMPENSATION_THRESHOLD         200

static int g_batt_len;
static int g_batt_id_channel = BAT_ID_CHANNEL;
static int g_min_fcc = MIN_DESIGN_UAH;
static int g_max_fcc = MAX_DESIGN_UAH;
static int g_fcc_design = MAX_DESIGN_UAH;
static int get_batt_id_index;

struct batt_id_voltage {
	char vendor_name[BAT_VENDOR_NAME_LEN];
	int id_vol_low;
	int id_vol_high;
};

static struct batt_id_voltage g_batt_id[VOLT_GROUP_MAX];
static int g_ntc_compensation_is;
static struct compensation_para g_ntc_para[NTC_PARA_LEVEL];

enum bat_id_info {
	PARA_BAT_ID = 0,
	PARA_VOL_LOW,
	PARA_VOL_HIGH,
	PARA_BAT_TOTAL,
};

void set_ui_batt_temp_hltherm(int batt_temp, int *ui_temp)
{
#ifdef CONFIG_HLTHERM_RUNTEST
	if (!ui_temp)
		return;

	if ((batt_temp * TEMP_UNIT_CONVERT) > HLTHERM_BATTERY_TEMP_UI_MAX)
		*ui_temp = HLTHERM_BATTERY_TEMP_UI_MAX;
#endif
}

void set_low_capacity_hltherm(int batt_capacity, int *set_capacity)
{
	if (!set_capacity)
		return;
#ifdef CONFIG_HLTHERM_RUNTEST
	if (batt_capacity < HLTHERM_LOW_BATT_CAPACITY)
		*set_capacity = HLTHERM_LOW_BATT_CAPACITY;
#else
	if (power_cmdline_is_factory_mode()) {
		if (batt_capacity < FACTORY_LOW_BATT_CAPACITY) {
			*set_capacity = FACTORY_LOW_BATT_CAPACITY;
			pr_err("FACTORY set low capacity to 10\n");
		}
	}
#endif
}

void set_power_supply_health(int batt_temp, int *health_status)
{
	if (!health_status)
		return;

	if (batt_temp > BATT_HEALTH_TEMP_HOT)
		*health_status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (batt_temp < BATT_HEALTH_TEMP_COLD)
		*health_status = POWER_SUPPLY_HEALTH_COLD;
	else if (batt_temp > BATT_HEALTH_TEMP_WARM)
		*health_status = POWER_SUPPLY_HEALTH_WARM;
	else if (batt_temp < BATT_HEALTH_TEMP_COOL)
		*health_status = POWER_SUPPLY_HEALTH_COOL;
	else
		*health_status = POWER_SUPPLY_HEALTH_GOOD;
}

int fg_get_soc_by_ocv(int ocv)
{
	fg_ocv_query_soc(ocv);
	msleep(DELAY_FOR_QRY_SOC);
	return gm.algo_ocv_to_soc;
}

int fg_get_fcc(void)
{
	return gm.fcc;
}

void huawei_mtk_battery_dts_parse(struct device_node *np)
{
	int fcc = 0;
	unsigned int vbus_r = 0;

	if (!np)
		return;

	if (!of_property_read_u32(np, "rbat_pull_up_r", &vbus_r))
		gm.rbat.rbat_pull_up_r = vbus_r;

	if (!of_property_read_u32(np, "vbus_r_charger_1", &vbus_r))
		fg_cust_data.r_charger_1 = vbus_r;

	if (!of_property_read_u32(np, "batt_id_channel", &g_batt_id_channel))
		pr_info("g_batt_id_channel:%d\n", g_batt_id_channel);

	if (!of_property_read_u32(np, "min_fcc", &fcc))
		g_min_fcc = fcc;
	else
		g_min_fcc = MIN_DESIGN_UAH;
	pr_info("get min_fcc : %d\n", g_min_fcc);

	if (!of_property_read_u32(np, "max_fcc", &fcc)) {
		g_max_fcc = fcc;
		g_fcc_design = fcc;
	} else {
		g_max_fcc = MAX_DESIGN_UAH;
		g_fcc_design = MAX_DESIGN_UAH;
	}
	pr_info("get max_fcc : %d\n", g_max_fcc);
}

bool is_set_aging_fatcor(void)
{
	static int first_in;

	/* every power on will trigger 2 times, skip */
	if (first_in < AGING_FACTOR_SKIP_TIMES) {
		pr_info("fg_set_aging_factor in %d\n", first_in);
		first_in++;
		return false;
	}
	return true;
}

int battery_get_fcc_design(void)
{
	return g_fcc_design;
}

void huawei_set_aging_factor(void)
{
	gm.fcc = ((g_fcc_design / MAH_TO_UAH) * gm.aging_factor) / MAH_TO_UAH;
	pr_info("gm.aging_factor:%d gm.algo_qmax:%d gm.fcc:%d\n",
		gm.aging_factor, gm.algo_qmax, gm.fcc);
	if ((gm.fcc < g_min_fcc) || (gm.fcc > g_max_fcc)) {
		pr_err("not ok use default fcc\n");
		gm.fcc = g_fcc_design;
	}

	cap_learning_event_done_notify();
	dsm_learning_event_report(gm.fcc);
}

static void get_batt_id_para(struct device_node *np, int len)
{
	int i;
	int row;
	int col;
	int ret;
	int idata = 0;
	const char *tmp_string = NULL;

	for (i = 0; i < len; i++) {
		ret = of_property_read_string_index(np, "bat_para",
			i, &tmp_string);
		if (ret) {
			pr_err("bat_para dts read failed\n");
			return;
		}

		row = i / PARA_BAT_TOTAL;
		col = i % PARA_BAT_TOTAL;

		switch (col) {
		case PARA_BAT_ID:
			if (strlen(tmp_string) >= BAT_VENDOR_NAME_LEN - 1)
				strncpy(g_batt_id[row].vendor_name,
					tmp_string, BAT_VENDOR_NAME_LEN - 1);
			else
				strncpy(g_batt_id[row].vendor_name,
					tmp_string, strlen(tmp_string));
			break;
		case PARA_VOL_LOW:
			ret = kstrtoint(tmp_string, DES_BASE, &idata);
			if ((ret != 0) || (idata < 0) || (idata > MAX_ID_VOL))
				return;

			g_batt_id[row].id_vol_low = idata;
			break;
		case PARA_VOL_HIGH:
			ret = kstrtoint(tmp_string, DES_BASE, &idata);
			if ((ret != 0) || (idata < 0) || (idata > MAX_ID_VOL))
				return;

			g_batt_id[row].id_vol_high = idata;
			break;
		default:
			break;
		}
	}
}

void battid_parse_para(struct device_node *np)
{
	int i;
	int array_len;

	if (!np)
		return;

	array_len = of_property_count_strings(np, "bat_para");
	g_batt_len = array_len / PARA_BAT_TOTAL;
	pr_info("bat_para array_len=%d\n", array_len);
	if (g_batt_len > VOLT_GROUP_MAX)
		g_batt_len = VOLT_GROUP_MAX;

	if ((array_len <= 0) ||
		(array_len % PARA_BAT_TOTAL != 0) ||
		(array_len > VOLT_GROUP_MAX * PARA_BAT_TOTAL)) {
		g_batt_id[0].id_vol_low = DEFAULT_ID_VOL_LOW;
		g_batt_id[0].id_vol_high = DEFAULT_ID_VOL_HIGH;
		strncpy(g_batt_id[0].vendor_name, "DeasyCoslight",
			strlen("DeasyCoslight"));
		pr_err("bat_para invalid\n");
		return;
	}

	get_batt_id_para(np, array_len);
	for (i = 0; i < g_batt_len; i++)
		pr_info("bat_para[%d]=%s %d %d\n", i,
			g_batt_id[i].vendor_name,
			g_batt_id[i].id_vol_low,
			g_batt_id[i].id_vol_high);
}

void hw_get_profile_id(void)
{
	int id_volt = 0;
	int i;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
#ifdef CONFIG_MTK_AUXADC
	int ret;
#endif

	if (bat_model_name()) {
		gm.battery_id = bat_model_id_index();
		return;
	}

#ifndef CONFIG_MTK_AUXADC
	id_volt = get_charger_bat_id_vol();
#else
	ret = IMM_GetOneChannelValue_Cali(g_batt_id_channel, &id_volt);
	if (ret != 0) {
		pr_err("getchannelvalue fail\n");
		gm.battery_id = 0;
		return;
	}
#endif
	get_batt_id_index++;
	pr_info("batt_id_volt = %d\n", id_volt);
	for (i = 0; i < g_batt_len; i++) {
		if ((id_volt > g_batt_id[i].id_vol_low) &&
			(id_volt < g_batt_id[i].id_vol_high)) {
			bm_err("voltag_low:%d  high:%d\n",
				g_batt_id[i].id_vol_low,
				g_batt_id[i].id_vol_high);
			gm.battery_id = i;
			break;
		}
	}

	if (i == g_batt_len) {
		gm.battery_id = 0;
		if (get_batt_id_index == GET_BATT_ID_INDEX_MAX) {
			get_batt_id_index = 0;
			snprintf(tmp_buf, sizeof(tmp_buf),
				"%s:battery_id use default", __func__);
			power_dsm_report_dmd(POWER_DSM_BATTERY_DETECT,
				DSM_BATTERY_DETECT_ERROR_NO, tmp_buf);
		}
	}
	pr_info("Battery id %d\n", gm.battery_id);
}

char *huawei_get_battery_type(void)
{
	union power_supply_propval val;

	if (g_batt_len == 0) {
		if (power_supply_get_property_value("battery",
			POWER_SUPPLY_PROP_BRAND, &val))
			return "default";
		return (char *)val.strval;
	}

	return g_batt_id[gm.battery_id].vendor_name;
}

void batt_ntc_parse_para(struct device_node *np)
{
	int array_len;
	int i;
	long idata = 0;
	const char *string = NULL;
	int ret;

	if (!np)
		return;
	if (of_property_read_u32(np, "ntc_compensation_is",
		&g_ntc_compensation_is)) {
		pr_info("get ntc_compensation_is failed\n");
		return;
	}
	array_len = of_property_count_strings(np, "ntc_temp_compensation_para");
	if ((array_len <= 0) || (array_len % NTC_PARA_TOTAL != 0)) {
		pr_err("ntc is invaild,please check ntc_temp_para number\n");
		return;
	}
	if (array_len > NTC_PARA_LEVEL * NTC_PARA_TOTAL) {
		array_len = NTC_PARA_LEVEL * NTC_PARA_TOTAL;
		pr_err("temp is too long use only front %d paras\n", array_len);
		return;
	}

	for (i = 0; i < array_len; i++) {
		ret = of_property_read_string_index(np,
			"ntc_temp_compensation_para", i, &string);
		if (ret) {
			pr_err("get ntc_temp_compensation_para failed\n");
			return;
		}
		/* 10 means decimalism */
		ret = kstrtol(string, 10, &idata);
		if (ret)
			break;

		switch (i % NTC_PARA_TOTAL) {
		case NTC_PARA_ICHG:
			g_ntc_para[i / NTC_PARA_TOTAL].refer = idata;
			break;
		case NTC_PARA_VALUE:
			g_ntc_para[i / NTC_PARA_TOTAL].comp_value = idata;
			break;
		default:
			pr_err("ntc_temp_compensation_para get failed\n");
		}
		pr_info("ntc_temp_compensation_para[%d][%d] = %ld\n",
			i / (NTC_PARA_TOTAL), i % (NTC_PARA_TOTAL), idata);
	}
}

int set_ntc_compensation_temp(int temp_val, bool cur_state, int cur_temp)
{
	int temp_with_compensation = temp_val;
	struct common_comp_data comp_data;

	if (!cur_state)
		return temp_with_compensation;

	if ((g_ntc_compensation_is == 1) &&
		(temp_val >= COMPENSATION_THRESHOLD)) {
		comp_data.refer = abs(cur_temp);
		comp_data.para_size = NTC_PARA_LEVEL;
		comp_data.para = g_ntc_para;
		temp_with_compensation = power_get_compensation_value(temp_val,
			&comp_data);
	}

	pr_debug("temp_with_compensation=%d temp_no_compensation=%d ichg=%d\n",
		temp_with_compensation, temp_val, cur_temp);
	return temp_with_compensation;
}
