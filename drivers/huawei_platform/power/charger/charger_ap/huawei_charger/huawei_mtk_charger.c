/*
 * huawei_mtk_charger.c
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

#include <linux/power/huawei_mtk_charger.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/charger_class.h>
#include "mtk_charger_init.h"
#include "mtk_battery_internal.h"
#include <linux/power/huawei_mtk_battery.h>

#define BATTERY_CURRENT_DIVI     10
#define BAT_VOLT_MIN_LIMINT      2800
#define BAT_VOLT_MAX_LIMINT      4550
#define BAT_TEMP_MAX_LIMINT      50
#define BAT_TEMP_MIN_LIMINT      0
#define BAT_TEMP_LOW_LIMINT      (-10)
#define BAT_TEMP_HIGH_LIMINT     68
#define VBUS_VOLT_MAX_LIMINT     6500
#define BAT_TEMP_OUT_RANGE_HIGH  50
#define BAT_TEMP_WARM_LIMINT     45

#define CHG_CUR_MAX_LIMINT       25000
#define BAT_VOLT_UI_SOC          95
#define BAT_CHR_CURRENT          7000
#define INCHG_OCP_COUNT          3
#define OUTCHG_OCP_COUNT         3
#define IS_OTG_EN                1
#define BATTERY_TEMP_MIN_LIMINT  (-20)
#define BAT_TEMP_OUT_RANGE_LOW   0
#define BAT_CHG_FULL_SOC_LIMINT  95
#define DSM_LEARNING_BUF_LEN     128
#define DSM_BAT_INFO_BUF_LEN     512
#define DMS_REPORT_BUF_LEN       1024
#define VDPM_MAX                 4600
#define VDPM_MAX_V600            4625
#define FCP_VDPM                 8100
#define MV_TO_UV                 1000
#define VDPM_PARA_LEVEL          5
#define VDPM_CBAT_MIN            (-32767)
#define VDPM_CBAT_MAX            32767
#define VDPM_VOLT_MIN            3880
#define VDPM_VOLT_MAX            5080
#define VDPM_DELTA_LIMIT         5
#define WAIT_SECONDS             10
#define WAIT_NS                  0
/* dynamic cv */
#define DYN_ITERM                900000
#define DYN_CV_STEP              22000
#define DYN_CV_IBAT_THRESHOLD    1000000
#define DYN_CV_THRESHOLD         4100000
#define CUSTOM_DEFAULT_ITERM     150000
#define JEITA_IBUS_T0_TO_T1      200000

static unsigned int g_charge_time;
static bool g_batfull_dsm_en = true;
static bool g_dpm_enable;
static int g_custom_iterm;
static bool g_dynamic_cv_en;
static bool g_dynamic_iterm_en;
static int g_vdpm_buf_limit = VDPM_DELTA_LIMIT;
static bool g_vdpm_first_run = true;
static struct charge_vdpm_data g_vdpm_data[VDPM_PARA_LEVEL];
static bool g_otg_wdt_timeout;
static struct task_struct *g_otg_wdt_task;
static struct charger_device *g_chg_dev;
static struct hrtimer g_otg_wdt_timer;
static wait_queue_head_t g_wait_otg_wdt_que;
static bool g_re_enable_chg_flag;
static int g_vbus_max_limit = VBUS_VOLT_MAX_LIMINT;
static bool g_jeita_ibus_limit_en;

/* dmd of power */
static void batt_info_dump(char *ext_buf, unsigned int buf_len)
{
	char buf[DSM_BAT_INFO_BUF_LEN] = {0};
	int vbus = battery_get_vbus();
	int batt_vol = battery_get_bat_voltage();
	int cap = battery_get_uisoc();
	bool batt_present = pmic_is_battery_exist();
	int batt_temp = battery_get_bat_temperature();
	int charger_type = mt_get_charger_type();

	if (!ext_buf)
		return;

	/* WARNNING: if extend, do not exceed max buf length */
	snprintf(buf, DSM_BAT_INFO_BUF_LEN - 1,
		"VBUS:%d,VBAT:%d,cap:%d,exist:%d,batt_temp:%d,chg_type:%d\n",
		vbus, batt_vol, cap, batt_present, batt_temp, charger_type);
	if (strlen(buf) > buf_len) {
		chr_err("%s buf len exceed\n", __func__);
		strncat(ext_buf, buf, buf_len);
		return;
	}
	strncat(ext_buf, buf, strlen(buf));
}

static bool check_batt_not_exist(char *buf, unsigned int size)
{
	bool is_bat_exist = pmic_is_battery_exist();

	if (!is_bat_exist)
		return true;

	return false;
}

static bool check_batt_volt(bool check_high)
{
	int vbatt;
	bool check_result = false;
	int check_cnt;

	vbatt = battery_get_bat_voltage();
	if (check_high) {
		if (vbatt > BAT_VOLT_MAX_LIMINT) {
			for (check_cnt = 0; check_cnt < (MAX_CONFIRM_CNT - 1); check_cnt++) {
				/* wait 100ms get vbat agsin */
				msleep(100);
				vbatt += battery_get_bat_voltage();
			}
			vbatt /= MAX_CONFIRM_CNT;
			/* get averaged value */
		}
		check_result = (vbatt > BAT_VOLT_MAX_LIMINT);
	} else {
		check_result = (vbatt < BAT_VOLT_MIN_LIMINT);
	}

	if (check_result)
		chr_info("%s:vbat=%d\n", __func__, vbatt);

	return check_result;
}

static bool check_batt_volt_overhigh(char *buf, unsigned int size)
{
	return check_batt_volt(true);
}

static bool check_batt_volt_overlow(char *buf, unsigned int size)
{
	return check_batt_volt(false);
}

static bool check_batt_not_terminate(char *buf, unsigned int size)
{
	int vbat;
	enum charger_type chg_type;

	vbat = battery_get_bat_voltage();
	chg_type = mt_get_charger_type();
	if ((vbat > BAT_VOLT_MAX_LIMINT) && (chg_type != CHARGER_UNKNOWN))
		return true;

	return false;
}

static bool check_curr_overhigh(bool check_chg, int *cnt)
{
	bool check_flag = false;
	int bat_current;
	enum charger_type chg_type;

	chg_type = mt_get_charger_type();
	if (check_chg)
		check_flag = (chg_type != CHARGER_UNKNOWN);
	else
		check_flag = (chg_type == CHARGER_UNKNOWN);

	bat_current = battery_get_bat_current();
	bat_current /= BATTERY_CURRENT_DIVI;
	if (check_flag && (abs(bat_current) > BAT_CHR_CURRENT)) {
		(*cnt)++;
		if (*cnt >= INCHG_OCP_COUNT) {
			*cnt = 0;
			return true;
		}
	} else {
		*cnt = 0;
	}

	return false;
}

static bool check_charge_curr_overhigh(char *buf, unsigned int size)
{
	static int curr_over_cnt;

	return check_curr_overhigh(true, &curr_over_cnt);
}

static bool check_discharge_curr_overhigh(char *buf, unsigned int size)
{
	static int curr_over_cnt;

	return check_curr_overhigh(false, &curr_over_cnt);
}

static bool check_vbus_volt_overhigh(char *buf, unsigned int size)
{
	enum charger_type chg_type;
	int vbus_volt;

	chg_type = mt_get_charger_type();
	vbus_volt = battery_get_vbus();
	if ((chg_type != CHARGER_UNKNOWN) && (vbus_volt > g_vbus_max_limit))
		return true;

	return false;
}

static bool check_batt_temp(bool check_high)
{
	int batt_temp;
	bool check_result = false;

	batt_temp = battery_get_bat_temperature();
	if (check_high)
		check_result = (batt_temp >= BAT_TEMP_HIGH_LIMINT);
	else
		check_result = (batt_temp < BATTERY_TEMP_MIN_LIMINT);

	return check_result;
}

static bool check_batt_high_temp(char *buf, unsigned int size)
{
	return check_batt_temp(true);
}

static bool check_batt_low_temp(char *buf, unsigned int size)
{
	return check_batt_temp(false);
}

static bool check_batt_temp_out_range(char *buf, unsigned int size)
{
	int batt_temp = battery_get_bat_temperature();

	if ((batt_temp > BAT_TEMP_OUT_RANGE_HIGH) ||
		(batt_temp < BAT_TEMP_OUT_RANGE_LOW))
		return true;
	return false;
}

static bool check_batt_warm_temp(bool check_high)
{
	int batt_temp;
	bool check_result = false;

	batt_temp = battery_get_bat_temperature();
	if (check_high)
		check_result = (batt_temp >= BAT_TEMP_WARM_LIMINT);
	else
		check_result = (batt_temp < BAT_TEMP_WARM_LIMINT);

	return check_result;
}

static bool check_batt_high_temp_warm(char *buf, unsigned int size)
{
	return check_batt_warm_temp(true);
}

static struct power_dsm_dump g_batt_dsm_array[] = {
	{
		POWER_DSM_BATTERY,
		ERROR_BATT_NOT_EXIST,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_not_exist
	},
	{
		POWER_DSM_BATTERY,
		ERROR_BATT_VOLT_HIGH,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_volt_overhigh
	},
	{
		POWER_DSM_BATTERY,
		ERROR_BATT_VOLT_LOW,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_volt_overlow
	},
	{
		POWER_DSM_BATTERY,
		ERROR_BATT_NOT_TERMINATE,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_not_terminate
	},
	{
		POWER_DSM_BATTERY,
		ERROR_VBUS_VOLT_HIGH,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_vbus_volt_overhigh
	},
	{
		POWER_DSM_BATTERY,
		ERROR_CHARGE_CURR_OVERHIGH,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_charge_curr_overhigh
	},
	{
		POWER_DSM_BATTERY,
		ERROR_DISCHARGE_CURR_OVERHIGH,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_discharge_curr_overhigh
	},
	{
		POWER_DSM_BATTERY,
		ERROR_CHARGE_BATT_TEMP_SHUTDOWN,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_high_temp
	},
	{
		POWER_DSM_BATTERY,
		ERROR_BATT_TEMP_LOW,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_low_temp
	},
	{
		POWER_DSM_BATTERY,
		ERROR_CHARGE_TEMP_FAULT,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_temp_out_range
	},
	{
		POWER_DSM_BATTERY,
		ERROR_CHARGE_VBAT_OVP,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_volt_overhigh
	},
	{
		POWER_DSM_BATTERY,
		ERROR_CHARGE_TEMP_WARM,
		true,
		true,
		NULL,
		NULL,
		batt_info_dump,
		check_batt_high_temp_warm
	},
};

void dsm_learning_event_report(int fcc)
{
	char buf[DSM_LEARNING_BUF_LEN + 1] = {0};

	snprintf(buf, DSM_LEARNING_BUF_LEN,
		"cap learning event done. FCC:%d\n", fcc);
	power_dsm_report_dmd(POWER_DSM_BATTERY, ERROR_REFRESH_FCC_OUTSIDE, buf);
}

void dsm_batt_full_early_report(void)
{
	int bat_soc = battery_get_soc();
	char buf[DMS_REPORT_BUF_LEN + 1] = {0};

	if ((g_batfull_dsm_en == false) || (bat_soc > BAT_CHG_FULL_SOC_LIMINT))
		return;

	snprintf(buf, DMS_REPORT_BUF_LEN,
		"Battery cutoff early. soc=%d\n", bat_soc);
	batt_info_dump(buf, DMS_REPORT_BUF_LEN - strlen(buf));
	power_dsm_report_dmd(POWER_DSM_BATTERY, ERROR_BATT_TERMINATE_TOO_EARLY, buf);
	g_batfull_dsm_en = false;
}

void reset_batt_dsm_notify_en(void)
{
	power_dsm_reset_dump_enable(g_batt_dsm_array,
		ARRAY_SIZE(g_batt_dsm_array));

	g_batfull_dsm_en = true;
}

void bat_check_error_func(void)
{
	power_dsm_dump_data(g_batt_dsm_array, ARRAY_SIZE(g_batt_dsm_array));
}

/* hwbatt : set_property interface */
void charger_enable(struct charger_consumer *consumer, int chg_en)
{
	struct charger_manager *info = NULL;
	enum charger_type chg_type;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s, input para is null\n", __func__);
		return;
	}
	info = consumer->cm;
	chg_type = mt_get_charger_type();
	chr_info("%s, chg_en = %d, charger_type = %d\n",
		__func__, chg_en, chg_type);

	info->cmd_discharging = chg_en ? false : true;

	if (chg_type == CHARGER_UNKNOWN) {
		chr_err("%s, adaptor is not online, do not set real chg_en\n",
			__func__);
		return;
	}

	if (chg_en) {
		charger_dev_enable(info->chg1_dev, true);
		charger_manager_notifier(info,
			CHARGER_NOTIFY_START_CHARGING);
	} else {
		charger_dev_enable(info->chg1_dev, false);
		charger_manager_notifier(info,
			CHARGER_NOTIFY_STOP_CHARGING);
	}
}

int charger_manager_reset_learned_cc(struct charger_consumer *consumer, int val)
{
	/* only reset 0 or 1, 0:do nothing 1: reset */
	if ((val != 0) && (val != 1))
		return 0;

	if (val == 0) {
		gm.is_reset_aging_factor = false;
	} else {
		gm.is_reset_aging_factor = true;
		wakeup_fg_algo_cmd(FG_INTR_KERNEL_CMD,
			FG_KERNEL_CMD_RESET_AGING_FACTOR, 0);
	}
	chr_err("%s=%d\n", __func__, gm.is_reset_aging_factor);
	return 0;
}

int charger_manager_set_voltage_max(struct charger_consumer *consumer, int val)
{
	struct charger_manager *info = NULL;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s input para is null\n", __func__);
		return 0;
	}

	info = consumer->cm;
	chr_info("%s, basp_cv = %d, val = %d\n", __func__,
		info->data.basp_cv, val);
	info->data.basp_cv = val;
	_wake_up_charger(info);
	return 0;
}

int charger_manager_set_current_max(struct charger_consumer *consumer, int val)
{
	struct charger_manager *info = NULL;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s input para is null\n", __func__);
		return 0;
	}

	info = consumer->cm;
	info->data.basp_current = min(info->data.basp_current, val);
	_wake_up_charger(info);

	return 0;
}

/* hwbatt : get_property interface */
int charger_manager_get_voltage_max(struct charger_consumer *consumer)
{
	struct charger_manager *info = NULL;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s input para is null\n", __func__);
		return BATTERY_CV;
	}

	info = consumer->cm;
	return info->data.basp_cv;
}

int charger_manager_get_charge_full_design(struct charger_consumer *consumer)
{
	struct charger_manager *info = NULL;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s input para is null\n", __func__);
		return BATTERY_FULL_DESIGN;
	}

	info = consumer->cm;
	return info->data.charge_full_design;
}


int charger_manager_const_charge_current_max(struct charger_consumer *consumer)
{
	struct charger_manager *info = NULL;

	if (!consumer || !(consumer->cm)) {
		chr_err("%s input para is null\n", __func__);
		return AC_CHARGER_CURRENT;
	}

	info = consumer->cm;
	return info->data.basp_current;
}

/* dpm and dts */
static int get_dpm_para(int indx, long data)
{
	switch (indx % VDPM_PARA_TOTAL) {
	case VDPM_PARA_CAP_MIN:
		if ((data < VDPM_CBAT_MIN) || (data > VDPM_CBAT_MAX)) {
			chr_err("the vdpm_para cap_min is out of range\n");
			return -EINVAL;
		}
		g_vdpm_data[indx / VDPM_PARA_TOTAL].cap_min = data;
		break;
	case VDPM_PARA_CAP_MAX:
		if ((data < VDPM_CBAT_MIN) || (data > VDPM_CBAT_MAX)) {
			chr_err("the vdpm_para cap_max is out of range\n");
			return -EINVAL;
		}
		g_vdpm_data[indx / VDPM_PARA_TOTAL].cap_max = data;
		break;
	case VDPM_PARA_DPM:
		if ((data < VDPM_VOLT_MIN) || (data > VDPM_VOLT_MAX)) {
			chr_err("the vdpm_para vin_dpm is out of range\n");
			return -EINVAL;
		}
		g_vdpm_data[indx / VDPM_PARA_TOTAL].vin_dpm = data;
		break;
	case VDPM_PARA_CAP_BACK:
		if ((data < 0) || (data > g_vdpm_buf_limit)) {
			chr_err("the vdpm_para cap_back is out of range\n");
			return -EINVAL;
		}
		g_vdpm_data[indx / VDPM_PARA_TOTAL].cap_back = data;
		break;
	default:
		chr_err("get vdpm_para failed\n");
		return -EINVAL;
	}

	return 0;
}

static int dpm_para_parse(struct device_node *np)
{
	int ret;
	int i;
	int array_len;
	long data = 0;
	const char *chrg_data_string = NULL;

	ret = of_property_read_u32(np, "vdpm_buf_limit", &g_vdpm_buf_limit);
	if (ret) {
		chr_err("get vdpm_buf_limit failed\n");
		g_vdpm_buf_limit = VDPM_DELTA_LIMIT;
	}
	chr_info("vdpm_buf_limit = %d\n", g_vdpm_buf_limit);

	array_len = of_property_count_strings(np, "vdpm_para");
	if ((array_len <= 0) || ((array_len % VDPM_PARA_TOTAL) != 0)) {
		chr_err("vdpm_para is invaild, please check vdpm_para number\n");
		return -EINVAL;
	}
	if (array_len > (VDPM_PARA_LEVEL * VDPM_PARA_TOTAL)) {
		array_len = VDPM_PARA_LEVEL * VDPM_PARA_TOTAL;
		chr_err("vdpm_para is too long, use only front %d paras\n",
			array_len);
		return -EINVAL;
	}

	for (i = 0; i < array_len; i++) {
		ret = of_property_read_string_index(np, "vdpm_para",
			i, &chrg_data_string);
		if (ret) {
			chr_err("get vdpm_para failed\n");
			return -EINVAL;
		}
		/* 10 means decimalism */
		ret = kstrtol(chrg_data_string, 10, &data);
		if (ret)
			break;

		ret = get_dpm_para(i, data);
		if (ret)
			break;

		chr_info("vdpm_para %d:%d = %ld\n",
			(i / VDPM_PARA_TOTAL), (i % VDPM_PARA_TOTAL), data);
	}

	return ret;
}

void reset_vdpm_first_run(void)
{
	g_vdpm_first_run = true;
}

void set_vdpm_by_vol(struct charger_manager *info, int vbat)
{
	int i;
	int ret;
	int vbat_dpm = VDPM_MAX;
	static int last_i;
	static int last_vdpm;
	static int last_vbat;
	int vbus;

#ifdef CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT
	if (info->data.enable_v600_buck)
		vbat_dpm = VDPM_MAX_V600;
#endif

	if (!info || !info->chg1_dev) {
		chr_err("%s input null\n", __func__);
		return;
	}
	if ((info->chr_type == FCP_CHARGER) ||
		(info->chr_type == WIRELESS_CHARGER)) {
		vbus = battery_get_vbus();
		/* adapt vdpm when vbus lower 6500mV */
		if (vbus < 6500) {
			goto adapt_vbat_dpm;
		} else {
			vbat_dpm = FCP_VDPM;
			goto set_dpm;
		}
	}

	if ((g_dpm_enable == false) ||
		(info->chr_type == NONSTANDARD_CHARGER) ||
		(info->chr_type == STANDARD_HOST)) {
		ret = charger_dev_set_dpm(info->chg1_dev, vbat_dpm * MV_TO_UV);
		chr_err("%s enalbe:%d, SDP set vbat_dpm=%d ret=%d\n",
			__func__, g_dpm_enable, vbat_dpm * MV_TO_UV, ret);
		return;
	}

adapt_vbat_dpm:
	chr_info("vdpm_first_run = %d\n", g_vdpm_first_run);
	for (i = 0; i < VDPM_PARA_LEVEL; i++) {
		if ((vbat < g_vdpm_data[i].cap_min) ||
			(vbat >= g_vdpm_data[i].cap_max))
			continue;

		if (((g_vdpm_data[i].cap_max - vbat) >
			g_vdpm_data[i].cap_back) ||
			(((last_vbat - vbat) <= 0) && (last_i <= i)) ||
			(abs(last_i - i) > 1) || (g_vdpm_first_run == true)) {
			vbat_dpm = g_vdpm_data[i].vin_dpm;
			last_i = i;
		} else {
			vbat_dpm = last_vdpm;
		}
		break;
	}
	g_vdpm_first_run = false;
	last_vbat = vbat;
	last_vdpm = vbat_dpm;
set_dpm:
	ret = charger_dev_set_dpm(info->chg1_dev, vbat_dpm * MV_TO_UV);
	chr_info("%s vbat=%d and set vbat_dpm=%d ret=%d\n",
		__func__, vbat, vbat_dpm * MV_TO_UV, ret);
}

void charger_parse_dt(struct device_node *np, struct charger_manager *info)
{
	u32 val = 0;

	if (!np || !info)
		return;

	g_dpm_enable = of_property_read_bool(np, "dpm_enable");
	chr_err("read dpm_enable:%d\n", g_dpm_enable);
	if (g_dpm_enable) {
		if (dpm_para_parse(np)) {
			g_dpm_enable = false;
			chr_info("call dpm_para_parse failed\n");
		}
	}

	g_dynamic_cv_en = of_property_read_bool(np, "dynamic_cv_en");
	chr_info("read dynamic_cv_en:%d\n", g_dynamic_cv_en);

	g_dynamic_iterm_en = of_property_read_bool(np, "dynamic_iterm_en");
	chr_info("read dynamic_iterm_en:%d\n", g_dynamic_iterm_en);

	if (of_property_read_u32(np, "vbus_max_limit", &val) >= 0)
		g_vbus_max_limit = val;

	if (of_property_read_u32(np, "iterm", &val) >= 0) {
		g_custom_iterm = val;
	} else {
		chr_err("use default iterm:%d\n", CUSTOM_DEFAULT_ITERM);
		g_custom_iterm = CUSTOM_DEFAULT_ITERM;
	}
	chr_info("read iterm:%d\n", g_custom_iterm);

	if (of_property_read_u32(np, "charge_full_design", &val) >= 0) {
		info->data.charge_full_design = val;
	} else {
		chr_err("use default BATTERY_FULL_DESIGN:%d\n",
			BATTERY_FULL_DESIGN);
		info->data.charge_full_design = BATTERY_FULL_DESIGN;
	}
	chr_info("read charge_full_design:%d\n", info->data.charge_full_design);

	if (of_property_read_u32(np, "jeita_temp_t0_to_t1_cur", &val) >= 0) {
		info->data.jeita_temp_t0_to_t1_cur = val;
	} else {
		chr_err("use default JEITA_TEMP_DEFAULT_CURRENT_T0_T1:%d\n",
			JEITA_TEMP_DEFAULT_CURRENT_T0_T1);
		info->data.jeita_temp_t0_to_t1_cur =
			JEITA_TEMP_DEFAULT_CURRENT_T0_T1;
	}

	if (of_property_read_u32(np, "jeita_temp_t1_to_t2_cur", &val) >= 0) {
		info->data.jeita_temp_t1_to_t2_cur = val;
	} else {
		chr_err("use default JEITA_TEMP_DEFAULT_CURRENT_T1_T2:%d\n",
			JEITA_TEMP_DEFAULT_CURRENT_T1_T2);
		info->data.jeita_temp_t1_to_t2_cur =
			JEITA_TEMP_DEFAULT_CURRENT_T1_T2;
	}

	if (of_property_read_u32(np, "jeita_temp_t2_to_t3_cur", &val) >= 0) {
		info->data.jeita_temp_t2_to_t3_cur = val;
	} else {
		chr_err("use default JEITA_TEMP_DEFAULT_CURRENT_T2_T3:%d\n",
			JEITA_TEMP_DEFAULT_CURRENT_T2_T3);
		info->data.jeita_temp_t2_to_t3_cur =
			JEITA_TEMP_DEFAULT_CURRENT_T2_T3;
	}

	if (of_property_read_u32(np, "jeita_temp_t3_to_t4_cur", &val) >= 0) {
		info->data.jeita_temp_t3_to_t4_cur = val;
	} else {
		chr_err("use default JEITA_TEMP_DEFAULT_CURRENT_T3_T4:%d\n",
			JEITA_TEMP_DEFAULT_CURRENT_T3_T4);
		info->data.jeita_temp_t3_to_t4_cur =
			JEITA_TEMP_DEFAULT_CURRENT_T3_T4;
	}

	if (of_property_read_bool(np, "power_path_disable")) {
		info->data.power_path_support = false;
		chr_info("set power_path_disable\n");
	}

	if (of_property_read_bool(np, "jeita_ibus_limit_en")) {
		g_jeita_ibus_limit_en = true;
		chr_info("get jeita_ibus_limit_en:%d\n", g_jeita_ibus_limit_en);
	}

	info->data.basp_cv = info->data.battery_cv;
	info->data.basp_current = info->data.ac_charger_current;

#ifdef CONFIG_HLTHERM_RUNTEST
	info->data.temp_t0_thres = HLTHERM_LOW_TEMP;
	info->data.temp_t4_thres = HLTHERM_HIGH_TEMP;
	info->data.temp_t4_thres_minus_x_degree = HLTHERM_HIGH_TEMP_HYSTERESIS;
#endif
}

void set_jeita_ibus_limit(struct charger_manager *info)
{
	struct charger_data *pdata = NULL;

	if (!info)
		return;

	pdata = &info->chg1_data;
	if (!pdata)
		return;

	if (info->enable_sw_jeita && g_jeita_ibus_limit_en) {
		if (info->sw_jeita.sm == TEMP_T0_TO_T1) {
			pdata->input_current_limit = min(pdata->input_current_limit,
				JEITA_IBUS_T0_TO_T1);
			chr_err("set ibus_limit_by_jeita, ibus:%d, jeita_sm=%d\n",
				pdata->input_current_limit, info->sw_jeita.sm);
		}
	}
}

void set_jeita_curr(struct sw_jeita_data *sw_jeita,
	struct charger_manager *info)
{
	if (!info || !sw_jeita) {
		chr_err("input para null\n");
		return;
	}

	switch (sw_jeita->sm) {
	case TEMP_ABOVE_T4:
	case TEMP_T3_TO_T4:
		sw_jeita->cur = info->data.jeita_temp_t3_to_t4_cur;
		break;
	case TEMP_T2_TO_T3:
		sw_jeita->cur = info->data.jeita_temp_t2_to_t3_cur;
		break;
	case TEMP_T1_TO_T2:
		sw_jeita->cur = info->data.jeita_temp_t1_to_t2_cur;
		break;
	case TEMP_T0_TO_T1:
	case TEMP_BELOW_T0:
		sw_jeita->cur = info->data.jeita_temp_t0_to_t1_cur;
		break;
	default:
		break;
	}

	chr_err("SW_JEITA curr:%d\n", sw_jeita->cur);
}

void charger_event_notify(struct charger_manager *info)
{
	enum charger_type chr_type;

	if (!info)
		return;

	chr_type = mt_get_charger_type();
	chr_err("mtk_charger_int_handler %d\n", chr_type);
	if (chr_type != CHARGER_UNKNOWN)
		charger_manager_notifier(info, CHARGER_NOTIFY_START_CHARGING);
	else
		charger_manager_notifier(info, CHARGER_NOTIFY_STOP_CHARGING);
}

unsigned int get_charging_time(void)
{
	return g_charge_time;
}

void set_charging_time(unsigned int chg_time)
{
	g_charge_time = chg_time;
}

static void wake_up_otg_wdt(void)
{
	g_otg_wdt_timeout = true;
	wake_up(&g_wait_otg_wdt_que);
}

static enum hrtimer_restart otg_wdt_hrtimer_func(struct hrtimer *timer)
{
	wake_up_otg_wdt();
	return HRTIMER_NORESTART;
}

static void huawei_otg_wdt_init_timer(void)
{
	hrtimer_init(&g_otg_wdt_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_otg_wdt_timer.function = otg_wdt_hrtimer_func;
}

static void otg_start_timer(void)
{
	ktime_t ktime = ktime_set(WAIT_SECONDS, WAIT_NS);

	hrtimer_start(&g_otg_wdt_timer, ktime, HRTIMER_MODE_REL);
}

static void huawei_otg_kick_wdt(void)
{
	if (!g_chg_dev || !(g_chg_dev->ops) || !(g_chg_dev->ops->kick_wdt)) {
		chr_err("%s input null err\n", __func__);
		return;
	}
	g_chg_dev->ops->kick_wdt(g_chg_dev);
}

static int otg_wdt_thread(void *arg)
{
	while (1) {
		wait_event(g_wait_otg_wdt_que, (g_otg_wdt_timeout == true));
		g_otg_wdt_timeout = false;
		huawei_otg_kick_wdt();
		otg_start_timer();
	}
	return 0;
}

void otg_kick_wdt(struct charger_device *chg_dev, bool en)
{
	if (!chg_dev)
		return;

	g_chg_dev = chg_dev;
	if (en) {
		huawei_otg_kick_wdt();
		otg_start_timer();
	} else {
		hrtimer_cancel(&g_otg_wdt_timer);
	}
}

void huawei_kick_otg_wdt(void)
{
	huawei_otg_wdt_init_timer();
	init_waitqueue_head(&g_wait_otg_wdt_que);
	g_otg_wdt_task = kthread_run(otg_wdt_thread, NULL, "otg_wdt_thread");
	if (IS_ERR(g_otg_wdt_task))
		pr_err("create otg_wdt_thread failed\n");
}

static void huawei_set_dynamic_cv(struct charger_manager *info, u32 cv_value)
{
	int curr = 0;
	int iterm = g_custom_iterm;

	if (cv_value <= DYN_CV_THRESHOLD) {
		chr_err("not do dynamic proc, cv:%d\n", cv_value);
		goto set_cv_iterm;
	}

	curr = battery_get_bat_current() * 100; /* 100:convert mA to uA */
	if (curr > DYN_CV_IBAT_THRESHOLD) {
		cv_value += DYN_CV_STEP;
		iterm = DYN_ITERM;
	}

set_cv_iterm:
	chr_info("set dyn cv ibat:%d, cv:%d, iterm:%d\n", curr, cv_value, iterm);
	charger_dev_set_eoc_current(info->chg1_dev, iterm);
	charger_dev_set_constant_voltage(info->chg1_dev, cv_value);
}

void huawei_dyn_cv_proc(struct charger_manager *info, u32 cv_value)
{
	if (g_dynamic_cv_en)
		huawei_set_dynamic_cv(info, cv_value);
	else
		charger_dev_set_constant_voltage(info->chg1_dev, cv_value);
}

void huawei_dyn_iterm_proc(struct charger_manager *info, u32 iterm_value)
{
	if (g_dynamic_iterm_en) {
		chr_info("set dyn iterm:%d\n", iterm_value);
		charger_dev_set_eoc_current(info->chg1_dev, iterm_value);
	}
}

void huawei_eoc_re_enable_chg(struct charger_manager *info)
{
	/* 100:battery full soc = 100 */
	if ((battery_get_soc() < 100) && (g_re_enable_chg_flag == false) &&
		get_pl_charger_status()) {
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_enable(info->chg1_dev, true);
		g_re_enable_chg_flag = true;
		chr_info("battery full but soc < 100, re-enable charger\n");
	}
}
