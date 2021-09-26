/*
 * huawei_charge_time.h
 *
 * Calculate the remaining time of charging.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#ifndef __HUAWEI_CHARGE_TIME_H__
#define __HUAWEI_CHARGE_TIME_H__

#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>

#define CHG_TYPE_DETECT_TIME 2000

#define NSEC_PRE_SECOND (1000 * 1000 * 1000)

/* 30min */
#define STEP_TIME_MAX (30 * 60)

#define BATTERY_NAME "battery"
#define MICRO2MINI 1000
#define DURATION_PARA_MAX 5
#define RESERVED_NUM 5
#define CHG_DUR_FILE_LIMIT 0666
#define SOC_FULL 100

#define CHG_DUR_SIZE 0x4000

/* 0x19000 offset from 100K */
#define CHG_DUR_OFFSET 0

#define CHG_DUR_BUF_SHOW_LEN 128

#define MSEC_PRE_SECOND 1000

#define MAX_NUM 102

#define PARAM_SIZE 401
#define MULT_100 100
#define FULL_SOC 100
#define NORMAL_TEMP 25

#define TEMP_HIGH 50
#define TEMP_LOW 10

#define ADJUST_FCC_SOC 95
#define ADJUST_FCC_CURRENT 300
#define SEC_PRE_HOUR 3600
#define MIN_START_TH (60 * 5)
#define SOC_START_TH 40
#define CYCLE_TH 5

#define ROW 10
#define COL 10

/* Add valid flag for framework to check */
#define VALID_FLAG 0x55000000

#define CHG_TIME_PATH "/data/log/charge_time"

static char *charge_type_str_enum[] = {
	"CHG_NONE", "CHG_USB", "CHG_5V2A", "CHG_FCP", "CHG_LVC", "CHG_SC",
	"CHG_OTHERS",
};

enum charge_type_enum {
	CHG_NONE,
	CHG_USB,
	CHG_5V2A,
	CHG_FCP,
	CHG_LVC,
	CHG_SC,
	CHG_OTHERS,
};

struct huawei_soc_durations {
	int soc;
	int durations;
};

struct huawei_chg_time_param {
	int step_time[SOC_FULL + 1];
	int volt[SOC_FULL + 1];
	int start_soc;
	int batt_cycles;
	int temp_high;
	int temp_low;
	int fcc;
	int reserved[RESERVED_NUM];
};

struct huawei_chg_time_info {
	struct huawei_chg_time_param standard;
	struct huawei_chg_time_param fcp;
	struct huawei_chg_time_param lvc;
	struct huawei_chg_time_param lvc_none_standard;
	struct huawei_chg_time_param sc;
	struct huawei_chg_time_param sc_none_standard;
};

struct huawei_batt_info {
	int fcc;
	int soc;
	int volt_mv;
	int curr_ma;
	int batt_temp;
	int batt_cycles;
};

struct huawei_chg_info {
	enum charge_type_enum charge_type;
	int cc_cable_detect_ok;
	int direct_charge_type;
	bool direct_charge_done;
	int direct_charge_done_soc;
};
struct param {
	int adjust_by_current;
	int adjust_by_fcc;
	int adjust_by_temp;
	int adjust_by_volt;
};

struct huawei_chg_time_device {
	int remaining_duration;
	int remaining_duration_with_valid;
	int pre_remaining_duration_with_valid;
	int test_flag;
	int test_output_val;
	int pre_capacity;
	u64 pre_timestemp;
	int start_capacity;
	u64 start_timestemp;
	int charge_type_change;
	int work_running;
	struct huawei_chg_info chg_info;
	struct huawei_batt_info batt_info;

	struct huawei_chg_time_info flash_param;
	struct huawei_chg_time_param cur_curve;
	struct huawei_chg_time_param *ref_curve;
	struct delayed_work charge_time_work;
	struct param param_dts;
	struct notifier_block nb;
	struct notifier_block direct_charger_nb;
	struct device *dev;
};

/* for huawei charge time debug */
void huawei_chg_time_write_param_to_flash(
	struct huawei_chg_time_device *di);
int huawei_chg_time_read_param_from_flash(
	struct huawei_chg_time_device *di);
int huawei_chg_time_file_write(void *buf, u32 buf_size);

#define ct_dbg(fmt, args...) pr_info("[hw_chg_time]" fmt, ##args)
#define ct_info(fmt, args...) pr_info("[hw_chg_time]" fmt, ##args)
#define ct_warn(fmt, args...) pr_warn("[hw_chg_time]" fmt, ##args)
#define ct_err(fmt, args...) pr_err("[hw_chg_time]" fmt, ##args)

#endif
