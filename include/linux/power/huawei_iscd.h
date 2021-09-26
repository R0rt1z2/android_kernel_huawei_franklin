/*
 * huawei_iscd.h
 *
 * for iscd
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

#ifndef HUAWEI_ISCD_H
#define HUAWEI_ISCD_H

#include <linux/time.h>
#include "mtk_charger_intf.h"

#define ERROR                     1
#define SUCCESS                   0
#define ENABLED                   1
#define DISABLED                  0
#define TENTH                     10
#define SAMPLE_ZERO               0
#define SAMPLE_ONE                1
#define SAMPLE_TWO                2
#define ISCD_SMAPLE_LEN_MAX       2
#define INVALID_ISC               (-1)
#define ISCD_ACT_LOOP             30 /* 20s x 30 = 10min */
#define ISCD_TBATT_MAX            50
#define ISCD_TBATT_MIN            10
#define ISCD_CURR_VALID_MIN       0
#define ISCD_CURR_VALID_MAX       500000 /* uA */
#define ISCD_OCV_UV_MIN           2500 /* 2.5V */
#define ISCD_OCV_UV_MAX           4500 /* 4.5V */
#define ISCD_RECHARGE_CC          1000 /* uAh */
#define ISCD_TBAT_DELTA_MAX       10
#define PERSENT_TRANS             100
#define SEC_PER_HOUR              3600
#define ISCD_DSM_THRESHOLD        100000 /* uA */
#define ISCD_DSM_REPORT_CNT_MAX   2
#define ISCD_DSM_REPORT_INTERVAL  259200 /* (72h x 3600s) */
#define ISCD_DSM_LOG_SIZE_MAX     2048
#define ISCD_ERR_NO_STR_SIZE      128

struct iscd_sample_info {
	struct timespec sample_time;
	int ocv_volt_mv; /* mV */
	s64 ocv_soc_uah; /* uAh */
	s64 cc_value; /* uAh */
	int tbatt;
};

struct iscd_level_config {
	int isc_threshold;
	int dsm_err_no;
	int dsm_report_cnt;
	time_t dsm_report_time;
};

struct iscd_info {
	int enable;
	bool chg_done;
	int isc; /* uA */
	int size;
	unsigned int loop_cnt;
	struct iscd_sample_info sample_info[ISCD_SMAPLE_LEN_MAX];
	struct iscd_level_config level_config;
};

void iscd_probe(struct charger_manager *info);
void iscd_main_proc(struct charger_manager *info);
void iscd_reset_sample_info(struct iscd_info *iscd);

#endif /* HUAWEI_ISCD_H */
