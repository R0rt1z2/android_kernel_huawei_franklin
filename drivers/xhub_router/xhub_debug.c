/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub debug module
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <securec.h>
#include "xhub_route.h"
#include "sensor_config.h"
#include "xhub_debug.h"
#include "xhub_pm.h"
#include "xhub_recovery.h"
#include "sensor_sysfs.h"
#include "sensor_detect.h"
#include "als_channel.h"
#include "cap_prox_channel.h"
#include "ps_channel.h"
#include <log/hwlog_kernel.h>
#include "ipc_adapter.h"

#define CONTEXTHUB_SUCCESS    0
#define CONTEXTHUB_FAIL       (-1)
#define FP_EVENT_ARGS_NUM     4

static char show_str[MAX_STR_SIZE];
static struct class *iomcu_power;
iomcu_power_status i_power_status;

static DEFINE_MUTEX(mutex_pstatus);

static struct t_sensor_debug_operations_list sensor_debug_operations_list = {
	.mlock = __MUTEX_INITIALIZER(sensor_debug_operations_list.mlock),
	.head = LIST_HEAD_INIT(sensor_debug_operations_list.head),
};

static char *iomcu_app_id_str[] = {
	[TAG_STEP_COUNTER] = "TAG_STEP_COUNTER",
	[TAG_SIGNIFICANT_MOTION] = "TAG_SIGNIFICANT_MOTION",
	[TAG_STEP_DETECTOR] = "TAG_STEP_DETECTOR",
	[TAG_AR] = "TAG_ACTIVITY",
	[TAG_ORIENTATION] = "TAG_ORIENTATION",
	[TAG_LINEAR_ACCEL] = "TAG_LINEAR_ACCEL",
	[TAG_GRAVITY] = "TAG_GRAVITY",
	[TAG_ROTATION_VECTORS] = "TAG_ROTATION_VECTORS",
	[TAG_GEOMAGNETIC_RV] = "TAG_GEOMAGNETIC_RV",
	[TAG_MOTION] = "TAG_MOTION",
	[TAG_ACCEL] = "TAG_ACCEL",
	[TAG_GYRO] = "TAG_GYRO",
	[TAG_MAG] = "TAG_MAG",
	[TAG_ALS] = "TAG_ALS",
	[TAG_PS] = "TAG_PS",
	[TAG_PRESSURE] = "TAG_PRESSURE",
	[TAG_PDR] = "TAG_PDR",
	[TAG_AR] = "TAG_AR",
	[TAG_FINGERSENSE] = "TAG_FINGERSENSE",
	[TAG_PHONECALL] = "TAG_PHONECALL",
	[TAG_CONNECTIVITY] = "TAG_CONNECTIVITY",
	[TAG_MAG_UNCALIBRATED] = "TAG_MAG_UNCALIBRATED",
	[TAG_GYRO_UNCALIBRATED] = "TAG_GYRO_UNCALIBRATED",
	[TAG_HANDPRESS] = "TAG_HANDPRESS",
	[TAG_CA] = "TAG_CA",
	[TAG_OIS] = "TAG_OIS",
	[TAG_FP] = "TAG_FP",
	[TAG_CAP_PROX] = "TAG_CAP_PROX",
	[TAG_KEY] = "TAG_KEY",
	[TAG_AOD] = "TAG_AOD",
	[TAG_MAGN_BRACKET] = "TAG_MAGN_BRACKET",
	[TAG_CONNECTIVITY_AGENT] = "TAG_CONNECTIVITY_AGENT",
	[TAG_FLP] = "TAG_FLP",
	[TAG_HINGE] = "TAG_HINGE",
	[TAG_RPC] = "TAG_RPC",
	[TAG_FP_UD] = "TAG_FP_UD",
	[TAG_ACCEL_UNCALIBRATED] = "TAG_ACCEL_UNCALIBRATED",
	[TAG_DROP] = "TAG_DROP",
	[TAG_BIG_DATA] = "TAG_BIG_DATA",
	[TAG_ACC1] = "TAG_ACCEL1",
	[TAG_GYRO1] = "TAG_GYRO1",
	[TAG_ALS1] = "TAG_ALS1",
	[TAG_MAG1] = "TAG_MAG1",
	[TAG_ALS2] = "TAG_ALS2",
	[TAG_CAP_PROX1] = "TAG_CAP_PROX1",
	[TAG_HW_PRIVATE_APP_END] = "TAG_HW_PRIVATE_APP_END",
	[TAG_THERMOMETER] = "TAG_THERMOMETER",
};

/* to find tag by str */
static const struct sensor_debug_tag_map tag_map_tab[] = {
	{ "accel", TAG_ACCEL },
	{ "magnitic", TAG_MAG },
	{ "gyro", TAG_GYRO },
	{ "als_light", TAG_ALS },
	{ "ps_promixy", TAG_PS },
	{ "linear_accel", TAG_LINEAR_ACCEL },
	{ "gravity", TAG_GRAVITY },
	{ "orientation", TAG_ORIENTATION },
	{ "rotationvector", TAG_ROTATION_VECTORS },
	{ "maguncalibrated", TAG_MAG_UNCALIBRATED },
	{ "gamerv", TAG_GAME_RV },
	{ "gyrouncalibrated", TAG_GYRO_UNCALIBRATED },
	{ "significantmotion", TAG_SIGNIFICANT_MOTION },
	{ "stepdetector", TAG_STEP_DETECTOR },
	{ "stepcounter", TAG_STEP_COUNTER },
	{ "geomagnetic", TAG_GEOMAGNETIC_RV },
	{ "airpress", TAG_PRESSURE },
	{ "cap_prox", TAG_CAP_PROX },
	{ "hall", TAG_HALL },
	{ "fault", TAG_FAULT },
	{ "ar", TAG_AR },
	{ "fingersense", TAG_FINGERSENSE },
	{ "fingerprint", TAG_FP },
	{ "key", TAG_KEY },
	{ "aod", TAG_AOD },
	{ "magn_bracket", TAG_MAGN_BRACKET },
	{ "hinge", TAG_HINGE },
	{ "environment", TAG_ENVIRONMENT },
	{ "fingerprint_ud", TAG_FP_UD },
	{ "acceluncalibrated", TAG_ACCEL_UNCALIBRATED },
	{ "tof", TAG_TOF },
	{ "drop", TAG_DROP },
	{ "ext_hall", TAG_EXT_HALL },
	{ "accel1", TAG_ACC1 },
	{ "gyro1", TAG_GYRO1 },
	{ "als1", TAG_ALS1 },
	{ "magnitic1", TAG_MAG1 },
	{ "als2", TAG_ALS2 },
	{ "cap_prox1", TAG_CAP_PROX1 },
	{ "thermometer", TAG_THERMOMETER },
};

static const char * const fault_type_table[] = {
	"hardfault",
	"busfault",
	"memfault",
	"usagefault",
	"rdrdump",
};

static int open_sensor(int tag, int argv[], int argc)
{
	int ret;
	interval_param_t delay_param = {
		.period = argv[0],
		.batch_count = 1,
		.mode = AUTO_MODE,
		.reserved[0] = TYPE_STANDARD /* for step counter only */
	};

	if (tag == -1)
		return -1;

	if (ap_sensor_enable(tag, true))
		return 0;

	hwlog_info("open sensor %d\n", tag);
	if (tag == TAG_STEP_COUNTER)
		ret = xhub_sensor_enable_stepcounter(true, TYPE_STANDARD);
	else
		ret = xhub_sensor_enable(tag, true);
	if (!ret && (argc > 0)) {
		hwlog_info("set sensor %d delay %d ms\n", tag, argv[0]);
		ret = xhub_sensor_setdelay(tag, &delay_param);
	}

	return ret;
}

static int set_delay(int tag, int argv[], int argc)
{
	interval_param_t delay_param = {
		.period = argv[0],
		.batch_count = 1,
		.mode = AUTO_MODE,
		.reserved[0] = TYPE_STANDARD /* for step counter only */
	};

	if (tag == -1 || argc == 0)
		return -1;

	if (ap_sensor_setdelay(tag, argv[0]))
		return 0;

	hwlog_info("set sensor %d delay %d ms\n", tag, argv[0]);
	xhub_sensor_setdelay(tag, &delay_param);
	return 0;
}

static int close_sensor(int tag, int argv[], int argc)
{
	if (tag == -1)
		return -1;

	if (ap_sensor_enable(tag, false))
		return 0;

	hwlog_info("close sensor %d\n", tag);
	if (tag == TAG_STEP_COUNTER)
		xhub_sensor_enable_stepcounter(false, TYPE_STANDARD);
	else
		xhub_sensor_enable(tag, false);

	return 0;
}

/*
 * This funciton is Only for Test:
 * I2C address should be modified by virtual address.
 * This virtual address is for Test.
 * And Tools out can put data into system for simulating
 */
static int set_sensor_slave_addr(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	read_info_t pkg_mcu;
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	unsigned int i2c_address;
	int ret;

	if (argc == 0)
		return -1;

	i2c_address = (unsigned int)argv[0] & 0xFF;
	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&pkg_mcu, 0, sizeof(pkg_mcu));

	pkg_ap.tag = tag;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	cpkt.subcmd = SUB_CMD_SET_SLAVE_ADDR_REQ;
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = sizeof(i2c_address) + SUBCMD_LEN;
	memcpy(cpkt.para, &i2c_address, sizeof(i2c_address));

	hwlog_info("%s, %s, i2c_addr:0x%x\n", __func__,
		obj_tag_str[tag], i2c_address);

	ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);
	if (ret != 0) {
		hwlog_err("set %s slave addr failed, ret = %d in %s\n",
			obj_tag_str[tag], ret, __func__);
		return -1;
	}
	if (pkg_mcu.errno != 0)
		hwlog_err("set %s slave addr failed errno = %d in %s\n",
			obj_tag_str[tag], pkg_mcu.errno, __func__);
	else
		hwlog_info("set %s new slave addr:0x%x success\n",
			obj_tag_str[tag], i2c_address);
	return 0;
}

#define SOFTIRON_ARGS_NUM	9

static int set_sensor_softiron(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	int gyro_args_to_mcu[SOFTIRON_ARGS_NUM*2] = {0};
	int ret;
	int i;

	hwlog_info("xhub_dbg set_sensor_paramet() argc = %d\n", argc);
	if (argc != SOFTIRON_ARGS_NUM)
		return -1;

	for (i = 0; i < SOFTIRON_ARGS_NUM; i++) {
		hwlog_info("%d", argv[i]);
		gyro_args_to_mcu[i * 2] =
			(unsigned char)(((unsigned short int)argv[i]) &
				0x000000FF);
		gyro_args_to_mcu[i * 2 + 1] =
			(unsigned char)((((unsigned short int)argv[i]) &
				0x0000FF00) >> 8);
		hwlog_info("%d %d",
			gyro_args_to_mcu[i * 2], gyro_args_to_mcu[i * 2 + 1]);
	}

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&cpkt, 0, sizeof(cpkt));

	pkg_ap.tag = tag;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	cpkt.subcmd = SUB_CMD_ADDITIONAL_INFO;
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = sizeof(int) *
		SOFTIRON_ARGS_NUM * 2 + SUBCMD_LEN;
	memcpy(cpkt.para, gyro_args_to_mcu,
		sizeof(int) * SOFTIRON_ARGS_NUM * 2);

	ret = write_customize_cmd(&pkg_ap, NULL, false);
	if (ret != 0) {
		hwlog_err("set %s sensor_softiron failed, ret = %d in %s\n",
			obj_tag_str[tag], ret, __func__);
		return -1;
	}
	return 0;
}

static void data_prepare(write_info_t *pkg_ap, pkt_parameter_req_t *req_pkg,
	int tag, const int *data, int data_size)
{
	req_pkg->subcmd = SUB_CMD_FINGERPRINT_CONFIG_DEBUG_EVENT_REQ;
	// para len is 128 * sizeof(char)
	if (memcpy_s((void *)(req_pkg->para), sizeof(req_pkg->para),
		(void *)data, data_size) != EOK) {
		hwlog_err("%s:memcpy fail\n", __func__);
		return;
	}

	pkg_ap->tag = tag;
	pkg_ap->cmd = CMD_CMN_CONFIG_REQ;
	// exclude head, get address of valid data
	pkg_ap->wr_buf = ((pkt_header_t *)req_pkg) + 1;
	pkg_ap->wr_len = SUBCMD_LEN + data_size;
}

static int set_fp_event(int tag, int data[], int data_count)
{
	write_info_t pkg_ap;
	pkt_parameter_req_t req_pkg;
	int ret;

	if (data_count != FP_EVENT_ARGS_NUM) {
		hwlog_err("xhub_dbg data_count = %d\n",
			data_count);
		return CONTEXTHUB_FAIL;
	}

	memset((void *)&pkg_ap, 0, sizeof(pkg_ap));
	memset((void *)&req_pkg, 0, sizeof(req_pkg));
	data_prepare(&pkg_ap, &req_pkg, tag, data,
		sizeof(data[0]) * data_count);
	ret = write_customize_cmd(&pkg_ap, NULL, false);
	if (ret != CONTEXTHUB_SUCCESS) {
		hwlog_err("set FP_EVENT failed, ret = %d in %s\n",
			ret, __func__);
		return CONTEXTHUB_FAIL;
	}

	return CONTEXTHUB_SUCCESS;
}
static int set_sensor_data_mode(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	int ret;

	if (argc != 1)
		return -1;

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&cpkt, 0, sizeof(cpkt));
	pkg_ap.tag = tag;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	cpkt.subcmd = SUB_CMD_SET_DATA_MODE;
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = sizeof(int)*SOFTIRON_ARGS_NUM*2+SUBCMD_LEN;
	cpkt.para[0] = (int)argv[0];

	ret = write_customize_cmd(&pkg_ap, NULL, false);
	if (ret != 0) {
		hwlog_err("set %s sensor_mode failed, ret = %d in %s\n",
			obj_tag_str[tag], ret, __func__);
		return -1;
	}
	return 0;
}

int set_log_level(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	uint32_t log_level;
	int ret;

	if (argc != 1)
		return -1;

	log_level = argv[0];

	if (log_level > DEBUG_LEVEL)
		return -1;

	memset(&pkg_ap, 0, sizeof(pkg_ap));

	pkg_ap.tag = TAG_SYS;
	pkg_ap.cmd = CMD_SYS_LOG_LEVEL_REQ;
	pkg_ap.wr_buf = &log_level;
	pkg_ap.wr_len = sizeof(log_level);

	ret = write_customize_cmd(&pkg_ap, NULL, true);
	if (ret != 0) {
		hwlog_err("%s faile to write cmd\n", __func__);
		return -1;
	}

	g_config_on_ddr->log_level = log_level;
	hwlog_info("%s set log level %d success\n", __func__, log_level);
	return 0;
}

static int set_fault_type(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	uint8_t fault_type;
	int ret;

	if (argc == 0)
		return -1;

	fault_type = (uint8_t)argv[0] & 0xFF;
	if (fault_type >=
		(sizeof(fault_type_table) /
		sizeof(fault_type_table[0]))) {
		hwlog_err("unsupported fault_type %d\n", fault_type);
		return -1;
	}

	memset(&pkg_ap, 0, sizeof(pkg_ap));

	pkg_ap.tag = TAG_FAULT;
	pkg_ap.cmd = CMD_SET_FAULT_TYPE_REQ;
	pkg_ap.wr_buf = &fault_type;
	pkg_ap.wr_len = sizeof(fault_type);

	hwlog_info("%s, %s, fault type:%s\n",
		__func__, obj_tag_str[TAG_FAULT],
		fault_type_table[fault_type]);
	ret = write_customize_cmd(&pkg_ap, NULL, true);
	if (ret != 0) {
		hwlog_err("set fault type %s failed, ret = %d in %s\n",
			fault_type_table[fault_type], ret, __func__);
		return -1;
	}
	hwlog_info("set fault type %s success\n",
		fault_type_table[fault_type]);
	return 0;
}

static int set_fault_addr(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	int ret;
	pkt_fault_addr_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;

	if (argc == 0)
		return -1;

	memset(&pkg_ap, 0, sizeof(pkg_ap));

	pkg_ap.tag = TAG_FAULT;
	pkg_ap.cmd = CMD_SET_FAULT_ADDR_REQ;
	cpkt.wr = (unsigned int)argv[0] & 0xFF;
	cpkt.fault_addr = argv[1];
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = 5;

	ret = write_customize_cmd(&pkg_ap, NULL, true);
	if (ret != 0) {
		hwlog_err("set fault addr, read/write: %d, 0x%x failed, ret = %d in %s\n",
			argv[0], argv[1], ret, __func__);
		return -1;
	}
	hwlog_info("set fault addr,  read/write: %d, fault addr: 0x%x success\n",
		argv[0], argv[1]);
	return 0;
}

static int therm_test(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	read_info_t pkg_mcu;
	therm_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	int ret;

	if (argc == 0)
		return -1;

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&pkg_mcu, 0, sizeof(pkg_mcu));

	pkg_ap.tag = TAG_THERMOMETER;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	if (argv[0] == 1) {
		cpkt.sub_cmd = SUB_CMD_THERM_SET_CALIBRATE_THRESHHOLD;
		cpkt.para[0] = argv[1];
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else if (argv[0] == 2) {
		cpkt.sub_cmd = SUB_CMD_THERM_START_MEASURE;
		cpkt.para[0] = argv[1];
		cpkt.para[1] = argv[2];
		cpkt.para[2] = argv[3];
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else if (argv[0] == 3) {
		cpkt.sub_cmd = SUB_CMD_SELFCALI_REQ;
		cpkt.para[0] = argv[1];
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else if (argv[0] == 4) {
		cpkt.sub_cmd = SUB_CMD_SELFTEST_REQ;
		cpkt.para[0] = 2;
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else if (argv[0] == 5) {
		cpkt.sub_cmd = SUB_CMD_THERM_WRITE_HAHB;
		// argv 1 mean high 16bit of para, argv 2 mean low 16bit of para
		cpkt.para[0] = (((unsigned int)argv[1] & 0x0000ffff) << 16) |
			((unsigned int)argv[2] & 0x0000ffff);
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else if (argv[0] == 6) {
		cpkt.sub_cmd = SUB_CMD_THERM_BLACKBODY_MEASURE;
		cpkt.para[0] = 0;
		pkg_ap.wr_len = sizeof(cpkt) - sizeof(cpkt.hd);
	} else {
		hwlog_err("error cmd %d\n", argv[0]);
		return -1;
	}
	hwlog_info("%s %d mode %d\n",
		__func__, cpkt.sub_cmd, cpkt.para[0]);
	pkg_ap.wr_buf = &hd[1];
	ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);
	if (ret != 0) {
		hwlog_err("%s failed, cmd is %d, ret = %d\n",
			__func__, argv[0], ret);
		return -1;
	}
	if (pkg_mcu.errno != 0) {
		hwlog_err("%s cmd is %d, errno = %d\n",
			__func__, argv[0], pkg_mcu.errno);
		return -1;
	}
	hwlog_info("%s is %d success\n",
		__func__, argv[0]);
	return 0;
}

static int activity_common_handle(const int argv[], write_info_t *pkg_ap,
	ar_start_cmd_t *ar_start, ar_stop_cmd_t *ar_stop,
	unsigned long long activity)
{
	ar_config_event_t *activity_list = NULL;
	unsigned int start_len = 0;
	unsigned int i;
	unsigned char cmd = (unsigned int)argv[0] & 0xff;
	unsigned char core = ((unsigned int)argv[0] & 0xff00) >> 8;
	unsigned int interval = ((unsigned int)argv[0] & 0xff0000) >> 16;

	if ((cmd == CMD_CMN_OPEN_REQ) || (cmd == CMD_CMN_CLOSE_REQ)) {
		pkg_ap->cmd = cmd;
		pkg_ap->wr_buf = NULL;
		pkg_ap->wr_len = 0;
	} else if ((cmd == SUB_CMD_FLP_AR_START_REQ) ||
			(cmd == SUB_CMD_FLP_AR_UPDATE_REQ)) {
		pkg_ap->cmd = CMD_CMN_CONFIG_REQ;
		ar_start = kzalloc(sizeof(ar_start_cmd_t) +
			(AR_UNKNOWN + 1) *
			(sizeof(ar_config_event_t) +
			sizeof(char)), GFP_KERNEL);
		if (!ar_start) {
			hwlog_info("ar_start kzalloc failed, in %s\n",
				__func__);
			return -1;
		}
		ar_start->core_cmd.sub_cmd = cmd;
		ar_start->core_cmd.core = core;

		for (i = 0; i < AR_UNKNOWN; i++) {
			if (activity & (1ULL << i)) {
				activity_list =
					kzalloc(sizeof(ar_config_event_t),
					GFP_KERNEL);
				if (!activity_list) {
					hwlog_info("activity_list kzalloc failed, in %s\n", __func__);
					break;
				}
				activity_list->activity = i;
				activity_list->report_interval =
					(interval > UINT_MAX / 1000 ?
					UINT_MAX : interval * 1000);
				activity_list->event_type = EVENT_BOTH;
				memcpy((char *)ar_start +
					sizeof(ar_start_cmd_t) +
					start_len, activity_list,
					sizeof(ar_config_event_t));
				start_len += sizeof(ar_config_event_t);
				ar_start->start_param.num++;
				kfree(activity_list);
			}
		}
		hwlog_info("receive activity num %d in %s\n",
			ar_start->start_param.num, __func__);
		pkg_ap->wr_buf = ar_start;
		pkg_ap->wr_len = (int)(sizeof(ar_start_cmd_t) +
			ar_start->start_param.num *
			sizeof(ar_config_event_t));
	} else {
		pkg_ap->cmd = CMD_CMN_CONFIG_REQ;
		ar_stop->core_cmd.sub_cmd = cmd;
		ar_stop->core_cmd.core = core;
		if (interval > UINT_MAX / 1000)
			ar_stop->para = UINT_MAX;
		else
			ar_stop->para = interval * 1000;
		pkg_ap->wr_buf = ar_stop;
		pkg_ap->wr_len = sizeof(*ar_stop);
	}
	hwlog_info("set mcucmd=%d core=%d interval=%d activity=0x%llx\n",
		cmd, core, interval, activity);
	return 0; /* handle succ */
}

static int activity_common_test(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	read_info_t pkg_mcu;
	ar_start_cmd_t *ar_start = NULL;
	ar_stop_cmd_t ar_stop;
	unsigned long long activity = 0;
	int ret = -1;

	if (argc < 2) {
		hwlog_err("argc less 2\n");
		return ret;
	}
	if (argc == 3)
		activity = (unsigned long long)argv[2] & 0xffffffff;
	activity = (activity << 32) |
		((unsigned long long)argv[1] & 0xffffffff);

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&pkg_mcu, 0, sizeof(pkg_mcu));
	memset(&ar_stop, 0, sizeof(ar_stop));

	pkg_ap.tag = tag;

	ret = activity_common_handle(argv,
		&pkg_ap, ar_start, &ar_stop, activity);
	if (ret != 0)
		return ret;
	ret = write_customize_cmd(&pkg_ap,
		&pkg_mcu, true);
	kfree(ar_start);
	hwlog_info("set mcu cmd errno=%d tag=%d\n",
		pkg_mcu.errno, tag);
	return ret;
}

/*
 * ar_test para1 para2 para3
 * cmd = para1 & 0xff, 0x1: enable 0x3:close 0x21:start 0x23:stop 0x27:flush
 * core = (para1 & 0xff00) >> 8, 0:ap 1:modem
 * interval = (para1 & 0xff0000) >> 16, 0x21 report interval, others ignore
 * para2 and para3: every bit mean a activity status, eg:0x3 is VEHICLE, RIDING
 */
static int ar_test(int tag, int argv[], int argc)
{
	int ret;

	ret = activity_common_test(TAG_AR, argv, argc);
	if (ret)
		hwlog_err("%s send cmd to mcu fail,ret=%d\n", __func__, ret);
	return ret;
}

static int set_ps_type(int tag, int argv[], int argc)
{
	int ret;
	unsigned int type = argv[0];
	int32_t err_no = 0;

	hwlog_info("%s: data type  is %d(0.nor_data1.raw_data)\n",
		__func__, argv[0]);
	if (type > SET_PS_TYPE_NUMB_MAX) {
		hwlog_err("%s:set data type is fail, invalid val\n", __func__);
		return -1;
	}

	ret = send_subcmd_data_to_mcu_lock(TAG_PS, SUB_CMD_SET_DATA_TYPE_REQ,
		(const void *)&type, sizeof(type), &err_no);
	if (err_no != 0)
		hwlog_err("%s fail,err_no=%d\n", __func__, err_no);
	return ret;
}

static int set_als_type(int tag, int argv[], int argc)
{
	int ret;
	unsigned int type = argv[0];
	int32_t err_no = 0;

	hwlog_info("%s: data type  is %d(0.nor_data1.raw_data)\n",
		__func__, argv[0]);
	if (type > SET_ALS_TYPE_NUMB_MAX) {
		hwlog_err("%s:set data type is fail, invalid val\n", __func__);
		return -1;
	}

	ret = send_subcmd_data_to_mcu_lock(tag, SUB_CMD_SET_DATA_TYPE_REQ,
		(const void *)&type, sizeof(type), &err_no);
	if (err_no != 0)
		hwlog_err("%s fail,err_no=%d\n", __func__, err_no);
	return ret;
}

static int als_param_write(int tag, int argv[], int argc)
{
	s16 als_para[30]; /* bh is 25 ,apds is 21, tmd is 29 */
	int ret;
	int i;
	struct als_platform_data *pf_data = NULL;

	pf_data = als_get_platform_data(tag);
	if (!pf_data) {
		hwlog_err("%s:tag %d is invalid\n", __func__, tag);
		return -1;
	}

	memset(&als_para, 0, sizeof(als_para));
	/* argv[0] is param_num */
	if (argv[0] > 29) {
		hwlog_err("%s:param_num %d is invalid\n", __func__, argv[0]);
		return -1;
	}
	for (i = 0; i < argv[0]; i++) {
		als_para[i] = argv[i + 1];
		hwlog_info("als_para[%d] is %d\n", i,  als_para[i]);
	}
	for (i = 0; i < argv[0]; i++) {
		if (als_para[i] > MAX_SINGNED_SHORT ||
			als_para[i] < MIN_SINGNED_SHORT) {
			hwlog_err("%s: als param data is invalid\n", __func__);
			return -1;
		}
	}
	if (memcpy_s(pf_data->als_extend_data, sizeof(pf_data->als_extend_data),
		als_para, sizeof(s16) * argv[0] >
		SENSOR_PLATFORM_EXTEND_ALS_DATA_SIZE ?
		SENSOR_PLATFORM_EXTEND_ALS_DATA_SIZE :
		sizeof(s16) * argv[0]) != EOK)
		return -1;
	ret = send_subcmd_data_to_mcu(tag, SUB_CMD_SET_PARAMET_REQ,
		(const void *)pf_data, sizeof(struct als_platform_data), NULL);
	return ret;
}

static int ps_param_write(int tag, int argv[], int argc)
{
	struct ps_platform_data *pf_data = NULL;
	int ret = -1;

	pf_data = ps_get_platform_data(TAG_PS);
	if (pf_data == NULL)
		return -1;

	if (argc < 3) {
		hwlog_err("%s argc less 3\n", __func__);
		return ret;
	}

	/* argv[0] pwindows_value,
	 * argv[1] pwave_value,
	 * argv[2] threshold_value
	 */
	if (argv[0] < 0 || argv[1] < 0 || argv[2] < 0 ||
		argv[0] > MAX_SINGNED_SHORT ||
		argv[1] > MAX_SINGNED_SHORT ||
		argv[2] > MAX_SINGNED_SHORT) {
		hwlog_err("%s is fail %d %d %d\n",
			__func__, argv[0], argv[1], argv[2]);
		return ret;
	}

	/* argv[0] pwindows_value,
	 * argv[1] pwave_value,
	 * argv[2] threshold_value
	 */
	pf_data->pwindows_value = argv[0];
	pf_data->pwave_value = argv[1];
	pf_data->threshold_value = argv[2];
	ret = send_subcmd_data_to_mcu(TAG_PS, SUB_CMD_SET_PARAMET_REQ,
		(const void *)pf_data, sizeof(struct ps_platform_data), NULL);
	return ret;
}

static int sar_param_set(int tag, int argv[], int argc)
{
	write_info_t pkg_ap;
	read_info_t pkg_mcu;
	pkt_parameter_req_t spkt;
	pkt_header_t *shd = (pkt_header_t *)&spkt;
	int ret;

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&pkg_mcu, 0, sizeof(pkg_mcu));
	memset(&spkt, 0, sizeof(spkt));

	pkg_ap.tag = TAG_CAP_PROX;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	pkg_ap.wr_buf = &shd[1];
	pkg_ap.wr_len = sizeof(struct sar_platform_data) + SUBCMD_LEN;

	hwlog_info("%s g_xhub_state = %d,tag =%d ,cmd =%d\n",
		__func__, g_xhub_state, pkg_ap.tag, pkg_ap.cmd);

	if (g_xhub_state == XHUB_ST_RECOVERY || iom3_power_state == ST_SLEEP)
		ret = write_customize_cmd(&pkg_ap, NULL, false);
	else
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);

	if (ret) {
		hwlog_err("send tag %d cfg data to mcu fail, ret=%d\n", pkg_ap.tag, ret);
	} else {
		if (pkg_mcu.errno != 0)
			hwlog_err("send sar param to mcu fail\n");
		else
			hwlog_info("send sar param to mcu succes\n");
	}
	return ret;
}

static int sar_param_write(int tag, int argv[], int argc)
{
	struct sar_platform_data *pf_data = NULL;
	int ret;
	int i;

	pf_data = cap_prox_get_platform_data(TAG_CAP_PROX);
	if (pf_data == NULL)
		return -1;
	if (strncmp(sensor_chip_info[CAP_PROX], "huawei,semtech-sx9323",
		strlen("huawei,semtech-sx9323"))) {
		hwlog_err("%s: This sar does not	 support the operation\n", __func__);
		return -1;
	}
	if (argv[0] == SAR_SET_REGISTER) {
		if ((argc - 1) > SEMTECH_SAR_INIT_REG_VAL_LENGTH) {
			hwlog_err("%s:data size is larger than init_reg_val array\n", __func__);
			return -1;
		}
		for (i = 1; i < argc; i++)
			pf_data->sar_datas.semteck_data.init_reg_val[i - 1] = argv[i];
	} else if (argv[0] == SAR_SET_THRESHOLD) {
		if ((argc - 1) > SEMTECH_SAR_THRESHOLD_TO_MODEM_LENGTH) {
			hwlog_err("%s:data size is larger than threshold_to_modem array\n",
				__func__);
			return -1;
		}
		for (i = 1; i < argc; i++)
			pf_data->sar_datas.semteck_data.threshold_to_modem[i - 1] = argv[i];
	} else if (argv[0] == SAR_SET_THRESHOLD_AND_REGISTER) {
		if ((argc - 1) > SEMTECH_SAR_INIT_REG_VAL_LENGTH +
			SEMTECH_SAR_THRESHOLD_TO_MODEM_LENGTH) {
			hwlog_err("%s: The input data size too larged\n", __func__);
			return -1;
		}
		for (i = 1; i < SEMTECH_SAR_THRESHOLD_TO_MODEM_LENGTH + 1; i++)
			pf_data->sar_datas.semteck_data.threshold_to_modem[i - 1] = argv[i];
		for (i = SEMTECH_SAR_THRESHOLD_TO_MODEM_LENGTH + 1; i < argc; i++)
			pf_data->sar_datas.semteck_data.init_reg_val[i - 1 -
			SEMTECH_SAR_THRESHOLD_TO_MODEM_LENGTH] = argv[i];
	}
	ret = send_subcmd_data_to_mcu(TAG_CAP_PROX,
		SUB_CMD_SET_PARAMET_REQ,
		(const void *)pf_data, sizeof(struct sar_platform_data), NULL);
	return ret;
}

static int change_sar_mode(int tag, int argv[], int argc)
{
	int ret;

	if (!strncmp(sensor_chip_info[CAP_PROX], "huawei,semtech-sx9323",
		strlen("huawei,semtech-sx9323"))) {
		if (argc != 1 || (argv[0] != SAR_DEBUG_MODE &&
			argv[0] != SAR_NORMAL_MODE)) {
			hwlog_err("%s: Input incorrect mode\n", __func__);
			return -1;
		}
	} else if (!strncmp(sensor_chip_info[CAP_PROX], "huawei,abov-a96t3x6",
		strlen("huawei,abov-a96t3x6"))) {
		if ((argc != 1) || ((argv[0] != SAR_DEBUG_MODE) &&
			(argv[0] != SAR_NORMAL_MODE))) {
			hwlog_err("%s: Input incorrect mode.\n", __func__);
			return -1;
		}
	} else {
		hwlog_err("%s: This sar does not support the operation\n",
			__func__);
		return -1;
	}

	ret = send_subcmd_data_to_mcu(TAG_CAP_PROX, SUB_CMD_SET_DATA_MODE,
		(const void *)&argv[0], sizeof(argv[0]), NULL);
	return ret;
}

static int register_xhub_debug_operation(const char *func_name,
	sensor_debug_pfunc op)
{
	struct sensor_debug_cmd *node = NULL, *n = NULL;
	int ret = 0;

	if (!func_name || !op) {
		hwlog_err("error in %s\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&sensor_debug_operations_list.mlock);
	list_for_each_entry_safe(node, n,
		&sensor_debug_operations_list.head, entry) {
		if (op == node->operation) {
			hwlog_warn("%s has already registed in %s\n!",
				func_name, __func__);
			goto out; /* return when already registed */
		}
	}
	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node) {
		ret = -ENOMEM;
		goto out;
	}
	node->str = func_name;
	node->operation = op;
	list_add_tail(&node->entry, &sensor_debug_operations_list.head);
out:
	mutex_unlock(&sensor_debug_operations_list.mlock);
	return ret;
}

static int unregister_xhub_debug_operation(sensor_debug_pfunc op)
{
	struct sensor_debug_cmd *pos = NULL, *n = NULL;

	if (!op) {
		hwlog_err("error in %s\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&sensor_debug_operations_list.mlock);
	list_for_each_entry_safe(pos, n,
		&sensor_debug_operations_list.head, entry) {
		if (op == pos->operation) {
			list_del(&pos->entry);
			kfree(pos);
			break;
		}
	}
	mutex_unlock(&sensor_debug_operations_list.mlock);
	return 0;
}

static void register_my_debug_operations(void)
{
	register_xhub_debug_op(open_sensor);
	register_xhub_debug_op(set_delay);
	register_xhub_debug_op(close_sensor);
	register_xhub_debug_op(set_sensor_slave_addr);
	register_xhub_debug_op(set_sensor_softiron);
	register_xhub_debug_op(set_sensor_data_mode);
	register_xhub_debug_op(set_fault_type);
	register_xhub_debug_op(set_fault_addr);
	register_xhub_debug_op(set_log_level);
	register_xhub_debug_op(ar_test);
	register_xhub_debug_op(set_als_type);
	register_xhub_debug_op(als_param_write);
	register_xhub_debug_op(set_ps_type);
	register_xhub_debug_op(ps_param_write);
	register_xhub_debug_op(sar_param_write);
	register_xhub_debug_op(change_sar_mode);
	register_xhub_debug_op(sar_param_set);
	register_xhub_debug_op(set_fp_event);
	register_xhub_debug_op(therm_test);
}

static void unregister_my_debug_operations(void)
{
	unregister_xhub_debug_operation(open_sensor);
	unregister_xhub_debug_operation(set_delay);
	unregister_xhub_debug_operation(close_sensor);
	unregister_xhub_debug_operation(set_sensor_slave_addr);
	unregister_xhub_debug_operation(set_sensor_softiron);
	unregister_xhub_debug_operation(set_sensor_data_mode);
	unregister_xhub_debug_operation(set_fault_type);
	unregister_xhub_debug_operation(set_fault_addr);
	unregister_xhub_debug_operation(set_log_level);
	unregister_xhub_debug_operation(ar_test);
	unregister_xhub_debug_operation(set_als_type);
	unregister_xhub_debug_operation(als_param_write);
	unregister_xhub_debug_operation(set_ps_type);
	unregister_xhub_debug_operation(ps_param_write);
	unregister_xhub_debug_operation(sar_param_write);
	unregister_xhub_debug_operation(change_sar_mode);
	unregister_xhub_debug_operation(sar_param_set);
	unregister_xhub_debug_operation(set_fp_event);
	unregister_xhub_debug_operation(therm_test);
}

static inline bool is_space_ch(char ch)
{
	return (ch == ' ') || (ch == '\t');
}

static bool end_of_string(char ch)
{
	bool ret = false;

	switch (ch) {
	case '\0':
	case '\r':
	case '\n':
		ret = true;
		break;
	default:
		ret = false;
		break;
	}

	return ret;
}

/* find first pos */
const char *get_str_begin(const char *cmd_buf)
{
	if (!cmd_buf)
		return NULL;

	while (is_space_ch(*cmd_buf))
		++cmd_buf;

	if (end_of_string(*cmd_buf))
		return NULL;

	return cmd_buf;
}

/* find last pos */
const char *get_str_end(const char *cmd_buf)
{
	if (!cmd_buf)
		return NULL;

	while (!is_space_ch(*cmd_buf) && !end_of_string(*cmd_buf))
		++cmd_buf;

	return cmd_buf;
}

/* fuzzy matching */
bool str_fuzzy_match(const char *cmd_buf, const char *target)
{
	if (!cmd_buf || !target)
		return false;

	for (; !is_space_ch(*cmd_buf) && !end_of_string(*cmd_buf) && *target;
		++target) {
		if (*cmd_buf == *target)
			++cmd_buf;
	}

	return is_space_ch(*cmd_buf) || end_of_string(*cmd_buf);
}

static sensor_debug_pfunc get_operation(const char *str)
{
	sensor_debug_pfunc op = NULL;
	struct sensor_debug_cmd *node = NULL, *n = NULL;

	mutex_lock(&sensor_debug_operations_list.mlock);
	list_for_each_entry_safe(node, n,
		&sensor_debug_operations_list.head, entry) {
		if (str_fuzzy_match(str, node->str)) {
			op = node->operation;
			break;
		}
	}
	mutex_unlock(&sensor_debug_operations_list.mlock);
	return op;
}

static int get_sensor_tag(const char *str)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tag_map_tab); ++i) {
		if (str_fuzzy_match(str, tag_map_tab[i].str))
			return tag_map_tab[i].tag;
	}
	return -1;
}

#define ch_is_digit(ch) ('0' <= (ch) && (ch) <= '9')
#define ch_is_hex(ch) ((('A' <= (ch)) && ((ch) <= 'F')) || (('a' <= (ch)) && ((ch) <= 'f')))
#define ch_is_hexdigit(ch) (ch_is_digit(ch) || ch_is_hex(ch))
bool get_arg(const char *str, int *arg)
{
	unsigned int val = 0;
	bool neg = false;
	bool hex = false;

	if (*str == '-') {
		++str;
		neg = true;
	}

	if ((*str == '0') && ((*(str + 1) == 'x') || (*(str + 1) == 'X'))) {
		str += 2;
		hex = true;
	}

	if (hex) {
		for (; !is_space_ch(*str) && !end_of_string(*str); ++str) {
			if (!ch_is_hexdigit(*str))
				return false;
			val <<= 4;
			val |= (ch_is_digit(*str) ?
				(*str - '0') : (((*str | 0x20) - 'a') + 10));
		}
	} else {
		for (; !is_space_ch(*str) && !end_of_string(*str); ++str) {
			if (!ch_is_digit(*str))
				return false;
			val *= 10;
			val += *str - '0';
		}
	}

	*arg = neg ? -val : val;
	return true;
}

static void parse_str(const char *cmd_buf)
{
	sensor_debug_pfunc operation = NULL;
	int tag = -1;
	int arg = -1;
	int argv[MAX_CMD_BUF_ARGC] = { 0 };
	int argc = 0;

	for (; (cmd_buf = get_str_begin(cmd_buf)) != NULL;
		cmd_buf = get_str_end(cmd_buf)) {
		if (!operation)
			operation = get_operation(cmd_buf);

		if (tag == -1)
			tag = get_sensor_tag(cmd_buf);

		if (get_arg(cmd_buf, &arg)) {
			if (argc < MAX_CMD_BUF_ARGC)
				argv[argc++] = arg;
			else
				hwlog_err("too many args, %d will be ignored\n", arg);
		}
	}

	if (operation != NULL)
		operation(tag, argv, argc);
}

static ssize_t cls_attr_debug_show_func(struct class *cls,
	struct class_attribute *attr, char *buf)
{
	int i;
	unsigned int offset = 0;
	struct sensor_debug_cmd *node = NULL, *n = NULL;

	offset += sprintf(buf + offset,
		"operations format:\necho operation+tag+optarg > %s\n",
		attr->attr.name);
	offset += sprintf(buf + offset,
		"for example:\nto open accel we input: echo open_sensor accel > %s\n",
		attr->attr.name);
	offset += sprintf(buf + offset,
		"to setdelay accel 100 ms we input: echo set_delay accel 100 > %s\n",
		attr->attr.name);

	offset += sprintf(buf + offset, "\noperations supported as follow:\n");
	mutex_lock(&sensor_debug_operations_list.mlock);
	list_for_each_entry_safe(node, n,
		&sensor_debug_operations_list.head, entry) {
		offset += sprintf(buf + offset, "%s\n", node->str);
	}
	mutex_unlock(&sensor_debug_operations_list.mlock);

	offset += sprintf(buf + offset, "\ntags supported as follow:\n");
	for (i = 0; i < ARRAY_SIZE(tag_map_tab); ++i)
		offset += sprintf(buf + offset, "%s\n", tag_map_tab[i].str);

	return offset;
}

static ssize_t cls_attr_debug_store_func(struct class *cls,
	struct class_attribute *attr, const char *buf, size_t size)
{
	parse_str(buf);
	return size;
}

static struct class_attribute class_attr_xhub_dbg =
	__ATTR(xhub_dbg, 0660, cls_attr_debug_show_func,
	cls_attr_debug_store_func);

static ssize_t cls_attr_dump_show_func(struct class *cls,
	struct class_attribute *attr, char *buf)
{
	hwlog_info("read xhub_dump node, IOM7 will restart\n");
	xhub_need_recovery(SH_FAULT_USER_DUMP);
	return snprintf(buf, MAX_STR_SIZE,
		"read xhub_dump node, IOM7 will restart\n");
}

static struct class_attribute class_attr_xhub_dump =
	__ATTR(xhub_dump, 0660, cls_attr_dump_show_func, NULL);

static ssize_t cls_attr_kernel_support_lib_ver_show_func(struct class *cls,
	struct class_attribute *attr, char *buf)
{
	uint32_t ver = 15; // for support large resolution acc sensor

	hwlog_info("read %s %d\n", __func__, ver);
	memcpy(buf, &ver, sizeof(ver));
	return sizeof(ver);
}

static struct class_attribute class_attr_libsensor_ver =
	__ATTR(libsensor_ver, 0660,
	cls_attr_kernel_support_lib_ver_show_func, NULL);

enum dbg_channel_stat {
	NO_TEST,
	TESTING,
	TEST_FINI_SUC,
	TEST_FINI_ERR,
	TEST_END,
};

static const char *const g_dbg_channel_test_stat_str[] = {
	"not in test",
	"testing",
	"test finish with success",
	"test finish with fail",
	"test not available",
};

enum user_cmd_para {
	USER_CMD_TAG,
	USER_CMD_CMD,
	USER_CMD_SUBCMD,
	USER_CMD_WRITE_LEN,
	USER_CMD_RESP,
};

struct user_cmd_t {
	struct write_info wr;
	uint32_t subcmd;
	int resp;
	enum dbg_channel_stat test_state;
	struct read_info rd;
};

#define MAX_CMD_NUM 5
#define MAX_DATA_BUF_SIZE (32 * 1024) // 32KB

static DEFINE_MUTEX(g_dbg_channel_mutex);

uint8_t *g_dbg_wbuf;
static struct user_cmd_t g_dbg_channel_usercmd;

#define dbg_chn_attr_show(ret, buf, offset, fmt, ...) \
do { \
	ret = sprintf_s(buf + offset, PAGE_SIZE - offset, fmt, ##__VA_ARGS__); \
	if (ret < 0) \
		return 0; \
	offset += ret; \
} while (0)

static ssize_t dbg_channel_show_func(struct class *cls,
	struct class_attribute *attr, char *buf)
{
	int ret, i;
	unsigned int offset = 0;
	struct user_cmd_t *ucmd = (struct user_cmd_t *)&g_dbg_channel_usercmd;
	struct write_info *wr = &ucmd->wr;
	int test_stat;

	dbg_chn_attr_show(ret, buf, offset,
		"help: format: echo tag cmd subcmd len resp > %s\n",
		attr->attr.name);

	mutex_lock(&g_dbg_channel_mutex);
	test_stat = ucmd->test_state;
	if (test_stat < NO_TEST || test_stat >= TEST_END)
		test_stat = TEST_END;

	dbg_chn_attr_show(ret, buf, offset,
		"test state: %s\n", g_dbg_channel_test_stat_str[test_stat]);

	if (test_stat != NO_TEST && test_stat != TEST_END) {
		dbg_chn_attr_show(ret, buf, offset,
			"test param: tag=%d, cmd=%d, subcmd=%u buflen=%u, resp=%d\n",
			wr->tag, wr->cmd, ucmd->subcmd,
			wr->wr_len, ucmd->resp);

		if (ucmd->rd.data_length <= 0 || ucmd->rd.data_length >= MAX_PKT_LENGTH)
			goto OUT;

		dbg_chn_attr_show(ret, buf, offset, "get data:\n");

		for (i = 0; i < ucmd->rd.data_length; i++) {
			dbg_chn_attr_show(ret, buf, offset, "%x ", ucmd->rd.data[i]);
			if (i % 10 == 0)
				dbg_chn_attr_show(ret, buf, offset,  "\n");
		}
		dbg_chn_attr_show(ret, buf, offset,  "\n");
	}

OUT:
	mutex_unlock(&g_dbg_channel_mutex);
	return offset;
}

// ret 0 suc: -1 err
static int parse_cmd(const char *buf, size_t size, struct user_cmd_t *ucmd)
{
	int arg = -1;
	int argc = 0;
	int argv[MAX_CMD_NUM] = {0};

	for (; (buf = get_str_begin(buf)) != NULL; buf = get_str_end(buf)) {
		if (get_arg(buf, &arg)) {
			argv[argc++] = arg;
			if (argc == MAX_CMD_NUM)
				break;
		}
	}

	if (argc < MAX_CMD_NUM)
		return -1;

	ucmd->wr.tag = argv[USER_CMD_TAG];
	ucmd->wr.cmd = argv[USER_CMD_CMD];
	ucmd->subcmd = argv[USER_CMD_SUBCMD];
	ucmd->wr.wr_len = argv[USER_CMD_WRITE_LEN];
	if (ucmd->wr.wr_len  < 4)
		ucmd->wr.wr_len = 4; // space for subcmd
	if (ucmd->wr.wr_len >= MAX_DATA_BUF_SIZE)
		ucmd->wr.wr_len = MAX_DATA_BUF_SIZE;
	ucmd->resp = argv[USER_CMD_RESP];

	return 0;
}

static int64_t gettimestamp(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	/* timevalToNano */
	return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static ssize_t dbg_channel_store_func(struct class *cls,
	struct class_attribute *attr, const char *buf, size_t size)
{
	struct user_cmd_t *ucmd = &g_dbg_channel_usercmd;
	struct write_info *wr = &ucmd->wr;
	struct read_info *p_rd = NULL;
	int i;
	int test_result = 0;
	int64_t timestamp;

	if (get_sensor_mcu_mode() != 1) {
		hwlog_err("mcu not ready\n");
		return size;
	}

	mutex_lock(&g_dbg_channel_mutex);

	(void)memset_s(&g_dbg_channel_usercmd, sizeof(struct user_cmd_t),
			0, sizeof(struct user_cmd_t));

	// get use cmd
	if (parse_cmd(buf, size, ucmd)) {
		hwlog_err("param err: should be tag, cmd, subcmd, resp, len\n");
		test_result = 1;
		goto OUT;
	}

	hwlog_info("%s: write cmd: tag=%d, cmd=%d, subcmd=%u buflen=%d, resp=%d\n",
		    __func__, wr->tag, wr->cmd, ucmd->subcmd, wr->wr_len, ucmd->resp);

	ucmd->test_state = TESTING;
	g_dbg_wbuf = (uint8_t *)vmalloc(wr->wr_len);
	if (g_dbg_wbuf == NULL) {
		hwlog_err("g_dbg_wbuf malloc err\n");
		goto OUT;
	}

	// fill config buff, 0, 1, 2...
	if (wr->wr_len > 0) {
		for (i = 4; i < wr->wr_len; i++)
			*(uint8_t *)((uintptr_t)g_dbg_wbuf + i) = i % 255;
	}

	*(uint32_t *)g_dbg_wbuf = ucmd->subcmd;
	wr->wr_buf = g_dbg_wbuf;

	if (ucmd->resp)
		p_rd = &ucmd->rd;

	timestamp = gettimestamp();
	// send cmd
	if (write_customize_cmd(wr, p_rd, false) != 0) {
		hwlog_err("%s write cmd failed\n", __func__);
		test_result = 1;
		goto OUT;
	}
	hwlog_info("delta time is %ld\n", gettimestamp() - timestamp);
OUT:
	if (g_dbg_wbuf != NULL)
		vfree(g_dbg_wbuf);
	ucmd->test_state = TEST_FINI_SUC;
	if (test_result != 0 || (p_rd && p_rd->errno != 0))
		ucmd->test_state = TEST_FINI_ERR;
	mutex_unlock(&g_dbg_channel_mutex);
	hwlog_info("%s finished\n", __func__);

	return size;
}

static struct class_attribute class_attr_dbg_channel =
	__ATTR(dbg_channel, 0660, dbg_channel_show_func,
	dbg_channel_store_func);

static ssize_t cls_attr_tell_mcu_streen_store_func(struct class *cls,
	struct class_attribute *attr, const char *buf, size_t size)
{
	unsigned long screen_status;

	if (!buf)
		return size;

	if (kstrtol(buf, 10, &screen_status))
		return -EINVAL;

	hwlog_info("get screen info = %d\n", screen_status);
	tell_screen_status_to_mcu((uint8_t)screen_status);
	return size;
}

static struct class_attribute class_attr_dual_screen_status =
	__ATTR(dual_screen_status, 0660, NULL,
	cls_attr_tell_mcu_streen_store_func);

void create_debug_files(void)
{
	if (class_create_file(sensors_class, &class_attr_xhub_dbg))
		hwlog_err("create files failed in %s\n", __func__);

	if (class_create_file(sensors_class, &class_attr_xhub_dump))
		hwlog_err("create files xhub_dump in %s\n", __func__);

	if (class_create_file(sensors_class, &class_attr_libsensor_ver))
		hwlog_err("create files libsensor_ver in %s\n", __func__);

	if (class_create_file(sensors_class, &class_attr_dual_screen_status))
		hwlog_err("create files dual_screen in %s\n", __func__);

	if (class_create_file(sensors_class, &class_attr_dbg_channel))
		hwlog_err("create files dbg_channel in %s\n", __func__);
}

static const char *get_iomcu_power_status(void)
{
	int status;

	memset(show_str, 0, MAX_STR_SIZE);

	mutex_lock(&mutex_pstatus);
	status = i_power_status.power_status;
	mutex_unlock(&mutex_pstatus);

	switch (status) {
	case SUB_POWER_ON:
		snprintf(show_str, MAX_STR_SIZE, "%s", "SUB_POWER_ON");
		break;
	case SUB_POWER_OFF:
		snprintf(show_str, MAX_STR_SIZE, "%s", "SUB_POWER_OFF");
		break;
	default:
		snprintf(show_str, MAX_STR_SIZE, "%s", "unknown status");
		break;
	}
	return show_str;
}

static const char *get_iomcu_current_opened_app(void)
{
	int i;
	char buf[SINGLE_STR_LENGTH_MAX] = {0};
	int index = 0;
	int copy_length;

	memset(show_str, 0, MAX_STR_SIZE);

	mutex_lock(&mutex_pstatus);
	for (i = 0; i < TAG_END; i++) {
		memset(buf, 0, SINGLE_STR_LENGTH_MAX);
		if (i_power_status.app_status[i]) {
			if (obj_tag_str[i] != NULL) {
				copy_length = (strlen(obj_tag_str[i]) >
					(SINGLE_STR_LENGTH_MAX - 1)) ?
					(SINGLE_STR_LENGTH_MAX - 1) :
					strlen(obj_tag_str[i]);
				strncpy(buf, obj_tag_str[i], copy_length);
			} else {
				copy_length = 2;
				snprintf(buf, 3, "%3d", i);
			}
			buf[copy_length] = '\n';
			index += (copy_length + 1);
			if (index < MAX_STR_SIZE) {
				strcat(show_str, buf);
			} else {
				show_str[MAX_STR_SIZE - 1] = 'X';
				hwlog_err("show_str too long\n");
				break;
			}
		}
	}
	mutex_unlock(&mutex_pstatus);
	return show_str;
}


static int get_iomcu_idle_time(void)
{
	return i_power_status.idle_time;
}

static const char *get_iomcu_active_app_during_suspend(void)
{
	int i;
	char buf[SINGLE_STR_LENGTH_MAX] = {0};
	int index = 0;
	int tf;
	uint64_t bit_map;
	int copy_length;

	memset(show_str, 0, MAX_STR_SIZE);

	mutex_lock(&mutex_pstatus);
	bit_map = i_power_status.active_app_during_suspend;
	mutex_unlock(&mutex_pstatus);

	for (i = 0; i < TAG_HW_PRIVATE_APP_END; i++) {
		memset(buf, 0, SINGLE_STR_LENGTH_MAX);
		tf = (bit_map >> i) & 0x01;
		if (tf) {
			if (iomcu_app_id_str[i] != NULL) {
				copy_length = (strlen(iomcu_app_id_str[i]) >
					(SINGLE_STR_LENGTH_MAX - 1)) ?
					(SINGLE_STR_LENGTH_MAX - 1) :
					strlen(iomcu_app_id_str[i]);
				strncpy(buf, iomcu_app_id_str[i], copy_length);
			} else {
				copy_length = 2;
				snprintf(buf, 3, "%3d", i);
			}
			buf[copy_length] = '\n';
			index += (copy_length + 1);
			if (index < MAX_STR_SIZE) {
				strcat(show_str, buf);
			} else {
				show_str[MAX_STR_SIZE - 1] = 'X';
				hwlog_err("show_str too long\n");
				break;
			}
		}
	}
	return show_str;
}


static int mcu_power_log_process(const pkt_header_t *head)
{
	hwlog_info("%s in\n", __func__);

	mutex_lock(&mutex_pstatus);
	i_power_status.idle_time = ((pkt_power_log_report_req_t *)head)->idle_time;
	i_power_status.active_app_during_suspend =
		((pkt_power_log_report_req_t *)head)->current_app_mask;
	mutex_unlock(&mutex_pstatus);

	hwlog_info("last suspend iomcu idle time is %d , active apps high is  0x%x, low is  0x%x\n",
		i_power_status.idle_time,
		(uint32_t)((i_power_status.active_app_during_suspend >> 32) & 0xffffffff),
		(uint32_t)(i_power_status.active_app_during_suspend & 0xffffffff));
	return 0;
}

static ssize_t show_power_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%s\n", get_iomcu_power_status());
}

static ssize_t show_app_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%s\n",
		get_iomcu_current_opened_app());
}
static ssize_t show_idle_time(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%d\n", get_iomcu_idle_time());
}

static ssize_t show_active_app_during_suspend(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%s\n",
		get_iomcu_active_app_during_suspend());
}

static DEVICE_ATTR(power_status, 0440, show_power_status, NULL);
static DEVICE_ATTR(current_app, 0440, show_app_status, NULL);
static DEVICE_ATTR(idle_time, 0440, show_idle_time, NULL);
static DEVICE_ATTR(active_app_during_suspend, 0440,
	show_active_app_during_suspend, NULL);

static struct attribute *power_info_attrs[] = {
	&dev_attr_power_status.attr,
	&dev_attr_current_app.attr,
	&dev_attr_idle_time.attr,
	&dev_attr_active_app_during_suspend.attr,
	NULL,
};

static const struct attribute_group power_info_attrs_grp = {
	.attrs = power_info_attrs,
};

static struct power_dbg power_info = {
	 .name = "power_info",
	 .attrs_group = &power_info_attrs_grp,
};

static int iomcu_power_info_init(void)
{
	memset(&i_power_status, 0, sizeof(iomcu_power_status));

	register_mcu_event_notifier(TAG_LOG, CMD_LOG_POWER_REQ,
		mcu_power_log_process);

	iomcu_power = class_create(THIS_MODULE, "iomcu_power");
	if (IS_ERR(iomcu_power)) {
		hwlog_err(" %s class creat fail\n", __func__);
		return -1;
	}

	power_info.dev = device_create(iomcu_power, NULL, 0,
		&power_info, power_info.name);
	if (!(power_info.dev)) {
		hwlog_err(" %s creat dev fail\n", __func__);
		class_destroy(iomcu_power);
		return -1;
	}

	if (power_info.attrs_group) {
		if (sysfs_create_group(&power_info.dev->kobj,
			power_info.attrs_group)) {
			hwlog_err("create files failed in %s\n", __func__);
		} else {
			hwlog_info("%s ok\n", __func__);
			return 0;
		}
	} else {
		hwlog_err("power_info.attrs_group is null\n");
	}

	device_destroy(iomcu_power, 0);
	class_destroy(iomcu_power);
	return -1;
}

static void iomcu_power_info_exit(void)
{
	device_destroy(iomcu_power, 0);
	class_destroy(iomcu_power);
}

static int xhub_debug_init(void)
{
	if (!is_scp_ready(SCP_A_ID))
		return -1;
	register_my_debug_operations();
	iomcu_power_info_init();

	return 0;
}

static void xhub_debug_exit(void)
{
	unregister_my_debug_operations();
	iomcu_power_info_exit();
}

late_initcall_sync(xhub_debug_init);
module_exit(xhub_debug_exit);

MODULE_AUTHOR("SensorHub <smartphone@huawei.com>");
MODULE_DESCRIPTION("SensorHub debug driver");
MODULE_LICENSE("GPL");
