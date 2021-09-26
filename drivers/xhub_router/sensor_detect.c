/*
 * drivers/inputhub/sensor_detect.c
 *
 * sensors detection driver
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include "sensor_feima.h"
#include "xhub_route.h"
#include "xhub_boot.h"
#include "protocol.h"
#include <sensor_sysfs.h>
#include <sensor_config.h>
#include <sensor_detect.h>
#include "acc_channel.h"
#include "airpress_channel.h"
#include "als_channel.h"
#include "cap_prox_channel.h"
#include "gyro_channel.h"
#include "mag_channel.h"
#include "ps_channel.h"
#include "therm_channel.h"
#include "xhub_debug.h"
#include "xhub_recovery.h"
#include <xhub_router/sensorhub.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <huawei_platform/dev_detect/hw_dev_detect.h>
#include <hwmanufac/dev_detect/dev_detect.h>
#endif
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "securec.h"

#define HANDPRESS_DEFAULT_STATE "huawei,default-state"
#define ADAPT_SENSOR_LIST_NUM 20
#define HORIZONTAL_PICKUP_PARA_LEN 16
#define RET_FAIL              (-1)
#define RET_SUCC              0
#define PHONE_TYPE_LIST 2

static struct sensor_redetect_state s_redetect_state;
static struct wakeup_source sensor_rd;
static struct work_struct redetect_work;
static char g_pkg_buf[MAX_PKT_LENGTH];
static char aux_buf[MAX_PKT_LENGTH];
static pkt_sys_dynload_req_t *dyn_req = (pkt_sys_dynload_req_t *)g_pkg_buf;
static pkt_sys_dynload_req_t *aux_req = (pkt_sys_dynload_req_t *)aux_buf;
struct sleeve_detect_pare sleeve_detect_paremeter[MAX_PHONE_COLOR_NUM];
struct sensorlist_info sensorlist_info[SENSOR_MAX];
uint8_t is_close;
int _device_detect(struct device_node *dn, int index, struct sensor_combo_cfg *p_succ_ret);
int get_combo_bus_tag(const char *bus, uint8_t *tag);
int hall_number = 1;
int hall_sen_type;
int support_hall_hishow = 0;
int support_hall_pen = 0;           /* defalut not support pen */
int support_hall_keyboard = 0;      /* defalut not support keyboard */
int support_hall_lightstrap;
uint32_t hall_hishow_value = 0x5;   /* 00101b, hishow default trigger mask */
uint32_t hall_pen_value = 0x10;     /* 10000b, pen default trigger mask */
uint32_t hall_keyboard_value = 0x4; /* 00100b, keyboard default trigger mask */
uint32_t hall_lightstrap_value = 0x2; /* 0010b, lightstrap trigger mask */
int mag_opend;
int gyro_detect_flag;
int sonic_ps_use_rcv;

static struct connectivity_platform_data connectivity_data = {
	.cfg = {
		.bus_type = TAG_I3C,
		.bus_num = 1,
		.disable_sample_thread = 1,
		{ .i2c_address = 0x6E },
	},
	.poll_interval = 50,
	.gpio1_gps_cmd_ap = 200,
	.gpio1_gps_cmd_sh = 230,
	.gpio2_gps_ready_ap = 213,
	.gpio2_gps_ready_sh = 242,
	.gpio3_wakeup_gps_ap = 214,
	.gpio3_wakeup_gps_sh = 243,
	.i3c_frequency = 0,
	.gpio1_gps_cmd_pinmux = 2,
	.gpio2_gps_ready_pinmux = 2,
	.gpio3_wakeup_gps_pinmux = 4,
};

struct aod_platform_data aod_data = {
	.cfg = DEF_SENSOR_COM_SETTING,
	.feature_set = { 0 },
};

struct rpc_platform_data rpc_data = {
	.table = { 0 },
	.mask = { 0 },
	.default_value = 0,
	.mask_enable = 0,
	.sar_choice = 0,
	.sim_type_swtich_flag = 0,
	.fusion_type = 0,
};

struct motion_platform_data motion_data = {
	.motion_horizontal_pickup_flag = 0,
	.angle_gap = 45,  // default angle
};

/*lint +e785*/
struct sensor_detect_manager sensor_manager[SENSOR_MAX] = {
	{ "acc", ACC, DET_INIT, TAG_ACCEL, NULL, 0 },
	{ "mag", MAG, DET_INIT, TAG_MAG, NULL, 0 },
	{ "gyro", GYRO, DET_INIT, TAG_GYRO, NULL, 0 },
	{ "als", ALS, DET_INIT, TAG_ALS, NULL, 0 },
	{ "ps", PS, DET_INIT, TAG_PS, NULL, 0 },
	{ "airpress", AIRPRESS, DET_INIT, TAG_PRESSURE, NULL, 0 },
	{ "cap_prox", CAP_PROX, DET_INIT, TAG_CAP_PROX, NULL, 0 },
	{ "connectivity", CONNECTIVITY, DET_INIT, TAG_CONNECTIVITY, &connectivity_data, sizeof(connectivity_data) },
	{ "rpc", RPC, DET_INIT, TAG_RPC, &rpc_data, sizeof(rpc_data) },
	{ "sh_aod", SH_AOD, DET_INIT, TAG_AOD, &aod_data, sizeof(aod_data) },
	{ "acc1", ACC1, DET_INIT, TAG_ACC1, NULL, 0 },
	{ "gyro1", GYRO1, DET_INIT, TAG_GYRO1, NULL, 0 },
	{ "als1", ALS1, DET_INIT, TAG_ALS1, NULL, 0 },
	{ "mag1", MAG1, DET_INIT, TAG_MAG1, NULL, 0 },
	{ "als2", ALS2, DET_INIT, TAG_ALS2, NULL, 0 },
	{ "cap_prox1", CAP_PROX1, DET_INIT, TAG_CAP_PROX1, NULL, 0 },
	{ "motion", MOTION, DET_INIT, TAG_MOTION, &motion_data, sizeof(motion_data) },
	{ "sound", SOUND, DET_INIT, TAG_SOUND, NULL, 0 },
	{ "thermometer", THERMOMETER, DET_INIT, TAG_THERMOMETER, NULL, 0 },
};

static const struct app_link_info app_link_info_gyro[] = {
	{ SENSORHUB_TYPE_ACCELEROMETER, TAG_ACCEL, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_LIGHT, TAG_ALS, 1, { TAG_ALS } },
	{ SENSORHUB_TYPE_PROXIMITY, TAG_PS, 1, { TAG_PS } },
	{ SENSORHUB_TYPE_GYROSCOPE, TAG_GYRO, 1, { TAG_GYRO } },
	{ SENSORHUB_TYPE_GRAVITY, TAG_GRAVITY, 3, { TAG_ACCEL, TAG_GYRO, TAG_MAG } },
	{ SENSORHUB_TYPE_MAGNETIC, TAG_MAG, 2, { TAG_GYRO, TAG_MAG, } },
	{ SENSORHUB_TYPE_LINEARACCELERATE, TAG_LINEAR_ACCEL, 3, { TAG_ACCEL, TAG_GYRO, TAG_MAG } },
	{ SENSORHUB_TYPE_ORIENTATION, TAG_ORIENTATION, 3, { TAG_ACCEL, TAG_GYRO, TAG_MAG } },
	{ SENSORHUB_TYPE_ROTATEVECTOR, TAG_ROTATION_VECTORS, 3, { TAG_ACCEL, TAG_GYRO, TAG_MAG } },
	{ SENSORHUB_TYPE_PRESSURE, TAG_PRESSURE, 1, { TAG_PRESSURE } },
	{ SENSORHUB_TYPE_HALL, TAG_HALL, 0, { 0 } },
	{ SENSORHUB_TYPE_MAGNETIC_FIELD_UNCALIBRATED, TAG_MAG_UNCALIBRATED, 2, { TAG_MAG, TAG_GYRO } },
	{ SENSORHUB_TYPE_GAME_ROTATION_VECTOR, TAG_GAME_RV, 2, { TAG_ACCEL, TAG_GYRO } },
	{ SENSORHUB_TYPE_GYROSCOPE_UNCALIBRATED, TAG_GYRO_UNCALIBRATED, 1, { TAG_GYRO } },
	{ SENSORHUB_TYPE_SIGNIFICANT_MOTION, TAG_SIGNIFICANT_MOTION, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_STEP_DETECTOR, TAG_STEP_DETECTOR, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_STEP_COUNTER, TAG_STEP_COUNTER, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_GEOMAGNETIC_ROTATION_VECTOR, TAG_GEOMAGNETIC_RV, 3, { TAG_ACCEL, TAG_GYRO, TAG_MAG } },
	{ SENSORHUB_TYPE_HANDPRESS, TAG_HANDPRESS, 1, { TAG_HANDPRESS } },
	{ SENSORHUB_TYPE_CAP_PROX, TAG_CAP_PROX, 1, { TAG_CAP_PROX } },
	{ SENSORHUB_TYPE_PHONECALL, TAG_PHONECALL, 2, { TAG_ACCEL, TAG_PS } },
	{ SENSORHUB_TYPE_MAGN_BRACKET, TAG_MAGN_BRACKET, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_HINGE, TAG_HINGE, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_META_DATA, TAG_FLUSH_META, 0, { 0 } },
	{ SENSORHUB_TYPE_RPC, TAG_RPC, 4, { TAG_ACCEL, TAG_GYRO, TAG_CAP_PROX, TAG_CAP_PROX1 } },
	{ SENSORHUB_TYPE_AGT, TAG_AGT, 0, { 0 } },
	{ SENSORHUB_TYPE_COLOR, TAG_COLOR, 0, { 0 } },
	{ SENSORHUB_TYPE_ACCELEROMETER_UNCALIBRATED, TAG_ACCEL_UNCALIBRATED, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_TOF, TAG_TOF, 1, { TAG_TOF } },
	{ SENSORHUB_TYPE_DROP, TAG_DROP, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_POSTURE, TAG_POSTURE, 2, { TAG_ACCEL, TAG_GYRO } },
	{ SENSORHUB_TYPE_EXT_HALL, TAG_EXT_HALL, 1, { TAG_MAG1 } },
	{ SENSORHUB_TYPE_ACCELEROMETER1, TAG_ACC1, 1, { TAG_ACC1 } },
	{ SENSORHUB_TYPE_GYROSCOPE1, TAG_GYRO1, 1, { TAG_GYRO1 } },
	{ SENSORHUB_TYPE_PROXIMITY1, TAG_PS1, 0, { 0 } },
	{ SENSORHUB_TYPE_LIGHT1, TAG_ALS1, 1, { TAG_ALS1 } },
	{ SENSORHUB_TYPE_MAGNETIC1, TAG_MAG1, 0, { 0 } },
	{ SENSORHUB_TYPE_LIGHT2, TAG_ALS2, 1, { TAG_ALS2 } },
	{ SENSORHUB_TYPE_CAP_PROX1, TAG_CAP_PROX1, 1, { TAG_CAP_PROX1 } },
};

static const struct app_link_info app_link_info_no_gyro[] = {
	{ SENSORHUB_TYPE_ACCELEROMETER, TAG_ACCEL, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_LIGHT, TAG_ALS, 1, { TAG_ALS } },
	{ SENSORHUB_TYPE_PROXIMITY, TAG_PS, 1, { TAG_PS } },
	{ SENSORHUB_TYPE_GYROSCOPE, TAG_GYRO, 2, { TAG_MAG, TAG_ACCEL } },
	{ SENSORHUB_TYPE_GRAVITY, TAG_GRAVITY, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_MAGNETIC, TAG_MAG, 1, { TAG_MAG } },
	{ SENSORHUB_TYPE_LINEARACCELERATE, TAG_LINEAR_ACCEL, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_ORIENTATION, TAG_ORIENTATION, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_ROTATEVECTOR, TAG_ROTATION_VECTORS, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_PRESSURE, TAG_PRESSURE, 1, { TAG_PRESSURE } },
	{ SENSORHUB_TYPE_HALL, TAG_HALL, 0, {0} },
	{ SENSORHUB_TYPE_MAGNETIC_FIELD_UNCALIBRATED, TAG_MAG_UNCALIBRATED, 1, { TAG_MAG } },
	{ SENSORHUB_TYPE_GAME_ROTATION_VECTOR, TAG_GAME_RV, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_GYROSCOPE_UNCALIBRATED, TAG_GYRO_UNCALIBRATED, 0, { 0 } },
	{ SENSORHUB_TYPE_SIGNIFICANT_MOTION, TAG_SIGNIFICANT_MOTION, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_STEP_DETECTOR, TAG_STEP_DETECTOR, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_STEP_COUNTER, TAG_STEP_COUNTER, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_GEOMAGNETIC_ROTATION_VECTOR, TAG_GEOMAGNETIC_RV, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_HANDPRESS, TAG_HANDPRESS, 1, { TAG_HANDPRESS } },
	{ SENSORHUB_TYPE_CAP_PROX, TAG_CAP_PROX, 1, { TAG_CAP_PROX } },
	{ SENSORHUB_TYPE_PHONECALL, TAG_PHONECALL, 2, { TAG_ACCEL, TAG_PS } },
	{ SENSORHUB_TYPE_MAGN_BRACKET, TAG_MAGN_BRACKET, 2, { TAG_ACCEL, TAG_MAG } },
	{ SENSORHUB_TYPE_HINGE, TAG_HINGE, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_META_DATA, TAG_FLUSH_META, 0, { 0 } },
	{ SENSORHUB_TYPE_RPC, TAG_RPC, 3, { TAG_ACCEL, TAG_CAP_PROX, TAG_CAP_PROX1 } },
	{ SENSORHUB_TYPE_AGT, TAG_AGT, 0, { 0 } },
	{ SENSORHUB_TYPE_COLOR, TAG_COLOR, 0, { 0 } },
	{ SENSORHUB_TYPE_ACCELEROMETER_UNCALIBRATED, TAG_ACCEL_UNCALIBRATED, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_TOF, TAG_TOF, 1, { TAG_TOF } },
	{ SENSORHUB_TYPE_DROP, TAG_DROP, 1, { TAG_ACCEL } },
	{ SENSORHUB_TYPE_POSTURE, TAG_POSTURE, 0, { 0 } },
	{ SENSORHUB_TYPE_EXT_HALL, TAG_EXT_HALL, 1, { TAG_MAG1 } },
	{ SENSORHUB_TYPE_ACCELEROMETER1, TAG_ACC1, 1, { TAG_ACC1 } },
	{ SENSORHUB_TYPE_GYROSCOPE1, TAG_GYRO1, 0, { 0 } },
	{ SENSORHUB_TYPE_PROXIMITY1, TAG_PS1, 0, { 0 } },
	{ SENSORHUB_TYPE_LIGHT1, TAG_ALS1, 1, { TAG_ALS1 } },
	{ SENSORHUB_TYPE_MAGNETIC1, TAG_MAG1, 0, { 0 } },
	{ SENSORHUB_TYPE_LIGHT2, TAG_ALS2, 1, { TAG_ALS2 } },
	{ SENSORHUB_TYPE_CAP_PROX1, TAG_CAP_PROX1, 1, { TAG_CAP_PROX1 } },
};

struct f03_reg_info {
	uint16_t addr;
	uint16_t value;
	uint16_t delay;
};

/* get app attach sensor info */
const struct app_link_info *get_app_link_info(int type)
{
	size_t i, size;
	const struct app_link_info *app_info = NULL;

	if (gyro_detect_flag) {
		app_info = app_link_info_gyro;
		size = sizeof(app_link_info_gyro) / sizeof(struct app_link_info);
	} else {
		app_info = app_link_info_no_gyro;
		size = sizeof(app_link_info_no_gyro) / sizeof(struct app_link_info);
	}

	for (i = 0; i < size; i++) {
		if (type == app_info[i].hal_sensor_type &&
			app_info[i].used_sensor_cnt > 0 &&
			app_info[i].used_sensor_cnt <= SENSORHUB_TAG_NUM_MAX)
			return &app_info[i];
	}

	return NULL;
}

sensor_detect_list get_id_by_sensor_tag(int tag)
{
	int i;

	for (i = 0; i < SENSOR_MAX; i++) {
		if (sensor_manager[i].tag == tag)
			break;
	}
	return i;
}

void read_sensorlist_info_continue(struct device_node *dn, int sensor)
{
	int temp = 0;

	if (of_property_read_u32(dn, "fifoMaxEventCount", &temp) == 0) {
		sensorlist_info[sensor].fifo_max_event_count = temp;
		hwlog_info("sensor sensor_detect_list %d get fifoMaxEventCount %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].fifo_max_event_count = -1;
	}
	if (of_property_read_u32(dn, "maxDelay", &temp) == 0) {
		sensorlist_info[sensor].max_delay = temp;
		hwlog_info("sensor sensor_detect_list %d get maxDelay %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].max_delay = -1;
	}
	if (of_property_read_u32(dn, "flags", &temp) == 0) {
		sensorlist_info[sensor].flags = temp;
		hwlog_info("sensor sensor_detect_list %d get flags %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].flags = -1;
	}
}

void read_sensorlist_info(struct device_node *dn, int sensor)
{
	int temp = 0;
	char *chip_info = NULL;

	if (of_property_read_string(dn, "sensorlist_name",
		(const char **)&chip_info) >= 0) {
		strncpy(sensorlist_info[sensor].name, chip_info,
			MAX_CHIP_INFO_LEN - 1);
		sensorlist_info[sensor].name[MAX_CHIP_INFO_LEN - 1] = '\0';
		hwlog_debug("sensor chip info name %s\n", chip_info);
		hwlog_debug("sensor sensor_detect_list %d get name %s\n",
			sensor, sensorlist_info[sensor].name);
	} else {
		sensorlist_info[sensor].name[0] = '\0';
	}
	if (of_property_read_string(dn, "vendor",
		(const char **)&chip_info) == 0) {
		strncpy(sensorlist_info[sensor].vendor, chip_info,
			MAX_CHIP_INFO_LEN - 1);
		sensorlist_info[sensor].vendor[MAX_CHIP_INFO_LEN - 1] = '\0';
		hwlog_debug("sensor sensor_detect_list %d get vendor %s\n",
			sensor, sensorlist_info[sensor].vendor);
	} else {
		sensorlist_info[sensor].vendor[0] = '\0';
	}
	if (of_property_read_u32(dn, "version", &temp) == 0) {
		sensorlist_info[sensor].version = temp;
		hwlog_info("sensor sensor_detect_list %d get version %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].version = -1;
	}
	if (of_property_read_u32(dn, "maxRange", &temp) == 0) {
		sensorlist_info[sensor].max_range = temp;
		hwlog_info("sensor sensor_detect_list %d get maxRange %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].max_range = -1;
	}
	if (of_property_read_u32(dn, "resolution", &temp) == 0) {
		sensorlist_info[sensor].resolution = temp;
		hwlog_info("sensor sensor_detect_list %d get resolution %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].resolution = -1;
	}
	if (of_property_read_u32(dn, "power", &temp) == 0) {
		sensorlist_info[sensor].power = temp;
		hwlog_info("sensor sensor_detect_list %d get power %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].power = -1;
	}
	if (of_property_read_u32(dn, "minDelay", &temp) == 0) {
		sensorlist_info[sensor].min_delay = temp;
		hwlog_info("sensor sensor_detect_list %d get minDelay %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].min_delay = -1;
	}
	if (of_property_read_u32(dn, "fifoReservedEventCount", &temp) == 0) {
		sensorlist_info[sensor].fifo_reserved_event_count = temp;
		hwlog_info("sensor sensor_detect_list %d get fifoReservedEventCount %d\n",
			sensor, temp);
	} else {
		sensorlist_info[sensor].fifo_reserved_event_count = -1;
	}
	read_sensorlist_info_continue(dn, sensor);
}

void read_chip_info(struct device_node *dn, sensor_detect_list sname)
{
	char *chip_info = NULL;
	int ret;

	ret = of_property_read_string(dn, "compatible",
		(const char **)&chip_info);
	if (ret)
		hwlog_err("%s:read name_id:%d info fail\n", __func__, sname);
	else
		strncpy(sensor_chip_info[sname], chip_info, MAX_CHIP_INFO_LEN - 1);
	hwlog_info("get chip info from dts success\n");
}

void read_aux_file_list(uint16_t fileid, uint16_t tag)
{
	aux_req->file_list[aux_req->file_count * 2] = fileid;
	aux_req->file_list[aux_req->file_count * 2 + 1] = tag;
	aux_req->file_count++;
}

void read_dyn_file_list(uint16_t fileid)
{
	dyn_req->file_list[dyn_req->file_count] = fileid;
	dyn_req->file_count++;
}

static void read_connectivity_bus_type(struct device_node *dn, uint8_t *bus_type)
{
	const char *bus_string = NULL;
	int temp = (int)*bus_type;

	if (of_property_read_string(dn, "bus_type", &bus_string)) {
		hwlog_err("%s:connectivity bus_type not configured\n", __func__);
		return;
	}
	if (get_combo_bus_tag(bus_string, (uint8_t *)&temp)) {
		hwlog_warn("connectivity %s bus_type string invalid, next detect digit\n", bus_string);
		if (of_property_read_u32(dn, "bus_type", &temp)) {
			hwlog_err("%s:read bus_type digit fail\n", __func__);
			return;
		}

		if (temp >= TAG_END) {
			hwlog_err("%s:read bus_type %d invalid\n", __func__, temp);
			return;
		}
	}
	*bus_type = (uint8_t)temp;
}

static void read_connectivity_data_from_dts_continue(struct device_node *dn)
{
	int temp = 0;

	if (of_property_read_u32(dn, "bus_number", &temp))
		hwlog_err("%s:read bus_number fail\n", __func__);
	else
		connectivity_data.cfg.bus_num = (uint8_t)temp;

	if (of_property_read_u32(dn, "i2c_address", &temp))
		hwlog_err("%s:read i2c_address fail\n", __func__);
	else
		connectivity_data.cfg.i2c_address = (uint32_t)temp;

	if (of_property_read_u32(dn, "i3c_frequency", &temp))
		hwlog_err("%s:read i3c_frequency fail\n", __func__);
	else
		connectivity_data.i3c_frequency = (uint32_t) temp;

	if (of_property_read_u32(dn, "file_id", &temp)) {
		hwlog_err("%s:read connectivity file_id fail\n", __func__);
	} else {
		dyn_req->file_list[dyn_req->file_count] = (uint16_t)temp;
		dyn_req->file_count++;
	}

	hwlog_err("connectivity file id is %d\n", temp);
	if (of_property_read_u32(dn, "sensor_list_info_id", &temp))
		hwlog_err("%s:read connectivity sensor_list_info_id fail\n",
			__func__);
	else
		sensorlist[++sensorlist[0]] = (uint16_t) temp;
}

static void read_connectivity_data_from_dts(struct device_node *dn)
{
	int temp = 0;

	read_chip_info(dn, CONNECTIVITY);

	if (of_property_read_u32(dn, "gpio1_gps_cmd_ap", &temp))
		hwlog_err("%s:read gpio1_gps_cmd_ap fail\n", __func__);
	else
		connectivity_data.gpio1_gps_cmd_ap = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio1_gps_cmd_sh", &temp))
		hwlog_err("%s:read gpio1_gps_cmd_sh fail\n", __func__);
	else
		connectivity_data.gpio1_gps_cmd_sh = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio1_gps_cmd_pinmux", &temp))
		hwlog_warn("%s:read gpio1_gps_cmd_pinmux fail\n", __func__);
	else
		connectivity_data.gpio1_gps_cmd_pinmux = (uint16_t)temp;

	if (of_property_read_u32(dn, "gpio2_gps_ready_ap", &temp))
		hwlog_err("%s:read gpio2_gps_ready_ap fail\n", __func__);
	else
		connectivity_data.gpio2_gps_ready_ap = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio2_gps_ready_sh", &temp))
		hwlog_err("%s:read gpio2_gps_ready_sh fail\n", __func__);
	else
		connectivity_data.gpio2_gps_ready_sh = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio2_gps_ready_pinmux", &temp))
		hwlog_warn("%s:read gpio2_gps_ready_pinmux fail\n", __func__);
	else
		connectivity_data.gpio2_gps_ready_pinmux = (uint16_t)temp;

	if (of_property_read_u32(dn, "gpio3_wakeup_gps_ap", &temp))
		hwlog_err("%s:read gpio3_wakeup_gps_ap fail\n", __func__);
	else
		connectivity_data.gpio3_wakeup_gps_ap = (gpio_num_type) temp;

	if (of_property_read_u32(dn, "gpio3_wakeup_gps_sh", &temp))
		hwlog_err("%s:read gpio3_wakeup_gps_sh fail\n", __func__);
	else
		connectivity_data.gpio3_wakeup_gps_sh = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio3_wakeup_gps_pinmux", &temp))
		hwlog_warn("%s:read gpio3_wakeup_gps_pinmux fail\n", __func__);
	else
		connectivity_data.gpio3_wakeup_gps_pinmux = (uint16_t)temp;

	read_connectivity_bus_type(dn, (uint8_t *)&connectivity_data.cfg.bus_type);
	read_connectivity_data_from_dts_continue(dn);
}

static void read_sound_data_from_dts(struct device_node *dn)
{
	struct ps_platform_data *pf_data = NULL;
	int temp = 0;

	hwlog_info("sound read dts\n");
	read_chip_info(dn, SOUND);
	if (of_property_read_u32(dn, "file_id", &temp)) {
		hwlog_err("%s:read sound file_id fail\n", __func__);
	} else {
		dyn_req->file_list[dyn_req->file_count] = (uint16_t)temp;
		dyn_req->file_count++;
		sonic_ps_use_rcv = 1;
	}

	pf_data = ps_get_platform_data(TAG_PS);
	if (pf_data == NULL)
		return;

	if (of_property_read_u32(dn, "gpio_irq_soc", &temp))
		hwlog_warn("%s:read sound gpio_irq_soc fail\n", __func__);
	else
		pf_data->sound_gpio_irq_soc = (gpio_num_type)temp;

	if (of_property_read_u32(dn, "gpio_irq_sh", &temp))
		hwlog_warn("%s:read sound gpio_irq_sh fail\n", __func__);
	else
		pf_data->sound_gpio_irq_sh = (gpio_num_type)temp;

	hwlog_info("sound_gpio_irq_soc = %d, sound_gpio_irq_sh = %d\n",
		pf_data->sound_gpio_irq_soc, pf_data->sound_gpio_irq_sh);
}

static void read_aod_data_from_dts(struct device_node *dn)
{
	int i;
	int len = of_property_count_u32_elems(dn, "feature");

	read_chip_info(dn, SH_AOD);

	if (len <= 0) {
		hwlog_warn("%s:count u32 data fail\n", __func__);
		return;
	}

	if (of_property_read_u32_array(dn, "feature", aod_data.feature_set, len))
		hwlog_err("%s:read chip_id_value fail\n", __func__);

	for (i = 0; i < len; i++)
		hwlog_info("aod_data.feature_set[%d]= 0x%x\n",
			i, aod_data.feature_set[i]);
}

static void read_rpc_config_from_dts(struct device_node *dn)
{
	int t = 0;
	int *temp = &t;

	if (of_property_read_u32(dn, "file_id", (u32 *)temp)) {
		hwlog_err("%s:read rpc file_id fail\n", __func__);
	} else {
		dyn_req->file_list[dyn_req->file_count] = (uint16_t) t;
		dyn_req->file_count++;
	}

	if (of_property_read_u32(dn, "sensor_list_info_id", (u32 *)temp))
		hwlog_err("%s:read rpc sensor_list_info_id fail\n", __func__);
	else
		sensorlist[++sensorlist[0]] = (uint16_t)t;

	if (of_property_read_u32(dn, "default_value", (u32 *)temp))
		hwlog_err("%s:read default_value fail\n", __func__);
	else
		rpc_data.default_value = (uint16_t)t;

	if (of_property_read_u32(dn, "mask_enable", (u32 *)temp))
		hwlog_err("%s:read mask_enable fail\n", __func__);
	else
		rpc_data.mask_enable = (uint16_t)t;

	if (of_property_read_u32(dn, "sar_choice", (u32 *)temp))
		hwlog_err("%s:read mask_enable fail\n", __func__);
	else
		rpc_data.sar_choice = (uint16_t)t;

	if (of_property_read_u32(dn, "sim_type_swtich_flag", (u32 *)temp))
		hwlog_err("%s:read sim type swtich flag fail\n", __func__);
	else
		rpc_data.sim_type_swtich_flag = (uint16_t)t;

	if (of_property_read_u32(dn, "fusion_type", (u32 *)temp))
		hwlog_err("%s:read fusion type fail\n", __func__);
	else
		rpc_data.fusion_type = (uint16_t)t;
}

static int read_rpc_data_from_dts(struct device_node *dn)
{
	unsigned int i;
	u32 wia[32] = { 0 };
	struct property *prop = NULL;
	unsigned int len;

	memset(&rpc_data, 0, sizeof(rpc_data));
	memset(wia, 0, sizeof(wia));

	read_chip_info(dn, RPC);

	prop = of_find_property(dn, "table", NULL);
	if (!prop) {
		hwlog_err("%s! prop is NULL\n", __func__);
		return -1;
	}

	len = (u32)(prop->length) / sizeof(u32);
	if (of_property_read_u32_array(dn, "table", wia, len)) {
		hwlog_err("%s:read rpc_table from dts fail\n", __func__);
		return -1;
	}
	for (i = 0; i < len; i++)
		rpc_data.table[i] = (u16)wia[i];

	memset(wia, 0, sizeof(wia));
	prop = of_find_property(dn, "mask", NULL);
	if (!prop) {
		hwlog_err("%s! prop is NULL\n", __func__);
		return -1;
	}

	len = (u32)(prop->length) / sizeof(u32);
	if (of_property_read_u32_array(dn, "mask", wia, len)) {
		hwlog_err("%s:read rpc_mask from dts fail\n", __func__);
		return -1;
	}
	for (i = 0; i < len; i++)
		rpc_data.mask[i] = (u16)wia[i];

	read_rpc_config_from_dts(dn);
	read_sensorlist_info(dn, RPC);
	return 0;
}

static void read_motion_data_from_dts(struct device_node *dn)
{
	u32 tmp;

	if (!dn) {
		hwlog_err("%s! motion node is NULL\n", __func__);
		return;
	}

	read_chip_info(dn, MOTION);

	if (of_property_read_u32(dn, "motion_horizontal_pickup_flag", &tmp)) {
		hwlog_err("%s:read horizontal_pickup_flag from dts fail\n", __func__);
	} else {
		motion_data.motion_horizontal_pickup_flag = (uint8_t)tmp;
		hwlog_info("%s:read horizontal_pickup_flag from dts succ\n", __func__);
	}

	hwlog_info("%s: pickup_flag %d\n", __func__,
		(int)motion_data.motion_horizontal_pickup_flag);

	if (of_property_read_u32(dn, "angle_gap", &tmp)) {
		hwlog_err("%s:read angle_gap from dts fail\n",  __func__);
	} else {
		motion_data.angle_gap = (uint8_t)tmp;
		hwlog_info("%s:read angle_gap from dts succ\n",  __func__);
	}

	hwlog_info("%s: angle_gap %d\n", __func__, (int)motion_data.angle_gap);
}

static int get_adapt_file_id_for_dyn_load(void)
{
	u32 wia[ADAPT_SENSOR_LIST_NUM] = { 0 };
	struct property *prop = NULL;
	unsigned int i;
	unsigned int len;
	struct device_node *xhub_node = NULL;
	const char *name = "adapt_file_id";
	char *temp = NULL;

	xhub_node = of_find_compatible_node(NULL, NULL, "huawei,xhub");
	if (!xhub_node) {
		hwlog_err("%s, can't find node sensorhub\n", __func__);
		return -1;
	}
	prop = of_find_property(xhub_node, name, NULL);
	if (!prop) {
		hwlog_err("%s! prop is NULL\n", __func__);
		return -EINVAL;
	}
	if (!prop->value) {
		hwlog_err("%s! prop->value is NULL\n", __func__);
		return -ENODATA;
	}
	len = prop->length / 4;
	if (of_property_read_u32_array(xhub_node, name, wia, len)) {
		hwlog_err("%s:read adapt_file_id from dts fail\n", name);
		return -1;
	}
	for (i = 0; i < len; i++) {
		dyn_req->file_list[dyn_req->file_count] = wia[i];
		dyn_req->file_count++;
	}
	/* find hifi supported or not */
	if (of_property_read_u32(xhub_node, "hifi_support", &i) == 0) {
		if (i == 1) {
			hifi_supported = 1;
			hwlog_info("sensor get hifi support %d\n", i);
		}
	}

	if (of_property_read_string(xhub_node,
		"docom_step_counter", (const char **)&temp) == 0) {
		if (!strcmp("enabled", temp)) {
			g_config_on_ddr->reserved |= 1 << 0;
			hwlog_info("%s:docom_step_counter status is %s\n", __func__, temp);
		}
	}

	if (of_property_read_string(xhub_node,
		"homo_activity", (const char **)&temp) == 0) {
		if (!strcmp("enabled", temp)) {
			g_config_on_ddr->reserved |= 1 << 1;
			hwlog_info("%s:homo_activity status is %s\n", __func__, temp);
		}
	}

	return 0;
}

static void find_hall_prop_u32(struct device_node *xhub_node)
{
	unsigned int i;

	/* find number of the hall sensor */
	if (!of_property_read_u32(xhub_node, "hall_number", &i)) {
		hall_number = i;
		hwlog_info("sensor get hall number %d\n", hall_number);
	}

	if (!of_property_read_u32(xhub_node, "hall_sen_type", &i)) {
		hall_sen_type = i;
		hwlog_info("sensor get hall sensor type %d\n", i);
		if (hall_sen_type == HUB_FOLD_HALL_TYPE)
			all_ap_sensor_operations[TAG_EXT_HALL].work_on_ap = false;
	} else {
		hall_sen_type = 0; /* set hall sen type to default type */
	}

	if (!of_property_read_u32(xhub_node, "is_support_hall_hishow", &i)) {
		support_hall_hishow = i;
		hwlog_info("sensor get support_hall_hishow: %d\n", support_hall_hishow);
	}

	if (!of_property_read_u32(xhub_node, "hall_hishow_value", &i)) {
		hall_hishow_value = (uint32_t)i;
		hwlog_info("sensor get hall_hishow_value: %#x\n", hall_hishow_value);
	}

	if (!of_property_read_u32(xhub_node, "is_support_hall_pen", &i)) {
		support_hall_pen = i;
		hwlog_info("sensor get support_hall_pen: %d\n", support_hall_pen);
	}

	if (!of_property_read_u32(xhub_node, "hall_pen_value", &i)) {
		hall_pen_value = (uint32_t)i;
		hwlog_info("sensor get hall_pen_value: %#x\n", hall_pen_value);
	}

	if (!of_property_read_u32(xhub_node, "is_support_hall_keyboard", &i)) {
		support_hall_keyboard = i;
		hwlog_info("sensor get support_hall_keyboard: %d\n", support_hall_keyboard);
	}

	if (!of_property_read_u32(xhub_node, "hall_keyboard_value", &i)) {
		hall_keyboard_value = (uint32_t)i;
		hwlog_info("sensor get hall_keyboard_value: %#x\n", hall_keyboard_value);
	}

	if (!of_property_read_u32(xhub_node, "is_support_hall_lightstrap", &i)) {
		support_hall_lightstrap = i;
		hwlog_info("sensor get is_support_hall_lightstrap: %d\n", support_hall_lightstrap);
	}

	if (!of_property_read_u32(xhub_node, "hall_lightstrap_value", &i)) {
		hall_lightstrap_value = (uint32_t)i;
		hwlog_info("sensor get hall_lightstrap_value: %u\n", hall_lightstrap_value);
	}
}

static int get_hall_config_from_dts(void)
{
	struct device_node *xhub_node = NULL;

	xhub_node = of_find_compatible_node(NULL, NULL, "huawei,xhub");
	if (xhub_node == NULL) {
		hwlog_err("%s, can't find node sensorhub\n", __func__);
		return -1; /* get huawei sensorhub node fail */
	}

	find_hall_prop_u32(xhub_node);

	return 0;
}

static void swap1(uint16_t *left, uint16_t *right)
{
	uint16_t temp;

	temp = *left;
	*left = *right;
	*right = temp;
}

/* delete the repeated file id by map table */
static uint8_t check_file_list(uint8_t file_count, uint16_t *file_list)
{
	int i, j, k;

	if ((file_count == 0) || (!file_list)) {
		hwlog_err("%s, val invalid\n", __func__);
		return 0;
	}

	for (i = 0; i < file_count; i++) {
		for (j = i + 1; j < file_count; j++) {
			if (file_list[i] == file_list[j]) {
				file_count -= 1;
				for (k = j; k < file_count; k++)
					file_list[k] = file_list[k + 1];
				j -= 1;
			}
		}
	}

	for (i = 0; i < file_count; i++) {
		for (j = 1; j < file_count - i; j++) {
			if (file_list[j - 1] > file_list[j])
				swap1(&file_list[j - 1], &file_list[j]);
		}
	}
	return file_count;
}

static int get_adapt_sensor_list_id(void)
{
	u32 wia[ADAPT_SENSOR_LIST_NUM] = { 0 };
	struct property *prop = NULL;
	unsigned int i;
	unsigned int len;
	struct device_node *xhub_node = NULL;
	const char *name = "adapt_sensor_list_id";

	xhub_node = of_find_compatible_node(NULL, NULL, "huawei,xhub");
	if (!xhub_node) {
		hwlog_err("%s, can't find node sensorhub\n", __func__);
		return -1;
	}
	prop = of_find_property(xhub_node, name, NULL);
	if (!prop) {
		hwlog_err("%s! prop is NULL\n", __func__);
		return -EINVAL;
	}
	if (!prop->value) {
		hwlog_err("%s! prop->value is NULL\n", __func__);
		return -ENODATA;
	}
	len = prop->length / 4;
	if (of_property_read_u32_array(xhub_node, name, wia, len)) {
		hwlog_err("%s:read adapt_sensor_list_id from dts fail\n", name);
		return -1;
	}
	for (i = 0; i < len; i++) {
		sensorlist[sensorlist[0] + 1] = (uint16_t)wia[i];
		sensorlist[0]++;
	}
	return 0;
}

static int get_sensors_id_from_dts(void)
{
	struct device_node *xhub_node = NULL;

	xhub_node = of_find_compatible_node(NULL, NULL, "huawei,xhub");
	if (!xhub_node) {
		hwlog_err("%s, can't find node sensorhub\n", __func__);
		return -1;
	}

	acc_get_sensors_id_from_dts(xhub_node);
	als_get_sensors_id_from_dts(xhub_node);
	cap_prox_get_sensors_id_from_dts(xhub_node);
	gyro_get_sensors_id_from_dts(xhub_node);
	mag_get_sensors_id_from_dts(xhub_node);

	if (get_hall_config_from_dts())
		hwlog_err("get hall config from dts fail\n");

	return 0;
}

static int send_dyn_to_mcu(void *buf, int len)
{
	write_info_t pkg_ap = { 0 };
	read_info_t pkg_mcu = { 0 };
	int ret;

	pkg_ap.tag = TAG_SYS;
	pkg_ap.cmd = CMD_SYS_DYNLOAD_REQ;
	pkg_ap.wr_buf = buf;
	pkg_ap.wr_len = len;

	if (g_xhub_state == XHUB_ST_RECOVERY || iom3_power_state == ST_SLEEP)
		ret = write_customize_cmd(&pkg_ap, NULL, false);
	else
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);

	if (ret) {
		hwlog_err("send file id to mcu fail,ret=%d\n", ret);
		return -1;
	}
	if (pkg_mcu.errno != 0) {
		hwlog_err("file id set fail\n");
		return -1;
	}

	return 0;
}

int send_fileid_to_mcu(void)
{
	int i;
	pkt_sys_dynload_req_t dynload_req;

	if (dyn_req->file_count) {
		hwlog_info("sensorhub after check, get dynload file id number = %d, fild id",
			dyn_req->file_count);
		for (i = 0; i < dyn_req->file_count; i++)
			hwlog_info("--%d", dyn_req->file_list[i]);
		hwlog_info("\n");
		dyn_req->file_flg = 0;
		if (send_dyn_to_mcu(&(dyn_req->file_flg),
		    dyn_req->file_count * sizeof(dyn_req->file_list[0]) +
		    sizeof(dyn_req->file_flg) + sizeof(dyn_req->file_count)))
			hwlog_err("%s send file_id to mcu failed\n", __func__);
	} else {
		hwlog_err("%s file_count = 0, not send file_id to mcu\n",
			__func__);
		return -EINVAL;
	}

	if (aux_req->file_count) {
		hwlog_info("sensorhub after check, get aux file id number = %d, aux file id and tag ",
			aux_req->file_count);
		for (i = 0; i < aux_req->file_count; i++)
			hwlog_info("--%d, %d", aux_req->file_list[2 * i],
				aux_req->file_list[2 * i + 1]);

		hwlog_info("\n");
		aux_req->file_flg = 1;
		if (send_dyn_to_mcu(&(aux_req->file_flg),
		    aux_req->file_count * sizeof(aux_req->file_list[0]) * 2 +
		    sizeof(aux_req->file_flg) + sizeof(aux_req->file_count)))
			hwlog_err("%s send aux file_id to mcu failed\n", __func__);
	} else {
		hwlog_err("%s aux count=0,not send file_id to mcu\n", __func__);
	}
	memset(&dynload_req, 0, sizeof(pkt_sys_dynload_req_t));
	dynload_req.file_flg = 2;

	return send_dyn_to_mcu(&(dynload_req.file_flg),
			sizeof(pkt_sys_dynload_req_t) - sizeof(pkt_header_t));
}

static int get_adapt_id_and_send(void)
{
	int ret, i;

	ret = get_adapt_file_id_for_dyn_load();
	if (ret < 0)
		hwlog_err("get_adapt_file_id_for_dyn_load() failed!\n");

	hwlog_info("get file id number = %d\n", dyn_req->file_count);

	ret = get_adapt_sensor_list_id();
	if (ret < 0)
		hwlog_err("get_adapt_sensor_list_id() failed\n");

	sensorlist[0] = check_file_list(sensorlist[0], &sensorlist[1]);
	if (sensorlist[0] > 0) {
		hwlog_info("sensorhub after check, get sensor list id number = %d, list id: ",
			sensorlist[0]);
		for (i = 0; i < sensorlist[0]; i++)
			hwlog_info("--%d", sensorlist[i + 1]);
		hwlog_info("\n");
	} else {
		hwlog_err("%s list num = 0, not send file_id to mcu\n", __func__);
		return -EINVAL;
	}
	dyn_req->file_count =
		check_file_list(dyn_req->file_count, dyn_req->file_list);

	return send_fileid_to_mcu();
}

int detect_disable_sample_task_prop(struct device_node *dn, uint32_t *value)
{
	int ret;

	ret = of_property_read_u32(dn, "disable_sample_task", value);
	if (ret)
		return -1;

	return 0;
}

int get_combo_bus_tag(const char *bus, uint8_t *tag)
{
	obj_tag_t tag_tmp = TAG_END;

	if (!strcmp(bus, "i2c"))
		tag_tmp = TAG_I2C;
	else if (!strcmp(bus, "spi"))
		tag_tmp = TAG_SPI;
	else if (!strcmp(bus, "i3c"))
		tag_tmp = TAG_I3C;

	if (tag_tmp == TAG_END)
		return -1;
	*tag = (uint8_t)tag_tmp;
	return 0;
}

static int get_combo_prop(struct device_node *dn, struct detect_word *p_det_wd)
{
	int ret;
	struct property *prop = NULL;
	const char *bus_type = NULL;
	uint32_t u32_temp;

	/* combo_bus_type */
	ret = of_property_read_string(dn, "combo_bus_type", &bus_type);
	if (ret) {
		hwlog_err("%s: get bus_type err\n", __func__);
		return ret;
	}
	if (get_combo_bus_tag(bus_type, &p_det_wd->cfg.bus_type)) {
		hwlog_err("%s: bus_type(%s) err\n", __func__, bus_type);
		return -1;
	}

	/* combo_bus_num */
	ret = of_property_read_u32(dn, "combo_bus_num", &u32_temp);
	if (ret) {
		hwlog_err("%s: get combo_data err\n", __func__);
		return ret;
	}
	p_det_wd->cfg.bus_num = (uint8_t)u32_temp;

	/* combo_data */
	ret = of_property_read_u32(dn, "combo_data", &p_det_wd->cfg.data);
	if (ret) {
		hwlog_err("%s: get combo_data err\n", __func__);
		return ret;
	}

	/* combo_tx */
	prop = of_find_property(dn, "combo_tx", NULL);
	if (!prop) {
		hwlog_err("%s: get combo_tx err\n", __func__);
		return -1;
	}
	p_det_wd->tx_len = (uint32_t)prop->length;
	if (p_det_wd->tx_len > sizeof(p_det_wd->tx)) {
		hwlog_err("%s: get combo_tx_len %d too big\n",
			__func__, p_det_wd->tx_len);
		return -1;
	}
	of_property_read_u8_array(dn, "combo_tx", p_det_wd->tx,
		(size_t)prop->length);

	/* combo_rx_mask */
	prop = of_find_property(dn, "combo_rx_mask", NULL);
	if (!prop) {
		hwlog_err("%s: get combo_rx_mask err\n", __func__);
		return -1;
	}
	p_det_wd->rx_len = (uint32_t)prop->length;
	if (p_det_wd->rx_len > sizeof(p_det_wd->rx_msk)) {
		hwlog_err("%s: get rx_len %d too big\n", __func__,
			p_det_wd->rx_len);
		return -1;
	}
	of_property_read_u8_array(dn, "combo_rx_mask", p_det_wd->rx_msk,
		(size_t)prop->length);

	/* combo_rx_exp */
	prop = of_find_property(dn, "combo_rx_exp", NULL);
	if (!prop) {
		hwlog_err("%s: get combo_rx_exp err\n", __func__);
		return -1;
	}
	prop->length = (uint32_t)prop->length;
	if ((ssize_t)prop->length > sizeof(p_det_wd->rx_exp) ||
	    ((uint32_t)prop->length % p_det_wd->rx_len)) {
		hwlog_err("%s: rx_exp_len %d not available\n",
			__func__, prop->length);
		return -1;
	}
	p_det_wd->exp_n = (uint32_t)prop->length / p_det_wd->rx_len;
	of_property_read_u8_array(dn, "combo_rx_exp", p_det_wd->rx_exp,
		(size_t)prop->length);

	return 0;
}

static int xhub_i2c_i3c_detect(struct detect_device_para *device_para, char *device_name)
{
	int ret;

	if (device_para->tag == TAG_I3C) {
		ret = mcu_i3c_rw((uint8_t)device_para->i2c_bus_num, (uint8_t)device_para->i2c_address,
			(uint8_t *)&device_para->register_add, 1, device_para->detected_device_id, 1);
	} else if (device_para->tag == TAG_SPI) {
		union spi_ctrl ctrl;

		ret = mcu_spi_rw((uint8_t)device_para->i2c_bus_num, ctrl, (uint8_t *)&device_para->register_add, device_para->commu_len,
			device_para->detected_device_id, 1);
		hwlog_info("%s:spi read succ id:0x%x 0x%x 0x%x\n", __func__, device_para->detected_device_id[0],
			device_para->detected_device_id[1], device_para->detected_device_id[2]);
		memcpy(&device_para->device_id, device_para->detected_device_id, 4);
	} else {
		if ((unsigned int)device_para->register_add & 0xFF00) {
			device_para->register_add_len = 2;
			device_para->rx_len = 4;
		} else {
			device_para->register_add_len = 1;
			device_para->rx_len = 1;
		}
		ret = mcu_i2c_rw(TAG_I2C, (uint8_t)device_para->i2c_bus_num, (uint8_t)device_para->i2c_address,
			(uint8_t *)&device_para->register_add, device_para->register_add_len,
			device_para->detected_device_id, device_para->rx_len);
	}
	return ret;
}

static int detect_device(struct device_node *dn, char *device_name)
{
	int i;
	int ret;
	int len;
	u32 wia[10] = { 0 };
	u32 wia_mask = 0xffffffff;
	u32 temp = 0;
	struct property *prop = NULL;
	const char *bus_type = NULL;
	struct detect_device_para device_para = {
		.i2c_address = 0,
		.i2c_bus_num = 0,
		.register_add = 0,
		.commu_len = 0,
		.detected_device_id = { 0, },
		.tag = TAG_I2C,
	};

	if (of_property_read_u32(dn, "bus_number", &device_para.i2c_bus_num) ||
		of_property_read_u32(dn, "reg", &device_para.i2c_address) ||
		of_property_read_u32(dn, "chip_id_register", &device_para.register_add)) {
		hwlog_err("%s:read i2c bus_number or bus address or chip_id_register from dts fail\n",
			device_name);
		return -1;
	}

	prop = of_find_property(dn, "chip_id_value", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	len = prop->length / 4;

	if (of_property_read_u32(dn, "commu_len", &temp)) {
		hwlog_err("%s:read commu_len from dts fail\n", device_name);
	} else {
		device_para.commu_len = temp;
		hwlog_info("%s:read succ commu_len:%d\n", device_name, device_para.commu_len);
	}

	if (of_property_read_u32_array(dn, "chip_id_value", wia, len)) {
		hwlog_err("%s:read chip_id_value (id0=%x) from dts fail len=%d\n",
			device_name, wia[0], len);
		return -1;
	}

	if (of_property_read_u32(dn, "chip_id_mask", &temp)) {
		hwlog_err("%s:read err chip_id_mask:0x%x\n", __func__, wia_mask);
	} else {
		wia_mask = temp;
		hwlog_info("%s:read succ chip_id_mask:0x%x\n", __func__, wia_mask);
	}

	if (!of_property_read_string(dn, "bus_type", &bus_type)) {
		hwlog_info("detect bus type %s\n", bus_type);
		get_combo_bus_tag(bus_type, (uint8_t *)&device_para.tag);
	}

	ret = xhub_i2c_i3c_detect(&device_para, device_name);
	if (ret) {
		hwlog_err("%s:%s:send cmd to mcu fail,ret=%d\n",
			__func__, device_name, ret);
		return -1;
	}
	if (device_para.tag != TAG_SPI)
		memcpy(&device_para.device_id, device_para.detected_device_id, 4);

	if (!strncmp(device_name, "thermometer", strlen("thermometer")))
		device_para.device_id = (device_para.device_id & 0xff00) >> 8;

	for (i = 0; i < len; i++) {
		if (device_para.device_id == wia[i]) {
			hwlog_info("%s:i2c detect suc!chip_value:0x%x\n",
				device_name, device_para.device_id);
			return 0;
		} else if (temp != 0) {
			if ((device_para.device_id & wia_mask) == wia[i]) {
				hwlog_info("%s:i2c detect suc!chip_value_with_mask:0x%x\n",
					device_name, device_para.device_id & wia_mask);
				return 0;
			}
		}
	}
	hwlog_info("%s:i2c detect fail,chip_value:0x%x,len:%d\n",
		device_name, device_para.device_id, len);
	return -1;
}

int _device_detect(struct device_node *dn, int index,
	struct sensor_combo_cfg *p_succ_ret)
{
	int ret;
	struct detect_word det_wd;
	struct property *prop = of_find_property(dn, "combo_bus_type", NULL);
	uint8_t r_buf[MAX_TX_RX_LEN];
	uint32_t i, n;
	int rx_exp_p;

	memset(&det_wd, 0, sizeof(det_wd));

	if (prop) {
		ret = get_combo_prop(dn, &det_wd);
		if (ret) {
			hwlog_err("%s:get_combo_prop fail\n", __func__);
			return ret;
		}

		hwlog_info("%s: combo detect bus type %d; num %d; data %d; txlen %d; tx[0] 0x%x; rxLen %d; rxmsk[0] 0x%x; n %d; rxexp[0] 0x%x",
			__func__,
			det_wd.cfg.bus_type,
			det_wd.cfg.bus_num,
			det_wd.cfg.data,
			det_wd.tx_len,
			det_wd.tx[0],
			det_wd.rx_len,
			det_wd.rx_msk[0],
			det_wd.exp_n,
			det_wd.rx_exp[0]);

		ret = combo_bus_trans(&det_wd.cfg, det_wd.tx,
			det_wd.tx_len, r_buf, det_wd.rx_len);
		hwlog_info("combo_bus_trans ret is %d; rx 0x%x;\n", ret, r_buf[0]);

		if (ret >= 0) { /* success */
			ret = -1; /* fail first */
			/* check expect value */
			for (n = 0; n < det_wd.exp_n; n++) {
				for (i = 0; i < det_wd.rx_len;) {
					rx_exp_p = n * det_wd.rx_len + i;
					/* check value */
					if ((r_buf[i] & det_wd.rx_msk[i]) !=
					     det_wd.rx_exp[rx_exp_p]) {
						break;
					}
					i++;
				}
				/* get the success device */
				if (i == det_wd.rx_len) {
					ret = 0;
					hwlog_info("%s: %s detect succ;\n", __func__,
						sensor_manager[index].sensor_name_str);
					break;
				}
			}
		}
	} else {
		hwlog_info("%s: [%s] donot find combo prop\n", __func__,
			sensor_manager[index].sensor_name_str);
		ret = detect_device(dn, sensor_manager[index].sensor_name_str);
		if (!ret) {
			uint32_t i2c_bus_num = 0;
			uint32_t i2c_address = 0;
			uint32_t register_add = 0;

			if (of_property_read_u32(dn, "bus_number", &i2c_bus_num) ||
				of_property_read_u32(dn, "reg", &i2c_address) ||
				of_property_read_u32(dn, "chip_id_register", &register_add)) {
				hwlog_err("%s:read i2c bus_number or bus address or chip_id_register from dts fail\n",
					sensor_manager[index].sensor_name_str);
				return -1;
			}

			det_wd.cfg.bus_type = TAG_I2C;
			det_wd.cfg.bus_num = (uint8_t)i2c_bus_num;
			det_wd.cfg.i2c_address = (uint8_t)i2c_address;
		}
	}

	if (!ret)
		*p_succ_ret = det_wd.cfg;

	return ret;
}

static int device_detect(struct device_node *dn, int index)
{
	int ret = 0;
	struct sensor_combo_cfg cfg;
	struct sensor_combo_cfg *p_cfg = NULL;
	uint32_t disable;

	if (sensor_manager[index].detect_result == DET_SUCC)
		return -1;

	if (sensor_manager[index].sensor_id == CAP_PROX) {
		ret = is_cap_prox_shared_with_sar(dn);
		if (!ret)
			goto out;
	} else if (sensor_manager[index].sensor_id == MOTION) {
		hwlog_info("%s:motion detect always ok\n", __func__);
		goto out;
	} else if (sensor_manager[index].sensor_id == PS) {
		ret = ps_sensor_detect(dn, index, &cfg);
		goto out;
	} else if (sensor_manager[index].sensor_id == AIRPRESS) {
		ret = airpress_sensor_detect(dn, index, &cfg);
		goto out;
	}
	ret = _device_detect(dn, index, &cfg);
	if (!ret)
		memcpy((void *)sensor_manager[index].spara,
			(void *)&cfg, sizeof(cfg));
out:
	if (ret) {
		sensor_manager[index].detect_result = DET_FAIL;
	} else {
		/* check disable sensor task */
		p_cfg = (struct sensor_combo_cfg *)sensor_manager[index].spara;
		ret = detect_disable_sample_task_prop(dn, &disable);
		/* get disbale_sample_task property value */
		if (!ret)
			p_cfg->disable_sample_thread = (uint8_t)disable;
		sensor_manager[index].detect_result = DET_SUCC;
	}
	return sensor_manager[index].detect_result;
}

static int get_sensor_index(const char *name_buf, int len)
{
	int i;

	for (i = 0; i < SENSOR_MAX; i++) {
		if (len != strlen(sensor_manager[i].sensor_name_str))
			continue;
		if (!strncmp(name_buf, sensor_manager[i].sensor_name_str, len))
			break;
	}
	if (i >= SENSOR_MAX) {
		hwlog_err("get_sensor_detect_index fail\n");
		i = -1;
	}
	return i;
}

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
static void __set_hw_dev_flag(sensor_detect_list s_id)
{
/* detect current device successful, set the flag as present */
	switch (s_id) {
	case ACC:
	case ACC1:
		set_hw_dev_flag(DEV_DETECT_G_SENSOR);
		break;
	case MAG:
	case MAG1:
		set_hw_dev_flag(DEV_DETECT_COMPASS);
		break;
	case GYRO:
	case GYRO1:
		set_hw_dev_flag(DEV_DETECT_GYROSCOPE);
		break;
	case ALS:
	case ALS1:
	case ALS2:
	case PS:
		set_hw_dev_flag(DEV_DETECT_L_SENSOR);
		break;
	case AIRPRESS:
		set_hw_dev_flag(DEV_DETECT_AIRPRESS);
		break;
	case CAP_PROX:
	case CAP_PROX1:
	case MOTION:
	case CONNECTIVITY:
	default:
		hwlog_err("%s:err id =%d\n", __func__, s_id);
		break;
	}
}
#endif

static int extend_config_before_sensor_detect(struct device_node *dn, int index)
{
	int ret = 0;
	sensor_detect_list s_id = sensor_manager[index].sensor_id;

	switch (s_id) {
	case CONNECTIVITY:
		sensor_manager[index].detect_result = DET_SUCC;
		read_connectivity_data_from_dts(dn);
		break;
	case RPC:
		sensor_manager[index].detect_result = DET_SUCC;
		read_rpc_data_from_dts(dn);
		break;
	case SH_AOD:
		sensor_manager[index].detect_result = DET_SUCC;
		read_aod_data_from_dts(dn);
		break;
	case SOUND:
		sensor_manager[index].detect_result = DET_SUCC;
		read_sound_data_from_dts(dn);
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

static void extend_config_after_sensor_detect(struct device_node *dn, int index)
{
	sensor_detect_list s_id = sensor_manager[index].sensor_id;

	switch (s_id) {
	case ACC:
	case ACC1:
		read_acc_data_from_dts(dn, &sensor_manager[index]);
		break;
	case MAG:
	case MAG1:
		read_mag_data_from_dts(dn, &sensor_manager[index]);
		break;
	case GYRO:
	case GYRO1:
		read_gyro_data_from_dts(dn, &sensor_manager[index]);
		break;
	case ALS:
	case ALS1:
	case ALS2:
		read_als_data_from_dts(dn, &sensor_manager[index]);
		break;
	case PS:
		read_ps_data_from_dts(dn);
		break;
	case AIRPRESS:
		read_airpress_data_from_dts(dn);
		break;
	case CAP_PROX:
	case CAP_PROX1:
		read_capprox_data_from_dts(dn, &sensor_manager[index]);
		break;
	case MOTION:
		read_motion_data_from_dts(dn);
		break;
	case THERMOMETER:
		break;
	default:
		hwlog_err("%s:err id =%d\n", __func__, s_id);
		break;
	}
}

#ifdef CONFIG_HUAWEI_DSM
static void update_detectic_client_info(void)
{
	char sensor_name[DSM_MAX_IC_NAME_LEN] = { 0 };
	uint8_t i;
	int total_len = 0;

	for (i = 0; i < SENSOR_MAX; i++) {
		if (sensor_manager[i].detect_result == DET_FAIL) {
			total_len += strlen(sensor_manager[i].sensor_name_str);
			if (total_len < DSM_MAX_IC_NAME_LEN)
				strcat(sensor_name,
					sensor_manager[i].sensor_name_str);
		}
	}
	sensor_name[DSM_MAX_IC_NAME_LEN - 1] = '\0';
	hwlog_debug("%s %s.\n", __func__, sensor_name);
	dsm_sensorhub.ic_name = sensor_name;
	if (dsm_update_client_vendor_info(&dsm_sensorhub))
		hwlog_info("dsm_update_client_vendor_info failed\n");
}
#endif

static void dsm_detect_result_record(const char *detect_result)
{
	if (!dsm_client_ocuppy(shb_dclient)) {
		update_detectic_client_info();
		dsm_client_record(shb_dclient, "[%s]%s",
			__func__, detect_result);
		dsm_client_notify(shb_dclient,
			DSM_SHB_ERR_IOM7_DETECT_FAIL);
	} else {
		hwlog_info("%s:dsm_client_ocuppy fail\n", __func__);
		dsm_client_unocuppy(shb_dclient);
		if (!dsm_client_ocuppy(shb_dclient)) {
			update_detectic_client_info();
			dsm_client_record(shb_dclient, "[%s]%s",
				__func__, detect_result);
			dsm_client_notify(shb_dclient,
				DSM_SHB_ERR_IOM7_DETECT_FAIL);
		}
	}
}

static uint8_t check_detect_result(detect_mode mode)
{
	int i;
	uint8_t detect_fail_num = 0;
	uint8_t  result;
	int total_len = 0;
	char detect_result[MAX_STR_SIZE] = { 0 };
	const char *sf = " detect fail!";

	for (i = 0; i < SENSOR_MAX; i++) {
		result = sensor_manager[i].detect_result;
		if (result == DET_FAIL) {
			detect_fail_num++;
			total_len += strlen(sensor_manager[i].sensor_name_str);
			total_len += 2;
			if (total_len < MAX_STR_SIZE) {
				strcat(detect_result,
					sensor_manager[i].sensor_name_str);
				strcat(detect_result, "  ");
			}
			hwlog_info("%s :  %s detect fail\n",
				__func__, sensor_manager[i].sensor_name_str);
		} else if (result == DET_SUCC) {
			hwlog_info("%s :  %s detect success\n",
				__func__, sensor_manager[i].sensor_name_str);
			if (i == GYRO)
				gyro_detect_flag = 1;
		}
	}

	if (detect_fail_num > 0) {
		s_redetect_state.need_redetect_sensor = 1;
		total_len += strlen(sf);
		if (total_len < MAX_STR_SIZE)
			strcat(detect_result, sf);

#ifdef CONFIG_HUAWEI_DSM
		if (mode == BOOT_DETECT_END)
			dsm_detect_result_record(detect_result);
#endif
	} else {
		s_redetect_state.need_redetect_sensor = 0;
	}

	if ((detect_fail_num < s_redetect_state.detect_fail_num) &&
	    (mode == REDETECT_LATER)) {
		s_redetect_state.need_recovery = 1;
		hwlog_info("%s : %u sensors detect success after redetect\n",
			__func__, s_redetect_state.detect_fail_num - detect_fail_num);
	}
	s_redetect_state.detect_fail_num = detect_fail_num;
	return detect_fail_num;
}

static void show_last_detect_fail_sensor(void)
{
	int i;
	uint8_t result;

	for (i = 0; i < SENSOR_MAX; i++) {
		result = sensor_manager[i].detect_result;
		if (result == DET_FAIL)
			hwlog_err("last detect fail sensor: %s\n",
				sensor_manager[i].sensor_name_str);
	}
}

static void redetect_failed_sensors(detect_mode mode)
{
	int index;
	char *sensor_ty = NULL;
	char *sensor_st = NULL;
	struct device_node *dn = NULL;
	const char *st = "disabled";

	for_each_node_with_property(dn, "sensor_type") {
		/* sensor type */
		if (of_property_read_string(dn, "sensor_type",
		    (const char **)&sensor_ty)) {
			hwlog_err("redetect get sensor type fail\n");
			continue;
		}
		index = get_sensor_index(sensor_ty, strlen(sensor_ty));
		if (index < 0) {
			hwlog_err("redetect get sensor index fail\n");
			continue;
		}
		/* sensor status:ok or disabled */
		if (of_property_read_string(dn, "status",
		    (const char **)&sensor_st)) {
			hwlog_err("redetect get sensor status fail\n");
			continue;
		}
		if (!strcmp(st, sensor_st)) {
			hwlog_info("%s : sensor %s status is %s\n",
				__func__, sensor_ty, sensor_st);
			continue;
		}
		if (device_detect(dn, index) != DET_SUCC)
			continue;

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
		__set_hw_dev_flag(sensor_manager[index].sensor_id);
#endif

		extend_config_after_sensor_detect(dn, index);
	}
	check_detect_result(mode);
}

static void sensor_detect_exception_process(uint8_t result)
{
	int i;

	if (result > 0) {
		for (i = 0; i < SENSOR_DETECT_RETRY; i++) {
			hwlog_info("%s :  %d redect sensor after detect all sensor, fail sensor num  %d\n",
				__func__, i, s_redetect_state.detect_fail_num);
			if (s_redetect_state.detect_fail_num > 0)
				redetect_failed_sensors(DETECT_RETRY + i);
		}
	}
}

static int get_phone_type_send(void)
{
	u32 temp[PHONE_TYPE_LIST] = { 0 };
	struct device_node *xhub_node = NULL;

	xhub_node = of_find_compatible_node(NULL, NULL, "huawei,xhub");
	if (!xhub_node) {
		hwlog_err("%s, can't find node sensorhub\n", __func__);
		return -1;
	}

	if (of_property_read_u32_array(
		xhub_node, "phone_type_info", temp, PHONE_TYPE_LIST) != 0) {
		hwlog_err("read phone type info from dts fail\n");
		return -1;
	}

	g_config_on_ddr->phone_type_info[0] = temp[0];
	g_config_on_ddr->phone_type_info[1] = temp[1];
	hwlog_info("%s: phone type is %d %d\n",
		__func__, g_config_on_ddr->phone_type_info[0],
		g_config_on_ddr->phone_type_info[1]);

	return 0;
}

static void init_sensors_cfg_data_each_node(void)
{
	int ret;
	int index;
	char *sensor_ty = NULL;
	char *sensor_st = NULL;
	struct device_node *dn = NULL;
	const char *st = "disabled";

	for_each_node_with_property(dn, "sensor_type") {
		/* sensor type */
		ret = of_property_read_string(dn, "sensor_type",
			(const char **)&sensor_ty);
		if (ret) {
			hwlog_err("get sensor type fail ret=%d\n", ret);
			continue;
		}
		hwlog_info("%s : get sensor type %s\n", __func__, sensor_ty);
		index = get_sensor_index(sensor_ty, strlen(sensor_ty));
		if (index < 0) {
			hwlog_err("get sensor index fail ret=%d\n", ret);
			continue;
		}
		if (sensor_manager[index].sensor_id == CAP_PROX)
			read_cap_prox_info(dn); /* for factory sar */

		if (sensor_manager[index].sensor_id == CAP_PROX1)
			read_cap_prox1_info(dn); /* for factory sar */

		/* sensor status:ok or disabled */
		ret = of_property_read_string(dn, "status",
			(const char **)&sensor_st);
		if (ret) {
			hwlog_err("get sensor status fail ret=%d\n", ret);
			continue;
		}

		ret = strcmp(st, sensor_st);
		if (!ret) {
			hwlog_info("%s : sensor %s status is %s\n", __func__,
				sensor_ty, sensor_st);
			continue;
		}
		if (!extend_config_before_sensor_detect(dn, index))
			continue;

		hwlog_info("%s : sensorhub %s detect\n", __func__, sensor_manager[index].sensor_name_str);

		ret = device_detect(dn, index);
		if (ret != DET_SUCC)
			continue;

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
		__set_hw_dev_flag(sensor_manager[index].sensor_id);
#endif

		extend_config_after_sensor_detect(dn, index);
	}
}

int init_sensors_cfg_data_from_dts(void)
{
	int i;
	uint8_t sensor_detect_result;

	get_phone_type_send(); // get phone type info

	memset(&sensorlist_info, 0, SENSOR_MAX * sizeof(struct sensorlist_info));
	/* init sensorlist_info struct array */
	for (i = 0; i < SENSOR_MAX; i++) {
		sensorlist_info[i].version = -1;
		sensorlist_info[i].max_range = -1;
		sensorlist_info[i].resolution = -1;
		sensorlist_info[i].power = -1;
		sensorlist_info[i].min_delay = -1;
		sensorlist_info[i].max_delay = -1;
		sensorlist_info[i].fifo_reserved_event_count = 0xFFFFFFFF;
		sensorlist_info[i].fifo_max_event_count = 0xFFFFFFFF;
		sensorlist_info[i].flags = 0xFFFFFFFF;
	}

	init_sensors_cfg_data_each_node();

	sensor_detect_result = check_detect_result(BOOT_DETECT);
	sensor_detect_exception_process(sensor_detect_result);

	get_sensors_id_from_dts();
	if (get_adapt_id_and_send())
		return -EINVAL;

	return 0;
}

void send_parameter_to_mcu(sensor_detect_list s_id, int cmd)
{
	int ret;
	write_info_t pkg_ap = { 0 };
	read_info_t pkg_mcu = { 0 };
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	char buf[50] = { 0 };

	pkg_ap.tag = sensor_manager[s_id].tag;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	cpkt.subcmd = cmd;
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = sensor_manager[s_id].cfg_data_length + SUBCMD_LEN;
	memcpy(cpkt.para, sensor_manager[s_id].spara,
		sensor_manager[s_id].cfg_data_length);

	hwlog_info("%s g_xhub_state = %d,tag =%d ,cmd =%d\n",
		__func__, g_xhub_state, sensor_manager[s_id].tag, cmd);

	if (g_xhub_state == XHUB_ST_RECOVERY || iom3_power_state == ST_SLEEP)
		ret = write_customize_cmd(&pkg_ap, NULL, false);
	else
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);

	if (ret) {
		hwlog_err("send tag %d cfg data to mcu fail,ret=%d\n",
			pkg_ap.tag, ret);
	} else {
		if (pkg_mcu.errno != 0) {
			snprintf(buf, 50, "set %s cfg fail\n",
				sensor_manager[s_id].sensor_name_str);
			hwlog_err("%s\n", buf);
		} else {
			snprintf(buf, 50, "set %s cfg to mcu success\n",
				sensor_manager[s_id].sensor_name_str);
			hwlog_info("%s\n", buf);
			if (g_xhub_state != XHUB_ST_RECOVERY)
			#ifdef CONFIG_HUAWEI_HW_DEV_DCT
				__set_hw_dev_flag(s_id);
			#endif
		}
	}
}

static void register_priv_notifier(sensor_detect_list s_id)
{
	switch (s_id) {
	case GYRO:
		register_mcu_event_notifier(TAG_GYRO, CMD_CMN_CONFIG_REQ,
			gyro_data_from_mcu);
		break;
	case MAG:
		register_mcu_event_notifier(TAG_MAG, CMD_CMN_CONFIG_REQ,
			mag_data_from_mcu);
		break;
	case ALS:
		register_mcu_event_notifier(TAG_ALS, CMD_CMN_CONFIG_REQ,
			als_data_from_mcu);
		break;
	case ALS1:
		register_mcu_event_notifier(TAG_ALS1, CMD_CMN_CONFIG_REQ,
			als_data_from_mcu);
		break;
	case ALS2:
		register_mcu_event_notifier(TAG_ALS2, CMD_CMN_CONFIG_REQ,
			als_data_from_mcu);
		break;
	case PS:
		register_mcu_event_notifier(TAG_PS, CMD_CMN_CONFIG_REQ,
			ps_data_from_mcu);
		break;
	case GYRO1:
		register_mcu_event_notifier(TAG_GYRO1, CMD_CMN_CONFIG_REQ,
			gyro1_data_from_mcu);
		break;
	case MAG1:
		register_mcu_event_notifier(TAG_MAG1, CMD_CMN_CONFIG_REQ,
			mag1_data_from_mcu);
		break;
	case THERMOMETER:
		break;
	default:
		break;
	}
}

int sensor_set_cfg_data(void)
{
	int32_t al_tag = 0;
	int ret = 0;
	sensor_detect_list s_id;

	for (s_id = ACC; s_id < SENSOR_MAX; s_id++) {
		if (strlen(sensor_chip_info[s_id]) != 0) {
			send_parameter_to_mcu(s_id, SUB_CMD_SET_PARAMET_REQ);
			if (!als_get_tag_by_sensor_id(s_id, &al_tag)) {
				struct als_device_info *info = NULL;

				info = als_get_device_info(al_tag);
				if (info != NULL)
					info->send_para_flag = 1;
			}

			if (g_xhub_state != XHUB_ST_RECOVERY)
				register_priv_notifier(s_id);
		}
	}
	return ret;
}

static bool need_download_fw(uint8_t tag)
{
	return ((tag == TAG_KEY) || (tag == TAG_TOF) || (tag == TAG_CAP_PROX) || (tag == TAG_CAP_PROX1));
}

int sensor_set_fw_load(void)
{
	int val = 1;
	int ret;
	write_info_t pkg_ap;
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;
	sensor_detect_list s_id;

	hwlog_info("write  fw dload\n");
	for (s_id = ACC; s_id < SENSOR_MAX; s_id++) {
		if (strlen(sensor_chip_info[s_id]) != 0) {
			if (need_download_fw(sensor_manager[s_id].tag)) {
				pkg_ap.tag = sensor_manager[s_id].tag;
				pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
				cpkt.subcmd = SUB_CMD_FW_DLOAD_REQ;
				pkg_ap.wr_buf = &hd[1];
				pkg_ap.wr_len = sizeof(val) + SUBCMD_LEN;
				memcpy(cpkt.para, &val, sizeof(val));
				ret = write_customize_cmd(&pkg_ap, NULL, false);
				hwlog_info("write %d fw dload\n", sensor_manager[s_id].tag);
			}
		}
	}
	return 0;
}

int motion_set_cfg_data(void)
{
	int ret = 0;
	uint8_t app_config[HORIZONTAL_PICKUP_PARA_LEN] = {0};

	hwlog_info("write motion cmd\n");
	app_config[0] = MOTION_TYPE_PICKUP;
	app_config[1] = SUB_CMD_MOTION_HORIZONTAL_PICKUP_REQ;
	if (motion_data.motion_horizontal_pickup_flag) {
		app_config[2] = motion_data.motion_horizontal_pickup_flag;
		app_config[3] = motion_data.angle_gap;

		if (g_xhub_state == XHUB_ST_RECOVERY || iom3_power_state == ST_SLEEP)
			ret = send_app_config_cmd(TAG_MOTION, app_config, false);
		else
			ret = send_app_config_cmd(TAG_MOTION, app_config, true);

		if (ret) {
			hwlog_err("send motion %d cfg data to mcu fail,ret=%d\n",
				(int)app_config[0], ret);
			return RET_FAIL;
		}
		hwlog_info("write motion success. pickup_flag:%d,angle_gap:%d\n",
			(int)app_config[2], (int)app_config[3]);
	}
	return RET_SUCC;
}

static void redetect_sensor_work_handler(struct work_struct *wk)
{
	__pm_stay_awake(&sensor_rd);
	redetect_failed_sensors(REDETECT_LATER);

	if (s_redetect_state.need_recovery == 1) {
		s_redetect_state.need_recovery = 0;
		hwlog_info("%s: some sensor detect success after %d redetect, begin recovery\n",
			__func__, s_redetect_state.redetect_num);
		xhub_need_recovery(SH_FAULT_REDETECT);
	} else {
		hwlog_info("%s: no sensor redetect success\n", __func__);
	}
	__pm_relax(&sensor_rd);
}

void sensor_redetect_enter(void)
{
	if (g_xhub_state == XHUB_ST_NORMAL) {
		if (s_redetect_state.need_redetect_sensor == 1) {
			if (s_redetect_state.redetect_num < MAX_REDETECT_NUM) {
				queue_work(system_power_efficient_wq,
					&redetect_work);
				s_redetect_state.redetect_num++;
			} else {
				hwlog_info("%s: some sensors detect fail, but the max redetect num is over flow\n",
					__func__);
				show_last_detect_fail_sensor();
			}
		}
	}
}

void sensor_redetect_init(void)
{
	memset(&s_redetect_state, 0, sizeof(s_redetect_state));
	acc_detect_init(sensor_manager, SENSOR_MAX);
	airpress_detect_init(sensor_manager, SENSOR_MAX);
	als_detect_init(sensor_manager, SENSOR_MAX);
	cap_prox_detect_init(sensor_manager, SENSOR_MAX);
	gyro_detect_init(sensor_manager, SENSOR_MAX);
	mag_detect_init(sensor_manager, SENSOR_MAX);
	ps_detect_init(sensor_manager, SENSOR_MAX);
	wakeup_source_init(&sensor_rd, "xhub_redetect");
	INIT_WORK(&redetect_work, redetect_sensor_work_handler);
}
