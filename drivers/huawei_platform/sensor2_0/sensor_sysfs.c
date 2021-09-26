/*
 * sensor_sysfs.c
 *
 * code for sensor debug sysfs
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#include <linux/delay.h>
#include <linux/of.h>
#include <charger_type.h>
#include <linux/kthread.h>
#include <upmu_common.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/time.h>
#if defined CONFIG_MTK_CHARGER
#include <mt-plat/mtk_charger.h>
#include <chipset_common/hwpower/common_module/power_interface.h>
#include <linux/wait.h>
#endif

#include "sensor_scp.h"
#include "securec.h"
#include "sensor_para.h"
#include "scp_err_info.h"
#include "als_route.h"

#include <lcd_kit_core.h>
#include <dsm/dsm_pub.h>
#define pr_fmt(fmt) "[scp_sysfs] " fmt

#define RES_ERR       -1
#define DECIMAL       10
#define TIMEOUT       1000
#define LCD_NAME_LEN  20
#define ALL_INFO_SIZE       50
#define DMD_DELAY           2500
#define INFO_SIZE           5
#define SENSOR_SUPPORT_SIZE 6
#define SENSORHUB_BUF_SIZE  50
#define BUFFER_SIZE         1024
#define ALSPS_POWER_RESET_DELAY    1000
#define ALSPS_POWER_RESET_ENABLE   1
#define RECOVER_NORMAL_DATA_MODE   4
#define RAW_DATA_MODE              1
#define TAG_ACCEL           1
#define TAG_GYRO            2
#define TAG_ACCEL1          35
#define TAG_GYRO1           36

enum {
	CALI_FOOL_NULL,
	CALI_FOOL_USB,
};

enum {
	USB_PLUG_OUT,
	USB_PLUG_IN,
};

struct sensor_dmd {
	int sensor_id;
	char sensor_info[INFO_SIZE];
};

static const struct sensor_dmd sensor_dmd_comment[] = {
	{ SENSOR_TYPE_ACCELEROMETER,  "ACC " },
	{ SENSOR_TYPE_MAGNETIC_FIELD, "MAG " },
	{ SENSOR_TYPE_GYROSCOPE,     "GYRO " },
	{ SENSOR_TYPE_LIGHT,          "ALS " },
	{ SENSOR_TYPE_PROXIMITY,      "PRO " },
	{ SENSOR_TYPE_SAR,            "SAR " },
};

static struct dsm_client* dsm_sensor_dclient = NULL;
static char sensor_all_info[ALL_INFO_SIZE] = {0};
static int dmd_flag = 0;
static uint8_t als_cali_fool_type;
static struct class *sensors_class;
static struct scp_ctrl_t scp_sysfs;
static struct delayed_work dmd_worker;
static struct delayed_work alsps_reset_worker;
static unsigned int scp_sensor_delay[SENSOR_TYPE_SENSOR_MAX];
static bool rpc_motion_request;
static int fingersense_enabled;
int finger_scp_data_update_ready;
struct finger_para_t *fg_para_latch;
#if defined CONFIG_MTK_CHARGER
struct task_struct *g_mag_task;
struct charger_consumer *g_mag_consumer;
struct notifier_block g_charger_detect_notify;
wait_queue_head_t g_wait_event;
#endif
uint32_t g_mag_need_charger_curr;
uint32_t g_mag_open_status;
static uint32_t g_alsps_power_reset = 0;
static uint32_t g_gyro_timeout = TIMEOUT;
static uint32_t g_calibrate_method = 1;
static struct regulator *g_alsps_vio;
static struct als_para_t *g_als_data;
static struct timeval time_last = {
	.tv_sec = 0,
	.tv_usec = 0,
};
static int cur_data_type = 0;
static int stored_delay = 0;
static int delay_time = 1; /* at least 1ms */

#define USEC_PER_SEC                1000000
#define USEC_PER_MS                 1000
#define TIME_OFFSET                 500
#define MAX_FINGER_SENSE_DATA_CNT   128
#define FINGER_SENSE_MAX_SIZE       (MAX_FINGER_SENSE_DATA_CNT * sizeof(short))
#define MAX_LATCH_DATA_SIZE         1024
#define MAX_FINGER_RING_CNT         256
#define SENSOR_DELAY_LOW            1000000  /* 1 ms */
#define SENSOR_DELAY_DEFAULT        10000000 // 10 ms
#define PS_GESTURE_ON               1
#define PS_GESTURE_OFF              0
#define ALS_UNDER_TP_RAWDATA_LEN 4

#define DTS_COMP_LCD_KIT_PANEL_TYPE     "huawei,lcd_panel_type"

struct dsm_dev dsm_sensorhub = {
	.name = "dsm_sensorhub",
	.device_name = NULL,
	.ic_name = NULL,
	.module_name = NULL,
	.fops = NULL,
	.buff_size = BUFFER_SIZE,
};

static void alsps_reset_work(struct work_struct *work)
{
	unsigned int val = 0;
	unsigned int ret = 0;
	struct hf_manager_cmd cmd_in;

	g_alsps_vio = regulator_get(NULL, "vtp");
	if (IS_ERR(g_alsps_vio)) {
		pr_err("%s : regulator_get fail\n", __func__);
	} else {
		val = regulator_is_enabled(g_alsps_vio);
		regulator_put(g_alsps_vio);
	}

	pr_err("%s : regulator_is_enabled= %u\n", __func__, val);
	if (val == 0) { // camera is not work
		ret = pmic_config_interface(
			PMIC_RG_LDO_VIO28_EN_ADDR,
			0,
			PMIC_RG_LDO_VIO28_EN_MASK,
			PMIC_RG_LDO_VIO28_EN_SHIFT);
		mdelay(20); // delay 20ms for vio28 power down
		ret = pmic_config_interface(
			PMIC_RG_LDO_VIO28_EN_ADDR,
			1,
			PMIC_RG_LDO_VIO28_EN_MASK,
			PMIC_RG_LDO_VIO28_EN_SHIFT);
		mdelay(450); // delay 450ms for sensor POR
		cmd_in.sensor_type = SENSOR_TYPE_PROXIMITY;
		cmd_in.action = HF_MANAGER_SENSOR_CONFIG_CALI;
		cmd_in.data[0] = 0xA5; // magic number
		cmd_in.data[1] = 0xFF; // magic number
		scp_sensor_cfg_data(&cmd_in);
	} else {
		pr_err("%s : camera is working, do not disable vio28\n", __func__);
	}
}

void report_hub_dmd(uint32_t case_id, uint32_t sensor_id, char *context)
{
	if (context == NULL)
		return;

	pr_info("%s : case : %u sensor : %u name : %s\n", __func__, case_id, sensor_id, context);
	switch (case_id) {
	case ERR_CASE_UNKNOWN:
		if (g_alsps_power_reset == ALSPS_POWER_RESET_ENABLE &&
			sensor_id == ERR_SENSOR_ALS_PS) {
			INIT_DELAYED_WORK(&alsps_reset_worker, alsps_reset_work);
			schedule_delayed_work(&alsps_reset_worker,
				ALSPS_POWER_RESET_DELAY);
			pr_err("%s : alsps power reset send\n", __func__);
		}
		break;
	default:
		break;
	}
}

// here define selftest sysfs for special sensors, if need then add
static ssize_t store_selftest(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	int ret;
	unsigned long val = 0;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	if (kstrtoul(buf, DECIMAL, &val))
		return -EINVAL;

	pr_info("do self test: sensortype = %d, val= %lu!\n",
		data->sensor_type, val);
	scp_sysfs.delay = scp_sensor_delay[data->sensor_type];
	scp_sysfs.sensor_type = data->sensor_type;

	if (val != 1)
		return size;

	scp_sysfs.action = HF_MANAGER_SENSOR_ENABLE;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		return -EINVAL;

	scp_sysfs.action = HF_MANAGER_SENSOR_SELFTEST;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t show_selftest(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rst;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	rst = scp_sensor_test_result(data->sensor_type, SCP_SENSOR_SELFTEST);
	pr_info("selftest type = %d, rst= %d\n", data->sensor_type, rst);

	scp_sysfs.delay = scp_sensor_delay[data->sensor_type];
	scp_sysfs.sensor_type = data->sensor_type;
	scp_sysfs.action = HF_MANAGER_SENSOR_DISABLE;
	send_scp_ctrl_cmd(&scp_sysfs);

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", rst);
}

static DEVICE_ATTR(self_test, 0660, show_selftest, store_selftest);

static ssize_t show_mag_calibrate_method(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", g_calibrate_method);
}

static DEVICE_ATTR(calibrate_method, 0440, show_mag_calibrate_method, NULL);

static ssize_t show_selftest_timeout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", TIMEOUT);
}

static DEVICE_ATTR(self_test_timeout, 0440, show_selftest_timeout, NULL);

static ssize_t show_calibrate_timeout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", g_gyro_timeout);
}

static DEVICE_ATTR(calibrate_timeout, 0440, show_calibrate_timeout, NULL);

static void set_als_calibration_oder(uint8_t cali_oder)
{
	struct als_para_t *als_priv = NULL;

	als_priv = get_sensor_share_mem_addr(SHR_MEM_TYPE_ALS);
	if (als_priv == NULL) {
		pr_err("%s: get als para fail", __func__);
		return;
	}

	als_priv->cali_oder = cali_oder;
}

static void set_sar_custom_cmd(uint8_t cali_oder)
{
	int ret;

	ret = send_scp_custom_cmd(SENSOR_TYPE_SAR, cali_oder);
	if (ret)
		pr_err("%s : hf_client_find_sensor sar fail!\n", __func__);
}

static void set_sensor_calibration_oder(uint8_t cali_oder, int sensor_type)
{
	switch (sensor_type) {
	case SENSOR_TYPE_LIGHT:
		set_als_calibration_oder(cali_oder);
		break;

	case SENSOR_TYPE_SAR:
		set_sar_custom_cmd(cali_oder);
		break;

	default:
		pr_info("%s: sensor %d not support!\n", __func__, sensor_type);
		break;
	}
}

static int check_sensor_calibration_fool(int sensor_type)
{
	int ret;
	enum charger_type als_charger_type;

	if (sensor_type != SENSOR_TYPE_LIGHT)
		return 0;

	if (als_cali_fool_type == CALI_FOOL_USB) {
		als_charger_type = mt_get_charger_type();
		pr_info("%s : charger = %d\n", __func__, als_charger_type);

		if (als_charger_type != STANDARD_HOST) {
			pr_info("%s : device is not in test equipment\n", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static ssize_t store_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	enum charger_type als_charger_type;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	if (kstrtoul(buf, DECIMAL, &val))
		return -EINVAL;

	pr_info("%s : sensor type = %d, cmd = %lu!\n", __func__,
		data->sensor_type, val);

	ret = check_sensor_calibration_fool(data->sensor_type);
	if (ret < 0)
		return -EINVAL;

	set_sensor_calibration_oder((uint8_t)val, data->sensor_type);
	scp_sysfs.delay = scp_sensor_delay[data->sensor_type];
	scp_sysfs.sensor_type = data->sensor_type;
	scp_sysfs.action = HF_MANAGER_SENSOR_ENABLE;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		return -EINVAL;

	if (data->sensor_type == SENSOR_TYPE_LIGHT)
		msleep(350); // wait for als data steady

	scp_sysfs.action = HF_MANAGER_SENSOR_ENABLE_CALI;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t show_calibrate(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rst;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	rst = scp_sensor_test_result(data->sensor_type, SCP_SENSOR_CALI);
	pr_info("%s : sensor type = %d, rst= %d\n", __func__,
		data->sensor_type, rst);

	scp_sysfs.delay = scp_sensor_delay[data->sensor_type];
	scp_sysfs.sensor_type = data->sensor_type;
	scp_sysfs.action = HF_MANAGER_SENSOR_DISABLE;
	send_scp_ctrl_cmd(&scp_sysfs);

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", rst);
}

static DEVICE_ATTR(calibrate, 0660, show_calibrate, store_calibrate);

/*
 * Description: ps request telephone call status to enable acc_assist_ps
 *              value:1--enable ps_gesture
 *              value:0--disable ps_gesture
 */
void ps_telecall_status_change(unsigned long value)
{
	static bool last_state = false;
	bool en = false;
	int ret;

	pr_info("%s enter, value: %lld\n", __func__, value);
	if (value == PS_GESTURE_ON)
		en = true;
	else if (value == PS_GESTURE_OFF)
		en = false;
	if (last_state != en) {
		last_state = en;
		ret = scp_sensor_ctrl_enable(SENSOR_TYPE_RPC_MOTION, en);
		if (ret)
			pr_err("%s rpc_motion enable fail: %d\n", __func__, ret);
	}
}

static ssize_t show_rpc_motion_req(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!buf)
		return -EINVAL;
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", rpc_motion_request);
}

static ssize_t store_rpc_motion_req(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		pr_err("%s: rpc motion request val %lu invalid",
			__func__, value);

	pr_info("%s: rpc motion request val %lu\n", __func__, value);

	if ((value != 0) && (value != 1)) {
		pr_err("%s: set enable fail, invalid val\n", __func__);
		return size;
	}
	rpc_motion_request = value;
	ps_telecall_status_change(value);
	return size;
}
static DEVICE_ATTR(rpc_motion_req, 0660, show_rpc_motion_req, store_rpc_motion_req);

static ssize_t store_als_under_tp_calidata(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);
	return als_under_tp_calidata_store(data->sensor_type,
		dev, attr, buf, size);
}

/* return underscreen als to node file */
static ssize_t show_als_under_tp_calidata(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);
	return als_under_tp_calidata_show(data->sensor_type, dev, attr, buf);
}

static DEVICE_ATTR(set_als_under_tp_calidata, 0660,
	show_als_under_tp_calidata, store_als_under_tp_calidata);

static void get_als_data(void)
{
	g_als_data = get_sensor_share_mem_addr(SHR_MEM_TYPE_ALS);

	if (g_als_data == NULL)
		pr_info("%s: get als data fail\n", __func__);
}

static ssize_t store_als_under_tp_rawdata(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int i;

	if (g_als_data == NULL) {
		pr_info("%s: get als data fail\n", __func__);
		return size;
	}
	for (i = 0; i < ALS_UNDER_TP_RAWDATA_LEN; i++)
		pr_info("%s: g_als_data->als_rawdata[%d]: %d",
			__func__, i, g_als_data->als_rawdata[i]);
	return size;
}

static ssize_t show_als_under_tp_rawdata(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);

	if (g_als_data == NULL) {
		pr_info("%s: get als data fail\n", __func__);
		return -EINVAL;
	}
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d,%d,%d,%d\n",
		g_als_data->als_rawdata[0],
		g_als_data->als_rawdata[1],
		g_als_data->als_rawdata[2],
		g_als_data->als_rawdata[3]);
}

static DEVICE_ATTR(als_calibrate_under_tp, 0660,
	show_als_under_tp_rawdata, store_als_under_tp_rawdata);

static void update_last_time()
{
	do_gettimeofday(&time_last);
}

static unsigned long caculate_time_difference(struct timeval *time_now, struct timeval *time_last)
{
	unsigned long sec_difference = time_now->tv_sec - time_last->tv_sec;
	return sec_difference * USEC_PER_SEC + time_now->tv_usec - time_last->tv_usec;
}

static void control_get_data_frequency()
{
	unsigned long usec_difference;
	struct timeval time_now;
	unsigned long usec_to_delay = delay_time * USEC_PER_MS - TIME_OFFSET;
	do_gettimeofday(&time_now);
	if (time_last.tv_sec == 0)
		usec_difference = usec_to_delay;
	else
		usec_difference = caculate_time_difference(&time_now, &time_last);
	if (usec_difference < usec_to_delay)
		udelay(usec_to_delay - usec_difference);
	update_last_time();
}

static int send_command_with_retry(int tag, struct custom_cmd *cmd)
{
	int ret;
	int retry = 3;
	while(retry-- > 0) {
		ret = send_scp_common_cmd(tag, cmd);
		if (ret == 0)
			return ret;
	}
	pr_err("%s fail type=%d have no retry chane\n", __func__, tag);
	return ret;
}

static ssize_t show_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int x_value;
	int y_value;
	int z_value;
	struct custom_cmd cmd;
	/* data comback in [2 - 4] */
	int start_offset = 2;
	/* the offset of value */
	int x_offset = 0;
	int y_offset = 1;
	int z_offset = 2;
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	control_get_data_frequency();
	CHECK_SENSOR_COOKIE(data);
	cmd.data[0] = CUST_CMD_GET_DATA;
	cmd.data[1] = 0;
	if (send_command_with_retry(data->sensor_type, &cmd))
		return RES_ERR;
	/* 10 is magnification factor */
	x_value = cmd.data[start_offset + x_offset] / 10;
	y_value = cmd.data[start_offset + y_offset] / 10;
	z_value = cmd.data[start_offset + z_offset] / 10;
	return (ssize_t)snprintf_s(buf, SENSORHUB_BUF_SIZE,
		SENSORHUB_BUF_SIZE - 1, "%d\t%d\t%d\t\n", x_value, y_value, z_value);
}

static ssize_t store_get_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(get_data, 0660, show_get_data, store_get_data);

static struct attribute *acc_sensor_attrs[] = {
	&dev_attr_self_test.attr,
	&dev_attr_self_test_timeout.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_calibrate_timeout.attr,
	&dev_attr_get_data.attr,
	NULL,
};

static const struct attribute_group acc_sensor_attrs_grp = {
	.attrs = acc_sensor_attrs,
};

static struct attribute *ps_sensor_attrs[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_calibrate_timeout.attr,
	NULL,
};

static const struct attribute_group ps_sensor_attrs_grp = {
	.attrs = ps_sensor_attrs,
};

static struct attribute *als_sensor_attrs[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_calibrate_timeout.attr,
	&dev_attr_set_als_under_tp_calidata.attr,
	&dev_attr_als_calibrate_under_tp.attr,
	NULL,
};

static const struct attribute_group als_sensor_attrs_grp = {
	.attrs = als_sensor_attrs,
};

static struct attribute *mag_sensor_attrs[] = {
	&dev_attr_self_test.attr,
	&dev_attr_self_test_timeout.attr,
	&dev_attr_calibrate_method.attr,
	NULL,
};

static const struct attribute_group mag_sensor_attrs_grp = {
	.attrs = mag_sensor_attrs,
};

static struct attribute *gyro_sensor_attrs[] = {
	&dev_attr_self_test.attr,
	&dev_attr_self_test_timeout.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_calibrate_timeout.attr,
	NULL,
};

static const struct attribute_group gyro_sensor_attrs_grp = {
	.attrs = gyro_sensor_attrs,
};

static struct attribute *sar_sensor_attrs[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_calibrate_timeout.attr,
	NULL,
};

static const struct attribute_group sar_sensor_attrs_grp = {
	.attrs = sar_sensor_attrs,
};

static struct attribute *rpc_sensor_attrs[] = {
	&dev_attr_rpc_motion_req.attr,
	NULL,
};

static const struct attribute_group rpc_sensor_attrs_grp = {
	.attrs = rpc_sensor_attrs,
};

static struct sensor_cookie all_sensors[] = {
	{
		.sensor_type = SENSOR_TYPE_ACCELEROMETER,
		.name = "acc_sensor",
		.attrs_group = &acc_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_PROXIMITY,
		.name = "ps_sensor",
		.attrs_group = &ps_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_LIGHT,
		.name = "als_sensor",
		.attrs_group = &als_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_MAGNETIC_FIELD,
		.name = "mag_sensor",
		.attrs_group = &mag_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_GYROSCOPE,
		.name = "gyro_sensor",
		.attrs_group = &gyro_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_SAR,
		.name = "cap_prox_sensor",
		.attrs_group = &sar_sensor_attrs_grp,
	},
	{
		.sensor_type = SENSOR_TYPE_RPC_MOTION,
		.name = "rpc_sensor",
		.attrs_group = &rpc_sensor_attrs_grp,
	},
};

// here define sensor name info for every phy sensor
static ssize_t sensor_show_ACC_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_ACCELEROMETER, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t sensor_show_MAG_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_MAGNETIC_FIELD, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t sensor_show_GYRO_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_GYROSCOPE, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t sensor_show_ALS_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_LIGHT, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t sensor_show_SAR_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_SAR, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t sensor_show_PS_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_info sensor_name;
	int ret;

	CHECK_NULL_ERR(buf);
	ret = scp_get_sensor_info(SENSOR_TYPE_PROXIMITY, &sensor_name);

	if (ret < 0) {
		pr_err("%s : fail\n", __func__);
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "unsupport\n");
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s\n", sensor_name.name);
}

static ssize_t store_fg_sense_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;

	if (!buf)
		return -EINVAL;

	if (strict_strtoul(buf, 10, &val)) {
		pr_err("%s finger enable val %lu invalid", __func__, val);
		return -EINVAL;
	}

	pr_info("%s: finger sense enable val %ld\n", __func__, val);
	if ((val != 0) && (val != 1)) {
		pr_err("%s finger sense set enable fail, invalid val\n",
			__func__);
		return size;
	}

	if (fingersense_enabled == val) {
		pr_err("%s finger sense already current state\n", __func__);
		return size;
	}

	ret = scp_sensor_ctrl_enable(SENSOR_TYPE_FINGER_SENSE, val);
	if (ret) {
		pr_err("%s finger sense enable fail: %d\n", __func__, ret);
		return size;
	}
	fingersense_enabled = val;
	return size;
}

static ssize_t show_fg_sensor_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!buf)
		return -EINVAL;

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1,
		"%d\n", fingersense_enabled);
}

static ssize_t store_fg_req_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	// consider vibrator
	pr_info("finger %s\n", __func__);
	if (!fingersense_enabled) {
		pr_err("%s finger sense not enable, dont req data\n", __func__);
		return size;
	}
	finger_scp_data_update_ready = 0;

	return size;
}

static void get_finger_share_mem_addr(void)
{
	static bool shr_finger_mem_ready;

	if (!shr_finger_mem_ready) {
		fg_para_latch = get_sensor_share_mem_addr(SHR_MEM_TYPE_FINGER);
		if (!fg_para_latch) {
			pr_err("finger share dram not ready\n");
			return;
		} else {
			pr_info("finger share dram ready\n");
			shr_finger_mem_ready = true;
		}
	}
}

static ssize_t show_fg_data_ready(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!buf)
		return -EINVAL;

	get_finger_share_mem_addr();
	if (fg_para_latch)
		finger_scp_data_update_ready = fg_para_latch->finger_ready;

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1,
		"%d\n", finger_scp_data_update_ready);
}

static ssize_t show_fg_latch_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int size = 0;
	int tail;
	int head_pos;
	int first_half_size;
	int sec_half_size;
	errno_t rt;

	get_finger_share_mem_addr();
	if (!fg_para_latch || !buf) {
		pr_err("%s: finger share memory pointer err\n", __func__);
		return size;
	}

	size = sizeof(fg_para_latch->fg_data) > FINGER_SENSE_MAX_SIZE ?
		FINGER_SENSE_MAX_SIZE : sizeof(fg_para_latch->fg_data);

	if ((!finger_scp_data_update_ready) || (!fingersense_enabled)) {
		pr_err("%s:fingersense zaxix not ready %d or not enable %d\n",
			__func__,
			finger_scp_data_update_ready, fingersense_enabled);
		return size;
	}

	tail = fg_para_latch->tail;
	pr_info("%s finger ring tail = %d\n", __func__, tail);
	if ((tail >= MAX_FINGER_RING_CNT) || (tail < 0)) {
		pr_err("%s : ring buffer tail pos err", __func__);
		return size;
	}

	if ((tail >= (MAX_FINGER_SENSE_DATA_CNT - 1)) &&
		(tail <= MAX_FINGER_RING_CNT - 1)) {
		head_pos = tail - 127; // calc 128 ring buf start size
		rt = memcpy_s(buf, FINGER_SENSE_MAX_SIZE,
			(char *)&fg_para_latch->fg_data[head_pos], size);
	} else {
		head_pos = MAX_FINGER_SENSE_DATA_CNT + 1 + tail;
		first_half_size = (MAX_FINGER_SENSE_DATA_CNT - tail - 1) * sizeof(short);
		rt = memcpy_s(buf, FINGER_SENSE_MAX_SIZE,
			(char *)&fg_para_latch->fg_data[head_pos],
			first_half_size);
		if (rt != EOK) {
			pr_err("finger copy err\n");
			return size;
		}
		sec_half_size = (tail + 1) * sizeof(short);
		rt = memcpy_s(buf + first_half_size,
			FINGER_SENSE_MAX_SIZE - first_half_size,
			(char *)&fg_para_latch->fg_data[0], sec_half_size);
	}
	if (rt != EOK)
		pr_err("finger data copy err\n");

	return size;
}

ssize_t store_set_data_type(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int sensor_type;
	struct custom_cmd cmd;
	int32_t set_data_type[2] = { 0 };
	const int32_t *set_type = NULL;
	set_type = (const int32_t *)buf;
	if (size == sizeof(set_data_type)) {
		set_data_type[0] = *set_type;
		set_data_type[1] = *(set_type + 1);
	} else {
		pr_err("%s:size %lu is not equal to 8\n",  __func__, size);
		return RES_ERR;
	}

	/* set_data_type[0]: 1.ACC 2.GYRO 35.ACC1 36.GYRO1 */
	/* set_data_type[1]: 1.raw_data 2.cali_data 4.nor_data */
	pr_info("%s: data type tag is %d, type is %d\n",
		__func__, set_data_type[0], set_data_type[1]);
	if (set_data_type[1] < 1 || set_data_type[1] > 4) {
		pr_err("%s:set data type is fail, invalid val\n", __func__);
		return RES_ERR;
	}
	if (set_data_type[0] == TAG_ACCEL || set_data_type[0] == TAG_ACCEL1) {
		sensor_type = SENSOR_TYPE_ACCELEROMETER;
	} else if (set_data_type[0] == TAG_GYRO || set_data_type[0] == TAG_GYRO1) {
		sensor_type = SENSOR_TYPE_GYROSCOPE;
	} else {
		pr_err("%s: sensor_type illegal\n", __func__);
		return RES_ERR;
	}
	cur_data_type = set_data_type[1];
	cmd.data[0] = CUST_CMD_SET_DATA_TYPE;
	/* carry 2 data */
	cmd.data[1] = 2;
	cmd.data[2] = set_data_type[0];
	cmd.data[3] = set_data_type[1];
	if (send_command_with_retry(sensor_type, &cmd))
		return RES_ERR;
	return size;
}

static DEVICE_ATTR(acc_info, 0444, sensor_show_ACC_info, NULL);
static DEVICE_ATTR(mag_info, 0444, sensor_show_MAG_info, NULL);
static DEVICE_ATTR(gyro_info, 0444, sensor_show_GYRO_info, NULL);
static DEVICE_ATTR(als_info, 0444, sensor_show_ALS_info, NULL);
static DEVICE_ATTR(sar_info, 0444, sensor_show_SAR_info, NULL);
static DEVICE_ATTR(ps_info, 0444, sensor_show_PS_info, NULL);
static DEVICE_ATTR(set_fingersense_enable, 0660, show_fg_sensor_enable,
	store_fg_sense_enable);
static DEVICE_ATTR(fingersense_req_data, 0220, NULL, store_fg_req_data);
static DEVICE_ATTR(fingersense_data_ready, 0440, show_fg_data_ready, NULL);
static DEVICE_ATTR(fingersense_latch_data, 0440, show_fg_latch_data, NULL);
static DEVICE_ATTR(set_data_type, 0220, NULL, store_set_data_type);

static struct attribute *sensor_attributes[] = {
	&dev_attr_acc_info.attr,
	&dev_attr_mag_info.attr,
	&dev_attr_gyro_info.attr,
	&dev_attr_als_info.attr,
	&dev_attr_sar_info.attr,
	&dev_attr_ps_info.attr,
	&dev_attr_set_fingersense_enable.attr,
	&dev_attr_fingersense_req_data.attr,
	&dev_attr_fingersense_data_ready.attr,
	&dev_attr_fingersense_latch_data.attr,
	&dev_attr_set_data_type.attr,
	NULL
};

static const struct attribute_group sensor_node = {
	.attrs = sensor_attributes,
};

static struct platform_device sensor_input_info = {
	.name = "huawei_sensor",
	.id = -1,
};

// here define sensor enable/delay node for every sensor
static ssize_t show_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n", 1);
}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);
	CHECK_SENSOR_COOKIE(data);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	pr_info("%s sensor-%d cmd = %lu!\n", __func__, data->sensor_type, val);
	scp_sysfs.delay = scp_sensor_delay[data->sensor_type];
	scp_sysfs.sensor_type = data->sensor_type;
	if (val == 0)
		scp_sysfs.action = HF_MANAGER_SENSOR_DISABLE;
	else
		scp_sysfs.action = HF_MANAGER_SENSOR_ENABLE;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		return -EINVAL;

	return size;
}

DEVICE_ATTR(enable, 0660, show_enable, store_enable);

static void update_sensor_delay(int sensor_type, int delay)
{
	int ret;
	scp_sysfs.delay = delay;
	scp_sysfs.sensor_type = sensor_type;
	scp_sysfs.action = HF_MANAGER_SENSOR_ENABLE;
	ret = send_scp_ctrl_cmd(&scp_sysfs);
	if (ret < 0)
		pr_err("%s: fail, errorno=%d", __func__, ret);
}

static ssize_t store_set_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val = 0;

	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	CHECK_SENSOR_COOKIE(data);
	if (cur_data_type == RAW_DATA_MODE && data->sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		stored_delay = scp_sensor_delay[SENSOR_TYPE_ACCELEROMETER];
		delay_time = val > 0 ? val : 1;
		val = SENSOR_DELAY_LOW;
	} else if (cur_data_type == RECOVER_NORMAL_DATA_MODE &&
		data->sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		delay_time = val > 0 ? val : 1;
		val = stored_delay;
	}
	scp_sensor_delay[data->sensor_type] = val;
	update_sensor_delay(data->sensor_type, val);
	return size;
}

static ssize_t show_set_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_cookie *data =
		(struct sensor_cookie *)dev_get_drvdata(dev);

	CHECK_SENSOR_COOKIE(data);

	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%d\n",
		scp_sensor_delay[data->sensor_type]);
}

DEVICE_ATTR(set_delay, 0660, show_set_delay, store_set_delay);

static struct attribute *sensors_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_set_delay.attr,
	NULL,
};
static const struct attribute_group sensors_attr_group = {
	.attrs = sensors_attributes,
};

static const struct attribute_group *sensors_attr_groups[] = {
	&sensors_attr_group,
	NULL,
};

static void get_cali_fool_info(void)
{
	int ret;
	uint32_t type = 0;
	struct device_node* sensor_info_node = NULL;

	sensor_info_node = of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");
	if (sensor_info_node == NULL) {
		pr_err("%s :cannot find huawei_sensor_info from dts\n", __func__);
		als_cali_fool_type = 0; // default
		return;
	} else {
		ret = of_property_read_u32(sensor_info_node, "als_cali_fool", &type);
		if (!ret) {
			pr_info("%s : find als_cali_fool type = %u\n", __func__, type);
			als_cali_fool_type = type;
		} else {
			pr_err("%s : cannot find product_type from dts\n", __func__);
			als_cali_fool_type = 0; // default
		}
	}
}

static int sensors_register(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(all_sensors); ++i) {
		all_sensors[i].dev = device_create(sensors_class, NULL, 0,
			&all_sensors[i], all_sensors[i].name);
		if (!all_sensors[i].dev)
			return -1;

		if ((all_sensors[i].attrs_group) &&
			(sysfs_create_group(&all_sensors[i].dev->kobj,
			all_sensors[i].attrs_group)))
			pr_err("create files failed in %s\n", __func__);
	}
	return 0;
}

#if defined CONFIG_MTK_CHARGER
static void charger_type_notifier_unregister(void)
{
	if (g_mag_consumer == NULL) {
		pr_err("%s, mag_consumer is null\n", __func__);
		return;
	}
	if (g_mag_task != NULL) {
		kthread_stop(g_mag_task);
		g_mag_task = NULL;
	}

	int ret = unregister_charger_manager_notifier(g_mag_consumer, &g_charger_detect_notify);
	if (ret < 0) {
		pr_err("%s, unregister charger notify fail, ret = %d\n",
			__func__, ret);
		return;
	}
}
#endif

static void sensors_unregister(void)
{
	device_destroy(sensors_class, 0);
	class_destroy(sensors_class);
}

static void sensor_scp_sysfs_exit(void)
{
	sensors_unregister();
#if defined CONFIG_MTK_CHARGER
	if (g_mag_need_charger_curr)
		charger_type_notifier_unregister();
#endif
}

static void acc_gravity_direction_read(const struct device_node *direction_node,
	struct acc_para_t *acc_priv)
{
	int ret;
	uint32_t direction = 0;

	ret = of_property_read_u32(direction_node, "lsm6dsm_direction",
		&direction);
	if (ret == 0) {
		acc_priv->lsm6dsm_direction = direction;
		pr_info("%s, get direction1 success : %u\n", __func__,
			acc_priv->lsm6dsm_direction);
	}

	ret = of_property_read_u32(direction_node, "bmi160_direction",
		&direction);
	if (ret == 0) {
		acc_priv->bmi160_direction = direction;
		pr_info("%s, get direction2 success : %u\n", __func__,
			acc_priv->bmi160_direction);
	}

	ret = of_property_read_u32(direction_node, "lis2dwl_direction",
		&direction);
	if (ret == 0) {
		acc_priv->lis2dwl_direction = direction;
		pr_info("%s, get direction3 success : %u\n", __func__,
			acc_priv->lis2dwl_direction);
	}

	ret = of_property_read_u32(direction_node, "bma4xy_direction",
		&direction);
	if (ret == 0) {
		acc_priv->bma4xy_direction = direction;
		pr_info("%s, get direction4 success : %u\n", __func__,
			acc_priv->bma4xy_direction);
	}

	ret = of_property_read_u32(direction_node, "kx022_direction",
		&direction);
	if (ret == 0) {
		acc_priv->kx022_direction = direction;
		pr_info("%s, get direction5 success : %u\n", __func__,
			acc_priv->kx022_direction);
	}

	ret = of_property_read_u32(direction_node, "bma253_direction",
		&direction);
	if (ret == 0) {
		acc_priv->bma253_direction = direction;
		pr_info("%s, get direction6 success : %u\n", __func__,
			acc_priv->bma253_direction);
	}

	ret = of_property_read_u32(direction_node, "da718_direction",
		&direction);
	if (ret == 0) {
		acc_priv->da718_direction = direction;
		pr_info("%s, get direction6 success : %u\n", __func__,
			acc_priv->da718_direction);
	}
}

static void get_acc_gravity_direction(void)
{
	int ret;
	uint32_t direction = 0;
	struct device_node *direction_node = NULL;
	struct acc_para_t *acc_priv = NULL;

	direction_node = of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");
	if (direction_node == NULL) {
		pr_err("Cannot find acc_gravity_direction from dts\n");
		return;
	}

	acc_priv = get_sensor_share_mem_addr(SHR_MEM_TYPE_ACC);
	if (acc_priv == NULL) {
		pr_err("%s: get acc para fail", __func__);
		return;
	}
	acc_priv->lsm6dsm_direction = 0; // default lsm6dsm_direction
	acc_priv->bmi160_direction = 3; // default bmi160_direction
	acc_priv->lis2dwl_direction = 5; // default lis2dwl_direction
	acc_priv->bma4xy_direction = 5; // default bma4xy_direction
	acc_priv->kx022_direction = 5; // default kx022_direction
	acc_priv->bma253_direction = 4; // default bma253_direction
	acc_priv->da718_direction = 2; // default da718_direction
	acc_gravity_direction_read(direction_node, acc_priv);
}

static void get_mag_calibrate_method(void)
{
	int ret;
	struct device_node *sensor_node =
		of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");

	if (sensor_node == NULL) {
		pr_err("Cannot find mag_calibrate_method from dts\n");
		return;
	}
	ret = of_property_read_u32(sensor_node, "calibrate_method",
		&g_calibrate_method);
	if (ret == 0)
		pr_info("%s, get mag_calibrate_method success : %d\n", __func__, g_calibrate_method);
}

static void get_gyro_timeout(void)
{
	int ret;
	struct device_node *sensor_node =
		of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");

	if (sensor_node == NULL) {
		pr_err("Cannot find gyro_timeout from dts\n");
		return;
	}
	ret = of_property_read_u32(sensor_node, "gyro_calibrate_timeout",
		&g_gyro_timeout);
	if (ret == 0)
		pr_info("%s, get gyro_timeout success : %d\n", __func__, g_gyro_timeout);
}

static void get_product_id(void)
{
	int ret;
	uint32_t id = 0;
	struct device_node *sensor_node = NULL;
	struct sensor_para_t *sensor_para = NULL;

	sensor_para = get_sensor_share_mem_addr(SHR_MEM_TYPE_ALL);
	if (sensor_para == NULL) {
		pr_err("%s: get sensor para fail", __func__);
		return;
	}
	sensor_para->product_id = 0;

	sensor_node = of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");
	if (sensor_node == NULL) {
		pr_err("Cannot find product from dts\n");
		return;
	}

	ret = of_property_read_u32(sensor_node, "product_number", &id);
	if (ret == 0) {
		sensor_para->product_id = id;
		pr_info("%s, get id sucess : %u\n", __func__,
			sensor_para->product_id);
	}

	ret = of_property_read_u32(sensor_node, "alsps_power_reset",
		&g_alsps_power_reset);
	if (ret == 0)
		pr_info("%s, get alsps_power_reset sucess : %u\n", __func__,
			g_alsps_power_reset);

	return;
}

static void sensor_delay_init(void)
{
	scp_sensor_delay[SENSOR_TYPE_ACCELEROMETER] = SENSOR_DELAY_DEFAULT;
	scp_sensor_delay[SENSOR_TYPE_MAGNETIC_FIELD] = SENSOR_DELAY_DEFAULT;
	scp_sensor_delay[SENSOR_TYPE_GYROSCOPE] = SENSOR_DELAY_DEFAULT;
}

static void get_lcd_type(void)
{
	int err;
	struct als_para_t *als_priv = NULL;
	struct lcd_kit_ops *tp_ops = NULL;
	char *lcd_model = NULL;
	struct device_node *np = NULL;
	char lcd_type[LCD_NAME_LEN] = {0};

	pr_info("%s\n", __func__);

	als_priv = get_sensor_share_mem_addr(SHR_MEM_TYPE_ALS);
	if (als_priv == NULL) {
		pr_err("%s: get acc para fail", __func__);
		return;
	}

	tp_ops = lcd_kit_get_ops();
	if (tp_ops && tp_ops->get_project_id) {
		err = tp_ops->get_project_id(lcd_type);
		if (err) {
			np = of_find_compatible_node(NULL, NULL, DTS_COMP_LCD_KIT_PANEL_TYPE);
			if (!np) {
				pr_err("not find device node %s!\n", DTS_COMP_LCD_KIT_PANEL_TYPE);
				return;
			}
			lcd_model = (char *)of_get_property(np, "lcd_panel_type", NULL);
			if (!lcd_model) {
				pr_err("can not get lcd kit compatible\n");
				return;
			}
			pr_err("%s : get lcd kit compatible %s", __func__, lcd_model);
			strncpy_s(als_priv->lcd_type, LCD_NAME_LEN, lcd_model, LCD_NAME_LEN - 1);
		} else {
			pr_info("%s : get lcd type : %s", __func__, lcd_type);
			strncpy_s(als_priv->lcd_type, LCD_NAME_LEN, lcd_type, LCD_NAME_LEN - 1);
		}
	} else {
		pr_err("%s : cannot get lcd type", __func__);
	}
}

static void get_mag_charger_effect_flag(void)
{
	int ret;
	struct device_node *sensor_node =
		of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");

	if (sensor_node == NULL) {
		pr_err("Cannot find sensor node from dts\n");
		return;
	}

	ret = of_property_read_u32(sensor_node, "mag_eliminate_current_effect",
		&g_mag_need_charger_curr);
	if (ret < 0) {
		g_mag_need_charger_curr = 0; // default: no need current info
		pr_err("%s, get mag_charger flag fail: %d\n", __func__, ret);
		return;
	}
	pr_info("%s, get mag_charger flag sucess : %u\n", __func__,
		g_mag_need_charger_curr);
}

static void set_mag_share_mem_default_info(void)
{
	struct mag_para_t *mag_para = get_sensor_share_mem_addr(SHR_MEM_TYPE_MAG);

	if (mag_para == NULL) {
		pr_err("%s: get mag para fail", __func__);
		return;
	}
	mag_para->usb_status = USB_PLUG_OUT;
	mag_para->charging_current = 0;
	pr_info("%s succ\n", __func__);
}

void hf_client_set_mag_status(uint8_t action)
{
	if (action == HF_MANAGER_SENSOR_ENABLE) {
		g_mag_open_status = 1;
#if defined CONFIG_MTK_CHARGER
	if (g_mag_need_charger_curr && g_mag_task != NULL)
		wake_up(&g_wait_event);
#endif
	} else {
		g_mag_open_status = 0;
	}
	pr_info("%s succ %u\n", __func__, g_mag_open_status);
}
EXPORT_SYMBOL(hf_client_set_mag_status);

#if defined CONFIG_MTK_CHARGER
static int mag_get_cur_charging_current(void *data)
{
	struct mag_para_t *mag_para = get_sensor_share_mem_addr(SHR_MEM_TYPE_MAG);

	if (mag_para == NULL) {
		pr_err("%s: get mag para fail", __func__);
		return -1;
	}

	unsigned int value;
	int ret;

	while (!kthread_should_stop()) {
		wait_event(g_wait_event,
			(g_mag_open_status && mag_para->usb_status == USB_PLUG_IN));

		pr_info("%s: start get current", __func__);
		msleep(4000); // wait chargerIC ready

		while (g_mag_open_status && mag_para->usb_status == USB_PLUG_IN) {
			msleep(500);
			value = 0;

			ret = power_if_kernel_sysfs_get(POWER_IF_OP_TYPE_SC,
				POWER_IF_SYSFS_IBUS, &value);
			if (ret >= 0) {
				mag_para->charging_current = value;
				continue;
			}
		}
	}
	return 0;
}

static int charger_detect_notifier_callback(struct notifier_block *self,
					unsigned long event, void *data)
{
	struct mag_para_t *mag_para = get_sensor_share_mem_addr(SHR_MEM_TYPE_MAG);

	if (mag_para == NULL || g_mag_task == NULL) {
		pr_err("%s: get mag para fail", __func__);
		return -1;
	}

	switch (event) {
	case CHARGER_NOTIFY_START_CHARGING:
		if (mag_para->usb_status == USB_PLUG_IN)
			break;
		mag_para->usb_status = USB_PLUG_IN;
		wake_up(&g_wait_event);
		pr_info("%s, charger plug in\n", __func__);
		break;
	case CHARGER_NOTIFY_STOP_CHARGING:
		if (mag_para->usb_status == USB_PLUG_OUT)
			break;
		mag_para->usb_status = USB_PLUG_OUT;
		mag_para->charging_current = 0;
		pr_info("%s, charger plug out\n", __func__);
		break;
	default:
		break;
	}
	return 0;
}

static void mag_charger_notifier_register(void)
{
	g_mag_consumer = charger_manager_get_by_name(&sensor_input_info.dev,
		"magnetometer");

	if (g_mag_consumer == NULL) {
		pr_err("%s get name fail\n", __func__);
		return;
	}
	init_waitqueue_head(&g_wait_event);

	g_charger_detect_notify.notifier_call = charger_detect_notifier_callback;
	int ret = register_charger_manager_notifier(g_mag_consumer,
		&g_charger_detect_notify);

	if (ret < 0) {
		pr_err("%s fail ret %d\n", __func__, ret);
		g_charger_detect_notify.notifier_call = NULL;
		return;
	}

	g_mag_task = kthread_create(mag_get_cur_charging_current, NULL,
		"mag_charging_current");
	if (IS_ERR(g_mag_task)) {
		pr_err("%s, creat task fail %d\n",
			__func__, PTR_ERR(g_mag_task));
		ret = unregister_charger_manager_notifier(g_mag_consumer, &g_charger_detect_notify);
		if (ret < 0)
			pr_err("%s, unregister charger notify fail, ret = %d\n",
				__func__, ret);
		g_mag_task = NULL;
		return;
	}
	wake_up_process(g_mag_task);
	pr_info("%s succ\n", __func__);
}
#endif

static void make_dmd_comment(int i)
{
	struct sensor_info sensor_name;
	int ret;

	ret = scp_get_sensor_info(sensor_dmd_comment[i].sensor_id, &sensor_name);
	if (ret < 0) {
		dmd_flag = 1; // need dmd report
		ret = strcat_s(sensor_all_info, ALL_INFO_SIZE,
			sensor_dmd_comment[i].sensor_info);
		if (ret != 0)
			pr_info("%s fail\n", __func__);
	}

}

static void report_dmd_code()
{
	int ret;

	ret = strcat_s(sensor_all_info, ALL_INFO_SIZE, "ERROR");
	if (ret != 0)
		pr_info("make_dmd_comment error fail\n");

	if (!dsm_sensor_dclient) {
		dsm_sensor_dclient = dsm_register_client(&dsm_sensorhub);
		pr_info("sensor dsm register success.\n");
	}
	if (!dsm_client_ocuppy(dsm_sensor_dclient)) {
		dsm_client_record(dsm_sensor_dclient, sensor_all_info);
		dsm_client_notify(dsm_sensor_dclient, DSM_SHB_ERR_IOM7_DETECT_FAIL);
	}

}

static void dmd_for_device_in_place(struct work_struct *work)
{
	struct device_node *node = NULL;
	int i;
	int ret;
	int sensor_support[SENSOR_SUPPORT_SIZE] = {0};
	pr_info("%s start", __func__);

	node = of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");
	if (node == NULL) {
		pr_info("Cannot find sensor dts from dts\n");
		return;
	}
	ret = of_property_read_u32_array(node, "sensor_support", sensor_support,
		SENSOR_SUPPORT_SIZE);
	if (ret != 0) {
		pr_info("Cannot find sensor_support list from dts\n");
		return;
	}
	for (i = 0; i < SENSOR_SUPPORT_SIZE; i++) {
		if (sensor_support[i])
			make_dmd_comment(i);
	}
	pr_info("dmd_flag %d\n", dmd_flag);
	if (dmd_flag)
		report_dmd_code();
}

static int __init sensor_scp_sysfs_init(void)
{
	int ret;

	pr_info("%s\n", __func__);
	sensors_class = class_create(THIS_MODULE, "sensors");
	if (IS_ERR(sensors_class))
		return PTR_ERR(sensors_class);
	sensors_class->dev_groups = sensors_attr_groups;
	sensors_register();

	ret = platform_device_register(&sensor_input_info);
	if (ret) {
		pr_err("%s: register failed, ret:%d\n", __func__, ret);
		goto register_fail;
	}

	ret = sysfs_create_group(&sensor_input_info.dev.kobj, &sensor_node);
	if (ret) {
		pr_err("sysfs_create_group error ret =%d\n", ret);
		goto sysfs_create_fail;
	}

	sensor_delay_init();

	get_cali_fool_info();

	get_acc_gravity_direction();

	get_product_id();

	get_lcd_type();

	get_gyro_timeout();

	get_aod_status();

	get_mag_charger_effect_flag();

	get_mag_calibrate_method();

	set_mag_share_mem_default_info();
	get_als_data();
#if defined CONFIG_MTK_CHARGER
	if (g_mag_need_charger_curr)
		mag_charger_notifier_register();
#endif

	INIT_DELAYED_WORK(&dmd_worker, dmd_for_device_in_place);
	schedule_delayed_work(&dmd_worker, DMD_DELAY);

	return 0;
sysfs_create_fail:
	platform_device_unregister(&sensor_input_info);

register_fail:
	sensors_unregister();
	return -1;
}

module_init(sensor_scp_sysfs_init);
module_exit(sensor_scp_sysfs_exit);

MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_DESCRIPTION("Sensor SCP sysfs driver");
MODULE_LICENSE("GPL v2");
