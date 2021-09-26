/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2020. All rights reserved.
 * Description: sensor sysfs source file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2012-01-06
 */

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "protocol.h"
#include "sensor_sysfs.h"
#include "sensor_info.h"
#include "acc_channel.h"
#include "airpress_channel.h"
#include "als_channel.h"
#include "cap_prox_channel.h"
#include "gyro_channel.h"
#include "mag_channel.h"
#include "ps_channel.h"
#include "therm_channel.h"
#include "xhub_pm.h"
#include "xhub_recovery.h"
#include "securec.h"
#include "ipc_adapter.h"

/* Wait 100ms for data transmitting */
#define FINGERSENSE_TRANS_TOUT  msecs_to_jiffies(100)
 /* We think the data is fresh if it is collected in 10ms */
#define FINGERSENSE_FRESH_TIME  msecs_to_jiffies(10)
#define FINGERSENSE_DISABLE 0
#define FINGERSENSE_ENABLE_ACCEL 1
#define FINGERSENSE_ENABLE_ACC1 2

#define MAX_VALUE 4294967296

#define SAR_SEMTECH_USE_PH_NUM 2
#define REG_16BIT_BYTE_LEN 2


uint16_t sensorlist[SENSOR_LIST_NUM];
unsigned int sensor_read_number[TAG_END];
char *sensor_in_board_status;
volatile int hall_value;
int hifi_supported;
int fingersense_enabled;
unsigned long stop_auto_accel;
int stop_auto_als;
int stop_auto_ps;

int calibrate_processing;
static uint8_t debug_read_data_buf[DEBUG_DATA_LENGTH];
static uint8_t i2c_rw16_data_buf[2]; /* 2: i2c data buf length */
static uint8_t i2c_rw_16bit_reg_data_buf[DEBUG_DATA_LENGTH];

/* pass mark as NA */
static unsigned long fingersense_data_ts; /* timestamp for the data */

#ifdef CONFIG_HUAWEI_DSM
extern struct dsm_client *shb_dclient;
#endif

static ssize_t show_sensor_list_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;

	hwlog_info("sensor list: ");
	for (i = 0; i <= sensorlist[0]; i++)
		hwlog_info(" %d  ", sensorlist[i]);
	hwlog_info("\n");
	memcpy(buf, sensorlist, ((sensorlist[0] + 1) * sizeof(uint16_t)));
	return (sensorlist[0] + 1) * sizeof(uint16_t);
}
static DEVICE_ATTR(sensor_list_info, 0444, show_sensor_list_info, NULL);

#define sensor_show_info(tagup, taglow) \
static ssize_t sensor_show_##taglow##_info(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, \
		"%s\n", sensor_chip_info[tagup]); \
}

sensor_show_info(PS, ps);
static DEVICE_ATTR(ps_info, 0444, sensor_show_ps_info, NULL);

sensor_show_info(ALS, als);
static DEVICE_ATTR(als_info, 0444, sensor_show_als_info, NULL);

sensor_show_info(ALS1, als1);
static DEVICE_ATTR(als1_info, 0444, sensor_show_als1_info, NULL);

sensor_show_info(ALS2, als2);
static DEVICE_ATTR(als2_info, 0444, sensor_show_als2_info, NULL);

sensor_show_info(GYRO, gyro);
static DEVICE_ATTR(gyro_info, 0444, sensor_show_gyro_info, NULL);

sensor_show_info(MAG, mag);
static DEVICE_ATTR(mag_info, 0444, sensor_show_mag_info, NULL);

sensor_show_info(ACC, acc);
static DEVICE_ATTR(acc_info, 0444, sensor_show_acc_info, NULL);

sensor_show_info(AIRPRESS, airpress);
static DEVICE_ATTR(airpress_info, 0444, sensor_show_airpress_info, NULL);

sensor_show_info(THERMOMETER, thermometer);
static DEVICE_ATTR(thermometer_info, 0444, sensor_show_thermometer_info, NULL);

#define sensor_show_value(tagup, taglow) \
static ssize_t sensor_show_##taglow##_read_data(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, \
		"%d\n", sensor_read_number[tagup]); \
}

sensor_show_value(TAG_ACCEL, accel);
static DEVICE_ATTR(acc_read_data, 0664, sensor_show_accel_read_data, NULL);

sensor_show_value(TAG_MAG, mag);
static DEVICE_ATTR(mag_read_data, 0664, sensor_show_mag_read_data, NULL);

sensor_show_value(TAG_GYRO, gyro);
static DEVICE_ATTR(gyro_read_data, 0664, sensor_show_gyro_read_data, NULL);

sensor_show_value(TAG_ALS, als);
static DEVICE_ATTR(als_read_data, 0664, sensor_show_als_read_data, NULL);

sensor_show_value(TAG_ALS1, als1);
static DEVICE_ATTR(als1_read_data, 0664, sensor_show_als1_read_data, NULL);

sensor_show_value(TAG_ALS2, als2);
static DEVICE_ATTR(als2_read_data, 0664, sensor_show_als2_read_data, NULL);

sensor_show_value(TAG_PS, ps);
static DEVICE_ATTR(ps_read_data, 0664, sensor_show_ps_read_data, NULL);

sensor_show_value(TAG_PRESSURE, pressure);
static DEVICE_ATTR(airpress_read_data, 0664,
	sensor_show_pressure_read_data, NULL);

#define show_selftest_result(tag) \
static ssize_t show_##tag##_selftest_result(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	return snprintf(buf, MAX_STR_SIZE, "%s\n", \
		sensor_status.tag##_selftest_result); \
}

show_selftest_result(gyro);
show_selftest_result(mag);
show_selftest_result(accel);
show_selftest_result(connectivity);

#define set_sensor_selftest(tagup, taglow) \
static ssize_t attr_set_##taglow##_selftest(struct device *dev, \
	struct device_attribute *attr, const char *buf, size_t size) \
{ \
	return attr_set_selftest(TAG_##tagup, \
		sensor_status.taglow##_selftest_result, buf, size); \
}

set_sensor_selftest(GYRO, gyro);
static DEVICE_ATTR(gyro_selfTest, 0664,
	show_gyro_selftest_result, attr_set_gyro_selftest);

set_sensor_selftest(MAG, mag);
static DEVICE_ATTR(mag_selfTest, 0664,
	show_mag_selftest_result, attr_set_mag_selftest);

set_sensor_selftest(ACCEL, accel);
static DEVICE_ATTR(acc_selfTest, 0664,
	show_accel_selftest_result, attr_set_accel_selftest);

set_sensor_selftest(CONNECTIVITY, connectivity);
static DEVICE_ATTR(connectivity_selfTest, 0664,
	show_connectivity_selftest_result, attr_set_connectivity_selftest);


#ifdef SENSOR_DATA_ACQUISITION
static int init_msg_for_enq(struct message *msg)
{
	if (!msg)
		return -1;

	memset(msg, 0, sizeof(struct message));
	msg->data_source = DATA_FROM_KERNEL;
	msg->num_events = 0;
	msg->version = 1;
	return 0;
}

static int init_event_of_msg(struct event *events,
	struct sensor_eng_cal_test sensor_test)
{
	if (!events)
		return -1;
	memset(events, 0, sizeof(struct event));
	events->error_code = 0;
	events->cycle = 0;
	memcpy(events->station, NA, sizeof(NA));
	memcpy(events->device_name, sensor_test.name, sizeof(sensor_test.name));
	memcpy(events->bsn, NA, sizeof(NA));
	memcpy(events->min_threshold, NA, sizeof(NA));
	memcpy(events->max_threshold, NA, sizeof(NA));
	memcpy(events->result, sensor_test.result, sizeof(sensor_test.result));
	memcpy(events->firmware, NA, sizeof(NA));
	memcpy(events->description, NA, sizeof(NA));
	return 0;
}

int enq_msg_data_in_xhub_single(struct event events)
{
	struct message *msg = NULL;
	int ret = -1;

	msg = kzalloc(sizeof(struct message), GFP_KERNEL);
	if (!msg)
		return ret;

	msg->data_source = DATA_FROM_KERNEL;
	msg->num_events = 1;
	msg->version = 1;
	events.error_code = 0;
	events.cycle = 0;
	memcpy(events.station, NA, sizeof(NA));
	memcpy(events.bsn, NA, sizeof(NA));
	memcpy(events.firmware, NA, sizeof(NA));
	memcpy(events.description, NA, sizeof(NA));
	memcpy(&(msg->events[0]), &events, sizeof(events));

	if (!dsm_client_ocuppy(shb_dclient)) {
		ret = dsm_client_copy_ext(shb_dclient, msg,
			sizeof(struct message));
		if (ret > 0)
			dsm_client_notify(shb_dclient, DA_SENSOR_HUB_ERROR_NO);
		else
			hwlog_err("dsm_client_notify failed");
	} else {
		hwlog_info("%s:dsm_client_ocuppy fail\n", __func__);
		dsm_client_unocuppy(shb_dclient);
		if (!dsm_client_ocuppy(shb_dclient)) {
			ret = dsm_client_copy_ext(shb_dclient, msg,
				sizeof(struct message));
			if (ret > 0)
				dsm_client_notify(shb_dclient,
					DA_SENSOR_HUB_ERROR_NO);
			else
				hwlog_err("dsm_client_notify failed");
		}
	}

	kfree(msg);

	return ret;
}

static void msg_process_for_enq(struct message *msg,
	const struct sensor_eng_cal_test *sensor_test, int num, int cal_value)
{
	if (snprintf_s(msg->events[num].value, MAX_VAL_LEN, MAX_VAL_LEN,
		"%d", *(sensor_test->cal_value + cal_value)) < 0)
		hwlog_err("%s snprintf_s failed\n", __func__);
	if (cal_value < sensor_test->threshold_num) {
		if (snprintf_s(msg->events[num].min_threshold,
			MAX_VAL_LEN, MAX_VAL_LEN, "%d",
			*(sensor_test->min_threshold + cal_value)) < 0)
			hwlog_err("snprintf_s failed\n");
		if (snprintf_s(msg->events[num].max_threshold,
			MAX_VAL_LEN, MAX_VAL_LEN, "%d",
			*(sensor_test->max_threshold + cal_value)) < 0)
			hwlog_err("snprintf_s failed\n");
	}
	if (memcpy_s(msg->events[num].test_name,
		sizeof(msg->events[num].test_name),
		sensor_test->test_name[cal_value],
		strlen(sensor_test->test_name[cal_value]) + 1) != EOK)
		hwlog_err("memcpy_s error\n");
	msg->events[num].item_id = sensor_test->first_item + cal_value;
}

static int enq_msg_data_in_sensorhub(struct sensor_eng_cal_test sensor_test)
{
	struct message *msg = NULL;
	int ret = -1;
	int cal_value = 0;
	int i;

	hwlog_info("%s:sart\n", __func__);
	if (sensor_test.value_num > MAX_COPY_EVENT_SIZE ||
	    !sensor_test.cal_value) {
		hwlog_info("%s bad value!!\n", __func__);
		return ret;
	}

	while (cal_value < sensor_test.value_num) {
		msg = kzalloc(sizeof(struct message), GFP_KERNEL);
		ret = init_msg_for_enq(msg);
		if (ret) {
			hwlog_err("alloc mesage failed\n");
			return ret;
		}

		for (i = 0; (i < MAX_MSG_EVENT_NUM &&
			cal_value < sensor_test.value_num);
			i++, cal_value++) {
			ret = init_event_of_msg(&(msg->events[i]),
				sensor_test);
			if (ret) {
				hwlog_err("init_event_of_msg failed\n");
				kfree(msg);
				return ret;
			}
			msg_process_for_enq(msg, &sensor_test, i, cal_value);
		}
		msg->num_events = i;
		ret = dsm_client_copy_ext(shb_dclient, msg, sizeof(struct message));
		if (ret <= 0) {
			ret = -1;
			hwlog_err("%s dsm_client_copy_ext failed\n", sensor_test.name);
			kfree(msg);
			return ret;
		}
		ret = 0;
		kfree(msg);
	}
	hwlog_info("%s %s succ\n", __func__, sensor_test.name);
	return ret;
}

void enq_notify_work_sensor(struct sensor_eng_cal_test sensor_test)
{
	int ret;

	if (!dsm_client_ocuppy(shb_dclient)) {
		ret = enq_msg_data_in_sensorhub(sensor_test);
		if (!ret)
			dsm_client_notify(shb_dclient, DA_SENSOR_HUB_ERROR_NO);
	} else {
		hwlog_info("%s:dsm_client_ocuppy fail\n", __func__);
		dsm_client_unocuppy(shb_dclient);
		if (!dsm_client_ocuppy(shb_dclient)) {
			ret = enq_msg_data_in_sensorhub(sensor_test);
			if (!ret)
				dsm_client_notify(shb_dclient,
					DA_SENSOR_HUB_ERROR_NO);
		}
	}
}
#endif

static ssize_t i2c_rw_pi(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	uint64_t val = 0;
	int ret;
	struct i2c_i3c_rw_info rw_info;

	memset_s(&rw_info, sizeof(rw_info), 0, sizeof(rw_info));
	if (kstrtoull(buf, 16, &val))
		return -EINVAL;
	/* ##(i2c/i3c)##(bus_num)##(i2c_addr)##(reg_addr)##(len) */
	rw_info.i2c_i3c = (val >> BITE_MOVE_FLAG) & 0xff;
	rw_info.bus_num = (val >> BITE_MOVE_BUS_NUM) & 0xff;
	rw_info.i2c_address = (val >> BITE_MOVE_ADDR) & 0xff;
	rw_info.buf_temp[0] = (val >> BITE_MOVE_BUF_TEMP) & 0xff;
	rw_info.len = (val >> BITE_MOVE_LEN) & 0xff;
	if (rw_info.len > DEBUG_DATA_LENGTH - 1) {
		hwlog_err("len exceed %d\n", rw_info.len);
		rw_info.len = DEBUG_DATA_LENGTH - 1;
	}
	rw_info.rw = (val >> BITE_MOVE_RW) & 0xff;
	rw_info.buf_temp[1] = (uint8_t)(val & 0xff);

	if (rw_info.rw)
		ret = mcu_i2c_rw(rw_info.i2c_i3c, rw_info.bus_num, rw_info.i2c_address,
			&rw_info.buf_temp[0], 1, &rw_info.buf_temp[1], rw_info.len);
	else
		ret = mcu_i2c_rw(rw_info.i2c_i3c, rw_info.bus_num,
			rw_info.i2c_address, rw_info.buf_temp, 2, NULL, 0);

	if (ret < 0)
		hwlog_err("oper %d(1/32:r 0/31:w) i2c reg fail\n", rw_info.rw);
	if (rw_info.rw) {
		hwlog_info("i2c reg %x value %x %x %x %x\n", rw_info.buf_temp[0],
			rw_info.buf_temp[1], rw_info.buf_temp[I2C_REG_TW],
			rw_info.buf_temp[I2C_REG_THR], rw_info.buf_temp[I2C_REG_FO]);
		if (memcpy_s(debug_read_data_buf, sizeof(debug_read_data_buf),
			&rw_info.buf_temp[1], rw_info.len) != EOK) {
			hwlog_err("%s memcpy_s data_buf error\n", __func__);
			return -1;
		}
	}
	return count;
}

static ssize_t i2c_rw_pi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int i;
	unsigned int len = 0;
	char *p = buf;

	if (!buf)
		return -1;

	/* 4,5,6 used to show i2c debug data */
	for (i = 0; i < DEBUG_DATA_LENGTH; i++) {
		snprintf(p, 6, "0x%x,", debug_read_data_buf[i]);
		if (debug_read_data_buf[i] > 0xf) {
			p += 5;
			len += 5;
		} else {
			p += 4;
			len += 4;
		}
	}

	p = buf;
	*(p + len - 1) = 0;

	p = NULL;
	return len;
}

static DEVICE_ATTR(i2c_rw, 0664, i2c_rw_pi_show, i2c_rw_pi);

static ssize_t i2c_rw16_pi(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	uint64_t val = 0;
	int ret;
	uint8_t bus_num;
	uint8_t i2c_address;
	uint8_t len;
	uint8_t rw;
	uint8_t buf_temp[3] = { 0 };

	if (kstrtoull(buf, 16, &val))
		return -EINVAL;
	/* ##(bus_num)##(i2c_addr)##(reg_addr)##(len) */
	bus_num = (val >> 48) & 0xff;
	i2c_address = (val >> 40) & 0xff;
	buf_temp[0] = (val >> 32) & 0xff;
	len = (val >> 24) & 0xff;
	if (len > 2) {
		hwlog_err("len exceed %d\n", len);
		len = 2;
	}
	rw = (val >> 16) & 0xff;
	buf_temp[1] = (uint8_t)(val >> 8);
	buf_temp[2] = (uint8_t)(val & 0xff);

	if (rw)
		ret = mcu_i2c_rw(TAG_I2C, bus_num, i2c_address, buf_temp,
				1, &buf_temp[1], (uint32_t)len);
	else
		ret = mcu_i2c_rw(TAG_I2C, bus_num, i2c_address, buf_temp,
				1 + len, NULL, 0);
	if (ret < 0)
		hwlog_err("oper %d(1:r 0:w) i2c reg fail\n", rw);

	if (rw) {
		hwlog_err("i2c reg %x val %x %x\n",
			buf_temp[0], buf_temp[1], buf_temp[2]);
		memcpy(i2c_rw16_data_buf, &buf_temp[1], 2);
	}
	return count;
}

static ssize_t i2c_rw16_pi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *p = buf;

	if (!buf)
		return -1;

	snprintf(p, 8, "0x%02x%02x\n",
		i2c_rw16_data_buf[0], i2c_rw16_data_buf[1]);
	*(p + 7) = 0;

	p = NULL;
	return 8;
}
static DEVICE_ATTR(i2c_rw16, 0664, i2c_rw16_pi_show, i2c_rw16_pi);

static int mcu_i2c_rw_16bit_reg(uint8_t bus_num, uint8_t i2c_add,
	uint8_t rw_info, uint8_t *buf, int buf_len)
{
	int ret;
	uint16_t reg_add;
	uint8_t len;
	uint8_t rw;
	uint8_t tx_buf_len;

	if (buf_len < DEBUG_DATA_LENGTH) {
		hwlog_err("buffer is invalid\n");
		return -EINVAL;
	}
	/* buf[0] is reg high byte, buf[1] is low byte */
	reg_add = (uint16_t)(buf[0] << 8) + buf[1];
	/* rw_info 4bit, highest bit is rw bit, left is buf length */
	rw = rw_info & 0x8;
	len = rw_info & 0x7;
	/* rw max buf len is 4 */
	if (len > 4) {
		hwlog_err("rw buf len is too long\n");
		return -EINVAL;
	}
	if (rw) {
		ret = mcu_i2c_rw(TAG_I2C, bus_num, i2c_add,
			&buf[0], REG_16BIT_BYTE_LEN,
			&buf[REG_16BIT_BYTE_LEN], len);
		/* buf 2~5 refer reg value */
		hwlog_info("i2c reg %04x value %x %x %x %x\n", reg_add,
			buf[2], buf[3], buf[4], buf[5]);
		if (memcpy_s(i2c_rw_16bit_reg_data_buf, DEBUG_DATA_LENGTH,
			&buf[REG_16BIT_BYTE_LEN], len) != EOK)
			hwlog_err("%s, memcpy_s fail\n", __func__);
	} else {
		/* buf 2~5 refer reg value */
		hwlog_info("reg %x write value is:%2x %2x %2x %2x\n", reg_add,
			buf[2], buf[3], buf[4], buf[5]);
		/* set highest bit 1 to indicate 16bit reg */
		tx_buf_len = (uint8_t)((REG_16BIT_BYTE_LEN + len) | 0x80);
		ret = mcu_i2c_rw(TAG_I2C, bus_num, i2c_add, buf, tx_buf_len, NULL, 0);
	}
	if (ret < 0)
		hwlog_err("oper %d(1:r 0:w) i2c reg fail\n", rw);

	return 0;
}

static ssize_t i2c_rw_16bit_reg_pi(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint64_t val = 0;
	uint8_t bus_num;
	uint8_t i2c_address;
	uint16_t reg_add;
	uint8_t rw_info;
	uint8_t buf_temp[DEBUG_DATA_LENGTH] = { 0 };

	if (kstrtoull(buf, 16, &val))
		return -EINVAL;
	/*
	 * ##(bus_num 4bit)##(i2c_addr 8bit)##(reg_addr 16bit )
	 * ##(rw 4bit)##(write data 32bit)
	 */
	bus_num = (uint8_t) (val >> 60) & 0xf;
	i2c_address = (val >> 52) & 0xff;
	reg_add = (val >> 36) & 0xffff;
	/* rw_info 4bit,highest bit rw,left is buf length */
	rw_info = (uint8_t)(val >> 32) & 0xf;
	buf_temp[0] = (uint8_t)(reg_add >> 8) & 0xff;
	buf_temp[1] = (uint8_t) (reg_add & 0xff);
	buf_temp[2] = (uint8_t)(val >> 24) & 0xff;
	buf_temp[3] = (uint8_t)(val >> 16) & 0xff;
	buf_temp[4] = (uint8_t)(val >> 8) & 0xff;
	buf_temp[5] = (uint8_t)val & 0xff;
	if (mcu_i2c_rw_16bit_reg(bus_num, i2c_address,
		rw_info, buf_temp, DEBUG_DATA_LENGTH) < 0)
		return -EINVAL;

	return count;
}

static ssize_t i2c_rw_16bit_reg_pi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int i;
	unsigned int len = 0;
	char *p = buf;

	if (!buf)
		return -1;

	for (i = 0; i < DEBUG_DATA_LENGTH; i++) {
		snprintf(p, 12, "0x%x,", i2c_rw_16bit_reg_data_buf[i]);
		if (i2c_rw_16bit_reg_data_buf[i] > 0xf) {
			p += 5;
			len += 5;
		} else {
			p += 4;
			len += 4;
		}
	}
	p = buf;
	*(p + len - 1) = 0;
	p = NULL;
	return len;
}

static DEVICE_ATTR(i2c_rw_16bit_reg, 0664,
	i2c_rw_16bit_reg_pi_show, i2c_rw_16bit_reg_pi);

read_info_t send_calibrate_cmd(uint8_t tag, unsigned long val,
	ret_type *rtype)
{
	int ret;
	write_info_t pkg_ap = { 0 };
	read_info_t pkg_mcu = { 0 };
	pkt_parameter_req_t spkt;
	pkt_header_t *shd = (pkt_header_t *)&spkt;

	pkg_ap.tag = tag;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	spkt.subcmd = SUB_CMD_SELFCALI_REQ;
	pkg_ap.wr_buf = &shd[1];
	pkg_ap.wr_len = sizeof(val) + SUBCMD_LEN;
	memcpy(spkt.para, &val, sizeof(val));
	hwlog_err("tag %d calibrator val is %lu len is %lu\n",
		tag, val, sizeof(val));
	ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);
	if (ret) {
		*rtype = COMMU_FAIL;
		hwlog_err("send tag %d calibrate cmd to mcu fail,ret=%d\n", tag, ret);
	} else if (pkg_mcu.errno != 0) {
		hwlog_err("send tag %d calibrate fail,%d\n", tag, pkg_mcu.errno);
		*rtype = EXEC_FAIL;
	} else {
		hwlog_info("send tag %d calibrate  success, data len=%d\n",
			tag, pkg_mcu.data_length);
		*rtype = SUC;
	}
	return pkg_mcu;
}

static DEVICE_ATTR(acc_calibrate, 0664,
	attr_acc_calibrate_show, attr_acc_calibrate_write);
static DEVICE_ATTR(gyro_calibrate, 0664,
	attr_gyro_calibrate_show, attr_gyro_calibrate_write);

static DEVICE_ATTR(ps_calibrate, 0664,
	attr_ps_calibrate_show, attr_ps_calibrate_write);
static DEVICE_ATTR(als_calibrate, 0664, attr_als_calibrate_show,
	attr_als_calibrate_write);
static DEVICE_ATTR(als1_calibrate, 0664, attr_als1_calibrate_show,
	attr_als1_calibrate_write);
static DEVICE_ATTR(als2_calibrate, 0664, attr_als2_calibrate_show,
	attr_als2_calibrate_write);
static DEVICE_ATTR(cap_prox_calibrate, 0664,
	attr_cap_prox_calibrate_show, attr_cap_prox_calibrate_write);

/*
 * if val is odd, then last status is sleep,
 * if is even number, then last status is wakeup
 */
static ssize_t attr_iom3_sr_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = 0;
	unsigned long times;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	times = val;
	if (val > 0) {
		for (; val > 0; val--) {
			disable_sensors_when_suspend();
			tell_ap_status_to_mcu(ST_SLEEP);
			msleep(2);
			tell_ap_status_to_mcu(ST_WAKEUP);
			enable_sensors_when_resume();
		}
		if (times % 2) {
			tell_ap_status_to_mcu(ST_SLEEP);
			enable_sensors_when_resume();
		}
	}
	return count;
}

static DEVICE_ATTR(iom3_sr_test, 0660, NULL, attr_iom3_sr_test_store);

int fingersense_commu(unsigned int cmd, unsigned int pare,
	unsigned int responsed, bool is_subcmd)
{
	int ret;
	write_info_t pkg_ap = { 0 };
	read_info_t pkg_mcu = { 0 };
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;

	if (is_subcmd) {
		pkg_ap.tag = TAG_FINGERSENSE;
		pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
		cpkt.subcmd = cmd;
		pkg_ap.wr_buf = &hd[1];
		pkg_ap.wr_len = sizeof(pare) + SUBCMD_LEN;
		memcpy(cpkt.para, &pare, sizeof(pare));
	} else {
		pkg_ap.tag = TAG_FINGERSENSE;
		pkg_ap.cmd = cmd;
		pkg_ap.wr_buf = &pare;
		pkg_ap.wr_len = sizeof(pare);
	}
	/*
	 * NO_RESP: enable/disable fingersense
	 * RESP: request fingersense data need response
	 */
	if (responsed == NO_RESP)
		ret = write_customize_cmd(&pkg_ap, NULL, true);
	else
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);
	if (ret) {
		hwlog_err("send finger sensor cmd %d to mcu fail,ret=%d\n",
			cmd, ret);
		return ret;
	}
	if (pkg_mcu.errno != 0)
		hwlog_err("send finger sensor cmd %d to mcu fail %d\n",
			cmd, pkg_mcu.errno);
	return ret;
}

int fingersense_enable(unsigned int enable)
{
	unsigned int cmd;
	unsigned int delay = 10000;
	unsigned int ret;

	/* enable-1: use main screen; enable-2: use assit screen */
	if (enable == FINGERSENSE_ENABLE_ACCEL ||
	    enable == FINGERSENSE_ENABLE_ACC1) {
		cmd = CMD_CMN_OPEN_REQ;
		ret = fingersense_commu(cmd, enable, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: finger sense enable fail\n", __func__);
			return ret;
		}

		cmd = CMD_CMN_INTERVAL_REQ;
		ret = fingersense_commu(cmd, delay, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: set delay fail\n", __func__);
			return ret;
		}

		if (enable == FINGERSENSE_ENABLE_ACCEL)
			cmd = SUB_CMD_TOUCH_FINGERSENSE_MAIN_SCREEN;
		else if (enable == FINGERSENSE_ENABLE_ACC1)
			cmd = SUB_CMD_TOUCH_FINGERSENSE_ASSIT_SCREEN;
		ret = fingersense_commu(cmd, cmd, NO_RESP, true);
		if (ret) {
			hwlog_err("%s: set config fail\n", __func__);
			return ret;
		}
		hwlog_info("%s: finger sense enable succsess\n", __func__);
	} else {
		cmd = CMD_CMN_CLOSE_REQ;
		ret = fingersense_commu(cmd, enable, NO_RESP, false);
		if (ret) {
			hwlog_info("%s: finger sense close fail\n", __func__);
			return ret;
		}
		hwlog_info("%s: finger sense close succsess\n", __func__);
	}

	return 0;
}

static ssize_t attr_set_fingersense_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;

	if (kstrtoul(buf, 10, &val)) {
		hwlog_err("%s: finger sense enable val %lu invalid", __func__, val);
		return -EINVAL;
	}

	hwlog_info("%s: finger sense enable val %ld\n", __func__, val);
	if ((val != FINGERSENSE_DISABLE) && (val != FINGERSENSE_ENABLE_ACCEL) &&
		(val != FINGERSENSE_ENABLE_ACC1)) {
		hwlog_err("%s:finger sense set enable fail, invalid val\n", __func__);
		return size;
	}

	if (fingersense_enabled == val) {
		hwlog_info("%s:finger sense already at seted state,fingersense_enabled:%d\n",
			__func__, fingersense_enabled);
		return size;
	}
	ret = fingersense_enable(val);
	if (ret) {
		hwlog_err("%s: finger sense enable fail: %d\n", __func__, ret);
		return size;
	}
	fingersense_enabled = val;

	return size;
}

static ssize_t attr_get_fingersense_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%d\n", fingersense_enabled);
}

static DEVICE_ATTR(set_fingersense_enable, 0660,
	attr_get_fingersense_enable, attr_set_fingersense_enable);

static ssize_t attr_fingersense_data_ready(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%d\n", fingersense_data_ready);
}

static DEVICE_ATTR(fingersense_data_ready, 0440,
	attr_fingersense_data_ready, NULL);

static ssize_t attr_fingersense_latch_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int size;

	size = sizeof(fingersense_data) < MAX_STR_SIZE ?
		sizeof(fingersense_data) : MAX_STR_SIZE;

	if ((!fingersense_data_ready) || (!fingersense_enabled)) {
		hwlog_err("%s:fingersense zaxix not ready %d or not enable %d\n",
		     __func__, fingersense_data_ready, fingersense_enabled);
		return size;
	}
	memcpy(buf, (char *)fingersense_data, size);
	return size;
}

static DEVICE_ATTR(fingersense_latch_data, 0440,
	attr_fingersense_latch_data, NULL);

/* Calculate whether a is in the range of [b, c] */
int is_time_inrange(unsigned long a, unsigned long b, unsigned long c)
{
	return ((long)(a - b) >= 0) && ((long)(a - c) <= 0);
}

static ssize_t attr_fingersense_req_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned int sub_cmd = SUB_CMD_ACCEL_FINGERSENSE_REQ_DATA_REQ;
	unsigned int skip = 0;
	unsigned long local_jiffies = jiffies;
	unsigned long flags = 0;

	if (!fingersense_enabled) {
		hwlog_err("%s: finger sense not enable,  dont req data\n", __func__);
		return -1;
	}

	spin_lock_irqsave(&fsdata_lock, flags);

	/*
	 * We started transmitting the data in recent time.
	 * It's just on the way. Wait for it.
	 */
	if (fingersense_data_intrans &&
	    is_time_inrange(fingersense_data_ts,
			local_jiffies - FINGERSENSE_TRANS_TOUT, local_jiffies))
		skip = 1;

	/* The data was collected a short while ago. Just use it. */
	if (fingersense_data_ready &&
	    (is_time_inrange(fingersense_data_ts,
			local_jiffies - FINGERSENSE_FRESH_TIME, local_jiffies)))
		skip = 1;

	if (skip) {
		spin_unlock_irqrestore(&fsdata_lock, flags);
		return size;
	}

	fingersense_data_ready = false;
	fingersense_data_intrans = true; /* the data is on the way */
	fingersense_data_ts = jiffies;   /* record timestamp for the data */
	spin_unlock_irqrestore(&fsdata_lock, flags);
	ret = fingersense_commu(sub_cmd, sub_cmd, NO_RESP, true);
	if (ret) {
		spin_lock_irqsave(&fsdata_lock, flags);
		fingersense_data_intrans = false;
		spin_unlock_irqrestore(&fsdata_lock, flags);
		hwlog_err("%s: finger sense send requst data failed\n", __func__);
	}
	return size;
}

void preread_fingersense_data(void)
{
	if (!fingersense_enabled)
		return;

	attr_fingersense_req_data(NULL, NULL, NULL, 0);
}
EXPORT_SYMBOL(preread_fingersense_data);

static DEVICE_ATTR(fingersense_req_data, 0220, NULL, attr_fingersense_req_data);

/* acc enable node */
#define show_enable_func(name, tag) \
static ssize_t show_##name##_enable_result(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, \
		"%d\n", sensor_status.status[tag]); \
}

#define store_enable_func(name, tag, cmd1, cmd2) \
static ssize_t attr_set_##name##_enable(struct device *dev, \
	struct device_attribute *attr, const char *buf, size_t size) \
{ \
	return attr_set_enable(tag, cmd1, cmd2, buf, size); \
}

#define show_delay_func(name, tag) \
static ssize_t show_##name##_delay_result(struct device *dev, \
	struct device_attribute *attr, char *buf) \
{ \
	return snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, \
		"%d\n", sensor_status.delay[tag]); \
}

#define store_delay_func(name, tag, cmd) \
static ssize_t attr_set_##name##_delay(struct device *dev, \
	struct device_attribute *attr, const char *buf, size_t size) \
{ \
	return attr_set_delay(tag, buf, size); \
}

show_enable_func(acc, TAG_ACCEL)
store_enable_func(acc, TAG_ACCEL, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(acc_enable, 0664, show_acc_enable_result, attr_set_acc_enable);
show_delay_func(acc, TAG_ACCEL)
store_delay_func(acc, TAG_ACCEL, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(acc_setdelay, 0664, show_acc_delay_result, attr_set_acc_delay);

static DEVICE_ATTR(gsensor_gather_enable, 0664,
	NULL, attr_set_gsensor_gather_enable);

show_enable_func(gyro, TAG_GYRO)
store_enable_func(gyro, TAG_GYRO, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(gyro_enable, 0664,
	show_gyro_enable_result, attr_set_gyro_enable);
show_delay_func(gyro, TAG_GYRO)
store_delay_func(gyro, TAG_GYRO, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(gyro_setdelay, 0664,
	show_gyro_delay_result, attr_set_gyro_delay);

show_enable_func(mag, TAG_MAG)
store_enable_func(mag, TAG_MAG, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(mag_enable, 0664, show_mag_enable_result, attr_set_mag_enable);
show_delay_func(mag, TAG_MAG)
store_delay_func(mag, TAG_MAG, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(mag_setdelay, 0664, show_mag_delay_result, attr_set_mag_delay);

show_enable_func(als, TAG_ALS)
store_enable_func(als, TAG_ALS, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(als_enable, 0664, show_als_enable_result, attr_set_als_enable);
show_delay_func(als, TAG_ALS)
store_delay_func(als, TAG_ALS, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(als_setdelay, 0664, show_als_delay_result, attr_set_als_delay);

show_enable_func(als1, TAG_ALS1)
store_enable_func(als1, TAG_ALS1, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(als1_enable, 0664,
	show_als1_enable_result, attr_set_als1_enable);
show_delay_func(als1, TAG_ALS1)
store_delay_func(als1, TAG_ALS1, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(als1_setdelay, 0664,
	show_als1_delay_result, attr_set_als1_delay);

show_enable_func(als2, TAG_ALS2)
store_enable_func(als2, TAG_ALS2, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(als2_enable, 0664,
	show_als2_enable_result, attr_set_als2_enable);
show_delay_func(als2, TAG_ALS2)
store_delay_func(als2, TAG_ALS2, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(als2_setdelay, 0664,
	show_als2_delay_result, attr_set_als2_delay);

show_enable_func(ps, TAG_PS)
store_enable_func(ps, TAG_PS, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(ps_enable, 0664, show_ps_enable_result, attr_set_ps_enable);
show_delay_func(ps, TAG_PS)
store_delay_func(ps, TAG_PS, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(ps_setdelay, 0664, show_ps_delay_result, attr_set_ps_delay);

show_enable_func(os, TAG_ORIENTATION)
store_enable_func(os, TAG_ORIENTATION, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(orientation_enable, 0664,
	show_os_enable_result, attr_set_os_enable);
show_delay_func(os, TAG_ORIENTATION)
store_delay_func(os, TAG_ORIENTATION, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(orientation_setdelay, 0664,
	show_os_delay_result, attr_set_os_delay);

show_enable_func(lines, TAG_LINEAR_ACCEL)
store_enable_func(lines, TAG_LINEAR_ACCEL, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(lines_enable, 0664,
	show_lines_enable_result, attr_set_lines_enable);
show_delay_func(lines, TAG_LINEAR_ACCEL)
store_delay_func(lines, TAG_LINEAR_ACCEL, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(lines_setdelay, 0664,
	show_lines_delay_result, attr_set_lines_delay);

show_enable_func(gras, TAG_GRAVITY)
store_enable_func(gras, TAG_GRAVITY, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(gras_enable, 0664,
	show_gras_enable_result, attr_set_gras_enable);
show_delay_func(gras, TAG_GRAVITY)
store_delay_func(gras, TAG_GRAVITY, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(gras_setdelay, 0664,
	show_gras_delay_result, attr_set_gras_delay);

show_enable_func(rvs, TAG_ROTATION_VECTORS)
store_enable_func(rvs, TAG_ROTATION_VECTORS, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(rvs_enable, 0664, show_rvs_enable_result, attr_set_rvs_enable);
show_delay_func(rvs, TAG_ROTATION_VECTORS)
store_delay_func(rvs, TAG_ROTATION_VECTORS, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(rvs_setdelay, 0664, show_rvs_delay_result, attr_set_rvs_delay);

show_enable_func(airpress, TAG_PRESSURE)
store_enable_func(airpress, TAG_PRESSURE, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(airpress_enable, 0664,
	show_airpress_enable_result, attr_set_airpress_enable);
show_delay_func(airpress, TAG_PRESSURE)
store_delay_func(airpress, TAG_PRESSURE, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(airpress_setdelay, 0664,
	show_airpress_delay_result, attr_set_airpress_delay);

store_enable_func(pdr, TAG_PDR, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(pdr_enable, 0220, NULL, attr_set_pdr_enable);
static DEVICE_ATTR(pdr_setdelay, 0220, NULL, attr_set_pdr_delay);

show_enable_func(cap_prox, TAG_CAP_PROX)
store_enable_func(cap_prox, TAG_CAP_PROX, CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(cap_prox_enable, 0664,
	show_cap_prox_enable_result, attr_set_cap_prox_enable);
show_delay_func(cap_prox, TAG_CAP_PROX)
store_delay_func(cap_prox, TAG_CAP_PROX, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(cap_prox_setdelay, 0664,
	show_cap_prox_delay_result, attr_set_cap_prox_delay);

show_enable_func(magn_bracket, TAG_MAGN_BRACKET)
store_enable_func(magn_bracket, TAG_MAGN_BRACKET,
	CMD_CMN_OPEN_REQ, CMD_CMN_CLOSE_REQ)
static DEVICE_ATTR(magn_bracket_enable, 0664,
	show_magn_bracket_enable_result, attr_set_magn_bracket_enable);
show_delay_func(magn_bracket, TAG_MAGN_BRACKET)
store_delay_func(magn_bracket, TAG_MAGN_BRACKET, CMD_CMN_INTERVAL_REQ)
static DEVICE_ATTR(magn_bracket_setdelay, 0664,
	show_magn_bracket_delay_result, attr_set_magn_bracket_delay);

static ssize_t start_iom3_recovery(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	hwlog_info("%s +\n", __func__);
	xhub_need_recovery(SH_FAULT_USER_DUMP);
	hwlog_info("%s -\n", __func__);
	return size;
}
static DEVICE_ATTR(iom3_recovery, 0664, NULL, start_iom3_recovery);

static DEVICE_ATTR(sensor_test, 0660, NULL, attr_set_sensor_test_mode);
static DEVICE_ATTR(dt_motion_stup, 0664, NULL, attr_set_dt_motion_stup);
static DEVICE_ATTR(dt_stop_auto_data, 0664, NULL, attr_set_stop_auto_data);
static DEVICE_ATTR(dt_stop_auto_motion, 0660,
	attr_stop_auto_motion_show, attr_set_stop_auto_motion);
static DEVICE_ATTR(dt_stop_als_auto_data, 0664, NULL, attr_set_stop_als_auto_data);
static DEVICE_ATTR(dt_stop_ps_auto_data, 0664, NULL, attr_set_stop_ps_auto_data);
static DEVICE_ATTR(dt_sensor_stup, 0664, NULL, attr_set_sensor_motion_stup);
static DEVICE_ATTR(dt_stepcounter_stup, 0664, NULL,
	attr_set_sensor_stepcounter_stup);
static DEVICE_ATTR(dt_hall_sensor_stup, 0664, NULL, attr_set_dt_hall_sensor_stup);
static DEVICE_ATTR(dt_als_sensor_stup, 0664, NULL, attr_set_dt_als_sensor_stup);
static DEVICE_ATTR(dt_ps_sensor_stup, 0664, NULL, attr_set_dt_ps_sensor_stup);

static ssize_t show_iom3_sr_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%s\n",
		(iom3_sr_status == ST_SLEEP) ? "ST_SLEEP" : "ST_WAKEUP");
}
static DEVICE_ATTR(iom3_sr_status, 0664, show_iom3_sr_status, NULL);
static DEVICE_ATTR(read_airpress, 0664, show_sensor_read_airpress, NULL);
static DEVICE_ATTR(read_temperature, 0664, show_sensor_read_temperature, NULL);
static DEVICE_ATTR(dump_sensor_status, 0664, show_dump_sensor_status, NULL);
static DEVICE_ATTR(airpress_set_calidata, 0664,
	show_airpress_set_calidata, store_airpress_set_calidata);
static DEVICE_ATTR(gyro_set_calidata, 0660,
	show_gyro_set_calidata, store_gyro_set_calidata);
static DEVICE_ATTR(gyro1_set_calidata, 0660,
	show_gyro1_set_calidata, store_gyro1_set_calidata);
static DEVICE_ATTR(acc_set_calidata, 0660,
	show_acc_set_calidata, store_acc_set_calidata);
static DEVICE_ATTR(acc1_set_calidata, 0660,
	show_acc1_set_calidata, store_acc1_set_calidata);
static DEVICE_ATTR(set_data_type, 0220, NULL, store_set_data_type);

int ois_commu(int tag, unsigned int cmd, unsigned int pare,
	unsigned int responsed, bool is_subcmd)
{
	int ret;
	write_info_t pkg_ap = { 0 };
	read_info_t pkg_mcu = { 0 };
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;

	if (is_subcmd) {
		pkg_ap.tag = tag;
		pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
		cpkt.subcmd = cmd;
		pkg_ap.wr_buf = &hd[1];
		pkg_ap.wr_len = sizeof(pare) + SUBCMD_LEN;
		memcpy(cpkt.para, &pare, sizeof(pare));
	} else {
		pkg_ap.tag = tag;
		pkg_ap.cmd = cmd;
		pkg_ap.wr_buf = &pare;
		pkg_ap.wr_len = sizeof(pare);
	}

	if (responsed == NO_RESP) {
		ret = write_customize_cmd(&pkg_ap, NULL, true);
		if (ret) {
			hwlog_err("send ois cmd %d to mcu fail,ret=%d\n", cmd, ret);
			return ret;
		}
	} else {
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);
		if (ret) {
			hwlog_err("send ois gyro cfg cmd failed, ret = %d\n", ret);
			return ret;
		}
		if (pkg_mcu.errno != 0) {
			hwlog_err("set ois gyro cfg cmd fail,err=%d\n", pkg_mcu.errno);
		} else {
			hwlog_info("set ois gyro cfg cmd %d success\n", pare);
			sensor_status.gyro_ois_status = pare;
		}
	}

	return ret;
}

static ssize_t show_ois_ctrl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_STR_SIZE, "%d\n", sensor_status.gyro_ois_status);
}

static ssize_t store_ois_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long source;
	int ret;
	unsigned int cmd;
	unsigned int delay = 10;

	if (kstrtoul(buf, 10, &source))
		return -EINVAL;

	if (source == sensor_status.gyro_ois_status) {
		hwlog_info("%s:gyro ois status unchange,source=%ld return\n",
			__func__, source);
		return size;
	}

	if (source == 1) {
		cmd = CMD_CMN_OPEN_REQ;
		ret = ois_commu(TAG_OIS, cmd, source, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: ois open gyro fail\n", __func__);
			return size;
		}

		cmd = CMD_CMN_INTERVAL_REQ;
		ret = ois_commu(TAG_OIS, cmd, delay, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: set delay fail\n", __func__);
			return size;
		}

		cmd = SUB_CMD_GYRO_OIS_REQ;
		ret = ois_commu(TAG_GYRO, cmd, source, RESP, true);
		if (ret) {
			hwlog_err("%s: ois enable fail\n", __func__);
			return size;
		}
		hwlog_info("%s:ois enable succsess\n", __func__);
	} else if (source == 0) {
		cmd = SUB_CMD_GYRO_OIS_REQ;
		ret = ois_commu(TAG_GYRO, cmd, source, RESP, true);
		if (ret) {
			hwlog_err("%s:ois close fail\n", __func__);
			return size;
		}
		cmd = CMD_CMN_CLOSE_REQ;
		ret = ois_commu(TAG_OIS, cmd, source, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: ois close gyro fail\n", __func__);
			return size;
		}
		hwlog_info("%s:ois close succsess\n", __func__);
	} else if (source == 2) {
		cmd = SUB_CMD_GYRO_OIS_REQ;
		ret = ois_commu(TAG_GYRO, cmd, source, RESP, true);
		if (ret) {
			hwlog_err("%s: ois enable fail\n", __func__);
			return size;
		}
		hwlog_info("%s:ois reset succsess\n", __func__);
	} else if (source == 3) {
		source = 1;
		cmd = CMD_CMN_OPEN_REQ;
		ret = ois_commu(TAG_OIS, cmd, source, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: ois open gyro fail\n", __func__);
			return size;
		}

		cmd = CMD_CMN_INTERVAL_REQ;
		ret = ois_commu(TAG_OIS, cmd, delay, NO_RESP, false);
		if (ret) {
			hwlog_err("%s: set delay fail\n", __func__);
			return size;
		}
		cmd = SUB_CMD_GYRO_OIS_REQ;
		ret = ois_commu(TAG_GYRO, cmd, source, RESP, true);
		if (ret) {
			hwlog_err("%s: ois enable no_resp fail\n", __func__);
			return size;
		}
		hwlog_info("%s:ois enable succsess\n", __func__);
	} else {
		hwlog_info("%s:ois commend is not right\n", __func__);
	}
	return size;
}
static DEVICE_ATTR(ois_ctrl, 0664, show_ois_ctrl, store_ois_ctrl);

static DEVICE_ATTR(sar_data, 0444, show_sar_data, NULL);
static DEVICE_ATTR(hifi_supported, 0664, show_hifi_supported, NULL);
static DEVICE_ATTR(sensor_detect, 0660, show_sensor_detect, store_sensor_detect);

static struct attribute *sensor_attributes[] = {
	&dev_attr_acc_info.attr,
	&dev_attr_mag_info.attr,
	&dev_attr_gyro_info.attr,
	&dev_attr_ps_info.attr,
	&dev_attr_als_info.attr,
	&dev_attr_als1_info.attr,
	&dev_attr_als2_info.attr,
	&dev_attr_acc_read_data.attr,
	&dev_attr_mag_read_data.attr,
	&dev_attr_gyro_read_data.attr,
	&dev_attr_ps_read_data.attr,
	&dev_attr_als_read_data.attr,
	&dev_attr_als1_read_data.attr,
	&dev_attr_als2_read_data.attr,
	&dev_attr_gyro_selfTest.attr,
	&dev_attr_mag_selfTest.attr,
	&dev_attr_acc_selfTest.attr,
	&dev_attr_connectivity_selfTest.attr,
	&dev_attr_i2c_rw.attr,
	&dev_attr_i2c_rw16.attr,
	&dev_attr_i2c_rw_16bit_reg.attr,
	&dev_attr_acc_calibrate.attr,
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_setdelay.attr,
	&dev_attr_acc_set_calidata.attr,
	&dev_attr_acc1_set_calidata.attr,
	&dev_attr_set_data_type.attr,
	&dev_attr_set_fingersense_enable.attr,
	&dev_attr_fingersense_req_data.attr,
	&dev_attr_fingersense_data_ready.attr,
	&dev_attr_fingersense_latch_data.attr,
	&dev_attr_gsensor_gather_enable.attr,
	&dev_attr_gyro_calibrate.attr,
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_setdelay.attr,
	&dev_attr_gyro_set_calidata.attr,
	&dev_attr_gyro1_set_calidata.attr,
	&dev_attr_mag_enable.attr,
	&dev_attr_mag_setdelay.attr,
	&dev_attr_als_calibrate.attr,
	&dev_attr_als1_calibrate.attr,
	&dev_attr_als2_calibrate.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_als1_enable.attr,
	&dev_attr_als2_enable.attr,
	&dev_attr_als_setdelay.attr,
	&dev_attr_als1_setdelay.attr,
	&dev_attr_als2_setdelay.attr,
	&dev_attr_ps_calibrate.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_setdelay.attr,
	&dev_attr_pdr_enable.attr,
	&dev_attr_pdr_setdelay.attr,
	&dev_attr_orientation_enable.attr,
	&dev_attr_orientation_setdelay.attr,
	&dev_attr_lines_enable.attr,
	&dev_attr_lines_setdelay.attr,
	&dev_attr_gras_enable.attr,
	&dev_attr_gras_setdelay.attr,
	&dev_attr_rvs_enable.attr,
	&dev_attr_rvs_setdelay.attr,
	&dev_attr_sensor_list_info.attr,
	&dev_attr_iom3_recovery.attr,
	&dev_attr_sensor_test.attr,
	&dev_attr_iom3_sr_test.attr,
	&dev_attr_dt_motion_stup.attr,
	&dev_attr_dt_sensor_stup.attr,
	&dev_attr_dt_stop_auto_data.attr,
	&dev_attr_dt_hall_sensor_stup.attr,
	&dev_attr_dt_stop_als_auto_data.attr,
	&dev_attr_dt_als_sensor_stup.attr,
	&dev_attr_dt_stop_ps_auto_data.attr,
	&dev_attr_dt_ps_sensor_stup.attr,
	&dev_attr_dt_stop_auto_motion.attr,
	&dev_attr_airpress_info.attr,
	&dev_attr_airpress_enable.attr,
	&dev_attr_airpress_setdelay.attr,
	&dev_attr_airpress_read_data.attr,
	&dev_attr_airpress_set_calidata.attr,
	&dev_attr_read_airpress.attr,
	&dev_attr_read_temperature.attr,
	&dev_attr_dt_stepcounter_stup.attr,
	&dev_attr_ois_ctrl.attr,
	&dev_attr_iom3_sr_status.attr,
	&dev_attr_dump_sensor_status.attr,
	&dev_attr_cap_prox_calibrate.attr,
	&dev_attr_cap_prox_enable.attr,
	&dev_attr_cap_prox_setdelay.attr,
	&dev_attr_sar_data.attr,
	&dev_attr_magn_bracket_enable.attr,
	&dev_attr_magn_bracket_setdelay.attr,
	&dev_attr_hifi_supported.attr,
	&dev_attr_thermometer_info.attr,
	&dev_attr_sensor_detect.attr,
	NULL
};

static const struct attribute_group sensor_node = {
	.attrs = sensor_attributes,
};

static struct platform_device sensor_input_info = {
	.name = "huawei_sensor",
	.id = -1,
};
static int __init sensor_input_info_init(void)
{
	int ret;

	if (!is_scp_ready(SCP_A_ID))
		return -1;

	hwlog_info("[%s] ++\n", __func__);
	spin_lock_init(&fsdata_lock);
	ret = platform_device_register(&sensor_input_info);
	if (ret) {
		hwlog_err("%s: platform_device_register failed %d\n", __func__, ret);
		goto REGISTER_ERR;
	}

	ret = sysfs_create_group(&sensor_input_info.dev.kobj, &sensor_node);
	if (ret) {
		hwlog_err("%s sysfs_create_group error ret =%d\n",
			__func__, ret);
		goto SYSFS_CREATE_CGOUP_ERR;
	}
	hwlog_info("[%s] --\n", __func__);
	return 0;
SYSFS_CREATE_CGOUP_ERR:
	platform_device_unregister(&sensor_input_info);
REGISTER_ERR:
	return ret;
}

late_initcall_sync(sensor_input_info_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sensor input info");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
