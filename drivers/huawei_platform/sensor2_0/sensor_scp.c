/*
 * sensor_scp.c
 *
 * code for sensor scp
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

#include "sensor_scp.h"
#include "securec.h"
#include "motion.h"
#include "motion_route.h"
#include "sensor_para.h"
#ifdef CONFIG_MACH_MT6853
#include "scp_helper.h"
#endif
#include <linux/suspend.h>
#include <linux/fb.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>

#define pr_fmt(fmt) "[scp_ctrl] " fmt
#define FINGER_SENSOR_DATA_READY   1
#define EXT_PEDO_VERSION           2000
#define EXT_PEDO_VERSION_SIZE      36
#define EXT_PEDO_MAX_LEN_SIZE      600
#define EXT_PEDO_DATA_LEN          480
static struct scp_ctrl_t scp_ctrl;
static int scp_sensor_delay[SENSOR_TYPE_SENSOR_MAX];
static int scp_sensor_test_rst[SCP_SENSOR_TEST_MAX][SENSOR_TYPE_SENSOR_MAX];
struct timeval curr_time;
static struct extend_step_para_t *extend_step;
static int g_aod_type = 0;

enum ap_scp_system_status_t {
	SCP_SCREEN_ON,
	SCP_SCREEN_OFF,
	SCP_SYSTEM_SLEEP,
};

void send_sensor_scp_udfp(unsigned int value)
{
	struct hf_manager_cmd cmd_in;

	cmd_in.sensor_type = SENSOR_TYPE_AOD;
	cmd_in.action = HF_MANAGER_SENSOR_CONFIG_CALI;
	cmd_in.data[0] = value;
	cmd_in.data[1] = 0;
	scp_sensor_cfg_data(&cmd_in);
}

int scp_get_sensor_info(int sensor_type, struct sensor_info *info)
{
	int ret;

	if (!scp_ctrl.client)
		return -EINVAL;

	ret = hf_client_find_sensor(scp_ctrl.client, sensor_type);
	if (ret) {
		pr_err("%s: not support sensor_type-%d\n", __func__, sensor_type);
		return -EINVAL;
	}

	ret = hf_client_get_sensor_info(scp_ctrl.client, sensor_type, info);
	if (ret) {
		pr_err("%s : fail! sensor_type-%d\n", __func__, sensor_type);
		return -EINVAL;
	}

	return 0;
}

int scp_sensor_ctrl_enable(int sensor_type, bool en)
{
	struct scp_ctrl_t scp_ct;

	scp_ct.sensor_type = sensor_type;
	scp_ct.delay = 0;
	if (en)
		scp_ct.action = HF_MANAGER_SENSOR_ENABLE;
	else
		scp_ct.action = HF_MANAGER_SENSOR_DISABLE;

	return send_scp_ctrl_cmd(&scp_ct);
}

int scp_sensor_cfg_data(struct hf_manager_cmd *cmd_in)
{
	int ret;
	struct hf_manager_cmd cmd;

	if (!scp_ctrl.client || !cmd_in)
		return -EINVAL;

	ret = hf_client_find_sensor(scp_ctrl.client, cmd_in->sensor_type);
	if (ret) {
		pr_err("hf_client_find_sensor %u fail!\n",
			scp_ctrl.sensor_type);
		return -EINVAL;
	}
	pr_info("%s\n", __func__);
	ret = hf_client_control_sensor(scp_ctrl.client, cmd_in);
	return ret;
}

int send_scp_ctrl_cmd(struct scp_ctrl_t *scp_ct)
{
	int ret;
	struct hf_manager_cmd cmd;

	if (!scp_ctrl.client || !scp_ct)
		return -EINVAL;

	scp_ctrl.sensor_type = scp_ct->sensor_type;
	scp_ctrl.action = scp_ct->action;
	scp_ctrl.delay = scp_ct->delay;

	ret = hf_client_find_sensor(scp_ctrl.client, scp_ctrl.sensor_type);
	if (ret) {
		pr_err("hf_client_find_sensor %u fail!\n",
			scp_ctrl.sensor_type);
		return -EINVAL;
	}

	memset_s(&cmd, sizeof(cmd), 0, sizeof(cmd));
	cmd.sensor_type = scp_ctrl.sensor_type;
	cmd.action = scp_ctrl.action;
	cmd.delay = scp_ctrl.delay;
	cmd.latency = 0;
	pr_info("%s sensor-%d, action-%d, delay-%d\n", __func__,
		cmd.sensor_type, cmd.action, cmd.delay);

	switch (cmd.action) {
	case CALI_ACTION:
		scp_ctrl.client->request[scp_ctrl.sensor_type].cali = true;
		break;

	case TEST_ACTION:
		scp_ctrl.client->request[scp_ctrl.sensor_type].test = true;
		break;

	default:
		break;
	}

	ret = hf_client_control_sensor(scp_ctrl.client, &cmd);
	return ret;
}

int send_scp_custom_cmd(int sensor_type, uint8_t order)
{
	int ret;

	pr_info("%s : sensor_type=%d order=%u\n", __func__, sensor_type, order);
	ret = hf_client_find_sensor(scp_ctrl.client, sensor_type);
	if (ret) {
		pr_err("%s : hf_client_find_sensor sar fail!\n", __func__);
		return -EINVAL;
	}
	struct custom_cmd cmd;
	cmd.data[0] = order;
	ret = hf_client_custom_cmd(scp_ctrl.client, sensor_type, &cmd);
	if (ret) {
		pr_err("%s : hf_client_custom_cmd!\n", __func__);
		return -EINVAL;
	}

	return ret;
}

int send_scp_common_cmd(int sensor_type, struct custom_cmd *cmd)
{
	int ret = hf_client_find_sensor(scp_ctrl.client, sensor_type);
	if (ret) {
		pr_err("%s : hf_client_find_sensor fail!\n", __func__);
		return -EINVAL;
	}
	ret = hf_client_custom_cmd(scp_ctrl.client, sensor_type, cmd);
	if (ret) {
		pr_err("%s : hf_client_custom_cmd!\n", __func__);
		return -EINVAL;
	}
	return ret;
}

int scp_sensor_test_result(int sensor_type,
	enum scp_sensor_test_type_t test_type)
{
	if (sensor_type >= SENSOR_TYPE_SENSOR_MAX)
		return -1;
	if (test_type >= SCP_SENSOR_TEST_MAX)
		return -1;
	return scp_sensor_test_rst[test_type][sensor_type];
}

static void scp_sensor_recv_test(struct hf_manager_event *data)
{
	if (data->sensor_type >= SENSOR_TYPE_SENSOR_MAX)
		return;

	if (data->action == CALI_ACTION) {
		scp_sensor_test_rst[SCP_SENSOR_CALI][data->sensor_type] =
			data->accurancy;
	} else if (data->action == TEST_ACTION) {
		scp_sensor_test_rst[SCP_SENSOR_SELFTEST][data->sensor_type] =
			data->word[0];
	}
}

static void get_extend_step_share_mem_addr(void)
{
	static bool shr_extend_step_mem_ready;

	if (!shr_extend_step_mem_ready) {
		extend_step = get_sensor_share_mem_addr(SHR_MEM_TYPE_PEDO);
		if (!extend_step) {
			pr_err("extend_step share dram not ready\n");
			return;
		} else {
			pr_info("extend_step share dram ready\n");
			shr_extend_step_mem_ready = true;
		}
	}
}

static void adapt_exend_step_timestamp(void)
{
	uint64_t exted_occur_rtc_time_us;
	uint64_t exted_recv_rtc_time_us;
	uint64_t gap_time_us;

	gap_time_us = extend_step->gap_time / 1000; // convert to us
	do_gettimeofday(&curr_time);
	exted_recv_rtc_time_us = curr_time.tv_sec * (1000000L) + curr_time.tv_usec;
	exted_occur_rtc_time_us = exted_recv_rtc_time_us - gap_time_us;
	extend_step->begin_time = exted_occur_rtc_time_us / 1000000;
	pr_debug("gaptime = %lu , rec_rtc_time = %llu, begin_rtc_time = %llu\n",
		gap_time_us, exted_recv_rtc_time_us, extend_step->begin_time);
}

static void scp_recv_extend_step_data(struct hf_manager_event *data)
{
	int ret;
	char step_counter_data[EXT_PEDO_MAX_LEN_SIZE] = {0};
	uint16_t extend_data_size;

	get_extend_step_share_mem_addr();
	if (!extend_step)
		return;

	adapt_exend_step_timestamp();
	pr_info("%s begin_rtc_time = %u, record_count = %u, total_count = %u\n",
		__func__, extend_step->begin_time, extend_step->record_count,
		extend_step->total_step_count);

	if ((extend_step->record_count > 0) &&
		(extend_step->record_count != EXT_PEDO_VERSION)) {
		extend_data_size = EXT_PEDO_VERSION_SIZE +
			(extend_step->record_count - EXT_PEDO_VERSION) * 4;
		if (extend_data_size > EXT_PEDO_DATA_LEN)
			return;
		pr_info("%s, get ar status = %d, extend_step = %d, size = %d\n",
			__func__, extend_step->action_record[0].ar_record,
			extend_step->action_record[0].step_record, extend_data_size);
		step_counter_data[0] = 11; // 11 means ext step counter
		ret = memcpy_s(&step_counter_data[1], EXT_PEDO_MAX_LEN_SIZE - 1,
			&extend_step->begin_time, extend_data_size);
		if (ret != EOK)
			return;
		motion_route_write(step_counter_data, extend_data_size + 1);
	}
}

static void scp_recv_data_disg_report(struct hf_manager_event *data)
{
	switch (data->sensor_type) {
	case SENSOR_TYPE_HW_MOTION:
		scp_motion_data_report(data);
		break;
	case SENSOR_TYPE_FINGER_SENSE:
		break;
	case SENSOR_TYPE_STEP_COUNTER:
		scp_recv_extend_step_data(data);
		break;
	default:
		break;
	}
}

static int scp_ctrl_kthread(void *arg)
{
	struct hf_manager_event data[POLL_DATA_SIZE];
	int size;
	int i;

	while (!kthread_should_stop()) {
		memset_s(data, sizeof(data), 0, sizeof(data));
		size = hf_client_poll_sensor(scp_ctrl.client, data,
			ARRAY_SIZE(data));
		for (i = 0; i < size; ++i) {
			scp_sensor_recv_test(&data[i]);
			scp_recv_data_disg_report(&data[i]);
			pr_debug("[type-%d,action-%d,%lld,%d,%d,%d]!\n",
				data[i].sensor_type,
				data[i].action,
				data[i].timestamp,
				data[i].word[0],
				data[i].word[1],
				data[i].word[2]);
		}
	}
	return 0;
}

static void tell_ap_status_to_scp(enum ap_scp_system_status_t status)
{
	struct hf_manager_cmd cmd_in;

	cmd_in.sensor_type = SENSOR_TYPE_STEP_COUNTER;
	cmd_in.action = HF_MANAGER_SENSOR_CONFIG_CALI;
	cmd_in.data[0] = status;
	cmd_in.data[1] = 0;

	scp_sensor_cfg_data(&cmd_in);
}

static void get_status_from_ap(enum ap_scp_system_status_t status)
{
	struct hf_manager_cmd app_config;

	if (g_aod_type == 0)
		return;
	app_config.sensor_type = SENSOR_TYPE_PROXIMITY;
	app_config.action = HF_MANAGER_SENSOR_SELFTEST;
	app_config.data[0] = status;
	app_config.data[1] = 0;
	scp_sensor_cfg_data(&app_config);
}

void get_aod_status(void)
{
	struct device_node *aod_support =
		of_find_compatible_node(NULL, NULL, "huawei,huawei_sensor_info");

	if (aod_support == NULL) {
		pr_err("Cannot find aod_support type from dts\n");
		return;
	}
	int ret = of_property_read_u32(aod_support, "aod_type", &g_aod_type);
	if (ret == 0)
		pr_info("%s, get aod_type success : %d\n", __func__, g_aod_type);
}

#ifdef CONFIG_MACH_MT6853
phys_addr_t get_als_reserve_mem_virt(void)
{
	return scp_get_reserve_mem_virt(ALS_MEM_ID);
}

phys_addr_t get_als_reserve_mem_phys(void)
{
	return scp_get_reserve_mem_phys(ALS_MEM_ID);
}

phys_addr_t get_als_reserve_mem_size(void)
{
	return scp_get_reserve_mem_size(ALS_MEM_ID);
}
#endif

static int scp_fb_notifier(struct notifier_block *nb,
	unsigned long action, void *data)
{
	if (!data)
		return NOTIFY_OK;

	switch (action) {
	case FB_EVENT_BLANK: // change finished
	{
		struct fb_event *event = data;
		int *blank = event->data;
		if (registered_fb[0] != event->info) {
			 // only main screen on/off info send to hub
			pr_info("%s, not main screen info, return\n", __func__);
			return NOTIFY_OK;
		}
		switch (*blank) {
		case FB_BLANK_UNBLANK: // screen on
			pr_info("%s, screen change to SCREEN_ON\n", __func__);
			tell_ap_status_to_scp(SCP_SCREEN_ON);
			get_status_from_ap(SCP_SCREEN_ON);
			break;
		case FB_BLANK_POWERDOWN: // screen off
			pr_info("%s, screen change to SCREEN_OFF\n", __func__);
			tell_ap_status_to_scp(SCP_SCREEN_OFF);
			break;
		default:
			pr_err("lcd unknown in %s\n", __func__);
			break;
		}
		break;
	}
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block fb_notify = {
	.notifier_call = scp_fb_notifier,
};

static int scp_pm_notify(struct notifier_block *nb,
	unsigned long mode, void *_unused)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE: // suspend
		pr_info("suspend in %s\n", __func__);
		break;

	case PM_POST_SUSPEND: // resume
		pr_info("resume in %s\n", __func__);
		break;

	case PM_HIBERNATION_PREPARE: // Going to hibernate
	case PM_POST_HIBERNATION: // Hibernation finished
	case PM_RESTORE_PREPARE: // Going to restore a saved image
	case PM_POST_RESTORE: // Restore failed
	default:
		break;
	}

	return 0;
}

static int __init scp_ctrl_init(void)
{
	struct hf_client *client = NULL;

	memset_s(scp_sensor_test_rst, sizeof(scp_sensor_test_rst),
		-1, sizeof(scp_sensor_test_rst));
	client = hf_client_create();
	if (!client) {
		pr_err("hf_client_create fail!\n");
		return -ENOMEM;
	}
	scp_ctrl.client = client;

	scp_ctrl.task = kthread_run(scp_ctrl_kthread, &scp_ctrl, "scp_ctrl");
	if (IS_ERR(scp_ctrl.task))
		pr_err("kthread_run create fail!\n");

	fb_register_client(&fb_notify);
	pm_notifier(scp_pm_notify, 0);
	return 0;
}

module_init(scp_ctrl_init);

MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_DESCRIPTION("Sensor SCP scp driver");
MODULE_LICENSE("GPL v2");
