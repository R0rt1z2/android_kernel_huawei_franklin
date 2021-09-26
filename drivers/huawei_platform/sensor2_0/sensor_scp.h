/*
 * sensor_sysfs.h
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

#ifndef __SENSOR_SCP_H__
#define __SENSOR_SCP_H__

#include "hf_manager.h"
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define POLL_DATA_SIZE      4
#define MAX_STR_SIZE        20
struct scp_ctrl_t {
	struct task_struct *task;
	struct hf_client *client;
	struct kobject *kobj;
	int sensor_type;
	int action;
	int delay;
};

struct sensor_cookie {
	int sensor_type;
	const char *name;
	const struct attribute_group *attrs_group;
	struct device *dev;
};

enum scp_sensor_test_type_t {
	SCP_SENSOR_CALI,
	SCP_SENSOR_SELFTEST,
	SCP_SENSOR_TEST_MAX
};

#define CHECK_NULL_ERR(data)                        \
do {                                                \
	if (!data) {                                \
		pr_err("err in %s\n", __func__);    \
		return -EINVAL;                     \
	}                                           \
} while (0)

#define CHECK_SENSOR_COOKIE(data)                                        \
do {                                                                     \
	if (!data || (!(data->sensor_type >= SENSOR_TYPE_ACCELEROMETER &&    \
		data->sensor_type < SENSOR_TYPE_SENSOR_MAX)) ||          \
		(!data->name)) {                                         \
		pr_err("error in %s\n", __func__);                       \
		return -EINVAL;                                          \
	}                                                                \
} while (0)

// function declare here
int scp_get_sensor_info(int sensor_type, struct sensor_info *info);
int send_scp_ctrl_cmd(struct scp_ctrl_t *scp_cr);
int send_scp_custom_cmd(int sensor_type, uint8_t order);
int send_scp_common_cmd(int sensor_type, struct custom_cmd *cmd);
int scp_sensor_test_result(int sensor_type,
	enum scp_sensor_test_type_t test_type);
int scp_sensor_ctrl_enable(int sensor_type, bool en);
int scp_sensor_cfg_data(struct hf_manager_cmd *cmd_in);
int thp_prox_event_report(const int value[], int length);
void send_sensor_scp_udfp(unsigned int value);
void get_aod_status(void);
#ifdef CONFIG_MACH_MT6853
phys_addr_t get_als_reserve_mem_virt(void);
phys_addr_t get_als_reserve_mem_phys(void);
phys_addr_t get_als_reserve_mem_size(void);
#endif

#endif /* __SENSOR_SCP_H__ */

