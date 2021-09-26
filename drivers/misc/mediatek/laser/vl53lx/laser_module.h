/*
 * laser_module.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * laser module header file
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

#ifndef _LASER_MODULE_H_
#define _LASER_MODULE_H_

#include <linux/module.h>
#include <linux/i2c.h>
#include <media/laser_cfg.h>

#define laser_err(fmt, ...) pr_err("[laser]ERROR: " fmt "\n", ##__VA_ARGS__)
#define laser_info(fmt, ...) pr_info("[laser]INFO: " fmt "\n", ##__VA_ARGS__)
#define laser_dbg(fmt, ...) pr_debug("[laser]DBG: " fmt "\n", ##__VA_ARGS__)

extern int stmvl53l1_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
extern int stmvl53l1_remove(struct i2c_client *client);
extern long stmvl53l1_ioctl_handler(void *hw_data, unsigned int cmd, void *p);

typedef struct _tag_laser_module_intf_t {
	char *laser_name;
	int (*data_init)(struct i2c_client *client,
		const struct i2c_device_id *id);
	int (*data_remove)(struct i2c_client *client);
	long (*laser_ioctl)(void *hw_data, unsigned int cmd, void *p);
} laser_module_intf_t;

#endif
