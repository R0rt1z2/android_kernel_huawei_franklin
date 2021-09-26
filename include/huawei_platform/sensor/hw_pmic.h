/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _HW_PMIC_H_
#define _HW_PMIC_H_

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <dsm/dsm_pub.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <huawei_platform/sensor/hw_comm_pmic.h>

#define DEFINE_HISI_PMIC_MUTEX(name) \
	static struct mutex pmic_mut_##name = __MUTEX_INITIALIZER(pmic_mut_##name)

/********************** v4l2 subdev ioctl case id define **********************/
struct pmic_cfg_data;

/***************************** cfg type define *******************************/
#define CFG_PMIC_INIT			0
#define CFG_PMIC_POWER_ON		1
#define CFG_PMIC_POWER_OFF		2
#define BUCK_REG_MAX			90
#define LDO_REG_MAX			66
#define PMIC_ENABLE_CNT                 1
#define PMIC_DISABLE_CNT                0
/********************** pmic controler struct define **********************/
struct hw_pmic_info;
struct hw_pmic_ctrl_t;

struct hw_pmic_fn_t {
	/* pmic function table */
	int (*pmic_config) (struct hw_pmic_ctrl_t *, void *);
	int (*pmic_on) (struct hw_pmic_ctrl_t *, void *);
	int (*pmic_off) (struct hw_pmic_ctrl_t *);
	int (*pmic_init) (struct hw_pmic_ctrl_t *);
	int (*pmic_exit) (struct hw_pmic_ctrl_t *);
	int (*pmic_match) (struct hw_pmic_ctrl_t *);
	int (*pmic_get_dt_data) (struct hw_pmic_ctrl_t *);
	int (*pmic_seq_config)(struct hw_pmic_ctrl_t *, pmic_seq_index_t, u32, int);
	int (*pmic_register_attribute)(struct hw_pmic_ctrl_t *, struct device *);
	int (*pmic_check_exception)(struct hw_pmic_ctrl_t *);
	int (*pmic_power_cfg)(enum pmic_power_req_src_t pmic_power_src,
		struct hw_comm_pmic_cfg_t *comm_pmic_cfg);
	void (*pmic_reset)(enum pmic_power_req_src_t src,
		struct hw_comm_pmic_cfg_t *pmic_cfg);
};

struct hw_pmic_i2c_client;
struct hw_pmic_i2c_fn_t;
struct hw_pmic_ctrl_t {
	struct platform_device *pdev;
	struct mutex *hw_pmic_mutex;
	struct device *dev;
	struct pinctrl *pctrl;
	struct v4l2_subdev subdev;
	struct v4l2_subdev_ops *pmic_v4l2_subdev_ops;
	struct hw_pmic_fn_t *func_tbl;
	struct hw_pmic_i2c_client *pmic_i2c_client;
	struct hw_pmic_info pmic_info;
	void *pdata;
	struct delayed_work pmic_err_work;
	struct wakeup_source pmic_wake_lock;
};

struct irq_err_monitor {
	struct timeval now;
	struct timeval last_time;
	long irq_time;
};

/********************* cfg data define ************************************/

struct pmic_i2c_reg {
	unsigned int address;
	unsigned int value;
};

struct pmic_cfg_data {
	int cfgtype;
	int mode;
	int data;

	union {
	char name[32];
	} cfg;
};

/***************extern function declare******************/
int32_t hw_pmic_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
int hw_pmic_config(struct hw_pmic_ctrl_t *pmic_ctrl, void *argp);
struct hw_pmic_ctrl_t *hw_get_pmic_ctrl(void);
int pmic_enable_boost(int value);
int pmic_ctl_otg_onoff(bool on_off);
void pmic_fault_reset_check(struct hw_pmic_ctrl_t *pmic_ctrl,
	struct irq_err_monitor *irq_err, unsigned int latch_time,
	const unsigned int sche_work_time);
void pmic_wake_lock(struct hw_pmic_ctrl_t *pmic_ctrl);
void pmic_wake_unlock(struct hw_pmic_ctrl_t *pmic_ctrl);
void hw_pmic_release_intr(struct hw_pmic_ctrl_t *pmic_ctrl);
int hw_pmic_setup_intr(struct hw_pmic_ctrl_t *pmic_ctrl);
int hw_pmic_gpio_boost_enable(struct hw_pmic_ctrl_t *pmic_ctrl, int state);

#endif
