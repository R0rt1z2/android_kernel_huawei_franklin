/*
 * hw_comm_pmic.h
 *
 * debug for pmic sensor
 *
 * Copyright (c) 2019 Huawei Technologies Co., Ltd.
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
#ifndef _HW_COMM_PMIC_H_
#define _HW_COMM_PMIC_H_

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
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>

#define return_error_on_null(x) \
	do { \
		if ((x) == NULL) { \
			hwlog_err("%s invalid params %s", __func__, #x); \
			return -1; \
		} \
	} while (0)

#define return_on_null(x) \
	do { \
		if ((x) == NULL) { \
			hwlog_err("%s invalid params %s", __func__, #x); \
			return; \
		} \
	} while (0)

#ifndef array_size
#define array_size(x) (sizeof(x) / sizeof((x)[0]))
#endif

/******************** pmic extern common func cfg declare ********************/
/** not allowed to add pmic types below without the permission of the owner **/
enum pmic_seq_index {
	VOUT_LDO_1 = 0,
	VOUT_LDO_2,
	VOUT_LDO_3,
	VOUT_LDO_4,
	VOUT_LDO_5,
	VOUT_BUCK_1,
	VOUT_BUCK_2,
	VOUT_MAX,
	VOUT_BOOST,
	VOUT_BOOST_EN,
	MAX_SEQ_INDEX,
};

// typedef pmic_seq_index_t for compatible history code
// exposure of external interfaces
typedef enum pmic_seq_index pmic_seq_index_t;

enum pmic_power_req_src_t {
	TP_PMIC_REQ = 0,
	FP_PMIC_REQ = 1,
	VIB_PMIC_REQ,
	CAM_PMIC_REQ,
	AUDIO_PMIC_REQ,
	DF_PMIC_REQ,
	DOT_PMIC_REQ,
	VCM_PMIC_REQ,
	WBG_PMIC_REQ,
	MAX_PMIC_REQ,
};

enum pmic_type_t {
	MAIN_PMIC = 0,
	SUB_PMIC,
	BUCK,
	MAX_PMIC_NUM,
};

typedef enum {
	PMIC_ERROR_TYPE_START = 0,
	PMIC_WRITE_I2C_FAIL = PMIC_ERROR_TYPE_START,
	PMIC_READ_I2C_FAIL,
	PMIC_OCP_ERROR,
	PMIC_OVP_ERROR,
	PMIC_UVP_ERROR,
	PMIC_ERROR_TYPE_END,
} pmic_error_type;

typedef enum {
	PMIC_ERROR_NONE,
	PMIC_LDO1_ERROR_EVENT,
	PMIC_LDO2_ERROR_EVENT,
	PMIC_LDO3_ERROR_EVENT,
	PMIC_LDO4_ERROR_EVENT,
	PMIC_BUCK_ERROR_EVENT,
	PMIC_BOOST_ERROR_EVENT,
} pmic_fault_event;

struct pmic_error_info_t {
	u8 reg_addr;
	u8 reg_normal_value;
	u8 boost_mask;
	pmic_error_type type;
	int error_no;
};

struct pmic_ctrl_vote_t {
	u32 power_state;
	int power_cnt[MAX_PMIC_REQ];
	u32 pmic_volt[MAX_PMIC_REQ];
};

struct hw_comm_pmic_cfg_t {
	u8 pmic_num;
	pmic_seq_index_t pmic_power_type;
	u32 pmic_power_voltage;
	bool pmic_power_state;
};

enum pmic_buck_mode_t {
	BUCK_NORMAL_MODE,
	BUCK_PWM_MODE,
};

int hw_pmic_power_cfg(enum pmic_power_req_src_t pmic_power_src,
	struct hw_comm_pmic_cfg_t *comm_pmic_cfg);
void hw_pmic_reset(enum pmic_power_req_src_t src,
	struct hw_comm_pmic_cfg_t *pmic_cfg);
/** not allowed to add pmic types above without the permission of the owner **/
/********************** pmic controler struct define **********************/
struct hw_pmic_info {
	const char *name;
	int index;
	unsigned int slave_address;
	unsigned int intr;
	unsigned int irq;
	unsigned int irq_flag;
	unsigned int boost_shield_dmd;
	unsigned int boost_shield_irq;
	unsigned int boost_en_pin;
	unsigned int power_vote[VOUT_BOOST];
	unsigned int fault_check;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pmic_intr;
	pmic_fault_event event_type;
};

struct hw_pmic_i2c_client {
	struct hw_pmic_i2c_fn_t *i2c_func_tbl;
	struct i2c_client *client;
};

struct hw_pmic_i2c_fn_t {
	int (*i2c_read)(struct hw_pmic_i2c_client *, u8, u8 *);
	int (*i2c_write)(struct hw_pmic_i2c_client *, u8, u8);
};

/* pmic function table */
struct hw_pmic_ctrl_t;
struct hw_pmic_fn_t;
/***************define pmic globle var******************/
#define PMIC_POWER_OFF 0x00
#define PMIC_POWER_ON  0x01
#define PMIC_VOTE_ON   1
#define PMIC_VOTE_OFF  0

/***************pmic comm func declare******************/
#define DEFINE_HW_PMIC_MUTEX(name) \
	static struct mutex pmic_mut_##name = __MUTEX_INITIALIZER(pmic_mut_##name)

/***************extern function declare******************/
int hw_pmic_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
struct hw_pmic_ctrl_t *hw_get_pmic_ctrl(void);
void hw_pmic_release_intr(struct hw_pmic_ctrl_t *pmic_ctrl);
int hw_pmic_setup_intr(struct hw_pmic_ctrl_t *pmic_ctrl);
int hw_pmic_gpio_boost_enable(struct hw_pmic_ctrl_t *pmic_ctrl, int state);
void pmic_fault_check(struct hw_pmic_ctrl_t *pmic_ctrl, u8 stat_reg,
	struct pmic_error_info_t *error_map, unsigned int map_size);
int pmic_fault_notifier_register(struct notifier_block *nb);
int pmic_fault_notifier_unregister(struct notifier_block *nb);
void pmic_fault_notify(struct hw_pmic_ctrl_t *pmic_ctrl);

#endif
