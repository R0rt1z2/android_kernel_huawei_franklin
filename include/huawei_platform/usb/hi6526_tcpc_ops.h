/*
 * Hi6526 I2C&GPIO IRG interfaces for Hi6526 charger module.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _HI6526_TCPC_OPS_H_
#define _HI6526_TCPC_OPS_H_

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/notifier.h>

struct hi6526_tcpc_device;

struct tcp_dpm_event_cb_data;

struct pd_pps_status;

struct pd_status;

struct tcpm_power_cap_val;

struct pd_source_cap_ext;

struct tcpm_remote_power_cap;

struct hi6526_tcpc_reg_ops {
	int (*block_read)(u16 reg, u8 *dst, unsigned len);
	int (*block_write)(u16 reg, u8 *src, unsigned len);
};

#ifdef CONFIG_TCPC_HI6526

bool hi6526_tcpc_ready(void);

void hi6526_tcpc_reg_ops_register(struct i2c_client *client,
		struct hi6526_tcpc_reg_ops *reg_ops);
void hi6526_tcpc_irq_gpio_register(struct i2c_client *client, int irq_gpio);
void hi6526_tcpc_set_vusb_uv_det_sts(bool en);
void hi6526_tcpc_notify_chip_version(unsigned int version);

uint8_t hi6526_get_no_rpsrc_state(void);

struct hi6526_tcpc_device *hi6526_get_tcpc_dev(void);
int hi6526_register_tcpc_notifier(struct notifier_block *nb);
int hi6526_unregister_tcpc_notifier(struct notifier_block *nb);

uint8_t hi6526_tcpm_inquire_typec_attach_state(struct hi6526_tcpc_device *tcpc_dev);

int hi6526_tcpm_inquire_vbus_level(struct hi6526_tcpc_device *tcpc_dev, bool from_ic);

int hi6526_tcpm_inquire_typec_remote_rp_curr(struct hi6526_tcpc_device *tcpc_dev);

int hi6526_tcpm_set_apdo_charging_policy(struct hi6526_tcpc_device *tcpc,
	uint8_t policy, int mv, int ma,
	const struct tcp_dpm_event_cb_data *cb_data);

int hi6526_tcpm_set_pd_charging_policy(struct hi6526_tcpc_device *tcpc,
	uint8_t policy, const struct tcp_dpm_event_cb_data *cb_data);

int hi6526_tcpm_dpm_pd_request(struct hi6526_tcpc_device *tcpc,
	int mv, int ma, const struct tcp_dpm_event_cb_data *cb_data);

int hi6526_tcpm_dpm_pd_get_pps_status(struct hi6526_tcpc_device *tcpc,
	const struct tcp_dpm_event_cb_data *cb_data,
	struct pd_pps_status *pps_status);

int hi6526_tcpm_dpm_pd_get_status(struct hi6526_tcpc_device *tcpc,
	const struct tcp_dpm_event_cb_data *cb_data, struct pd_status *status);

int hi6526_tcpm_inquire_pd_source_apdo(struct hi6526_tcpc_device *tcpc,
	uint8_t apdo_type, uint8_t *cap_i, struct tcpm_power_cap_val *cap_val);

int hi6526_tcpm_dpm_pd_get_source_cap_ext(struct hi6526_tcpc_device *tcpc,
	const struct tcp_dpm_event_cb_data *cb_data,
	struct pd_source_cap_ext *src_cap_ext);

int hi6526_tcpm_get_remote_power_cap(struct hi6526_tcpc_device *tcpc_dev,
		struct tcpm_remote_power_cap *remote_cap);

#else

bool hi6526_tcpc_ready(void)
{
	return false;
}

uint8_t hi6526_get_no_rpsrc_state(void)
{
	return 0;
}

static inline void hi6526_tcpc_reg_ops_register(struct i2c_client *client,
	struct hi6526_tcpc_reg_ops *reg_ops)
{
}

static inline void hi6526_tcpc_irq_gpio_register(struct i2c_client *client,
	int irq_gpio)
{
}

static inline void hi6526_tcpc_set_vusb_uv_det_sts(bool en)
{
}

static inline void hi6526_tcpc_notify_chip_version(unsigned int version)
{
}

int hi6526_register_tcpc_notifier(struct notifier_block *nb)
{
	pr_info("Not Support Hi6526 PD!\n");
	return -EPERM;
}

int hi6526_unregister_tcpc_notifier(struct notifier_block *nb)
{
	pr_info("Not Support Hi6526 PD!\n");
	return -EPERM;
}

struct hi6526_tcpc_device {
	int hi6526_tcpc_not_supported;
};

struct hi6526_tcpc_device *hi6526_get_tcpc_dev(void)
{
	pr_err("Not Support Hi6526 PD!\n");
	return NULL;
}

#endif /* CONFIG_TCPC_HI6526 */

#endif /* _HI6526_TCPC_OPS_H_ */

