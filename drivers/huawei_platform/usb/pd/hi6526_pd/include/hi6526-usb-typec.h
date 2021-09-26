/*
 * Copyright (C) 2018 Huawei Technology Co.
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

#ifndef _LOCAL_HI6526_USB_TYPEC_H_
#define _LOCAL_HI6526_USB_TYPEC_H_

#include <linux/notifier.h>

#define ADAPTER_CAP_MAX_NR 10

struct adapter_power_cap {
	uint8_t selected_cap_idx;
	uint8_t nr;
	uint8_t pdp;
	uint8_t pwr_limit[ADAPTER_CAP_MAX_NR];
	int32_t max_mv[ADAPTER_CAP_MAX_NR];
	int32_t min_mv[ADAPTER_CAP_MAX_NR];
	int32_t ma[ADAPTER_CAP_MAX_NR];
	int32_t maxwatt[ADAPTER_CAP_MAX_NR];
	int32_t minwatt[ADAPTER_CAP_MAX_NR];
	uint8_t type[ADAPTER_CAP_MAX_NR];
	int info[ADAPTER_CAP_MAX_NR];
};

enum adapter_cap_type {
	HI6526_PD_APDO_START,
	HI6526_PD_APDO_END,
	HI6526_PD,
	HI6526_PD_APDO,
	HI6526_CAP_TYPE_UNKNOWN,
};

enum adapter_return_value {
	ADAPTER_OK = 0,
	ADAPTER_NOT_SUPPORT,
	ADAPTER_TIMEOUT,
	ADAPTER_REJECT,
	ADAPTER_ERROR,
	ADAPTER_ADJUST,
};

int hi6526_usb_typec_register_pd_dpm(void);
void hi6526_usb_typec_register_tcpc_device(struct hi6526_tcpc_device *tcpc_dev);
void hi6526_usb_typec_cc_status_change(uint8_t cc1, uint8_t cc2);
void hi6526_usb_typec_cc_alert(uint8_t cc1, uint8_t cc2);
void hi6526_usb_pd_ufp_update_dock_svid(uint16_t svid);
void hi6526_pd_put_cc_detached_event(struct hi6526_tcpc_device *tcpc_dev);

bool usb_typec_charger_type_pd(void);
void usb_typec_cc_ovp_dmd_report(void);
void usb_typec_set_vconn(int enable);
int usb_typec_otg_pwr_src(void);

void hi6526_tcp_ny_vbus_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, int mv, int ma, uint8_t type);
void hi6526_tcp_ny_en_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, bool en_state);
void hi6526_tcp_ny_swap_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t new_role);
void hi6526_tcp_ny_typec_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t new_state, uint8_t old_state,
	uint8_t polarity, uint8_t rp_level);
void hi6526_tcp_ny_pd_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t pd_connect_state);

#endif
