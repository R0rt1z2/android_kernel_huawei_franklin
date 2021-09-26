/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/debugfs.h>

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif

#include <huawei_platform/charger/huawei_charger.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#include <chipset_common/hwpower/common_module/power_dsm.h>
#include <chipset_common/hwpower/hardware_ic/boost_5v.h>
#include <chipset_common/hwpower/protocol/adapter_protocol_uvdm.h>

#include <securec.h>

#include "include/pd_dbg_info.h"
#include "include/hi6526_tcpm.h"
#include "include/hi6526_typec.h"
#include "include/hi6526-usb-typec.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "[typec]"

#define ANALOG_SINGLE_CC_STS_MASK 0xF
#define ANALOG_CC2_STS_OFFSET 4
#define ANALOG_CC_WITH_UNDEFINED_RESISTANCE 1

struct usb_typec {
	struct platform_device *pdev;
	struct hi6526_tcpc_device *tcpc_dev;

	struct mutex lock;
	struct tcp_ny_vbus_state vbus_state;
	struct tcp_ny_typec_state typec_state;
	struct tcp_ny_pd_state pd_state;
	struct tcp_ny_uvdm uvdm_msg;

	uint16_t dock_svid;

	int power_role;
	int data_role;
	int vconn;
	int audio_accessory;

	bool direct_charge_cable;
	bool direct_charging;
	int pd_adapter_voltage;

	uint8_t rt_cc1;
	uint8_t rt_cc2;
	unsigned long time_stamp_cc_alert;
	unsigned long time_stamp_typec_attached;
	unsigned long time_stamp_pd_attached;

	/* for handling tcpc notify */
	struct notifier_block tcpc_nb;
	struct list_head tcpc_notify_list;
	spinlock_t tcpc_notify_list_lock;
	unsigned int tcpc_notify_count;
	struct work_struct tcpc_notify_work;
	struct workqueue_struct *hi6526_typec_wq;

	struct notifier_block wakelock_control_nb;

	unsigned int suspend_count;
	unsigned int resume_count;

	/* Notifier for MTK PD Manager, USB OTG, Power, Charger */
	struct srcu_notifier_head evt_nh;
};

static struct usb_typec *_typec;

#ifdef CONFIG_TCPC_CLASS

bool hi6526_tcpc_ready(void)
{
	return (_typec != NULL);
}

static bool hi6526_usb_typec_ready(struct usb_typec *typec)
{
	if (!typec || !typec->tcpc_dev) {
		E("usb tcpc not ready\n");
		return false;
	}

	return true;
}

static void handle_typec_unattached(struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
	if (typec->audio_accessory)
		typec->audio_accessory = 0;

	if (typec->data_role == PD_ROLE_UFP)
		typec->data_role = PD_ROLE_UNATTACHED;
	else if (typec->data_role == PD_ROLE_DFP)
		typec->data_role = PD_ROLE_UNATTACHED;

	typec->power_role = PD_ROLE_UNATTACHED;

	typec->direct_charge_cable = false;
	typec->direct_charging = false;
}

static void handle_typec_attached_sink(struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
	if (typec_state->old_state != HI6526_TYPEC_UNATTACHED) {
		D("Data Role Unchanged!\n");
		return;
	}

	if (typec->data_role == PD_ROLE_UFP) {
		D("Already UFP\n");
		return;
	}

	typec->data_role = PD_ROLE_UFP;
}

static void handle_typec_attached_source(struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
	if (typec_state->old_state != HI6526_TYPEC_UNATTACHED) {
		D("Data Role Unchanged!\n");
		return;
	}

	if (typec->data_role == PD_ROLE_DFP) {
		D("Already DFP\n");
		return;
	}

	typec->data_role = PD_ROLE_DFP;
}

static inline void handle_typec_attached_audio_accessory(
		struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
	typec->audio_accessory = true;
}

static inline void handle_typec_attached_debug_accessory(
		struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
}

static inline void handle_typec_attached_debug_accessory_sink(
		struct usb_typec *typec,
		struct tcp_ny_typec_state *typec_state)
{
	typec->data_role = PD_ROLE_UFP;
}

/*
 * Handle typec state change event
 */
static void hi6526_usb_typec_state_change(struct tcp_ny_typec_state *typec_state)
{
	struct usb_typec *typec = _typec;
	int ret;

	mutex_lock(&typec->lock);
	I("typec_state: %s --> %s / %s / %s\n",
		typec_attach_type_name(typec_state->old_state),
		typec_attach_type_name(typec_state->new_state),
		typec_state->polarity ? "fliped" : "normal",
		tcpm_cc_voltage_status_string(typec_state->rp_level));

	/* Save typec_state for futher use. */
	ret = memcpy_s(&typec->typec_state, sizeof(struct tcp_ny_typec_state),
		typec_state, sizeof(typec->typec_state));
	if (ret != EOK)
		E("memcpy_s failed\n");

	switch (typec_state->new_state) {
	case HI6526_TYPEC_UNATTACHED:
		handle_typec_unattached(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_SNK:
		handle_typec_attached_sink(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_SRC:
		handle_typec_attached_source(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_AUDIO:
		handle_typec_attached_audio_accessory(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_DEBUG:
		handle_typec_attached_debug_accessory(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_DBGACC_SNK:
	case HI6526_TYPEC_ATTACHED_CUSTOM_SRC:
		handle_typec_attached_debug_accessory_sink(typec, typec_state);
		break;
	case HI6526_TYPEC_ATTACHED_VBUS_ONLY:
	case HI6526_TYPEC_DETTACHED_VBUS_ONLY:
		break;
	default:
		E("unknown new_sate %u\n", typec_state->new_state);
		break;
	}

	typec->time_stamp_typec_attached = jiffies;

	mutex_unlock(&typec->lock);
}

/*
 * Handle PD state change, save PD state
 * pd_state:Should be one of pd_connect_result.
 */
static void hi6526_usb_typec_pd_state_change(struct tcp_ny_pd_state *pd_state)
{
	struct usb_typec *typec = _typec;

	mutex_lock(&typec->lock);
	typec->pd_state.connected = pd_state->connected;
	typec->time_stamp_pd_attached = jiffies;
	mutex_unlock(&typec->lock);
}

/*
 * Handle data role swap event
 */
static void hi6526_usb_typec_data_role_swap(u8 role)
{
	struct usb_typec *typec = _typec;

	mutex_lock(&typec->lock);

	if (typec->data_role == PD_ROLE_UNATTACHED) {
		E("Unattached!\n");
		goto done;
	}

	if (typec->data_role == role) {
		D("Data role not change!\n");
		goto done;
	}

	I("new role: %s", role == PD_ROLE_DFP ? "PD_ROLE_DFP" : "PD_ROLE_UFP");
	if (role == PD_ROLE_DFP)
		typec->data_role = PD_ROLE_DFP;
	else
		typec->data_role = PD_ROLE_UFP;
done:
	mutex_unlock(&typec->lock);
}

/*
 * Handle source vbus operation
 */
static void hi6526_usb_typec_source_vbus(struct tcp_ny_vbus_state *vbus_state)
{
	struct usb_typec *typec = _typec;
	int ret;

	mutex_lock(&typec->lock);
	I("vbus_state: %d %d 0x%02x\n", vbus_state->ma,
			vbus_state->mv, vbus_state->type);

	/* Must save vbus_state first */
	ret = memcpy_s(&typec->vbus_state, sizeof(struct tcp_ny_vbus_state),
		vbus_state, sizeof(typec->vbus_state));
	if (ret != EOK)
		E("memcpy_s failed\n");

	I("power role: %d, data role: %d\n",
			typec->power_role, typec->data_role);

	if (vbus_state->mv != 0)
		typec->power_role = PD_ROLE_SOURCE;

	mutex_unlock(&typec->lock);
}

/*
 * Handle sink vbus operation
 */
static void hi6526_usb_typec_sink_vbus(struct tcp_ny_vbus_state *vbus_state)
{
	struct usb_typec *typec = _typec;

	mutex_lock(&typec->lock);

	/* save vbus_state. */
	typec->vbus_state.ma = vbus_state->ma;
	typec->vbus_state.mv = vbus_state->mv;
	typec->vbus_state.type = vbus_state->type;
	I("vbus_state: %d %d 0x%02x\n", vbus_state->ma,
			vbus_state->mv, vbus_state->type);
	I("power role: %d, data role: %d\n",
			typec->power_role, typec->data_role);

	if (vbus_state->mv != 0)
		typec->power_role = PD_ROLE_SINK;

	mutex_unlock(&typec->lock);
}

static void hi6526_usb_typec_disable_vbus_control(
		struct tcp_ny_vbus_state *vbus_state)
{
	struct usb_typec *typec = _typec;

	mutex_lock(&typec->lock);

	typec->vbus_state.ma = vbus_state->ma;
	typec->vbus_state.mv = vbus_state->mv;
	typec->vbus_state.type = vbus_state->type;
	I("vbus_state: %d %d 0x%02x\n", vbus_state->ma,
			vbus_state->mv, vbus_state->type);

	typec->power_role = PD_ROLE_UNATTACHED;
	mutex_unlock(&typec->lock);
}
#endif

void hi6526_usb_pd_ufp_update_dock_svid(uint16_t svid)
{
	struct usb_typec *typec = _typec;

	D("0x%04x\n", svid);
	typec->dock_svid = svid;
}

/* Monitor cc status which pass through CCDebounce */
void hi6526_usb_typec_cc_status_change(uint8_t cc1, uint8_t cc2)
{
#ifdef CONFIG_TCPC_CLASS
	pd_dpm_handle_pe_event(PD_DPM_PE_ABNORMAL_CC_CHANGE_HANDLER, NULL);
#endif
}

/*
 * Monitor the raw cc status
 */
void hi6526_usb_typec_cc_alert(uint8_t cc1, uint8_t cc2)
{
	struct usb_typec *typec = _typec;

	/* only record the time of first cc connection. */
	if ((typec->rt_cc1 == TYPEC_CC_VOLT_OPEN) &&
			(typec->rt_cc2 == TYPEC_CC_VOLT_OPEN)) {
		if ((cc1 != TYPEC_CC_VOLT_OPEN) ||
			(cc2 != TYPEC_CC_VOLT_OPEN)) {
			D("update time_stamp_cc_alert\n");
			typec->time_stamp_cc_alert = jiffies;
		}
	}

	/* record real cc status */
	typec->rt_cc1 = cc1;
	typec->rt_cc2 = cc2;
}

struct tcpc_notify {
	struct list_head node;
	struct tcp_notify notify;
	unsigned long tcpc_notify_type;
};

#define TCPC_NOTIFY_MAX_COUNT 4096
static int queue_notify(struct usb_typec *typec, unsigned long action,
		const void *data)
{
	struct tcpc_notify *noti = NULL;

	if (typec->tcpc_notify_count > TCPC_NOTIFY_MAX_COUNT) {
		E("tcpc_notify_list too long, %u\n", typec->tcpc_notify_count);
		return -EBUSY;
	}

	noti = kzalloc(sizeof(*noti), GFP_KERNEL);
	if (!noti) {
		E("No memory!\n");
		return -ENOMEM;
	}

	noti->tcpc_notify_type = action;
	if (memcpy_s(&noti->notify, sizeof(struct tcp_notify),
			data, sizeof(noti->notify)) != EOK)
		E("memcpy_s failed\n");

	spin_lock(&typec->tcpc_notify_list_lock);
	list_add_tail(&noti->node, &typec->tcpc_notify_list);
	typec->tcpc_notify_count++;
	spin_unlock(&typec->tcpc_notify_list_lock);

	return 0;
}

static struct tcpc_notify *get_notify(struct usb_typec *typec)
{
	struct tcpc_notify *noti = NULL;

	spin_lock(&typec->tcpc_notify_list_lock);
	noti = list_first_entry_or_null(&typec->tcpc_notify_list,
				struct tcpc_notify, node);

	if (noti) {
		list_del_init(&noti->node);
		typec->tcpc_notify_count--;
	}
	spin_unlock(&typec->tcpc_notify_list_lock);

	return noti;
}

static void free_notify(struct tcpc_notify *noti)
{
	kfree(noti);
}

#ifdef CONFIG_TCPC_CLASS

static void hi6526_usb_typec_issue_hardreset(void *dev_data)
{
	struct usb_typec *typec = _typec;
	int ret;

	if (!hi6526_usb_typec_ready(typec))
		return;

#ifdef CONFIG_USB_POWER_DELIVERY_SUPPORT
	ret = hi6526_tcpm_hard_reset(typec->tcpc_dev);
	if (ret != TCPM_SUCCESS)
		E("hi6526_tcpm_hard_reset ret %d\n", ret);
#endif
}

static bool hi6526_usb_pd_get_hw_dock_svid_exist(void *client)
{
	struct usb_typec *typec = _typec;

	if (!typec) {
		E("hi6526-tcpc not ready\n");
		return false;
	}

	return (typec->dock_svid == PD_DPM_HW_DOCK_SVID);
}

static int hi6526_usb_typec_mark_direct_charging(void *data, bool direct_charging)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return -ENODEV;

	I("%s\n", direct_charging ? "true" : "false");
	(void)hi6526_tcpm_typec_notify_direct_charge(typec->tcpc_dev,
				direct_charging);
	typec->direct_charging = direct_charging;

	return 1;
}

static void hi6526_usb_typec_set_pd_adapter_voltage(int voltage_mv, void *dev_data)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return;

	I("%d mV\n", voltage_mv);
	typec->pd_adapter_voltage = voltage_mv;

#ifdef CONFIG_USB_POWER_DELIVERY_SUPPORT
	hi6526_tcpm_request_voltage(typec->tcpc_dev, voltage_mv);
#endif
}

static void hi6526_usb_typec_send_uvdm(uint32_t *data,
		uint8_t cnt, bool wait_resp, void *dev_data)
{
#ifdef CONFIG_ADAPTER_PROTOCOL_UVDM
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return;

	if (!data)
		return;

	hi6526_tcpm_send_uvdm(typec->tcpc_dev, cnt, data, wait_resp);
#endif
}

static int hi6526_usb_get_cc_state(void)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return 0;

	return typec->rt_cc1 | (typec->rt_cc2 << 2);
}

static bool hi6526_usb_check_cc_vbus_short(void)
{
	struct usb_typec *typec = _typec;
	uint8_t cc, cc1, cc2;

	if (!hi6526_usb_typec_ready(typec))
		return false;

	cc = hi6526_tcpc_get_cc_from_analog_ch(typec->tcpc_dev);
	cc1 = cc & ANALOG_SINGLE_CC_STS_MASK;
	cc2 = (cc >> ANALOG_CC2_STS_OFFSET) & ANALOG_SINGLE_CC_STS_MASK;

	I("analog CC status: 0x%x\n", cc);

	return (cc1 == ANALOG_CC_WITH_UNDEFINED_RESISTANCE
		|| cc2 == ANALOG_CC_WITH_UNDEFINED_RESISTANCE);
}

static void hi6526_usb_set_cc_mode(int mode)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return;

	hi6526_tcpm_force_cc_mode(typec->tcpc_dev, mode);
}

static int hi6526_usb_pd_dpm_data_role_swap(void *client)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return -1;

	D("DataRole Swap\n");
	return hi6526_tcpm_data_role_swap(typec->tcpc_dev);
}

static void hi6526_usb_detect_emark_cable(void *client)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return;

	D("en emark_cable det\n");
	hi6526_tcpm_detect_emark_cable(typec->tcpc_dev);
}

static void hi6526_usb_force_enable_drp(int mode)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return;

	if (typec->typec_state.new_state == HI6526_TYPEC_UNATTACHED)
		hi6526_tcpm_force_cc_mode(typec->tcpc_dev, mode);
}

static int hi6526_usb_disable_pd(void *client, bool disable)
{
	struct usb_typec *typec = _typec;

	if (!hi6526_usb_typec_ready(typec))
		return -1;

#ifdef CONFIG_USB_POWER_DELIVERY_SUPPORT
	if (disable)
		hi6526_pd_put_cc_detached_event(typec->tcpc_dev);
#endif

	return 0;
}

static struct pd_dpm_ops hi6526_device_pd_dpm_ops = {
	.pd_dpm_get_hw_dock_svid_exist = hi6526_usb_pd_get_hw_dock_svid_exist,
	.pd_dpm_notify_direct_charge_status =
			hi6526_usb_typec_mark_direct_charging,
	.pd_dpm_get_cc_state = hi6526_usb_get_cc_state,
	.pd_dpm_set_cc_mode = hi6526_usb_set_cc_mode,
	.pd_dpm_enable_drp = hi6526_usb_force_enable_drp,
	.pd_dpm_disable_pd = hi6526_usb_disable_pd,
	.pd_dpm_check_cc_vbus_short = hi6526_usb_check_cc_vbus_short,
	.pd_dpm_detect_emark_cable = hi6526_usb_detect_emark_cable,
	.data_role_swap = hi6526_usb_pd_dpm_data_role_swap,
};

static struct pd_protocol_ops hi6526_device_pd_protocol_ops = {
	.chip_name = "scharger_v600",
	.hard_reset_master = hi6526_usb_typec_issue_hardreset,
	.set_output_voltage = hi6526_usb_typec_set_pd_adapter_voltage,
};

static struct uvdm_protocol_ops hi6526_device_uvdm_protocol_ops = {
	.chip_name = "scharger_v600",
	.send_data = hi6526_usb_typec_send_uvdm,
};

/*
 * Check the cable for direct charge or not.
 */
int hi6526_usb_typec_direct_charge_cable_detect(void)
{
	struct usb_typec *typec = _typec;
	uint8_t cc1, cc2;
	int ret;

	if (!hi6526_usb_typec_ready(typec))
		return -1;

	ret = hi6526_tcpm_inquire_remote_cc(typec->tcpc_dev, &cc1, &cc2, true);
	if (ret) {
		E("inquire remote cc failed\n");
		return -1;
	}

	if ((cc1 == PD_DPM_CC_VOLT_SNK_DFT) &&
			(cc2 == PD_DPM_CC_VOLT_SNK_DFT)) {
		I("using \"direct charge cable\" !\n");
		typec->direct_charge_cable = true;
		return 0;
	}

	I("not \"direct charge cable\" !\n");
	typec->direct_charge_cable = false;

	return -1;
}

static struct cc_check_ops direct_charge_cable_check_ops = {
	.is_cable_for_direct_charge = hi6526_usb_typec_direct_charge_cable_detect,
};

struct hi6526_tcpc_device *hi6526_get_tcpc_dev(void)
{
	struct usb_typec *typec = _typec;

	if (unlikely(!typec || !typec->tcpc_dev)) {
		E("%s: tcpc not init yet!\n", __func__);
		return NULL;
	}
	return typec->tcpc_dev;
}

int hi6526_register_tcpc_notifier(struct notifier_block *nb)
{
	struct usb_typec *typec = _typec;

	if (!typec) {
		E("%s: typec not init yet!\n", __func__);
		return -ENODEV;
	}

	return srcu_notifier_chain_register(&typec->evt_nh, nb);
}

int hi6526_unregister_tcpc_notifier(struct notifier_block *nb)
{
	struct usb_typec *typec = _typec;

	if (!typec) {
		E("%s: typec not init yet!\n", __func__);
		return -ENODEV;
	}

	return srcu_notifier_chain_unregister(&typec->evt_nh, nb);
}

static void hi6526_notify_tcpc_state(const struct tcpc_notify *noti)
{
	struct usb_typec *typec = _typec;

	switch (noti->tcpc_notify_type) {
	case HI6526_TCP_NOTIFY_SINK_VBUS:
		hi6526_tcp_ny_vbus_state(&typec->evt_nh,
					 noti->tcpc_notify_type,
					 noti->notify.vbus_state.mv,
					 noti->notify.vbus_state.ma,
					 noti->notify.vbus_state.type);
		break;

	case HI6526_TCP_NOTIFY_SOURCE_VCONN:
		hi6526_tcp_ny_en_state(&typec->evt_nh,
				       noti->tcpc_notify_type,
				       noti->notify.en_state.en);
		break;

	case HI6526_TCP_NOTIFY_SOURCE_VBUS:
		hi6526_tcp_ny_vbus_state(&typec->evt_nh,
					 noti->tcpc_notify_type,
					 noti->notify.vbus_state.mv,
					 noti->notify.vbus_state.ma,
					 noti->notify.vbus_state.type);
		break;

	case HI6526_TCP_NOTIFY_DR_SWAP:
		hi6526_tcp_ny_swap_state(&typec->evt_nh,
					 noti->tcpc_notify_type,
					 noti->notify.swap_state.new_role);
		break;

	case HI6526_TCP_NOTIFY_TYPEC_STATE:
		hi6526_tcp_ny_typec_state(&typec->evt_nh,
					  noti->tcpc_notify_type,
					  noti->notify.typec_state.new_state,
					  noti->notify.typec_state.old_state,
					  noti->notify.typec_state.polarity,
					  noti->notify.typec_state.rp_level);
		break;

	case HI6526_TCP_NOTIFY_PD_STATE:
		hi6526_tcp_ny_pd_state(&typec->evt_nh,
				       noti->tcpc_notify_type,
				       noti->notify.pd_state.connected);
		break;

	default:
		E("%s: not supported notify type: %lu!\n", __func__,
			noti->tcpc_notify_type);
		return;
	}
}

static int __tcpc_notifier_work(struct tcpc_notify *noti)
{
	struct pd_dpm_typec_state tc_state = {0};
	struct pd_dpm_vbus_state vbus_state = {0};
	struct pd_dpm_swap_state swap_state = {0};
	struct pd_dpm_pd_state pd_state = {0};
	uint32_t cable_vdo = 0;

	I("tcpc_notify_type %lu\n", noti->tcpc_notify_type);

	switch (noti->tcpc_notify_type) {
	case HI6526_TCP_NOTIFY_DIS_VBUS_CTRL:
		hi6526_usb_typec_disable_vbus_control(&noti->notify.vbus_state);

		vbus_state.mv = noti->notify.vbus_state.mv;
		vbus_state.ma = noti->notify.vbus_state.ma;
		vbus_state.vbus_type = noti->notify.vbus_state.type;

		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DIS_VBUS_CTRL,
				(void *)&vbus_state);
		break;

	case HI6526_TCP_NOTIFY_SOURCE_VBUS:
		hi6526_usb_typec_source_vbus(&noti->notify.vbus_state);

		vbus_state.mv = noti->notify.vbus_state.mv;
		vbus_state.ma = noti->notify.vbus_state.ma;
		vbus_state.vbus_type = noti->notify.vbus_state.type;

		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SOURCE_VBUS,
				(void *)&vbus_state);
		break;

	case HI6526_TCP_NOTIFY_SINK_VBUS:
		hi6526_usb_typec_sink_vbus(&noti->notify.vbus_state);

		vbus_state.mv = noti->notify.vbus_state.mv;
		vbus_state.ma = noti->notify.vbus_state.ma;
		vbus_state.vbus_type = noti->notify.vbus_state.type;
		vbus_state.ext_power = noti->notify.vbus_state.ext_power;
		vbus_state.remote_rp_level =
				noti->notify.vbus_state.remote_rp_level;

		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SINK_VBUS,
				(void *)&vbus_state);
		break;

	case HI6526_TCP_NOTIFY_PR_SWAP:
		swap_state.new_role = noti->notify.swap_state.new_role;
		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PR_SWAP, &swap_state);

		break;

	case HI6526_TCP_NOTIFY_DR_SWAP:
		hi6526_usb_typec_data_role_swap(noti->notify.swap_state.new_role);

		swap_state.new_role = noti->notify.swap_state.new_role;
		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DR_SWAP, &swap_state);
		break;

	case HI6526_TCP_NOTIFY_TYPEC_STATE:
		hi6526_usb_typec_state_change(&noti->notify.typec_state);

		tc_state.polarity = noti->notify.typec_state.polarity;
		tc_state.old_state = noti->notify.typec_state.old_state;
		tc_state.new_state = noti->notify.typec_state.new_state;

		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE,
				(void *)&tc_state);
		break;

	case HI6526_TCP_NOTIFY_PD_STATE:
		hi6526_usb_typec_pd_state_change(&noti->notify.pd_state);

		pd_state.connected = noti->notify.pd_state.connected;
		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PD_STATE,
				(void *)&pd_state);
		break;

	case HI6526_TCP_NOTIFY_CABLE_VDO:
		cable_vdo = noti->notify.cable_vdo.vdo;
		pd_dpm_handle_pe_event(PD_DPM_PE_CABLE_VDO, (void *)&cable_vdo);
		break;

	default:
		break;
	}

	hi6526_notify_tcpc_state(noti);

	return 0;
}

/*
 * Register pd dpm ops for hw_pd driver.
 */
int hi6526_usb_typec_register_pd_dpm(void)
{
	int ret;
	struct usb_typec *typec = _typec;

	if (!_typec) {
		E("hi6526 usb typec is not ready\n");
		return -EPERM;
	}

	ret = pd_dpm_ops_register(&hi6526_device_pd_dpm_ops, typec);
	if (ret) {
		I("Need not hi6526 pd\n");
		return -EBUSY;
	}

	ret = pd_protocol_ops_register(&hi6526_device_pd_protocol_ops);
	if (ret) {
		I("pd protocol register failed\n");
		return -EBUSY;
	}

	ret = uvdm_protocol_ops_register(&hi6526_device_uvdm_protocol_ops);
	if (ret)
		E("uvdm protocol register failed\n");

	ret = cc_check_ops_register(&direct_charge_cable_check_ops);
	if (ret) {
		E("cc_check_ops register failed!\n");
		return -EBUSY;
	}

	return ret;
}

#else

static inline int __tcpc_notifier_work(struct tcpc_notify *noti)
{
	E("[Not support Hi6526 typeC] >> tcpc_notify_type %lu\n",
			noti->tcpc_notify_type);
	return 0;
}

int hi6526_usb_typec_register_pd_dpm(void)
{
	E("Not Support Hi6526 PD\n");
	return -EPERM;
}

int hi6526_register_tcpc_notifier(struct notifier_block *nb)
{
	E("Not Support Hi6526 PD\n");
	return -EPERM;
}

int hi6526_unregister_tcpc_notifier(struct notifier_block *nb)
{
	E("Not Support Hi6526 PD\n");
	return -EPERM;
}

#endif /* CONFIG_TCPC_CLASS */

uint8_t hi6526_get_no_rpsrc_state(void)
{
#ifdef CONFIG_TYPEC_CAP_NORP_SRC
	struct usb_typec *typec = _typec;

	if (!typec || !typec->tcpc_dev)
		return 0;
	return typec->tcpc_dev->no_rpsrc_state;
#else
	return 0;
#endif
}

bool usb_typec_charger_type_pd(void)
{
#ifdef CONFIG_TCPC_CLASS
	return pd_dpm_get_pd_finish_flag();
#endif
	return false;
}

#define TYPEC_DSM_BUF_SIZE_256	256
void usb_typec_cc_ovp_dmd_report(void)
{
#ifdef CONFIG_HUAWEI_DSM
	int ret;
	char msg_buf[TYPEC_DSM_BUF_SIZE_256] = { 0 };

	ret = snprintf_s(msg_buf, TYPEC_DSM_BUF_SIZE_256,
			TYPEC_DSM_BUF_SIZE_256 - 1,
			"%s\n",
			"vbus ovp happened");
	if (ret < 0)
		E("Fill cc ovp dmd msg\n");

	power_dsm_report_dmd(POWER_DSM_BATTERY,
			     ERROR_NO_TYPEC_CC_OVP, (void *)msg_buf);
#endif
}

/*
 * Turn on/off vconn power.
 * enable:0 - off, 1 - on
 */
void usb_typec_set_vconn(int enable)
{
	struct usb_typec *typec = _typec;

	I(" Vconn enable: %d\n", enable);
	typec->vconn = enable;
#ifdef CONFIG_TCPC_CLASS
	pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SOURCE_VCONN, &enable);
#endif
}

int usb_typec_otg_pwr_src(void)
{
	/* Only Support Inside OTG */
	return 0;
}

static void tcpc_notifier_work(struct work_struct *work)
{
	struct usb_typec *typec =
			container_of(work, struct usb_typec, tcpc_notify_work);
	struct tcpc_notify *noti = NULL;
	int ret;

	while (1) {
		noti = get_notify(typec);
		if (!noti)
			break;

		ret = __tcpc_notifier_work(noti);
		if (ret)
			D("__tcpc_notifier_work ret %d\n", ret);

		free_notify(noti);
	}
}

static int tcpc_notifier_call(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct usb_typec *typec = container_of(nb, struct usb_typec, tcpc_nb);
	int ret;

	ret = queue_notify(typec, action, data);
	if (ret) {
		E("queue_notify failed!\n");
		return NOTIFY_DONE;
	}

	/* Returns %false if @work was already on a queue, %true otherwise. */
	ret = queue_work(typec->hi6526_typec_wq, &typec->tcpc_notify_work);
	D("queue_work ret %d\n", ret);

	return NOTIFY_OK;
}

/*
 * Save tcpc_device pointer for futher use.
 * tcpc_dev:Pointer of tcpc_device structure.
 */
void hi6526_usb_typec_register_tcpc_device(struct hi6526_tcpc_device *tcpc_dev)
{
	struct usb_typec *typec = _typec;
	int ret;

	if (!typec) {
		E("hi6526 usb typec is not ready\n");
		return;
	}

	/* save the tcpc handler */
	typec->tcpc_dev = tcpc_dev;

	INIT_LIST_HEAD(&typec->tcpc_notify_list);
	spin_lock_init(&typec->tcpc_notify_list_lock);
	typec->tcpc_notify_count = 0;
	INIT_WORK(&typec->tcpc_notify_work, tcpc_notifier_work);

	typec->tcpc_nb.notifier_call = tcpc_notifier_call;
	ret = hi6526_tcpm_register_tcpc_dev_notifier(tcpc_dev, &typec->tcpc_nb);
	if (ret)
		E("register tcpc notifier failed ret %d\n", ret);
}

static int typec_probe(struct platform_device *pdev)
{
	struct usb_typec *typec = NULL;
	int ret;

	typec = devm_kzalloc(&pdev->dev, sizeof(*typec), GFP_KERNEL);
	if (!typec)
		return -ENOMEM;

	typec->power_role = PD_ROLE_UNATTACHED;
	typec->data_role = PD_ROLE_UNATTACHED;
	typec->vconn = PD_ROLE_VCONN_OFF;
	typec->audio_accessory = 0;

	mutex_init(&typec->lock);

	srcu_init_notifier_head(&typec->evt_nh);

	typec->hi6526_typec_wq = create_singlethread_workqueue("hi6526_usb_typec");

	_typec = typec;

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret) {
		E("populate child failed ret %d\n", ret);
		_typec = NULL;
		return ret;
	}

	return 0;
}

static int typec_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	_typec = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int typec_suspend(struct device *dev)
{
	struct usb_typec *typec = _typec;

	typec->suspend_count++;

	return 0;
}

static int typec_resume(struct device *dev)
{
	struct usb_typec *typec = _typec;

	typec->resume_count++;

	return 0;
}

static const struct dev_pm_ops typec_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(typec_suspend, typec_resume)
};
#define TYPEC_PM_OPS	(&typec_pm_ops)
#else
#define TYPEC_PM_OPS	(NULL)
#endif /* CONFIG_PM */

static const struct of_device_id typec_match_table[] = {
	{ .compatible = "huawei,hi6526-usb-typec", },
	{},
};

static struct platform_driver typec_driver = {
	.driver = {
		.name           = "hi6526-usb-typec",
		.owner          = THIS_MODULE,
		.of_match_table = typec_match_table,
		.pm = TYPEC_PM_OPS,
	},
	.probe  = typec_probe,
	.remove = typec_remove,
};

static int __init hi6526_typec_init(void)
{
	return platform_driver_register(&typec_driver);
}

static void __exit hi6526_typec_exit(void)
{
	platform_driver_unregister(&typec_driver);
}

arch_initcall_sync(hi6526_typec_init);
module_exit(hi6526_typec_exit);

MODULE_DESCRIPTION("Hi6526 USB Type-C Driver");
MODULE_LICENSE("GPL");

