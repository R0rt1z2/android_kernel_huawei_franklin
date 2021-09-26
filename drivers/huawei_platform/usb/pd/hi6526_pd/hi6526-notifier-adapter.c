/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/notifier.h>
#include <tcpm.h>

#define UNDEFINED_STATE 0xff

#ifdef CONFIG_HI6526_USB_TYPEC_DBG
#define D(format, arg...) pr_info("[tcpc-noti-adpt]" format, ##arg)
#else
#define D(format, arg...) do {} while (0)
#endif
#define I(format, arg...) pr_info("[tcpc-noti-adpt]" format, ##arg)
#define E(format, arg...) pr_err("[ERR][tcpc-noti-adpt]" format, ##arg)

static const uint8_t g_typec_state_map[] = {
	TYPEC_UNATTACHED,           /* HI6526_TYPEC_UNATTACHED */
	TYPEC_ATTACHED_SNK,         /* HI6526_TYPEC_ATTACHED_SNK */
	TYPEC_ATTACHED_SRC,         /* HI6526_TYPEC_ATTACHED_SRC */
	TYPEC_ATTACHED_AUDIO,       /* HI6526_TYPEC_ATTACHED_AUDIO */
	TYPEC_ATTACHED_DEBUG,       /* HI6526_TYPEC_ATTACHED_DEBUG */
	TYPEC_ATTACHED_DBGACC_SNK,  /* HI6526_TYPEC_ATTACHED_DBGACC_SNK */
	TYPEC_ATTACHED_CUSTOM_SRC,  /* HI6526_TYPEC_ATTACHED_CUSTOM_SRC */
	TYPEC_ATTACHED_NORP_SRC,    /* HI6526_TYPEC_ATTACHED_VBUS_ONLY */
	TYPEC_UNATTACHED,           /* HI6526_TYPEC_DETTACHED_VBUS_ONLY */
};
static const int g_typec_state_map_size =
	sizeof(g_typec_state_map) / sizeof(g_typec_state_map[0]);

static uint8_t hi6526_typec_state_map(uint8_t state)
{
	if (state >= g_typec_state_map_size)
		return UNDEFINED_STATE;
	return g_typec_state_map[state];
}

static const uint8_t g_pd_connect_state_map[] = {
	PD_CONNECT_NONE,                /* HI6526_PD_CONNECT_NONE */
	UNDEFINED_STATE,                /* HI6526_PD_CONNECT_TYPEC_ONLY */
	PD_CONNECT_TYPEC_ONLY_SNK_DFT,  /* HI6526_PD_CONNECT_TYPEC_ONLY_SNK_DFT */
	PD_CONNECT_TYPEC_ONLY_SNK,      /* HI6526_PD_CONNECT_TYPEC_ONLY_SNK */
	PD_CONNECT_TYPEC_ONLY_SRC,      /* HI6526_PD_CONNECT_TYPEC_ONLY_SRC */
	UNDEFINED_STATE,                /* HI6526_PD_CONNECT_PE_READY */
	PD_CONNECT_PE_READY_SNK,        /* HI6526_PD_CONNECT_PE_READY_SNK */
	PD_CONNECT_PE_READY_SRC,        /* HI6526_PD_CONNECT_PE_READY_SRC */
	PD_CONNECT_PE_READY_DBGACC_UFP, /* HI6526_PD_CONNECT_PE_READY_DBGACC_UFP */
	PD_CONNECT_PE_READY_DBGACC_DFP, /* HI6526_PD_CONNECT_PE_READY_DBGACC_DFP */
};
static const int g_pd_connect_state_map_size =
	sizeof(g_pd_connect_state_map) / sizeof(g_pd_connect_state_map[0]);

static uint8_t hi6526_pd_connect_state_map(uint8_t state)
{
	if (state >= g_pd_connect_state_map_size)
		return UNDEFINED_STATE;
	return g_pd_connect_state_map[state];
}

static const uint8_t g_tcp_notifier_map[] = {
	TCP_NOTIFY_DIS_VBUS_CTRL,    /* HI6526_TCP_NOTIFY_DIS_VBUS_CTRL */
	TCP_NOTIFY_SOURCE_VCONN,     /* HI6526_TCP_NOTIFY_SOURCE_VCONN */
	TCP_NOTIFY_SOURCE_VBUS,      /* HI6526_TCP_NOTIFY_SOURCE_VBUS */
	TCP_NOTIFY_SINK_VBUS,        /* HI6526_TCP_NOTIFY_SINK_VBUS */
	TCP_NOTIFY_PR_SWAP,          /* HI6526_TCP_NOTIFY_PR_SWAP */
	TCP_NOTIFY_DR_SWAP,          /* HI6526_TCP_NOTIFY_DR_SWAP */
	TCP_NOTIFY_VCONN_SWAP,       /* HI6526_TCP_NOTIFY_VCONN_SWAP */
	TCP_NOTIFY_ENTER_MODE,       /* HI6526_TCP_NOTIFY_ENTER_MODE */
	TCP_NOTIFY_EXIT_MODE,        /* HI6526_TCP_NOTIFY_EXIT_MODE */
	TCP_NOTIFY_AMA_DP_STATE,     /* HI6526_TCP_NOTIFY_AMA_DP_STATE */
	TCP_NOTIFY_AMA_DP_ATTENTION, /* HI6526_TCP_NOTIFY_AMA_DP_ATTENTION */
	TCP_NOTIFY_AMA_DP_HPD_STATE, /* HI6526_TCP_NOTIFY_AMA_DP_HPD_STATE */
	TCP_NOTIFY_TYPEC_STATE,      /* HI6526_TCP_NOTIFY_TYPEC_STATE */
	TCP_NOTIFY_PD_STATE,         /* HI6526_TCP_NOTIFY_PD_STATE */
	TCP_NOTIFY_UVDM,             /* HI6526_TCP_NOTIFY_UVDM */
	UNDEFINED_STATE,             /* HI6526_TCP_NOTIFY_CABLE_VDO */
	TCP_NOTIFY_ALERT,            /* HI6526_TCP_NOTIFY_ALERT */
	TCP_NOTIFY_STATUS,           /* HI6526_TCP_NOTIFY_STATUS */
	UNDEFINED_STATE,             /* HI6526_TCP_NOTIFY_PPS_STATUS */
	UNDEFINED_STATE,             /* HI6526_TCP_NOTIFY_PPS_READY */
};
static const unsigned long g_tcp_notifier_map_size =
	sizeof(g_tcp_notifier_map) / sizeof(g_tcp_notifier_map[0]);

static unsigned long hi6526_tcp_notifier_map(unsigned long state)
{
	if (state >= g_tcp_notifier_map_size)
		return UNDEFINED_STATE;
	return g_tcp_notifier_map[state];
}

void hi6526_tcp_ny_vbus_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, int mv, int ma, uint8_t type)
{
	unsigned long notify_type = hi6526_tcp_notifier_map(tcpc_notify_type);
	struct tcp_notify notify;
	int ret;

	if (notify_type == UNDEFINED_STATE) {
		E("%s: notify_type %lu not supported!\n", __func__,
			tcpc_notify_type);
		return;
	}

	notify.vbus_state.mv = mv;
	notify.vbus_state.ma = ma;
	notify.vbus_state.type = type;
	D("%s: vbus_state mv:%d ma:%d type:%u\n", __func__, mv, ma, type);

	ret = srcu_notifier_call_chain(evt_nh, notify_type, &notify);
	if (ret < 0)
		E("%s: srcu_notifier_call failed! ret=%d\n", __func__, ret);
}

void hi6526_tcp_ny_en_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, bool en_state)
{
	unsigned long notify_type = hi6526_tcp_notifier_map(tcpc_notify_type);
	struct tcp_notify notify;
	int ret;

	if (notify_type == UNDEFINED_STATE) {
		E("%s: notify_type %lu not supported!\n", __func__,
			tcpc_notify_type);
		return;
	}

	notify.en_state.en = en_state;
	D("%s: en_state:%d\n", __func__, en_state);

	ret = srcu_notifier_call_chain(evt_nh, notify_type, &notify);
	if (ret < 0)
		E("%s: srcu_notifier_call failed! ret=%d\n", __func__, ret);
}

void hi6526_tcp_ny_swap_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t new_role)
{
	unsigned long notify_type = hi6526_tcp_notifier_map(tcpc_notify_type);
	struct tcp_notify notify;
	int ret;

	if (notify_type == UNDEFINED_STATE) {
		E("%s: notify_type %lu not supported!\n", __func__,
			tcpc_notify_type);
		return;
	}

	notify.swap_state.new_role = new_role;
	D("%s: new_role:%u\n", __func__, new_role);

	ret = srcu_notifier_call_chain(evt_nh, notify_type, &notify);
	if (ret < 0)
		E("%s: srcu_notifier_call failed! ret=%d\n", __func__, ret);
}

void hi6526_tcp_ny_typec_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t new_state, uint8_t old_state,
	uint8_t polarity, uint8_t rp_level)
{
	unsigned long notify_type = hi6526_tcp_notifier_map(tcpc_notify_type);
	struct tcp_notify notify;
	int ret;

	if (notify_type == UNDEFINED_STATE) {
		E("%s: notify_type %lu not supported!\n", __func__,
			tcpc_notify_type);
		return;
	}

	notify.typec_state.new_state = hi6526_typec_state_map(new_state);
	notify.typec_state.old_state = hi6526_typec_state_map(old_state);
#ifdef CONFIG_TYPEC_CAP_NORP_SRC
	/* if typec state is HI6526_TYPEC_DETTACHED_VBUS_ONLY */
	if (new_state == g_typec_state_map_size - 1)
		notify.typec_state.old_state = TYPEC_ATTACHED_NORP_SRC;
#endif /* CONFIG_TYPEC_CAP_NORP_SRC */
	notify.typec_state.polarity = polarity;
	notify.typec_state.rp_level = rp_level;

	D("%s: typec_state new_state:%u old_state:%u polarity:%u rp_level:%u\n",
		__func__, new_state, old_state, polarity, rp_level);

	if (notify.typec_state.new_state == UNDEFINED_STATE ||
	    notify.typec_state.old_state == UNDEFINED_STATE)
		return;

	ret = srcu_notifier_call_chain(evt_nh, notify_type, &notify);
	if (ret < 0)
		E("%s: srcu_notifier_call failed! ret=%d\n", __func__, ret);
}

void hi6526_tcp_ny_pd_state(struct srcu_notifier_head *evt_nh,
	unsigned long tcpc_notify_type, uint8_t pd_connect_state)
{
	unsigned long notify_type = hi6526_tcp_notifier_map(tcpc_notify_type);
	struct tcp_notify notify;
	int ret;

	if (notify_type == UNDEFINED_STATE) {
		E("%s: notify_type %lu not supported!\n", __func__,
			tcpc_notify_type);
		return;
	}

	notify.pd_state.connected =
		hi6526_pd_connect_state_map(pd_connect_state);
	D("%s: pd_state %u\n", __func__, pd_connect_state);
	if (notify.pd_state.connected == UNDEFINED_STATE)
		return;

	ret = srcu_notifier_call_chain(evt_nh, notify_type, &notify);
	if (ret < 0)
		E("%s: srcu_notifier_call failed! ret=%d\n", __func__, ret);
}

