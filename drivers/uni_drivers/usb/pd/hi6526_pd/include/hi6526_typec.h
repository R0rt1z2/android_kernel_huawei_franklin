/*
 * hi6526_typec.h
 *
 * Hi6526 tcpc interface defination
 *
 * Copyright (c) 2017-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef HI6526_TYPEC_H_
#define HI6526_TYPEC_H_

#include <linux/kernel.h>
#include <linux/notifier.h>

enum typec_attach_type {
	HI6526_TYPEC_UNATTACHED = 0,
	HI6526_TYPEC_ATTACHED_SNK,
	HI6526_TYPEC_ATTACHED_SRC,
	HI6526_TYPEC_ATTACHED_AUDIO,
	HI6526_TYPEC_ATTACHED_DEBUG,

	/* CONFIG_TYPEC_CAP_DBGACC_SNK_SUPPORT */
	HI6526_TYPEC_ATTACHED_DBGACC_SNK, /* Rp, Rp */

	/* CONFIG_TYPEC_CAP_CUSTOM_SRC_SUPPORT */
	HI6526_TYPEC_ATTACHED_CUSTOM_SRC, /* Same Rp */

	HI6526_TYPEC_ATTACHED_VBUS_ONLY,
	HI6526_TYPEC_DETTACHED_VBUS_ONLY,
};

enum {
	TCP_VBUS_CTRL_REMOVE		= 0,
	TCP_VBUS_CTRL_TYPEC		= 1,
	TCP_VBUS_CTRL_PD		= 2,

	TCP_VBUS_CTRL_HRESET		= TCP_VBUS_CTRL_PD,
	TCP_VBUS_CTRL_PR_SWAP		= 3,
	TCP_VBUS_CTRL_REQUEST		= 4,
	TCP_VBUS_CTRL_STANDBY		= 5,
	TCP_VBUS_CTRL_STANDBY_UP	= 6,
	TCP_VBUS_CTRL_STANDBY_DOWN	= 7,

	TCP_VBUS_CTRL_PD_DETECT		= (1 << 7),
	TCP_VBUS_CTRL_PD_HRESET		= TCP_VBUS_CTRL_HRESET | TCP_VBUS_CTRL_PD_DETECT,
	TCP_VBUS_CTRL_PD_PR_SWAP	= TCP_VBUS_CTRL_PR_SWAP | TCP_VBUS_CTRL_PD_DETECT,
	TCP_VBUS_CTRL_PD_REQUEST	= TCP_VBUS_CTRL_REQUEST | TCP_VBUS_CTRL_PD_DETECT,
	TCP_VBUS_CTRL_PD_STANDBY	= TCP_VBUS_CTRL_STANDBY | TCP_VBUS_CTRL_PD_DETECT,
	TCP_VBUS_CTRL_PD_STANDBY_UP	= TCP_VBUS_CTRL_STANDBY_UP | TCP_VBUS_CTRL_PD_DETECT,
	TCP_VBUS_CTRL_PD_STANDBY_DOWN	= TCP_VBUS_CTRL_STANDBY_DOWN | TCP_VBUS_CTRL_PD_DETECT,
};

struct tcp_ny_vbus_state {
	int mv;
	int ma;
	uint8_t type;
	uint8_t ext_power;
	uint8_t remote_rp_level;
};

#define TYPEC_CC_VOLT_ACT_AS_SINK (1 << 2)

enum tcpm_cc_voltage_status {
	TYPEC_CC_VOLT_OPEN = 0,
	TYPEC_CC_VOLT_RA = 1,
	TYPEC_CC_VOLT_RD = 2,

	TYPEC_CC_VOLT_SNK_DFT = 5,
	TYPEC_CC_VOLT_SNK_1_5 = 6,
	TYPEC_CC_VOLT_SNK_3_0 = 7,

	TYPEC_CC_DRP_TOGGLING = 15,
};

static inline char *tcpm_cc_voltage_status_string(uint8_t cc)
{
	if (cc == TYPEC_CC_VOLT_OPEN)
		return "OPEN";
	else if (cc == TYPEC_CC_VOLT_RA)
		return "RA";
	else if (cc == TYPEC_CC_VOLT_RD)
		return "RD";
	else if (cc == TYPEC_CC_VOLT_SNK_DFT)
		return "Default";
	else if (cc == TYPEC_CC_VOLT_SNK_1_5)
		return "1.5";
	else if (cc == TYPEC_CC_VOLT_SNK_3_0)
		return "3.0";
	else if (cc == TYPEC_CC_DRP_TOGGLING)
		return "DRP";
	else
		return "unknown";
}

enum tcpm_vbus_level {
#ifdef CONFIG_TCPC_VSAFE0V_DETECT_SUPPORT
	TCPC_VBUS_SAFE0V = 0,
	TCPC_VBUS_INVALID,
	TCPC_VBUS_VALID,
#else
	TCPC_VBUS_INVALID = 0,
	TCPC_VBUS_VALID,
#endif
};

static inline char *typec_attach_type_name(uint8_t type)
{
	switch (type) {
	case HI6526_TYPEC_UNATTACHED: return "UNATTACHED";
	case HI6526_TYPEC_ATTACHED_SNK: return "SINK";
	case HI6526_TYPEC_ATTACHED_SRC: return "SOURCE";
	case HI6526_TYPEC_ATTACHED_AUDIO: return "AUDIO";
	case HI6526_TYPEC_ATTACHED_DEBUG: return "DBGACC";
#ifdef CONFIG_TYPEC_CAP_DBGACC_SNK_SUPPORT
	case HI6526_TYPEC_ATTACHED_DBGACC_SNK: return "DBGACC_SNK";
#endif
#ifdef CONFIG_TYPEC_CAP_CUSTOM_SRC_SUPPORT
	case HI6526_TYPEC_ATTACHED_CUSTOM_SRC: return "CUSTOM_SRC";
#endif
	case HI6526_TYPEC_ATTACHED_VBUS_ONLY: return "ATTACHED_VBUS_ONLY";
	case HI6526_TYPEC_DETTACHED_VBUS_ONLY: return "DETTACHED_VBUS_ONLY";
	default: return "uknown";
	}
}

enum typec_role_defination {
	TYPEC_ROLE_UNKNOWN = 0,
	TYPEC_ROLE_SNK,
	TYPEC_ROLE_SRC,
	TYPEC_ROLE_DRP,
	TYPEC_ROLE_TRY_SRC,
	TYPEC_ROLE_TRY_SNK,
	TYPEC_ROLE_NR,
};

static const char * const typec_role_name[] = {
	"UNKNOWN",
	"SNK",
	"SRC",
	"DRP",
	"TrySRC",
	"TrySNK",
};

enum pd_cable_current_limit {
	PD_CABLE_CURR_UNKNOWN = 0,
	PD_CABLE_CURR_1A5 = 1,
	PD_CABLE_CURR_3A = 2,
	PD_CABLE_CURR_5A = 3,
};

#endif /* HI6526_TYPEC_H_ */
