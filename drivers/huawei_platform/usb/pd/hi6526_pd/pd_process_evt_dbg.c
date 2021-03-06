/*
 * pd_process_evt_dbg.c
 *
 * Power Delivery Process Event For DBGACC
 *
 * Copyright (c) 2019 Huawei Technologies Co., Ltd.
 *
 * Copyright (C) 2016 Richtek Technology Corp.
 * Author: TH <tsunghan_tsai@richtek.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "[EVT]"

#include "include/pd_core.h"
#include "include/pd_tcpm.h"
#include "include/tcpci_event.h"
#include "include/pd_process_evt.h"

#ifdef CONFIG_USB_PD_CUSTOM_DBGACC_SUPPORT

DECL_PE_STATE_TRANSITION(PD_PE_MSG_IDLE) = {
	{ PE_IDLE1, PE_IDLE2 },
};
DECL_PE_STATE_REACTION(PD_PE_MSG_IDLE);

bool hi6526_pd_process_event_dbg(pd_port_t *pd_port, pd_event_t *pd_event)
{
	if (pd_event->event_type == PD_EVT_HW_MSG) {
		switch (pd_event->msg) {
		case PD_HW_CC_DETACHED:
			PE_TRANSIT_STATE(pd_port, PE_IDLE1);
			return true;

		case PD_HW_CC_ATTACHED:
			PE_TRANSIT_STATE(pd_port, PE_DBG_READY);
			return true;

		default:
			break;
		}
	}

	if (pd_event_msg_match(pd_event, PD_EVT_PE_MSG, PD_PE_IDLE))
		return PE_MAKE_STATE_TRANSIT(PD_PE_MSG_IDLE);

	return false;
}

#endif /* CONFIG_USB_PD_CUSTOM_DBGACC_SUPPORT */
