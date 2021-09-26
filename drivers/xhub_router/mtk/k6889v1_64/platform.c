/*
 * platform.c
 *
 * functions for mtk adapter
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
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
#include <linux/types.h>
#include <linux/delay.h>
#include "xhub_route.h"
#include "scp_excep.h"
#include "scp_helper.h"
#include "scp_ipi_pin.h"
#include "xhub_boot.h"
#include "xhub_recovery.h"

void scp_wdt_reset(enum scp_core_id cpu_id);

void reset_trigger(void)
{
	scp_wdt_reset(0);
}

void ramdump_trigger(void)
{
	scp_aed(RESET_TYPE_CMD, SCP_A_ID);
}

static int scp_ready_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	static int once_flag = 0;
	static int scp_reset_counts_temp = 0;

	if (!once_flag) {
		scp_reset_counts_temp = scp_reset_counts;
		once_flag = 1;
	}

	hwlog_warn("%s event %d reset counts %d reset count temp %d reset status %d rec state %d\n",
			__func__, event, scp_reset_counts, scp_reset_counts_temp,
			atomic_read(&scp_reset_status), atomic_read(&xhub_rec_state));

	if (event == SCP_EVENT_STOP) {
		if (scp_reset_counts_temp > scp_reset_counts &&
			atomic_read(&scp_reset_status) == RESET_STATUS_START) {
			if (atomic_read(&xhub_rec_state) == XHUB_RECOVERY_IDLE) {
				__pm_stay_awake(&xhub_rec_wl);
				atomic_set(&xhub_rec_state, SCP_RECOVERY_START);
				xhub_extern_notify(SCP_RECOVERY_START);
				xhub_before_scp_reset(SH_FAULT_INTERNELFAULT);
			}
		}
	}

	if (event == SCP_EVENT_READY) {
		if (scp_reset_counts_temp > scp_reset_counts) {
			scp_reset_counts_temp = scp_reset_counts;
			if (atomic_read(&xhub_rec_state) > SCP_RECOVERY_START) {
				reinit_completion(&recovery_completion);
				msleep(5);
				wait_for_completion(&recovery_completion);
			}
			atomic_set(&xhub_rec_state, SCP_RECOVERY_READY);
			queue_work(xhub_rec_wq, &xhub_rec_work);
		} else {
			send_status_req_to_mcu();
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block scp_ready_notifier = {
	.notifier_call = scp_ready_event,
};

void register_platform_notify(void)
{
	scp_A_register_notify(&scp_ready_notifier);
}
