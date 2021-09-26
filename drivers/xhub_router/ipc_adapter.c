/*
 *  drivers/misc/inputhub/xhub_route.c
 *  Sensor Hub Channel driver
 *
 *  Copyright (C) 2012 Huawei, Inc.
 *  Author: qindiwen <inputhub@huawei.com>
 *
 */
#include <linux/delay.h>
#include "scp_mbox_layout.h"
#include "ipc_adapter.h"

#include <log/hw_log.h>

#define HWLOG_TAG sensorhub
HWLOG_REGIST();

/*
 * mbox slot size definition
 * 1 slot for 4 bytes
 */
#define MBOX_SLOT_SIZE 4
#define XHUB_IPI_SEND_CN_TIMEOUT 500 // ms
#define XHUB_WAITCHECK_INTERVAL_MS 1

int ipc_adapter_send(const char *buf, unsigned int length)
{
#ifdef HW_CUST_IPC
	enum scp_ipi_status status;
	unsigned int retry_cnt;
	unsigned int retry_times = XHUB_IPI_SEND_CN_TIMEOUT;

	if (length > (PIN_OUT_SIZE_SCP_MPOOL - 2) * MBOX_SLOT_SIZE) {
		hwlog_warn("ipc msg len %d is too long, should use share memory\n", length);
		return 0;
	}
	/* for mtk */
	for (retry_cnt = 0; retry_cnt < retry_times; retry_cnt++) {
		status = scp_ipi_send(IPI_HW_CUST, (void *)buf, length, 0, SCP_A_ID);
		if (status == SCP_IPI_DONE)
			break;
		hwlog_warn("scp_ipi_send fail, status %d, retry_cnt %d, try again\n", status, retry_cnt);
		msleep(XHUB_WAITCHECK_INTERVAL_MS);
	}
	if (retry_cnt == retry_times)
		hwlog_err("scp_ipi_send fail, status %d, retry_cnt %d\n", status, retry_cnt);
	else
		hwlog_info("%s success\n", __func__);
#endif

	return 0;
}
