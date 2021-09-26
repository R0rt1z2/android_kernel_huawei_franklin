/*
 * drivers/inputhub/contexthub_recovery.h
 *
 * sensors sysfs header
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
#ifndef __LINUX_RECOVERY_H__
#define __LINUX_RECOVERY_H__
#include "xhub_boot.h"
#include "protocol.h"
#include "sensor_detect.h"

#define XHUB_RECOVERY_UNINIT		0
#define XHUB_RECOVERY_IDLE		(XHUB_RECOVERY_UNINIT + 1)
#define SCP_RECOVERY_START		(XHUB_RECOVERY_IDLE + 1)
#define SCP_RECOVERY_READY   (SCP_RECOVERY_START + 1)
#define XHUB_RECOVERY_MINISYS  (SCP_RECOVERY_READY + 1)
#define XHUB_RECOVERY_DOING			(XHUB_RECOVERY_MINISYS + 1)
#define XHUB_RECOVERY_3RD_DOING		(XHUB_RECOVERY_DOING + 1)
#define XHUB_RECOVERY_FAILED		(XHUB_RECOVERY_3RD_DOING + 1)

#define DIR_LIMIT		0770
#define FILE_LIMIT		0660
#define SH_DMP_DIR  "/data/log/sensorhub-log/"
#define SH_DMP_FS  "/data/lost+found"
#define SH_DMP_HISTORY_FILE "history.log"
#define DATATIME_MAXLEN     24 /* 14+8 +2, 2: '-'+'\0' */
#define DATA_MAXLEN 14

typedef enum {
	SH_FAULT_HARDFAULT = 0,
	SH_FAULT_BUSFAULT,
	SH_FAULT_USAGEFAULT,
	SH_FAULT_MEMFAULT,
	SH_FAULT_NMIFAULT,
	SH_FAULT_ASSERT,
	SH_FAULT_INTERNELFAULT = 16,
	SH_FAULT_IPC_RX_TIMEOUT,
	SH_FAULT_IPC_TX_TIMEOUT,
	SH_FAULT_RESET,
	SH_FAULT_USER_DUMP,
	SH_FAULT_RESUME,
	SH_FAULT_REDETECT,
	SH_FAULT_PANIC,
	SH_FAULT_NOC,
	SH_FAULT_REACT,
	SH_FAULT_EXP_BOTTOM,
} exp_source_t;

extern atomic_t xhub_rec_state;

#define SENSORHUB_TRACK_SIZE 32
#define PATH_MAXLEN         128
#define CUR_PATH_LEN 64
#define MAX_DUMP_CNT     32
#define HISTORY_LOG_SIZE 256
#define HISTORY_LOG_MAX  0x80000 /* 512k */

extern struct config_on_ddr *g_config_on_ddr;
extern struct type_record type_record;
extern struct workqueue_struct *xhub_rec_wq;
extern struct work_struct xhub_rec_work;
extern struct completion recovery_completion;
extern struct wakeup_source xhub_rec_wl;

extern void disable_motions_when_sysreboot(void);
extern int xhub_need_recovery(exp_source_t f);
extern int xhub_recovery_init(void);
extern int register_xhub_recovery_notifier(struct notifier_block *nb);
extern int xhub_rec_sys_callback(const pkt_header_t *head);
extern void xhub_extern_notify(int notify_status);
extern void xhub_before_scp_reset(exp_source_t f);
#endif /* __LINUX_RECOVERY_H__ */
