/*
 * contexthub_recovery.c
 *
 * functions for sensorhub recovery
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
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/pm_wakeup.h>
#include <asm/cacheflush.h>
#include <securec.h>
#include "protocol.h"
#include "xhub_route.h"
#include "xhub_boot.h"
#include "xhub_recovery.h"
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "scp_excep.h"
#include "scp_helper.h"
#include "platform.h"

#define DUMP_FAIL (-1)

BLOCKING_NOTIFIER_HEAD(xhub_recovery_notifier_list);

wait_queue_head_t xhub_rec_waitq;

atomic_t xhub_rec_state = ATOMIC_INIT(XHUB_RECOVERY_UNINIT);

struct workqueue_struct *xhub_rec_wq;
struct work_struct xhub_rec_work;
struct completion recovery_completion;
struct wakeup_source xhub_rec_wl;

static struct mutex mutex_recovery_cmd;
static struct completion xhub_rec_done;
static char g_dump_dir[PATH_MAXLEN] = SH_DMP_DIR;
static char g_dump_fs[PATH_MAXLEN] = SH_DMP_FS;
static uint32_t g_dump_index = -1;

static const char * const sh_reset_reasons[] = {
	"SH_FAULT_HARDFAULT", // 0
	"SH_FAULT_BUSFAULT",
	"SH_FAULT_USAGEFAULT",
	"SH_FAULT_MEMFAULT",
	"SH_FAULT_NMIFAULT",
	"SH_FAULT_ASSERT", // 5
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT", // 10
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT",
	"UNKNOW DUMP FAULT", // 15
	"SH_FAULT_INTERNELFAULT", // 16
	"SH_FAULT_IPC_RX_TIMEOUT",
	"SH_FAULT_IPC_TX_TIMEOUT",
	"SH_FAULT_RESET",
	"SH_FAULT_USER_DUMP",
	"SH_FAULT_RESUME",
	"SH_FAULT_REDETECT",
	"SH_FAULT_PANIC",
	"SH_FAULT_NOC",
	"SH_FAULT_EXP_BOTTOM", // also use as unknow dump
};

void xhub_extern_notify(int notify_status)
{
	blocking_notifier_call_chain(&xhub_recovery_notifier_list,
			notify_status, NULL);
}

int register_xhub_recovery_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&xhub_recovery_notifier_list, nb);
}

int xhub_rec_sys_callback(const pkt_header_t *head)
{
	int ret;

	hwlog_err("%s in xhub_rec_state %d\n", __func__, atomic_read(&xhub_rec_state));
	if (atomic_read(&xhub_rec_state) == XHUB_RECOVERY_MINISYS) {
		if (((pkt_sys_statuschange_req_t *) head)->status == ST_MINSYSREADY) {
			hwlog_info("REC sys ready mini!\n");
			ret = send_fileid_to_mcu();
			if (ret)
				hwlog_err("REC get sensors cfg data from dts fail,ret=%d, use default config data!\n",
							ret);
			else
				hwlog_info("REC get sensors cfg data from dts success!\n");
		} else if (ST_MCUREADY == ((pkt_sys_statuschange_req_t *)head)->status) {
			hwlog_info("REC mcu all ready!\n");
			ret = motion_set_cfg_data();
			if (ret < 0)
				hwlog_err("motion set cfg data err ret=%d\n", ret);
			ret = sensor_set_cfg_data();
			if (ret < 0)
				hwlog_err("REC sensor_chip_detect ret=%d\n", ret);
			else
				complete(&xhub_rec_done);
		}
	}

	return 0;
}

static void disable_sensors_when_sysreboot(void)
{
	int tag;

	for (tag = TAG_SENSOR_BEGIN; tag < TAG_SENSOR_END; ++tag) {
		if (sensor_status.status[tag]) {
			if (tag == TAG_STEP_COUNTER)
				xhub_sensor_enable_stepcounter(false, TYPE_STANDARD);
			else
				xhub_sensor_enable(tag, false);
			msleep(50);
			hwlog_info("disable sensor - %d before reboot\n", tag);
		}
	}
}

static int xhub_panic_notify(struct notifier_block *nb,
		unsigned long action, void *data)
{
	hwlog_warn("%s start\n", __func__);
	__pm_stay_awake(&xhub_rec_wl);
	ramdump_trigger();
	__pm_relax(&xhub_rec_wl);
	hwlog_warn("%s done\n", __func__);
	return NOTIFY_OK;
}

static struct notifier_block xhub_panic_block = {
	.notifier_call = xhub_panic_notify,
};

static void xhub_recovery_work(struct work_struct *work)
{
	hwlog_err("%s enter\n", __func__);

	atomic_set(&xhub_rec_state, XHUB_RECOVERY_MINISYS);

	send_status_req_to_mcu();

	/* startup xhub system */
	reinit_completion(&xhub_rec_done);

	/* dynamic loading */
	if (!wait_for_completion_timeout(&xhub_rec_done, 5 * HZ)) {
		hwlog_err("wait for xhub system ready timeout\n");
		atomic_set(&xhub_rec_state, XHUB_RECOVERY_FAILED);
		blocking_notifier_call_chain(&xhub_recovery_notifier_list,
								XHUB_RECOVERY_FAILED, NULL);
		atomic_set(&xhub_rec_state, XHUB_RECOVERY_IDLE);
		__pm_relax(&xhub_rec_wl);
		complete(&recovery_completion);
		hwlog_err("%s exit\n", __func__);
		return;
	}

	atomic_set(&xhub_rec_state, XHUB_RECOVERY_DOING);
	hwlog_err("%s doing\n", __func__);
	blocking_notifier_call_chain(&xhub_recovery_notifier_list,
							XHUB_RECOVERY_DOING, NULL);
	/* recovery 3rd app */
	blocking_notifier_call_chain(&xhub_recovery_notifier_list,
							XHUB_RECOVERY_3RD_DOING, NULL);
	hwlog_err("%s 3rd app recovery\n", __func__);
	atomic_set(&xhub_rec_state, XHUB_RECOVERY_IDLE);

	__pm_relax(&xhub_rec_wl);
	hwlog_err("%s finish recovery\n", __func__);
	blocking_notifier_call_chain(&xhub_recovery_notifier_list,
							XHUB_RECOVERY_IDLE, NULL);
	complete(&recovery_completion);
	hwlog_err("%s exit\n", __func__);
}

static int __sh_create_dir(char *path, unsigned int len)
{
	int fd;
	mm_segment_t old_fs;

	if (len > CUR_PATH_LEN) {
		hwlog_err("invalid  parameter. len is %d\n", len);
		return -1;
	}
	if (!path) {
		hwlog_err("invalid  parameter. path:%pK\n", path);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_access(path, 0);
	if (fd != 0) {
		hwlog_info("sh: need create dir %s\n", path);
		fd = sys_mkdir(path, DIR_LIMIT);
		if (fd < 0) {
			hwlog_err("sh: create dir %s failed! ret = %d\n", path, fd);
			set_fs(old_fs);
			return fd;
		}

		hwlog_info("sh: create dir %s successed [%d]!!!\n", path, fd);
	}
	set_fs(old_fs);
	return 0;
}

static void sh_wait_fs(const char *path)
{
	int fd;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	do {
		fd = sys_access(path, 0);
		if (fd) {
			msleep(10);
			hwlog_info("%s wait ...\n", __func__);
		}
	} while (fd);
	set_fs(old_fs);
}

static int sh_savebuf2fs(char *logpath, char *filename, void *buf, u32 len,
		u32 is_append)
{
	int ret;
	int flags;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	char path[PATH_MAXLEN];

	if (!logpath || !filename || !buf || len <= 0) {
		hwlog_err("invalid  parameter. path:%pK, name:%pK buf:%pK len:0x%x\n",
				logpath, filename, buf, len);
		ret = -1;
		goto param_err;
	}

	snprintf(path, PATH_MAXLEN, "%s/%s", logpath, filename);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	flags = O_CREAT | O_RDWR | (is_append ? O_APPEND : O_TRUNC);
	fp = filp_open(path, flags, FILE_LIMIT);
	if (IS_ERR(fp)) {
		set_fs(old_fs);
		hwlog_err("%s():create file %s err.\n", __func__, path);
		ret = -1;
		goto param_err;
	}

	vfs_llseek(fp, 0L, SEEK_END);
	ret = vfs_write(fp, buf, len, &(fp->f_pos));
	if (ret != len) {
		hwlog_err("%s:write file %s exception with ret %d.\n",
				__func__, path, ret);
		goto write_err;
	}

	vfs_fsync(fp, 0);
write_err:
	filp_close(fp, NULL);
	set_fs(old_fs);
param_err:
	return ret;
}

static int sh_readfs2buf(char *logpath, char *filename, void *buf, u32 len)
{
	int ret = -1;
	int flags;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	char path[PATH_MAXLEN];

	if (!logpath || !filename || !buf || len <= 0) {
		hwlog_err("invalid  parameter. path:%pK, name:%pK buf:%pK len:0x%x\n",
				logpath, filename, buf, len);
		goto param_err;
	}

	snprintf(path, PATH_MAXLEN, "%s/%s", logpath, filename);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (sys_access(path, 0) != 0)
		goto file_err;

	flags = O_RDWR;
	fp = filp_open(path, flags, FILE_LIMIT);
	if (IS_ERR(fp)) {
		hwlog_err("%s():open file %s err.\n", __func__, path);
		goto file_err;
	}

	vfs_llseek(fp, 0L, SEEK_SET);
	ret = vfs_read(fp, buf, len, &(fp->f_pos));
	if (ret != len) {
		hwlog_err("%s:read file %s exception with ret %d.\n",
				__func__, path, ret);
		ret = -1;
	}
	filp_close(fp, NULL);
file_err:
	set_fs(old_fs);
param_err:
	return ret;
}

static int sh_create_dir(const char *path)
{
	char cur_path[CUR_PATH_LEN];
	int index = 0;

	if (!path) {
		hwlog_err("invalid  parameter. path:%pK\n", path);
		return -1;
	}
	memset(cur_path, 0, 64);
	if (*path != '/')
		return -1;
	cur_path[index++] = *path++;
	while (*path != '\0') {
		if (*path == '/')
			__sh_create_dir(cur_path, CUR_PATH_LEN);

		cur_path[index] = *path;
		path++;
		index++;
	}
	return 0;
}

static int get_dump_reason_idx(void)
{
	if (g_config_on_ddr->dump_config.reason >= ARRAY_SIZE(sh_reset_reasons))
		return ARRAY_SIZE(sh_reset_reasons) - 1;
	else
		return g_config_on_ddr->dump_config.reason;
}

char *xhub_get_timestamp(void)
{
	struct rtc_time tm;
	struct timeval tv;
	static char databuf[DATA_MAXLEN + 1];

	(void)memset_s(databuf, DATA_MAXLEN + 1, 0, DATA_MAXLEN + 1);

	(void)memset_s(&tv, sizeof(tv), 0, sizeof(tv));

	(void)memset_s(&tm, sizeof(tm), 0, sizeof(tm));

	do_gettimeofday(&tv);
	tv.tv_sec -= (long)sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(tv.tv_sec, &tm);

	(void)snprintf(databuf, DATA_MAXLEN + 1, "%04d%02d%02d%02d%02d%02d",
		 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		 tm.tm_hour, tm.tm_min, tm.tm_sec);

	hwlog_err("xhub: %s [%s] !\n", __func__, databuf);
	return databuf;
}

u64 xhub_get_tick(void)
{
	/* use only one int value to save time: */
	struct timespec uptime;
#if (KERNEL_VERSION(4, 4, 0) > LINUX_VERSION_CODE)
	do_posix_clock_monotonic_gettime(&uptime);
#else
	ktime_get_ts(&uptime);
#endif
	get_monotonic_boottime(&uptime);
	return (u64)uptime.tv_sec;
}

static int write_sh_dump_history(void)
{
	int ret = 0;
	char buf[HISTORY_LOG_SIZE];
	struct kstat historylog_stat;
	mm_segment_t old_fs;
	char local_path[PATH_MAXLEN];
	char date[DATATIME_MAXLEN];

	hwlog_info("%s: write sensorhub dump history file\n", __func__);
	memset(date, 0, DATATIME_MAXLEN);

	snprintf(date, DATATIME_MAXLEN, "%s-%08lld", xhub_get_timestamp(),
			xhub_get_tick());

	memset(local_path, 0, PATH_MAXLEN);
	snprintf(local_path, PATH_MAXLEN, "%s/%s", g_dump_dir, "history.log");

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	/* check file size */
	if (vfs_stat(local_path, &historylog_stat) == 0 &&
				historylog_stat.size > HISTORY_LOG_MAX) {
		hwlog_info("truncate dump history log\n");
		sys_unlink(local_path); /* delete history.log */
	}

	set_fs(old_fs);
	/* write history file */
	memset(buf, 0, HISTORY_LOG_SIZE);

	if (snprintf_s(buf, HISTORY_LOG_SIZE, HISTORY_LOG_SIZE - 1, "reason [%s], [%02d], time [%s]\n",
			sh_reset_reasons[get_dump_reason_idx()], g_dump_index, date) == DUMP_FAIL)
		hwlog_err("%s write history failed\n", __func__);
	sh_savebuf2fs(g_dump_dir, "history.log", buf, strlen(buf), 1);
	return ret;
}

static void get_max_dump_cnt(void)
{
	int ret;
	uint32_t index;

	/* find max index */
	ret = sh_readfs2buf(g_dump_dir, "dump_max", &index, sizeof(index));
	if (ret < 0)
		g_dump_index = -1;
	else
		g_dump_index = index;
	g_dump_index++;
	if (g_dump_index == MAX_DUMP_CNT)
		g_dump_index = 0;
	sh_savebuf2fs(g_dump_dir, "dump_max", &g_dump_index,
				sizeof(g_dump_index), 0);
}

static int write_sh_dump_file(void *dump_addr, u32 len)
{
	char date[DATATIME_MAXLEN];
	char path[PATH_MAXLEN];

	memset(date, 0, DATATIME_MAXLEN);

	snprintf(date, DATATIME_MAXLEN, "%s-%08lld", xhub_get_timestamp(),
				xhub_get_tick());

	memset(path, 0, PATH_MAXLEN);
	snprintf(path, PATH_MAXLEN, "sensorhub-%02d.dmp", g_dump_index);
	hwlog_info("%s: write sensorhub dump  file %s\n", __func__, path);
	hwlog_err("sensorhub recovery source is %s\n",
				sh_reset_reasons[get_dump_reason_idx()]);
#if (KERNEL_VERSION(4, 4, 0) > LINUX_VERSION_CODE)
	flush_cache_all();
#endif
	pr_notice("%s, @%px, size = %x\n", __func__, dump_addr, len);
	sh_savebuf2fs(g_dump_dir, path, dump_addr, len, 0);
	return 0;
}

int save_sh_dump_file(void *dump_addr, u32 len)
{
	sh_wait_fs(g_dump_fs);
	hwlog_info("%s fs ready\n", __func__);
	/* check and create dump dir */
	if (sh_create_dir(g_dump_dir)) {
		hwlog_err("%s failed to create dir %s\n", __func__, g_dump_dir);
		return -1;
	}
	get_max_dump_cnt();
	/* write history file */
	write_sh_dump_history();
	/* write dump file */
	write_sh_dump_file(dump_addr, len);
	return 0;
}

void xhub_before_scp_reset(exp_source_t f)
{
	/* Complete the completion for response because for sensorhub reset and ipc will discard */
	complete(&type_record.resp_complete);

	if (f > SH_FAULT_INTERNELFAULT)
		g_config_on_ddr->dump_config.reason = (uint8_t)f;
}

int xhub_need_recovery(exp_source_t f)
{
	int old_state;

	old_state = atomic_read(&xhub_rec_state);
	hwlog_err("recovery prev state %d, f %u\n", old_state, (uint8_t)f);

	/* prev state is IDLE start recovery progress */
	if (old_state == XHUB_RECOVERY_IDLE) {
		atomic_set(&xhub_rec_state, SCP_RECOVERY_START);
		blocking_notifier_call_chain(&xhub_recovery_notifier_list,
						SCP_RECOVERY_START, NULL);
		xhub_before_scp_reset(f);

		reset_trigger();
	} else {
		hwlog_err("maybe in recovery process, xhub_rec_state %d\n", atomic_read(&xhub_rec_state));
	}
	return 0;
}

static int xhub_recovery_notifier(struct notifier_block *nb,
		unsigned long foo, void *bar)
{
	mutex_lock(&mutex_recovery_cmd);
	switch (foo) {
	case SCP_RECOVERY_START:
		g_xhub_state = XHUB_ST_RECOVERY;
		break;
	case XHUB_RECOVERY_DOING:
	case XHUB_RECOVERY_3RD_DOING:
		g_xhub_state = XHUB_ST_REPEAT;
		break;
	case XHUB_RECOVERY_FAILED:
		hwlog_err("%s -recovery failed\n", __func__);
		/* fallthrough */
	case XHUB_RECOVERY_IDLE:
		g_xhub_state = XHUB_ST_NORMAL;
		wake_up_all(&xhub_rec_waitq);
		break;
	default:
		hwlog_err("%s -unknow state %ld\n", __func__, foo);
		break;
	}
	mutex_unlock(&mutex_recovery_cmd);
	return 0;
}

static int xhub_reboot_notifier(struct notifier_block *nb, unsigned long foo,
			       void *bar)
{
	/* prevent access the emmc now: */
	hwlog_info("%s: %lu +\n", __func__, foo);
	if (foo == SYS_RESTART) {
		disable_sensors_when_sysreboot();
		disable_motions_when_sysreboot();
	}
	hwlog_info("%s: -\n", __func__);
	return 0;
}

static struct notifier_block reboot_notify = {
	.notifier_call = xhub_reboot_notifier,
	.priority = -1,
};

static struct notifier_block recovery_notify = {
	.notifier_call = xhub_recovery_notifier,
	.priority = -1,
};

int xhub_recovery_init(void)
{
	mutex_init(&mutex_recovery_cmd);
	atomic_set(&xhub_rec_state, XHUB_RECOVERY_IDLE);
	xhub_rec_wq = create_singlethread_workqueue("XHUB_REC_WQ");

	INIT_WORK(&xhub_rec_work, xhub_recovery_work);
	init_completion(&xhub_rec_done);
	init_completion(&recovery_completion);
	wakeup_source_init(&xhub_rec_wl, "xhub_rec_wl");

	init_waitqueue_head(&xhub_rec_waitq);
	register_xhub_recovery_notifier(&recovery_notify);
	register_reboot_notifier(&reboot_notify);
	atomic_notifier_chain_register(&panic_notifier_list, &xhub_panic_block);

	return 0;
}
