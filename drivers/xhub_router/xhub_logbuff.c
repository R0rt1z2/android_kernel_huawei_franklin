/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub logbuff module
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <log/hw_log.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include "xhub_route.h"
#include "xhub_logbuff.h"
#include "protocol.h"
#include "ipc_adapter.h"
#include "scp_helper.h"
#include "scp_ipi_pin.h"

#define LOG_BUFF_SIZE (1024*4)
#define DDR_LOG_BUFF_UPDATE_NR (DDR_LOG_BUFF_SIZE / LOG_BUFF_SIZE)
#define DDR_LOG_BUFF_UPDATE_WATER_MARK (DDR_LOG_BUFF_UPDATE_NR * 3 / 4)
#define DDR_LOG_BUFF_COPY_SIZE (DDR_LOG_BUFF_UPDATE_WATER_MARK * LOG_BUFF_SIZE)

#define LOG_SERIAL 1
#define LOG_BUFF 2

/* unite jiffies */
#define FLUSH_TIMEOUT (2*HZ)

static int is_opened;
static int is_new_data_available;
static int is_flush_complete;
static int flush_cnt;

static uint32_t xhub_log_r;
static uint32_t xhub_log_buf_head;
static uint32_t xhub_log_buf_rear;
static uint8_t *ddr_log_buff;
static uint8_t *local_log_buff;
static struct mutex logbuff_mutex;
static struct mutex logbuff_flush_mutex;
static struct proc_dir_entry *logbuff_dentry;
static uint32_t log_method = LOG_BUFF;
static uint32_t xhub_log_full_flag;
static bool inited;

#define CONFIG_FLUSH '1'
#define CONFIG_SERIAL '2'
#define CONFIG_BUFF '3'

#define EMG_LEVEL   0
#define ERR_LEVEL   1
#define WARN_LEVEL  2
#define INFO_LEVEL  3
#define DEBUG_LEVEL 4

#define NO_LOG_DEFAULT_LEVEL EMG_LEVEL

int32_t gmark = 0xF0;

static DECLARE_WAIT_QUEUE_HEAD(xhub_log_waitq);
static DECLARE_WAIT_QUEUE_HEAD(xhub_log_flush_waitq);


static inline void print_stat(int i)
{
	hwlog_debug("[%d][r %x][head %x][rear %x][full_flag %d]\n",
		i, xhub_log_r, xhub_log_buf_head,
		xhub_log_buf_rear, xhub_log_full_flag);
}

static int xhub_log_buff_left(void)
{
	hwlog_debug("%s %d\n", __func__,
		(xhub_log_buf_rear >= xhub_log_r) ?
		(xhub_log_buf_rear - xhub_log_r) :
		(DDR_LOG_BUFF_SIZE - (xhub_log_r - xhub_log_buf_rear)));
	if (xhub_log_full_flag &&
		(xhub_log_buf_rear == xhub_log_r))
		return 0;
	return (xhub_log_buf_rear >= xhub_log_r) ?
		(xhub_log_buf_rear - xhub_log_r) :
		(DDR_LOG_BUFF_SIZE - (xhub_log_r - xhub_log_buf_rear));
}

static void update_local_buff_index(uint32_t new_rear)
{
	/* update xhub_log_r */
	if (flush_cnt && (xhub_log_buff_left() >=
		(DDR_LOG_BUFF_SIZE - DDR_LOG_BUFF_COPY_SIZE)))
		xhub_log_r = new_rear;
	/* update xhub_log_buf_head */
	if (flush_cnt)
		xhub_log_buf_head = new_rear;
	/* update xhub_log_buf_rear */
	xhub_log_buf_rear = new_rear;
	xhub_log_full_flag = 0;
	hwlog_debug("[%s] %d %d %d\n", __func__, xhub_log_r,
		xhub_log_buf_head, xhub_log_buf_rear);
}

static int xhub_logbuff_open(struct inode *inode, struct file *file)
{
	hwlog_info("[%s]\n", __func__);
	if (is_opened) {
		hwlog_err("%s sensorhub logbuff already opened !\n", __func__);
		return -1;
	}
	xhub_log_r = xhub_log_buf_head;
	is_opened = 1;
	return 0;
}

static int xhub_logbuff_release(struct inode *inode, struct file *file)
{
	hwlog_info("sensorhub logbuff release\n");
	is_opened = 0;
	return 0;
}

static ssize_t xhub_logbuff_read(
	struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int error = 0;
	int cnt;
	int remain;

	cnt = (count > DDR_LOG_BUFF_COPY_SIZE) ? DDR_LOG_BUFF_COPY_SIZE : count;
	hwlog_debug("[%s]\n", __func__);
	if (!buf || !count || !local_log_buff)
		goto err;

	/* check cap */
	if (xhub_log_buff_left() >= count)
		goto copy;

	error = wait_event_interruptible(xhub_log_waitq, is_new_data_available != 0);
	if (error) {
		error = 0;
		goto err;
	}
copy:
	print_stat(2);
	hwlog_debug("[%s] copy cnt %d, %x\n", __func__, cnt, xhub_log_r);
	mutex_lock(&logbuff_mutex);
	is_new_data_available = 0;

	if (xhub_log_r > DDR_LOG_BUFF_SIZE) {
		remain = 0;
		hwlog_err("%s xhub_log_r is too large\n", __func__);
	} else {
		remain = DDR_LOG_BUFF_SIZE - xhub_log_r;
	}
	error = cnt;
	/* copy to user */
	if (cnt > remain) {
		if (copy_to_user(buf + remain, local_log_buff, (cnt - remain))) {
			hwlog_err("%s failed to copy to user\n", __func__);
			error = -EFAULT;
			goto out;
		}
		cnt = remain;
	}
	if (copy_to_user(buf, local_log_buff + xhub_log_r, cnt)) {
		hwlog_err("%s failed to copy to user\n", __func__);
		error = -EFAULT;
		goto out;
	}
	/* update reader pointer */
	xhub_log_r = (xhub_log_r + error) % DDR_LOG_BUFF_SIZE;
	if (!xhub_log_full_flag && xhub_log_r == xhub_log_buf_rear)
		xhub_log_full_flag = 1;

out:
	mutex_unlock(&logbuff_mutex);
err:
	print_stat(3);
	return error;
}

static const struct file_operations xhub_logbuff_operations = {
	.open = xhub_logbuff_open,
	.read = xhub_logbuff_read,
	.release = xhub_logbuff_release,
};

static int logbuff_full_callback(const pkt_header_t *head)
{
	int cnt = DDR_LOG_BUFF_COPY_SIZE;
	int remain;
	uint32_t update_index;
	uint32_t new_rear;
	log_buff_req_t *pkt = (log_buff_req_t *) head;

	if (pkt->index > DDR_LOG_BUFF_SIZE / LOG_BUFF_SIZE) {
		hwlog_err("%s index is too large, log maybe lost\n", __func__);
		update_index = DDR_LOG_BUFF_SIZE;
	} else {
		update_index = (pkt->index * LOG_BUFF_SIZE);
	}
	new_rear = (update_index + DDR_LOG_BUFF_COPY_SIZE) % DDR_LOG_BUFF_SIZE;
	hwlog_info("[%s]\n", __func__);
	if (update_index != xhub_log_buf_rear)
		hwlog_err("%s unsync index, log maybe lost\n", __func__);
	/* get rotate log buff index */
	mutex_lock(&logbuff_mutex);
	print_stat(4); /* stat 4 */
	update_local_buff_index(new_rear);
	print_stat(5); /* stat 5 */
	remain = DDR_LOG_BUFF_SIZE - update_index;
	/* update reader pointer */
	if (remain < DDR_LOG_BUFF_COPY_SIZE) {
		memcpy(local_log_buff, ddr_log_buff, cnt - remain);
		cnt = remain;
	}
	memcpy(local_log_buff + update_index, ddr_log_buff + update_index, cnt);
	if (!flush_cnt)
		flush_cnt = 1;

	is_new_data_available = 1;
	mutex_unlock(&logbuff_mutex);

	/* wake up reader */
	hwlog_debug("%s wakeup\n", __func__);
	wake_up_interruptible(&xhub_log_waitq);
	return 0;
}

static int logbuff_flush_callback(const pkt_header_t *head)
{
	/* sensorhub has flush tcm log buff
	 * we need update logbuff global vars and flush it to file system
	 */
	log_buff_req_t *pkt = (log_buff_req_t *) head;
	uint32_t flush_head = (pkt->index * LOG_BUFF_SIZE);
	uint32_t flush_size;
	int remain;
	int timeout_cnt = 100;

	flush_size = (flush_head > xhub_log_buf_rear) ?
		(flush_head - xhub_log_buf_rear) :
		(DDR_LOG_BUFF_SIZE - (xhub_log_buf_rear - flush_head));
	hwlog_debug("[%s] index: %d\n", __func__, pkt->index);
	/* wait reader till we can update the head */
	while (xhub_log_buff_left() >
		(DDR_LOG_BUFF_COPY_SIZE - flush_size) && timeout_cnt--)
		msleep(1);
	if (timeout_cnt < 0)
		hwlog_warn("%s timeout, some log will lost", __func__);

	/* get rotate log buff index */
	mutex_lock(&logbuff_mutex);
	remain = DDR_LOG_BUFF_SIZE - xhub_log_buf_rear;
	if (remain < flush_size) {
		memcpy(local_log_buff, ddr_log_buff, flush_size - remain);
		flush_size = remain;
	}

	memcpy(local_log_buff + xhub_log_buf_rear,
		ddr_log_buff + xhub_log_buf_rear,
		flush_size);
	print_stat(6); /* stat 6 */
	update_local_buff_index(flush_head);
	print_stat(7); /* stat 7 */
	if (!flush_cnt)
		flush_cnt = 1;

	is_new_data_available = 1;
	mutex_unlock(&logbuff_mutex);

	/* wake up reader */
	wake_up_interruptible(&xhub_log_waitq);
	is_flush_complete = 1;
	wake_up_interruptible(&xhub_log_flush_waitq);
	return 0;
}

static void __manual_flush(pkt_header_t *pkt, int size)
{
	write_info_t winfo;

	hwlog_debug("flush sensorhub log buff\n");
	/* do log flush */
	mutex_lock(&logbuff_flush_mutex);
	winfo.tag = pkt->tag;
	winfo.cmd = CMD_LOG_BUFF_FLUSH;
	winfo.wr_len = pkt->length;
	winfo.wr_buf = NULL;
	write_customize_cmd(&winfo, NULL, true);
	if (!wait_event_interruptible_timeout(
		xhub_log_flush_waitq,
		is_flush_complete != 0, FLUSH_TIMEOUT))
		hwlog_err("%s no response", __func__);
	hwlog_debug("flush sensorhub log buff done\n");
	is_flush_complete = 0;
	mutex_unlock(&logbuff_flush_mutex);
}

static ssize_t logbuff_config_set(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	write_info_t winfo;
	pkt_header_t pkt = {
		.tag = TAG_LOG_BUFF,
		.resp = NO_RESP,
		.length = 0
	};
	winfo.tag = TAG_LOG_BUFF,
	winfo.wr_len = 0;
	winfo.wr_buf = NULL;

	if (buf[0] == CONFIG_FLUSH) {
		__manual_flush(&pkt, sizeof(pkt));
	} else if (buf[0] == CONFIG_SERIAL) {
		hwlog_info("sensorhub log use serial port\n");
		winfo.cmd = CMD_LOG_SER_REQ;
		write_customize_cmd(&winfo, NULL, true);
		log_method = LOG_SERIAL;
	} else if (buf[0] == CONFIG_BUFF) {
		hwlog_info("sensorhub log use log buff\n");
		winfo.cmd = CMD_LOG_USEBUF_REQ;
		write_customize_cmd(&winfo, NULL, true);
		log_method = LOG_BUFF;
	} else {
		hwlog_err("%s wrong input, \'1\' for flush \'2\' for serial \'3\' for buff\n", __func__);
		return -EINVAL;
	}
	return count;
}

static ssize_t logbuff_config_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(
		buf, "%s\n", (log_method == LOG_SERIAL) ? "serial" : "buff");
}

static ssize_t logbuff_flush_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	pkt_header_t pkt = {
		.tag = TAG_LOG_BUFF,
		.resp = NO_RESP,
		.length = 0
	};

	/* do flush here */
	__manual_flush(&pkt, sizeof(pkt));
	return sprintf(buf, "0\n");
}

static DEVICE_ATTR(logbuff_config, S_IWUSR | S_IRUGO,
	logbuff_config_show, logbuff_config_set);

static DEVICE_ATTR(logbuff_flush, S_IRUGO, logbuff_flush_show, NULL);

static struct platform_device xhub_logbuff = {
	.name = "huawei_sensorhub_logbuff",
	.id = -1,
};

void reset_logbuff(void)
{
	xhub_log_r = 0;
	xhub_log_buf_head = 0;
	xhub_log_buf_rear = 0;
	flush_cnt = 0;
	/* write the head to ddr config block */
	g_config_on_ddr->log_buff_cb_backup.mutex = 0;
	if (local_log_buff && ddr_log_buff) {
		memset(local_log_buff, 0, DDR_LOG_BUFF_SIZE);
		memset(ddr_log_buff, 0, DDR_LOG_BUFF_SIZE);
	}
}

void emg_flush_logbuff(void)
{
	log_buff_req_t pkt;

	pkt.index = xhub_log_buf_rear / LOG_BUFF_SIZE;
	hwlog_info("[%s] update head %x\n", __func__, pkt.index);
	if (!inited) {
		hwlog_err("logbuff not inited!\n");
		return;
	}
	/* notify userspace */
	logbuff_full_callback((const pkt_header_t *)&pkt);
	msleep(100); // sleep 100 ticks
}

static struct delayed_work xhub_logbuff_off_check_work;

static void  xhub_logbuff_off_check(struct work_struct *work)
{
	int level = NO_LOG_DEFAULT_LEVEL;

	hwlog_info("%s\n", __func__);
	if (!is_opened)
		// set log level to emg
		set_log_level(TAG_LOG_BUFF, &level, 1);
}

int logbuff_dentry_init(void)
{
	logbuff_dentry = proc_create("sensorhub_logbuff", S_IRUSR | S_IRGRP,
		NULL, &xhub_logbuff_operations);
	if (!logbuff_dentry) {
		hwlog_err("%s failed to create logbuff_dentry\n", __func__);
		return -1;
	}
	return 0;
}

int logbuff_device_register(void)
{
	int ret;

	ret = platform_device_register(&xhub_logbuff);
	if (ret) {
		hwlog_err("%s: platform_device_register failed, ret:%d.\n",
			__func__, ret);
		return -1;
	}
	return 0;
}

int creat_logbuff_config_file(void)
{
	int ret;

	ret = device_create_file(&xhub_logbuff.dev,
		&dev_attr_logbuff_config);
	if (ret) {
		hwlog_err("%s: create %s file failed, ret:%d.\n",
			__func__, "dev_attr_logbuff_config", ret);
		return -1;
	}
	return 0;
}

int creat_logbuff_flush_file(void)
{
	int ret;

	ret = device_create_file(&xhub_logbuff.dev,
		&dev_attr_logbuff_flush);
	if (ret) {
		hwlog_err("%s: create %s file failed, ret:%d.\n",
			__func__, "dev_attr_logbuff_flush", ret);
		return -1;
	}
	return 0;
}

int logbuff_register_alert_notifier(void)
{
	int ret;

	ret = register_mcu_event_notifier(TAG_LOG_BUFF,
		CMD_LOG_BUFF_ALERT, logbuff_full_callback);
	if (ret) {
		hwlog_err("%s failed register notifier CMD_LOG_BUFF_ALERT:ret %d\n",
			__func__, ret);
		return -1;
	}
	return 0;
}

int logbuff_register_event_notifier(void)
{
	int ret;

	ret = register_mcu_event_notifier(TAG_LOG_BUFF,
		CMD_LOG_BUFF_FLUSHP, logbuff_flush_callback);
	if (ret) {
		hwlog_err("%s failed register notifier CMD_LOG_BUFF_FLUSH:ret %d\n",
			__func__, ret);
		return -1;
	}
	return 0;
}

int remap_ddr_log_buff(void)
{
	ddr_log_buff = (uint8_t *)(uintptr_t)scp_get_reserve_mem_virt(SCP_LOGGER_MEM_ID);
	if (!ddr_log_buff) {
		hwlog_err("%s failed remap log buff\n", __func__);
		return -1;
	}
	return 0;
}

int mloc_local_log_buff(void)
{
	local_log_buff = (uint8_t *) vmalloc(DDR_LOG_BUFF_SIZE);
	if (!local_log_buff) {
		hwlog_err("%s failed to malloc\n", __func__);
		return -1;
	}
	return 0;
}

static void logbuff_init_success(void)
{
	memset(local_log_buff, 0, DDR_LOG_BUFF_SIZE);
	mutex_init(&logbuff_mutex);
	mutex_init(&logbuff_flush_mutex);
	inited = true;
	INIT_DELAYED_WORK(&xhub_logbuff_off_check_work,
		xhub_logbuff_off_check);
	queue_delayed_work(system_freezable_wq,
		&xhub_logbuff_off_check_work, 5 * 60 * HZ); // delay 300 Hz
	hwlog_info("[%s] done\n", __func__);
}

static int xhub_logbuff_init(void)
{
	if (!is_scp_ready(SCP_A_ID))
		return -1;
	hwlog_info("[%s]\n", __func__);
	if (!get_sensor_mcu_mode()) {
		hwlog_err("%s :mcu boot fail, logbuff init err\n", __func__);
		return -1;
	}
	if (logbuff_dentry_init())
		goto PROC_ERR;
	if (logbuff_device_register())
		goto REGISTER_ERR;
	if (creat_logbuff_config_file())
		goto SYSFS_ERR_1;
	if (creat_logbuff_flush_file())
		goto SYSFS_ERR_2;
	if (logbuff_register_alert_notifier())
		goto NOTIFY_ERR;
	if (logbuff_register_event_notifier())
		goto NOTIFY_ERR_1;
	if (remap_ddr_log_buff())
		goto REMAP_ERR;
	if (mloc_local_log_buff())
		goto MALLOC_ERR;
	logbuff_init_success();
	return 0;
MALLOC_ERR:
	iounmap(ddr_log_buff);
REMAP_ERR:
	unregister_mcu_event_notifier(TAG_LOG_BUFF,
		CMD_LOG_BUFF_FLUSHP, logbuff_flush_callback);
NOTIFY_ERR_1:
	unregister_mcu_event_notifier(TAG_LOG_BUFF,
		CMD_LOG_BUFF_ALERT, logbuff_full_callback);
NOTIFY_ERR:
	device_remove_file(&xhub_logbuff.dev, &dev_attr_logbuff_flush);
SYSFS_ERR_2:
	device_remove_file(&xhub_logbuff.dev, &dev_attr_logbuff_config);
SYSFS_ERR_1:
	platform_device_unregister(&xhub_logbuff);
REGISTER_ERR:
	proc_remove(logbuff_dentry);
PROC_ERR:
	return -1;
}

static void xhub_logbuff_exit(void)
{
	iounmap(ddr_log_buff);
	ddr_log_buff = NULL;
	vfree(local_log_buff);
	local_log_buff = NULL;
	unregister_mcu_event_notifier(
		TAG_LOG_BUFF, CMD_LOG_BUFF_FLUSHP, logbuff_flush_callback);
	unregister_mcu_event_notifier(
		TAG_LOG_BUFF, CMD_LOG_BUFF_ALERT, logbuff_full_callback);
	device_remove_file(&xhub_logbuff.dev, &dev_attr_logbuff_flush);
	device_remove_file(&xhub_logbuff.dev, &dev_attr_logbuff_config);
	platform_device_unregister(&xhub_logbuff);
	proc_remove(logbuff_dentry);
}

late_initcall_sync(xhub_logbuff_init);
module_exit(xhub_logbuff_exit);

MODULE_LICENSE("GPL");
