/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub route module
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
#include <linux/fb.h>
#include <linux/rtc.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include <securec.h>
#include "protocol.h"
#include "sensor_config.h"
#include "xhub_recovery.h"
#include "xhub_pm.h"
#include <xhub_router/motionhub.h>
#include <log/imonitor.h>
#include "xhub_route.h"
#include "xhub_boot.h"
#include "als_channel.h"
#include "ipc_adapter.h"

/* -----------------------------
 * | section_id | section_size |
 * -----------------------------
 * |      0     |      8KB     |
 * -----------------------------
 * |      1     |      8KB     |
 * -----------------------------
 * |      2     |      8KB     |
 * -----------------------------
 * |      3     |      8KB     |
 * -----------------------------
 * |      4     |      8KB     |
 * -----------------------------
 * |      5     |      8KB     |
 * -----------------------------
 * |      6     |      8KB     |
 * -----------------------------
 * |      7     |      8KB     |
 * -----------------------------
 * |      8     |      16KB    |
 * -----------------------------
 * |      9     |      16KB    |
 * -----------------------------
 * |      10    |      32KB    |
 * -----------------------------
 * in each section, 8 bytes are reserved for control word.
 */
#define SHMEM_SIZE                (256 * 1024)
#define SECTION_UNIT_SIZE         (8 * 1024)
#define SECTION_CTL_CMD_LEN       8 // byte
#define LITTLE_SECTION_SIZE       SECTION_UNIT_SIZE
#define LITTLE_SECTION_N          8
#define MIDDLE_SECTION_SIZE       (2 * SECTION_UNIT_SIZE)
#define MIDDLE_SECTION_N          2
#define LARGE_SECTION_SIZE        (4 * SECTION_UNIT_SIZE)
#define LARGE_SECTION_N           1
#define TOTAL_SECTION_N           (LITTLE_SECTION_N + MIDDLE_SECTION_N + LARGE_SECTION_N)

#define MAX_SECTION_N             (SHMEM_SIZE / 2 / SECTION_UNIT_SIZE)
#define SECTION_ID_ERR            255

#define SHMEM_SEND                0
#define SHMEM_RECV                1

static atomic_t g_section_atomic = ATOMIC_INIT(1);
u64 g_shmem_send_start;
u64 g_shmem_recv_start;

#define ROUTE_BUFFER_MAX_SIZE (1024 * 128)
#define SENSOR_DROP_IMONITOR_ID 936005000
#ifdef TIMESTAMP_SIZE
#undef TIMESTAMP_SIZE
#define TIMESTAMP_SIZE 8
#endif

int step_ref_cnt;
int g_xhub_state = XHUB_ST_NORMAL;
int get_airpress_data;
int get_temperature_data;
/* once this flag set to 1, ap/iom3 will not transfer commd */
int fobidden_comm;
int ext_hall_type;
struct sensor_status sensor_status;
spinlock_t fsdata_lock;
bool fingersense_data_ready = false;
bool fingersense_data_intrans; /* the data is on the way */
s16 fingersense_data[FINGERSENSE_DATA_NSAMPLES];
struct type_record type_record;

static struct wakeup_source wlock;
static struct mutex mutex_write_cmd;
static struct mutex mutex_write_adapter;
static struct mutex mutex_unpack;
static struct mcu_notifier mcu_notifier = { LIST_HEAD_INIT(mcu_notifier.head) };
static int ps_value; /* save ps value for phonecall dmd */
static int als_retry_cnt = ALS_RESET_COUNT;
static int ps_retry_cnt = PS_RESET_COUNT;
static int tp_retry_cnt = TP_RESET_COUNT;
static int64_t sensors_tm[TAG_SENSOR_END];
static spinlock_t ref_cnt_lock;
static uint64_t als_last_reset_period;
static uint64_t ps_last_reset_period;
static uint64_t tp_last_reset_period;
static uint32_t valid_step_count;
static uint32_t recovery_step_count;
static uint32_t valid_floor_count;
static uint32_t recovery_floor_count;

static uint16_t tranid;
static int xhub_timeout = 2000;

#ifdef CONFIG_WIRELESS_ACCESSORY
static int g_pre_wireless_status;
static uint32_t g_pre_kbhall_value;
static uint32_t g_pre_pen_state;
static uint32_t g_pre_kb_state;
static uint32_t pre_lightstrap_state;
#endif

struct hw_ap_cp_route_t icc_route; // hanping
static unsigned int adapt_ext_hall_index;
static DEFINE_MUTEX(mutex_update);

static struct xhub_route_table package_route_tbl[] = {
	{ ROUTE_SHB_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[0].read_wait) },
	{ ROUTE_MOTION_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[1].read_wait) },
	{ ROUTE_CA_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[2].read_wait) },
	{ ROUTE_FHB_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[3].read_wait) },
	{ ROUTE_FHB_UD_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[4].read_wait) },
	{ ROUTE_KB_PORT, {NULL, 0}, {NULL, 0}, {NULL, 0},
		__WAIT_QUEUE_HEAD_INITIALIZER(package_route_tbl[5].read_wait) },
};

static void update_shmem_tranid(pkt_header_t *head);
static struct {
	int ext_hall_adapt;
	int ext_hall_value[HALL_ONE_DATA_NUM];
} ext_hall_table[] = {
	{ 0, { 1, 0, 2, -1 } },
};

static int report_sensor_event_batch(int tag, int value[],
	int length, uint64_t timestamp);
bool really_do_enable_disable(int *ref_cnt, bool enable, int bit)
{
	bool ret = false;
	unsigned long flags = 0;

	if ((bit < 0) || (bit >= sizeof(int32_t) << 3)) {
		hwlog_err("bit %d out of range in %s.\n", bit, __func__);
		return false;
	}

	spin_lock_irqsave(&ref_cnt_lock, flags);
	if (enable) {
		ret = (*ref_cnt == 0);
		*ref_cnt |= 1 << bit;
	} else {
		*ref_cnt &= ~(1 << bit);
		ret = (*ref_cnt == 0);
	}
	spin_unlock_irqrestore(&ref_cnt_lock, flags);
	return ret;
}

static int g_tsa_event;
static int g_volume_key;
void get_tp_data(unsigned int version, unsigned int tsa_event,
	unsigned int volume_key)
{
	hwlog_info("%s version: %d, tsaEvent: %d, volumeKey: %d\n",
		__func__, version, tsa_event, volume_key);
	g_tsa_event = tsa_event;
	g_volume_key = volume_key;
}

static int xhub_route_item(unsigned short port,
	struct xhub_route_table **route_item)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(package_route_tbl); ++i) {
		if (port == package_route_tbl[i].port) {
			*route_item = &package_route_tbl[i];
			return 0;
		}
	}

	hwlog_err("unknown port: %d in %s.\n", port, __func__);
	return -EINVAL;
}

int xhub_route_open(unsigned short port)
{
	int ret;
	char *pos = NULL;
	struct xhub_route_table *route_item = NULL;

	hwlog_info("%s\n", __func__);
	ret = xhub_route_item(port, &route_item);
	if (ret < 0)
		return -EINVAL;

	if (route_item->phead.pos) {
		hwlog_err("port:%d was already opened in %s.\n",
			port, __func__);
		return -EINVAL;
	}

	pos = vzalloc(ROUTE_BUFFER_MAX_SIZE);
	if (!pos)
		return -ENOMEM;

	route_item->phead.pos = pos;
	route_item->pwrite.pos = pos;
	route_item->pread.pos = pos;
	route_item->phead.buffer_size = ROUTE_BUFFER_MAX_SIZE;
	route_item->pwrite.buffer_size = ROUTE_BUFFER_MAX_SIZE;
	route_item->pread.buffer_size = 0;
	return 0;
}

void xhub_route_close(unsigned short port)
{
	int ret;
	struct xhub_route_table *route_item = NULL;

	hwlog_info("%s\n", __func__);
	ret = xhub_route_item(port, &route_item);
	if (ret < 0)
		return;

	if (route_item->phead.pos)
		vfree(route_item->phead.pos);

	route_item->phead.pos = NULL;
	route_item->pwrite.pos = NULL;
	route_item->pread.pos = NULL;
}

void xhub_route_clean_buffer(unsigned short port)
{
	int ret;
	struct xhub_route_table *route_item = NULL;
	unsigned long flags = 0;

	hwlog_info("%s\n", __func__);
	ret = xhub_route_item(port, &route_item);
	if (ret < 0)
		return;

	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	route_item->pread.pos = route_item->pwrite.pos;
	route_item->pwrite.buffer_size = ROUTE_BUFFER_MAX_SIZE;
	route_item->pread.buffer_size = 0;
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
}

static inline bool data_ready(struct xhub_route_table *route_item,
	struct xhub_buffer_pos *reader)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	*reader = route_item->pread;
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
	return reader->buffer_size > 0;
}

ssize_t xhub_route_read(unsigned short port, char __user *buf,
	size_t count)
{
	struct xhub_route_table *route_item = NULL;
	struct xhub_buffer_pos reader;
	char *buffer_end = NULL;
	unsigned int full_pkg_length;
	unsigned int tail_half_len;
	unsigned long flags = 0;

	if (xhub_route_item(port, &route_item) != 0) {
		hwlog_err("xhub_route_item failed in %s\n", __func__);
		return 0;
	}

	buffer_end = route_item->phead.pos + route_item->phead.buffer_size;

	/* woke up by signal */
	if (wait_event_interruptible(route_item->read_wait,
			data_ready(route_item, &reader)) != 0)
		return 0;

	if (reader.buffer_size > ROUTE_BUFFER_MAX_SIZE) {
		hwlog_err("error reader.buffer_size = %d in port %d!\n",
			(int)reader.buffer_size, (int)port);
		goto clean_buffer;
	}

	if (buffer_end - reader.pos >= LENGTH_SIZE) {
		full_pkg_length = *((unsigned int *)reader.pos);
		reader.pos += LENGTH_SIZE;
		if (reader.pos == buffer_end)
			reader.pos = route_item->phead.pos;
	} else {
		tail_half_len = buffer_end - reader.pos;
		memcpy(&full_pkg_length, reader.pos, tail_half_len);
		memcpy((char *)&full_pkg_length + tail_half_len,
			route_item->phead.pos, LENGTH_SIZE - tail_half_len);
		reader.pos = route_item->phead.pos +
			(LENGTH_SIZE - tail_half_len);
	}

	if (full_pkg_length + LENGTH_SIZE > reader.buffer_size ||
			full_pkg_length > count) {
		hwlog_err("full_pkg_length = %u is too large in port %d!\n",
			full_pkg_length, (int)port);
		goto clean_buffer;
	}

	if (buffer_end - reader.pos >= full_pkg_length) {
		if (copy_to_user(buf, reader.pos, full_pkg_length) == 0) {
			reader.pos += full_pkg_length;
			if (reader.pos == buffer_end)
				reader.pos = route_item->phead.pos;
		} else {
			hwlog_err("copy to user failed\n");
			return 0;
		}
	} else {
		tail_half_len = buffer_end - reader.pos;
		if ((copy_to_user(buf, reader.pos, tail_half_len) == 0) &&
		    (copy_to_user(buf + tail_half_len, route_item->phead.pos,
				(full_pkg_length - tail_half_len)) == 0)) {
			reader.pos = route_item->phead.pos +
				(full_pkg_length - tail_half_len);
		} else {
			hwlog_err("copy to user failed\n");
			return 0;
		}
	}
	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	route_item->pread.pos = reader.pos;
	route_item->pread.buffer_size -= (full_pkg_length + LENGTH_SIZE);
	if ((route_item->pwrite.buffer_size > ROUTE_BUFFER_MAX_SIZE) ||
		(route_item->pwrite.buffer_size +
			(full_pkg_length + LENGTH_SIZE) >
			ROUTE_BUFFER_MAX_SIZE)) {
		hwlog_err("%s:%d write buffer error size=%u len=%u\n",
			__func__, __LINE__,
			route_item->pwrite.buffer_size, full_pkg_length);
		spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
		goto clean_buffer;
	} else {
		route_item->pwrite.buffer_size +=
			(full_pkg_length + LENGTH_SIZE);
	}
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
	return full_pkg_length;

clean_buffer:
	hwlog_err("now we will clear the receive buffer in port %d!\n", (int)port);
	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	route_item->pread.pos = route_item->pwrite.pos;
	route_item->pwrite.buffer_size = ROUTE_BUFFER_MAX_SIZE;
	route_item->pread.buffer_size = 0;
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(xhub_route_read);

static int64_t get_timestamp(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	/* timevalToNano */
	return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static inline void write_to_fifo(struct xhub_buffer_pos *pwriter,
	char *const buffer_begin, char *const buffer_end, char *buf, int count)
{
	int cur_to_tail_len = buffer_end - pwriter->pos;

	if (cur_to_tail_len >= count) {
		memcpy(pwriter->pos, buf, count);
		pwriter->pos += count;
		if (buffer_end == pwriter->pos)
			pwriter->pos = buffer_begin;
	} else {
		memcpy(pwriter->pos, buf, cur_to_tail_len);
		memcpy(buffer_begin, buf + cur_to_tail_len,
			count - cur_to_tail_len);
		pwriter->pos = buffer_begin + (count - cur_to_tail_len);
	}
}

t_ap_sensor_ops_record all_ap_sensor_operations[TAG_SENSOR_END] = {
#ifdef CONFIG_HUAWEI_HALL_INPUTHUB
	[TAG_HALL] = {
		.work_on_ap = true,
		.ops = {.setdelay = ak8789_register_report_data},
	},
	[TAG_EXT_HALL] = {
		.work_on_ap = true,
	},
#endif
#ifdef CONFIG_SENSORS_COLOR_AP
	[TAG_COLOR] = {
		.work_on_ap = true,
		.ops = {
			.enable = color_sensor_enable,
		},
	},
#endif
};

int register_ap_sensor_operations(int tag, sensor_operation_t *ops)
{
	if (!(tag >= TAG_SENSOR_BEGIN && tag < TAG_SENSOR_END)) {
		hwlog_err("tag %d range error in %s\n", tag, __func__);
		return -EINVAL;
	}

	if (!all_ap_sensor_operations[tag].work_on_ap) {
		memcpy(&all_ap_sensor_operations[tag].ops, ops,
			sizeof(sensor_operation_t));
		all_ap_sensor_operations[tag].work_on_ap = true;
	} else {
		hwlog_warn("tag %d has registered already in %s\n",
			tag, __func__);
	}

	return 0;
}

int unregister_ap_sensor_operations(int tag)
{
	if (!(tag >= TAG_SENSOR_BEGIN && tag < TAG_SENSOR_END)) {
		hwlog_err("tag %d range error in %s\n", tag, __func__);
		return -EINVAL;
	}
	memset(&all_ap_sensor_operations[tag], 0,
		sizeof(all_ap_sensor_operations[tag]));
	return 0;
}

static void sensor_get_data(struct sensor_data *data)
{
	struct t_sensor_get_data *get_data = NULL;

	if (!data || (!((data->type >= SENSORHUB_TYPE_BEGIN) &&
		(data->type < SENSORHUB_TYPE_END)))) {
		hwlog_err("%s, para err\n", __func__);
		return;
	}

	get_data = &sensor_status.get_data[data->type];
	if (atomic_cmpxchg(&get_data->reading, 1, 0)) {
		memcpy(&get_data->data, data, sizeof(get_data->data));
		complete(&get_data->complete);
	}
}

int report_sensor_event(int tag, int value[], int length)
{
	struct sensor_data event;

	if ((!(tag >= TAG_SENSOR_BEGIN && tag < TAG_SENSOR_END)) ||
		(length > sizeof(event.value))) {
		hwlog_err("para error (tag : %d), (length : %d) in %s\n",
			tag, length, __func__);
		return -EINVAL;
	}

	event.type = tag_to_hal_sensor_type[tag];
	event.length = length;
	memcpy(&event.value, value, length);
	sensor_get_data(&event);

	return xhub_route_write(ROUTE_SHB_PORT, (char *)&event,
		event.length + offset_of_end_mem(struct sensor_data, length));
}

#ifdef CONFIG_HUAWEI_HISHOW
void check_hall_hishow_state(uint32_t type)
{
	if ((type & hall_hishow_value) == hall_hishow_value) {
		hwlog_info("check_hall_hishow_state is connected type: %d\n", type);
		/* start hishow */
		hishow_notify_android_uevent(HISHOW_DEVICE_ONLINE, HISHOW_HALL_DEVICE);
	}
	if ((type & hall_hishow_value) == 0) {
		hwlog_info("check_hall_hishow_state is disconnect type: %d\n", type);
		/* stop hishow */
		hishow_notify_android_uevent(HISHOW_DEVICE_OFFLINE, HISHOW_HALL_DEVICE);
	}
}
#endif

#ifdef CONFIG_WIRELESS_ACCESSORY
static void check_hall_pen_state(uint32_t value)
{
	uint32_t pen_state;

	pen_state = value & hall_pen_value;
	if (g_pre_pen_state != pen_state) {
		if (pen_state) {
			hwlog_info("%s, pen hall approach, open wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_aux_event_nh, WL_TX_EVENT_HALL_APPROACH, NULL);
		} else {
			hwlog_info("%s, pen hall away from, close wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_aux_event_nh, WL_TX_EVENT_HALL_AWAY_FROM, NULL);
		}
		g_pre_pen_state = pen_state;
	}
}

static void check_hall_keyboard_state(uint32_t value)
{
	uint32_t kb_state;

	kb_state = value & hall_keyboard_value;
	if (g_pre_kb_state != kb_state) {
		if (kb_state) {
			hwlog_info("%s, kb hall approach, open wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_event_nh, WL_TX_EVENT_HALL_APPROACH, NULL);
		} else {
			hwlog_info("%s, kb hall away from, close wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_event_nh, WL_TX_EVENT_HALL_AWAY_FROM, NULL);
		}
		g_pre_kb_state = kb_state;
	}
}

static void check_hall_lightstrap_state(uint32_t value)
{
	uint32_t light_state;

	light_state = value & hall_lightstrap_value;
	if (pre_lightstrap_state != light_state) {
		if (light_state) {
			hwlog_info("%s, approach, open wireless tx\n", __func__);
			lightstrap_event_notify(LIGHTSTRAP_EVENT_ON, NULL);
		} else {
			hwlog_info("%s, away, close wireless tx\n", __func__);
			lightstrap_event_notify(LIGHTSTRAP_EVENT_OFF, NULL);
		}
		pre_lightstrap_state = light_state;
	}
}
#endif

static int adapt_hall_value(int value)
{
	if (adapt_ext_hall_index >= ARRAY_SIZE(ext_hall_table))
		return -EPERM;
	if (value > MAX_EXT_HALL_VALUE)
		return -EPERM;
	return ext_hall_table[adapt_ext_hall_index].ext_hall_value[value];
}
#ifdef CONFIG_WIRELESS_ACCESSORY
static void kbhall_status_change_notify(uint32_t kb_value)
{
	if (g_pre_kbhall_value != kb_value) {
		if (kb_value) {
			hwlog_info("%s, kb hall approach, open wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_event_nh, WL_TX_EVENT_HALL_APPROACH, NULL);
		} else {
			hwlog_info("%s, kb hall away from, close wireless aux tx\n", __func__);
			blocking_notifier_call_chain(&tx_event_nh, WL_TX_EVENT_HALL_AWAY_FROM, NULL);
		}
		g_pre_kbhall_value = kb_value;
	}
}
#endif

int ap_hall_report(int value)
{
	int double_hall_value[3] = {0};
	int ext_hall_value[HALL_DATA_NUM] = {0};
#ifdef CONFIG_WIRELESS_ACCESSORY
	int wireless_hall_status;
#endif

	if (get_sensor_mcu_mode() == 0) { // xhub not ready do not report
		hwlog_info("%s : xhub not ready do not report\n", __func__);
		return 0;
	}

#ifdef CONFIG_HUAWEI_HISHOW
	if (support_hall_hishow)
		check_hall_hishow_state((uint32_t)value);
#endif

#ifdef CONFIG_WIRELESS_ACCESSORY
	if (support_hall_pen)
		check_hall_pen_state((uint32_t)value);

	if (support_hall_keyboard)
		check_hall_keyboard_state((uint32_t)value);

	if (support_hall_lightstrap)
		check_hall_lightstrap_state((uint32_t)value);
#endif

	/* here hall trigger wireless aux tx function */
	if (hall_number == 4) { /* hall number of marx is 4 */
		hwlog_info("%s, value = %d\n", __func__, value);
#ifdef CONFIG_WIRELESS_ACCESSORY
		/* report 0x10000  if wirelessHall triggered */
		wireless_hall_status = ((uint32_t)value & 0x1F) >> 4;
		if (g_pre_wireless_status != wireless_hall_status) {
			if (wireless_hall_status) {
				hwlog_info("%s, hall approach, open wireless aux tx\n", __func__);
				blocking_notifier_call_chain(&tx_aux_event_nh, WL_TX_EVENT_HALL_APPROACH, NULL);
			} else {
				hwlog_info("%s, hall away from, close wireless aux tx\n", __func__);
				blocking_notifier_call_chain(&tx_aux_event_nh, WL_TX_EVENT_HALL_AWAY_FROM, NULL);
			}
			g_pre_wireless_status = wireless_hall_status;
		}
		/* report 0x100  if wirelessHall triggered */
		kbhall_status_change_notify(((uint32_t)value & 0x7) >> 2);
#endif
	}
	hall_value = value;
	double_hall_value[0] = value;
	if (hall_number == 2) {
		double_hall_value[1] = 2;
	} else {
		ext_hall_value[0] = SLIDE_HALL_TYPE;
		ext_hall_value[1] = adapt_hall_value(value);
	}
	/* return diff sensor type */
	report_sensor_event(TAG_HALL, double_hall_value, sizeof(double_hall_value));
	if (hall_sen_type == SLIDE_HALL_TYPE)
		report_sensor_event(TAG_EXT_HALL, ext_hall_value, sizeof(ext_hall_value));

	return 0;
}

int ap_color_report(int value[], int length)
{
	return report_sensor_event(TAG_COLOR, value, length);
}

bool ap_sensor_enable(int tag, bool enable)
{
	bool work_on_ap = false;

	if (tag < TAG_SENSOR_BEGIN || tag >= TAG_SENSOR_END)
		return false;

	work_on_ap = all_ap_sensor_operations[tag].work_on_ap;
	if (work_on_ap)
		if (all_ap_sensor_operations[tag].ops.enable)
			all_ap_sensor_operations[tag].ops.enable(enable);

	return work_on_ap;
}

bool ap_sensor_setdelay(int tag, int ms)
{
	bool work_on_ap = false;

	if (tag < TAG_SENSOR_BEGIN || tag >= TAG_SENSOR_END)
		return false;

	work_on_ap = all_ap_sensor_operations[tag].work_on_ap;
	if (work_on_ap)
		if (all_ap_sensor_operations[tag].ops.setdelay)
			all_ap_sensor_operations[tag].ops.setdelay(ms);
	return work_on_ap;
}

ssize_t xhub_route_write_batch(unsigned short port, char *buf,
	size_t count, int64_t timestamp)
{
	struct xhub_route_table *route_item = NULL;
	struct xhub_buffer_pos writer;
	char *buffer_begin = NULL;
	char *buffer_end = NULL;
	t_head header;
	unsigned long flags = 0;

	if (xhub_route_item(port, &route_item) != 0) {
		hwlog_err("xhub_route_item failed in %s port = %d!\n",
			__func__, (int)port);
		return 0;
	}
	header.timestamp = timestamp;

	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	writer = route_item->pwrite;

	if (writer.buffer_size < count + HEAD_SIZE) {
		spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
		return 0;
	}

	buffer_begin = route_item->phead.pos;
	buffer_end = route_item->phead.pos + route_item->phead.buffer_size;
	if (UINT_MAX - count < sizeof(int64_t)) {
		hwlog_err("%s :count is too large :%zd\n", __func__, count);
		spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
		return 0;
	}
	header.pkg_length = count + sizeof(int64_t);
	write_to_fifo(&writer, buffer_begin, buffer_end,
		header.effect_addr, HEAD_SIZE);
	write_to_fifo(&writer, buffer_begin, buffer_end, buf, count);

	route_item->pwrite.pos = writer.pos;
	route_item->pwrite.buffer_size -= (count + HEAD_SIZE);
	if ((UINT_MAX - route_item->pread.buffer_size) < (count + HEAD_SIZE)) {
		hwlog_err("%s:pread :count is too large :%zd\n",
				__func__, count);
		spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
		return 0;
	}
	route_item->pread.buffer_size += (count + HEAD_SIZE);
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
	atomic_set(&route_item->data_ready, 1);
	wake_up_interruptible(&route_item->read_wait);

	return (count + HEAD_SIZE);
}

ssize_t xhub_route_write(unsigned short port, char *buf, size_t count)
{
	struct xhub_route_table *route_item = NULL;
	struct xhub_buffer_pos writer;
	char *buffer_begin = NULL;
	char *buffer_end = NULL;
	t_head header;
	unsigned long flags = 0;

	if (xhub_route_item(port, &route_item) != 0) {
		hwlog_err("xhub_route_item failed in %s port = %d!\n",
			__func__, (int)port);
		return 0;
	}
	header.timestamp = get_timestamp();

	spin_lock_irqsave(&route_item->buffer_spin_lock, flags);
	writer = route_item->pwrite;

	if (writer.buffer_size < count + HEAD_SIZE) {
		spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
		return 0;
	}

	buffer_begin = route_item->phead.pos;
	buffer_end = route_item->phead.pos + route_item->phead.buffer_size;
	header.pkg_length = count + sizeof(int64_t);
	write_to_fifo(&writer, buffer_begin, buffer_end,
		header.effect_addr, HEAD_SIZE);
	write_to_fifo(&writer, buffer_begin, buffer_end, buf, count);

	route_item->pwrite.pos = writer.pos;
	route_item->pwrite.buffer_size -= (count + HEAD_SIZE);
	route_item->pread.buffer_size += (count + HEAD_SIZE);
	spin_unlock_irqrestore(&route_item->buffer_spin_lock, flags);
	atomic_set(&route_item->data_ready, 1);
	wake_up_interruptible(&route_item->read_wait);

	return (count + HEAD_SIZE);
}
EXPORT_SYMBOL_GPL(xhub_route_write);

static int xhub_mcu_send(const char *buf, unsigned int length)
{
	pkt_header_t *spkt = (pkt_header_t *)buf;

	hwlog_info("%s tag %d cmd %d tranid %d length %d\n", __func__,
		spkt->tag, spkt->cmd, spkt->tranid, spkt->length);
	ipc_adapter_send(buf, length);
	return 0;
}

static int unpack(const void *buf, unsigned int length)
{
	int ret;

	mutex_lock(&mutex_unpack);
	/* send head */
	ret = xhub_mcu_send((const char *)buf, length);
	mutex_unlock(&mutex_unpack);
	return ret;
}

static struct mcu_event_wait_list {
	spinlock_t slock;
	struct list_head head;
} mcu_event_wait_list;

void init_wait_node_add_list(struct mcu_event_waiter *waiter, t_match match,
			     void *out_data, int out_data_len, void *priv)
{
	unsigned long flags = 0;

	waiter->match = match;
	init_completion(&waiter->complete);
	waiter->out_data = out_data;
	waiter->out_data_len = out_data_len;
	waiter->priv = priv;

	spin_lock_irqsave(&mcu_event_wait_list.slock, flags);
	list_add(&waiter->entry, &mcu_event_wait_list.head);
	spin_unlock_irqrestore(&mcu_event_wait_list.slock, flags);
}

void list_del_mcu_event_waiter(struct mcu_event_waiter *self)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&mcu_event_wait_list.slock, flags);
	list_del(&self->entry);
	spin_unlock_irqrestore(&mcu_event_wait_list.slock, flags);
}

static void wake_up_mcu_event_waiter(const pkt_header_t *head)
{
	unsigned long flags = 0;
	struct mcu_event_waiter *pos = NULL;
	struct mcu_event_waiter *n = NULL;

	spin_lock_irqsave(&mcu_event_wait_list.slock, flags);
	list_for_each_entry_safe(pos, n, &mcu_event_wait_list.head, entry) {
		if (pos->match && pos->match(pos->priv, head)) {
			if (pos->out_data != NULL)
				memcpy(pos->out_data, head, pos->out_data_len);
			complete(&pos->complete);
			/*
			 * to support diffrent task wait for same event,
			 * here we don't break;
			 */
		}
	}
	spin_unlock_irqrestore(&mcu_event_wait_list.slock, flags);
}


static int xhub_mcu_write_cmd_nolock(const void *buf, unsigned int length)
{
	int ret = 0;
	pkt_header_t *pkt = (pkt_header_t *) buf;

	((pkt_header_t *) buf)->resp = RESP;
	if (!wait_for_mcu_resp_after_send(buf, unpack(buf, length), 2000)) {
		hwlog_err("wait for tag:%s:%d\tcmd:%d resp timeout in %s\n",
			obj_tag_str[pkt->tag], pkt->tag, pkt->cmd, __func__);
		ret = -1;
	}

	return ret;
}

char *obj_tag_str[] = {
	[TAG_ACCEL] = "TAG_ACCEL",
	[TAG_GYRO] = "TAG_GYRO",
	[TAG_MAG] = "TAG_MAG",
	[TAG_ALS] = "TAG_ALS",
	[TAG_PS] = "TAG_PS",
	[TAG_LINEAR_ACCEL] = "TAG_LINEAR_ACCEL",
	[TAG_GRAVITY] = "TAG_GRAVITY",
	[TAG_ORIENTATION] = "TAG_ORIENTATION",
	[TAG_ROTATION_VECTORS] = "TAG_ROTATION_VECTORS",
	[TAG_PRESSURE] = "TAG_PRESSURE",
	[TAG_HALL] = "TAG_HALL",
	[TAG_MAG_UNCALIBRATED] = "TAG_MAG_UNCALIBRATED",
	[TAG_GAME_RV] = "TAG_GAME_RV",
	[TAG_GYRO_UNCALIBRATED] = "TAG_GYRO_UNCALIBRATED",
	[TAG_SIGNIFICANT_MOTION] = "TAG_SIGNIFICANT_MOTION",
	[TAG_STEP_DETECTOR] = "TAG_STEP_DETECTOR",
	[TAG_STEP_COUNTER] = "TAG_STEP_COUNTER",
	[TAG_GEOMAGNETIC_RV] = "TAG_GEOMAGNETIC_RV",
	[TAG_HANDPRESS] = "TAG_HANDPRESS",
	[TAG_CAP_PROX] = "TAG_CAP_PROX",
	[TAG_FINGERSENSE] = "TAG_FINGERSENSE",
	[TAG_PHONECALL] = "TAG_PHONECALL",
	[TAG_CONNECTIVITY] = "TAG_CONNECTIVITY",
	[TAG_CA] = "TAG_CA",
	[TAG_OIS] = "TAG_OIS",
	[TAG_THP] = "TAG_THP",
	[TAG_TP] = "TAG_TP",
	[TAG_SPI] = "TAG_SPI",
	[TAG_I2C] = "TAG_I2C",
	[TAG_RGBLIGHT] = "TAG_RGBLIGHT",
	[TAG_BUTTONLIGHT] = "TAG_BUTTONLIGHT",
	[TAG_BACKLIGHT] = "TAG_BACKLIGHT",
	[TAG_VIBRATOR] = "TAG_VIBRATOR",
	[TAG_SYS] = "TAG_SYS",
	[TAG_LOG] = "TAG_LOG",
	[TAG_MOTION] = "TAG_MOTION",
	[TAG_LOG_BUFF] = "TAG_LOG_BUFF",
	[TAG_PDR] = "TAG_PDR",
	[TAG_AR] = "TAG_AR",
	[TAG_FP] = "TAG_FP",
	[TAG_KEY] = "TAG_KEY",
	[TAG_HINGE] = "TAG_HINGE",
	[TAG_FAULT] = "TAG_FAULT",
	[TAG_MAGN_BRACKET] = "TAG_MAGN_BRACKET",
	[TAG_RPC] = "TAG_RPC",
	[TAG_AGT] = "TAG_AGT",
	[TAG_COLOR] = "TAG_COLOR",
	[TAG_FP_UD] = "TAG_FP_UD",
	[TAG_ACCEL_UNCALIBRATED] = "TAG_ACCEL_UNCALIBRATED",
	[TAG_TOF] = "TAG_TOF",
	[TAG_DROP] = "TAG_DROP",
	[TAG_EXT_HALL] = "TAG_EXT_HALL",
	[TAG_ACC1] = "TAG_ACCEL1",
	[TAG_GYRO1] = "TAG_GYRO1",
	[TAG_ALS1] = "TAG_ALS1",
	[TAG_MAG1] = "TAG_MAG1",
	[TAG_ALS2] = "TAG_ALS2",
	[TAG_CAP_PROX1] = "TAG_CAP_PROX1",
	[TAG_POSTURE] = "TAG_POSTURE",
	[TAG_THERMOMETER] = "TAG_THERMOMETER",
	[TAG_KB] = "TAG_KB",
	[TAG_END] = "TAG_END",
};

static bool is_extend_step_counter_cmd(const pkt_header_t *pkt)
{
	bool ret = false;

	if (pkt->tag != TAG_STEP_COUNTER)
		return false;

	switch (pkt->cmd) {
	case CMD_CMN_OPEN_REQ:
	case CMD_CMN_CLOSE_REQ:
		/* could not judge which type step counter in open protocol */
		break;
	case CMD_CMN_CONFIG_REQ:
		ret =
		(TYPE_EXTEND ==
			((pkt_cmn_motion_req_t *) pkt)->app_config[1]);
		break;

	case CMD_CMN_INTERVAL_REQ:
		ret =
		(TYPE_EXTEND ==
			((pkt_cmn_interval_req_t *) pkt)->param.reserved[0]);
		break;

	default:
		break;
	}

	return ret;
}

static void update_sensor_info(const pkt_header_t *pkt)
{
	if (pkt->tag >= TAG_SENSOR_BEGIN && pkt->tag < TAG_SENSOR_END) {
		mutex_lock(&mutex_update);
		if (pkt->cmd == CMD_CMN_OPEN_REQ) {
			sensor_status.opened[pkt->tag] = 1;
		} else if (pkt->cmd == CMD_CMN_CLOSE_REQ) {
			sensor_status.opened[pkt->tag] = 0;
			sensor_status.status[pkt->tag] = 0;
			sensor_status.delay[pkt->tag] = 0;
		} else if (pkt->cmd == CMD_CMN_INTERVAL_REQ) {
			sensor_status.delay[pkt->tag] =
			((pkt_cmn_interval_req_t *)pkt)->param.period;
			sensor_status.status[pkt->tag] = 1;
			sensor_status.batch_cnt[pkt->tag] =
			((pkt_cmn_interval_req_t *)pkt)->param.batch_count;
		}
		mutex_unlock(&mutex_update);
	}
}


void xhub_update_info(const void *buf, int ret, bool is_in_recovery)
{
	update_current_app_status(((pkt_header_t *)buf)->tag,
		((pkt_header_t *)buf)->cmd);
	if (!is_extend_step_counter_cmd(((const pkt_header_t *)buf)))
		update_sensor_info(((const pkt_header_t *)buf));
	if (ret && (is_in_recovery == false))
		xhub_need_recovery(SH_FAULT_IPC_TX_TIMEOUT);
}

int xhub_mcu_write_cmd(const void *buf, unsigned int length)
{
	bool is_in_recovery = false;
	int ret = 0;

	if (length > MAX_PKT_LENGTH) {
		hwlog_err("length = %d is too large in %s\n",
			(int)length, __func__);
		return -EINVAL;
	}
	if (((pkt_header_t *)buf)->tag < TAG_SENSOR_BEGIN ||
		((pkt_header_t *) buf)->tag >= TAG_END) {
		hwlog_err("tag = %d is wrong in %s\n",
			((pkt_header_t *)buf)->tag, __func__);
		return -EINVAL;
	}
	mutex_lock(&mutex_write_cmd);
	if (g_xhub_state == XHUB_ST_RECOVERY) { /* true - only update */
		if (((pkt_header_t *)buf)->tag != TAG_SYS &&
				((pkt_header_t *)buf)->cmd != CMD_SYS_STATUSREADY_REQ)
			is_in_recovery = true;
	} else if (g_xhub_state == XHUB_ST_REPEAT) {
		/* XHUB_ST_REPEAT...BLOCK IN HERE, WAIT FOR REPEAT COMPLETE */
		hwlog_err("wait for iom3 recovery complete. tag %d cmd %d.\n",
			((pkt_header_t *)buf)->tag, ((pkt_header_t *)buf)->cmd);
		mutex_unlock(&mutex_write_cmd);
		wait_event(xhub_rec_waitq, (g_xhub_state == XHUB_ST_NORMAL));
		hwlog_err("wakeup for xhub recovery complete\n");
		mutex_lock(&mutex_write_cmd);
	}

	if (is_in_recovery == true) {
		mutex_unlock(&mutex_write_cmd);
		goto update_info;
	}
	((pkt_header_t *) buf)->tranid = tranid++;
	update_shmem_tranid((pkt_header_t *)buf);
	ret = unpack(buf, length);
	mutex_unlock(&mutex_write_cmd);

update_info:
	xhub_update_info(buf, ret, is_in_recovery);
	return 0;
}

/* use lock for all command to avoid conflict */
static int xhub_mcu_write_cmd_adapter(const void *buf, unsigned int length,
	struct read_info *rd)
{
	int ret = 0;
	unsigned long flags = 0;
	int retry_count = 2;

	mutex_lock(&mutex_write_adapter);
	if (!rd) {
		ret = xhub_mcu_write_cmd(buf, length);
	} else {
		mutex_lock(&type_record.lock_mutex);
		spin_lock_irqsave(&type_record.lock_spin, flags);
		type_record.pkt_info = ((pkt_header_t *) buf);
		type_record.rd = rd;
		spin_unlock_irqrestore(&type_record.lock_spin, flags);
		while (retry_count--) { /* send retry 3 times */
			/* send data to mcu */
			reinit_completion(&type_record.resp_complete);
			if (xhub_mcu_write_cmd(buf, length)) {
				hwlog_err("send cmd to mcu failed in %s, retry %d\n",
					__func__, retry_count);
				ret = -1;
			} else if (!wait_for_completion_timeout(
				&type_record.resp_complete,
				msecs_to_jiffies(xhub_timeout))) {
				hwlog_err("wait for response timeout in %s, retry %d. tag %d cmd %d. g_xhub_state %d.\n",
					__func__, retry_count,	((pkt_header_t *) buf)->tag,
					((pkt_header_t *) buf)->cmd, g_xhub_state);
				if (retry_count == 0)
					xhub_need_recovery(SH_FAULT_IPC_RX_TIMEOUT);
				ret = -1;
			} else {
				ret = 0; /* send success & response success */
				break;
			}
		}

		/* clear info */
		spin_lock_irqsave(&type_record.lock_spin, flags);
		type_record.pkt_info = NULL;
		type_record.rd = NULL;
		spin_unlock_irqrestore(&type_record.lock_spin, flags);
		mutex_unlock(&type_record.lock_mutex);
	}

	mutex_unlock(&mutex_write_adapter);

	return ret;
}

static int xhub_mcu_writeback_cmd_nolock(const void *buf,
	unsigned int length, struct read_info *rd)
{
	int ret;
	pkt_header_t *pkt = (pkt_header_t *)buf;
	pkt_subcmd_para_t resp_pkt;
	int retry_count = 2; /* for 3 times resend */

	memset(&resp_pkt, 0, sizeof(resp_pkt));
	while (retry_count--) { /* send retry 3 times */
		if (!wait_for_mcu_resp_data_after_send(buf,
			unpack(buf, length), 2000,
			&resp_pkt, sizeof(resp_pkt))) {
			hwlog_err("wait for tag:%s:%d\tcmd:%d resp timeout in %s\n",
				obj_tag_str[pkt->tag],
				pkt->tag, pkt->cmd, __func__);
			if (retry_count == 0)
				xhub_need_recovery(SH_FAULT_IPC_RX_TIMEOUT);
			ret = -1;
		} else {
			/* data_length means data lenght below */
			rd->data_length = resp_pkt.hd.length -
				sizeof(resp_pkt.hd.errno) -
				sizeof(resp_pkt.subcmd);
			/* only support SUBCMD length para */
			if (rd->data_length <= SUBCMD_LEN) {
				/* fill errno to app */
				rd->errno = resp_pkt.hd.errno;
				/* fill resp data to app */
				memcpy(rd->data, resp_pkt.data,
					rd->data_length);
			} else { /* resp data too large */
				rd->errno = -EINVAL;
				rd->data_length = 0;
				hwlog_err("data too large from mcu in %s\n",
					__func__);
			}
			ret = 0; /* send & resp success */
			break;
		}
	}

	return ret;
}

int send_app_config_cmd_with_resp(int tag, void *app_config, bool use_lock)
{
	pkt_cmn_motion_req_t i_pkt;
	struct read_info rd;

	memset(&rd, 0, sizeof(rd));
	i_pkt.hd.tag = tag;
	i_pkt.hd.cmd = CMD_CMN_CONFIG_REQ;
	i_pkt.hd.resp = RESP;
	i_pkt.hd.length = sizeof(i_pkt.app_config);
	memcpy(i_pkt.app_config, app_config, sizeof(i_pkt.app_config));

	if (use_lock)
		return xhub_mcu_write_cmd_adapter(&i_pkt,
			sizeof(i_pkt), &rd);
	else
		return xhub_mcu_write_cmd_nolock(&i_pkt, sizeof(i_pkt));
}

int send_app_config_cmd(int tag, void *app_config, bool use_lock)
{
	pkt_cmn_motion_req_t i_pkt;

	i_pkt.hd.tag = tag;
	i_pkt.hd.cmd = CMD_CMN_CONFIG_REQ;
	i_pkt.hd.resp = NO_RESP;
	i_pkt.hd.length = sizeof(i_pkt.app_config);
	memcpy(i_pkt.app_config, app_config, sizeof(i_pkt.app_config));

	if (use_lock)
		return xhub_mcu_write_cmd_adapter(&i_pkt,
			sizeof(i_pkt), NULL);
	else
		return xhub_mcu_write_cmd_nolock(&i_pkt, sizeof(i_pkt));
}

static int xhub_sensor_enable_internal(int tag, bool enable, bool is_lock)
{
	if (tag < TAG_BEGIN || tag >= TAG_END) {
		hwlog_err("NULL pointer param in %s or tag %d is error.\n",
			__func__, tag);
		return -EINVAL;
	}

	if (ap_sensor_enable(tag, enable))
		return 0;

	if (enable) {
		pkt_header_t pkt = (pkt_header_t) {
			.tag = tag,
			.cmd = CMD_CMN_OPEN_REQ,
			.resp = NO_RESP,
			.length = 0
		};
		if (tag == TAG_MAG || tag == TAG_ORIENTATION)
			mag_opend++;
		hwlog_info("open sensor %s (tag:%d)!\n",
			obj_tag_str[tag] ? obj_tag_str[tag] : "TAG_UNKNOWN",
			tag);
		if (is_lock)
			return xhub_mcu_write_cmd_adapter(&pkt,
				sizeof(pkt), NULL);
		else
			return xhub_mcu_write_cmd_nolock(&pkt, sizeof(pkt));
	} else {
		pkt_header_t pkt = (pkt_header_t) {
			.tag = tag,
			.cmd = CMD_CMN_CLOSE_REQ,
			.resp = NO_RESP,
			.length = 0
		};
		if (tag == TAG_MAG || tag == TAG_ORIENTATION) {
			if (mag_opend > 0)
				mag_opend--; // 0 for mag is not opened
		}
		hwlog_info("close sensor %s (tag:%d)!\n",
			obj_tag_str[tag] ? obj_tag_str[tag] : "TAG_UNKNOWN",
			tag);
		if (is_lock)
			return xhub_mcu_write_cmd_adapter(&pkt,
				sizeof(pkt), NULL);
		else
			return xhub_mcu_write_cmd_nolock(&pkt, sizeof(pkt));
	}
}

static int xhub_sensor_setdelay_internal(int tag, interval_param_t *param,
	bool use_lock)
{
	pkt_cmn_interval_req_t pkt;

	if (!param || (tag < TAG_BEGIN || tag >= TAG_END)) {
		hwlog_err("NULL pointer param in %s or tag %d is error.\n",
			__func__, tag);
		return -EINVAL;
	}

	if (ap_sensor_setdelay(tag, param->period))
		return 0;

	memset(&pkt, 0, sizeof(pkt));
	pkt.hd.tag = tag;
	pkt.hd.cmd = CMD_CMN_INTERVAL_REQ;
	pkt.hd.resp = NO_RESP;
	pkt.hd.length = sizeof(pkt.param);
	memcpy(&pkt.param, param, sizeof(pkt.param));
	hwlog_info("set sensor %s (tag:%d) delay %d ms!\n",
		obj_tag_str[tag] ? obj_tag_str[tag] : "TAG_UNKNOWN",
		tag, param->period);
	if (use_lock)
		return xhub_mcu_write_cmd_adapter(&pkt, sizeof(pkt), NULL);
	else
		return xhub_mcu_write_cmd_nolock(&pkt, sizeof(pkt));
}

int xhub_sensor_enable_nolock(int tag, bool enable)
{
	return xhub_sensor_enable_internal(tag, enable, false);
}

int xhub_sensor_setdelay_nolock(int tag, interval_param_t *interval_param)
{
	return xhub_sensor_setdelay_internal(tag, interval_param, false);
}

int xhub_sensor_enable(int tag, bool enable)
{
	return xhub_sensor_enable_internal(tag, enable, true);
}
int xhub_sensor_enable_stepcounter(bool enable,
	type_step_counter_t steptype)
{
	uint8_t app_config[16] = { 0 };

	hwlog_info("TAG_STEP_COUNTER %d, enabe:%d in %s!\n",
		TAG_STEP_COUNTER, enable, __func__);
	if (!enable) {
		app_config[0] = enable;
		app_config[1] = steptype;
		send_app_config_cmd(TAG_STEP_COUNTER, app_config, true);
	}
	if (really_do_enable_disable(&step_ref_cnt, enable, steptype)) {
		return xhub_sensor_enable_internal(TAG_STEP_COUNTER,
			enable, true);
	}
	return 0;
}

int xhub_sensor_setdelay(int tag, interval_param_t *interval_param)
{
	return xhub_sensor_setdelay_internal(tag, interval_param, true);
}

static int get_section_offset(uint8_t section_id)
{
	int section_offset;

	if (section_id < LITTLE_SECTION_N) {
		section_offset = section_id * LITTLE_SECTION_SIZE;
	} else if (section_id < LITTLE_SECTION_N + MIDDLE_SECTION_N) {
		section_offset = LITTLE_SECTION_N * LITTLE_SECTION_SIZE +
			(section_id - LITTLE_SECTION_N) * MIDDLE_SECTION_SIZE;
	} else {
		section_offset = LITTLE_SECTION_N * LITTLE_SECTION_SIZE + MIDDLE_SECTION_N * MIDDLE_SECTION_SIZE +
			(section_id - LITTLE_SECTION_N - MIDDLE_SECTION_N) * LARGE_SECTION_SIZE;
	}
	return section_offset;
}
/* only called by shmem send, we assume shmem length is legal */
static uint8_t get_section_id(int length)
{
	uint8_t section_id;
	uint8_t loop_start;
	uint8_t loop_end;
	uint8_t i;
	bool allocated = false;
	int section_offset;

	if (!atomic_dec_and_test(&g_section_atomic)) {
		hwlog_err("section atomic get fail\n");
		atomic_inc(&g_section_atomic);
		return SECTION_ID_ERR;
	}
	if (length <= LITTLE_SECTION_SIZE - SECTION_CTL_CMD_LEN) {
		loop_start = 0;
		loop_end = LITTLE_SECTION_N;
	} else if (length <= MIDDLE_SECTION_SIZE - SECTION_CTL_CMD_LEN) {
		loop_start = LITTLE_SECTION_N;
		loop_end = LITTLE_SECTION_N + MIDDLE_SECTION_N;
	} else {
		loop_start = LITTLE_SECTION_N + MIDDLE_SECTION_N;
		loop_end = TOTAL_SECTION_N;
	}
	for (i = loop_start; i < loop_end; i++) {
		section_offset = get_section_offset(i);
		if (*((u32 *)(uintptr_t)(g_shmem_send_start + section_offset)) == 0) {
			/* set to busy mode */
			*((u32 *)(uintptr_t)(g_shmem_send_start + section_offset)) = 1;
			section_id = i;
			allocated = true;
			break;
		}
	}

	atomic_inc(&g_section_atomic);
	if (allocated) {
		return section_id;
	}
	return SECTION_ID_ERR;
}

/* only called by shmem recv */
static void put_section_id(uint8_t section_id)
{
	int section_offset;

	section_offset = get_section_offset(section_id);
	if (section_offset > SHMEM_SIZE / 2)
		return;
	/* set to idle mode */
	*((u32 *)(uintptr_t)(g_shmem_recv_start + section_offset)) = 0;
}

/* direction 0 for send, direction 1 for recv */
static u64 get_shmem_section_addr(uint8_t section_id, int direction)
{
	int section_offset;

	section_offset = get_section_offset(section_id);

	return (direction == SHMEM_SEND) ?
		(g_shmem_send_start + section_offset + SECTION_CTL_CMD_LEN) :
		(g_shmem_recv_start + section_offset + SECTION_CTL_CMD_LEN);
}

static uint8_t shmem_trans_prepare(const struct write_info *wr, struct read_info *rd)
{
	uint8_t section_id;
	int try_times_ns = 5;
	int try_times_ms = 200;
	u64 shmem_buff_addr;
	pkt_header_t pkt_hd;

	pkt_hd.tag = wr->tag;
	pkt_hd.cmd = wr->cmd;
	pkt_hd.resp = ((rd != NULL) ? (RESP) : (NO_RESP));
	pkt_hd.length = wr->wr_len;
	pkt_hd.tranid = 0;
	section_id = get_section_id(wr->wr_len + sizeof(pkt_header_t));
	while (section_id == SECTION_ID_ERR) {
		if (try_times_ns > 0) {
			udelay(100);
			section_id = get_section_id(wr->wr_len + sizeof(pkt_header_t));
			--try_times_ns;
		} else if (try_times_ms > 0) {
			mdelay(1);
			section_id = get_section_id(wr->wr_len + sizeof(pkt_header_t));
			--try_times_ms;
		} else {
			break;
		}
	}
	if (section_id == SECTION_ID_ERR && try_times_ns == 0 && try_times_ms == 0) {
		hwlog_err("section_id is too busy, please try later\n");
		return SECTION_ID_ERR;
	}
	pkt_hd.section_id = section_id;
	hwlog_info("trans with shmem, section_id is %d\n", section_id);

	shmem_buff_addr = get_shmem_section_addr(section_id, SHMEM_SEND);
	/*
	 * head.length should less then LARGE_SECTION_SIZE
	 * shmem will not support pack
	 */
	memcpy((void *)(uintptr_t)shmem_buff_addr, &pkt_hd, sizeof(pkt_header_t));
	memcpy((void *)(uintptr_t)(shmem_buff_addr + sizeof(pkt_header_t)), wr->wr_buf, wr->wr_len);

	return section_id;
}

static bool is_mass_ipc(const pkt_header_t *pkt_hd, int direction)
{
	uint16_t max_len;

	max_len = (direction == SHMEM_SEND) ? MAX_PKT_SEND_LENGTH : MAX_PKT_RECV_LENGTH;
	return (sizeof(pkt_header_t) + pkt_hd->length > max_len) ? true : false;
}

static void update_shmem_tranid(pkt_header_t *head)
{
	u64 shmem_buff_addr;

	if (is_mass_ipc(head, SHMEM_SEND)) {
		shmem_buff_addr = get_shmem_section_addr(head->section_id, SHMEM_SEND);
		((pkt_header_t *)(uintptr_t)shmem_buff_addr)->tranid = head->tranid;
	}
}

int write_customize_cmd(const struct write_info *wr, struct read_info *rd,
	bool is_lock)
{
	char buf[MAX_PKT_LENGTH];
	bool use_shmem = false;
	uint8_t section_id = SECTION_ID_ERR;
	unsigned int msg_len;

	if (!wr) {
		hwlog_err("NULL pointer in %s\n", __func__);
		return -EINVAL;
	}
	if (wr->tag < TAG_BEGIN || wr->tag >= TAG_END) {
		hwlog_err("tag = %d error in %s\n", wr->tag, __func__);
		return -EINVAL;
	}
	if (wr->wr_len < 0 ||
		wr->wr_len + sizeof(pkt_header_t) > LARGE_SECTION_SIZE) {
		hwlog_err("-----------> wr_len = %d is too large in %s\n",
			wr->wr_len, __func__);
		return -EINVAL;
	}
	memset(&buf, 0, sizeof(buf));
	msg_len = wr->wr_len + sizeof(pkt_header_t);
	if (msg_len > MAX_PKT_LENGTH) {
		use_shmem = true;
		// copy data to shmem
		section_id = shmem_trans_prepare(wr, rd);
		if (section_id == SECTION_ID_ERR) {
			hwlog_err("shmem is busy, try later. in %s\n", __func__);
			return -EBUSY;
		}
		// only send msg head
		msg_len = sizeof(pkt_header_t);
	}
	((pkt_header_t *) buf)->tag = wr->tag;
	((pkt_header_t *) buf)->cmd = wr->cmd;
	((pkt_header_t *) buf)->resp = ((rd != NULL) ? (RESP) : (NO_RESP));
	((pkt_header_t *) buf)->length = wr->wr_len;
	((pkt_header_t *) buf)->section_id = section_id;
	if (wr->wr_buf != NULL && !use_shmem)
		memcpy(buf + sizeof(pkt_header_t), wr->wr_buf, wr->wr_len);
	if ((wr->tag == TAG_SHAREMEM) &&
		(g_xhub_state == XHUB_ST_REPEAT ||
			g_xhub_state == XHUB_ST_RECOVERY ||
			iom3_power_state == ST_SLEEP))
		return xhub_mcu_write_cmd_nolock(buf, msg_len);
	if (is_lock)
		return xhub_mcu_write_cmd_adapter(buf, msg_len, rd);
	else if (!rd || g_xhub_state != XHUB_ST_NORMAL)
		return xhub_mcu_write_cmd_nolock(buf, msg_len);
	else
		return xhub_mcu_writeback_cmd_nolock(buf, msg_len, rd);
}

static bool is_als_ud_data_report(const pkt_header_t *head)
{
	return ((head->tag == TAG_ALS) &&
		(head->cmd == CMD_ALS_RUN_STOP_PARA_REQ));
}

static bool is_als_ud_bl_report(const pkt_header_t *head)
{
	return ((head->tag == TAG_ALS) &&
		(head->cmd == CMD_ALS_BL_PARA_REQ));
}

static bool is_als_ud_coef_report(const pkt_header_t *head)
{
	return ((head->tag == TAG_ALS) &&
		(head->cmd == CMD_ALS_COEF_BLOCK_REQ));
}

static bool is_als_ud_type_report(const pkt_header_t *head)
{
	return (is_als_ud_data_report(head) ||
		is_als_ud_bl_report(head) ||
		is_als_ud_coef_report(head));
}

static bool is_motion_data_report(const pkt_header_t *head)
{
	/* all sensors report data with command CMD_PRIVATE */
	return (head->tag == TAG_MOTION) && (head->cmd == CMD_DATA_REQ);
}

static bool is_sensor_data_report(const pkt_header_t *head)
{
	/* all sensors report data with command CMD_PRIVATE */
	return (head->tag >= TAG_SENSOR_BEGIN && head->tag < TAG_SENSOR_END) &&
	    (head->cmd == CMD_DATA_REQ);
}

static bool is_fingersense_zaxis_data_report(const pkt_header_t *head)
{
	return (head->tag == TAG_FINGERSENSE) && (head->cmd == CMD_DATA_REQ);
}

static bool cmd_match(int req, int resp)
{
	return (req + 1) == resp;
}

static bool is_requirement_resp(const pkt_header_t *head)
{
	return ((head->cmd & 1) == 0); /* even cmds are resp cmd */
}

static int report_resp_data(const pkt_header_t *msg_head)
{
	int ret = 0;
	unsigned long flags = 0;
	pkt_subcmd_resp_t *head = (pkt_subcmd_resp_t *)msg_head;

	hwlog_info("%s in tag=0x%x, cmd=0x%x\n",
			__func__, head->hd.tag, head->hd.cmd);
	spin_lock_irqsave(&type_record.lock_spin, flags);
	if (type_record.rd != NULL &&
		type_record.pkt_info != NULL &&
		(cmd_match(type_record.pkt_info->cmd, head->hd.cmd)) &&
		/* rcv resp from mcu */
		(type_record.pkt_info->tranid == head->hd.tranid)) {
		/* data length ok */
		if (head->hd.length <=
			(MAX_PKT_LENGTH + sizeof(head->hd.errno) +
				sizeof(head->subcmd))) {
			/* fill errno to app */
			type_record.rd->errno = head->hd.errno;
			/* data_length means data lenght below */
			type_record.rd->data_length =
				(head->hd.length - sizeof(head->hd.errno) -
					sizeof(head->subcmd));
			/* fill resp data to app */
			memcpy(type_record.rd->data,
				(char *)head + sizeof(pkt_subcmd_resp_t),
				type_record.rd->data_length);
		} else { /* resp data too large */
			type_record.rd->errno = -EINVAL;
			type_record.rd->data_length = 0;
			hwlog_err("data too large from mcu in %s\n", __func__);
		}
		hwlog_info("%s release resp_complete\n", __func__);
		complete(&type_record.resp_complete);
	}
	spin_unlock_irqrestore(&type_record.lock_spin, flags);

	return ret;
}

void init_mcu_notifier_list(void)
{
	INIT_LIST_HEAD(&mcu_notifier.head);
	spin_lock_init(&mcu_notifier.lock);
	mcu_notifier.mcu_notifier_wq =
		create_freezable_workqueue("mcu_event_notifier");
}

static void init_mcu_event_wait_list(void)
{
	INIT_LIST_HEAD(&mcu_event_wait_list.head);
	spin_lock_init(&mcu_event_wait_list.slock);
}

static void mcu_notifier_handler(struct work_struct *work)
{
	/* find data according work */
	struct mcu_notifier_work *p =
		container_of(work, struct mcu_notifier_work, worker);
	/* search mcu_notifier, call all call_backs */
	struct mcu_notifier_node *pos = NULL;
	struct mcu_notifier_node *n = NULL;

	list_for_each_entry_safe(pos, n, &mcu_notifier.head, entry) {
		if ((pos->tag == ((const pkt_header_t *)p->data)->tag) &&
			(pos->cmd == ((const pkt_header_t *)p->data)->cmd)) {
			if (pos->notify)
				pos->notify((const pkt_header_t *)p->data);
		}
	}

	kfree(p->data);
	kfree(p);
}

static void mcu_notifier_queue_work(int handler_id, const pkt_header_t *head,
	void (*fn)(struct work_struct *work))
{
	struct mcu_notifier_work *notifier_work;
	u64 shmem_buff_addr;

	notifier_work = kmalloc(sizeof(struct mcu_notifier_work), GFP_ATOMIC);
	if (!notifier_work)
		return;
	memset(notifier_work, 0, sizeof(struct mcu_notifier_work));
	notifier_work->data =
		kmalloc(head->length + sizeof(pkt_header_t), GFP_ATOMIC);
	if (!(notifier_work->data)) {
		kfree(notifier_work);
		return;
	}
	notifier_work->handler_id = handler_id;
	memset(notifier_work->data, 0, head->length + sizeof(pkt_header_t));
	if (is_mass_ipc(head, SHMEM_RECV)) {
		shmem_buff_addr = get_shmem_section_addr(head->section_id, SHMEM_RECV);
		memcpy(notifier_work->data, (void *)(uintptr_t)shmem_buff_addr, sizeof(pkt_header_t) + head->length);
		put_section_id(head->section_id);
	} else {
		memcpy(notifier_work->data, head, sizeof(pkt_header_t) + head->length);
	}
	INIT_WORK(&notifier_work->worker, fn);
	queue_work(mcu_notifier.mcu_notifier_wq, &notifier_work->worker);
}

static bool is_mcu_notifier(const pkt_header_t *head)
{
	struct mcu_notifier_node *pos = NULL;
	struct mcu_notifier_node *n = NULL;
	unsigned long flags = 0;

	spin_lock_irqsave(&mcu_notifier.lock, flags);
	list_for_each_entry_safe(pos, n, &mcu_notifier.head, entry) {
		if ((pos->tag == head->tag) && (pos->cmd == head->cmd)) {
			spin_unlock_irqrestore(&mcu_notifier.lock, flags);
			return true;
		}
	}
	spin_unlock_irqrestore(&mcu_notifier.lock, flags);
	return false;
}

int register_mcu_event_notifier(int tag, int cmd,
	int (*notify)(const pkt_header_t *head))
{
	struct mcu_notifier_node *pnode = NULL;
	struct mcu_notifier_node *n = NULL;
	int ret = 0;
	unsigned long flags = 0;

	if ((!(tag >= TAG_BEGIN && tag < TAG_END)) || !notify)
		return -EINVAL;

	spin_lock_irqsave(&mcu_notifier.lock, flags);
	/* avoid regist more than once */
	list_for_each_entry_safe(pnode, n, &mcu_notifier.head, entry) {
		if ((tag == pnode->tag) && (cmd == pnode->cmd) &&
			(notify == pnode->notify)) {
			hwlog_warn("tag = %d, cmd = %d, notify = %pK has already registed in %s\n!",
				tag, cmd, notify, __func__);
			goto out; /* return when already registed */
		}
	}

	/* make mcu_notifier_node */
	pnode = kmalloc(sizeof(struct mcu_notifier_node), GFP_ATOMIC);
	if (!pnode) {
		ret = -ENOMEM;
		goto out;
	}
	memset(pnode, 0, sizeof(struct mcu_notifier_node));
	pnode->tag = tag;
	pnode->cmd = cmd;
	pnode->notify = notify;

	/* add to list */
	list_add(&pnode->entry, &mcu_notifier.head);
out:
	spin_unlock_irqrestore(&mcu_notifier.lock, flags);
	return ret;
}

int unregister_mcu_event_notifier(int tag, int cmd,
	int (*notify)(const pkt_header_t *head))
{
	struct mcu_notifier_node *pos = NULL;
	struct mcu_notifier_node *n = NULL;
	unsigned long flags = 0;

	if ((!(tag >= TAG_BEGIN && tag < TAG_END)) || !notify)
		return -EINVAL;

	spin_lock_irqsave(&mcu_notifier.lock, flags);
	list_for_each_entry_safe(pos, n, &mcu_notifier.head, entry) {
		if ((tag == pos->tag) && (cmd == pos->cmd) &&
			(notify == pos->notify)) {
			list_del(&pos->entry);
			kfree(pos);
			break;
		}
	}
	spin_unlock_irqrestore(&mcu_notifier.lock, flags);
	return 0;
}

static void step_counter_data_process(pkt_step_counter_data_req_t *head)
{
	int standard_data_len = sizeof(head->step_count);

	valid_step_count = recovery_step_count + head->step_count;
	head->total_step_count = head->step_count = valid_step_count;
	valid_floor_count = recovery_floor_count + head->total_floor_ascend;
	head->total_floor_ascend = valid_floor_count;

	/* extend step counter data structure changed */
	if ((head->record_count > 0) &&
		(head->record_count != EXT_PEDO_VERSION_2)) {
		/* skip offset of begin_time */
		int extend_effect_len = head->hd.length + sizeof(pkt_header_t) -
			offset(pkt_step_counter_data_req_t, begin_time);
		/* reserve 1 byte for motion type */
		char motion_data[extend_effect_len + 1];

		/* add motion type */
		motion_data[0] = MOTIONHUB_TYPE_HW_STEP_COUNTER;
		/*
		 * the offset rely on sizeof enum motion_type_t of mcu,
		 * now it is 1,
		 * we suggest motion_type_t use uint8_t,
		 * because sizeof(enum) may diffrernt between AP and mcu;
		 */
		memcpy(motion_data + 1, &head->begin_time, extend_effect_len);
		/* report extend step counter date to motion HAL */
		xhub_route_write(ROUTE_MOTION_PORT, motion_data,
			extend_effect_len + 1);
	}

	hwlog_info("convert to standard step counter data to sensor event buffer\n");
	/*
	 * avoid report extend data to sensor HAL,
	 * convert to standard step counter data,
	 * just report member step_count to sensor HAL
	 */
	head->hd.length = standard_data_len;
}

void save_step_count(void)
{
	/* save step count when iom3 recovery */
	recovery_step_count = valid_step_count;
	recovery_floor_count = valid_floor_count;
}

static bool is_dmd_log_data_report(const pkt_header_t *head)
{
	/* all sensors report data with command CMD_PRIVATE */
	pkt_dmd_log_report_req_t *buf = (pkt_dmd_log_report_req_t *)head;

	return ((head->tag == TAG_LOG) &&
		(head->cmd == CMD_LOG_REPORT_REQ) &&
		(buf->level == LOG_LEVEL_FATAL));
}

static bool is_additional_info_report(const pkt_header_t *head)
{
	return (head->cmd == CMD_CMN_CONFIG_REQ) &&
		(((pkt_subcmd_req_t *)head)->subcmd == SUB_CMD_ADDITIONAL_INFO);
}

static int sensor_need_reset_power(pkt_dmd_log_report_req_t *pkt)
{
	struct timespec ts;
	int ret = 0;

	if (!need_reset_io_power)
		goto OUT;

	get_monotonic_boottime(&ts);
/*
 *	if ((pkt->dmd_id == DSM_SHB_ERR_MCU_ALS) &&
 *			(pkt->dmd_case == DMD_CASE_ALS_NEED_RESET_POWER)) {
 */
	if (pkt->dmd_case == DMD_CASE_ALS_NEED_RESET_POWER) {
		pkt->hd.tag = TAG_ALS;
		if ((ts.tv_sec - als_last_reset_period) >
			RESET_REFRESH_PERIOD) {
			als_last_reset_period = ts.tv_sec;
			als_retry_cnt = ALS_RESET_COUNT;
		}

		if (als_retry_cnt) {
			als_retry_cnt--;
			ret = 1;
		} else {
			hwlog_err("als abnormal exceed reset limit\n");
		}
	}
/*
 *	if ((pkt->dmd_id == DSM_SHB_ERR_MCU_PS) &&
 *			(pkt->dmd_case == DMD_CASE_PS_NEED_RESET_POWER)) {
 */
	if (pkt->dmd_case == DMD_CASE_PS_NEED_RESET_POWER) {
		pkt->hd.tag = TAG_PS;
		if ((ts.tv_sec - ps_last_reset_period) > RESET_REFRESH_PERIOD) {
			ps_last_reset_period = ts.tv_sec;
			ps_retry_cnt = PS_RESET_COUNT;
		}

		if (ps_retry_cnt) {
			ps_retry_cnt--;
			ret = 1;
		} else {
			hwlog_err("ps abnormal exceed reset limit\n");
		}
	}
/*
 *	if ((pkt->dmd_case == DMD_CASE_TP_NEED_RESET_POWER) &&
 *		((pkt->dmd_id == DMDE_SHB_THP_TASK_TIMEOUT) ||
 *		(pkt->dmd_id == DMDE_SHB_THP_STATE_FAULT) ||
 *		(pkt->dmd_id == DMDE_SHB_THP_AFE_START) ||
 *		(pkt->dmd_id == DMDE_SHB_THP_AFE_DRIVER_NULL) ||
 *		(pkt->dmd_id == DMDW_SHB_THP_AFE_RESTART))) {
 */
	if (pkt->dmd_case == DMD_CASE_TP_NEED_RESET_POWER) {
		pkt->hd.tag = TAG_THP;
		if ((ts.tv_sec - tp_last_reset_period) > RESET_REFRESH_PERIOD) {
			tp_last_reset_period = ts.tv_sec;
			tp_retry_cnt = TP_RESET_COUNT;
		}
		if (tp_retry_cnt) {
			tp_retry_cnt--;
			ret = 1; /* thp need reset */
		} else {
			hwlog_err("tp abnormal exceed reset limit\n");
		}
	}

OUT:
	return ret;
}

#ifdef CONFIG_HUAWEI_DSM
static void update_client_info(uint8_t dmd_case)
{
	switch (dmd_case) {
	case 0x68:
	case 0x69:
		dsm_sensorhub.ic_name = "AG_BOSCH_INV";
		break;
	case 0x6A:
	case 0x6B:
		dsm_sensorhub.ic_name = "AG_LSM6DS";
		break;
	case 0x0C:
		dsm_sensorhub.ic_name = "MAG_AKM";
		break;
	case 0x0D:
		dsm_sensorhub.ic_name = "MAG_AKM09911";
		break;
	case 0x2E:
		dsm_sensorhub.ic_name = "MAG_YAS537";
		break;
	case 0x38:
	case 0x39:
		dsm_sensorhub.ic_name = "ALS_BH1745";
		break;
	case 0x52:
		dsm_sensorhub.ic_name = "ALS_APDS9251";
		break;
	case 0x53:
		dsm_sensorhub.ic_name = "PS_APDS9110";
		break;
	case 0x1E:
		dsm_sensorhub.ic_name = "PS_PA224";
		break;
	case 0x5C:
		dsm_sensorhub.ic_name = "AIR_LPS22BH";
		break;
	case 0x5D:
		dsm_sensorhub.ic_name = "AIR_LPS_BM";
		break;
	case 0x76:
	case 0x77:
		dsm_sensorhub.ic_name = "AIR_BMP380";
		break;
	case 0x28:
		dsm_sensorhub.ic_name = "SAR_SX9323";
		break;
	case 0x2C:
		dsm_sensorhub.ic_name = "SAR_ADUX1050";
		break;
	case 0x49:
		dsm_sensorhub.ic_name = "TP_FTM4CD56D_ALS_TMD3702";
		break;
	case 0x41:
		dsm_sensorhub.ic_name = "TOF_TFM8701";
		break;
	case 0x29:
		dsm_sensorhub.ic_name = "TOF_GP2AP02VT00F";
		break;
	default:
		hwlog_err("%s uncorrect dmd case %x.\n",
			__func__, dmd_case);
		return;
	}
	hwlog_info("%s ic name is %s.\n", __func__, dsm_sensorhub.ic_name);
	if (dsm_update_client_vendor_info(&dsm_sensorhub))
		hwlog_err("dsm_update_client_vendor_info failed\n");
}
#endif

static void iom7_dmd_log_handle(struct work_struct *work)
{
	struct iom7_log_work *iom7_log;
	pkt_dmd_log_report_req_t *pkt;

	iom7_log = container_of(work, struct iom7_log_work, log_work.work);
	pkt = (pkt_dmd_log_report_req_t *)iom7_log->log_p;

	hwlog_info("%s, dmd id is %d.\n", __func__, pkt->dmd_id);
#ifdef CONFIG_HUAWEI_DSM
	if (!dsm_client_ocuppy(shb_dclient)) {
		if (pkt->dmd_id == DSM_SHB_ERR_MCU_I2C_ERR ||
			pkt->dmd_id == DSM_SHB_ERR_PS_OIL_POLLUTION)
			update_client_info(pkt->dmd_case);
		dsm_client_record(shb_dclient, "dmd_case = %d", pkt->dmd_case);
		dsm_client_notify(shb_dclient, pkt->dmd_id);
	} else {
		hwlog_info("%s:dsm_client_ocuppy fail\n", __func__);
		dsm_client_unocuppy(shb_dclient);
		if (!dsm_client_ocuppy(shb_dclient)) {
			if (pkt->dmd_id == DSM_SHB_ERR_MCU_I2C_ERR ||
				pkt->dmd_id == DSM_SHB_ERR_PS_OIL_POLLUTION)
				update_client_info(pkt->dmd_case);
			dsm_client_record(shb_dclient,
				"dmd_case = %d", pkt->dmd_case);
			dsm_client_notify(shb_dclient, pkt->dmd_id);
		}
	}
#endif
	if (sensor_need_reset_power(pkt)) {
		hwlog_err(" %s reset sensorhub\n", obj_tag_str[pkt->hd.tag]);
		xhub_need_recovery(SH_FAULT_RESET);
	}

	kfree(iom7_log->log_p);
	kfree(iom7_log);
}

static int report_sensor_event_batch(int tag, int value[], int length,
	uint64_t timestamp)
{
	struct sensor_data event;
	int64_t ltimestamp;
	struct sensor_data_xyz *sensor_batch_data;

	sensor_batch_data = (struct sensor_data_xyz *)value;
	if ((!(tag >= TAG_FLUSH_META && tag < TAG_SENSOR_END)) ||
		(length > sizeof(event.value))) {
		hwlog_err("para error (tag : %d), (length : %d) in %s\n",
			tag, length, __func__);
		return -EINVAL;
	}
	if (tag != TAG_FLUSH_META) {
		ltimestamp = timestamp;
		event.type = tag_to_hal_sensor_type[tag];
		event.length = length;
		memcpy(&event.value, (char *)&(sensor_batch_data->x),
			event.length);
		sensor_get_data(&event);
	} else {
		ltimestamp = 0;
		event.type = tag_to_hal_sensor_type[tag];
		event.length = 4;
		event.value[0] =
			tag_to_hal_sensor_type[((pkt_header_t *)value)->tag];
	}
	return xhub_route_write_batch(ROUTE_SHB_PORT, (char *)&event,
		event.length + offset_of_end_mem(struct sensor_data, length),
		ltimestamp);
}

static int64_t get_sensor_syscounter_timestamp(
	pkt_batch_data_req_t *sensor_event)
{
	int64_t timestamp;

	timestamp = get_timestamp();

	if (sensor_event->data_hd.data_flag & DATA_FLAG_VALID_TIMESTAMP)
		timestamp = sensor_event->data_hd.timestamp;

	hwlog_debug("sensor %d origin tick %lld transfer timestamp %lld.\n",
		sensor_event->data_hd.hd.tag,
		sensor_event->data_hd.timestamp, timestamp);
	return timestamp;
}

static void process_ps_report(const pkt_header_t *head)
{
	pkt_batch_data_req_t *sensor_event = (pkt_batch_data_req_t *)head;

	ps_value = sensor_event->xyz[0].x;
	if (sensor_event->xyz[0].x != 0) {
		__pm_wakeup_event(&wlock, jiffies_to_msecs(HZ));
		hwlog_info("Kernel get far event!pdata=%d\n",
			sensor_event->xyz[0].y);
	} else {
		hwlog_info("Kernel get near event!!!!pdata=%d\n",
			sensor_event->xyz[0].y);
	}
}

static void process_ext_hall_report(const pkt_header_t *head)
{
	pkt_batch_data_req_t *sensor_event = NULL;

	sensor_event = (pkt_batch_data_req_t *)head;
	if (!sensor_event)
		return;
	hwlog_info("get fold_ext_hall event = %d, %d\n",
		sensor_event->xyz[0].x, sensor_event->xyz[0].y);
	/* when fold open, set 5s lock for avoid system suspend */
	if (sensor_event->xyz[0].y == 1)
		__pm_wakeup_event(&wlock, 5000);
}

static void process_phonecall_report(const pkt_header_t *head)
{
	pkt_batch_data_req_t *sensor_event = (pkt_batch_data_req_t *)head;

	__pm_wakeup_event(&wlock, jiffies_to_msecs(HZ));
	hwlog_info("Kernel get phonecall event! %d %d %d\n",
		sensor_event->xyz[0].x, sensor_event->xyz[0].y,
		sensor_event->xyz[0].z);
	if (sensor_event->xyz[0].y == 1 && ps_value != 0)
		hwlog_info("ps don't get the point!\n");
}

static void process_step_counter_report(const pkt_header_t *head)
{
	__pm_wakeup_event(&wlock, jiffies_to_msecs(HZ));
	hwlog_info("Kernel get pedometer event!\n");
	step_counter_data_process((pkt_step_counter_data_req_t *)head);
	report_sensor_event(head->tag,
		(int *)(&((pkt_step_counter_data_req_t *) head)->step_count),
		head->length);
}

static int process_drop_report(const pkt_drop_data_req_t *head)
{
	struct imonitor_eventobj *obj = NULL;
	int ret = 0;
	int yaw = 0;
	int speed = 0;
	int shell = 0;
	int film = 0;

	if (!head) {
		hwlog_err("%s para error\n", __func__);
		return -1;
	}

	hwlog_info("Kernel get drop type %d, initial_speed %d, height %d, angle_pitch %d, angle_roll %d, impact %d\n",
		head->data.type, head->data.initial_speed,
		head->data.height, head->data.angle_pitch,
		head->data.angle_roll, head->data.material);

	obj = imonitor_create_eventobj(SENSOR_DROP_IMONITOR_ID);
	if (!obj) {
		hwlog_err("%s imonitor_create_eventobj failed\n", __func__);
		return -1;
	}

	ret += imonitor_set_param_integer_v2(obj, "Type",
		(long)(head->data.type));
	ret += imonitor_set_param_integer_v2(obj, "InitSpeed",
		(long)(head->data.initial_speed));
	ret += imonitor_set_param_integer_v2(obj, "Height",
		(long)(head->data.height));
	ret += imonitor_set_param_integer_v2(obj, "Pitch",
		(long)(head->data.angle_pitch));
	ret += imonitor_set_param_integer_v2(obj, "Roll",
		(long)(head->data.angle_roll));
	ret += imonitor_set_param_integer_v2(obj, "Material",
		(long)(head->data.material));

	ret += imonitor_set_param_integer_v2(obj, "Yaw", (long)(yaw));
	ret += imonitor_set_param_integer_v2(obj, "Speed", (long)(speed));
	ret += imonitor_set_param_integer_v2(obj, "Shell", (long)(shell));
	ret += imonitor_set_param_integer_v2(obj, "Film", (long)(film));
	if (ret) {
		imonitor_destroy_eventobj(obj);
		hwlog_err("%s imonitor_set_param failed, ret %d\n", __func__, ret);
		return ret;
	}

	ret = imonitor_send_event(obj);
	if (ret < 0)
		hwlog_err("%s imonitor_send_event failed, ret %d\n", __func__, ret);

	imonitor_destroy_eventobj(obj);
	return ret;
}

static int process_sensors_report(const pkt_header_t *head)
{
	pkt_batch_data_req_t *sensor_event = (pkt_batch_data_req_t *)head;

	switch (head->tag) {
	case TAG_PS:
		process_ps_report(head);
		if (unlikely(stop_auto_ps)) {
			hwlog_info("%s not report ps data for dt\n", __func__);
			return -1;
		}
		break;
	case TAG_EXT_HALL:
		process_ext_hall_report(head);
		break;
	case TAG_CAP_PROX:
		hwlog_debug("TAG_CAP_PROX!!!!data[0]=%d,data[1]=%d,data[2]=%d.\n",
				sensor_event->xyz[0].x, sensor_event->xyz[0].y,
				sensor_event->xyz[0].z);
		break;
	case TAG_ACCEL:
		if (unlikely(stop_auto_accel)) {
			hwlog_info("%s not report accel_data for dt\n",
					__func__);
			return -1;
		}
		break;
	case TAG_ALS:
		if (unlikely(stop_auto_als)) {
			hwlog_info("%s not report als_data for dt\n", __func__);
			return -1;
		}
		break;
	case TAG_PRESSURE:
		hwlog_debug("pressure x %x y %x z %x.\n",
			sensor_event->xyz[0].x, sensor_event->xyz[0].y,
			sensor_event->xyz[0].z);
		get_airpress_data = sensor_event->xyz[0].x;
		get_temperature_data = sensor_event->xyz[0].y;
		break;
	case TAG_PHONECALL:
		process_phonecall_report(head);
		break;
	case TAG_RPC:
		hwlog_debug("TAG_RPC!data[0]=%d,data[1]=%d,data[2]=%d.\n",
				sensor_event->xyz[0].x, sensor_event->xyz[0].y,
				sensor_event->xyz[0].z);
		break;
	case TAG_DROP:
		process_drop_report((pkt_drop_data_req_t *)head);
		break;
	default:
		break;
	}
	return 0;
}

static int xhub_process_sensor_report(const pkt_header_t *head)
{
	uint64_t delta;
	uint64_t i;
	int64_t timestamp;
	int64_t head_timestamp;
	int16_t flush_flag = 0;
	pkt_batch_data_req_t *sensor_event = (pkt_batch_data_req_t *)head;

	if ((head->tag == TAG_PS) || (head->tag == TAG_EXT_HALL)) {
		hwlog_info("hold lock avoid suspend for ps, ext_hall");
		/* hold wakelock 1ms avoid system suspend */
		__pm_wakeup_event(&wlock, jiffies_to_msecs(1));
	}

	if (head->tag == TAG_STEP_COUNTER) { /* extend step counter date */
		process_step_counter_report(head);
		flush_flag = ((pkt_step_counter_data_req_t *)head)->data_flag &
			FLUSH_END;
		if (flush_flag == 1)
			goto flush_event;
		else
			return 0;
	}

	sensor_read_number[head->tag] += sensor_event->data_hd.cnt;
	timestamp = get_sensor_syscounter_timestamp(sensor_event);
	if ((sensor_event->data_hd.hd.tag == 1) ||
		(sensor_event->data_hd.hd.tag == 2))
		hwlog_debug("sensor %d origin tick %lld transfer timestamp %lld\n",
			sensor_event->data_hd.hd.tag,
			sensor_event->data_hd.timestamp, timestamp);

	if (timestamp <= sensors_tm[head->tag])
		timestamp = sensors_tm[head->tag] + 1;

	if (sensor_event->data_hd.cnt < 1) {
		goto flush_event;
	} else if (sensor_event->data_hd.cnt > 1) {
		delta = (uint64_t)(sensor_event->data_hd.sample_rate) * 1000000;
		head_timestamp = timestamp -
			(sensor_event->data_hd.cnt - 1) * (int64_t)delta;
		if (head_timestamp <= sensors_tm[head->tag]) {
			delta = (timestamp - sensors_tm[head->tag]) /
				sensor_event->data_hd.cnt;
			timestamp = sensors_tm[head->tag] + delta;
		} else {
			timestamp = head_timestamp;
		}
		for (i = 0; i < sensor_event->data_hd.cnt; i++) {
			report_sensor_event_batch(head->tag,
				(int *)((char *)head +
					offset(pkt_batch_data_req_t, xyz) +
					i * sensor_event->data_hd.len_element),
				sensor_event->data_hd.len_element, timestamp);
			timestamp += delta;
		}
		timestamp -= delta;
		sensors_tm[head->tag] = timestamp;
		goto flush_event;
	}

	if (process_sensors_report(head) < 0)
		return -1;
	report_sensor_event_batch(head->tag, (int *)(sensor_event->xyz),
		sensor_event->data_hd.len_element, timestamp);
	sensors_tm[head->tag] = timestamp;
flush_event:
	if ((sensor_event->data_hd.data_flag & FLUSH_END) || flush_flag == 1)
		report_sensor_event_batch(TAG_FLUSH_META, (int *)head,
			sizeof(pkt_header_t), 0);
	return 0;
}

static int update_fingersense_zaxis_data(const pkt_header_t *head)
{
	unsigned long flags = 0;
	pkt_fingersense_data_report_req_t *zaxis_report =
		(pkt_fingersense_data_report_req_t *)head;
	s16 *buffer = zaxis_report->zaxis_data;
	int nsamples = zaxis_report->hd.length / sizeof(s16);

	if (nsamples < 0) {
		hwlog_err("The second parameter of %s is wrong(negative number)\n",
			__func__);
		return -1;
	}
	spin_lock_irqsave(&fsdata_lock, flags);
	memcpy(fingersense_data, buffer,
		min(nsamples, FINGERSENSE_DATA_NSAMPLES) * sizeof(*buffer));
	fingersense_data_ready = true;
	fingersense_data_intrans = false;
	spin_unlock_irqrestore(&fsdata_lock, flags);
	return 0;
}

static int xhub_process_additional_info_report(const pkt_header_t *head)
{
	int64_t timestamp;
	pkt_additional_info_req_t *addi_info = NULL;
	struct sensor_data event;
	int info_len;

	addi_info = (pkt_additional_info_req_t *)head;
	timestamp = get_timestamp();
	if (head->tag >= TAG_SENSOR_END) {
		hwlog_err("%s head->tag = %d\n", __func__, head->tag);
		return -1;
	}
	if (addi_info->serial == 1) { /* create a begin event */
		event.type = SENSORHUB_TYPE_ADDITIONAL_INFO;
		event.serial = 0;
		event.sensor_type = tag_to_hal_sensor_type[head->tag];
		event.data_type = AINFO_BEGIN;
		event.length = 12;
		xhub_route_write_batch(ROUTE_SHB_PORT, (char *)&event,
			event.length + offset_of_end_mem(struct sensor_data, length), timestamp);
		hwlog_info("###report sensor type %d first addition info event!\n",
			event.sensor_type);
	}

	info_len = head->length - 8;
	event.type = SENSORHUB_TYPE_ADDITIONAL_INFO;
	event.serial = addi_info->serial;
	event.sensor_type = tag_to_hal_sensor_type[head->tag];
	event.data_type = addi_info->type;
	event.length = info_len + 12;
	if (info_len < 0 || info_len > sizeof(event.info)) {
		hwlog_err("%s memcpy length illegal, info_len=%d\n", __func__, info_len);
		return -1;
	}
	memcpy(event.info, addi_info->data_int32, info_len);
	xhub_route_write_batch(ROUTE_SHB_PORT, (char *)&event,
		event.length + offset_of_end_mem(struct sensor_data, length),
		timestamp);
	hwlog_info("report sensor type %d addition info: %d\n",
		event.sensor_type, event.info[0]);

	if (addi_info->end == 1) {
		event.type = SENSORHUB_TYPE_ADDITIONAL_INFO;
		event.serial = ++addi_info->serial;
		event.sensor_type = tag_to_hal_sensor_type[head->tag];
		event.data_type = AINFO_END;
		event.length = 12;
		xhub_route_write_batch(ROUTE_SHB_PORT, (char *)&event,
			event.length + offset_of_end_mem(struct sensor_data, length),
			timestamp);
		hwlog_info("report sensor_type %d end addition info event\n",
			event.sensor_type);
	}
	return 0;
}

static int xhub_process_dmd_log_report(const pkt_header_t *head)
{
	struct iom7_log_work *work = NULL;

	hwlog_info("[iom7]dmd_log_data_report");
	work = kmalloc(sizeof(struct iom7_log_work), GFP_ATOMIC);
	if (!work)
		return -ENOMEM;

	memset(work, 0, sizeof(struct iom7_log_work));
	work->log_p = kmalloc(head->length + sizeof(pkt_header_t), GFP_ATOMIC);
	if (!work->log_p) {
		kfree(work);
		return -ENOMEM;
	}
	memset(work->log_p, 0, head->length + sizeof(pkt_header_t));
	memcpy(work->log_p, head, head->length + sizeof(pkt_header_t));
	INIT_DELAYED_WORK(&(work->log_work), iom7_dmd_log_handle);
	queue_delayed_work(system_power_efficient_wq, &(work->log_work),
		msecs_to_jiffies(250));
	return 0;
}

void tell_screen_status_to_mcu(uint8_t status)
{
	write_info_t pkg_ap;
	read_info_t pkg_mcu;
	int ret;
	pkt_parameter_req_t spkt;
	pkt_header_t *shd = (pkt_header_t *)&spkt;

	memset(&pkg_ap, 0, sizeof(pkg_ap));
	memset(&pkg_mcu, 0, sizeof(pkg_mcu));
	memset(&spkt, 0, sizeof(spkt));

	spkt.subcmd = SUB_CMD_SCREEN_STATUS;
	pkg_ap.tag = TAG_POSTURE;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	pkg_ap.wr_buf = &shd[1]; /* buf data from shd buf[1] */
	pkg_ap.wr_len = min(sizeof(spkt.para), sizeof(status)) + SUBCMD_LEN;
	memcpy(spkt.para, &status, min(sizeof(spkt.para), sizeof(status)));

	hwlog_info("%s, get screen status = %d\n", __func__, status);
	if ((g_xhub_state == XHUB_ST_RECOVERY) ||
		(iom3_power_state == ST_SLEEP))
		ret = write_customize_cmd(&pkg_ap, NULL, false);
	else
		ret = write_customize_cmd(&pkg_ap, &pkg_mcu, true);

	if (ret) {
		hwlog_err("send screen status to mcu fail,ret=%d\n", ret);
		return;
	}
	if (pkg_mcu.errno != 0)
		hwlog_err("screen status send fail\n");
}

static int xhub_process_motion_report(const pkt_header_t *head)
{
	char *motion_data = (char *)head + sizeof(pkt_common_data_t);

	if ((((int)motion_data[0]) == MOTIONHUB_TYPE_TAKE_OFF) ||
		(((int)motion_data[0]) == MOTIONHUB_TYPE_PICKUP) ||
		(((int)motion_data[0]) == MOTION_TYPE_REMOVE)) {
		__pm_wakeup_event(&wlock, jiffies_to_msecs(HZ));
		hwlog_err("%s weaklock HZ motiontype=%d\n",
			__func__, motion_data[0]);
	}

	hwlog_info("%s : motiontype = %d motion_result = %d, motion_status = %d\n",
		__func__, motion_data[0], motion_data[1], motion_data[2]);
	return xhub_route_write(ROUTE_MOTION_PORT, motion_data,
		head->length -
		(sizeof(pkt_common_data_t) - sizeof(pkt_header_t)));
}

/*
 * Handle parameters uploaded by ALS_UD
 * Caution: ensure that the thread can sleep in the block node
 */
static int xhub_process_als_ud_report(const pkt_header_t *head,
	struct als_device_info *dev_info)
{
	als_run_stop_para_t *para = &dev_info->als_ud_data_upload;

	if (memcpy_s(para, sizeof(als_run_stop_para_t), (head + 1),
		sizeof(als_run_stop_para_t)) != EOK)
		return 0;
	hwlog_info("%s para is %llu %u %u\n", __func__,
		para->sample_start_time, para->sample_interval,
		para->integ_time);

	if (dev_info->sem_als_ud_rgbl_block_flag == 1) {
		dev_info->als_ud_rgbl_block = 1;
		dev_info->sem_als_ud_rgbl_block_flag = 0;
		wake_up_als_ud_block();
	}
	return 0;
}

static int als_ud_bl_report(const pkt_header_t *head)
{
	return 0;
}

static int als_ud_run_stop_report(const pkt_header_t *head,
	struct als_device_info *dev_info)
{
	als_run_stop_para_t *para = &dev_info->als_ud_data_upload;

	if (memcpy_s(para, sizeof(als_run_stop_para_t), (head + 1),
		sizeof(als_run_stop_para_t)) != EOK)
		return -1;
	hwlog_info("%s para is %llu %u %u\n", __func__,
		para->sample_start_time, para->sample_interval,
		para->integ_time);
	return 0;
}

static int als_ud_coef_block_report(const pkt_header_t *head,
	struct als_device_info *dev_info)
{
	return 0;
}

static int xhub_recv_msg_als_ud(const pkt_header_t *head)
{
	struct als_platform_data *pf_data = NULL;
	struct als_device_info *dev_info = NULL;

	pf_data = als_get_platform_data(head->tag);
	dev_info = als_get_device_info(head->tag);
	if (!pf_data || !dev_info)
		return 0;

	if (pf_data->als_ud_type == ALS_UD_MINUS_DSS_NOISE) {
		if (is_als_ud_bl_report(head))
			return als_ud_bl_report(head);
		else if (is_als_ud_data_report(head))
			return als_ud_run_stop_report(head, dev_info);
		else if (is_als_ud_coef_report(head))
			return als_ud_coef_block_report(head, dev_info);
	} else if (is_als_ud_data_report(head)) {
		hwlog_info("als_ud_run_stop_data_report");
		return xhub_process_als_ud_report(head, dev_info);
	}

	return 0;
}

typedef bool (*msg_head_check)(const pkt_header_t *);
typedef int (*msg_data_process)(const pkt_header_t *);

struct msg_process_ops {
	msg_head_check msg_head_checker;
	msg_data_process msg_data_handler;
};

static struct msg_process_ops msg_handler_array[] = {
	{ is_fingersense_zaxis_data_report, update_fingersense_zaxis_data },
	{ is_sensor_data_report, xhub_process_sensor_report },
	{ is_additional_info_report, xhub_process_additional_info_report },
	{ is_dmd_log_data_report, xhub_process_dmd_log_report },
	{ is_requirement_resp, report_resp_data },
	{ is_motion_data_report, xhub_process_motion_report },
	{ is_als_ud_type_report, xhub_recv_msg_als_ud },
};

static void xhub_process_msg_handler(struct work_struct *work)
{
	/* find data according work */
	struct mcu_notifier_work *p = NULL;

	p = container_of(work, struct mcu_notifier_work, worker);
	if (!p) {
		hwlog_err("get mcu notifier ptr err, p is NULL\n");
		return;
	}
	if (!p->data) {
		hwlog_err("get mcu notifier ptr err, p->data is NULL\n");
		kfree(p);
		return;
	}
	if (p->handler_id < 0 || p->handler_id >= ARRAY_SIZE(msg_handler_array)) {
		hwlog_err("get mcu notifier ptr err\n");
		goto out;
	}
	msg_handler_array[p->handler_id].msg_data_handler((const pkt_header_t *)p->data);
out:
	kfree(p->data);
	kfree(p);
}

int xhub_recv_msg_app_handler(const pkt_header_t *head, bool is_notifier)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(msg_handler_array); i++) {
		if (!(msg_handler_array[i].msg_head_checker(head)))
			continue;
		if (is_mass_ipc(head, SHMEM_RECV))
			mcu_notifier_queue_work(i, head, xhub_process_msg_handler);
		else
			msg_handler_array[i].msg_data_handler(head);
		return 0;
	}
	if (!is_notifier) {
		hwlog_err("--------->tag = %d, cmd = %d is not implement!\n",
			head->tag, head->cmd);
		fobidden_comm = 1;
		return -EINVAL;
	}
	return 0;
}

void xhub_debuginfo_update(pkt_header_t *head)
{
	++ipc_debug_info.event_cnt[head->tag];
}

int xhub_route_recv_mcu_data(const char *buf, unsigned int length)
{
	const pkt_header_t *head = (const pkt_header_t *)buf;
	bool is_notifier = false;
	int ret;

	if (head->tag < TAG_BEGIN || head->tag >= TAG_END) {
		hwlog_err("----------->head value : tag=%#.2x, cmd=%#.2x, length=%#.2x in %s\n",
		head->tag, head->cmd, head->length, __func__);
		return -EINVAL;
	}

	xhub_debuginfo_update((pkt_header_t *)head);
	wake_up_mcu_event_waiter(head);

	if (is_mcu_notifier(head)) {
		mcu_notifier_queue_work(-1, head, mcu_notifier_handler);
		is_notifier = true;
	}

	ret = xhub_recv_msg_app_handler(head, is_notifier);

	return ret;
}

void init_locks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(package_route_tbl); ++i)
		spin_lock_init(&package_route_tbl[i].buffer_spin_lock);
	mutex_init(&mutex_write_cmd);
	mutex_init(&mutex_write_adapter);
	mutex_init(&mutex_unpack);
	init_completion(&type_record.resp_complete);
	mutex_init(&type_record.lock_mutex);
	spin_lock_init(&type_record.lock_spin);
	spin_lock_init(&ref_cnt_lock);
	/* Initialize wakelock */
	wakeup_source_init(&wlock, "sensorhub");
}

phys_addr_t get_flp_reserve_mem_virt(void)
{
	return scp_get_reserve_mem_virt(FLP_MEM_ID);
}

phys_addr_t get_flp_reserve_mem_phys(void)
{
	return scp_get_reserve_mem_phys(FLP_MEM_ID);
}

phys_addr_t get_flp_reserve_mem_size(void)
{
	return scp_get_reserve_mem_size(FLP_MEM_ID);
}

phys_addr_t get_als_reserve_mem_virt(void)
{
	return scp_get_reserve_mem_virt(ALS_MEM_ID);
}

phys_addr_t get_als_reserve_mem_phys(void)
{
	return scp_get_reserve_mem_phys(ALS_MEM_ID);
}

phys_addr_t get_als_reserve_mem_size(void)
{
	return scp_get_reserve_mem_size(ALS_MEM_ID);
}

void send_aod_status(int value)
{
	int ret;
	uint32_t para = (uint32_t)value;
	write_info_t pkg_ap = { 0 };
	pkt_parameter_req_t cpkt;
	pkt_header_t *hd = (pkt_header_t *)&cpkt;

	pkg_ap.tag = TAG_ALS;
	pkg_ap.cmd = CMD_CMN_CONFIG_REQ;
	cpkt.subcmd = SUB_CMD_UPDATE_AOD_STATUS;
	pkg_ap.wr_buf = &hd[1];
	pkg_ap.wr_len = sizeof(para) + SUBCMD_LEN;
	if (memcpy_s(cpkt.para, sizeof(cpkt.para), (const void *)&(para),
		sizeof(para)) != EOK) {
		hwlog_err("%s: memcpy para fail\n", __func__);
		return;
	}
	ret = write_customize_cmd(&pkg_ap, NULL, true);
	if (ret)
		hwlog_err("send als ud data to mcu fail,ret=%d\n", ret);
	return;
}

void xhub_route_init(void)
{
	int section_offset;
	uint8_t i;

	g_shmem_send_start = scp_get_reserve_mem_virt(SENS_MEM_ID);
	g_shmem_recv_start = g_shmem_send_start + SHMEM_SIZE / 2;
	for (i = 0; i < TOTAL_SECTION_N; i++) {
		section_offset = get_section_offset(i);
		memset((void *)(uintptr_t)(g_shmem_send_start + section_offset), 0, SECTION_CTL_CMD_LEN);
		memset((void *)(uintptr_t)(g_shmem_recv_start + section_offset), 0, SECTION_CTL_CMD_LEN);
	}
	init_locks();
	init_mcu_notifier_list();
	init_mcu_event_wait_list();
}

static void close_all_ports(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(package_route_tbl); ++i)
		xhub_route_close(package_route_tbl[i].port);
}

void xhub_route_exit(void)
{
	/* close all ports */
	close_all_ports();
	wakeup_source_trash(&wlock);
}
