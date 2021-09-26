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

#ifndef __LINUX_XHUB_ROUTE_H__
#define __LINUX_XHUB_ROUTE_H__
#include "protocol.h"
#include <linux/version.h>
#include <log/hw_log.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <xhub_router/sensorhub.h>

#define HWLOG_TAG sensorhub
HWLOG_REGIST();

#define XHUB_ST_NORMAL			0
#define XHUB_ST_RECOVERY		1
#define XHUB_ST_REPEAT			2
#define MAX_SEND_LEN 32

/* max gap is 600ms */
#define MAX_SYSCOUNT_TIME_GAP 600000000LL
/* limit is 1 hour */
#define SYSCOUNT_DMD_LIMIT (3600 * 1000000000LL)
#define LENGTH_SIZE sizeof(unsigned int)
#define HEAD_SIZE (LENGTH_SIZE + TIMESTAMP_SIZE)
/* k3v5 report stairs level,record_count base on 1000 */
#define EXT_PEDO_VERSION_2 2000
#define ALS_RESET_COUNT 1
#define PS_RESET_COUNT 1
#define TP_RESET_COUNT 5
#define RESET_REFRESH_PERIOD (3600 * 8)
#define READ_CURRENT_INTERVAL 500
#define LOG_LEVEL_FATAL 0
/* Number of z-axis samples required by FingerSense at 1.6KHz ODR */
#define FINGERSENSE_DATA_NSAMPLES 128
#define HALL_ONE_DATA_NUM 4 /* number type of one ext_hall data */
#define HALL_DATA_NUM 3
#define MAX_EXT_HALL_VALUE 3


struct hw_ap_cp_route_t {
	unsigned int channel_id;
	int (*icc_open)(unsigned int channel_id, void *read_cb);
	int (*icc_read)(unsigned int channel_id, unsigned char *pdata,
		int size);
	int (*icc_write)(unsigned int channel_id, unsigned char *pdata,
		int size);
};

enum ext_hall_sensor_type {
	EXT_HALL_TYPE_START = 0,
	SLIDE_HALL_TYPE = 1,
	HUB_FOLD_HALL_TYPE = 2,
	EXT_HALL_TYPE_END,
};

struct link_package {
	int partial_order;
	char link_buffer[MAX_PKT_LENGTH_AP];
	int total_pkg_len;
	int offset;
};

struct mcu_notifier_work {
	struct work_struct worker;
	int handler_id;
	void *data;
};

struct mcu_notifier_node {
	int tag;
	int cmd;
	int (*notify)(const pkt_header_t *data);
	struct list_head entry;
};

struct mcu_notifier {
	struct list_head head;
	spinlock_t lock;
	struct workqueue_struct *mcu_notifier_wq;
};

#define offset(struct_t, member) \
		offsetof(struct_t, member)
#define offset_of_end_mem(struct_t, member) \
		((unsigned long)(&(((struct_t *)0)->member) + 1))
#define offset_interval(struct_t, member1, member2) \
		(offset_of_end_mem(struct_t, member2) - \
		offset_of_end_mem(struct_t, member1))

typedef enum {
	DMD_CASE_ALS_NEED_RESET_POWER,
	DMD_CASE_PS_NEED_RESET_POWER,
	DMD_CASE_TP_NEED_RESET_POWER,
} sensor_reset_power;

enum port {
	ROUTE_SHB_PORT = 0x01,
	ROUTE_MOTION_PORT = 0x02,
	ROUTE_CA_PORT = 0x03,
	ROUTE_FHB_PORT = 0x04,
	ROUTE_FHB_UD_PORT = 0x05,
	ROUTE_KB_PORT = 0x06,
};

/* value length increase to 16 */
struct sensor_data {
	unsigned short type;
	unsigned short length;
	union {
		int value[16]; /* xyza... */
		struct {
			int serial;
			int data_type;
			int sensor_type;
			int info[13];
		};
	};
};

struct t_sensor_get_data {
	atomic_t reading;
	struct completion complete;
	struct sensor_data data;
};
#define SELFTEST_RESULT_MAXLEN 5
struct sensor_status {
	/*
	 * record whether sensor is in activate status,
	 * already opened and setdelay
	 */
	int status[TAG_SENSOR_END];
	/* record sensor delay time */
	int delay[TAG_SENSOR_END];
	/* record whether sensor was opened */
	int opened[TAG_SENSOR_END];
	int batch_cnt[TAG_SENSOR_END];
	char gyro_selftest_result[5];
	char mag_selftest_result[5];
	char accel_selftest_result[5];
	char connectivity_selftest_result[5];
	char handpress_selftest_result[5];
	char selftest_result[TAG_SENSOR_END][SELFTEST_RESULT_MAXLEN];
	int gyro_ois_status;
	struct t_sensor_get_data get_data[SENSORHUB_TYPE_END];
};

typedef struct {
	int for_alignment;
	union {
		char effect_addr[sizeof(int)];
		int pkg_length;
	};
	int64_t timestamp;
} t_head;

struct type_record {
	const pkt_header_t *pkt_info;
	struct read_info *rd;
	struct completion resp_complete;
	struct mutex lock_mutex;
	spinlock_t lock_spin;
};

struct iom7_log_work {
	void *log_p;
	struct delayed_work log_work;
};

struct xhub_buffer_pos {
	char *pos;
	unsigned int buffer_size;
};

/*
 * Every route item can be used by one reader and one writer.
 */
struct xhub_route_table {
	unsigned short port;
	struct xhub_buffer_pos phead; /* point to the head of buffer */
	struct xhub_buffer_pos pread; /* point to read position of buffer */
	struct xhub_buffer_pos pwrite; /* point to write position of buffer */
	wait_queue_head_t read_wait; /* to block read when no data in buffer */
	atomic_t data_ready;
	spinlock_t buffer_spin_lock; /* for read write buffer */
};

typedef struct write_info {
	int tag;
	int cmd;
	const void *wr_buf; /* maybe NULL */
	int wr_len; /* maybe zero */
} write_info_t;

typedef struct read_info {
	int errno;
	int data_length;
	char data[MAX_PKT_LENGTH];
} read_info_t;

typedef struct pkt_subcmd_para {
	pkt_header_resp_t hd;
	uint32_t subcmd;
	char data[SUBCMD_LEN];
} __packed pkt_subcmd_para_t;

typedef struct sensor_operation {
	int (*enable)(bool enable);
	int (*setdelay)(int ms);
} sensor_operation_t;

typedef struct ap_sensor_ops_record {
	sensor_operation_t ops;
	bool work_on_ap;
} t_ap_sensor_ops_record;

typedef bool(*t_match) (void *priv, const pkt_header_t *pkt);
struct mcu_event_waiter {
	struct completion complete;
	t_match match;
	void *out_data;
	int out_data_len;
	void *priv;
	struct list_head entry;
};

struct iom7_charging_work {
	uint32_t event;
	struct work_struct charging_work;
};

static inline bool match_tag_cmd(void *priv, const pkt_header_t *recv)
{
	pkt_header_t *send = (pkt_header_t *) priv;

	return (send->tag == recv->tag) && ((send->cmd + 1) == recv->cmd) &&
		(send->tranid == recv->tranid);
}

#define wait_for_mcu_resp(trigger, match, wait_ms, out_data, out_data_len, \
		priv) \
({ \
	int __ret = 0; \
	struct mcu_event_waiter waiter; \
	init_wait_node_add_list(&waiter, match, out_data, out_data_len, \
		(void *)priv); \
	reinit_completion(&waiter.complete); \
	trigger; \
	__ret = wait_for_completion_timeout(&waiter.complete, \
		msecs_to_jiffies(wait_ms)); \
	list_del_mcu_event_waiter(&waiter); \
	__ret; \
})

#define wait_for_mcu_resp_data_after_send(send_pkt, trigger, wait_ms, \
		out_data, out_data_len) \
wait_for_mcu_resp(trigger, match_tag_cmd, wait_ms, \
		out_data, out_data_len, send_pkt)

#define wait_for_mcu_resp_after_send(send_pkt, trigger, wait_ms) \
wait_for_mcu_resp_data_after_send(send_pkt, trigger, wait_ms, NULL, 0)

extern int support_hall_hishow;
extern int support_hall_pen;
extern int support_hall_keyboard;
extern int support_hall_lightstrap;
extern uint32_t hall_hishow_value;
extern uint32_t hall_pen_value;
extern uint32_t hall_keyboard_value;
extern uint32_t hall_lightstrap_value;
extern char *obj_tag_str[];
extern struct sensor_status sensor_status;
extern struct config_on_ddr *g_config_on_ddr;
extern wait_queue_head_t xhub_rec_waitq;
extern int g_xhub_state;
extern struct ipc_debug ipc_debug_info;
extern volatile int hall_value;
extern struct completion iom3_reboot;
extern unsigned long stop_auto_accel;
extern int stop_auto_als;
extern int stop_auto_ps;
extern int hall_sen_type;
extern int hall_number;
#ifdef CONFIG_HUAWEI_DSM
extern struct dsm_client *shb_dclient;
extern struct dsm_dev dsm_sensorhub;
#endif
extern unsigned int sensor_read_number[];
extern int iom3_power_state;
extern uint32_t need_reset_io_power;
extern uint8_t tag_to_hal_sensor_type[TAG_SENSOR_END];
extern t_ap_sensor_ops_record all_ap_sensor_operations[TAG_SENSOR_END];
#ifdef CONFIG_WIRELESS_ACCESSORY
extern struct blocking_notifier_head tx_event_nh;
#endif

extern int ak8789_register_report_data(int ms);
extern int color_sensor_enable(bool enable);
/* called by sensorhub or tp modules. */
extern int xhub_route_open(unsigned short port);
extern void xhub_route_close(unsigned short port);
extern ssize_t xhub_route_read(unsigned short port, char __user *buf, size_t count);
/* called by xhub_mcu module or test module. */
extern void xhub_route_init(void);
extern void xhub_route_exit(void);
extern ssize_t xhub_route_write(unsigned short port, char *buf, size_t count);
extern int xhub_route_recv_mcu_data(const char *buf, unsigned int length);
extern int xhub_recv_msg_app_hander(const pkt_header_t *head, bool is_notifier);
extern int write_customize_cmd(const struct write_info *wr,
		struct read_info *rd, bool is_lock);
extern int register_mcu_event_notifier(int tag, int cmd,
		int (*notify)(const pkt_header_t *head));
extern int unregister_mcu_event_notifier(int tag,
		int cmd, int (*notify)(const pkt_header_t *head));
extern int xhub_sensor_enable(int tag, bool enable);
extern int xhub_sensor_enable_stepcounter(bool enable,
		type_step_counter_t steptype);
extern int xhub_sensor_setdelay(int tag, interval_param_t *interval_param);
extern int xhub_sensor_enable_nolock(int tag, bool enable);
extern int xhub_sensor_setdelay_nolock(int tag,
		interval_param_t *interval_param);
extern int xhub_mcu_write_cmd(const void *buf, unsigned int length);
extern void xhub_update_info(const void *buf, int ret, bool is_in_recovery);
extern int register_ap_sensor_operations(int tag, sensor_operation_t *ops);
extern int unregister_ap_sensor_operations(int tag);
extern ssize_t xhub_route_write_batch(unsigned short port, char *buf,
		size_t count, int64_t timestamp);
extern bool ap_sensor_enable(int tag, bool enable);
extern bool ap_sensor_setdelay(int tag, int ms);
extern int report_sensor_event(int tag, int value[], int length);
extern void init_wait_node_add_list(struct mcu_event_waiter *waiter,
		t_match match, void *out_data, int out_data_len, void *priv);
extern void list_del_mcu_event_waiter(struct mcu_event_waiter *self);
void tell_screen_status_to_mcu(uint8_t status);
int send_app_config_cmd(int tag, void *app_config, bool use_lock);

phys_addr_t get_flp_reserve_mem_virt(void);
phys_addr_t get_flp_reserve_mem_phys(void);
phys_addr_t get_flp_reserve_mem_size(void);

phys_addr_t get_als_reserve_mem_virt(void);
phys_addr_t get_als_reserve_mem_phys(void);
phys_addr_t get_als_reserve_mem_size(void);
void send_aod_status(int value);
#endif /* __LINUX_XHUB_ROUTE_H__ */
