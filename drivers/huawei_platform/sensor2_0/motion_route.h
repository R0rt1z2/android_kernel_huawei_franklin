/*
 * motion_route.h
 *
 * code for motion route
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#ifndef __LINUX_MOTION_ROUTE_H__
#define __LINUX_MOTION_ROUTE_H__
#include <linux/version.h>
#include <huawei_platform/log/hw_log.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define HWLOG_TAG hw_motion_route
HWLOG_REGIST();

#define LENGTH_SIZE             sizeof(unsigned int)
#define TIMESTAMP_MOTION_SIZE   8
#define HEAD_SIZE               (LENGTH_SIZE + TIMESTAMP_MOTION_SIZE)
#define ROUTE_BUFFER_MAX_SIZE   1024

struct t_head {
	int for_alignment;
	union {
		char effect_addr[sizeof(int)];
		int pkg_length;
	};
	int64_t timestamp;
};

struct inputhub_buffer_pos {
	char *pos;
	unsigned int buffer_size;
};

// Every route item can be used by one reader and one writer.
struct inputhub_route_table {
	struct inputhub_buffer_pos phead; // point to the head of buffer
	struct inputhub_buffer_pos pread; // point to the read position of buf
	struct inputhub_buffer_pos pwrite; // point to the write position of buf
	wait_queue_head_t read_wait; // to block read when no data in buffer
	atomic_t data_ready;
	spinlock_t buffer_spin_lock; //for read write buffer
};

int motion_route_init(void);
void motion_route_destroy(void);
ssize_t motion_route_read(char __user *buf, size_t count);
ssize_t motion_route_write(char *buf, size_t count);
#endif
