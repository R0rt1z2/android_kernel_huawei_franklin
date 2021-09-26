/*
 *  drivers/misc/inputhub/inputhub_route.h
 *  Sensor Hub Channel driver
 *
 *  Copyright (C) 2012 Huawei, Inc.
 *  Author: qindiwen <inputhub@huawei.com>
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

#define LENGTH_SIZE sizeof(unsigned int)
#define TIMESTAMP_MOTION_SIZE (8)

#define HEAD_SIZE (LENGTH_SIZE+TIMESTAMP_MOTION_SIZE)
#define ROUTE_BUFFER_MAX_SIZE (1024)

typedef struct {
	int for_alignment;
	union {
		char effect_addr[sizeof(int)];
		int pkg_length;
	};
	int64_t timestamp;
} t_head;


struct inputhub_buffer_pos {
	char *pos;
	unsigned int buffer_size;
};

/*
 *Every route item can be used by one reader and one writer.
 */
struct inputhub_route_table {
	struct inputhub_buffer_pos phead;	/*point to the head of buffer*/
	struct inputhub_buffer_pos pRead;	/*point to the read position of buffer*/
	struct inputhub_buffer_pos pWrite;	/*point to the write position of buffer*/
	wait_queue_head_t read_wait;	/*to block read when no data in buffer*/
	atomic_t data_ready;
	spinlock_t buffer_spin_lock;	/*for read write buffer*/
};
#endif 
