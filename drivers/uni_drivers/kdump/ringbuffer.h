/*
 * ringbuffer.h
 *
 * This file wraps the ring buffer.
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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
#ifndef __HUAWEI_AP_RINGBUFFER_H__
#define __HUAWEI_AP_RINGBUFFER_H__

#include <linux/kernel.h>

/* attention: must be aligned by 8, or it will data abort for unaligned data access */
#define AP_KEYS_MAX  71
struct ap_ringbuffer_s {
		unsigned int max_num;
		unsigned int field_count;
		unsigned int rear; /* write pointer, where the buffer is available to write in */
		unsigned int r_idx; /* read pointer, where the buffer is to be read */
		unsigned int count; /* how many items in the buffer are not read */
		unsigned int is_full;
		char keys[AP_KEYS_MAX + 1]; /* For parsing with PC tools */
		unsigned char data[1];
};

int  ap_ringbuffer_init(struct ap_ringbuffer_s *q, u32 bytes, u32 fieldcnt, const char *keys);
void ap_ringbuffer_write(struct ap_ringbuffer_s *q, u8 *element);
int  ap_ringbuffer_read(struct ap_ringbuffer_s *q, u8 *element, u32 len);
int  ap_is_ringbuffer_full(const void *buffer_addr);
void get_ringbuffer_start_end(struct ap_ringbuffer_s *q, u32 *start, u32 *end);
bool is_ringbuffer_empty(struct ap_ringbuffer_s *q);
bool is_ringbuffer_invalid(u32 field_count, u32 len, struct ap_ringbuffer_s *q);

#endif /* __HUAWEI_AP_RINGBUFFER_H__ */
