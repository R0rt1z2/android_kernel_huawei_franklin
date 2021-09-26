/*
 * track.h
 *
 * record the irq and task data.(head file)
 *
 * Copyright (c) 2013-2020 Huawei Technologies Co., Ltd.
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
#ifndef __HUAWEI_AP_TRACK_H__
#define __HUAWEI_AP_TRACK_H__

#include <linux/types.h>
#include <linux/cpu.h>

#define AP_HOOK_CPU_NUMBERS 8
#define PERCPU_TOTAL_RATIO 16

typedef u64 (*arch_timer_func_ptr)(void);

struct task_info_ap {
	u64 clock;
	u64 stack;
	u32 pid;
	char comm[TASK_COMM_LEN];
};

struct hook_irq_info_ap {
	u64 clock;
	u64 jiff;
	u32 irq;
	u8 dir;
};

enum hook_type_ap {
	AP_HK_IRQ = 0,
	AP_HK_TASK,
	AP_HK_PERCPU_TAG, /*notify above track is percpu*/
	AP_HK_MAX
};

struct ap_percpu_buffer_info {
	unsigned char *buffer_addr;
	unsigned char *percpu_addr[NR_CPUS];
	unsigned int percpu_length[NR_CPUS];
	unsigned int buffer_size;
};

struct ap_hook {
	unsigned char *hook_buffer_addr[AP_HK_MAX];
	struct ap_percpu_buffer_info hook_percpu_buffer[AP_HK_PERCPU_TAG];
};

#endif /* __HUAWEI_AP_TRACK_H__ */
