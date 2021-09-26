/*
 * rtg_pseudo.h
 *
 * frame freq timer header
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
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

#ifndef __RTG_PSEUDO_H__
#define __RTG_PSEUDO_H__

#include <asm/atomic.h>
#include <linux/types.h>

/* rtg_pseudo: Global now */
struct rtg_pseudo_timer {
	struct hrtimer		timer;
	atomic_t		enabled;
	atomic_t		inited;
	atomic_t		interval;
	atomic_t		running;
};

#ifdef CONFIG_PROC_SYSCTL
extern unsigned int sysctl_sched_enable_frame_pseudo;
int sched_proc_frame_pseudo_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos);
#endif

void rtg_pseudo_stop(void);
void frame_pseudo_create(void);
void frame_pseudo_destroy(void);
bool frame_pseudo_is_running(void);
void frame_pseudo_update(struct related_thread_group *grp);

#endif
