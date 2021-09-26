/*
 * frame_timer.c
 *
 * frame freq timer
 *
 * Copyright (c) 2019-2021 Huawei Technologies Co., Ltd.
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

#define pr_fmt(fmt) "[IPROVISION-FRAME_TIMER]: " fmt

#include "include/frame_timer.h"

#include <linux/version.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <trace/events/sched.h>
#include <linux/wait_bit.h>

#include "include/frame.h"
#include "include/proc_state.h"
#include "include/aux.h"


#define DISABLE_FRAME_SCHED 0

struct timer_list g_frame_timer_boost;
static atomic_t g_timer_boost_on = ATOMIC_INIT(0);
static struct task_struct *g_frame_timer_thread;
static unsigned long g_thread_flag;

static int frame_timer_boost_func(void *data)
{
#ifdef CONFIG_HW_RTG_MULTI_FRAME
	int id;
#endif

	for (;;) {
		wait_on_bit(&g_thread_flag, DISABLE_FRAME_SCHED,
				TASK_INTERRUPTIBLE);

		pr_debug("frame timer boost [thread] HANDLER\n");
		set_frame_min_util(rtg_frame_info(DEFAULT_RT_FRAME_ID), 0, true);
#ifdef CONFIG_HW_RTG_AUX
		set_aux_boost_util(0);
#endif
		set_boost_thread_min_util(0);
		if (!is_frame_freq_enable(DEFAULT_RT_FRAME_ID))
			set_frame_sched_state(rtg_frame_info(DEFAULT_RT_FRAME_ID), false);
#ifdef CONFIG_HW_RTG_MULTI_FRAME
		for (id = MULTI_FRAME_ID; id < (MULTI_FRAME_ID + MULTI_FRAME_NUM); id++) {
			if (!is_frame_freq_enable(id))
				set_frame_sched_state(rtg_frame_info(id), false);
		}
#endif
		set_bit(DISABLE_FRAME_SCHED, &g_thread_flag);
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void frame_timer_boost_handler(unsigned long data)
#else
static void frame_timer_boost_handler(struct timer_list *t)
#endif
{
	trace_rtg_frame_sched(0, "frame_timer_boost", 0);
	pr_debug("frame timer boost [timer] HANDLER\n");

	/* wakeup thread */
	clear_bit(DISABLE_FRAME_SCHED, &g_thread_flag);
	smp_mb__after_atomic();
	wake_up_bit(&g_thread_flag, DISABLE_FRAME_SCHED);
}

void frame_timer_boost_init(void)
{
	if (atomic_read(&g_timer_boost_on) == 1)
		return;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	setup_timer(&g_frame_timer_boost, frame_timer_boost_handler, 0);
#else
	timer_setup(&g_frame_timer_boost, frame_timer_boost_handler, 0);
#endif

	/* create thread */
	set_bit(DISABLE_FRAME_SCHED, &g_thread_flag);
	g_frame_timer_thread = kthread_create(frame_timer_boost_func,
		NULL, "frame_timer_boost");
	if (g_frame_timer_thread)
		wake_up_process(g_frame_timer_thread);
	else
		pr_err("[AWARE_RTG] g_timer_thread create failed\n");

	atomic_set(&g_timer_boost_on, 1);
	pr_debug("frame timer boost INIT\n");
}

void frame_timer_boost_stop(void)
{
	if (atomic_read(&g_timer_boost_on) == 0)
		return;

	atomic_set(&g_timer_boost_on, 0);
	del_timer_sync(&g_frame_timer_boost);
	pr_debug("frame timer boost STOP\n");
}

void frame_timer_boost_start(u32 duration, u32 min_util)
{
	unsigned long dur = msecs_to_jiffies(duration);
	int id = DEFAULT_RT_FRAME_ID;

	if (atomic_read(&g_timer_boost_on) == 0)
		return;

	if (timer_pending(&g_frame_timer_boost) &&
		time_after(g_frame_timer_boost.expires, jiffies + dur))
		return;

	set_frame_min_util(rtg_frame_info(id), min_util, true);
#ifdef CONFIG_HW_RTG_MULTI_FRAME
	for (id = MULTI_FRAME_ID; id < (MULTI_FRAME_ID + MULTI_FRAME_NUM); id++)
		set_frame_min_util(rtg_frame_info(id), min_util, true);
#endif
#ifdef CONFIG_HW_RTG_AUX
	set_aux_boost_util(min_util);
#endif
	set_boost_thread_min_util(min_util);
	mod_timer(&g_frame_timer_boost, jiffies + dur);
	trace_rtg_frame_sched(0, "frame_timer_boost", dur);
	pr_debug("frame timer boost START, duration = %ums, min_util = %u\n", duration, min_util);
}
