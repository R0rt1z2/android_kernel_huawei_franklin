/*
 * rtg_pseudo.c
 *
 * rtg pseudo tick timer
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

#define pr_fmt(fmt) "[IPROVISION-FRAME_PSEUDO]: " fmt

#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <trace/events/sched.h>

#include "include/rtg.h"
#include "include/rtg_pseudo.h"
#include "include/frame_info.h"
#include "include/iaware_rtg.h"


static enum hrtimer_restart rtg_pseudo_handler(struct hrtimer *timer);

/*
 * enabled type for sysctl_sched_enable_frame_pseudo
 *	0 -=> disable
 *	1 -=> enabled, but start when update interval less than tick
 *	2 -=> enabled and use pseuddo tick force.
 */
enum rtg_pseudo_enabled_type {
	PSEUDO_DISABLED = 0,
	PSEUDO_ENABLED,
	PSEUDO_ENABLED_ALL,
	PSEUDO_ENABLED_TYPE_NR,
};

struct rtg_pseudo_timer rtg_pseudo = {
	.enabled = ATOMIC_INIT(PSEUDO_DISABLED),
	.inited = ATOMIC_INIT(0),
	.running = ATOMIC_INIT(0),
	.interval = ATOMIC_INIT(8000000),
	.timer = {
		.function = rtg_pseudo_handler,
	},
};
static struct rtg_pseudo_timer *pseudo = &rtg_pseudo;

static ktime_t rtg_pseudo_interval(void)
{
	return ns_to_ktime((u64)atomic_read(&pseudo->interval));
}

static enum hrtimer_restart rtg_pseudo_handler(struct hrtimer *timer)
{
	struct related_thread_group *grp = NULL;

	if (atomic_read(&pseudo->running) == 0)
		return HRTIMER_NORESTART;

#ifdef CONFIG_HW_RTG_FRAME
	/* only frame use rtg_pseudo now */
	grp = frame_rtg();
	if (!grp || !grp->nr_running || !grp->rtg_class)
		goto restart;

	/* update rtg tick pseudo */
	update_frame_info_tick_common(grp);
	trace_rtg_frame_sched("frame_pseudo", grp->id);

	pr_debug("rtg pseudo timer HANDLER\n");

restart:
	/* restart hrtimer */
	hrtimer_forward_now(timer, rtg_pseudo_interval());

	return HRTIMER_RESTART;
#else
	return HRTIMER_NORESTART;
#endif
}

void rtg_pseudo_init(void)
{
	if (atomic_read(&pseudo->inited) == 1)
		return;

	hrtimer_init(&pseudo->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pseudo->timer.function = rtg_pseudo_handler;
	atomic_set(&pseudo->inited, 1);

	pr_info("rtg pseudo timer INIT\n");
}

void rtg_pseudo_start(void)
{
	if (atomic_read(&pseudo->running) == 1) {
		pr_debug("rtg pseudo already START\n");
		return;
	}

	if (atomic_read(&pseudo->inited) == 0)
		rtg_pseudo_init();

	hrtimer_start(&pseudo->timer, rtg_pseudo_interval(), HRTIMER_MODE_REL);
	atomic_set(&pseudo->running, 1);

	trace_rtg_frame_sched("frame_pseudo", NR_CPUS);
	pr_info("rtg pseudo timer START\n");
}

void rtg_pseudo_stop(void)
{
	if (atomic_read(&pseudo->running) == 0) {
		pr_debug("rtg pseudo already STOP\n");
		return;
	}

	hrtimer_cancel(&pseudo->timer);
	atomic_set(&pseudo->running, 0);

	trace_rtg_frame_sched("frame_pseudo", NR_CPUS + 1);
	pr_info("rtg pseudo timer STOP\n");
}

bool rtg_pseudo_should_start()
{
	/* pseudo not enabled */
	if (atomic_read(&pseudo->enabled) == PSEUDO_DISABLED)
		return false;

	/*
	 * freq update interval large than tick(HZ)
	 * eg. HZ=250
	 * if interval >= 4ms, we shouldn't start freq
	 */
	if (atomic_read(&pseudo->interval) / NSEC_PER_MSEC * HZ >= MSEC_PER_SEC &&
	    atomic_read(&pseudo->enabled) == PSEUDO_ENABLED)
		return false;

	/*
	 * interval / NSEC_PER_MSEC * HZ >= MSEC_PER_SEC ||
	 * enabled == PSEUDO_ENABLED_ALL
	 */
	return true;
}

void rtg_pseudo_update(unsigned long interval/* NS */)
{
	if (atomic_read(&pseudo->interval) == interval)
		return;

	atomic_set(&pseudo->interval, interval);
	trace_rtg_frame_sched("frame_pseudo_interval", interval);

	if (rtg_pseudo_should_start())
		rtg_pseudo_start();
	else
		rtg_pseudo_stop();
}

#ifdef CONFIG_HW_RTG_FRAME
void frame_pseudo_create(void)
{
	unsigned long flag;
	struct related_thread_group *grp = NULL;

	grp = frame_rtg();
	if (!grp)
		return;

	raw_spin_lock_irqsave(&grp->lock, flag);
	rtg_pseudo_update(grp->freq_update_interval);
	raw_spin_unlock_irqrestore(&grp->lock, flag);
}

void frame_pseudo_destroy(void)
{
	struct related_thread_group *grp = NULL;

	grp = frame_rtg();
	if (!grp)
		return;

	rtg_pseudo_stop();
}

void frame_pseudo_update(struct related_thread_group *grp)
{
	if (!grp || !is_frame_rtg(grp->id))
		return;

	lockdep_assert_held(&grp->lock);

	rtg_pseudo_update(grp->freq_update_interval);
}

bool frame_pseudo_is_running(void)
{
	return  (atomic_read(&pseudo->running) == 1);
}

void frame_pseudo_enable(enum rtg_pseudo_enabled_type enable)
{
	if (enable < PSEUDO_DISABLED || enable >= PSEUDO_ENABLED_TYPE_NR)
		return;

	if (atomic_read(&pseudo->enabled) == enable)
		return;

	atomic_set(&pseudo->enabled, enable);

	if (rtg_pseudo_should_start())	/* start rtg_pseudo when need */
		rtg_pseudo_start();
	else				/* stop rtg_pseudo when it's running */
		rtg_pseudo_stop();
}

#ifdef CONFIG_PROC_SYSCTL
unsigned int sysctl_sched_enable_frame_pseudo = PSEUDO_DISABLED;

int sched_proc_frame_pseudo_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	enum rtg_pseudo_enabled_type enable;

	if (ret || !write)
		return ret;

	enable = sysctl_sched_enable_frame_pseudo;
	frame_pseudo_enable(enable);

	return 0;
}
#endif
#endif
