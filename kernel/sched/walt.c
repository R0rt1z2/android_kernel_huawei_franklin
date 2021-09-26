/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Window Assisted Load Tracking (WALT) implementation credits:
 * Srivatsa Vaddagiri, Steve Muckle, Syed Rameez Mustafa, Joonwoo Park,
 * Pavan Kumar Kondeti, Olav Haugan
 *
 * 2016-03-06: Integration with EAS/refactoring by Vikram Mulukutla
 *             and Todd Kjos
 */

#include <linux/acpi.h>
#include <linux/syscore_ops.h>
#include <trace/events/sched.h>
#include "sched.h"
#include "walt.h"
#ifdef CONFIG_HW_RTG
#include <linux/sched/hw_rtg/rtg_sched.h>
#endif
#ifdef CONFIG_HW_SCHED_CLUSTER
#include "hw_cluster/sched_cluster.h"
#endif
#include "hw_walt/pred_load.h"
#ifdef CONFIG_HW_ED_TASK
#include "hw_task/hw_ed_task.h"
#endif
#include "tune.h"
#include "hw_task/hw_top_task.h"

#ifdef CONFIG_HW_EAS_SCHED
__read_mostly unsigned int sysctl_sched_walt_cpu_high_irqload =
    (2 * NSEC_PER_SEC / CONFIG_HZ);
#else
__read_mostly unsigned int sysctl_sched_walt_cpu_high_irqload =
    (10 * NSEC_PER_MSEC);
#endif

#ifdef CONFIG_HW_SCHED_CHECK_IRQLOAD
__read_mostly unsigned int sysctl_sched_walt_cpu_overload_irqload =
   (3 * NSEC_PER_SEC / CONFIG_HZ);
#endif

#define EXITING_TASK_MARKER	0xdeaddead

static __read_mostly unsigned int walt_ravg_hist_size = 5;
static __read_mostly unsigned int walt_window_stats_policy =
	WINDOW_STATS_MAX_RECENT_AVG;
static __read_mostly unsigned int walt_account_wait_time = 1;
static __read_mostly unsigned int walt_freq_account_wait_time = 0;
static __read_mostly unsigned int walt_io_is_busy = 0;

unsigned int sysctl_sched_walt_init_task_load_pct = 15;

/* true -> use PELT based load stats, false -> use window-based load stats */
bool __read_mostly walt_disabled = false;

/*
 * Window size (in ns). Adjust for the tick size so that the window
 * rollover occurs just before the tick boundary.
 */
__read_mostly unsigned int walt_ravg_window =
					    (20000000 / TICK_NSEC) * TICK_NSEC;
#define MIN_SCHED_RAVG_WINDOW ((10000000 / TICK_NSEC) * TICK_NSEC)
#define MAX_SCHED_RAVG_WINDOW ((1000000000 / TICK_NSEC) * TICK_NSEC)

static unsigned int sync_cpu;
static ktime_t ktime_last;
static __read_mostly bool walt_ktime_suspended;

static unsigned int task_load(struct task_struct *p)
{
	return p->ravg.demand;
}

static inline void fixup_cum_window_demand(struct rq *rq, s64 delta)
{
	rq->cum_window_demand += delta;
	if (unlikely((s64)rq->cum_window_demand < 0))
		rq->cum_window_demand = 0;
}

void
walt_inc_cumulative_runnable_avg(struct rq *rq,
				 struct task_struct *p)
{
	rq->cumulative_runnable_avg += p->ravg.demand;

	/*
	 * Add a task's contribution to the cumulative window demand when
	 *
	 * (1) task is enqueued with on_rq = 1 i.e migration,
	 *     prio/cgroup/class change.
	 * (2) task is waking for the first time in this window.
	 */
	if (p->on_rq || (p->last_sleep_ts < rq->window_start))
		fixup_cum_window_demand(rq, p->ravg.demand);

#ifdef CONFIG_HW_SCHED_PRED_LOAD
	fixup_pred_load(rq, p->ravg.predl);
	if (predict_util(rq) > capacity_curr_of(cpu_of(rq)))
		sugov_mark_util_change(cpu_of(rq), PRED_LOAD_ENQUEUE);
#endif
}

void
walt_dec_cumulative_runnable_avg(struct rq *rq,
				 struct task_struct *p)
{
	rq->cumulative_runnable_avg -= p->ravg.demand;
	BUG_ON((s64)rq->cumulative_runnable_avg < 0);

	/*
	 * on_rq will be 1 for sleeping tasks. So check if the task
	 * is migrating or dequeuing in RUNNING state to change the
	 * prio/cgroup/class.
	 */
	if (task_on_rq_migrating(p) || p->state == TASK_RUNNING)
		fixup_cum_window_demand(rq, -(s64)p->ravg.demand);

#ifdef CONFIG_HW_SCHED_PRED_LOAD
	fixup_pred_load(rq, -(s64)p->ravg.predl);
#endif
}

static void
fixup_cumulative_runnable_avg(struct rq *rq,
			      struct task_struct *p, u64 new_task_load)
{
	s64 task_load_delta = (s64)new_task_load - task_load(p);

	rq->cumulative_runnable_avg += task_load_delta;
	if ((s64)rq->cumulative_runnable_avg < 0)
		panic("cra less than zero: tld: %lld, task_load(p) = %u\n",
			task_load_delta, task_load(p));

	fixup_cum_window_demand(rq, task_load_delta);
}

u64 walt_ktime_clock(void)
{
	if (unlikely(walt_ktime_suspended))
		return ktime_to_ns(ktime_last);
	return ktime_get_ns();
}

static void walt_resume(void)
{
	walt_ktime_suspended = false;
}

static int walt_suspend(void)
{
	ktime_last = ktime_get();
	walt_ktime_suspended = true;
	return 0;
}

static struct syscore_ops walt_syscore_ops = {
	.resume	= walt_resume,
	.suspend = walt_suspend
};

static int __init walt_init_ops(void)
{
	register_syscore_ops(&walt_syscore_ops);
	return 0;
}
late_initcall(walt_init_ops);

#ifdef CONFIG_CFS_BANDWIDTH
void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *cfs_rq,
		struct task_struct *p)
{
	cfs_rq->cumulative_runnable_avg += p->ravg.demand;
}

void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *cfs_rq,
		struct task_struct *p)
{
	cfs_rq->cumulative_runnable_avg -= p->ravg.demand;
}
#endif

static int exiting_task(struct task_struct *p)
{
	if (p->flags & PF_EXITING) {
		if (p->ravg.sum_history[0] != EXITING_TASK_MARKER) {
			p->ravg.sum_history[0] = EXITING_TASK_MARKER;
		}
		return 1;
	}
	return 0;
}

static int __init set_walt_ravg_window(char *str)
{
	unsigned int adj_window;
	bool no_walt = walt_disabled;

	get_option(&str, &walt_ravg_window);

	/* Adjust for CONFIG_HZ */
	adj_window = (walt_ravg_window / TICK_NSEC) * TICK_NSEC;

	/* Warn if we're a bit too far away from the expected window size */
	WARN(adj_window < walt_ravg_window - NSEC_PER_MSEC,
	     "tick-adjusted window size %u, original was %u\n", adj_window,
	     walt_ravg_window);

	walt_ravg_window = adj_window;

	walt_disabled = walt_disabled ||
			(walt_ravg_window < MIN_SCHED_RAVG_WINDOW ||
			 walt_ravg_window > MAX_SCHED_RAVG_WINDOW);

	WARN(!no_walt && walt_disabled,
	     "invalid window size, disabling WALT\n");

	return 0;
}

early_param("walt_ravg_window", set_walt_ravg_window);

static void
update_window_start(struct rq *rq, u64 wallclock)
{
	s64 delta;
	int nr_windows;

	delta = wallclock - rq->window_start;
	/* If the MPM global timer is cleared, set delta as 0 to avoid kernel BUG happening */
	if (delta < 0) {
		delta = 0;
		WARN_ONCE(1, "WALT wallclock appears to have gone backwards or reset\n");
	}

	if (delta < walt_ravg_window)
		return;

	nr_windows = div64_u64(delta, walt_ravg_window);
	rq->window_start += (u64)nr_windows * (u64)walt_ravg_window;

	rq->cum_window_demand = rq->cumulative_runnable_avg;
}


static int cpu_is_waiting_on_io(struct rq *rq)
{
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
	if (!walt_io_is_busy && !sched_io_is_busy)
		return 0;
#else
	if (!walt_io_is_busy)
		return 0;
#endif
	return atomic_read(&rq->nr_iowait);
}

void walt_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags, nr_windows;
	u64 cur_jiffies_ts;

	raw_spin_lock_irqsave(&rq->lock, flags);

	/*
	 * cputime (wallclock) uses sched_clock so use the same here for
	 * consistency.
	 */
	delta += sched_clock() - wallclock;
	cur_jiffies_ts = get_jiffies_64();

	if (is_idle_task(curr))
		walt_update_task_ravg(curr, rq, IRQ_UPDATE, walt_ktime_clock(),
				 delta);

	nr_windows = cur_jiffies_ts - rq->irqload_ts;

	if (nr_windows) {
		if (nr_windows < 10) {
			/* Decay CPU's irqload by 3/4 for each window. */
			rq->avg_irqload *= (3 * nr_windows);
			rq->avg_irqload = div64_u64(rq->avg_irqload,
						    4 * nr_windows);
		} else {
			rq->avg_irqload = 0;
		}
		rq->avg_irqload += rq->cur_irqload;
		rq->cur_irqload = 0;
	}

	rq->cur_irqload += delta;
	rq->irqload_ts = cur_jiffies_ts;
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}


#ifdef CONFIG_HW_EAS_SCHED
#define WALT_HIGH_IRQ_TIMEOUT (20L * CONFIG_HZ / MSEC_PER_SEC)
#else
#define WALT_HIGH_IRQ_TIMEOUT 3
#endif

u64 walt_irqload(int cpu) {
	struct rq *rq = cpu_rq(cpu);
	s64 delta;
	delta = get_jiffies_64() - rq->irqload_ts;

        /*
	 * Current context can be preempted by irq and rq->irqload_ts can be
	 * updated by irq context so that delta can be negative.
	 * But this is okay and we can safely return as this means there
	 * was recent irq occurrence.
	 */

        if (delta < WALT_HIGH_IRQ_TIMEOUT)
		return rq->avg_irqload;
        else
		return 0;
}

int walt_cpu_high_irqload(int cpu) {
	return walt_irqload(cpu) >= sysctl_sched_walt_cpu_high_irqload;
}

static int account_busy_for_cpu_time(struct rq *rq, struct task_struct *p,
				     u64 irqtime, int event)
{
	if (is_idle_task(p)) {
		/* TASK_WAKE && TASK_MIGRATE is not possible on idle task! */
		if (event == PICK_NEXT_TASK)
			return 0;

		/* PUT_PREV_TASK, TASK_UPDATE && IRQ_UPDATE are left */
		return irqtime || cpu_is_waiting_on_io(rq);
	}

	if (event == TASK_WAKE)
		return 0;

	if (event == PUT_PREV_TASK || event == IRQ_UPDATE ||
					 event == TASK_UPDATE)
		return 1;

	/* Only TASK_MIGRATE && PICK_NEXT_TASK left */
	return walt_freq_account_wait_time;
}

#ifdef CONFIG_HW_RTG_WALT
#include "./hw_rtg/rtg_walt.c"
#endif

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
/* Prevent rasing sugov work in very low load case. */
static bool should_update_freq(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (rq->cluster->cur_freq > rq->cluster->min_freq)
		return true;

	if (boosted_freq_policy_util(cpu) >
	    (capacity_curr_of(cpu) >> 1))
		return true;

	if (sugov_iowait_boost_pending(cpu))
		return true;

	/*
	 * Lowest freq and low util and no iowait boost.
	 * No need to trigger freq update.
	 */
	return false;
}

static void
mark_util_change_for_rollover(struct task_struct *p, struct rq *rq)
{
	int cpu = cpu_of(rq);

	/* Rollover condition: p is curr task and p sees new window. */
	if (p != rq->curr)
		return;

#ifdef CONFIG_HW_SCHED_PRED_LOAD
	/* When pred load in use, mark PRED_LOAD_WINDOW_ROLLOVER only. */
	if (use_pred_load(cpu)) {
		if (p->ravg.mark_start < rq->predl_window_start) {
			if (should_update_freq(cpu))
				sugov_mark_util_change(cpu,
					PRED_LOAD_WINDOW_ROLLOVER);

			trace_predl_window_rollover(cpu);
		}

		return;
	}
#endif

	if (p->ravg.mark_start < rq->window_start) {
		if (should_update_freq(cpu))
			sugov_mark_util_change(cpu, WALT_WINDOW_ROLLOVER);

		trace_walt_window_rollover(cpu);
	}
}
#else
static void
mark_util_change_for_rollover(struct task_struct *p, struct rq *rq)
{
}
#endif

#ifdef CONFIG_HW_TOP_TASK
#include "hw_task/hw_top_task.c"
#endif
/*
 * Account cpu activity in its busy time counters (rq->curr/prev_runnable_sum)
 */
static void update_cpu_busy_time(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	int new_window, nr_full_windows = 0;
	int p_is_curr_task = (p == rq->curr);
	u64 mark_start = p->ravg.mark_start;
	u64 window_start = rq->window_start;
	u32 window_size = walt_ravg_window;
	u64 delta;
	u64 *curr_runnable_sum = &rq->curr_runnable_sum;
	u64 *prev_runnable_sum = &rq->prev_runnable_sum;
#ifdef CONFIG_HW_RTG_CPU_TIME
	struct group_cpu_time *cpu_time;
	struct related_thread_group *grp;
#endif

	new_window = mark_start < window_start;
	if (new_window) {
		nr_full_windows = div64_u64((window_start - mark_start),
						window_size);
		if (p->ravg.active_windows < USHRT_MAX)
			p->ravg.active_windows++;
#ifdef CONFIG_HW_TOP_TASK
		if (p_is_curr_task)
			rollover_top_task_table(rq, nr_full_windows);
#endif
	}

#ifdef CONFIG_HW_RTG_CPU_TIME
	rcu_read_lock();
	grp = task_related_thread_group(p);
	rcu_read_unlock();
	if (grp && grp->mode.freq_enabled && rq->cluster == grp->preferred_cluster) {
		cpu_time = group_update_cpu_time(rq, p->grp);
		if (cpu_time) {
			curr_runnable_sum = &cpu_time->curr_runnable_sum;
			prev_runnable_sum = &cpu_time->prev_runnable_sum;
		}
	}
#endif

	/* Handle per-task window rollover. We don't care about the idle
	 * task or exiting tasks. */
	if (new_window && !is_idle_task(p) && !exiting_task(p)) {
		u32 curr_window = 0;

		if (!nr_full_windows)
			curr_window = p->ravg.curr_window;

		p->ravg.prev_window = curr_window;
		p->ravg.curr_window = 0;
#ifdef CONFIG_HW_TOP_TASK
		rollover_top_task_load(p, nr_full_windows);
#endif
	}

	if (!account_busy_for_cpu_time(rq, p, irqtime, event)) {
		/* account_busy_for_cpu_time() = 0, so no update to the
		 * task's current window needs to be made. This could be
		 * for example
		 *
		 *   - a wakeup event on a task within the current
		 *     window (!new_window below, no action required),
		 *   - switching to a new task from idle (PICK_NEXT_TASK)
		 *     in a new window where irqtime is 0 and we aren't
		 *     waiting on IO */

		if (!new_window)
			return;

		/* A new window has started. The RQ demand must be rolled
		 * over if p is the current task. */
		if (p_is_curr_task) {
			u64 prev_sum = 0;

			/* p is either idle task or an exiting task */
			if (!nr_full_windows) {
				prev_sum = rq->curr_runnable_sum;
			}

			rq->prev_runnable_sum = prev_sum;
			rq->curr_runnable_sum = 0;
		}
#ifdef CONFIG_HW_TOP_TASK
		update_top_task_load(p, rq, event, wallclock, false);
		add_top_task(p, rq);
#endif

		return;
	}

	if (!new_window) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. No rollover
		 * since we didn't start a new window. An example of this is
		 * when a task starts execution and then sleeps within the
		 * same window. */

		if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq))
			delta = wallclock - mark_start;
		else
			delta = irqtime;
		delta = scale_exec_time(delta, rq);
		*curr_runnable_sum += delta;
		if (!is_idle_task(p) && !exiting_task(p)) {
			p->ravg.curr_window += delta;
#ifdef CONFIG_HW_TOP_TASK
			p->ravg.load_sum += delta;
#endif
		}

		return;
	}

#ifdef CONFIG_HW_TOP_TASK
	update_top_task_load(p, rq, event, wallclock, true);
	add_top_task(p, rq);
#endif

	if (!p_is_curr_task) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has also started, but p is not the current task, so the
		 * window is not rolled over - just split up and account
		 * as necessary into curr and prev. The window is only
		 * rolled over when a new window is processed for the current
		 * task.
		 *
		 * Irqtime can't be accounted by a task that isn't the
		 * currently running task. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!exiting_task(p))
				p->ravg.prev_window += delta;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!exiting_task(p))
				p->ravg.prev_window = delta;
		}
		*prev_runnable_sum += delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		*curr_runnable_sum += delta;
		if (!exiting_task(p))
			p->ravg.curr_window = delta;

		return;
	}

	if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq)) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. If any of these three above conditions are true
		 * then this busy time can't be accounted as irqtime.
		 *
		 * Busy time for the idle task or exiting tasks need not
		 * be accounted.
		 *
		 * An example of this would be a task that starts execution
		 * and then sleeps once a new window has begun. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window += delta;

			delta += rq->curr_runnable_sum;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window = delta;
		}
		/*
		 * Rollover for normal runnable sum is done here by overwriting
		 * the values in prev_runnable_sum and curr_runnable_sum.
		 * Rollover for new task runnable sum has completed by previous
		 * if-else statement.
		 */
		*prev_runnable_sum = delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		*curr_runnable_sum = delta;
		if (!is_idle_task(p) && !exiting_task(p))
			p->ravg.curr_window = delta;

		return;
	}

	if (irqtime) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. The current task must be the idle task because
		 * irqtime is not accounted for any other task.
		 *
		 * Irqtime will be accounted each time we process IRQ activity
		 * after a period of idleness, so we know the IRQ busy time
		 * started at wallclock - irqtime. */

		BUG_ON(!is_idle_task(p));
		mark_start = wallclock - irqtime;

		/* Roll window over. If IRQ busy time was just in the current
		 * window then that is all that need be accounted. */
		rq->prev_runnable_sum = rq->curr_runnable_sum;
		if (mark_start > window_start) {
			*curr_runnable_sum = scale_exec_time(irqtime, rq);
			return;
		}

		/* The IRQ busy time spanned multiple windows. Process the
		 * busy time preceding the current window start first. */
		delta = window_start - mark_start;
		if (delta > window_size)
			delta = window_size;
		delta = scale_exec_time(delta, rq);
		*prev_runnable_sum += delta;

		/* Process the remaining IRQ busy time in the current window. */
		delta = wallclock - window_start;
		*curr_runnable_sum = scale_exec_time(delta, rq);

		return;
	}

	BUG();
}

static int account_busy_for_task_demand(struct task_struct *p, int event)
{
	/* No need to bother updating task demand for exiting tasks
	 * or the idle task. */
	if (exiting_task(p) || is_idle_task(p))
		return 0;

	/* When a task is waking up it is completing a segment of non-busy
	 * time. Likewise, if wait time is not treated as busy time, then
	 * when a task begins to run or is migrated, it is not running and
	 * is completing a segment of non-busy time. */
	if (event == TASK_WAKE || (!walt_account_wait_time &&
			 (event == PICK_NEXT_TASK || event == TASK_MIGRATE)))
		return 0;

	return 1;
}

/*
 * Called when new window is starting for a task, to record cpu usage over
 * recently concluded window(s). Normally 'samples' should be 1. It can be > 1
 * when, say, a real-time task runs without preemption for several windows at a
 * stretch.
 */
static void update_history(struct rq *rq, struct task_struct *p,
			 u32 runtime, int samples, int event)
{
	u32 *hist = &p->ravg.sum_history[0];
	int ridx, widx;
	u32 max = 0, avg, demand;
	u64 sum = 0;

	/* Ignore windows where task had no activity */
	if (!runtime || is_idle_task(p) || exiting_task(p) || !samples)
			goto done;

	/* Push new 'runtime' value onto stack */
	widx = walt_ravg_hist_size - 1;
	ridx = widx - samples;
	for (; ridx >= 0; --widx, --ridx) {
		hist[widx] = hist[ridx];
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	for (widx = 0; widx < samples && widx < walt_ravg_hist_size; widx++) {
		hist[widx] = runtime;
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	p->ravg.sum = 0;

	if (walt_window_stats_policy == WINDOW_STATS_RECENT) {
		demand = runtime;
	} else if (walt_window_stats_policy == WINDOW_STATS_MAX) {
		demand = max;
	} else {
		avg = div64_u64(sum, walt_ravg_hist_size);
		if (walt_window_stats_policy == WINDOW_STATS_AVG)
			demand = avg;
		else
			demand = max(avg, runtime);
	}

	/*
	 * A throttled deadline sched class task gets dequeued without
	 * changing p->on_rq. Since the dequeue decrements hmp stats
	 * avoid decrementing it here again.
	 *
	 * When window is rolled over, the cumulative window demand
	 * is reset to the cumulative runnable average (contribution from
	 * the tasks on the runqueue). If the current task is dequeued
	 * already, it's demand is not included in the cumulative runnable
	 * average. So add the task demand separately to cumulative window
	 * demand.
	 */
	if (!task_has_dl_policy(p) || !p->dl.dl_throttled) {
		if (task_on_rq_queued(p))
			fixup_cumulative_runnable_avg(rq, p, demand);
		else if (rq->curr == p)
			fixup_cum_window_demand(rq, demand);
	}

	p->ravg.demand = demand;

done:
	trace_walt_update_history(rq, p, runtime, samples, event);
	return;
}

static void add_to_task_demand(struct rq *rq, struct task_struct *p,
				u64 delta)
{
	delta = scale_exec_time(delta, rq);
	p->ravg.sum += delta;
	if (unlikely(p->ravg.sum > walt_ravg_window))
		p->ravg.sum = walt_ravg_window;
}

#ifdef CONFIG_HW_TASK_RAVG_SUM
static int account_busy_for_task_ravg_sum(struct task_struct *p, int event)
{
	/* No need to bother updating task demand for exiting tasks
	 * or the idle task. */
	if (exiting_task(p) || is_idle_task(p))
		return 0;

	/* When a task is waking up it is completing a segment of non-busy
	 * time. Likewise, if wait time is not treated as busy time, then
	 * when a task begins to run or is migrated, it is not running and
	 * is completing a segment of non-busy time. */
	if (event == TASK_WAKE || event == PICK_NEXT_TASK || event == TASK_MIGRATE)
		return 0;

	return 1;
}

static void add_to_task_ravg_sum(struct rq *rq, struct task_struct *p,
				u64 delta, int event)
{
	if (!account_busy_for_task_ravg_sum(p, event))
		return ;

	delta = scale_exec_time(delta, rq);
	p->ravg.ravg_sum += delta;
}
#else
static inline void add_to_task_ravg_sum(struct rq *rq, struct task_struct *p, u64 delta, int event) {}
#endif

/*
 * Account cpu demand of task and/or update task's cpu demand history
 *
 * ms = p->ravg.mark_start;
 * wc = wallclock
 * ws = rq->window_start
 *
 * Three possibilities:
 *
 *	a) Task event is contained within one window.
 *		window_start < mark_start < wallclock
 *
 *		ws   ms  wc
 *		|    |   |
 *		V    V   V
 *		|---------------|
 *
 *	In this case, p->ravg.sum is updated *iff* event is appropriate
 *	(ex: event == PUT_PREV_TASK)
 *
 *	b) Task event spans two windows.
 *		mark_start < window_start < wallclock
 *
 *		ms   ws   wc
 *		|    |    |
 *		V    V    V
 *		-----|-------------------
 *
 *	In this case, p->ravg.sum is updated with (ws - ms) *iff* event
 *	is appropriate, then a new window sample is recorded followed
 *	by p->ravg.sum being set to (wc - ws) *iff* event is appropriate.
 *
 *	c) Task event spans more than two windows.
 *
 *		ms ws_tmp			   ws  wc
 *		|  |				   |   |
 *		V  V				   V   V
 *		---|-------|-------|-------|-------|------
 *		   |				   |
 *		   |<------ nr_full_windows ------>|
 *
 *	In this case, p->ravg.sum is updated with (ws_tmp - ms) first *iff*
 *	event is appropriate, window sample of p->ravg.sum is recorded,
 *	'nr_full_window' samples of window_size is also recorded *iff*
 *	event is appropriate and finally p->ravg.sum is set to (wc - ws)
 *	*iff* event is appropriate.
 *
 * IMPORTANT : Leave p->ravg.mark_start unchanged, as update_cpu_busy_time()
 * depends on it!
 */
static void update_task_demand(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock)
{
	u64 mark_start = p->ravg.mark_start;
	u64 delta, window_start = rq->window_start;
	int new_window, nr_full_windows;
	u32 window_size = walt_ravg_window;

#ifdef CONFIG_HW_RTG_WALT
	update_group_demand(p, rq, event, wallclock);
#endif
	new_window = mark_start < window_start;
	if (!account_busy_for_task_demand(p, event)) {
		if (new_window)
			/* If the time accounted isn't being accounted as
			 * busy time, and a new window started, only the
			 * previous window need be closed out with the
			 * pre-existing demand. Multiple windows may have
			 * elapsed, but since empty windows are dropped,
			 * it is not necessary to account those. */
			update_history(rq, p, p->ravg.sum, 1, event);
		return;
	}

	if (!new_window) {
		/* The simple case - busy time contained within the existing
		 * window. */
		add_to_task_demand(rq, p, wallclock - mark_start);
		add_to_task_ravg_sum(rq, p, wallclock - mark_start, event);
		return;
	}

	/* Busy time spans at least two windows. Temporarily rewind
	 * window_start to first window boundary after mark_start. */
	delta = window_start - mark_start;
	nr_full_windows = div64_u64(delta, window_size);
	window_start -= (u64)nr_full_windows * (u64)window_size;

	/* Process (window_start - mark_start) first */
	add_to_task_demand(rq, p, window_start - mark_start);
	add_to_task_ravg_sum(rq, p, wallclock - mark_start, event);

	/* Push new sample(s) into task's demand history */
	update_history(rq, p, p->ravg.sum, 1, event);
	if (nr_full_windows)
		update_history(rq, p, scale_exec_time(window_size, rq),
			       nr_full_windows, event);

	/* Roll window_start back to current to process any remainder
	 * in current window. */
	window_start += (u64)nr_full_windows * (u64)window_size;

	/* Process (wallclock - window_start) next */
	mark_start = window_start;
	add_to_task_demand(rq, p, wallclock - mark_start);
	add_to_task_ravg_sum(rq, p, wallclock - mark_start, event);
}

/* Reflect task activity on its demand and cpu's busy time statistics */
void walt_update_task_ravg(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	if (walt_disabled || !rq->window_start)
		return;

	/* there's a bug here - there are many cases where
	 * we enter here without holding this lock, coming from
	 * walt_fixup_busy_time - looks like in 4.14 we don't
	 * hold the dest_rq at time of migration, but I haven't
	 * yet worked out if it is safe to always lock dest_rq there.
	 *
	 * temporarily disable this assert to continue checking the
	 * rest of the locking here.
	 */
	//lockdep_assert_held(&rq->lock);

	update_window_start(rq, wallclock);
#ifdef CONFIG_HW_SCHED_PRED_LOAD
	update_predl_window_start(rq, wallclock);
#endif

#ifdef CONFIG_HW_RTG_WALT
	update_group_nr_running(p, event, wallclock);
#endif

	if (!p->ravg.mark_start)
		goto done;

	update_task_demand(p, rq, event, wallclock);
#ifdef CONFIG_HW_SCHED_PRED_LOAD
	update_task_predl(p, rq, event, wallclock);
#endif
	update_cpu_busy_time(p, rq, event, wallclock, irqtime);

	mark_util_change_for_rollover(p, rq);

done:
	trace_walt_update_task_ravg(p, rq, event, wallclock, irqtime);

	p->ravg.mark_start = wallclock;
}

static void reset_task_stats(struct task_struct *p)
{
	u32 sum = 0;

	if (exiting_task(p))
		sum = EXITING_TASK_MARKER;

	memset(&p->ravg, 0, sizeof(struct ravg));
	/* Retain EXITING_TASK marker */
	p->ravg.sum_history[0] = sum;
}

void walt_mark_task_starting(struct task_struct *p)
{
	u64 wallclock;
	struct rq *rq = task_rq(p);

	if (!rq->window_start) {
		reset_task_stats(p);
		return;
	}

	/*
	 * Add the new task to top tasks.
	 * Called in wake_up_new_task(), after task load initialized.
	 */
#ifdef CONFIG_HW_TOP_TASK
	add_top_task(p, rq);
#endif

	wallclock = walt_ktime_clock();
	p->ravg.mark_start = wallclock;
#ifdef CONFIG_HW_ED_TASK
	p->last_wake_ts = wallclock;
#endif
}

void walt_set_window_start(struct rq *rq, struct rq_flags *rf)
{
	if (likely(rq->window_start))
		return;

	if (cpu_of(rq) == sync_cpu) {
		rq->window_start = 1;
#ifdef CONFIG_HW_SCHED_PRED_LOAD
		rq->predl_window_start = 1;
#endif
	} else {
		struct rq *sync_rq = cpu_rq(sync_cpu);
		rq_unpin_lock(rq, rf);
		double_lock_balance(rq, sync_rq);
		rq->window_start = sync_rq->window_start;
#ifdef CONFIG_HW_SCHED_PRED_LOAD
		rq->predl_window_start = sync_rq->predl_window_start;
#endif
		rq->curr_runnable_sum = rq->prev_runnable_sum = 0;
		raw_spin_unlock(&sync_rq->lock);
		rq_repin_lock(rq, rf);
	}

	rq->curr->ravg.mark_start = rq->window_start;
}

void walt_migrate_sync_cpu(int cpu)
{
	if (cpu == sync_cpu)
		sync_cpu = smp_processor_id();
}

#ifdef CONFIG_HW_MIGRATION_NOTIFY
#include "hw_migration_notify/migration_notify_hw.c"
#endif

void walt_fixup_busy_time(struct task_struct *p, int new_cpu)
{
	struct rq *src_rq = task_rq(p);
	struct rq *dest_rq = cpu_rq(new_cpu);
	u64 wallclock;

#if defined(CONFIG_HW_TOP_TASK) || defined(CONFIG_HW_MIGRATION_NOTIFY)
	int src_cpu = task_cpu(p);
#endif

	if (!p->on_rq && p->state != TASK_WAKING)
		return;

	if (exiting_task(p)) {
#ifdef CONFIG_HW_ED_TASK
		clear_ed_task(p, src_rq);
#endif
		return;
	}

	if (p->state == TASK_WAKING)
		double_rq_lock(src_rq, dest_rq);

	wallclock = walt_ktime_clock();

//#define LOCK_CONDITION(rq) (debug_locks && !lockdep_is_held(&rq->lock))
//	WARN(LOCK_CONDITION(task_rq(p)), "task_rq(p) not held. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);
//	WARN(LOCK_CONDITION(dest_rq), "dest_rq not held. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);

	/*
	 * It seems that in lots of cases we don't have
	 * dest_rq locked when we get here, which means
	 * we can't be sure to the WALT stats - someone
	 * needs to fix this.
	 */
	walt_update_task_ravg(task_rq(p)->curr, task_rq(p),
			TASK_UPDATE, wallclock, 0);
	walt_update_task_ravg(dest_rq->curr, dest_rq,
			TASK_UPDATE, wallclock, 0);

//	WARN(LOCK_CONDITION(task_rq(p)), "task_rq(p) not held after rq update. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);
	walt_update_task_ravg(p, task_rq(p), TASK_MIGRATE, wallclock, 0);

	/*
	 * When a task is migrating during the wakeup, adjust
	 * the task's contribution towards cumulative window
	 * demand.
	 */
	if (p->state == TASK_WAKING &&
	    p->last_sleep_ts >= src_rq->window_start) {
		fixup_cum_window_demand(src_rq, -(s64)p->ravg.demand);
		fixup_cum_window_demand(dest_rq, p->ravg.demand);
	}

	if (p->ravg.curr_window) {
		src_rq->curr_runnable_sum -= p->ravg.curr_window;
		dest_rq->curr_runnable_sum += p->ravg.curr_window;
	}

	if (p->ravg.prev_window) {
		src_rq->prev_runnable_sum -= p->ravg.prev_window;
		dest_rq->prev_runnable_sum += p->ravg.prev_window;
	}

	if ((s64)src_rq->prev_runnable_sum < 0) {
		src_rq->prev_runnable_sum = 0;
		WARN_ON(1);
	}
	if ((s64)src_rq->curr_runnable_sum < 0) {
		src_rq->curr_runnable_sum = 0;
		WARN_ON(1);
	}

	trace_walt_migration_update_sum(src_rq, p);
	trace_walt_migration_update_sum(dest_rq, p);
#ifdef CONFIG_HW_TOP_TASK
	if (same_freq_domain(new_cpu, src_cpu)) {
		/* Only need to migrate top task when same cluster */
		migrate_top_task(p, src_rq, dest_rq);
	}
#endif

#ifdef CONFIG_HW_MIGRATION_NOTIFY
	if (!same_freq_domain(new_cpu, src_cpu))
		inter_cluster_migration_fixup(p, src_rq, dest_rq);
#endif

#ifdef CONFIG_HW_ED_TASK
	migrate_ed_task(p, src_rq, dest_rq, wallclock);
#endif
	if (p->state == TASK_WAKING)
		double_rq_unlock(src_rq, dest_rq);
}

void walt_init_new_task_load(struct task_struct *p)
{
	int i;
	u32 init_load_windows =
			div64_u64((u64)sysctl_sched_walt_init_task_load_pct *
                          (u64)walt_ravg_window, 100);
	u32 init_load_pct = current->init_load_pct;

	p->init_load_pct = 0;
	memset(&p->ravg, 0, sizeof(struct ravg));
#ifdef CONFIG_HW_RTG_WALT
	init_task_rtg(p);
#endif
	if (init_load_pct) {
		init_load_windows = div64_u64((u64)init_load_pct *
			  (u64)walt_ravg_window, 100);
	}

	p->ravg.demand = init_load_windows;
	for (i = 0; i < RAVG_HIST_SIZE_MAX; ++i)
		p->ravg.sum_history[i] = init_load_windows;
}

#ifdef CONFIG_HW_SCHED_PRED_LOAD
#include "hw_walt/pred_load.c"
#endif

#ifdef CONFIG_HW_SCHED_WALT
#include "hw_walt/walt_hw.c"
#endif
