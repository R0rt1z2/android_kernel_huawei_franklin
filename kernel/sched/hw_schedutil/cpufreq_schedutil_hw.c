/*
 * cpufreq_schedutil_hw.c
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "walt.h"
#include <linux/sched/hw_rtg/rtg_sched.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_schedutil.h>
#include "../hw_walt/pred_load.h"

#ifdef CONFIG_HW_ED_TASK
#include "../hw_task/hw_ed_task.h"
#endif

#include "../hw_task/hw_top_task.h"

#ifdef CONFIG_HW_SCHED_CLUSTER
#include "../hw_cluster/sched_cluster.h"
#endif

#include "../hw_task/hw_top_task.h"

enum {
	FREQ_STAT_CPU_LOAD		= (1 << 0),
	FREQ_STAT_TOP_TASK		= (1 << 1),
	FREQ_STAT_PRED_LOAD		= (1 << 2),
	FREQ_STAT_MAX_PRED_LS		= (1 << 3),
	FREQ_STAT_PRED_CPU_LOAD_MIN	= (1 << 4),
};

/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 90
unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};

#define DEFAULT_RATE_LIMIT_US (79 * USEC_PER_MSEC)
unsigned int default_above_hispeed_delay[] = {
		DEFAULT_RATE_LIMIT_US };
unsigned int default_min_sample_time[] = {
		DEFAULT_RATE_LIMIT_US };

#define FREQ_STAT_USE_PRED_WINDOW (FREQ_STAT_PRED_LOAD |\
				FREQ_STAT_MAX_PRED_LS |\
				FREQ_STAT_PRED_CPU_LOAD_MIN)

__read_mostly unsigned int sched_io_is_busy = 0;

unsigned long freq_boosted(unsigned long util, int cpu);

void sched_set_io_is_busy(int val)
{
	sched_io_is_busy = val;
}

static BLOCKING_NOTIFIER_HEAD(sugov_status_notifier_list);

int sugov_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&sugov_status_notifier_list, nb);
}

int sugov_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&sugov_status_notifier_list, nb);
}

/* defaut value for sugov_tunables */
#define DEFAULT_OVERLOAD_DURATION_MS	250

#define DEFAULT_GO_HISPEED_LOAD 99

#define DEFAULT_BOOSTPULSE_DURATION	(80 * USEC_PER_MSEC)

#define DEFAULT_MIN_BOOSTPULSE_INTERVAL (500 * USEC_PER_MSEC)

#define DEFAULT_TIMER_SLACK (80 * USEC_PER_MSEC)

#define DEFAULT_FREQ_REPORTING_POLICY \
			(FREQ_STAT_CPU_LOAD | FREQ_STAT_TOP_TASK)

#define IOWAIT_BOOST_INC_STEP 200000 /* 200MHz */
#define IOWAIT_BOOST_CLEAR_NS 8000000 /* 8ms */

static inline bool use_pelt(void)
{
	return use_pelt_freq();
}

static inline unsigned int get_freq_reporting_policy(int cpu)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_cpu_hw *sg_cpu_hw = &sg_cpu->sg_cpu_hw;
	struct sugov_policy *sg_policy = NULL;
	struct sugov_policy_hw *sg_policy_hw = NULL;
	struct sugov_tunables_hw *tunables_hw = NULL;
	unsigned int policy;

	if (!sg_cpu_hw->enabled)
		return DEFAULT_FREQ_REPORTING_POLICY;

	sg_policy = sg_cpu->sg_policy;
	sg_policy_hw = &sg_policy->sg_policy_hw;
	if (!sg_policy || !sg_policy_hw->governor_enabled)
		return DEFAULT_FREQ_REPORTING_POLICY;

	tunables_hw = &sg_policy->tunables->tunables_hw;
	policy = tunables_hw->freq_reporting_policy;

#ifdef CONFIG_HW_SCHED_PRED_LOAD
	if (!predl_enable &&
	    (policy & FREQ_STAT_USE_PRED_WINDOW))
		return DEFAULT_FREQ_REPORTING_POLICY;
#endif

	return policy;
}

#ifdef CONFIG_HW_SCHED_PRED_LOAD
bool use_pred_load(int cpu)
{
	if (!predl_enable)
		return false;

	return !!(get_freq_reporting_policy(cpu) & FREQ_STAT_USE_PRED_WINDOW);
}
#endif

static unsigned int freq_to_targetload(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;
	struct sugov_tunables_hw *tunables_hw = &tunables->tunables_hw;

	spin_lock_irqsave(&tunables_hw->target_loads_lock, flags);

	for (i = 0; i < tunables_hw->ntarget_loads - 1 &&
	     freq >= tunables_hw->target_loads[i + 1]; i += 2)
		;

	ret = tunables_hw->target_loads[i];
	spin_unlock_irqrestore(&tunables_hw->target_loads_lock, flags);
	return ret;
}

static unsigned int freq_to_above_hispeed_delay(struct sugov_tunables *tunables,
						unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;
	struct sugov_tunables_hw *tunables_hw = &tunables->tunables_hw;

	spin_lock_irqsave(&tunables_hw->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables_hw->nabove_hispeed_delay - 1 &&
	     freq >= tunables_hw->above_hispeed_delay[i + 1]; i += 2)
		;

	ret = tunables_hw->above_hispeed_delay[i];
	spin_unlock_irqrestore(&tunables_hw->above_hispeed_delay_lock, flags);
	return ret;
}

static unsigned int freq_to_min_sample_time(struct sugov_tunables *tunables,
					    unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;
	struct sugov_tunables_hw *tunables_hw = &tunables->tunables_hw;

	spin_lock_irqsave(&tunables_hw->min_sample_time_lock, flags);

	for (i = 0; i < tunables_hw->nmin_sample_time - 1 &&
	     freq >= tunables_hw->min_sample_time[i + 1]; i += 2)
		;

	ret = tunables_hw->min_sample_time[i];
	spin_unlock_irqrestore(&tunables_hw->min_sample_time_lock, flags);
	return ret;
}

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */
static unsigned int choose_freq(struct sugov_policy *sg_policy,
				unsigned int loadadjfreq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	unsigned int prevfreq, freqmin = 0, freqmax = UINT_MAX, tl;
	unsigned int freq = policy->cur;
	int index;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(sg_policy->tunables, freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		index = cpufreq_frequency_table_target(policy, loadadjfreq / tl,
			    CPUFREQ_RELATION_L);

		freq = freq_table[index].frequency;

		if (freq > prevfreq) {
			/* The previous frequency is too low */
			freqmin = prevfreq;

			if (freq < freqmax)
				continue;

			/* Find highest frequency that is less than freqmax */
			index = cpufreq_frequency_table_target(policy,
				    freqmax - 1, CPUFREQ_RELATION_H);

			freq = freq_table[index].frequency;

			if (freq == freqmin) {
				/*
				 * The first frequency below freqmax has already
				 * been found to be too low. freqmax is the
				 * lowest speed we found that is fast enough.
				 */
				freq = freqmax;
				break;
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq > freqmin)
				continue;

			/* Find lowest frequency that is higher than freqmin */
			index = cpufreq_frequency_table_target(policy,
				    freqmin + 1, CPUFREQ_RELATION_L);

			freq = freq_table[index].frequency;

			/*
			 * If freqmax is the first frequency above
			 * freqmin then we have already found that
			 * this speed is fast enough.
			 */
			if (freq == freqmax)
				break;
		}

		trace_cpufreq_schedutil_choose_freq(tl, loadadjfreq, freq);
		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

	return freq;
}

/* Re-evaluate load to see if a frequency change is required or not */
static unsigned int eval_target_freq(struct sugov_policy *sg_policy,
				     unsigned long util, unsigned long max)
{
	u64 now;
	int cpu_load = 0;
	unsigned int new_freq;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct sugov_tunables_hw *tunables_hw = &tunables->tunables_hw;
	struct cpufreq_policy *policy = sg_policy->policy;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	unsigned int min_util_freq = 0;
	unsigned int rtg_util = 0;
	unsigned int rtg_freq = 0;

	now = ktime_to_us(ktime_get());
	tunables_hw->boosted = tunables_hw->boost ||
			    now < tunables_hw->boostpulse_endtime;

	if (tunables_hw->boosted && policy->cur < tunables_hw->hispeed_freq) {
		new_freq = tunables_hw->hispeed_freq;
	} else {
		cpu_load = util * 100 / capacity_curr_of(policy->cpu);
		new_freq = choose_freq(sg_policy, cpu_load * policy->cur);

		if ((cpu_load >= tunables_hw->go_hispeed_load ||
		     tunables_hw->boosted) &&
		     new_freq < tunables_hw->hispeed_freq)
			new_freq = tunables_hw->hispeed_freq;
	}

	new_freq = max(sg_policy_hw->iowait_boost, new_freq);
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	new_freq = max(sg_policy->rtg_freq, new_freq);
	rtg_util = sg_policy->rtg_util;
	rtg_freq = sg_policy->rtg_freq;
#endif

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	min_util_freq = util_to_freq(policy->cpu, sg_policy_hw->min_util);
	new_freq = max(min_util_freq, new_freq);
#endif

	trace_cpufreq_schedutil_eval_target(sg_policy_hw->max_cpu,
					    util, rtg_util,
					    capacity_curr_of(policy->cpu),
					    max, cpu_load,
					    policy->cur, sg_policy_hw->iowait_boost,
					    rtg_freq, min_util_freq, new_freq);

	return new_freq;
}

static void sugov_slack_timer_resched(struct sugov_policy *sg_policy)
{
	u64 expires;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	struct sugov_tunables_hw *tunables_hw =
		&sg_policy->tunables->tunables_hw;

	raw_spin_lock(&sg_policy_hw->timer_lock);
	if (!sg_policy_hw->governor_enabled)
		goto unlock;

	del_timer(&sg_policy_hw->pol_slack_timer);
	if (tunables_hw->timer_slack_val >= 0 &&
	    sg_policy->next_freq > sg_policy->policy->min) {
		expires = jiffies +
			usecs_to_jiffies(tunables_hw->timer_slack_val);
		sg_policy_hw->pol_slack_timer.expires = expires;
		add_timer_on(&sg_policy_hw->pol_slack_timer,
				sg_policy_hw->trigger_cpu);
	}
unlock:
	raw_spin_unlock(&sg_policy_hw->timer_lock);
}

/************************HUAWEI pull function from orgin **********************/
static bool hw_sugov_should_update_freq(struct sugov_policy *sg_policy,
					u64 time)
{
	s64 delta_ns;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;

	struct cpufreq_policy *policy = sg_policy->policy;

	if (policy->governor != &schedutil_gov ||
		!policy->governor_data)
		return false;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-cpu data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-cpu
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * For the slow switching platforms, the kthread is always scheduled on
	 * the right set of CPUs and any CPU can find the next frequency and
	 * schedule the kthread.
	 */
	if (sg_policy->policy->fast_switch_enabled &&
	    !cpufreq_can_do_remote_dvfs(sg_policy->policy))
		return false;

	if (sg_policy->work_in_progress)
		return false;

	if (!sg_policy_hw->governor_enabled)
		return false;

	if (!sg_policy->util_changed)
		return false;

	if (likely(!use_pelt()))
		return true;

	if (atomic_read(&sg_policy_hw->skip_hispeed_logic) ||
	    atomic_read(&sg_policy_hw->skip_min_sample_time))
		return true;

	delta_ns = time - sg_policy_hw->time;

	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static void hw_sugov_update_commit(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;

	sg_policy_hw->time = time;
	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (!next_freq)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

static void hw_sugov_set_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
				      unsigned int flags)
{
	unsigned int max_boost;
	struct sugov_tunables_hw *tunables_hw = NULL;
	struct sugov_policy_hw *sg_policy_hw = &sg_cpu->sg_policy->sg_policy_hw;

	if ((flags & SCHED_CPUFREQ_IOWAIT) ||
			walt_cpu_overload_irqload(sg_cpu->cpu)) {
		sg_policy_hw->last_iowait = time;
		sg_cpu->iowait_boost_pending = true;

		/*
		 * Boost FAIR tasks only up to the CPU clamped utilization.
		 *
		 * Since DL tasks have a much more advanced bandwidth control,
		 * it's safe to assume that IO boost does not apply to
		 * those tasks.
		 * Instead, since RT tasks are currently not utiliation clamped,
		 * we don't want to apply clamping on IO boost while there is
		 * blocked RT utilization.
		 */
		max_boost = sg_cpu->iowait_boost_max;
		max_boost = uclamp_util(cpu_rq(sg_cpu->cpu), max_boost);

		if (sg_cpu->iowait_boost) {
			/*
			 * Ignore pending and increase iowait_boost every time
			 * in a smooth way.
			 */
			tunables_hw = &sg_cpu->sg_policy->tunables->tunables_hw;
			sg_cpu->iowait_boost +=
				(sched_io_is_busy != 0) ?
				(tunables_hw->iowait_boost_step >> 1) :
				tunables_hw->iowait_boost_step;
			if (sg_cpu->iowait_boost > max_boost)
				sg_cpu->iowait_boost = max_boost;
		} else {
			sg_cpu->iowait_boost = sg_cpu->sg_policy->policy->min;
		}
	} else if (sg_cpu->iowait_boost) {
		/*
		 * For WALT, (time - sg_cpu->last_update) doesn't imply any
		 * information about how long the CPU has been idle.
		 * Use delta time from cluster's last SCHED_CPUFREQ_IOWAIT
		 * instead.
		 */
		s64 delta_ns = time - sg_policy_hw->last_iowait;

		if (delta_ns > IOWAIT_BOOST_CLEAR_NS) {
			sg_cpu->iowait_boost = 0;
			sg_cpu->iowait_boost_pending = false;
		}
	}
}

static void hw_sugov_iowait_boost(struct sugov_cpu *sg_cpu, unsigned long *util,
				  unsigned long *max)
{
	struct sugov_policy_hw *sg_policy_hw = &sg_cpu->sg_policy->sg_policy_hw;

	if (!sg_cpu->iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost_pending = false;
	} else {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < sg_cpu->sg_policy->policy->min) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	if (sg_cpu->iowait_boost > sg_policy_hw->iowait_boost)
		sg_policy_hw->iowait_boost = sg_cpu->iowait_boost;
}

/************ HUAWEI pull function from orgin basic framework******************/
static void sugov_nop_timer(unsigned long data)
{
}

static void hw_sugov_start(struct sugov_policy *sg_policy,
			   struct cpufreq_policy *policy)
{
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	struct sugov_tunables_hw *tunables_hw =
		&sg_policy->tunables->tunables_hw;

	sg_policy_hw->overload_duration_ns =
		tunables_hw->overload_duration * NSEC_PER_MSEC;
	sg_policy_hw->floor_validate_time = 0;
	sg_policy_hw->hispeed_validate_time = 0;
	/* allow freq drop as soon as possible when policy->min restored */
	sg_policy_hw->min_freq = policy->min;
	atomic_set(&sg_policy_hw->skip_min_sample_time, 0);
	atomic_set(&sg_policy_hw->skip_hispeed_logic, 0);
	sg_policy->util_changed = false;
	sg_policy_hw->time = 0;
	sg_policy_hw->last_iowait = 0;

	sg_policy->next_freq = policy->cur;
	sg_policy_hw->trigger_cpu = policy->cpu;
	raw_spin_lock(&sg_policy_hw->timer_lock);
	init_timer(&sg_policy_hw->pol_slack_timer);
	sg_policy_hw->pol_slack_timer.function = sugov_nop_timer;
	add_timer_on(&sg_policy_hw->pol_slack_timer, policy->cpu);
	sg_policy_hw->governor_enabled = true;
	raw_spin_unlock(&sg_policy_hw->timer_lock);

#ifdef CONFIG_HW_CPU_FREQ_LOCK_DETECT
	sg_policy_hw->start_time = ktime_get();
	sg_policy_hw->end_time = sg_policy_hw->start_time;
	sg_policy_hw->locked = false;
#endif

	blocking_notifier_call_chain(&sugov_status_notifier_list,
		SUGOV_START, &policy->cpu);
}

static void hw_sugov_stop(struct sugov_policy *sg_policy,
			  struct cpufreq_policy *policy)
{
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;

	blocking_notifier_call_chain(&sugov_status_notifier_list,
		SUGOV_STOP, &policy->cpu);

	raw_spin_lock(&sg_policy_hw->timer_lock);
	sg_policy_hw->governor_enabled = false;
	del_timer_sync(&sg_policy_hw->pol_slack_timer);
	raw_spin_unlock(&sg_policy_hw->timer_lock);
}

static void hw_sugov_limits(struct sugov_policy *sg_policy,
			    struct cpufreq_policy *policy)
{
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
#ifdef CONFIG_HW_CPU_FREQ_LOCK_DETECT
	u64 delta;
	ktime_t now;

	if (policy->min == policy->cpuinfo.max_freq &&
	    policy->min > sg_policy_hw->min_freq) {
		sg_policy_hw->start_time = ktime_get();
		sg_policy_hw->locked = true;
	} else if (sg_policy_hw->locked && policy->min < policy->max) {
		now = ktime_get();
		delta = ktime_to_ns(ktime_sub(now, sg_policy_hw->start_time));
		if (delta >= NSEC_PER_SEC)
			sg_policy_hw->end_time = now;

		sg_policy_hw->locked = false;
	}
#endif

	/* policy->min restored */
	if (sg_policy_hw->min_freq > policy->min) {
		sugov_mark_util_change(policy->cpu, POLICY_MIN_RESTORE);
		sugov_check_freq_update(policy->cpu);
	}

	sg_policy_hw->min_freq = policy->min;
}

static inline unsigned long freq_policy_util(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned int reporting_policy = get_freq_reporting_policy(cpu);
	unsigned long util = 0;

	/* MTK use get_cpu_util */
	if (reporting_policy & FREQ_STAT_CPU_LOAD)
		util = max(util, get_cpu_util(cpu));

	if (reporting_policy & FREQ_STAT_TOP_TASK)
		util = max(util, top_task_util(rq));

	if (reporting_policy & FREQ_STAT_PRED_LOAD)
		util = max(util, predict_util(rq));

	if (reporting_policy & FREQ_STAT_MAX_PRED_LS)
		util = max(util, max_pred_ls(rq));

	if (reporting_policy & FREQ_STAT_PRED_CPU_LOAD_MIN)
		util = max(util, cpu_util_pred_min(rq));

	return util;
}

/************************ HUAWEI SCHEDUTIL APIs ***********************/
unsigned long boosted_freq_policy_util(int cpu)
{
	unsigned long util;

	util = freq_policy_util(cpu);

	return freq_boosted(util, cpu);
}

static void sugov_update_util(int cpu, u64 time,
		unsigned int early_detection)
{
	unsigned long max_cap, util;

	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

	max_cap = capacity_orig_of(cpu);

	if (early_detection) {
		util = max_cap;
	} else {
		util = boosted_freq_policy_util(cpu);

		util = min(util, max_cap);
	}

	sg_cpu->util = util;
	sg_cpu->max = max_cap;
}

bool sugov_iowait_boost_pending(int cpu)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_cpu_hw *sg_cpu_hw = &sg_cpu->sg_cpu_hw;

	return sg_cpu_hw->enabled && sg_cpu->iowait_boost_pending;
}

unsigned int check_freq_reporting_policy(int cpu, unsigned int flags)
{
	unsigned int reporting_policy = get_freq_reporting_policy(cpu);
	unsigned int ignore_events = 0;

	if (reporting_policy & FREQ_STAT_USE_PRED_WINDOW)
		ignore_events |= WALT_WINDOW_ROLLOVER;
	else
		ignore_events |= (PRED_LOAD_WINDOW_ROLLOVER |
				PRED_LOAD_CHANGE | PRED_LOAD_ENQUEUE);

	if (!(reporting_policy & FREQ_STAT_TOP_TASK))
		ignore_events |= ADD_TOP_TASK;

	if (!(reporting_policy & FREQ_STAT_PRED_LOAD))
		ignore_events |= PRED_LOAD_ENQUEUE;

	return flags & (~ignore_events);
}

/********************** cpufreq governor interface *********************/
static inline bool sugov_cpu_is_overload(struct sugov_cpu *sg_cpu, u64 time)
{
	u64 idle_time, delta;
	struct sugov_policy_hw *sg_policy_hw;
	struct sugov_cpu_hw *sg_cpu_hw;

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	if (cpu_rq(sg_cpu->cpu)->skip_overload_detect)
		return false;
#endif

	idle_time = get_cpu_idle_time(sg_cpu->cpu, NULL, 0);
	sg_policy_hw = &sg_cpu->sg_policy->sg_policy_hw;
	sg_cpu_hw = &sg_cpu->sg_cpu_hw;
	if (sg_cpu_hw->last_idle_time != idle_time ||
	    sg_cpu_hw->idle_update_ts > time)
		sg_cpu_hw->idle_update_ts = time;

	sg_cpu_hw->last_idle_time = idle_time;
	delta = time - sg_cpu_hw->idle_update_ts;

	return (delta > sg_policy_hw->overload_duration_ns);
}

static unsigned int sugov_next_freq_shared_policy(struct sugov_policy
						  *sg_policy, u64 time,
						  unsigned int *early_detection)
{
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int overload_detection;
	int j, i = 0;

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	int max_min_util = 0;
#endif

	sg_policy_hw->max_cpu = cpumask_first(policy->cpus);
	sg_policy_hw->iowait_boost = 0;
	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max, j_min_util;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - sg_policy_hw->last_iowait;

		if (delta_ns > IOWAIT_BOOST_CLEAR_NS) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
		}

		overload_detection = sugov_cpu_is_overload(j_sg_cpu, time);
		sugov_update_util(j, time,
				  early_detection[i] | overload_detection);

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
			sg_policy_hw->max_cpu = j;
		}

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
		j_min_util = get_min_util(cpu_rq(j));
		if (j_min_util > max_min_util)
			max_min_util = j_min_util;
#endif

		hw_sugov_iowait_boost(j_sg_cpu, &util, &max);

		trace_cpufreq_schedutil_get_util(j, j_sg_cpu->util,
						 j_sg_cpu->max,
						 get_cpu_util(j),
						 top_task_util(cpu_rq(j)),
						 predict_util(cpu_rq(j)),
						 max_pred_ls(cpu_rq(j)),
						 cpu_util_pred_min(cpu_rq(j)),
						 j_sg_cpu->iowait_boost,
						 j_sg_cpu->flags,
						 early_detection[i],
						 overload_detection);

		j_sg_cpu->flags = 0;

		i++;
	}

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	sched_get_max_group_util(policy->cpus, &sg_policy->rtg_util, &sg_policy->rtg_freq);
	util = max(sg_policy->rtg_util, util);
#endif

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	sg_policy_hw->min_util = max_min_util;
#endif
	/*
	 * eval_target_freq equals to get_next_freq
	 */
	return eval_target_freq(sg_policy, util, max);
}

static bool sugov_time_limit(struct sugov_policy *sg_policy,
			     unsigned int next_freq, int skip_min_sample_time,
			     int skip_hispeed_logic)
{
	u64 delta_ns;
	unsigned int min_sample_time;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	struct sugov_tunables_hw *tunables_hw =
		&sg_policy->tunables->tunables_hw;

	if (!skip_hispeed_logic &&
	    next_freq > sg_policy->next_freq &&
	    sg_policy->next_freq >= tunables_hw->hispeed_freq) {
		delta_ns = sg_policy_hw->time -
			sg_policy_hw->hispeed_validate_time;
		if (delta_ns < NSEC_PER_USEC *
		    freq_to_above_hispeed_delay(sg_policy->tunables,
			    sg_policy->next_freq)) {
			trace_cpufreq_schedutil_notyet(sg_policy_hw->max_cpu,
						       "above_hispeed_delay",
						       delta_ns,
						       sg_policy->next_freq,
						       next_freq);
			return true;
		}
	}

	sg_policy_hw->hispeed_validate_time = sg_policy_hw->time;
	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */

	if (next_freq < sg_policy->next_freq) {
		min_sample_time = freq_to_min_sample_time(sg_policy->tunables,
							  sg_policy->next_freq);

#ifdef CONFIG_HW_CPU_FREQ_LOCK_DETECT
		if (ktime_to_ns(ktime_sub(ktime_get(), sg_policy_hw->end_time))
				< LONGER_MIN_SAMPLE_TIME_ELAPSED_DURATION) {
			min_sample_time = (min_sample_time >
					DEFAULT_RATE_LIMIT_US) ?
					min_sample_time : DEFAULT_RATE_LIMIT_US;
			skip_min_sample_time = 0;
		}
#endif

		if (!skip_min_sample_time) {
			delta_ns = sg_policy_hw->time -
				sg_policy_hw->floor_validate_time;
			if (delta_ns < NSEC_PER_USEC * min_sample_time) {
				trace_cpufreq_schedutil_notyet(
							sg_policy_hw->max_cpu,
							"min_sample_time",
							delta_ns,
							sg_policy->next_freq,
							next_freq);
				return true;
			}
		}
	}

	if (!tunables_hw->boosted ||
	    next_freq > tunables_hw->hispeed_freq)
		sg_policy_hw->floor_validate_time = sg_policy_hw->time;

	return false;
}

static void hw_sugov_boost(struct gov_attr_set *attr_set)
{
	struct sugov_policy *sg_policy = NULL;
	struct sugov_tunables_hw *tunables_hw = NULL;
	u64 now;

	now = use_pelt() ? ktime_get_ns() : walt_ktime_clock();

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		tunables_hw = &sg_policy->tunables->tunables_hw;
		hw_sugov_update_commit(sg_policy, now,
				       tunables_hw->hispeed_freq);
	}
}

static void hw_sugov_update_shared(struct update_util_data *hook, u64 time,
			    unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu,
						update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	unsigned int next_f;
	unsigned long irq_flag;

	hw_sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	raw_spin_lock_irqsave(&sg_policy->update_lock, irq_flag);

	if (hw_sugov_should_update_freq(sg_policy, time)) {
		sg_policy->util_changed = false;
		sg_policy_hw->trigger_cpu = sg_cpu->cpu;
		next_f = 0;
		hw_sugov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock_irqrestore(&sg_policy->update_lock, irq_flag);
}


static void hw_sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy,
						      work);
	struct sugov_policy_hw *sg_policy_hw = &sg_policy->sg_policy_hw;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int next_freq, cpu;
	struct rq *rq = NULL;
	int i = 0;
	const int cpus = cpumask_weight(policy->cpus);
	int early_detection[cpus];
	int skip_min_sample_time, skip_hispeed_logic;

	mutex_lock(&sg_policy->work_lock);

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, cpu);
		struct sugov_cpu_hw *j_sg_cpu_hw = &j_sg_cpu->sg_cpu_hw;

		if (!j_sg_cpu_hw->enabled)
			goto out;

		rq = cpu_rq(cpu);
		raw_spin_lock_irq(&rq->lock);

#ifdef CONFIG_HW_ED_TASK
		early_detection[i] = (rq->ed_task != NULL);
		if (early_detection[i])
			atomic_set(&sg_policy->sg_policy_hw.skip_hispeed_logic, 1);
		else
			walt_update_task_ravg(rq->curr, rq, TASK_UPDATE,
					       walt_ktime_clock(), 0);
#else
		early_detection[i] = 0;
		walt_update_task_ravg(rq->curr, rq, TASK_UPDATE,
				      walt_ktime_clock(), 0);
#endif

		raw_spin_unlock_irq(&rq->lock);
		i++;
	}

	/* Clear util changes marked above in walt update. */
	sg_policy->util_changed = false;

	/* Allow a new sugov_work to be queued now. */
	sg_policy->work_in_progress = false;

	/*
	 * Before we getting util, read and clear the skip_xxx requirements
	 * for this update.
	 *
	 * Otherwise,
	 * sugov_work():               on cpuX:
	 *   get cpuX's util(high)
	 *                               lower down cpuX's util(e.g. migration)
	 *                               set skip_min_sample_time
	 *   read and clear skip_xxx
	 *   set freq(high)
	 *                               another sugov_work()
	 *                               limited by min_sample_time
	 *
	 * In the following order we will make it consistent:
	 *   atomic_xchg(&skip_xxx, 0)   update any cpu util
	 *   get all cpu's util          atomic_set(&skip_xxx, 1)
	 *   ...                         ...
	 *   adjust frequency            adjust frequency
	 */
	skip_min_sample_time = atomic_xchg(&sg_policy_hw->skip_min_sample_time,
					   0);
	skip_hispeed_logic = atomic_xchg(&sg_policy_hw->skip_hispeed_logic, 0);

	next_freq = sugov_next_freq_shared_policy(sg_policy, sg_policy_hw->time,
						  early_detection);

	if (sugov_time_limit(sg_policy, next_freq, skip_min_sample_time,
			     skip_hispeed_logic)) {
		goto out;
	}

	if (next_freq == sg_policy->next_freq) {
		trace_cpufreq_schedutil_already(sg_policy_hw->max_cpu,
						sg_policy->next_freq,
						next_freq);
		goto out;
	}

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = sg_policy_hw->time;

	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);

out:
	mutex_unlock(&sg_policy->work_lock);

	sugov_slack_timer_resched(sg_policy);

	blocking_notifier_call_chain(&sugov_status_notifier_list,
		SUGOV_ACTIVE, &policy->cpu);
}

/********************** cpufreq governor interface *********************/
static void hw_sugov_tunables_init(struct sugov_tunables *tunables,
				   struct sugov_tunables *backup)
{
	struct sugov_tunables_hw *tunables_hw = &tunables->tunables_hw;
	struct sugov_tunables_hw *backup_hw = &backup->tunables_hw;
#ifdef CONFIG_HW_CPUFREQ_GOVERNOR_BACKUP
	/* initialize the tunable parameters */
	if (backup) {
		tunables_hw->overload_duration = backup_hw->overload_duration;
		tunables_hw->above_hispeed_delay =
			backup_hw->above_hispeed_delay;
		tunables_hw->nabove_hispeed_delay =
			backup_hw->nabove_hispeed_delay;
		tunables_hw->min_sample_time = backup_hw->min_sample_time;
		tunables_hw->nmin_sample_time = backup_hw->nmin_sample_time;
		tunables_hw->go_hispeed_load = backup_hw->go_hispeed_load;
		tunables_hw->hispeed_freq = backup_hw->hispeed_freq;
		tunables_hw->target_loads = backup_hw->target_loads;
		tunables_hw->ntarget_loads = backup_hw->ntarget_loads;
		tunables_hw->boostpulse_duration =
			backup_hw->boostpulse_duration;
		tunables_hw->boostpulse_min_interval =
			backup_hw->boostpulse_min_interval;
		tunables_hw->timer_slack_val = backup_hw->timer_slack_val;
		tunables_hw->fast_ramp_up = backup_hw->fast_ramp_up;
		tunables_hw->fast_ramp_down = backup_hw->fast_ramp_down;
		tunables_hw->freq_reporting_policy =
			backup_hw->freq_reporting_policy;
		tunables_hw->iowait_boost_step = backup_hw->iowait_boost_step;

#ifdef CONFIG_HW_ED_TASK
		tunables->ed_task_running_duration = backup->ed_task_running_duration;
		tunables->ed_task_waiting_duration = backup->ed_task_waiting_duration;
		tunables->ed_new_task_running_duration = backup->ed_new_task_running_duration;
#endif
#ifdef CONFIG_HW_TOP_TASK
		tunables->top_task_hist_size = backup->top_task_hist_size;
		tunables->top_task_stats_empty_window = backup->top_task_stats_empty_window;
		tunables->top_task_stats_policy = backup->top_task_stats_policy;
#endif
#ifdef CONFIG_HW_MIGRATION_NOTIFY
		tunables->freq_inc_notify = backup->freq_inc_notify;
		tunables->freq_dec_notify = backup->freq_dec_notify;
#endif

		return;
	}
#endif

	tunables_hw->overload_duration = DEFAULT_OVERLOAD_DURATION_MS;
	tunables_hw->above_hispeed_delay = default_above_hispeed_delay;
	tunables_hw->nabove_hispeed_delay =
		ARRAY_SIZE(default_above_hispeed_delay);
	tunables_hw->min_sample_time = default_min_sample_time;
	tunables_hw->nmin_sample_time =
		ARRAY_SIZE(default_min_sample_time);
	tunables_hw->go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
	tunables_hw->target_loads = default_target_loads;
	tunables_hw->ntarget_loads = ARRAY_SIZE(default_target_loads);
	tunables_hw->boostpulse_duration = DEFAULT_BOOSTPULSE_DURATION;
	tunables_hw->boostpulse_min_interval = DEFAULT_MIN_BOOSTPULSE_INTERVAL;
	tunables_hw->timer_slack_val = DEFAULT_TIMER_SLACK;
	tunables_hw->freq_reporting_policy = DEFAULT_FREQ_REPORTING_POLICY;
	tunables_hw->iowait_boost_step = IOWAIT_BOOST_INC_STEP;

#ifdef CONFIG_HW_ED_TASK
	tunables->ed_task_running_duration = EARLY_DETECTION_TASK_RUNNING_DURATION;
	tunables->ed_task_waiting_duration = EARLY_DETECTION_TASK_WAITING_DURATION;
	tunables->ed_new_task_running_duration = EARLY_DETECTION_NEW_TASK_RUNNING_DURATION;
#endif
#ifdef CONFIG_HW_TOP_TASK
	tunables->top_task_stats_empty_window = DEFAULT_TOP_TASK_STATS_EMPTY_WINDOW;
	tunables->top_task_stats_policy = DEFAULT_TOP_TASK_STATS_POLICY;
	tunables->top_task_hist_size = DEFAULT_TOP_TASK_HIST_SIZE;
#endif
#ifdef CONFIG_HW_MIGRATION_NOTIFY
	tunables->freq_inc_notify = DEFAULT_FREQ_INC_NOTIFY;
	tunables->freq_dec_notify = DEFAULT_FREQ_DEC_NOTIFY;
#endif

	spin_lock_init(&tunables_hw->target_loads_lock);
	spin_lock_init(&tunables_hw->above_hispeed_delay_lock);
	spin_lock_init(&tunables_hw->min_sample_time_lock);
}

static int cpufreq_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
#ifdef CONFIG_HW_SCHED_CLUSTER
	struct cpufreq_policy *policy = (struct cpufreq_policy *)data;
	struct sched_cluster *cluster = NULL;
	struct cpumask policy_cluster = *policy->related_cpus;
	int i, j;

	if (val != CPUFREQ_NOTIFY)
		return 0;

	for_each_cpu(i, &policy_cluster) {
		cluster = cpu_rq(i)->cluster;
		cpumask_andnot(&policy_cluster, &policy_cluster,
			        &cluster->cpus);

		cluster->min_freq = policy->min;
		cluster->max_freq = policy->max;

		if (!cluster->freq_init_done) {
			for_each_cpu(j, &cluster->cpus)
				cpumask_copy(&cpu_rq(j)->freq_domain_cpumask,
					      policy->related_cpus);
			cluster->freq_init_done = true;
			continue;
		}
	}
#endif

	return 0;
}

static int cpufreq_notifier_trans(struct notifier_block *nb,
		unsigned long val, void *data)
{
#ifdef CONFIG_HW_SCHED_CLUSTER
	struct cpufreq_freqs *freq = (struct cpufreq_freqs *)data;
	unsigned int cpu = freq->cpu, new_freq = freq->new;
	struct sched_cluster *cluster = NULL;
	struct cpumask policy_cpus = cpu_rq(cpu)->freq_domain_cpumask;
	int i;

	if (val != CPUFREQ_POSTCHANGE)
		return 0;

	BUG_ON(!new_freq);

	for_each_cpu(i, &policy_cpus) {
		cluster = cpu_rq(i)->cluster;

		cluster->cur_freq = new_freq;
		cpumask_andnot(&policy_cpus, &policy_cpus, &cluster->cpus);
	}
#endif

	return 0;
}

static struct notifier_block notifier_policy_block = {
	.notifier_call = cpufreq_notifier_policy
};

static struct notifier_block notifier_trans_block = {
	.notifier_call = cpufreq_notifier_trans
};

static int register_sched_callback(void)
{
	int ret;

	ret = cpufreq_register_notifier(&notifier_policy_block,
						CPUFREQ_POLICY_NOTIFIER);

	if (!ret)
		ret = cpufreq_register_notifier(&notifier_trans_block,
						CPUFREQ_TRANSITION_NOTIFIER);

	return 0;
}

/*
 * cpufreq callbacks can be registered at core_initcall or later time.
 * Any registration done prior to that is "forgotten" by cpufreq. See
 * initialization of variable init_cpufreq_transition_notifier_list_called
 * for further information.
 */
core_initcall(register_sched_callback);
