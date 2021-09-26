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

#ifdef CONFIG_HW_SCHED_PRED_LOAD

#ifndef CONFIG_HAVE_QCOM_SCHED_WALT
__read_mostly unsigned int predl_enable;
__read_mostly unsigned int predl_window_size =
					(16000000 / TICK_NSEC) * TICK_NSEC;
__read_mostly unsigned int predl_do_predict;
#endif

unsigned long predict_util(struct rq *rq)
{
	unsigned long util, limit;

	if (!predl_enable)
		return 0;

#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
	util = rq->walt_stats.pred_demands_sum_scaled;
#else
	util = (rq->sum_pred_load << SCHED_CAPACITY_SHIFT) / predl_window_size;
#endif
	/* Get rid of predicting the cluster's highest opp. */
	limit = capacity_orig_of(cpu_of(rq)) * 90 / 100;

	return min(util, limit);
}

unsigned long task_pred_util(struct task_struct *p)
{
	if (!predl_enable)
		return 0;

#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
	return p->ravg.pred_demand_scaled;
#else
	return ((u64)p->ravg.predl << SCHED_CAPACITY_SHIFT) / predl_window_size;
#endif
}
#endif /*HW_SCHED_PRED_LOAD*/


#ifndef CONFIG_HAVE_QCOM_SCHED_WALT
/*
 * Be careful to have our predl_window_size tunnable.
 * To get all rq's predl_window_start synchronized(at least when
 * predl_window_size is 'steady'), let predl_window_start be
 * n * predl_window_size;
 */
void
update_predl_window_start(struct rq *rq, u64 wallclock)
{
	s64 delta;
	u64 window_size = predl_window_size;

	delta = wallclock - rq->predl_window_start;
	if (delta < window_size)
		return;

	rq->predl_window_start = div64_u64(wallclock, window_size) * window_size;
}
#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
static inline u32 max_runtime(struct task_struct *p)
{
	u64 max_util = p->util_req.max_util;
	return (max_util * walt_ravg_window) >> SCHED_CAPACITY_SHIFT;
}

#ifdef CONFIG_HW_SCHED_PRED_LOAD
static inline u32 predl_max_runtime(struct task_struct *p)
{
	u64 max_util = p->util_req.max_util;
	return (max_util * predl_window_size) >> SCHED_CAPACITY_SHIFT;
}
#endif
#else
#define max_runtime(p) walt_ravg_window
#ifdef CONFIG_HW_SCHED_PRED_LOAD
#define predl_max_runtime(p) predl_window_size
#endif
#endif

#ifdef CONFIG_HW_SCHED_PRED_LOAD
#define INC_STEP 8
#define DEC_STEP 2
#define CONSISTENT_THRES 16
#define INC_STEP_BIG 16
#define DEC_STEP_SMALL 1
/*
 * bucket_increase - update the count of all buckets
 *
 * @buckets: array of buckets tracking busy time of a task
 * @idx: the index of bucket to be incremented
 *
 * Each time a complete window finishes, count of bucket that runtime
 * falls in (@idx) is incremented. Counts of all other buckets are
 * decayed. The rate of increase and decay could be different based
 * on current count in the bucket.
 */
static inline void bucket_increase(u8 *buckets, int idx)
{
	int i, inc_step, dec_step = DEC_STEP_SMALL;

	if (!predl_do_predict)
		return;

	for (i = NUM_BUSY_BUCKETS - 1; i >= 0; i--) {
		if (idx != i) {
			if (buckets[i] > 0) {
				if (buckets[i] > dec_step)
					buckets[i] -= dec_step;
				else
					buckets[i] = 0;

				dec_step = DEC_STEP;
			}
		} else {
			inc_step = buckets[i] >= CONSISTENT_THRES ?
						INC_STEP_BIG : INC_STEP;

			if (buckets[i] > U8_MAX - inc_step)
				buckets[i] = U8_MAX;
			else
				buckets[i] += inc_step;
		}
	}
}

static inline int busy_to_bucket(u32 normalized_rt)
{
	int bidx;

	if (!predl_do_predict)
		return 1;

	bidx = mult_frac(normalized_rt, NUM_BUSY_BUCKETS, predl_window_size);
	bidx = min(bidx, NUM_BUSY_BUCKETS - 1);

	/*
	 * Combine lowest two buckets. The lowest frequency falls into
	 * 2nd bucket and thus keep predicting lowest bucket is not
	 * useful.
	 */
	if (!bidx)
		bidx++;

	return bidx;
}

/*
 * get_pred_busy - calculate predicted demand for a task on runqueue
 *
 * @p: task whose prediction is being updated
 * @start: starting bucket. returned prediction should not be lower than
 *         this bucket.
 * @runtime: runtime of the task. returned prediction should not be lower
 *           than this runtime.
 * Note: @start can be derived from @runtime. It's passed in only to
 * avoid duplicated calculation in some cases.
 *
 * A new predicted busy time is returned for task @p based on @runtime
 * passed in. The function searches through buckets that represent busy
 * time equal to or bigger than @runtime and attempts to find the bucket to
 * to use for prediction. Once found, it searches through historical busy
 * time and returns the latest that falls into the bucket. If no such busy
 * time exists, it returns the medium of that bucket.
 */
static u32 get_pred_busy(struct task_struct *p, int start, u32 runtime)
{
	int i;
	u8 *buckets = p->ravg.predl_busy_buckets;
	u32 *hist = p->ravg.predl_sum_history;
	u32 dmin, dmax;
	int first = NUM_BUSY_BUCKETS, final;
	u32 ret = runtime;

	if (!predl_do_predict)
		goto out;

	/* skip prediction for new tasks due to lack of history */
	if (unlikely(is_new_task(p)))
		goto out;

	/* find minimal bucket index to pick */
	for (i = start; i < NUM_BUSY_BUCKETS; i++) {
		if (buckets[i]) {
			first = i;
			break;
		}
	}
	/* if no higher buckets are filled, predict runtime */
	if (first >= NUM_BUSY_BUCKETS)
		goto out;

	/* compute the bucket for prediction */
	final = first;

	/* determine demand range for the predicted bucket */
	if (final < 2) {
		/* lowest two buckets are combined */
		dmin = 0;
		final = 1;
	} else {
		dmin = mult_frac(final, predl_window_size, NUM_BUSY_BUCKETS);
	}
	dmax = mult_frac(final + 1, predl_window_size, NUM_BUSY_BUCKETS);

	/*
	 * search through runtime history and return first runtime that falls
	 * into the range of predicted bucket.
	 */
	for (i = 0; i < RAVG_HIST_SIZE_MAX; i++) {
		if (hist[i] >= dmin && hist[i] < dmax) {
			ret = hist[i];
			break;
		}
	}
	/* no historical runtime within bucket found, use average of the bin */
	if (ret < dmin)
		ret = (dmin + dmax) / 2;
	/*
	 * when updating in middle of a window, runtime could be higher
	 * than all recorded history. Always predict at least runtime.
	 */
	ret = max(runtime, ret);
out:
	ret = min(predl_max_runtime(p), ret);
	trace_predl_get_busy(p, runtime, start, ret);
	return ret;
}

/*
 * For each walt_update_task_ravg event happened in the current
 * window, pick the max of the task's runtime in curr window into
 * rq->curr_max_predls.
 * When a task sees rollover, rq->prev_max_predls will be updated.
 *
 * No prediction is accounted. And migration is not accounted.
 *
 * When window rolls over and a task is runnable, we won't see the
 * task's last runnable time until next event of the task.
 * It won't affect too much for our purpose. The running task's
 * runtime is perfectly accounted at window rollover.
 * We do it in that way because we don't want to see runtime in the
 * window before prev window, like top_task_load() do.
 */
unsigned long max_pred_ls(struct rq *rq)
{
	unsigned long util;

	if (!predl_enable)
		return 0;

	util = max(rq->curr_max_predls, rq->prev_max_predls);
	util = (util << SCHED_CAPACITY_SHIFT) / predl_window_size;
	return util;
}

unsigned long cpu_util_pred_min(struct rq *rq)
{
	int i;
	u64 min_cpu_load = ULONG_MAX;

	if (!predl_enable)
		return 0;

	for_each_cpu(i, &rq->cluster->cpus)
		min_cpu_load = min(min_cpu_load, cpu_rq(i)->predl_prev_busy_time);

	return (min_cpu_load << SCHED_CAPACITY_SHIFT) / predl_window_size;
}

void fixup_pred_load(struct rq *rq, s64 pred_load_delta)
{
	rq->sum_pred_load += pred_load_delta;
	BUG_ON((s64)rq->sum_pred_load < 0);
}

unsigned int predl_jump_load = 90;

#ifdef CONFIG_HW_EAS_SCHED
/*
 * 1. The default config of predl_jump_load should disable any adjustment.
 * 2. Boost like schedtune_margin is better than multiplying a fixed value.
 * To be optimized later.
 */
#define adjust_runtime(rq, p, runtime) (runtime)
#else
/*
 * Most of the time, when a task takes nearly the whole window's
 * time running, it means the task's needs is not satisfied.
 * In other words it requires more than what it has taken.
 * Adjust the needs by multiplying 125% in such case.
 */
static u32 adjust_runtime(struct rq *rq,
		struct task_struct *p, u32 runtime)
{
	u64 util, cap;

#ifdef CONFIG_HW_TOP_TASK
	if (!schedtune_top_task(p))
		return runtime;
#endif

	/* If the runtime is long ago, do not change it. */
	if (p->ravg.mark_start + predl_window_size < rq->predl_window_start)
		return runtime;

	util = ((u64)runtime << SCHED_CAPACITY_SHIFT) / predl_window_size;
	cap = capacity_curr_of(cpu_of(rq));

	if (util * 100 > cap * predl_jump_load)
		runtime = runtime + (runtime >> 2); /* 25% bonus */

	trace_predl_adjust_runtime(p, util, cap);
	return runtime;
}
#endif

static void predl_update_history(struct task_struct *p, u32 runtime)
{
	u32 *hist = &p->ravg.predl_sum_history[0];
	int i;

	for (i = RAVG_HIST_SIZE_MAX - 1; i >= 1; --i)
		hist[i] = hist[i-1];
	hist[0] = runtime;

	p->ravg.predl_sum = 0;

	trace_predl_update_history(p);
}

/*
 * Do not account task load to rq max_predls if it run on other
 * clusters at the most of time.
 */
static const unsigned int sysctl_acount_max_predls_threshold = 25;
static inline bool task_has_contribution(struct rq *rq, struct task_struct *p)
{
	int i;
	u64 sum_contribution = 0;

	for_each_cpu(i, &rq->cluster->cpus)
		sum_contribution += p->ravg.predl_prev_window_cpu[i];

	return sum_contribution * 100 >=
		rq->predl_prev_busy_time * sysctl_acount_max_predls_threshold;
}

static void add_to_max_predls(struct rq *rq,
		struct task_struct *p, bool rollover)
{
	u32 runtime = p->ravg.predl_sum;
	u32 *max_predls = rollover ? &rq->prev_max_predls
				: &rq->curr_max_predls;

	if (runtime > *max_predls &&
	    task_has_contribution(rq, p))
		*max_predls = runtime;
}

/*
 * Update predl using runtime and update buckets when rollover.
 */
static void predict_and_update(struct rq *rq,
		struct task_struct *p, bool rollover)
{
	int bidx;
	u32 pred_load;
	u32 runtime = p->ravg.predl_sum;

	add_to_max_predls(rq, p, rollover);

	/*
	 * Force update predl when rollover.
	 * Otherwise skip update if runtime has not exceed last predl or
	 * if prediction disabled, in which cases we won't need to increase
	 * cpufreq.
	 */
	if (rollover) {
		runtime = adjust_runtime(rq, p, runtime);
		predl_update_history(p, runtime);
	} else {
		if (runtime <= p->ravg.predl || !predl_do_predict)
			return;
	}

	bidx = busy_to_bucket(runtime);

	pred_load = get_pred_busy(p, bidx, runtime);
	if (rollover)
		bucket_increase(p->ravg.predl_busy_buckets, bidx);

	if (!task_has_dl_policy(p) || !p->dl.dl_throttled) {
		if (task_on_rq_queued(p))
			fixup_pred_load(rq, (s64)pred_load - p->ravg.predl);
	}

	p->ravg.predl = pred_load;

	/* Trigger freq update if we have predicted high load. */
	if (((u64)pred_load << SCHED_CAPACITY_SHIFT) >
	    ((u64)capacity_curr_of(cpu_of(rq)) * predl_window_size))
		sugov_mark_util_change(cpu_of(rq), PRED_LOAD_CHANGE);
}

static void add_to_task_predl(struct rq *rq, struct task_struct *p,
			u64 delta, int event, bool prev_window)
{
	delta = scale_exec_time_limit(delta, rq,
			predl_max_runtime(p) - p->ravg.predl_sum);

	p->ravg.predl_sum += delta;

	/*
	 * Task predl accounts waiting time.
	 * Accumulate non-waiting times for cpu busy time.
	 */
	if (event == PUT_PREV_TASK ||
	    (event == TASK_UPDATE && rq->curr == p)) {
		if (prev_window)
			rq->predl_prev_busy_time += delta;
		else
			rq->predl_curr_busy_time += delta;
	}

	/*
	 * Account p's predl_sum on this cpu.
	 * We have rolled it over before.
	 */
	if (prev_window)
		p->ravg.predl_prev_window_cpu[cpu_of(rq)] += delta;
	else
		p->ravg.predl_curr_window_cpu[cpu_of(rq)] += delta;
}

static inline void rollover_cpu_predl(struct rq *rq, bool full_windows)
{
	if (full_windows) {
		rq->curr_max_predls = 0;
		rq->predl_curr_busy_time = 0;
	}

	rq->prev_max_predls = rq->curr_max_predls;
	rq->curr_max_predls = 0;
	rq->predl_prev_busy_time = rq->predl_curr_busy_time;
	rq->predl_curr_busy_time = 0;
}

static inline void rollover_task_predl_window_cpu(struct task_struct *p,
						bool full_windows)
{
	unsigned int i;

	for (i = 0; i < nr_cpu_ids; i++) {
		u32 curr = 0;

		if (!full_windows)
			curr = p->ravg.predl_curr_window_cpu[i];

		p->ravg.predl_prev_window_cpu[i] = curr;
		p->ravg.predl_curr_window_cpu[i] = 0;
	}
}

/* Account runnable and not account empty windows. */
void update_task_predl(struct task_struct *p,
		struct rq *rq, int event, u64 wallclock)
{
	u64 mark_start = p->ravg.mark_start;
	u64 window_start = rq->predl_window_start;
	u32 window_size = predl_window_size;
	bool new_window = (mark_start < window_start);
	bool account_busy = (event != TASK_WAKE);
	bool full_windows = (mark_start + window_size < window_start);

	if (!predl_enable)
		return;

	if (new_window && (p == rq->curr))
		rollover_cpu_predl(rq, full_windows);

	/*
	 * Just ignore exiting task. The rq's signal don't need to be
	 * very accurate.
	 */
	if (exiting_task(p) || is_idle_task(p))
		return;

	if (new_window)
		rollover_task_predl_window_cpu(p, full_windows);

	if (new_window && !account_busy) {
		predict_and_update(rq, p, true);
	} else if (new_window && account_busy) {
		/*
		 * It's possible to have full windows. However we do
		 * not really care about it.
		 * Treat as if it has run a full window time.
		 */
		if (full_windows) {
			p->ravg.predl_sum = 0;
			mark_start = window_start - window_size;
		}

		add_to_task_predl(rq, p, window_start - mark_start, event, true);
		predict_and_update(rq, p, true);
		add_to_task_predl(rq, p, wallclock - window_start, event, false);
	} else if (!new_window && account_busy) {
		add_to_task_predl(rq, p, wallclock - mark_start, event, false);
		predict_and_update(rq, p, false);
	}
}

#endif /* CONFIG_HW_SCHED_PRED_LOAD */
#endif /*NO_HAVE_QCOM_SCHED_WALT*/
