/*
 * migration_notify_hw.c
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

static inline bool
nearly_same_freq(struct rq *rq, unsigned int cur_freq,
		 unsigned int freq_required)
{
	int delta = freq_required - cur_freq;

	if (freq_required > cur_freq)
		return delta < rq->freq_inc_notify;

	delta = -delta;
	return delta < rq->freq_dec_notify;
}

extern unsigned long cpu_util(int cpu);
extern unsigned long task_util_est(struct task_struct *p);
static inline unsigned int estimate_freq_required(int cpu,
						  struct task_struct *p,
						  bool is_with_p)
{
	if (is_with_p)
		return util_to_freq(cpu, cpu_util(cpu) + task_util_est(p));
	else
		return util_to_freq(cpu, cpu_util(cpu));
}

static void
inter_cluster_migration_fixup(struct task_struct *p,
			      struct rq *src_rq, struct rq *dest_rq)
{
	int src_cpu = cpu_of(src_rq), dest_cpu = cpu_of(dest_rq);
	unsigned int src_freq_before, dest_freq_before;
	unsigned int src_freq_after, dest_freq_after;
	unsigned int flags;

	src_freq_before  = estimate_freq_required(src_cpu, p, true);
	dest_freq_before = estimate_freq_required(dest_cpu, p, false);

#ifdef CONFIG_HW_TOP_TASK
	migrate_top_task(p, src_rq, dest_rq);
#endif

	src_freq_after = estimate_freq_required(src_cpu, p, false);
	dest_freq_after = estimate_freq_required(dest_cpu, p, true);

	/*
	 * To lower down overhead of sugov_work, skip triggering src
	 * cpu freq update if:
	 * 1. migrating a small load, or
	 * 2. cur_freq == min_freq and estimated_freq_before <= min_freq
	 * Note that when a big load is moved to the cluster and quickly
	 * moved away, the freq increasing might have not completed so
	 * that we see cur_freq == min_freq. Freq update should not be
	 * skipped in such case so we double check estimated_freq_before
	 * must be less than min_freq.
	 */
	if ((src_rq->cluster->cur_freq != src_rq->cluster->min_freq ||
	     src_freq_before > src_rq->cluster->min_freq) &&
	    !nearly_same_freq(src_rq, src_freq_before, src_freq_after))
		sugov_mark_util_change(src_cpu, INTER_CLUSTER_MIGRATION_SRC);

	/*
	 * Skip triggering dest cpu freq update if:
	 * 1. migrating a small load, or
	 * 2. cur_freq == max_freq, or
	 * 3. min_freq satisfies estimated_freq_after
	 * Sometimes both src cluster and dest cluster's min_freq is locked
	 * to a high freq and there are loads migrate between clusters.
	 * This situation could last long and sugov_work can be skipped to
	 * save energy by checking the 3rd condition.
	 */
	if (dest_rq->cluster->cur_freq != dest_rq->cluster->max_freq &&
	    dest_freq_after > (dest_rq->cluster->min_freq >> 1) &&
	    !nearly_same_freq(dest_rq, dest_freq_before, dest_freq_after)) {
		flags = INTER_CLUSTER_MIGRATION_DST;

		sugov_mark_util_change(dest_cpu, flags);
	}
}
