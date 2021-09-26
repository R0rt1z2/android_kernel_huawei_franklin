/*
 * tune_hw.c
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

static int
schedtune_freq_boostgroup_update(int idx, int freq_boost)
{
	struct boost_groups *bg = NULL;
	int cur_freq_boost_max;
	int old_freq_boost;
	int cpu;
	u64 now;

	/* Update per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);

		/*
		 * Keep track of current boost values to compute the per CPU
		 * maximum only when it has been affected by the new value of
		 * the updated boost group
		 */
		cur_freq_boost_max = bg->freq_boost_max;
		old_freq_boost = bg->group[idx].freq_boost;

		/* Update the boost value of this boost group */
		bg->group[idx].freq_boost = freq_boost;

		/* Check if this update increase current max */
		if (freq_boost > cur_freq_boost_max && bg->group[idx].tasks) {
			bg->freq_boost_max = freq_boost;
			trace_sched_tune_freqboostgroup_update(cpu, 1,
							bg->freq_boost_max);
			continue;
		}

		/* Check if this update has decreased current max */
		if (cur_freq_boost_max == old_freq_boost &&
		    old_freq_boost > freq_boost) {
			now = sched_clock_cpu(cpu);
			schedtune_cpu_update(cpu, now);
			trace_sched_tune_freqboostgroup_update(cpu, -1,
							bg->freq_boost_max);
			continue;
		}

		trace_sched_tune_freqboostgroup_update(cpu, 0,
						       bg->freq_boost_max);
	}

	return 0;
}

int schedtune_freq_boost(int cpu)
{
	struct boost_groups *bg;

	bg = &per_cpu(cpu_boost_groups, cpu);
	return bg->freq_boost_max;
}

static s64
freq_boost_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->freq_boost;
}

static int
freq_boost_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    s64 boost)
{
	struct schedtune *st = css_st(css);

	if (boost < -100 || boost > 100)
		return -EINVAL;

	st->freq_boost = boost;

	/* Update CPU boost */
	schedtune_freq_boostgroup_update(st->idx, st->freq_boost);

	/* trace stune_name and value */
	trace_sched_tune_freqboost(css->cgroup->kn->name, boost);

	return 0;
}
