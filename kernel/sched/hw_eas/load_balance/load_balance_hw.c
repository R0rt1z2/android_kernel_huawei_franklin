/*
 * Huawei Load Balance File
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

#ifdef CONFIG_HW_EAS_SCHED
void check_force_upmigrate(struct task_struct *p, struct rq *rq)
{
	unsigned long capacity_orig;
	int i;

	if (!render_rt_inited())
		return;

	capacity_orig = capacity_orig_of(cpu_of(rq));
	if (capacity_orig == rq->rd->max_cpu_capacity.val)
		return;

	if (rq->misfit_task_load)
		return;

	if (likely(walt_ktime_clock() - p->last_enqueued_ts <
			sysctl_sched_force_upmigrate_duration))
		return;

	for_each_cpu_and(i, &p->cpus_allowed, cpu_online_mask) {
		if (capacity_orig_of(i) > capacity_orig) {
			/* Suitable for upmigration. Mark it misfit. */
			rq->misfit_task_load = task_h_load(p);
			return;
		}
	}
}

/*
 * Reset balance_interval at all sched_domain levels of given cpu, so that it
 * honors kick.
 */
void reset_balance_interval(int cpu)
{
	struct sched_domain *sd = NULL;

	if (cpu >= nr_cpu_ids)
		return;

	rcu_read_lock();
	for_each_domain(cpu, sd)
		sd->balance_interval = 0;
	rcu_read_unlock();
}

/*
 * We are going to trigger a nohz balance. If we have vip_preempt_type, allow
 * relative sched domains to do balance(ASYM_CPUCAPACITY level sd needs
 * sd_overutilized flag).
 */
#ifdef CONFIG_HUAWEI_SCHED_VIP
void vip_balance_set_overutilized(int cpu)
{
	struct sched_domain *sd = NULL;

	if (likely(cpu_rq(cpu)->vip_preempt_type == NO_VIP_PREEMPT))
		return;

	rcu_read_lock();
	for_each_domain(cpu, sd)
		set_sd_overutilized(sd);
	rcu_read_unlock();
}
#else
void vip_balance_set_overutilized(int cpu) { }
#endif

#define NOHZ_KICK_ANY		0
#define NOHZ_KICK_RESTRICT	1
#define NOHZ_KICK_BOOST		2

static inline int nohz_kick_type(int call_cpu, struct sched_domain *sd)
{
	int type = NOHZ_KICK_ANY;
	int i;

	if (hw_test_fast_cpu(call_cpu))
		return NOHZ_KICK_ANY;

#ifdef CONFIG_HUAWEI_SCHED_VIP
	if (cpu_rq(call_cpu)->vip_preempt_type > VIP_PREEMPT_OTHER)
		return NOHZ_KICK_ANY;
#endif

	if (energy_aware() && cpu_rq(call_cpu)->misfit_task_load) {
		type = NOHZ_KICK_BOOST;
	} else if (!sd_overutilized(sd) && !cpu_overutilized(call_cpu)) {
		type = NOHZ_KICK_RESTRICT;
	} else {
		for_each_cpu(i, sched_domain_span(sd)) {

			if (cpu_util(i) * hw_sd_capacity_margin(i) < capacity_orig_of(i) * 1024) {
				/* Change the kick type to limit to CPUs that
				 * are of equal or lower capacity.
				 */
				type = NOHZ_KICK_RESTRICT;
				break;
			}
		}
	}

	return type;
}

int hw_find_new_ilb(void)
{
	struct sched_domain *sd = NULL;
	int call_cpu = smp_processor_id();
	int type = NOHZ_KICK_ANY;
	int ilb = nr_cpu_ids;
	int i;

	rcu_read_lock();

	sd = rcu_dereference_check_sched_domain(cpu_rq(call_cpu)->sd);
	if (!sd) {
		rcu_read_unlock();
		return nr_cpu_ids;
	}

	type = nohz_kick_type(call_cpu, sd);

	for_each_domain(call_cpu, sd) {
		for_each_cpu(i, sched_domain_span(sd)) {
			if (cpu_isolated(i))
				continue;
			if (idle_cpu(i)) {
				bool is_bigger_cpu = capacity_orig_of(i) > capacity_orig_of(call_cpu);

				if ((type == NOHZ_KICK_ANY) ||
				    (type == NOHZ_KICK_BOOST && is_bigger_cpu) ||
				    (type == NOHZ_KICK_RESTRICT && !is_bigger_cpu)) {
					ilb = i;
					break;
				}

			}
		}

		if (ilb < nr_cpu_ids)
			break;
	}

	rcu_read_unlock();

	reset_balance_interval(ilb);
	vip_balance_set_overutilized(call_cpu);

	return ilb;
}

struct task_struct *hw_get_heaviest_task(struct task_struct *p, int cpu)
{
	int num_tasks = 5;
	struct sched_entity *se = &p->se;
	unsigned long max_util = task_util_est(p), max_preferred_util = 0;
	struct task_struct *tsk = NULL, *max_preferred_tsk = NULL, *max_util_task = p;

	/* The currently running task is not on the runqueue */
	se = __pick_first_entity(cfs_rq_of(se));

	while (num_tasks && se) {
		if (!entity_is_task(se)) {
			se = __pick_next_entity(se);
			num_tasks--;
			continue;
		}

		tsk = task_of(se);

		if (cpumask_test_cpu(cpu, &tsk->cpus_allowed)) {
			bool boosted = schedtune_task_boost(tsk) > 0;
			bool prefer_idle = schedtune_prefer_idle(tsk) > 0;
			unsigned long util = boosted_task_util(tsk);

#ifdef CONFIG_HUAWEI_SCHED_VIP
			/* prefer vip task */
			if (tsk->vip_prio)
				return tsk;
#endif

			if (boosted || prefer_idle) {
				if (util > max_preferred_util) {
					max_preferred_util = util;;
					max_preferred_tsk = tsk;
				}
			} else {
				if (util > max_util) {
					max_util = util;
					max_util_task = tsk;
				}
			}
		}

		se = __pick_next_entity(se);
		num_tasks--;
	}

	return max_preferred_tsk ? max_preferred_tsk : max_util_task;
}

int find_max_idle_cpu(struct task_struct *p)
{
	int i, max_cap_cpu = -1;
	unsigned long max_cap = 0;

	for_each_cpu(i, &p->cpus_allowed) {
		unsigned long cap;

		if (!idle_cpu(i))
			continue;

		if (skip_cpu(i))
			continue;

		cap = capacity_orig_of(i);
		if (cap > max_cap) {
			max_cap = cap;
			max_cap_cpu = i;
		}
	}

	return max_cap_cpu;
}

#endif
