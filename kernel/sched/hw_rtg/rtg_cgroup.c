/*
 * rtg_cgroup.c
 *
 * Default cgroup load tracking for RTG
 *
 * Copyright (c) 2020-2021 Huawei Technologies Co., Ltd.
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
#ifdef CONFIG_HW_CGROUP_RTG

static inline
void update_best_cluster(struct related_thread_group *grp,
				   u64 demand, bool boost)
{
	if (boost) {
		/*
		 * since we are in boost, we can keep grp on min, the boosts
		 * will ensure tasks get to bigs
		 */
		grp->skip_min = false;
		return;
	}

	if (is_suh_max())
		demand = sched_group_upmigrate;

	if (!grp->skip_min) {
		if (demand >= sched_group_upmigrate) {
			grp->skip_min = true;
		}
		return;
	}
	if (demand < sched_group_downmigrate) {
		if (!sysctl_sched_coloc_downmigrate_ns) {
			grp->skip_min = false;
			return;
		}
		if (!grp->downmigrate_ts) {
			grp->downmigrate_ts = grp->last_update;
			return;
		}
		if (grp->last_update - grp->downmigrate_ts >
				sysctl_sched_coloc_downmigrate_ns) {
			grp->downmigrate_ts = 0;
			grp->skip_min = false;
		}
	} else if (grp->downmigrate_ts)
		grp->downmigrate_ts = 0;
}

void _do_update_preferred_cluster(struct related_thread_group *grp)
{
	struct task_struct *p = NULL;
	u64 combined_demand = 0;
	bool group_boost = false;
	u64 wallclock;
	bool prev_skip_min = grp->skip_min;

	if (grp->id != DEFAULT_CGROUP_COLOC_ID)
		return;

	if (list_empty(&grp->tasks)) {
		grp->skip_min = false;
		goto out;
	}

	if (!hmp_capable()) {
		grp->skip_min = false;
		goto out;
	}

	wallclock = walt_ktime_clock();

	/*
	 * wakeup of two or more related tasks could race with each other and
	 * could result in multiple calls to _do_update_preferred_cluster being issued
	 * at same time. Avoid overhead in such cases of rechecking preferred
	 * cluster
	 */
	if (wallclock - grp->last_update < sched_ravg_window / 10)
		return;

	list_for_each_entry(p, &grp->tasks, grp_list) {
		if (task_boost_policy(p) == SCHED_BOOST_ON_BIG) {
			group_boost = true;
			break;
		}

		if (p->ravg.mark_start < wallclock -
		    (sched_ravg_window * sched_ravg_hist_size))
			continue;

		combined_demand += p->ravg.coloc_demand;
		if (!trace_rtg_update_preferred_cluster_enabled()) {
			if (combined_demand > sched_group_upmigrate)
				break;
		}
	}

	grp->last_update = wallclock;
	update_best_cluster(grp, combined_demand, group_boost);
	trace_rtg_update_preferred_cluster(grp, combined_demand);
out:
	if (grp->id == DEFAULT_CGROUP_COLOC_ID
	    && grp->skip_min != prev_skip_min) {
		if (grp->skip_min)
			grp->start_ts = sched_clock();
		sched_update_hyst_times();
	}
}

void do_update_preferred_cluster(struct related_thread_group *grp)
{
	raw_spin_lock(&grp->lock);
	_do_update_preferred_cluster(grp);
	raw_spin_unlock(&grp->lock);
}

int update_preferred_cluster(struct related_thread_group *grp,
		struct task_struct *p, u32 old_load, bool from_tick)
{
	u32 new_load = task_load(p);

	if (!grp || grp->id != DEFAULT_CGROUP_COLOC_ID)
		return 0;

	if (unlikely(from_tick && is_suh_max()))
		return 1;

	/*
	 * Update if task's load has changed significantly or a complete window
	 * has passed since we last updated preference
	 */
	if (abs(new_load - old_load) > sched_ravg_window / 4 ||
		walt_ktime_clock() - grp->last_update > sched_ravg_window)
		return 1;

	return 0;
}

static void update_cgroup_rtg_tick(struct related_thread_group *grp,
				   struct rtg_tick_info *tick_info)
{
	if (!tick_info)
		return;

	if (update_preferred_cluster(grp, tick_info->curr, tick_info->old_load, true))
		do_update_preferred_cluster(grp);
}

const struct rtg_class cgroup_rtg_class = {
	.sched_update_rtg_tick = update_cgroup_rtg_tick,
};


/*
 * We create a default colocation group at boot. There is no need to
 * synchronize tasks between cgroups at creation time because the
 * correct cgroup hierarchy is not available at boot. Therefore cgroup
 * colocation is turned off by default even though the colocation group
 * itself has been allocated. Furthermore this colocation group cannot
 * be destroyted once it has been created. All of this has been as part
 * of runtime optimizations.
 *
 * The job of synchronizing tasks to the colocation group is done when
 * the colocation flag in the cgroup is turned on.
 */
static int __init create_default_coloc_group(void)
{
	struct related_thread_group *grp = NULL;
	unsigned long flags;

	grp = lookup_related_thread_group(DEFAULT_CGROUP_COLOC_ID);
	write_lock_irqsave(&related_thread_group_lock, flags);
	grp->rtg_class = &cgroup_rtg_class;
	list_add(&grp->list, &active_related_thread_groups);
	write_unlock_irqrestore(&related_thread_group_lock, flags);

	return 0;
}
late_initcall(create_default_coloc_group);

int sync_cgroup_colocation(struct task_struct *p, bool insert)
{
	unsigned int grp_id = insert ? DEFAULT_CGROUP_COLOC_ID : 0;

	return __sched_set_group_id(p, grp_id);
}

#endif
