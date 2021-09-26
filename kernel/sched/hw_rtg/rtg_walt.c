/*
 * rtg_walt.c
 *
 * rtg sched file
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

static void add_to_group_task_time(struct related_thread_group *grp,
	struct rq *rq, struct task_struct *p, u64 wallclock)
{
	u64 mark_start = p->ravg.mark_start;
	u64 window_start = grp->window_start;
	u64 delta_exec, delta_load;

	if (unlikely(wallclock <= mark_start))
		return;

	/* per task load tracking in RTG */
	if (likely(mark_start >= window_start)) {
		/*
		 *   ws   ms  wc
		 *   |    |   |
		 *   V    V   V
		 *   |---------------|
		 */
		delta_exec = wallclock - mark_start;
		p->ravg.curr_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		p->ravg.curr_window_load += delta_load;
	} else {
		/*
		 *   ms   ws  wc
		 *   |    |   |
		 *   V    V   V
		 *   -----|----------
		 */
		/* prev window task statistic */
		delta_exec = window_start - mark_start;
		p->ravg.prev_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		p->ravg.prev_window_load += delta_load;

		/* curr window task statistic */
		delta_exec = wallclock - window_start;
		p->ravg.curr_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		p->ravg.curr_window_load += delta_load;
	}
}

static void add_to_group_time(struct related_thread_group *grp,
	struct rq *rq, u64 wallclock)
{
	u64 delta_exec, delta_load;
	u64 mark_start = grp->mark_start;
	u64 window_start = grp->window_start;

	if (unlikely(wallclock <= mark_start))
		return;

	/* per group load tracking in RTG */
	if (likely(mark_start >= window_start)) {
		/*
		 *   ws   ms  wc
		 *   |    |   |
		 *   V    V   V
		 *   |---------------|
		 */
		delta_exec = wallclock - mark_start;
		grp->time.curr_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		grp->time.curr_window_load += delta_load;
	} else {
		/*
		 *   ms   ws  wc
		 *   |    |   |
		 *   V    V   V
		 *   -----|----------
		 */
		/* prev window statistic */
		delta_exec = window_start - mark_start;
		grp->time.prev_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		grp->time.prev_window_load += delta_load;

		/* curr window statistic */
		delta_exec = wallclock - window_start;
		grp->time.curr_window_exec += delta_exec;

		delta_load = scale_exec_time(delta_exec, rq);
		grp->time.curr_window_load += delta_load;
	}
}

static inline void add_to_group_demand(struct related_thread_group *grp,
	struct rq *rq, struct task_struct *p, u64 wallclock)
{
	if (unlikely(wallclock <= grp->window_start))
		return;

	add_to_group_task_time(grp, rq, p, wallclock);
	add_to_group_time(grp, rq, wallclock);
}

static int account_busy_for_group_demand(struct task_struct *p, int event)
{
	/*
	* No need to bother updating task demand for exiting tasks
	* or the idle task.
	*/
	if (exiting_task(p) || is_idle_task(p))
		return 0;

	if (event == TASK_WAKE || event == TASK_MIGRATE)
		return 0;

	return 1;
}

static void update_group_demand(struct task_struct *p, struct rq *rq,
	int event, u64 wallclock)
{
	struct related_thread_group *grp = NULL;

	if (!account_busy_for_group_demand(p, event))
		return;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	if (!grp) {
		rcu_read_unlock();
		return;
	}

	raw_spin_lock(&grp->lock);
	if (!grp->mode.util_enabled)
		goto unlock;

	if (grp->nr_running == 1)
		grp->mark_start = max(grp->mark_start, p->ravg.mark_start);

	add_to_group_demand(grp, rq, p, wallclock);

	grp->mark_start = wallclock;

unlock:
	raw_spin_unlock(&grp->lock);

	rcu_read_unlock();
}

void update_group_nr_running(struct task_struct *p, int event, u64 wallclock)
{
	struct related_thread_group *grp = NULL;
	bool need_update = false;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	if (!grp) {
		rcu_read_unlock();
		return;
	}

	raw_spin_lock(&grp->lock);

	if (event == PICK_NEXT_TASK)
		grp->nr_running++;
	else if (event == PUT_PREV_TASK)
		grp->nr_running--;

	if ((int)grp->nr_running < 0) {
		WARN_ON(1);
		grp->nr_running = 0;
	}

	/* update preferred cluster if no update long */
	if (wallclock - grp->last_util_update_time > grp->util_update_timeout)
		need_update = true;

	raw_spin_unlock(&grp->lock);

	rcu_read_unlock();

	if (need_update && grp->rtg_class && grp->id != DEFAULT_CGROUP_COLOC_ID)
		grp->rtg_class->sched_update_rtg_tick(grp, NULL);
}
