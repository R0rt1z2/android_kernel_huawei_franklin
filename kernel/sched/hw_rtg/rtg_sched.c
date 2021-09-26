/*
 * rtg_sched.c
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

#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <trace/events/sched.h>

#include <../kernel/sched/sched.h>
#include <../kernel/sched/walt.h>
#include <../kernel/sched/tune.h>
#include "include/rtg_perf_ctrl.h"
#include "include/rtg_sched.h"
#include "include/rtg_pseudo.h"
#include "../hw_walt/walt_hw.h"

#if defined(CONFIG_HW_RTG_NORMALIZED_UTIL) && defined(CONFIG_HW_SCHED_CLUSTER)
#include "../hw_cluster/sched_cluster.h"
#endif

#ifdef CONFIG_HW_GLOBAL_BOOST
#include "../kernel/sched/hw_eas/boost/boost_hw.h"
#endif

#define DEFAULT_FREQ_UPDATE_INTERVAL	8000000  /* ns */
#define DEFAULT_UTIL_INVALID_INTERVAL	(~0U) /* ns */
#define DEFAULT_UTIL_UPDATE_TIMEOUT	20000000  /* ns */
#define DEFAULT_GROUP_RATE		60 /* 60FPS */
#define DEFAULT_RTG_CAPACITY_MARGIN	1077 /* 5% */

struct related_thread_group *related_thread_groups[MAX_NUM_CGROUP_COLOC_ID];
static DEFINE_RWLOCK(related_thread_group_lock);
#ifdef CONFIG_HW_RTG_DEBUG
LIST_HEAD(active_related_thread_groups);
#else
static LIST_HEAD(active_related_thread_groups);
#define for_each_related_thread_group(grp) \
	list_for_each_entry(grp, &active_related_thread_groups, list)
#endif

#define ADD_TASK	0
#define REM_TASK	1

extern unsigned long capacity_spare_without(int cpu, struct task_struct *p);

struct related_thread_group *
lookup_related_thread_group(unsigned int group_id)
{
	return related_thread_groups[group_id];
}

void init_task_rtg(struct task_struct *p)
{
	rcu_assign_pointer(p->grp, NULL);
	INIT_LIST_HEAD(&p->grp_list);
}

int sched_set_group_window_size(unsigned int grp_id, unsigned int rate)
{
	struct related_thread_group *grp = NULL;
	unsigned long flag;

	if (!rate)
		return -EINVAL;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set window size for group %d fail\n", grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);
	grp->window_size = NSEC_PER_SEC / rate;
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}

void group_time_rollover(struct group_time *time)
{
	time->prev_window_load = time->curr_window_load;
	time->curr_window_load = 0;
	time->prev_window_exec = time->curr_window_exec;
	time->curr_window_exec = 0;
}

int sched_set_group_window_rollover(unsigned int grp_id)
{
	struct related_thread_group *grp = NULL;
	u64 wallclock;
	struct task_struct *p = NULL;
	unsigned long flag;
#ifdef CONFIG_HW_RTG_BOOST
	int boost;
#endif

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set window start for group %d fail\n", grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);

	wallclock = walt_ktime_clock();
	grp->prev_window_time = wallclock - grp->window_start;
	grp->window_start = wallclock;

#ifdef CONFIG_HW_RTG_BOOST
	grp->max_boost = INT_MIN;
#endif
	list_for_each_entry(p, &grp->tasks, grp_list) {
		p->ravg.prev_window_load = p->ravg.curr_window_load;
		p->ravg.curr_window_load = 0;
		p->ravg.prev_window_exec = p->ravg.curr_window_exec;
		p->ravg.curr_window_exec = 0;

#ifdef CONFIG_HW_RTG_BOOST
		boost = schedtune_task_boost(p);
		if (boost > grp->max_boost)
			grp->max_boost = boost;
#endif
	}

	group_time_rollover(&grp->time);
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	group_time_rollover(&grp->time_pref_cluster);
#endif
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}

int sched_set_group_freq_update_interval(unsigned int grp_id, unsigned int interval)
{
	struct related_thread_group *grp = NULL;
	unsigned long flag;

	if ((signed int)interval <= 0)
		return -EINVAL;

	/* DEFAULT_CGROUP_COLOC_ID is a reserved id */
	if (grp_id == DEFAULT_CGROUP_COLOC_ID ||
	    grp_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set update interval for group %d fail\n", grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);
	grp->freq_update_interval = interval * NSEC_PER_MSEC;
#ifdef CONFIG_HW_RTG_PSEUDO_TICK
	frame_pseudo_update(grp);
#endif
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}

static bool group_should_update_freq(struct related_thread_group *grp,
			      int cpu, unsigned int flags, u64 now)
{
	if (!grp || !is_frame_rtg(grp->id))
		return true;

	if (flags & FRAME_FORCE_UPDATE) {
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
		sugov_mark_util_change(cpu, FRAME_UPDATE);
#endif
		return true;
	} else if (flags & FRAME_NORMAL_UPDATE) {
		if (now - grp->last_freq_update_time >=
		    grp->freq_update_interval) {
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
			sugov_mark_util_change(cpu, FRAME_UPDATE);
#endif
			return true;
		}
	}

	return false;
}

int sched_set_group_load_mode(struct rtg_load_mode *mode)
{
	struct related_thread_group *grp = NULL;
	unsigned long flag;

	if (mode->grp_id <= DEFAULT_CGROUP_COLOC_ID ||
	    mode->grp_id >= MAX_NUM_CGROUP_COLOC_ID) {
		pr_err("grp_id=%d is invalid.\n", mode->grp_id);
		return -EINVAL;
	}

	grp = lookup_related_thread_group(mode->grp_id);
	if (!grp) {
		pr_err("set invalid interval for group %d fail\n", mode->grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);
	grp->mode.freq_enabled = !!mode->freq_enabled;
	grp->mode.util_enabled = !!mode->util_enabled;
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}

#ifdef CONFIG_HW_ED_TASK
int sched_set_group_ed_params(struct rtg_ed_params *params)
{
	struct related_thread_group *grp = NULL;
	unsigned long flag;

	if (params->grp_id <= DEFAULT_CGROUP_COLOC_ID ||
	    params->grp_id >= MAX_NUM_CGROUP_COLOC_ID) {
		pr_err("grp_id=%d is invalid.\n", params->grp_id);
		return -EINVAL;
	}

	grp = lookup_related_thread_group(params->grp_id);
	if (!grp) {
		pr_err("set invalid interval for group %d fail\n",
			params->grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);
	grp->ed_enabled = params->enabled > 0 ? true : false;
	grp->ed_task_running_duration = params->running_ns;
	grp->ed_task_waiting_duration = params->waiting_ns;
	grp->ed_new_task_running_duration = params->nt_running_ns;
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}
#endif /* CONFIG_HW_ED_TASK */

#ifdef CONFIG_HW_RTG_CPU_TIME
static struct group_cpu_time *_group_cpu_time(struct related_thread_group *grp,
				       int cpu)
{
	return grp ? per_cpu_ptr(grp->cpu_time, cpu) : NULL;
}

/*
 * A group's window_start may be behind. When moving it forward, flip prev/curr
 * counters. When moving forward > 1 window, prev counter is set to 0
 */
void group_sync_window_start(struct rq *rq, struct group_cpu_time *cpu_time)
{
	u64 delta;
	int nr_windows;
	u64 curr_sum = cpu_time->curr_runnable_sum;

	delta = rq->window_start - cpu_time->window_start;
	if (!delta)
		return;

	nr_windows = div64_u64(delta, walt_ravg_window);
	if (nr_windows > 1)
		curr_sum = 0;

	cpu_time->prev_runnable_sum  = curr_sum;
	cpu_time->curr_runnable_sum  = 0;

	cpu_time->window_start = rq->window_start;
}

struct group_cpu_time *group_update_cpu_time(struct rq *rq,
	struct related_thread_group *grp)
{
	struct group_cpu_time *cpu_time = NULL;

	if (grp) {
		/* cpu_time protected by rq_lock */
		cpu_time = _group_cpu_time(grp, cpu_of(rq));

		/* group window roll over */
		group_sync_window_start(rq, cpu_time);
	}

	return cpu_time;
}

bool group_migrate_task(struct task_struct *p,
			struct rq *src_rq, struct rq *dest_rq)
{
	struct group_cpu_time *cpu_time = NULL;
	struct related_thread_group *grp = p->grp;
	u64 *src_curr_runnable_sum = NULL;
	u64 *dst_curr_runnable_sum = NULL;
	u64 *src_prev_runnable_sum = NULL;
	u64 *dst_prev_runnable_sum = NULL;

	if (!grp)
		return false;

	/* Protected by rq_lock */
	cpu_time = _group_cpu_time(grp, cpu_of(src_rq));
	src_curr_runnable_sum = &cpu_time->curr_runnable_sum;
	src_prev_runnable_sum = &cpu_time->prev_runnable_sum;

	/* Protected by rq_lock */
	cpu_time = _group_cpu_time(grp, cpu_of(dest_rq));
	dst_curr_runnable_sum = &cpu_time->curr_runnable_sum;
	dst_prev_runnable_sum = &cpu_time->prev_runnable_sum;

	group_sync_window_start(dest_rq, cpu_time);

	if (p->ravg.curr_window) {
		*src_curr_runnable_sum -= p->ravg.curr_window;
		*dst_curr_runnable_sum += p->ravg.curr_window;
	}

	if (p->ravg.prev_window) {
		*src_prev_runnable_sum -= p->ravg.prev_window;
		*dst_prev_runnable_sum += p->ravg.prev_window;
	}

	return true;
}

/*
 * Task's cpu usage is accounted in:
 *	rq->curr/prev_runnable_sum,  when its ->grp is NULL
 *	grp->cpu_time[cpu]->curr/prev_runnable_sum, when its ->grp is !NULL
 *
 * Transfer task's cpu usage between those counters when transitioning between
 * groups
 */
static void transfer_busy_time(struct rq *rq, struct related_thread_group *grp,
			       struct task_struct *p, int event)
{
	u64 wallclock;
	struct group_cpu_time *cpu_time = NULL;
	u64 *src_curr_runnable_sum = NULL;
	u64 *dst_curr_runnable_sum = NULL;
	u64 *src_prev_runnable_sum = NULL;
	u64 *dst_prev_runnable_sum = NULL;

	wallclock = walt_ktime_clock();

	walt_update_task_ravg(rq->curr, rq, TASK_UPDATE, wallclock, 0);
	walt_update_task_ravg(p, rq, TASK_UPDATE, wallclock, 0);

	raw_spin_lock(&grp->lock);
	/* cpu_time protected by related_thread_group_lock, grp->lock rq_lock */
	cpu_time = _group_cpu_time(grp, cpu_of(rq));
	if (event == ADD_TASK) {
		group_sync_window_start(rq, cpu_time);
		src_curr_runnable_sum = &rq->curr_runnable_sum;
		dst_curr_runnable_sum = &cpu_time->curr_runnable_sum;
		src_prev_runnable_sum = &rq->prev_runnable_sum;
		dst_prev_runnable_sum = &cpu_time->prev_runnable_sum;
	} else {
		/*
		 * In case of REM_TASK, cpu_time->window_start would be
		 * uptodate, because of the update_task_ravg() we called
		 * above on the moving task. Hence no need for
		 * group_sync_window_start()
		 */
		src_curr_runnable_sum = &cpu_time->curr_runnable_sum;
		dst_curr_runnable_sum = &rq->curr_runnable_sum;
		src_prev_runnable_sum = &cpu_time->prev_runnable_sum;
		dst_prev_runnable_sum = &rq->prev_runnable_sum;
	}

	*src_curr_runnable_sum -= p->ravg.curr_window;
	*dst_curr_runnable_sum += p->ravg.curr_window;

	*src_prev_runnable_sum -= p->ravg.prev_window;
	*dst_prev_runnable_sum += p->ravg.prev_window;
	raw_spin_unlock(&grp->lock);
}

void sched_update_group_load(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct related_thread_group *grp = NULL;
	unsigned long flags;

	/*
	 * This function could be called in timer context, and the
	 * current task may have been executing for a long time. Ensure
	 * that the window stats are current by doing an update.
	 */
	read_lock_irqsave(&related_thread_group_lock, flags);

	for_each_related_thread_group(grp) {
		/* Protected by rq_lock */
		struct group_cpu_time *cpu_time =
					_group_cpu_time(grp, cpu);
		group_sync_window_start(rq, cpu_time);
		rq->group_load += cpu_time->prev_runnable_sum;
	}

	read_unlock_irqrestore(&related_thread_group_lock, flags);
}
#else
static inline void group_sync_window_start(struct rq *rq,
			struct group_cpu_time *cpu_time)
{
}

static inline void transfer_busy_time(struct rq *rq,
				      struct related_thread_group *grp,
				      struct task_struct *p, int event)
{
}
#endif

int sched_set_group_util_invalid_interval(unsigned int grp_id,
					  unsigned int interval)
{
	struct related_thread_group *grp = NULL;
	unsigned long flag;

	if (interval == 0)
		return -EINVAL;

	/* DEFAULT_CGROUP_COLOC_ID is a reserved id */
	if (grp_id == DEFAULT_CGROUP_COLOC_ID ||
	    grp_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set invalid interval for group %d fail\n", grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flag);
	if ((signed int)interval < 0)
		grp->util_invalid_interval = DEFAULT_UTIL_INVALID_INTERVAL;
	else
		grp->util_invalid_interval = interval * NSEC_PER_MSEC;
	raw_spin_unlock_irqrestore(&grp->lock, flag);

	return 0;
}

static inline bool
group_should_invalid_util(struct related_thread_group *grp, u64 now)
{
	if (grp->util_invalid_interval == DEFAULT_UTIL_INVALID_INTERVAL)
		return false;

	return (now - grp->last_freq_update_time >= grp->util_invalid_interval);
}

#ifdef CONFIG_HW_RTG_CPU_TIME
static inline void free_group_cputime(struct related_thread_group *grp)
{
	if (grp->cpu_time)
		free_percpu(grp->cpu_time);
}

static int alloc_group_cputime(struct related_thread_group *grp)
{
	int i;
	int ret;
	struct group_cpu_time *cpu_time = NULL;
	int cpu = raw_smp_processor_id();
	struct rq *rq = cpu_rq(cpu);
	u64 window_start = rq->window_start;

	grp->cpu_time = alloc_percpu(struct group_cpu_time);
	if (!grp->cpu_time)
		return -ENOMEM;

	for_each_possible_cpu(i) {
		cpu_time = per_cpu_ptr(grp->cpu_time, i);
		ret = memset_s(cpu_time, sizeof(*cpu_time), 0,
			       sizeof(*cpu_time));
		if (ret != 0) {
			pr_err("memset cpu time fail\n");
			return -ENODEV;
		}

		cpu_time->window_start = window_start;
	}

	return 0;
}
#endif

int alloc_related_thread_groups(void)
{
	int i, ret;
	struct related_thread_group *grp = NULL;

	/* groupd_id = 0 is invalid as it's special id to remove group. */
	for (i = 1; i < MAX_NUM_CGROUP_COLOC_ID; i++) {
		grp = kzalloc(sizeof(*grp), GFP_NOWAIT);
		if (!grp) {
			ret = -ENOMEM;
			goto err;
		}

#ifdef CONFIG_HW_RTG_CPU_TIME
		if (alloc_group_cputime(grp)) {
			kfree(grp);
			ret = -ENOMEM;
			goto err;
		}
#endif

		grp->id = i;
		grp->freq_update_interval = DEFAULT_FREQ_UPDATE_INTERVAL;
		grp->util_invalid_interval = DEFAULT_UTIL_INVALID_INTERVAL;
		grp->util_update_timeout = DEFAULT_UTIL_UPDATE_TIMEOUT;
#ifdef CONFIG_HW_RTG_BOOST
		grp->max_boost = INT_MIN;
#endif
		grp->window_size = NSEC_PER_SEC / DEFAULT_GROUP_RATE;
		grp->capacity_margin = DEFAULT_RTG_CAPACITY_MARGIN;
		INIT_LIST_HEAD(&grp->tasks);
		INIT_LIST_HEAD(&grp->list);
#ifdef CONFIG_HW_RTG_PSEUDO_TICK
		frame_pseudo_update(grp);
#endif
		raw_spin_lock_init(&grp->lock);

		related_thread_groups[i] = grp;
	}

	return 0;

err:
	for (i = 1; i < MAX_NUM_CGROUP_COLOC_ID; i++) {
		grp = lookup_related_thread_group(i);
		if (grp) {
#ifdef CONFIG_HW_RTG_CPU_TIME
			free_group_cputime(grp);
#endif
			kfree(grp);
			related_thread_groups[i] = NULL;
		} else {
			break;
		}
	}

	return ret;
}

static void _set_preferred_cluster(struct related_thread_group *grp,
				   int sched_cluster_id);
static void remove_task_from_group(struct task_struct *p)
{
	struct related_thread_group *grp = p->grp;
	struct rq *rq = NULL;
	bool empty_group = true;
	struct rq_flags flag;
	unsigned long irqflag;

	rq = __task_rq_lock(p, &flag);
	transfer_busy_time(rq, p->grp, p, REM_TASK);

	raw_spin_lock_irqsave(&grp->lock, irqflag);
	list_del_init(&p->grp_list);
	rcu_assign_pointer(p->grp, NULL);

	if (p->on_cpu)
		grp->nr_running--;

	if ((int)grp->nr_running < 0) {
		WARN_ON(1);
		grp->nr_running = 0;
	}

	if (!list_empty(&grp->tasks))
		empty_group = false;
#ifdef CONFIG_HW_CGROUP_RTG
		if (grp->id == DEFAULT_CGROUP_COLOC_ID)
			_do_update_preferred_cluster(grp);
#endif
	else {
#ifdef CONFIG_HW_RTG_BOOST
		grp->max_boost = 0;
#endif
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
		_set_preferred_cluster(grp, -1);
		grp->time.normalized_util = 0;
#endif
	}

	raw_spin_unlock_irqrestore(&grp->lock, irqflag);
	__task_rq_unlock(rq, &flag);

	/* Reserved groups cannot be destroyed */
	if (empty_group && grp->id != DEFAULT_CGROUP_COLOC_ID) {
		 /*
		  * We test whether grp->list is attached with list_empty()
		  * hence re-init the list after deletion.
		  */
		write_lock(&related_thread_group_lock);
		list_del_init(&grp->list);
		write_unlock(&related_thread_group_lock);
	}
}

static int
add_task_to_group(struct task_struct *p, struct related_thread_group *grp)
{
	struct rq *rq = NULL;
	struct rq_flags flag;
	unsigned long irqflag;
#ifdef CONFIG_HW_RTG_BOOST
	int boost;
#endif

	/*
	 * Change p->grp under rq->lock. Will prevent races with read-side
	 * reference of p->grp in various hot-paths
	 */
	rq = __task_rq_lock(p, &flag);
	transfer_busy_time(rq, grp, p, ADD_TASK);

	raw_spin_lock_irqsave(&grp->lock, irqflag);
	list_add(&p->grp_list, &grp->tasks);
	rcu_assign_pointer(p->grp, grp);

	if (p->on_cpu) {
		grp->nr_running++;
		if (grp->nr_running == 1)
			grp->mark_start = max(grp->mark_start,
					      walt_ktime_clock());
	}

#ifdef CONFIG_HW_RTG_BOOST
	boost = schedtune_task_boost(p);
	if (boost > grp->max_boost)
		grp->max_boost = boost;
#endif

#ifdef CONFIG_HW_CGROUP_RTG
	if (grp->id == DEFAULT_CGROUP_COLOC_ID)
		_do_update_preferred_cluster(grp);
#endif

	raw_spin_unlock_irqrestore(&grp->lock, irqflag);
	__task_rq_unlock(rq, &flag);

	return 0;
}

static int __sched_set_group_id(struct task_struct *p, unsigned int group_id)
{
	int rc = 0;
	unsigned long flags;
	struct related_thread_group *grp = NULL;
	struct related_thread_group *old_grp = NULL;

	if (group_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	raw_spin_lock_irqsave(&p->pi_lock, flags);
	old_grp = p->grp;
	if ((current != p && (p->flags & PF_EXITING)) ||
	    (!old_grp && !group_id))
		goto done;

	/*
	 * If the system has CONFIG_HW_CGROUP_RTG, only tasks in DEFAULT group
	 * can be directly switched to other groups.
	 *
	 * In other cases, Switching from one group to another directly is not permitted.
	 */
	if (old_grp && group_id) {
#ifdef CONFIG_HW_CGROUP_RTG
		if (old_grp->id == DEFAULT_CGROUP_COLOC_ID) {
			remove_task_from_group(p);
		} else {
#endif
			pr_err("%s[%d] switching group from %d to %d failed.\n",
			       p->comm, p->pid, old_grp->id, group_id);
			rc = -EINVAL;
			goto done;
#ifdef CONFIG_HW_CGROUP_RTG
		}
#endif
	}

	if (!group_id) {
		remove_task_from_group(p);
		goto done;
	}

	grp = lookup_related_thread_group(group_id);
	write_lock(&related_thread_group_lock);
	if (list_empty(&grp->list))
		list_add(&grp->list, &active_related_thread_groups);
	write_unlock(&related_thread_group_lock);

	rc = add_task_to_group(p, grp);
done:
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

	return rc;
}

/* group_id == 0: remove task from rtg */
int _sched_set_group_id(struct task_struct *p, unsigned int group_id)
{
	/* DEFAULT_CGROUP_COLOC_ID is a reserved id */
	if (group_id == DEFAULT_CGROUP_COLOC_ID)
		return -EINVAL;

	return __sched_set_group_id(p, group_id);
}

int sched_set_group_id(pid_t pid, unsigned int group_id)
{
	struct task_struct *p = NULL;
	int ret;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	/* Prevent p going away */
	get_task_struct(p);
	rcu_read_unlock();

	ret = _sched_set_group_id(p, group_id);
	put_task_struct(p);

	return ret;
}

unsigned int sched_get_group_id(struct task_struct *p)
{
	unsigned int group_id;
	struct related_thread_group *grp = NULL;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	group_id = grp ? grp->id : 0;
	rcu_read_unlock();

	return group_id;
}

void add_new_task_to_grp(struct task_struct *new)
{
	struct related_thread_group *grp = NULL;
	struct task_struct *leader = new->group_leader;
	unsigned int leader_grp_id = sched_get_group_id(leader);
	unsigned long flag;

	if (leader_grp_id != DEFAULT_CGROUP_COLOC_ID)
		return;

	if (thread_group_leader(new))
		return;
	if (leader_grp_id == DEFAULT_CGROUP_COLOC_ID) {
		if (!same_schedtune(new, leader))
			return;
	}

	rcu_read_lock();
	grp = task_related_thread_group(leader);
	rcu_read_unlock();

	/*
	 * It's possible that someone already added the new task to the
	 * group. A leader's thread group is updated prior to calling
	 * this function. It's also possible that the leader has exited
	 * the group. In either case, there is nothing else to do.
	 */
	if (!grp || new->grp)
		return;

	raw_spin_lock_irqsave(&grp->lock, flag);

	rcu_assign_pointer(new->grp, grp);
	list_add(&new->grp_list, &grp->tasks);

	raw_spin_unlock_irqrestore(&grp->lock, flag);
}

void sched_update_rtg_tick(struct rtg_tick_info *tick_info)
{
	struct related_thread_group *grp = NULL;

	if (tick_info == NULL || tick_info->curr == NULL)
		return;

	rcu_read_lock();
	grp = task_related_thread_group(tick_info->curr);
	if (!grp || list_empty(&grp->tasks)) {
		rcu_read_unlock();
		return;
	}

	if (grp->rtg_class)
		grp->rtg_class->sched_update_rtg_tick(grp, tick_info);

	rcu_read_unlock();
}

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
unsigned int get_cluster_grp_running(int cluster_id)
{
	struct related_thread_group *grp = NULL;
	unsigned int total_grp_running = 0;
	unsigned long flag, rtg_flag;
	unsigned int i;

	read_lock_irqsave(&related_thread_group_lock, rtg_flag);

	/* grp_id 0 is used for exited tasks */
	for (i = 1; i < MAX_NUM_CGROUP_COLOC_ID; i++) {
		grp = lookup_related_thread_group(i);
		if (!grp)
			continue;

		raw_spin_lock_irqsave(&grp->lock, flag);
		if (grp->preferred_cluster != NULL &&
		    grp->preferred_cluster->id == cluster_id)
			total_grp_running += grp->nr_running;
		raw_spin_unlock_irqrestore(&grp->lock, flag);
	}
	read_unlock_irqrestore(&related_thread_group_lock, rtg_flag);

	return total_grp_running;
}

int sched_set_group_freq(unsigned int grp_id, unsigned int freq)
{
	struct related_thread_group *grp = NULL;
	struct sched_cluster *prefer_cluster = NULL;
	int prefer_cluster_cpu;
	unsigned long flag;
	u64 now;

	/* DEFAULT_CGROUP_COLOC_ID is a reserved id */
	if (grp_id == DEFAULT_CGROUP_COLOC_ID ||
	    grp_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set freq for group %d fail\n", grp_id);
		return -ENODEV;
	}
	prefer_cluster = grp->preferred_cluster;
	if (!prefer_cluster) {
		pr_err("set freq for group %d fail, no cluster\n", grp_id);
		return -ENODEV;
	}
	prefer_cluster_cpu = cpumask_first(&prefer_cluster->cpus);

	raw_spin_lock_irqsave(&grp->lock, flag);
	now = ktime_get_ns();
	grp->us_set_min_freq = freq;
	grp->last_freq_update_time = now;
	raw_spin_unlock_irqrestore(&grp->lock, flag);

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
	sugov_mark_util_change(prefer_cluster_cpu, FORCE_UPDATE);
	sugov_check_freq_update(prefer_cluster_cpu);
#endif

	return 0;
}

static inline bool valid_normalized_util(struct related_thread_group *grp)
{
	struct task_struct *p = NULL;
	cpumask_t rtg_cpus = CPU_MASK_NONE;
	bool valid = false;

	if (grp->nr_running != 0) {
		list_for_each_entry(p, &grp->tasks, grp_list) {
			get_task_struct(p);
			if (p->state == TASK_RUNNING)
				cpumask_set_cpu(task_cpu(p), &rtg_cpus);
			trace_sched_rtg_task_each(grp->id, grp->nr_running, p);
			put_task_struct(p);
		}

		valid = cpumask_intersects(&rtg_cpus,
					  &grp->preferred_cluster->cpus);
	}

	trace_sched_rtg_valid_normalized_util(grp->id, grp->nr_running, &rtg_cpus, valid);
	return valid;
}

void sched_get_max_group_util(const struct cpumask *query_cpus,
			      unsigned long *util, unsigned int *freq)
{
	struct related_thread_group *grp = NULL;
	unsigned long max_grp_util = 0;
	unsigned int max_grp_freq = 0;
	u64 now = ktime_get_ns();
	unsigned long rtg_flag;
	unsigned long flag;

	/*
	 *  sum the prev_runnable_sum for each rtg,
	 *  return the max rtg->load
	 */
	read_lock_irqsave(&related_thread_group_lock, rtg_flag);
	if (list_empty(&active_related_thread_groups))
		goto unlock;

	for_each_related_thread_group(grp) {
		raw_spin_lock_irqsave(&grp->lock, flag);
		if (!list_empty(&grp->tasks) &&
		    grp->preferred_cluster != NULL &&
		    cpumask_intersects(query_cpus,
				       &grp->preferred_cluster->cpus) &&
		    !group_should_invalid_util(grp, now)) {
			if (grp->us_set_min_freq > max_grp_freq)
				max_grp_freq = grp->us_set_min_freq;

			if (grp->time.normalized_util > max_grp_util &&
			    valid_normalized_util(grp))
				max_grp_util = grp->time.normalized_util;
		}
		raw_spin_unlock_irqrestore(&grp->lock, flag);
	}

unlock:
	read_unlock_irqrestore(&related_thread_group_lock, rtg_flag);

	*freq = max_grp_freq;
	*util = max_grp_util;
}

int preferred_cluster(struct sched_cluster *cluster, struct task_struct *p)
{
	struct related_thread_group *grp = NULL;
	int rc = 1;

	rcu_read_lock();

	grp = task_related_thread_group(p);
	if (grp != NULL)
		rc = (grp->preferred_cluster == cluster);

	rcu_read_unlock();
	return rc;
}

static void _set_preferred_cluster(struct related_thread_group *grp,
				   int sched_cluster_id)
{
	struct sched_cluster *cluster = NULL;
	struct sched_cluster *cluster_found = NULL;

	if (sched_cluster_id == -1) {
		grp->preferred_cluster = NULL;
		return;
	}

	for_each_sched_cluster_reverse(cluster) {
		if (cluster->id == sched_cluster_id) {
			cluster_found = cluster;
			break;
		}
	}

	if (cluster_found != NULL)
		grp->preferred_cluster = cluster_found;
	else
		pr_err("cannot found sched_cluster_id=%d\n", sched_cluster_id);
}

/*
 * sched_cluster_id == -1: grp will set to NULL
 */
static void set_preferred_cluster(struct related_thread_group *grp,
				  int sched_cluster_id)
{
	unsigned long flag;

	raw_spin_lock_irqsave(&grp->lock, flag);
	_set_preferred_cluster(grp, sched_cluster_id);
	raw_spin_unlock_irqrestore(&grp->lock, flag);
}

int sched_set_group_preferred_cluster(unsigned int grp_id, int sched_cluster_id)
{
	struct related_thread_group *grp = NULL;

	/* DEFAULT_CGROUP_COLOC_ID is a reserved id */
	if (grp_id == DEFAULT_CGROUP_COLOC_ID ||
	    grp_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set preferred cluster for group %d fail\n", grp_id);
		return -ENODEV;
	}
	set_preferred_cluster(grp, sched_cluster_id);

	return 0;
}

#if defined(CONFIG_HW_RTG_BOOST) && defined(CONFIG_SCHED_TUNE)
#ifdef CONFIG_MTK_PLATFORM
extern long schedtune_margin(int cpu, unsigned long signal, long boost);
#else
extern long schedtune_margin(unsigned long signal, long boost);
#endif
int schedtune_grp_margin(unsigned long util, struct related_thread_group *grp)
{
	int boost = grp->max_boost;
	if (boost == 0)
		return 0;

#ifdef CONFIG_MTK_PLATFORM
	return schedtune_margin(-1, util, boost);
#else
	return schedtune_margin(util, boost);
#endif
}
#else
static inline int schedtune_grp_margin(unsigned long util,
				       struct related_thread_group *grp)
{
	return 0;
}
#endif

static struct sched_cluster *best_cluster(struct related_thread_group *grp)
{
	struct sched_cluster *cluster = NULL;
	struct sched_cluster *max_cluster = NULL;
	int cpu;
	unsigned long util = grp->time.normalized_util;
	unsigned long boosted_grp_util = util + schedtune_grp_margin(util, grp);
	unsigned long max_cap = 0;
	unsigned long cap = 0;

#ifdef CONFIG_HW_RTG_BOOST
	if (grp->max_boost > 0)
#ifdef CONFIG_HW_GLOBAL_BOOST
		if (global_boost_enable)
#endif
			return max_cap_cluster();
#endif

	/* find new cluster */
	for_each_sched_cluster(cluster) {
		cpu = cpumask_first(&cluster->cpus);
		cap = capacity_orig_of(cpu);
		if (cap > max_cap) {
			max_cap = cap;
			max_cluster = cluster;
		}

		if (boosted_grp_util <= cap)
			return cluster;
	}

	return max_cluster;
}

int sched_set_group_normalized_util(unsigned int grp_id, unsigned long util,
				    unsigned int flag)
{
	struct related_thread_group *grp = NULL;
	bool need_update_prev_freq = false;
	bool need_update_next_freq = false;
	u64 now;
	unsigned long flags;
	struct sched_cluster *preferred_cluster = NULL;
	int prev_cpu;
	int next_cpu;

	grp = lookup_related_thread_group(grp_id);
	if (!grp) {
		pr_err("set normalized util for group %d fail\n", grp_id);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&grp->lock, flags);

	if (list_empty(&grp->tasks)) {
		raw_spin_unlock_irqrestore(&grp->lock, flags);
		return 0;
	}

	grp->time.normalized_util = util;

	preferred_cluster = best_cluster(grp);

	/* update prev_cluster force when preferred_cluster changed */
	if (!grp->preferred_cluster) {
		grp->preferred_cluster = preferred_cluster;
	} else if (grp->preferred_cluster != preferred_cluster) {
		prev_cpu = cpumask_first(&grp->preferred_cluster->cpus);
		grp->preferred_cluster = preferred_cluster;

		need_update_prev_freq = true;
	}

	if (grp->preferred_cluster != NULL)
		next_cpu = cpumask_first(&grp->preferred_cluster->cpus);
	else
		next_cpu = 0;

	now = ktime_get_ns();
	grp->last_util_update_time = now;
	need_update_next_freq =
		group_should_update_freq(grp, next_cpu, flag, now);
	if (need_update_next_freq)
		grp->last_freq_update_time = now;

	raw_spin_unlock_irqrestore(&grp->lock, flags);

	if (need_update_prev_freq) {
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
		sugov_mark_util_change(prev_cpu, FRAME_UPDATE); //lint !e644
		sugov_check_freq_update(prev_cpu);
#endif
	}
	if (need_update_next_freq) {
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
		sugov_check_freq_update(next_cpu);
#endif
	}

	return 0;
}

struct cpumask *find_rtg_target(struct task_struct *p)
{
	struct related_thread_group *grp = NULL;
	struct sched_cluster *preferred_cluster = NULL;
	struct cpumask *rtg_target = NULL;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	rcu_read_unlock();

	if (!grp)
		return NULL;

#ifdef CONFIG_ARCH_KONA
	if (grp->id == DEFAULT_RT_FRAME_ID) {
		for_each_sched_cluster(preferred_cluster) {
			if (cpumask_first(&preferred_cluster->cpus) == 4)
				return &preferred_cluster->cpus;
		}
	}
#endif

	preferred_cluster = grp->preferred_cluster;
	if (!preferred_cluster)
		return NULL;

	rtg_target = &preferred_cluster->cpus;
#ifdef CONFIG_HW_EAS_SCHED
	if (!task_fits_max(p, cpumask_first(rtg_target)))
		return NULL;
#endif
	return rtg_target;
}

int find_rtg_cpu(struct task_struct *p, struct cpumask *preferred_cpus)
{
	int i;
	cpumask_t search_cpus = CPU_MASK_NONE;
	int max_spare_cap_cpu =-1;
	unsigned long max_spare_cap = 0;
	int idle_backup_cpu = -1;

	cpumask_and(&search_cpus, &p->cpus_allowed, cpu_online_mask);
	cpumask_andnot(&search_cpus, &search_cpus, cpu_isolated_mask);

	/* search the perferred idle cpu */
	for_each_cpu_and(i, &search_cpus, preferred_cpus) {
		if (is_reserved(i))
			continue;

		if (idle_cpu(i) || (i == task_cpu(p) && p->state == TASK_RUNNING)) {
			trace_find_rtg_cpu(p, preferred_cpus, "prefer_idle", i);
			return i;
		}
	}

	for_each_cpu(i, &search_cpus) {
		unsigned long spare_cap;

		if (walt_cpu_high_irqload(i))
			continue;

		if (is_reserved(i))
			continue;

		/* take the Active LB CPU as idle_backup_cpu */
		if (idle_cpu(i) || (i == task_cpu(p) && p->state == TASK_RUNNING)) {
			/* find the idle_backup_cpu with max capacity */
			if (idle_backup_cpu == -1 ||
				capacity_orig_of(i) > capacity_orig_of(idle_backup_cpu))
				idle_backup_cpu = i;

			continue;
		}

		spare_cap = capacity_spare_without(i, p);
		if (spare_cap > max_spare_cap) {
			max_spare_cap = spare_cap;
			max_spare_cap_cpu = i;
		}
	}

	if (idle_backup_cpu != -1) {
		trace_find_rtg_cpu(p, preferred_cpus, "idle_backup", idle_backup_cpu);
		return idle_backup_cpu;
	}

	trace_find_rtg_cpu(p, preferred_cpus, "max_spare", max_spare_cap_cpu);

	return max_spare_cap_cpu;
}
#else /* CONFIG_HW_RTG_NORMALIZED_UTIL */
static inline void _set_preferred_cluster(struct related_thread_group *grp,
					  int sched_cluster_id)
{
}
#endif /* CONFIG_HW_RTG_NORMALIZED_UTIL */

#ifdef CONFIG_HW_RTG_UCLAMP_UTIL
static inline int set_task_min_util_tick(struct related_thread_group *grp,
				struct task_struct *p, unsigned int new_util)
{
	struct rq *rq = NULL;
	unsigned int old_util;
	int cpu;
	bool should_update_freq = false;
	bool need_update_freq = false;
	u64 now = ktime_get_ns();
	unsigned long flag;

	if (!p || new_util > SCHED_CAPACITY_SCALE) {
		pr_err("%s invalid arg %u\n", __func__, new_util);
		return -EINVAL;
	}

	rq = task_rq(p);
	cpu = cpu_of(rq);

	old_util = p->util_req.min_util;
	p->util_req.min_util = new_util;
	trace_task_min_util(p);
	/*
	 * Nothing more to do for sleeping tasks or no change.
	 * Protected by p->pi_lock, we can safely check p->on_rq
	 * and check p's old min_util.
	 * Protected by rq->lock, we can safely manipulate rq's
	 * min_util_req list.
	 */
	if (task_on_rq_queued(p) && new_util != old_util) {
		if (new_util > old_util &&
			new_util > capacity_curr_of(cpu))
			should_update_freq = true;

		write_lock_irqsave(&rq->util_lock, flag);
		if (old_util == 0) {
			if (likely(list_empty(&p->util_req.min_util_entry)))
				list_add(&p->util_req.min_util_entry, &rq->min_util_req);
		}

		if (new_util == 0) {
			if (likely(!list_empty(&p->util_req.min_util_entry)))
				list_del_init(&p->util_req.min_util_entry);
		}
		write_unlock_irqrestore(&rq->util_lock, flag);
	}

	if (!should_update_freq)
		return 0;

	need_update_freq = group_should_update_freq(grp, cpu,
			FRAME_NORMAL_UPDATE,
			ktime_get_ns());

	if (need_update_freq) {
		grp->last_freq_update_time = now;
		read_lock_irqsave(&rq->util_lock, flag);
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
		sugov_check_freq_update(cpu);
#endif
		read_unlock_irqrestore(&rq->util_lock, flag);
	}

	return 0;
}

void sched_set_group_min_util(struct related_thread_group *grp, int min_util)
{
	struct task_struct *p = NULL;
	unsigned long flags;
	u64 now;

	if (min_util < 0 || min_util > SCHED_CAPACITY_SCALE)
		return;

	raw_spin_lock_irqsave(&grp->lock, flags);
	if (list_empty(&grp->tasks))
		goto unlock;

	list_for_each_entry(p, &grp->tasks, grp_list)
		set_task_min_util_tick(grp, p, min_util);

	now = ktime_get_ns();
	grp->last_util_update_time = now;

unlock:
	raw_spin_unlock_irqrestore(&grp->lock, flags);
}
#endif

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
int set_task_rtg_min_freq(struct task_struct *p, unsigned int freq)
{
	unsigned int cluster_cpu;
	struct related_thread_group *grp = NULL;
	struct sched_cluster *prefer_cluster = NULL;
#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	unsigned int util;
#endif
	unsigned long flags;
	int ret = 0;

	if (!p)
		return -EINVAL;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	rcu_read_unlock();

	if (!grp)
		return -ENODEV;
	prefer_cluster = grp->preferred_cluster;
	if (!prefer_cluster)
		return -ENODEV;

	raw_spin_lock_irqsave(&grp->lock, flags);
	grp->us_set_min_freq = freq;
	grp->last_freq_update_time = ktime_get_ns();
	raw_spin_unlock_irqrestore(&grp->lock, flags);

	cluster_cpu = cpumask_first(&prefer_cluster->cpus);

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
	sugov_mark_util_change(cluster_cpu, FORCE_UPDATE);
#endif

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	util = freq_to_util(cluster_cpu, freq);
	ret = set_task_min_util(p, util);
#endif

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
	sugov_check_freq_update(cluster_cpu);
#endif

	return ret;
}
#endif

#ifdef CONFIG_HW_CGROUP_RTG
#include "rtg_cgroup.c"
#endif
