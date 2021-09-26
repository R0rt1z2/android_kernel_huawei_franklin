/*
 * vip.c
 *
 * huawei vip scheduling
 *
 * Copyright (c) 2018-2020 Huawei Technologies Co., Ltd.
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

#define VIP_TASK_LIMIT_RUNNING_NS 50000000 /* 50ms */
#define VIP_TASK_LIMIT_ITER_NUM   20 /* max 20 for an rq */
unsigned int vip_task_load_balance_delay = 300000UL; /* 300 us */

static void enqueue_vip_prio(struct rq *rq, struct task_struct *p)
{
	if (!p->vip_prio)
		return;

	if (unlikely(!list_empty(&p->vip_prio_entry))) {
		pr_warn("task %d not dequeued as vip\n", p->pid);
		return;
	}

	p->vip_last_queued = rq_clock(rq);
	list_add(&p->vip_prio_entry, &rq->vip_prio_thread_list);

#ifdef CONFIG_HUAWEI_SCHED_VIP
	if (p->vip_prio > rq->highest_vip_prio)
		rq->highest_vip_prio = p->vip_prio;
#endif
}

static void dequeue_vip_prio(struct rq *rq, struct task_struct *p)
{
	unsigned int highest_prio, task_cnt;
	struct task_struct *task = NULL;

	if (!p->vip_prio)
		return;

	if (unlikely(list_empty(&p->vip_prio_entry))) {
		pr_warn("task %d not enqueued as vip\n", p->pid);
		return;
	}

	p->vip_last_queued = 0;
	list_del_init(&p->vip_prio_entry);

#ifdef CONFIG_HUAWEI_SCHED_VIP
	highest_prio = 0;
	task_cnt = 0;
	list_for_each_entry(task, &rq->vip_prio_thread_list, vip_prio_entry) {
		highest_prio = max(highest_prio, task->vip_prio);

		if (++task_cnt >= VIP_TASK_LIMIT_ITER_NUM)
			break;
	}

	rq->highest_vip_prio = highest_prio;
#endif
}

void sched_enqueue_vip_thread(struct rq *rq, struct task_struct *p)
{
	enqueue_vip_prio(rq, p);

#ifdef CONFIG_HW_VIP_THREAD
	enqueue_vip_thread(rq, p);
#endif
}

void sched_dequeue_vip_thread(struct rq *rq, struct task_struct *p)
{
	dequeue_vip_prio(rq, p);

#ifdef CONFIG_HW_VIP_THREAD
	dequeue_vip_thread(rq, p);
#endif
}

struct task_struct *pick_next_vip_thread(struct rq *rq)
{
	struct task_struct *task = NULL, *leftmost_task = NULL, *n = NULL;
	unsigned int task_cnt = 0;

	list_for_each_entry_safe(task, n, &rq->vip_prio_thread_list, vip_prio_entry) {
		if (unlikely(task_rq(task) != rq)) {
			pr_warn("task %d on another rq\n", task->pid);
			list_del_init(&task->vip_prio_entry);
			continue;
		}

		if (unlikely(!task_on_rq_queued(task))) {
			pr_warn("task %d not queued\n", task->pid);
			list_del_init(&task->vip_prio_entry);
			continue;
		}

		if (!leftmost_task ||
		    task->vip_prio > leftmost_task->vip_prio ||
		    (task->vip_prio == leftmost_task->vip_prio &&
		     entity_before(&task->se, &leftmost_task->se)))
			leftmost_task = task;

		if (++task_cnt >= VIP_TASK_LIMIT_ITER_NUM)
			break;
	}

	if (leftmost_task) {
		s64 enqueue_time;

		trace_sched_pick_next_vip(leftmost_task);

		/*
		 * Waking someone with WF_SYNC.
		 * Let the wakee have a chance to preempt and run.
		 */
		if (test_tsk_thread_flag(leftmost_task, TIF_WAKE_SYNC))
			return NULL;

		/*
		 * Skip picking the vip task if it has been enqueued too long
		 * to prevent starving or even livelock.
		 */
		enqueue_time = rq_clock(rq) -
				leftmost_task->vip_last_queued;
		if (unlikely(enqueue_time > VIP_TASK_LIMIT_RUNNING_NS))
			return NULL;
	}

	return leftmost_task;
}

int set_vip_prio(struct task_struct *p, unsigned int prio)
{
	struct rq_flags rf;
	struct rq *rq = NULL;
	bool queued_in_vip_list = false;

	rq = task_rq_lock(p, &rf);

	queued_in_vip_list = (p->sched_class == &fair_sched_class) &&
			     task_on_rq_queued(p);

	if (queued_in_vip_list) {
		update_rq_clock(rq);
		dequeue_vip_prio(rq, p);
	}

	p->vip_prio = prio;

	if (queued_in_vip_list)
		enqueue_vip_prio(rq, p);

	task_rq_unlock(rq, p, &rf);
	return 0;
}

#ifdef CONFIG_HUAWEI_SCHED_VIP
int find_lowest_vip_cpu(struct task_struct *p,
			struct cpumask *search_cpus)
{
	int i;
	int target_cpu = -1;
	unsigned int lowest_prio = p->vip_prio;
	unsigned long max_cap = 0;
	unsigned long min_util = ULONG_MAX;

	for_each_cpu(i, search_cpus) {
		unsigned int prio = cpu_rq(i)->highest_vip_prio;
		unsigned long cap = capacity_orig_of(i);
		unsigned long util = cpu_util_without(i, p);

		if (p->state == TASK_RUNNING &&
		    is_reserved(i))
			continue;

		if (prio > lowest_prio)
			continue;

		if (prio == lowest_prio) {
			if (cap < max_cap)
				continue;
			if (cap == max_cap &&
			    util >= min_util)
				continue;
		}

		lowest_prio = prio;
		max_cap = cap;
		min_util = util;
		target_cpu = i;
	}

	return target_cpu;
}

static const unsigned int sysctl_vip_separate_threshold = 20;
static inline bool check_vip_contend(int cpu, struct task_struct *p)
{
	struct rq *rq = cpu_rq(cpu);
	struct rt_rq *rt_rq = &rq->rt;

	/* If curr task is a per-cpu task, it contends vip. */
	if (cpu_curr(cpu)->nr_cpus_allowed == 1 &&
	    cpu_curr(cpu)->sched_class == p->sched_class)
		return true;

	/* If the cpu has rt task , it contends vip. */
	if (rt_rq->rt_nr_running)
		return true;

	/* No vip, no contention. */
	if (list_empty(&rq->vip_prio_thread_list))
		return false;

	/* VIP_STRIC_EXCLUSIVE always try to separate vip threads. */
	if (sched_feat(VIP_STRIC_EXCLUSIVE))
		return true;

	/*
	 * Now we know there're vip threads on this cpu.
	 * Compare the vip prio.
	 */
	if (p->vip_prio < rq->highest_vip_prio)
		return true;
	else if (p->vip_prio > rq->highest_vip_prio)
		return false;

	/*
	 * Now we see same prio vip threads on this cpu.
	 * If p is checking for migration, allow task_cpu only.
	 */
	if (p->state == TASK_RUNNING)
		return task_cpu(p) != cpu;

	/*
	 * If the cpu util is larger than curr_cpu_cap * 20%, contention
	 * might harm performance; Otherwise we accept it.
	 */
	return cpu_util_without(cpu, p) * 100 >
		capacity_curr_of(cpu) * sysctl_vip_separate_threshold;
}

int find_vip_cpu(struct task_struct *p)
{
	int i;
	int target_cpu = -1;
	int preferred_cpu = -1;
	cpumask_t search_cpus = CPU_MASK_NONE;
	struct cpumask *rtg_target = NULL;
	bool favor_larger_capacity = false;
	int min_idle_idx = INT_MAX;
	int shallowest_idle_cpu = -1;
	u64 latest_idle_timestamp = 0;
	unsigned int min_vip_prio = UINT_MAX;
	unsigned long min_wake_util = ULONG_MAX;
	unsigned long target_capacity = ULONG_MAX;

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	rtg_target = find_rtg_target(p);
#endif
	if (rtg_target)
		favor_larger_capacity = true;

#ifdef CONFIG_HW_GLOBAL_BOOST
	if (global_boost_enable && schedtune_task_boost(p) > 0)
		favor_larger_capacity = true;
#endif

	if (favor_larger_capacity)
		target_capacity = 0;

	cpumask_and(&search_cpus, &p->cpus_allowed, cpu_online_mask);
	cpumask_andnot(&search_cpus, &search_cpus, cpu_isolated_mask);

	for_each_cpu(i, &search_cpus) {
		struct rq *rq = cpu_rq(i);
		unsigned int cpu_vip_prio = rq->highest_vip_prio;
		unsigned long capacity_orig = capacity_orig_of(i);
		unsigned long wake_util = cpu_util_without(i, p);
		int idle_idx = idle_cpu(i) ? idle_get_state_idx(rq)
					: INT_MAX;

		trace_sched_find_vip_cpu_each(
			p, i, !list_empty(&rq->vip_prio_thread_list),
			cpu_vip_prio, is_reserved(i), walt_cpu_high_irqload(i),
			capacity_orig, wake_util,
			rtg_target && cpumask_test_cpu(i, rtg_target));

		if (check_vip_contend(i, p))
			continue;

		if (is_reserved(i))
			continue;

		if (walt_cpu_high_irqload(i))
			continue;

		if (rtg_target && cpumask_test_cpu(i, rtg_target)) {
			if (idle_cpu(i)) {
				if (idle_idx < min_idle_idx) {
					min_idle_idx = idle_idx;
					latest_idle_timestamp = rq->idle_stamp;
					shallowest_idle_cpu = i;
				} else if (idle_idx == min_idle_idx &&
					   rq->idle_stamp > latest_idle_timestamp) {
					/*
					 * If equal or no active idle state, then
					 * the most recently idled CPU might have
					 * a warmer cache.
					 */
					latest_idle_timestamp = rq->idle_stamp;
					shallowest_idle_cpu = i;
				}
			} else if (shallowest_idle_cpu == -1) {
				if (preferred_cpu != -1) {
					if (cpu_vip_prio > min_vip_prio)
						continue;

					if (cpu_vip_prio == min_vip_prio &&
					    wake_util > min_wake_util)
						continue;
				}

				preferred_cpu = i;
				min_vip_prio = cpu_vip_prio;
				min_wake_util = wake_util;
			}
		}

		if (shallowest_idle_cpu != -1 || preferred_cpu != -1)
			continue;

#ifdef CONFIG_HW_EAS_SCHED
		if (!task_fits_max(p, i))
			continue;
#endif

		/* Favor larger/smaller capacity cpu when possible. */
		if (favor_larger_capacity) {
			if (capacity_orig < target_capacity)
				continue;
		} else {
			if (capacity_orig > target_capacity)
				continue;
		}

		if (capacity_orig == target_capacity) {
			if (cpu_vip_prio > min_vip_prio)
				continue;
			if (cpu_vip_prio == min_vip_prio) {
				if (wake_util > min_wake_util)
					continue;
				if (wake_util == min_wake_util &&
				    idle_idx >= min_idle_idx)
					continue;
			}
		}

		target_cpu = i;
		target_capacity = capacity_orig;
		min_vip_prio = cpu_vip_prio;
		min_wake_util = wake_util;
		min_idle_idx = idle_idx;
	}

	if (shallowest_idle_cpu != -1)
		target_cpu = shallowest_idle_cpu;
	else if (preferred_cpu != -1)
		target_cpu = preferred_cpu;

	/*
	 * If we didn't find a suitable non-vip cpu, try to find a cpu
	 * with lowest vip prio and do not consider power efficiency.
	 */
	if (target_cpu == -1)
		target_cpu = find_lowest_vip_cpu(p, &search_cpus);

	trace_sched_find_vip_cpu(p, preferred_cpu,
			favor_larger_capacity, target_cpu);

	return target_cpu;
}

enum vip_preempt_type vip_preempt_classify(struct rq *rq,
	struct task_struct *prev, struct task_struct *next, int wake_sync)
{
	/* Switching to a normal(not vip) cfs task or idle task. */
	if (!next->vip_prio && next->sched_class != &rt_sched_class)
		return NO_VIP_PREEMPT;

	/* Prev give up the cpu willingly. */
	if (wake_sync)
		return NO_VIP_PREEMPT;
	/*
	 * Switching to a vip/rt task.
	 * There might be a runnable task starving so give load balancer
	 * a hint.
	 */
	if (prev->on_rq) {
		/*
		 * Mark witch type of task is preempted.
		 */
		if (prev->vip_prio)
			return VIP_PREEMPT_VIP;
		if (schedtune_prefer_idle(prev))
			return VIP_PREEMPT_TOPAPP;
		if (prev->sched_class == &fair_sched_class)
			return VIP_PREEMPT_OTHER;
	} else {
		/*
		 * No preemption.
		 * Check if there are runnable tasks(prev task
		 * has been deactivated at this point).
		 */
		return rq->nr_running > 1 ?
			VIP_PREEMPT_OTHER : NO_VIP_PREEMPT;
	}

	/*
	 * Keep it unchanged.
	 * Note that cpu_overutilized_for_lb() will update it if necessary.
	 */
	return rq->vip_preempt_type;
}

void fixup_vip_preempt_type(struct rq *rq)
{
	if (rq->vip_preempt_type &&
	    rq->nr_running <= 1) {
		rq->vip_preempt_type = NO_VIP_PREEMPT;
		return;
	}

	if (!rq->vip_preempt_type &&
	    rq->highest_vip_prio &&
	    rq->nr_running > 1 &&
	    !(rq->sync_waiting && rq->nr_running == 2)) {
		rq->vip_preempt_type = VIP_PREEMPT_OTHER;
		return;
	}
}

/*
 * We are going to trigger a nohz balance. If we have vip_preempt_type, allow
 * relative sched domains to do balance(ASYM_CPUCAPACITY level sd needs
 * sd_overutilized flag).
 */
void sched_vip_balance_set_overutilized(int cpu)
{
	struct sched_domain *sd = NULL;

	if (likely(cpu_rq(cpu)->vip_preempt_type == NO_VIP_PREEMPT))
		return;

	rcu_read_lock();
	for_each_domain(cpu, sd)
		set_sd_overutilized(sd);
	rcu_read_unlock();
}
#endif
