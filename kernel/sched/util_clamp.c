/*
 * util_clamp.c
 *
 * task util limit
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
int set_task_max_util(struct task_struct *p, unsigned int new_util)
{
	if (!p || new_util > SCHED_CAPACITY_SCALE) {
		pr_err("%s invalid arg %u\n", __func__, new_util);
		return -EINVAL;
	}

	p->util_req.max_util = new_util;
	return 0;
}

unsigned int get_task_max_util(struct task_struct *p)
{
	return p->util_req.max_util;
}

/* called with rq->lock held */
bool update_rq_min_util_req(struct task_struct *p, struct rq *rq,
			    unsigned int new_util)
{
	bool need_update = false;
	unsigned int old_util;

	if (!p || !rq)
		return false;

	lockdep_assert_held(&rq->lock);

	old_util = p->util_req.min_util;
	p->util_req.min_util = new_util;

	/*
	 * Nothing more to do for sleeping tasks or no change.
	 * Protected by p->pi_lock, we can safely check p->on_rq
	 * and check p's old min_util.
	 * Protected by rq->lock, we can safely manipulate rq's
	 * min_util_req list.
	 */
	if (task_on_rq_queued(p) && new_util != old_util) {
		if (new_util > old_util &&
		    new_util > capacity_curr_of(cpu_of(rq)))
			need_update = true;

		if (old_util == 0) {
			WARN_ON(!list_empty(&p->util_req.min_util_entry));
			list_add(&p->util_req.min_util_entry, &rq->min_util_req);
		}

		if (new_util == 0) {
			WARN_ON(list_empty(&p->util_req.min_util_entry));
			list_del_init(&p->util_req.min_util_entry);
		}
	}

	return need_update;
}

int set_task_min_util(struct task_struct *p, unsigned int new_util)
{
	struct rq_flags rf;
	struct rq *rq = NULL;
	int cpu;
	bool should_update_freq = false;

	if (!p || new_util > SCHED_CAPACITY_SCALE) {
		pr_err("%s invalid arg %u\n", __func__, new_util);
		return -EINVAL;
	}

	rq = task_rq_lock(p, &rf);
	cpu = cpu_of(rq);

	should_update_freq = update_rq_min_util_req(p, rq, new_util);

	task_rq_unlock(rq, p, &rf);

	if (should_update_freq) {
		sugov_mark_util_change(cpu, RQ_SET_MIN_UTIL);
		sugov_check_freq_update(cpu);
	}

	return 0;
}

unsigned int get_task_min_util(struct task_struct *p)
{
	return p->util_req.min_util;
}

void add_freq_request(struct rq *rq, struct task_struct *p)
{
	struct util_clamp *req = &p->util_req;

	if (unlikely(req->min_util)) {
		if (unlikely(!list_empty(&req->min_util_entry))) {
			pr_warn("error when add req %d\n", p->pid);
			return;
		}

		list_add(&req->min_util_entry, &rq->min_util_req);

		if (req->min_util > capacity_curr_of(cpu_of(rq)))
			sugov_mark_util_change(cpu_of(rq), RQ_ENQUEUE_MIN_UTIL);
	}
}

void del_freq_request(struct task_struct *p)
{
	struct util_clamp *req = &p->util_req;

	if (unlikely(req->min_util)) {
		if (unlikely(list_empty(&req->min_util_entry))) {
			pr_warn("error when del req %d\n", p->pid);
			return;
		}

		list_del_init(&req->min_util_entry);
	}
}

unsigned int get_min_util(struct rq *rq)
{
	struct util_clamp *req = NULL;
	unsigned int ret = 0;
	int loop_max = 10;

	if (likely(list_empty(&rq->min_util_req)))
		return 0;

	lockdep_assert_held(&rq->lock);

	list_for_each_entry(req, &rq->min_util_req, min_util_entry) {
		/* Return the highest min_util req. */
		if (req->min_util > ret)
			ret = req->min_util;

		/*
		 * Meaningless to have too many min_util reqs.
		 * Protect us against attack.
		 */
		if (--loop_max <= 0)
			break;
	}

	return ret;
}
#endif
