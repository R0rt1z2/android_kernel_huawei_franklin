/*
 * Huawei Energy Aware Scheduling File
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

#include <linux/sched/sysctl.h>
#include <chipset_common/hwcfs/hwcfs_common.h>

#ifdef CONFIG_HW_SCHED_PRED_LOAD
#include "../hw_walt/pred_load.h"
#endif

unsigned int up_migration_util_filter = 25;
unsigned int sysctl_sched_force_upmigrate_duration = 50000000; /* 50 ms */

bool cpu_overutilized(int cpu)
{
	return (capacity_of(cpu) * 1024) < (cpu_util(cpu) * hw_sd_capacity_margin(cpu));
}

void mark_reserved(int cpu)
{
	cpu_rq(cpu)->reserved = 1;
}

void clear_reserved(int cpu)
{
	cpu_rq(cpu)->reserved = 0;
}

static inline bool skip_cpu(int cpu)
{
	/* Check all conditions other than util here. */
	return !cpu_online(cpu) ||
	       cpu_isolated(cpu) ||
	       walt_cpu_high_irqload(cpu) ||
	       is_reserved(cpu);
}

int wake_to_idle(struct task_struct *p)
{
	return p->flags & PF_WAKE_UP_IDLE;
}

/*
 * This function determines if we should do load balance
 * in ASYM_CPUCAPACITY level sched domain.
 */
bool cpu_overutilized_for_lb(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (idle_cpu(cpu))
		return false;

#ifdef CONFIG_HUAWEI_SCHED_VIP
	fixup_vip_preempt_type(rq);
	if (rq->vip_preempt_type)
		return true;
#endif

	if (capacity_of(cpu) * SCHED_CAPACITY_SCALE <
	    cpu_util(cpu) * hw_sd_capacity_margin(cpu))
		return true;

	/* For misfit task and rtg misfit task. */
	if (rq->misfit_task_load)
		return true;

	return false;
}

static inline unsigned int
hw_capacity_margin_general(struct task_struct *p, int cpu)
{
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	struct related_thread_group *grp = NULL;

	rcu_read_lock();
	grp = task_related_thread_group(p);
	rcu_read_unlock();
	if (grp && grp->preferred_cluster)
		return grp->capacity_margin;
#endif

#ifdef CONFIG_HW_RT_CAS
	if (p->sched_class == &rt_sched_class)
		return sysctl_sched_rt_capacity_margin;
#endif
	return hw_capacity_margin(cpu);
}

static bool task_fits(struct task_struct *p, int cpu)
{
	unsigned long capacity;
	unsigned long boosted_util = boosted_task_util(p);
	unsigned int margin = hw_capacity_margin_general(p, cpu);

	/*
	 * For RT threads, the available CPU capacity is the whole,
	 * so use capacity_orig_of here.
	 *
	 * But for the CFS threads, unfortunately, it should subtract
	 * the capacity that RT consumed.
	 */
	if (p->sched_class == &rt_sched_class)
		capacity = capacity_orig_of(cpu);
	else
		capacity = capacity_of(cpu);

#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	if (capacity < p->util_req.min_util)
		return false;
	if (capacity >= p->util_req.max_util)
		return true;
#endif
	if (p->sched_class == &rt_sched_class) {
		return ((capacity << SCHED_CAPACITY_SHIFT) >=
			(boosted_util * margin)) &&
		       (capacity >= boosted_util);
	} else {
		return (capacity << SCHED_CAPACITY_SHIFT) >=
			(boosted_util * margin);
	}
}

bool task_fits_max(struct task_struct *p, int cpu)
{
	unsigned long capacity_orig = capacity_orig_of(cpu);
	unsigned long max_capacity = cpu_rq(cpu)->rd->max_cpu_capacity.val;
	cpumask_t allowed_cpus;
	int allowed_cpu;

	if (capacity_orig == max_capacity)
		return true;

	if (task_fits(p, cpu))
		return true;

	/* Now task does not fit. Check if there's a better one. */
	cpumask_and(&allowed_cpus, &p->cpus_allowed, cpu_online_mask);
	for_each_cpu(allowed_cpu, &allowed_cpus) {
		if (capacity_orig_of(allowed_cpu) > capacity_orig)
			return false; /* Misfit */
	}

	/* Already largest capacity in allowed cpus. */
	return true;
}

unsigned long capacity_min_of(int cpu)
{
	if (!sched_feat(MIN_CAPACITY_CAPPING))
		return 0;
	return arch_scale_cpu_capacity(NULL, cpu) *
		  arch_scale_min_freq_capacity(NULL, cpu)
		  >> SCHED_CAPACITY_SHIFT;
}

void check_prev_cpu_in_candidates(struct energy_env *eenv)
{
	int prev_cpu = eenv->cpu[EAS_CPU_PRV].cpu_id;
	unsigned long new_util;

	new_util = cpu_util_without(prev_cpu, eenv->p) + task_util_est(eenv->p);

	/* If prev cpu is not a good choice, drop it from energy compare candidates */
	if ((capacity_of(prev_cpu) * 1024 < new_util * hw_sd_capacity_margin(prev_cpu)) ||
	    skip_cpu(prev_cpu)) {
		int next_cpu = eenv->cpu[EAS_CPU_NXT].cpu_id;

		/* Replace PREV slot with next_cpu */
		if (next_cpu != -1)
			eenv->cpu[EAS_CPU_PRV].cpu_id = next_cpu;
	}
}

/*
 * find_boost_cpu - find the idlest cpu among the fast_cpus.
 */
int find_boost_cpu(struct cpumask *group_cpus, struct task_struct *p, int this_cpu)
{
	unsigned long load, min_load = ULONG_MAX;
	unsigned int min_exit_latency = UINT_MAX;
	u64 latest_idle_timestamp = 0;
	int least_loaded_cpu = this_cpu;
	int shallowest_idle_cpu = -1;
	int i;

	rcu_read_lock();
	/* Traverse only the allowed CPUs */
	for_each_cpu_and(i, group_cpus, &p->cpus_allowed) {
		if (!cpumask_test_cpu(i, cpu_online_mask))
			continue;

		if (idle_cpu(i)) {
			struct rq *rq = cpu_rq(i);
			struct cpuidle_state *idle = idle_get_state(rq);
			if (idle && idle->exit_latency < min_exit_latency) {
				/*
				 * We give priority to a CPU whose idle state
				 * has the smallest exit latency irrespective
				 * of any idle timestamp.
				 */
				min_exit_latency = idle->exit_latency;
				latest_idle_timestamp = rq->idle_stamp;
				shallowest_idle_cpu = i;
			} else if ((!idle || idle->exit_latency == min_exit_latency) &&
				   rq->idle_stamp > latest_idle_timestamp) {
				/*
				 * If equal or no active idle state, then
				 * the most recently idled CPU might have
				 * a warmer cache.
				 */
				latest_idle_timestamp = rq->idle_stamp;
				shallowest_idle_cpu = i;
			}
		} else {
			load = weighted_cpuload(cpu_rq(i));
			if (load < min_load || (load == min_load && i == this_cpu)) {
				min_load = load;
				least_loaded_cpu = i;
			}
		}
	}

	rcu_read_unlock();
	return shallowest_idle_cpu != -1 ? shallowest_idle_cpu : least_loaded_cpu;
}

/*
 * Find the smallest cap cpus with spared capacity and pick the
 * max spared capacity one.
 */
int find_slow_cpu(struct task_struct *p)
{
	cpumask_t search_cpus;
	unsigned long target_cap = ULONG_MAX;
	unsigned long max_spare_cap = 0;
	int i;
	int target_cpu = -1;

	if (p->state != TASK_WAKING)
		return -1;

	cpumask_and(&search_cpus, &p->cpus_allowed, cpu_online_mask);
	cpumask_andnot(&search_cpus, &search_cpus, cpu_isolated_mask);

	for_each_cpu(i, &search_cpus) {
		unsigned long cap_orig, spare_cap;

		cap_orig = capacity_orig_of(i);
		if (cap_orig > target_cap)
			continue;

		spare_cap = capacity_spare_without(i, p);
		if (spare_cap <= max_spare_cap)
			continue;

		target_cpu = i;
		target_cap = cap_orig;
		max_spare_cap = spare_cap;
	}

	return target_cpu;
}

void print_hung_task_sched_info(struct task_struct *p)
{
	u64 sleep_ts = 0;
	u64 enqueue_ts = 0;
	u64 min_vruntime = 0;
	u64 vruntime = 0;

	if (!p)
		return;

	get_task_struct(p);
	enqueue_ts = p->last_enqueued_ts;

	if (p->sched_class == &fair_sched_class) {
		vruntime = p->se.vruntime;
		if (p->se.on_rq)
			min_vruntime = p->se.cfs_rq->min_vruntime;
	}

	pr_err("comm:%s pid:%d prio:%d cpu:%d vruntime:%llu min:%llu "
	       "allowed:0x%lx la:%llu lq:%llu sleep:%llu enqueue:%llu\n",
	       p->comm, p->pid, p->prio, task_cpu(p), vruntime, min_vruntime,
	       p->cpus_allowed.bits[0], p->sched_info.last_arrival,
	       p->sched_info.last_queued, sleep_ts, enqueue_ts);

	put_task_struct(p);
}

void print_cpu_rq_info(void)
{
	int cpu;
	struct rq *rq = NULL;
	struct task_struct *p = NULL;
	struct task_struct *n = NULL;
	unsigned long flags;

	get_online_cpus();
	for_each_online_cpu(cpu) { /*lint !e574*/
		rq = cpu_rq(cpu);

		raw_spin_lock_irqsave(&rq->lock, flags);

		if (!rq->nr_running)
			goto unlock;

		pr_err("cpu:%d clock:%lld nr:%d cfs_nr:%d\n",
		       cpu, rq->clock, rq->nr_running, rq->cfs.h_nr_running);

		print_hung_task_sched_info(rq->curr);

		if (!rq->cfs.h_nr_running)
			goto unlock;

		list_for_each_entry_safe(p, n, &rq->cfs_tasks, se.group_node) {
			if (!p)
				break;

			if (p == rq->curr)
				continue;

			if (rq->clock - p->sched_info.last_queued < 5 * NSEC_PER_SEC)
				continue;

			print_hung_task_sched_info(p);
		}
unlock:
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}
	put_online_cpus();
}

#ifdef CONFIG_HW_SCHED_PRED_LOAD
static ssize_t predl_window_size_show(struct kobject *kobj,
			struct kobj_attribute *kattr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", predl_window_size);
}

#define MIN_PREDL_WINDOW_SIZE 4000000 /* 4ms */
#define MAX_PREDL_WINDOW_SIZE 64000000 /* 64ms */
static ssize_t predl_window_size_store(struct kobject *kobj,
		struct kobj_attribute *kattr, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	if (val < MIN_PREDL_WINDOW_SIZE ||
	    val > MAX_PREDL_WINDOW_SIZE)
		return -EINVAL;

	predl_window_size = val;

	return count;
}
#endif

#define EAS_DATA_SYSFS_MAX	11

struct eas_global_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
			struct kobj_attribute *kattr, char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *kattr,
			const char *buf, size_t count);
	int *value;
};

struct eas_data_struct {
	struct attribute_group attr_group;
	struct attribute *attributes[EAS_DATA_SYSFS_MAX + 1];
	struct eas_global_attr attr[EAS_DATA_SYSFS_MAX];
} eas_data;

static ssize_t eas_show(struct kobject *kobj,
				struct kobj_attribute *kattr, char *buf)
{
	struct eas_global_attr *eas_attr =
		container_of(&kattr->attr, struct eas_global_attr, attr);
	int temp;

	temp = *(eas_attr->value);
	return (ssize_t)sprintf_s(buf, PAGE_SIZE, "%d\n", temp);
}

static ssize_t eas_store(struct kobject *kobj, struct kobj_attribute *kattr,
				const char *buf, size_t count)
{
	int rc;
	int temp;
	ssize_t ret = count;
	struct eas_global_attr *eas_attr =
		container_of(&kattr->attr, struct eas_global_attr, attr);
	char *str = vmalloc(count + 1);

	if (str == NULL)
		return -ENOMEM;

	rc = memcpy_s(str, count, buf, count);
	if (rc != 0)
		return -EINVAL;

	str[count] = 0;
	if (sscanf_s(str, "%d", &temp) < 1)
		ret = -EINVAL;
	else {
		if (temp < 0)
			ret = -EINVAL;
		else
			*(eas_attr->value) = temp;
	}

#ifdef CONFIG_HW_EAS_TRACE
	/* trace the name and value of the attribute */
	trace_eas_attr_store(kattr->attr.name, temp);
#endif

	vfree(str);
	return ret;
}

static void eas_attr_add(
	const char *name,
	int *value,
	ssize_t (*show)(struct kobject *,
			struct kobj_attribute *, char *),
	ssize_t (*store)(struct kobject *, struct kobj_attribute *,
			const char *, size_t),
	umode_t mode)
{
	int i = 0;

	while (eas_data.attributes[i] != NULL) {
		i++;
		if (i >= EAS_DATA_SYSFS_MAX)
			return;
	}

	if (mode)
		eas_data.attr[i].attr.mode = mode;
	else
		eas_data.attr[i].attr.mode = 0640;
	if (show)
		eas_data.attr[i].show = show;
	else
		eas_data.attr[i].show = eas_show;
	if (store)
		eas_data.attr[i].store = store;
	else
		eas_data.attr[i].store = eas_store;

	eas_data.attr[i].attr.name = name;
	eas_data.attr[i].value = value;
	eas_data.attributes[i] = &eas_data.attr[i].attr;
	eas_data.attributes[i + 1] = NULL;
}

/*
 * Note:
 * 1. Do not forget to increase array size EAS_DATA_SYSFS_MAX when
 * adding a new attribute.
 * 2. The second parameter of eas_attr_add will be transformed to
 * pointer to int, so don't pass type shorter than int.
 */
static int eas_attr_init(void)
{
	int ret;

	memset(&eas_data, 0, sizeof(eas_data));

#ifdef CONFIG_HW_GLOBAL_BOOST
	eas_attr_add("boost",
		&global_boost_enable,
		NULL,
		NULL,
		0640);
#endif

	eas_attr_add("up_migration_util_filter",
		&up_migration_util_filter,
		NULL,
		NULL,
		0640);

#ifndef CONFIG_HW_MULTI_MARGIN
	eas_attr_add("sd_capacity_margin",
		&sd_capacity_margin,
		NULL,
		NULL,
		0640);

	eas_attr_add("capacity_margin",
		&capacity_margin,
		NULL,
		NULL,
		0640);
#else
	eas_attr_add("sd_capacity_margin",
		NULL,
		sd_capacity_margin_show,
		sd_capacity_margin_store,
		0640);

	eas_attr_add("capacity_margin",
		NULL,
		capacity_margin_show,
		capacity_margin_store,
		0640);
#endif

#ifdef CONFIG_HW_BOOT_BOOST
	eas_attr_add("boot_boost",
		&boot_boost,
		NULL,
		NULL,
		0640);
#endif

#ifdef CONFIG_HW_BOOST
	eas_attr_add("task_boost_limit",
		&task_boost_limit,
		NULL,
		NULL,
		0640);
#endif
#ifdef CONFIG_HW_SCHED_PRED_LOAD
	eas_attr_add("predl_enable",
		&predl_enable,
		NULL,
		NULL,
		0640);

	eas_attr_add("predl_jump_load",
		&predl_jump_load,
		NULL,
		NULL,
		0640);

	eas_attr_add("predl_do_predict",
		&predl_do_predict,
		NULL,
		NULL,
		0640);

	eas_attr_add("predl_window_size",
		NULL,
		predl_window_size_show,
		predl_window_size_store,
		0640);
#endif
#ifdef CONFIG_SCHED_WALT
	eas_attr_add("walt_init_task_load_pct",
		&sysctl_sched_walt_init_task_load_pct,
#ifdef CONFIG_HW_SCHED_WALT
		walt_init_task_load_pct_show,
		walt_init_task_load_pct_store,
#else
		NULL,
		NULL,
#endif
		0640);
#endif

	eas_data.attr_group.name = "eas";
	eas_data.attr_group.attrs = eas_data.attributes;
	ret = sysfs_create_group(kernel_kobj,
		&eas_data.attr_group);

	return 0;
}

late_initcall(eas_attr_init);

#include "multi_margin/multi_margin_hw.c"
#include "load_balance/load_balance_hw.c"
#include "favor_small/favor_small_hw.c"
#include "spread/spread_hw.c"
#endif /* CONFIG_HW_EAS_SCHED */
