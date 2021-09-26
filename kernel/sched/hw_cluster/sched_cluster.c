/*
 * Huawei Sched Cluster File
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
#include "sched_cluster.h"
#include "../kernel/sched/sched.h"
#include <linux/hw/eas_hw.h>

struct list_head cluster_head;
int num_clusters;

struct sched_cluster init_cluster = {
	.list			=	LIST_HEAD_INIT(init_cluster.list),
	.id			=	0,
	.max_possible_capacity	=	1024,
	.cur_freq		=	1,
	.max_freq		=	1,
	.min_freq		=	1,
	.capacity_margin	=	1280,
	.sd_capacity_margin	=	1280,
};

static void assign_cluster_ids(struct list_head *head)
{
	struct sched_cluster *cluster = NULL;
	int pos = 0;

	list_for_each_entry(cluster, head, list) {
		cluster->id = pos++;
	}
}

static void
move_list(struct list_head *dst, struct list_head *src, bool sync_rcu)
{
	struct list_head *first = NULL, *last = NULL;

	first = src->next;
	last = src->prev;

	if (sync_rcu) {
		INIT_LIST_HEAD_RCU(src);
		synchronize_rcu();
	}

	first->prev = dst;
	dst->prev = last;
	last->next = dst;

	/* Ensure list sanity before making the head visible to all CPUs. */
	smp_mb();
	dst->next = first;
}

static void
insert_cluster(struct sched_cluster *cluster, struct list_head *head)
{
	struct sched_cluster *tmp = NULL;
	struct list_head *iter = head;

	list_for_each_entry(tmp, head, list) {
		/*
		 * Because sched_init is called before cpufreq_init, we
		 * got wrong max_possible_capacity here in kernel 4.14. We
		 * should sort clusters later in a cpufreq notify.
		 * Temporarily use cpu id here.
		 */
		if (cpumask_first(&cluster->cpus) < cpumask_first(&tmp->cpus))
			break;
		iter = &tmp->list;
	}

	list_add(&cluster->list, iter);
}

static struct sched_cluster *alloc_new_cluster(const struct cpumask *cpus)
{
	struct sched_cluster *cluster = NULL;

	cluster = kzalloc(sizeof(struct sched_cluster), GFP_ATOMIC);
	if (!cluster) {
		pr_warn("Cluster allocation failed. Possible bad scheduling\n");
		return NULL;
	}

	INIT_LIST_HEAD(&cluster->list);
	cluster->max_possible_capacity	=	arch_scale_cpu_capacity(NULL, cpumask_first(cpus));
	cluster->cur_freq		=	1;
	cluster->max_freq		=	1;
	cluster->min_freq		=	1;
	cluster->freq_init_done		=	false;
	cluster->capacity_margin	=	capacity_margin;
	cluster->sd_capacity_margin	=	1280;

	cluster->cpus = *cpus;

	return cluster;
}

static void add_cluster(const struct cpumask *cpus, struct list_head *head)
{
	struct sched_cluster *cluster = alloc_new_cluster(cpus);
	int i;

	if (!cluster)
		return;

	for_each_cpu(i, cpus)
		cpu_rq(i)->cluster = cluster;

	insert_cluster(cluster, head);
	num_clusters++;
	trace_printk("[SCHED_CLUSTER] %d %*pbl %*pbl\n",
		num_clusters, cpumask_pr_args(cpus),
		cpumask_pr_args(&cluster->cpus));
}

void init_clusters(void)
{
	init_cluster.cpus = *cpu_possible_mask;
	INIT_LIST_HEAD(&cluster_head);
}

void update_cluster_topology(void)
{
	struct cpumask cpus = *cpu_possible_mask;
	const struct cpumask *cluster_cpus = NULL;
	struct list_head new_head;
	int i;

	INIT_LIST_HEAD(&new_head);

	for_each_cpu(i, &cpus) {
		cluster_cpus = cpu_coregroup_mask(i);
		cpumask_andnot(&cpus, &cpus, cluster_cpus);
		add_cluster(cluster_cpus, &new_head);
	}

	assign_cluster_ids(&new_head);

	/*
	 * Ensure cluster ids are visible to all CPUs before making
	 * cluster_head visible.
	 */
	move_list(&cluster_head, &new_head, false);
}

unsigned int freq_to_util(unsigned int cpu, unsigned int freq)
{
	unsigned int util = 0U;

	util = arch_scale_cpu_capacity(NULL, cpu) *
		(unsigned long)freq / cpu_rq(cpu)->cluster->max_freq;
	util = clamp(util, 0U, SCHED_CAPACITY_SCALE);

	return util;
}

/*
 * Note that util_to_freq(i, freq_to_util(i, *freq*)) is lower than *freq*.
 * That's ok since we use CPUFREQ_RELATION_L in __cpufreq_driver_target().
 */
unsigned int util_to_freq(unsigned int cpu, unsigned int util)
{
	unsigned int max_freq = arch_get_cpu_max_freq(cpu);
	unsigned int freq = 0U;

	freq = cpu_rq(cpu)->cluster->max_freq *
		(unsigned long)util / arch_scale_cpu_capacity(NULL, cpu);
	freq = clamp(freq, 0U, max_freq);

	return freq;
}

#ifdef CONFIG_HUAWEI_SCHED_VIP
void kick_load_balance(struct rq *rq)
{
	struct sched_cluster *cluster = NULL;
	struct sched_cluster *kick_cluster = NULL;
	int kick_cpu;

	/* Find the prev cluster. */
	for_each_sched_cluster(cluster) {
		if (cluster == rq->cluster)
			break;
		kick_cluster = cluster;
	}

	if (!kick_cluster)
		return;

	/* Find first unisolated idle cpu in the cluster. */
	for_each_cpu(kick_cpu, &kick_cluster->cpus) {
		if (!cpu_online(kick_cpu) || cpu_isolated(kick_cpu))
			continue;
		if (!idle_cpu(kick_cpu))
			continue;
		break;
	}

	if (kick_cpu >= nr_cpu_ids)
		return;

	reset_balance_interval(kick_cpu);
	vip_balance_set_overutilized(cpu_of(rq));
	if (test_and_set_bit(NOHZ_BALANCE_KICK, nohz_flags(kick_cpu)))
		return;
	smp_send_reschedule(kick_cpu);
}
#else
void kick_load_balance(struct rq *rq) { }
#endif
