/*
 * Huawei sched cluster declaration
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

#ifndef SCHED_CLUSTER_HW_H
#define SCHED_CLUSTER_HW_H

#ifdef CONFIG_HW_SCHED_CLUSTER

#include <linux/types.h>
#include <linux/cpumask.h>

struct sched_cluster {
	struct list_head list;
	struct cpumask cpus;
	int id;
	int max_possible_capacity;
	unsigned int cur_freq, max_freq, min_freq;
	bool freq_init_done;
	unsigned int capacity_margin;
	unsigned int sd_capacity_margin;
};

struct rq;

extern struct list_head cluster_head;
extern int num_clusters;
extern struct sched_cluster init_cluster;

extern void update_cluster_topology(void);
extern void init_clusters(void);
extern void kick_load_balance(struct rq *rq);

/* Iterate in increasing order of cluster max possible capacity */
#define for_each_sched_cluster(cluster) \
	list_for_each_entry(cluster, &cluster_head, list)

#define for_each_sched_cluster_reverse(cluster) \
	list_for_each_entry_reverse(cluster, &cluster_head, list)

#define min_cap_cluster()	\
	list_first_entry(&cluster_head, struct sched_cluster, list)
#define max_cap_cluster()	\
	list_last_entry(&cluster_head, struct sched_cluster, list)

unsigned int freq_to_util(unsigned int cpu, unsigned int freq);

/*
 * Note that util_to_freq(i, freq_to_util(i, *freq*)) is lower than *freq*.
 * That's ok since we use CPUFREQ_RELATION_L in __cpufreq_driver_target().
 */
unsigned int util_to_freq(unsigned int cpu, unsigned int util);

#else /* CONFIG_HW_SCHED_CLUSTER */

static inline void update_cluster_topology(void) {}
static inline void init_clusters(void) {}

#endif /* CONFIG_HW_SCHED_CLUSTER */

#endif
