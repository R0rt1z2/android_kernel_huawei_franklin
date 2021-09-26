/*
 * Huawei Mutil Margin File
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

unsigned int sd_capacity_margin = 1280;

#ifdef CONFIG_HW_MULTI_MARGIN
#include <linux/topology_hw.h>
#include "multi_margin_hw.h"

int find_global_boost_cpu(struct task_struct *p)
{
	int boost_cpu = -1;
	struct sched_cluster *cluster = NULL;

	for_each_sched_cluster_reverse(cluster) {
		boost_cpu = find_boost_cpu(&cluster->cpus, p, cpumask_first(&cluster->cpus));
		if (boost_cpu == -1)
			continue;

		if (idle_cpu(boost_cpu))
			break;

		/* If util of boost_cpu is over 90%, check other cluster.*/
		if ((capacity_of(boost_cpu) * 1024) >= (cpu_util_without(boost_cpu, p) * 1138))
			break;
	}

	return boost_cpu;
}

static unsigned int *get_tokenized_data(const char *buf)
{
	const char *cp = NULL;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data = NULL;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (ntokens != num_clusters)
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf_s(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

ssize_t capacity_margin_show(struct kobject *kobj,
			     struct kobj_attribute *kattr, char *buf)
{
	struct sched_cluster *cluster = NULL;
	ssize_t ret = 0;

	for_each_sched_cluster(cluster) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				cluster->capacity_margin, ":");
	}

	(void)sprintf_s(buf + ret - 1, PAGE_SIZE - ret, "\n");
	return ret;
}

ssize_t sd_capacity_margin_show(struct kobject *kobj,
				struct kobj_attribute *kattr, char *buf)
{
	struct sched_cluster *cluster = NULL;
	ssize_t ret = 0;

	for_each_sched_cluster(cluster) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				cluster->sd_capacity_margin, ":");
	}

	(void)sprintf_s(buf + ret - 1, PAGE_SIZE - ret, "\n");
	return ret;
}

ssize_t capacity_margin_store(struct kobject *kobj, struct kobj_attribute *kattr,
				     const char *buf, size_t count)
{
	char *str = NULL;
	unsigned int *new_margin = NULL;
	struct sched_cluster *cluster = NULL;
	int ret;
	int i;

	str = vmalloc(count + 1);
	if (str == NULL)
		return -ENOMEM;

	ret = memcpy_s(str, count, buf, count);
	if (ret != 0)
		return -EINVAL;

	str[count] = 0;
	new_margin = get_tokenized_data(str);
	vfree(str);

	if (IS_ERR(new_margin))
		return PTR_ERR(new_margin);

	i = 0;
	for_each_sched_cluster(cluster) {
		cluster->capacity_margin = new_margin[i++];
	}

	kfree(new_margin);
	return count;
}

ssize_t sd_capacity_margin_store(struct kobject *kobj, struct kobj_attribute *kattr,
				const char *buf, size_t count)
{
	char *str = NULL;
	unsigned int *new_margin = NULL;
	struct sched_cluster *cluster = NULL;
	int ret;
	int i;

	str = vmalloc(count + 1);
	if (str == NULL)
		return -ENOMEM;

	ret = memcpy_s(str, count, buf, count);
	if (ret != 0)
		return -EINVAL;

	str[count] = 0;
	new_margin = get_tokenized_data(str);
	vfree(str);

	if (IS_ERR(new_margin))
		return PTR_ERR(new_margin);

	i = 0;
	for_each_sched_cluster(cluster) {
		cluster->sd_capacity_margin = new_margin[i++];
	}

	kfree(new_margin);
	return count;
}

#else /* CONFIG_HW_MULTI_MARGIN */

static unsigned long cpu_spare_capacity(int cpu, unsigned long util)
{
	unsigned long spare_capacity;
	spare_capacity = capacity_of(cpu) - util;
	spare_capacity = clamp(spare_capacity, 0UL, capacity_of(cpu));

	return spare_capacity;
}

static inline bool __task_fits(struct task_struct *p, int cpu, int util)
{
	unsigned long capacity = capacity_of(cpu);

	util += boosted_task_util(p);

	return (capacity * 1024) > (util * hw_capacity_margin(cpu));
}

static int
find_spare_boost_cpu(struct cpumask *group_cpus, struct task_struct *p)
{
	int spare_boost_cpu = -1;
	unsigned long max_spare_capacity = 0;
	unsigned long spare_capacity;
	int i;
	int spare_idle_cpu = -1;
	unsigned long max_idle_cap = 0;
	unsigned long wake_util;

	for_each_cpu_and(i, group_cpus, &p->cpus_allowed) {
		/*
		 * If the CPU's utilizaiton is over 60%,
		 * then we don't consider the cpu as spare one.
		 */
		wake_util = cpu_util_without(i, p);
		if (!__task_fits(p, i, cpu_util(i)))
			continue;

		spare_capacity = cpu_spare_capacity(i, wake_util);
		if (idle_cpu(i)) {
			if (spare_idle_cpu != i && spare_capacity > max_idle_cap) {
				spare_idle_cpu = i;
				max_idle_cap = spare_capacity;
			}
		} else {
			if (spare_capacity > max_spare_capacity) {
				max_spare_capacity = spare_capacity;
				spare_boost_cpu = i;
			}
		}
	}

	spare_boost_cpu = (spare_idle_cpu != -1) ? spare_idle_cpu : spare_boost_cpu;

	return spare_boost_cpu;
}

static int select_boost_cpu(struct task_struct *p, int spare_cpu, int boost_cpu)
{
	unsigned long cap_boost_cpu, cap_spare_cpu;

	cap_boost_cpu = cpu_spare_capacity(boost_cpu, cpu_util_without(boost_cpu, p));
	cap_spare_cpu = cpu_spare_capacity(spare_cpu, cpu_util_without(spare_cpu, p));

	/* select the cpu with max spare cap */
	if (cap_boost_cpu < cap_spare_cpu)
		boost_cpu = spare_cpu;

	return boost_cpu;
}

int find_global_boost_cpu(struct task_struct *p)
{
	struct cpumask fast_cpus;
	struct cpumask spare_cpus;
	int boost_cpu = -1;
	int spare_cpu = -1;

	hw_get_fast_cpus(&fast_cpus);

	if (cpumask_empty(&fast_cpus) ||
	    !cpumask_intersects(&p->cpus_allowed, &fast_cpus) ||
	    !cpumask_intersects(&fast_cpus, cpu_online_mask))
		return -1;

	boost_cpu = find_boost_cpu(&fast_cpus, p, cpumask_first(&fast_cpus));
	if (boost_cpu != -1) {
		if (idle_cpu(boost_cpu))
		    return boost_cpu;

		/* Enable spare boost cpu feature */
		/* If util of boost_cpu is over 90%, check if any spare cpu is available.*/
		if ((capacity_of(boost_cpu) * 1024) < (cpu_util_without(boost_cpu, p) * 1138)) {
			cpumask_xor(&spare_cpus, &fast_cpus, cpu_online_mask);
			spare_cpu = find_spare_boost_cpu(&spare_cpus, p);

			/* if spare_cpu available, select max spare one . */
			if (spare_cpu != -1)
				boost_cpu= select_boost_cpu(p, spare_cpu, boost_cpu);

		}
	}

	return boost_cpu;
}

#endif
