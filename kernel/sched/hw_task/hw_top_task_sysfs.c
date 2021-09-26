/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: huawei top task feature sysfs interfaces
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

static ssize_t top_task_hist_size_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_hist_size);
}

static ssize_t top_task_hist_size_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Allowed range: [1, RAVG_HIST_SIZE_MAX] */
	if (val < 1 || val > RAVG_HIST_SIZE_MAX)
		return -EINVAL;

	tunables->top_task_hist_size = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_hist_size = val;
		}
	}

	return count;
}

static ssize_t top_task_stats_policy_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_stats_policy);
}

static ssize_t top_task_stats_policy_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->top_task_stats_policy = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_stats_policy = val;
		}
	}

	return count;
}

static ssize_t top_task_stats_empty_window_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_stats_empty_window);
}

static ssize_t top_task_stats_empty_window_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->top_task_stats_empty_window = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_stats_empty_window = val;
		}
	}

	return count;
}

static struct governor_attr top_task_hist_size = __ATTR_RW(top_task_hist_size);
static struct governor_attr top_task_stats_policy = __ATTR_RW(top_task_stats_policy);
static struct governor_attr top_task_stats_empty_window = __ATTR_RW(top_task_stats_empty_window);
