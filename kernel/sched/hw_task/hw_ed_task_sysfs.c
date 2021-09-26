/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: early detection tasks feature sysfs interfaces
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

#define DECIMAL_BASE 10

static ssize_t ed_task_running_duration_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->ed_task_running_duration);
}

static ssize_t ed_task_running_duration_store(struct gov_attr_set *attr_set,
					      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, DECIMAL_BASE, &val))
		return -EINVAL;

	tunables->ed_task_running_duration = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->ed_task_running_duration = val;
		}
	}

	return count;
}

static ssize_t ed_task_waiting_duration_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->ed_task_waiting_duration);
}

static ssize_t ed_task_waiting_duration_store(struct gov_attr_set *attr_set,
					      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, DECIMAL_BASE, &val))
		return -EINVAL;

	tunables->ed_task_waiting_duration = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->ed_task_waiting_duration = val;
		}
	}

	return count;
}

static ssize_t ed_new_task_running_duration_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->ed_new_task_running_duration);
}

static ssize_t ed_new_task_running_duration_store(struct gov_attr_set *attr_set,
						  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy = NULL;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, DECIMAL_BASE, &val))
		return -EINVAL;

	tunables->ed_new_task_running_duration = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->ed_new_task_running_duration = val;
		}
	}

	return count;
}

static struct governor_attr ed_task_running_duration = __ATTR_RW(ed_task_running_duration);
static struct governor_attr ed_task_waiting_duration = __ATTR_RW(ed_task_waiting_duration);
static struct governor_attr ed_new_task_running_duration = __ATTR_RW(ed_new_task_running_duration);
