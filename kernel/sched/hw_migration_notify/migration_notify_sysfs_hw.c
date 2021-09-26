/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: migration notify feature sysfs interfaces
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

static ssize_t freq_inc_notify_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->freq_inc_notify);
}

static ssize_t freq_inc_notify_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->freq_inc_notify = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->freq_inc_notify = val;
		}
	}

	return count;
}

static ssize_t freq_dec_notify_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->freq_dec_notify);
}

static ssize_t freq_dec_notify_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->freq_dec_notify = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->freq_dec_notify = val;
		}
	}

	return count;
}

static struct governor_attr freq_inc_notify = __ATTR_RW(freq_inc_notify);
static struct governor_attr freq_dec_notify = __ATTR_RW(freq_dec_notify);
