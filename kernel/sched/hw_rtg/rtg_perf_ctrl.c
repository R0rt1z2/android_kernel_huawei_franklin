/*
 * sched_perf_ctrl.c
 *
 * sched related interfaces defined for perf_ctrl
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
 */
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sched/cputime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/hw/sched_perf_ctrl.h>
#include "include/rtg_perf_ctrl.h"
#include "include/rtg_sched.h"

#ifdef CONFIG_HW_RTG
int perf_ctrl_set_task_rtg(void __user *uarg)
{
	struct rtg_group_task task;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&task, uarg, sizeof(struct rtg_group_task))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_id(task.pid, task.grp_id);
	return ret;
}

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
int perf_ctrl_set_rtg_cpus(void __user *uarg)
{
	struct rtg_cpus cpus;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&cpus, uarg, sizeof(struct rtg_cpus))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_preferred_cluster(cpus.grp_id, cpus.cluster_id);
	return ret;
}

int perf_ctrl_set_rtg_freq(void __user *uarg)
{
	struct rtg_freq freqs;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&freqs, uarg, sizeof(struct rtg_freq))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_freq(freqs.grp_id, freqs.freq);
	return ret;
}
#endif

int perf_ctrl_set_rtg_freq_update_interval(void __user *uarg)
{
	struct rtg_interval interval;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&interval, uarg, sizeof(struct rtg_interval))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_freq_update_interval(interval.grp_id,
						   interval.interval);
	return ret;
}

int perf_ctrl_set_rtg_util_invalid_interval(void __user *uarg)
{
	struct rtg_interval interval;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&interval, uarg, sizeof(struct rtg_interval))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_util_invalid_interval(interval.grp_id,
						    interval.interval);
	return ret;
}

int perf_ctrl_set_rtg_load_mode(void __user *uarg)
{
	struct rtg_load_mode mode;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&mode, uarg, sizeof(struct rtg_load_mode))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_load_mode(&mode);
	return ret;
}

#ifdef CONFIG_HW_ED_TASK
int perf_ctrl_set_rtg_ed_params(void __user *uarg)
{
	struct rtg_ed_params params;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&params, uarg, sizeof(struct rtg_ed_params))) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = sched_set_group_ed_params(&params);
	return ret;
}
#endif

#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
int perf_ctrl_set_task_rtg_min_freq(void __user *uarg)
{
	struct task_config cfg;
	struct task_struct *p = NULL;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&cfg, uarg, sizeof(struct task_config)))
		return -EFAULT;

	rcu_read_lock();

	p = find_task_by_vpid(cfg.pid);
	if (p == NULL) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);
	rcu_read_unlock();

	ret = set_task_rtg_min_freq(p, cfg.value);

	put_task_struct(p);
	return ret;
}
#endif
#endif
