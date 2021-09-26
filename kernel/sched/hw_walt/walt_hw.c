/*
 * walt_hw.c
 *
 * common platform WALT source file
 *
 * Copyright (c) 2021 Huawei Technologies Co., Ltd
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

#ifdef CONFIG_HW_SCHED_WALT

#include <linux/kernel.h>
#include <linux/kobject.h>

#ifdef CONFIG_HW_SCHED_CHECK_IRQLOAD
int walt_cpu_overload_irqload(int cpu)
{
	return walt_irqload(cpu) >= sysctl_sched_walt_cpu_overload_irqload;
}
#else
int walt_cpu_overload_irqload(int cpu)
{
	return 0;
}
#endif

int sysctl_sched_walt_init_task_load_pct_sysctl_handler(struct ctl_table *table,
	int write, void __user *buffer, size_t *length, loff_t *ppos)
{
	int rc;

	rc = proc_dointvec(table, write, buffer, length, ppos);
	if (rc)
		return rc;

#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
	sysctl_sched_init_task_load_pct = sysctl_sched_walt_init_task_load_pct;
#endif

	return 0;
}


ssize_t walt_init_task_load_pct_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
	sysctl_sched_walt_init_task_load_pct = sysctl_sched_init_task_load_pct;
#endif
	return (ssize_t)sprintf(buf, "%u\n", sysctl_sched_walt_init_task_load_pct);
}

ssize_t walt_init_task_load_pct_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	sysctl_sched_walt_init_task_load_pct = (unsigned int)val;
#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
	sysctl_sched_init_task_load_pct = sysctl_sched_walt_init_task_load_pct;
#endif
	return count;
}

#endif
