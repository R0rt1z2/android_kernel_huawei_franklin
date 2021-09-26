/*
 * Huawei Mutil Margin Declaration
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
#ifndef __SCHED_EAS_MULTI_MARGIN_H__
#define __SCHED_EAS_MULTI_MARGIN_H__

int find_global_boost_cpu(struct task_struct *p);

extern unsigned int sd_capacity_margin;
extern unsigned int capacity_margin;

#ifdef CONFIG_HW_MULTI_MARGIN

static inline unsigned int hw_capacity_margin(int cpu)
{
	return cpu_rq(cpu)->cluster->capacity_margin;
}

static inline unsigned int hw_sd_capacity_margin(int cpu)
{
	return cpu_rq(cpu)->cluster->sd_capacity_margin;
}

ssize_t capacity_margin_show(struct kobject *kobj,
			     struct kobj_attribute *kattr, char *buf);

ssize_t sd_capacity_margin_show(struct kobject *kobj,
				struct kobj_attribute *kattr, char *buf);

ssize_t capacity_margin_store(struct kobject *kobj, struct kobj_attribute *kattr,
			      const char *buf, size_t count);

ssize_t sd_capacity_margin_store(struct kobject *kobj, struct kobj_attribute *kattr,
				 const char *buf, size_t count);
#else
static inline unsigned int hw_capacity_margin(int cpu)
{
	return capacity_margin;
}
#ifdef CONFIG_HW_EAS_SCHED
static inline unsigned int hw_sd_capacity_margin(int cpu)
{
	return sd_capacity_margin;
}
#else
static inline unsigned int hw_sd_capacity_margin(int cpu)
{
	return capacity_margin;
}
#endif
#endif

#endif
