/*
 * drg.h
 *
 * header file of drg module
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#ifndef _DRG_H
#define _DRG_H

#include <linux/cpufreq.h>
#include <linux/devfreq.h>

enum drg_dev_type {
	DRG_NONE_DEV = 0,
	/* CPU TYPE if new cpu add after(eg. small strong ...) */
	DRG_CPU_CLUSTER0 = (1 << 0) << 8,
	DRG_CPU_CLUSTER1,
	DRG_CPU_CLUSTER2,
	/* CACHE TYPE if new cache add after(eg. l4 l5 l6 ...)*/
	DRG_L3_CACHE = (1 << 1) << 8,
	/* If new type add after here */
};

struct drg_dev_freq {
	/* enum drg_dev_type */
	unsigned int type;
	/* described as Hz  */
	unsigned long max_freq;
	unsigned long min_freq;
};

#define DEV_TYPE_CPU_CLUSTER (1 << 0) << 8

#define MAX_DRG_MARGIN		1024U

#ifdef CONFIG_DRG
int perf_ctrl_get_drg_dev_freq(void __user *uarg);
#else
static inline int perf_ctrl_get_drg_dev_freq(void __user *uarg)
{
	return -EFAULT;
}
#endif

#endif /* _DRG_H */
