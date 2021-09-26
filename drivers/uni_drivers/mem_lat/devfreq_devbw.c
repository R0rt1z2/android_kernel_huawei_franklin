/*
 * devfreq_devbw.c
 *
 * devbw driver
 *
 * Copyright (c) 2015-2020 Huawei Technologies Co., Ltd.
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

#define pr_fmt(fmt) "devbw: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/devfreq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/cpufreq.h>
#include <linux/memlat_plat.h>
#include "governor_memlat.h"

#define CREATE_TRACE_POINTS
#include <trace/events/mem_lat.h>

enum freq_table_sorting {
	FREQ_TABLE_SORTED_ASCENDING,
	FREQ_TABLE_SORTED_DESCENDING
};

struct dev_data {
	unsigned long cur_dev_freq;
	struct vote_reg hw_vote;
	cpumask_t cpus;
	spinlock_t vote_spinlock; /* spinlock for vote */
	bool suspended; /* if suspended do not set freq and votefreq */
	struct devfreq *df;
	struct devfreq_dev_profile dp;
	enum freq_table_sorting freq_table_sorted;
};

void __weak plat_init_hw_vote(struct vote_reg *hw_vote, const char *name_str) {}
void __weak plat_set_hw_vote(struct vote_reg *hw_vote, unsigned long freq, int idx) {}

#define DEFAULT_POLLING_INTERVAL_MS	50

static void set_freq_table_sorted(struct dev_data *d)
{
	struct devfreq_dev_profile *p = &d->dp;
	int ascending = 0;
	int prev = -1;
	int i;

	for (i = 0; i < p->max_state; i++) {
		if (prev == -1) {
			prev = i;
			continue;
		}

		if (p->freq_table[i] == p->freq_table[prev]) {
			pr_warn("Duplicate freq-table frequency: %u\n",
				p->freq_table[i]);
		}

		/* Frequency increased from prev to pos */
		if (p->freq_table[i] > p->freq_table[prev]) {
			/* But frequency was decreasing earlier */
			if (ascending < 0) {
				pr_debug("Freq table is unsorted\n");
				return;
			}

			ascending++;
		} else {
			/* Frequency decreased from prev to pos */

			/* But frequency was increasing earlier */
			if (ascending > 0) {
				pr_debug("Freq table is unsorted\n");
				return;
			}

			ascending--;
		}

		prev = i;
	}

	if (ascending > 0)
		d->freq_table_sorted = FREQ_TABLE_SORTED_ASCENDING;
	else
		d->freq_table_sorted = FREQ_TABLE_SORTED_DESCENDING;
}

/* Find lowest freq at or above target in a table in ascending order */
static inline int freq_table_find_index_a(struct dev_data *d,
					  unsigned long *freq)
{
	struct devfreq_dev_profile *p = &d->dp;
	int i, best = p->max_state - 1;

	for (i = 0; i < p->max_state; i++) {
		if (p->freq_table[i] >= *freq) {
			best = i;
			break;
		}
		best = i;
	}

	*freq = p->freq_table[best];
	return best;
}

/* Find lowest freq at or above target in a table in descending order */
static inline int freq_table_find_index_d(struct dev_data *d,
					  unsigned long *freq)
{
	int i, best = 0;
	struct devfreq_dev_profile *p = &d->dp;

	for (i = 0; i < p->max_state; i++) {
		if (p->freq_table[i] == *freq)
			return i;

		if (p->freq_table[i] > *freq) {
			best = i;
			continue;
		}

		/* No freq found above target_freq */
		break;
	}

	*freq = p->freq_table[best];
	return best;
}

static int find_freq(struct dev_data *d, unsigned long *freq)
{
	if (d->freq_table_sorted == FREQ_TABLE_SORTED_ASCENDING)
		return freq_table_find_index_a(d, freq);

	return freq_table_find_index_d(d, freq);
}

void set_hw_vote_reg(struct vote_reg *hw_vote, unsigned long freq, int idx)
{
	plat_set_hw_vote(hw_vote, freq, idx);
}

void set_dev_votefreq(struct device *dev, unsigned long new_freq)
{
	struct dev_data *d = dev_get_drvdata(dev);
	unsigned long freq = new_freq;
	int freq_idx;

	spin_lock(&d->vote_spinlock);
	freq_idx = find_freq(d, &freq);

	if (freq == d->cur_dev_freq || d->suspended) {
		spin_unlock(&d->vote_spinlock);
		return;
	}

	d->cur_dev_freq = freq;
	spin_unlock(&d->vote_spinlock);

	set_hw_vote_reg(&d->hw_vote, freq, freq_idx);
}
EXPORT_SYMBOL(set_dev_votefreq);

unsigned long get_dev_votefreq(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);
	unsigned int freq;

	spin_lock(&d->vote_spinlock);
	freq = d->cur_dev_freq;
	spin_unlock(&d->vote_spinlock);

	return freq;
}
EXPORT_SYMBOL(get_dev_votefreq);

static int devbw_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_data *d = dev_get_drvdata(dev);

	set_dev_votefreq(dev, *freq);

	trace_memlat_set_dev_freq(dev_name(dev), "periodic_update",
				  cpumask_any(&d->cpus), 0, 0, *freq);

	return 0;
}

static int devbw_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	return 0;
}

static int get_mask_from_dev_handle(struct platform_device *pdev,
				    cpumask_t *mask)
{
	struct device *dev = &pdev->dev;
	struct device_node *dev_phandle = NULL;
	struct device *cpu_dev = NULL;
	int cpu;
	int i = 0;
	int ret = -ENOENT;

	dev_phandle = of_parse_phandle(dev->of_node, "cpulist", i++);
	while (dev_phandle) {
		for_each_possible_cpu(cpu) {
			cpu_dev = get_cpu_device(cpu);
			if (cpu_dev && cpu_dev->of_node == dev_phandle) {
				cpumask_set_cpu(cpu, mask);
				ret = 0;
				break;
			}
		}
		dev_phandle = of_parse_phandle(dev->of_node,
					       "cpulist", i++);
	}

	return ret;
}

#define PROP_TBL "freq-tbl"

int devfreq_add_devbw(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_data *d = NULL;
	struct devfreq_dev_profile *p = NULL;
	u32 *data = NULL;
	const char *gov_name = NULL;
	int ret, len, i;
	const char *name_str = NULL;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d) {
		dev_err(dev, "fail to alloc mem!\n");
		ret = -ENOMEM;
		goto error;
	}
	dev_set_drvdata(dev, d);

	if (get_mask_from_dev_handle(pdev, &d->cpus)) {
		dev_err(dev, "CPU list is empty\n");
		ret = -ENODEV;
		goto error;
	}

	name_str = of_device_get_match_data(dev);
	if (name_str != NULL)
		dev_set_name(dev, "%s%d", name_str, cpumask_first(&d->cpus));
	else
		dev_err(dev, "name_str cannot found!\n");

	spin_lock_init(&d->vote_spinlock);
	d->suspended = false;

	plat_init_hw_vote(&d->hw_vote, name_str);

	p = &d->dp;
	p->polling_ms = DEFAULT_POLLING_INTERVAL_MS;
	p->target = devbw_target;
	p->get_dev_status = devbw_get_dev_status;

	if (of_find_property(dev->of_node, PROP_TBL, &len) != NULL) {
		len /= sizeof(*data);
		data = devm_kzalloc(dev, len * sizeof(*data), GFP_KERNEL);
		if (data == NULL) {
			dev_err(dev, "fail to alloc mem for freq_table\n");
			ret = -ENOMEM;
			goto error;
		}

		p->freq_table = devm_kzalloc(dev,
					     len * sizeof(*p->freq_table),
					     GFP_KERNEL);
		if (p->freq_table == NULL) {
			dev_err(dev, "fail to alloc mem for freq_table\n");
			ret = -ENOMEM;
			goto error;
		}

		ret = of_property_read_u32_array(dev->of_node, PROP_TBL,
						 data, len);
		if (ret != 0) {
			dev_err(dev, "fail to alloc mem for freq_table\n");
			goto error;
		}

		for (i = 0; i < len; i++)
			p->freq_table[i] = data[i] * HZ_PER_MHZ;

		p->max_state = len;
		set_freq_table_sorted(d);
	}

	if (of_property_read_string(dev->of_node, "governor", &gov_name) != 0)
		gov_name = "mem_latency";

	d->df = devfreq_add_device(dev, p, gov_name, NULL);
	if (IS_ERR(d->df)) {
		dev_err(dev, "fail to register devfreq memlatency\n");
		ret = PTR_ERR(d->df);
		goto error;
	}

	return 0;

error:
	return ret;
}

int devfreq_remove_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);

	set_dev_votefreq(dev, 0);
	devfreq_remove_device(d->df);

	return 0;
}

int devfreq_suspend_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);

	set_dev_votefreq(dev, 0);

	spin_lock(&d->vote_spinlock);
	d->suspended = true;
	spin_unlock(&d->vote_spinlock);

	return 0;
}

int devfreq_resume_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);

	spin_lock(&d->vote_spinlock);
	d->suspended = false;
	spin_unlock(&d->vote_spinlock);

	return 0;
}

static SIMPLE_DEV_PM_OPS(devfreq_devbw_pm, devfreq_suspend_devbw,
			 devfreq_resume_devbw);

static int devfreq_devbw_probe(struct platform_device *pdev)
{
	return devfreq_add_devbw(pdev);
}

static int devfreq_devbw_remove(struct platform_device *pdev)
{
	return devfreq_remove_devbw(&pdev->dev);
}

static const struct of_device_id match_table[] = {
	{ .compatible = "huawei,ddr-devbw", .data = "memlat_cpu" },
	{ .compatible = "huawei,l3-devbw", .data = "l3_memlat_cpu"},
	{}
};

static struct platform_driver devbw_driver = {
	.probe = devfreq_devbw_probe,
	.remove = devfreq_devbw_remove,
	.driver = {
		.name = "devbw",
		.pm = &devfreq_devbw_pm,
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init devbw_init(void)
{
	platform_driver_register(&devbw_driver);
	return 0;
}
device_initcall(devbw_init);

MODULE_DESCRIPTION("Device DDR Frequency hw-voting driver");
MODULE_LICENSE("GPL v2");
