/*
 * Huawei CPU Topology
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
#ifdef CONFIG_ARCH_HW

#include <linux/errno.h>
#include <linux/of.h>
#include <linux/hw/topology_hw.h>
#include <asm/smp_plat.h>

static const char * const little_cores[] = {
	"arm,cortex-a53",
	"arm,cortex-a55",
	NULL,
};

static bool is_little_cpu(struct device_node *cn)
{
	const char * const *lc;
	for (lc = little_cores; *lc; lc++)
		if (of_device_is_compatible(cn, *lc))
			return true;
	return false;
}

struct cpumask slow_cpu_mask;
struct cpumask fast_cpu_mask;

#ifndef CONFIG_ARCH_HW_MPIDR_SHIFT
#define CONFIG_ARCH_HW_MPIDR_SHIFT 0
#endif

#define MPIDR_SHIFT (CONFIG_ARCH_HW_MPIDR_SHIFT)

void __init arch_get_fast_and_slow_cpus(struct cpumask *fast,
					struct cpumask *slow)
{
	struct device_node *cn = NULL;
	int cpu;

	cpumask_clear(&fast_cpu_mask);
	cpumask_clear(&slow_cpu_mask);

	/*
	 * Else, parse device tree for little cores.
	 */
	while ((cn = of_find_node_by_type(cn, "cpu"))) {
		const u32 *mpidr;
		int len;

		mpidr = of_get_property(cn, "reg", &len);
		if (!mpidr || !(len == 4 || len == 8)) {
			pr_err("%s missing reg property\n", cn->full_name);
			continue;
		}

		cpu = get_logical_index(be32_to_cpup(mpidr + MPIDR_SHIFT));
		if (cpu == -EINVAL) {
			pr_err("couldn't get logical index for mpidr %x\n",
							be32_to_cpup(mpidr + MPIDR_SHIFT));
			break;
		}

		if (is_little_cpu(cn))
			cpumask_set_cpu(cpu, &slow_cpu_mask);
		else
			cpumask_set_cpu(cpu, &fast_cpu_mask);
	}

	if (!cpumask_empty(&fast_cpu_mask) && !cpumask_empty(&slow_cpu_mask))
		return;

	/*
	 * We didn't find both big and little cores so let's call all cores
	 * fast as this will keep the system running, with all cores being
	 * treated equal.
	 */
	cpumask_setall(&fast_cpu_mask);
	cpumask_clear(&slow_cpu_mask);
}

void hw_get_fast_cpus(struct cpumask *cpumask)
{
	cpumask_copy(cpumask, &fast_cpu_mask);
}
EXPORT_SYMBOL(hw_get_fast_cpus);

void hw_get_slow_cpus(struct cpumask *cpumask)
{
	cpumask_copy(cpumask, &slow_cpu_mask);
}
EXPORT_SYMBOL(hw_get_slow_cpus);

int hw_test_fast_cpu(int cpu)
{
	return cpumask_test_cpu(cpu, &fast_cpu_mask);
}
EXPORT_SYMBOL(hw_test_fast_cpu);

int hw_test_slow_cpu(int cpu)
{
	return cpumask_test_cpu(cpu, &slow_cpu_mask);
}
EXPORT_SYMBOL(hw_test_slow_cpu);
#endif //CONFIG_ARCH_HW
