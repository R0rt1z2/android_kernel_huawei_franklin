/*
 * huawei_arm_pmu.c
 *
 * huawei arm pmu process
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

#define USE_CPUHP_STATE CPUHP_AP_PERF_ARM_STARTING
#define USE_CPUHP_STR "AP_PERF_ARM_STARTING"

DEFINE_PER_CPU(int, pmu_irq) = {0};

static void cpu_pm_pmu_common(void *info)
{
	struct cpu_pm_pmu_args *data	= info;
	struct arm_pmu *armpmu		= data->armpmu;
	unsigned long cmd		= data->cmd;
	int cpu				= data->cpu;
	struct pmu_hw_events *hw_events = this_cpu_ptr(armpmu->hw_events);
	int enabled = bitmap_weight(hw_events->used_mask, armpmu->num_events);

	if (!cpumask_test_cpu(cpu, &armpmu->supported_cpus)) {
		data->ret = NOTIFY_DONE;
		return;
	}

	/*
	 * Always reset the PMU registers on power-up even if
	 * there are no events running.
	 */
	if (cmd == CPU_PM_EXIT && armpmu->reset)
		armpmu->reset(armpmu);

	if (!enabled) {
		data->ret = NOTIFY_OK;
		return;
	}

	data->ret = NOTIFY_OK;

	switch (cmd) {
	case CPU_PM_ENTER:
		armpmu->stop(armpmu);
		cpu_pm_pmu_setup(armpmu, cmd);
		break;
	case CPU_PM_EXIT:
	case CPU_PM_ENTER_FAILED:
		cpu_pm_pmu_setup(armpmu, cmd);
		armpmu->start(armpmu);
		break;
	default:
		data->ret = NOTIFY_DONE;
		break;
	}

	return;
}

static int cpu_pm_pmu_notify(struct notifier_block *b, unsigned long cmd,
			     void *v)
{
	struct cpu_pm_pmu_args data = {
		.armpmu	= container_of(b, struct arm_pmu, cpu_pm_nb),
		.cmd	= cmd,
		.cpu	= smp_processor_id(),
	};

	cpu_pm_pmu_common(&data);
	return data.ret;
}

/*
 * PMU hardware loses all context when a CPU goes offline.
 * When a CPU is hotplugged back in, since some hardware registers are
 * UNKNOWN at reset, the PMU must be explicitly reset to avoid reading
 * junk values out of them.
 */
static int arm_perf_starting_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct arm_pmu *pmu = hlist_entry_safe(node, struct arm_pmu, node);
	int err;

	struct cpu_pm_pmu_args data = {
		.armpmu	= pmu,
		.cpu	= (int)cpu,
	};

	if (!pmu || !cpumask_test_cpu(cpu, &pmu->supported_cpus))
		return 0;

	if (pmu->reset)
		pmu->reset(pmu);

	if (data.armpmu->pmu_state != ARM_PMU_STATE_OFF &&
	    data.armpmu->plat_device) {
		int irq = data.armpmu->percpu_irq;

		/* irq is not percpu */
		if (irq == -1) {
			irq = per_cpu(pmu_irq, cpu);
			if (irq) {
				err = irq_force_affinity(irq, cpumask_of(cpu));
				if (err && num_possible_cpus() > 1)
					pr_warn("Unable to set irq affinity (irq=%d, cpu=%u)\n", irq, cpu);
			}
		} else if (irq > 0 && irq_is_percpu(irq)) {
				enable_percpu_irq(irq, IRQ_TYPE_NONE);
		}
	}

	return 0;
}

static int arm_perf_stopping_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct arm_pmu *pmu = hlist_entry_safe(node, struct arm_pmu, node);

	struct cpu_pm_pmu_args data = {
		.armpmu	= pmu,
		.cpu	= (int)cpu,
	};

	if (!pmu || !cpumask_test_cpu(cpu, &pmu->supported_cpus))
		return 0;

	/* Disarm the PMU IRQ before disappearing. */
	if (data.armpmu->pmu_state == ARM_PMU_STATE_RUNNING &&
	    data.armpmu->plat_device) {
		int irq = data.armpmu->percpu_irq;

		if (irq > 0 && irq_is_percpu(irq))
			disable_percpu_irq(irq);

	}

	return 0;
}
