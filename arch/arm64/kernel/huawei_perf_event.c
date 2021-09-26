/*
 * huawei_perf_event.c
 *
 * huawei arm perf event
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

static const unsigned armv8_a55_perf_cache_map[PERF_COUNT_HW_CACHE_MAX]
					      [PERF_COUNT_HW_CACHE_OP_MAX]
					      [PERF_COUNT_HW_CACHE_RESULT_MAX] = {
	PERF_CACHE_MAP_ALL_UNSUPPORTED,

	[C(L1D)][C(OP_READ)][C(RESULT_ACCESS)]   = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_RD,
	[C(L1D)][C(OP_READ)][C(RESULT_MISS)]     = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_REFILL_RD,
	[C(L1D)][C(OP_WRITE)][C(RESULT_ACCESS)]  = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_WR,
	[C(L1D)][C(OP_WRITE)][C(RESULT_MISS)]    = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_REFILL_WR,
	[C(L1D)][C(OP_PREFETCH)][C(RESULT_MISS)] = ARMV8_A55_L1D_CACHE_REFILL_PREFETCH,

	[C(LL)][C(OP_READ)][C(RESULT_ACCESS)]    = ARMV8_PMUV3_PERFCTR_LL_CACHE_RD,
	[C(LL)][C(OP_READ)][C(RESULT_MISS)]      = ARMV8_PMUV3_PERFCTR_LL_CACHE_MISS_RD,

	[C(NODE)][C(OP_READ)][C(RESULT_ACCESS)]  = ARMV8_IMPDEF_PERFCTR_BUS_ACCESS_RD,
	[C(NODE)][C(OP_WRITE)][C(RESULT_ACCESS)] = ARMV8_IMPDEF_PERFCTR_BUS_ACCESS_WR,
};

static const unsigned armv8_a77_perf_cache_map[PERF_COUNT_HW_CACHE_MAX]
					      [PERF_COUNT_HW_CACHE_OP_MAX]
					      [PERF_COUNT_HW_CACHE_RESULT_MAX] = {
	PERF_CACHE_MAP_ALL_UNSUPPORTED,

	[C(L1D)][C(OP_READ)][C(RESULT_ACCESS)]   = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_RD,
	[C(L1D)][C(OP_READ)][C(RESULT_MISS)]     = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_REFILL_RD,
	[C(L1D)][C(OP_WRITE)][C(RESULT_ACCESS)]  = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_WR,
	[C(L1D)][C(OP_WRITE)][C(RESULT_MISS)]    = ARMV8_IMPDEF_PERFCTR_L1D_CACHE_REFILL_WR,

	[C(LL)][C(OP_READ)][C(RESULT_ACCESS)]    = ARMV8_PMUV3_PERFCTR_LL_CACHE_RD,
	[C(LL)][C(OP_READ)][C(RESULT_MISS)]      = ARMV8_PMUV3_PERFCTR_LL_CACHE_MISS_RD,

	[C(DTLB)][C(OP_READ)][C(RESULT_ACCESS)]  = ARMV8_IMPDEF_PERFCTR_L1D_TLB_RD,
	[C(DTLB)][C(OP_READ)][C(RESULT_MISS)]    = ARMV8_IMPDEF_PERFCTR_L1D_TLB_REFILL_RD,
	[C(DTLB)][C(OP_WRITE)][C(RESULT_ACCESS)] = ARMV8_IMPDEF_PERFCTR_L1D_TLB_WR,
	[C(DTLB)][C(OP_WRITE)][C(RESULT_MISS)]   = ARMV8_IMPDEF_PERFCTR_L1D_TLB_REFILL_WR,

	[C(NODE)][C(OP_READ)][C(RESULT_ACCESS)]  = ARMV8_IMPDEF_PERFCTR_BUS_ACCESS_RD,
	[C(NODE)][C(OP_WRITE)][C(RESULT_ACCESS)] = ARMV8_IMPDEF_PERFCTR_BUS_ACCESS_WR,
};

static inline u32 armv8pmu_get_enabled_ints(void)
{
	u32 int_enset;

	int_enset = read_sysreg(pmintenset_el1);
	write_sysreg(0xffffffff, pmintenclr_el1);
	isb();
	return int_enset;
}

static inline u32 armv8pmu_update_enabled_ints(u32 value, int idx, int set)
{
	if (set)
		value |=  BIT(ARMV8_IDX_TO_COUNTER(idx));
	else
		value &= ~(BIT(ARMV8_IDX_TO_COUNTER(idx)));

	return value;
}

static inline void armv8pmu_set_enabled_ints(u32 mask)
{
	write_sysreg(mask, pmintenset_el1);
	isb();
}

static int armv8_a55_map_event(struct perf_event *event)
{
	return __armv8_pmuv3_map_event(event, NULL, &armv8_a55_perf_cache_map);
}

static int armv8_a77_map_event(struct perf_event *event)
{
	return __armv8_pmuv3_map_event(event, NULL, &armv8_a77_perf_cache_map);
}

static void armv8pmu_idle_update(struct arm_pmu *cpu_pmu)
{
	struct pmu_hw_events *hw_events = NULL;
	struct perf_event *event = NULL;
	int idx;

	if (!cpu_pmu)
		return;

	if (__this_cpu_read(is_hotplugging))
		return;

	hw_events = this_cpu_ptr(cpu_pmu->hw_events);

	if (!hw_events)
		return;

	for (idx = 0; idx < cpu_pmu->num_events; ++idx) {

		if (!test_bit(idx, hw_events->used_mask))
			continue;

		event = hw_events->events[idx];

		if (!event || !event->attr.exclude_idle ||
				event->state != PERF_EVENT_STATE_ACTIVE)
			continue;

		cpu_pmu->pmu.read(event);
	}
}

static int perf_cpu_idle_notifier(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct arm_pmu_and_idle_nb *pmu_nb = container_of(nb,
				struct arm_pmu_and_idle_nb, perf_cpu_idle_nb);

	if (action == IDLE_START)
		armv8pmu_idle_update(pmu_nb->cpu_pmu);

	return NOTIFY_OK;
}

static int armv8_a55_pmu_init(struct arm_pmu *cpu_pmu)
{
	int ret = armv8_pmu_init(cpu_pmu);
	if (ret)
		return ret;

	cpu_pmu->name			= "armv8_cortex_a55";
	cpu_pmu->map_event		= armv8_a55_map_event;
	cpu_pmu->attr_groups[ARMPMU_ATTR_GROUP_EVENTS] =
		&armv8_pmuv3_events_attr_group;
	cpu_pmu->attr_groups[ARMPMU_ATTR_GROUP_FORMATS] =
		&armv8_pmuv3_format_attr_group;

	return 0;
}

static int armv8_a77_pmu_init(struct arm_pmu *cpu_pmu)
{
	int ret = armv8_pmu_init(cpu_pmu);
	if (ret)
		return ret;

	cpu_pmu->name			= "armv8_cortex_a77";
	cpu_pmu->map_event		= armv8_a77_map_event;
	cpu_pmu->attr_groups[ARMPMU_ATTR_GROUP_EVENTS] =
		&armv8_pmuv3_events_attr_group;
	cpu_pmu->attr_groups[ARMPMU_ATTR_GROUP_FORMATS] =
		&armv8_pmuv3_format_attr_group;

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int perf_event_hotplug_coming_up(unsigned int cpu)
{
	per_cpu(is_hotplugging, cpu) = false;
	return 0;
}

static int perf_event_hotplug_going_down(unsigned int cpu)
{
	per_cpu(is_hotplugging, cpu) = true;
	return 0;
}

static int perf_event_cpu_hp_init(void)
{
	int ret;

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_NOTIFY_PERF_ONLINE,
				"PERF_EVENT/CPUHP_AP_NOTIFY_PERF_ONLINE",
				perf_event_hotplug_coming_up,
				perf_event_hotplug_going_down);
	if (ret)
		pr_err("CPU hotplug notifier for perf_event.c could not be registered: %d\n",
		       ret);

	return ret;
}
#endif

