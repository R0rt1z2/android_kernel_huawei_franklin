/*
 * huawei_perf_event.h
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

#ifndef __HUAWEI_PERF_EVENT_H
#define __HUAWEI_PERF_EVENT_H

static DEFINE_PER_CPU(bool, is_hotplugging);

struct arm_pmu_and_idle_nb {
	struct arm_pmu *cpu_pmu;
	struct notifier_block perf_cpu_idle_nb;
};

#define ARMV8_PMUV3_PERFCTR_REMOTE_ACCESS			0x31
#define ARMV8_PMUV3_PERFCTR_LL_CACHE				0x32
#define ARMV8_PMUV3_PERFCTR_LL_CACHE_MISS			0x33
#define ARMV8_PMUV3_PERFCTR_DTLB_WALK				0x34
#define ARMV8_PMUV3_PERFCTR_ITLB_WALK				0x35
#define ARMV8_PMUV3_PERFCTR_LL_CACHE_RD				0x36
#define ARMV8_PMUV3_PERFCTR_LL_CACHE_MISS_RD			0x37
#define ARMV8_PMUV3_PERFCTR_REMOTE_ACCESS_RD			0x38

/* ARMv8 Cortex-A55 specific event types. */
#define ARMV8_A55_L1D_CACHE_REFILL_PREFETCH			0xC2

static inline u32 armv8pmu_get_enabled_ints(void);
static inline u32 armv8pmu_update_enabled_ints(u32 value, int idx, int set);
static inline void armv8pmu_set_enabled_ints(u32 mask);
static int armv8_a55_map_event(struct perf_event *event);
static int armv8_a77_map_event(struct perf_event *event);
static void armv8pmu_idle_update(struct arm_pmu *cpu_pmu);
static int perf_cpu_idle_notifier(struct notifier_block *nb,
				unsigned long action, void *data);
static int armv8_a55_pmu_init(struct arm_pmu *cpu_pmu);
static int armv8_a77_pmu_init(struct arm_pmu *cpu_pmu);
#ifdef CONFIG_HOTPLUG_CPU
static int perf_event_hotplug_coming_up(unsigned int cpu);
static int perf_event_hotplug_going_down(unsigned int cpu);
static int perf_event_cpu_hp_init(void);

#else
static inline int perf_event_cpu_hp_init(void) { return 0; }
#endif

#endif
