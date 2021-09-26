/*
 * huawei_core.h
 *
 * huawei perf event core
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

#ifndef __HUAWEI_CORE_H
#define __HUAWEI_CORE_H

/* The shared events struct. */
#define SHARED_EVENTS_MAX 7

struct shared_events_str {
	/*
	 * Mutex to serialize access to shared list. Needed for the
	 * read/modify/write sequences.
	 */
	struct mutex		list_mutex;

	/*
	 * A 1 bit for an index indicates that the slot is being used for
	 * an event. A 0 means that the slot can be used.
	 */
	DECLARE_BITMAP(used_mask, SHARED_EVENTS_MAX);

	/*
	 * The kernel events that are shared for a cpu;
	 */
	struct perf_event	*events[SHARED_EVENTS_MAX];
	struct perf_event_attr	attr[SHARED_EVENTS_MAX];
	atomic_t		refcount[SHARED_EVENTS_MAX];
};

static struct shared_events_str __percpu *shared_events;

static DEFINE_PER_CPU(bool, is_idle);
static DEFINE_PER_CPU(bool, is_hotplugging);

struct notifier_block perf_event_idle_nb;

#if defined CONFIG_HOTPLUG_CPU || defined CONFIG_KEXEC_CORE
static LIST_HEAD(dormant_event_list);
static DEFINE_SPINLOCK(dormant_event_list_lock);

static void perf_prepare_install_in_context(struct perf_event *event);
static void perf_deferred_install_in_context(int cpu);
#endif

static int
perf_event_delete_kernel_shared(struct perf_event *event);
static int __perf_event_release_kernel(struct perf_event *event);
int perf_event_release_kernel(struct perf_event *event);
static struct perf_event *
perf_event_create_kernel_shared_check(struct perf_event_attr *attr, int cpu,
		struct task_struct *task,
		perf_overflow_handler_t overflow_handler,
		struct perf_event *group_leader);
static void
perf_event_create_kernel_shared_add(struct perf_event_attr *attr, int cpu,
				 struct task_struct *task,
				 perf_overflow_handler_t overflow_handler,
				 void *context,
				 struct perf_event *event);
#ifdef CONFIG_PERF_USER_SHARE
static void perf_group_shared_event(struct perf_event *event,
		struct perf_event *group_leader);
#endif

int perf_event_restart_events(unsigned int cpu);
static int event_idle_notif(struct notifier_block *nb, unsigned long action,
			    void *data);
#endif
