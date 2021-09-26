/*
 * rtg_sched.h
 *
 * rtg sched header
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#ifndef HW_RTG_COMMON_H
#define HW_RTG_COMMON_H

#ifdef CONFIG_HW_RTG_CPU_TIME
struct group_cpu_time {
	u64 window_start;
	u64 curr_runnable_sum;
	u64 prev_runnable_sum;
};
#endif

struct group_time {
	unsigned long curr_window_load;
	unsigned long curr_window_exec;
	unsigned long prev_window_load;
	unsigned long prev_window_exec;
	unsigned long normalized_util;
};

#define MULTI_FRAME_NUM 5
enum RTG_GRP_ID {
	DEFAULT_RTG_GRP_ID,
	DEFAULT_CGROUP_COLOC_ID = 1,
	DEFAULT_AI_ID = 2,
	DEFAULT_AI_RENDER_THREAD_ID = 3,
	DEFAULT_AI_OTHER_THREAD_ID = 4,
	DEFAULT_RT_FRAME_ID = 8,
	DEFAULT_AUX_ID = 9,
	DEFAULT_GAME_ID = 10,
	MULTI_FRAME_ID = 11,
	MAX_NUM_CGROUP_COLOC_ID = MULTI_FRAME_ID + MULTI_FRAME_NUM,
};

struct grp_load_mode {
	bool freq_enabled;
	bool util_enabled;
};

struct rtg_class;

struct related_thread_group {
	int id;
	raw_spinlock_t lock;
	struct list_head tasks;
	struct list_head list;
#ifdef CONFIG_HW_RTG_CPU_TIME
	struct group_cpu_time * __percpu cpu_time;
#endif
#ifdef CONFIG_HW_RTG_NORMALIZED_UTIL
	struct sched_cluster *preferred_cluster;
	struct group_time time_pref_cluster;
#endif
	struct group_time time;
	unsigned long freq_update_interval; /* in nanoseconds */
	unsigned long util_invalid_interval; /* in nanoseconds */
	unsigned long util_update_timeout; /* in nanoseconds */
	u64 last_freq_update_time;
	u64 last_util_update_time;
	u64 window_start;
	u64 mark_start;
	u64 prev_window_time;
	/* rtg window information for WALT */
	unsigned int window_size;
	unsigned int nr_running;
	/* the min freq set by userspace */
	unsigned int us_set_min_freq;
	struct grp_load_mode mode;
#ifdef CONFIG_SCHED_TUNE
	int max_boost;
	unsigned int capacity_margin;
#endif
#ifdef CONFIG_HW_ED_TASK
	bool ed_enabled;
	unsigned int ed_task_running_duration;
	unsigned int ed_task_waiting_duration;
	unsigned int ed_new_task_running_duration;
#endif /* CONFIG_HW_ED_TASK */
	void *private_data;
	const struct rtg_class *rtg_class;
#ifdef CONFIG_HW_CGROUP_RTG
	u64 start_ts;
	u64 last_update;
	u64 downmigrate_ts;
	bool skip_min;
#endif
};

struct rtg_tick_info {
	struct task_struct *curr;
	u32 old_load;
};

struct rtg_class {
	void (*sched_update_rtg_tick)(struct related_thread_group *grp,
				      struct rtg_tick_info *tick_info);
};

enum rtg_freq_update_flags {
	FRAME_FORCE_UPDATE = (1 << 0),
	FRAME_NORMAL_UPDATE = (1 << 1),
	AI_FORCE_UPDATE = (1 << 2),
};

#endif
