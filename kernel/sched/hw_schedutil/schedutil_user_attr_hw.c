/*
 * schedutil_user_attr_hw.c
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

static struct governor_user_attr schedutil_user_attrs[] = {
#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
	{.name = "overload_duration", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "target_loads", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "above_hispeed_delay", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "hispeed_freq", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "go_hispeed_load", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "min_sample_time", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "timer_slack", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "boostpulse", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0220},
	{.name = "boostpulse_duration", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "fast_ramp_down", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "fast_ramp_up", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "freq_reporting_policy", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "iowait_boost_step", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "up_rate_limit_us", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},
	{.name = "down_rate_limit_us", .uid = SYSTEM_UID, .gid = SYSTEM_GID,
		.mode = 0660},

#endif

#ifdef CONFIG_HW_ED_TASK
	{.name = "ed_task_running_duration", .uid = SYSTEM_UID,
	 .gid = SYSTEM_GID, .mode = 0660},
	{.name = "ed_task_waiting_duration", .uid = SYSTEM_UID,
	 .gid = SYSTEM_GID, .mode = 0660},
	{.name = "ed_new_task_running_duration", .uid = SYSTEM_UID,
	 .gid = SYSTEM_GID, .mode = 0660},
#endif

#ifdef CONFIG_HW_TOP_TASK
	{.name = "top_task_hist_size", .uid = SYSTEM_UID,
	.gid = SYSTEM_GID, .mode = 0660},
	{.name = "top_task_stats_policy", .uid = SYSTEM_UID,
	 .gid = SYSTEM_GID, .mode = 0660},
	{.name = "top_task_stats_empty_window", .uid = SYSTEM_UID,
	 .gid = SYSTEM_GID, .mode = 0660},
#endif

#ifdef CONFIG_HW_MIGRATION_NOTIFY
	{.name = "freq_inc_notify", .uid = SYSTEM_UID, .gid = SYSTEM_GID, .mode = 0660},
	{.name = "freq_dec_notify", .uid = SYSTEM_UID, .gid = SYSTEM_GID, .mode = 0660},
#endif

	/* add here */
	INVALID_ATTR,
};
