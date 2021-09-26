/*
 * cpufreq_schedutil_hw.h
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct sugov_tunables_hw {
	/*
	 * boost freq to max when running without idle over this duration time
	 */
	unsigned int overload_duration;
	/* Hi speed to bump to from lo speed when load burst (default max) */
	unsigned int hispeed_freq;

	/* Go to hi speed when CPU load at or above this value. */
	unsigned long go_hispeed_load;

	/* Target load. Lower values result in higher CPU speeds. */
	spinlock_t target_loads_lock;
	unsigned int *target_loads;
	int ntarget_loads;

	/*
	 * Wait this long before raising speed above hispeed, by default a
	 * single timer interval.
	 */
	spinlock_t above_hispeed_delay_lock;
	unsigned int *above_hispeed_delay;
	int nabove_hispeed_delay;

	/*
	 * The minimum amount of time to spend at a frequency before we can ramp
	 * down.
	 */
	spinlock_t min_sample_time_lock;
	unsigned int *min_sample_time;
	int nmin_sample_time;

	/* Non-zero means indefinite speed boost active */
	int boost;
	/* Duration of a boot pulse in usecs */
	int boostpulse_duration;
	/* End time of boost pulse in ktime converted to usecs */
	u64 boostpulse_endtime;
	bool boosted;
	/* Minimun boostpulse interval */
	int boostpulse_min_interval;

	/*
	 * Max additional time to wait in idle, beyond timer_rate, at speeds
	 * above minimum before wakeup to reduce speed, or -1 if unnecessary.
	 */
	int timer_slack_val;

	bool io_is_busy;
	bool fast_ramp_up;
	bool fast_ramp_down;
	unsigned int freq_reporting_policy;
	unsigned int iowait_boost_step;
};

struct sugov_policy_hw {
	u64 overload_duration_ns;
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	u64 time;
	/* protect slack timer */
	raw_spinlock_t timer_lock;
	/* policy slack timer */
	struct timer_list pol_slack_timer;
	unsigned int min_freq;    /* in kHz */
	/* remember last active cpu and set slack timer on it */
	unsigned int trigger_cpu;
	/* most busy cpu */
	unsigned int max_cpu;
	unsigned int iowait_boost;
	u64 last_iowait;
	atomic_t skip_min_sample_time;
	atomic_t skip_hispeed_logic;
	bool governor_enabled;

#ifdef CONFIG_HW_CPU_FREQ_LOCK_DETECT
#define LONGER_MIN_SAMPLE_TIME_ELAPSED_DURATION (4 * NSEC_PER_SEC)
	ktime_t start_time;
	ktime_t end_time;
	bool locked;
#endif
#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
	unsigned int min_util;
#endif
};

struct sugov_cpu_hw {
	u64 idle_update_ts;
	u64 last_idle_time;
	bool enabled;
};
