/*
 * cpufreq_schedutil_sysfs_hw.c
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/************************** hw sysfs interface ************************/
static struct sugov_tunables_hw *to_sugov_tunables_hw(struct gov_attr_set
						      *attr_set)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return &tunables->tunables_hw;
}

static ssize_t overload_duration_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->overload_duration);
}

static ssize_t overload_duration_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	struct sugov_policy *sg_policy = NULL;
	struct sugov_policy_hw *sg_policy_hw = NULL;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->overload_duration = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy_hw = &sg_policy->sg_policy_hw;
		sg_policy_hw->overload_duration_ns = val * NSEC_PER_MSEC;
	}

	return count;
}

static ssize_t go_hispeed_load_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", tunables->go_hispeed_load);
}

static ssize_t go_hispeed_load_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->go_hispeed_load = val;

	return count;
}

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->hispeed_freq = val;

	return count;
}

static ssize_t boost_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->boost);
}

static ssize_t boost_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->boost = val;

	if (tunables->boost) {
		trace_cpufreq_schedutil_boost("on");
		if (!tunables->boosted)
			hw_sugov_boost(attr_set);
	} else {
		tunables->boostpulse_endtime = ktime_to_us(ktime_get());
		trace_cpufreq_schedutil_unboost("off");
	}

	return count;
}

static ssize_t boostpulse_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;
	u64 now;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	now = ktime_to_us(ktime_get());
	if (tunables->boostpulse_endtime +
	    tunables->boostpulse_min_interval > now)
		return count;

	tunables->boostpulse_endtime = now + tunables->boostpulse_duration;
	trace_cpufreq_schedutil_boost("pulse");

	if (!tunables->boosted)
		hw_sugov_boost(attr_set);

	return count;
}

static ssize_t boostpulse_duration_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->boostpulse_duration);
}

static ssize_t boostpulse_duration_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->boostpulse_duration = val;

	return count;
}

static ssize_t io_is_busy_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->io_is_busy);
}

static ssize_t io_is_busy_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->io_is_busy = val;
	sched_set_io_is_busy(val);

	return count;
}

static ssize_t fast_ramp_down_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->fast_ramp_down);
}

static ssize_t fast_ramp_down_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->fast_ramp_down = val;

	return count;
}

static ssize_t fast_ramp_up_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->fast_ramp_up);
}

static ssize_t fast_ramp_up_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->fast_ramp_up = val;

	return count;
}

static ssize_t freq_reporting_policy_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			tunables->freq_reporting_policy);
}

static ssize_t freq_reporting_policy_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->freq_reporting_policy = val;

	return count;
}

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp = NULL;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data = NULL;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;
	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				 tunables->target_loads[i],
				 i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return ret;
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int *new_target_loads;
	unsigned long flags;
	int ntokens;
	int i;

	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_target_loads))
		return PTR_ERR(new_target_loads);

	for (i = 0; i < ntokens; i++) {
		if (new_target_loads[i] == 0) {
			kfree(new_target_loads);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);
	tunables->target_loads = new_target_loads;
	tunables->ntarget_loads = ntokens;
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return count;
}

static ssize_t above_hispeed_delay_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->nabove_hispeed_delay; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				 tunables->above_hispeed_delay[i],
				 i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);

	return ret;
}

static ssize_t above_hispeed_delay_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int *new_above_hispeed_delay;
	unsigned long flags;
	int ntokens;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_above_hispeed_delay))
		return PTR_ERR(new_above_hispeed_delay);

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);
	if (tunables->above_hispeed_delay != default_above_hispeed_delay)
		kfree(tunables->above_hispeed_delay);
	tunables->above_hispeed_delay = new_above_hispeed_delay;
	tunables->nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);

	return count;
}

static ssize_t min_sample_time_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->min_sample_time_lock, flags);

	for (i = 0; i < tunables->nmin_sample_time; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				 tunables->min_sample_time[i],
				 i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->min_sample_time_lock, flags);

	return ret;
}

static ssize_t min_sample_time_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);
	unsigned int *new_min_sample_time;
	unsigned long flags;
	int ntokens;

	new_min_sample_time = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_min_sample_time))
		return PTR_ERR(new_min_sample_time);

	spin_lock_irqsave(&tunables->min_sample_time_lock, flags);
	if (tunables->min_sample_time != default_min_sample_time)
		kfree(tunables->min_sample_time);
	tunables->min_sample_time = new_min_sample_time;
	tunables->nmin_sample_time = ntokens;
	spin_unlock_irqrestore(&tunables->min_sample_time_lock, flags);

	return count;
}

static ssize_t timer_slack_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%d\n", tunables->timer_slack_val);
}

static ssize_t timer_slack_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	tunables->timer_slack_val = val;
	return count;
}

static ssize_t iowait_boost_step_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->iowait_boost_step);
}

static ssize_t iowait_boost_step_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	int ret;
	unsigned int val;
	struct sugov_tunables_hw *tunables = to_sugov_tunables_hw(attr_set);

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	tunables->iowait_boost_step = val;
	return count;
}

static struct governor_attr overload_duration = __ATTR_RW(overload_duration);
static struct governor_attr go_hispeed_load = __ATTR_RW(go_hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr boost = __ATTR_RW(boost);
static struct governor_attr target_loads = __ATTR_RW(target_loads);
static struct governor_attr above_hispeed_delay =
					__ATTR_RW(above_hispeed_delay);
static struct governor_attr min_sample_time = __ATTR_RW(min_sample_time);
static struct governor_attr boostpulse = __ATTR_WO(boostpulse);
static struct governor_attr boostpulse_duration =
					__ATTR_RW(boostpulse_duration);
static struct governor_attr io_is_busy = __ATTR_RW(io_is_busy);
static struct governor_attr timer_slack = __ATTR_RW(timer_slack);
static struct governor_attr fast_ramp_down = __ATTR_RW(fast_ramp_down);
static struct governor_attr fast_ramp_up = __ATTR_RW(fast_ramp_up);
static struct governor_attr freq_reporting_policy =
					__ATTR_RW(freq_reporting_policy);
static struct governor_attr iowait_boost_step = __ATTR_RW(iowait_boost_step);
