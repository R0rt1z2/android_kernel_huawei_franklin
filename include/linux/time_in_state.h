/*
 * time_in_state.h
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

#ifndef __HUAWEI_TIME_IN_STATE_H
#define __HUAWEI_TIME_IN_STATE_H

#include <linux/cpu.h>
#include <linux/cpumask.h>

#ifdef CONFIG_HUAWEI_FREQ_STATS_COUNTING_IDLE
void time_in_state_update_freq(struct cpumask *cpus, unsigned int new_freq_index);
ssize_t time_in_state_print(int cpu, char *buf);
int time_in_freq_get(int cpu, u64 *freqs, int freqs_len);

#else /* !CONFIG_HUAWEI_FREQ_STATS_COUNTING_IDLE */

static inline void time_in_state_update_freq(struct cpumask *cpus, unsigned int new_freq_index)
{
}

static ssize_t time_in_state_print(int cpu, char *buf)
{
	return -EINVAL;
}

static inline int time_in_freq_get(int cpu, u64 *freqs, int freqs_len)
{
	return -EINVAL;
}
#endif /* CONFIG_HUAWEI_FREQ_STATS_COUNTING_IDLE */
#endif /* __HUAWEI_TIME_IN_STATE_H */
