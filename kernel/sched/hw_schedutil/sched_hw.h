/*
 * sched_hw.h
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SCHED_HW_H__
#define __SCHED_HW_H__
#include "../hw_walt/walt_hw.h"

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
extern unsigned int sched_io_is_busy;
extern void sched_set_io_is_busy(int val);
extern unsigned long boosted_freq_policy_util(int cpu);
extern bool sugov_iowait_boost_pending(int cpu);
#else
#ifdef CONFIG_HAVE_QCOM_SCHED_WALT
extern void sched_set_io_is_busy(int val);
#else
static inline void sched_set_io_is_busy(int val) {}
static inline unsigned long boosted_freq_policy_util(int cpu) { return 0; }
static inline bool sugov_iowait_boost_pending(int cpu) { return false; }
#endif
#endif

#endif
