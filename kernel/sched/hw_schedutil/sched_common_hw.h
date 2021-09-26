/*
 * sched_common_hw.h
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SCHED_COMMON_HW_H__
#define __SCHED_COMMON_HW_H__

/* bits in struct sugov_cpu flags field */
enum {
	WALT_WINDOW_ROLLOVER            = (1 << 0),  /* 1 */
	INTER_CLUSTER_MIGRATION_SRC     = (1 << 1),  /* 2 */
	INTER_CLUSTER_MIGRATION_DST     = (1 << 2),  /* 4 */
	ADD_TOP_TASK                    = (1 << 3),  /* 8 */
	ADD_ED_TASK                     = (1 << 4),  /* 16 */
	CLEAR_ED_TASK                   = (1 << 5),  /* 32 */
	POLICY_MIN_RESTORE              = (1 << 6),  /* 64 */
	FORCE_UPDATE                    = (1 << 7),  /* 128 */
	PRED_LOAD_CHANGE                = (1 << 8),  /* 256 */
	PRED_LOAD_WINDOW_ROLLOVER       = (1 << 9),  /* 512 */
	PRED_LOAD_ENQUEUE               = (1 << 10), /* 1024 */
	RQ_SET_MIN_UTIL                 = (1 << 11), /* 2048 */
	RQ_ENQUEUE_MIN_UTIL             = (1 << 12), /* 4096 */
	FRAME_UPDATE                    = (1 << 13), /* 8192 */
};

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL_COMMON
void sugov_mark_util_change(int cpu, unsigned int flags);
void sugov_check_freq_update(int cpu);
#else
static inline void sugov_mark_util_change(int cpu, unsigned int flags) {}
static inline void sugov_check_freq_update(int cpu) {}
#endif

#endif
