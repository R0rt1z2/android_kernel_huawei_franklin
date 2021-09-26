/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * FileName: vltmm.h
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef __VLTMM__H
#define __VLTMM__H

#include "vltmm_agent.h"

enum {
	SMEM_SECCM_XM = 0,
	SMEM_SECOS_1M = 1,
	SMEM_SECOS_2M = 2,
	SMEM_SECOS_4M = 3,
	SMEM_HEAPS_MAX,
};

enum {
	ASYNC_ALLOC,
	ASYNC_FREE,
};

enum {
	SUB_CMD_FREE,
	SUB_CMD_DUMP,
	SUB_CMD_UT_MALLOC,
	SUB_CMD_UT_FREE,
	SUB_CMD_UT_SHAREMEM,
	SUB_CMD_UT_PERF,     /* malloc N timer, then all free */
	SUB_CMD_UT_PERF_EXT, /* malloc-free N times*/
	SUB_CMA_CMA_ALLOC_TEST
};

struct smem_heap {
	u64 avail;
	u64 used;
	u64 min;
	u64 low;
	u64 high;
	u64 max;
	u64 gran;
	u64 sid;
	u64 init;
	u64 escape_flag;
	struct cma *cma;
	phys_addr_t *phy_array;
	struct task_struct *alloc_task;
	atomic_t alloc_run;
	struct mutex mutex;
	wait_queue_head_t alloc_wq;
	atomic_t wait_flag;
	unsigned long watermark;
};

struct smem_free_task {
	struct task_struct *free_task;
	wait_queue_head_t free_wq;
	atomic_t wait_flag;
};

#define SMEM_SCAN_CNT   8
#define SMEM_GRAN_CNT   128

/* num of gran */
#define SMEM_DEFAULT_MIN    0x2
#define SMEM_DEFAULT_LOW    0x20
#define SMEM_DEFAULT_HIGH   0x60

#define SMEM_GRAN_2M        0x200000
#define SMEM_POOL_MAX       0x10000000

#define SMEM_SHRINKER_NUM         16

#define SMEM_TEST_MAGIC 0xAA

int __smem_allocator_msg_alloc(struct vltmm_msg_t *msg, phys_addr_t *data, uint32_t len, uint32_t num);
void __smem_allocator_msg_free(uint32_t sid, phys_addr_t *data, uint32_t num);
int smemheaps_init(void);
void smem_sysfs_init(void);
#ifdef CONFIG_ZONE_MOVABLE_CMA
void set_vltmm_cma(struct cma *cma);
#endif

#endif
