/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: rootagent memory initial
 * Create: 2016-04-01
 */

#include "rootagent_mem.h"
#include <linux/mm.h>
#include "rootagent_common.h"

#define G_MEM_FLAG_VALID 1

static void *g_rwmem;
static void *g_swap;
static int g_mem_flag;

int rm_mem_init(void)
{
	if (g_mem_flag != 0)
		goto out;

	g_rwmem = (void *)(uintptr_t) __get_free_pages(GFP_KERNEL,
		get_order(RM_PRE_ALLOCATE_SIZE));
	if (g_rwmem == NULL) {
		antiroot_error("mem allocated failed\n");
		return -ENOMEM;
	}
	g_swap = (void *)(uintptr_t) __get_free_pages(GFP_KERNEL,
		get_order(RM_PRE_ALLOCATE_SIZE));
	if (g_swap == NULL) {
		antiroot_error("Swap mem allocated failed\n");
		free_pages((uintptr_t)g_rwmem, get_order(RM_PRE_ALLOCATE_SIZE));
		g_rwmem = NULL;
		return -ENOMEM;
	}
	g_mem_flag = G_MEM_FLAG_VALID;
out:
	antiroot_debug(ROOTAGENT_DEBUG_MEM, "rm_mem_init successful!\n");
	return 0;
}

void rm_mem_destroy(void)
{
	if (g_rwmem != NULL) {
		free_pages((uintptr_t)g_rwmem, get_order(RM_PRE_ALLOCATE_SIZE));
		g_rwmem = NULL;
	}
	if (g_swap != NULL) {
		free_pages((uintptr_t)g_swap, get_order(RM_PRE_ALLOCATE_SIZE));
		g_swap = NULL;
	}
}

int initial_rm_shared_mem(TEEC_TempMemoryReference *rwMem,
			TEEC_TempMemoryReference *swapMem)
{
	if ((swapMem == NULL) || (rwMem == NULL)) {
		antiroot_error("Bad param!\n");
		return -EINVAL;
	}

	if (g_swap != NULL) {
		swapMem->buffer = g_swap;
		swapMem->size = RM_PRE_ALLOCATE_SIZE;
	} else {
		antiroot_error("initial_rm_shared_mem failed!\n");
		return -ENOMEM;
	}

	if (g_rwmem != NULL) {
		rwMem->buffer = g_rwmem;
		rwMem->size = RM_PRE_ALLOCATE_SIZE;
	} else {
		antiroot_error("initial_rm_shared_mem failed!\n");
		return -ENOMEM;
	}
	antiroot_debug(ROOTAGENT_DEBUG_MEM, "initial_rm_shared_mem successful!\n");
	return 0;
}
