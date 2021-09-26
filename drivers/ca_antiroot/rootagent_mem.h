/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: rootagent memory interface
 * Create: 2016-04-01
 */
#ifndef _ROOT_MEM_H_
#define _ROOT_MEM_H_

#include <linux/mm.h>
#include "teek_client_type.h"

/*
 * @Function: rm_mem_init
 *
 * @brief: This function create two share memory,
 * one is used to read, and anther is used to write.
 *
 * @attention: This function is invoked by root_monitor_tee_init first.
 *
 * @param: void
 *
 * @retval: 0(successful), -1(failed)
 *
 * @depend on: null
 *
 */
int rm_mem_init(void);

/*
 * @Function:  rm_mem_destroy
 *
 * @brief: This function free the memory
 * created by rm_mem_init.
 *
 * @attention:
 * Two memory blocks are persistent,
 * but sometime you should consider to free them.
 *
 * @param: void
 *
 * @retval: void
 *
 * @depend on: null
 *
 */
void rm_mem_destroy(void);

/*
 * @Function:  initial_rm_shared_mem
 *
 * @brief:
 * This function initial two TEEC_SharedMemory
 * structures for communication with TEE.
 *
 * @attention: null
 *
 * @param: see teek_client_type.h
 *
 * @retval: void
 *
 * @depend on: teek_client_type.h
 *
 */
int initial_rm_shared_mem(TEEC_TempMemoryReference *rwMem,
			TEEC_TempMemoryReference *swapMem);

#endif
