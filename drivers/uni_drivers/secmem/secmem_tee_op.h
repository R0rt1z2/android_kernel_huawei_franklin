/* Copyright (c) Huawei Technologies Co., Ltd. 2001-2019. All rights reserved.
 * FileName: secmem_tee_op.h
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef _SECMEM_TEE_OP_H
#define _SECMEM_TEE_OP_H

#include <teek_client_api.h>
#include <teek_client_id.h>
#include <teek_client_constants.h>

enum SEC_Task {
	SEC_TASK_DRM = 0x0,
	SEC_TASK_SEC,
	SEC_TASK_TINY,
	SEC_TASK_MAX,
};

enum ion_ta_tag {
	ION_SEC_CMD_PGATBLE_INIT = 0,
	ION_SEC_CMD_ALLOC,
	ION_SEC_CMD_FREE,
	ION_SEC_CMD_MAP_IOMMU,
	ION_SEC_CMD_UNMAP_IOMMU,
	ION_SEC_CMD_MAP_USER,
	ION_SEC_CMD_UNMAP_USER,
	ION_SEC_CMD_TABLE_SET,
	ION_SEC_CMD_TABLE_CLEAN,
	ION_SEC_CMD_VLTMM,
#ifdef SECMEM_UT
	ION_SEC_CMD_TEST,
	ION_SEC_CMD_TEST_RECY,
#endif
	ION_SEC_CMD_MAX,
};



#define SECBOOT_CMD_ID_MEM_ALLOCATE 0x1

#define sec_debug(fmt, ...) \
	pr_info(fmt, ##__VA_ARGS__)


struct mem_chunk_list {
	u32 protect_id;
	union {
		u32 nents;
		u32 buff_id;
	};
	u32 va;
	void *phys_addr;  /* Must be the start addr of struct tz_pageinfo */
	u32 size;
	u32 cache;
	int prot;
	int mode;
	u32 smmuid;
	u32 sid;
	u32 ssid;
};

struct tz_pageinfo {
	u64 addr;
	u32 nr_pages;
} __aligned(8);

#define TEE_SECMEM_NAME "sec_mem"
#define TEE_VLTMM_NAME  "vltmm"

int secmem_tee_init(TEEC_Context *context, TEEC_Session *session,
				const char *package_name);

int secmem_tee_exec_cmd(TEEC_Session *session,
		       struct mem_chunk_list *mcl, u32 cmd);

void secmem_tee_destroy(TEEC_Context *context, TEEC_Session *session);

#endif
