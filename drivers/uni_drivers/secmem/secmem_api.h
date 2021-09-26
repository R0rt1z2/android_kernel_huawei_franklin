/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * FileName: secmem_api.h
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */

#ifndef __HW_SECMEM_API_H
#define __HW_SECMEM_API_H
#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/dma-buf.h>


enum SEC_SVC {
	SEC_TUI = 0,
	SEC_EID = 1,
	SEC_FACE_ID,
	SEC_FACE_ID_3D,
	SEC_DRM_TEE,
	SEC_HIAI,
	SEC_SVC_MAX,
};



struct sg_table *cma_secmem_alloc(int id, unsigned long size);
void cma_secmem_free(int id, struct sg_table *table);
int ion_secmem_get_phys(struct dma_buf *dmabuf,
			phys_addr_t *addr, size_t *len);

int secmem_get_buffer(int fd, struct sg_table **table, uint64_t *id,
		      enum SEC_SVC *type);

void change_secpage_range(unsigned long addr,
			unsigned long size, int enable);

#ifdef CONFIG_ZONE_MOVABLE_CMA
struct cma;
void secmem_set_cma(struct cma *cma);
#endif

#endif
