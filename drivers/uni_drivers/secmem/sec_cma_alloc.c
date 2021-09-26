/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * FileName: sec_cma_alloc.c
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */

#define pr_fmt(fmt) "secmem: " fmt

#include "secmem_api.h"
#include <linux/cma.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <asm/pgalloc.h>

#include <ion_drv.h>
#include <ion_priv.h>
#include <ion.h>
#include "secmem_tee_op.h"
#include "ion_sec_contig.h"
#ifdef CONFIG_HW_SECMEM
#include "vltmm.h"
#endif

static struct cma *sec_cmas[SEC_SVC_MAX];
#define ION_HEAP_TYPE_MULTIMEDIA_SEC 13

struct svc_cap {
	const char *name;
	unsigned long align;
	bool is_contig;
	bool pg_valid; /* page table valid in kernel */
};

static struct svc_cap svc_table[SEC_SVC_MAX] = {
	{ "TUI", SZ_2M, true, true },
	{ "eID", SZ_2M, true, false },
};
#define SVC_ID_MASK       0x03

static void set_svc_cma(u64 svc_id, struct cma *cma)
{
	unsigned int i;

	for (i = 0; i < SEC_SVC_MAX; i++) {
		if (svc_id & (1UL << i)) {
			pr_info("set svc cma, id: %u\n", i);
			sec_cmas[i] = cma;
		}
	}
}

static struct cma *get_svc_cma(int id)
{
	return sec_cmas[id];
}

#ifndef CONFIG_ZONE_MOVABLE_CMA
static int sec_cma_reserve(struct reserved_mem *rmem)
{
	struct cma *cma = NULL;
	u64 *val = NULL;
	u64 svc_id;
	phys_addr_t align = PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	phys_addr_t mask = align - 1;
	unsigned long node = rmem->fdt_node;
	int err;

	val = (u64 *)of_get_flat_dt_prop(node, "svc-id", NULL);
	if (!val)
		return -EINVAL;

	svc_id = be64_to_cpu(*val);

	if (!of_get_flat_dt_prop(node, "reusable", NULL) ||
		of_get_flat_dt_prop(node, "no-map", NULL))
		return -EINVAL;

	pr_info("secmem cma init , base: %llx, size %llx, mask: %llx\n",
			rmem->base, rmem->size, mask);
	if ((rmem->base & mask) || (rmem->size & mask))
		return -EINVAL;

	if (!memblock_is_memory(rmem->base)) {
		memblock_free(rmem->base, rmem->size);
		return -EINVAL;
	}

	err = cma_init_reserved_mem(rmem->base, rmem->size, 0,
				rmem->name, &cma);
	if (err)
		return err;

	set_svc_cma(svc_id, cma);

	set_seccg_cma(cma);

	return 0;
}

RESERVEDMEM_OF_DECLARE(dynamic_cma, "dynamic-cma-pool", sec_cma_reserve);
#else
void secmem_set_cma(struct cma *cma)
{
	if (!cma)
		return;

	set_svc_cma(SVC_ID_MASK, cma);
	set_seccg_cma(cma);
#ifdef CONFIG_HW_VLTMM
	set_vltmm_cma(cma);
#endif
}
#endif

void change_secpage_range(unsigned long addr,
			unsigned long size, int enable)
{
	if (!PAGE_ALIGNED(addr) || !PAGE_ALIGNED(size))
		return;

	if (enable) {
		set_memory_valid(addr, size >> PAGE_SHIFT, 1);
	} else {
		__flush_dcache_area((void *)(uintptr_t)addr, size);
		set_memory_valid(addr, size >> PAGE_SHIFT, 0);
	}
}

static void __change_page_attr(int svc_id, unsigned long addr,
			unsigned long size, int enable)
{
	if (svc_table[svc_id].pg_valid) {
		__flush_dcache_area((void *)(uintptr_t)addr, size);
		return;
	}

	change_secpage_range(addr, size, enable);
}

static void do_free_pages(u32 nents, struct sg_table *table,
			struct cma *cma, int id)
{
	unsigned long i;
	struct scatterlist *sg = NULL;
	struct page *page = NULL;

	for_each_sg(table->sgl, sg, nents, i) {
		page = sg_page(sg);

		__change_page_attr(id, (unsigned long)page_address(page),
				sg->length, 1);

		cma_release(cma, page, sg->length / PAGE_SIZE);
	}

	flush_tlb_all();
	sg_free_table(table);
}

static struct sg_table *secmem_alloc_contig(struct cma *cma,
					unsigned long size, unsigned long align, int id)
{
	struct sg_table *table = NULL;
	struct page *page = NULL;
	u32 count;

	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return NULL;

	if (sg_alloc_table(table, 1, GFP_KERNEL)) {
		pr_err("secmem alloc sg failed\n");
		goto err_sg;
	}

	count = (u32)size / PAGE_SIZE;
	page = cma_alloc(cma, count, (u32)get_order((u32)align), false);
	if (!page) {
		pr_err("secmem cma alloc failed\n");
		goto err_cma;
	}

	__change_page_attr(id, (unsigned long)page_address(page),
			size, 0);

	sg_set_page(table->sgl, page, (unsigned int)size, 0);

	return table;

err_cma:
	sg_free_table(table);
err_sg:
	kfree(table);

	return NULL;
}

static struct sg_table *secmem_alloc_sg(struct cma *cma, unsigned long size,
					unsigned long align, int id)
{
	struct sg_table *table = NULL;
	struct scatterlist *sg = NULL;
	struct page *page = NULL;
	unsigned long size_remaining = size;
	unsigned long nents;
	unsigned long alloc_size;
	unsigned long count;
	u32 i = 0;

	if (!align)
		return NULL;

	nents = size / align;

	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		return NULL;

	if (sg_alloc_table(table, (unsigned int)nents, GFP_KERNEL))
		goto free_table;

	sg = table->sgl;
	while (size_remaining) {
		if (size_remaining > align)
			alloc_size = align;
		else
			alloc_size = size_remaining;

		count = alloc_size / PAGE_SIZE;
		page = cma_alloc(cma, count, (u32)get_order((u32)align), false);
		if (!page)
			goto free_pages;

		__change_page_attr(id, (unsigned long)page_address(page),
				alloc_size, 0);
		size_remaining -= alloc_size;
		sg_set_page(sg, page, (unsigned int)alloc_size, 0);
		sg = sg_next(sg);
		i++;
	}

	return table;

free_pages:
	do_free_pages(i, table, cma, id);
free_table:
	kfree(table);

	return NULL;
}

struct sg_table *cma_secmem_alloc(int id, unsigned long size)
{
	struct cma *cma = NULL;
	struct sg_table *table = NULL;
	unsigned long align;

	if (id < 0 || id >= SEC_SVC_MAX || !size) {
		pr_err("secmem err id %d\n", id);
		return NULL;
	}

	cma = get_svc_cma(id);
	if (!cma) {
		pr_err("secmem can't find cma, id %d\n", id);
		return NULL;
	}

	align = svc_table[id].align;
	size = ALIGN(size, align);
	if (cma_get_size(cma) < size) {
		pr_err("secmem err size: 0x%lx, cma_size: %lx\n", size, cma_get_size(cma));
		return NULL;
	}

	if (svc_table[id].is_contig)
		table = secmem_alloc_contig(cma, size, align, id);
	else
		table = secmem_alloc_sg(cma, size, align, id);

	return table;
}

void cma_secmem_free(int id, struct sg_table *table)
{
	struct cma *cma = NULL;
	struct page *page = NULL;
	struct scatterlist *sg = NULL;
	u32 i;

	if (id < 0 || id >= SEC_SVC_MAX || !table)
		return;

	cma = get_svc_cma(id);
	if (!cma)
		return;

	for_each_sg(table->sgl, sg, table->nents, i) {
		page = sg_page(sg);
		__change_page_attr(id, (unsigned long)page_address(page),
				sg->length, 1);
		cma_release(cma, page, sg->length / PAGE_SIZE);
	}

	sg_free_table(table);
	kfree(table);
}

int ion_secmem_get_phys(struct dma_buf *dmabuf,
			phys_addr_t *addr, size_t *len)
{
	struct ion_buffer *buffer = NULL;
	int ret;
	unsigned long phys = 0;

	if (!dmabuf || !addr || !len)
		return -EINVAL;

	buffer = dmabuf->priv;
	if (!buffer) {
		pr_err("%s: Ion buffer pointer is NULL!\n", __func__);
		return -EINVAL;
	}

	if (!buffer->heap || !buffer->heap->ops) {
		pr_err("ion buffer heap NULL\n");
		return -ENODEV;
	}

	if (!buffer->heap->ops->phys) {
		pr_err("ion phys handle NULL\n");
		return -ENODEV;
	}

	mutex_lock(&buffer->lock);
	ret = buffer->heap->ops->phys(buffer->heap, buffer, &phys, len);
	*addr = (phys_addr_t)phys;
	mutex_unlock(&buffer->lock);

	pr_info("ion get phys, name: %s type: %d addr: %lx\n", buffer->heap->name,
		buffer->heap->type, (unsigned long)*addr);

	return ret;
}

int secmem_get_buffer(int fd, struct sg_table **table, uint64_t *id,
		      enum SEC_SVC *type)
{
	struct dma_buf *dmabuf = NULL;
	struct ion_buffer *buffer = NULL;
	struct ion_heap *heap = NULL;
	unsigned long addr = 0;
	size_t len = 0;

	if (!table || !id || !type)
		return -EINVAL;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("%s: invalid fd!\n", __func__);
		return -EINVAL;
	}

	buffer = dmabuf->priv;
	if (!buffer) {
		pr_err("%s: dmabuf has no ION buffer!\n", __func__);
		goto err;
	}

	heap = buffer->heap;
	if (!heap) {
		pr_err("%s: invalid ION buffer's heap!\n", __func__);
		goto err;
	}

	if (heap->type == ION_HEAP_TYPE_MULTIMEDIA_SEC) {
		if (!buffer->heap->ops->phys) {
			pr_err("ion phys handle NULL\n");
			goto err;
		}

		mutex_lock(&buffer->lock);
		buffer->heap->ops->phys(buffer->heap, buffer, &addr, &len);
		mutex_unlock(&buffer->lock);

		*id = addr;
		*type = SEC_DRM_TEE;
	} else {
		*table = buffer->sg_table;
		*type = SEC_SVC_MAX;
	}
	pr_info("ion get buf, type: %d sg_nents: %d \n",
			heap->type,
			buffer->sg_table ? buffer->sg_table->orig_nents : 0);

	dma_buf_put(dmabuf);

	return 0;
err:
	dma_buf_put(dmabuf);
	return -EINVAL;
}

#ifdef CONFIG_HW_SECMEM_DEBUG
void secmem_ion_test(int fd)
{
	struct sg_table *table = NULL;
	uint64_t id = 0;
	enum SEC_SVC type = 0;
	int ret;
	struct dma_buf *dmabuf = NULL;
	phys_addr_t addr = 0;
	size_t len = 0;

	pr_err("ion test: fd %d\n", fd);

	ret = secmem_get_buffer(fd, &table, &id, &type);
	pr_err("ion test get buf: ret:%d, table:%p, id:%d, type:%d\n",
				ret, table, id, type);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("%s: invalid fd!\n", __func__);
		return;
	}
	ret = ion_secmem_get_phys(dmabuf, &addr, &len);
	pr_err("ion test get phys: ret:%d, addr:%lx, len: %zu\n",
				ret, (unsigned long)addr, len);
}
#endif
