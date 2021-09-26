/*
 * mm/mem_track/mem_trace_test.c	memory track test function
 *
 * Copyright(C) 2020 Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "mem_stack.h"
#include "slab.h"

#define T_SLUB_NAME_LEN    256
#define SLUB_CACHE_LEN	  250
static int slub_type[] = { SLUB_ALLOC, SLUB_FREE };
static const char * const slub_text[] = {
	"SLUB_ALLOC",
	"SLUB_FREE",
};

static const char * const track_text[] = {
	"ION_TRACK",
	"SLUB_TRACK",
	"LSLUB_TRACK",
	"VMALLOC_TRACK",
	"CMA_TRACK",
	"ZSPAGE_TRACK",
	"BUDDY_TRACK",
	"SKB_TRACK",
};

static int vmalloc_type[] = { VM_IOREMAP, VM_ALLOC, VM_MAP, VM_USERMAP };
static const char * const vmalloc_text[] = {
	"VM_IOREMAP",
	"VM_ALLOC",
	"VM_MAP",
	"VM_USERMAP",
};

static struct kmem_cache *slub_test_cache;

struct slub_string_type {
	int type;
	char name[T_SLUB_NAME_LEN];
};

enum {
	KMALLOC_SIZE_32 = 1,
	KMALLOC_SIZE_64,
	KMALLOC_SIZE_128,
	KMALLOC_SIZE_256,
	KMALLOC_SIZE_512,
	KMALLOC_SIZE_1024,
	KMALLOC_SIZE_2048,
	KMALLOC_SIZE_4096,
	KMALLOC_SIZE_8192
};

static struct slub_string_type slub_case[] = {
	{KMALLOC_SIZE_32,  "kmalloc-32"},
	{KMALLOC_SIZE_64,  "kmalloc-64"},
	{KMALLOC_SIZE_128,  "kmalloc-128"},
	{KMALLOC_SIZE_256,  "kmalloc-256"},
	{KMALLOC_SIZE_512,  "kmalloc-512"},
	{KMALLOC_SIZE_1024,  "kmalloc-1024"},
	{KMALLOC_SIZE_2048,  "kmalloc-2048"},
	{KMALLOC_SIZE_4096,  "kmalloc-4096"},
	{KMALLOC_SIZE_8192,  "kmalloc-8192"},
};

void mm_slub_truck_on(int type)
{
	unsigned int i;
	char name[T_SLUB_NAME_LEN];

	for (i = 0; i < ARRAY_SIZE(slub_case); i++) {
		if (slub_case[i].type == type) {
			mm_slub_track_on(slub_case[i].name);
			pr_err("%s: name:%s truck on\n",
				__func__, slub_case[i].name);
			return;
		}
	}
	/* other situation */
	slub_test_cache = kmem_cache_create("slub_cache",
		SLUB_CACHE_LEN, 0, 0, NULL);
	if (!slub_test_cache) {
		pr_err("%s: alloc failed!\n", __func__);
		return;
	}
	strncpy(name, slub_test_cache->name, strlen(slub_test_cache->name));
	mm_slub_track_on(name);
	pr_err("%s: slub_test_cache truck on\n", __func__);
}

void mm_slub_truck_off(int type)
{
	unsigned int i;
	char name[T_SLUB_NAME_LEN];

	for (i = 0; i < ARRAY_SIZE(slub_case); i++) {
		if (slub_case[i].type == type) {
			mm_slub_track_off(slub_case[i].name);
			pr_err("%s: name:%s truck off\n",
				__func__, slub_case[i].name);
			return;
		}
	}
	/* other situation */
	if (!slub_test_cache)
		return;
	strncpy(name, slub_test_cache->name, strlen(slub_test_cache->name));
	mm_slub_track_off(name);
	pr_err("%s: slub_test_cache truck off\n", __func__);
}

void mm_kmalloc_leak_test(size_t size, int num)
{
	void *va = NULL;
	int i;

	if (!size || size > SZ_4M)
		return;

	for (i = 0; i < num; i++) {
		va = kmalloc(size, GFP_KERNEL);
		kfree(va);
	}
}

void mm_kcache_alloc_test(int num)
{
	int i;
	void *va = NULL;

	if (!slub_test_cache) {
		pr_err("%s, slub_test_cache is null\n", __func__);
		return;
	}
	for (i = 0; i < num; i++) {
		va = kmem_cache_alloc(slub_test_cache, GFP_KERNEL);
		if (!va)
			return;
		kmem_cache_free(slub_test_cache, va);
	}
}

int mm_vmap_test(unsigned int npages)
{
	int i;
	void *vaddr = NULL;
	pgprot_t pgprot;
	struct page **pages = NULL;
	struct page **tmp = NULL;

	pages = vmalloc(sizeof(struct page *) * (unsigned long)npages);
	if (!pages)
		return -ENOMEM;

	tmp = pages;
	for (i = 0; i < npages; i++) {
		struct page *p = alloc_pages(GFP_USER, 0);

		if (!p) {
			pr_err("%s, alloc_pages failed\n", __func__);
			goto alloc_err;
		}
		*(tmp++) = p;
	}
	pgprot = PAGE_KERNEL;
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);
	if (!vaddr) {
		pr_err("%s, vmap failed\n", __func__);
		return -ENOMEM;
	}
	return 0;

alloc_err:
	tmp = pages;
	while (i > 0) {
		__free_pages(*tmp, 0);
		tmp++;
		i--;
	}
	vfree(pages);
	return -ENOMEM;
}

void mm_ioremap_test(phys_addr_t offset, phys_addr_t size)
{
	ioremap(offset, size);
}

void mm_mem_trace_test(void)
{
	size_t size;
	int i;
	ktime_t time_start, time_stop;
	long long t_ns;

	pr_err("========mem stat start==========\n");
	for (i = START_TRACK; i < NR_TRACK; i++) {
		time_start = ktime_get();
		size = mm_get_mem_total(i);
		time_stop = ktime_get();
		t_ns = ktime_to_ns(ktime_sub(time_stop, time_start));
		pr_err("%s used: %ld KB\n", track_text[i], size / 1024L);
		pr_err("cost time is %lld us\n", t_ns / 1000L);
	}
	pr_err("=========mem stat end==========\n");
}

void mm_mem_stats_test(int type)
{
	size_t size;
	int i;
	ktime_t time_start, time_stop;
	long long t_ns;

	pr_err("========mem stat start==========\n");
	for (i = START_TRACK; i < NR_TRACK; i++) {
		if (type == i) {
			time_start = ktime_get();
			size = mm_get_mem_total(i);
			time_stop = ktime_get();
			t_ns = ktime_to_ns(ktime_sub(time_stop, time_start));
			pr_err("%s used: %ld KB\n",
				track_text[i], size / 1024L);
			pr_err("cost time is %lld us\n", t_ns / 1000L);
			break;
		}
	}
	pr_err("=========mem stat end==========\n");
}

void mm_slub_detail_test(size_t len)
{
	int k;
	size_t size;
	struct mm_slub_detail_info *buf = NULL;

	pr_err("========get slub info start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;

	size = mm_get_mem_detail(SLUB_TRACK, (void *)buf, len);
	for (k = 0; k < min(len, size); k++)
		pr_err("name:%s, active:0x%lx, objsize:0x%x,total:0x%lx\n",
			(buf + k)->name, (buf + k)->active_objs,
			(buf + k)->objsize, (buf + k)->size);

	vfree(buf);
	pr_err("========get slub info end==========\n");
}

void mm_ion_detail_test(size_t len)
{
	int k;
	size_t size;
	struct mm_ion_detail_info *buf = NULL;

	pr_err("========get ion info start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;

	size = mm_get_mem_detail(ION_TRACK, (void *)buf, len);
	for (k = 0; k < min(len, size); k++)
		pr_err("pid:%d, size:0x%lx\n",
			(buf + k)->pid, (buf + k)->size);

	vfree(buf);
	pr_err("========get ion info end==========\n");
}

void mm_vmalloc_detail_test(size_t len, int type)
{
	size_t size;
	ktime_t time_start, time_stop;
	long long t_ns;
	struct mm_vmalloc_detail_info *buf = NULL;
	int k;

	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;
	pr_err("========get vmalloc info start==========\n");
	time_start = ktime_get();
	size = mm_get_mem_detail(VMALLOC_TRACK, buf, len);
	time_stop = ktime_get();
	t_ns = ktime_to_ns(ktime_sub(time_stop, time_start));
	for (k = 0; k < min(len, size); k++) {
		if (type == (buf + k)->type)
			pr_err("vmalloc type:%d, size:0x%lx\n",
				(buf + k)->type, (buf + k)->size);
	}
	pr_err("cost time is %lld us\n", t_ns / 1000L);
	vfree(buf);

	pr_err("========get vmalloc info end==========\n");
}

void mm_get_vmalloc_stack_test(size_t len)
{
	size_t size;
	ktime_t time_start;
	long long t_ns;
	unsigned int i;
	int k;
	int j = 0;
	struct mm_stack_info *buf = NULL;

	pr_err("========get vmalloc stack start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;

	for (i = 0; i < ARRAY_SIZE(vmalloc_type); i++) {
		time_start = ktime_get();
		mm_page_trace_open(VMALLOC_TRACK, vmalloc_type[i]);
		size = mm_page_trace_read(VMALLOC_TRACK, buf, len, 0);
		mm_page_trace_close(VMALLOC_TRACK, vmalloc_type[i]);
		t_ns = ktime_to_ns(ktime_sub(ktime_get(), time_start));
		pr_err("type:%s  used: %ld items\n", vmalloc_text[i], size);
		pr_err("cost time is %lld us\n", t_ns / 1000L);
		for (k = 0; k < min(len, size); k++) {
			unsigned long caller = (buf + k)->caller;
			atomic_t ref = (buf + k)->ref;

			pr_err("vmalloc stackinfo[%d]:%pS, ref:0x%x\n",
				j++, (void *)(uintptr_t)caller, atomic_read(&ref));
		}
	}
	vfree(buf);
	pr_err("========get vmalloc stack end==========\n");
}

void mm_get_buddy_stack_test(size_t len)
{
	int k;
	int j = 0;
	size_t size;
	ktime_t time_start;
	long long t_ns;
	struct mm_stack_info *buf = NULL;

	pr_err("========get buddy stack start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;

	time_start = ktime_get();
	mm_page_trace_open(BUDDY_TRACK, BUDDY_TRACK);
	size = mm_page_trace_read(BUDDY_TRACK, buf, len, BUDDY_TRACK);
	mm_page_trace_close(BUDDY_TRACK, BUDDY_TRACK);
	t_ns = ktime_to_ns(ktime_sub(ktime_get(), time_start));
	pr_err("BUDDY TRACK  there are %ld items, cost:%lld us\n",
		size, t_ns / 1000L);
	for (k = 0; k < min(len, size); k++) {
		unsigned long caller = (buf + k)->caller;
		atomic_t ref = (buf + k)->ref;

		pr_err("buddy stackinfo[%d]:%pS, ref:0x%x\n",
			j++, (void *)(uintptr_t)caller, atomic_read(&ref));
	}

	vfree(buf);
	pr_err("========get buddy stack end==========\n");
}

void mm_get_lslub_stack_test(size_t len)
{
	int k;
	int j = 0;
	size_t size;
	ktime_t time_start;
	long long t_ns;
	struct mm_stack_info *buf = NULL;

	pr_err("========get lslub stack start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;
	time_start = ktime_get();
	mm_page_trace_open(LSLUB_TRACK, LSLUB_TRACK);
	size = mm_page_trace_read(LSLUB_TRACK, buf, len, LSLUB_TRACK);
	mm_page_trace_close(LSLUB_TRACK, LSLUB_TRACK);
	t_ns = ktime_to_ns(ktime_sub(ktime_get(), time_start));
	pr_err("lslub TRACK  there are %ld items, cost:%lld us\n",
		size, t_ns / 1000L);
	for (k = 0; k < min(len, size); k++) {
		unsigned long caller = (buf + k)->caller;
		atomic_t ref = (buf + k)->ref;

		pr_err("lslub stackinfo[%d]:%pS, ref:0x%x\n",
			j++, (void *)(uintptr_t)caller, atomic_read(&ref));
	}

	vfree(buf);
	pr_err("========get lslub stack end==========\n");
}

void mm_get_all_slub_stack_test(size_t len)
{
	int k;
	int j = 0;
	unsigned int i;
	size_t size;
	ktime_t time_start;
	long long t_ns;
	struct mm_stack_info *buf = NULL;

	pr_err("========get slub stack start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;
	for (i = 0; i < ARRAY_SIZE(slub_type); i++) {
		time_start = ktime_get();
		mm_page_trace_open(SLUB_TRACK, slub_type[i]);
		size = mm_page_trace_read(SLUB_TRACK,
			buf, len, slub_type[i]);
		mm_page_trace_close(SLUB_TRACK, slub_type[i]);
		t_ns = ktime_to_ns(ktime_sub(ktime_get(), time_start));
		pr_err("type:%s  used: %ld items\n", slub_text[i], size);
		for (k = 0; k < min(len, size); k++) {
			unsigned long caller = (buf + k)->caller;
			atomic_t ref = (buf + k)->ref;

			pr_err("slub stackinfo[%d]:%pS, ref:0x%x\n",
				j++, (void *)(uintptr_t)caller, atomic_read(&ref));
		}
	}
	vfree(buf);
	pr_err("========get slub stack end==========\n");
}

void mm_get_slub_stack_test(size_t len)
{
	int k;
	int j = 0;
	unsigned int i;
	size_t size;
	struct mm_stack_info *buf = NULL;

	pr_err("========get slub stack start==========\n");
	buf = vmalloc(len * sizeof(*buf));
	if (!buf)
		return;
	for (i = 0; i < ARRAY_SIZE(slub_type); i++) {
		size = mm_page_trace_read(SLUB_TRACK,
			buf, len, slub_type[i]);
		pr_err("type:%s  used: %ld items\n", slub_text[i], size);
		for (k = 0; k < min(len, size); k++) {
			unsigned long caller = (buf + k)->caller;
			atomic_t ref = (buf + k)->ref;

			pr_err("slub stackinfo[%d]:%pS, ref:0x%x\n",
				j++, (void *)(uintptr_t)caller, atomic_read(&ref));
		}
	}
	vfree(buf);
	pr_err("========get slub stack end==========\n");
}

