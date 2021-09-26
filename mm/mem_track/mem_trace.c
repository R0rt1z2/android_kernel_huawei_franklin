/*
 * mm/mem_track/mem_trace.c  memory trace function
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

#include <linux/err.h>
#include <linux/mmzone.h>
#include <linux/printk.h>
#include <linux/freezer.h>
#include <linux/kernel.h>
#include <linux/ksm.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/oom.h>
#include "slab.h"
#include "mem_stack.h"
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif

#define pages_to_byte(pages) ((pages) << PAGE_SHIFT)

static size_t get_stats_cma(void);
static size_t get_stats_ion(void);
static size_t get_stats_slub(void);
static size_t get_stats_lslub(void);
static size_t get_stats_vmalloc(void);
static size_t get_stats_skb(void);
static size_t get_stats_zspage(void);
static size_t get_stats_buddy(void);
static size_t mm_get_slub_detail(void *buf, size_t len);


struct page_stack_trace {
	int (*page_stack_on)(char *);
	int (*page_stack_off)(char *);
	int (*page_stack_open)(int);
	int (*page_stack_close)(void);
	size_t (*page_stack_read)(struct mm_stack_info *, size_t, int);
};

struct mem_trace {
	int type;
	size_t (*get_mem_stats)(void);
	size_t (*get_mem_detail)(void *, size_t);
	struct page_stack_trace *stack_trace;
};

/* on->off->open->read->close */
struct page_stack_trace vmalloc_page_stack = {
	.page_stack_on = mm_vmalloc_track_on,
	.page_stack_off = mm_vmalloc_track_off,
	.page_stack_open = mm_vmalloc_stack_open,
	.page_stack_close = mm_vmalloc_stack_close,
	.page_stack_read = mm_vmalloc_stack_read,
};

struct page_stack_trace slub_page_stack = {
	.page_stack_on = mm_slub_track_on,
	.page_stack_off = mm_slub_track_off,
	.page_stack_open = mm_slub_stack_open,
	.page_stack_close = mm_slub_stack_close,
	.page_stack_read = mm_slub_stack_read,
};

struct page_stack_trace buddy_page_stack = {
	.page_stack_on = mm_buddy_track_on,
	.page_stack_off = mm_buddy_track_off,
	.page_stack_open = mm_buddy_stack_open,
	.page_stack_close = mm_buddy_stack_close,
	.page_stack_read = mm_buddy_stack_read,
};

struct mem_trace memtrace[] = {
	{ VMALLOC_TRACK,	   get_stats_vmalloc,
	 mm_get_vmalloc_detail, &vmalloc_page_stack },

	{ BUDDY_TRACK,		   get_stats_buddy,
	 NULL,			  &buddy_page_stack },

	{ SLUB_TRACK,		   get_stats_slub,
	 mm_get_slub_detail,	&slub_page_stack },

	{ LSLUB_TRACK,		   get_stats_lslub,
	 NULL,			  &buddy_page_stack }, /* the same with buddy */

	{ SKB_TRACK,		   get_stats_skb,
	 NULL,			  NULL },

	{ ZSPAGE_TRACK, 	   get_stats_zspage,
	 NULL,			  NULL },

	{ ION_TRACK,		   get_stats_ion,
	 mm_get_ion_detail,	NULL },

	{ CMA_TRACK,		   get_stats_cma,
	 NULL,			  NULL },
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

static size_t get_stats_cma(void)
{
	return pages_to_byte(totalcma_pages);
}

static size_t get_stats_ion(void)
{
	return mm_ion_total();
}

static size_t get_stats_slub(void)
{
	return pages_to_byte(global_node_page_state(NR_SLAB_UNRECLAIMABLE) +
		global_node_page_state(NR_SLAB_RECLAIMABLE));
}

static size_t get_stats_lslub(void)
{
	return pages_to_byte(global_zone_page_state(NR_LSLAB_PAGES));
}

static size_t get_stats_vmalloc(void)
{
	unsigned int i;
	size_t size = 0;

	for (i = 0; i < ARRAY_SIZE(vmalloc_type); i++)
		size += vm_type_detail_get(vmalloc_type[i]);

	return size;
}

static size_t get_stats_zspage(void)
{
	return pages_to_byte(global_zone_page_state(NR_ZSPAGES));
}

static size_t get_stats_skb(void)
{
	return pages_to_byte(global_zone_page_state(NR_SKB_PAGES));
}

static size_t get_stats_buddy(void)
{
	struct sysinfo i;

	si_meminfo(&i);
	return pages_to_byte(i.totalram - i.freeram);
}

#ifdef CONFIG_SLABINFO
static size_t mm_get_slub_detail(void *buf, size_t len)
{
	size_t cnt = 0;
	unsigned long size;
	struct kmem_cache *s = NULL;
	struct slabinfo sinfo;
	struct mm_slub_detail_info *info =
		(struct mm_slub_detail_info *)buf;

	mutex_lock(&slab_mutex);
	list_for_each_entry(s, &slab_caches, list) {
		if (cnt >= len) {
			mutex_unlock(&slab_mutex);
			return len;
		}
		memset(&sinfo, 0, sizeof(sinfo));
		get_slabinfo(s, &sinfo);
		size = (unsigned long)(unsigned int)s->size;
		(info + cnt)->active_objs = sinfo.active_objs;
		(info + cnt)->num_objs = sinfo.num_objs;
		(info + cnt)->objsize = (unsigned int)s->size;
		(info + cnt)->size = sinfo.num_objs * size;
		strncpy((info + cnt)->name, s->name, SLUB_NAME_LEN - 1);
		cnt++;
	}
	mutex_unlock(&slab_mutex);

	return cnt;
}
#else
static size_t mm_get_slub_detail(void *buf, size_t len)
{
	pr_info("[%s] couldn't get slabinfo\n", __func__);
	return 0;
}
#endif

size_t mm_get_ion_by_pid(pid_t pid)
{
	return mm_get_ion_size_by_pid(pid);
}

size_t mm_get_mem_total(int type)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type)
			return memtrace[i].get_mem_stats();

	return 0;
}

size_t mm_get_mem_detail(int type, void *buf, size_t len)
{
	unsigned int i;

	if (!buf)
		return 0;
	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].get_mem_detail)
			return memtrace[i].get_mem_detail(buf, len);

	return 0;
}

int mm_page_trace_on(int type, char *name)
{
	unsigned int i;

	if (!name)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].stack_trace)
			return memtrace[i].stack_trace->page_stack_on(name);
	return -EINVAL;
}

int mm_page_trace_off(int type, char *name)
{
	unsigned int i;

	if (!name)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].stack_trace)
			return memtrace[i].stack_trace->page_stack_off(name);

	return -EINVAL;
}

int mm_page_trace_open(int type, int subtype)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].stack_trace)
			return memtrace[i].stack_trace->page_stack_open(subtype);

	return -EINVAL;
}

int mm_page_trace_close(int type, int subtype)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].stack_trace)
			return memtrace[i].stack_trace->page_stack_close();

	return -EINVAL;
}

size_t mm_page_trace_read(int type,
				struct mm_stack_info *info,
				size_t len, int subtype)
{
	unsigned int i;

	if (!info)
		return 0;
	for (i = 0; i < ARRAY_SIZE(memtrace); i++)
		if (type == memtrace[i].type && memtrace[i].stack_trace)
			return memtrace[i].stack_trace->page_stack_read(
				info, len, subtype);

	return 0;
}

void mm_mem_stats_show(void)
{
	size_t size;
	int i;

	pr_err("========mem stat start==========\n");
	for (i = START_TRACK; i < NR_TRACK; i++) {
		size = mm_get_mem_total(i);
		pr_err("%s used: %ld kB\n", track_text[i], size / SZ_1K);
	}
	pr_err("=========mem stat end==========\n");
}

void mm_vmalloc_detail_show(void)
{
	unsigned int i;
	size_t size;

	pr_err("========get vmalloc info start==========\n");

	for (i = 0; i < ARRAY_SIZE(vmalloc_type); i++) {
		size = vm_type_detail_get(vmalloc_type[i]);
		pr_err("vmalloc type:%s, size:%zu kB\n",
			vmalloc_text[i], size / SZ_1K);
	}

	pr_err("========get vmalloc info end==========\n");
}

static void mm_dump_tasks(bool verbose)
{
	struct task_struct *p = NULL;
	struct task_struct *task = NULL;
	char task_state;
	char frozen_mark;

	pr_info("[ pid ]   total_vm  rss nptes  swap  anon  file  shmm s name\n");

	rcu_read_lock();
	for_each_process(p) {
		task = find_lock_task_mm(p);
		if (!task) {
			/*
			 * This is a kthread or all of p's threads have already
			 * detached their mm's.  There's no need to report them.
			 */
			continue;
		}
		task_state = task_state_to_char(task);
		frozen_mark = frozen(task) ? '*' : ' ';

		pr_info("[%5d] %8lu %6lu %5lu %5lu %5lu %5lu %c %s%c\n",
			task->pid, task->mm->total_vm, get_mm_rss(task->mm),
			get_mm_counter(task->mm, MM_SWAPENTS),
			get_mm_counter(task->mm, MM_ANONPAGES),
			get_mm_counter(task->mm, MM_FILEPAGES),
			get_mm_counter(task->mm, MM_SHMEMPAGES),
			task_state, task->comm, frozen_mark);
		task_unlock(task);
	}
	rcu_read_unlock();
}

void user_memory_dump(bool verbose)
{
	mm_dump_tasks(verbose);

#ifdef CONFIG_ION
	mm_ion_memory_info(verbose);
#endif

	if (verbose) {
		mm_mem_stats_show();
		mm_vmalloc_detail_show();
	}
	pr_info("user mm dump end, verbose-%d\n", verbose);
}

