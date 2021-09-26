/*
 * mm/mem_track/mem_stack.h	   inter function interface declare
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

#ifndef __MEM_STACK_H
#define __MEM_STACK_H
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/mem_track/mem_trace.h>

static inline int mm_vmalloc_track_on(char *name)
{
	return 0;
}
static inline int mm_vmalloc_track_off(char *name)
{
	return 0;
}
int mm_vmalloc_stack_open(int type);
int mm_vmalloc_stack_close(void);
size_t mm_vmalloc_stack_read(struct mm_stack_info *buf,
	size_t len, int type);

int mm_slub_track_on(char *name);
int mm_slub_track_off(char *name);
static inline int mm_slub_stack_open(int type)
{
	return 0;
}
static inline int mm_slub_stack_close(void)
{
	return 0;
}
size_t mm_slub_stack_read(struct mm_stack_info *buf,
	size_t len, int type);

static inline int mm_buddy_track_on(char *name)
{
	return 0;
}
static inline int mm_buddy_track_off(char *name)
{
	return 0;
}
int mm_buddy_stack_open(int type);
int mm_buddy_stack_close(void);
size_t mm_buddy_stack_read(struct mm_stack_info *buf,
	size_t len, int type);

size_t vm_type_detail_get(int subtype);
size_t mm_get_vmalloc_detail(void *buf, size_t len);
unsigned long get_buddy_caller(unsigned long pfn);
extern unsigned long mm_ion_total(void);
extern size_t mm_get_ion_size_by_pid(pid_t pid);
extern size_t mm_get_ion_detail(void *buf, size_t len);
extern int mm_ion_memory_info(bool verbose);
#endif

