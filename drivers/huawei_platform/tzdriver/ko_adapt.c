/*
 * ko_adapt.c
 *
 * function for find symbols not exported
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "ko_adapt.h"
#include <linux/types.h>
#include <linux/mm_types.h>
#include <linux/stddef.h>
#include <linux/cred.h>
#include <linux/version.h>
#if (KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE)
#include <linux/sched/task.h>
#endif
#include <linux/cpumask.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include "tc_ns_log.h"

typedef const struct cred *(get_task_cred_func)(struct task_struct *);
typedef void (kthread_bind_mask_func)(struct task_struct *, const struct cpumask *);
typedef long (sys_chown_func)(const char __user *filename,
		uid_t user, gid_t group);
typedef ssize_t (vfs_write_func)(struct file *file, const char __user *buf,
		size_t count, loff_t *pos);

typedef struct page *(alloc_pages_func)(gfp_t gfp_mask, unsigned int order);

const struct cred *koadpt_get_task_cred(struct task_struct *task)
{
	static get_task_cred_func *get_task_cred_pt = NULL;

	if (!task)
		return NULL;

	if (!get_task_cred_pt) {
		get_task_cred_pt = (get_task_cred_func *)
			(uintptr_t)kallsyms_lookup_name("get_task_cred");
		if (IS_ERR_OR_NULL(get_task_cred_pt)) {
			tloge("fail to find symbol get task cred\n");
			return NULL;
		}
	}
	return get_task_cred_pt(task);
}

void koadpt_kthread_bind_mask(struct task_struct *task,
	const struct cpumask *mask)
{
	static kthread_bind_mask_func *kthread_bind_mask_pt = NULL;

	if (!task || !mask)
		return;

	if (!kthread_bind_mask_pt) {
		kthread_bind_mask_pt = (kthread_bind_mask_func *)
			(uintptr_t)kallsyms_lookup_name("kthread_bind_mask");
		if (IS_ERR_OR_NULL(kthread_bind_mask_pt)) {
			tloge("fail to find symbol kthread bind mask\n");
			return;
		}
	}
	kthread_bind_mask_pt(task, mask);
}

long koadpt_sys_chown(const char __user *filename, uid_t user, gid_t group)
{
	static sys_chown_func *sys_chown_pt = NULL;

	if (!filename)
		return -EINVAL;

	if (!sys_chown_pt) {
		sys_chown_pt = (sys_chown_func *)
			(uintptr_t)kallsyms_lookup_name("sys_chown");
		if (IS_ERR_OR_NULL(sys_chown_pt)) {
			tloge("fail to find symbol kthread bind mask\n");
			return -EINVAL;
		}
	}
	return sys_chown_pt(filename, user, group);
}

ssize_t koadpt_vfs_write(struct file *file, const char __user *buf,
	size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 19, 116))
	static vfs_write_func *vfs_write_pt = NULL;

	if (!file || !buf || !pos)
		return -EINVAL;

	if (!vfs_write_pt) {
		vfs_write_pt = (vfs_write_func *)
			(uintptr_t)kallsyms_lookup_name("vfs_write");
		if (IS_ERR_OR_NULL(vfs_write_pt)) {
			tloge("fail to find symbol vfs write\n");
			return -EINVAL;
		}
	}
	return vfs_write_pt(file, buf, count, pos);
#else
	return vfs_write(file, buf, count, pos);
#endif
}

struct page *koadpt_alloc_pages(gfp_t gfp_mask, unsigned int order)
{
#ifdef CONFIG_NUMA
	static alloc_pages_func *alloc_pages_pt = NULL;

	if (!alloc_pages_pt) {
		alloc_pages_pt = (alloc_pages_func *)
			(uintptr_t)kallsyms_lookup_name("alloc_pages_current");
		if (IS_ERR_OR_NULL(alloc_pages_pt)) {
			tloge("fail to find symbol alloc pages current\n");
			return NULL;
		}
	}
	return alloc_pages_pt(gfp_mask, order);
#else
	return alloc_pages(gfp_mask, order);
#endif
}
