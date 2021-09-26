/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: Implementation of the userspace process guideline. It reads
 *     the code segments of the target process and all linked libraries. The
 *     contents are hashed and collected as result_entry struct instances.
 *     Additionally the guideline reads the flags on all hashed memory
 *     segments and searches for differences between PTE flags of the same
 *     segment.
 * Create: 2017-12-20
 */

#include "dkm.h"

#include <linux/dcache.h>
#include <linux/kallsyms.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <securec.h>

#include "eima_utils.h"
#include "hash_generator.h"

/*
 * Parameter struct for the local result calculation functions.
 * Stores information necessary for result calculation. These include
 * pages, VM areas for the pages, number of pages and the main VM area
 * taken from the mm_struct of the process.
 */
struct calc_params {
	/* Task struct of the measurement target */
	struct task_struct *task;
	/* Primary VM area struct, taken from the process mm_struct */
	struct vm_area_struct *vm_a;
	/*
	 * Number of pinned userspace pages, describes length of pages
	 * and pages_vma.
	 */
	size_t num_pages;
};

static inline size_t eima_get_user_pages_remote(struct calc_params *params,
	unsigned long start, struct page **pages, int *locked)
{
	return (size_t)get_user_pages_remote(params->task, params->vm_a->vm_mm, start,
		1, FOLL_FORCE, pages, NULL, locked);
}

static inline void eima_up_read(struct rw_semaphore *sem, int locked)
{
	if (locked)
		up_read(sem);
}

/*
 * Determine if a set of bit flags has a specific flag set. Comparision is done
 * by bitwise AND operation.
 * input value:
 *     @flags: flags Set of bit flags
 *     @flag: flag Flag to test for
 * return value:
 *     true if flag is present, false otherwise
 */
static bool __has_flag(unsigned long flags, unsigned long flag)
{
	return (flags & flag) ? true : false;
}

/*
 * Test if pages for this memory segment have the access rights "xp" set.
 * This means that they must have execute access. Write and share access
 * must be prohibited.
 * input value:
 *     @vm_a: VM area for the memory segment in question
 * return value:
 *     true if @c "xp" are set, false otherwise
 */
static bool __is_xp_segment(struct vm_area_struct *vm_a)
{
	return __has_flag(vm_a->vm_flags, VM_EXEC) &&
		!__has_flag(vm_a->vm_flags, VM_WRITE) &&
		!__has_flag(vm_a->vm_flags, VM_MAYSHARE);
}

/*
 * Calculate the hash digest for the memory segment of a process, specified
 * by the parameters struct. The hash algorithm to use is specified by the
 * already initialized handle in @c hgh.
 * input value:
 *     @params: Parameters for this result calculation
 *     @hgh: Already initialized hash generator handle to use
 * return
 *     0 on success, negative on failure
 */
static int __do_hash(struct calc_params *params, struct hashgen_handle *hgh)
{
	int rv = 0;
	size_t i;

	for (i = 0; i < params->num_pages; ++i) {
		int locked = 1;
		size_t loc_rv;
		unsigned long addr;
		struct page *page = NULL;
		void *vaddr = NULL;

		/* Calculate the address we want to start to read from */
		addr = params->vm_a->vm_start + (i * PAGE_SIZE);

		/* Lock the mmap_sem to gain read access to memory */
		down_read(&params->vm_a->vm_mm->mmap_sem);

		/* Get a handle of the page we want to read */
		loc_rv = eima_get_user_pages_remote(params, addr, &page, &locked);
		if (loc_rv != 1) {
			eima_warning("Failed to get user page, loc_rv=%lu",
				loc_rv);

			/* In case the lock is still present, unlock */
			eima_up_read(&params->vm_a->vm_mm->mmap_sem, locked);
			/*
			 * Abort the operation, we could not get a handle
			 * on the page
			 */
			rv = -1;
			goto fn_exit;
		}

		/* Get a kernel virtual address for the page contents */
		vaddr = kmap(page);

		/* Add the data to the hash algorithm */
		hash_handle_update(hgh, vaddr, PAGE_SIZE);

		/* Cleanup page handle, address mapping and lock */
		kunmap(page);
		put_page(page);

		eima_up_read(&params->vm_a->vm_mm->mmap_sem, locked);
	}

fn_exit:
	return rv;
}

static int hash_measure(int measure_libs, const process_msg_t *tmsg,
			struct hashgen_handle **res_hash,
			struct task_struct *target_proc, int digest_size)
{
	struct hashgen_handle *result_e_hash = NULL;
	struct vm_area_struct *vma = NULL;
	int ret = 0;

	if (digest_size != EIMA_SHA384_DIGEST_SIZE)
		result_e_hash = hash_handle_init(HASH_ALG);
	else
		result_e_hash = hash_handle_init(HASH_ALG_384);

	if (result_e_hash == NULL)
		return -1;

	eima_debug("mm->start_code: %lu", tmsg->mm->start_code);
	eima_debug("mm->end_code: %lu", tmsg->mm->end_code);
	eima_debug("mm code length: %lu -> %#08lx",
		(tmsg->mm->end_code - tmsg->mm->start_code),
		(tmsg->mm->end_code - tmsg->mm->start_code));

	vma = tmsg->mm->mmap;
	do {
		if (__is_xp_segment(vma)) {
			size_t seg_len = vma->vm_end - vma->vm_start;
			struct calc_params params;

			params.task = target_proc;
			params.vm_a = vma;
			params.num_pages = (seg_len / PAGE_SIZE);

			if (seg_len % PAGE_SIZE != 0) {
				++(params.num_pages);
				eima_debug("Adjusted num_pages from %lu to %lu",
					params.num_pages - 1,
					params.num_pages);
			}

			if (measure_libs || tmsg->target_file == vma->vm_file)
				/* Calculate segment hash */
				ret = __do_hash(&params, result_e_hash);
		}

		vma = vma->vm_next;
	} while (vma && vma != tmsg->mm->mmap);

	hash_handle_final(result_e_hash);
	*res_hash = result_e_hash;

	return ret;
}

static int get_target_file_msg(process_msg_t *tmsg, char *process_pn,
		size_t pn_len)
{
	char *process_p = NULL;

	down_read(&tmsg->mm->mmap_sem);
	if (!(tmsg->mm->exe_file)) {
		up_read(&tmsg->mm->mmap_sem);
		return -1;
	}

	process_p = d_path(&tmsg->mm->exe_file->f_path, process_pn, pn_len);
	if (IS_ERR(process_p)) {
		up_read(&tmsg->mm->mmap_sem);
		return -1;
	}

	tmsg->target_file = tmsg->mm->exe_file;
	up_read(&tmsg->mm->mmap_sem);
	return 0;
}

int measure_process(struct task_struct *target_proc,
		int measure_libs, char *hash, int hash_len, int *hlen)
{
	process_msg_t target_msg = { NULL, NULL };
	struct hashgen_handle *result_e_hash = NULL;
	char *process_pn = NULL;
	int ret;

	if (target_proc == NULL) {
		eima_warning("Guideline has received no measurement request");
		return -1;
	}

	target_msg.mm = get_task_mm(target_proc);
	if (target_msg.mm == NULL) {
		eima_info("Could not get mm_struct for process with PID:%d and TGID:%d",
			target_proc->pid, target_proc->tgid);
		return -1;
	}

	process_pn = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!process_pn) {
		mmput(target_msg.mm);
		return -ENOMEM;
	}

	if (get_target_file_msg(&target_msg, process_pn, PATH_MAX) != 0) {
		kfree(process_pn);
		mmput(target_msg.mm);
		return -EINVAL;
	}

	ret = hash_measure(measure_libs, &target_msg, &result_e_hash,
		target_proc, hash_len);
	/* Cleanup resources requested by the guideline function */
	mmput(target_msg.mm);

	if (result_e_hash == NULL) {
		kfree(process_pn);
		return ret;
	}

	ret = memcpy_s(hash, hash_len, result_e_hash->hash->hash,
		result_e_hash->hash->len);
	if (ret != EOK) {
		kfree(process_pn);
		hash_handle_free(result_e_hash);
		eima_error("memcpy_s failed, s_ret = %d", ret);
		return ret;
	}
	*hlen = result_e_hash->hash->len;

	kfree(process_pn);
	hash_handle_free(result_e_hash);
	return ret;
}

