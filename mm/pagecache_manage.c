/*
 * Copyright (c) 2017-2018 Huawei Technologies Co., Ltd.
 *
 * This source code is released for free distribution under the terms of the
 * GNU General Public License
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author:       Huawei Storage DEV
 * File Name:    pagecache_manage.c
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/namei.h>
#include <linux/kernel.h>
#include <linux/backing-dev.h>
#include <linux/writeback.h>
#include <linux/oom.h>
#include <linux/pid.h>
#include <linux/path.h>
#include <linux/mount.h>
#include <linux/statfs.h>
#include <linux/swap.h>
#include <linux/mmzone.h>
#include <linux/workqueue.h>
#include <linux/pagecache_manage.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/err.h>

#ifdef CONFIG_FILE_MAP
#include <linux/file_map.h>
#endif

#ifdef CONFIG_MAS_BOOTDEVICE
#include <linux/bootdevice.h>
#endif

#include "internal.h"
#include "pagecache_manage_interface.h"

/* stub for compile */
#define SUBSCRIBER_NAME_LEN 32

#ifndef CONFIG_MAS_BOOTDEVICE
enum bootdevice_type { BOOT_DEVICE_EMMC = 0, BOOT_DEVICE_UFS = 1, BOOT_DEVICE_MAX };

enum bootdevice_type get_bootdevice_type(void)
{
	return BOOT_DEVICE_UFS;
}
#endif

/* stub for compile */

typedef enum tag_pch_log_level {
	LOG_LEVEL_ERROR = 0,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_INFO,
	LOG_LEVEL__DEBUG,
	LOG_LEVEL_TOTAL = LOG_LEVEL_INFO,
} pch_log_level;

pch_log_level g_pch_print_level = LOG_LEVEL_ERROR;

typedef enum tag_pch_fs_used {
	FS_TYPE_EXT4 = 0,
	FS_TYPE_F2FS,
	FS_TYPE_TOTAL = FS_TYPE_EXT4,
} pch_fs_type_used;

#define pch_print(level, fmt, ...) do {\
	if (level <= g_pch_print_level) { \
		pr_err(fmt, ##__VA_ARGS__); \
	} \
} while (0)

/* default value set */
static unsigned int page_cache_func_enable = 1;
static unsigned int emmc_ra_ahead_size = 8;
static unsigned int emmc_ra_round_size = 8;
static unsigned int ufs_ra_ahead_size = 16;
static unsigned int ufs_ra_round_size = 16;
static unsigned int extend_ra_size = 16;
static unsigned int lowmem_ra_size = 8;
static unsigned int lowstorage_ratio = 1;
static unsigned int lowstorage_ra_size = 4;
static unsigned int lowstorage_extend_size = 16;
static unsigned int lowmem_reserved;	/* reserved memory setted to per zone */

struct pch_info_desc {
	/* for huawei block busy/idle func */
	struct blk_busyidle_event_node event_node;
	struct block_device	*bdev;
	struct timer_list	timer;
	/* Work Queues */
	struct work_struct	timeout_work;
	struct semaphore	work_sem;

	unsigned long timer_work_state;
	unsigned int func_enable;
	unsigned int boot_device;
	unsigned int readaround_ra_size;
	unsigned int readahead_ra_size;
	unsigned int readextend_ra_size;
	unsigned int lowmem_ra_size;
	unsigned int lowstorage_ratio;
	unsigned int fs_mount_stat;
};


static struct pch_info_desc *pch_info;

#define ARRAY_LOWMEM_ADJ_SIZE 4
#define BIT_MAP_COUNT  2
#define BAVAIL_RATION 100
static int lowmem_adj_size = 4;
static unsigned long lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;

extern int read_pages(struct address_space *mapping, struct file *filp,
		struct list_head *pages, unsigned int nr_pages, gfp_t gfp);

static inline int is_userdata_mounted(void)
{
	return (pch_info->fs_mount_stat & USRDATA_IS_MOUNTED) ? 1 : 0;
}

static inline int is_userdata_umounted(void)
{
	return (pch_info->fs_mount_stat & USRDATA_IS_UMOUNTED) ? 1 : 0;
}

static inline int is_userdata_umounting(void)
{
	return (pch_info->fs_mount_stat & USRDATA_IS_UMOUNTING) ? 1 : 0;
}

static inline void clear_userdata_mounted(void)
{
	pch_info->fs_mount_stat &= ~USRDATA_IS_MOUNTED;
}

static inline void clear_userdata_umounted(void)
{
	pch_info->fs_mount_stat &= ~USRDATA_IS_UMOUNTED;
}

static inline void clear_userdata_umounting(void)
{
	pch_info->fs_mount_stat &= ~USRDATA_IS_UMOUNTING;
}

static inline void set_userdata_mounted(void)
{
	pch_info->fs_mount_stat |= USRDATA_IS_MOUNTED;
}

static inline void set_userdata_umounted(void)
{
	pch_info->fs_mount_stat |= USRDATA_IS_UMOUNTED;
}

static inline void set_userdata_umounting(void)
{
	pch_info->fs_mount_stat |= USRDATA_IS_UMOUNTING;
}

#ifdef CONFIG_MAS_DEBUG_FS
static struct ctl_table pch_table[] = {
	{
	  .procname	= "enabled",
	  .data		= &page_cache_func_enable,
	  .maxlen	= sizeof(page_cache_func_enable),
	  .mode		= 0400,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "emmc_readahead_ra_size",
	  .data		= &emmc_ra_ahead_size,
	  .maxlen	= sizeof(emmc_ra_ahead_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "emmc_readaound_ra_size",
	  .data		= &emmc_ra_round_size,
	  .maxlen	= sizeof(emmc_ra_round_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "ufs_readahead_ra_size",
	  .data		= &ufs_ra_ahead_size,
	  .maxlen	= sizeof(ufs_ra_ahead_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "ufs_readaound_ra_size",
	  .data		= &ufs_ra_round_size,
	  .maxlen	= sizeof(ufs_ra_round_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "readextend_ra_size",
	  .data		= &extend_ra_size,
	  .maxlen	= sizeof(extend_ra_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "lowmem_ra_size",
	  .data		= &lowmem_ra_size,
	  .maxlen	= sizeof(lowmem_ra_size),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "lowstorage_ratio",
	  .data		= &lowstorage_ratio,
	  .maxlen	= sizeof(lowstorage_ratio),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{
	  .procname	= "lowmem_reserved",
	  .data		= &lowmem_reserved,
	  .maxlen	= sizeof(lowmem_reserved),
	  .mode		= 0640,
	  .proc_handler	= proc_dointvec,
	},
	{ }
};

static struct ctl_table pch_root_table[] = {
	{ .procname	= "page_cache",
	  .mode		= 0555,
	  .child	= pch_table },
	{ }
};

static struct ctl_table_header *pch_table_header;
#endif

/*
 * Function name: pch_thread_task_state
 * Discription: check wether thread foreground
 * Parameters:
 *      @ NULL
 * return value:
 *      @ 0: foreground state
 *      @ other: background state
 */

/*
 * Function name: pch_lowmem_check
 * Discription: check wether lowmem state
 * Parameters:
 *      @ NULL
 * return value:
 *      @ 2: mem high_wmark state
 *      @ 1: mem low_wmark state
 *      @ 0: normal state
 */
int pch_lowmem_check(void)
{
	unsigned long watermark;
	struct zone *zone = NULL;

	/*
	 * Only if all zones' free pages below watermark, low memory
	 * status will be return.
	 */
	for_each_zone(zone) {
		watermark = high_wmark_pages(zone) +
				(2UL << (unsigned int)lowmem_reserved);
		if (zone_watermark_ok(zone, 0, watermark, 0, 0))
			return MEM_NORMAL_WMARK;
	}

	for_each_zone(zone) {
		watermark = low_wmark_pages(zone);
		if (zone_watermark_ok(zone, 0, watermark, 0, 0))
			return MEM_HIGH_WMARK;
	}

	return MEM_LOW_WMARK;
}


/*
 * Function name: pch_lowmem_check2
 * Discription: check wether lowmem state for io using purpose
 * Parameters:
 *      @ NULL
 * return value:
 *      @ 2: mem high_wmark state
 *      @ 1: mem low_wmark state
 *      @ 0: normal state
 */
int pch_lowmem_check2(void)
{
	int array_size = ARRAY_LOWMEM_ADJ_SIZE;
	int i;
	unsigned long minfree = 0;

	unsigned long other_file = global_node_page_state(NR_FILE_PAGES) -
				global_node_page_state(NR_SHMEM) -
				total_swapcache_pages();
	unsigned long other_free = global_zone_page_state(NR_FREE_PAGES) -
				global_zone_page_state(NR_FREE_CMA_PAGES) -
				totalreserve_pages;

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree)
			return 1;
	}

	return 0;
}

#ifdef CONFIG_MAS_DEBUG_FS

enum GET_PAGE_SIMULATE_TYPE_E {
	GET_PAGE_NO_SIMULATE = 0, /* The value can't be changed! */
	GET_PAGE_SIMULATE_ALWAYS_FAIL,
	GET_PAGE_SIMULATE_ALWAYS_SUCC,
};

static enum GET_PAGE_SIMULATE_TYPE_E get_page_simulate_type
		= GET_PAGE_NO_SIMULATE;
int pch_get_page_sim_type_set(int type)
{
	get_page_simulate_type = type;
	return get_page_simulate_type;
}

#endif

/**
 * pch_get_page - find and get a page reference
 * @mapping: the address_space to search
 * @offset: the page index
 * @fgp_flags: PCG flags
 * @gfp_mask: gfp mask to use for the page cache data page allocation
 *
 */
struct page *pch_get_page(struct address_space *mapping, pgoff_t offset,
	int fgp_flags, gfp_t gfp_mask)
{
#ifdef CONFIG_MAS_DEBUG_FS
	struct page *page = NULL;
	int pcg_flag;

	if (likely(get_page_simulate_type == GET_PAGE_NO_SIMULATE))
		return pagecache_get_page(mapping, offset, fgp_flags, gfp_mask);
	switch (get_page_simulate_type) {
	case GET_PAGE_SIMULATE_ALWAYS_FAIL:
		return NULL;
	case GET_PAGE_SIMULATE_ALWAYS_SUCC:
		pcg_flag = fgp_flags | ((int)(FGP_CREAT | FGP_LOCK));
		if (!(fgp_flags & FGP_LOCK)) {
			page = pagecache_get_page(mapping, offset,
				pcg_flag, gfp_mask);
			if (page) {
				unlock_page(page);
				put_page(page);
			}
			return page;
		}
		pcg_flag = fgp_flags | ((int)(FGP_CREAT));
		return pagecache_get_page(mapping, offset,
				pcg_flag, gfp_mask);
	default:
		break;
	}
#endif
	return pagecache_get_page(mapping, offset, fgp_flags, gfp_mask);
}


/*
 * Function name: pch_shrink_read_pages
 * Discription: shrink generic read page size
 * Parameters:
 *      @ unsigned long nr
 * return value:
 *      @ ra_pages: the page size need to read
 */
unsigned long pch_shrink_read_pages(unsigned long nr)
{
	unsigned long ra_pages;

	if (!pch_info || !pch_info->func_enable)
		return min(nr, MAX_RA_PAGES);

	if (pch_info->readahead_ra_size < nr)
		ra_pages = pch_info->readaround_ra_size;
	else
		ra_pages = nr;

	if ((pch_lowmem_check() >= MEM_HIGH_WMARK) &&
			(pch_info->lowmem_ra_size < ra_pages))
		ra_pages = pch_info->lowmem_ra_size;

	return min(ra_pages, MAX_RA_PAGES);
}

/*
 * Function name: pch_shrink_mmap_pages
 * Discription: shrink mmap read page size
 * Parameters:
 *      @ unsigned long nr
 * return value:
 *      @ ra_pages: the page size need to read
 */
unsigned long pch_shrink_mmap_pages(unsigned long nr)
{
	unsigned long ra_pages;

	if (!pch_info || !pch_info->func_enable)
		return min(nr, MAX_RA_PAGES);

	if (pch_info->readaround_ra_size < nr)
		ra_pages = pch_info->readaround_ra_size;
	else
		ra_pages = nr;

	if ((pch_lowmem_check() >= MEM_HIGH_WMARK) &&
			(pch_info->lowmem_ra_size < ra_pages))
		ra_pages = pch_info->lowmem_ra_size;

	return min(ra_pages, MAX_RA_PAGES);
}

/*
 * Function name: pch_shrink_mmap_async_pages
 * Discription: shrink mmap async read page size
 * Parameters:
 *      @ unsigned long nr
 * return value:
 *      @ ra_pages: the page size need to read
 */
unsigned long pch_shrink_mmap_async_pages(unsigned long nr)
{
	unsigned long ra_pages;

	if (!pch_info || !pch_info->func_enable)
		return min(nr, MAX_RA_PAGES);

	if (pch_lowmem_check())
		ra_pages = 0;
	else
		ra_pages = nr;

	return ra_pages;
}

/*
 * Function name: pch_get_index_vip_first
 * Discription: get index by vip first
 * Parameters:
 *      @ int i: count
 *      @ int min, max: range is [min, max], closed interval
 *      @ int vip: Mr Alpha
 * return value:
 *      @ int: >= 0 for a index
 *             other means no index at this time
 */
static inline int pch_get_index_vip_first(int i, int min, int max, int vip)
{
	int ret;

	ret = vip + ((i % 2) ? (-(i + 1) / 2) : ((i + 1) / 2));

	if (ret < min)
		return -1;

	if (ret > max)
		return -1;

	return ret;
}

static inline void pch_reorder_io_list(unsigned long long *pg_idx_bm,
	int bm_count, struct list_head *page_tmp, struct list_head *page_pool,
	pgoff_t offset)
{
	unsigned int i;
	struct page *page = NULL;

	for (i = 0; i < (BITS_PER_LONG_LONG << 1); i++) {
		if ((i >> 6) > bm_count)
			return;

		if (pg_idx_bm[i >> 6] & (1ULL << (unsigned int)(i % 64))) {
			page = list_first_entry(page_tmp,
				struct page, lru);
			list_del(&page->lru);
			page->index = i + offset;
			list_add(&page->lru, page_pool);
			if (list_empty(page_tmp))
				return;
		}
	}
}

/*
 * pch_do_page_cache_readaround actually reads a chunk of disk.
 * It allocates all the pages first, then submits them all for I/O.
 * This avoids the very bad behaviour which would occur
 * if page allocations are causing VM writeback.
 * We really don't want to intermingle reads and writes like that.
 *
 * vip_n for very important page No.
 *
 * Returns the number of pages requested, or the maximum amount of I/O allowed.
 */
static int pch_do_page_cache_readaround(struct address_space *mapping,
		struct file *filp, pgoff_t offset, unsigned long nr_to_read,
		unsigned long lookahead_size, unsigned long vip_n)
{
	struct inode *inode = mapping->host;
	struct page *page = NULL;
	unsigned long end_index;	/* The last page we want to read */
	LIST_HEAD(page_tmp);
	LIST_HEAD(page_pool);
	int page_idx;
	unsigned long i;
	pgoff_t pgofs;
	int ret = 0;
	loff_t isize = i_size_read(inode);
	gfp_t gfp_mask = readahead_gfp_mask(mapping);
	unsigned long long pg_idx_bm[BIT_MAP_COUNT] = {0};
#ifdef CONFIG_FILE_MAP
	bool badpted = false;
	unsigned long old_nr_to_read = nr_to_read;
#endif
	/*
	 * If the number of read is greater than size of pg_idx_bm,
	 * the optimise of vip should be disabled.
	 */
	if (unlikely(nr_to_read >= (BITS_PER_LONG_LONG << 1)))
		vip_n = 0;

	if (unlikely(isize == 0))
		goto out;

	pgcache_log_path(BIT_DO_PAGECACHE_READAHEAD_DUMP, &(filp->f_path),
			"%s, offset, %ld, nr_to_read, %ld, lookahead_size, %ld",
			__func__, offset, nr_to_read, lookahead_size);

	end_index = ((isize - 1) >> PAGE_SHIFT);

#ifdef CONFIG_FILE_MAP
	badpted = file_map_ra_adapt();
	if (badpted)
		nr_to_read = file_map_data_analysis(inode, offset,
					nr_to_read, end_index, true);
#endif

	/*
	 * Actually read count is dependent on the cache missed count
	 * and available page number.
	 * The sequence of page allocation is from center to two sides
	 */
	for (i = 0; i < (nr_to_read << 1); i++) {
		page_idx = pch_get_index_vip_first(i, 0,
			min_t(int, nr_to_read - 1, end_index - offset), vip_n);
		if (page_idx < 0)
			continue;

		pgofs = offset + page_idx;

#ifdef CONFIG_FILE_MAP
		if (badpted && (old_nr_to_read == nr_to_read) &&
		!file_map_is_set(inode, pgofs)){
			file_map_stat_ignore_inc(1);
			continue;
		}
#endif
		rcu_read_lock();

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0))
		page = radix_tree_lookup(&mapping->page_tree, pgofs);
#else
		page = radix_tree_lookup(&mapping->i_pages, pgofs);
#endif

		rcu_read_unlock();
		if (page && !radix_tree_exceptional_entry(page))
			continue;

		page = __page_cache_alloc(gfp_mask);
		if (unlikely(!page))
			break;

		if (likely(vip_n)) {
			pg_idx_bm[page_idx >> 6] |=
				1ULL << (unsigned int)(page_idx % 64);
			list_add(&page->lru, &page_tmp);
		} else {
			page->index = pgofs;
			list_add(&page->lru, &page_pool);
		}

		if (page_idx == nr_to_read - lookahead_size)
			SetPageReadahead(page);
		ret++;
	}

#ifdef CONFIG_FILE_MAP
	if (badpted)
		file_map_stat_total_inc((long)ret);
#endif

	/*
	 * Now start the IO.  We ignore I/O errors - if the page is not
	 * uptodate then the caller will launch readpage again, and
	 * will then handle the error.
	 */
	if (likely(ret)) {
		/*
		 * Before we start the IO, we should reorder the io list
		 * by the file offset
		 */
		if (likely(vip_n))
			pch_reorder_io_list(pg_idx_bm, BIT_MAP_COUNT,
				&page_tmp, &page_pool, offset);
		read_pages(mapping, filp, &page_pool, ret, gfp_mask);
	}
#ifdef CONFIG_MAS_DEBUG_FS
	BUG_ON(!list_empty(&page_pool));
#endif
out:
	return ret;
}

void pch_read_around(struct file_ra_state *ra,
				   struct address_space *mapping,
				   struct file *file,
				   pgoff_t offset)
{
	pgoff_t start;
	/* The unit of size is 4K page */
	unsigned long size;
	unsigned long async_size;

	/* shrink pre-fetch size when low memory */
	size = pch_shrink_mmap_pages(ra->ra_pages);

	/* calculate start address */
	start = (pgoff_t)max_t(long, 0, offset - (size >> 1));

	async_size = size >> 2;

	/* use temporary variable to avoid conflicts */
	pch_do_page_cache_readaround(mapping, file, start, size,
			async_size, offset - start);

	ra->start = start;
	ra->size = size;
	pgcache_log_path(BIT_MMAP_SYNC_READ_DUMP, &(file->f_path),
			"mmap sync readaround, pg_offset, %ld, pages, %ld",
			start, size);
}

static void __pch_mmap_readextend(struct vm_area_struct *vma,
				    struct file_ra_state *ra,
				    struct file *file,
				    pgoff_t offset,
				    int forward)
{
	struct address_space *mapping = file->f_mapping;
	struct inode *inode = mapping->host;
	loff_t isize = i_size_read(inode);
	pgoff_t end_index, pch_index;
	unsigned long req_size;
	unsigned long async_size;

	if (isize <= 0)
		return;
	end_index = ((unsigned int)(isize - 1) >> PAGE_SHIFT);
	/* If we don't want any read-ahead, don't bother */
	if (vma->vm_flags & VM_RAND_READ)
		return;
	if (!ra->ra_pages)
		return;
	if (vma->vm_flags & VM_SEQ_READ)
		return;

	/*
	 * Do we miss much more than hit in this file? If so,
	 * stop bothering with read-ahead. It will only hurt.
	 */
	if (ra->mmap_miss > MAX_MMAP_LOTSAMISS)
		return;

	/*
	 * Defer asynchronous read-ahead on IO congestion.
	 */
	if (inode_read_congested(mapping->host))
		return;
	/*
	 * As the extend readahead is after the sync readahead,
	 * we can't disable extend RA by check the PG_readahead.
	 * The page with PG_readahead has been checked or submitted
	 * by sync readahead just now.
	 * It should be exist, so we skip the readahead page check.
	 *
	 * When this function was called, the page which cause the
	 * filemap fault wasn't in cache, so it's unnecessary to move
	 * the readahead window according to the offset.
	 * We can set extend readahead window just as sync readahead.
	 */

	if (!forward) {
		/*
		 * Check if the sync read window include the file start pos
		 * If yes, unnecessary do extend readahead.
		 */
		pch_index = (pgoff_t)max_t(long, 0, offset - ra->size / 2);
		if (pch_index == 0)
			return;

		/*
		 * Check if the extend async read window
		 * include the file start pos or not
		 */
		pch_index = (pgoff_t)max_t(
			long, 0, offset - pch_info->readextend_ra_size);
		/*
		 * If yes, we read all from start of the file
		 * If no, we set extend async read window as expected
		 */
		if (pch_index == 0)
			req_size = offset;
		else
			req_size = pch_info->readextend_ra_size - ra->size / 2;
		async_size = req_size >> 2;

		pgcache_log_path(BIT_MMAP_SYNC_READ_DUMP, &(file->f_path),
				"Back Readahead readextend, pch_index, %ld",
				pch_index);
	} else {
		/*
		 * Check if the sync read window include the file end pos or not
		 * If yes, unnecessary do extend readahead.
		 */
		pch_index =
			(pgoff_t)min_t(long, end_index, offset + ra->size / 2);
		if (pch_index == end_index)
			return;
		/*
		 * We don't check the file end condition, because
		 * __do_page_cache_readahead will help us do it!
		 * We can just set window and read it directly.
		 */
		pch_index = offset + ra->size / 2;

		req_size = pch_info->readaround_ra_size;
		async_size = req_size >> 2;
	}
	__do_page_cache_readahead(mapping, file, pch_index,
					req_size, async_size);
}

/*
 * Function name: pch_mmap_readextend
 * Discription: mmap extend read flow
 * Parameters:
 *      @ struct vm_area_struct
 *      @ struct file_ra_state
 *      @ struct file
 *      @ pgoff_t offset
 * return value:
 *      @ NULL
 */
void pch_mmap_readextend(struct vm_area_struct *vma,
				    struct file_ra_state *ra,
				    struct file *file,
				    pgoff_t offset)
{

	if (!pch_info || !pch_info->func_enable)
		return;

	/*
	 * if lowmem status, no extend read
	 */
	if (pch_lowmem_check())
		return;

	__pch_mmap_readextend(vma, ra, file, offset, 1);
}

/**
 * timer_work_in_progress - determine whether there is timer work in progress
 *
 * Determine whether there is work waiting to be handled.
 */
static inline bool timer_work_in_progress(void)
{
	return test_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);
}

/*
 * Function name: pch_user_statfs
 * Discription: get user volume size
 * Parameters:
 *      @ NULL
 * return value:
 *      @ 1: user volume size is low
 *      @ 0: user volume size is high
 *      @ other: error failed
 */
static int pch_user_statfs(void)
{
	char *data = "/data";
	struct kstatfs st;
	struct path path;
	int error;
	unsigned int lookup_flags = LOOKUP_FOLLOW | LOOKUP_AUTOMOUNT;
	unsigned int ratio, ret;

	memset(&st, 0, sizeof(st));/* unsafe_function_ignore: memset */

	error = kern_path(data, lookup_flags, &path);
	if (!error) {
		error = vfs_statfs(&path, &st);
		path_put(&path);
	}

	if  (error)
		return error;

	ratio = (int) ((st.f_bavail * (u64)BAVAIL_RATION) / st.f_blocks);
	if  ((ratio <= pch_info->lowstorage_ratio) &&
			(st.f_bavail <= STORAGE_IS_1G))
		ret = USER_SPACE_IS_LOW;
	else
		ret = USER_SPACE_IS_HIGH;

	return ret;
}

void pch_timeout_work(struct work_struct *work)
{
	struct super_block *sb = NULL;
	struct backing_dev_info *bdi = NULL;
	struct bdi_writeback *wb = NULL;
	unsigned long background_thresh;
	unsigned long dirty_thresh;
	unsigned long nr_dirty;
	int userdata_state;

	down(&pch_info->work_sem);
	set_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);

	if (!is_userdata_mounted()) {
		clear_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);
		up(&pch_info->work_sem);
		return;
	}

	sb = pch_info->bdev->bd_super;
	bdi = sb->s_bdi;
	wb = &bdi->wb;

	userdata_state = pch_user_statfs();
	if (userdata_state == USER_SPACE_IS_LOW) {
		pch_info->readaround_ra_size = lowstorage_ra_size;
		pch_info->readahead_ra_size = lowstorage_ra_size;
		pch_info->readextend_ra_size = lowstorage_extend_size;
	} else if (userdata_state == USER_SPACE_IS_HIGH) {
		pch_info->readextend_ra_size = extend_ra_size;
		if (pch_info->boot_device == BOOT_DEVICE_EMMC) {
			pch_info->readahead_ra_size = emmc_ra_ahead_size;
			pch_info->readaround_ra_size = emmc_ra_round_size;
		} else if (pch_info->boot_device  == BOOT_DEVICE_UFS) {
			pch_info->readahead_ra_size = ufs_ra_ahead_size;
			pch_info->readaround_ra_size = ufs_ra_round_size;
		}
	} else {
		pch_print(LOG_LEVEL_ERROR, "%s: err userdata_state = %d.\n", __func__, userdata_state);
	}

	global_dirty_limits(&background_thresh, &dirty_thresh);

	nr_dirty = global_node_page_state(NR_FILE_DIRTY) +
				global_node_page_state(NR_UNSTABLE_NFS) +
				global_node_page_state(NR_WRITEBACK);

	if (nr_dirty < dirty_thresh) {
		clear_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);
		up(&pch_info->work_sem);
		return;
	}

	if (!wb) {
		clear_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);
		up(&pch_info->work_sem);
		return;
	}

	if (unlikely(!writeback_in_progress(wb)))
		wb_start_background_writeback(wb);

	clear_bit(TIMER_WORK_IS_RUNING, &pch_info->timer_work_state);
	up(&pch_info->work_sem);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0))
void pch_timeout_timer(unsigned long data)
{
	if (!timer_work_in_progress())
		schedule_work(&pch_info->timeout_work);
}
#else
void pch_timeout_timer(struct timer_list *data)
{
	if (!timer_work_in_progress())
		schedule_work(&pch_info->timeout_work);
}
#endif

/*
 * Function name: pch_io_idle_notify_handler
 * Discription: blk busy/idle event notify handler
 * Parameters:
 *      @ struct blk_busyidle_nb
 *      @ enum blk_idle_notify_state
 * return value:
 *      @ BUSYIDLE_NO_IO
 *      @ BUSYIDLE_IO_TRIGGERED
 *      @ other: failed
 */
static enum blk_busyidle_callback_ret pch_io_idle_notify_handler(
		struct blk_busyidle_event_node *nb, enum blk_idle_notify_state state)
{
	if (state == BLK_IDLE_NOTIFY)
		mod_timer(&pch_info->timer, jiffies + msecs_to_jiffies(5000));
	else if (state == BLK_BUSY_NOTIFY)
		del_timer(&pch_info->timer);

	return BUSYIDLE_NO_IO;
}

/*
 * Function name: mount_fs_register_pch
 * Discription: register blk busy/idle event
 *		    when fs is mounted
 * Parameters:
 *      @ struct path
 * return value:
 *      @ NULL
 */
void mount_fs_register_pch(struct vfsmount *mnt)
{
	struct super_block *sb = mnt->mnt_sb;
	int ret = 0;

	if (!pch_info) {
		pch_print(LOG_LEVEL_WARNING, "%s: lack of pch_info, return.\n",
			__func__);
		return;
	}

	if (sb->s_magic == F2FS_SUPER_MAGIC) {
		if (is_userdata_umounted()) {
			clear_userdata_umounted();
			set_userdata_mounted();
		} else {
			pch_print(LOG_LEVEL_INFO,
				"%s: data remount, return.\n", __func__);
			return;
		}
	} else {
		pch_print(LOG_LEVEL_INFO,
				"%s: other partitions, return.\n", __func__);
		return;
	}

	if (is_userdata_mounted()) {
		if (pch_info->bdev)
			return;

		if (!sb->s_bdev)
			return;

		pch_info->bdev = sb->s_bdev;
		pch_info->event_node.busyidle_callback = pch_io_idle_notify_handler;
		if (sizeof(pch_info->event_node.subscriber) <= strlen(PCH_DEV_NAME)) {
			pch_print(LOG_LEVEL_ERROR,
				"%s: subscriber length is too small\n", __func__);
			return;
		}

		strncpy(pch_info->event_node.subscriber, PCH_DEV_NAME, strlen(PCH_DEV_NAME));
		ret = blk_busyidle_event_subscribe(
				pch_info->bdev, &pch_info->event_node);
		if (ret)
			pch_print(LOG_LEVEL_ERROR, "%s: blk busy/idle init failed.\n", __func__);
	}
}

/*
 * Function name: umounting_fs_register_pch
 * Discription: unregister blk busy/idle event
 *		    when fs unmount
 * Parameters:
 *      @ struct super_block
 * return value:
 *      @ NULL
 */
void umounting_fs_register_pch(struct super_block *sb)
{
	int ret = 0;

	if (!pch_info) {
		pch_print(LOG_LEVEL_WARNING,
			"%s: lack of pch_info, return.\n",
			__func__);
		return;
	}

	if (sb->s_magic == F2FS_SUPER_MAGIC) {
		if (is_userdata_mounted()) {
			clear_userdata_mounted();
			set_userdata_umounting();

			if (pch_info->bdev) {
				del_timer_sync(&pch_info->timer);
				flush_work(&pch_info->timeout_work);
				down(&pch_info->work_sem);
				ret = blk_busyidle_event_unsubscribe(&pch_info->event_node);
				if (ret) {
					pch_print(LOG_LEVEL_ERROR, "%s: blk busy/idle clear failed.\n", __func__);
					up(&pch_info->work_sem);
					return;
				}
				up(&pch_info->work_sem);
			}
		} else {
			pch_print(LOG_LEVEL_INFO,
				"%s: data re-umount, return.\n", __func__);
			return;
		}
	} else {
		pch_print(LOG_LEVEL_INFO,
				"%s: other partitions, return.\n", __func__);
		return;
	}
}

/*
 * Function name: umounted_fs_register_pch
 * Discription: unregister blk busy/idle event
 * Parameters:
 *      @ struct super_block
 * return value:
 *      @ NULL
 */
void umounted_fs_register_pch(struct super_block *sb)
{
	if (sb->s_magic == F2FS_SUPER_MAGIC) {
		if (is_userdata_umounting()) {
			clear_userdata_umounting();
			set_userdata_umounted();
		} else {
			pch_print(LOG_LEVEL_INFO,
				"%s: data remount, return.\n", __func__);
			return;
		}
	} else {
		pch_print(LOG_LEVEL_INFO,
				"%s: other partitions, return.\n", __func__);
		return;
	}
}

/* optimize write-back */
/* pgae lru-list reintegration */

/*
 * Function name:pch_ioctl.
 * Discription:complement get usr info.
 * return value:
 *          @ 0 - success.
 *          @ -1- failure.
 */
#ifdef CONFIG_MAS_DEBUG_FS
static long pch_ioctl(struct file *file, u_int cmd, u_long arg)
{
	pch_print(LOG_LEVEL_INFO, "%s: %d: cmd=0x%x.\n", __func__, __LINE__, cmd);

	switch (cmd) {
	case PCH_IOCTL_CMD_Z:
		break;
	case PCH_IOCTL_CMD_O:
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations pch_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = pch_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pch_ioctl,
#endif
};
#endif

int pch_emui_bootstat_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", pch_info->func_enable);
	return 0;
}

int pch_emui_bootstat_proc_open(struct inode *p_inode,
		struct file *p_file)
{
	return single_open(p_file, __cfi_pch_emui_bootstat_proc_show, NULL);
}

ssize_t pch_emui_bootstat_proc_write(struct file *p_file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf;
	char getvalue;

	if (pch_info->func_enable == 1)
		return -1;

	if (count == 0)
		return -1;

	if (!userbuf)
		return -1;

	if (copy_from_user(&buf, userbuf, sizeof(char)))
		return -1;

	getvalue = buf - 48;

	if (getvalue != 1) {
		pch_print(LOG_LEVEL_ERROR, "%s: input error.\n", __func__);
		return -1;
	}

	pch_info->func_enable = 1;
	page_cache_func_enable = 1;

	return (ssize_t)count;
}

static const struct file_operations pch_emui_boot_done_fops = {
	.open		= __cfi_pch_emui_bootstat_proc_open,
	.read		= seq_read,
	.write		= __cfi_pch_emui_bootstat_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void pch_emui_boot_check_init(void)
{
	if (!proc_mkdir("pch_status", NULL)) {
		pch_print(LOG_LEVEL_ERROR, "%s: proc mk error.\n", __func__);
		return;
	}

	proc_create("pch_status/pch_enabled", 0660,
		 (struct proc_dir_entry *)NULL, &pch_emui_boot_done_fops);
}

/*
 * Function name: huawei_page_cache_manage_init
 * Discription: huawei page cache module init
 * Parameters:
 *      @ NULL
 * return value:
 *      @ 0: init succeed
 *      @ other: init failed
 */
int __init huawei_page_cache_manage_init(void)
{
	int ret = 0;
	enum bootdevice_type boot_device_type;
#ifdef CONFIG_MAS_DEBUG_FS
	int major = 0;
	struct class *pch_class = NULL;
	struct device *pch_device = NULL;
#endif

	pch_print(LOG_LEVEL_INFO, "%s: module init ++.\n", __func__);

	pch_info = kzalloc(sizeof(struct pch_info_desc), GFP_KERNEL);
	if (!pch_info)
		return -ENOMEM;

	pch_info->func_enable = page_cache_func_enable;
	pch_info->lowmem_ra_size = lowmem_ra_size;
	pch_info->readextend_ra_size = extend_ra_size;
	pch_info->lowstorage_ratio = lowstorage_ratio;
	pch_info->fs_mount_stat = 0;
	boot_device_type = get_bootdevice_type();
	if (boot_device_type == BOOT_DEVICE_EMMC) {
		pch_info->boot_device = BOOT_DEVICE_EMMC;
		pch_info->readahead_ra_size = emmc_ra_ahead_size;
		pch_info->readaround_ra_size = emmc_ra_round_size;
	} else if (boot_device_type == BOOT_DEVICE_UFS) {
		pch_info->boot_device = BOOT_DEVICE_UFS;
		pch_info->readahead_ra_size = ufs_ra_ahead_size;
		pch_info->readaround_ra_size = ufs_ra_round_size;
	} else {
		pch_print(LOG_LEVEL_ERROR,
				"%s: device not supported.\n", __func__);
		ret = -1;
		goto pch_kfree;
	}

#ifdef CONFIG_MAS_DEBUG_FS
	/*
	 * creat a debug fs node in proc/sys/page_cache
	 */
	pch_table_header = register_sysctl_table(pch_root_table);
	if (!pch_table_header) {
		pch_print(LOG_LEVEL_ERROR,
				"%s: sysctl register fail.\n", __func__);
		ret = -ENOMEM;
		goto pch_kfree;
	}

	/*
	 * creat a char dev in sys/class/pagecache_helper
	 */
	major = register_chrdev(0, PCH_DEV_NAME, &pch_fops);
	if (major <= 0) {
		pch_print(LOG_LEVEL_ERROR,
				"%s: unable to get major dev.\n", __func__);
		ret = -EFAULT;
		goto register_chrdev_error;
	}

	pch_class = class_create(THIS_MODULE, PCH_DEV_NAME);
	if (IS_ERR(pch_class)) {
		pch_print(LOG_LEVEL_ERROR,
				"%s: class_create error.\n", __func__);
		ret = -EFAULT;
		goto class_create_error;
	}

	pch_device = device_create(pch_class, NULL,
			MKDEV((unsigned int)major, 0), NULL, PCH_DEV_NAME);
	if (IS_ERR(pch_device)) {
		pch_print(LOG_LEVEL_ERROR,
				"%s: pdev_create error.\n", __func__);
		ret = -EFAULT;
		goto device_create_error;
	}
#endif

	pch_emui_boot_check_init();

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0))
	setup_timer(&pch_info->timer,
			__cfi_pch_timeout_timer, (uintptr_t)pch_info);
#else
	timer_setup(&pch_info->timer,
			__cfi_pch_timeout_timer, 0);
#endif

	/* Initialize work queues */
	INIT_WORK(&pch_info->timeout_work, __cfi_pch_timeout_work);

	sema_init(&pch_info->work_sem, 1);

	set_userdata_umounted();

	pch_print(LOG_LEVEL_INFO, "%s: module init --.\n", __func__);

	return ret;

#ifdef CONFIG_MAS_DEBUG_FS
device_create_error:
	class_destroy(pch_class);
class_create_error:
	unregister_chrdev(major, PCH_DEV_NAME);
register_chrdev_error:
	unregister_sysctl_table(pch_table_header);
#endif

pch_kfree:
	kfree(pch_info);
	pch_info = NULL;

	return ret;
}

/*
 * Function name: huawei_page_cache_manage_exit
 * Discription: huawei page cache  module exit
 * Parameters:
 *      @ NULL
 * return value:
 *      @ NULL
 */
void __exit huawei_page_cache_manage_exit(void)
{
	pch_print(LOG_LEVEL_INFO, "%s: module exit ++.\n", __func__);

	remove_proc_entry("pch_status/pch_enabled", NULL);
	remove_proc_entry("pch_status", NULL);

#ifdef CONFIG_MAS_DEBUG_FS
	unregister_sysctl_table(pch_table_header);
#endif

	kfree(pch_info);
	pch_info = NULL;
	pch_print(LOG_LEVEL_INFO, "%s: module exit --.\n", __func__);
}

module_init(__cfi_huawei_page_cache_manage_init)
module_exit(__cfi_huawei_page_cache_manage_exit)

MODULE_AUTHOR("Huawei DRV-Storage Team");
MODULE_DESCRIPTION("Flash Friendly Page Cache Evo-System");
MODULE_LICENSE("GPL");
