/* Copyright (c) Huawei Technologies Co., Ltd. 2017-2019. All rights reserved.
 * FileName: pagecache_debug.c
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/pagecache_debug.h>
#include <securec.h>

unsigned int pagecache_dump;

static struct dentry *pagecache_debugfs_root;
struct fs_pagecache_info *pagecache_info;

static char *no_log_file[] = {
	"kmsgcat-log"
};

static inline const char *pgcache_dump_path(const struct path *path, char *buf, int buflen)
{
	if (pagecache_dump & BIT_PGCACHE_DUMP_FULLPATH_ENABLE)
		return d_path(path, buf, buflen);
	else
		return path->dentry->d_name.name;
}

static inline const char *pgcache_dump_dentry(const struct dentry *dentry, char *buf, int buflen)
{
	if (pagecache_dump & BIT_PGCACHE_DUMP_FULLPATH_ENABLE)
		return dentry_path((struct dentry *)dentry, buf, buflen);
	else
		return dentry->d_name.name;
}

static inline bool bypass_no_log_file(const struct dentry *dentry)
{
	int i;
	size_t len1, len2;

	for (i = 0; i < sizeof(no_log_file) / sizeof(char *); i++) {
		len1 = strlen(no_log_file[i]);
		len2 = strlen(dentry->d_name.name);
		if (len1 == len2) {
			if (!strncmp(no_log_file[i], dentry->d_name.name, len1))
				return true;
		}
	}
	return false;
}

static inline void __pgcache_dump_log(u32 ctrl, const struct path *path, const struct dentry *dentry,
			struct va_format *vaf)
{
	char buf[DUMP_PATH_LENGTH];
	char stat_log[DUMP_STAT_LENGTH];
	const struct dentry *d_tmp = NULL;

	if ((!path) && (!dentry)) {
		printk("%s%s%pV\n",
			pagecache_dump & BIT_PGCACHE_DUMP_ERROR_PRINT ? KERN_ERR : KERN_DEBUG,
			PGCACHE_DUMP_HEAD_LINE, vaf);
		return;
	}

	if (pagecache_dump & BIT_PGCACHE_DUMP_DENTRY_STATUS) {
		if (path)
			d_tmp = path->dentry;
		else if (dentry)
			d_tmp = dentry;
	}

	if (d_tmp != NULL) {
		if (bypass_no_log_file(d_tmp))
			return;
		snprintf_s(stat_log, DUMP_STAT_LENGTH, DUMP_STAT_LENGTH,
			"times: m_Sync_ra, %lu, g_Sync_ra, %lu, Async_ra, %lu, shrink, %lu",
			d_tmp->mapping_stat.mmap_sync_read_times, d_tmp->mapping_stat.generic_sync_read_times,
			d_tmp->mapping_stat.async_read_times, d_tmp->mapping_stat.shrink_page_times);
	} else {
		stat_log[0] = '\0';
	}

	printk("%s%s%pV,(file, %s, %s)\n",
		pagecache_dump & BIT_PGCACHE_DUMP_ERROR_PRINT ? KERN_ERR : KERN_DEBUG,
		PGCACHE_DUMP_HEAD_LINE, vaf,
		path ? pgcache_dump_path(path, buf, DUMP_PATH_LENGTH) : pgcache_dump_dentry(dentry, buf, DUMP_PATH_LENGTH), stat_log);
}

void pgcache_log(u32 ctrl, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!(pagecache_dump & ctrl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	__pgcache_dump_log(ctrl, NULL, NULL, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(pgcache_log);

void pgcache_log_path(u32 ctrl, const struct path *path, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!(pagecache_dump & ctrl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	__pgcache_dump_log(ctrl, path, NULL, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(pgcache_log_path);

void pgcache_log_dentry(u32 ctrl, const struct dentry *dentry, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!(pagecache_dump & ctrl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	__pgcache_dump_log(ctrl, NULL, dentry, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(pgcache_log_dentry);

void pagecache_debug_msg(const char *level, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	printk("%spagecache-stat: %pV\n", level, &vaf);
	va_end(args);
}

int pagecache_build_stats(void)
{
	struct pagecache_stat *stats = kzalloc(sizeof(struct pagecache_stat), GFP_KERNEL);

	if (!stats)
		return -ENOMEM;

	pagecache_info = kzalloc(sizeof(struct fs_pagecache_info), GFP_KERNEL);
	if (!pagecache_info) {
		kfree(stats);
		return -ENOMEM;
	}
	pagecache_info->enable = 1;
	pagecache_info->stat = stats;

	return 0;
}

static int pagecache_stat_show(struct seq_file *s, void *v)
{
	seq_printf(s, "\n=====[ pagecache info ]=====\n");
	seq_printf(s, "  - hit: %ld\n", atomic64_read(&pagecache_info->stat->hit_total));
	seq_printf(s, "  - miss: %ld\n", atomic64_read(&pagecache_info->stat->miss_total));
	seq_printf(s, "  - sync read pages: %ld\n", atomic64_read(&pagecache_info->stat->syncread_pages_count));
	seq_printf(s, "  - async read pages: %ld\n", atomic64_read(&pagecache_info->stat->asyncread_pages_count));
	seq_printf(s, "  - mmap hit: %ld\n", atomic64_read(&pagecache_info->stat->mmap_hit_total));
	seq_printf(s, "  - mmap miss: %ld\n", atomic64_read(&pagecache_info->stat->mmap_miss_total));
	seq_printf(s, "  - mmap sync read pages: %ld\n", atomic64_read(&pagecache_info->stat->mmap_syncread_pages_count));
	seq_printf(s, "  - mmap async read pages: %ld\n", atomic64_read(&pagecache_info->stat->mmap_asyncread_pages_count));
	seq_printf(s, "  - write back count: %ld\n", atomic64_read(&pagecache_info->stat->wb_count));
	seq_printf(s, "  - write back pages: %ld\n", atomic64_read(&pagecache_info->stat->wb_pages_count));
	seq_printf(s, "  - dirty pages: %ld\n", atomic64_read(&pagecache_info->stat->dirty_pages_count));
	seq_printf(s, "  - shrink pages: %ld\n", atomic64_read(&pagecache_info->stat->shrink_pages_count));

	return 0;
}

static int pagecache_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, pagecache_stat_show, inode->i_private);
}

static const struct file_operations stat_fops = {
	.owner = THIS_MODULE,
	.open = pagecache_stat_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int pagecache_stat_enable_get(void *data, u64 *val)
{
	*val = pagecache_info->enable;
	return 0;
}

static int pagecache_stat_enable_set(void *data, u64 val)
{
	if (val == 0 || val == 1) {
		pagecache_info->enable = val;
	} else if (val == 2) {
		/*clear the statistics*/
		pagecache_info->enable = val;
		atomic64_set(&pagecache_info->stat->hit_total, 0);
		atomic64_set(&pagecache_info->stat->miss_total, 0);
		atomic64_set(&pagecache_info->stat->syncread_pages_count, 0);
		atomic64_set(&pagecache_info->stat->asyncread_pages_count, 0);
		atomic64_set(&pagecache_info->stat->mmap_hit_total, 0);
		atomic64_set(&pagecache_info->stat->mmap_miss_total, 0);
		atomic64_set(&pagecache_info->stat->mmap_syncread_pages_count, 0);
		atomic64_set(&pagecache_info->stat->mmap_asyncread_pages_count, 0);
		atomic64_set(&pagecache_info->stat->wb_count, 0);
		atomic64_set(&pagecache_info->stat->wb_pages_count, 0);
		atomic64_set(&pagecache_info->stat->dirty_pages_count, 0);
		atomic64_set(&pagecache_info->stat->shrink_pages_count, 0);
	} else {
		pagecache_debug_msg(KERN_ERR, "Invalid enable value");
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pagecache_stat_enable_fops,
		pagecache_stat_enable_get, pagecache_stat_enable_set, "%llu\n");

int __init pagecache_create_root_stats(void)
{
	struct dentry *dent_s = NULL;
	struct dentry *dent_e = NULL;

	pagecache_debugfs_root = debugfs_create_dir("pagecache", NULL);
	if (!pagecache_debugfs_root)
		return -ENOMEM;

	dent_s = debugfs_create_file("status", S_IRUGO, pagecache_debugfs_root,
			NULL, &stat_fops);
	dent_e = debugfs_create_file("enable", S_IRUSR | S_IWUSR, pagecache_debugfs_root,
			NULL, &pagecache_stat_enable_fops);

	if (!dent_s || !dent_e) {
		debugfs_remove(pagecache_debugfs_root);
		pagecache_debugfs_root = NULL;
		return -ENOMEM;
	}

	return 0;
}

/* module init */
static int __init huawei_page_cache_debug_init(void)
{
	int err;

	err = pagecache_create_root_stats();
	if (err)
		return err;

	err = pagecache_build_stats();
	if (err)
		return err;
	return 0;
}

/* module exit*/
static void __exit huawei_page_cache_debug_exit(void)
{
	if (!pagecache_debugfs_root)
		return;

	debugfs_remove_recursive(pagecache_debugfs_root);
	pagecache_debugfs_root = NULL;

	if (pagecache_info)
		kfree(pagecache_info->stat);
	kfree(pagecache_info);
	pagecache_info = NULL;
}

module_init(huawei_page_cache_debug_init)
module_exit(huawei_page_cache_debug_exit)

MODULE_AUTHOR("Huawei DRV-Storage Team");
MODULE_DESCRIPTION("Flash Friendly Page Cache Evo-System");
MODULE_LICENSE("GPL");

