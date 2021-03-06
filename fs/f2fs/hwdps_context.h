/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Implementation of set/get hwdps flag and context.
 * Create: 2020-06-16
 */

#ifndef __HWDPS_CONTEXT_H
#define __HWDPS_CONTEXT_H

#ifdef CONFIG_HWDPS
#include <linux/fs.h>

s32 f2fs_get_hwdps_attr(struct inode *inode, void *buf, size_t len,
	struct page *page);

s32 f2fs_set_hwdps_attr(struct inode *inode, const void *attr, size_t len,
	void *fs_data);

s32 f2fs_update_hwdps_attr(struct inode *inode, const void *attr,
	size_t len, void *fs_data);

s32 f2fs_get_hwdps_flags(struct inode *inode, void *fs_data, u32 *flags);

s32 f2fs_set_hwdps_flags(struct inode *inode, void *fs_data, u32 *flags);

#endif
#endif
