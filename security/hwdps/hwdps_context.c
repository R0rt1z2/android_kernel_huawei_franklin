/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Implementation of set/get hwdps flag and context.
 * Create: 2020-06-16
 */

#ifdef CONFIG_HWDPS
#include "hwdps_context.h"
#include <linux/f2fs_fs.h>
#include <f2fs.h>
#include <xattr.h>

#define HWDPS_XATTR_NAME "hwdps"

s32 f2fs_get_hwdps_attr(struct inode *inode, void *buf, size_t len,
	struct page *dpage)
{
	return f2fs_getxattr(inode, F2FS_XATTR_INDEX_ENCRYPTION,
		HWDPS_XATTR_NAME, buf, len, dpage);
}

s32 f2fs_set_hwdps_attr(struct inode *inode, const void *attr, size_t len,
	void *fs_data)
{
	return f2fs_setxattr(inode, F2FS_XATTR_INDEX_ENCRYPTION,
		HWDPS_XATTR_NAME, attr, len, fs_data, XATTR_CREATE);
}

s32 f2fs_update_hwdps_attr(struct inode *inode, const void *attr,
	size_t len, void *fs_data)
{
	return f2fs_setxattr(inode, F2FS_XATTR_INDEX_ENCRYPTION,
		HWDPS_XATTR_NAME, attr, len, fs_data, XATTR_REPLACE);
}

static struct f2fs_xattr_header *get_xattr_header(struct inode *inode,
	struct page *ipage, struct page **xpage)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	void *xattr_addr = NULL;

	*xpage = NULL;
	if (f2fs_has_inline_xattr(inode)) {
		if (ipage) {
			xattr_addr = inline_xattr_addr(inode, ipage);
			f2fs_wait_on_page_writeback(ipage, NODE, true, true);
		} else {
			*xpage = f2fs_get_node_page(sbi, inode->i_ino);
			if (IS_ERR(*xpage))
				return (struct f2fs_xattr_header *)(*xpage);
			xattr_addr = inline_xattr_addr(inode, *xpage);
		}
		return (struct f2fs_xattr_header *)xattr_addr;
	} else if (F2FS_I(inode)->i_xattr_nid) {
		*xpage = f2fs_get_node_page(sbi, F2FS_I(inode)->i_xattr_nid);
		if (IS_ERR(*xpage))
			return (struct f2fs_xattr_header *)(*xpage);
		f2fs_wait_on_page_writeback(*xpage, NODE, true, true);
		xattr_addr = page_address(*xpage);
		return (struct f2fs_xattr_header *)xattr_addr;
	} else {
		return NULL;
	}
}

/* mainly copied from f2fs_get_sdp_encrypt_flags */
s32 f2fs_get_hwdps_flags(struct inode *inode, void *fs_data, u32 *flags)
{
	struct f2fs_xattr_header *hdr = NULL;
	struct page *xpage = NULL;
	s32 err = 0;

	if (!inode || !flags)
		return -EINVAL;
	if (!fs_data)
		down_read(&F2FS_I(inode)->i_sem);

	*flags = 0;
	hdr = get_xattr_header(inode, (struct page *)fs_data, &xpage);
	if (IS_ERR_OR_NULL(hdr)) {
		err = (s32)PTR_ERR(hdr);
		goto out_unlock;
	}

	*flags = hdr->h_xattr_flags;
	f2fs_put_page(xpage, 1);
out_unlock:
	if (!fs_data)
		up_read(&F2FS_I(inode)->i_sem);
	return err;
}

/* mainly copied from f2fs_set_sdp_encrypt_flags */
s32 f2fs_set_hwdps_flags(struct inode *inode, void *fs_data, u32 *flags)
{
	struct f2fs_sb_info *sbi = NULL;
	struct f2fs_xattr_header *hdr = NULL;
	struct page *xpage = NULL;
	s32 err = 0;

	if (!inode || !flags)
		return -EINVAL;
	sbi = F2FS_I_SB(inode);
	if (!fs_data) {
		f2fs_lock_op(sbi);
		down_write(&F2FS_I(inode)->i_sem);
	}

	hdr = get_xattr_header(inode, (struct page *)fs_data, &xpage);
	if (IS_ERR_OR_NULL(hdr)) {
		err = (s32)PTR_ERR(hdr);
		f2fs_put_page(xpage, 1);
		goto out_unlock;
	}

	hdr->h_xattr_flags = *flags;
	if (fs_data)
		set_page_dirty(fs_data);
	else if (xpage)
		set_page_dirty(xpage);

	f2fs_put_page(xpage, 1);

	f2fs_mark_inode_dirty_sync(inode, true);
	if (S_ISDIR(inode->i_mode))
		set_sbi_flag(sbi, SBI_NEED_CP);

out_unlock:
	if (!fs_data) {
		up_write(&F2FS_I(inode)->i_sem);
		f2fs_unlock_op(sbi);
	}
	return err;
}
#endif
