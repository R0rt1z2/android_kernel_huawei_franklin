/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: Implementation of set/get sdp flag and context.
 * Create: 2020.08.22
 */

#include "sdp_context.h"
#include <xattr.h>

#ifdef F2FS_FS_SDP_ENCRYPTION
int f2fs_get_sdp_context(struct inode *inode, void *ctx, size_t len,
	void *fs_data)
{
	return f2fs_getxattr(inode, F2FS_XATTR_INDEX_ECE_ENCRYPTION,
		F2FS_XATTR_NAME_ENCRYPTION_CONTEXT, ctx, len, fs_data);
}

int f2fs_set_sdp_context(struct inode *inode, const void *ctx,
	size_t len, void *fs_data)
{
	return f2fs_setxattr(inode, F2FS_XATTR_INDEX_ECE_ENCRYPTION,
		F2FS_XATTR_NAME_ENCRYPTION_CONTEXT, ctx, len,
		fs_data, XATTR_CREATE);
}

int f2fs_update_sdp_context(struct inode *inode, const void *ctx,
	size_t len, void *fs_data)
{
	return f2fs_setxattr(inode, F2FS_XATTR_INDEX_ECE_ENCRYPTION,
		F2FS_XATTR_NAME_ENCRYPTION_CONTEXT, ctx, len,
		fs_data, XATTR_REPLACE);
}

int f2fs_update_context(struct inode *inode, const void *ctx,
	size_t len, void *fs_data)
{
	return f2fs_setxattr(inode, F2FS_XATTR_INDEX_ENCRYPTION,
		F2FS_XATTR_NAME_ENCRYPTION_CONTEXT, ctx, len,
		fs_data, XATTR_REPLACE);
}

static struct f2fs_xattr_header *get_xattr_header(struct inode *inode,
	struct page *ipage, struct page **sdp_page)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	void *xattr_addr = NULL;

	*sdp_page = NULL;
	if (f2fs_has_inline_xattr(inode)) {
		if (ipage) {
			xattr_addr = inline_xattr_addr(inode, ipage);
			f2fs_wait_on_page_writeback(ipage, NODE, true, true);
		} else {
			*sdp_page = f2fs_get_node_page(sbi, inode->i_ino);
			if (IS_ERR(*sdp_page))
				return (struct f2fs_xattr_header *)(*sdp_page);
			xattr_addr = inline_xattr_addr(inode, *sdp_page);
		}
		return (struct f2fs_xattr_header *)xattr_addr;
	} else if (F2FS_I(inode)->i_xattr_nid) {
		*sdp_page = f2fs_get_node_page(sbi, F2FS_I(inode)->i_xattr_nid);
		if (IS_ERR(*sdp_page))
			return (struct f2fs_xattr_header *)(*sdp_page);
		f2fs_wait_on_page_writeback(*sdp_page, NODE, true, true);
		xattr_addr = page_address(*sdp_page);
		return (struct f2fs_xattr_header *)xattr_addr;
	} else {
		return NULL;
	}
}

int f2fs_get_sdp_encrypt_flags(struct inode *inode, void *fs_data, u32 *flags)
{
	int err;
	struct f2fs_xattr_header *hdr = NULL;
	struct page *xpage = NULL;

	if (!inode || !flags)
		return -EINVAL;
	if (!fs_data)
		down_read(&F2FS_I(inode)->i_sem);

	*flags = 0;
	hdr = get_xattr_header(inode, (struct page *)fs_data, &xpage);
	if (IS_ERR_OR_NULL(hdr)) {
		err = -EFAULT;
		goto out_unlock;
	}

	*flags = hdr->h_xattr_flags;
	err = 0;
	f2fs_put_page(xpage, 1);
out_unlock:
	if (!fs_data)
		up_read(&F2FS_I(inode)->i_sem);
	return err;
}

int f2fs_set_sdp_encrypt_flags(struct inode *inode, void *fs_data, u32 *flags)
{
	struct f2fs_sb_info *sb = NULL;
	struct f2fs_xattr_header *hdr = NULL;
	struct page *xpage = NULL;
	int err = 0;

	if (!inode || !flags)
		return -EINVAL;

	sb = F2FS_I_SB(inode);

	if (!fs_data) {
		f2fs_lock_op(sb);
		down_write(&F2FS_I(inode)->i_sem);
	}

	hdr = get_xattr_header(inode, (struct page *)fs_data, &xpage);
	if (IS_ERR_OR_NULL(hdr)) {
		err = -EFAULT;
		goto out_unlock;
	}

	hdr->h_xattr_flags = *flags;
	if (fs_data)
		set_page_dirty(fs_data);
	else if (xpage)
		set_page_dirty(xpage);

	f2fs_put_page(xpage, 1); /* unlock flag */

	f2fs_mark_inode_dirty_sync(inode, true);
	if (S_ISDIR(inode->i_mode))
		set_sbi_flag(sb, SBI_NEED_CP);

out_unlock:
	if (!fs_data) {
		up_write(&F2FS_I(inode)->i_sem);
		f2fs_unlock_op(sb);
	}
	return err;
}
#endif
