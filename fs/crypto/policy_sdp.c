/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: Implementation of 1) sdp context inherit;
 *                                2) set/get sdp context.
 * Create: 2020.08.22
 */

#include <keys/user-type.h>
#include <linux/printk.h>
#include <linux/mount.h>
#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include <securec.h>
#include <fscrypt_private.h>
#include "keyinfo_sdp.h"

#ifdef F2FS_FS_SDP_ENCRYPTION

#ifdef CONFIG_HWDPS
#include <huawei_platform/hwdps/hwdps_fs_hooks.h>
#include <huawei_platform/hwdps/hwdps_limits.h>
#endif

/* sdp crypto policy for f2fs */
struct fscrypt_sdp_policy {
	u8 version;
	u8 sdp_class;
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
} __packed;

/* crypto policy type for f2fs */
struct fscrypt_policy_type {
	u8 version;
	u8 encryption_type;
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
} __packed;

#ifdef CONFIG_HWDPS
static const int RESERVED_DATA_LEN = 8;

/* dps crypto policy for f2fs */
struct fscrypt_dps_policy {
	__u8 version;
	__u8 reserved[RESERVED_DATA_LEN];
} __packed;
#endif

static inline bool f2fs_inode_is_config_encryption(struct inode *inode)
{
	if (!inode->i_sb->s_cop->get_context)
		return false;

	return (inode->i_sb->s_cop->get_context(inode, NULL, 0L) > 0);
}

static void set_sdp_context_from_policy(struct sdp_fscrypt_context *sdp_ctx,
	const struct fscrypt_sdp_policy *policy)
{
	sdp_ctx->format = FS_ENCRYPTION_CONTEXT_FORMAT_V2;
	(void)memcpy_s(sdp_ctx->master_key_descriptor,
		sizeof(sdp_ctx->master_key_descriptor),
		policy->master_key_descriptor,
		sizeof(policy->master_key_descriptor));
	sdp_ctx->contents_encryption_mode = policy->contents_encryption_mode;
	sdp_ctx->filenames_encryption_mode = policy->filenames_encryption_mode;
	sdp_ctx->flags = policy->flags;
	sdp_ctx->sdp_class = policy->sdp_class;
	sdp_ctx->version = policy->version;
}

static int f2fs_create_sdp_encryption_context_from_policy(struct inode *inode,
	const struct fscrypt_sdp_policy *policy)
{
	int res;
	struct sdp_fscrypt_context sdp_ctx = {0};

	if ((policy->sdp_class != FSCRYPT_SDP_ECE_CLASS) &&
		(policy->sdp_class != FSCRYPT_SDP_SECE_CLASS)) {
		sdp_pr_err("%s class err %x\n", __func__, policy->sdp_class);
		return -EINVAL;
	}
	if (!fscrypt_valid_enc_modes(policy->contents_encryption_mode,
		policy->filenames_encryption_mode)) {
		sdp_pr_err("%s modes invalid content %x, file %x\n",
			__func__, policy->contents_encryption_mode,
			policy->filenames_encryption_mode);
		return -EINVAL;
	}
	if (policy->flags & ~FS_POLICY_FLAGS_VALID) {
		sdp_pr_err("%s flags is err %x\n", __func__, policy->flags);
		return -EINVAL;
	}
	if (S_ISREG(inode->i_mode)) {
		res = f2fs_inode_check_sdp_keyring(
			(const u8 *)policy->master_key_descriptor,
			NO_NEED_TO_CHECK_KEYFING);
		if (res != 0)
			return res;
	}
	set_sdp_context_from_policy(&sdp_ctx, policy);
	res = inode->i_sb->s_cop->set_sdp_context(inode, &sdp_ctx,
		sizeof(sdp_ctx), NULL);
	if (res != 0) {
		sdp_pr_err("%s: inode %lu set sdp ctx failed res %d\n",
			__func__, inode->i_ino, res);
		return res;
	}
	if (policy->sdp_class == FSCRYPT_SDP_ECE_CLASS)
		res = f2fs_inode_set_sdp_encryption_flags(inode, NULL,
			F2FS_XATTR_SDP_ECE_CONFIG_FLAG);
	else
		res = f2fs_inode_set_sdp_encryption_flags(inode, NULL,
			F2FS_XATTR_SDP_SECE_CONFIG_FLAG);
	if (res != 0)
		sdp_pr_err("%s: inode %lu set flags failed res %d\n",
			__func__, inode->i_ino, res);
	if (S_ISREG(inode->i_mode) && (res == 0) && (inode->i_crypt_info)) {
		res = f2fs_change_to_sdp_crypto(inode, NULL);
		if (res != 0) {
			u32 flag = 0;
			(void)inode->i_sb->s_cop->set_sdp_encrypt_flags(inode,
				NULL, &flag);
		}
	}
	return res;
}

static bool f2fs_is_sdp_context_consistent_with_policy(struct inode *inode,
	const struct fscrypt_sdp_policy *policy)
{
	int res;
	struct sdp_fscrypt_context ctx = {0};

	if (!inode->i_sb->s_cop->get_sdp_context)
		return false;

	res = inode->i_sb->s_cop->get_sdp_context(inode, &ctx, sizeof(ctx),
		NULL);
	if (res != sizeof(ctx))
		return false;

	return (memcmp(ctx.master_key_descriptor, policy->master_key_descriptor,
		FS_KEY_DESCRIPTOR_SIZE) == 0 &&
		(ctx.sdp_class == policy->sdp_class) &&
		(ctx.flags == policy->flags) &&
		(ctx.contents_encryption_mode ==
		policy->contents_encryption_mode) &&
		(ctx.filenames_encryption_mode ==
		policy->filenames_encryption_mode));
}

int f2fs_fscrypt_ioctl_set_sdp_policy(struct file *filp,
	const void __user *arg)
{
	int ret;
	u32 flag = 0;
	struct fscrypt_sdp_policy policy = {0};
	struct inode *inode = NULL;

	if (!filp || !arg)
		return -EINVAL;
	inode = file_inode(filp);
	if (!inode_owner_or_capable(inode))
		return -EACCES;
	ret = copy_from_user(&policy, arg, sizeof(policy));
	if (ret != 0) {
		sdp_pr_err("%s copy failed ret %d\n", __func__, ret);
		return -EFAULT;
	}
	if (policy.version != 0) {
		sdp_pr_err("%s version err %x\n", __func__, policy.version);
		return -EINVAL;
	}
	ret = mnt_want_write_file(filp);
	if (ret != 0) {
		sdp_pr_err("%s mnt write failed ret:%d\n", __func__, ret);
		return ret;
	}

	inode_lock(inode);
	down_write(&inode->i_sdp_sem);

	ret = f2fs_inode_get_sdp_encrypt_flags(inode, NULL, &flag);
	if (ret != 0) {
		sdp_pr_err("%s get flag failed res %d\n", __func__, ret);
		goto err;
	}
	if (!f2fs_inode_is_config_encryption(inode)) {
		sdp_pr_err("%s is not config\n", __func__);
		ret = -EINVAL;
	} else if (!f2fs_inode_is_config_sdp_encryption(flag)) {
		ret = f2fs_create_sdp_encryption_context_from_policy(inode,
			&policy);
	} else if (!f2fs_is_sdp_context_consistent_with_policy(inode,
			&policy)) {
		sdp_pr_err("%s: consistent with sdp context\n", __func__);
		ret = -EINVAL;
	}
err:
	up_write(&inode->i_sdp_sem);
	inode_unlock(inode);
	mnt_drop_write_file(filp);
	return ret;
}

#ifdef CONFIG_HWDPS

static int check_inode_flag(struct inode *inode)
{
	int res;
	u32 flags = 0;

	if (!inode || !inode->i_sb || !inode->i_sb->s_cop ||
		!inode->i_sb->s_cop->get_hwdps_flags) {
		pr_err("%s get_sdp_encrypt_flags NULL\n", __func__);
		return -EFAULT;
	}

	res = inode->i_sb->s_cop->get_hwdps_flags(inode, NULL, &flags);
	if (res != 0) {
		pr_err("%s get_sdp_encrypt_flags err = %d\n", __func__, res);
		return -EFAULT;
	}

	if ((flags & HWDPS_ENABLE_FLAG) != 0) {
		pr_err("%s has flag, no need to set again\n", __func__);
		return -EEXIST;
	}

	if (f2fs_inode_is_sdp_encrypted(flags) != 0) {
		pr_err("%s has sdp flags, failed to set dps flags\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static int set_dps_attr_and_flag(struct inode *inode)
{
	int ret;
	uint8_t *encoded_wfek = NULL;
	uint32_t encoded_len = HWDPS_ENCODED_WFEK_SIZE;
	secondary_buffer_t buffer_wfek = { &encoded_wfek, &encoded_len };

	if (S_ISREG(inode->i_mode)) {
		ret = hwdps_get_fek_from_origin(
			inode->i_crypt_info->ci_master_key_descriptor,
			inode, &buffer_wfek);
		if (ret != 0) {
			pr_err("f2fs_dps hwdps_get_fek_from_origin err %d\n",
				ret);
			goto err;
		}
	} else {
		encoded_wfek = kzalloc(HWDPS_ENCODED_WFEK_SIZE, GFP_NOFS);
		if (!encoded_wfek) {
			pr_err("f2fs_dps kzalloc err\n");
			ret = -ENOMEM;
			goto err;
		}
	}
	ret = inode->i_sb->s_cop->set_hwdps_attr(inode, encoded_wfek,
		encoded_len, NULL);
	if (ret != 0) {
		pr_err("f2fs_dps set_hwdps_attr err\n");
		goto err;
	}

	ret = f2fs_set_hwdps_enable_flags(inode, NULL);
	if (ret != 0) {
		pr_err("f2fs_dps f2fs_set_hwdps_enable_flags err %d\n", ret);
		goto err;
	}
err:
	kzfree(encoded_wfek);
	return ret;
}

static int set_dps_policy_inner(struct file *filp, struct inode *inode)
{
	int ret;

	ret = mnt_want_write_file(filp);
	if (ret != 0) {
		pr_err("f2fs_dps mnt_want_write_file erris %d\n", ret);
		return ret;
	}

	inode_lock(inode);
	ret = check_inode_flag(inode);
	if (ret == -EEXIST) {
		ret = 0;
		goto err;
	}
	if (ret != 0)
		goto err;

	ret = set_dps_attr_and_flag(inode);
	if (ret != 0)
		goto err;
	/* just set hw_enc_flag for regular file */
	if (S_ISREG(inode->i_mode) && inode->i_crypt_info)
		inode->i_crypt_info->ci_hw_enc_flag |=
			HWDPS_XATTR_ENABLE_FLAG_NEW;
err:
	inode_unlock(inode);
	mnt_drop_write_file(filp);
	return ret;
}

int f2fs_fscrypt_ioctl_set_dps_policy(struct file *filp,
		const void __user *arg)
{
	struct fscrypt_dps_policy policy = {0};
	struct inode *inode = NULL;

	pr_debug("%s: start!\n", __func__);
	if (!filp || !arg)
		return -EINVAL;

	inode = file_inode(filp);

	if (!inode->i_crypt_info) {
		pr_err("%s crypt info null\n", __func__);
		return -EFAULT;
	}

	if (!inode_owner_or_capable(inode)) {
		pr_err("%s inode_owner_or_capable err\n", __func__);
		return -EACCES;
	}

	if (copy_from_user(&policy, arg, sizeof(policy))) {
		pr_err("%s copy_from_user err\n", __func__);
		return -EFAULT;
	}

	if (policy.version != 0) {
		pr_err("%s policy.version err : %u\n", __func__,
			policy.version);
		return -EINVAL;
	}

	return set_dps_policy_inner(filp, inode);
}
#endif

int f2fs_fscrypt_ioctl_get_sdp_policy(struct file *filp,
	void __user *arg)
{
	int res;
	struct sdp_fscrypt_context ctx = {0};
	struct fscrypt_sdp_policy policy = {0};
	struct inode *inode = NULL;

	if (!filp || !arg) {
		sdp_pr_err("%s get policy invalid param\n", __func__);
		return -EINVAL;
	}
	inode = file_inode(filp);
	if (!inode->i_sb->s_cop->get_sdp_context) {
		sdp_pr_err("%s get context is null\n", __func__);
		return -ENODATA;
	}

	res = inode->i_sb->s_cop->get_sdp_context(inode, &ctx, sizeof(ctx),
		NULL);
	if (res != sizeof(ctx)) {
		sdp_pr_err("%s get sdp context failed, res%d\n", __func__,
			res);
		return -ENODATA;
	}

	if (ctx.format != FS_ENCRYPTION_CONTEXT_FORMAT_V2) {
		sdp_pr_err("%s format is err %x\n", __func__, ctx.format);
		return -EINVAL;
	}

	policy.version = ctx.version;
	policy.sdp_class = ctx.sdp_class;
	policy.contents_encryption_mode = ctx.contents_encryption_mode;
	policy.filenames_encryption_mode = ctx.filenames_encryption_mode;
	policy.flags = ctx.flags;
	(void)memcpy_s(policy.master_key_descriptor,
		sizeof(policy.master_key_descriptor),
		ctx.master_key_descriptor,
		sizeof(ctx.master_key_descriptor));
	res = copy_to_user(arg, &policy, sizeof(policy));
	if (res != 0) {
		sdp_pr_err("%s copy failed res:%d\n", __func__, res);
		return -EFAULT;
	}
	return res;
}

int f2fs_fscrypt_ioctl_get_policy_type(struct file *filp,
	void __user *arg)
{
	int res;
	u32 flags = 0;
	struct inode *inode = NULL;
	struct fscrypt_info *ci = NULL;
	struct fscrypt_policy_type policy = {0};

	if (!filp || !arg) {
		sdp_pr_err("%s invalid param\n", __func__);
		return -EINVAL;
	}

	inode = file_inode(filp);
	ci = inode->i_crypt_info;
	if (!ci) {
		sdp_pr_err("%s ci is null\n", __func__);
		return -ENOMEM;
	}

	policy.contents_encryption_mode = FS_ENCRYPTION_MODE_AES_256_XTS;
	policy.filenames_encryption_mode = FS_ENCRYPTION_MODE_AES_256_CTS;
	policy.version = 0;
	policy.encryption_type = FSCRYPT_CE_CLASS;
	(void)memcpy_s(policy.master_key_descriptor,
		sizeof(policy.master_key_descriptor),
		ci->ci_master_key_descriptor,
		sizeof(ci->ci_master_key_descriptor));

	if (!inode->i_sb->s_cop->get_sdp_encrypt_flags) {
		sdp_pr_warn("%s get flags is null\n", __func__);
		return -EINVAL;
	}

	res = inode->i_sb->s_cop->get_sdp_encrypt_flags(inode, NULL, &flags);
	if (res != 0) {
		sdp_pr_warn("%s get flags failed res:%d\n", __func__, res);
		goto copy_and_out;
	}
	if ((flags & F2FS_XATTR_SDP_ECE_ENABLE_FLAG) != 0)
		policy.encryption_type = FSCRYPT_SDP_ECE_CLASS;
	else if ((flags & F2FS_XATTR_SDP_SECE_ENABLE_FLAG) != 0)
		policy.encryption_type = FSCRYPT_SDP_SECE_CLASS;
#ifdef CONFIG_HWDPS
	else if ((flags & HWDPS_ENABLE_FLAG) != 0)
		policy.encryption_type = FSCRYPT_DPS_CLASS;
#endif

copy_and_out:
	res = copy_to_user(arg, &policy, sizeof(policy));
	if (res != 0) {
		sdp_pr_err("%s copy failed res%d\n", __func__, res);
		return -EFAULT;
	}
	return res;
}

int sdp_crypt_inherit(struct inode *parent, struct inode *child,
	void *page, void *fs_data)
{
	int res;
	struct sdp_fscrypt_context sdp_ctx = {0};

	/* page can be null, no need check */
	if (!parent || !child || !fs_data) {
		sdp_pr_err("%s invalid param\n", __func__);
		return -EINVAL;
	}
	res = parent->i_sb->s_cop->get_sdp_context(parent, &sdp_ctx,
		sizeof(sdp_ctx), page);
	if (res != sizeof(sdp_ctx))
		return 0;

	if (S_ISREG(child->i_mode)) {
		res = f2fs_inode_check_sdp_keyring(
			sdp_ctx.master_key_descriptor,
			NO_NEED_TO_CHECK_KEYFING);
		if (res != 0)
			return res;
	}

	down_write(&child->i_sdp_sem);
	res = parent->i_sb->s_cop->set_sdp_context(child, &sdp_ctx,
		sizeof(sdp_ctx), fs_data);
	if (res != 0) {
		sdp_pr_err("%s set child context failed, res:%d\n", __func__,
			res);
		goto out;
	}

	if (sdp_ctx.sdp_class == FSCRYPT_SDP_ECE_CLASS)
		res = f2fs_inode_set_sdp_encryption_flags(child, fs_data,
			F2FS_XATTR_SDP_ECE_CONFIG_FLAG);
	else if (sdp_ctx.sdp_class == FSCRYPT_SDP_SECE_CLASS)
		/* SECE can not be enable at the first time */
		res = f2fs_inode_set_sdp_encryption_flags(child, fs_data,
			F2FS_XATTR_SDP_SECE_CONFIG_FLAG);
	else
		res = -EOPNOTSUPP;
	if (res != 0)
		sdp_pr_err("%s failed, res:%d\n", __func__, res);
out:
	up_write(&child->i_sdp_sem);
	return res;
}
#endif
