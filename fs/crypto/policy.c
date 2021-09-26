// SPDX-License-Identifier: GPL-2.0
/*
 * Encryption policy functions for per-file encryption support.
 *
 * Copyright (C) 2015, Google, Inc.
 * Copyright (C) 2015, Motorola Mobility.
 *
 * Written by Michael Halcrow, 2015.
 * Modified by Jaegeuk Kim, 2015.
 */

#include <linux/random.h>
#include <linux/string.h>
#include <linux/mount.h>
#include <keys/user-type.h>
#include <uapi/linux/keyctl.h>
#include <linux/hie.h>
#include "fscrypt_private.h"
#include "sdp_internal.h"

/*
 * check whether an encryption policy is consistent with an encryption context
 */
static bool is_encryption_context_consistent_with_policy(
				const struct fscrypt_context *ctx,
				const struct fscrypt_policy *policy,
				const struct inode *inode)
{

	if ((ctx->contents_encryption_mode !=
		 policy->contents_encryption_mode) &&
		!(hie_is_capable(inode->i_sb) &&
		 (ctx->contents_encryption_mode ==
		 FS_ENCRYPTION_MODE_PRIVATE)))
		return 0;

	return memcmp(ctx->master_key_descriptor, policy->master_key_descriptor,
		      FS_KEY_DESCRIPTOR_SIZE) == 0 &&
		(ctx->flags == policy->flags) &&
		(ctx->filenames_encryption_mode ==
		 policy->filenames_encryption_mode);
}

static int create_encryption_context_from_policy(struct inode *inode,
				const struct fscrypt_policy *policy)
{
	struct fscrypt_context ctx;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	u8 *nonce = NULL;
	u8 *plain_text = NULL;
	int res;
	const struct user_key_payload *ukp = NULL;
	struct crypto_aead *tfm = NULL;
	struct key *keyring_key = NULL;
	struct fscrypt_key *master_key = NULL;
#endif

#ifdef CONFIG_FS_UNI_ENCRYPTION
	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V2;
#else
	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
#endif
	memcpy(ctx.master_key_descriptor, policy->master_key_descriptor,
					FS_KEY_DESCRIPTOR_SIZE);

	if (!fscrypt_valid_enc_modes(policy->contents_encryption_mode,
				     policy->filenames_encryption_mode))
		return -EINVAL;

	if (policy->flags & ~FS_POLICY_FLAGS_VALID)
		return -EINVAL;

	ctx.contents_encryption_mode =
		fscrypt_data_crypt_mode(inode,
		policy->contents_encryption_mode);
	ctx.filenames_encryption_mode = policy->filenames_encryption_mode;
	ctx.flags = policy->flags;
#ifndef CONFIG_FS_UNI_ENCRYPTION
	BUILD_BUG_ON(sizeof(ctx.nonce) != FS_KEY_DERIVATION_NONCE_SIZE);
	get_random_bytes(ctx.nonce, FS_KEY_DERIVATION_NONCE_SIZE);

	return inode->i_sb->s_cop->set_context(inode, &ctx, sizeof(ctx), NULL);
#else

	BUILD_BUG_ON(sizeof(ctx.nonce) != FS_KEY_DERIVATION_CIPHER_SIZE);

	nonce = kmalloc(FS_KEY_DERIVATION_NONCE_SIZE, GFP_NOFS);
	if (!nonce)
		return -ENOMEM;
	plain_text = kmalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!plain_text) {
		kfree(nonce);
		nonce = NULL;
		return -ENOMEM;
	}
	get_random_bytes(nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	get_random_bytes(ctx.iv, FS_KEY_DERIVATION_IV_SIZE);
	memcpy(plain_text, nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	keyring_key = fscrypt_request_key(ctx.master_key_descriptor,
				FS_KEY_DESC_PREFIX, FS_KEY_DESC_PREFIX_SIZE);
	if (IS_ERR(keyring_key)) {
		if (inode->i_sb->s_cop->key_prefix) {
			u8 *prefix = (u8 *)inode->i_sb->s_cop->key_prefix;
			int prefix_size;

			prefix_size = strlen(prefix);
			keyring_key = fscrypt_request_key(ctx.master_key_descriptor,
						prefix, prefix_size);
			if (!IS_ERR(keyring_key))
				goto got_key;
		}
		return PTR_ERR(keyring_key);
	}

got_key:
	if (keyring_key->type != &key_type_logon) {
		printk_once(KERN_WARNING
				"%s: key type must be logon\n", __func__);
		res = -ENOKEY;
		goto out;
	}

	down_read(&keyring_key->sem);

	ukp = user_key_payload_rcu(keyring_key);
	if (ukp->datalen != sizeof(struct fscrypt_key)) {
		res = -EINVAL;
		up_read(&keyring_key->sem);
		goto out;
	}

	master_key = (struct fscrypt_key *)ukp->data;
	//force the size equal to FS_AES_256_GCM_KEY_SIZE since user space might pass FS_AES_256_XTS_KEY_SIZE
	master_key->size = FS_AES_256_GCM_KEY_SIZE;
	if (master_key->size != FS_AES_256_GCM_KEY_SIZE) {
		printk_once(KERN_WARNING
				"%s: key size incorrect: %d\n",
				__func__, master_key->size);
		res = -ENOKEY;
		up_read(&keyring_key->sem);
		goto out;
	}

	tfm = (struct crypto_aead *)crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm)) {
		up_read(&keyring_key->sem);
		res = PTR_ERR(tfm);
		tfm = NULL;
		pr_err("fscrypt %s : tfm allocation failed!\n", __func__);
		goto out;
	}

	res = fscrypt_set_gcm_key(tfm, master_key->raw);
	up_read(&keyring_key->sem);
	if (res)
		goto out;

	res = fscrypt_derive_gcm_key(tfm, plain_text, (u8 *)ctx.nonce, ctx.iv, 1);
	if (res)
		goto out;

	res = inode->i_sb->s_cop->set_context(inode, &ctx, sizeof(ctx), NULL);
out:
	if (nonce)
		kfree(nonce);
	if (plain_text)
		kfree(plain_text);
	if (tfm)
		crypto_free_aead(tfm);
	key_put(keyring_key);
	return res;
#endif
}

int fscrypt_ioctl_set_policy(struct file *filp, const void __user *arg)
{
	struct fscrypt_policy policy;
	struct inode *inode = file_inode(filp);
	int ret;
	struct fscrypt_context ctx;

	if (copy_from_user(&policy, arg, sizeof(policy)))
		return -EFAULT;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (policy.version != 0)
		return -EINVAL;

	ret = mnt_want_write_file(filp);
	if (ret)
		return ret;

	inode_lock(inode);

	ret = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (ret == -ENODATA) {
		if (!S_ISDIR(inode->i_mode))
			ret = -ENOTDIR;
		else if (IS_DEADDIR(inode))
			ret = -ENOENT;
		else if (!inode->i_sb->s_cop->empty_dir(inode))
			ret = -ENOTEMPTY;
		else
			ret = create_encryption_context_from_policy(inode,
								    &policy);
	} else if (ret == sizeof(ctx) &&
		   is_encryption_context_consistent_with_policy(&ctx,
								&policy,
								inode)) {
		/* The file already uses the same encryption policy. */
		ret = 0;
	} else if (ret >= 0 || ret == -ERANGE) {
		/* The file already uses a different encryption policy. */
		ret = -EEXIST;
	}

	inode_unlock(inode);

	mnt_drop_write_file(filp);
	return ret;
}
EXPORT_SYMBOL(fscrypt_ioctl_set_policy);

int fscrypt_ioctl_get_policy(struct file *filp, void __user *arg)
{
	struct inode *inode = file_inode(filp);
	struct fscrypt_context ctx;
	struct fscrypt_policy policy;
	int res;

	if (!IS_ENCRYPTED(inode))
		return -ENODATA;

	res = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (res < 0 && res != -ERANGE)
		return res;
	if (res != sizeof(ctx))
		return -EINVAL;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (ctx.format != FS_ENCRYPTION_CONTEXT_FORMAT_V2)
		return -EINVAL;
#endif

	policy.version = 0;
	policy.contents_encryption_mode = ctx.contents_encryption_mode;
	policy.filenames_encryption_mode = ctx.filenames_encryption_mode;
	policy.flags = ctx.flags;

	/* in compliance with android */
	if (S_ISDIR(inode->i_mode) &&
		policy.contents_encryption_mode !=
		FS_ENCRYPTION_MODE_INVALID)
		policy.contents_encryption_mode =
			FS_ENCRYPTION_MODE_AES_256_XTS;

	memcpy(policy.master_key_descriptor, ctx.master_key_descriptor,
				FS_KEY_DESCRIPTOR_SIZE);

	if (copy_to_user(arg, &policy, sizeof(policy)))
		return -EFAULT;
	return 0;
}
EXPORT_SYMBOL(fscrypt_ioctl_get_policy);

/**
 * fscrypt_has_permitted_context() - is a file's encryption policy permitted
 *				     within its directory?
 *
 * @parent: inode for parent directory
 * @child: inode for file being looked up, opened, or linked into @parent
 *
 * Filesystems must call this before permitting access to an inode in a
 * situation where the parent directory is encrypted (either before allowing
 * ->lookup() to succeed, or for a regular file before allowing it to be opened)
 * and before any operation that involves linking an inode into an encrypted
 * directory, including link, rename, and cross rename.  It enforces the
 * constraint that within a given encrypted directory tree, all files use the
 * same encryption policy.  The pre-access check is needed to detect potentially
 * malicious offline violations of this constraint, while the link and rename
 * checks are needed to prevent online violations of this constraint.
 *
 * Return: 1 if permitted, 0 if forbidden.  If forbidden, the caller must fail
 * the filesystem operation with EPERM.
 */
int fscrypt_has_permitted_context(struct inode *parent, struct inode *child)
{
	const struct fscrypt_operations *cops = parent->i_sb->s_cop;
	const struct fscrypt_info *parent_ci, *child_ci;
	struct fscrypt_context parent_ctx, child_ctx;
	int res;

	/* No restrictions on file types which are never encrypted */
	if (!S_ISREG(child->i_mode) && !S_ISDIR(child->i_mode) &&
	    !S_ISLNK(child->i_mode))
		return 1;

	/* No restrictions if the parent directory is unencrypted */
	if (!IS_ENCRYPTED(parent))
		return 1;

	/* Encrypted directories must not contain unencrypted files */
	if (!IS_ENCRYPTED(child))
		return 0;
#ifdef F2FS_FS_SDP_ENCRYPTION
	/*
	 * for SDP file we should use original CE context
	 * since the ci_master_key in struct i_crypt_info is changed. This
	 * should be done before fscrypt_get_encryption_info since it can be
	 * called only once for sece file when lock.
	 */
	if (child->i_sb->s_cop && child->i_sb->s_cop->is_file_sdp_encrypted) {
		if (child->i_sb->s_cop->is_file_sdp_encrypted(child))
			goto sdp_perm;
	}
#endif
	/*
	 * Both parent and child are encrypted, so verify they use the same
	 * encryption policy.  Compare the fscrypt_info structs if the keys are
	 * available, otherwise retrieve and compare the fscrypt_contexts.
	 *
	 * Note that the fscrypt_context retrieval will be required frequently
	 * when accessing an encrypted directory tree without the key.
	 * Performance-wise this is not a big deal because we already don't
	 * really optimize for file access without the key (to the extent that
	 * such access is even possible), given that any attempted access
	 * already causes a fscrypt_context retrieval and keyring search.
	 *
	 * In any case, if an unexpected error occurs, fall back to "forbidden".
	 */

	res = fscrypt_get_encryption_info(parent);
	if (res)
		return 0;
	res = fscrypt_get_encryption_info(child);
	if (res)
		return 0;
	parent_ci = parent->i_crypt_info;
	child_ci = child->i_crypt_info;

	if (parent_ci && child_ci) {
		return memcmp(parent_ci->ci_master_key_descriptor,
			      child_ci->ci_master_key_descriptor,
			      FS_KEY_DESCRIPTOR_SIZE) == 0 &&
			(parent_ci->ci_data_mode == child_ci->ci_data_mode) &&
			(parent_ci->ci_filename_mode ==
			 child_ci->ci_filename_mode) &&
			(parent_ci->ci_flags == child_ci->ci_flags);
	}
#ifdef F2FS_FS_SDP_ENCRYPTION
sdp_perm:
#endif
	res = cops->get_context(parent, &parent_ctx, sizeof(parent_ctx));
	if (res != sizeof(parent_ctx))
		return 0;

	res = cops->get_context(child, &child_ctx, sizeof(child_ctx));
	if (res != sizeof(child_ctx))
		return 0;

	parent_ctx.contents_encryption_mode =
		fscrypt_data_crypt_mode(parent,
		parent_ctx.contents_encryption_mode);
	child_ctx.contents_encryption_mode =
		fscrypt_data_crypt_mode(child,
		child_ctx.contents_encryption_mode);

	return memcmp(parent_ctx.master_key_descriptor,
		      child_ctx.master_key_descriptor,
		      FS_KEY_DESCRIPTOR_SIZE) == 0 &&
		(parent_ctx.contents_encryption_mode ==
		 child_ctx.contents_encryption_mode) &&
		(parent_ctx.filenames_encryption_mode ==
		 child_ctx.filenames_encryption_mode) &&
		(parent_ctx.flags == child_ctx.flags);
}
EXPORT_SYMBOL(fscrypt_has_permitted_context);

/**
 * fscrypt_inherit_context() - Sets a child context from its parent
 * @parent: Parent inode from which the context is inherited.
 * @child:  Child inode that inherits the context from @parent.
 * @fs_data:  private data given by FS.
 * @preload:  preload child i_crypt_info if true
 *
 * Return: 0 on success, -errno on failure
 */
int fscrypt_inherit_context(struct inode *parent, struct inode *child,
						void *fs_data, bool preload)
{
	struct fscrypt_context ctx;
	struct fscrypt_info *ci;
	int res;

#ifdef CONFIG_FS_UNI_ENCRYPTION
	u8 *nonce = NULL;
	u8 *plain_text = NULL;
#endif

	res = fscrypt_get_encryption_info(parent);
	if (res < 0)
		return res;

	ci = parent->i_crypt_info;
	if (ci == NULL)
		return -ENOKEY;

#ifdef CONFIG_FS_UNI_ENCRYPTION
	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V2;
#else
	ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
#endif
	ctx.contents_encryption_mode = ci->ci_data_mode;
	ctx.filenames_encryption_mode = ci->ci_filename_mode;
	ctx.flags = ci->ci_flags;
	memcpy(ctx.master_key_descriptor, ci->ci_master_key_descriptor,
	       FS_KEY_DESCRIPTOR_SIZE);
#ifdef CONFIG_FS_UNI_ENCRYPTION
	nonce = kmalloc(FS_KEY_DERIVATION_NONCE_SIZE, GFP_KERNEL);
	if (!nonce)
		return -ENOMEM;
	plain_text = kmalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_KERNEL);
	if (!plain_text) {
		kfree(nonce);
		nonce = NULL;
		return -ENOMEM;
	}
	get_random_bytes(nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	get_random_bytes(ctx.iv, FS_KEY_DERIVATION_IV_SIZE);
	memcpy(plain_text, nonce, FS_KEY_DERIVATION_NONCE_SIZE);
	res = fscrypt_derive_gcm_key(ci->ci_gtfm, plain_text, (u8 *)ctx.nonce, ctx.iv, 1);
#else
	get_random_bytes(ctx.nonce, FS_KEY_DERIVATION_NONCE_SIZE);
#endif

#ifndef CONFIG_FS_UNI_ENCRYPTION
	BUILD_BUG_ON(sizeof(ctx) != FSCRYPT_SET_CONTEXT_MAX_SIZE);
#endif
	res = parent->i_sb->s_cop->set_context(child, &ctx,
						sizeof(ctx), fs_data);
#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (nonce)
		kfree(nonce);
	if (plain_text)
		kfree(plain_text);
#endif
	if (res)
		return res;
	return preload ? fscrypt_get_encryption_info(child): 0;
}
EXPORT_SYMBOL(fscrypt_inherit_context);

int fscrypt_set_bio_ctx(struct inode *inode, struct bio *bio)
{
	struct fscrypt_info *ci;
	int ret = -ENOENT;

	if (!inode || !bio)
		return ret;

	ci = inode->i_crypt_info;

#ifdef CONFIG_FS_UNI_ENCRYPTION
	if(ci && ci->ci_key) {
		bio_bcf_clear(bio, BC_CRYPT);
		return 0;
	}
#endif

	if (S_ISREG(inode->i_mode) && ci &&
	    (ci->ci_data_mode == FS_ENCRYPTION_MODE_PRIVATE)) {
		WARN_ON(!hie_is_capable(inode->i_sb));
		/* HIE: default use aes-256-xts */
		bio_bcf_set(bio, BC_CRYPT | BC_AES_256_XTS);
		bio->bi_crypt_ctx.bc_key_size = FS_AES_256_XTS_KEY_SIZE;
		bio->bi_crypt_ctx.bc_ino = inode->i_ino;
		bio->bi_crypt_ctx.bc_sb = inode->i_sb;

		bio->bi_crypt_ctx.bc_info_act = &fscrypt_crypt_info_act;
		bio->bi_crypt_ctx.bc_info =
			fscrypt_crypt_info_act(
			ci, BIO_BC_INFO_GET);
		WARN_ON(!bio->bi_crypt_ctx.bc_info);

#ifdef CONFIG_HIE_DEBUG
		if (hie_debug(HIE_DBG_FS))
			pr_info("HIE: %s: ino: %ld, bio: %p\n",
				__func__, inode->i_ino, bio);
#endif
		ret = 0;
	} else
		bio_bcf_clear(bio, BC_CRYPT);

	return ret;
}

int fscrypt_key_payload(struct bio_crypt_ctx *ctx,
		const unsigned char **key)
{
	struct fscrypt_info *fi;

	fi = (struct fscrypt_info *)ctx->bc_info;

	if (!fi) {
		pr_info("HIE: %s: missing crypto info\n", __func__);
		return -ENOKEY;
	}

	if (key)
		*key = &(fi->ci_raw_key[0]);

	return ctx->bc_key_size;
}

int fscrypt_is_hw_encrypt(const struct inode *inode)
{
	struct fscrypt_info *ci = inode->i_crypt_info;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (ci && ci->ci_key)
		return 0;
	else
		return S_ISREG(inode->i_mode) && ci &&
			ci->ci_data_mode == FS_ENCRYPTION_MODE_PRIVATE;
#else
	return S_ISREG(inode->i_mode) && ci &&
		ci->ci_data_mode == FS_ENCRYPTION_MODE_PRIVATE;
#endif
}

int fscrypt_is_sw_encrypt(const struct inode *inode)
{
	struct fscrypt_info *ci = inode->i_crypt_info;

	return S_ISREG(inode->i_mode) && ci &&
		ci->ci_data_mode != FS_ENCRYPTION_MODE_INVALID &&
		ci->ci_data_mode != FS_ENCRYPTION_MODE_PRIVATE;
}
