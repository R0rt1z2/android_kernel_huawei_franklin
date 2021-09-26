/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: Implementation of 1) get sdp encryption key info;
 *                                2) update sdp context.
 * Create: 2020.08.22
 */

#include "keyinfo_sdp.h"
#include <linux/scatterlist.h>
#include <linux/hashtable.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/random.h>
#include <linux/f2fs_fs.h>
#include <securec.h>
#include <uapi/linux/keyctl.h>
#include <keys/user-type.h>
#include <crypto/skcipher.h>
#include <crypto/aead.h>
#include <fscrypt_private.h>
#include "ecdh.h"

#ifdef F2FS_FS_SDP_ENCRYPTION

struct context_t {
	struct fscrypt_context *ctx;
	struct sdp_fscrypt_context *sdp_ctx;
};

static int get_payload_for_sdp(const struct user_key_payload *payload,
	u8 *raw, int *size, bool file_pub_key)
{
	/*
	 * if filepubkey is ture, class private key doesn't
	 * need exist.
	 * if filepubkey is false, class private key must exist
	 */
	int res = 0;
	struct fscrypt_sdp_key *mst_sdp =
		(struct fscrypt_sdp_key *)payload->data;

	if (mst_sdp->sdp_class == FSCRYPT_SDP_ECE_CLASS &&
		mst_sdp->size == FS_AES_256_GCM_KEY_SIZE) {
		if (memcpy_s(raw, *size, mst_sdp->raw, mst_sdp->size) != EOK)
			return -EINVAL;
		*size = mst_sdp->size;
	} else if (mst_sdp->sdp_class == FSCRYPT_SDP_SECE_CLASS &&
		(!file_pub_key || mst_sdp->size ==
		FS_SDP_ECC_PRI_KEY_SIZE) &&
		mst_sdp->pub_key_size == FS_SDP_ECC_PUB_KEY_SIZE) {
		if (memcpy_s(raw, *size, mst_sdp->raw,
			mst_sdp->size) != EOK)
			return -EINVAL;
		if (memcpy_s(raw + mst_sdp->size, *size + mst_sdp->size,
			mst_sdp->pub_key, mst_sdp->pub_key_size) != EOK)
			return -EINVAL;
		*size = mst_sdp->pub_key_size + mst_sdp->size;
	} else {
		sdp_pr_err("%s invalid class\n", __func__);
		res = -EKEYREVOKED;
	}
	return res;
}

static int f2fs_do_get_keyring_payload(
	const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE], u8 *raw, int *size,
	bool file_pub_key)
{
	struct key *keyring_key = NULL;
	struct user_key_payload *payload = NULL;
	struct fscrypt_key *mst_key = NULL;
	int res = -EKEYREVOKED;

	keyring_key = fscrypt_request_key(descriptor, FS_KEY_DESC_PREFIX,
		FS_KEY_DESC_PREFIX_SIZE);
	if (IS_ERR(keyring_key)) {
		sdp_pr_err("%s keyring key is err\n", __func__);
		return PTR_ERR(keyring_key);
	}
	down_read(&keyring_key->sem);
	if (keyring_key->type != &key_type_logon)
		goto out;
	payload = user_key_payload_locked(keyring_key);
	if (!payload)
		goto out;
	if (payload->datalen == sizeof(struct fscrypt_key)) {
		mst_key = (struct fscrypt_key *)payload->data;
		if (mst_key->size == FS_AES_256_GCM_KEY_SIZE) {
			if (memcpy_s(raw, *size, mst_key->raw,
				mst_key->size) != EOK) {
				res = -EINVAL;
				goto out;
			}
			*size = mst_key->size;
			res = 0;
		}
	} else if (payload->datalen == sizeof(struct fscrypt_sdp_key)) {
		res = get_payload_for_sdp(payload, raw, size, file_pub_key);
		if (res != 0)
			sdp_pr_err("%s get payload failed res:%d\n", __func__,
				res);
	}
out:
	up_read(&keyring_key->sem);
	key_put(keyring_key);
	return res;
}

#ifdef CONFIG_FS_UNI_ENCRYPTION
static int f2fs_do_get_keyindex(const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE],
	int *key_index)
{
	struct key *keyring_key = NULL;
	const struct user_key_payload *payload = NULL;
	struct fscrypt_key *master_key = NULL;
	int res = -EKEYREVOKED;

	keyring_key = fscrypt_request_key(descriptor, FS_KEY_DESC_PREFIX,
		FS_KEY_DESC_PREFIX_SIZE);
	if (IS_ERR(keyring_key))
		return PTR_ERR(keyring_key);

	down_read(&keyring_key->sem);
	if (keyring_key->type != &key_type_logon)
		goto out;

	payload = user_key_payload_locked(keyring_key);
	if (!payload)
		goto out;

	if (payload->datalen == sizeof(struct fscrypt_key)) {
		master_key = (struct fscrypt_key *)payload->data;
		if (master_key->size == FS_AES_256_GCM_KEY_SIZE) {
			*key_index = (int)(*(master_key->raw +
				FS_KEY_INDEX_OFFSET) & 0xff);
			res = 0;
		}
	}
out:
	up_read(&keyring_key->sem);
	key_put(keyring_key);
	return res;
}
#endif

static int f2fs_do_get_fek(const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE],
	u8 *nonce, u8 *fek, u8 *iv, int enc)
{
	int res;
	struct crypto_aead *tfm = NULL;
	u8 raw[FS_MAX_KEY_SIZE] = {0};
	int size = FS_MAX_KEY_SIZE;

	tfm = crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm))
		return (int)PTR_ERR(tfm);

	res = f2fs_do_get_keyring_payload(descriptor, raw, &size, false);
	if (res != 0)
		goto out;

	res = file_protect_set_gcm_key(tfm, raw, size);
	if (res != 0) {
		sdp_pr_err("%s set key failed, res:%d", __func__, res);
		goto out;
	}

	if (enc != 0)
		res = file_protect_derive_gcm_key(tfm, fek,
			FS_KEY_DERIVATION_CIPHER_SIZE, nonce,
			FS_KEY_DERIVATION_CIPHER_SIZE, iv,
			FS_KEY_DERIVATION_IV_SIZE, FLAG_ENCRYPT);
	else
		res = file_protect_derive_gcm_key(tfm, nonce,
			FS_KEY_DERIVATION_CIPHER_SIZE, fek,
			FS_KEY_DERIVATION_CIPHER_SIZE, iv,
			FS_KEY_DERIVATION_IV_SIZE, FLAG_DECRYPT);
out:
	if (res != 0)
		sdp_pr_err("%s derive key failed, enc:%d, res:%d", __func__,
			enc, res);
	crypto_free_aead(tfm);
	memzero_explicit(raw, (size_t)FS_MAX_KEY_SIZE);
	return res;
}

static int get_shared_key_with_file_priv(buffer_t *dev_pub_key,
	buffer_t *file_pub_key, buffer_t *shared_key_buf,
	const struct sdp_fscrypt_context *ctx, const buffer_t *raw)
{
	int res;

	dev_pub_key->data = raw->data;
	dev_pub_key->len = raw->len;
	file_pub_key->data = (u8 *)ctx->file_pub_key;
	file_pub_key->len = FS_SDP_ECC_PUB_KEY_SIZE;
	res = get_file_pubkey_shared_secret(ECC_CURVE_NIST_P256,
		dev_pub_key, file_pub_key, shared_key_buf);
	if (res != 0)
		sdp_pr_err("%s failed\n", __func__);
	return res;
}

static int get_shared_key_with_file_pub(buffer_t *dev_priv_key,
	buffer_t *file_pub_key, buffer_t *shared_key_buf,
	const struct sdp_fscrypt_context *ctx, const buffer_t *raw)
{
	int res;

	dev_priv_key->data = raw->data;
	dev_priv_key->len = raw->len;
	file_pub_key->data = (u8 *)ctx->file_pub_key;
	file_pub_key->len = FS_SDP_ECC_PUB_KEY_SIZE;
	res = get_shared_secret(ECC_CURVE_NIST_P256,
		dev_priv_key, file_pub_key, shared_key_buf);
	if (res != 0)
		sdp_pr_err("%s failed\n", __func__);
	return res;
}

static int get_sece_shared_key(const struct sdp_fscrypt_context *ctx,
	buffer_t *shared_key_buf, buffer_t *fek, u8 has_pub_key)
{
	int res;
	int offset_len;
	u8 raw[FS_SDP_ECC_PRI_KEY_SIZE + FS_SDP_ECC_PUB_KEY_SIZE] = {0};
	int size = FS_SDP_ECC_PRI_KEY_SIZE + FS_SDP_ECC_PUB_KEY_SIZE;
	buffer_t dev_priv_key = { NULL, 0 };
	buffer_t dev_pub_key = { NULL, 0 };
	buffer_t file_pub_key = { NULL, 0 };
	buffer_t raw_buf = { NULL, 0 };

	res = f2fs_do_get_keyring_payload(ctx->master_key_descriptor,
		(u8 *)raw, &size, has_pub_key == ENHANCED_CHECK_KEYING);
	if (res != 0) {
		sdp_pr_err("%s get keyring failed res:%d\n", __func__, res);
		goto out_and_set;
	}
	offset_len = (size == FS_SDP_ECC_PUB_KEY_SIZE) ? 0 :
		FS_SDP_ECC_PRI_KEY_SIZE;

	if (has_pub_key == 0) {
		raw_buf.data = raw + offset_len;
		raw_buf.len = FS_SDP_ECC_PUB_KEY_SIZE;
		res = get_shared_key_with_file_priv(&dev_pub_key, &file_pub_key,
			shared_key_buf, ctx, &raw_buf);
	} else {
		raw_buf.data = raw;
		raw_buf.len = FS_SDP_ECC_PRI_KEY_SIZE;
		res = get_shared_key_with_file_pub(&dev_priv_key, &file_pub_key,
			shared_key_buf, ctx, &raw_buf);
	}
	if (res != 0)
		sdp_pr_err("%s result failed %d\n", __func__, res);
out_and_set:
	memzero_explicit(raw, (size_t)(FS_SDP_ECC_PRI_KEY_SIZE +
		FS_SDP_ECC_PUB_KEY_SIZE));
	return res;
}

static int f2fs_do_get_sece_fek(const struct sdp_fscrypt_context *ctx,
	buffer_t *fek, u8 has_pub_key)
{
	int res;
	struct crypto_aead *tfm = NULL;
	u8 shared_key[FS_AES_256_GCM_KEY_SIZE] = {0};
	buffer_t shared_key_buf = { shared_key, FS_AES_256_GCM_KEY_SIZE };

	tfm = crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm)) {
		sdp_pr_err("%s alloc aead failed\n", __func__);
		return (int)PTR_ERR(tfm);
	}
	res = get_sece_shared_key(ctx, &shared_key_buf, fek, has_pub_key);
	if (res != 0)
		goto out_and_free;

	res = file_protect_set_gcm_key(tfm, shared_key, sizeof(shared_key));
	if (res != 0) {
		sdp_pr_err("%s set failed res %d\n", __func__, res);
		goto out_and_free;
	}

	if (has_pub_key != 0)
		res = file_protect_derive_gcm_key(tfm, (const u8 *)ctx->nonce,
			FS_KEY_DERIVATION_CIPHER_SIZE,
			fek->data, FS_KEY_DERIVATION_CIPHER_SIZE, (u8 *)ctx->iv,
			sizeof(ctx->iv), FLAG_DECRYPT);
	else
		res = file_protect_derive_gcm_key(tfm, fek->data,
			FS_KEY_DERIVATION_CIPHER_SIZE,
			(u8 *)ctx->nonce, FS_KEY_DERIVATION_CIPHER_SIZE,
			(u8 *)ctx->iv, sizeof(ctx->iv), FLAG_ENCRYPT);
out_and_free:
	if (res != 0)
		sdp_pr_err("%s derive key failed, res:%d", __func__, res);
	crypto_free_aead(tfm);
	memzero_explicit(shared_key, (size_t)(FS_AES_256_GCM_KEY_SIZE));
	return res;
}

static inline int f2fs_do_set_sece_fek(const struct sdp_fscrypt_context *ctx,
	buffer_t *fek)
{
	return f2fs_do_get_sece_fek(ctx, fek, NO_PUB_KEY);
}

static void change_to_sdp_init_ci_info(struct fscrypt_info *ci_info_temp,
	const struct fscrypt_info *ci_info,
	const struct sdp_fscrypt_context *sdp_ctx)
{
	ci_info_temp->ci_data_mode = ci_info->ci_data_mode;
	ci_info_temp->ci_filename_mode = ci_info->ci_filename_mode;
	ci_info_temp->ci_mode = ci_info->ci_mode;
	(void)memcpy_s(ci_info_temp->ci_master_key_descriptor,
		sizeof(ci_info_temp->ci_master_key_descriptor),
		sdp_ctx->master_key_descriptor,
		sizeof(sdp_ctx->master_key_descriptor));
	ci_info_temp->ci_flags = sdp_ctx->flags;
	(void)memcpy_s(ci_info_temp->ci_nonce, sizeof(ci_info_temp->ci_nonce),
		ci_info->ci_nonce, sizeof(ci_info->ci_nonce));
#ifdef CONFIG_FS_UNI_ENCRYPTION
	ci_info_temp->ci_key = NULL;
	ci_info_temp->ci_key_len = 0;
	ci_info_temp->ci_key_index = -1; /* init with invalid value */
#endif
}

static int fill_ci_key_and_set_cipher(struct inode *inode,
	const struct fscrypt_context *ctx,
	struct sdp_fscrypt_context *sdp_ctx,
	struct fscrypt_info *crypt_info,
	buffer_t *fek)
{
	int res;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	int key_index;

	res = f2fs_do_get_keyindex(ctx->master_key_descriptor, &key_index);
	if (res != 0) {
		sdp_pr_err("%s get index failed, res:%d\n", __func__, res);
		return res;
	}
	crypt_info->ci_key_index = key_index;
	if (crypt_info->ci_key_index < 0 || crypt_info->ci_key_index >
		MAX_UFS_SLOT_INDEX) {
		sdp_pr_err("sdp key %s: %d\n", __func__,
			crypt_info->ci_key_index);
		return -EINVAL;
	}
	kzfree(crypt_info->ci_key);
	crypt_info->ci_key = kzalloc(FS_KEY_DERIVATION_NONCE_SIZE,
		GFP_NOFS);
	if (!crypt_info->ci_key)
		return -ENOMEM;
	crypt_info->ci_key_len = FS_KEY_DERIVATION_NONCE_SIZE;
	if (memcpy_s(crypt_info->ci_key, crypt_info->ci_key_len, fek->data,
		fek->len) != EOK) {
		kzfree(crypt_info->ci_key);
		crypt_info->ci_key = NULL;
		sdp_pr_err("%s memcpy_s ci_key failed, len:%u\n", __func__,
			fek->len);
		return -EINVAL;
	}
#else
	res = set_cipher_for_sdp(crypt_info, sdp_ctx, fek->data,
		fek->len, inode);
	if (res != 0) {
		sdp_pr_err("%s set cipher failed res%d\n", __func__, res);
		return res;
	}
#endif
	return res;
}

static int encrypt_fek_with_new_ctx(const struct sdp_fscrypt_context *sdp_ctx,
	buffer_t *fek, struct inode *inode)
{
	int res;

	if (sdp_ctx->sdp_class == FSCRYPT_SDP_ECE_CLASS) {
		res = f2fs_do_get_fek(sdp_ctx->master_key_descriptor,
			(u8 *)sdp_ctx->nonce, fek->data, (u8 *)sdp_ctx->iv,
			FLAG_ENCRYPT);
		if (res != 0)
			sdp_pr_err("%s: error %d (inode %lu) en_ece failed!\n",
				__func__, res, inode->i_ino);
		return res;
	}
	if (sdp_ctx->sdp_class != FSCRYPT_SDP_SECE_CLASS) {
		sdp_pr_err("%s invalid sdpClass %x\n", __func__, sdp_ctx->sdp_class);
		return -EINVAL;
	}
	res = f2fs_do_set_sece_fek(sdp_ctx, fek);
	if (res != 0)
		sdp_pr_err("%s: error %d inode %lu encrypt sece key failed\n",
			__func__, res, inode->i_ino);
	return res;
}

static int get_or_generate_fek(struct inode *inode,
	const struct context_t *ctx_all, buffer_t *fek, buffer_t *iv, u32 flag)
{
	int res;
	int inherit = 0;

	if (!f2fs_inode_is_enabled_sdp_encryption(flag) &&
		i_size_read(inode))
		inherit = 1; /* 1 means the file is not null */

	if (iv->len != sizeof(ctx_all->sdp_ctx->iv))
		return -EINVAL;
	if (inherit != 0) {
#ifdef CONFIG_FS_UNI_ENCRYPTION
		res = f2fs_do_get_fek(ctx_all->ctx->master_key_descriptor,
			ctx_all->ctx->nonce, fek->data, ctx_all->ctx->iv,
			FLAG_DECRYPT);
		if (res != 0) {
			sdp_pr_err("%s inherit fail res:%d\n", __func__, res);
			return res;
		}
		(void)memcpy_s(iv->data, iv->len, ctx_all->ctx->iv,
			sizeof(ctx_all->ctx->iv));
		return res;
#else
		sdp_pr_err("not support uni encrypt can't inherit\n");
		return -EINVAL;
#endif
	}
	if (f2fs_inode_is_enabled_sdp_ece_encryption(flag)) {
		res = f2fs_do_get_fek(ctx_all->sdp_ctx->master_key_descriptor,
			ctx_all->sdp_ctx->nonce, fek->data,
			ctx_all->sdp_ctx->iv, FLAG_DECRYPT);
		if (res != 0)
			return res;
		(void)memcpy_s(iv->data, iv->len, ctx_all->sdp_ctx->iv,
			sizeof(ctx_all->sdp_ctx->iv));
		return res;
	}
	if (f2fs_inode_is_enabled_sdp_sece_encryption(flag)) {
		res = f2fs_do_get_sece_fek(ctx_all->sdp_ctx, fek, HAS_PUB_KEY);
		if (res != 0) {
			sdp_pr_err("%s get sece fek fail, res:%d\n",
				__func__, res);
			return res;
		}
		(void)memcpy_s(iv->data, iv->len, ctx_all->sdp_ctx->iv,
			sizeof(ctx_all->sdp_ctx->iv));
		return res;
	}
	get_random_bytes(fek->data, fek->len);
	get_random_bytes(iv->data, iv->len);
	return 0;
}

static int f2fs_get_sdp_crypt_info_from_context(struct inode *inode,
	struct fscrypt_context *ctx, struct sdp_fscrypt_context *sdp_ctx,
	struct fscrypt_info *crypt_info, u32 flag)
{
	int res;
	u8 iv[FS_KEY_IV_SIZE] = {0};
	u8 fek[FS_KEY_CIPHER_SIZE] = {0};
	buffer_t fek_buf = { fek, FS_KEY_SDP_DERIVATION_NONCE_SIZE };
	buffer_t iv_buf = { iv, FS_KEY_IV_SIZE };
	struct context_t context_all = { ctx, sdp_ctx };

	res = get_or_generate_fek(inode, &context_all, &fek_buf, &iv_buf, flag);
	if (res != 0)
		goto free_and_out;
	res = fill_ci_key_and_set_cipher(inode, ctx, sdp_ctx, crypt_info,
		&fek_buf);
	if (res != 0)
		goto free_and_out;
	if (f2fs_inode_is_enabled_sdp_ece_encryption(flag))
		goto free_and_out;
	if (f2fs_inode_is_enabled_sdp_sece_encryption(flag))
		goto free_and_out;

	if (memcpy_s(sdp_ctx->iv, sizeof(sdp_ctx->iv), iv_buf.data,
		iv_buf.len) != EOK) {
			res = -EFAULT;
			goto free_and_out;
	}
	res = encrypt_fek_with_new_ctx(sdp_ctx, &fek_buf, inode);
free_and_out:
	memzero_explicit(fek, (size_t)sizeof(fek));
	return res;
}

static int change_to_sdp_fill_crypt_info(struct inode *inode,
	const struct fscrypt_context *ctx,
	struct sdp_fscrypt_context *sdp_ctx,
	struct fscrypt_info *crypt_info)
{
	int res;
	u8 iv[FS_KEY_IV_SIZE] = {0};
	u8 fek[FS_KEY_CIPHER_SIZE] = {0};
	buffer_t fek_buf = { NULL, 0 };
	int inherit = 0;
	/*
	 * file is not null, ece should inherit ce nonce iv,
	 * sece also support
	 */
	if (i_size_read(inode))
		inherit = 1; /* 1 means the file is not null */

	if (inherit != 0) {
#ifdef CONFIG_FS_UNI_ENCRYPTION
		res = f2fs_do_get_fek(ctx->master_key_descriptor,
			(u8 *)ctx->nonce, fek, (u8 *)ctx->iv, FLAG_DECRYPT);
		if (res != 0) {
			sdp_pr_err("%s inherit get fek failed, res:%d\n",
				__func__, res);
			goto free_and_out;
		}
		(void)memcpy_s(iv, sizeof(iv), ctx->iv, sizeof(ctx->iv));
#else
		sdp_pr_err("%s not support uni encryption can't inherit\n",
			__func__);
		return -EINVAL;
#endif
	} else {
		/*
		 * the fek buffer is for decrypt or encrypt which size is 80,
		 * and only fek plaintext is 64.
		 */
		get_random_bytes(fek, FS_KEY_SDP_DERIVATION_NONCE_SIZE);
		get_random_bytes(iv, FS_KEY_IV_SIZE);
	}
	fek_buf.data = fek;
	fek_buf.len = FS_KEY_SDP_DERIVATION_NONCE_SIZE;
	(void)memcpy_s(sdp_ctx->iv, sizeof(sdp_ctx->iv), iv, sizeof(iv));
	res = fill_ci_key_and_set_cipher(inode, ctx, sdp_ctx, crypt_info,
		&fek_buf);
	if (res != 0)
		goto free_and_out;

	res = encrypt_fek_with_new_ctx(sdp_ctx, &fek_buf, inode);
free_and_out:
	memzero_explicit(fek, (size_t)FS_MAX_KEY_SIZE);
	return res;
}

static int get_context_all(struct inode *inode,
	struct sdp_fscrypt_context *sdp_ctx, struct fscrypt_context *ctx,
	void *fs_data)
{
	int res = inode->i_sb->s_cop->get_sdp_context(inode, sdp_ctx,
		sizeof(struct sdp_fscrypt_context), fs_data);
	if (res != sizeof(struct sdp_fscrypt_context)) {
		sdp_pr_err(" %s get sdp context fail res:%d\n", __func__, res);
		return -EINVAL;
	}
	res = inode->i_sb->s_cop->get_context(inode, ctx,
		sizeof(struct fscrypt_context));
	if (res != sizeof(struct fscrypt_context)) {
		sdp_pr_err("%s get context failed, res:%d\n", __func__, res);
		return -EINVAL;
	}
	return 0;
}

static int do_update_sdp_and_clear_old_context(struct inode *inode,
	const struct sdp_fscrypt_context *sdp_ctx, struct fscrypt_context *ctx,
	void *fs_data, u32 sdp_flag)
{
	/* should set sdp context back for getting the nonce */
	int res = inode->i_sb->s_cop->update_sdp_context(inode, sdp_ctx,
		sizeof(struct sdp_fscrypt_context), fs_data);
	if (res != 0) {
		sdp_pr_err("%s: inode(%lu) set sdp ctx failed res %d.\n",
			__func__, inode->i_ino, res);
		return res;
	}

	res = f2fs_inode_set_sdp_encryption_flags(inode, fs_data,
		sdp_flag);
	if (res != 0) {
		sdp_pr_err("%s: set sdp crypt flag failed! need to check!\n",
			__func__);
		return res;
	}

	/* clear the ce crypto info */
	(void)memset_s(ctx->nonce, sizeof(ctx->nonce), 0, sizeof(ctx->nonce));
#ifdef CONFIG_FS_UNI_ENCRYPTION
	(void)memset_s(ctx->iv, sizeof(ctx->iv), 0, sizeof(ctx->iv));
#endif
	res = inode->i_sb->s_cop->update_context(inode, ctx,
		sizeof(struct fscrypt_context), fs_data);
	if (res != 0)
		sdp_pr_err("%s: inode(%lu) update ce ctx failed res %d.\n",
			__func__, inode->i_ino, res);
	return res;
}

static int update_sdp_and_clear_old_context(struct inode *inode,
	struct fscrypt_context *ctx,
	const struct sdp_fscrypt_context *sdp_ctx, void *fs_data)
{
	int res;
	u32 flag = 0;

	res = f2fs_inode_get_sdp_encrypt_flags(inode, fs_data, &flag);
	if (res != 0)
		return res;

	if (f2fs_inode_is_config_sdp_ece_encryption(flag)) {
		res = do_update_sdp_and_clear_old_context(inode, sdp_ctx, ctx,
			fs_data, F2FS_XATTR_SDP_ECE_ENABLE_FLAG);
	} else {
		res = do_update_sdp_and_clear_old_context(inode, sdp_ctx, ctx,
			fs_data, F2FS_XATTR_SDP_SECE_ENABLE_FLAG);
	}
	if (res != 0)
		sdp_pr_err("sdp %s: update sdp flag flags failed\n", __func__);
	return res;
}

static int init_crypt_info_from_context(struct inode *inode,
	struct fscrypt_info *crypt_info,
	const struct sdp_fscrypt_context *sdp_ctx)
{
	struct fscrypt_mode *mode = NULL;

	crypt_info->ci_flags = sdp_ctx->flags;
	crypt_info->ci_data_mode = fscrypt_data_crypt_mode(inode,
		sdp_ctx->contents_encryption_mode);
	crypt_info->ci_filename_mode = sdp_ctx->filenames_encryption_mode;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	crypt_info->ci_key = NULL;
	crypt_info->ci_key_len = 0;
	crypt_info->ci_key_index = -1;
#endif
	(void)memcpy_s(crypt_info->ci_master_key_descriptor,
		sizeof(crypt_info->ci_master_key_descriptor),
		sdp_ctx->master_key_descriptor,
		sizeof(sdp_ctx->master_key_descriptor));
	mode = select_encryption_mode(crypt_info, inode);
	if (IS_ERR(mode)) {
		sdp_pr_err("%s sdp select_encryption_mode failed\n", __func__);
		return -EINVAL;
	}
	WARN_ON(mode->ivsize > FSCRYPT_MAX_IV_SIZE);
	crypt_info->ci_mode = mode;
	return 0;
}

static int do_get_sdp_crypt_info(struct inode *inode, void *fs_data,
	u32 flag)
{
	struct fscrypt_context ctx = {0};
	struct sdp_fscrypt_context sdp_ctx = {0};
	struct fscrypt_info *crypt_info = NULL;

	int res = get_context_all(inode, &sdp_ctx, &ctx, fs_data);

	if (res != 0)
		return res;

	crypt_info = kmem_cache_zalloc(fscrypt_info_cachep, GFP_NOFS);
	if (!crypt_info)
		return -ENOMEM;
	fscrypt_get_crypt_info(crypt_info, true);
	res = init_crypt_info_from_context(inode, crypt_info, &sdp_ctx);
	if (res != 0)
		goto out;

	res = f2fs_get_sdp_crypt_info_from_context(inode, &ctx, &sdp_ctx,
		crypt_info, flag);
	if (res != 0) {
		sdp_pr_err("sdp %s: get cypt info failed\n", __func__);
		goto out;
	}
	if (sdp_ctx.sdp_class == FSCRYPT_SDP_ECE_CLASS)
		crypt_info->ci_hw_enc_flag = F2FS_XATTR_SDP_ECE_ENABLE_FLAG;
	else
		crypt_info->ci_hw_enc_flag = F2FS_XATTR_SDP_SECE_ENABLE_FLAG;
	if (cmpxchg(&inode->i_crypt_info, NULL, crypt_info) == NULL)
		crypt_info = NULL;

	if (f2fs_inode_is_enabled_sdp_ece_encryption(flag))
		goto out;
	if (f2fs_inode_is_enabled_sdp_sece_encryption(flag))
		goto out;

	res = update_sdp_and_clear_old_context(inode, &ctx,
		&sdp_ctx, fs_data);
out:
	if (res == -ENOKEY)
		res = 0;
	put_crypt_info(crypt_info);
	return res;
}

int f2fs_change_to_sdp_crypto(struct inode *inode, void *fs_data)
{
	int res;
	struct fscrypt_info *ci_info = NULL;
	struct fscrypt_info *ci_info_temp = NULL;
	struct fscrypt_context ctx = {0};
	struct sdp_fscrypt_context sdp_ctx = {0};

	if (!inode) /* fs_data is NULL so not need to jduge */
		return -EINVAL;
	ci_info = inode->i_crypt_info;
	if (!ci_info) {
		sdp_pr_err("%s ci_info is null\n", __func__);
		return -EINVAL;
	}
	res = get_context_all(inode, &sdp_ctx, &ctx, fs_data);
	if (res != 0)
		return res;

	ci_info_temp = kmem_cache_zalloc(fscrypt_info_cachep, GFP_NOFS);
	if (!ci_info_temp)
		return -ENOMEM;
	fscrypt_get_crypt_info(ci_info_temp, true);
	change_to_sdp_init_ci_info(ci_info_temp, ci_info, &sdp_ctx);

	res = change_to_sdp_fill_crypt_info(inode, &ctx, &sdp_ctx,
		ci_info_temp);
	if (res != 0)
		goto out_and_free;

	res = update_sdp_and_clear_old_context(inode, &ctx, &sdp_ctx, fs_data);
	if (res != 0)
		goto out_and_free;
	if (sdp_ctx.sdp_class == FSCRYPT_SDP_ECE_CLASS)
		ci_info_temp->ci_hw_enc_flag = F2FS_XATTR_SDP_ECE_ENABLE_FLAG;
	else
		ci_info_temp->ci_hw_enc_flag = F2FS_XATTR_SDP_SECE_ENABLE_FLAG;
	/* free the i_crypt_info and point to the ci_info_temp */
	put_crypt_info(inode->i_crypt_info);
	inode->i_crypt_info = ci_info_temp;
	return res;
out_and_free:
	put_crypt_info(ci_info_temp); /* free ci_info_temp */
	return res;
}

int f2fs_get_sdp_crypt_info(struct inode *inode, void *fs_data)
{
	int res;
	u32 flag = 0;
	struct fscrypt_info *ci_info = NULL;

	if (!inode) /* fs_data can be null so not need to check */
		return -EINVAL;

	ci_info = inode->i_crypt_info;
	res = f2fs_inode_get_sdp_encrypt_flags(inode, fs_data, &flag);
	if (res != 0) {
		sdp_pr_err("%s get flag failed res:%d\n", __func__, res);
		return res;
	}

	if (ci_info && f2fs_inode_is_enabled_sdp_encryption(flag))
		return f2fs_inode_check_sdp_keyring(
			ci_info->ci_master_key_descriptor,
			ENHANCED_CHECK_KEYING);

	/* change from ce to sdp crypto */
	if (ci_info && f2fs_inode_is_config_sdp_encryption(flag))
		return f2fs_change_to_sdp_crypto(inode, fs_data);

	if (f2fs_inode_is_sdp_encrypted(flag))
		return do_get_sdp_crypt_info(inode, fs_data, flag);

	sdp_pr_err("%s flag is not enable sdp\n", __func__);
	return -EINVAL;
}

int f2fs_is_file_sdp_encrypted(struct inode *inode)
{
	u32 flag = 0;

	/* if the inode is not null, i_sb, s_cop is not null */
	if (!inode)
		return 0; /* In this condition 0 means not enable sdp */
	if (inode->i_sb->s_cop->get_sdp_encrypt_flags(inode, NULL, &flag))
		return 0;
	return f2fs_inode_is_sdp_encrypted(flag);
}

int f2fs_inode_check_sdp_keyring(const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE],
	int enforce)
{
	struct key *keyring_key = NULL;
	const struct user_key_payload *payload = NULL;
	struct fscrypt_sdp_key *master_sdp_key = NULL;
	int res = -EKEYREVOKED;
	u8 sdp_pri_key[FS_MAX_KEY_SIZE] = {0};

	if (!descriptor)
		return -EINVAL;
	keyring_key = fscrypt_request_key(descriptor,
		FS_KEY_DESC_PREFIX, FS_KEY_DESC_PREFIX_SIZE);
	if (IS_ERR(keyring_key)) {
		sdp_pr_err("%s request key failed\n", __func__);
		return res;
	}

	down_read(&keyring_key->sem);
	if (keyring_key->type != &key_type_logon) {
		sdp_pr_err("%s: key type must be logon\n", __func__);
		goto out;
	}
	payload = user_key_payload_locked(keyring_key);
	if (!payload)
		goto out;
	if (payload->datalen != sizeof(struct fscrypt_sdp_key)) {
		sdp_pr_err("%s: sdp full key size incorrect: %d\n",
			__func__, payload->datalen);
		goto out;
	}

	master_sdp_key = (struct fscrypt_sdp_key *)payload->data;
	if (master_sdp_key->sdp_class == FSCRYPT_SDP_SECE_CLASS) {
		if (enforce == ENHANCED_CHECK_KEYING) {
			if (master_sdp_key->size == 0 ||
				memcmp(master_sdp_key->raw, sdp_pri_key,
				master_sdp_key->size) == 0)
				goto out;
		}
	} else if (master_sdp_key->sdp_class != FSCRYPT_SDP_ECE_CLASS) {
		goto out;
	}

	res = 0;
out:
	up_read(&keyring_key->sem);
	key_put(keyring_key);
	return res;
}

int f2fs_inode_get_sdp_encrypt_flags(struct inode *inode, void *fs_data,
	u32 *flag)
{
	if (!inode || !flag) /* fs_data can be null, no need to check */
		return -EINVAL;
	/* if the inode is not null, i_sb, s_cop is not null */
	if (!inode->i_sb->s_cop->get_sdp_encrypt_flags)
		return -EOPNOTSUPP;
	return inode->i_sb->s_cop->get_sdp_encrypt_flags(inode, fs_data, flag);
}

int f2fs_inode_set_sdp_encryption_flags(struct inode *inode, void *fs_data,
	u32 sdp_enc_flag)
{
	int res;
	u32 flags = 0;

	if (!inode) /* fs_data can be null, no need to check */
		return -EINVAL;

	/* if the inode is not null, i_sb, s_cop is not null */
	if (!inode->i_sb->s_cop->get_sdp_encrypt_flags ||
		!inode->i_sb->s_cop->set_sdp_encrypt_flags)
		return -EOPNOTSUPP;

	if (sdp_enc_flag & (~0x0F)) /* get the lower 4 bits */
		return -EINVAL;

	res = inode->i_sb->s_cop->get_sdp_encrypt_flags(inode, fs_data, &flags);
	if (res != 0)
		return res;

	flags |= sdp_enc_flag;

	return inode->i_sb->s_cop->set_sdp_encrypt_flags(inode, fs_data,
		&flags);
}

#endif
