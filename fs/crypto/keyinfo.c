// SPDX-License-Identifier: GPL-2.0
/*
 * key management facility for FS encryption support.
 *
 * Copyright (C) 2015, Google, Inc.
 *
 * This contains encryption key functions.
 *
 * Written by Michael Halcrow, Ildar Muslukhov, and Uday Savagaonkar, 2015.
 */

#include <keys/user-type.h>
#include <linux/hashtable.h>
#include <linux/scatterlist.h>
#include <crypto/aead.h>
#include <linux/ratelimit.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/sha.h>
#include <crypto/skcipher.h>
#include <linux/hie.h>
#include "fscrypt_private.h"
#include <securec.h>
#ifdef CONFIG_HWDPS
#include <linux/security.h>
#include <huawei_platform/hwdps/hwdps_limits.h>
#include <huawei_platform/hwdps/hwdps_fs_hooks.h>
#define HWDPS_KEY_DESC_STANDARD_FLAG 0x42
#endif
#include "sdp_internal.h"

static struct crypto_shash *essiv_hash_tfm;

/* Table of keys referenced by FS_POLICY_FLAG_DIRECT_KEY policies */
static DEFINE_HASHTABLE(fscrypt_master_keys, 6); /* 6 bits = 64 buckets */
static DEFINE_SPINLOCK(fscrypt_master_keys_lock);
static void derive_crypt_complete(struct crypto_async_request *req, int rc)
{
	struct fscrypt_completion_result *ecr = req->data;
	if (rc == -EINPROGRESS)
		return;
	ecr->res = rc;
	complete(&ecr->completion);
}

int fscrypt_set_gcm_key(struct crypto_aead *tfm,
			const u8 *deriving_key)
{
	int res = 0;
	unsigned int iv_len;

	crypto_aead_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);

	iv_len = crypto_aead_ivsize(tfm);
	if (iv_len > FS_KEY_DERIVATION_IV_SIZE) {
		res = -EINVAL;
		pr_err("fscrypt %s : IV length is incompatible\n", __func__);
		goto out;
	}

	res = crypto_aead_setauthsize(tfm, FS_KEY_DERIVATION_TAG_SIZE);
	if (res < 0) {
		pr_err("fscrypt %s : Failed to set authsize\n", __func__);
		goto out;
	}

	res = crypto_aead_setkey(tfm, deriving_key,
					FS_AES_256_GCM_KEY_SIZE);
	if (res < 0)
		pr_err("fscrypt %s : Failed to set deriving key\n", __func__);
out:
	return res;
}

int fscrypt_derive_gcm_key(struct crypto_aead *tfm,
			 const u8 *source_key,
			       u8 *derived_key,
			       u8 iv[FS_KEY_DERIVATION_IV_SIZE],
			       int enc)
{
	int res = 0;
	struct aead_request *req = NULL;
	DECLARE_FS_COMPLETION_RESULT(ecr);
	struct scatterlist src_sg, dst_sg;
	unsigned int ilen;
	u8 *src = NULL;
	u8 *dst = NULL;

	if (!tfm) {
		res = -EINVAL;
		goto out;
	}

	if (IS_ERR(tfm)) {
		res = PTR_ERR(tfm);
		goto out;
	}

	req = aead_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}

	src =  kmalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!src) {
		res = -ENOMEM;
		goto out;
	}

	dst =  kmalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!dst) {
		res = -ENOMEM;
		goto out;
	}

	aead_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			derive_crypt_complete, &ecr);


	ilen = enc ? FS_KEY_DERIVATION_NONCE_SIZE :
			FS_KEY_DERIVATION_CIPHER_SIZE;
	memcpy(src, source_key, FS_KEY_DERIVATION_CIPHER_SIZE);
	memcpy(dst, derived_key, FS_KEY_DERIVATION_CIPHER_SIZE);

	sg_init_one(&src_sg, src, FS_KEY_DERIVATION_CIPHER_SIZE);
	sg_init_one(&dst_sg, dst, FS_KEY_DERIVATION_CIPHER_SIZE);

	aead_request_set_ad(req, 0);

	aead_request_set_crypt(req, &src_sg, &dst_sg, ilen, iv);
	res = enc ? crypto_aead_encrypt(req) : crypto_aead_decrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	memcpy(derived_key, dst, FS_KEY_DERIVATION_CIPHER_SIZE);
out:
	if (req)
		aead_request_free(req);
	if (src)
		kfree(src);
	if (dst)
		kfree(dst);
	return res;
}

/*
 * Key derivation function.  This generates the derived key by encrypting the
 * master key with AES-128-ECB using the inode's nonce as the AES key.
 *
 * The master key must be at least as long as the derived key.  If the master
 * key is longer, then only the first 'derived_keysize' bytes are used.
 */
static int derive_key_aes(const u8 *master_key,
			  const struct fscrypt_context *ctx,
			  u8 *derived_key, unsigned int derived_keysize)
{
	int res = 0;
	struct skcipher_request *req = NULL;
	DECLARE_CRYPTO_WAIT(wait);
	struct scatterlist src_sg, dst_sg;
	struct crypto_skcipher *tfm = crypto_alloc_skcipher("ecb(aes)", 0, 0);

	if (IS_ERR(tfm)) {
		res = PTR_ERR(tfm);
		tfm = NULL;
		goto out;
	}
	crypto_skcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	req = skcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}
	skcipher_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			crypto_req_done, &wait);
#ifndef CONFIG_FS_UNI_ENCRYPTION
	res = crypto_skcipher_setkey(tfm, ctx->nonce, sizeof(ctx->nonce));
#else
	res = crypto_skcipher_setkey(tfm, ctx->nonce, 16);
#endif
	if (res < 0)
		goto out;

	sg_init_one(&src_sg, master_key, derived_keysize);
	sg_init_one(&dst_sg, derived_key, derived_keysize);
	skcipher_request_set_crypt(req, &src_sg, &dst_sg, derived_keysize,
				   NULL);
	res = crypto_wait_req(crypto_skcipher_encrypt(req), &wait);
out:
	skcipher_request_free(req);
	crypto_free_skcipher(tfm);
	return res;
}


struct key *fscrypt_request_key(const u8 *descriptor, const u8 *prefix,
				int prefix_size)
{
	u8 *full_key_descriptor;
	struct key *keyring_key = NULL;
	int full_key_len = prefix_size + (FS_KEY_DESCRIPTOR_SIZE * 2) + 1;

	full_key_descriptor = kmalloc(full_key_len, GFP_NOFS);
	if (!full_key_descriptor)
		return (struct key *)ERR_PTR(-ENOMEM);

	memcpy(full_key_descriptor, prefix, prefix_size);
	sprintf(full_key_descriptor + prefix_size,
		"%*phN", FS_KEY_DESCRIPTOR_SIZE,
		descriptor);
	full_key_descriptor[full_key_len - 1] = '\0';
	keyring_key = request_key(&key_type_logon, full_key_descriptor, NULL);
	kfree(full_key_descriptor);

	return keyring_key;
}

/*
 * Search the current task's subscribed keyrings for a "logon" key with
 * description prefix:descriptor, and if found acquire a read lock on it and
 * return a pointer to its validated payload in *payload_ret.
 */
static struct key *
find_and_lock_process_key(const struct fscrypt_context *ctx,
			  const char *prefix,
			  const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE],
			  unsigned int min_keysize,
			  const struct fscrypt_key **payload_ret)
{
	struct key *key;
	const struct user_key_payload *ukp;
	int prefix_size = strlen(prefix);
	struct fscrypt_key *payload;

	key = fscrypt_request_key(ctx->master_key_descriptor,
					prefix, prefix_size);
	if (IS_ERR(key))
		return key;

	down_read(&key->sem);
	ukp = user_key_payload_locked(key);

	if (!ukp) /* was the key revoked before we acquired its semaphore? */
		goto invalid;

	payload = (struct fscrypt_key *)ukp->data;

#ifdef CONFIG_HIE_DEBUG
	if (hie_debug(HIE_DBG_FS))
		pr_info("HIE: %s: prefix:%s ci:%p, payload:%p, size:%d, mode:%d, min_keysize:%d\n",
			__func__, prefix, payload,
			payload->size, payload->mode, min_keysize);
#endif

	if (ukp->datalen != sizeof(struct fscrypt_key) ||
	    payload->size < 1 || payload->size > FS_MAX_KEY_SIZE) {
		fscrypt_warn(NULL,
			     "key with description '%s' has invalid payload",
			     key->description);
		goto invalid;
	}

	//force the size equal to FS_AES_256_GCM_KEY_SIZE since user space might pass FS_AES_256_XTS_KEY_SIZE
#ifdef CONFIG_FS_UNI_ENCRYPTION
	payload->size = FS_AES_256_GCM_KEY_SIZE;
	if (payload->size != FS_AES_256_GCM_KEY_SIZE) {
#else
	if (payload->size < min_keysize) {
#endif
		fscrypt_warn(NULL,
			     "key with description '%s' is too short (got %u bytes, need %u+ bytes)",
			     key->description, payload->size, min_keysize);
		goto invalid;
	}

	*payload_ret = payload;
	return key;

invalid:
	up_read(&key->sem);
	key_put(key);
	return ERR_PTR(-ENOKEY);
}

static struct fscrypt_mode available_modes[] = {
	[FS_ENCRYPTION_MODE_AES_256_XTS] = {
		.friendly_name = "AES-256-XTS",
		.cipher_str = "xts(aes)",
		.keysize = 64,
		.ivsize = 16,
	},
	[FS_ENCRYPTION_MODE_AES_256_CTS] = {
		.friendly_name = "AES-256-CTS-CBC",
		.cipher_str = "cts(cbc(aes))",
		.keysize = 32,
		.ivsize = 16,
	},
	[FS_ENCRYPTION_MODE_AES_128_CBC] = {
		.friendly_name = "AES-128-CBC",
		.cipher_str = "cbc(aes)",
		.keysize = 16,
		.ivsize = 16,
		.needs_essiv = true,
	},
	[FS_ENCRYPTION_MODE_AES_128_CTS] = {
		.friendly_name = "AES-128-CTS-CBC",
		.cipher_str = "cts(cbc(aes))",
		.keysize = 16,
		.ivsize = 16,
	},
	[FS_ENCRYPTION_MODE_ADIANTUM] = {
		.friendly_name = "Adiantum",
		.cipher_str = "adiantum(xchacha12,aes)",
		.keysize = 32,
		.ivsize = 32,
	},
};

struct fscrypt_mode *select_encryption_mode(struct fscrypt_info *ci,
	const struct inode *inode)
{
	if (!fscrypt_valid_enc_modes(ci->ci_data_mode, ci->ci_filename_mode)) {
		fscrypt_warn(inode->i_sb,
			     "inode %lu uses unsupported encryption modes (contents mode %d, filenames mode %d)",
			     inode->i_ino, ci->ci_data_mode,
			     ci->ci_filename_mode);
		return ERR_PTR(-EINVAL);
	}

	if (S_ISREG(inode->i_mode)) {
		ci->ci_format = CI_DATA_MODE;
		/* HIE: default use aes-256-xts */
		if (ci->ci_data_mode == FS_ENCRYPTION_MODE_PRIVATE)
			return &available_modes[FS_ENCRYPTION_MODE_AES_256_XTS];
		return &available_modes[ci->ci_data_mode];
	}

	if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode)) {
		ci->ci_format = CI_FNAME_MODE;
		return &available_modes[ci->ci_filename_mode];
	}

	WARN_ONCE(1, "fscrypt: filesystem tried to load encryption info for inode %lu, which is not encryptable (file type %d)\n",
		  inode->i_ino, (inode->i_mode & S_IFMT));
	return ERR_PTR(-EINVAL);
}

/* Find the master key, then derive the inode's actual encryption key */
static int find_and_derive_key(struct fscrypt_info *crypt_info,
				   const struct inode *inode,
			       const struct fscrypt_context *ctx,
			       u8 *derived_key, const struct fscrypt_mode *mode)
{
	struct key *key;
	const struct fscrypt_key *payload;
	int err;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	u8 *plain_text = NULL;
	struct crypto_aead *tfm = NULL;
#endif

	key = find_and_lock_process_key(ctx, FS_KEY_DESC_PREFIX,
					ctx->master_key_descriptor,
					mode->keysize, &payload);
	if (key == ERR_PTR(-ENOKEY) && inode->i_sb->s_cop->key_prefix) {
		key = find_and_lock_process_key(ctx, inode->i_sb->s_cop->key_prefix,
						ctx->master_key_descriptor,
						mode->keysize, &payload);
	}
	if (IS_ERR(key))
		return PTR_ERR(key);

	memcpy(crypt_info->ci_raw_key,
	       payload->raw, sizeof(crypt_info->ci_raw_key));

#ifdef CONFIG_FS_UNI_ENCRYPTION
	crypt_info->ci_key_index = (int) *(payload->raw + 63) & 0xff;
	plain_text = kmalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!plain_text)
		goto out;
	tfm = (struct crypto_aead *)crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm)) {
		up_read(&key->sem);
		err = PTR_ERR(tfm);
		tfm = NULL;
		pr_err("fscrypt %s : tfm allocation failed!\n", __func__);
		goto out;
	}

	err = fscrypt_set_gcm_key(tfm, payload->raw);
	if (err)
		goto out;

	err = fscrypt_derive_gcm_key(tfm, (u8 *)ctx->nonce, plain_text, (u8 *) ctx->iv, 0);
	if (err)
		goto out;
	memcpy(derived_key, plain_text, FS_KEY_DERIVATION_NONCE_SIZE);
	kfree(plain_text);
	crypt_info->ci_gtfm = tfm;
	up_read(&key->sem);
	key_put(key);
	return 0;
#else
	if (ctx->flags & FS_POLICY_FLAG_DIRECT_KEY) {
		if (mode->ivsize < offsetofend(union fscrypt_iv, nonce)) {
			fscrypt_warn(inode->i_sb,
				     "direct key mode not allowed with %s",
				     mode->friendly_name);
			err = -EINVAL;
		} else if (ctx->contents_encryption_mode !=
			   ctx->filenames_encryption_mode) {
			fscrypt_warn(inode->i_sb,
				     "direct key mode not allowed with different contents and filenames modes");
			err = -EINVAL;
		} else {
			memcpy(derived_key, payload->raw, mode->keysize);
			err = 0;
		}
	} else {
		if (!fscrypt_is_private_mode(crypt_info)) {
			err = derive_key_aes(payload->raw, ctx, derived_key,
					     mode->keysize);
		} else
			err = 0;
	}
#endif
#ifdef CONFIG_FS_UNI_ENCRYPTION
out:
	if (tfm)
		crypto_free_aead(tfm);
	if (plain_text)
		kfree(plain_text);
#endif
	up_read(&key->sem);
	key_put(key);
	return err;
}

/* Allocate and key a symmetric cipher object for the given encryption mode */
static struct crypto_skcipher *
allocate_skcipher_for_mode(struct fscrypt_mode *mode, const u8 *raw_key,
			   const struct inode *inode)
{
	struct crypto_skcipher *tfm;
	int err;

	tfm = crypto_alloc_skcipher(mode->cipher_str, 0, 0);
	if (IS_ERR(tfm)) {
		fscrypt_warn(inode->i_sb,
			     "error allocating '%s' transform for inode %lu: %ld",
			     mode->cipher_str, inode->i_ino, PTR_ERR(tfm));
		return tfm;
	}
	if (unlikely(!mode->logged_impl_name)) {
		/*
		 * fscrypt performance can vary greatly depending on which
		 * crypto algorithm implementation is used.  Help people debug
		 * performance problems by logging the ->cra_driver_name the
		 * first time a mode is used.  Note that multiple threads can
		 * race here, but it doesn't really matter.
		 */
		mode->logged_impl_name = true;
		pr_info("fscrypt: %s using implementation \"%s\"\n",
			mode->friendly_name,
			crypto_skcipher_alg(tfm)->base.cra_driver_name);
	}
	crypto_skcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	err = crypto_skcipher_setkey(tfm, raw_key, mode->keysize);
	if (err)
		goto err_free_tfm;

	return tfm;

err_free_tfm:
	crypto_free_skcipher(tfm);
	return ERR_PTR(err);
}

/* Master key referenced by FS_POLICY_FLAG_DIRECT_KEY policy */
struct fscrypt_master_key {
	struct hlist_node mk_node;
	refcount_t mk_refcount;
	const struct fscrypt_mode *mk_mode;
	struct crypto_skcipher *mk_ctfm;
	u8 mk_descriptor[FS_KEY_DESCRIPTOR_SIZE];
	u8 mk_raw[FS_MAX_KEY_SIZE];
};

static void free_master_key(struct fscrypt_master_key *mk)
{
	if (mk) {
		crypto_free_skcipher(mk->mk_ctfm);
		kzfree(mk);
	}
}

static void put_master_key(struct fscrypt_master_key *mk)
{
	if (!refcount_dec_and_lock(&mk->mk_refcount, &fscrypt_master_keys_lock))
		return;
	hash_del(&mk->mk_node);
	spin_unlock(&fscrypt_master_keys_lock);
	free_master_key(mk);
}

/*
 * Find/insert the given master key into the fscrypt_master_keys table.  If
 * found, it is returned with elevated refcount, and 'to_insert' is freed if
 * non-NULL.  If not found, 'to_insert' is inserted and returned if it's
 * non-NULL; otherwise NULL is returned.
 */
static struct fscrypt_master_key *
find_or_insert_master_key(struct fscrypt_master_key *to_insert,
			  const u8 *raw_key, const struct fscrypt_mode *mode,
			  const struct fscrypt_info *ci)
{
	unsigned long hash_key;
	struct fscrypt_master_key *mk;

	/*
	 * Careful: to avoid potentially leaking secret key bytes via timing
	 * information, we must key the hash table by descriptor rather than by
	 * raw key, and use crypto_memneq() when comparing raw keys.
	 */

	BUILD_BUG_ON(sizeof(hash_key) > FS_KEY_DESCRIPTOR_SIZE);
	memcpy(&hash_key, ci->ci_master_key_descriptor, sizeof(hash_key));

	spin_lock(&fscrypt_master_keys_lock);
	hash_for_each_possible(fscrypt_master_keys, mk, mk_node, hash_key) {
		if (memcmp(ci->ci_master_key_descriptor, mk->mk_descriptor,
			   FS_KEY_DESCRIPTOR_SIZE) != 0)
			continue;
		if (mode != mk->mk_mode)
			continue;
		if (crypto_memneq(raw_key, mk->mk_raw, mode->keysize))
			continue;
		/* using existing tfm with same (descriptor, mode, raw_key) */
		refcount_inc(&mk->mk_refcount);
		spin_unlock(&fscrypt_master_keys_lock);
		free_master_key(to_insert);
		return mk;
	}
	if (to_insert)
		hash_add(fscrypt_master_keys, &to_insert->mk_node, hash_key);
	spin_unlock(&fscrypt_master_keys_lock);
	return to_insert;
}

/* Prepare to encrypt directly using the master key in the given mode */
static struct fscrypt_master_key *
fscrypt_get_master_key(const struct fscrypt_info *ci, struct fscrypt_mode *mode,
		       const u8 *raw_key, const struct inode *inode)
{
	struct fscrypt_master_key *mk;
	int err;

	/* Is there already a tfm for this key? */
	mk = find_or_insert_master_key(NULL, raw_key, mode, ci);
	if (mk)
		return mk;

	/* Nope, allocate one. */
	mk = kzalloc(sizeof(*mk), GFP_NOFS);
	if (!mk)
		return ERR_PTR(-ENOMEM);
	refcount_set(&mk->mk_refcount, 1);
	mk->mk_mode = mode;
	mk->mk_ctfm = allocate_skcipher_for_mode(mode, raw_key, inode);
	if (IS_ERR(mk->mk_ctfm)) {
		err = PTR_ERR(mk->mk_ctfm);
		mk->mk_ctfm = NULL;
		goto err_free_mk;
	}
	memcpy(mk->mk_descriptor, ci->ci_master_key_descriptor,
	       FS_KEY_DESCRIPTOR_SIZE);
	memcpy(mk->mk_raw, raw_key, mode->keysize);

	return find_or_insert_master_key(mk, raw_key, mode, ci);

err_free_mk:
	free_master_key(mk);
	return ERR_PTR(err);
}

static int derive_essiv_salt(const u8 *key, int keysize, u8 *salt)
{
	struct crypto_shash *tfm = READ_ONCE(essiv_hash_tfm);

	/* init hash transform on demand */
	if (unlikely(!tfm)) {
		struct crypto_shash *prev_tfm;

		tfm = crypto_alloc_shash("sha256", 0, 0);
		if (IS_ERR(tfm)) {
			fscrypt_warn(NULL,
				     "error allocating SHA-256 transform: %ld",
				     PTR_ERR(tfm));
			return PTR_ERR(tfm);
		}
		prev_tfm = cmpxchg(&essiv_hash_tfm, NULL, tfm);
		if (prev_tfm) {
			crypto_free_shash(tfm);
			tfm = prev_tfm;
		}
	}

	{
		SHASH_DESC_ON_STACK(desc, tfm);
		desc->tfm = tfm;
		desc->flags = 0;

		return crypto_shash_digest(desc, key, keysize, salt);
	}
}

static int init_essiv_generator(struct fscrypt_info *ci, const u8 *raw_key,
				int keysize)
{
	int err;
	struct crypto_cipher *essiv_tfm;
	u8 salt[SHA256_DIGEST_SIZE];

	essiv_tfm = crypto_alloc_cipher("aes", 0, 0);
	if (IS_ERR(essiv_tfm))
		return PTR_ERR(essiv_tfm);

	ci->ci_essiv_tfm = essiv_tfm;

	err = derive_essiv_salt(raw_key, keysize, salt);
	if (err)
		goto out;

	/*
	 * Using SHA256 to derive the salt/key will result in AES-256 being
	 * used for IV generation. File contents encryption will still use the
	 * configured keysize (AES-128) nevertheless.
	 */
	err = crypto_cipher_setkey(essiv_tfm, salt, sizeof(salt));
	if (err)
		goto out;

out:
	memzero_explicit(salt, sizeof(salt));
	return err;
}

void __exit fscrypt_essiv_cleanup(void)
{
	crypto_free_shash(essiv_hash_tfm);
}

u8 fscrypt_data_crypt_mode(const struct inode *inode, u8 mode)
{
	if (mode == FS_ENCRYPTION_MODE_INVALID)
		return FS_ENCRYPTION_MODE_INVALID;

	return hie_is_capable(inode->i_sb) ?
		FS_ENCRYPTION_MODE_PRIVATE : mode;
}
/*
 * Given the encryption mode and key (normally the derived key, but for
 * FS_POLICY_FLAG_DIRECT_KEY mode it's the master key), set up the inode's
 * symmetric cipher transform object(s).
 */
static int setup_crypto_transform(struct fscrypt_info *ci,
				  struct fscrypt_mode *mode,
				  const u8 *raw_key, const struct inode *inode)
{
	struct fscrypt_master_key *mk;
	struct crypto_skcipher *ctfm;
	int err;

	if (ci->ci_flags & FS_POLICY_FLAG_DIRECT_KEY) {
		mk = fscrypt_get_master_key(ci, mode, raw_key, inode);
		if (IS_ERR(mk))
			return PTR_ERR(mk);
		ctfm = mk->mk_ctfm;
	} else {
		mk = NULL;
		ctfm = allocate_skcipher_for_mode(mode, raw_key, inode);
		if (IS_ERR(ctfm))
			return PTR_ERR(ctfm);
	}
	ci->ci_master_key = mk;
	ci->ci_ctfm = ctfm;

	if (mode->needs_essiv) {
		/* ESSIV implies 16-byte IVs which implies !DIRECT_KEY */
		WARN_ON(mode->ivsize != AES_BLOCK_SIZE);
		WARN_ON(ci->ci_flags & FS_POLICY_FLAG_DIRECT_KEY);

		err = init_essiv_generator(ci, raw_key, mode->keysize);
		if (err) {
			fscrypt_warn(inode->i_sb,
				     "error initializing ESSIV generator for inode %lu: %d",
				     inode->i_ino, err);
			return err;
		}
	}
	return 0;
}

void put_crypt_info(struct fscrypt_info *ci)
{
#ifdef CONFIG_FS_UNI_ENCRYPTION
	void *prev = NULL;
	void *key = NULL;
#endif
	if (!ci)
		return;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	/*lint -save -e529 -e438*/
	key = ACCESS_ONCE(ci->ci_key);
	/*lint -restore*/
	/*lint -save -e1072 -e747 -e50*/
	prev = cmpxchg(&ci->ci_key, key, NULL);
	/*lint -restore*/
	if (prev == key && key) {
		memzero_explicit(key, (size_t)FS_MAX_KEY_SIZE);
		kfree(key);
		ci->ci_key_len = 0;
	}
	if (ci->ci_gtfm)
		crypto_free_aead(ci->ci_gtfm);
#endif

	if (ci->ci_master_key) {
		put_master_key(ci->ci_master_key);
	} else {
		crypto_free_skcipher(ci->ci_ctfm);
		crypto_free_cipher(ci->ci_essiv_tfm);
	}
	kmem_cache_free(fscrypt_info_cachep, ci);
}

static void fscrypt_put_crypt_info(struct fscrypt_info *ci)
{
	unsigned long flags;

	if (!ci)
		return;

	/* only ci_count == 1, add lock protection */
	if (atomic_dec_and_lock_irqsafe(&ci->ci_count, &ci->ci_lock, &flags)) {
		ci->ci_status |= CI_FREEING;
		spin_unlock_irqrestore(&ci->ci_lock, flags);
		put_crypt_info(ci);
	}
}

struct fscrypt_info *fscrypt_get_crypt_info(struct fscrypt_info *ci,
	bool init)
{
	unsigned long flags;

	if (init) {
		spin_lock_init(&ci->ci_lock);
		atomic_set(&ci->ci_count, 0);
		ci->ci_status = 0;
	}

	spin_lock_irqsave(&ci->ci_lock, flags);
	if (!(ci->ci_status & CI_FREEING)) {
		atomic_inc(&ci->ci_count);
		spin_unlock_irqrestore(&ci->ci_lock, flags);
	} else {
		spin_unlock_irqrestore(&ci->ci_lock, flags);
		ci = NULL;
	}

	return ci;
}

void *fscrypt_crypt_info_act(void *ci, int act)
{
	struct fscrypt_info *fi;

	fi = (struct fscrypt_info *)ci;

	if (act & BIO_BC_INFO_GET)
		return fscrypt_get_crypt_info(ci, false);
	else if (act & BIO_BC_INFO_PUT)
		fscrypt_put_crypt_info(ci);

	return NULL;
}

int fscrypt_get_encryption_info(struct inode *inode)
{
	struct fscrypt_info *crypt_info;
	struct fscrypt_context ctx;
	struct fscrypt_mode *mode;
	u8 *raw_key = NULL;
	int res;
	int flag = 0;

	/* Hwdps file needs to check access control begin. */
	if (inode->i_crypt_info && (inode->i_crypt_info->ci_hw_enc_flag == 0))
		return 0;
	/* Hwdps file needs to check access control end. */

	res = fscrypt_initialize(inode->i_sb->s_cop->flags);
	if (res)
		return res;

	/* For hwdps protection begin. */
	if (inode->i_sb->s_cop && inode->i_sb->s_cop->get_keyinfo) {
		res = inode->i_sb->s_cop->get_keyinfo(inode, NULL, &flag);
		if (res != 0) { /* err case */
			pr_err("hwdps_sdp: get_keyinfo failed res:%d\n",
				res);
			return res;
		}
		if (flag != 0)
			return 0;
	}
	/* For hwdps protection end. */

	res = inode->i_sb->s_cop->get_context(inode, &ctx, sizeof(ctx));
	if (res < 0) {
		if (!fscrypt_dummy_context_enabled(inode) ||
		    IS_ENCRYPTED(inode))
			return res;
		/* Fake up a context for an unencrypted directory */
		memset(&ctx, 0, sizeof(ctx));
#ifdef CONFIG_FS_UNI_ENCRYPTION
		ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V2;
#else
		ctx.format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
#endif

		ctx.contents_encryption_mode = FS_ENCRYPTION_MODE_AES_256_XTS;
		ctx.filenames_encryption_mode = FS_ENCRYPTION_MODE_AES_256_CTS;
		memset(ctx.master_key_descriptor, 0x42, FS_KEY_DESCRIPTOR_SIZE);
	} else if (res != sizeof(ctx)) {
		return -EINVAL;
	}

#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (ctx.format != FS_ENCRYPTION_CONTEXT_FORMAT_V2)
		return -EINVAL;
#endif

	if (ctx.flags & ~FS_POLICY_FLAGS_VALID)
		return -EINVAL;

	crypt_info = kmem_cache_zalloc(fscrypt_info_cachep, GFP_NOFS);
	if (!crypt_info)
		return -ENOMEM;

	fscrypt_get_crypt_info(crypt_info, true);
	crypt_info->ci_flags = ctx.flags;
	crypt_info->ci_data_mode =
		fscrypt_data_crypt_mode(inode, ctx.contents_encryption_mode);
	crypt_info->ci_filename_mode = ctx.filenames_encryption_mode;
	crypt_info->ci_gtfm = NULL;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	crypt_info->ci_key = NULL;
	crypt_info->ci_key_len = 0;
	crypt_info->ci_key_index = -1;
#endif

	memcpy(crypt_info->ci_master_key_descriptor, ctx.master_key_descriptor,
	       FS_KEY_DESCRIPTOR_SIZE);
	memcpy(crypt_info->ci_nonce, ctx.nonce, FS_KEY_DERIVATION_NONCE_SIZE);
#ifdef CONFIG_HIE_DEBUG
	if (hie_debug(HIE_DBG_FS))
		pr_info("HIE: %s: inode: %p, %ld, res: %d, dmode: %d, fmode: %d\n",
			__func__, inode, inode->i_ino,
			res, crypt_info->ci_data_mode,
			crypt_info->ci_filename_mode);
#endif
	mode = select_encryption_mode(crypt_info, inode);
	if (IS_ERR(mode)) {
		res = PTR_ERR(mode);
		goto out;
	}
	WARN_ON(mode->ivsize > FSCRYPT_MAX_IV_SIZE);
	crypt_info->ci_mode = mode;

#ifdef CONFIG_HIE_DEBUG
	if (hie_debug(HIE_DBG_FS))
		pr_info("HIE: %s: fscrypt_mode<%s> key_size<%d>\n",
		__func__, mode->friendly_name, mode->keysize);
#endif

	/*
	 * This cannot be a stack buffer because it may be passed to the
	 * scatterlist crypto API as part of key derivation.
	 */
	res = -ENOMEM;
	raw_key = kmalloc(mode->keysize, GFP_NOFS);
	if (!raw_key)
		goto out;

	res = find_and_derive_key(crypt_info, inode, &ctx, raw_key, mode);
	if (res)
		goto out;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (S_ISREG(inode->i_mode) &&
			inode->i_sb->s_cop->is_inline_encrypted &&
			inode->i_sb->s_cop->is_inline_encrypted(inode)) {
		crypt_info->ci_key = kzalloc((size_t)FS_KEY_DERIVATION_NONCE_SIZE, GFP_NOFS);
		if (!crypt_info->ci_key) {
			res = -ENOMEM;
			goto out;
		}
		crypt_info->ci_key_len = FS_KEY_DERIVATION_NONCE_SIZE;
		/*lint -save -e732 -e747*/
		memcpy(crypt_info->ci_key, raw_key, crypt_info->ci_key_len);
		goto hw_encrypt_out;
		/*lint -restore*/
	}
#endif

	if (fscrypt_is_private_mode(crypt_info))
		goto hw_encrypt_out;

	res = setup_crypto_transform(crypt_info, mode, raw_key, inode);
	if (res)
		goto out;

hw_encrypt_out:
	if (cmpxchg(&inode->i_crypt_info, NULL, crypt_info) == NULL)
		crypt_info = NULL;
out:
	if (res == -ENOKEY)
		res = 0;
	put_crypt_info(crypt_info);
	kzfree(raw_key);
	return res;
}
EXPORT_SYMBOL(fscrypt_get_encryption_info);

void fscrypt_put_encryption_info(struct inode *inode)
{
	fscrypt_put_crypt_info(inode->i_crypt_info);
	inode->i_crypt_info = NULL;
}
EXPORT_SYMBOL(fscrypt_put_encryption_info);

#ifdef CONFIG_HWDPS
static int hwdps_do_get_context(struct inode *inode,
	struct fscrypt_context *ctx)
{
	int err;

	pr_info_once("%s enter\n", __func__);
	if (!inode || !inode->i_sb || !inode->i_sb->s_cop ||
		!inode->i_sb->s_cop->get_context || !ctx)
		return -EOPNOTSUPP;

	err = inode->i_sb->s_cop->get_context(inode, ctx, sizeof(*ctx));
	if (err < 0) {
		if (!fscrypt_dummy_context_enabled(inode) ||
			IS_ENCRYPTED(inode))
			return err;
		/* Fake up a context for an unencrypted directory. */
		if (memset_s(ctx, sizeof(struct fscrypt_context), 0,
			sizeof(struct fscrypt_context)) != EOK)
			return err;
#ifdef CONFIG_FS_UNI_ENCRYPTION
		ctx->format = FS_ENCRYPTION_CONTEXT_FORMAT_V2;
#else
		ctx->format = FS_ENCRYPTION_CONTEXT_FORMAT_V1;
#endif
		ctx->contents_encryption_mode = FS_ENCRYPTION_MODE_AES_256_XTS;
		ctx->filenames_encryption_mode = FS_ENCRYPTION_MODE_AES_256_CTS;
		if (memset_s(ctx->master_key_descriptor, FS_KEY_DESCRIPTOR_SIZE,
			HWDPS_KEY_DESC_STANDARD_FLAG,
			FS_KEY_DESCRIPTOR_SIZE) != EOK)
			return err;
	} else if (err != sizeof(*ctx)) {
		pr_err("hwdps ino %lu ctx size [%d : %lu]\n",
			inode->i_ino, err, sizeof(*ctx));
		return -EINVAL;
	}
#ifdef CONFIG_FS_UNI_ENCRYPTION
	if (ctx->format != FS_ENCRYPTION_CONTEXT_FORMAT_V2) {
		pr_err("hwdps format error, format %x", ctx->format);
		return -EINVAL;
	}
#endif
	if (ctx->flags & ~FS_POLICY_FLAGS_VALID)
		return -EINVAL;

	return 0;
}

#ifdef CONFIG_FS_UNI_ENCRYPTION
static int hwdps_get_index(struct inode *inode, const struct fscrypt_mode *mode,
	const struct fscrypt_context *ctx, struct fscrypt_info *ci)
{
	struct key *key = NULL;
	const struct fscrypt_key *payload = NULL;

	key = find_and_lock_process_key(ctx, FS_KEY_DESC_PREFIX,
		ctx->master_key_descriptor,
		mode->keysize, &payload);
	if (key == ERR_PTR(-ENOKEY) && inode->i_sb->s_cop->key_prefix)
		key = find_and_lock_process_key(ctx, inode->i_sb->s_cop->key_prefix,
		ctx->master_key_descriptor,
		mode->keysize, &payload);

	if (IS_ERR(key))
		return PTR_ERR(key);
	/* the payload->raw size is 64, 63 means get the last byte */
	ci->ci_key_index = (int) *(payload->raw + 63) & 0xff;
	up_read(&key->sem);
	key_put(key);
	if (ci->ci_key_index < 0 || ci->ci_key_index > 31) {
		pr_err("%s hwdps err index:%d", __func__, ci->ci_key_index);
		return -EFAULT;
	}
	return 0;
}

static int hwdps_set_cipher_inline_crypto(struct inode *inode,
	buffer_t *fek_data, struct fscrypt_info *ci,
	const struct fscrypt_context *ctx)
{
	int res = 0;

	if (S_ISREG(inode->i_mode) &&
		inode->i_sb->s_cop->is_inline_encrypted &&
		inode->i_sb->s_cop->is_inline_encrypted(inode)) {
		ci->ci_key = kzalloc(FS_KEY_DERIVATION_NONCE_SIZE, GFP_NOFS);
		if (!ci->ci_key)
			return -ENOMEM;
		ci->ci_key_len = FS_KEY_DERIVATION_NONCE_SIZE;
		if (memcpy_s(ci->ci_key, ci->ci_key_len, fek_data->data,
			fek_data->len) != EOK) {
			pr_err("%s memcpy failed, ci_len:%d, data_len:%u\n",
				__func__, ci->ci_key_len, fek_data->len);
			res = -EFAULT;
		}
	}
	if (res != 0) {
		kzfree(ci->ci_key);
		ci->ci_key = NULL;
	}
	return res;
}
#endif

static int find_and_derive_key_for_hwdps(struct fscrypt_info *crypt_info,
	const struct fscrypt_context *ctx, buffer_t *derived_key, buffer_t *fek,
	const struct fscrypt_mode *mode)
{
	int err = 0;

	if (memcpy_s(crypt_info->ci_raw_key, sizeof(crypt_info->ci_raw_key),
		fek->data, fek->len) != EOK) {
		pr_err("hwdps In derive, memcpy_s failed\n");
		return -EFAULT;
	}
	if (fscrypt_is_private_mode(crypt_info)) {
		crypt_info->ci_hw_enc_flag = (u8)(HWDPS_XATTR_ENABLE_FLAG_NEW);
		pr_info("hwdps set key success\n");
		return err;
	}

	if (ctx->flags & FS_POLICY_FLAG_DIRECT_KEY) {
		if (mode->ivsize < offsetofend(union fscrypt_iv, nonce)) {
			pr_err("hwdps direct key mode not allowed with %s",
				mode->friendly_name);
			err = -EINVAL;
		} else if (ctx->contents_encryption_mode !=
			ctx->filenames_encryption_mode) {
			pr_err("hwdps with diff contents and filenames modes");
			err = -EINVAL;
		} else {
			if (memcpy_s(derived_key->data, derived_key->len,
				fek->data, fek->len) != EOK) {
				pr_err("%s memcpy_s failed\n", __func__);
				err = -EFAULT;
			}
		}
	} else {
		if (!fscrypt_is_private_mode(crypt_info))
			err = derive_key_aes(fek->data, ctx, derived_key->data,
				mode->keysize);
	}
	return err;
}

static int hwdps_do_set_cipher(struct inode *inode,
	buffer_t *fek_data, struct fscrypt_info *ci,
	const struct fscrypt_context *ctx)
{
	struct fscrypt_mode *mode = select_encryption_mode(ci, inode);
	u8 *derived_key = NULL;
	int res;
	buffer_t derived_key_buf = { NULL, 0 };

	if (IS_ERR(mode))
		return -EINVAL;
	WARN_ON(mode->ivsize > FSCRYPT_MAX_IV_SIZE);
	ci->ci_mode = mode;

#ifdef CONFIG_FS_UNI_ENCRYPTION
	res = hwdps_get_index(inode, mode, ctx, ci);
	if (res != 0) {
		pr_err("hwdps get index failed, res:%d\n", res);
		goto free_out;
	}
	res = hwdps_set_cipher_inline_crypto(inode, fek_data, ci, ctx);
	if (res == 0) {
		pr_info("%s success\n", __func__);
		ci->ci_hw_enc_flag = (u8)(HWDPS_XATTR_ENABLE_FLAG_NEW);
	}
	goto free_out;
#endif
	derived_key = kmalloc(mode->keysize, GFP_NOFS);
	if (!derived_key)
		return -ENOMEM;

	derived_key_buf.data = derived_key;
	derived_key_buf.len = mode->keysize;
	res = find_and_derive_key_for_hwdps(ci, ctx, &derived_key_buf,
		fek_data, mode);
	if (res != 0) {
		pr_err("%s hwdps failed, res:%d\n", __func__, res);
		goto free_out;
	}
	if (fscrypt_is_private_mode(ci))
		goto free_out;

	res = setup_crypto_transform(ci, mode, derived_key, inode);
	if (res != 0) {
		pr_err("hwdps setup_crypto_transform failed, res:%d\n", res);
		goto free_out;
	}
	ci->ci_hw_enc_flag = (u8)(HWDPS_XATTR_ENABLE_FLAG_NEW);
free_out:
	kzfree(derived_key);
	return res;
}

static struct fscrypt_info *hwdps_get_fscrypt_info(struct fscrypt_context *ctx,
	struct inode *inode)
{
	struct fscrypt_info *ci = NULL;
	int err = hwdps_do_get_context(inode, ctx);

	if (err != 0) {
		pr_err("hwdps_do_get_context failed %d\n", err);
		return NULL;
	}
	ci = kmem_cache_alloc(fscrypt_info_cachep, GFP_NOFS);
	if (!ci)
		return NULL;
	fscrypt_get_crypt_info(ci, true);
	ci->ci_flags = ctx->flags;
	ci->ci_data_mode = fscrypt_data_crypt_mode(inode,
		ctx->contents_encryption_mode);
	ci->ci_filename_mode = ctx->filenames_encryption_mode;
	ci->ci_ctfm = NULL;
	ci->ci_essiv_tfm = NULL;
	ci->ci_master_key = NULL;
	ci->ci_gtfm = NULL;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	ci->ci_key = NULL;
	ci->ci_key_len = 0;
	ci->ci_key_index = -1;
#endif
	if (memcpy_s(ci->ci_master_key_descriptor,
		sizeof(ci->ci_master_key_descriptor),
		ctx->master_key_descriptor,
		sizeof(ctx->master_key_descriptor)) != EOK) {
		put_crypt_info(ci);
		pr_err("%s memcpy_s fail\n", __func__);
		return NULL;
	}
	/*
	 * In fbe2 the sizeof(ctx->nonce) is not equal to
	 * FS_KEY_DERIVATION_NONCE_SIZE
	 */
	if (memcpy_s(ci->ci_nonce, sizeof(ci->ci_nonce), ctx->nonce,
		FS_KEY_DERIVATION_NONCE_SIZE) != EOK) {
		put_crypt_info(ci);
		pr_err("%s memcpy_s 2 failed\n", __func__);
		return NULL;
	}
	return ci;
}

static int hwdps_get_context_inner(struct inode *inode,
	struct fscrypt_context *ctx,
	secondary_buffer_t *fek_buffer, buffer_t *encoded_wfek, u32 flags)
{
	int err = hwdps_get_fek(ctx->master_key_descriptor, inode, encoded_wfek,
		fek_buffer, flags);
	if (err != 0) {
		pr_err("hwdps ino %lu get fek err %d\n", inode->i_ino, err);
		return err;
	}
	if (*(fek_buffer->len) > HWDPS_FEK_SIZE_MAX) {
		pr_err("hwdps fek length too large %u\n", *(fek_buffer->len));
		err = -EINVAL;
	}
	return err;
}

int hwdps_update_context(struct inode *inode, uid_t new_uid)
{
	int err;
	struct fscrypt_context ctx;
	uint8_t *encoded_wfek = NULL;
	uint8_t *fek = NULL;
	uint32_t fek_len;
	secondary_buffer_t fek_buffer = { &fek, &fek_len };
	buffer_t encoded_wfek_buffer = { NULL, 0 };
	uint32_t flags = 0;

	if (hwdps_check_support(inode, &flags) != 0)
		return 0;
	encoded_wfek = hwdps_do_get_attr(inode, HWDPS_ENCODED_WFEK_SIZE, flags);
	if (!encoded_wfek) {
		pr_err("hwdps_do_get_attr failed\n");
		return -ENOMEM;
	}
	encoded_wfek_buffer.data = encoded_wfek;
	encoded_wfek_buffer.len = HWDPS_ENCODED_WFEK_SIZE;
	err = hwdps_has_access(inode, &encoded_wfek_buffer, flags);
	if (err != 0) {
		pr_err("hwdps_has_access %d\n", err);
		goto free_encoded_wfek;
	}
	err = hwdps_do_get_context(inode, &ctx);
	if (err != 0) {
		pr_err("hwdps_do_get_context failed %d\n", err);
		goto free_encoded_wfek;
	}
	err = hwdps_update_fek(ctx.master_key_descriptor, &encoded_wfek_buffer,
		&fek_buffer, new_uid, inode->i_uid.val);
	if (err != 0) {
		pr_err("hwdps ino %lu update_fek err %d\n", inode->i_ino, err);
		hiview_for_hwdps_update(inode, err, new_uid);
		goto free_fek;
	}
	if (inode->i_sb->s_cop->update_hwdps_attr)
		err = inode->i_sb->s_cop->update_hwdps_attr(inode, encoded_wfek,
			HWDPS_ENCODED_WFEK_SIZE, NULL);
	else
		err = -EINVAL;
	if (err != 0)
		pr_err("hwdps ino %lu updatexattr err %d\n", inode->i_ino, err);

free_fek:
	kzfree(fek);
	if (err == -ENOKEY)
		err = 0;
free_encoded_wfek:
	kzfree(encoded_wfek);
	return err;
}
EXPORT_SYMBOL(hwdps_update_context);

static int hwdps_get_attr_by_flag(u32 *flags, struct inode *inode, u8 **attr,
	u32 *len)
{
	int err = hwdps_check_support(inode, flags);
	if (err != 0)
		return err;

	if (*flags == HWDPS_XATTR_ENABLE_FLAG) {
		*attr = hwdps_do_get_attr(inode,
			HWDPS_ENCODED_WFEK_SIZE_V2, *flags);
		*len = HWDPS_ENCODED_WFEK_SIZE_V2;
	} else {
		*attr = hwdps_do_get_attr(inode,
			HWDPS_ENCODED_WFEK_SIZE, *flags);
		*len = HWDPS_ENCODED_WFEK_SIZE;
	}
	if (!(*attr)) {
		pr_err("hwdps_do_get_attr failed\n");
		return -ENOMEM;
	}
	return 0;
}

int hwdps_get_context(struct inode *inode)
{
	int err;
	struct fscrypt_context ctx;
	struct fscrypt_info *ci = NULL;
	buffer_t encoded_wfek = { NULL, 0 };
	uint8_t *fek = NULL;
	uint32_t fek_len = 0;
	uint32_t flags = 0;
	secondary_buffer_t fek_buffer = { &fek, &fek_len };
	buffer_t fek_data = { NULL, 0 };

	err = hwdps_get_attr_by_flag(&flags, inode, &encoded_wfek.data,
		&encoded_wfek.len);
	if (err != 0)
		return err;
	if (inode->i_crypt_info) {
		err = hwdps_has_access(inode, &encoded_wfek, flags);
		goto free_fek;
	}
	ci = hwdps_get_fscrypt_info(&ctx, inode);
	if (!ci) {
		err = -EINVAL;
		goto free_fek;
	}
	err = hwdps_get_context_inner(inode, &ctx, &fek_buffer,
		&encoded_wfek, flags);
	if (err != 0)
		goto free_fek;
	fek_data.len = fek_len;
	fek_data.data = fek;
	err = hwdps_do_set_cipher(inode, &fek_data, ci, &ctx);
	if (err != 0) {
		pr_err("hwdps_do_set_cipher failed %d\n", err);
		goto free_fek;
	}
	if (cmpxchg(&inode->i_crypt_info, NULL, ci) == NULL)
		ci = NULL;
free_fek:
	kzfree(encoded_wfek.data);
	kzfree(fek);
	if (err == -ENOKEY)
		err = 0;
	put_crypt_info(ci);
	return err;
}
EXPORT_SYMBOL(hwdps_get_context);
#endif

#ifdef F2FS_FS_SDP_ENCRYPTION
#ifndef CONFIG_FS_UNI_ENCRYPTION
static int find_and_derive_key_for_sdp(struct fscrypt_info *crypt_info,
	const struct sdp_fscrypt_context *ctx, buffer_t *derived_key,
	buffer_t *fek, const struct fscrypt_mode *mode)
{
	int err = 0;

	if (memcpy_s(crypt_info->ci_raw_key, sizeof(crypt_info->ci_raw_key),
		fek->data, fek->len) != EOK) {
		pr_err("sdp In derive, memcpy_s failed\n");
		return -EFAULT;
	}
	if (fscrypt_is_private_mode(crypt_info)) {
		pr_info("sdp set key success\n");
		return err;
	}

	if (ctx->flags & FS_POLICY_FLAG_DIRECT_KEY) {
		if (mode->ivsize < offsetofend(union fscrypt_iv, nonce)) {
			pr_err("sdp direct key mode not allowed with %s",
				mode->friendly_name);
			err = -EINVAL;
		} else if (ctx->contents_encryption_mode !=
			ctx->filenames_encryption_mode) {
			pr_err("sdp with diff contents and filenames modes");
			err = -EINVAL;
		} else {
			if (memcpy_s(derived_key->data, derived_key->len,
				fek->data, fek->len) != EOK) {
				pr_err("%s memcpy_s failed\n", __func__);
				err = -EFAULT;
			}
		}
	} else {
		if (!fscrypt_is_private_mode(crypt_info))
			err = memcpy_s(derived_key->data, derived_key->len,
				fek->data, fek->len);
	}
	return err;
}

int set_cipher_for_sdp(struct fscrypt_info *ci,
	const struct sdp_fscrypt_context *ctx, u8 *fek, u32 fek_len,
	struct inode *inode)
{
	int res;
	u8 *derived_key = NULL;
	buffer_t derived_key_buf = { NULL, 0 };
	buffer_t fek_data = { NULL, 0 };
	struct fscrypt_mode *mode = NULL;

	if (!ci || !ctx || !fek || !inode)
		return -EINVAL;
	mode = ci->ci_mode;

	derived_key = kmalloc(mode->keysize, GFP_NOFS);
	if (!derived_key)
		return -ENOMEM;

	derived_key_buf.data = derived_key;
	derived_key_buf.len = mode->keysize;
	fek_data.data = fek;
	fek_data.len = fek_len;
	res = find_and_derive_key_for_sdp(ci, ctx, &derived_key_buf,
		&fek_data, mode);
	if (res != 0) {
		pr_err("sdp: %s hwdps failed, res:%d\n", __func__, res);
		goto free_out;
	}
	if (fscrypt_is_private_mode(ci))
		goto free_out;

	res = setup_crypto_transform(ci, mode, derived_key_buf.data, inode);
	if (res != 0)
		pr_err("sdp: setup_crypto_transform failed, res:%d\n", res);
free_out:
	kzfree(derived_key);
	return res;
}
#endif
#endif
