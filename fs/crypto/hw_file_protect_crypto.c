/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for crypto algorithm
 *              needed by hwsdp and hwdps.
 * Create: 2020-12-15
 */

#include <linux/scatterlist.h>
#include <crypto/aead.h>
#include <crypto/aes.h>
#include <crypto/skcipher.h>
#include <securec.h>
#include <fscrypt_private.h>

#define HWDPS_FEK_LEN 64

static void crypt_complete(struct crypto_async_request *req, int rc)
{
	struct fscrypt_completion_result *ecr = req->data;
	if (rc == -EINPROGRESS)
		return;
	ecr->res = rc;
	complete(&ecr->completion);
}

int file_protect_set_gcm_key(struct crypto_aead *tfm,
	const u8 *derive_key, u32 derive_key_len)
{
	int res;
	unsigned int iv_len;

	if (!tfm || !derive_key || derive_key_len != FS_AES_256_GCM_KEY_SIZE) {
		pr_err("%s params invalid len:%u\n", __func__, derive_key_len);
		return -EINVAL;
	}

	crypto_aead_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);

	iv_len = crypto_aead_ivsize(tfm);
	if (iv_len > FS_KEY_DERIVATION_IV_SIZE) {
		pr_err("%s : iv length is invalid:%u\n", __func__, iv_len);
		return -EINVAL;
	}

	res = crypto_aead_setauthsize(tfm, FS_KEY_DERIVATION_TAG_SIZE);
	if (res != 0) {
		pr_err("%s : fail to set authsize, res:%d\n", __func__, res);
		return res;
	}

	res = crypto_aead_setkey(tfm, derive_key, derive_key_len);
	if (res != 0)
		pr_err("%s : fail to set derive key, res:%d\n", __func__, res);
	return res;
}
EXPORT_SYMBOL(file_protect_set_gcm_key);

int file_protect_derive_gcm_key(struct crypto_aead *tfm,
	const u8 *src, u32 src_len, u8 *dst, u32 dst_len, u8 *iv, u32 iv_len,
	int enc)
{
	int res;
	struct scatterlist src_sg;
	struct scatterlist dst_sg;
	unsigned int len;
	struct aead_request *req = NULL;
	DECLARE_FS_COMPLETION_RESULT(ecr);
	u8 *src_buf = NULL;
	u8 *dst_buf = NULL;

	if (!tfm || !src || src_len !=
		FS_KEY_DERIVATION_CIPHER_SIZE || !dst ||
		dst_len != FS_KEY_DERIVATION_CIPHER_SIZE || !iv ||
		iv_len != FS_KEY_DERIVATION_IV_SIZE)
		return -EINVAL;

	req = aead_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto out;
	}

	src_buf =  kzalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!src) {
		res = -ENOMEM;
		goto out;
	}

	dst_buf =  kzalloc(FS_KEY_DERIVATION_CIPHER_SIZE, GFP_NOFS);
	if (!dst) {
		res = -ENOMEM;
		goto out;
	}

	aead_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		crypt_complete, &ecr);

	len = enc ? HWDPS_FEK_LEN :
		FS_KEY_DERIVATION_CIPHER_SIZE;
	(void)memcpy_s(src_buf, FS_KEY_DERIVATION_CIPHER_SIZE, src,
		FS_KEY_DERIVATION_CIPHER_SIZE);
	(void)memcpy_s(dst_buf, FS_KEY_DERIVATION_CIPHER_SIZE, dst,
		FS_KEY_DERIVATION_CIPHER_SIZE);

	sg_init_one(&src_sg, src_buf, FS_KEY_DERIVATION_CIPHER_SIZE);
	sg_init_one(&dst_sg, dst_buf, FS_KEY_DERIVATION_CIPHER_SIZE);

	aead_request_set_ad(req, 0);

	aead_request_set_crypt(req, &src_sg, &dst_sg, len, iv);
	res = enc ? crypto_aead_encrypt(req) : crypto_aead_decrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	if (res != 0)
		pr_err("%s encrypt or decrypt failed :%d\n", __func__, res);
	else
		(void)memcpy_s(dst, FS_KEY_DERIVATION_CIPHER_SIZE,
			dst_buf, FS_KEY_DERIVATION_CIPHER_SIZE);
out:
	if (req)
		aead_request_free(req);
	kzfree(src_buf);
	kzfree(dst_buf);
	return res;
}
EXPORT_SYMBOL(file_protect_derive_gcm_key);

