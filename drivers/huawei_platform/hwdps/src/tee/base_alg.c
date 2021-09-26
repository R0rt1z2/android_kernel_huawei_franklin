/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for operations about
 *              basic algorithm.
 * Create: 2020-06-16
 */

#include "inc/tee/base_alg.h"
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include <crypto/aead.h>
#include <securec.h>
#include <huawei_platform/hwdps/hwdps_error.h>
#include "inc/base/hwdps_utils.h"

#define HWDPS_CBC_AES_ALG "cbc(aes)"
#define HWDPS_HKDF_HMAC_ALG "hmac(sha256)"
#define HWDPS_HKDF_HASHLEN SHA256_DIGEST_SIZE
#define HWDPS_HMAC_TAG_LEN SHA256_DIGEST_SIZE
#define HWDPS_CBC_AES_MAX_INPUT 512
#define HWDPS_HKDF_MAX_INPUT_LEN 256
#define HWDPS_HKDF_MAX_OUTPUT_LEN 256
#define HWDPS_HMAC_MAX_MESSAGE_LEN 512
#define HWDPS_CRYPTO_ALLOC_SHASH 0
#define HWDPS_CRYPTO_ALLOC_MASK 0

struct tcrypt_result_t {
	struct completion completion;
	s32 err;
};

static void tcrypt_complete(struct crypto_async_request *req, s32 err)
{
	struct tcrypt_result_t *res = NULL;

	if ((err == -EINPROGRESS) || (req == NULL))
		return;
	res = req->data;
	if (!res)
		return;
	res->err = err;
	complete(&res->completion);
}

static s32 wait_async_op(s32 ret, struct tcrypt_result_t *tr)
{
	s32 result = ret;

	if ((result == -EINPROGRESS) || (result == -EBUSY)) {
		wait_for_completion(&tr->completion);
		reinit_completion(&tr->completion);
		result = tr->err;
	}
	return result;
}

static bool check_aes_cbc_para(buffer_t key, buffer_t iv,
	buffer_t in, out_buffer_t out)
{
	if (!key.data || (key.len != AES_KEYSIZE_256) || !iv.data ||
		(iv.len != AES_BLOCK_SIZE) || !in.data || (in.len == 0) ||
		((in.len % AES_BLOCK_SIZE) != 0) ||
		(in.len > HWDPS_CBC_AES_MAX_INPUT) || !out.len ||
		(*out.len < in.len))
		return false;
	return true;
}

/*
 * This function realize the aes cbc algorithm.
 * Which may exceed fifty lines.
 */
static s32 _aes_cbc_inter(buffer_t key, buffer_t iv,
	buffer_t in, out_buffer_t out, bool enc)
{
	s32 res;
	struct tcrypt_result_t result;
	struct scatterlist src_sg;
	struct scatterlist dst_sg;
	struct crypto_skcipher *tfm = NULL;
	struct skcipher_request *req = NULL;
	u8 *tmp_iv = NULL;
#ifdef CONFIG_VMAP_STACK
	u8 *in_dynamic = NULL;
	u8 *out_dynamic = NULL;
#endif

	if (!check_aes_cbc_para(key, iv, in, out)) {
		hwdps_pr_err("check_aes_cbc_para failed\n");
		return -EINVAL;
	}

#ifdef CONFIG_VMAP_STACK
	in_dynamic = kzalloc(in.len, GFP_NOFS);
	/* out len is no longer than in len */
	out_dynamic = kzalloc(in.len, GFP_NOFS);
	if (!in_dynamic || !out_dynamic) {
		res = -ENOMEM;
		hwdps_pr_err("in_dynamic or !out_dynamic failed\n");
		goto free_0;
	}
	if (memcpy_s(in_dynamic, in.len, in.data, in.len) != EOK) {
		res = -EINVAL;
		hwdps_pr_err("memcpy_s failed\n");
		goto free_0;
	}
#endif

	tfm = crypto_alloc_skcipher(HWDPS_CBC_AES_ALG, 0, 0);
	if (IS_ERR(tfm)) {
		res = PTR_ERR(tfm);
		hwdps_pr_err("alloc %s cipher failed res %d\n",
			HWDPS_CBC_AES_ALG, res);
#ifdef CONFIG_VMAP_STACK
		goto free_0;
#else
		return res;
#endif
	}

	init_completion(&result.completion);

	crypto_skcipher_set_flags(tfm, CRYPTO_TFM_REQ_WEAK_KEY);
	req = skcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		res = -ENOMEM;
		goto free_1;
	}
	skcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		tcrypt_complete, &result);
	res = crypto_skcipher_setkey(tfm, key.data, key.len);
	if (res < 0)
		goto free_2;
#ifdef CONFIG_VMAP_STACK
	sg_init_one(&src_sg, in_dynamic, in.len);
	sg_init_one(&dst_sg, out_dynamic, in.len);
#else
	sg_init_one(&src_sg, in.data, in.len);
	sg_init_one(&dst_sg, out.data, in.len);
#endif
	tmp_iv = kmalloc(AES_BLOCK_SIZE, GFP_NOFS);
	if (!tmp_iv) {
		res = -ENOMEM;
		goto free_2;
	}
	if (memcpy_s(tmp_iv, AES_BLOCK_SIZE, iv.data, iv.len) != EOK) {
		res = -EINVAL;
		kzfree(tmp_iv);
		goto free_2;
	}

	skcipher_request_set_crypt(req, &src_sg, &dst_sg, in.len, tmp_iv);

	if (enc)
		res = wait_async_op(crypto_skcipher_encrypt(req), &result);
	else
		res = wait_async_op(crypto_skcipher_decrypt(req), &result);

	if (res == 0) {
#ifdef CONFIG_VMAP_STACK
		if (memcpy_s(out.data, *out.len, out_dynamic, in.len) == EOK)
			*out.len = in.len;
		else {
			res = -EINVAL;
			hwdps_pr_err("memcpy_s esle failed\n");
		}
#else
		*out.len = in.len;
#endif
	}

	kzfree(tmp_iv);
free_2:
	skcipher_request_free(req);
free_1:
	crypto_free_skcipher(tfm);
#ifdef CONFIG_VMAP_STACK
free_0:
	kzfree(in_dynamic);
	kzfree(out_dynamic);
#endif
	return res;
}

static bool aes_cbc_check_para(buffer_t key, buffer_t iv,
	buffer_t in, secondary_buffer_t out)
{
	return (!key.data || (key.len != AES_KEYSIZE_256) ||
		!iv.data || (iv.len != AES_BLOCK_SIZE) || !in.data ||
		(in.len == 0) || (in.len > HWDPS_CBC_AES_MAX_INPUT) ||
		!out.data || !out.len);
}

s32 aes_cbc(buffer_t key, buffer_t iv, buffer_t in,
	secondary_buffer_t out, bool enc)
{
	s32 res;
	u8 *tmp_out = NULL;
	out_buffer_t temp_buffer = { NULL, NULL };

	if (aes_cbc_check_para(key, iv, in, out)) {
		hwdps_pr_err("%s aes_cbc_check_para failed\n", __func__);
		return -EINVAL;
	}

	if ((in.len % AES_BLOCK_SIZE) != 0 || in.len <= 0) {
		hwdps_pr_err("in_len %u\n", in.len);
		res = -EINVAL;
		goto out;
	}
	tmp_out = kmalloc(in.len, GFP_NOFS);
	if (!tmp_out) {
		res = -ENOMEM;
		goto out;
	}
	*out.len = in.len;

	temp_buffer.data = tmp_out;
	temp_buffer.len = out.len;

	res = _aes_cbc_inter(key, iv, in, temp_buffer, enc);
	if (res != 0) {
		hwdps_pr_err("_aes_cbc_inter failed %d\n", res);
		goto out;
	}

	*out.data = tmp_out;
	tmp_out = NULL;
out:
	/* if NULL, kzfree does nothing. */
	kzfree(tmp_out);
	tmp_out = NULL;

	return res;
}

static s32 shash_digest(struct crypto_shash *hmac_alg, buffer_t *salt,
	buffer_t *ikm, u8 *prk)
{
	SHASH_DESC_ON_STACK(desc, hmac_alg);
	s32 res;

	desc->tfm = hmac_alg;
	desc->flags = 0;
	res = crypto_shash_setkey(hmac_alg, salt->data, salt->len);
	if (res != 0)
		goto free;
	res = crypto_shash_digest(desc, ikm->data, ikm->len, prk);
free:
	shash_desc_zero(desc);
	return res;
}

s32 hash_generate_mac(buffer_t *hmac_key, buffer_t *msg, buffer_t *tag)
{
	s32 res;
	struct crypto_shash *hmac_tfm = NULL;

	if (!hmac_key || !hmac_key->data ||
		(hmac_key->len != HWDPS_HKDF_HASHLEN) ||
		!msg || !msg->data || (msg->len == 0) ||
		(msg->len > HWDPS_HMAC_MAX_MESSAGE_LEN) ||
		!tag || !tag->data || (tag->len != HWDPS_HMAC_TAG_LEN))
		return -EINVAL;

	hmac_tfm = crypto_alloc_shash(HWDPS_HKDF_HMAC_ALG,
		HWDPS_CRYPTO_ALLOC_SHASH,
		HWDPS_CRYPTO_ALLOC_MASK);
	if (IS_ERR(hmac_tfm)) {
		res = (s32)PTR_ERR(hmac_tfm);
		return res;
	}

	if (crypto_shash_digestsize(hmac_tfm) != tag->len) {
		crypto_free_shash(hmac_tfm);
		return -EINVAL;
	}
	res = shash_digest(hmac_tfm, hmac_key, msg, tag->data);
	crypto_free_shash(hmac_tfm);
	return res;
}
