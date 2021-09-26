/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for operations about
 *              hwdps algorithm.
 * Create: 2020-06-16
 */

#include "inc/tee/hwdps_alg.h"
#include <linux/key.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/string.h>
#include <securec.h>
#include "inc/base/hwdps_utils.h"
#include "inc/tee/base_alg.h"

static s32 hwdps_dec_enc_fek_check_param(buffer_t aes_key, buffer_t cipher,
	buffer_t plaintext_fek)
{
	if (!aes_key.data || !cipher.data || !plaintext_fek.data)
		return -HWDPS_ERR_INVALID_ARGS;

	if (aes_key.len != FS_AES_256_GCM_KEY_SIZE) {
		hwdps_pr_err("%s aes len err:%u\n", __func__, aes_key.len);
		return -HWDPS_ERR_INVALID_ARGS;
	}

	if (cipher.len != PHASE3_CIPHERTEXT_LENGTH) {
		hwdps_pr_err("%s cipher len err:%u\n", __func__, cipher.len);
		return -HWDPS_ERR_INVALID_ARGS;
	}

	if (plaintext_fek.len != FEK_LENGTH) {
		hwdps_pr_err("%s plaintext len err:%u\n", __func__,
			plaintext_fek.len);
		return -HWDPS_ERR_INVALID_ARGS;
	}
	return HWDPS_SUCCESS;
}

s32 hwdps_generate_dec_cbc(buffer_t aes_key,
	buffer_t ciphertext, buffer_t plaintext_fek)
{
	s32 ret;
	struct xattribs_t *xattr = NULL;
	u8 *de_ret = NULL;
	secondary_buffer_t ret_out = { &de_ret, &plaintext_fek.len };
	buffer_t iv = { NULL, 0 };
	buffer_t enc_fek = { NULL, 0 };

	if (!aes_key.data || aes_key.len != AES256_KEY_LEN)
		return -EINVAL;

	if (!plaintext_fek.data || (plaintext_fek.len != FEK_LENGTH))
		return -EINVAL;

	if (!ciphertext.data || (ciphertext.len != sizeof(struct xattribs_t))) {
		hwdps_pr_err("%s, ciphertext err, length:%u", __func__,
			ciphertext.len);
		return -EINVAL;
	}

	xattr = (struct xattribs_t *)ciphertext.data;
	if (!xattr)
		return -EINVAL;
	iv.data = xattr->iv;
	iv.len = sizeof(xattr->iv);
	enc_fek.data = xattr->enc_fek;
	enc_fek.len = sizeof(xattr->enc_fek);

	ret = aes_cbc(aes_key, iv, enc_fek, ret_out, false);
	if ((ret != 0) || (plaintext_fek.len != FEK_LENGTH)) {
		kzfree(de_ret);
		return (ret == 0) ? -EINVAL : ret;
	}

	if (memcpy_s(plaintext_fek.data, plaintext_fek.len,
		de_ret, FEK_LENGTH) != EOK) {
		kzfree(de_ret);
		return -EINVAL;
	}
	kzfree(de_ret);
	return ret;
}

s32 hwdps_generate_dec(buffer_t aes_key,
	buffer_t ciphertext, buffer_t plaintext_fek)
{
	s32 ret;
	struct xattribs_v3_t *xattr = NULL;
	struct crypto_aead *tfm = NULL;
	u8 plaintext_fek_buf[FS_KEY_DERIVATION_CIPHER_SIZE] = {0};

	hwdps_pr_debug("%s enter\n", __func__);

	ret = hwdps_dec_enc_fek_check_param(aes_key, ciphertext, plaintext_fek);
	if (ret != 0)
		return ret;
	xattr = (struct xattribs_v3_t *)ciphertext.data;

	tfm = crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm))
		return (int)PTR_ERR(tfm);

	ret = file_protect_set_gcm_key(tfm, aes_key.data, aes_key.len);
	if (ret != 0) {
		hwdps_pr_err("%s, set key failed, ret:%d", __func__, ret);
		goto out;
	}

	ret = file_protect_derive_gcm_key(tfm,
		xattr->enc_fek, sizeof(xattr->enc_fek), plaintext_fek_buf,
		sizeof(plaintext_fek_buf), xattr->iv, sizeof(xattr->iv),
		0); /* 0 means decrypt */
	if (ret != 0) {
		hwdps_pr_err("%s, derive key failed, ret:%d", __func__, ret);
		goto out;
	}
	if (memcpy_s(plaintext_fek.data, plaintext_fek.len,
		plaintext_fek_buf,  plaintext_fek.len) != EOK) {
		hwdps_pr_err("%s, memcpy failed\n", __func__);
		ret = -HWDPS_ERR_INTERNAL;
	}
	hwdps_pr_debug("%s end\n", __func__);
out:
	crypto_free_aead(tfm);
	(void)memset_s(plaintext_fek_buf, sizeof(plaintext_fek_buf), 0,
		sizeof(plaintext_fek_buf));
	return ret;
}

static s32 hwdps_enc_fek(buffer_t aes_key, buffer_t ciphertext,
	buffer_t plaintext_fek)
{
	s32 ret;
	struct xattribs_v3_t *xattr = NULL;
	u32 fek_temp_len = FS_KEY_DERIVATION_CIPHER_SIZE;
	u8 fek_temp[FS_KEY_DERIVATION_CIPHER_SIZE] = {0};
	struct crypto_aead *tfm = NULL;

	hwdps_pr_debug("%s enter gcm\n", __func__);
	ret = hwdps_dec_enc_fek_check_param(aes_key, ciphertext, plaintext_fek);
	if (ret != HWDPS_SUCCESS)
		return ret;

	xattr = (struct xattribs_v3_t *)ciphertext.data;
	xattr->version[0] = VERSION_3;
	get_random_bytes(xattr->iv, sizeof(xattr->iv));

	if (memcpy_s(fek_temp, fek_temp_len, plaintext_fek.data,
		plaintext_fek.len) != EOK) {
		hwdps_pr_err("memcpy fek temp failed\n");
		return -EINVAL;
	}

	tfm = crypto_alloc_aead("gcm(aes)", 0, 0);
	if (IS_ERR(tfm))
		return (s32)PTR_ERR(tfm);

	ret = file_protect_set_gcm_key(tfm, aes_key.data, aes_key.len);
	if (ret != 0) {
		hwdps_pr_err("%s, set key failed ret:%d", __func__, ret);
		goto out;
	}

	ret = file_protect_derive_gcm_key(tfm, fek_temp, sizeof(fek_temp),
		xattr->enc_fek, sizeof(xattr->enc_fek),
		xattr->iv, sizeof(xattr->iv), 1); /* 1 means encrypt */
	if (ret != 0) {
		hwdps_pr_err("%s, derive key failed ret:%d", __func__, ret);
		goto out;
	}

	hwdps_pr_debug("%s end gcm\n", __func__);

out:
	crypto_free_aead(tfm);
	(void)memset_s(fek_temp, fek_temp_len, 0, fek_temp_len);
	return ret;
}

s32 hwdps_generate_enc(buffer_t aes_key, buffer_t ciphertext,
	buffer_t plaintext_fek, bool is_update)
{
	hwdps_pr_debug("%s enter\n", __func__);
	if (!is_update)
		get_random_bytes(plaintext_fek.data, plaintext_fek.len);

	return hwdps_enc_fek(aes_key, ciphertext, plaintext_fek);
}

s32 hwdps_refresh_enc(buffer_t aes_key, buffer_t ciphertext,
	buffer_t plaintext_fek)
{
	hwdps_pr_debug("%s enter\n", __func__);
	return hwdps_enc_fek(aes_key, ciphertext, plaintext_fek);
}
