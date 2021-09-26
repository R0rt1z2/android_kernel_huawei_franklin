/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for generate key and
 *              encrypt key management.
 * Create: 2020-06-16
 */

#include "inc/tee/hwdps_keyinfo.h"
#include <keys/user-type.h>
#include <linux/key.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <securec.h>
#include "inc/base/hwdps_defines.h"
#include "inc/base/hwdps_utils.h"
#include "inc/tee/base_alg.h"
#include "inc/tee/hwdps_alg.h"

#define FS_KEY_DESCRIPTOR_LEN ((FS_KEY_DESCRIPTOR_SIZE * 2) + 1)

s32 hwdps_get_key(const u8 *descriptor,
	buffer_t *aes_key, uid_t uid)
{
	struct key *keyring_key = NULL;
	const struct user_key_payload *ukp = NULL;
	struct fscrypt_key *master_key = NULL;
	s32 res = 0;
	u8 tag[AES256_KEY_LEN] = {0};
	buffer_t tag_buffer = { tag, AES256_KEY_LEN };
	buffer_t msg_buffer = { (u8 *)&uid, sizeof(uid) };

	if (!descriptor || !aes_key || !aes_key->data ||
		aes_key->len != AES256_KEY_LEN)
		return -EINVAL;

	keyring_key = fscrypt_request_key(descriptor, FS_KEY_DESC_PREFIX,
		FS_KEY_DESC_PREFIX_SIZE);
	if (IS_ERR(keyring_key)) {
		return PTR_ERR(keyring_key);
	}

	down_read(&keyring_key->sem);
	if (keyring_key->type != &key_type_logon) {
		hwdps_pr_err("hwdps key type must be logon\n");
		res = -ENOKEY;
		goto out;
	}

	ukp = user_key_payload_locked(keyring_key);
	if (!ukp) {
		/* key was revoked before we acquired its semaphore */
		hwdps_pr_err("hwdps key was revoked\n");
		res = -EKEYREVOKED;
		goto out;
	}
	if (ukp->datalen != sizeof(struct fscrypt_key)) {
		hwdps_pr_err("hwdps fscrypt key size err %d\n", ukp->datalen);
		res = -EINVAL;
		goto out;
	}
	master_key = (struct fscrypt_key *)ukp->data;

	/* check master_key size failed on mtk size is 64 */
	if (memcpy_s(tag, AES256_KEY_LEN,
		master_key->raw, AES256_KEY_LEN) != EOK) {
		hwdps_pr_err("master key error size %d\n", master_key->size);
		res = -ENOKEY;
		goto out;
	}

	res = hash_generate_mac(&tag_buffer, &msg_buffer, aes_key);
	if (res != 0) {
		hwdps_pr_err("hash_generate_mac res %d\n", res);
		res = -ENOKEY;
		goto out;
	}

out:
	up_read(&keyring_key->sem);
	key_put(keyring_key);
	(void)memset_s(tag, AES256_KEY_LEN, 0, AES256_KEY_LEN);
	return res;
}

static void clear_new_key(buffer_t *fek,
	buffer_t *encoded_buffer, buffer_t *plaintext_fek_buffer)
{
	(void)memset_s(fek->data, fek->len, 0, fek->len);
	(void)memset_s(encoded_buffer->data, encoded_buffer->len,
		0, encoded_buffer->len);
	(void)memset_s(plaintext_fek_buffer->data, plaintext_fek_buffer->len,
		0, plaintext_fek_buffer->len);
}

s32 kernel_new_fek(u8 *desc, uid_t uid,
	secondary_buffer_t *encoded_buf, buffer_t *fek)
{
	s32 err_code;
	u8 aes_key[AES256_KEY_LEN] = {0};
	u8 encoded_ciphertext[PHASE3_CIPHERTEXT_LENGTH] = {0};
	u8 plaintext_fek[FEK_LENGTH] = {0};
	buffer_t aes_key_buffer = { aes_key, AES256_KEY_LEN };
	buffer_t encoded_ciphertext_buffer = {
		encoded_ciphertext, PHASE3_CIPHERTEXT_LENGTH
	};
	buffer_t plaintext_fek_buffer = { plaintext_fek, FEK_LENGTH };

	if (!desc || !encoded_buf ||
		!encoded_buf->data || !fek || !fek->data ||
		fek->len != FEK_LENGTH) {
		return HWDPS_ERR_INTERNAL;
	}

	err_code = hwdps_get_key(desc, &aes_key_buffer, uid);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_get_key failed:%d\n", err_code);
		goto cleanup;
	}

	err_code = hwdps_generate_enc(aes_key_buffer, encoded_ciphertext_buffer,
		plaintext_fek_buffer, false);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_generate_enc failed:%d\n", err_code);
		goto cleanup;
	}

	if (memcpy_s(fek->data, fek->len, plaintext_fek,
		FEK_LENGTH) != EOK) {
		hwdps_pr_err("%s memcpy fek failed", __func__);
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}
	if (memcpy_s(*encoded_buf->data, *encoded_buf->len,
		encoded_ciphertext_buffer.data,
		encoded_ciphertext_buffer.len) != EOK) {
		hwdps_pr_err("%s memcpy material failed %u, %u", __func__,
			*encoded_buf->len, encoded_ciphertext_buffer.len);
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}
	hwdps_pr_info("%s success gcm\n", __func__);
cleanup:
	clear_new_key(&aes_key_buffer, &encoded_ciphertext_buffer,
		&plaintext_fek_buffer);
	return err_code;
}

static void clear_exist_key(buffer_t *fek, buffer_t *plaintext_fek_buffer)
{
	(void)memset_s(fek->data, fek->len, 0, fek->len);
	(void)memset_s(plaintext_fek_buffer->data, plaintext_fek_buffer->len,
		0, plaintext_fek_buffer->len);
}

static kernel_get_fek_inner(const u8 *desc, uid_t uid,
	buffer_t *plaintext_fek_buffer, buffer_t *key_material,
	u32 flags)
{
	s32 err_code;
	u8 aes_key[AES256_KEY_LEN] = {0};
	buffer_t aes_key_buffer = { aes_key, AES256_KEY_LEN };

	err_code = hwdps_get_key(desc, &aes_key_buffer, uid);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_get_key in get key failed\n");
		goto out;
	}
	if (flags == HWDPS_XATTR_ENABLE_FLAG) {
		err_code = hwdps_generate_dec_cbc(aes_key_buffer,
			*key_material, *plaintext_fek_buffer);
	} else if (flags == HWDPS_XATTR_ENABLE_FLAG_NEW) {
		err_code = hwdps_generate_dec(aes_key_buffer,
			*key_material, *plaintext_fek_buffer);
	} else {
		hwdps_pr_err("%s error flags:%u\n", __func__, flags);
		err_code = ERR_MSG_GENERATE_FAIL;
	}
out:
	(void)memset_s(aes_key, AES256_KEY_LEN, 0, AES256_KEY_LEN);
	return err_code;
}

s32 kernel_get_fek(u8 *desc, uid_t uid,
	buffer_t *encoded_buf, secondary_buffer_t *fek, u32 flags)
{
	s32 err_code;
	u8 plaintext_fek[FEK_LENGTH] = {0};
	u32 plaintext_fek_len = FEK_LENGTH;
	buffer_t plaintext_fek_buffer = { plaintext_fek, FEK_LENGTH };

	hwdps_pr_info("%s enter\n", __func__);
	if (!desc || !fek || !encoded_buf || !encoded_buf->data ||
		!fek->len || !fek->data) {
		hwdps_pr_err("invalid fek params\n");
		err_code = ERR_MSG_NULL_PTR;
		return err_code;
	} else if (encoded_buf->len != PHASE3_CIPHERTEXT_LENGTH &&
		encoded_buf->len != sizeof(struct xattribs_t)) {
		hwdps_pr_err("invalid material len\n");
		err_code = ERR_MSG_LENGTH_ERROR;
		return err_code;
	}

	err_code = kernel_get_fek_inner(desc, uid, &plaintext_fek_buffer,
		encoded_buf, flags);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_generate_dec failed\n");
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	*fek->data = kzalloc(plaintext_fek_len, GFP_KERNEL);
	if (*fek->data == NULL) {
		hwdps_pr_err("bp_file_key malloc\n");
		err_code = ERR_MSG_OUT_OF_MEMORY;
		goto cleanup;
	}
	if (memcpy_s(*fek->data, plaintext_fek_len,
		plaintext_fek, FEK_LENGTH) != EOK) {
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}
	*fek->len = plaintext_fek_len;
	hwdps_pr_info("%s success\n", __func__);
cleanup:
	(void)memset_s(plaintext_fek_buffer.data, plaintext_fek_buffer.len,
		0, plaintext_fek_buffer.len);
	return err_code;
}

static bool check_update_params(u8 *desc, buffer_t *key_material,
	secondary_buffer_t *fek)
{
	return (!desc || !fek || !key_material || !fek->len ||
		!fek->data || !key_material->data ||
		key_material->len != PHASE3_CIPHERTEXT_LENGTH);
}

s32 kernel_update_fek(u8 *desc, buffer_t *key_material,
	secondary_buffer_t *fek, uid_t new_uid, uid_t old_uid)
{
	s32 err_code;
	u8 plaintext_fek[FEK_LENGTH] = {0};
	u8 aes_key[AES256_KEY_LEN] = {0};
	buffer_t aes_key_buffer = { aes_key, AES256_KEY_LEN };
	buffer_t plaintext_fek_buffer = { plaintext_fek, FEK_LENGTH };

	if (check_update_params(desc, key_material, fek)) {
		hwdps_pr_err("invalid fek or key_material\n");
		err_code = ERR_MSG_NULL_PTR;
		return err_code;
	}

	err_code = hwdps_get_key(desc, &aes_key_buffer, old_uid);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_get_key in get key failed\n");
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	err_code = hwdps_generate_dec(aes_key_buffer,
		*key_material, plaintext_fek_buffer);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_generate_dec failed\n");
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	err_code = hwdps_get_key(desc, &aes_key_buffer, new_uid);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_get_key in get key failed\n");
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	err_code = hwdps_generate_enc(aes_key_buffer,
		*key_material, plaintext_fek_buffer, true);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_generate_enc failed\n");
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	*fek->data = kzalloc(FEK_LENGTH, GFP_KERNEL);
	if (*fek->data == NULL) {
		err_code = ERR_MSG_OUT_OF_MEMORY;
		goto cleanup;
	}
	if (memcpy_s(*fek->data, FEK_LENGTH,
		plaintext_fek, FEK_LENGTH) != EOK) {
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}
	*fek->len = FEK_LENGTH;
	hwdps_pr_info("%s success\n", __func__);
cleanup:
	clear_exist_key(&aes_key_buffer, &plaintext_fek_buffer);
	return err_code;
}

s32 kernel_encrypt_key(const u8 *desc, uid_t uid,
	secondary_buffer_t *encoded_buf, buffer_t *fek)
{
	s32 err_code;
	u8 aes_key[AES256_KEY_LEN] = {0};
	u8 encoded_ciphertext[PHASE3_CIPHERTEXT_LENGTH] = {0};
	buffer_t aes_key_buffer = { aes_key, AES256_KEY_LEN };
	buffer_t encoded_ciphertext_buffer = {
		encoded_ciphertext, PHASE3_CIPHERTEXT_LENGTH
	};

	if (!desc || !encoded_buf ||
		!encoded_buf->data || !fek || !fek->data) {
		hwdps_pr_err("%s invalid params\n", __func__);
		err_code = ERR_MSG_NULL_PTR;
		goto cleanup;
	} else if (fek->len != FEK_LENGTH) {
		hwdps_pr_err("%s invalid length:%u\n", __func__, fek->len);
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	err_code = hwdps_get_key(desc, &aes_key_buffer, uid);
	if (err_code != 0) {
		hwdps_pr_err("%s, hwdps_get_key failed\n", __func__);
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	err_code = hwdps_refresh_enc(aes_key_buffer, encoded_ciphertext_buffer,
		*fek);
	if (err_code != 0) {
		hwdps_pr_err("hwdps_refresh_enc failed res:%d\n", err_code);
		err_code = ERR_MSG_GENERATE_FAIL;
		goto cleanup;
	}

	if (memcpy_s(*encoded_buf->data, *encoded_buf->len,
		encoded_ciphertext_buffer.data,
		encoded_ciphertext_buffer.len) != EOK) {
		err_code = ERR_MSG_GENERATE_FAIL;
		hwdps_pr_err("%s memcpy_s fail\n", __func__);
		goto cleanup;
	}
	hwdps_pr_info("%s success\n", __func__);
cleanup:
	(void)memset_s(aes_key, AES256_KEY_LEN, 0, AES256_KEY_LEN);
	(void)memset_s(encoded_ciphertext, PHASE3_CIPHERTEXT_LENGTH,
		0, PHASE3_CIPHERTEXT_LENGTH);
	return err_code;
}
