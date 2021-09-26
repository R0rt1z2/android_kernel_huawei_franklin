/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: the rootagent_crypto.c for encrypt and decrypt.
 * Create: 2016-5-19
 */

#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#include <linux/version.h>
#include "rootagent_common.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
#include <crypto/skcipher.h>
#endif

static int check_aes256_cbc_param(const u8 *output, const u8 *input,
				const u8 *iv, int iv_len,
				const u8 *key, int key_len, int size)
{
	if ((output == NULL) || (input == NULL) || (iv == NULL) ||
		(key == NULL) || (iv_len < IV_SIZE) || (key_len <= 0)) {
		antiroot_error("rootagent: do_aes256_cbc invalid params!\n");
		return -EINVAL;
	}
	if ((size <= 0) || (size % CHALLENGE_KEY_LENGTH > 0)) {
		antiroot_error("rootagent: do_aes256_cbc size = %d\n", size);
		return -EINVAL;
	}
	return 0;
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 9, 0))
/*
 * size of [iv] is 16 and [key] must be 32 bytes.
 * [size] is the size of [output] and [input].
 * [size] must be mutiple of 32.
 */
int do_aes256_cbc(u8 *output, const u8 *input, const u8 *iv, int iv_len,
		const u8 *key, int key_len, int size, int encrypt)
{
	struct scatterlist src;
	struct scatterlist dst;
	struct blkcipher_desc desc;
	struct crypto_blkcipher *cipher = NULL;
	int ret;

	ret = check_aes256_cbc_param(output, input, iv, iv_len,
				key, key_len, size);
	if (ret == -EINVAL)
		return ret;

	cipher = crypto_alloc_blkcipher("cbc(aes)", 0, 0);

	if (IS_ERR(cipher)) {
		antiroot_error("rootagent: crypto_alloc_blkcipher() failed (%ld)\n",
				PTR_ERR(cipher));
		return -EFAULT;
	}

	ret = crypto_blkcipher_setkey(cipher, key, key_len);
	if (ret) {
		antiroot_error("rootagent:crypto_blkcipher_setkey failed %d\n",
			ret);
		crypto_free_blkcipher(cipher);
		return -EFAULT;
	}

	crypto_blkcipher_set_iv(cipher, iv, IV_SIZE);
	sg_init_one(&dst, output, size);
	sg_init_one(&src, input, size);
	desc.tfm   = cipher;
	desc.flags = 0;

	if (encrypt)
		ret = crypto_blkcipher_encrypt(&desc, &dst, &src, size);
	else
		ret = crypto_blkcipher_decrypt(&desc, &dst, &src, size);

	crypto_free_blkcipher(cipher);
	return ret;
}
#else
int do_aes256_cbc(u8 *output, const u8 *input, const u8 *iv, int iv_len,
		const u8 *key, int key_len, int size, int encrypt)
{
	struct scatterlist src;
	struct scatterlist dst;
	struct crypto_skcipher *skcipher = NULL;
	struct skcipher_request *req = NULL;
	int ret;
	uint8_t temp_iv[IV_SIZE] = { 0 };

	ret = check_aes256_cbc_param(output, input, iv, iv_len,
				key, key_len, size);
	if (ret == -EINVAL)
		return ret;

	skcipher = crypto_alloc_skcipher("cbc(aes)", 0, 0);
	if (IS_ERR(skcipher)) {
		antiroot_error("rootagent: crypto_alloc_skcipher() failed\n");
		return -EFAULT;
	}
	req = skcipher_request_alloc(skcipher, GFP_KERNEL);
	if (req == NULL) {
		antiroot_error("rootagent: skcipher_request_alloc() failed\n");
		crypto_free_skcipher(skcipher);
		return -ENOMEM;
	}
	ret = crypto_skcipher_setkey(skcipher, key, key_len);
	if (ret) {
		antiroot_error("rootagent: crypto_skcipher_setkey failed %d\n",
			ret);
		skcipher_request_free(req);
		crypto_free_skcipher(skcipher);
		return -EFAULT;
	}
	ret = memcpy_s(temp_iv, IV_SIZE, iv, IV_SIZE);
	if (ret != EOK) {
		antiroot_error("rootagent: memcpy_s failed, ret = %d\n", ret);
		skcipher_request_free(req);
		crypto_free_skcipher(skcipher);
		return -EFAULT;
	}
	sg_init_one(&dst, output, size);
	sg_init_one(&src, input, size);
	skcipher_request_set_crypt(req, &src, &dst, size, temp_iv);
	if (encrypt)
		ret = crypto_skcipher_encrypt(req);
	else
		ret = crypto_skcipher_decrypt(req);
	skcipher_request_free(req);
	crypto_free_skcipher(skcipher);
	return ret;
}
#endif
