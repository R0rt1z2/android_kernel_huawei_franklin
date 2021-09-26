/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: the rootagent_crypto.h for encrypt and decrypt.
 * Create: 2016-5-19
 */

#ifndef _ROOT_CRYPTO_WRAP_H_
#define _ROOT_CRYPTO_WRAP_H_

#define ENCRYPT 1
#define DECRYPT 0

int do_aes256_cbc(u8 *output, const u8 *input, const u8 *iv, int iv_len,
		const u8 *key, int key_len, int size, int encrypt);
#endif
