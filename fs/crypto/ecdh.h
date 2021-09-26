/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Internal implementation of ECDH for
 *              sdp(sensitive data protection) SECE case.
 * Create: 2020.08.22
 */

#ifndef _ECDH_H
#define _ECDH_H

#include <crypto/kpp.h>
#include <crypto/ecdh.h>
#include <huawei_platform/hwdps/hwdps_defines.h>
#include "sdp_internal.h"

#ifdef F2FS_FS_SDP_ENCRYPTION
/*
 * Use ECDH to generate file public key
 * and shared secret.
 *
 * cuive_id:          Curve id, only ECC_CURVE_NIST_P256 now
 * dev_pub_key:       Device public key
 * file_pub_key[out]: File public key to be generated
 * shared secret[out]:Compute shared secret with file private key
 * and device public key
 *
 * Return: 0 for success, error code in case of error.
 *         The contents of @file_pub_key and @shared_secret are
 *         undefined in case of error.
 */
int get_file_pubkey_shared_secret(u32 curve_id, const buffer_t *dev_pub_key,
	buffer_t *file_pub_key, buffer_t *shared_secret);

/*
 * Use ECDH to generate shared secret.
 *
 * cuive_id:          Curve id, only ECC_CURVE_NIST_P256 now
 * dev_privkey:       Device private key
 * file_pub_key:      File public key
 * shared secret[out]:Compute shared secret with file public key and device
 *                     private key
 *
 * Return: 0 for success, error code in case of error.
 *         The content of @shared_secret is undefined in case of error.
 */
int get_shared_secret(u32 curve_id, const buffer_t *dev_priv_key,
	const buffer_t *file_pub_key, buffer_t *shared_secret);

#endif
#endif
