/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Internal functions for per-f2fs sdp(sensitive data protection).
 * Create: 2020.08.22
 */

#ifndef _SDP_INTERNAL_H
#define _SDP_INTERNAL_H

#include <linux/types.h>
#include <huawei_platform/hwdps/hwdps_defines.h>
#include <fscrypt_private.h>

#ifdef CONFIG_FS_SDP_ENCRYPTION
#define F2FS_FS_SDP_ENCRYPTION 1
#endif

#ifdef F2FS_FS_SDP_ENCRYPTION
#define F2FS_XATTR_SDP_ECE_ENABLE_FLAG	0x01
#define F2FS_XATTR_SDP_ECE_CONFIG_FLAG	0x02
#define F2FS_XATTR_SDP_SECE_ENABLE_FLAG	0x04
#define F2FS_XATTR_SDP_SECE_CONFIG_FLAG	0x08

#define f2fs_inode_is_sdp_encrypted(flag)	(flag & 0x0f)

#define f2fs_inode_is_config_sdp_ece_encryption(flag) \
	((flag) & (F2FS_XATTR_SDP_ECE_CONFIG_FLAG))
#define f2fs_inode_is_config_sdp_sece_encryption(flag) \
	((flag) & (F2FS_XATTR_SDP_SECE_CONFIG_FLAG))
#define f2fs_inode_is_config_sdp_encryption(flag) \
	(((flag) & (F2FS_XATTR_SDP_SECE_CONFIG_FLAG)) || \
	((flag) & (F2FS_XATTR_SDP_ECE_CONFIG_FLAG)))
#define f2fs_inode_is_enabled_sdp_ece_encryption(flag) \
	((flag) & (F2FS_XATTR_SDP_ECE_ENABLE_FLAG))
#define f2fs_inode_is_enabled_sdp_sece_encryption(flag) \
	((flag) & (F2FS_XATTR_SDP_SECE_ENABLE_FLAG))
#define f2fs_inode_is_enabled_sdp_encryption(flag) \
	(((flag) & (F2FS_XATTR_SDP_ECE_ENABLE_FLAG)) || \
	((flag) & (F2FS_XATTR_SDP_SECE_ENABLE_FLAG)))

#define SDP_PRINT_TAG "sdp"
#define sdp_pr_err(fmt, args...) pr_err(" %s: " fmt "\n", \
	SDP_PRINT_TAG, ## args)
#define sdp_pr_info(fmt, args...) pr_info(" %s: " fmt "\n", \
	SDP_PRINT_TAG, ## args)
#define sdp_pr_debug(fmt, args...) pr_debug(" %s: " fmt "\n", \
	SDP_PRINT_TAG, ## args)
#define sdp_pr_warn(fmt, args...) pr_warn(" %s: " fmt "\n", \
	SDP_PRINT_TAG, ## args)

#define FSCRYPT_INVALID_CLASS	0
#define FSCRYPT_CE_CLASS	1
#define FSCRYPT_SDP_ECE_CLASS	2
#define FSCRYPT_SDP_SECE_CLASS	3
#define FSCRYPT_DPS_CLASS	4

#define FS_SDP_ECC_PUB_KEY_SIZE	64
#define FS_SDP_ECC_PRI_KEY_SIZE	32
#define FS_AES_256_GCM_KEY_SIZE	32
#define FS_AES_256_CBC_KEY_SIZE	32
#define FS_AES_256_CTS_KEY_SIZE	32
#define FS_AES_256_XTS_KEY_SIZE	64

#define FS_KEY_INDEX_OFFSET	63

#define FS_KEY_CIPHER_SIZE	80
#define FS_KEY_IV_SIZE	16
#define FS_KEY_SDP_DERIVATION_NONCE_SIZE	64
#define ENHANCED_CHECK_KEYING	1
#define NO_NEED_TO_CHECK_KEYFING	0
#define FLAG_ENCRYPT	1
#define FLAG_DECRYPT	0
#define MAX_UFS_SLOT_INDEX	31
#define MAX_ECDH_SIZE	4096
#define HAS_PUB_KEY	1
#define NO_PUB_KEY	0
struct fscrypt_sdp_key {
	u32 version;
	u32 sdp_class;
	u32 mode;
	u8 raw[FS_MAX_KEY_SIZE];
	u32 size;
	u8 pub_key[FS_MAX_KEY_SIZE];
	u32 pub_key_size;
} __packed;

struct sdp_fscrypt_context {
	u8 version;
	u8 sdp_class;
	u8 format;
	u8 contents_encryption_mode;
	u8 filenames_encryption_mode;
	u8 flags;
	u8 master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
	u8 nonce[FS_KEY_CIPHER_SIZE];
	u8 iv[FS_KEY_IV_SIZE];
	u8 file_pub_key[FS_SDP_ECC_PUB_KEY_SIZE];
} __packed;

int set_cipher_for_sdp(struct fscrypt_info *ci,
	const struct sdp_fscrypt_context *ctx, u8 *fek, u32 fek_len,
	struct inode *inode);

#endif
#endif
