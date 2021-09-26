/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Implementation of 1) get sdp encryption key info;
 *                                2) update sdp context.
 * Create: 2020.08.22
 */

#ifndef _KEYINFO_SDP_H
#define _KEYINFO_SDP_H

#include "sdp_internal.h"

#ifdef F2FS_FS_SDP_ENCRYPTION

int f2fs_change_to_sdp_crypto(struct inode *inode, void *fs_data);

int f2fs_get_sdp_crypt_info(struct inode *inode, void *fs_data);

int f2fs_inode_check_sdp_keyring(const u8 descriptor[FS_KEY_DESCRIPTOR_SIZE],
	int enforce);

int f2fs_inode_get_sdp_encrypt_flags(struct inode *inode, void *fs_data,
	u32 *flag);
int f2fs_inode_set_sdp_encryption_flags(struct inode *inode, void *fs_data,
	u32 sdp_enc_flag);

#endif
#endif
