/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex common headers
 * Author : security-ap
 * Create : 2020/07/16
 */

#ifndef __HUAWEI_FBE_CTRL_H_
#define __HUAWEI_FBE_CTRL_H_

#include <linux/types.h>

#ifndef _unused
#define _unused __attribute__((unused))
#endif

enum FILE_ENCRY_CMD {
	SEC_FILE_ENCRY_CMD_ID_VOLD_ADD_IV = 0x1,
	SEC_FILE_ENCRY_CMD_ID_VOLD_DELETE_IV = 0x2,
	SEC_FILE_ENCRY_CMD_ID_LOCK_SCREEN,
	SEC_FILE_ENCRY_CMD_ID_UNLOCK_SCREEN,
	SEC_FILE_ENCRY_CMD_ID_KEY_RESTORE = 0x5,
	SEC_FILE_ENCRY_CMD_ID_NEW_SECE,
	SEC_FILE_ENCRY_CMD_ID_GEN_METADATA,
	SEC_FILE_ENCRY_CMD_ID_USER_LOGOUT = 0x8,
	SEC_FILE_ENCRY_CMD_ID_ENABLE_KDF,
	SEC_FILE_ENCRY_CMD_ID_PRELOADING,
	SEC_FILE_ENCRY_CMD_ID_MSPC_CHECK,
	SEC_FILE_ENCRY_CMD_ID_STATUS_REPORT,
	/* Reserve cmd id 0xD - 0xF for future using */
	SEC_FILE_ENCRY_CMD_ID_MAX_SUPPORT = 0x10,
};
#define SEC_FILE_ENCRY_CMD_ID_MASK 0xF

#define KEY_LEN      64 /* 64 * 8 = 512bits */
#define PUBKEY_LEN   0x41 /* 0x41 for p256; 0x61 for p384 */
#define METADATA_LEN 16 /* 16 * 8 = 128bits */

#ifdef CONFIG_HUAWEI_FBE
u32 huawei_fbex_restore_key(void);
#else
static inline u32 huawei_fbex_restore_key(void) {return 0; }
#endif
#endif /* __HUAWEI_FBE_CTRL_H_ */
