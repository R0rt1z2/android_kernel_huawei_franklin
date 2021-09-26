/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex ca header files
 * Author : security-ap
 * Create : 2020/07/16
 */

#ifndef __HUAWEI_FBEX_DERVER_H_
#define __HUAWEI_FBEX_DERVER_H_

#include "fbex_dev.h"

#include <teek_client_type.h>
#include <linux/types.h>
#include <uni/fbe/fbe_ctrl.h>

#define FILE_DE        0
#define FILE_CE        1
#define FILE_ECE       2
#define FILE_SECE      3
#define FILE_GLOBAL_DE 4

#define FBEX_FILE_LEN     0x8
#define FBEX_FILE_MASK    0xff
#define FBEX_KEY_TIMEOUT  12000 /* ms */

/* error ID 0xFBE202XX is error ID for kernel */
#define FBE2_ERROR_CMD_START          0xFBE20200
#define FBE2_ERROR_CMD_INVALID        0xFBE20201
#define FBE2_ERROR_CMD_UNSUPPORT      0xFBE20202
#define FBE2_ERROR_CMD_UNDEFINED      0xFBE20203
#define FBE2_ERROR_SECE_TIMEER_OUT    0xFBE20204
#define FBE2_ERROR_CKEY_TIMEER_OUT    0xFBE20205
#define FBE2_ERROR_DEBUG_TIMEER_OUT   0xFBE20206
#define FBE2_ERROR_BUFFER_NULL        0xFBE20207
#define FBE2_ERROR_COPY_FAIL          0xFBE20208
#define FBE2_ERROR_INIT_INFO          0xFBE20209
#define FBE2_ERROR_INIT_BUFFER        0xFBE2020A
#define FBE2_ERROR_IV_BUFFER          0xFBE2020B
#define FBE2_ERROR_SLOT_ID            0xFBE2020C
#define FBE2_ERROR_BUFFER_INVALID     0xFBE2020D

void file_encry_record_error(u32 cmd, u32 user, u32 file, u32 error);
u32 file_encry_unsupported(u32 user, u32 file, u8 *iv, u32 iv_len);
u32 file_encry_undefined(u32 user, u32 file, u8 *iv, u32 iv_len);
u32 file_encry_add_iv(u32 user, u32 file, u8 *iv, u32 iv_len);
u32 file_encry_delete_iv(u32 user, u32 file, u8 *iv, u32 iv_len);
u32 file_encry_logout_iv(u32 user, u32 file, u8 *iv, u32 iv_len);

void fbex_work_finish(u32 ret);

typedef u32 (*file_encry_cb)(u32 user, u32 file, u8 *iv, u32 iv_len);

#define CALLBACK_FN(num, name) [num] = file_encry_##name,

#define FILE_ENCRY_LIST                  \
	CALLBACK_FN(0x00, undefined)     \
	CALLBACK_FN(0x01, add_iv)        \
	CALLBACK_FN(0x02, delete_iv)     \
	CALLBACK_FN(0x03, unsupported)   \
	CALLBACK_FN(0x04, unsupported) \
	CALLBACK_FN(0x05, unsupported)   \
	CALLBACK_FN(0x06, unsupported)   \
	CALLBACK_FN(0x07, unsupported)   \
	CALLBACK_FN(0x08, logout_iv)     \
	CALLBACK_FN(0x09, unsupported)   \
	CALLBACK_FN(0x0A, unsupported)   \
	CALLBACK_FN(0x0B, unsupported)   \
	CALLBACK_FN(0x0C, unsupported) \
	CALLBACK_FN(0x0D, undefined)     \
	CALLBACK_FN(0x0E, undefined)     \
	CALLBACK_FN(0x0F, undefined)
#endif /* __HUAWEI_FBEX_DERVER_H_ */
