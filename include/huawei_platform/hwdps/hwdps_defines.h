/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for operations about
 *              some tools.
 * Create: 2020-06-16
 */

#ifndef _HWDPS_DEFINES_H
#define _HWDPS_DEFINES_H

#include <linux/types.h>

#define HWDPS_ENABLE_FLAG 0x00F0

enum {
	ERR_MSG_SUCCESS = 0,
	ERR_MSG_OUT_OF_MEMORY = 1,
	/* A pointer (that should not be null) is null. */
	ERR_MSG_NULL_PTR = 2,
	/* Kernel submits a request with undefined request ID. */
	ERR_MSG_UNKNOWN_REQ_ID = 3,
	/* Messages between kernel and TA are in wrong format. */
	ERR_MSG_LENGTH_ERROR = 4,
	ERR_MSG_GENERATE_FAIL = 2006,
};

typedef struct {
	u8 *data;
	u32 len;
} buffer_t;

typedef struct {
	u8 *data;
	u32 *len;
} out_buffer_t;

typedef struct {
	u8 **data;
	u32 *len;
} secondary_buffer_t;

typedef struct {
	pid_t pid;
	uid_t uid;
	uid_t task_uid;
} encrypt_id;

typedef struct {
	u8 **fek;
	u32 *fek_len;
	u8 **efek;
	u32 *efek_len;
} fek_efek_t;

#endif
