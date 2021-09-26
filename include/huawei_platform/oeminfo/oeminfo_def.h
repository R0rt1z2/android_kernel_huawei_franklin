/*
 * oeminfo_def.h
 *
 * oeminfo id type definition
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef LIBOEMINFO_OEMINFO_H
#define LIBOEMINFO_OEMINFO_H
#include "oeminfo_def_v6.h"
#define OEMINFO_DATA_SIZE            (8 * 1024 - 512)
#define OEMINFO_WRITE                0 /* OEMINFO write operation */
#define OEMINFO_READ                 1  /* OEMINFO read  operation */
#define OEMINFO_BLOCK_ERASE_VALUE    0xFF

struct oeminfo_info_user {
	uint32_t oeminfo_operation;
	uint32_t oeminfo_id;
	uint32_t valid_size;
	u_char oeminfo_data[OEMINFO_DATA_SIZE];
};

int oeminfo_direct_access(struct oeminfo_info_user *user_info);
#endif
