/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex ca header files
 * Author : security-ap
 * Create : 2020/07/16
 */

#ifndef __HUAWEI_FBEX_DEV_H_
#define __HUAWEI_FBEX_DEV_H_

#include <linux/types.h>
#include <uni/fbe/fbe_ctrl.h>

#define FBEX_IOC_MAGIC 'f'

struct fbex_req {
	u32 user;
	u32 file; /* D:0, C:1, A:2, B:3 */
	u32 len;
	u8 key[KEY_LEN];
	u8 flag;
};

#define HUAWEI_FBEX_ADD_IV _IOWR(FBEX_IOC_MAGIC, 1, struct fbex_req)
#define HUAWEI_FBEX_DEL_IV _IOW(FBEX_IOC_MAGIC, 2, struct fbex_req)
#define HUAWEI_FBEX_USER_LOGOUT _IOW(FBEX_IOC_MAGIC, 8, struct fbex_req)

struct cmd_table {
	u32 cmd;
	u32 id;
};

#endif /* __HUAWEI_FBEX_DEV_H_ */
