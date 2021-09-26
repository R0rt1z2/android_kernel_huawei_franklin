/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex ca header files
 * Author : security-ap
 * Create : 2020/07/16
 */

#ifndef __HUAWEI_FBEX_DEBUG_H_
#define __HUAWEI_FBEX_DEBUG_H_

#include <linux/types.h>
#include <uni/fbe/fbe_ctrl.h>

#define TEST_TIMEOUT  4000

#ifdef CONFIG_HUAWEI_FBE_DBG

int fbex_init_debugfs(void);
void fbex_cleanup_debugfs(void);
#else
static inline int fbex_init_debugfs(void) {return 0; }
static inline void fbex_cleanup_debugfs(void) {}
#endif
#endif /* __HUAWEI_FBEX_DEBUG_H_ */
