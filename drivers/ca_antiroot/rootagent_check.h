/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2020. All rights reserved.
 * Description: the rootagent_check.h is the api for debug mode checking.
 * Create: 2019-5-22
 */

#ifndef _ROOT_CHECK_H_
#define _ROOT_CHECK_H_

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "teek_client_id.h"
#include "teek_client_api.h"
#include "rootagent_api.h"
#include "rootagent_common.h"
#include "rootagent.h"

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG

void handle_proc_create(void);
#endif
#endif
#endif
