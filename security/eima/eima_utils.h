/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: the eima_utils.h for print debug.
 * Create: 2017-12-20
 */

#ifndef _EIMA_UTILS_H_
#define _EIMA_UTILS_H_

#include <linux/printk.h>
#include <linux/string.h>

#ifdef CONFIG_ARCH_QCOM
#define EIMABIT 17
#else
#include "rootagent.h"
#endif

#define EIMA_TAG "EIMA"
#define ERROR_CODE (-1)

#ifdef CONFIG_HUAWEI_ENG_EIMA

#define EIMA_PRINT_DEBUG 1

#define eima_error(fmt, args...) pr_err("%s: %s: %d " fmt "\n", \
	EIMA_TAG, __func__, __LINE__, ## args)

#define eima_warning(fmt, args...) pr_warn("%s: %s: %d " fmt "\n", \
	EIMA_TAG, __func__, __LINE__, ## args)

#define eima_info(fmt, args...) pr_info("%s: %s: %d " fmt "\n", \
	EIMA_TAG, __func__, __LINE__, ## args)

#define eima_debug(fmt, ...) \
	do { if (EIMA_PRINT_DEBUG) pr_info("%s:DEBUG[%s:%d]: " fmt "\n", \
		EIMA_TAG, __FILE__, __LINE__, ##__VA_ARGS__); } while (0)

#else /* CONFIG_HUAWEI_ENG_EIMA */

#define eima_error(fmt, args...) pr_err("%s: %d " fmt "\n", \
	EIMA_TAG, __LINE__, ## args)

#define eima_warning(fmt, args...) pr_warn("%s: %d " fmt "\n", \
	EIMA_TAG, __LINE__, ## args)

#define eima_info(fmt, args...) pr_info("%s: %d " fmt "\n", \
	EIMA_TAG, __LINE__, ## args)

#define eima_debug(fmt, ...)

#endif /* CONFIG_HUAWEI_ENG_EIMA */

struct eima_async_send_work {
	struct m_list_msg *msg;
	int type;
	struct work_struct work;
};

#endif /* _EIMA_UTILS_H_ */
