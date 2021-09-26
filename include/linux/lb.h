/*
 * lb.h
 *
 * Copyright (c) 2018-2019 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LB_H
#define __LB_H

#ifdef CONFIG_LB_GEMINI
extern int perf_ctrl_lb_set_policy(void __user *uarg);
extern int perf_ctrl_lb_reset_policy(void __user *uarg);
extern int perf_ctrl_lb_lite_info(void __user *uarg);
#else
static inline int perf_ctrl_lb_set_policy(void __user *uarg) { return 0; }
static inline int perf_ctrl_lb_reset_policy(void __user *uarg) { return 0; }
static inline int perf_ctrl_lb_lite_info(void __user *uarg) { return 0; }
#endif

#endif
