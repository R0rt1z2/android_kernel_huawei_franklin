/*
 * platform.c
 *
 * functions for mtk adapter
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
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

#ifndef __XHUB_PLATFORM_H__
#define __XHUB_PLATFORM_H__


extern void reset_trigger(void);
extern void ramdump_trigger(void);
extern void register_platform_notify(void);

#endif /* __XHUB_PLATFORM_H__ */
