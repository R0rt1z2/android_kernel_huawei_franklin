/*
 * mm_ion_dump.h	ion dump interface declare
 *
 * Copyright(C) 2020 Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MM_ION_DUMP_H
#define _MM_ION_DUMP_H

#include <linux/types.h>

#ifdef CONFIG_MM_PAGE_TRACE
int mm_ion_proecss_info(void);
#else
static inline int mm_ion_proecss_info(void)
{
	return 0;
}
#endif /* CONFIG_MM_PAGE_TRACE */
#endif /* _MM_ION_DUMP_H */
