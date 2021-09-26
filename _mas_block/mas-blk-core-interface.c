/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: mas block core interface
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "mas-blk-core-interface.h"

void _cfi_mas_blk_queue_usr_ctrl_recovery_timer_expire(unsigned long data)
{
	mas_blk_queue_usr_ctrl_recovery_timer_expire(data);
}

ssize_t __cfi_mas_queue_status_show(struct request_queue *q, char *page)
{
	return mas_queue_status_show(q, page, PAGE_SIZE);
}

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
ssize_t __cfi_mas_queue_io_prio_sim_show(struct request_queue *q, char *page)
{
	return mas_queue_io_prio_sim_show(q, page);
}

ssize_t __cfi_mas_queue_io_prio_sim_store(
	struct request_queue *q, const char *page, size_t count)
{
	return mas_queue_io_prio_sim_store(q, page, count);
}
#endif
