/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub logbuff module
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

#ifndef __LINUX_XHUB_LOGBUFF_H__
#define __LINUX_XHUB_LOGBUFF_H__

#include "xhub_boot.h"

#define DDR_LOG_BUFF_ADDR_AP 0x100000 // hanping
#define DDR_LOG_BUFF_SIZE 0x7F000 // 0x100000//0x4000 //hanping

typedef struct {
	pkt_header_t hd;
	uint32_t index;
} log_buff_req_t;

extern struct config_on_ddr *g_config_on_ddr;
extern int set_log_level(int tag, int argv[], int argc);

#endif
