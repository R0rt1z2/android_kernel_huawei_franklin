/*
 * declare_static_ion.c
 *
 * get and set static mem info.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "declare_static_ion.h"
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include "tc_ns_log.h"

static u64 g_secos_ex_addr;
static u64 g_secos_ex_size;

static int secos_reserve_tee_mem(struct reserved_mem *rmem)
{
	if (rmem) {
		g_secos_ex_addr = rmem->base;
		g_secos_ex_size = rmem->size;
	} else {
		tloge("secos reserve tee mem is NULL\n");
	}
	return 0;
}

RESERVEDMEM_OF_DECLARE(secos_ex, "mediatek,tee_os_reserved_memory",
	secos_reserve_tee_mem);

void set_ion_mem_info(struct register_ion_mem_tag *memtag)
{
	uint32_t pos = 0;

	if (!memtag) {
		tloge("invalid memtag\n");
		return;
	}

	tloge("ion mem static reserved for tee secos=0x%x\n", (uint32_t)g_secos_ex_size);

	if (g_secos_ex_addr != (u64)0 && g_secos_ex_size != (u64)0) {
		memtag->memaddr[pos] = g_secos_ex_addr;
		memtag->memsize[pos] = g_secos_ex_size;
		memtag->memtag[pos] = PP_MEM_TAG;
		pos++;
	}

	/* here pos max is 1, memaddr[] has 10 positions, just 9 free */
	memtag->size = pos;
	return;
}

#ifdef DEF_ENG
void get_secos_mem(uint64_t *addr, uint32_t *size)
{
	if (!addr || !size) {
		tloge("addr or size invalid\n");
		return;
	}

	tloge("secos mem g_secos_ex_size:0x%x\n", (uint32_t)g_secos_ex_size);
	*addr = g_secos_ex_addr;
	*size = (uint32_t)g_secos_ex_size;
}
#endif
