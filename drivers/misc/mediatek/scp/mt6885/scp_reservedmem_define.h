/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __SCP_RESERVEDMEM_DEFINE_H__
#define __SCP_RESERVEDMEM_DEFINE_H__

static struct scp_reserve_mblock scp_reserve_mblock[] = {
#ifdef CONFIG_MTK_VOW_SUPPORT
	{
		.num = VOW_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x7D000,  /* 500KB ((80+210+210) * 1024)*/
	},
#endif
	{
		.num = SENS_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x80000,  /* 512 KB */
	},
	{
		.num = XHUB_CFG_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x1000,  /* 4KB bytes*/
	},
	{
		.num = XHUB_RSV_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x7F000,  /* 508KB bytes*/
	},
	{
		.num = SCP_A_LOGGER_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x101000,  /* 1 MB + 4KB */
	},
	{
		.num = SCP_LOGGER_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x7F000,  /* 0.5 MB - 4KB*/
	},
#if defined(CONFIG_SND_SOC_MTK_SCP_SMARTPA) || \
	defined(CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT) || \
	defined(CONFIG_MTK_AUDIO_TUNNELING_SUPPORT) || \
	defined(CONFIG_MTK_VOW_SUPPORT)
	{
		.num = AUDIO_IPI_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x19000,  /* 100 KB */
	},
#endif
#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT
	{
		.num = VOW_BARGEIN_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x4600,  /* 17KB */
	},
#endif
#ifdef SCP_PARAMS_TO_SCP_SUPPORT
	{
		.num = SCP_DRV_PARAMS_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x800,  /* 256 -> 2k bytes */
	},
#endif
	{
		.num = FLP_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0xA01000,  /* 10M +4K bytes, 4K for vmap align */
	},
	{
		.num = ALS_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x1E000,  /* 120KB bytes*/
	},
};


#endif

