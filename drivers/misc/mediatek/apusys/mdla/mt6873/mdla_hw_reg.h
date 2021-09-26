/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MDLA_HW_REG_H_
#define _MDLA_HW_REG_H_

#define MDLA_IRQ_SWCMD_DONE        (1 << 2)
#define MDLA_IRQ_CDMA_FIFO_EMPTY   (1 << 5)
#define MDLA_IRQ_PMU_INTE  (1 << 9)
#define MDLA_IRQ_MASK      (0x1FFFFF)
#define MDLA_AXI_CTRL_MASK ((1 << 1) | (1 << 10))

/* MDLA config */
#define MDLA_CG_CON		(0x000)
#define MDLA_CG_SET		(0x004)
#define MDLA_CG_CLR		(0x008)
#define MDLA_SW_RST		(0x00C)
#define MDLA_MBIST_MODE0	(0x010)
#define MDLA_MBIST_MODE1	(0x014)
#define MDLA_MBIST_CTL		(0x018)
#define MDLA_MBIST_DEFAULT_DELSEL	(0x024)
#define MDLA_RP_RST		(0x060)
#define MDLA_RP_CON		(0x064)
#define MDLA_AXI_CTRL		(0x120)
#define MDLA_AXI1_CTRL		(0x124)

/* MDLA command */
#define MREG_TOP_G_REV     (0x0500)
#define MREG_TOP_G_INTP0   (0x0504)
#define MREG_TOP_G_INTP1   (0x0508)
#define MREG_TOP_G_INTP2   (0x050C)
#define MREG_TOP_G_CDMA0   (0x0510)
#define MREG_TOP_G_CDMA1   (0x0514)
#define MREG_TOP_G_CDMA2   (0x0518)
#define MREG_TOP_G_CDMA3   (0x051C)
#define MREG_TOP_G_CDMA4   (0x0520)
#define MREG_TOP_G_CDMA5   (0x0524)
#define MREG_TOP_G_CDMA6   (0x0528)
#define MREG_TOP_G_CUR0    (0x052C)
#define MREG_TOP_G_CUR1    (0x0530)
#define MREG_TOP_G_FIN0    (0x0534)
#define MREG_TOP_G_FIN1    (0x0538)
#define MREG_TOP_G_IDLE    (0x0544)

#define MREG_TOP_ENG0      (0x0550)
#define MREG_TOP_ENG1      (0x0554)
#define MREG_TOP_ENG2      (0x0558)
#define MREG_TOP_ENG11     (0x057C)
#define MREG_TOP_G_FIN3     (0x0584)
#define MREG_TOP_G_COREINFO     (0x0588)
#define MREG_CMD_SIZE      (0x1C0)

/* MDLA PMU */

#define CFG_PMCR_DEFAULT   (0x1F021)
#define PMU_CNT_SHIFT      (0x0010)
#define PMU_CLR_CMDE_SHIFT (0x5)

#define PMU_PMCR_CCNT_EN   (0x10000)
#define PMU_PMCR_CCNT_RST  (0x4)
#define PMU_PMCR_CNT_RST   (0x2)

#define PMU_CFG_PMCR       (0x0E00)
#define PMU_CYCLE          (0x0E04)
#define PMU_START_TSTAMP   (0x0E08)
#define PMU_END_TSTAMP     (0x0E0C)
#define PMU_EVENT_OFFSET   (0x0E10)
#define PMU_CNT_OFFSET     (0x0E14)
#define PMU_CMDID_LATCH    (0x0F00)

/* MDLA Debug Reg*/
#define MREG_IT_FRONT_C_INVALID  (0x0D74)
#define MREG_DEBUG_IF_0  (0x0DB0)
#define MREG_DEBUG_IF_2  (0x0DB8)

#endif
