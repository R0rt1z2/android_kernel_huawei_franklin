/*
 * Copyright (C) 2020 MediaTek Inc.
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

/* This file is generated by GenLP_setting.pl v1.5.7 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

const unsigned int AP_DCM_Golden_Setting_tcl_gs_dpidle_data[] = {
/*	Address	Mask	Golden Setting Value */
	0xC53A2A0,	0x800,	0x800,	/* CPU_PLLDIV_CFG0 */
	0xC53A2A4,	0x800,	0x800,	/* CPU_PLLDIV_CFG1 */
	0xC53A2A8,	0x800,	0x800,	/* CPU_PLLDIV_CFG2 */
	0xC53A2AC,	0x800,	0x800,	/* CPU_PLLDIV_CFG3 */
	0xC53A2B0,	0x800,	0x800,	/* CPU_PLLDIV_CFG4 */
	0xC53A2E0,	0x80000800,	0x80000800,	/* BUS_PLLDIV_CFG */
	0xC53A440,	0xFFFF,	0xFFFF,	/* MCSIC_DCM0 */
	0xC53A500,	0x20000,	0x20000,	/* MP_ADB_DCM_CFG0 */
	0xC53A510,	0x278000,	0x278000,	/* MP_ADB_DCM_CFG4 */
	0xC53A518,	0x3A,	0x3A,	/* MP_MISC_DCM_CFG0 */
	0xC53A5C0,	0x78100,	0x78100,	/* MCUSYS_DCM_CFG0 */
	0xC53A900,	0xF,	0xF,	/* EMI_WFIFO */
	0xC53C880,	0x1000F,	0x1000F,	/* MP0_DCM_CFG0 */
	0xC53C89C,	0x11,	0x11,	/* MP0_DCM_CFG7 */
	0xD0A007C,	0x2,	0x0,	/* dbg_mode */
	0x10001070,	0xC0D07FFB,	0xC0D00603,	/* INFRA_BUS_DCM_CTRL */
	0x10001074,	0xA03FFFFB,	0xA03F83E3,	/* PERI_BUS_DCM_CTRL */
	0x100010A0,	0xF,	0x0,	/* P2P_RX_CLK_ON */
	0x100010A4,	0x100,	0x100,	/* MODULE_SW_CG_2_SET */
	0x100010A8,	0x100,	0x0,	/* MODULE_SW_CG_2_CLR */
	/* INFRA_AXIMEM_IDLE_BIT_EN_0 */
	0x10001A30,	0x7F000,	0x30000,
	0x10002028,	0x7E,	0x7E,	/* INFRA_EMI_DCM_CFG0 */
	0x1000202C,	0xFFFFFFFF,	0x40388000,	/* INFRA_EMI_DCM_CFG1 */
	0x10002034,	0xFFFFFFFF,	0xFF,	/* INFRA_EMI_DCM_CFG3 */
	0x10002038,	0xFFFFFFFF,	0x7,	/* TOP_CK_ANCHOR_CFG */
	0x1000A000,	0x80000000,	0x80000000,	/* SEJ_CON */
	0x1001A208,	0xFFFF,	0xFFFF,	/* DXCC_NEW_HWDCM_CFG */
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_0 */
	0x10022034,	0x10000003,	0x10000003,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_1 */
	0x10022038,	0x40003E0,	0x4000000,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_2 */
	0x1002203C,	0x3E0,	0x0,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_10 */
	0x1002205C,	0x2001000,	0x2001000,
	0x10024008,	0x1,	0x1,	/* DCM_CTRL */
	0x10024010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10024014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10026008,	0x1,	0x1,	/* DCM_CTRL */
	0x10026010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10026014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10027008,	0x1,	0x1,	/* DCM_CTRL */
	0x10027010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10027014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10028004,	0x200,	0x200,	/* DCM_CTRL */
	0x1002800C,	0x780,	0x780,	/* OTHER_CK_DCM_EN */
	0x10219060,	0xFF000000,	0x0,	/* EMI_CONM */
	0x10219068,	0xFF000000,	0x0,	/* EMI_CONN */
	0x10219830,	0x2000,	0x0,	/* EMI_THRO_CTRL0 */
	0x102280F0,	0x7,	0x7,	/* GCE_CTL_INT0 */
	0x102301FC,	0x80001000,	0x1000,	/* DRAMC_PD_CTRL */
	0x1023023C,	0x2,	0x2,	/* TX_CG_SET0 */
	0x10235008,	0xFF000000,	0x0,	/* CHN_EMI_CONB */
	0x102401FC,	0x80001000,	0x1000,	/* DRAMC_PD_CTRL */
	0x1024023C,	0x2,	0x2,	/* TX_CG_SET0 */
	0x1025E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1025F300,	0xFE,	0x0,	/* SMI_DCM */
	0x10309300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030A300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030B300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030C300,	0xFE,	0x0,	/* SMI_DCM */
	0x10443004,	0x100,	0x100,	/* SSPM_MCLK_DIV */
	0x10443008,	0x1FFFFF,	0x1FBFFF,	/* SSPM_DCM_CTRL */
	0x10728F88,	0x7,	0x7,	/* I3C0_CHN_HW_CG_EN */
	0x10729F88,	0x7,	0x7,	/* I2C1_CHN_HW_CG_EN */
	0x10943008,	0xFFF,	0xFFF,	/* SSPM_DCM_CTRL */
	0x11200950,	0x1000000,	0x1000000,	/* HDMA_CFG */
	0x11203E00,	0x10000,	0x10000,	/* SSUSB_IP_PW_CTRL0 */
	0x11203E04,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL1 */
	0x11203E08,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL2 */
	0x11203E0C,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL3 */
	0x11203E30,	0xA,	0xA,	/* SSUSB_U3_CTRL_0P */
	0x11203E50,	0xA,	0xA,	/* SSUSB_U2_CTRL_0P */
	0x11203E88,	0x3,	0x3,	/* SSUSB_CSR_CK_CTRL */
	0x11203E8C,	0x1F,	0x1F,	/* SSUSB_REF_CK_CTRL */
	0x11210000,	0x60000000,	0x60000000,	/* AUDIO_TOP_CON0 */
	0x112300B4,	0xFFA00000,	0x0,	/* PATCH_BIT1 */
	0x11300728,	0x3,	0x0,	/* REG_CG_DIS */
	0x11FA0014,	0x2000,	0x0,	/* MP_GLB_DIG_14 */
	0x13FBF010,	0x3FF7F,	0xC03F,	/* MFG_DCM_CON_0 */
	0x13FBF024,	0x1,	0x1,	/* MFG_ASYNC_CON_1 */
	0x13FBF0B0,	0x700,	0x0,	/* MFG_GLOBAL_CON */
	0x13FBF0B4,	0x10,	0x10,	/* MFG_QCHANNEL_CON */
	0x14000120,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS0 */
	0x14000130,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS1 */
	0x14000140,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS0 */
	0x14000150,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS1 */
	0x140001B0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS2 */
	0x140001C0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS2 */
	0x14002300,	0xFE,	0x0,	/* SMI_DCM */
	0x14003014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x14004014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1400E1F0,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM0 */
	0x1400E1F4,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM1 */
	0x1401B300,	0xFE,	0x0,	/* SMI_DCM */
	0x1401C300,	0xFE,	0x0,	/* SMI_DCM */
	0x1401D050,	0x7FFFFF,	0x0,	/* MMU_DCM_DIS */
	0x1401E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1502E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1502F300,	0xFE,	0x0,	/* SMI_DCM */
	0x1582E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1602E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1602F018,	0x1,	0x0,	/* VDEC_DCM_CON */
	0x1602F218,	0x1,	0x0,	/* LAT_DCM_CON */
	0x17010014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x17030300,	0x1,	0x0,	/* JPGENC_DCM_CTRL */
	0x19020010,	0x1,	0x1,	/* APU_CONN_DBG_APB_CTRL0 */
	0x1A001014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A002014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A00C300,	0xFE,	0x0,	/* SMI_DCM */
	0x1A00D300,	0xFE,	0x0,	/* SMI_DCM */
	0x1A00F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A010014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1B00E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1B00F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1B10F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1F000150,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS0 */
	0x1F000160,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS1 */
	0x1F000170,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS2 */
	0x1F000180,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS3 */
	0x1F000190,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS4 */
	0x1F0001A0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS0 */
	0x1F0001B0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS1 */
	0x1F0001C0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS2 */
	0x1F0001D0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS3 */
	0x1F0001E0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS4 */
	0x1F002014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
};

const unsigned int *AP_DCM_Golden_Setting_tcl_gs_dpidle =
		AP_DCM_Golden_Setting_tcl_gs_dpidle_data;

unsigned int AP_DCM_Golden_Setting_tcl_gs_dpidle_len = 369;

const unsigned int AP_DCM_Golden_Setting_tcl_gs_suspend_data[] = {
/*	Address	Mask	Golden Setting Value */
	0xC53A2A0,	0x800,	0x800,	/* CPU_PLLDIV_CFG0 */
	0xC53A2A4,	0x800,	0x800,	/* CPU_PLLDIV_CFG1 */
	0xC53A2A8,	0x800,	0x800,	/* CPU_PLLDIV_CFG2 */
	0xC53A2AC,	0x800,	0x800,	/* CPU_PLLDIV_CFG3 */
	0xC53A2B0,	0x800,	0x800,	/* CPU_PLLDIV_CFG4 */
	0xC53A2E0,	0x80000800,	0x80000800,	/* BUS_PLLDIV_CFG */
	0xC53A440,	0xFFFF,	0xFFFF,	/* MCSIC_DCM0 */
	0xC53A500,	0x20000,	0x20000,	/* MP_ADB_DCM_CFG0 */
	0xC53A510,	0x278000,	0x278000,	/* MP_ADB_DCM_CFG4 */
	0xC53A518,	0x3A,	0x3A,	/* MP_MISC_DCM_CFG0 */
	0xC53A5C0,	0x78100,	0x78100,	/* MCUSYS_DCM_CFG0 */
	0xC53A900,	0xF,	0xF,	/* EMI_WFIFO */
	0xC53C880,	0x1000F,	0x1000F,	/* MP0_DCM_CFG0 */
	0xC53C89C,	0x11,	0x11,	/* MP0_DCM_CFG7 */
	0xD0A007C,	0x2,	0x0,	/* dbg_mode */
	0x10001070,	0xC0D07FFB,	0xC0D00603,	/* INFRA_BUS_DCM_CTRL */
	0x10001074,	0xA03FFFFB,	0xA03F83E3,	/* PERI_BUS_DCM_CTRL */
	0x100010A0,	0xF,	0x0,	/* P2P_RX_CLK_ON */
	0x100010A4,	0x100,	0x100,	/* MODULE_SW_CG_2_SET */
	0x100010A8,	0x100,	0x0,	/* MODULE_SW_CG_2_CLR */
	/* INFRA_AXIMEM_IDLE_BIT_EN_0 */
	0x10001A30,	0x7F000,	0x30000,
	0x10002028,	0x7E,	0x7E,	/* INFRA_EMI_DCM_CFG0 */
	0x1000202C,	0xFFFFFFFF,	0x40388000,	/* INFRA_EMI_DCM_CFG1 */
	0x10002034,	0xFFFFFFFF,	0xFF,	/* INFRA_EMI_DCM_CFG3 */
	0x10002038,	0xFFFFFFFF,	0x7,	/* TOP_CK_ANCHOR_CFG */
	0x1000A000,	0x80000000,	0x80000000,	/* SEJ_CON */
	0x1001A208,	0xFFFF,	0xFFFF,	/* DXCC_NEW_HWDCM_CFG */
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_0 */
	0x10022034,	0x10000003,	0x10000003,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_1 */
	0x10022038,	0x40003E0,	0x4000000,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_2 */
	0x1002203C,	0x3E0,	0x0,
	/* VDNR_DCM_TOP_INFRA_PAR_BUS_u_INFRA_PAR_BUS_CTRL_10 */
	0x1002205C,	0x2001000,	0x2001000,
	0x10024008,	0x1,	0x1,	/* DCM_CTRL */
	0x10024010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10024014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10026008,	0x1,	0x1,	/* DCM_CTRL */
	0x10026010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10026014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10027008,	0x1,	0x1,	/* DCM_CTRL */
	0x10027010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10027014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10028004,	0x200,	0x200,	/* DCM_CTRL */
	0x1002800C,	0x780,	0x780,	/* OTHER_CK_DCM_EN */
	0x10219060,	0xFF000000,	0x0,	/* EMI_CONM */
	0x10219068,	0xFF000000,	0x0,	/* EMI_CONN */
	0x10219830,	0x2000,	0x0,	/* EMI_THRO_CTRL0 */
	0x102280F0,	0x7,	0x7,	/* GCE_CTL_INT0 */
	0x10230000,	0x1,	0x0,	/* DDRCOMMON0 */
	0x10230100,	0x80000,	0x0,	/* TEST2_A0 */
	0x10230108,	0x100,	0x0,	/* TEST2_A3 */
	0x10230110,	0x1000000,	0x1000000,	/* DUMMY_RD */
	0x10230168,	0x1000,	0x0,	/* SREF_DPD_CTRL */
	0x10230190,	0x7,	0x0,	/* ACTIMING_CTRL */
	0x102301A0,	0x180000,	0x0,	/* ZQ_SET0 */
	0x102301B0,	0x620000,	0x0,	/* TX_TRACKING_SET0 */
	0x102301FC,	0xC000106F,	0xC0000007,	/* DRAMC_PD_CTRL */
	0x10230200,	0x4,	0x0,	/* DCM_CTRL0 */
	0x10230208,	0x80000,	0x80000,	/* DVFS_CTRL0 */
	0x1023021C,	0x10,	0x0,	/* CMD_DEC_CTRL0 */
	0x1023023C,	0x7,	0x0,	/* TX_CG_SET0 */
	0x10230244,	0xC0000000,	0x0,	/* RX_CG_SET0 */
	0x10230250,	0x2,	0x0,	/* MISCTL0 */
	0x10230260,	0x8007FFF,	0x0,	/* CLKAR */
	0x10230288,	0xC0000000,	0x0,	/* SCSMCTRL_CG */
	/* SHU_APHY_TX_PICG_CTRL */
	0x102316AC,	0x80000000,	0x80000000,
	0x10235008,	0xFF000000,	0x0,	/* CHN_EMI_CONB */
	0x102384EC,	0x5FFF00,	0x100,	/* MISC_CG_CTRL0 */
	0x102384F4,	0xFFFFFFC0,	0x80600B80,	/* MISC_CG_CTRL2 */
	0x10238500,	0x770000,	0x770000,	/* MISC_CG_CTRL5 */
	0x10238624,	0x8,	0x0,	/* MISC_DUTYSCAN1 */
	0x10238648,	0xC7F007F,	0x0,	/* MISC_CTRL3 */
	0x1023864C,	0x1FF,	0x1FF,	/* MISC_CTRL4 */
	0x10238670,	0x8,	0x0,	/* MISC_RX_AUTOK_CFG0 */
	0x10238884,	0xE0000,	0x0,	/* SHU_B0_DQ7 */
	0x10238888,	0xFDF80000,	0x1200000,	/* SHU_B0_DQ8 */
	0x10238A04,	0xE0000,	0x0,	/* SHU_B1_DQ7 */
	0x10238A08,	0xFDF80000,	0x1200000,	/* SHU_B1_DQ8 */
	0x10238B88,	0xFC300000,	0x0,	/* SHU_CA_CMD8 */
	0x10238DA0,	0x732,	0x222,	/* MISC_SHU_RX_CG_CTRL */
	/* MISC_SHU_CG_CTRL0 */
	0x10238DA4,	0xFFFFFFFF,	0x33403000,
	0x10240000,	0x1,	0x0,	/* DDRCOMMON0 */
	0x10240100,	0x80000,	0x0,	/* TEST2_A0 */
	0x10240108,	0x100,	0x0,	/* TEST2_A3 */
	0x10240110,	0x1000000,	0x1000000,	/* DUMMY_RD */
	0x10240168,	0x1000,	0x0,	/* SREF_DPD_CTRL */
	0x10240190,	0x7,	0x0,	/* ACTIMING_CTRL */
	0x102401A0,	0x180000,	0x0,	/* ZQ_SET0 */
	0x102401B0,	0x620000,	0x0,	/* TX_TRACKING_SET0 */
	0x102401FC,	0xC000106F,	0xC0000007,	/* DRAMC_PD_CTRL */
	0x10240200,	0x4,	0x0,	/* DCM_CTRL0 */
	0x10240208,	0x80000,	0x80000,	/* DVFS_CTRL0 */
	0x1024021C,	0x10,	0x0,	/* CMD_DEC_CTRL0 */
	0x1024023C,	0x7,	0x0,	/* TX_CG_SET0 */
	0x10240244,	0xC0000000,	0x0,	/* RX_CG_SET0 */
	0x10240250,	0x2,	0x0,	/* MISCTL0 */
	0x10240260,	0x8007FFF,	0x0,	/* CLKAR */
	0x10240288,	0xC0000000,	0x0,	/* SCSMCTRL_CG */
	/* SHU_APHY_TX_PICG_CTRL */
	0x102416AC,	0x80000000,	0x80000000,
	0x102484EC,	0x5FFF00,	0x100,	/* MISC_CG_CTRL0 */
	0x102484F4,	0xFFFFFFC0,	0x80600B80,	/* MISC_CG_CTRL2 */
	0x10248500,	0x770000,	0x770000,	/* MISC_CG_CTRL5 */
	0x10248624,	0x8,	0x0,	/* MISC_DUTYSCAN1 */
	0x10248648,	0xC7F007F,	0x0,	/* MISC_CTRL3 */
	0x1024864C,	0x1FF,	0x1FF,	/* MISC_CTRL4 */
	0x10248670,	0x8,	0x0,	/* MISC_RX_AUTOK_CFG0 */
	0x10248884,	0xE0000,	0x0,	/* SHU_B0_DQ7 */
	0x10248888,	0xFDF80000,	0x1200000,	/* SHU_B0_DQ8 */
	0x10248A04,	0xE0000,	0x0,	/* SHU_B1_DQ7 */
	0x10248A08,	0xFDF80000,	0x1200000,	/* SHU_B1_DQ8 */
	0x10248B88,	0xFC300000,	0x0,	/* SHU_CA_CMD8 */
	0x10248DA0,	0x732,	0x222,	/* MISC_SHU_RX_CG_CTRL */
	/* MISC_SHU_CG_CTRL0 */
	0x10248DA4,	0xFFFFFFFF,	0x33403000,
	0x1025E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1025F300,	0xFE,	0x0,	/* SMI_DCM */
	0x10309300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030A300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030B300,	0xFE,	0x0,	/* SMI_DCM */
	0x1030C300,	0xFE,	0x0,	/* SMI_DCM */
	0x10443004,	0x100,	0x100,	/* SSPM_MCLK_DIV */
	0x10443008,	0x1FFFFF,	0x1FBFFF,	/* SSPM_DCM_CTRL */
	0x10728F88,	0x7,	0x7,	/* I3C0_CHN_HW_CG_EN */
	0x10729F88,	0x7,	0x7,	/* I2C1_CHN_HW_CG_EN */
	0x10943008,	0xFFF,	0xFFF,	/* SSPM_DCM_CTRL */
	0x11200950,	0x1000000,	0x1000000,	/* HDMA_CFG */
	/* SSUSB_IP_PW_CTRL0 */
	0x11203E00,	0x10000,	0x10000,
	0x11203E04,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL1 */
	0x11203E08,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL2 */
	0x11203E0C,	0x1,	0x1,	/* SSUSB_IP_PW_CTRL3 */
	0x11203E30,	0xA,	0xA,	/* SSUSB_U3_CTRL_0P */
	0x11203E50,	0xA,	0xA,	/* SSUSB_U2_CTRL_0P */
	0x11203E88,	0x3,	0x3,	/* SSUSB_CSR_CK_CTRL */
	0x11203E8C,	0x1F,	0x1F,	/* SSUSB_REF_CK_CTRL */
	0x11210000,	0x60000000,	0x60000000,	/* AUDIO_TOP_CON0 */
	0x112300B4,	0xFFA00000,	0x0,	/* PATCH_BIT1 */
	0x11300728,	0x3,	0x0,	/* REG_CG_DIS */
	0x11FA0014,	0x2000,	0x0,	/* MP_GLB_DIG_14 */
	0x13FBF010,	0x3FF7F,	0xC03F,	/* MFG_DCM_CON_0 */
	0x13FBF024,	0x1,	0x1,	/* MFG_ASYNC_CON_1 */
	0x13FBF0B0,	0x700,	0x0,	/* MFG_GLOBAL_CON */
	0x13FBF0B4,	0x10,	0x10,	/* MFG_QCHANNEL_CON */
	0x14000120,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS0 */
	0x14000130,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS1 */
	0x14000140,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS0 */
	0x14000150,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS1 */
	0x140001B0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS2 */
	0x140001C0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS2 */
	0x14002300,	0xFE,	0x0,	/* SMI_DCM */
	0x14003014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x14004014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1400E1F0,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM0 */
	0x1400E1F4,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM1 */
	0x1401B300,	0xFE,	0x0,	/* SMI_DCM */
	0x1401C300,	0xFE,	0x0,	/* SMI_DCM */
	0x1401D050,	0x7FFFFF,	0x0,	/* MMU_DCM_DIS */
	0x1401E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1502E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1502F300,	0xFE,	0x0,	/* SMI_DCM */
	0x1582E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1602E014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1602F018,	0x1,	0x0,	/* VDEC_DCM_CON */
	0x1602F218,	0x1,	0x0,	/* LAT_DCM_CON */
	0x17010014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x17030300,	0x1,	0x0,	/* JPGENC_DCM_CTRL */
	0x19020010,	0x1,	0x1,	/* APU_CONN_DBG_APB_CTRL0 */
	0x1A001014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A002014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A00C300,	0xFE,	0x0,	/* SMI_DCM */
	0x1A00D300,	0xFE,	0x0,	/* SMI_DCM */
	0x1A00F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1A010014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1B00E300,	0xFE,	0x0,	/* SMI_DCM */
	0x1B00F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1B10F014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
	0x1F000150,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS0 */
	0x1F000160,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS1 */
	0x1F000170,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS2 */
	0x1F000180,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS3 */
	0x1F000190,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS4 */
	0x1F0001A0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS0 */
	0x1F0001B0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS1 */
	0x1F0001C0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS2 */
	0x1F0001D0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS3 */
	0x1F0001E0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS4 */
	0x1F002014,	0xFFF0,	0xFFF0,	/* SMI_LARB_CON_SET */
};

const unsigned int *AP_DCM_Golden_Setting_tcl_gs_suspend =
		AP_DCM_Golden_Setting_tcl_gs_suspend_data;

unsigned int AP_DCM_Golden_Setting_tcl_gs_suspend_len = 549;

const unsigned int AP_DCM_Golden_Setting_tcl_gs_sodi_data[] = {
/*	Address	Mask	Golden Setting Value */
	0xD0A007C,	0x2,	0x0,	/* dbg_mode */
	0x1001A208,	0xFFFF,	0xFFFF,	/* DXCC_NEW_HWDCM_CFG */
	0x10024008,	0x1,	0x1,	/* DCM_CTRL */
	0x10024010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10024014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10026008,	0x1,	0x1,	/* DCM_CTRL */
	0x10026010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10026014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10027008,	0x1,	0x1,	/* DCM_CTRL */
	0x10027010,	0xFFFFFFFF,	0xFFFFFFFF,	/* INF_CK_DCM_EN */
	0x10027014,	0x7F,	0x7F,	/* OTHER_CK_DCM_EN */
	0x10028004,	0x200,	0x200,	/* DCM_CTRL */
	0x1002800C,	0x780,	0x780,	/* OTHER_CK_DCM_EN */
	0x10219060,	0xFF000000,	0x0,	/* EMI_CONM */
	0x10219068,	0xFF000000,	0x0,	/* EMI_CONN */
	0x10219830,	0x2000,	0x0,	/* EMI_THRO_CTRL0 */
	0x10235008,	0xFF000000,	0x0,	/* CHN_EMI_CONB */
	0x10943008,	0xFFF,	0xFFF,	/* SSPM_DCM_CTRL */
	0x112300B4,	0xFFA00000,	0x0,	/* PATCH_BIT1 */
	0x14000120,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS0 */
	0x14000130,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS1 */
	0x14000140,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS0 */
	0x14000150,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS1 */
	0x140001B0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_1ST_DIS2 */
	0x140001C0,	0xFFFFFFFF,	0x0,	/* MMSYS_HW_DCM_2ND_DIS2 */
	0x1400E1F0,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM0 */
	0x1400E1F4,	0xFFFFFFFF,	0x0,	/* DISP_POSTMASK_FUNC_DCM1 */
	0x1401D050,	0x7FFFFF,	0x0,	/* MMU_DCM_DIS */
	0x1602F018,	0x1,	0x0,	/* VDEC_DCM_CON */
	0x1602F218,	0x1,	0x0,	/* LAT_DCM_CON */
	0x17030300,	0x1,	0x0,	/* JPGENC_DCM_CTRL */
	0x19020010,	0x1,	0x1,	/* APU_CONN_DBG_APB_CTRL0 */
	0x1F000150,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS0 */
	0x1F000160,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS1 */
	0x1F000170,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS2 */
	0x1F000180,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS3 */
	0x1F000190,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_1ST_DIS4 */
	0x1F0001A0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS0 */
	0x1F0001B0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS1 */
	0x1F0001C0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS2 */
	0x1F0001D0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS3 */
	0x1F0001E0,	0xFFFFFFFF,	0x0,	/* MDPSYS_HW_DCM_2ND_DIS4 */
};

const unsigned int *AP_DCM_Golden_Setting_tcl_gs_sodi =
		AP_DCM_Golden_Setting_tcl_gs_sodi_data;

unsigned int AP_DCM_Golden_Setting_tcl_gs_sodi_len = 126;
