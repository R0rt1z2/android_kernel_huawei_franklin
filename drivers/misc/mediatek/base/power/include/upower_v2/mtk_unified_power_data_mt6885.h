/*
 * Copyright (C) 2016 MediaTek Inc.
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

/* Petrus_v1p3_2k6_game_MP_190815 */

/**********************************************
 * unified_power_data.h
 * This header file includes:
 * 1. Macros of SRAM related address
 * 2. Raw datas of unified power tables for each bank
 **********************************************/

#ifndef UNIFIED_POWER_DATA_H
#define UNIFIED_POWER_DATA_H

/* remember to sync to sspm upower */
#define UPOWER_CSRAM_BASE 0x0012a000
#define UPOWER_CSRAM_SIZE 0x3000 /* 12K bytes */
#define UPOWER_DVFS_OFF_BOTTOM 0x8 /* ignore the last 8 bytes */
/* limit should be at 0x12CFF4 */
#define UPOWER_TBL_LIMIT \
	((UPOWER_CSRAM_BASE)+(UPOWER_CSRAM_SIZE)-(UPOWER_DVFS_OFF_BOTTOM))

struct upower_tbl upower_tbl_l_6885 = {
	.row = {
		{.cap = 127, .volt = 60000, .dyn_pwr = 13161,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 155, .volt = 60000, .dyn_pwr = 17109,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 163, .volt = 61250, .dyn_pwr = 19283,
			.lkg_pwr = {20392, 20392, 20392, 20392, 20392, 20392} },
		{.cap = 171, .volt = 63125, .dyn_pwr = 22026,
			.lkg_pwr = {21408, 21408, 21408, 21408, 21408, 21408} },
		{.cap = 185, .volt = 65625, .dyn_pwr = 27143,
			.lkg_pwr = {22811, 22811, 22811, 22811, 22811, 22811} },
		{.cap = 197, .volt = 68750, .dyn_pwr = 33453,
			.lkg_pwr = {24743, 24743, 24743, 24743, 24743, 24743} },
		{.cap = 206, .volt = 71250, .dyn_pwr = 38899,
			.lkg_pwr = {26288, 26288, 26288, 26288, 26288, 26288} },
		{.cap = 222, .volt = 75000, .dyn_pwr = 47760,
			.lkg_pwr = {28604, 28604, 28604, 28604, 28604, 28604} },
		{.cap = 232, .volt = 77500, .dyn_pwr = 54821,
			.lkg_pwr = {30356, 30356, 30356, 30356, 30356, 30356} },
		{.cap = 241, .volt = 80625, .dyn_pwr = 63336,
			.lkg_pwr = {32605, 32605, 32605, 32605, 32605, 32605} },
		{.cap = 249, .volt = 83750, .dyn_pwr = 72555,
			.lkg_pwr = {35089, 35089, 35089, 35089, 35089, 35089} },
		{.cap = 260, .volt = 88125, .dyn_pwr = 84931,
			.lkg_pwr = {38818, 38818, 38818, 38818, 38818, 38818} },
		{.cap = 266, .volt = 90625, .dyn_pwr = 92858,
			.lkg_pwr = {41065, 41065, 41065, 41065, 41065, 41065} },
		{.cap = 272, .volt = 93125, .dyn_pwr = 101103,
			.lkg_pwr = {43483, 43483, 43483, 43483, 43483, 43483} },
		{.cap = 278, .volt = 96875, .dyn_pwr = 113304,
			.lkg_pwr = {47374, 47374, 47374, 47374, 47374, 47374} },
		{.cap = 284, .volt = 100000, .dyn_pwr = 124630,
			.lkg_pwr = {50836, 50836, 50836, 50836, 50836, 50836} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
	},
};

struct upower_tbl upower_tbl_cluster_l_6885 = {
	.row = {
		{.cap = 127, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 155, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 163, .volt = 61250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 171, .volt = 63125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 185, .volt = 65625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 197, .volt = 68750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 206, .volt = 71250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 222, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 232, .volt = 77500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 241, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 249, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 260, .volt = 88125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 266, .volt = 90625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 272, .volt = 93125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 278, .volt = 96875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 284, .volt = 100000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_b_6885 = {
	.row = {
		{.cap = 580, .volt = 60000, .dyn_pwr = 79872,
			.lkg_pwr = {81046, 81046, 81046, 81046, 81046, 81046} },
		{.cap = 627, .volt = 62500, .dyn_pwr = 97945,
			.lkg_pwr = {87427, 87427, 87427, 87427, 87427, 87427} },
		{.cap = 668, .volt = 64375, .dyn_pwr = 116001,
			.lkg_pwr = {92213, 92213, 92213, 92213, 92213, 92213} },
		{.cap = 718, .volt = 67500, .dyn_pwr = 145123,
			.lkg_pwr = {100176, 100176, 100176, 100176, 100176,
						100176} },
		{.cap = 769, .volt = 70625, .dyn_pwr = 177071,
			.lkg_pwr = {108133, 108133, 108133, 108133, 108133,
						108133} },
		{.cap = 801, .volt = 72500, .dyn_pwr = 200653,
			.lkg_pwr = {112905, 112905, 112905, 112905, 112905,
						112905} },
		{.cap = 830, .volt = 75000, .dyn_pwr = 229764,
			.lkg_pwr = {119268, 119268, 119268, 119268, 119268,
						119268} },
		{.cap = 870, .volt = 78750, .dyn_pwr = 280365,
			.lkg_pwr = {130024, 130024, 130024, 130024, 130024,
						130024} },
		{.cap = 898, .volt = 80625, .dyn_pwr = 310738,
			.lkg_pwr = {135442, 135442, 135442, 135442, 135442,
						135442} },
		{.cap = 932, .volt = 83750, .dyn_pwr = 356771,
			.lkg_pwr = {144608, 144608, 144608, 144608, 144608,
						144608} },
		{.cap = 944, .volt = 85625, .dyn_pwr = 381998,
			.lkg_pwr = {150469, 150469, 150469, 150469, 150469,
						150469} },
		{.cap = 964, .volt = 89375, .dyn_pwr = 433914,
			.lkg_pwr = {163636, 163636, 163636, 163636, 163636,
						163636} },
		{.cap = 978, .volt = 91875, .dyn_pwr = 472664,
			.lkg_pwr = {172612, 172612, 172612, 172612, 172612,
						172612} },
		{.cap = 993, .volt = 94375, .dyn_pwr = 513335,
			.lkg_pwr = {181656, 181656, 181656, 181656, 181656,
						181656} },
		{.cap = 1005, .volt = 96250, .dyn_pwr = 545057,
			.lkg_pwr = {189284, 189284, 189284, 189284, 189284,
						189284} },
		{.cap = 1024, .volt = 100000, .dyn_pwr = 608474,
			.lkg_pwr = {205387, 205387, 205387, 205387, 205387,
						205387} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
	},
};

struct upower_tbl upower_tbl_cluster_b_6885 = {
	.row = {
		{.cap = 580, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 627, .volt = 62500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 668, .volt = 64375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 718, .volt = 67500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 769, .volt = 70625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 801, .volt = 72500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 830, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 870, .volt = 78750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 898, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 932, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 944, .volt = 85625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 964, .volt = 89375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 978, .volt = 91875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 993, .volt = 94375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1005, .volt = 96250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1024, .volt = 100000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_cci_6885 = {
	.row = {
		{.cap = 0, .volt = 60000, .dyn_pwr = 8776,
			.lkg_pwr = {44739, 44739, 44739, 44739, 44739, 44739} },
		{.cap = 0, .volt = 62500, .dyn_pwr = 10677,
			.lkg_pwr = {47558, 47558, 47558, 47558, 47558, 47558} },
		{.cap = 0, .volt = 65000, .dyn_pwr = 13232,
			.lkg_pwr = {50377, 50377, 50377, 50377, 50377, 50377} },
		{.cap = 0, .volt = 66875, .dyn_pwr = 14908,
			.lkg_pwr = {52793, 52793, 52793, 52793, 52793, 52793} },
		{.cap = 0, .volt = 70625, .dyn_pwr = 19105,
			.lkg_pwr = {57625, 57625, 57625, 57625, 57625, 57625} },
		{.cap = 0, .volt = 72500, .dyn_pwr = 21710,
			.lkg_pwr = {60041, 60041, 60041, 60041, 60041, 60041} },
		{.cap = 0, .volt = 75000, .dyn_pwr = 24921,
			.lkg_pwr = {63262, 63262, 63262, 63262, 63262, 63262} },
		{.cap = 0, .volt = 78125, .dyn_pwr = 29473,
			.lkg_pwr = {67758, 67758, 67758, 67758, 67758, 67758} },
		{.cap = 0, .volt = 80625, .dyn_pwr = 33980,
			.lkg_pwr = {71562, 71562, 71562, 71562, 71562, 71562} },
		{.cap = 0, .volt = 83750, .dyn_pwr = 39296,
			.lkg_pwr = {77092, 77092, 77092, 77092, 77092, 77092} },
		{.cap = 0, .volt = 86875, .dyn_pwr = 44442,
			.lkg_pwr = {83030, 83030, 83030, 83030, 83030, 83030} },
		{.cap = 0, .volt = 89375, .dyn_pwr = 49171,
			.lkg_pwr = {87998, 87998, 87998, 87998, 87998, 87998} },
		{.cap = 0, .volt = 91875, .dyn_pwr = 54216,
			.lkg_pwr = {93641, 93641, 93641, 93641, 93641, 93641} },
		{.cap = 0, .volt = 94375, .dyn_pwr = 59545,
			.lkg_pwr = {99509, 99509, 99509, 99509, 99509, 99509} },
		{.cap = 0, .volt = 96875, .dyn_pwr = 64634,
			.lkg_pwr = {106501, 106501, 106501, 106501, 106501,
						106501} },
		{.cap = 0, .volt = 100000, .dyn_pwr = 72199,
			.lkg_pwr = {115708, 115708, 115708, 115708, 115708,
						115708} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
	},
};

struct upower_tbl upower_tbl_l_6885T = {
	.row = {
		{.cap = 120, .volt = 60000, .dyn_pwr = 13161,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 146, .volt = 60000, .dyn_pwr = 17109,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 154, .volt = 61250, .dyn_pwr = 19283,
			.lkg_pwr = {20392, 20392, 20392, 20392, 20392, 20392} },
		{.cap = 162, .volt = 63125, .dyn_pwr = 22026,
			.lkg_pwr = {21408, 21408, 21408, 21408, 21408, 21408} },
		{.cap = 175, .volt = 65625, .dyn_pwr = 27143,
			.lkg_pwr = {22811, 22811, 22811, 22811, 22811, 22811} },
		{.cap = 186, .volt = 68750, .dyn_pwr = 33453,
			.lkg_pwr = {24743, 24743, 24743, 24743, 24743, 24743} },
		{.cap = 195, .volt = 71250, .dyn_pwr = 38899,
			.lkg_pwr = {26288, 26288, 26288, 26288, 26288, 26288} },
		{.cap = 209, .volt = 75000, .dyn_pwr = 47760,
			.lkg_pwr = {28604, 28604, 28604, 28604, 28604, 28604} },
		{.cap = 219, .volt = 77500, .dyn_pwr = 54821,
			.lkg_pwr = {30356, 30356, 30356, 30356, 30356, 30356} },
		{.cap = 228, .volt = 80625, .dyn_pwr = 63336,
			.lkg_pwr = {32605, 32605, 32605, 32605, 32605, 32605} },
		{.cap = 235, .volt = 83750, .dyn_pwr = 72555,
			.lkg_pwr = {35089, 35089, 35089, 35089, 35089, 35089} },
		{.cap = 245, .volt = 88125, .dyn_pwr = 84931,
			.lkg_pwr = {38818, 38818, 38818, 38818, 38818, 38818} },
		{.cap = 251, .volt = 90625, .dyn_pwr = 92858,
			.lkg_pwr = {41065, 41065, 41065, 41065, 41065, 41065} },
		{.cap = 257, .volt = 93125, .dyn_pwr = 101103,
			.lkg_pwr = {43483, 43483, 43483, 43483, 43483, 43483} },
		{.cap = 263, .volt = 96875, .dyn_pwr = 113304,
			.lkg_pwr = {47374, 47374, 47374, 47374, 47374, 47374} },
		{.cap = 268, .volt = 100000, .dyn_pwr = 124630,
			.lkg_pwr = {50836, 50836, 50836, 50836, 50836, 50836} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
	},
};

struct upower_tbl upower_tbl_cluster_l_6885T = {
	.row = {
		{.cap = 120, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 146, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 154, .volt = 61250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 162, .volt = 63125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 175, .volt = 65625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 186, .volt = 68750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 195, .volt = 71250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 209, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 219, .volt = 77500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 228, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 235, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 245, .volt = 88125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 251, .volt = 90625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 257, .volt = 93125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 263, .volt = 96875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 268, .volt = 100000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_b_6885T = {
	.row = {
		{.cap = 548, .volt = 60000, .dyn_pwr = 79872,
			.lkg_pwr = {81046, 81046, 81046, 81046, 81046, 81046} },
		{.cap = 592, .volt = 62500, .dyn_pwr = 97945,
			.lkg_pwr = {87427, 87427, 87427, 87427, 87427, 87427} },
		{.cap = 661, .volt = 64375, .dyn_pwr = 116001,
			.lkg_pwr = {92213, 92213, 92213, 92213, 92213, 92213} },
		{.cap = 678, .volt = 67500, .dyn_pwr = 145123,
			.lkg_pwr = {100176, 100176, 100176, 100176, 100176,
				100176} },
		{.cap = 726, .volt = 70625, .dyn_pwr = 177071,
			.lkg_pwr = {108133, 108133, 108133, 108133, 108133,
				108133} },
		{.cap = 756, .volt = 72500, .dyn_pwr = 200653,
			.lkg_pwr = {112905, 112905, 112905, 112905, 112905,
				112905} },
		{.cap = 783, .volt = 75000, .dyn_pwr = 229764,
			.lkg_pwr = {119268, 119268, 119268, 119268, 119268,
				119268} },
		{.cap = 821, .volt = 78750, .dyn_pwr = 280365,
			.lkg_pwr = {130024, 130024, 130024, 130024, 130024,
				130024} },
		{.cap = 848, .volt = 80625, .dyn_pwr = 310738,
			.lkg_pwr = {135442, 135442, 135442, 135442, 135442,
				135442} },
		{.cap = 880, .volt = 83750, .dyn_pwr = 356771,
			.lkg_pwr = {144608, 144608, 144608, 144608, 144608,
				144608} },
		{.cap = 915, .volt = 86875, .dyn_pwr = 414560,
			.lkg_pwr = {154858, 154858, 154858, 154858, 154858,
				154858} },
		{.cap = 941, .volt = 90000, .dyn_pwr = 469988,
			.lkg_pwr = {165830, 165830, 165830, 165830, 165830,
				165830} },
		{.cap = 974, .volt = 93125, .dyn_pwr = 537928,
			.lkg_pwr = {177134, 177134, 177134, 177134, 177134,
				177134} },
		{.cap = 995, .volt = 95625, .dyn_pwr = 603074,
			.lkg_pwr = {186600, 186600, 186600, 186600, 186600,
				186600} },
		{.cap = 1013, .volt = 98750, .dyn_pwr = 681394,
			.lkg_pwr = {200019, 200019, 200019, 200019, 200019,
				200019} },
		{.cap = 1024, .volt = 100000, .dyn_pwr = 718370,
			.lkg_pwr = {205387, 205387, 205387, 205387, 205387,
				205387} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
	},
};

struct upower_tbl upower_tbl_cluster_b_6885T = {
	.row = {
		{.cap = 548, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 592, .volt = 62500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 631, .volt = 64375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 678, .volt = 67500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 726, .volt = 70625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 756, .volt = 72500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 783, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 821, .volt = 78750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 848, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 880, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 915, .volt = 86875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 941, .volt = 90000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 974, .volt = 93125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 995, .volt = 95625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1013, .volt = 98750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1024, .volt = 100000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_cci_6885T = {
	.row = {
		{.cap = 0, .volt = 60000, .dyn_pwr = 8776,
			.lkg_pwr = {44739, 44739, 44739, 44739, 44739, 44739} },
		{.cap = 0, .volt = 62500, .dyn_pwr = 10677,
			.lkg_pwr = {47558, 47558, 47558, 47558, 47558, 47558} },
		{.cap = 0, .volt = 65000, .dyn_pwr = 13232,
			.lkg_pwr = {50377, 50377, 50377, 50377, 50377, 50377} },
		{.cap = 0, .volt = 66875, .dyn_pwr = 14908,
			.lkg_pwr = {52793, 52793, 52793, 52793, 52793, 52793} },
		{.cap = 0, .volt = 70625, .dyn_pwr = 19105,
			.lkg_pwr = {57625, 57625, 57625, 57625, 57625, 57625} },
		{.cap = 0, .volt = 72500, .dyn_pwr = 21710,
			.lkg_pwr = {60041, 60041, 60041, 60041, 60041, 60041} },
		{.cap = 0, .volt = 75000, .dyn_pwr = 24921,
			.lkg_pwr = {63262, 63262, 63262, 63262, 63262, 63262} },
		{.cap = 0, .volt = 78125, .dyn_pwr = 29473,
			.lkg_pwr = {67758, 67758, 67758, 67758, 67758, 67758} },
		{.cap = 0, .volt = 80625, .dyn_pwr = 33980,
			.lkg_pwr = {71562, 71562, 71562, 71562, 71562, 71562} },
		{.cap = 0, .volt = 83750, .dyn_pwr = 39296,
			.lkg_pwr = {77092, 77092, 77092, 77092, 77092, 77092} },
		{.cap = 0, .volt = 86875, .dyn_pwr = 44442,
			.lkg_pwr = {83030, 83030, 83030, 83030, 83030, 83030} },
		{.cap = 0, .volt = 89375, .dyn_pwr = 49171,
			.lkg_pwr = {87998, 87998, 87998, 87998, 87998, 87998} },
		{.cap = 0, .volt = 91875, .dyn_pwr = 54216,
			.lkg_pwr = {93641, 93641, 93641, 93641, 93641, 93641} },
		{.cap = 0, .volt = 94375, .dyn_pwr = 59545,
			.lkg_pwr = {99509, 99509, 99509, 99509, 99509, 99509} },
		{.cap = 0, .volt = 96875, .dyn_pwr = 64634,
			.lkg_pwr = {106501, 106501, 106501, 106501, 106501,
						106501} },
		{.cap = 0, .volt = 100000, .dyn_pwr = 72199,
			.lkg_pwr = {115708, 115708, 115708, 115708, 115708,
						115708} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
	},
};

struct upower_tbl upower_tbl_l_6883 = {
	.row = {
		{.cap = 132, .volt = 60000, .dyn_pwr = 13161,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 161, .volt = 60000, .dyn_pwr = 17109,
			.lkg_pwr = {19715, 19715, 19715, 19715, 19715, 19715} },
		{.cap = 170, .volt = 61250, .dyn_pwr = 19283,
			.lkg_pwr = {20392, 20392, 20392, 20392, 20392, 20392} },
		{.cap = 179, .volt = 63125, .dyn_pwr = 22026,
			.lkg_pwr = {21408, 21408, 21408, 21408, 21408, 21408} },
		{.cap = 193, .volt = 65625, .dyn_pwr = 27143,
			.lkg_pwr = {22811, 22811, 22811, 22811, 22811, 22811} },
		{.cap = 205, .volt = 68750, .dyn_pwr = 33453,
			.lkg_pwr = {24743, 24743, 24743, 24743, 24743, 24743} },
		{.cap = 215, .volt = 71250, .dyn_pwr = 38899,
			.lkg_pwr = {26288, 26288, 26288, 26288, 26288, 26288} },
		{.cap = 231, .volt = 75000, .dyn_pwr = 47760,
			.lkg_pwr = {28604, 28604, 28604, 28604, 28604, 28604} },
		{.cap = 242, .volt = 77500, .dyn_pwr = 54821,
			.lkg_pwr = {30356, 30356, 30356, 30356, 30356, 30356} },
		{.cap = 252, .volt = 80625, .dyn_pwr = 63336,
			.lkg_pwr = {32605, 32605, 32605, 32605, 32605, 32605} },
		{.cap = 260, .volt = 83750, .dyn_pwr = 72555,
			.lkg_pwr = {35089, 35089, 35089, 35089, 35089, 35089} },
		{.cap = 271, .volt = 88125, .dyn_pwr = 84931,
			.lkg_pwr = {38818, 38818, 38818, 38818, 38818, 38818} },
		{.cap = 278, .volt = 90625, .dyn_pwr = 92858,
			.lkg_pwr = {41065, 41065, 41065, 41065, 41065, 41065} },
		{.cap = 284, .volt = 93125, .dyn_pwr = 101103,
			.lkg_pwr = {43483, 43483, 43483, 43483, 43483, 43483} },
		{.cap = 290, .volt = 96875, .dyn_pwr = 113304,
			.lkg_pwr = {47374, 47374, 47374, 47374, 47374, 47374} },
		{.cap = 296, .volt = 100000, .dyn_pwr = 124630,
			.lkg_pwr = {50836, 50836, 50836, 50836, 50836, 50836} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
		{{0}, {19715} },
	},
};

struct upower_tbl upower_tbl_cluster_l_6883 = {
	.row = {
		{.cap = 132, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 161, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 170, .volt = 61250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 179, .volt = 63125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 193, .volt = 65625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 205, .volt = 68750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 215, .volt = 71250, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 231, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 242, .volt = 77500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 252, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 260, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 271, .volt = 88125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 278, .volt = 90625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 284, .volt = 93125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 290, .volt = 96875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 296, .volt = 100000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_b_6883 = {
	.row = {
		{.cap = 605, .volt = 60000, .dyn_pwr = 79872,
			.lkg_pwr = {81046, 81046, 81046, 81046, 81046, 81046} },
		{.cap = 655, .volt = 62500, .dyn_pwr = 97945,
			.lkg_pwr = {87427, 87427, 87427, 87427, 87427, 87427} },
		{.cap = 697, .volt = 64375, .dyn_pwr = 116001,
			.lkg_pwr = {92213, 92213, 92213, 92213, 92213, 92213} },
		{.cap = 749, .volt = 67500, .dyn_pwr = 145123,
			.lkg_pwr = {100176, 100176, 100176, 100176, 100176,
						100176} },
		{.cap = 802, .volt = 70625, .dyn_pwr = 177071,
			.lkg_pwr = {108133, 108133, 108133, 108133, 108133,
						108133} },
		{.cap = 836, .volt = 72500, .dyn_pwr = 200653,
			.lkg_pwr = {112905, 112905, 112905, 112905, 112905,
						112905} },
		{.cap = 865, .volt = 75000, .dyn_pwr = 229764,
			.lkg_pwr = {119268, 119268, 119268, 119268, 119268,
						119268} },
		{.cap = 890, .volt = 76875, .dyn_pwr = 257017,
			.lkg_pwr = {124646, 124646, 124646, 124646, 124646,
						124646} },
		{.cap = 907, .volt = 78750, .dyn_pwr = 280365,
			.lkg_pwr = {130024, 130024, 130024, 130024, 130024,
						130024} },
		{.cap = 937, .volt = 80625, .dyn_pwr = 310738,
			.lkg_pwr = {135442, 135442, 135442, 135442, 135442,
						135442} },
		{.cap = 972, .volt = 83750, .dyn_pwr = 356771,
			.lkg_pwr = {144608, 144608, 144608, 144608, 144608,
						144608} },
		{.cap = 985, .volt = 85625, .dyn_pwr = 381998,
			.lkg_pwr = {150469, 150469, 150469, 150469, 150469,
						150469} },
		{.cap = 994, .volt = 87500, .dyn_pwr = 405972,
			.lkg_pwr = {157052, 157052, 157052, 157052, 157052,
						157052} },
		{.cap = 1006, .volt = 89375, .dyn_pwr = 433914,
			.lkg_pwr = {163636, 163636, 163636, 163636, 163636,
						163636} },
		{.cap = 1020, .volt = 92500, .dyn_pwr = 479117,
			.lkg_pwr = {174873, 174873, 174873, 174873, 174873,
						174873} },
		{.cap = 1024, .volt = 93125, .dyn_pwr = 490386,
			.lkg_pwr = {177134, 177134, 177134, 177134, 177134,
						177134} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
		{{0}, {81046} },
	},
};

struct upower_tbl upower_tbl_cluster_b_6883 = {
	.row = {
		{.cap = 605, .volt = 60000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 655, .volt = 62500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 697, .volt = 64375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 749, .volt = 67500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 802, .volt = 70625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 836, .volt = 72500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 865, .volt = 75000, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 890, .volt = 76875, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 907, .volt = 78750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 937, .volt = 80625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 972, .volt = 83750, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 985, .volt = 85625, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 994, .volt = 87500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1006, .volt = 89375, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1020, .volt = 92500, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
		{.cap = 1024, .volt = 93125, .dyn_pwr = 0,
			.lkg_pwr = {0, 0, 0, 0, 0, 0} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
		{{0}, {0} },
	},
};

struct upower_tbl upower_tbl_cci_6883 = {
	.row = {
		{.cap = 0, .volt = 60000, .dyn_pwr = 8776,
			.lkg_pwr = {44739, 44739, 44739, 44739, 44739, 44739} },
		{.cap = 0, .volt = 62500, .dyn_pwr = 10677,
			.lkg_pwr = {47558, 47558, 47558, 47558, 47558, 47558} },
		{.cap = 0, .volt = 65000, .dyn_pwr = 13232,
			.lkg_pwr = {50377, 50377, 50377, 50377, 50377, 50377} },
		{.cap = 0, .volt = 66875, .dyn_pwr = 14908,
			.lkg_pwr = {52793, 52793, 52793, 52793, 52793, 52793} },
		{.cap = 0, .volt = 70625, .dyn_pwr = 19105,
			.lkg_pwr = {57625, 57625, 57625, 57625, 57625, 57625} },
		{.cap = 0, .volt = 72500, .dyn_pwr = 21710,
			.lkg_pwr = {60041, 60041, 60041, 60041, 60041, 60041} },
		{.cap = 0, .volt = 75000, .dyn_pwr = 24921,
			.lkg_pwr = {63262, 63262, 63262, 63262, 63262, 63262} },
		{.cap = 0, .volt = 78125, .dyn_pwr = 29473,
			.lkg_pwr = {67758, 67758, 67758, 67758, 67758, 67758} },
		{.cap = 0, .volt = 80625, .dyn_pwr = 33980,
			.lkg_pwr = {71562, 71562, 71562, 71562, 71562, 71562} },
		{.cap = 0, .volt = 83750, .dyn_pwr = 39296,
			.lkg_pwr = {77092, 77092, 77092, 77092, 77092, 77092} },
		{.cap = 0, .volt = 86875, .dyn_pwr = 44442,
			.lkg_pwr = {83030, 83030, 83030, 83030, 83030, 83030} },
		{.cap = 0, .volt = 89375, .dyn_pwr = 49171,
			.lkg_pwr = {87998, 87998, 87998, 87998, 87998, 87998} },
		{.cap = 0, .volt = 91875, .dyn_pwr = 54216,
			.lkg_pwr = {93641, 93641, 93641, 93641, 93641, 93641} },
		{.cap = 0, .volt = 94375, .dyn_pwr = 59545,
			.lkg_pwr = {99509, 99509, 99509, 99509, 99509, 99509} },
		{.cap = 0, .volt = 96875, .dyn_pwr = 64634,
			.lkg_pwr = {106501, 106501, 106501, 106501, 106501,
						106501} },
		{.cap = 0, .volt = 100000, .dyn_pwr = 72199,
			.lkg_pwr = {115708, 115708, 115708, 115708, 115708,
						115708} },
	},
	.lkg_idx = DEFAULT_LKG_IDX,
	.row_num = UPOWER_OPP_NUM,
	.nr_idle_states = NR_UPOWER_CSTATES,
	.idle_states = {
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
		{{0}, {44739} },
	},
};

#endif /* UNIFIED_POWER_DATA_H */
