/*
 * kd_camkit_define_xa.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: define image sensor parameters
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

#ifndef KD_CAMKIT_DEFINE_XA_H
#define KD_CAMKIT_DEFINE_XA_H

/*
 * define sensor normalized parameters as follow:
 */
// sensor id wn Main:577 Sub:441 Wide:646 Macro:562
// IMX
#define C645WMR_M010_WN_SENSOR_ID            0x0258011
#define C645WMR_M0A0_WN_SENSOR_ID            0x02580A1
// OV
#define C585WGU_WN_SENSOR_ID                 0x002B001

#define C562YGA_M010_WN_SENSOR_ID            0x2509011
// Hynix
#define C645UAI_M090_WN_SENSOR_ID            0x1336091

#define C441UVO_M060_WN_SENSOR_ID            0x0846061
#define C441UVO_M030_WN_SENSOR_ID            0x0846031

#define C646HQF_M0B0_WN_SENSOR_ID            0x05560B1

#define C637QVV_M0C0_WN_SENSOR_ID            0x00E10C1
// GC
#define C646KEH_M030_WN_SENSOR_ID            0x5035031

#define C441FZB_M050_WN_SENSOR_ID            0x8054051

#define C585GFI_WN_SENSOR_ID                 0x2375001
#define C562EOY_M020_WN_SENSOR_ID            0x2375021
#define C562EOY_M0B0_WN_SENSOR_ID            0x23750b1
// Sumsung
#define C645XBA_M0C0_WN_SENSOR_ID            0x30C60c1

#define C441FAH_M0C0_WN_SENSOR_ID            0x487B0c1

#define C646TBQ_M050_WN_SENSOR_ID            0x059b051

// SENSOR_DRVNAME
// IMX
#define SENSOR_DRVNAME_C645WMR_M010_WN       "c645wmr_m010_wn"
#define SENSOR_DRVNAME_C645WMR_M0A0_WN       "c645wmr_m0a0_wn"
// OV
#define SENSOR_DRVNAME_C585WGU_WN            "c585wgu_wn"

#define SENSOR_DRVNAME_C562YGA_M010_WN       "c562yga_m010_wn"
// Hynix
#define SENSOR_DRVNAME_C645UAI_M090_WN       "c645uai_m090_wn"

#define SENSOR_DRVNAME_C441UVO_M060_WN       "c441uvo_m060_wn"
#define SENSOR_DRVNAME_C441UVO_M030_WN       "c441uvo_m030_wn"

#define SENSOR_DRVNAME_C646HQF_M0B0_WN       "c646hqf_m0b0_wn"

#define SENSOR_DRVNAME_C637QVV_M0C0_WN       "c637qvv_m0c0_wn"
// GC
#define SENSOR_DRVNAME_C646KEH_M030_WN       "c646keh_m030_wn"

#define SENSOR_DRVNAME_C441FZB_M050_WN       "c441fzb_m050_wn"

#define SENSOR_DRVNAME_C585GFI_WN            "c585gfi_wn"
#define SENSOR_DRVNAME_C562EOY_M020_WN       "c562eoy_m020_wn"
#define SENSOR_DRVNAME_C562EOY_M0B0_WN       "c562eoy_m0b0_wn"
// Sumsung
#define SENSOR_DRVNAME_C645XBA_M0C0_WN       "c645xba_m0c0_wn"

#define SENSOR_DRVNAME_C441FAH_M0C0_WN       "c441fah_m0c0_wn"

#define SENSOR_DRVNAME_C646TBQ_M050_WN       "c646tbq_m050_wn"

#endif // KD_CAMKIT_MERIDA2CAM_H
