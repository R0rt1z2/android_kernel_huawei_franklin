/*
 * ke_camkit_define.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * define image sensor parameters
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

#ifndef KD_CAMKIT_DEFINE_H
#define KD_CAMKIT_DEFINE_H

#include "kd_camkit_define_bj.h"
#include "kd_camkit_define_sh.h"
#include "kd_camkit_define_xa.h"
#include "kd_camkit_define_wh.h"

/*
 * define module id and module name as follow:
 */

// Main
#define OV48B2Q_LUXVISIONS_SENSOR_ID      0x4842030
#define OV48B2Q_OFILM_SENSOR_ID           0x4842060
#define IMX582_SUNNY_SENSOR_ID            0x0582010
#define IMX582_FOXCONN_SENSOR_ID          0x0582020
#define S5KGW1_TXD_SENSOR_ID              0x0971190
#define S5KGW1_XL_SENSOR_ID               0x0971060
#define S5KGW1_QT_SENSOR_ID               0x09710B0
#define HI1336_QTECH_SENSOR_ID            0x1336
#define S5K3L6_TRULY_SENSOR_ID            0x30C6
#define IMX258_SUNNY_SENSOR_ID            0x0258010
#define IMX258_HOLITECH_SENSOR_ID         0x02580A0
#define C577QCP_M060_SENSOR_ID            0x0d42060
#define C591RAO_M020_SENSOR_ID            0x0600020
#define C591RAO_M060_SENSOR_ID            0x0600060
#define C591RAO_M090_SENSOR_ID            0x0600090
#define C591LKY_M020_SENSOR_ID            0x0589020
#define C591LKY_M060_SENSOR_ID            0x0589060
#define C591LKY_M090_SENSOR_ID            0x0589090
#define C483DVY_M030_SENSOR_ID            0x4842030
#define C483DVY_M060_SENSOR_ID            0x4842060
#define C483DBC_M010_SENSOR_ID            0x0582010
#define C483DBC_M020_SENSOR_ID            0x0582020
#define C601KES_M060_JSC_SENSOR_ID        0x068206F
#define C601KNK_M010_JSC_SENSOR_ID        0x097101F
#define C601GHA_M090_JSC_SENSOR_ID        0x644309F
#define S5KGW1_TXD_LARGAN_SENSOR_ID       0x0971191
#define C647KES_M060_CDY_SENSOR_ID        0x068206E
#define C647KES_M020_CDY_SENSOR_ID        0x068202E
#define C647KNK_M090_CDY_SENSOR_ID        0x097109E
#define C7021DVY_M010_SENSOR_ID           0x4842190
#define C7011DVY_M120_SENSOR_ID           0x4842120

// Sub
#define S5K3P9_SUNNY_SENSOR_ID            0x3109010
#define IMX471_FOXCONN_SENSOR_ID          0x0471020
#define HI1634Q_FOXCONN_SENSOR_ID         0x1634020
#define OV16A1Q_OFILM_SENSOR_ID           0x1641060
#define HI846_OFILM_SENSOR_ID             0x0846060
#define HI846_LUXVISIONS_SENSOR_ID        0x0846030
#define S5K4H7_TRULY_SENSOR_ID            0x487B0C0
#define GC8054_BYD_SENSOR_ID              0x8054050
#define S5K3P9_TXD_SENSOR_ID              0x3109190
#define S5K3P9_TXD_TRULY_SENSOR_ID        0x3109191
#define HI1631Q_ST_SENSOR_ID              0x1631170
#define C576OGI_M020_SENSOR_ID            0x1634020
#define C576FUV_M010_SENSOR_ID            0x1641010
#define C576JDI_M090_SENSOR_ID            0x0471090
#define C576JDI_M060_SENSOR_ID            0x0471060
#define C627OGU_M010_JSC_SENSOR_ID        0x310901F
#define C627OGI_M020_JSC_SENSOR_ID        0x163402F
#define C627FUV_M060_JSC_SENSOR_ID        0x164106F
#define C627JDI_M020_JSC_SENSOR_ID        0x047102F
#define C627OGU_M010_CDY_SENSOR_ID        0x310901E
#define C627OGI_M020_CDY_SENSOR_ID        0x163402E
#define C627FUV_M060_CDY_SENSOR_ID        0x164106E
#define C627JDI_M020_CDY_SENSOR_ID        0x047102E
#define HI1631Q_ST_TRULY_SENSOR_ID        0x1631171
#define C7041UVO_M120_SENSOR_ID           0x0846180
#define C441FAH_M120_SENSOR_ID            0x487B0C0
#define C441UVO_M060_SENSOR_ID            0x0846060
#define C441FZB_M050_SENSOR_ID            0x8054050
#define C441UVO_M030_SENSOR_ID            0x0846030
#define C7031HRJ_M170_SENSOR_ID           0x8044170

// Wide
#define HI846_SUNNY_SENSOR_ID             0x0846010
#define OV8856_FOXCONN_SENSOR_ID          0x885A020
#define OV8856_OFILM_SENSOR_ID            0x885A060
#define GC8054_QTECH_SENSOR_ID            0x8054090
#define HI846_TXD_SENSOR_ID               0x0846190
#define GC8034_LY_SENSOR_ID               0x8044180
#define C512JWG_M020_SENSOR_ID            0x885A020
#define C512JWG_M060_SENSOR_ID            0x885A060
#define C512TBQ_M050_SENSOR_ID            0x559b050
#define C512KEH_M030_SENSOR_ID            0x5035030
#define C646HQF_M110_SENSOR_ID            0x05560B0
#define C570OAK_M020_SENSOR_ID            0x48AB020
#define C570FZB_M030_SENSOR_ID            0x8054030
#define C570JWG_M090_SENSOR_ID            0x885A090
#define C570UVO_M010_SENSOR_ID            0x0846010
#define IMX355_QTECH_SENSOR_ID            0x0355090
#define C628UVO_M010_JSC_SENSOR_ID        0x084601F
#define C628JWG_M020_JSC_SENSOR_ID        0x885A02F
#define C628JWG_M060_JSC_SENSOR_ID        0x885A06F
#define C628BDH_M090_JSC_SENSOR_ID        0x035509F
#define C628UVO_M010_CDY_SENSOR_ID        0x084601E
#define C628JWG_M020_CDY_SENSOR_ID        0x885A02E
#define C628JWG_M060_CDY_SENSOR_ID        0x885A06E
#define C628BDH_M090_CDY_SENSOR_ID        0x035509E
#define C6021UVO_M190_SENSOR_ID           0x0846190
#define C4021HRJ_M180_SENSOR_ID           0x8044180

// Macro
#define OV02A10_SUNNY_SENSOR_ID           0x2509010
#define GC2375_FOXCONN_SENSOR_ID          0x2375020
#define GC2375_SUNWIN_SENSOR_ID           0x23750B0  // 0: DM, 1: OK
#define HI259_SUNWIN_SENSOR_ID            0x00E10B0
#define C394YGA_M010_SENSOR_ID            0x2509010
#define C394YGA_M0B0_SENSOR_ID            0x25090B0
#define C394EOY_M090_SENSOR_ID            0x2375090
#define C394EOY_M0C0_SENSOR_ID            0x23750C0
#define C394QVV_M0B0_FRL_SENSOR_ID        0xC00E10B0
#define C648YGA_M010_JSC_SENSOR_ID        0x250901F
#define C648YGA_M110_JSC_SENSOR_ID        0x25090BF
#define C648EOY_M120_JSC_SENSOR_ID        0x23750CF
#define C637EOY_M110_CDY_SENSOR_ID        0x237511E
#define C637EOY_M020_CDY_SENSOR_ID        0x237502E
#define C637YGA_M010_CDY_SENSOR_ID        0x250901E
#define C637QVV_M020_CDY_SENSOR_ID        0x00E102E

// depth
#define GC2375B_SENSOR_ID                 0x2375D
#define OV02B1B_SENSOR_ID                 0x002BD
#define C632GFI_JSC_SENSOR_ID             0x2375F
#define C632WGU_JSC_SENSOR_ID             0x002BF
#define C585GFI_CDY_SENSOR_ID             0x2375E
#define C585WGU_CDY_SENSOR_ID             0x002BE
#define C585GFI_SENSOR_ID                 0x2375
#define C585QUV_SENSOR_ID                 0x002B

// Main
#define SENSOR_DRVNAME_OV48B2Q_LUXVISIONS "ov48b2q_luxvisions"
#define SENSOR_DRVNAME_OV48B2Q_OFILM      "ov48b2q_ofilm"
#define SENSOR_DRVNAME_IMX582_SUNNY       "imx582_sunny"
#define SENSOR_DRVNAME_IMX582_FOXCONN     "imx582_foxconn"
#define SENSOR_DRVNAME_S5KGW1_TXD         "s5kgw1_txd"
#define SENSOR_DRVNAME_S5KGW1_XL          "s5kgw1_xl"
#define SENSOR_DRVNAME_S5KGW1_QT          "s5kgw1_qt"
#define SENSOR_DRVNAME_C591RAO_M020       "c591rao_m020"
#define SENSOR_DRVNAME_C591RAO_M060       "c591rao_m060"
#define SENSOR_DRVNAME_C591RAO_M090       "c591rao_m090"
#define SENSOR_DRVNAME_C591LKY_M020       "c591lky_m020"
#define SENSOR_DRVNAME_C591LKY_M060       "c591lky_m060"
#define SENSOR_DRVNAME_C591LKY_M090       "c591lky_m090"
#define SENSOR_DRVNAME_C483DVY_M030       "c483dvy_m030"
#define SENSOR_DRVNAME_C483DVY_M060       "c483dvy_m060"
#define SENSOR_DRVNAME_C483DBC_M010       "c483dbc_m010"
#define SENSOR_DRVNAME_C483DBC_M020       "c483dbc_m020"
#define SENSOR_DRVNAME_C601KES_M060_JSC   "c601kes_m060_jsc"
#define SENSOR_DRVNAME_C601KNK_M010_JSC   "c601knk_m010_jsc"
#define SENSOR_DRVNAME_C601GHA_M090_JSC   "c601gha_m090_jsc"
#define SENSOR_DRVNAME_S5KGW1_TXD_LARGAN  "s5kgw1_txd_largan"
#define SENSOR_DRVNAME_C647KES_M060_CDY   "c647kes_m060_cdy"
#define SENSOR_DRVNAME_C647KES_M020_CDY   "c647kes_m020_cdy"
#define SENSOR_DRVNAME_C647KNK_M090_CDY   "c647knk_m090_cdy"
#define SENSOR_DRVNAME_C7021DVY_M010      "c7021dvy_m010"
#define SENSOR_DRVNAME_C7011DVY_M120      "c7011dvy_m120"

// Sub
#define SENSOR_DRVNAME_S5K3P9_SUNNY       "s5k3p9_sunny"
#define SENSOR_DRVNAME_IMX471_FOXCONN     "imx471_foxconn"
#define SENSOR_DRVNAME_HI1634Q_FOXCONN    "hi1634q_foxconn"
#define SENSOR_DRVNAME_OV16A1Q_OFILM      "ov16a1q_ofilm"
#define SENSOR_DRVNAME_HI846_OFILM        "hi846_ofilm"
#define SENSOR_DRVNAME_HI846_LUXVISIONS   "hi846_luxvisions"
#define SENSOR_DRVNAME_S5K4H7_TRULY       "s5k4h7_truly"
#define SENSOR_DRVNAME_GC8054_BYD         "gc8054_byd"
#define SENSOR_DRVNAME_S5K3P9_TXD         "s5k3p9_txd"
#define SENSOR_DRVNAME_S5K3P9_TXD_TRULY   "s5k3p9_txd_truly"
#define SENSOR_DRVNAME_HI1631Q_ST         "hi1631q_st"
#define SENSOR_DRVNAME_C576OGI_M020       "c576ogi_m020"
#define SENSOR_DRVNAME_C576FUV_M010       "c576fuv_m010"
#define SENSOR_DRVNAME_C576JDI_M090       "c576jdi_m090"
#define SENSOR_DRVNAME_C576JDI_M060       "c576jdi_m060"
#define SENSOR_DRVNAME_C627OGU_M010_JSC   "c627ogu_m010_jsc"
#define SENSOR_DRVNAME_C627OGI_M020_JSC   "c627ogi_m020_jsc"
#define SENSOR_DRVNAME_C627FUV_M060_JSC   "c627fuv_m060_jsc"
#define SENSOR_DRVNAME_C627JDI_M020_JSC   "c627jdi_m020_jsc"
#define SENSOR_DRVNAME_C627OGU_M010_CDY   "c627ogu_m010_cdy"
#define SENSOR_DRVNAME_C627OGI_M020_CDY   "c627ogi_m020_cdy"
#define SENSOR_DRVNAME_C627FUV_M060_CDY   "c627fuv_m060_cdy"
#define SENSOR_DRVNAME_C627JDI_M020_CDY   "c627jdi_m020_cdy"
#define SENSOR_DRVNAME_HI1631Q_ST_TRULY   "hi1631q_st_truly"
#define SENSOR_DRVNAME_C7041UVO_M120      "c7041uvo_m120"
#define SENSOR_DRVNAME_C441FAH_M120       "c441fah_m120"
#define SENSOR_DRVNAME_C441UVO_M060       "c441uvo_m060"
#define SENSOR_DRVNAME_C441FZB_M050       "c441fzb_m050"
#define SENSOR_DRVNAME_C441UVO_M030       "c441uvo_m030"
#define SENSOR_DRVNAME_C7031HRJ_M170      "c7031hrj_m170"

// Wide
#define SENSOR_DRVNAME_HI846_SUNNY        "hi846_sunny"
#define SENSOR_DRVNAME_OV8856_FOXCONN     "ov8856_foxconn"
#define SENSOR_DRVNAME_OV8856_OFILM       "ov8856_ofilm"
#define SENSOR_DRVNAME_GC8054_QTECH       "gc8054_qtech"
#define SENSOR_DRVNAME_HI846_TXD          "hi846_txd"
#define SENSOR_DRVNAME_GC8034_LY          "gc8034_ly"
#define SENSOR_DRVNAME_C512JWG_M020       "c512jwg_m020"
#define SENSOR_DRVNAME_C512JWG_M060       "c512jwg_m060"
#define SENSOR_DRVNAME_C512TBQ_M050       "c512tbq_m050"
#define SENSOR_DRVNAME_C512KEH_M030       "c512keh_m030"
#define SENSOR_DRVNAME_C646HQF_M110       "c646hqf_m110"
#define SENSOR_DRVNAME_C570OAK_M020       "c570oak_m020"
#define SENSOR_DRVNAME_C570FZB_M030       "c570fzb_m030"
#define SENSOR_DRVNAME_C570JWG_M090       "c570jwg_m090"
#define SENSOR_DRVNAME_C570UVO_M010       "c570uvo_m010"
#define SENSOR_DRVNAME_IMX355_QTECH       "imx355_qtech"
#define SENSOR_DRVNAME_C628UVO_M010_JSC   "c628uvo_m010_jsc"
#define SENSOR_DRVNAME_C628JWG_M020_JSC   "c628jwg_m020_jsc"
#define SENSOR_DRVNAME_C628JWG_M060_JSC   "c628jwg_m060_jsc"
#define SENSOR_DRVNAME_C628BDH_M090_JSC   "c628bdh_m090_jsc"
#define SENSOR_DRVNAME_C628UVO_M010_CDY   "c628uvo_m010_cdy"
#define SENSOR_DRVNAME_C628JWG_M020_CDY   "c628jwg_m020_cdy"
#define SENSOR_DRVNAME_C628JWG_M060_CDY   "c628jwg_m060_cdy"
#define SENSOR_DRVNAME_C628BDH_M090_CDY   "c628bdh_m090_cdy"
#define SENSOR_DRVNAME_C6021UVO_M190      "c6021uvo_m190"
#define SENSOR_DRVNAME_C4021HRJ_M180      "c4021hrj_m180"

// Macro
#define SENSOR_DRVNAME_OV02A10_SUNNY      "ov02a10_sunny"
#define SENSOR_DRVNAME_GC2375_FOXCONN     "gc2375_foxconn"
#define SENSOR_DRVNAME_GC2375_SUNWIN      "gc2375_sunwin"
#define SENSOR_DRVNAME_HI259_SUNWIN       "hi259_sunwin"
#define SENSOR_DRVNAME_C394YGA_M010       "c394yga_m010"
#define SENSOR_DRVNAME_C394YGA_M0B0       "c394yga_m0b0"
#define SENSOR_DRVNAME_C394EOY_M090       "c394eoy_m090"
#define SENSOR_DRVNAME_C394EOY_M0C0       "c394eoy_m0c0"
#define SENSOR_DRVNAME_C394QVV_M0B0_FRL   "c394qvv_m0b0_frl"
#define SENSOR_DRVNAME_C648YGA_M010_JSC   "c648yga_m010_jsc"
#define SENSOR_DRVNAME_C648YGA_M110_JSC   "c648yga_m110_jsc"
#define SENSOR_DRVNAME_C648EOY_M120_JSC   "c648eoy_m120_jsc"
#define SENSOR_DRVNAME_C637EOY_M110_CDY   "c637eoy_m110_cdy"
#define SENSOR_DRVNAME_C637EOY_M020_CDY   "c637eoy_m020_cdy"
#define SENSOR_DRVNAME_C637YGA_M010_CDY   "c637yga_m010_cdy"
#define SENSOR_DRVNAME_C637QVV_M020_CDY   "c637qvv_m020_cdy"

// depth
#define SENSOR_DRVNAME_GC2375B            "gc2375b"
#define SENSOR_DRVNAME_OV02B1B            "ov02b1b"
#define SENSOR_DRVNAME_C632GFI_JSC        "c632gfi_jsc"
#define SENSOR_DRVNAME_C632WGU_JSC        "c632wgu_jsc"
#define SENSOR_DRVNAME_C585GFI_CDY        "c585gfi_cdy"
#define SENSOR_DRVNAME_C585WGU_CDY        "c585wgu_cdy"
#define SENSOR_DRVNAME_C585GFI            "c585gfi"
#define SENSOR_DRVNAME_C585QUV            "c585quv"

/*
 * define sensor normalized parameters as follow:
 */
#define int8 signed char
#define int16 signed short
#define int32 signed int
#define uint8 unsigned char
#define uint16 unsigned short
#define uint32 unsigned int

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define CAMKIT_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define CAMKIT_LINGLENGTH_GAP 80

#define MAX_OUTPUT_INFO_SIZE 16
#define MAX_AEC_REGS 25
#define MAX_AGAIN_REGS 16
#define SENSOR_NAME_LEN 32
#define MOTOR_NAME_LEN 32
#define OIS_NAME_LEN 32
#define MAX_DRV_NUM 3
#define MAX_FUSE_ID_SIZE 30
#define MAX_HIGH_IMPEDANCE_SENSOR_NUM 3
#define LCD_NAME_LEN 32
#define MAX_EXT_POWER_INFO_NUM 5
#define MAX_VC_NUM 8

typedef enum {
	CAMKIT_I2C_BYTE_ADDR = 1,
	CAMKIT_I2C_WORD_ADDR,
	CAMKIT_I2C_ADDR_MAX,
} camkit_i2c_addr_type;

typedef enum {
	CAMKIT_I2C_BYTE_DATA = 1,
	CAMKIT_I2C_WORD_DATA,
	CAMKIT_I2C_3B_DATA,
	CAMKIT_I2C_DWORD_DATA,
	CAMKIT_I2C_DATA_MAX,
} camkit_i2c_data_type;

typedef enum {
	CAMKIT_I2C_WRITE,
	CAMKIT_I2C_READ,
	CAMKIT_I2C_POLL,
	CAMKIT_I2C_READ_BUFFER,
} camkit_i2c_operation;

typedef enum {
	CAMKIT_I2C_DEV_0,
	CAMKIT_I2C_DEV_1,
	CAMKIT_I2C_DEV_2,
	CAMKIT_I2C_DEV_3,
	CAMKIT_I2C_DEV_4,
	CAMKIT_I2C_DEV_5,
	CAMKIT_I2C_DEV_6,
	CAMKIT_I2C_DEV_7,
	CAMKIT_I2C_DEV_MAX_NUM,
} camkit_i2c_dev_id_t;

struct camkit_i2c_reg {
	uint16 addr;
	uint16 data;
	uint16 delay;
};

struct camkit_i2c_reg_table {
	uint16 addr;
	uint16 data;
	camkit_i2c_data_type data_type;
	camkit_i2c_operation i2c_operation;
	uint16 delay;
};

struct camkit_i2c_reg_table_array {
	struct camkit_i2c_reg_table *setting;
	uint16 size;
};

typedef enum {
	CAMKIT_MODE_INIT,
	CAMKIT_MODE_PREVIEW,
	CAMKIT_MODE_CAPTURE,
	CAMKIT_MODE_VIDEO,
	CAMKIT_MODE_HIGH_SPEED_VIDEO,
	CAMKIT_MODE_SLIM_VIDEO,
	CAMKIT_MODE_CUSTOM1,
	CAMKIT_MODE_CUSTOM2,
	CAMKIT_MODE_CUSTOM3,
	CAMKIT_MODE_CUSTOM4,
	CAMKIT_MODE_CUSTOM5,
} camkit_common_mode;

typedef enum {
	CAMKIT_SCENARIO_ID_CAMERA_PREVIEW = 0,
	CAMKIT_SCENARIO_ID_CAMERA_CAPTURE_JPEG,
	CAMKIT_SCENARIO_ID_VIDEO_PREVIEW,
	CAMKIT_SCENARIO_ID_HIGH_SPEED_VIDEO,
	CAMKIT_SCENARIO_ID_SLIM_VIDEO,
	CAMKIT_SCENARIO_ID_CUSTOM1,
	CAMKIT_SCENARIO_ID_CUSTOM2,
	CAMKIT_SCENARIO_ID_CUSTOM3,
	CAMKIT_SCENARIO_ID_CUSTOM4,
	CAMKIT_SCENARIO_ID_CUSTOM5,
	CAMKIT_SCENARIO_ID_CAMERA_ZSD,
	CAMKIT_SCENARIO_ID_CAMERA_3D_PREVIEW,
	CAMKIT_SCENARIO_ID_CAMERA_3D_CAPTURE,
	CAMKIT_SCENARIO_ID_CAMERA_3D_VIDEO,
	CAMKIT_SCENARIO_ID_TV_OUT,
	CAMKIT_SCENARIO_ID_MAX,
} camkit_scenario_type;

struct camkit_mode_info {
	uint32 pclk;
	uint32 linelength;
	uint32 framelength;

	uint8 startx;
	uint8 starty;

	uint16 grabwindow_width;
	uint16 grabwindow_height;

	/*
	 * following for MIPIDataLowPwr2HighSpeedSettleDelayCount
	 * by different scenario
	 */
	uint8 mipi_data_lp2hs_settle_dc;

	/* following for GetDefaultFramerateByScenario() */
	uint16 max_framerate;
	uint32 mipi_pixel_rate;
	uint32 mipi_trail_val;
	uint8 pdaf_support;
	uint8 need_set_awb_gain;
	uint32 mipi_mask_val;
};

struct camkit_i2c_reg_setting {
	struct camkit_i2c_reg *setting;
	uint16 size;
	uint16 delay;
	camkit_i2c_addr_type addr_type;
	camkit_i2c_data_type data_type;
};

typedef enum {
	CAMKIT_SEAMLESS_UNSUPPORT = 0,
	CAMKIT_SEAMLESS_SUPPORT = 1,
} camkit_seamless_support_t;

struct camkit_seamless_scenario_id_t {
	camkit_scenario_type src_scenario_id;
	camkit_scenario_type dst_scenario_id;
};

struct camkit_seamless_scenario_cfg_t {
	camkit_scenario_type dst_scenario_id;
	struct camkit_i2c_reg_setting seamless_setting;
};

struct camkit_seamless_cfg_t {
	camkit_seamless_support_t seamless_support;
	struct camkit_seamless_scenario_id_t *scenario_id_map;
	uint32 scenario_id_map_size;
	struct camkit_seamless_scenario_cfg_t *scenario_cfg;
	uint32 scenario_cfg_size;
};

typedef enum {
	CAMKIT_VC_NONE = 0,
	CAMKIT_VC_MIN_NUM = 1,
	CAMKIT_VC_RAW_DATA = CAMKIT_VC_MIN_NUM,
	CAMKIT_VC_RAW_DATA_MAX = 2,

	CAMKIT_VC_PDAF_MIN_NUM = CAMKIT_VC_RAW_DATA_MAX,
	CAMKIT_VC_PDAF_STATS = CAMKIT_VC_PDAF_MIN_NUM,
	CAMKIT_VC_PDAF_MAX_NUM = 3,

	CAMKIT_VC_HDR_MIN_NUM = CAMKIT_VC_PDAF_MAX_NUM,
	CAMKIT_VC_HDR_MVHDR = CAMKIT_VC_HDR_MIN_NUM,
	CAMKIT_VC_HDR_MAX_NUM = 4,

	CAMKIT_VC_3HDR_MIN_NUM = CAMKIT_VC_HDR_MAX_NUM,
	CAMKIT_VC_3HDR_EMBEDDED = CAMKIT_VC_3HDR_MIN_NUM,
	CAMKIT_VC_3HDR_FLICKER = 5,
	CAMKIT_VC_3HDR_Y = 6,
	CAMKIT_VC_3HDR_AE = 7,
	CAMKIT_VC_3HDR_MAX_NUM = 8,
	CAMKIT_VC_STAGGER_MIN_NUM = CAMKIT_VC_3HDR_MAX_NUM,
	CAMKIT_VC_STAGGER_EMBEDDED = CAMKIT_VC_STAGGER_MIN_NUM,
	CAMKIT_VC_STAGGER_NE = 9,
	CAMKIT_VC_STAGGER_ME = 10,
	CAMKIT_VC_STAGGER_SE = 11,
	CAMKIT_VC_STAGGER_MAX_NUM = 12,
	CAMKIT_VC_MAX_NUM = CAMKIT_VC_STAGGER_MAX_NUM,
} camkit_vc_feature_t;

typedef enum {
	CAMKIT_HDR_SUPPORT_NA = 0,
	CAMKIT_HDR_SUPPORT_RAW = 1,
	CAMKIT_HDR_SUPPORT_CAMSV = 2,
	CAMKIT_ZHDR_SUPPORT_RAW = 3,
	CAMKIT_MVHDR_SUPPORT_MULTICAMSV = 4,
	CAMKIT_HDR_SUPPORT_STAGGER_MIN = 5,
	CAMKIT_HDR_SUPPORT_STAGGER_DOL = CAMKIT_HDR_SUPPORT_STAGGER_MIN,
	CAMKIT_HDR_SUPPORT_STAGGER_FDOL = 6,
	CAMKIT_HDR_SUPPORT_STAGGER_NDOL = 7,
	CAMKIT_HDR_SUPPORT_STAGGER_MAX,
} camkit_hdr_support_type_t;

typedef enum {
	CAMKIT_HDR_NONE = 0,
	CAMKIT_HDR_RAW = 1,
	CAMKIT_HDR_CAMSV = 2,
	CAMKIT_HDR_RAW_ZHDR = 9,
	CAMKIT_HDR_MULTICAMSV = 0x0A,
	CAMKIT_HDR_RAW_STAGGER_2EXP = 0x0B,
	CAMKIT_HDR_RAW_STAGGER_MIN = CAMKIT_HDR_RAW_STAGGER_2EXP,
	CAMKIT_HDR_RAW_STAGGER_3EXP = 0x0C,
	CAMKIT_HDR_RAW_STAGGER_MAX = CAMKIT_HDR_RAW_STAGGER_3EXP,
} camkit_hdr_mode_t;

struct camkit_stagger_max_expo_t {
	camkit_vc_feature_t vc_feature;
	uint32 max_expo;
};

struct camkit_stagger_scenario_id_t {
	camkit_scenario_type src_scenario_id;
	camkit_hdr_mode_t hdr_type;
	camkit_scenario_type dst_scenario_id;
};

struct camkit_stagger_type_id_t {
	camkit_scenario_type scenario_id;
	camkit_hdr_mode_t stagger_hdr_type;
};

struct camkit_stagger_cfg_t {
	uint8  hdr_support;
	struct camkit_stagger_scenario_id_t *scenario_id_map;
	uint32 scenario_id_map_size;
	struct camkit_stagger_type_id_t *stagger_type_map;
	uint32 stagger_type_map_size;
	struct camkit_stagger_max_expo_t *stagger_max_expo;
	uint32 stagger_max_expo_size;
};

/* SENSOR PRIVATE STRUCT FOR CURRENT VARIABLES */
struct camkit_sensor_ctrl_t {
	uint8  mirror;
	uint8  sensor_mode;
	uint32 shutter;
	uint16 gain;
	uint32 pclk;
	uint32 frame_length;
	uint32 line_length;
	uint32 min_frame_length;
	uint16 dummy_pixel;
	uint16 dummy_line;
	uint16 current_fps;
	uint8  autoflicker_en;
	uint8  test_pattern;
	camkit_scenario_type current_scenario_id;
	uint8  ihdr_en;
	uint8  i2c_write_id;
	uint32 i2c_speed;
	camkit_i2c_addr_type addr_type;
	uint8  pdaf_mode;
};

typedef enum {
	CAMKIT_PDAF_SUPPORT_NA = 0,
	CAMKIT_PDAF_SUPPORT_RAW = 1,
	CAMKIT_PDAF_SUPPORT_CAMSV = 2,
	CAMKIT_PDAF_SUPPORT_CAMSV_LEGACY = 3,
	CAMKIT_PDAF_SUPPORT_RAW_DUALPD = 4,
	CAMKIT_PDAF_SUPPORT_CAMSV_DUALPD = 5,
	CAMKIT_PDAF_SUPPORT_RAW_LEGACY = 6,
} pdaf_support_type;

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct camkit_sensor_info_t {
	/* sensor chip id reg addr */
	uint16 sensor_id_reg;
	uint32 sensor_id;   // sensor chip id: 0x0582, 0xD855
	camkit_i2c_data_type sensor_id_dt;  // default: CAMKIT_I2C_WORD_DATA
	/* checksum value for Camera Auto Test */
	uint32 checksum_value;
	struct camkit_mode_info pre;
	struct camkit_mode_info cap;
	/* capture for PIP relative info */
	struct camkit_mode_info cap1;
	struct camkit_mode_info normal_video;
	struct camkit_mode_info hs_video;
	/* slim video for VT scenario relative info */
	struct camkit_mode_info slim_video;
	struct camkit_mode_info custom1;
	struct camkit_mode_info custom2;
	struct camkit_mode_info custom3;
	struct camkit_mode_info custom4;
	struct camkit_mode_info custom5;
	struct camkit_i2c_reg_setting id_init_setting;
	struct camkit_i2c_reg_setting init_setting;
	struct camkit_i2c_reg_setting init_burst_setting;  // block
	struct camkit_i2c_reg_setting pre_setting;
	struct camkit_i2c_reg_setting cap_setting;
	struct camkit_i2c_reg_setting cap1_setting;
	struct camkit_i2c_reg_setting normal_video_setting;
	struct camkit_i2c_reg_setting hs_video_setting;
	struct camkit_i2c_reg_setting slim_setting;
	struct camkit_i2c_reg_setting custom1_setting;
	struct camkit_i2c_reg_setting custom2_setting;
	struct camkit_i2c_reg_setting custom3_setting;
	struct camkit_i2c_reg_setting custom4_setting;
	struct camkit_i2c_reg_setting custom5_setting;
	struct camkit_i2c_reg_setting streamon_setting;
	struct camkit_i2c_reg_setting streamoff_setting;
	struct camkit_i2c_reg_setting test_pattern_on_setting;
	struct camkit_i2c_reg_setting test_pattern_off_setting;
	struct camkit_i2c_reg_table_array dump_info;
	struct camkit_i2c_reg_setting normal_to_long_ready_settings;
	struct camkit_i2c_reg_setting normal_to_long_end_settings;
	struct camkit_i2c_reg_table_array fuse_id_info;
	struct camkit_i2c_reg_table_array streamoff_check;
	struct camkit_seamless_cfg_t seamless_cfg;
	struct camkit_stagger_cfg_t stagger_cfg;
	/*
	 * ae_shut_delay_frame: shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	uint8  ae_shut_delay_frame;
	/*
	 * ae_sensor_gain_delay_frame:sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	uint8  ae_sensor_gain_delay_frame;
	/* isp gain delay frame for AE cycle */
	uint8  ae_ispGain_delay_frame;
	uint8  ihdr_support;        /* 1, support; 0,not support */
	uint8  ihdr_le_firstline;   /* 1,le first ; 0, se first */
	uint8  sensor_mode_num;     /* support sensor mode num */
	uint8  cap_delay_frame;
	uint8  pre_delay_frame;
	uint8  video_delay_frame;
	uint8  hs_video_delay_frame;
	uint8  slim_video_delay_frame;
	uint8  custom1_delay_frame;
	uint8  custom2_delay_frame;
	uint8  custom3_delay_frame;
	uint8  custom4_delay_frame;
	uint8  custom5_delay_frame;
	/* mclk driving current */
	uint8  isp_driving_current;
	/* sensor_interface_type */
	uint8  sensor_interface_type;
	/*
	 * mipi_sensor_type:
	 * 0,MIPI_OPHY_NCSI2
	 * 1,MIPI_OPHY_CSI2
	 * default is NCSI2
	 */
	uint8  mipi_sensor_type;
	/*
	 * mipi_settle_delay_mode:
	 * 0, high speed signal auto detect;
	 * 1, use settle delay,unit is ns
	 * default is auto detect
	 * don't modify this para
	 */
	uint8  mipi_settle_delay_mode;
	/* sensor output first pixel color */
	uint8  sensor_output_dataformat;
	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	uint8  mclk;
	uint8  mipi_lane_num;
	/*
	 * record sensor support write id addrs,
	 * only supprt 4 must end with 0xff
	 */
	uint8  i2c_addr_table[5];
	uint32 i2c_speed;
	camkit_i2c_addr_type addr_type;
	uint8  pdaf_support;
	uint16 binning_ratio[MAX_OUTPUT_INFO_SIZE];  // 4cell sensor need
	uint8  need_correction;
	uint8  pdaf_support_by_scenario;
	uint8  need_high_impedance;
	uint8  probe_flag;
	uint8  need_get_fuse_id;
	uint8  pdaf_info_by_scenario;
	uint8  check_streamoff_support;
	uint16 gain_step[MAX_OUTPUT_INFO_SIZE];  // stagger hdr need
};

typedef enum {
	CAMKIT_SENSOR_IDX_MIN_NUM = 0,
	CAMKIT_SENSOR_IDX_MAIN = CAMKIT_SENSOR_IDX_MIN_NUM,
	CAMKIT_SENSOR_IDX_SUB,
	CAMKIT_SENSOR_IDX_MAIN2,
	CAMKIT_SENSOR_IDX_SUB2,
	CAMKIT_SENSOR_IDX_MAIN3,
	CAMKIT_SENSOR_IDX_SUB3,
	CAMKIT_SENSOR_IDX_MAIN4,
	CAMKIT_SENSOR_IDX_SUB4,
	CAMKIT_SENSOR_IDX_MAIN5,
	CAMKIT_SENSOR_IDX_SUB5,
	CAMKIT_SENSOR_IDX_MAIN6,
	CAMKIT_SENSOR_IDX_SUB6,
	CAMKIT_SENSOR_IDX_MAX_NUM,
	CAMKIT_SENSOR_IDX_NONE,
} camkit_sensor_index;

typedef enum {
	CAMKIT_ISP_DRIVING_2MA = 0,
	CAMKIT_ISP_DRIVING_4MA,
	CAMKIT_ISP_DRIVING_6MA,
	CAMKIT_ISP_DRIVING_8MA,
	CAMKIT_ISP_DRIVING_MAX_NUM
} camkit_isp_driver_current;

typedef enum {
	CAMKIT_IMAGE_NORMAL = 0,
	CAMKIT_IMAGE_H_MIRROR,
	CAMKIT_IMAGE_V_MIRROR,
	CAMKIT_IMAGE_HV_MIRROR
} camkit_image_type;

typedef enum {
	CAMKIT_SENSOR_INTF_TYPE_PARALLEL = 0,
	CAMKIT_SENSOR_INTF_TYPE_MIPI,
	CAMKIT_SENSOR_INTF_TYPE_SERIAL,
	CAMKIT_SENSOR_INTF_TYPE_MAX
} camkit_sensor_intf_type;

typedef enum {
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_B = 0,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_Gb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_Gr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_UYVY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_VYUY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YUYV,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YVYU,
	CAMKIT_SENSOR_OUTPUT_FORMAT_CbYCrY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_CrYCbY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YCbYCr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YCrYCb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_Gb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_Gr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_Wb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_Wr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_MONO,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_MONO,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_IR,
} camkit_sensor_output_format;

typedef enum {
	CAMKIT_SENSOR_MIPI_1_LANE = 0,
	CAMKIT_SENSOR_MIPI_2_LANE,
	CAMKIT_SENSOR_MIPI_3_LANE,
	CAMKIT_SENSOR_MIPI_4_LANE
} camkit_sensor_mipi_lane;

typedef enum {
	CAMKIT_SCAM_1_DATA_CHANNEL = 0,
	CAMKIT_SCAM_2_DATA_CHANNEL,
	CAMKIT_SCAM_3_DATA_CHANNEL,
	CAMKIT_SCAM_4_DATA_CHANNEL,
} camkit_sensor_data_channel;

typedef enum {
	CAMKIT_MIPI_OPHY_NCSI2 = 0,
	CAMKIT_MIPI_OPHY_CSI2 = 1,
	CAMKIT_MIPI_CPHY = 2,
} camkit_mipi_type;

struct camkit_sensor_output_info_t {
	uint16 full_w;
	uint16 full_h;
	uint16 x0_offset;
	uint16 y0_offset;
	uint16 w0_size;
	uint16 h0_size;
	uint16 scale_w;
	uint16 scale_h;
	uint16 x1_offset;
	uint16 y1_offset;
	uint16 w1_size;
	uint16 h1_size;
	uint16 x2_tg_offset;
	uint16 y2_tg_offset;
	uint16 w2_tg_size;
	uint16 h2_tg_size;
};

struct camkit_sensor_pdaf_info_t {
	/* start offset of first PD block */
	uint32 i4OffsetX;
	uint32 i4OffsetY;
	/* PD block pitch */
	uint32 i4PitchX;
	uint32 i4PitchY;
	/* PD pair num in one block */
	uint32 i4PairNum;
	/* sub block width (one PD pair in one sub block) */
	uint32 i4SubBlkW;
	/* sub block height */
	uint32 i4SubBlkH;
	uint32 i4PosL[16][2];	/* left pd pixel position in one block*/
	uint32 i4PosR[16][2];	/* right pd pixel position in one block*/
	/* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR*/
	uint32 iMirrorFlip;
	uint32 i4BlockNumX;
	uint32 i4BlockNumY;
	/* 1: 1st line is long exposure, 0: 1st line is short exposure*/
	uint32 i4LeFirst;
	uint32 i4Crop[10][2]; /* [scenario][crop] -> (xcrop, ycrop) */
};

#define CAMKIT_POWER_INFO_MAX 20

enum camkit_hw_pin_type {
	CAMKIT_HW_PIN_NONE = 0,
	/* gpio pin begin */
	CAMKIT_HW_PIN_PDN,
	CAMKIT_HW_PIN_RST,
	CAMKIT_HW_PIN_AVDD_EN,
	CAMKIT_HW_PIN_AVDD_SEL,
	CAMKIT_HW_PIN_DVDD_EN,
	CAMKIT_HW_PIN_DVDD_SEL,
	CAMKIT_HW_PIN_IOVDD_EN,
	CAMKIT_HW_PIN_AVDD1_EN,
	CAMKIT_HW_PIN_AFVDD_EN,
	CAMKIT_HW_PIN_RST1,
	CAMKIT_HW_PIN_5V_BOOST,
	/* gpio pin end */

	/* regulator pin begin */
	CAMKIT_HW_PIN_AVDD,
	CAMKIT_HW_PIN_AVDD1,
	CAMKIT_HW_PIN_DVDD,
	CAMKIT_HW_PIN_DOVDD,
	CAMKIT_HW_PIN_VMCH,
	CAMKIT_HW_PIN_AFVDD,
	/* regulator pin end */

	/* pmic begin */
	CAMKIT_HW_PIN_LDO1_PMIC,
	CAMKIT_HW_PIN_LDO2_PMIC,
	CAMKIT_HW_PIN_LDO3_PMIC,
	CAMKIT_HW_PIN_LDO4_PMIC,
	CAMKIT_HW_PIN_XBUCK1_PMIC,
	/* pmic end */

	CAMKIT_HW_PIN_MIPI_SWITCH_EN,
	CAMKIT_HW_PIN_MIPI_SWITCH_SEL,

	CAMKIT_HW_PIN_MCLK,
	CAMKIT_HW_PIN_MAX_NUM,
	CAMKIT_HW_PIN_UNDEF = -1
};

enum camkit_hw_pin_value {
	CAMKIT_HW_PIN_VALUE_NONE = -1,
	CAMKIT_HW_PIN_VALUE_LOW = 0,
	CAMKIT_HW_PIN_VALUE_HIGH = 1,
	CAMKIT_HW_PIN_VALUE_1000,
	CAMKIT_HW_PIN_VALUE_1050,
	CAMKIT_HW_PIN_VALUE_1100,
	CAMKIT_HW_PIN_VALUE_1200,
	CAMKIT_HW_PIN_VALUE_1210,
	CAMKIT_HW_PIN_VALUE_1220,
	CAMKIT_HW_PIN_VALUE_1250,
	CAMKIT_HW_PIN_VALUE_1500,
	CAMKIT_HW_PIN_VALUE_1800,
	CAMKIT_HW_PIN_VALUE_2500,
	CAMKIT_HW_PIN_VALUE_2800,
	CAMKIT_HW_PIN_VALUE_2850,
	CAMKIT_HW_PIN_VALUE_2900,
	CAMKIT_HW_PIN_VALUE_3000,
	CAMKIT_HW_PIN_VALUE_3300,
	CAMKIT_HW_PIN_VALUE_5000,
};

struct camkit_hw_power_info_t {
	int32 pin_type;
	int32 pin_val;
	uint32 pin_delay;
};

enum camkit_again_type {
	CAMKIT_AGAIN_STD = 0,   // the sensor accord with SMIA standard, include gc8054
	CAMKIT_AGAIN_GC,        // the sensor from galaxycore, such as gc2375, gc8034
	CAMKIT_AGAIN_OV13855,   // the analog gain is not continuous, such as ov13855,ov16b10 and so on.
	CAMKIT_AGAIN_MAX,
};

/*
 * Constants used in standard smia gain formula
 * Analog Gain = (m0 * X + c0) / (m1 * X + c1)
 * X is register gain
 */
struct smia_again_coeff {
	int32 m0;
	int32 m1;
	int32 m2;
	int32 c0;
	int32 c1;
	int32 c2;
};

enum aec_op_type {
	SENSOR_AEC_OP_GROUPON,
	SENSOR_AEC_OP_GROUPOFF,
	SENSOR_AEC_OP_VTS,
	SENSOR_AEC_OP_LC,          // linecount
	SENSOR_AEC_OP_SHIFT,     // for sony long exposure
	SENSOR_AEC_OP_AGAIN,
	SENSOR_AEC_OP_DGAIN,
	SENSOR_AEC_OP_CTRL,      // other control registers, such as: switch page mode about hi259
	SENSOR_AEC_OP_AGAIN_TABLE,  // only for gc sensor, such as: gc2375, gc8034
	SENSOR_AEC_OP_PRESHUTTER, //for sony dual camera preshutter, setup only befor stream on
	SENSOR_AEC_OP_LONG_EXPO_PSV_MODE,  // for ov long expo mode,like HWI OV16B10
	SENSOR_AEC_OP_LONG_EXPO_MODE_CONFIG,  // for ov long expo mode,like HWI OV16B10
	SENSOR_AEC_OP_LONG_EXPO_LIMIT,
	SENSOR_AEC_OP_LONG_LC,
	SENSOR_AEC_OP_AWB_GAIN_GR,
	SENSOR_AEC_OP_AWB_GAIN_R,
	SENSOR_AEC_OP_AWB_GAIN_B,
	SENSOR_AEC_OP_AWB_GAIN_GB,
	SENSOR_AEC_OP_AWB_GAIN_GRGB,
	SENSOR_AEC_OP_MAX,
};

enum camkit_i2c_mode {
	SENSOR_WRITE = 0,
	SENSOR_READ,
	SENSOR_READ_MASK,
	SENSOR_READ_OR_MASK_WRITE,
	SENSOR_READ_AND_MASK_WRITE,
	SENSOR_POLL,
	SENSOR_I2C_MODE_MAX,
};

struct camkit_i2c_cfg {
	uint16 addr;
	uint16 val;
	uint16 len;
	uint16 mask;
	enum camkit_i2c_mode mode;
	uint16 addr_type;
	uint16 data_type;
	uint16 delay;
};

struct camkit_aec_cfg {
	uint16 addr;
	uint16 max_val;
	int16  shift;     // >= 0, left shift; < 0, right shift
	uint16 mask;
	uint16 addr_type;
	uint16 data_type;
};

struct camkit_aec_op {
	enum aec_op_type op_type;
	union {
		struct camkit_i2c_cfg i2c_setting[MAX_AEC_REGS];
		struct camkit_aec_cfg aec_setting[MAX_AEC_REGS];
	};
	int32 size;
};

struct aec_ops_map {
	struct camkit_aec_op* aec_ops;
	int32 size;
};

struct priv_again_map {
	uint32 again_val; // again = real again * 64 = platform gain
	struct camkit_i2c_reg setting[MAX_AGAIN_REGS];
	uint32 size;
	uint16 addr_type;
	uint16 data_type;
};

struct private_again_info {
	struct priv_again_map *again_map;
	int32 size;
};

enum camkit_long_expo_type {
	SENSOR_LONG_EXPO_NONE = 0,  // long exposure unsupportted
	SENSOR_LONG_EXPO_SONY,      // sony long exposure: cit_lshift
	SENSOR_LONG_EXPO_OV,        // OV long exposure
	SENSOR_LONG_EXPO_SAMSUNG,   // SumSung long exposure
	SENSOR_LONG_EXPO_SETTINGS,  // implement long exposure by settings
	SENSOR_LONG_EXPO_MAX,
};

enum camkit_awb_gain_type {
	CAMKIT_AWB_GAIN_NONE = 0,
	CAMKIT_AWB_GAIN_SONY,
	CAMKIT_AWB_GAIN_SAMSUNG,
};

struct camkit_aec_info_t {
	uint16 min_again;
	uint16 max_again;
	uint16 min_dgain;
	uint16 max_dgain;
	uint16 min_gain;
	uint16 max_gain;
	uint16 min_iso;
	uint16 gain_type;
	uint16 gain_step;
	uint16 reg_gain_1x;
	uint16 dgain_decimator;
	uint16 max_shift;    // max value of sony cit_lshift
	enum camkit_long_expo_type lexpo_type;
	uint32 max_linecount;
	uint32 min_linecount;
	uint32 vts_offset;
	uint32 max_frame_length;
	uint16 vblank_flag;  // the flag that fps control by vblank
	uint32 base_fl;      // vblank = frame length - base_fl
	uint16 min_vblank;
	uint16 max_vblank;
	enum camkit_again_type again_type;
	struct smia_again_coeff smia_coeff;
	struct private_again_info priv_again_info;
	struct aec_ops_map gain_ops_map;
	struct aec_ops_map expo_ops_map;
	struct aec_ops_map vts_ops_map;
	struct aec_ops_map normal2long_ops_map;
	struct aec_ops_map long2normal_ops_map;
	struct aec_ops_map awb_gain_ops_map;
	enum camkit_awb_gain_type awb_gain_type;
	struct aec_ops_map hdr_m_gain_ops_map;
	struct aec_ops_map hdr_m_expo_ops_map;
};

/* virtual channel configuration */
struct camkit_sensor_vc_info_t {
	uint16 vc_num;
	uint16 vc_pixel_num;
	uint16 mode_select;   /* 0: auto mode, 1:direct mode  */
	uint16 expo_ratio;    /* 1/1, 1/2, 1/4, 1/8 */
	uint16 od_value;      /* OD Value */

	/* STATS divistion mdoe 0: 16x16, 1:8x8, 2:4x4, 3:1x1*/
	uint16 rg_stats_mode;
	uint16 vc0_id;
	uint16 vc0_data_type;
	uint16 vc0_sizeh;
	uint16 vc0_sizev;
	uint16 vc1_id;
	uint16 vc1_data_type;
	uint16 vc1_sizeh;
	uint16 vc1_sizev;
	uint16 vc2_id;
	uint16 vc2_data_type;
	uint16 vc2_sizeh;
	uint16 vc2_sizev;
	uint16 vc3_id;
	uint16 vc3_data_type;
	uint16 vc3_sizeh;
	uint16 vc3_sizev;
	uint16 vc4_id;
	uint16 vc4_data_type;
	uint16 vc4_sizeh;
	uint16 vc4_sizev;
	uint16 vc5_id;
	uint16 vc5_data_type;
	uint16 vc5_sizeh;
	uint16 vc5_sizev;
};

struct camkit_sensor_vc_info2 {
	uint16 vc_feature;
	uint16 vc_id;
	uint16 vc_datatype;
	uint16 vc_sizeh_pixel;
	uint16 vc_sizev;
	uint16 vc_size_byte;
};

struct camkit_sensor_vc_info2_t {
	uint16 vc_num;
	uint16 vc_pixel_num;
	uint16 mode_select;
	uint16 expo_ratio;
	uint16 od_value;
	uint16 rg_stats_mode;
	struct camkit_sensor_vc_info2 vc_info[MAX_VC_NUM];
	uint16 updated;
};

enum camkit_spc_type {
	PDAF_SPC_NONE = 0,
	PDAF_SPC_LRC, /* sony pdaf correction: LSC and RSC */
	PDAF_SPC_PDC, /* OV pdaf correction, pdc discontinuous burn, need extract */
	PDAF_SPC_PDC_EXT, /* OV pdaf correction, pdc continuous burn */
	PDAF_SPC_MAX,
};

enum pdc_config_type {
	BEFORE_PDC_CONFIG,
	AFTER_PDC_CONFIG,
};

struct camkit_pdc_config {
	uint8 need_config;
	struct camkit_i2c_reg_setting before_pdc_setting;
	struct camkit_i2c_reg_setting after_pdc_setting;
};

/* sensor correction configuration */
struct camkit_sensor_correction_t {
	uint16 qsc_apply;         // quad raw coding sensitivity correction
	uint16 qsc_read_flag;     // use in kernel, can't configure
	uint16 eeprom_qsc_addr;   // the start address of qsc calibration in eeprom
	uint16 sensor_qsc_addr;   // the start address of qsc in sensor
	uint8 *qsc_buf;
	uint16 qsc_len;

	uint16 spc_apply;         // pdaf shield point correction
	uint16 spc_read_flag;     // use in kernel, can't configure
	uint16 eeprom_pdaf_addr;  // the start address of pdaf calibration in eeprom
	uint8 *pdaf_buf;
	uint16 pdaf_len;

	uint16 spc_type;          // SONY, OV and so on
	uint16 lsc_start_addr;    // sony lsc start address
	uint16 lsc_addr_len;
	uint16 rsc_start_addr;    // sony rsc start address
	uint16 rsc_addr_len;
	uint16 pdc_addr;          // ov pdc start addr
	uint16 pdc_len;
	struct camkit_pdc_config pdc_config;
	uint16 block_read_support;
	uint16 eeprom_i2c_speed;
};

/* high impedance config for each sensor */
struct camkit_high_impedance_sensor_t {
	uint32 sensor_index;
	char sensor_name[SENSOR_NAME_LEN];
	int32 i2c_index;
	uint32 i2c_speed;
	uint8  i2c_addr_table[5];
	struct camkit_hw_power_info_t power_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_hw_power_info_t power_down_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_i2c_reg_setting high_impedance_setting;
};

/* total high impedance info */
struct camkit_high_impedance_info_t {
	/*
	 * when matching the high impedance sensor,
	 * set the matched flag and matched index.
	 */
	uint8 matched_index;
	uint8 matched_flag;
	/* number of sensors which need to set high impedance */
	uint8 total_num;
	struct camkit_high_impedance_sensor_t sensor[MAX_HIGH_IMPEDANCE_SENSOR_NUM];
};

struct camkit_hw_power_info_pair_t {
	struct camkit_hw_power_info_t power_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_hw_power_info_t power_down_info[CAMKIT_POWER_INFO_MAX];
};

struct camkit_extend_hw_power_info_t {
	uint8 total_num;
	struct camkit_hw_power_info_pair_t info_pair[MAX_EXT_POWER_INFO_NUM];
};

struct camkit_sensor_params {
	uint8  sensor_name[SENSOR_NAME_LEN];  // sensor name: imx582, ov13855
	struct camkit_sensor_info_t sensor_info;
	struct camkit_sensor_ctrl_t sensor_ctrl;
	struct camkit_sensor_output_info_t output_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_pdaf_info_t pdaf_info;
	struct camkit_sensor_pdaf_info_t pdaf_info_scenario[MAX_OUTPUT_INFO_SIZE];
	struct camkit_hw_power_info_t power_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_hw_power_info_t power_down_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_extend_hw_power_info_t ext_power_info;
	struct camkit_aec_info_t aec_info;
	struct camkit_sensor_vc_info_t vc_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_correction_t correction_info;
	struct camkit_high_impedance_info_t high_impedance_info;
	struct camkit_sensor_vc_info2_t vc_info2[MAX_OUTPUT_INFO_SIZE];
};

enum eeprom_config_type {
	EEPROM_ATTACH_CONFIG,
	EEPROM_DETACH_CONFIG,
};

struct camkit_config_eeprom {
	uint8 need_config;
	int32 i2c_index;
	uint32 i2c_speed;
	uint8 i2c_addr;
	struct camkit_i2c_reg_setting attach_setting;
	struct camkit_i2c_reg_setting detach_setting;
};

struct lcd_name_params {
	uint8 need_lcd_name;
	char lcd_name[LCD_NAME_LEN];
};

struct camkit_module_params {
	/* set 1 while skip to match module vendor */
	uint8  skip_module_id;
	uint8  allow_no_eeprom;
	uint16 eeprom_i2c_addr;
	uint16 module_code_addr;
	/* module vendor, such as: 01-sunny, 02-foxconn, 03-luxvisions, 06-ofilm */
	uint16 module_code;
	uint16 lens_type_addr;
	uint16 lens_type;
	camkit_i2c_addr_type addr_type;
	camkit_i2c_data_type data_type;

	struct lcd_name_params lcd_params;
	/* defined in kd_camkit_define.h, such as: 0x058202 */
	uint32 match_id;
	/* defined in kd_camkit_define.h, such as: imx582_foxconn */
	char sensor_name[SENSOR_NAME_LEN];
	char vcm_name[MAX_DRV_NUM][MOTOR_NAME_LEN];
	char ois_name[MAX_DRV_NUM][OIS_NAME_LEN];
	uint8 skip_open_match_id;
	struct camkit_config_eeprom config_eeprom;
};

struct camkit_sensor_index_info_t {
	uint32 sensor_index;
	char sensor_name[SENSOR_NAME_LEN];
};

struct camkit_sensor_power_settings_idx_map_t {
	uint8 is_map_vaild;
	uint32 power_setting_idx;
};

struct camkit_params {
	struct camkit_sensor_params *sensor_params;
	struct camkit_module_params *module_params;
};

struct camkit_probe_sensor_params {
	uint32 sensor_idx;
	uint32 probe_succeed;  // out parameters
	struct camkit_params *kit_params;
	uint8 fuse_id[MAX_FUSE_ID_SIZE];
};

struct vcm_motor_name {
	char motor_name[MOTOR_NAME_LEN];
};

struct ois_ic_name {
	char ois_name[OIS_NAME_LEN];
};

#define KDIMGSENSORIOC_X_PROBESENSOR \
	_IOWR(IMGSENSORMAGIC, 0x1000, struct camkit_probe_sensor_params)

#define KDIMGSENSORIOC_X_SETVCMNAME \
	_IOW(IMGVCMMAGIC, 10, struct vcm_motor_name)

#define KDIMGSENSORIOC_X_SETOISNAME \
	_IOW(IMGVCMMAGIC, 19, struct ois_ic_name)

#define KDIMGSENSORIOC_X_SETOISDOWNLOAD \
	_IOW(IMGVCMMAGIC, 20, struct ois_ic_name)

#endif // _CAMKIT_DEFINE_H
