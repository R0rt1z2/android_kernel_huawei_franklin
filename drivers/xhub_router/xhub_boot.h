/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub boot module
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
 
#ifndef __LINUX_XHUB_CMU_H__
#define __LINUX_XHUB_CMU_H__
#include <linux/types.h>


#define IOMCU_CONFIG_SIZE   DDR_CONFIG_SIZE
#define IOMCU_CONFIG_START  DDR_CONFIG_ADDR_AP

#define SENSOR_MAX_RESET_TIME_MS 400
#define SENSOR_DETECT_AFTER_POWERON_TIME_MS 50
#define SENSOR_POWER_DO_RESET 0
#define SENSOR_POWER_NO_RESET 1
#define SENSOR_REBOOT_REASON_MAX_LEN 32

#define WARN_LEVEL 2
#define INFO_LEVEL 3

#define LG_TPLCD 1
#define JDI_TPLCD 2
#define KNIGHT_BIEL_TPLCD 3
#define KNIGHT_LENS_TPLCD 4
#define BOE_TPLCD 5
#define EBBG_TPLCD 6
#define INX_TPLCD 7
#define SAMSUNG_TPLCD 8
#define SHARP_TPLCD 9
#define BIEL_TPLCD 10
#define VITAL_TPLCD 11
#define TM_TPLCD 12
#define AUO_TPLCD 13
#define TCL_TPLCD 14
#define CTC_TPLCD 15
#define CSOT_TPLCD 16
#define VISI_TPLCD 17
#define BOE_TPLCD2 18

#define DTS_COMP_LG_ER69006A "hisilicon,mipi_lg_eR69006A"
#define DTS_COMP_JDI_NT35695_CUT3_1 "hisilicon,mipi_jdi_NT35695_cut3_1"
#define DTS_COMP_JDI_NT35695_CUT2_5 "hisilicon,mipi_jdi_NT35695_cut2_5"

#define DTS_COMP_LG_ER69007  "hisilicon,mipi_lg_eR69007"
#define DTS_COMP_SHARP_NT35597  "hisilicon,mipi_sharp_knt_NT35597"
#define DTS_COMP_LG_ER69006_FHD      "hisilicon,mipi_lg_eR69006_FHD"
#define DTS_COMP_SHARP_NT35695  "hisilicon,mipi_sharp_NT35695_5p7"
#define DTS_COMP_MIPI_BOE_ER69006  "hisilicon,mipi_boe_ER69006_5P7"

#define DTS_COMP_BOE_OTM1906C  "hisilicon,boe_otm1906c_5p2_1080p_cmd"
#define DTS_COMP_EBBG_OTM1906C  "hisilicon,ebbg_otm1906c_5p2_1080p_cmd"
#define DTS_COMP_INX_OTM1906C  "hisilicon,inx_otm1906c_5p2_1080p_cmd"
#define DTS_COMP_JDI_NT35695  "hisilicon,jdi_nt35695_5p2_1080p_cmd"
#define DTS_COMP_LG_R69006  "hisilicon,lg_r69006_5p2_1080p_cmd"
#define DTS_COMP_SAMSUNG_S6E3HA3X02 "hisilicon,mipi_samsung_S6E3HA3X02"

#define DTS_COMP_LG_R69006_5P2  "hisilicon,mipi_lg_R69006_5P2"
#define DTS_COMP_SHARP_NT35695_5P2  "hisilicon,mipi_sharp_NT35695_5P2"
#define DTS_COMP_JDI_R63452_5P2  "hisilicon,mipi_jdi_R63452_5P2"

#define DTS_COMP_SAM_WQ_5P5  "hisilicon,mipi_samsung_S6E3HA3X02_5P5_AMOLED"
#define DTS_COMP_SAM_FHD_5P5  "hisilicon,mipi_samsung_D53G6EA8064T_5P5_AMOLED"

#define DTS_COMP_JDI_R63450_5P7 "hisilicon,mipi_jdi_duke_R63450_5P7"
#define DTS_COMP_SHARP_DUKE_NT35597 "hisilicon,mipi_sharp_duke_NT35597"

#define DTS_COMP_AUO_OTM1901A_5P2 "auo_otm1901a_5p2_1080p_video"
#define DTS_COMP_AUO_TD4310_5P2 "auo_td4310_5p2_1080p_video"

#define DTS_COMP_AUO_NT36682A_6P72 "auo_nt36682a_6p57"
#define DTS_COMP_AUO_OTM1901A_5P2_1080P_VIDEO_DEFAULT "auo_otm1901a_5p2_1080p_video_default"
#define DTS_COMP_BOE_NT36682A_6P57 "boe_nt36682a_6p57"
#define DTS_COMP_BOE_TD4320_6P57 "boe_td4320_6p57"
#define DTS_COMP_TCL_NT36682A_6P57 "tcl_nt36682a_6p57"
#define DTS_COMP_TCL_TD4320_6P57 "tcl_td4320_6p57"
#define DTS_COMP_TM_NT36682A_6P57 "tm_nt36682a_6p57"
#define DTS_COMP_TM_TD4320_6P57 "tm_td4320_6p57"

#define DTS_COMP_TM_FT8716_5P2 "tm_ft8716_5p2_1080p_video"
#define DTS_COMP_EBBG_NT35596S_5P2 "ebbg_nt35596s_5p2_1080p_video"
#define DTS_COMP_JDI_ILI7807E_5P2 "jdi_ili7807e_5p2_1080p_video"
#define DTS_COMP_BOE_NT37700F "boe_nt37700f_vogue_6p47_1080plus_cmd"
#define DTS_COMP_BOE_NT36672_6P26 "boe_nt36672a_6p26"
#define DTS_COMP_LG_NT37280 "lg_nt37280_vogue_6p47_1080plus_cmd"
#define DTS_COMP_BOE_NT37700F_EXT "boe_nt37700f_vogue_p_6p47_1080plus_cmd"
#define DTS_COMP_LG_NT37280_EXT "lg_nt37280_vogue_p_6p47_1080plus_cmd"
#define DTS_COMP_LG_TD4320_6P26 "lg_td4320_6p26"
#define DTS_COMP_SAMSUNG_EA8074 "samsung_ea8074_elle_6p10_1080plus_cmd"
#define DTS_COMP_SAMSUNG_EA8076 "samsung_ea8076_elle_6p11_1080plus_cmd"
#define DTS_COMP_SAMSUNG_EA8076_V2 "samsung_ea8076_elle_v2_6p11_1080plus_cmd"
#define DTS_COMP_BOE_NT37700F_TAH "boe-nt37800f-tah-8p03-3lane-2mux-cmd"
#define DTS_COMP_BOE_NT37800ECO_TAH "boe-nt37800eco-tah-8p03-3lane-2mux-cmd"
#define DTS_COMP_HLK_AUO_OTM1901A "hlk_auo_otm1901a_5p2_1080p_video_default"
#define DTS_COMP_BOE_NT36682A "boe_nt36682a_6p59_1080p_video"
#define DTS_COMP_BOE_TD4320 "boe_td4320_6p59_1080p_video"
#define DTS_COMP_INX_NT36682A "inx_nt36682a_6p59_1080p_video"
#define DTS_COMP_TCL_NT36682A "tcl_nt36682a_6p59_1080p_video"
#define DTS_COMP_TM_NT36682A "tm_nt36682a_6p59_1080p_video"
#define DTS_COMP_TM_TD4320 "tm_td4320_6p59_1080p_video"
#define DTS_COMP_TM_TD4320_6P26 "tm_td4320_6p26"
#define DTS_COMP_TM_TD4330_6P26 "tm_td4330_6p26"
#define DTS_COMP_TM_NT36672A_6P26 "tm_nt36672a_6p26"

#define DTS_COMP_CTC_FT8719_6P26 "ctc_ft8719_6p26"
#define DTS_COMP_CTC_NT36672A_6P26 "ctc_nt36672a_6p26"
#define DTS_COMP_BOE_TD4321_6P26 "boe_td4321_6p26_1080p_video"

#define DTS_COMP_CSOT_NT36682A_6P5 "csot_nt36682a_6p5"
#define DTS_COMP_BOE_FT8719_6P5 "boe_ft8719_6p5"
#define DTS_COMP_TM_NT36682A_6P5 "tm_nt36682a_6p5"
#define DTS_COMP_BOE_TD4320_6P5 "boe_td4320_6p5"

#define DTS_COMP_BOE_EW813P_6P57 "boe_ew813p_6p57"
#define DTS_COMP_BOE_NT37700P_6P57 "boe_nt37700p_6p57"
#define DTS_COMP_VISI_NT37700C_6P57_ONCELL "visi_nt37700c_6p57_oncell"
#define DTS_COMP_VISI_NT37700C_6P57 "visi_nt37700c_6p57"

#define DTS_COMP_TCL_NT36682C_6P63 "tcl_nt36682c_6p63_1080p_video"
#define DTS_COMP_TM_NT36682C_6P63 "tm_nt36682c_6p63_1080p_video"
#define DTS_COMP_BOE_NT36682C_6P63 "boe_nt36682c_6p63_1080p_video"
#define DTS_COMP_INX_NT36682C_6P63 "inx_nt36682c_6p63_1080p_video"
#define DTS_COMP_BOE_FT8720_6P63 "boe_ft8720_6p63_1080p_video"

#define DTS_COMP_TM_TD4321_6P59 "tm_td4321_6p59"
#define DTS_COMP_TCL_NT36682A_6P59 "tcl_nt36682a_6p59"
#define DTS_COMP_BOE_NT36682A_6P59 "boe_nt36682a_6p59"
#define DTS_COMP_BOE_B "190_207_6p72"
#define DTS_COMP_VISI_V "310_207_6p72"

#define DTS_COMP_BOE "190"
#define DTS_COMP_VISI "310"

#define DTS_COMP_BOE_NAME "boe"
#define DTS_COMP_SAMSUNG_NAME "sm"

#define SC_EXISTENCE   1
#define SC_INEXISTENCE 0

enum SENSOR_POWER_CHECK {
	SENSOR_POWER_STATE_OK = 0,
	SENSOR_POWER_STATE_INIT_NOT_READY,
	SENSOR_POWER_STATE_CHECK_ACTION_FAILED,
	SENSOR_POWER_STATE_CHECK_RESULT_FAILED,
	SENSOR_POWER_STATE_NOT_PMIC,
};

typedef struct {
	u32 mutex;
	u16 index;
	u16 pingpang;
	u32 buff;
	u32 ddr_log_buff_cnt;
	u32 ddr_log_buff_index;
	u32 ddr_log_buff_last_update_index;
} log_buff_t;

typedef enum DUMP_LOC {
	DL_NONE = 0,
	DL_TCM,
	DL_EXT,
	DL_BOTTOM = DL_EXT,
} dump_loc_t;

enum DUMP_REGION {
	DE_TCM_CODE,
	DE_DDR_CODE,
	DE_DDR_DATA,
	DE_BOTTOM = 16,
};

typedef struct dump_region_config {
	u8 loc;
	u8 reserved[3];
} dump_region_config_t;

typedef struct dump_config {
	u64 dump_addr;
	u32 dump_size;
	u32 reserved1;
	u64 ext_dump_addr;
	u32 ext_dump_size;
	u8 enable;
	u8 finish;
	u8 reason;
	u8 reserved2;
	dump_region_config_t elements[DE_BOTTOM];
} dump_config_t;

typedef struct {
	const char *dts_comp_mipi;
	uint8_t tplcd;
} lcd_module;

typedef struct {
	const char *dts_comp_lcd_model;
	uint8_t tplcd;
} lcd_model;

struct bright_data {
	uint32_t mipi_data;
	uint32_t bright_data;
	uint64_t time_stamp;
};

struct read_data_als_ud {
	float rdata;
	float gdata;
	float bdata;
	float irdata;
};

struct als_ud_config_t {
	u8 screen_status;
	u8 reserved[7]; // 7 is reserved nums
	u64 als_rgb_pa;
	struct bright_data bright_data_input;
	struct read_data_als_ud read_data_history;
};

struct config_on_ddr { // 200bytes, max size should less than 4KB
	u32 magic;
	dump_config_t dump_config;
	log_buff_t log_buff_cb_backup;
	u32 log_level;
	u64 reserved;
	struct als_ud_config_t als_ud_config;
	u8 phone_type_info[2]; // 2 is phone type info nums
	u8 rsv[6];
	u32 screen_status;
};

extern int (*api_xhub_mcu_recv) (const char *buf, unsigned int length);
extern int (*api_mculog_process) (const char *buf, unsigned int length);
int get_sensor_mcu_mode(void);
void sync_time_to_xhub(void);
extern int init_sensors_cfg_data_from_dts(void);
extern int send_status_req_to_mcu(void);
#ifdef CONFIG_HUAWEI_DSM
struct dsm_client *xhub_get_shb_dclient(void);
#endif
void init_write_nv_work(void);
void close_nv_workqueue(void);
#endif /* __LINUX_XHUB_CMU_H__ */
