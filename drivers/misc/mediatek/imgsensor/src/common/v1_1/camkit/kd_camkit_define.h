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

#ifndef _KD_CAMKIT_DEFINE_H
#define _KD_CAMKIT_DEFINE_H

#include "camkit/kd_camkit_define.h"

#ifdef CONFIG_COMPAT

#include <linux/compat.h>

struct camkit_i2c_reg_table_array32 {
	compat_uptr_t setting;
	uint16 size;
};

struct camkit_i2c_reg_setting32 {
	compat_uptr_t setting;
	uint16 size;
	uint16 delay;
    camkit_i2c_addr_type addr_type;
	camkit_i2c_data_type data_type;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct camkit_sensor_info_32 {
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
	struct camkit_i2c_reg_setting32 id_init_setting;
	struct camkit_i2c_reg_setting32 init_setting;
	struct camkit_i2c_reg_setting32 pre_setting;
	struct camkit_i2c_reg_setting32 cap_setting;
	struct camkit_i2c_reg_setting32 cap1_setting;
	struct camkit_i2c_reg_setting32 normal_video_setting;
	struct camkit_i2c_reg_setting32 hs_video_setting;
	struct camkit_i2c_reg_setting32 slim_setting;
	struct camkit_i2c_reg_setting32 custom1_setting;
	struct camkit_i2c_reg_setting32 custom2_setting;
	struct camkit_i2c_reg_setting32 custom3_setting;
	struct camkit_i2c_reg_setting32 custom4_setting;
	struct camkit_i2c_reg_setting32 custom5_setting;
	struct camkit_i2c_reg_setting32 streamon_setting;
	struct camkit_i2c_reg_setting32 streamoff_setting;
	struct camkit_i2c_reg_setting32 test_pattern_on_setting;
	struct camkit_i2c_reg_setting32 test_pattern_off_setting;
	struct camkit_i2c_reg_table_array32 dump_info;
	struct camkit_i2c_reg_setting32 normal_to_long_ready_settings;
	struct camkit_i2c_reg_setting32 normal_to_long_end_settings;
	struct camkit_i2c_reg_table_array32 fuse_id_info;
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
	uint8 pdaf_support;
	uint16 binning_ratio[MAX_OUTPUT_INFO_SIZE];  // 4cell sensor need
	uint8  need_correction;
	uint8  pdaf_support_by_scenario;
	uint8  need_get_fuse_id;
	uint8  pdaf_info_by_scenario;
};

struct aec_ops_map_32 {
	compat_uptr_t aec_ops;
	int32 size;
};

struct private_again_info_32 {
	compat_uptr_t again_map;
	int32 size;
};

struct camkit_aec_info_32 {
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
	uint32 max_linecount;
	uint32 min_linecount;
	uint32 vts_offset;
	uint32 max_frame_length;
	enum camkit_again_type again_type;
	struct smia_again_coeff smia_coeff;
	struct private_again_info_32 priv_again_info;
	struct aec_ops_map_32 gain_ops_map;
	struct aec_ops_map_32 expo_ops_map;
	struct aec_ops_map_32 vts_ops_map;
	struct aec_ops_map_32 normal2long_ops_map;
	struct aec_ops_map_32 long2normal_ops_map;
};

struct camkit_sensor_params32 {
	uint8  sensor_name[SENSOR_NAME_LEN];
	struct camkit_sensor_info_32 sensor_info;
	struct camkit_sensor_ctrl_t sensor_ctrl;
	struct camkit_sensor_output_info_t output_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_pdaf_info_t pdaf_info;
	struct camkit_sensor_pdaf_info_t pdaf_info_scenario[MAX_OUTPUT_INFO_SIZE];
	struct camkit_hw_power_info_t power_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_hw_power_info_t power_down_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_aec_info_32 aec_info;
	struct camkit_sensor_vc_info_t vc_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_correction_t correction_info;
};

struct camkit_params32 {
	struct camkit_sensor_params32 *sensor_params;
	struct camkit_module_params *module_params;
};

struct camkit_probe_sensor_params32 {
	uint32 sensor_idx;
	uint32 probe_succeed;  // out parameters
	struct camkit_params32 *kit_params;
};

#define COMPAT_KDIMGSENSORIOC_X_PROBESENSOR \
	_IOWR(IMGSENSORMAGIC, 0x1000, struct camkit_probe_sensor_params32)

#endif

#endif // _KD_CAMKIT_DEFINE_H
