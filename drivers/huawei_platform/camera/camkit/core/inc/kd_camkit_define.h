/*
 * ke_camkit_define.h
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

#ifndef KD_CAMKIT_DEFINE_H
#define KD_CAMKIT_DEFINE_H

#include "kd_camkit_types.h"

struct camkit_i2c_reg {
	uint16 addr;
	uint16 data;
	uint16 delay;
};

struct camkit_i2c_reg_table {
	uint16 addr;
	uint16 data;
	enum camkit_i2c_data_type data_type;
	enum camkit_i2c_operation i2c_operation;
	uint16 delay;
};

struct camkit_i2c_reg_table_array {
	struct camkit_i2c_reg_table *setting;
	uint16 size;
};

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
};

struct camkit_i2c_reg_setting {
	struct camkit_i2c_reg *setting;
	uint16 size;
	uint16 delay;
	enum camkit_i2c_addr_type addr_type;
	enum camkit_i2c_data_type data_type;
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
	enum camkit_scenario_type current_scenario_id;
	uint8  ihdr_en;
	uint8  i2c_write_id;
	uint32 i2c_speed;
	enum camkit_i2c_addr_type addr_type;
	uint8  pdaf_mode;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct camkit_sensor_info_t {
	/* sensor chip id reg addr */
	uint16 sensor_id_reg;
	uint32 sensor_id;   // sensor chip id: 0x0582, 0xD855
	enum camkit_i2c_data_type sensor_id_dt;  // default: CAMKIT_I2C_WORD_DATA
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
    /* register addr must be continuous in burst mode */
	struct camkit_i2c_reg_setting init_burst_setting;
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
	uint8  ae_isp_gain_delay_frame;
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
	enum camkit_i2c_addr_type addr_type;
	uint8  pdaf_support;
	uint16 binning_ratio[MAX_OUTPUT_INFO_SIZE];  // 4cell sensor need
	uint8  need_correction;
	uint8  pdaf_support_by_scenario;
	uint8  need_high_impedance;
	uint8  probe_flag;
	uint8  need_get_fuse_id;
};

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
	uint32 offset_x;
	uint32 offset_y;
	/* PD block pitch */
	uint32 pitch_x;
	uint32 pitch_y;
	/* PD pair num in one block */
	uint32 pair_num;
	/* sub block width (one PD pair in one sub block) */
	uint32 sub_blk_w;
	/* sub block height */
	uint32 sub_blk_h;
	/* left pd pixel position in one block */
	uint32 pos_l[16][2];
	/* right pd pixel position in one block */
	uint32 pos_r[16][2];
	/* 0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR */
	uint32 mirror_flip;
	uint32 block_num_x;
	uint32 block_num_y;
	/* 1: 1st line is long exposure, 0: 1st line is short exposure */
	uint32 lexpo_first;
	uint32 crop[10][2]; /* [scenario][crop] -> (xcrop, ycrop) */
};

struct camkit_hw_power_info_t {
	int32 pin_type;
	int32 pin_val;
	uint32 pin_delay;
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
	struct camkit_aec_op *aec_ops;
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
};

/* virtual channel configuration */
struct camkit_sensor_vc_info_t {
	uint16 vc_num;
	uint16 vc_pixel_num;
	uint16 mode_select;   /* 0: auto mode, 1:direct mode  */
	uint16 expo_ratio;    /* 1/1, 1/2, 1/4, 1/8 */
	uint16 od_value;      /* OD Value */

	/* STATS divistion mdoe 0: 16x16, 1:8x8, 2:4x4, 3:1x1 */
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

/* sensor correction configuration */
struct camkit_sensor_correction_t {
	uint16 qsc_apply;        // quad raw coding sensitivity correction
	uint16 qsc_read_flag;    // use in kernel, can't configure
	uint16 eeprom_qsc_addr;  // the start address of qsc calibration in eeprom
	uint16 sensor_qsc_addr;  // the start address of qsc in sensor
	uint8 *qsc_buf;
	uint16 qsc_len;

	uint16 spc_apply;        // pdaf shield point correction
	uint16 spc_read_flag;    // use in kernel, can't configure
	uint16 eeprom_pdaf_addr; // the start address of pdaf calibration in eeprom
	uint8 *pdaf_buf;
	uint16 pdaf_len;

	uint16 spc_type;         // SONY, OV and so on
	uint16 lsc_start_addr;   // sony lsc start address
	uint16 lsc_addr_len;
	uint16 rsc_start_addr;   // sony rsc start address
	uint16 rsc_addr_len;
	uint16 pdc_addr;         // ov pdc start addr
	uint16 pdc_len;
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

struct camkit_sensor_params {
	uint8  sensor_name[SENSOR_NAME_LEN];  // sensor name: imx582, ov13855
	struct camkit_sensor_info_t sensor_info;
	struct camkit_sensor_ctrl_t sensor_ctrl;
	struct camkit_sensor_output_info_t output_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_pdaf_info_t pdaf_info;
	struct camkit_hw_power_info_t power_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_hw_power_info_t power_down_info[CAMKIT_POWER_INFO_MAX];
	struct camkit_aec_info_t aec_info;
	struct camkit_sensor_vc_info_t vc_info[MAX_OUTPUT_INFO_SIZE];
	struct camkit_sensor_correction_t correction_info;
	struct camkit_high_impedance_info_t high_impedance_info;
};

struct lcd_name_params {
	uint8 need_lcd_name;
	char lcd_name[LCD_NAME_LEN];
};

struct camkit_module_params {
	/* set 1 while skip to match module vendor */
	uint8  skip_module_id;
	uint16 eeprom_i2c_addr;
	uint16 module_code_addr;
	/* module vendor, such as: 01-sunny, 02-foxconn, 03-luxvisions, 06-ofilm */
	uint16 module_code;
	uint16 lens_type_addr;
	uint16 lens_type;
	enum camkit_i2c_addr_type addr_type;
	enum camkit_i2c_data_type data_type;

	struct lcd_name_params lcd_params;

	/* defined in kd_camkit_define.h, such as: 0x058202 */
	uint32 match_id;
	/* defined in kd_camkit_define.h, such as: imx582_foxconn */
	uint8  sensor_name[SENSOR_NAME_LEN];
	char vcm_name[MOTOR_NAME_LEN];
};

struct camkit_sensor_index_info_t {
	uint32 sensor_index;
	char sensor_name[SENSOR_NAME_LEN];
};

struct camkit_params {
	uint32 customized_enable;
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

#define KDIMGSENSORIOC_X_PROBESENSOR \
	_IOWR(IMGSENSORMAGIC, 0x1000, struct camkit_probe_sensor_params)
#define KDIMGSENSORIOC_X_ENABLELOG \
	_IOW(IMGVCMMAGIC, 0x1001, uint32)
#define KDIMGSENSORIOC_X_SETVCMNAME \
	_IOW(IMGVCMMAGIC, 10, struct vcm_motor_name)

#endif // KD_CAMKIT_DEFINE_H
