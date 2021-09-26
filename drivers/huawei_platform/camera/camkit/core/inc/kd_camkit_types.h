/*
 * ke_camkit_types.h
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

#ifndef KD_CAMKIT_TYPES_H
#define KD_CAMKIT_TYPES_H

#include "kd_camkit_define_xa.h"
/*
 * define sensor normalized parameters as follow:
 */
typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define camkit_array_size(x) (sizeof(x) / sizeof((x)[0]))
#define CAMKIT_LINGLENGTH_GAP 80

#define MAX_OUTPUT_INFO_SIZE 16
#define MAX_AEC_REGS 25
#define MAX_AGAIN_REGS 16
#define SENSOR_NAME_LEN 32
#define MOTOR_NAME_LEN 32
#define MAX_FUSE_ID_SIZE 30
#define MAX_HIGH_IMPEDANCE_SENSOR_NUM 2
#define LCD_NAME_LEN 32

#define CAMKIT_POWER_INFO_MAX 20

enum camkit_i2c_addr_type {
	CAMKIT_I2C_BYTE_ADDR = 1,
	CAMKIT_I2C_WORD_ADDR,
	CAMKIT_I2C_ADDR_MAX,
};

enum camkit_i2c_data_type {
	CAMKIT_I2C_BYTE_DATA = 1,
	CAMKIT_I2C_WORD_DATA,
	CAMKIT_I2C_3B_DATA,
	CAMKIT_I2C_DWORD_DATA,
	CAMKIT_I2C_DATA_MAX,
};

enum camkit_i2c_operation {
	CAMKIT_I2C_WRITE,
	CAMKIT_I2C_READ,
	CAMKIT_I2C_POLL,
	CAMKIT_I2C_READ_BUFFER,
};

enum camkit_i2c_dev_id_t {
	CAMKIT_I2C_DEV_0,
	CAMKIT_I2C_DEV_1,
	CAMKIT_I2C_DEV_2,
	CAMKIT_I2C_DEV_3,
	CAMKIT_I2C_DEV_4,
	CAMKIT_I2C_DEV_5,
	CAMKIT_I2C_DEV_6,
	CAMKIT_I2C_DEV_7,
	CAMKIT_I2C_DEV_MAX_NUM,
};

enum camkit_common_mode {
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
};

enum camkit_scenario_type {
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
};

enum pdaf_support_type {
	CAMKIT_PDAF_SUPPORT_NA = 0,
	CAMKIT_PDAF_SUPPORT_RAW = 1,
	CAMKIT_PDAF_SUPPORT_CAMSV = 2,
	CAMKIT_PDAF_SUPPORT_CAMSV_LEGACY = 3,
	CAMKIT_PDAF_SUPPORT_RAW_DUALPD = 4,
	CAMKIT_PDAF_SUPPORT_CAMSV_DUALPD = 5,
	CAMKIT_PDAF_SUPPORT_RAW_LEGACY = 6,
};

enum camkit_sensor_index {
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
};

enum camkit_isp_driver_current {
	CAMKIT_ISP_DRIVING_2MA = 0,
	CAMKIT_ISP_DRIVING_4MA,
	CAMKIT_ISP_DRIVING_6MA,
	CAMKIT_ISP_DRIVING_8MA,
	CAMKIT_ISP_DRIVING_MAX_NUM
};

enum camkit_image_type {
	CAMKIT_IMAGE_NORMAL = 0,
	CAMKIT_IMAGE_H_MIRROR,
	CAMKIT_IMAGE_V_MIRROR,
	CAMKIT_IMAGE_HV_MIRROR
};

enum camkit_sensor_intf_type {
	CAMKIT_SENSOR_INTF_TYPE_PARALLEL = 0,
	CAMKIT_SENSOR_INTF_TYPE_MIPI,
	CAMKIT_SENSOR_INTF_TYPE_SERIAL,
	CAMKIT_SENSOR_INTF_TYPE_MAX
};

enum camkit_sensor_output_format {
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_B = 0,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_GB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_GR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_UYVY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_VYUY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YUYV,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YVYU,
	CAMKIT_SENSOR_OUTPUT_FORMAT_CBYCRY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_CRYCBY,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YCBYCR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_YCRYCB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_GB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_GR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_WB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_WR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_RWB_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_MONO,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_GB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_GR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_GB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_GR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_GB,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_GR,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW8_MONO,
	CAMKIT_SENSOR_OUTPUT_FORMAT_RAW_IR,
};

enum camkit_sensor_mipi_lane {
	CAMKIT_SENSOR_MIPI_1_LANE = 0,
	CAMKIT_SENSOR_MIPI_2_LANE,
	CAMKIT_SENSOR_MIPI_3_LANE,
	CAMKIT_SENSOR_MIPI_4_LANE
};

enum camkit_sensor_data_channel {
	CAMKIT_SCAM_1_DATA_CHANNEL = 0,
	CAMKIT_SCAM_2_DATA_CHANNEL,
	CAMKIT_SCAM_3_DATA_CHANNEL,
	CAMKIT_SCAM_4_DATA_CHANNEL,
};

enum camkit_mipi_type {
	CAMKIT_MIPI_OPHY_NCSI2 = 0,
	CAMKIT_MIPI_OPHY_CSI2 = 1,
	CAMKIT_MIPI_CPHY = 2,
};

enum camkit_power_status {
	CAMKIT_HW_POWER_STATUS_OFF,
	CAMKIT_HW_POWER_STATUS_ON
};

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
	/* gpio pin end */

	/* regulator pin begin */
	CAMKIT_HW_PIN_AVDD,
	CAMKIT_HW_PIN_DVDD,
	CAMKIT_HW_PIN_DOVDD,
	CAMKIT_HW_PIN_AFVDD,
	/* regulator pin end */

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
	CAMKIT_HW_PIN_VALUE_2900,
	CAMKIT_HW_PIN_VALUE_3000,
};

enum camkit_again_type {
	/* the sensor accord with SMIA standard, include gc8054 */
	CAMKIT_AGAIN_STD = 0,
	/* the sensor from galaxycore, such as gc2375, gc8034 */
	CAMKIT_AGAIN_GC,
	/* the analog gain is not continuous, such as ov13855, ov16b10 and so on */
	CAMKIT_AGAIN_OV13855,
	CAMKIT_AGAIN_MAX,
};

enum aec_op_type {
	SENSOR_AEC_OP_GROUPON,
	SENSOR_AEC_OP_GROUPOFF,
	SENSOR_AEC_OP_VTS,
	SENSOR_AEC_OP_LC,
	SENSOR_AEC_OP_SHIFT,
	SENSOR_AEC_OP_AGAIN,
	SENSOR_AEC_OP_DGAIN,
	/* other control registers, such as: switch page mode about hi259 */
	SENSOR_AEC_OP_CTRL,
	/* only for gc sensor, such as: gc2375, gc8034 */
	SENSOR_AEC_OP_AGAIN_TABLE,
	/* for sony dual camera preshutter, setup only befor stream on */
	SENSOR_AEC_OP_PRESHUTTER,

	/* for ov long expo mode,like HWI OV16B10 */
	SENSOR_AEC_OP_LONG_EXPO_PSV_MODE,
	SENSOR_AEC_OP_LONG_EXPO_MODE_CONFIG,

	SENSOR_AEC_OP_LONG_EXPO_LIMIT,
	SENSOR_AEC_OP_LONG_LC,
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

enum camkit_long_expo_type {
	SENSOR_LONG_EXPO_NONE = 0,  // long exposure unsupportted
	SENSOR_LONG_EXPO_SONY,      // sony long exposure: cit_lshift
	SENSOR_LONG_EXPO_OV,        // OV long exposure
	SENSOR_LONG_EXPO_SAMSUNG,   // SumSung long exposure
	SENSOR_LONG_EXPO_SETTINGS,  // implement long exposure by settings
	SENSOR_LONG_EXPO_MAX,
};

enum camkit_spc_type {
	PDAF_SPC_NONE = 0,
	PDAF_SPC_LRC,               // sony pdaf correction: LSC and RSC
	PDAF_SPC_PDC,               // OV pdaf correction
	PDAF_SPC_MAX,
};

#endif // KD_CAMKIT_TYPES_H
