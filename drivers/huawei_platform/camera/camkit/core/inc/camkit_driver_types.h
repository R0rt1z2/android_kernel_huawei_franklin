/*
 * camkit_driver_types.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: define common structures and interface for image sensor driver
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

#ifndef CAMKIT_DRIVER_TYPES_H
#define CAMKIT_DRIVER_TYPES_H

#include <linux/printk.h>
#include "kd_camkit_define.h"

#include "camkit_log.h"

#define MAX_I2C_REG_NUM 64

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define return_err_if_null(param) do { \
	if ((param) == NULL) {          \
		log_err("invalid param");   \
		return ERR_INVAL; \
	} \
} while (0)

#define return_void_if_null(param) do { \
	if ((param) == NULL) {          \
		log_err("invalid param");   \
		return; \
	} \
} while (0)

#define return_0_if_null(param) do { \
	if ((param) == NULL) {          \
		log_err("invalid param");   \
		return ERR_NONE; \
	} \
} while (0)

enum camkit_err_code {
	ERR_NONE = 0,
	ERR_INVAL,
	ERR_NOMEM,
	ERR_IO,
};

struct camkit_sensor {
	uint32 probe_success;
	uint32 sensor_idx;
	uint8  sensor_name[SENSOR_NAME_LEN];
	struct sensor_kit_ops *sensor_ops;
	struct camkit_params *kit_params;
};

struct aec_ctrl_cfg {
	uint32 again;      // record the register value of again
	uint8 again_valid;
	uint32 dgain;      // record the register value of dgain
	uint8 dgain_valid;
	uint32 line_count;
	uint8 lc_valid;
	uint32 frame_length;
	uint8 fl_valid;
	uint32 group_hold;
	uint8 group_hold_valid;
	uint32 shift;
	uint8 shift_valid;
	uint32 long_line_count;
	uint8 llc_valid;
};

struct sensor_i2c_cfg {
	uint16 addr;
	uint16 val;
	uint16 addr_type;
	uint16 data_type;
	enum camkit_i2c_mode i2c_mode;
	uint16 delay;
};

struct sensor_setting_cfg {
	struct sensor_i2c_cfg i2c_setting[MAX_I2C_REG_NUM];
	uint16 size;
	uint16 delay;
};

struct sensor_kit_ops {
	uint32 (*sensor_open)(struct camkit_sensor *sensor);
	uint32 (*sensor_close)(struct camkit_sensor *sensor);

	uint32 (*match_id)(struct camkit_sensor *sensor, uint32 *match_id);
	uint32 (*sensor_init)(struct camkit_sensor *sensor);
	uint32 (*get_sensor_info)(struct camkit_sensor *sensor,
		struct camkit_sensor_info_t *sensor_info);
	uint32 (*control)(struct camkit_sensor *sensor,
		const uint32 scenario_id);

	uint32 (*get_scenario_pclk)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *pclk);
	uint32 (*get_scenario_period)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *period);
	uint32 (*set_test_pattern)(struct camkit_sensor *sensor,
		const uint8 enable);
	uint32 (*dump_reg)(struct camkit_sensor *sensor);
	uint32 (*set_auto_flicker)(struct camkit_sensor *sensor,
		const uint8 enable, const uint16 framerate);
	uint32 (*get_default_framerate)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *framerate);
	uint32 (*get_crop_info)(struct camkit_sensor *sensor,
		const uint32 scenario_id,
		struct camkit_sensor_output_info_t *win_info);
	uint32 (*streaming_control)(struct camkit_sensor *sensor,
		const uint8 enable);
	uint32 (*get_mipi_pixel_rate)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *mipi_pixel_rate);
	uint32 (*get_mipi_trail_val)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *mipi_trail_val);
	uint32 (*get_sensor_pixel_rate)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *pixel_rate);
	uint32 (*get_pdaf_capacity)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *pdaf_support, uint32 para_len);
	uint32 (*get_binning_ratio)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 *ratio);
	uint32 (*get_pdaf_info)(struct camkit_sensor *sensor,
		const uint32 scenario_id, struct camkit_sensor_pdaf_info_t *pdaf_info);
	uint32 (*get_vc_info)(struct camkit_sensor *sensor,
		const uint32 scenario_id, struct camkit_sensor_vc_info_t *vc_info);
	uint32 (*get_pdaf_regs_data)(struct camkit_sensor *sensor,
		uint16 *reg_pairs, const uint32 reg_num);
	uint32 (*set_pdaf_setting)(struct camkit_sensor *sensor,
		uint16 *setting, const uint32 reg_num);
	uint32 (*set_video_mode)(struct camkit_sensor *sensor,
		const uint16 framerate);
	uint32 (*set_shutter)(struct camkit_sensor *sensor, const uint32 shutter);
	uint32 (*set_gain)(struct camkit_sensor *sensor, const uint16 gain);
	uint32 (*set_dummy)(struct camkit_sensor *sensor);
	uint32 (*set_max_framerate)(struct camkit_sensor *sensor,
		uint16 framerate, uint8 min_framelength_en);
	uint32 (*set_scenario_framerate)(struct camkit_sensor *sensor,
		const uint32 scenario_id, uint32 framerate);
	uint32 (*set_shutter_frame_length)(struct camkit_sensor *sensor,
		uint32 shutter, uint32 frame_length);
	uint32 (*set_current_fps)(struct camkit_sensor *sensor,
		const uint16 framerate);
	uint32 (*set_pdaf_mode)(struct camkit_sensor *sensor, const uint16 mode);
};

enum sensor_shutter_mode {
	SHUTTER_INVALID = -1,
	SHUTTER_DEFAULT,
	SHUTTER_NORMAL2LONG,
	SHUTTER_LONG2NORMAL,
};

struct customized_driver {
	uint32 sensor_idx;
	uint32 sensor_id;
	uint32 (*get_ops)(struct sensor_kit_ops **ops);
};

#define register_customized_driver(name, index, id, func)   \
	static struct customized_driver _customized_driver_##name	\
	__used __attribute__ ((unused, __section__("__customized_driver"))) \
	= { index, id, func }

#endif // CAMKIT_DRIVER_TYPES_H
