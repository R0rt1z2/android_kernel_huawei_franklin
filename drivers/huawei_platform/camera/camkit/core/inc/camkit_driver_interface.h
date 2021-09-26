/*
 * camkit_driver_interface.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: camkit sensor interface layer
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

#ifndef CAMKIT_SENSOR_INTERFACE_H
#define CAMKIT_SENSOR_INTERFACE_H

#include "camkit_driver_types.h"

int32 hwsensor_set_log_mask(const uint32 mask);

int32 hwsensor_power(struct camkit_sensor *sensor,
	enum camkit_power_status pwr_status);
uint32 hwsensor_match_id(const uint32 sensor_idx, uint32 *match_id);
char *hwsensor_get_sensor_name(const uint32 sensor_idx);

int32 hwsensor_open(const uint32 sensor_idx);
int32 hwsensor_close(const uint32 sensor_idx);
uint32 hwsensor_get_sensor_info(const uint32 sensor_idx,
	struct camkit_sensor_info_t *sensor_info);
uint32 hwsensor_control(const uint32 sensor_idx,
	const uint32 scenario_id);

// feature control
uint32 hwsensor_get_gain_range(const uint32 sensor_idx,
	uint32 *min_gain, uint32 *max_gain);
uint32 hwsensor_get_gain_info(const uint32 sensor_idx,
	uint32 *min_iso, uint32 *gain_step, uint32 *gain_type);
uint32 hwsensor_get_min_shutter(const uint32 sensor_idx,
	uint32 *min_shutter);
uint32 hwsensor_get_period(const uint32 sensor_idx,
	uint16 *line_length, uint16 *frame_length);
uint32 hwsensor_get_pclk(const uint32 sensor_idx, uint32 *pclk);
uint32 hwsensor_get_checksum(const uint32 sensor_idx, uint32 *checksum);
uint32 hwsensor_get_scenario_pclk(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *pclk);
uint32 hwsensor_get_scenario_period(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *period);
uint32 hwsensor_set_shutter(const uint32 sensor_idx,
	const uint32 shutter);
uint32 hwsensor_set_gain(const uint32 sensor_idx,
	const uint16 gain);
uint32 hwsensor_set_video_mode(const uint32 sensor_idx,
	const uint16 framerate);
uint32 hwsensor_set_auto_flicker(const uint32 sensor_idx,
	const uint8 enable, const uint16 framerate);
uint32 hwsensor_set_current_fps(const uint32 sensor_idx,
	const uint16 framerate);
uint32 hwsensor_streaming_control(const uint32 sensor_idx, uint8 enable);
uint32 hwsensor_get_crop_info(const uint32 sensor_idx,
	const uint32 scenario_id, struct camkit_sensor_output_info_t *win_info);
uint32 hwsensor_get_pdaf_info(const uint32 sensor_idx,
	const uint32 scenario_id, struct camkit_sensor_pdaf_info_t *pdaf_info);
uint32 hwsensor_set_test_pattern(const uint32 sensor_idx, uint8 enable);
uint32 hwsensor_dump_reg(const uint32 sensor_idx);
uint32 hwsensor_get_mipi_pixel_rate(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *mipi_pixel_rate);
uint32 hwsensor_get_mipi_trail_val(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *mipi_trail_val);
uint32 hwsensor_get_sensor_pixel_rate(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *pixel_rate);
uint32 hwsensor_set_pdaf_mode(const uint32 sensor_idx,
	const uint16 pdaf_mode);
uint32 hwsensor_set_pdaf_setting(const uint32 sensor_idx,
	uint16 *setting, const uint32 reg_num);
uint32 hwsensor_get_pdaf_regs_data(const uint32 sensor_idx,
	uint16 *reg_pairs, const uint32 reg_num);
uint32 hwsensor_get_vc_info(const uint32 sensor_idx,
	const uint32 scenario_id, struct camkit_sensor_vc_info_t *vc_info);
uint32 hwsensor_get_binning_ratio(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *ratio);
uint32 hwsensor_set_shutter_frame_length(const uint32 sensor_idx,
	uint32 shutter, uint32 frame_length);
uint32 hwsensor_get_default_framerate(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *framerate);
uint32 hwsensor_get_pdaf_capacity(const uint32 sensor_idx,
	const uint32 scenario_id, uint32 *pdaf_support, uint32 para_len);
uint32 hwsensor_set_scenario_framerate(const uint32 sensor_idx,
	const uint32 scenario_id, const uint32 framerate);

#endif // CAMKIT_SENSOR_INTERFACE_H

