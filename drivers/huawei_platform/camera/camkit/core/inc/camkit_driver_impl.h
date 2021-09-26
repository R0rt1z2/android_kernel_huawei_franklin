/*
 * camkit_driver_impl.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: normalized driver interface
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

#ifndef CAMKIT_DRIVER_IMPL_H
#define CAMKIT_DRIVER_IMPL_H

#include <linux/spinlock.h>

#include "camkit_driver_types.h"

extern spinlock_t camkit_lock;

uint32 camkit_get_normalized_ops(struct sensor_kit_ops **ops);

uint32 camkit_open(struct camkit_sensor *sensor);
uint32 camkit_close(struct camkit_sensor *sensor);

uint32 camkit_match_id(struct camkit_sensor *sensor, uint32 *match_id);
uint32 camkit_sensor_init(struct camkit_sensor *sensor);
uint32 camkit_get_sensor_info(struct camkit_sensor *sensor,
	struct camkit_sensor_info_t *sensor_info);
uint32 camkit_control(struct camkit_sensor *sensor,
	const uint32 scenario_id);

uint32 camkit_get_scenario_pclk(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pclk);
uint32 camkit_get_scenario_period(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *period);
uint32 camkit_set_test_pattern(struct camkit_sensor *sensor,
	const uint8 enable);
uint32 camkit_dump_reg(struct camkit_sensor *sensor);
uint32 camkit_set_auto_flicker(struct camkit_sensor *sensor,
	const uint8 enable, const uint16 framerate);
uint32 camkit_get_default_framerate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *framerate);
uint32 camkit_get_crop_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_output_info_t *win_info);
uint32 camkit_streaming_control(struct camkit_sensor *sensor,
	const uint8 enable);
uint32 camkit_get_mipi_pixel_rate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *mipi_pixel_rate);
uint32 (camkit_get_mipi_trail_val)(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *mipi_trail_val);
uint32 camkit_get_sensor_pixel_rate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pixel_rate);
uint32 camkit_get_pdaf_capacity(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pdaf_support, uint32 para_len);
uint32 camkit_get_binning_ratio(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *ratio);
uint32 camkit_get_pdaf_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_pdaf_info_t *pdaf_info);
uint32 camkit_get_vc_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_vc_info_t *vc_info);
uint32 camkit_get_pdaf_regs_data(struct camkit_sensor *sensor,
	uint16 *reg_pairs, const uint32 reg_num);
uint32 camkit_set_pdaf_setting(struct camkit_sensor *sensor,
	uint16 *setting, const uint32 reg_num);
uint32 camkit_set_video_mode(struct camkit_sensor *sensor,
	const uint16 framerate);
uint32 camkit_set_current_fps(struct camkit_sensor *sensor,
	const uint16 framerate);
uint32 camkit_set_pdaf_mode(struct camkit_sensor *sensor, const uint16 mode);

uint32 camkit_set_scenario_framerate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 framerate);
uint32 camkit_set_max_framerate(struct camkit_sensor *sensor,
	uint16 framerate, uint8 min_framelength_en);
uint32 camkit_set_shutter(struct camkit_sensor *sensor, const uint32 shutter);
uint32 camkit_set_gain(struct camkit_sensor *sensor, const uint16 gain);
uint32 camkit_set_dummy(struct camkit_sensor *sensor);
uint32 camkit_set_shutter_frame_length(struct camkit_sensor *sensor,
	uint32 shutter, uint32 frame_length);

#endif // CAMKIT_DRIVER_IMPL_H
