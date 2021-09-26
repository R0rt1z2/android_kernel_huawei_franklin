/*
 * camkit_driver.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * sensor interface define: 3A, initial operators
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

#ifndef CAMKIT_DRIVER_H
#define CAMKIT_DRIVER_H

#include <securec.h>
#include "camkit_driver_types.h"
#include "../../../../../hwcam_hiview/hwcam_hiview.h"

typedef enum {
	DRIVER_ERROR_TYPE_START = 0,
	READ_EEPROM_QSC_DATA_FAILED = DRIVER_ERROR_TYPE_START,
	READ_EEPROM_SPC_DATA_FAILED,
	WRITE_LSC_DATA_FAILED,
	WRITE_RSC_DATA_FAILED,
	WRITE_PDC_DATA_FAILED,
	WRITE_QSC_DATA_FAILED,
	SET_INIT_SETTING_FAILED,
	SET_RESOLUTION_SETTING_FAILED,
	DRIVER_ERROR_TYPE_END,
} driver_error_type;

struct scenaio_pdaf_table {
	enum MSDK_SCENARIO_ID_ENUM scenaio_id;
	uint8 pdaf_support;
};

struct scenaio_index_table {
	enum MSDK_SCENARIO_ID_ENUM scenaio_id;
	uint16 index;
};

uint32 sensor_driver_init(struct sensor_kit_ops **ops);

void set_shutter(struct camkit_sensor_params *params, uint32 shutter);
uint16 set_gain(struct camkit_sensor_params *params, uint16 gain);
void set_dummy(struct camkit_sensor_params *params);
void set_max_framerate(struct camkit_sensor_params *params,
	uint16 framerate, bool min_framelength_en);
void set_shutter_frame_length(struct camkit_sensor_params *params,
	uint32 shutter, uint32 frame_length, struct aec_ops_map *expo_ops_map);
void camkit_hiview_report(int error_no, const char *ic_name,
	driver_error_type error_type);
void set_awb_gain(struct camkit_sensor_params *params,
	struct SET_SENSOR_AWB_GAIN *set_sensor_awb);
uint16 hdr_set_gain(struct camkit_sensor_params *params, uint16 l_gain, uint16 m_gain);
void hdr_set_shutter(struct camkit_sensor_params *params, uint32 l_shutter, uint32 s_shutter);
#endif // CAMKIT_DRIVER_H
