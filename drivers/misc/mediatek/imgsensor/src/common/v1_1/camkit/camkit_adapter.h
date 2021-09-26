/*
 * camkit_adapter.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camkit interface adapted on the platform
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

#ifndef CAMKIT_ADAPTER_H
#define CAMKIT_ADAPTER_H

typedef struct {
	uint32 camkit_sensor_idx;
	enum IMGSENSOR_SENSOR_IDX platform_sensor_idx;
} sensor_idx_map_t;

int32 camkit_sensor_open(struct IMGSENSOR_SENSOR *psensor);

uint32 camkit_sensor_get_info(
	struct IMGSENSOR_SENSOR *psensor,
	uint32 scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data);

uint32 camkit_sensor_get_resolution(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution);

uint32 camkit_sensor_control(
	struct IMGSENSOR_SENSOR *psensor,
	enum MSDK_SCENARIO_ID_ENUM scenario_id);

uint32 camkit_sensor_feature_control(
		struct IMGSENSOR_SENSOR *psensor,
		MSDK_SENSOR_FEATURE_ENUM feature_id,
		MUINT8 *feature_param,
		MUINT32 *feature_param_len);

int32 camkit_sensor_close(struct IMGSENSOR_SENSOR *psensor);

int adopt_camera_hw_probe_sensor(void *pBuf);

enum IMGSENSOR_RETURN camkit_hw_power(
	struct IMGSENSOR_HW *phw,
	struct IMGSENSOR_SENSOR *psensor,
	enum IMGSENSOR_HW_POWER_STATUS pwr_status);

int camkit_driver_init(struct IMGSENSOR *img_sensor);

#endif // CAMKIT_ADAPTER_H

