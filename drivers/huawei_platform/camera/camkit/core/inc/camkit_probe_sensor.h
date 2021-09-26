/*
 * camkit_probe_sensor.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: probe sensor interface
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

#ifndef CAMKIT_PROBE_SENSOR_H
#define CAMKIT_PROBE_SENSOR_H

#include "camkit_driver_types.h"

int32 hwsensor_probe_sensor(const uint32 sensor_idx,
	struct camkit_params *user_kit_params);

struct camkit_sensor *get_camkit_sensor(const uint32 index);

#endif // CAMKIT_PROBE_SENSOR_H

