/*
 * camkit_dts_adapter.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: parse pin type and value from dts
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

#ifndef CAMKIT_DTS_ADAPTER_H
#define CAMKIT_DTS_ADAPTER_H

#include "imgsensor_hw.h"

struct IMGSENSOR_HW_CFG *get_sensor_hw_cfg(void);
enum IMGSENSOR_I2C_DEV get_sensor_i2c_dev(const int sensor_idx);

#endif // CAMKIT_DTS_ADAPTER_H
