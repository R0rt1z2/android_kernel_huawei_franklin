/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __IMGSENSOR_HW_HWPMIC_H__
#define __IMGSENSOR_HW_HWPMIC_H__

#include "imgsensor_hw.h"
#include "imgsensor_common.h"

#include <linux/of.h>
#include <linux/device.h>
#include <linux/platform_device.h>

enum HWPMIC_VOLTAGE {
	HWPMIC_VOLTAGE_0 = 0,
	HWPMIC_VOLTAGE_MIN = 600000,
	HWPMIC_VOLTAGE_1000 = 1000000,
	HWPMIC_VOLTAGE_1050 = 1050000,
	HWPMIC_VOLTAGE_1100 = 1100000,
	HWPMIC_VOLTAGE_1200 = 1200000,
	HWPMIC_VOLTAGE_1210 = 1210000,
	HWPMIC_VOLTAGE_1220 = 1220000,
	HWPMIC_VOLTAGE_1250 = 1250000,
	HWPMIC_VOLTAGE_1500 = 1500000,
	HWPMIC_VOLTAGE_1800 = 1800000,
	HWPMIC_VOLTAGE_2500 = 2500000,
	HWPMIC_VOLTAGE_2800 = 2800000,
	HWPMIC_VOLTAGE_2850 = 2850000,
	HWPMIC_VOLTAGE_2900 = 2900000,
	HWPMIC_VOLTAGE_3000 = 3000000,
	HWPMIC_VOLTAGE_3300 = 3300000,
	HWPMIC_VOLTAGE_5000 = 5000000,
};

struct hwpmic_t {
	struct hw_pmic_ctrl_t *hw_pmic_ctrl;
	struct mutex *ghwpmic_mutex;
};

enum IMGSENSOR_RETURN imgsensor_hw_hwpmic_open(
	struct IMGSENSOR_HW_DEVICE **pdevice);

#endif

