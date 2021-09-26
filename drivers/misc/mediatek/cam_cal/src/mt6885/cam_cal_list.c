/*
 * Copyright (C) 2018 MediaTek Inc.
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
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"
#include "eeprom_hw_driver.h"
#include "kd_camkit_define.h"
#include "product_cal_list.h"

#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

const product_cam_cal_map g_product_cal_map[] = {
	{ "York", &yor_camCalList[0] }, // for default
	{ "YorkV4", &yor_camCalList[0] },
	{ "Ocean", &oce_camCalList[0] },
	{ "Mrr", &mrr_camCalList[0] },
	{ "Amber", &abr_camCalList[0] },
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	unsigned int size;
	unsigned int i;

	if (!ppCamcalList)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	size = sizeof(g_product_cal_map) / sizeof(g_product_cal_map[0]);
	for (i = 0; i < size; ++i) {
		if (!strcmp(g_product_name, g_product_cal_map[i].product_name)) {
			*ppCamcalList = g_product_cal_map[i].cam_cal_list;
			pr_debug("imgsensor custom config get %s", g_product_name);
			break;
		}
	}
	return 0;
}


