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
#include "eeprom_hw_map.h"
#include "kd_camkit_define.h"

#include "hi846_otp_driver.h"

#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	{
		.sensorID = OV48B2Q_LUXVISIONS_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = OV48B2Q_OFILM_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = IMX582_SUNNY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = IMX582_FOXCONN_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = HI846_OFILM_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = HI846_LUXVISIONS_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = S5K4H7_TRULY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = GC8054_BYD_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = S5K3P9_SUNNY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = OV16A1Q_OFILM_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = IMX471_FOXCONN_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = HI1634Q_FOXCONN_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = HI846_SUNNY_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = OV8856_FOXCONN_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = OV8856_OFILM_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = GC8054_QTECH_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = IMX355_QTECH_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = OV02A10_SUNNY_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = GC2375_FOXCONN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = GC2375_SUNWIN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = HI259_SUNWIN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	},
	/*  ADD before this line */
	{ 0, 0, 0, 0 }       /*end of list */
};

struct stCAM_CAL_LIST_STRUCT ts_camCalList[] = {
	{
		.sensorID = S5KGW1_TXD_SENSOR_ID,
		.slaveID = 0xA2,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = S5KGW1_XL_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = S5KGW1_QT_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = S5KGW1_TXD_LARGAN_SENSOR_ID,
 		.slaveID = 0xA2,
 		.getCamCalData = eeprom_hw_get_data,
 		.getMap = eeprom_back_camera_get_map,
 	}, {
		.sensorID = S5K3P9_TXD_SENSOR_ID,
		.slaveID = 0xA2,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = S5K3P9_TXD_TRULY_SENSOR_ID,
		.slaveID = 0xA2,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = HI1631Q_ST_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = HI1631Q_ST_TRULY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = HI846_TXD_SENSOR_ID,
		.slaveID = 0x44,
		.getCamCalData = hi846_read_buf_region,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = GC8034_LY_SENSOR_ID,
		.slaveID = 0xA6,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = GC2375_FOXCONN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = OV02A10_SUNNY_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = GC2375_SUNWIN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = HI259_SUNWIN_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	},
	/*  ADD before this line */
	{ 0, 0, 0, 0 }       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;
	if (g_product_name) {
		if (!strcmp(g_product_name, "Tianshan") || !strcmp(g_product_name, "TianshanSub"))
			*ppCamcalList = &ts_camCalList[0];
		else
			*ppCamcalList = &g_camCalList[0];
	}
	return 0;
}


