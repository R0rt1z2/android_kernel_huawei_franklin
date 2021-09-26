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
#include "product_cal_list.h"
#include "c6021uvo_otp_driver.h"

#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	{
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
	},
	/*  ADD before this line */
	{ 0, 0, 0, 0 }       /*end of list */

};

struct stCAM_CAL_LIST_STRUCT wn_camCalList[] = {
	{
		.sensorID = S5K3L6_TRULY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = HI1336_QTECH_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = IMX258_SUNNY_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = IMX258_HOLITECH_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = C577QCP_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = C441UVO_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = C441UVO_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = C441FAH_M120_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = C441FZB_M050_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_camera_get_map,
	}, {
		.sensorID = C512JWG_M020_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C512JWG_M060_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C512TBQ_M050_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_ex_camera_get_map,
	}, {
		.sensorID = C512KEH_M030_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_ex_camera_get_map,
	}, {
		.sensorID = C646HQF_M110_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_ex_camera_get_map,
	},
	/*  ADD before this line */
	{ 0, 0, 0, 0 }       /*end of list */
};

struct stCAM_CAL_LIST_STRUCT frl_camCalList[] = {
	{
		.sensorID = C576OGI_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = C576JDI_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = C576JDI_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	}, {
		.sensorID = C576FUV_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front16m_camera_get_map,
	},  {
		.sensorID = C394YGA_M010_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = C394YGA_M0B0_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = C394EOY_M090_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = C394EOY_M0C0_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = C394QVV_M0B0_FRL_SENSOR_ID,
		.slaveID = 0xAC,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_aux_camera_get_map,
	}, {
		.sensorID = C570OAK_M020_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C570FZB_M030_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C570JWG_M090_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C570UVO_M010_SENSOR_ID,
		.slaveID = 0xA4,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_wide_camera_get_map,
	}, {
		.sensorID = C483DVY_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = C483DVY_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_camera_get_map,
	}, {
		.sensorID = C483DBC_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_c483dbc_camera_get_map,
	}, {
		.sensorID = C483DBC_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_c483dbc_camera_get_map,
	}, {
		.sensorID = C591RAO_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = C591RAO_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = C591RAO_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = C591LKY_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = C591LKY_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	}, {
		.sensorID = C591LKY_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_ex_camera_get_map,
	},
	/*  ADD before this line */
	{ 0, 0, 0, 0 }       /*end of list */
};

struct stCAM_CAL_LIST_STRUCT tsh_camcallist[] = {
	{
		.sensorID = C7021DVY_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_back_tsh_camera_get_map,
	}, {
		.sensorID = C7041UVO_M120_SENSOR_ID,
		.slaveID = 0xA0,
		.getCamCalData = eeprom_hw_get_data,
		.getMap = eeprom_front_tsh_camera_get_map,
	}, {
		.sensorID = C6021UVO_M190_SENSOR_ID,
		.slaveID = 0x44,
		.getCamCalData = c6021uvo_read_buf_region,
		.getMap = eeprom_back_wide_tsh_camera_get_map,
	},
	/* ADD before this line */
	{ 0, 0, 0, 0 }
};

const product_cam_cal_map g_product_cal_map[] = {
	{ "Wukong", &wn_camCalList[0] }, // for default
	{ "WukongV3", &wn_camCalList[0] },
	{ "Frl", &frl_camCalList[0] },
	{ "Cindy", &cdy_camCalList[0] },
	{ "Jessica", &jsc_camCalList[0] },
	{ "Tianshanb", &tsh_camcallist[0] },
	{ "Chanel", &chl_camCalList[0] },
	{ "ChanelV4", &chl_camCalList[0] },
	{ "Ntn", &ntn_camCalList[0] },
	{ "Ntn40m", &ntn_camCalList[0] },
	{ "NtnV4", &ntn_camCalList[0] },
	{ "Angela", &ang_camCalList[0] },
	{ "Ntn40mV4", &ntn_camCalList[0] },
	{ "Pkc", &pkc_camCalList[0] },
	{ "ChanelV4Lite", &chl_camCalList[0] },
	{ "PrdLite", &chl_camCalList[0] },
	{ "Nto", &ntn_camCalList[0] },
	{ "Julia", &jlh_camCalList[0] },
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


