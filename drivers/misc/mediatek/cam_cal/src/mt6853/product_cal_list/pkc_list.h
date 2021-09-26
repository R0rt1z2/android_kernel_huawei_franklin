/*
 * pkc_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * pkc_cal_list
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

#ifndef _PKC_CAL_LIST_H_
#define _PKC_CAL_LIST_H_

// main
static struct eeprom_hw_i2c_reg pkc_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x23C3, EEPROM_I2C_WORD_ADDR, 4320,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map pkc_main_eeprom_map_table = {
	.map = pkc_main_eeprom_map,
	.map_size = ARRAY_SIZE(pkc_main_eeprom_map),
};

// front
static struct eeprom_hw_i2c_reg pkc_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map pkc_front_eeprom_map_table = {
	.map = pkc_front_eeprom_map,
	.map_size = ARRAY_SIZE(pkc_front_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT pkc_camCalList[] = {
	{ // main-13m
		.sensorID = C645UAI_M090_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x34A2,
		.eeprom_hw_map = &pkc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C645WMR_M010_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x34A2,
		.eeprom_hw_map = &pkc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C645WMR_M100_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x34A2,
		.eeprom_hw_map = &pkc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C645XBA_M0C0_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x34A2,
		.eeprom_hw_map = &pkc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // front-8m
		.sensorID = C441FAH_M0C0_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1F49,
		.eeprom_hw_map = &pkc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C441UVO_M030_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1F49,
		.eeprom_hw_map = &pkc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C441UVO_M060_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1F49,
		.eeprom_hw_map = &pkc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C441FZB_M050_PKC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1F49,
		.eeprom_hw_map = &pkc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},

	// ADD before this line
	{ 0, 0, 0, 0 } // end of list
};

#endif // _PKC_CAL_LIST_H_
