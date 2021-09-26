/*
 * chl_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * chl_cal_list
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

#ifndef _CHL_CAL_LIST_H_
#define _CHL_CAL_LIST_H_

// total: 9684(0x25D4)
static struct eeprom_hw_i2c_reg chl_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_main_eeprom_map_table = {
	.map = chl_main_eeprom_map,
	.map_size = ARRAY_SIZE(chl_main_eeprom_map),
};

// total: 5677(0x162D) knk_m020_chl
static struct eeprom_hw_i2c_reg chl_knk_m020_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1A82, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_knk_m020_main_eeprom_map_table = {
	.map = chl_knk_m020_main_eeprom_map,
	.map_size = ARRAY_SIZE(chl_knk_m020_main_eeprom_map),
};

// total: 6405(0x1905) ov48b
static struct eeprom_hw_i2c_reg chl_48b_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1798, EEPROM_I2C_WORD_ADDR, 2510,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x235c, EEPROM_I2C_WORD_ADDR, 3856,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_48b_main_eeprom_map_table = {
	.map = chl_48b_main_eeprom_map,
	.map_size = ARRAY_SIZE(chl_48b_main_eeprom_map),
};

// total: 7974(0x1f26) imx582
static struct eeprom_hw_i2c_reg chl_582_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0X0b04, EEPROM_I2C_WORD_ADDR, 5518,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x20F7, EEPROM_I2C_WORD_ADDR, 2418,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_582_main_eeprom_map_table = {
	.map = chl_582_main_eeprom_map,
	.map_size = ARRAY_SIZE(chl_582_main_eeprom_map),
};

// total: 4439(0x1157)
static struct eeprom_hw_i2c_reg chl_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x258a, EEPROM_I2C_WORD_ADDR, 4400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_front_eeprom_map_table = {
	.map = chl_front_eeprom_map,
	.map_size = ARRAY_SIZE(chl_front_eeprom_map),
};

// total: 2439(0x0987)
static struct eeprom_hw_i2c_reg chl_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_wide_eeprom_map_table = {
	.map = chl_wide_eeprom_map,
	.map_size = ARRAY_SIZE(chl_wide_eeprom_map),
};

// total: 1939(0x0793)
static struct eeprom_hw_i2c_reg chl_micro_eeprom_map_648[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1000, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_micro_eeprom_map_table_648 = {
	.map = chl_micro_eeprom_map_648,
	.map_size = ARRAY_SIZE(chl_micro_eeprom_map_648),
};

static struct eeprom_hw_i2c_reg chl_micro_eeprom_map_637[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xfa0, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map chl_micro_eeprom_map_table_637 = {
	.map = chl_micro_eeprom_map_637,
	.map_size = ARRAY_SIZE(chl_micro_eeprom_map_637),
};

struct stCAM_CAL_LIST_STRUCT chl_camCalList[] = {
	{ // main-64m
		.sensorID = C615KES_M060_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &chl_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KES_M021_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &chl_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KNK_M010_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &chl_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KNK_M020_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x162D,
		.eeprom_hw_map = &chl_knk_m020_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615GHA_M021_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &chl_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // main-48m
		.sensorID = C606DVY_M030_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1905,
		.eeprom_hw_map = &chl_48b_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DVY_M060_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1905,
		.eeprom_hw_map = &chl_48b_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DBC_M010_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1f26,
		.eeprom_hw_map = &chl_582_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DBC_M020_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1f26,
		.eeprom_hw_map = &chl_582_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // front
		.sensorID = C627OGU_M010_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &chl_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627FUV_M060_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &chl_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627JDI_M020_CHL_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &chl_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // wide
		.sensorID = C628UVO_M010_CHL_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &chl_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M020_CHL_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &chl_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M060_CHL_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &chl_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628BDH_M090_CHL_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &chl_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // macro
		.sensorID = C637YGA_M010_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M0B0_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M020_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M0B0_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M0C1_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M010_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M0B0_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648EOY_M0C0_CHL_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &chl_micro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	},
	// ADD before this line
	{ 0, 0, 0, 0 } // end of list
};

#endif // _CHL_CAL_LIST_H_
