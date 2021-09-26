/*
 * ntn_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * ntn_cal_list
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

#ifndef _NTN_CAL_LIST_H_
#define _NTN_CAL_LIST_H_

// total: 9684(0x25D4) imx600
static struct eeprom_hw_i2c_reg ntn_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_main_eeprom_map_table = {
	.map = ntn_main_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_main_eeprom_map),
};

// total: 5677(0x162D) c615knk_m021
static struct eeprom_hw_i2c_reg ntn_knk_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1A82, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_knk_main_eeprom_map_table = {
	.map = ntn_knk_main_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_knk_main_eeprom_map),
};

// total: 6405(0x1905) ov48b
static struct eeprom_hw_i2c_reg ntn_48b_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1798, EEPROM_I2C_WORD_ADDR, 2510,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x235c, EEPROM_I2C_WORD_ADDR, 3856,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_48b_main_eeprom_map_table = {
	.map = ntn_48b_main_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_48b_main_eeprom_map),
};

// total: 7974(0x1f26) imx582
static struct eeprom_hw_i2c_reg ntn_582_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0X0b04, EEPROM_I2C_WORD_ADDR, 5518,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x20F7, EEPROM_I2C_WORD_ADDR, 2418,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_582_main_eeprom_map_table = {
	.map = ntn_582_main_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_582_main_eeprom_map),
};

// total: 4439(0x1157)
static struct eeprom_hw_i2c_reg ntn_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x258a, EEPROM_I2C_WORD_ADDR, 4400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_front_eeprom_map_table = {
	.map = ntn_front_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_front_eeprom_map),
};

// total: 2439(0x0987)
static struct eeprom_hw_i2c_reg ntn_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_wide_eeprom_map_table = {
	.map = ntn_wide_eeprom_map,
	.map_size = ARRAY_SIZE(ntn_wide_eeprom_map),
};

// total: 1939(0x0793)
static struct eeprom_hw_i2c_reg ntn_macro_eeprom_map_648[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1000, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_macro_eeprom_map_table_648 = {
	.map = ntn_macro_eeprom_map_648,
	.map_size = ARRAY_SIZE(ntn_macro_eeprom_map_648),
};

static struct eeprom_hw_i2c_reg ntn_macro_eeprom_map_637[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xfa0, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ntn_macro_eeprom_map_table_637 = {
	.map = ntn_macro_eeprom_map_637,
	.map_size = ARRAY_SIZE(ntn_macro_eeprom_map_637),
};

struct stCAM_CAL_LIST_STRUCT ntn_camCalList[] = {
	{
		.sensorID = C606DBC_M011_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1f26,
		.eeprom_hw_map = &ntn_582_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DBC_M021_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1f26,
		.eeprom_hw_map = &ntn_582_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DVY_M061_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1905,
		.eeprom_hw_map = &ntn_48b_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C606DVY_M031_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1905,
		.eeprom_hw_map = &ntn_48b_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // main 40m
		.sensorID = C669RAO_M060_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x1f26,
		.eeprom_hw_map = &ntn_582_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // main 64m
		.sensorID = C615GHA_M020_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &ntn_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KES_M020_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &ntn_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KES_M061_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &ntn_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KNK_M011_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D4,
		.eeprom_hw_map = &ntn_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KNK_M021_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x162D,
		.eeprom_hw_map = &ntn_knk_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // front
		.sensorID = C658OGU_M011_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ntn_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C658JDI_M021_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ntn_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C658FUV_M061_NTN_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ntn_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // wide
		.sensorID = C628BDH_M091_NTN_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ntn_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628UVO_M011_NTN_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ntn_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M021_NTN_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ntn_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M061_NTN_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ntn_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // macro
		.sensorID = C648EOY_M0C1_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M012_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M0B2_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648QVV_M0B3_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_648,
		.getCamCalData = eeprom_hw_get_data,
	}, { // macro
		.sensorID = C637YGA_M013_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M0B1_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M021_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M0B2_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M0C0_NTN_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ntn_macro_eeprom_map_table_637,
		.getCamCalData = eeprom_hw_get_data,
	},
	// ADD before this line
	{ 0, 0, 0, 0 } // end of list
};

#endif // _NTN_CAL_LIST_H_
