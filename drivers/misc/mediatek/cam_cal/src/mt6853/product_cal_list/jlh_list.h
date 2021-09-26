/*
 * jlh_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * jlh_cal_list
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

#ifndef _JLH_CAL_LIST_H_
#define _JLH_CAL_LIST_H_

/* total: 9684(0x25D4) */
static struct eeprom_hw_i2c_reg jlh_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jlh_main_eeprom_map_table = {
	.map = jlh_main_eeprom_map,
	.map_size = ARRAY_SIZE(jlh_main_eeprom_map),
};

/* total: 4987(0x137b) */
static struct eeprom_hw_i2c_reg jlh_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 37,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xAEE, EEPROM_I2C_WORD_ADDR, 3056,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x16DF, EEPROM_I2C_WORD_ADDR, 1895,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jlh_front_eeprom_map_table = {
	.map = jlh_front_eeprom_map,
	.map_size = ARRAY_SIZE(jlh_front_eeprom_map),
};

/* total: 2439(0x0987) */
static struct eeprom_hw_i2c_reg jlh_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jlh_wide_eeprom_map_table = {
	.map = jlh_wide_eeprom_map,
	.map_size = ARRAY_SIZE(jlh_wide_eeprom_map),
};

/* total: 1939(0x0793) */
static struct eeprom_hw_i2c_reg jlh_micro_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1000, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jlh_micro_eeprom_map_table = {
	.map = jlh_micro_eeprom_map,
	.map_size = ARRAY_SIZE(jlh_micro_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT jlh_camCalList[] = {
	{ /* main */
		.sensorID = C668MUF_M090_JLH_SENSOR_ID,
		.slaveID = 0xA6,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &jlh_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* front */
		.sensorID = C651FUV_M090_JLH_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jlh_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C695UVO_M010_JLH_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jlh_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C695UVO_M090_JLH_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jlh_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* macro */
		.sensorID = C648YGA_M110_JLH_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jlh_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M010_JLH_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jlh_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648EOY_M120_JLH_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jlh_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
	/* ADD before this line */
	{ 0, 0, 0, 0 } /* end of list */
};

#endif /* _JLH_CAL_LIST_H_ */
