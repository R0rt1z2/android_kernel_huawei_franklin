/*
 * cdy_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * cdy_cal_list
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

#ifndef _CDY_CAL_LIST_H_
#define _CDY_CAL_LIST_H_

static struct eeprom_hw_i2c_reg main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282D, EEPROM_I2C_WORD_ADDR, 5595,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map main_eeprom_map_table = {
	.map = main_eeprom_map,
	.map_size = ARRAY_SIZE(main_eeprom_map),
};

static struct eeprom_hw_i2c_reg front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x258a, EEPROM_I2C_WORD_ADDR, 4400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map front_eeprom_map_table = {
	.map = front_eeprom_map,
	.map_size = ARRAY_SIZE(front_eeprom_map),
};

static struct eeprom_hw_i2c_reg wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map wide_eeprom_map_table = {
	.map = wide_eeprom_map,
	.map_size = ARRAY_SIZE(wide_eeprom_map),
};

static struct eeprom_hw_i2c_reg micro_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xfa0, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map micro_eeprom_map_table = {
	.map = micro_eeprom_map,
	.map_size = ARRAY_SIZE(micro_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT cdy_camCalList[] = {
	{
		.sensorID = C647KES_M060_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D2,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C647KES_M020_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D2,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C647KNK_M090_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D2,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627OGU_M010_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627OGI_M020_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627FUV_M060_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627JDI_M020_CDY_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628UVO_M010_CDY_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M020_CDY_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M060_CDY_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628BDH_M090_CDY_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637YGA_M010_CDY_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M020_CDY_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M110_CDY_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		0, 0, 0, 0
	} /* end of list */
};

#endif /* _CDY_CAL_LIST_H_ */
