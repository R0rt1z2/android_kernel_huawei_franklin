/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: sensor eeprom config
 * Author: liming
 * Create: 2020-08-20
 */

#ifndef __MRR_CAL_LIST_H
#define __MRR_CAL_LIST_H

// main 4320 + 39 = 4359  0x11CF
static struct eeprom_hw_i2c_reg main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x23C3, EEPROM_I2C_WORD_ADDR, 4320,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map main_eeprom_map_table = {
	.map = main_eeprom_map,
	.map_size = ARRAY_SIZE(main_eeprom_map),
};

// sub 2400 + 39 = 2439 0x987
static struct eeprom_hw_i2c_reg front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15EA, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },

};

static struct eeprom_hw_map front_eeprom_map_table = {
	.map = front_eeprom_map,
	.map_size = ARRAY_SIZE(front_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT mrr_camCalList[] = {
	{
		.sensorID = C645UAI_M090_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x11CF,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, 	{
		.sensorID = C645WMR_M050_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x11CF,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, 	{
		.sensorID = C645XBA_M120_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x11CF,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C645WMR_M010_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x11CF,
		.eeprom_hw_map = &main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C474UVO_M050_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x987,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, 	{
		.sensorID = C474FAH_M120_MRR_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x987,
		.eeprom_hw_map = &front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		0, 0, 0, 0
	} /* end of list */
};

#endif /* __CDY_CAL_LIST_H */
