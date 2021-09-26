/*
 * jsc_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * jsc_cal_list
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

#ifndef _JSC_CAL_LIST_H_
#define _JSC_CAL_LIST_H_

/* total: 9684(0x25D4) */
static struct eeprom_hw_i2c_reg jsc_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jsc_main_eeprom_map_table = {
	.map = jsc_main_eeprom_map,
	.map_size = ARRAY_SIZE(jsc_main_eeprom_map),
};

/* 601knk */
static struct eeprom_hw_i2c_reg eeprom_attach_setting[] = {
	{ 0x02, EEPROM_I2C_BYTE_ADDR, 0x00,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x05 },
};

static struct eeprom_hw_i2c_reg eeprom_detach_setting[] = {
	{ 0x02, EEPROM_I2C_BYTE_ADDR, 0x01,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
};

static struct eeprom_hw_map jsc_601knk_main_eeprom_map_table = {
	.map = jsc_main_eeprom_map,
	.map_size = ARRAY_SIZE(jsc_main_eeprom_map),
	.config_eeprom = {
		.need_config = 1,
		.i2c_addr = 0x18 >> 1,
		.attach_setting = {
			.setting = eeprom_attach_setting,
			.size = ARRAY_SIZE(eeprom_attach_setting),
		},
		.detach_setting = {
			.setting = eeprom_detach_setting,
			.size = ARRAY_SIZE(eeprom_detach_setting),
		},
	}
};

/* total: 4439(0x1157) */
static struct eeprom_hw_i2c_reg jsc_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x258a, EEPROM_I2C_WORD_ADDR, 4400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jsc_front_eeprom_map_table = {
	.map = jsc_front_eeprom_map,
	.map_size = ARRAY_SIZE(jsc_front_eeprom_map),
};

/* total: 2439(0x0987) */
static struct eeprom_hw_i2c_reg jsc_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jsc_wide_eeprom_map_table = {
	.map = jsc_wide_eeprom_map,
	.map_size = ARRAY_SIZE(jsc_wide_eeprom_map),
};

/* total: 1939(0x0793) */
static struct eeprom_hw_i2c_reg jsc_micro_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1000, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map jsc_micro_eeprom_map_table = {
	.map = jsc_micro_eeprom_map,
	.map_size = ARRAY_SIZE(jsc_micro_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT jsc_camCalList[] = {
	{ /* main */
		.sensorID = C601KES_M060_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &jsc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C601GHA_M090_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &jsc_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C601KNK_M010_JSC_SENSOR_ID,
		.slaveID = 0xB0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &jsc_601knk_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* front */
		.sensorID = C627OGU_M010_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jsc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627OGI_M020_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jsc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627FUV_M060_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jsc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C627JDI_M020_JSC_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &jsc_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C628UVO_M010_JSC_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &jsc_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M020_JSC_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &jsc_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M060_JSC_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &jsc_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628BDH_M090_JSC_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &jsc_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* macro */
		.sensorID = C648YGA_M110_JSC_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jsc_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M010_JSC_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jsc_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648EOY_M120_JSC_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &jsc_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
	/* ADD before this line */
	{ 0, 0, 0, 0 } /* end of list */
};

#endif /* _JSC_CAL_LIST_H_ */
