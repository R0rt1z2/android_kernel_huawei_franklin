/*
 * ang_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * ang_cal_list
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

#ifndef _ANG_CAL_LIST_H_
#define _ANG_CAL_LIST_H_

// total: 9684(0x25D4) need modify
static struct eeprom_hw_i2c_reg ang_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 5597,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ang_main_eeprom_map_table = {
	.map = ang_main_eeprom_map,
	.map_size = ARRAY_SIZE(ang_main_eeprom_map),
};

/* 615knk */
static struct eeprom_hw_i2c_reg ang_eeprom_attach_setting[] = {
	{ 0x02, EEPROM_I2C_BYTE_ADDR, 0x00,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x05 },
};

static struct eeprom_hw_i2c_reg ang_eeprom_detach_setting[] = {
	{ 0x02, EEPROM_I2C_BYTE_ADDR, 0x01,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
};

static struct eeprom_hw_map ang_615knk_main_eeprom_map_table = {
	.map = ang_main_eeprom_map,
	.map_size = ARRAY_SIZE(ang_main_eeprom_map),
	.config_eeprom = {
		.need_config = 1,
		.i2c_addr = 0x18 >> 1,
		.attach_setting = {
			.setting = ang_eeprom_attach_setting,
			.size = ARRAY_SIZE(ang_eeprom_attach_setting),
		},
		.detach_setting = {
			.setting = ang_eeprom_detach_setting,
			.size = ARRAY_SIZE(ang_eeprom_detach_setting),
		},
	}
};

// total: 4726(0x1276) need modify
static struct eeprom_hw_i2c_reg ang_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AFF, EEPROM_I2C_WORD_ADDR, 2539,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x169A, EEPROM_I2C_WORD_ADDR, 2150,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ang_front_eeprom_map_table = {
	.map = ang_front_eeprom_map,
	.map_size = ARRAY_SIZE(ang_front_eeprom_map),
};

// total: 2439(0x0987)
static struct eeprom_hw_i2c_reg ang_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ang_wide_eeprom_map_table = {
	.map = ang_wide_eeprom_map,
	.map_size = ARRAY_SIZE(ang_wide_eeprom_map),
};

static struct eeprom_hw_i2c_reg ang_micro_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xfa0, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map ang_micro_eeprom_map_table = {
	.map = ang_micro_eeprom_map,
	.map_size = ARRAY_SIZE(ang_micro_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT ang_camCalList[] = {
	{ // main
		.sensorID = C615KES_M060_ANG_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &ang_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615GHA_M090_ANG_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &ang_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C615KNK_M010_ANG_SENSOR_ID,
		.slaveID = 0xB0,
		.maxEepromSize = 0x25D5,
		.eeprom_hw_map = &ang_615knk_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // front need modify
		.sensorID = C652YRV_M020_ANG_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ang_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C652RFU_M060_ANG_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ang_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C652IZR_M090_ANG_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &ang_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // wide
		.sensorID = C628UVO_M010_ANG_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ang_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M020_ANG_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ang_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628JWG_M060_ANG_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ang_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C628BDH_M090_ANG_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &ang_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { // macro
		.sensorID = C637YGA_M010_ANG_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ang_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M020_ANG_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ang_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637EOY_M110_ANG_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ang_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M110_ANG_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ang_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C637QVV_M120_ANG_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &ang_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
	// ADD before this line
	{ 0, 0, 0, 0 } // end of list
};

#endif // _ANG_CAL_LIST_H_
