/*
 * yor_cal_list.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * yor_cal_list
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

#ifndef __YOR_CAL_LIST_H
#define __YOR_CAL_LIST_H

// total: 8061(0x1F7D) need to check
static struct eeprom_hw_i2c_reg yor_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AEE, EEPROM_I2C_WORD_ADDR, 42,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x17E7, EEPROM_I2C_WORD_ADDR, 4007,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x282E, EEPROM_I2C_WORD_ADDR, 3965,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x26, EEPROM_I2C_WORD_ADDR, 8,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x35, EEPROM_I2C_WORD_ADDR, 1,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map yor_main_eeprom_map_table = {
	.map = yor_main_eeprom_map,
	.map_size = ARRAY_SIZE(yor_main_eeprom_map),
};

/*
 * c658fuv-total: eeprom 4439(0x1157) + sensor otp 1920 = 6359 bytes
 * c658jdi-total: eeprom 4439(0x1157) + sensor otp 955 = 5394 bytes
 * c658ogu-total: eeprom 4439(0x1157)
 */
static struct eeprom_hw_i2c_reg yor_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x258a, EEPROM_I2C_WORD_ADDR, 4400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

/* for 658fuv sensor otp attach */
static struct eeprom_hw_i2c_reg yor_658fuv_dpc_attach_setting[] = {
	{ 0x0103, EEPROM_I2C_WORD_ADDR, 0x01,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
	{ 0x3D84, EEPROM_I2C_WORD_ADDR, 0x00,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
	{ 0x3D85, EEPROM_I2C_WORD_ADDR, 0x1B,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
	{ 0x0100, EEPROM_I2C_WORD_ADDR, 0x01,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
};

/* for 658fuv sensor otp detach */
static struct eeprom_hw_i2c_reg yor_658fuv_dpc_detach_setting[] = {
	{ 0x0100, EEPROM_I2C_WORD_ADDR, 0x00,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_WRITE, 0x00 },
};

static struct sensor_otp_hw_config yor_658fuv_dpc_switch = {
	.attach_setting = {
		.setting = yor_658fuv_dpc_attach_setting,
		.size = ARRAY_SIZE(yor_658fuv_dpc_attach_setting),
	},
	.detach_setting = {
		.setting = yor_658fuv_dpc_detach_setting,
		.size = ARRAY_SIZE(yor_658fuv_dpc_detach_setting),
	},
};

static struct sensor_otp_hw_i2c_reg yor_658fuv_sensor_otp_map[] = {
	{
		0x7010, EEPROM_I2C_WORD_ADDR, 1920,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00,
		(0x6c >> 1), /* sensor slave addr */
		&yor_658fuv_dpc_switch,
	},
};

static struct eeprom_hw_map yor_front_658fuv_map_table = {
	.map = yor_front_eeprom_map,
	.map_size = ARRAY_SIZE(yor_front_eeprom_map),
	.sensor_otp_map = yor_658fuv_sensor_otp_map,
	.sensor_otp_map_size = ARRAY_SIZE(yor_658fuv_sensor_otp_map),
};

static struct sensor_otp_hw_i2c_reg yor_658jdi_sensor_otp_map[] = {
	{
		0x7678, EEPROM_I2C_WORD_ADDR, 4,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00,
		(0x34 >> 1), /* sensor slave addr */
		NULL,
	}, {
		0x8B00, EEPROM_I2C_WORD_ADDR, 16,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00,
		(0x34 >> 1), /* sensor slave addr */
		NULL,
	}, {
		0x8B10, EEPROM_I2C_WORD_ADDR, 935,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00,
		(0x34 >> 1), /* sensor slave addr */
		NULL,
	},
};

static struct eeprom_hw_map yor_front_658jdi_map_table = {
	.map = yor_front_eeprom_map,
	.map_size = ARRAY_SIZE(yor_front_eeprom_map),
	.sensor_otp_map = yor_658jdi_sensor_otp_map,
	.sensor_otp_map_size = ARRAY_SIZE(yor_658jdi_sensor_otp_map),
};

static struct eeprom_hw_map yor_front_eeprom_map_table = {
	.map = yor_front_eeprom_map,
	.map_size = ARRAY_SIZE(yor_front_eeprom_map),
};

/* total: 2439(0x0987) */
static struct eeprom_hw_i2c_reg yor_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map yor_wide_eeprom_map_table = {
	.map = yor_wide_eeprom_map,
	.map_size = ARRAY_SIZE(yor_wide_eeprom_map),
};

/* total: 1939(0x0793) */
static struct eeprom_hw_i2c_reg yor_micro_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x1000, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map yor_micro_eeprom_map_table = {
	.map = yor_micro_eeprom_map,
	.map_size = ARRAY_SIZE(yor_micro_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT yor_camCalList[] = {
	{
		.sensorID = C602DVP_M020_YOR_SENSOR_ID,
		.slaveID = 0xA6,
		.eeprom_hw_map = &yor_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C602DVP_M030_YOR_SENSOR_ID,
		.slaveID = 0xA6,
		.eeprom_hw_map = &yor_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C602DVP_M090_YOR_SENSOR_ID,
		.slaveID = 0xA6,
		.eeprom_hw_map = &yor_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C658OGU_M010_YOR_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &yor_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C658JDI_M020_YOR_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &yor_front_658jdi_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C658FUV_M060_YOR_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &yor_front_658fuv_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C611UVO_M010_YOR_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &yor_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C611JWG_M090_YOR_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &yor_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C611OAK_M020_YOR_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &yor_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M110_YOR_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &yor_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648YGA_M010_YOR_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &yor_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648EOY_M120_YOR_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &yor_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C648QVV_M110_YOR_SENSOR_ID,
		.slaveID = 0xAC,
		.eeprom_hw_map = &yor_micro_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
};

#endif /* __yor_CAL_LIST_H */
