

#ifndef _ABR_CAL_LIST_H_
#define _ABR_CAL_LIST_H_

/* total: 16381(0x3FFD) */
static struct eeprom_hw_i2c_reg abr_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 16381,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map abr_main_eeprom_map_table = {
	.map = abr_main_eeprom_map,
	.map_size = ARRAY_SIZE(abr_main_eeprom_map),
};

/* total: 7901(0x1EDD) */
static struct eeprom_hw_i2c_reg abr_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 7901,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map abr_front_eeprom_map_table = {
	.map = abr_front_eeprom_map,
	.map_size = ARRAY_SIZE(abr_front_eeprom_map),
};

/* total: 16381(0x3FFD) */
static struct eeprom_hw_i2c_reg abr_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 16381,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map abr_wide_eeprom_map_table = {
	.map = abr_wide_eeprom_map,
	.map_size = ARRAY_SIZE(abr_wide_eeprom_map),
};

/* total: 16381(0x3FFD) */
static struct eeprom_hw_i2c_reg abr_tele_eeprom_map[] = {
	{ 0x8000, EEPROM_I2C_WORD_ADDR, 16381,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map abr_tele_eeprom_map_table = {
	.map = abr_tele_eeprom_map,
	.map_size = ARRAY_SIZE(abr_tele_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT abr_camCalList[] = {
	{ /* main */
		.sensorID =  C642BXB_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID =  C642BXB_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C642BXB_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* front */
		.sensorID = C604UTF_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &abr_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &abr_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &abr_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &abr_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &abr_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C643UTF_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C643UTF_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C643UTF_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C029OBY_M010_SENSOR_ID,
		.slaveID = 0xA2,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C029OBY_M030_SENSOR_ID,
		.slaveID = 0xA2,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C029OBY_M060_SENSOR_ID,
		.slaveID = 0xA2,
		.maxEepromSize = 0x3FFD,
		.eeprom_hw_map = &abr_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
	/* ADD before this line */
	{ 0, 0, 0, 0 } /* end of list */
};

#endif /* _ABR_CAL_LIST_H_ */
