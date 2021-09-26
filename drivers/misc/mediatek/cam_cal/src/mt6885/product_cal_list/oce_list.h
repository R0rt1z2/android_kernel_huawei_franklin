

#ifndef _OCE_CAL_LIST_H_
#define _OCE_CAL_LIST_H_

/* total: 32761(0x7FF9) */
static struct eeprom_hw_i2c_reg oce_main_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 32761,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map oce_main_eeprom_map_table = {
	.map = oce_main_eeprom_map,
	.map_size = ARRAY_SIZE(oce_main_eeprom_map),
};

/* total: 7901(0x1EDD) */
static struct eeprom_hw_i2c_reg oce_front_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 7901,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map oce_front_eeprom_map_table = {
	.map = oce_front_eeprom_map,
	.map_size = ARRAY_SIZE(oce_front_eeprom_map),
};

/* total: 16381(0x3FFD) */
static struct eeprom_hw_i2c_reg oce_wide_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 16381,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map oce_wide_eeprom_map_table = {
	.map = oce_wide_eeprom_map,
	.map_size = ARRAY_SIZE(oce_wide_eeprom_map),
};

/* total: 16381(0x3FFD) */
static struct eeprom_hw_i2c_reg oce_tele_eeprom_map[] = {
	{ 0x8000, EEPROM_I2C_WORD_ADDR, 16381,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map oce_tele_eeprom_map_table = {
	.map = oce_tele_eeprom_map,
	.map_size = ARRAY_SIZE(oce_tele_eeprom_map),
};

struct stCAM_CAL_LIST_STRUCT oce_camCalList[] = {
	{ /* main */
		.sensorID = C020FUC_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x7FF9,
		.eeprom_hw_map = &oce_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID =  C020FUC_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x7FF9,
		.eeprom_hw_map = &oce_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID =  C020FUC_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x7FF9,
		.eeprom_hw_map = &oce_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C020FUC_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.maxEepromSize = 0x7FF9,
		.eeprom_hw_map = &oce_main_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* front */
		.sensorID = C604UTF_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M020_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C604UTF_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_front_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C621ERE_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ERE_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ERE_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ERE_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* wide */
		.sensorID = C621ORS_M010_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ORS_M030_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ORS_M060_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C621ORS_M090_SENSOR_ID,
		.slaveID = 0xA0,
		.eeprom_hw_map = &oce_wide_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, { /* tele */
		.sensorID = C589OBY_M010_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &oce_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C589OBY_M030_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &oce_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C589OBY_M060_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &oce_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	}, {
		.sensorID = C589OBY_M090_SENSOR_ID,
		.slaveID = 0xA4,
		.eeprom_hw_map = &oce_tele_eeprom_map_table,
		.getCamCalData = eeprom_hw_get_data,
	},
	/* ADD before this line */
	{ 0, 0, 0, 0 } /* end of list */
};

#endif /* _OCE_CAL_LIST_H_ */
