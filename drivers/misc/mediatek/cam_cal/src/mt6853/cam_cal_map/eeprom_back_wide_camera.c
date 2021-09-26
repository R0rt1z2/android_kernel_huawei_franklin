#include "eeprom_back_wide_camera.h"

// wn project wide camera otp
static struct eeprom_hw_i2c_reg eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 39,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x15ea, EEPROM_I2C_WORD_ADDR, 2400,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map eeprom_map_table = {
	.map = eeprom_map,
	.map_size = ARRAY_SIZE(eeprom_map),
};

int eeprom_back_wide_camera_get_map(unsigned int sensor_id,
	struct eeprom_hw_map **map)
{
	pr_err("%s, %d: sensor_id: 0x%x\n", __func__, __LINE__, sensor_id);
	if (!map) {
		pr_err("%s, %d: map is null\n", __func__, __LINE__);
		return -1;
	}

	*map = &eeprom_map_table;

	return 0;
}
