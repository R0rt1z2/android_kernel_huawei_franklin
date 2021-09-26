#include "eeprom_front16m_camera.h"

static struct eeprom_hw_i2c_reg frl_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 38,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0x0AF3, EEPROM_I2C_WORD_ADDR, 1180,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
	{ 0xF91, EEPROM_I2C_WORD_ADDR, 1900,
		EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map frl_eeprom_map_table = {
	.map = frl_eeprom_map,
	.map_size = ARRAY_SIZE(frl_eeprom_map),
};

int eeprom_front16m_camera_get_map(unsigned int sensor_id,
	struct eeprom_hw_map **map)
{
	pr_err("%s, %d: sensor_id: 0x%x\n", __func__, __LINE__, sensor_id);
	if (!map) {
		pr_err("%s, %d: map is null\n", __func__, __LINE__);
		return -1;
	}
	*map = &frl_eeprom_map_table;
	return 0;
}
