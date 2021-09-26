/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: eeprom back tsh camera
 * Author: mazhaung
 * Create: 2020-08-01
 */

#include "eeprom_back_tsh_camera.h"

static struct eeprom_hw_i2c_reg tsh_eeprom_map[] = {
	{ 0x00, EEPROM_I2C_WORD_ADDR, 4558, EEPROM_I2C_BYTE_DATA, EEPROM_I2C_READ, 0x00 },
};

static struct eeprom_hw_map tsh_eeprom_map_table = {
	.map = tsh_eeprom_map,
	.map_size = ARRAY_SIZE(tsh_eeprom_map),
};

int eeprom_back_tsh_camera_get_map(unsigned int sensor_id, struct eeprom_hw_map **map)
{
	pr_info("%s, %d: sensor_id: 0x%x\n", __func__, __LINE__, sensor_id);
	if (!map) {
		pr_err("%s, %d: map is null\n", __func__, __LINE__);
		return -1;
	}
	*map = &tsh_eeprom_map_table;
	return 0;
}
