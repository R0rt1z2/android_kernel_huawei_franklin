#ifndef __EEPROM_FRONT_CAMERA_H
#define __EEPROM_FRONT_CAMERA_H
#include "eeprom_hw_driver.h"

int eeprom_front_camera_get_map(unsigned int sensor_id,
	struct eeprom_hw_map **map);

#endif /* __EEPROM_BACK_CAMERA_H */
