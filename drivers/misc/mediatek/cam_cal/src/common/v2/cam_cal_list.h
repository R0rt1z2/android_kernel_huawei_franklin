/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#ifndef __CAM_CAL_LIST_H
#define __CAM_CAL_LIST_H
#include <linux/i2c.h>
#include "eeprom_hw_driver/eeprom_hw_common.h"

#define DEFAULT_MAX_EEPROM_SIZE 0x8000

struct stCAM_CAL_LIST_STRUCT;

typedef unsigned int (*cam_cal_cmd_func) (struct i2c_client *client,
	unsigned int addr, unsigned char *data, unsigned int size);

struct stCAM_CAL_LIST_STRUCT {
	unsigned int sensorID;
	unsigned int slaveID;
	cam_cal_cmd_func readCamCalData;
	unsigned int maxEepromSize;
	cam_cal_cmd_func writeCamCalData;
	struct eeprom_hw_map *eeprom_hw_map;
	int (*getCamCalData)(struct i2c_client *client,
		struct stCAM_CAL_LIST_STRUCT *list,
		unsigned int addr, unsigned char *data, unsigned int size);

	int (*getMap)(unsigned int sensor_id, struct eeprom_hw_map **map);
};
#define MAX_PRODUCT_LEN 32
typedef struct {
	char product_name[MAX_PRODUCT_LEN];
	struct stCAM_CAL_LIST_STRUCT *cam_cal_list;
} product_cam_cal_map;

unsigned int cam_cal_get_sensor_list
		(struct stCAM_CAL_LIST_STRUCT **ppCamcalList);

#endif				/* __CAM_CAL_LIST_H */
