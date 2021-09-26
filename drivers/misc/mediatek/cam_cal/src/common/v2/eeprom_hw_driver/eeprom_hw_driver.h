#ifndef __EEPROM_HW_DRIVER_H
#define __EEPROM_HW_DRIVER_H

#include "eeprom_hw_i2c.h"
#include "eeprom_hw_common.h"
#include "cam_cal_list.h"


int eeprom_hw_get_data(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

extern const char *g_product_name;
#endif /* __EEPROM_HW_DRIVER_H */
