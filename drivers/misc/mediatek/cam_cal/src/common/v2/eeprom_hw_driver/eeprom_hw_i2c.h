#ifndef __EEPROM_HW_I2C_H
#define __EEPROM_HW_I2C_H

#include <linux/i2c.h>
#include "eeprom_hw_common.h"

int eeprom_i2c_read(struct i2c_client *client,
	unsigned int addr, unsigned char *data,
	enum eeprom_i2c_addr_type addr_type,
	enum eeprom_i2c_data_type data_type);

int eeprom_i2c_read_seq(struct i2c_client *client,
	unsigned int addr, unsigned char *data,
	unsigned int data_length,
	enum eeprom_i2c_addr_type addr_type);

int eeprom_i2c_write(struct i2c_client *client,
	unsigned int addr, unsigned short data,
	enum eeprom_i2c_addr_type addr_type,
	enum eeprom_i2c_data_type data_type);

#endif /* __EEPROM_HW_I2C_H */
