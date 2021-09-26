#ifndef __EEPROM_HW_COMMON_H
#define __EEPROM_HW_COMMON_H

enum eeprom_i2c_addr_type {
	EEPROM_I2C_BYTE_ADDR = 1,
	EEPROM_I2C_WORD_ADDR,
	EEPROM_I2C_ADDR_MAX,
};

enum eeprom_i2c_data_type {
	EEPROM_I2C_BYTE_DATA = 1,
	EEPROM_I2C_WORD_DATA,
	EEPROM_I2C_DATA_MAX,
};

enum eeprom_i2c_operation {
	EEPROM_I2C_WRITE,
	EEPROM_I2C_READ,
	EEPROM_I2C_POLL,
};

struct eeprom_hw_i2c_reg {
	unsigned int addr;
	enum eeprom_i2c_addr_type addr_type;
	unsigned short data;
	enum eeprom_i2c_data_type data_type;
	enum eeprom_i2c_operation operation;
	unsigned short delay;
};

enum eeprom_hw_config_type {
	EEPROM_HW_ATTACH_CONFIG,
	EEPROM_HW_DETACH_CONFIG,
};

struct eeprom_hw_i2c_reg_setting {
	struct eeprom_hw_i2c_reg *setting;
	unsigned int size;
};

struct eeprom_hw_config {
	unsigned short need_config;
	unsigned int i2c_addr;
	struct eeprom_hw_i2c_reg_setting attach_setting;
	struct eeprom_hw_i2c_reg_setting detach_setting;
};

struct sensor_otp_hw_config {
	struct eeprom_hw_i2c_reg_setting attach_setting;
	struct eeprom_hw_i2c_reg_setting detach_setting;
};

struct sensor_otp_hw_i2c_reg {
	unsigned int addr;
	enum eeprom_i2c_addr_type addr_type;
	unsigned short data;
	enum eeprom_i2c_data_type data_type;
	enum eeprom_i2c_operation operation;
	unsigned short delay;
	unsigned int sensor_i2c_addr; /* sensor slave id */
	struct sensor_otp_hw_config *sensor_otp_switch;
};

struct eeprom_hw_map {
	struct eeprom_hw_i2c_reg *map;
	unsigned int map_size;
	struct eeprom_hw_config config_eeprom;
	struct sensor_otp_hw_i2c_reg *sensor_otp_map;
	unsigned int sensor_otp_map_size;
};

#endif /* __EEPROM_HW_COMMON_H */
