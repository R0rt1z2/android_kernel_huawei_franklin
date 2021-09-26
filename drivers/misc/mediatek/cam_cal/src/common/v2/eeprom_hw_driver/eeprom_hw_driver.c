#include "eeprom_hw_driver.h"
#include <linux/delay.h>
#define LOG_DBG_ENABLE 0
#define LOG_DBG(fmt, args...) \
	do { \
	if (LOG_DBG_ENABLE) \
		pr_info("%s %d " \
		fmt, __func__, __LINE__, ##args); \
	} while (0)

#define loge_if(x) \
	do { \
		if (x) \
			pr_err("'%s' failed, ret = %d", #x, x); \
	} while (0)

static int eeprom_hw_i2c_process(struct i2c_client *client,
	struct eeprom_hw_i2c_reg *map, unsigned char *data, unsigned int size)
{
	int rc = 0;
	enum eeprom_i2c_operation operation = map->operation;

	switch (operation) {
	case EEPROM_I2C_READ:
		rc = eeprom_i2c_read_seq(client, map->addr, data,
			map->data, map->addr_type);
	break;
	case EEPROM_I2C_WRITE:
		rc = eeprom_i2c_write(client, map->addr, map->data,
			map->addr_type, map->data_type);
	break;
	default:
		pr_err("%s, %d: Invalid operations: %d\n", __func__,
			__LINE__, operation);
		rc = -1;
	break;
	}
	if (rc < 0) {
		pr_err("%s, %d: operation %d failed\n", __func__, __LINE__,
			operation);
		return rc;
	}

	if (map->delay > 0)
		msleep(map->delay);

	return rc;
}

void eeprom_hw_driver_dump_data(unsigned char *data, unsigned int size)
{
	unsigned int i;

	if (!data) {
		pr_err("%s,%d: Null ptr\n", __func__, __LINE__);
		return;
	}
	LOG_DBG("begin dump\n");
	for (i = 0; i < size; ++i)
		LOG_DBG("0x%x\n", data[i]);
	LOG_DBG("end dump\n");
}

static int eeprom_hw_write_setting(struct i2c_client *client,
	struct eeprom_hw_i2c_reg_setting *reg_setting)
{
	unsigned int i;
	int ret = 0;
	struct eeprom_hw_i2c_reg *setting = NULL;

	if (!reg_setting || !reg_setting->setting) {
		pr_err("setting is null\n");
		return -1;
	}

	for (i = 0; i < reg_setting->size; ++i) {
		setting = &reg_setting->setting[i];
		ret = eeprom_i2c_write(client, setting->addr, setting->data,
			setting->addr_type, setting->data_type);
		if (ret < 0) {
			pr_err("%s,%u: index: %u error, ret: %d\n", __func__,
				__LINE__, i, ret);
			return -1;
		}
		if (setting->delay > 0)
			msleep(setting->delay);
	}
	return 0;
}

static int config_eeprom(struct i2c_client *client,
	struct eeprom_hw_map *map_table,
	enum eeprom_hw_config_type config_type)
{
	int ret = 0;
	struct eeprom_hw_i2c_reg_setting *reg_setting = NULL;
	unsigned short init_addr;

	if (!map_table) {
		pr_err("param is null\n");
		return -1;
	}

	/* 1: need config */
	if (map_table->config_eeprom.need_config != 1) {
		LOG_DBG("no need config eeprom");
		return 0;
	}

	if (config_type == EEPROM_HW_ATTACH_CONFIG)
		reg_setting = &map_table->config_eeprom.attach_setting;
	else if (config_type == EEPROM_HW_DETACH_CONFIG)
		reg_setting = &map_table->config_eeprom.detach_setting;

	init_addr = client->addr;
	client->addr = map_table->config_eeprom.i2c_addr;

	if (eeprom_hw_write_setting(client, reg_setting) < 0) {
		pr_err("eeprom_hw_write_setting error\n");
		ret = -1;
	}
	client->addr = init_addr;

	return ret;
}

/* local static function */
static int read_data_from_sensor_otp(struct eeprom_hw_map *map,
	struct i2c_client *client,
	unsigned char *buf,
	unsigned int *size)
{
	int rc = 0;
	unsigned int i;
	unsigned int data_size;
	struct sensor_otp_hw_config *sensor_otp_switch = NULL;
	unsigned int left_size = *size;
	unsigned int origial_i2c_addr = client->addr;
	struct sensor_otp_hw_i2c_reg *sensor_otp_map = map->sensor_otp_map;

	for (i = 0; i < map->sensor_otp_map_size; ++i) {
		data_size = sensor_otp_map[i].data;
		if (left_size < data_size) {
			pr_err("%s no enough room left, left_size %u, data size %u\n",
				__func__, left_size, data_size);
			rc = -1;
			break;
		}

		sensor_otp_switch = sensor_otp_map[i].sensor_otp_switch;
		client->addr = sensor_otp_map[i].sensor_i2c_addr;

		if (sensor_otp_switch && sensor_otp_switch->attach_setting.setting)
			loge_if(eeprom_hw_write_setting(client,
				&sensor_otp_switch->attach_setting));

		rc = eeprom_i2c_read_seq(client, sensor_otp_map[i].addr, buf,
			data_size, sensor_otp_map[i].addr_type);

		if (sensor_otp_switch && sensor_otp_switch->detach_setting.setting)
			loge_if(eeprom_hw_write_setting(client,
				&sensor_otp_switch->detach_setting));

		if (rc < 0) {
			pr_err("%s read sensor otp error, index %u\n", __func__, i);
			break;
		}

		left_size -= data_size;
		buf += data_size;
		pr_info("read data from sensor otp %u bytes, left size %u\n",
			data_size, left_size);
	}

	*size = left_size;
	client->addr = origial_i2c_addr;

	return rc;
}

int eeprom_hw_get_data(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size)
{
	int rc;
	int i = 0;
	int tmp_size = size;
	struct eeprom_hw_map *map_table = NULL;
	struct eeprom_hw_i2c_reg *map = NULL;
	unsigned char *buf = data;

	pr_info("%s, %d: Enter\n", __func__, __LINE__);
	if (!client || !list || !data || size == 0) {
		pr_err("%s, %d: NULL ptr\n", __func__, __LINE__);
		return -1;
	}

	if (!list->eeprom_hw_map) {
		pr_info("%s, %d: no eeprom_hw_map\n", __func__, __LINE__);
		if (!list->getMap) {
			pr_err("no map ops\n");
			return -1;
		}

		rc = list->getMap(list->sensorID, &map_table);
		if (rc < 0) {
			pr_err("get map failed\n");
			return -1;
		}
	} else {
		map_table = list->eeprom_hw_map;
	}

	loge_if(config_eeprom(client, map_table, EEPROM_HW_ATTACH_CONFIG));

	for (i = 0; i < map_table->map_size; ++i) {
		map = &map_table->map[i];
		if (map->operation == EEPROM_I2C_READ) {
			if (tmp_size < map->data) {
				pr_err("size err: %s,%d: read size: %d, block size:%d\n",
					__func__, __LINE__,
					tmp_size, map->data);
				return -1;
			}
		}
		LOG_DBG("read size: %d, block size: %d\n", tmp_size, map->data);
		rc = eeprom_hw_i2c_process(client, map, buf, tmp_size);
		if (rc < 0) {
			pr_err("%s,%d: index: %d error\n", __func__,
				__LINE__, i);
			return rc;
		}
		if (map->operation == EEPROM_I2C_READ) {
			tmp_size -= map->data;
			buf += map->data;
		}
	}

	loge_if(config_eeprom(client, map_table, EEPROM_HW_DETACH_CONFIG));

	if (map_table->sensor_otp_map &&
		map_table->sensor_otp_map_size != 0) {
		rc = read_data_from_sensor_otp(map_table, client, buf, &tmp_size);
		if (rc < 0) {
			pr_err("%s: get sensor otp failed, rc = %d\n",
				__func__, rc);
			return rc;
		}
	}
	eeprom_hw_driver_dump_data(data, size);
	pr_info("%s,%d: total size: %d, read size: %d\n", __func__, __LINE__,
		size, tmp_size);
	/* return read out size*/
	return size - tmp_size;
}
