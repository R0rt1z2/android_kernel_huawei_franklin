/*
 * camkit_sensor_i2c.c
 *
 * Copyright (c) 2018-2020 Huawei Technologies Co., Ltd.
 *
 * i2c interface for image sensor
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include "camkit_sensor_i2c.h"

#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/string.h>
#include "kd_camera_typedef.h"
#include <linux/slab.h>
#include <securec.h>

#include "imgsensor_common.h"
#include "imgsensor_i2c.h"
#include "imgsensor_legacy.h"

#define PFX "[camkit_i2c]"
#define DEBUG_SENSOR_KIT 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_SENSOR_KIT) \
			pr_info(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

static uint16 calc_cmd_len(camkit_i2c_addr_type addr_type,
	camkit_i2c_data_type data_type)
{
	uint16 len_per_cycle = 0;
	if (addr_type == CAMKIT_I2C_BYTE_ADDR)
		len_per_cycle += 1;
	else
		len_per_cycle += 2;  // 1word=2bytes

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		len_per_cycle += 1;
	else
		len_per_cycle += 2;  // 1word=2bytes

	return len_per_cycle;
}

int32 camkit_sensor_i2c_read(struct camkit_sensor_ctrl_t *sensor,
				uint32 addr, uint16 *data,
				camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 buf[CAMKIT_I2C_DATA_MAX] = { 0 };
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };

	LOG_DBG("Enter\n");

	if (!data || !sensor) {
		pr_err("Fatal, Null ptr. data:%p, sensor:%p\n", data, sensor);
		return -EFAULT;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return -EFAULT;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR)
		pu_send_cmd[len++] = (uint8)addr;
	else {
		pu_send_cmd[len++] = (uint8)(addr >> 8);
		pu_send_cmd[len++] = (uint8)(addr & 0xFF);
	}

	rc = iReadRegI2C(pu_send_cmd, len, buf,
		data_type, sensor->i2c_write_id);
	if (rc < 0) {
		pr_err("i2c read failed. addr: 0x%x\n", addr);
		return -EFAULT;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	LOG_DBG("addr: 0x%x, data: 0x%x\n", addr, *data);
	LOG_DBG("Exit\n");
	return 0;
}

int32 camkit_i2c_read(uint8 i2c_addr, uint32 addr,
	camkit_i2c_addr_type addr_type, uint16 *data,
	camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 buf[CAMKIT_I2C_DATA_MAX] = { 0 };
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };

	LOG_DBG("Enter\n");

	if (!data || !i2c_addr) {
		pr_err("Fatal, Null ptr. data:%p, i2c_addr:%u\n", data, i2c_addr);
		return -EFAULT;
	}

	if ((addr_type != CAMKIT_I2C_BYTE_ADDR &&
		addr_type != CAMKIT_I2C_WORD_ADDR) ||
		(data_type != CAMKIT_I2C_BYTE_DATA &&
		data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return -EFAULT;
	}

	if (addr_type == CAMKIT_I2C_BYTE_ADDR)
		pu_send_cmd[len++] = (uint8)addr;
	else {
		pu_send_cmd[len++] = (uint8)(addr >> 8);
		pu_send_cmd[len++] = (uint8)(addr & 0xFF);
	}

	rc = iReadRegI2C(pu_send_cmd, len, buf, data_type, i2c_addr);
	if (rc < 0) {
		pr_err("i2c read failed. addr: 0x%x\n", addr);
		return -EFAULT;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	LOG_DBG("addr: 0x%x, data: 0x%x\n", addr, *data);
	LOG_DBG("Exit\n");
	return 0;
}

int32 camkit_eeprom_block_read(uint16 i2c_addr,
	uint16 addr, uint32 size, uint8 *data, u16 i2c_speed)
{
	uint8 buf[CAMKIT_I2C_ADDR_MAX] = {0};
	uint16 read_size;
	long long residue_size = size;
	uint16 cur_addr = addr;
	uint8 *read_buf = data;

	if (!data || i2c_addr == 0) {
		LOG_ERR("Fatal, Null ptr. data:%p, i2c_addr:%u\n", data, i2c_addr);
		return -EFAULT;
	}

	do {
		/* WORD_ADDR, buf len is 2 */
		buf[0] = (uint8)(cur_addr >> 8);
		buf[1] = (uint8)(cur_addr & 0xFF);
		read_size = (residue_size >= IMGSENSOR_I2C_CMD_LENGTH_MAX) ?
			IMGSENSOR_I2C_CMD_LENGTH_MAX : residue_size;
		/* buf len is 2 */
		if (iReadRegI2CTiming(buf, 2, read_buf, read_size,
			i2c_addr, i2c_speed) < 0) {
			LOG_ERR("read i2c failed. addr: 0x%x\n", addr);
			return -EIO;
		}
		residue_size -= read_size;
		cur_addr += read_size;
		read_buf += read_size;
	} while (residue_size > 0);

	LOG_DBG("addr: 0x%x, data_length: %u\n", addr, size);

	return 0;
}

int32 camkit_read_eeprom(uint8 i2c_addr, uint16 addr, uint8 *data)
{
	int32 rc;
	int32 len = 0;
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };
	LOG_DBG("Enter\n");

	if (!data || !i2c_addr) {
		pr_err("Fatal, Null ptr. data:%p, i2c_addr:%u\n", data, i2c_addr);
		return -EFAULT;
	}

	pu_send_cmd[len++] = (uint8)(addr >> 8);
	pu_send_cmd[len++] = (uint8)(addr & 0xFF);

	rc = iReadRegI2C(pu_send_cmd, len, data, CAMKIT_I2C_BYTE_DATA, i2c_addr);
	if (rc < 0) {
		pr_err("i2c read failed. addr: 0x%x\n", addr);
		return -EIO;
	}

	LOG_DBG("read register: [0x%x, 0x%x]\n", addr, *data);
	LOG_DBG("Exit\n");

	return 0;
}

int32 camkit_sensor_i2c_write(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX + CAMKIT_I2C_DATA_MAX] = { 0 };

	LOG_DBG("Enter\n");

	if (!sensor) {
		pr_err("Fatal, Null ptr\n");
		return -EFAULT;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return -EFAULT;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR)
		pu_send_cmd[len++] = (u8)addr;
	else {
		pu_send_cmd[len++] = (u8)(addr >> 8);
		pu_send_cmd[len++] = (u8)(addr & 0xFF);
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		pu_send_cmd[len++] = (u8)data;
	else {
		pu_send_cmd[len++] = (u8)(data >> 8);
		pu_send_cmd[len++] = (u8)(data & 0xFF);
	}

	rc = iWriteRegI2C(pu_send_cmd, len, sensor->i2c_write_id);
	if (rc < 0) {
		pr_err("i2c write failed. addr: 0x%x data:0x%x\n",
			addr, data);
		return -EFAULT;
	}
	LOG_DBG("Exit\n");

	return 0;
}

int32 camkit_sensor_i2c_poll(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	camkit_i2c_data_type data_type,
	uint16 delay)
{
	int32 rc;
	int32 i;
	uint16 temp_data = 0;

	LOG_DBG("Enter\n");
	if (!sensor) {
		pr_err("Fatal, Null ptr\n");
		return -EFAULT;
	}

	for (i = 0; i < delay; i++) {
		rc = camkit_sensor_i2c_read(sensor, addr,
			&temp_data, data_type);
		LOG_DBG("i=%d, addr: 0x%x, data: 0x%x, delay:%d\n",
			i, addr, temp_data, delay);
		if (rc < 0) {
			pr_err("failed to read addr: 0x%x\n", addr);
			return -EFAULT;
		}
		if (temp_data == data) {
			LOG_INF("poll sucess\n");
			return 0;
		}
		usleep_range(1000, 1100);
	}

	LOG_DBG("poll failed\n");
	return -EFAULT;
}

int32 camkit_sensor_write_table(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting,
	uint32 size,
	camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *pu_send_cmd = NULL;
	uint32 tosend = 0;
	uint32 i = 0;
	uint16 addr = 0;
	uint16 data = 0;
	uint16 delay = 0;
	uint16 len_per_cycle = 0;

	LOG_DBG("Enter. size:%d\n", size);

	if (!setting || !sensor) {
		pr_err("Fatal.Null ptr. setting:%p, sensor:%p\n",
			setting, sensor);
		return -EFAULT;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return -EFAULT;
	}

	len_per_cycle = calc_cmd_len(sensor->addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		addr = setting[i].addr;
		data = setting[i].data;
		delay = setting[i].delay;

		if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
			pu_send_cmd[tosend++] = (uint8)addr;
		} else {
			pu_send_cmd[tosend++] = (uint8)(addr >> 8);
			pu_send_cmd[tosend++] = (uint8)(addr & 0xFF);
		}

		if (data_type == CAMKIT_I2C_BYTE_DATA) {
			pu_send_cmd[tosend++] = (uint8)data;
		} else {
			pu_send_cmd[tosend++] = (uint8)(data >> 8);
			pu_send_cmd[tosend++] = (uint8)(data & 0xFF);
		}

		if ((len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || delay > 0 || (i + 1) == size) {
			LOG_DBG("begin send: tosend:%d, delay:%d i:%d size:%d\n",
				tosend, delay, i, size);
			rc = iBurstWriteReg_multi(pu_send_cmd, tosend,
						sensor->i2c_write_id,
						len_per_cycle,
						sensor->i2c_speed);
			if (rc < 0) {
				pr_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return -EIO;
			}
			if (delay > 0)
				mdelay(delay);
			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	LOG_DBG("Exit. size:%d\n", size);

	return 0;
}

int32 camkit_i2c_write_table(struct camkit_sensor_ctrl_t *sensor,
	uint16 *setting,
	uint32 size,
	camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *pu_send_cmd = NULL;
	uint32 tosend = 0;
	uint32 i;
	uint16 addr;
	uint16 data;
	uint16 len_per_cycle = 0;

	LOG_DBG("Enter. size:%d\n", size);

	if (!setting || !sensor) {
		pr_err("Fatal.Null ptr. setting:%p, sensor:%p\n",
			setting, sensor);
		return -EFAULT;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return -EFAULT;
	}

	len_per_cycle = calc_cmd_len(sensor->addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return -ENOMEM;

	for (i = 0; i < size; i += 2) {
		addr = setting[i];
		data = setting[i + 1];
		LOG_DBG("write register: [0x%x, 0x%x]\n", addr, data);
		if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
			pu_send_cmd[tosend++] = (uint8)addr;
		} else {
			pu_send_cmd[tosend++] = (uint8)(addr >> 8);
			pu_send_cmd[tosend++] = (uint8)(addr & 0xFF);
		}

		if (data_type == CAMKIT_I2C_BYTE_DATA) {
			pu_send_cmd[tosend++] = (uint8)data;
		} else {
			pu_send_cmd[tosend++] = (uint8)(data >> 8);
			pu_send_cmd[tosend++] = (uint8)(data & 0xFF);
		}

		if ((len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || (i + 2) == size) {
			LOG_INF("begin send: tosend:%d, i:%d size:%d\n",
				tosend, i, size);
			rc = iBurstWriteReg_multi(pu_send_cmd, tosend,
						sensor->i2c_write_id,
						len_per_cycle,
						sensor->i2c_speed);
			if (rc < 0) {
				pr_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return -EFAULT;
			}

			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	LOG_DBG("Exit. size:%d\n", size);

	return 0;
}

int32 camkit_sensor_write_setting(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_setting *settings)
{
	int32 rc;

	if (!sensor || !settings) {
		pr_err("Fatal. Null ptr. sensor:%p, settings:%p\n",
			sensor, settings);
		return -EFAULT;
	}

	rc = camkit_sensor_write_table(sensor, settings->setting,
		settings->size, settings->data_type);
	if (rc < 0) {
		pr_err("Failed\n");
		return rc;
	}
	if (settings->delay > 0)
		msleep(settings->delay);

	return 0;
}

int32 camkit_sensor_i2c_process(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_table_array *settings, uint8 *data_buf, int data_size)
{
	int32 rc = 0;
	int32 i;
	camkit_i2c_operation i2c_operation;
	struct camkit_i2c_reg_table *setting = NULL;
	uint16 temp_data = 0;
	int data_index = 0;

	if (!sensor || !settings) {
		pr_err("Fatal. Null ptr. sensor:%p, settings:%p\n",
			sensor, settings);
		return -EFAULT;
	}

	setting = settings->setting;
	for (i = 0; i < settings->size; i++) {
		i2c_operation = setting[i].i2c_operation;
		switch (i2c_operation) {
		case CAMKIT_I2C_READ:
			rc = camkit_sensor_i2c_read(sensor,
					setting[i].addr,
					&temp_data,
					setting[i].data_type);
			if (rc < 0)
				pr_err("Read setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, temp_data);
			else // donnot decrease the print level used to dump reg
				pr_info("Read-addr:0x%x, data:0x%x\n",
					setting[i].addr, temp_data);
			break;
		case CAMKIT_I2C_WRITE:
			rc = camkit_sensor_i2c_write(sensor, setting[i].addr,
						setting[i].data,
						setting[i].data_type);
			if (rc < 0)
				pr_err("Write setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, setting[i].data);
			else // donnot decrease the print level used to dump reg
				pr_info("Write-addr:0x%x, data:0x%x\n",
					setting[i].addr, setting[i].data);
			break;
		case CAMKIT_I2C_POLL:
			rc = camkit_sensor_i2c_poll(sensor, setting[i].addr,
				setting[i].data, setting[i].data_type,
				setting[i].delay);
			if (rc < 0)
				pr_err("Poll setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, setting[i].data);
			else // donnot decrease the print level used to dump reg
				pr_info("Poll-addr:0x%x, data:0x%x\n",
					setting[i].addr, setting[i].data);
			break;
		case CAMKIT_I2C_READ_BUFFER:
			if (!data_buf || (data_index + setting[i].data_type) > data_size) {
				pr_err("Null ptr or invalid index: %d data_size:%d\n", data_index, data_size);
				break;
			}
			temp_data = 0;
			rc = camkit_sensor_i2c_read(sensor,
					setting[i].addr,
					&temp_data,
					setting[i].data_type);
			if (rc < 0) {
				pr_err("Read setting[%d].addr:0x%x, data_buf:0x%x failed\n",
					i, setting[i].addr, data_buf[data_index]);
				break;
			}
			if (setting[i].data_type == CAMKIT_I2C_BYTE_DATA) {
				data_buf[data_index] = (uint8)temp_data & 0xFF;
			} else if (setting[i].data_type == CAMKIT_I2C_WORD_DATA) {
				*((uint16 *)&data_buf[data_index]) = temp_data;
			}
			data_index += setting[i].data_type;

			pr_debug("Read-addr:0x%x, data_buf:0x%x\n",
					setting[i].addr, data_buf[data_index]);
			break;
		default:
			pr_err("Invalid i2c_operation\n");
			break;
		}

		if (setting[i].delay > 0 && i2c_operation != CAMKIT_I2C_POLL)
			mdelay(setting[i].delay);
	}

	return rc;
}

// Note: Register address must be continuous
int32 camkit_i2c_write_block(uint8 i2c_addr, uint32 i2c_speed,
	uint32 start_addr, camkit_i2c_addr_type addr_type,
	uint8 *data, uint32 data_len)
{
	int32 rc;
	uint8 *i2c_buf = NULL;
	uint32 tosend = 0;
	uint32 buf_len;

	LOG_DBG("Enter. start block write:%d\n", data_len);

	if (!data || !data_len) {
		pr_err("Fatal.Null ptr\n");
		return -EFAULT;
	}

	if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
		buf_len = data_len + 1;
	} else if (addr_type == CAMKIT_I2C_WORD_ADDR) {
		buf_len = data_len + 2;
	} else {
		pr_err("unsupported addr_type=%d\n", addr_type);
		return -EFAULT;
	}

	if (buf_len > PAGE_SIZE) {
		pr_err("buffer too long, buffer len:%d\n", buf_len);
		return -EFAULT;
	}

	i2c_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!i2c_buf)
		return -ENOMEM;

	if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
		i2c_buf[tosend++] = (uint8)start_addr;
	} else {
		i2c_buf[tosend++] = (uint8)(start_addr >> 8);
		i2c_buf[tosend++] = (uint8)(start_addr & 0xFF);
	}

	rc = memcpy_s((void *)&i2c_buf[tosend], buf_len - tosend, data, data_len);
	if (rc != EOK) {
		pr_err("memcpy_s fail\n");
		kfree(i2c_buf);
		return -EFAULT;
	}

	rc = iBurstWriteReg_multi(i2c_buf, buf_len, i2c_addr, buf_len, i2c_speed);
	if (rc < 0) {
		pr_err("Burst i2c write failed: %d\n", buf_len);
		kfree(i2c_buf);
		return -EIO;
	}

	kfree(i2c_buf);

	LOG_INF("Exit. writed size:%d\n", data_len + tosend);

	return 0;
}

// Note: Register address must be continuous
int32 camkit_sensor_write_block(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting, uint32 size, camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *i2c_buf = NULL;
	uint32 tosend = 0;
	uint32 buf_len;
	uint32 i;
	uint16 data = 0;

	if (!sensor || !setting) {
		pr_err("Fatal.Null ptr\n");
		return -EFAULT;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA) {
		buf_len = size;
	} else if (data_type == CAMKIT_I2C_WORD_DATA) {
		buf_len = size * 2; // word data is 2 bytes
	} else {
		pr_err("unsupported data_type=%d\n", data_type);
		return -EFAULT;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
		buf_len += 1; // byte addr is 1 bytes
	} else if (sensor->addr_type == CAMKIT_I2C_WORD_ADDR) {
		buf_len += 2; // word addr is 2 bytes
	} else {
		pr_err("Invalid addr_type=%d\n", sensor->addr_type);
		return -EFAULT;
	}

	if (buf_len > PAGE_SIZE) {
		pr_err("buffer too long, buffer len:%d\n", buf_len);
		return -EFAULT;
	}

	i2c_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!i2c_buf)
		return -ENOMEM;

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
		i2c_buf[tosend++] = (uint8)(setting->addr);
	} else {
		i2c_buf[tosend++] = (uint8)(setting->addr >> 8);
		i2c_buf[tosend++] = (uint8)(setting->addr & 0xFF);
	}

	for (i = 0; i < size; i++) {
		data = setting[i].data;

		if (data_type == CAMKIT_I2C_BYTE_DATA) {
			i2c_buf[tosend++] = (uint8)data;
		} else {
			i2c_buf[tosend++] = (uint8)(data >> 8);
			i2c_buf[tosend++] = (uint8)(data & 0xFF);
		}
	}

	rc = iBurstWriteReg_multi(i2c_buf, buf_len, sensor->i2c_write_id,
		buf_len, sensor->i2c_speed);
	if (rc < 0) {
		pr_err("Burst i2c write failed: %d\n", buf_len);
		kfree(i2c_buf);
		return -EFAULT;
	}

	kfree(i2c_buf);

	LOG_INF("Exit. writed size:%d, tosend:%d\n", buf_len, tosend);

	return 0;
}

int32 camkit_i2c_write_setting(struct IMGSENSOR_I2C_CFG *i2c_cfg,
	uint32 i2c_speed, uint8 i2c_write_id,
	struct camkit_i2c_reg_setting *settings)
{
	int32 rc;
	uint8 *pu_send_cmd = NULL;
	uint32 tosend = 0;
	uint32 i;
	uint16 addr;
	uint16 data;
	uint32 size;
	uint16 delay;
	uint16 len_per_cycle;
	camkit_i2c_data_type data_type;
	camkit_i2c_addr_type addr_type;

	struct camkit_i2c_reg *setting = NULL;

	LOG_DBG("Enter. i2c_speed:%u i2c_write_id:%u\n", i2c_speed, i2c_write_id);

	if (!settings || !i2c_cfg) {
		LOG_ERR("Fatal. Null ptr settings %p or i2c_cfg %p \n",
			settings, i2c_cfg);
		return -EFAULT;
	}

	data_type = settings->data_type;
	addr_type = settings->addr_type;
	setting = settings->setting;
	size = settings->size;

	if ((addr_type != CAMKIT_I2C_BYTE_ADDR &&
		addr_type != CAMKIT_I2C_WORD_ADDR) ||
		(data_type != CAMKIT_I2C_BYTE_DATA &&
		data_type != CAMKIT_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return -EFAULT;
	}

	len_per_cycle = calc_cmd_len(addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		addr = setting[i].addr;
		data = setting[i].data;
		delay = setting[i].delay;
		LOG_DBG("write register: [0x%x, 0x%x]\n", addr, data);
		if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
			pu_send_cmd[tosend++] = (uint8)addr;
		} else {
			pu_send_cmd[tosend++] = (uint8)(addr >> 8);
			pu_send_cmd[tosend++] = (uint8)(addr & 0xFF);
		}

		if (data_type == CAMKIT_I2C_BYTE_DATA) {
			pu_send_cmd[tosend++] = (uint8)data;
		} else {
			pu_send_cmd[tosend++] = (uint8)(data >> 8);
			pu_send_cmd[tosend++] = (uint8)(data & 0xFF);
		}

		if ((len_per_cycle * IMGSENSOR_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || delay > 0 || (i + 1) == size) {
			LOG_DBG("begin send: tosend:%d, delay:%d i:%d size:%d\n",
				tosend, delay, i, size);
			rc = burst_write_reg_multi(i2c_cfg, pu_send_cmd, tosend,
						i2c_write_id,
						len_per_cycle,
						i2c_speed);
			if (rc < 0) {
				pr_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return -EFAULT;
			}
			if (delay > 0)
				mdelay(delay);

			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	LOG_DBG("Exit. size:%u\n", size);

	return 0;
}
