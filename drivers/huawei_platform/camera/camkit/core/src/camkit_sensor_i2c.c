/*
 * camkit_sensor_i2c.c
 *
 * Copyright (c) huawei technologies co., ltd. 2020-2020. all rights reserved.
 *
 * Description: i2c interface for image sensor
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

#include <linux/delay.h>
#include <linux/slab.h>
#include <securec.h>

#include "camkit_driver_types.h"
#include "camkit_io_adapter.h"

static uint16 calc_cmd_len(enum camkit_i2c_addr_type addr_type,
	enum camkit_i2c_data_type data_type)
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
	uint32 addr, uint16 *data, enum camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 buf[CAMKIT_I2C_DATA_MAX] = { 0 };
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };

	i2c_dbg("Enter\n");

	if (!data || !sensor) {
		log_err("Fatal, Null ptr. data:%p, sensor:%p\n", data, sensor);
		return ERR_INVAL;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return ERR_INVAL;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
		pu_send_cmd[len++] = (uint8)addr;
	} else {
		pu_send_cmd[len++] = (uint8)(addr >> 8);
		pu_send_cmd[len++] = (uint8)(addr & 0xFF);
	}

	rc = adopt_i2c_read(pu_send_cmd, len, buf,
		data_type, sensor->i2c_write_id);
	if (rc < 0) {
		log_err("i2c read failed. addr: 0x%x\n", addr);
		return ERR_IO;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = (buf[0] << 8) | buf[1];

	i2c_dbg("addr: 0x%x, data: 0x%x\n", addr, *data);
	i2c_dbg("Exit\n");
	return 0;
}

int32 camkit_i2c_read(uint8 i2c_addr, uint32 addr,
	enum camkit_i2c_addr_type addr_type, uint16 *data,
	enum camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 buf[CAMKIT_I2C_DATA_MAX] = { 0 };
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };

	i2c_dbg("Enter\n");

	if (!data || !i2c_addr) {
		log_err("Fatal, Null ptr. data:%p, i2c_addr:%u\n", data, i2c_addr);
		return ERR_INVAL;
	}

	if ((addr_type != CAMKIT_I2C_BYTE_ADDR &&
		addr_type != CAMKIT_I2C_WORD_ADDR) ||
		(data_type != CAMKIT_I2C_BYTE_DATA &&
		data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return ERR_INVAL;
	}

	if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
		pu_send_cmd[len++] = (uint8)addr;
	} else {
		pu_send_cmd[len++] = (uint8)(addr >> 8);
		pu_send_cmd[len++] = (uint8)(addr & 0xFF);
	}

	rc = adopt_i2c_read(pu_send_cmd, len, buf, data_type, i2c_addr);
	if (rc < 0) {
		log_err("i2c read failed. addr: 0x%x\n", addr);
		return ERR_IO;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = (buf[0] << 8) | buf[1];

	i2c_dbg("addr: 0x%x, data: 0x%x\n", addr, *data);
	i2c_dbg("Exit\n");
	return 0;
}

int32 camkit_read_eeprom(uint8 i2c_addr, uint16 addr, uint8 *data)
{
	int32 rc;
	int32 len = 0;
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX] = { 0 };

	i2c_dbg("Enter\n");

	if (!data || !i2c_addr) {
		log_err("Fatal, Null ptr. data:%p, i2c_addr:%u\n", data, i2c_addr);
		return ERR_INVAL;
	}

	pu_send_cmd[len++] = (uint8)(addr >> 8);
	pu_send_cmd[len++] = (uint8)(addr & 0xFF);

	rc = adopt_i2c_read(pu_send_cmd, len, data,
		CAMKIT_I2C_BYTE_DATA, i2c_addr);
	if (rc < 0) {
		log_err("i2c read failed. addr: 0x%x\n", addr);
		return ERR_IO;
	}

	i2c_dbg("read register: [0x%x, 0x%x]\n", addr, *data);
	i2c_dbg("Exit\n");

	return 0;
}

int32 camkit_sensor_i2c_write(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	enum camkit_i2c_data_type data_type)
{
	int32 rc;
	int32 len = 0;
	uint8 pu_send_cmd[CAMKIT_I2C_ADDR_MAX + CAMKIT_I2C_DATA_MAX] = { 0 };

	i2c_dbg("Enter\n");

	if (!sensor) {
		log_err("Fatal, Null ptr\n");
		return ERR_INVAL;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return ERR_INVAL;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
		pu_send_cmd[len++] = (u8)addr;
	} else {
		pu_send_cmd[len++] = (u8)(addr >> 8);
		pu_send_cmd[len++] = (u8)(addr & 0xFF);
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA) {
		pu_send_cmd[len++] = (u8)data;
	} else {
		pu_send_cmd[len++] = (u8)(data >> 8);
		pu_send_cmd[len++] = (u8)(data & 0xFF);
	}

	rc = adopt_i2c_write(pu_send_cmd, len, sensor->i2c_write_id);
	if (rc < 0) {
		log_err("i2c write failed. addr: 0x%x data:0x%x\n",
			addr, data);
		return ERR_IO;
	}
	i2c_dbg("Exit\n");

	return 0;
}

int32 camkit_sensor_i2c_poll(struct camkit_sensor_ctrl_t *sensor,
	uint32 addr, uint16 data,
	enum camkit_i2c_data_type data_type,
	uint16 delay)
{
	int32 rc;
	int32 i;
	uint16 temp_data = 0;

	i2c_dbg("Enter\n");
	if (!sensor) {
		log_err("Fatal, Null ptr\n");
		return ERR_INVAL;
	}

	for (i = 0; i < delay; i++) {
		rc = camkit_sensor_i2c_read(sensor, addr,
			&temp_data, data_type);
		i2c_dbg("i=%d, addr: 0x%x, data: 0x%x, delay:%d\n",
			i, addr, temp_data, delay);
		if (rc != ERR_NONE) {
			log_err("failed to read addr: 0x%x\n", addr);
			return ERR_IO;
		}
		if (temp_data == data) {
			i2c_dbg("poll sucess\n");
			return 0;
		}
		usleep_range(1000, 1100);
	}

	i2c_dbg("poll failed\n");
	return ERR_IO;
}

int32 camkit_sensor_write_table(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting,
	uint32 size,
	enum camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *pu_send_cmd = NULL;
	uint32 tosend = 0;
	uint32 i;
	uint16 addr = 0;
	uint16 data = 0;
	uint16 delay = 0;
	uint16 len_per_cycle;

	i2c_dbg("Enter. size:%d\n", size);

	if (!setting || !sensor) {
		log_err("Fatal.Null ptr. setting:%p, sensor:%p\n",
			setting, sensor);
		return ERR_INVAL;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return ERR_INVAL;
	}

	len_per_cycle = calc_cmd_len(sensor->addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return ERR_NOMEM;

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

		if ((len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || delay > 0 || (i + 1) == size) {
			i2c_dbg("begin send: tosend:%d, delay:%d i:%d size:%d\n",
				tosend, delay, i, size);
			rc = adopt_i2c_burst_write(pu_send_cmd, tosend,
						sensor->i2c_write_id,
						len_per_cycle,
						sensor->i2c_speed);
			if (rc < 0) {
				log_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return ERR_IO;
			}
			if (delay > 0)
				mdelay(delay);
			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	i2c_dbg("Exit. size:%d\n", size);

	return ERR_NONE;
}

int32 camkit_i2c_write_table(struct camkit_sensor_ctrl_t *sensor,
	uint16 *setting,
	uint32 size,
	enum camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *pu_send_cmd = NULL;
	uint32 tosend = 0;
	uint32 i;
	uint16 addr;
	uint16 data;
	uint16 len_per_cycle;

	i2c_dbg("Enter. size:%d\n", size);

	if (!setting || !sensor) {
		log_err("Fatal.Null ptr. setting:%p, sensor:%p\n",
			setting, sensor);
		return ERR_INVAL;
	}

	if ((sensor->addr_type != CAMKIT_I2C_BYTE_ADDR &&
	     sensor->addr_type != CAMKIT_I2C_WORD_ADDR) ||
	    (data_type != CAMKIT_I2C_BYTE_DATA &&
	     data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			sensor->addr_type, data_type);
		return ERR_INVAL;
	}

	len_per_cycle = calc_cmd_len(sensor->addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return ERR_NOMEM;

	for (i = 0; i < size; i += 2) {
		addr = setting[i];
		data = setting[i + 1];
		i2c_dbg("write register: [0x%x, 0x%x]\n", addr, data);
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

		if ((len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || (i + 2) == size) {
			log_info("begin send: tosend:%d, i:%d size:%d\n",
				tosend, i, size);
			rc = adopt_i2c_burst_write(pu_send_cmd, tosend,
						sensor->i2c_write_id,
						len_per_cycle,
						sensor->i2c_speed);
			if (rc < 0) {
				log_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return ERR_IO;
			}

			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	i2c_dbg("Exit. size:%d\n", size);

	return ERR_NONE;
}

int32 camkit_sensor_write_setting(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_setting *settings)
{
	int32 rc;

	if (!sensor || !settings) {
		log_err("Fatal. Null ptr. sensor:%p, settings:%p\n",
			sensor, settings);
		return ERR_INVAL;
	}

	rc = camkit_sensor_write_table(sensor, settings->setting,
		settings->size, settings->data_type);
	if (rc != ERR_NONE) {
		log_err("Failed\n");
		return ERR_IO;
	}
	if (settings->delay > 0)
		mdelay(settings->delay);

	return ERR_NONE;
}

int32 camkit_sensor_i2c_process(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg_table_array *settings)
{
	int32 rc;
	int32 i;
	enum camkit_i2c_operation i2c_operation;
	struct camkit_i2c_reg_table *setting = NULL;
	uint16 temp_data = 0;

	if (!sensor || !settings) {
		log_err("Fatal. Null ptr. sensor:%p, settings:%p\n",
			sensor, settings);
		return ERR_INVAL;
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
			if (rc != ERR_NONE)
				log_err("Read setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, temp_data);
			else // donnot decrease the print level used to dump reg
				log_info("Read-addr:0x%x, data:0x%x\n",
					setting[i].addr, temp_data);
			break;
		case CAMKIT_I2C_WRITE:
			rc = camkit_sensor_i2c_write(sensor, setting[i].addr,
						setting[i].data,
						setting[i].data_type);
			if (rc != ERR_NONE)
				log_err("Write setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, setting[i].data);
			else // donnot decrease the print level used to dump reg
				log_info("Write-addr:0x%x, data:0x%x\n",
					setting[i].addr, setting[i].data);
			break;
		case CAMKIT_I2C_POLL:
			rc = camkit_sensor_i2c_poll(sensor, setting[i].addr,
				setting[i].data, setting[i].data_type,
				setting[i].delay);
			if (rc != ERR_NONE)
				log_err("Poll setting[%d].addr:0x%x, data:0x%x failed\n",
					i, setting[i].addr, setting[i].data);
			else // donnot decrease the print level used to dump reg
				log_info("Poll-addr:0x%x, data:0x%x\n",
					setting[i].addr, setting[i].data);
			break;
		default:
			log_err("Invalid i2c_operation\n");
			break;
		}

		if (setting[i].delay > 0 && i2c_operation != CAMKIT_I2C_POLL)
			mdelay(setting[i].delay);
	}

	return ERR_NONE;
}

// Note: Register address must be continuous
int32 camkit_i2c_write_block(uint8 i2c_addr, uint32 i2c_speed,
	uint32 start_addr, enum camkit_i2c_addr_type addr_type,
	uint8 *data, uint32 data_len)
{
	int32 rc;
	uint8 *i2c_buf = NULL;
	uint32 tosend = 0;
	uint32 buf_len;

	i2c_dbg("Enter. start block write:%d\n", data_len);

	if (!data || !data_len) {
		log_err("Fatal.Null ptr\n");
		return ERR_INVAL;
	}

	if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
		buf_len = data_len + 1;
	} else if (addr_type == CAMKIT_I2C_WORD_ADDR) {
		buf_len = data_len + 2;
	} else {
		log_err("unsupported addr_type=%d\n", addr_type);
		return -EFAULT;
	}

	if (buf_len > PAGE_SIZE) {
		log_err("buffer too long, buffer len:%d\n", buf_len);
		return ERR_INVAL;
	}

	i2c_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!i2c_buf)
		return ERR_NOMEM;

	if (addr_type == CAMKIT_I2C_BYTE_ADDR) {
		i2c_buf[tosend++] = (uint8)start_addr;
	} else {
		i2c_buf[tosend++] = (uint8)(start_addr >> 8);
		i2c_buf[tosend++] = (uint8)(start_addr & 0xFF);
	}

	rc = memcpy_s((void *)&i2c_buf[tosend], buf_len - tosend, data, data_len);
	if (rc != EOK) {
		log_err("memcpy_s fail\n");
		kfree(i2c_buf);
		return ERR_NOMEM;
	}

	rc = adopt_i2c_burst_write(i2c_buf, buf_len, i2c_addr, buf_len, i2c_speed);
	if (rc < 0) {
		log_err("Burst i2c write failed: %d\n", buf_len);
		kfree(i2c_buf);
		return ERR_IO;
	}

	kfree(i2c_buf);

	log_info("Exit. writed size:%d\n", data_len + tosend);

	return ERR_NONE;
}

// Note: Register address must be continuous
int32 camkit_sensor_write_block(struct camkit_sensor_ctrl_t *sensor,
	struct camkit_i2c_reg *setting, uint32 size,
	enum camkit_i2c_data_type data_type)
{
	int32 rc;
	uint8 *i2c_buf = NULL;
	uint32 tosend = 0;
	uint32 buf_len;
	uint32 i;
	uint16 data = 0;

	if (!sensor || !setting) {
		log_err("Fatal.Null ptr\n");
		return ERR_INVAL;
	}

	if (data_type == CAMKIT_I2C_BYTE_DATA) {
		buf_len = size;
	} else if (data_type == CAMKIT_I2C_WORD_DATA) {
		buf_len = size * 2; // word data is 2 bytes
	} else {
		log_err("unsupported data_type=%d\n", data_type);
		return ERR_INVAL;
	}

	if (sensor->addr_type == CAMKIT_I2C_BYTE_ADDR) {
		buf_len += 1; // byte addr is 1 bytes
	} else if (sensor->addr_type == CAMKIT_I2C_WORD_ADDR) {
		buf_len += 2; // word addr is 2 bytes
	} else {
		log_err("Invalid addr_type=%d\n", sensor->addr_type);
		return ERR_INVAL;
	}

	if (buf_len > PAGE_SIZE) {
		log_err("buffer too long, buffer len:%d\n", buf_len);
		return ERR_INVAL;
	}

	i2c_buf = kzalloc(buf_len, GFP_KERNEL);
	if (!i2c_buf)
		return ERR_NOMEM;

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

	rc = adopt_i2c_burst_write(i2c_buf, buf_len, sensor->i2c_write_id,
		buf_len, sensor->i2c_speed);
	if (rc < 0) {
		log_err("Burst i2c write failed: %d\n", buf_len);
		kfree(i2c_buf);
		return ERR_IO;
	}

	kfree(i2c_buf);

	log_info("Exit. writed size:%d, tosend:%d\n", buf_len, tosend);

	return ERR_NONE;
}

int32 camkit_i2c_write_setting(uint32 i2c_speed, uint8 i2c_write_id,
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
	enum camkit_i2c_data_type data_type;
	enum camkit_i2c_addr_type addr_type;
	struct camkit_i2c_reg *setting = NULL;

	i2c_dbg("Enter. i2c_speed:%u i2c_write_id:%u\n", i2c_speed, i2c_write_id);

	if (!settings) {
		log_err("Fatal. Null ptr settings:%p\n", settings);
		return ERR_INVAL;
	}

	data_type = settings->data_type;
	addr_type = settings->addr_type;
	setting = settings->setting;
	size = settings->size;

	if ((addr_type != CAMKIT_I2C_BYTE_ADDR &&
		addr_type != CAMKIT_I2C_WORD_ADDR) ||
		(data_type != CAMKIT_I2C_BYTE_DATA &&
		data_type != CAMKIT_I2C_WORD_DATA)) {
		log_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return ERR_INVAL;
	}

	len_per_cycle = calc_cmd_len(addr_type, data_type);
	pu_send_cmd = kzalloc(len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX,
		GFP_KERNEL);
	if (!pu_send_cmd)
		return ERR_NOMEM;

	for (i = 0; i < size; i++) {
		addr = setting[i].addr;
		data = setting[i].data;
		delay = setting[i].delay;
		i2c_dbg("write register: [0x%x, 0x%x]\n", addr, data);
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

		if ((len_per_cycle * CAMKIT_I2C_CMD_LENGTH_MAX - tosend) <
			len_per_cycle || delay > 0 || (i + 1) == size) {
			i2c_dbg("begin send: tosend:%d, delay:%d i:%d size:%d\n",
				tosend, delay, i, size);
			rc = adopt_i2c_burst_write(pu_send_cmd, tosend,
						i2c_write_id,
						len_per_cycle,
						i2c_speed);
			if (rc < 0) {
				log_err("write i2c failed. i=%d\n", i);
				kfree(pu_send_cmd);
				return ERR_IO;
			}
			if (delay > 0)
				mdelay(delay);

			tosend = 0;
		}
	}

	kfree(pu_send_cmd);

	i2c_dbg("Exit. size:%u\n", size);

	return ERR_NONE;
}
