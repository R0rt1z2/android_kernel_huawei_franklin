#include "eeprom_hw_i2c.h"

#define EEPROM_HW_I2C_MSG_LENGTH_MAX 2
#define EEPROM_I2C_MSG_SIZE_READ 2
#define EEPROM_I2C_MSG_SIZE_WRITE 1
#define EEPROM_I2C_READ_MSG_LENGTH_MAX 1024

static int eeprom_i2c_read_transfer(struct i2c_client *client,
	unsigned char *cmd, unsigned int cmd_len, unsigned char *data,
	unsigned int length)
{
	int rc;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	memset(msg, 0, sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = cmd_len;
	msg[0].buf = cmd;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = data;

	rc = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (rc < 0)
		pr_err("I2C read data failed!!\n");
	return rc;
}

static int eeprom_i2c_write_transfer(struct i2c_client *client,
	unsigned char *cmd, unsigned int length)
{
	int rc;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = length;
	msg.buf = cmd;

	rc = i2c_transfer(client->adapter, &msg,
				EEPROM_I2C_MSG_SIZE_WRITE);

	if (rc < 0)
		pr_err("I2C read data failed!!\n");
	return rc;
}

int eeprom_i2c_read_seq(struct i2c_client *client,
	unsigned int addr, unsigned char *data,
	unsigned int data_length,
	enum eeprom_i2c_addr_type addr_type)
{
	int rc;
	unsigned int cmd_len = 0;
	unsigned char buf[EEPROM_I2C_DATA_MAX] = {0};
	unsigned int read_size = data_length;
	unsigned int residue_size = data_length;
	unsigned int cur_addr = addr;
	unsigned char *read_buf = data;

	pr_debug("Enter\n");
	if (!client || !data) {
		pr_err("%s %d: client is NULL\n", __func__, __LINE__);
		return -EFAULT;
	}

	if ((addr_type != EEPROM_I2C_BYTE_ADDR &&
	     addr_type != EEPROM_I2C_WORD_ADDR)) {
		pr_err("Invalid addr_type=%d\n",
			addr_type);
		return -EFAULT;
	}

	if (data_length == 0) {
		pr_err("Invalid data_length=%d\n", data_length);
		return -EFAULT;
	}

	do {
		cmd_len = 0;
		if (addr_type == EEPROM_I2C_BYTE_ADDR) {
			buf[cmd_len++] = (unsigned char)cur_addr;
		} else {
			buf[cmd_len++] = (unsigned char)(cur_addr >> 8);
			buf[cmd_len++] = (unsigned char)(cur_addr & 0xFF);
		}
		read_size = (residue_size >= EEPROM_I2C_READ_MSG_LENGTH_MAX)
			? EEPROM_I2C_READ_MSG_LENGTH_MAX : residue_size;

		rc = eeprom_i2c_read_transfer(client, buf, cmd_len, read_buf,
			read_size);
		if (rc < 0) {
			pr_err("I2C read failed-addr:0x%x, data_length:%d\n",
				addr, data_length);
			return rc;
		}

		residue_size -= read_size;
		cur_addr += read_size;
		read_buf += read_size;
	} while (residue_size > 0);


	pr_debug("addr: 0x%x, data_length: %d\n", addr, data_length);
	pr_debug("Exit\n");
	return 0;
}

int eeprom_i2c_read(struct i2c_client *client,
	unsigned int addr, unsigned char *data,
	enum eeprom_i2c_addr_type addr_type,
	enum eeprom_i2c_data_type data_type)
{
	int rc;
	int len = 0;
	unsigned char buf[EEPROM_I2C_DATA_MAX] = {0};

	pr_debug("Enter\n");
	if (!client) {
		pr_err("%s %d: client is NULL\n", __func__, __LINE__);
		return -EFAULT;
	}

	if ((addr_type != EEPROM_I2C_BYTE_ADDR &&
			addr_type != EEPROM_I2C_WORD_ADDR) ||
			(data_type != EEPROM_I2C_BYTE_DATA &&
			data_type != EEPROM_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return -EFAULT;
	}

	if (addr_type == EEPROM_I2C_BYTE_ADDR) {
		buf[len++] = (unsigned char)addr;
	} else {
		buf[len++] = (unsigned char)(addr >> 8);
		buf[len++] = (unsigned char)(addr & 0xFF);
	}

	rc = eeprom_i2c_read_transfer(client, buf, len, buf, data_type);
	if (rc < 0) {
		pr_err("I2C read data failed-addr:0x%x, rc: %d!!\n", addr, rc);
		return rc;
	}

	if (data_type == EEPROM_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	pr_debug("addr: 0x%x\n", addr);
	pr_debug("Exit\n");
	return 0;
}

int eeprom_i2c_write(struct i2c_client *client,
				unsigned  int addr, unsigned short data,
				enum eeprom_i2c_addr_type addr_type,
				enum eeprom_i2c_data_type data_type)
{
	int rc;
	int len = 0;
	unsigned char send_cmd[EEPROM_I2C_ADDR_MAX + EEPROM_I2C_DATA_MAX] = {
		0 };

	pr_debug("Enter\n");

	if (!client) {
		pr_err("Fatal, Null ptr\n");
		return -EFAULT;
	}

	if ((addr_type != EEPROM_I2C_BYTE_ADDR &&
	     addr_type != EEPROM_I2C_WORD_ADDR) ||
	    (data_type != EEPROM_I2C_BYTE_DATA &&
	     data_type != EEPROM_I2C_WORD_DATA)) {
		pr_err("Invalid addr_type=%d or data_type=%d\n",
			addr_type, data_type);
		return -EFAULT;
	}

	if (addr_type == EEPROM_I2C_BYTE_ADDR)
		send_cmd[len++] = (u8)addr;
	else {
		send_cmd[len++] = (u8)(addr >> 8);
		send_cmd[len++] = (u8)(addr & 0xFF);
	}

	if (data_type == EEPROM_I2C_BYTE_DATA)
		send_cmd[len++] = (u8)data;
	else {
		send_cmd[len++] = (u8)(data >> 8);
		send_cmd[len++] = (u8)(data & 0xFF);
	}

	rc = eeprom_i2c_write_transfer(client, send_cmd, len);
	if (rc < 0) {
		pr_err("i2c write failed. addr: 0x%x data:0x%x\n",
			addr, data);
		return -EFAULT;
	}
	pr_debug("Exit\n");

	return 0;
}

