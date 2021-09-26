/*
 * flashlights-mp3331.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "flashlight-core.h"
#include "flashlight-dt.h"
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "securec.h"

#define MP3331_NAME "flashlights-mp3331"
#ifndef MP3331_DTNAME
#define MP3331_DTNAME "mediatek,flashlights_mp3331"
#endif
#ifndef MP3331_DTNAME_I2C
#define MP3331_DTNAME_I2C "mediatek,flashlights_mp3331_i2c"
#endif

#define MP3331_REG_SILICON_REVISION      0x00
#define MP3331_REG_MODE_SET              0x01
#define MP3331_REG_FLASH_TIMER_SET       0x03
#define MP3331_REG_TORCH_CURRENT_CONTROL 0x0A
#define MP3331_REG_FLASH_CURRENT_CONTROL 0x06
#define MP3331_REG_FLASH_FAULT_H         0x0B
#define MP3331_REG_FLASH_FAULT_L         0x0C
#define MP3331_REG_LOW_VLO_SET           0x04
#define MP3331_FL_TIM                    0xF0 /* set Flash timer, 800ms */
#define MP3331_MAX_TIMEOUT               680 /* ms */
#define MP3331_SW_FS                     0x02
#define MP3331_STB_LV                    0x80
#define MP3331_FLASH_MODE_CUR            0x06 /* flash mode bit */
#define MP3331_TORCH_MODE_CUR            0x04 /* torch mode bit */
#define MP3331_LED1_EN                   0x10 /* LED1 enable bit */
#define MP3331_LEVEL_NUM                 18
#define MP3331_LEVEL_TORCH               4
#define MP3331_HW_TIMEOUT                800
#define MP3331_CHIP_ID                   0x18
#define MP3331_CHIP_ID_MASK              0xF8
#define MP3331_OVER_VOLTAGE_PROTECT      0x40
#define MP3331_VOUT_SHORT                0x20
#define MP3331_LED_SHORT                 0x10
#define MP3331_OVER_TEMP_PROTECT         0x08
#define MP3331_LED_OPEN                  0x01
#define MP3331_VBL_RUN                   0x40 /* 3.2V */
#define UVLO_MASK                        0x0F
#define TIMEOUT_MASK                     0x0C

static DEFINE_MUTEX(mp3331_mutex);
static struct work_struct mp3331_work;
static struct i2c_client *mp3331_i2c_client;

static struct flash_name_info_t g_name_info = {
	.ic_name = "MP3331",
	.module_name = "MPS",
};

static struct flash_error_info_t g_flash_error_info[] = {
	{
		.reg_addr = MP3331_REG_FLASH_FAULT_H,
		.error_table = {
			{
				MP3331_OVER_TEMP_PROTECT,
				REG_VAL_AND_MASK,
				DSM_FLASH_HOT_DIE_ERROR_NO,
				"flash temperature is too hot",
			}, {
				(MP3331_OVER_VOLTAGE_PROTECT |
				 MP3331_VOUT_SHORT | MP3331_LED_SHORT),
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash OVP, LED or VOUT short",
			},
		},
		.size = 2,
	}, {
		.reg_addr = MP3331_REG_FLASH_FAULT_L,
		.error_table = {
			{
				MP3331_LED_OPEN,
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash LED Open",
			},
		},
		.size = 1,
	}
};

/* platform data */
struct mp3331_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* mp3331 chip data */
struct mp3331_chip_data {
	struct i2c_client *client;
	struct mp3331_platform_data *pdata;
	struct mutex lock;
};

static struct hrtimer mp3331_timer;
static unsigned int mp3331_timeout_ms;

/* mp3331 operations */
static const int mp3331_current[MP3331_LEVEL_NUM] = {
	32,  95,  158,  221,  285,  380,  476,  571, 665, 761,
	856, 951, 1046, 1142, 1237, 1331, 1426, 1522
};

static const unsigned char mp3331_flash_level[MP3331_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0C, 0x0F, 0x12, 0x15, 0x18,
	0x1B, 0x1E, 0x21, 0x24, 0x27, 0x2A, 0x2D, 0x30
};

static int g_mp3331_level;
static int g_use_count;
static bool g_match_id;

static bool mp3331_is_torch(int level)
{
	if (level >= MP3331_LEVEL_TORCH)
		return false;

	return true;
}

static int mp3331_verify_level(int level)
{
	if (level < 0)
		return 0;
	if (level >= MP3331_LEVEL_NUM)
		return MP3331_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int mp3331_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct mp3331_chip_data *chip = NULL;

	if (!client) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	chip = i2c_get_clientdata(client);
	if (!chip) {
		pr_err("chip data is null\n");
		return -ENODEV;
	}

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

static int mp3331_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct mp3331_chip_data *chip = NULL;

	if (!client) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	chip = i2c_get_clientdata(client);
	if (!chip) {
		pr_err("chip data is null\n");
		return -ENODEV;
	}

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

/* flashlight enable function */
static int mp3331_enable(void)
{
	unsigned char reg;
	unsigned char val;

	(void)flash_clear_error_and_unlock(mp3331_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	reg = MP3331_REG_MODE_SET;
	if (mp3331_is_torch(g_mp3331_level))
		/* torch mode */
		val = MP3331_STB_LV | MP3331_TORCH_MODE_CUR | MP3331_LED1_EN;
	else
		/* flash mode */
		val = MP3331_STB_LV | MP3331_FLASH_MODE_CUR | MP3331_LED1_EN;

	return mp3331_write_reg(mp3331_i2c_client, reg, val);
}

/* flashlight disable function */
static int mp3331_disable(void)
{
	(void)flash_clear_error_and_unlock(mp3331_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	return mp3331_write_reg(mp3331_i2c_client, MP3331_REG_MODE_SET,
		MP3331_STB_LV);
}

/* set flashlight level */
static int mp3331_set_level(int level)
{
	unsigned char reg;
	unsigned char val;
	unsigned char mode_reg;
	unsigned char mode_val;

	level = mp3331_verify_level(level);
	pr_info("set level %d\n", level);
	g_mp3331_level = level;

	if (mp3331_is_torch(g_mp3331_level)) {
		/* torch mode */
		pr_info("set torch mode\n");
		reg = MP3331_REG_TORCH_CURRENT_CONTROL;
		mode_val = MP3331_STB_LV | MP3331_TORCH_MODE_CUR |
			MP3331_LED1_EN;
	} else {
		/* flash mode */
		pr_info("set flash mode\n");
		reg = MP3331_REG_FLASH_CURRENT_CONTROL;
		mode_val = MP3331_STB_LV | MP3331_FLASH_MODE_CUR |
			MP3331_LED1_EN;
	}
	val = mp3331_flash_level[level];
	mode_reg = MP3331_REG_MODE_SET;
	mp3331_write_reg(mp3331_i2c_client, mode_reg, MP3331_STB_LV |
		MP3331_TORCH_MODE_CUR);

	mp3331_write_reg(mp3331_i2c_client, reg, val);
	mp3331_write_reg(mp3331_i2c_client, mode_reg, mode_val);

	return 0;
}

static int mp3331_get_flag(void)
{
	int ocp_val;
	int ovp_val;
	unsigned int flag;

	ocp_val = mp3331_read_reg(mp3331_i2c_client, MP3331_REG_FLASH_FAULT_H);
	if (ocp_val < 0) {
		pr_err("reg ocp error\n");
		return -EINVAL;
	}
	ovp_val = mp3331_read_reg(mp3331_i2c_client, MP3331_REG_FLASH_FAULT_L);
	if (ovp_val < 0) {
		pr_err("reg ovp error\n");
		return -EINVAL;
	}

	flag = (unsigned int)(ocp_val) | (unsigned int)(ovp_val);
	return flag;
}

int mp3331_init(void)
{
	unsigned char id;
	int val;

	(void)flash_clear_error_and_unlock(mp3331_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	id = mp3331_read_reg(
			mp3331_i2c_client, MP3331_REG_SILICON_REVISION);
	pr_info("MP3331 revision %d\n", id);

	/* disable */
	(void)mp3331_write_reg(mp3331_i2c_client, MP3331_REG_MODE_SET,
			MP3331_STB_LV);

	/* set flash ramp time and timeout */
	val = mp3331_read_reg(mp3331_i2c_client, MP3331_REG_FLASH_TIMER_SET);
	if (val < 0) {
		val = MP3331_FL_TIM | MP3331_SW_FS;
		pr_err("%s, flash timer reg read fail", __func__);
	} else {
		val = ((u8)val & TIMEOUT_MASK)| MP3331_FL_TIM | MP3331_SW_FS;
	}

	return mp3331_write_reg(mp3331_i2c_client, MP3331_REG_FLASH_TIMER_SET, (u8)val);
}

/* flashlight uninit */
int mp3331_uninit(void)
{
	return mp3331_disable();
}

static void mp3331_set_flash_on_off(int flash_on)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* flash off */
	if (flash_on != 1) {
		mp3331_disable();
		hrtimer_cancel(&mp3331_timer);
		return;
	}
	/* flash on */
	if (mp3331_is_torch(g_mp3331_level)) {
		/* torch mode */
		mp3331_timeout_ms = 0;
	} else { /* flash mode */
		if (mp3331_timeout_ms == 0 ||
			mp3331_timeout_ms > MP3331_MAX_TIMEOUT) {
			mp3331_timeout_ms = MP3331_MAX_TIMEOUT;
			pr_info("flash update timeout");
		}
	}

	if (mp3331_timeout_ms) {
		hrtimer_cancel(&mp3331_timer);
		s = mp3331_timeout_ms / 1000;
		ns = mp3331_timeout_ms % 1000 * 1000000;
		ktime = ktime_set(s, ns);
		hrtimer_start(&mp3331_timer, ktime, HRTIMER_MODE_REL);
		pr_info("hrtimer start");
	}
	mp3331_enable();
}

/* Timer and work queue */
static void mp3331_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	mp3331_disable();
}

static enum hrtimer_restart mp3331_timer_func(struct hrtimer *timer)
{
	schedule_work(&mp3331_work);
	return HRTIMER_NORESTART;
}

/* Flashlight operations */
static int mp3331_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg = NULL;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	if (!fl_arg) {
		pr_err("fl_arg is null\n");
		return -EINVAL;
	}
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS %d: %d\n",
			channel, (int)fl_arg->arg);
		mp3331_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY %d: %d\n",
			channel, (int)fl_arg->arg);
		mp3331_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF %d: %d\n",
			channel, (int)fl_arg->arg);
		mp3331_set_flash_on_off(fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER %d\n", channel);
		fl_arg->arg = MP3331_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY %d\n", channel);
		fl_arg->arg = MP3331_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = mp3331_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT %d: %d\n",
			channel, (int)fl_arg->arg);
		fl_arg->arg = mp3331_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT %d\n", channel);
		fl_arg->arg = MP3331_MAX_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT %d\n", channel);
		fl_arg->arg = mp3331_get_flag();
		break;

	default:
		pr_err("No such command and arg %d:  %d, %d\n",
			channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int mp3331_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int mp3331_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int mp3331_set_driver(int set)
{
	int ret = 0;

	mutex_lock(&mp3331_mutex);
	if (set) {
		if (g_use_count == 0)
			ret = mp3331_init();
		g_use_count++;
		pr_info("Set driver: %d\n", g_use_count);
	} else {
		g_use_count--;
		if (g_use_count == 0)
			ret = mp3331_uninit();
		if (g_use_count < 0)
			g_use_count = 0;
		pr_info("Unset driver: %d\n", g_use_count);
	}
	mutex_unlock(&mp3331_mutex);

	return ret;
}

static ssize_t mp3331_strobe_store(struct flashlight_arg arg)
{
	mp3331_set_driver(1); /* set driver on */
	mp3331_set_level(arg.level);
	mp3331_timeout_ms = 0;
	mp3331_enable();
	msleep(arg.dur);
	mp3331_disable();
	mp3331_set_driver(0); /* set driver off */

	return 0;
}

static struct flashlight_operations mp3331_ops = {
	mp3331_open,
	mp3331_release,
	mp3331_ioctl,
	mp3331_strobe_store,
	mp3331_set_driver
};

/* I2C device and driver */
static int mp3331_match_id(void)
{
	int id;
	unsigned int chip_id;
	int val;

	id = mp3331_read_reg(
		mp3331_i2c_client, MP3331_REG_SILICON_REVISION);
	if (id < 0) {
		pr_err("read id err, ret = %d\n", id);
		return -EINVAL;
	}

	chip_id = (unsigned int)(id) & MP3331_CHIP_ID_MASK;
	pr_info("MP3331 id %u\n", chip_id);
	if (chip_id != MP3331_CHIP_ID) {
		g_match_id = false;
		return -EINVAL;
	}
	g_match_id = true;

	(void)mp3331_disable();

	val = mp3331_read_reg(mp3331_i2c_client, MP3331_REG_LOW_VLO_SET);
	if (val >= 0) {
		val = ((u8)val & UVLO_MASK) | MP3331_VBL_RUN;
		(void)mp3331_write_reg(mp3331_i2c_client, MP3331_REG_LOW_VLO_SET, (u8)val);
	}

	return 0;
}

static int mp3331_parse_dt(struct device *dev,
	struct mp3331_platform_data *pdata)
{
	struct device_node *np = NULL;
	struct device_node *cnp = NULL;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata) {
		pr_err("invalid params\n");
		return -ENODEV;
	}

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (pdata->channel_num == 0) {
		pr_err("Parse no dt, node\n");
		return -ENODEV;
	}
	pr_info("Channel number %d\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple\n");

	pdata->dev_id = devm_kzalloc(dev,
		pdata->channel_num *
		sizeof(struct flashlight_device_id),
		GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		if (snprintf_s(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
			strlen(MP3331_NAME), MP3331_NAME) < 0) {
			pr_err("snprintf_s fail\n");
			goto err_node_put;
		}
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)= %d,%d,%d,%s,%d,%d\n",
			pdata->dev_id[i].type, pdata->dev_id[i].ct,
			pdata->dev_id[i].part, pdata->dev_id[i].name,
			pdata->dev_id[i].channel,
			pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	devm_kfree(dev, pdata->dev_id);
	pdata->dev_id = NULL;
	return -EINVAL;
}

static int mp3331_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mp3331_chip_data *chip = NULL;
	int ret;

	if (!client) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	pr_info("i2c probe start\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality\n");
		return -ENODEV;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct mp3331_chip_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	i2c_set_clientdata(client, chip);
	mp3331_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	if (mp3331_match_id()) {
		ret = -ENODEV;
		pr_err("match chip failed\n");
		goto err_out;
	}

	pr_info("i2c probe done\n");

	return 0;

err_out:
	mp3331_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return ret;
}

static int mp3331_i2c_remove(struct i2c_client *client)
{
	struct mp3331_chip_data *chip = NULL;

	if (!client) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	chip = i2c_get_clientdata(client);
	if (!chip) {
		pr_err("chip is null\n");
		return -ENODEV;
	}

	pr_debug("Remove start\n");

	/* platform_data will be free in platform dev remove */
	client->dev.platform_data = NULL;

	/* free resource */
	mp3331_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;

	pr_debug("Remove done\n");

	return 0;
}

static const struct i2c_device_id mp3331_i2c_id[] = {
	{ MP3331_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, mp3331_i2c_id);

static const struct of_device_id mp3331_i2c_of_match[] = {
	{ .compatible = MP3331_DTNAME_I2C },
	{},
};
MODULE_DEVICE_TABLE(of, mp3331_i2c_of_match);

static struct i2c_driver mp3331_i2c_driver = {
	.driver = {
		.name = MP3331_NAME,
		.of_match_table = mp3331_i2c_of_match,
	},
	.probe = mp3331_i2c_probe,
	.remove = mp3331_i2c_remove,
	.id_table = mp3331_i2c_id,
};

/* Platform device and driver */
static int mp3331_probe(struct platform_device *pdev)
{
	struct mp3331_platform_data *pdata = NULL;
	struct mp3331_chip_data *chip = NULL;
	int err;
	int i;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}
	pr_info("Probe start\n");

	g_mp3331_level = -1;

	if (i2c_add_driver(&mp3331_i2c_driver)) {
		pr_err("Failed to add i2c driver\n");
		return -ENODEV;
	}

	if (g_match_id != true) {
		err = -EFAULT;
		pr_err("flash match fail\n");
		return -ENODEV;
	}

	/* init platform data */
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err_probe;

		pdev->dev.platform_data = pdata;
		err = mp3331_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_parse_dt;
	}

	/* init work queue */
	INIT_WORK(&mp3331_work, mp3331_work_disable);

	/* init timer */
	hrtimer_init(&mp3331_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mp3331_timer.function = mp3331_timer_func;
	mp3331_timeout_ms = MP3331_MAX_TIMEOUT;

	/* clear usage count */
	g_use_count = 0;

	/* register flashlight device */
	for (i = 0; i < pdata->channel_num; i++)
		if (flashlight_dev_register_by_device_id(
			&pdata->dev_id[i], &mp3331_ops)) {
			err = -EFAULT;
			goto err_free;
		}

	pr_info("Probe done\n");

	return 0;

err_free:
	devm_kfree(&pdev->dev, pdata->dev_id);
	pdata->dev_id = NULL;

err_parse_dt:
	devm_kfree(&pdev->dev, pdata);
	pdata = NULL;
	pdev->dev.platform_data = NULL;

err_probe:
	chip = i2c_get_clientdata(mp3331_i2c_client);
	i2c_set_clientdata(mp3331_i2c_client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return err;
}

static int mp3331_remove(struct platform_device *pdev)
{
	int i;
	struct mp3331_platform_data *pdata = NULL;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}

	pr_debug("Remove start\n");

	/* flush work queue */
	flush_work(&mp3331_work);
	i2c_del_driver(&mp3331_i2c_driver);
	pdata = dev_get_platdata(&pdev->dev);

	if (!pdata) {
		pr_err("pdata is null\n");
		return -ENODEV;
	}

	/* unregister flashlight device */
	if (pdata->dev_id) {
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
				&pdata->dev_id[i]);

		devm_kfree(&pdev->dev, pdata->dev_id);
		pdata->dev_id = NULL;
	}

	devm_kfree(&pdev->dev, pdata);
	pdata = NULL;
	pdev->dev.platform_data = NULL;

	pr_debug("Remove done\n");

	return 0;
}

static const struct of_device_id mp3331_of_match[] = {
	{ .compatible = MP3331_DTNAME },
	{},
};
MODULE_DEVICE_TABLE(of, mp3331_of_match);

static struct platform_driver mp3331_platform_driver = {
	.probe = mp3331_probe,
	.remove = mp3331_remove,
	.driver = {
		.name = MP3331_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mp3331_of_match,
	},
};

static int __init flashlight_mp3331_init(void)
{
	int ret;

	pr_info("Init start\n");

	ret = platform_driver_register(&mp3331_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver, ret = %d\n", ret);
		return ret;
	}

	pr_info("Init done\n");

	return 0;
}

static void __exit flashlight_mp3331_exit(void)
{
	pr_debug("Exit start\n");

	platform_driver_unregister(&mp3331_platform_driver);

	pr_debug("Exit done\n");
}

late_initcall(flashlight_mp3331_init);
module_exit(flashlight_mp3331_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flashlight MP3331 Driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

