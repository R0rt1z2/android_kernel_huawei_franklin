/*
 * flashlights-sy7804.c
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

#define SY7804_NAME "flashlights-sy7804"
#ifndef SY7804_DTNAME
#define SY7804_DTNAME "mediatek,flashlights_sy7804"
#endif
#ifndef SY7804_DTNAME_I2C
#define SY7804_DTNAME_I2C "mediatek,flashlights_sy7804_i2c"
#endif

/* define registers */
#define SY7804_REG_SILICON_REVISION    0x00
#define SY7804_REG_FLASH_FEATURE       0x08
#define SY7804_INDUCTOR_CURRENT_LIMIT  0x40
#define SY7804_FLASH_RAMP_TIME         0x00
#define SY7804_FLASH_TIMEOUT           0x07
#define SY7804_REG_CURRENT_CONTROL     0x09
#define SY7804_REG_ENABLE              0x0A
#define SY7804_ENABLE_STANDBY          0x00
#define SY7804_ENABLE_TORCH            0x02
#define SY7804_ENABLE_FLASH            0x03
#define SY7804_REG_FLAG                0x0B
#define SY7804_REG_IVFM                0x01
#define SY7804_SLAVE_ADDRESS           0x63

/* define level */
#define SY7804_LEVEL_NUM               18
#define SY7804_LEVEL_TORCH             4
#define SY7804_HW_TIMEOUT              800 /* ms */
#define SY7804_MAX_TIMEOUT             680 /* ms */
#define SY7804_CHIP_ID                 0x52
#define SY7804_UNDER_VOLTAGE_LOCKOUT   0x10
#define SY7804_OVER_VOLTAGE_PROTECT    0x08
#define SY7804_LED_VOUT_SHORT          0x04
#define SY7804_OVER_TEMP_PROTECT       0x02
#define SY7804_IVFM_EN                 0x80
#define SY7804_UVLO_EN                 0x80
#define SY7804_UVLO_VOLTAGE            0x0C /* 3.2v */
#define IVFM_MASK                      0x7F
#define UVLO_MASK                      0x63
#define TIMEOUT_MASK                   0x80

/* define mutex and work queue */
static DEFINE_MUTEX(sy7804_mutex);
static struct work_struct sy7804_work;

static struct flash_name_info_t g_name_info = {
	.ic_name = "SY7804",
	.module_name = "SY",
};

static struct flash_error_info_t g_flash_error_info[] = {
	{
		.reg_addr = SY7804_REG_FLAG,
		.error_table = {
			{
				SY7804_OVER_TEMP_PROTECT,
				REG_VAL_AND_MASK,
				DSM_FLASH_HOT_DIE_ERROR_NO,
				"flash temperature is too hot",
			}, {
				(SY7804_OVER_VOLTAGE_PROTECT | SY7804_LED_VOUT_SHORT),
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash OVP, LED or VOUT short",
			}, {
				SY7804_UNDER_VOLTAGE_LOCKOUT,
				REG_VAL_AND_MASK,
				DSM_FLASH_UNDER_VOLTAGE_LOCKOUT_ERROR_NO,
				"flash uvlo",
			},
		},
		.size = 3,
	}
};

/* define i2c */
static struct i2c_client *sy7804_i2c_client;

/* platform data */
struct sy7804_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* sy7804 chip data */
struct sy7804_chip_data {
	struct i2c_client *client;
	struct sy7804_platform_data *pdata;
	struct mutex lock;
};

static struct hrtimer sy7804_timer;
static unsigned int sy7804_timeout_ms;

/* sy7804 operations */
static const int sy7804_current[SY7804_LEVEL_NUM] = {
	48,  93,  141,  188,  281,  375,  469,  563, 656, 750,
	844, 938, 1031, 1125, 1219, 1313, 1406, 1500
};

static const unsigned char sy7804_flash_level[SY7804_LEVEL_NUM] = {
	0x00, 0x10, 0x20, 0x30, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static int g_sy7804_level;
static int g_use_count;
static bool g_match_id;

static bool sy7804_is_torch(int level)
{
	if (level >= SY7804_LEVEL_TORCH)
		return false;

	return true;
}

static int sy7804_verify_level(int level)
{
	if (level < 0)
		return 0;
	if (level >= SY7804_LEVEL_NUM)
		return SY7804_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int sy7804_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct sy7804_chip_data *chip = NULL;

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

static int sy7804_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct sy7804_chip_data *chip = NULL;

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
static int sy7804_enable(void)
{
	unsigned char reg;
	unsigned char val;

	(void)flash_clear_error_and_unlock(sy7804_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	reg = SY7804_REG_ENABLE;
	if (sy7804_is_torch(g_sy7804_level))
		/* torch mode */
		val = SY7804_ENABLE_TORCH;
	else
		/* flash mode */
		val = SY7804_ENABLE_FLASH;

	return sy7804_write_reg(sy7804_i2c_client, reg, val);
}

/* flashlight disable function */
static int sy7804_disable(void)
{
	(void)flash_clear_error_and_unlock(sy7804_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	return sy7804_write_reg(sy7804_i2c_client, SY7804_REG_ENABLE,
		SY7804_ENABLE_STANDBY);
}

/* set flashlight level */
static int sy7804_set_level(int level)
{
	unsigned char reg;
	unsigned char val;

	level = sy7804_verify_level(level);
	pr_info("set level %d\n", level);
	g_sy7804_level = level;

	reg = SY7804_REG_CURRENT_CONTROL;
	val = sy7804_flash_level[level];

	return sy7804_write_reg(sy7804_i2c_client, reg, val);
}

static int sy7804_get_flag(void)
{
	return sy7804_read_reg(sy7804_i2c_client, SY7804_REG_FLAG);
}

/* flashlight init */
int sy7804_init(void)
{
	unsigned char id;
	int val;

	(void)flash_clear_error_and_unlock(sy7804_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	id = sy7804_read_reg(
		sy7804_i2c_client, SY7804_REG_SILICON_REVISION);
	pr_info("SY7804 revision %d\n", id);

	/* disable */
	(void)sy7804_write_reg(sy7804_i2c_client, SY7804_REG_ENABLE,
		SY7804_ENABLE_STANDBY);

	val = sy7804_read_reg(sy7804_i2c_client, SY7804_REG_FLASH_FEATURE);
	if (val < 0) {
		val = SY7804_INDUCTOR_CURRENT_LIMIT | SY7804_FLASH_RAMP_TIME | SY7804_FLASH_TIMEOUT;
		pr_err("%s, flash timer reg read fail", __func__);
	} else {
		val = ((u8)val & TIMEOUT_MASK)| SY7804_INDUCTOR_CURRENT_LIMIT |
			SY7804_FLASH_RAMP_TIME | SY7804_FLASH_TIMEOUT;
	}

	/* set flash ramp time and timeout */
	return sy7804_write_reg(sy7804_i2c_client, SY7804_REG_FLASH_FEATURE, (u8)val);
}

/* flashlight uninit */
int sy7804_uninit(void)
{
	return sy7804_disable();
}

static void sy7804_set_flash_on_off(int flash_on)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* flash off */
	if (flash_on != 1) {
		sy7804_disable();
		hrtimer_cancel(&sy7804_timer);
		return;
	}
	/* flash on */
	if (sy7804_is_torch(g_sy7804_level)) {
		/* torch mode */
		sy7804_timeout_ms = 0;
	} else { /* flash mode */
		if (sy7804_timeout_ms == 0 ||
			sy7804_timeout_ms > SY7804_MAX_TIMEOUT) {
			sy7804_timeout_ms = SY7804_MAX_TIMEOUT;
			pr_info("flash update timeout");
		}
	}

	if (sy7804_timeout_ms) {
		hrtimer_cancel(&sy7804_timer);
		s = sy7804_timeout_ms / 1000;
		ns = sy7804_timeout_ms % 1000 * 1000000;
		ktime = ktime_set(s, ns);
		hrtimer_start(&sy7804_timer, ktime, HRTIMER_MODE_REL);
		pr_info("hrtimer start");
	}
	sy7804_enable();
}

/* Timer and work queue */
static void sy7804_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	sy7804_disable();
}

static enum hrtimer_restart sy7804_timer_func(struct hrtimer *timer)
{
	schedule_work(&sy7804_work);
	return HRTIMER_NORESTART;
}

/* Flashlight operations */
static int sy7804_ioctl(unsigned int cmd, unsigned long arg)
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
		sy7804_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY %d: %d\n",
			channel, (int)fl_arg->arg);
		sy7804_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF %d: %d\n",
			channel, (int)fl_arg->arg);
		sy7804_set_flash_on_off(fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER %d\n", channel);
		fl_arg->arg = SY7804_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY %d\n", channel);
		fl_arg->arg = SY7804_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = sy7804_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT %d: %d\n",
			channel, (int)fl_arg->arg);
		fl_arg->arg = sy7804_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT %d\n", channel);
		fl_arg->arg = SY7804_MAX_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT %d\n", channel);
		fl_arg->arg = sy7804_get_flag();
		break;

	default:
		pr_err("No such command and arg %d:  %d, %d\n",
			channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sy7804_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int sy7804_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int sy7804_set_driver(int set)
{
	int ret = 0;

	mutex_lock(&sy7804_mutex);
	if (set) {
		if (g_use_count == 0)
			ret = sy7804_init();
		g_use_count++;
		pr_debug("Set driver: %d\n", g_use_count);
	} else {
		g_use_count--;
		if (g_use_count == 0)
			ret = sy7804_uninit();
		if (g_use_count < 0)
			g_use_count = 0;
		pr_debug("Unset driver: %d\n", g_use_count);
	}
	mutex_unlock(&sy7804_mutex);

	return ret;
}

static ssize_t sy7804_strobe_store(struct flashlight_arg arg)
{
	sy7804_set_driver(1); /* set driver on */
	sy7804_set_level(arg.level);
	sy7804_timeout_ms = 0;
	sy7804_enable();
	msleep(arg.dur);
	sy7804_disable();
	sy7804_set_driver(0); /* set driver off */

	return 0;
}

static struct flashlight_operations sy7804_ops = {
	sy7804_open,
	sy7804_release,
	sy7804_ioctl,
	sy7804_strobe_store,
	sy7804_set_driver
};

/* I2C device and driver */
static int sy7804_match_id(void)
{
	int id;
	int val;

	id = sy7804_read_reg(
		sy7804_i2c_client, SY7804_REG_SILICON_REVISION);
	pr_info("sy7804 id %d\n", id);
	if (id != SY7804_CHIP_ID) {
		g_match_id = false;
		return -EINVAL;
	}
	g_match_id = true;

	(void)sy7804_disable();

	val = sy7804_read_reg(sy7804_i2c_client, SY7804_REG_ENABLE);
	if (val >= 0) {
		val = ((u8)val & IVFM_MASK) | SY7804_IVFM_EN;
		(void)sy7804_write_reg(sy7804_i2c_client, SY7804_REG_ENABLE, (u8)val);
	}

	val = sy7804_read_reg(sy7804_i2c_client, SY7804_REG_IVFM);
	if (val >= 0) {
		val = ((u8)val & UVLO_MASK) | SY7804_UVLO_EN | SY7804_UVLO_VOLTAGE;
		(void)sy7804_write_reg(sy7804_i2c_client, SY7804_REG_IVFM, (u8)val);
	}

	return 0;
}

static int sy7804_parse_dt(struct device *dev,
	struct sy7804_platform_data *pdata)
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
			strlen(SY7804_NAME), SY7804_NAME) < 0) {
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

static int sy7804_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sy7804_chip_data *chip = NULL;
	int ret;

	if (!client) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	pr_info("i2c probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(struct sy7804_chip_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	i2c_set_clientdata(client, chip);
	sy7804_i2c_client = client;
	sy7804_i2c_client->addr = SY7804_SLAVE_ADDRESS;
	pr_info("sy7804 real i2c addr = 0x%x", sy7804_i2c_client->addr);

	mutex_init(&chip->lock);

	if (sy7804_match_id()) {
		ret = -ENODEV;
		pr_err("match chip failed\n");
		goto err_out;
	}

	pr_info("i2c probe done\n");

	return 0;

err_out:
	sy7804_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return ret;
}

static int sy7804_i2c_remove(struct i2c_client *client)
{
	struct sy7804_chip_data *chip = NULL;

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
	sy7804_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;

	pr_debug("Remove done\n");

	return 0;
}

static const struct i2c_device_id sy7804_i2c_id[] = {
	{ SY7804_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sy7804_i2c_id);

static const struct of_device_id sy7804_i2c_of_match[] = {
	{.compatible = SY7804_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, sy7804_i2c_of_match);

static struct i2c_driver sy7804_i2c_driver = {
	.driver = {
		.name = SY7804_NAME,
		.of_match_table = sy7804_i2c_of_match,
	},
	.probe = sy7804_i2c_probe,
	.remove = sy7804_i2c_remove,
	.id_table = sy7804_i2c_id,
};

/* Platform device and driver */
static int sy7804_probe(struct platform_device *pdev)
{
	struct sy7804_platform_data *pdata =  NULL;
	struct sy7804_chip_data *chip = NULL;
	int err;
	int i;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}
	pr_info("Probe start\n");

	g_sy7804_level = -1;
	if (i2c_add_driver(&sy7804_i2c_driver)) {
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
		err = sy7804_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_parse_dt;
	}

	/* init work queue */
	INIT_WORK(&sy7804_work, sy7804_work_disable);

	/* init timer */
	hrtimer_init(&sy7804_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sy7804_timer.function = sy7804_timer_func;
	sy7804_timeout_ms = SY7804_MAX_TIMEOUT;

	/* clear usage count */
	g_use_count = 0;

	/* register flashlight device */
	for (i = 0; i < pdata->channel_num; i++)
		if (flashlight_dev_register_by_device_id(
			&pdata->dev_id[i], &sy7804_ops)) {
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
	chip = i2c_get_clientdata(sy7804_i2c_client);
	i2c_set_clientdata(sy7804_i2c_client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return err;
}

static int sy7804_remove(struct platform_device *pdev)
{
	int i;
	struct sy7804_platform_data *pdata = NULL;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}

	pr_debug("Remove start\n");

	/* flush work queue */
	flush_work(&sy7804_work);
	i2c_del_driver(&sy7804_i2c_driver);
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

static const struct of_device_id sy7804_of_match[] = {
	{.compatible = SY7804_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, sy7804_of_match);

static struct platform_driver sy7804_platform_driver = {
	.probe = sy7804_probe,
	.remove = sy7804_remove,
	.driver = {
		.name = SY7804_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7804_of_match,
	},
};

static int __init flashlight_sy7804_init(void)
{
	int ret;

	pr_info("Init start\n");

	ret = platform_driver_register(&sy7804_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver, ret = %d\n", ret);
		return ret;
	}

	pr_info("Init done\n");

	return 0;
}

static void __exit flashlight_sy7804_exit(void)
{
	pr_debug("Exit start\n");

	platform_driver_unregister(&sy7804_platform_driver);

	pr_debug("Exit done\n");
}

late_initcall(flashlight_sy7804_init);
module_exit(flashlight_sy7804_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flashlight SY7804 Driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

