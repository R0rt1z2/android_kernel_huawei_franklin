/*
 * Copyright (C) 2015 MediaTek Inc.
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

/* device tree should be defined in flashlight-dt.h */
#ifndef LM3642_DTNAME
#define LM3642_DTNAME "mediatek,flashlights_lm3642"
#endif
#ifndef LM3642_DTNAME_I2C
#define LM3642_DTNAME_I2C "mediatek,flashlights_lm3642_i2c"
#endif

#define LM3642_NAME "flashlights-lm3642"

/* define registers */
#define LM3642_REG_SILICON_REVISION (0x00)

#define LM3642_REG_FLASH_FEATURE      (0x08)
#define LM3642_INDUCTOR_CURRENT_LIMIT (0x40)
#define LM3642_FLASH_RAMP_TIME        (0x00)
#define LM3642_FLASH_TIMEOUT          (0x07) /* flash hardware timeout, 800ms */

#define LM3642_REG_CURRENT_CONTROL (0x09)

#define LM3642_REG_ENABLE (0x0A)
#define LM3642_ENABLE_STANDBY (0x00)
#define LM3642_ENABLE_TORCH (0x02)
#define LM3642_ENABLE_FLASH (0x03)

#define LM3642_REG_FLAG (0x0B)

/* define level */
#define LM3642_LEVEL_NUM 18
#define LM3642_LEVEL_TORCH 4
#define LM3642_MAX_TIMEOUT 680 /* ms */

#define LM3642_CHIP_ID_MASK 0x07
#define LM3642_CHIP_ID 0x00
#define LM3642_UNDER_VOLTAGE_LOCKOUT 0x10
#define LM3642_OVER_VOLTAGE_PROTECT 0x08
#define LM3642_LED_VOUT_SHORT 0x04
#define LM3642_OVER_TEMP_PROTECT 0x02
#define REG_IVFM 0x01
#define UVLO_EN 0x80
#define UVLO_VOLTAGE 0x0C /* 3.2V */
#define IVFM_EN 0x80
#define IVFM_MASK 0x7F
#define UVLO_MASK 0x63
#define TIMEOUT_MASK 0x80

/* define mutex and work queue */
static DEFINE_MUTEX(lm3642_mutex);
static struct work_struct lm3642_work;

static struct flash_name_info_t g_name_info = {
	.ic_name = "lm3642",
	.module_name = "TI",
};

static struct flash_error_info_t g_flash_error_info[] = {
	{
		.reg_addr = LM3642_REG_FLAG,
		.error_table = {
			{
				LM3642_OVER_TEMP_PROTECT,
				REG_VAL_AND_MASK,
				DSM_FLASH_HOT_DIE_ERROR_NO,
				"flash temperature is too hot",
			}, {
				(LM3642_OVER_VOLTAGE_PROTECT | LM3642_LED_VOUT_SHORT),
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash OVP, LED or VOUT short",
			}, {
				LM3642_UNDER_VOLTAGE_LOCKOUT,
				REG_VAL_AND_MASK,
				DSM_FLASH_UNDER_VOLTAGE_LOCKOUT_ERROR_NO,
				"flash uvlo",
			},
		},
		.size = 3,
	}
};

/* lm3642 revision */
static int is_lm3642lt;
static bool g_match_id;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *lm3642_i2c_client;

/* platform data */
struct lm3642_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* lm3642 chip data */
struct lm3642_chip_data {
	struct i2c_client *client;
	struct lm3642_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * lm3642 operations
 *****************************************************************************/
static const int lm3642_current[LM3642_LEVEL_NUM] = {
	 48,  93,  141,  188,  281,  375,  469,  563, 656, 750,
	844, 938, 1031, 1125, 1219, 1313, 1406, 1500
};

static const unsigned char lm3642_flash_level[LM3642_LEVEL_NUM] = {
	0x00, 0x10, 0x20, 0x30, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static const unsigned char lm3642lt_flash_level[LM3642_LEVEL_NUM] = {
	0x10, 0x30, 0x50, 0x70, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static int lm3642_level = -1;

static bool lm3642_is_torch(int level)
{
	if (level >= LM3642_LEVEL_TORCH)
		return false;

	return true;
}

static int lm3642_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= LM3642_LEVEL_NUM)
		level = LM3642_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int lm3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct lm3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

static int lm3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct lm3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

/* flashlight enable function */
static int lm3642_enable(void)
{
	unsigned char reg, val;

	(void)flash_clear_error_and_unlock(lm3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	reg = LM3642_REG_ENABLE;
	if (lm3642_is_torch(lm3642_level)) {
		/* torch mode */
		val = LM3642_ENABLE_TORCH;
	} else {
		/* flash mode */
		val = LM3642_ENABLE_FLASH;
	}

	return lm3642_write_reg(lm3642_i2c_client, reg, val);
}

/* flashlight disable function */
static int lm3642_disable(void)
{
	unsigned char reg, val;

	(void)flash_clear_error_and_unlock(lm3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	reg = LM3642_REG_ENABLE;
	val = LM3642_ENABLE_STANDBY;

	return lm3642_write_reg(lm3642_i2c_client, reg, val);
}

/* set flashlight level */
static int lm3642_set_level(int level)
{
	unsigned char reg, val;

	level = lm3642_verify_level(level);
	lm3642_level = level;

	reg = LM3642_REG_CURRENT_CONTROL;
	if (is_lm3642lt)
		val = lm3642lt_flash_level[level];
	else
		val = lm3642_flash_level[level];

	return lm3642_write_reg(lm3642_i2c_client, reg, val);
}

static int lm3642_get_flag(void)
{
	return lm3642_read_reg(lm3642_i2c_client, LM3642_REG_FLAG);
}

/* flashlight init */
int lm3642_init(void)
{
	int val;

	(void)flash_clear_error_and_unlock(lm3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	/* get silicon revision */
	is_lm3642lt = lm3642_read_reg(
			lm3642_i2c_client, LM3642_REG_SILICON_REVISION);
	pr_info("LM3642(LT) revision(%d).\n", is_lm3642lt);

	/* disable */
	(void)lm3642_write_reg(lm3642_i2c_client, LM3642_REG_ENABLE,
			LM3642_ENABLE_STANDBY);

	val = lm3642_read_reg(lm3642_i2c_client, LM3642_REG_FLASH_FEATURE);
	if (val < 0) {
		val = LM3642_INDUCTOR_CURRENT_LIMIT | LM3642_FLASH_RAMP_TIME | LM3642_FLASH_TIMEOUT;
		pr_err("%s, flash timer reg read fail", __func__);
	} else {
		val = ((u8)val & TIMEOUT_MASK)| LM3642_INDUCTOR_CURRENT_LIMIT |
			LM3642_FLASH_RAMP_TIME | LM3642_FLASH_TIMEOUT;
	}

	/* set flash ramp time and timeout */
	return lm3642_write_reg(lm3642_i2c_client, LM3642_REG_FLASH_FEATURE, (u8)val);
}

/* flashlight uninit */
int lm3642_uninit(void)
{
	return lm3642_disable();
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer lm3642_timer;
static unsigned int lm3642_timeout_ms;

static void lm3642_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	lm3642_disable();
}

static enum hrtimer_restart lm3642_timer_func(struct hrtimer *timer)
{
	schedule_work(&lm3642_work);
	return HRTIMER_NORESTART;
}

static void lm3642_set_flash_on_off(int flash_on)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* flash off */
	if (flash_on != 1) {
		lm3642_disable();
		hrtimer_cancel(&lm3642_timer);
		return;
	}
	/* flash on */
	if (lm3642_is_torch(lm3642_level)) {
		/* torch mode */
		lm3642_timeout_ms = 0;
	} else { /* flash mode */
		if (lm3642_timeout_ms == 0 ||
			lm3642_timeout_ms > LM3642_MAX_TIMEOUT) {
			lm3642_timeout_ms = LM3642_MAX_TIMEOUT;
			pr_info("flash update timeout");
		}
	}

	if (lm3642_timeout_ms) {
		hrtimer_cancel(&lm3642_timer);
		s = lm3642_timeout_ms / 1000;
		ns = lm3642_timeout_ms % 1000 * 1000000;
		ktime = ktime_set(s, ns);
		hrtimer_start(&lm3642_timer, ktime, HRTIMER_MODE_REL);
		pr_info("hrtimer start");
	}

	lm3642_enable();
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int lm3642_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	if (!fl_arg) {
		pr_err("fl_arg is null\n");
		return -EINVAL;
	}
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm3642_timeout_ms = fl_arg->arg;

		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm3642_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF %d: %d\n",
			channel, (int)fl_arg->arg);
		lm3642_set_flash_on_off(fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = LM3642_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = LM3642_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = lm3642_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = lm3642_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = LM3642_MAX_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT(%d)\n", channel);
		fl_arg->arg = lm3642_get_flag();
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm3642_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3642_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3642_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&lm3642_mutex);
	if (set) {
		if (!use_count)
			ret = lm3642_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = lm3642_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&lm3642_mutex);

	return ret;
}

static ssize_t lm3642_strobe_store(struct flashlight_arg arg)
{
	lm3642_set_driver(1);
	lm3642_set_level(arg.level);
	lm3642_timeout_ms = 0;
	lm3642_enable();
	msleep(arg.dur);
	lm3642_disable();
	lm3642_set_driver(0);

	return 0;
}

static struct flashlight_operations lm3642_ops = {
	lm3642_open,
	lm3642_release,
	lm3642_ioctl,
	lm3642_strobe_store,
	lm3642_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int lm3642_match_id(void)
{
	int id;
	unsigned int chip_id;
	int val;

	id = lm3642_read_reg(
		lm3642_i2c_client, LM3642_REG_SILICON_REVISION);
	if (id < 0) {
		pr_err("read id err, ret = %d\n", id);
		return -EINVAL;
	}

	chip_id = (unsigned int)(id) & LM3642_CHIP_ID_MASK;
	pr_info("lm3642 id %u\n", chip_id);
	if (chip_id != LM3642_CHIP_ID) {
		g_match_id = false;
		return -EINVAL;
	}
	g_match_id = true;

	(void)lm3642_disable();

	val = lm3642_read_reg(lm3642_i2c_client, LM3642_REG_ENABLE);
	if (val >= 0) {
		val = ((u8)val & IVFM_MASK) | IVFM_EN;
		(void)lm3642_write_reg(lm3642_i2c_client, LM3642_REG_ENABLE, (u8)val);
	}

	val = lm3642_read_reg(lm3642_i2c_client, REG_IVFM);
	if (val >= 0) {
		val = ((u8)val & UVLO_MASK) | UVLO_EN | UVLO_VOLTAGE;
		(void)lm3642_write_reg(lm3642_i2c_client, REG_IVFM, (u8)val);
	}

	return 0;
}

static int lm3642_parse_dt(struct device *dev,
		struct lm3642_platform_data *pdata)
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
			strlen(LM3642_NAME), LM3642_NAME) < 0) {
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

static int lm3642_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3642_chip_data *chip = NULL;
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
	chip = kzalloc(sizeof(struct lm3642_chip_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	i2c_set_clientdata(client, chip);
	lm3642_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	if (lm3642_match_id()) {
		ret = -ENODEV;
		pr_err("match chip failed\n");
		goto err_out;
	}

	pr_info("i2c probe done\n");

	return 0;

err_out:
	lm3642_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return ret;
}

static int lm3642_i2c_remove(struct i2c_client *client)
{
	struct lm3642_chip_data *chip = NULL;

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
	lm3642_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;

	pr_debug("Remove done\n");

	return 0;
}

static const struct i2c_device_id lm3642_i2c_id[] = {
	{LM3642_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lm3642_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id lm3642_i2c_of_match[] = {
	{.compatible = LM3642_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, lm3642_i2c_of_match);
#endif

static struct i2c_driver lm3642_i2c_driver = {
	.driver = {
		.name = LM3642_NAME,
#ifdef CONFIG_OF
		.of_match_table = lm3642_i2c_of_match,
#endif
	},
	.probe = lm3642_i2c_probe,
	.remove = lm3642_i2c_remove,
	.id_table = lm3642_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int lm3642_probe(struct platform_device *pdev)
{
	struct lm3642_platform_data *pdata = NULL;
	struct lm3642_chip_data *chip = NULL;
	int err;
	int i;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}
	pr_info("Probe start\n");

	if (i2c_add_driver(&lm3642_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
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
		err = lm3642_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_parse_dt;
	}

	/* init work queue */
	INIT_WORK(&lm3642_work, lm3642_work_disable);

	/* init timer */
	hrtimer_init(&lm3642_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3642_timer.function = lm3642_timer_func;
	lm3642_timeout_ms = LM3642_MAX_TIMEOUT;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	for (i = 0; i < pdata->channel_num; i++)
		if (flashlight_dev_register_by_device_id(
			&pdata->dev_id[i], &lm3642_ops)) {
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
	chip = i2c_get_clientdata(lm3642_i2c_client);
	i2c_set_clientdata(lm3642_i2c_client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return err;
}

static int lm3642_remove(struct platform_device *pdev)
{
	int i;
	struct lm3642_platform_data *pdata = NULL;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}

	pr_debug("Remove start\n");

	/* flush work queue */
	flush_work(&lm3642_work);
	i2c_del_driver(&lm3642_i2c_driver);
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

#ifdef CONFIG_OF
static const struct of_device_id lm3642_of_match[] = {
	{.compatible = LM3642_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, lm3642_of_match);
#else
static struct platform_device lm3642_platform_device[] = {
	{
		.name = LM3642_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, lm3642_platform_device);
#endif

static struct platform_driver lm3642_platform_driver = {
	.probe = lm3642_probe,
	.remove = lm3642_remove,
	.driver = {
		.name = LM3642_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3642_of_match,
#endif
	},
};

static int __init flashlight_lm3642_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&lm3642_platform_device);
	if (ret) {
		pr_err("Failed to register platform device, ret = %d\n", ret);
		return ret;
	}
#endif

	ret = platform_driver_register(&lm3642_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver, ret = %d\n", ret);
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_lm3642_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&lm3642_platform_driver);

	pr_debug("Exit done.\n");
}

late_initcall(flashlight_lm3642_init);
module_exit(flashlight_lm3642_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xi Chen <xixi.chen@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight LM3642 Driver");

