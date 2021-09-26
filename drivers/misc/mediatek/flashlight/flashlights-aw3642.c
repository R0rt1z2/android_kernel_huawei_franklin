/*
 * flashlights-aw3642.c
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

#define AW3642_NAME "flashlights-aw3642"
#ifndef AW3642_DTNAME
#define AW3642_DTNAME "mediatek,flashlights_aw3642"
#endif
#ifndef AW3642_DTNAME_I2C
#define AW3642_DTNAME_I2C "mediatek,flashlights_aw3642_i2c"
#endif
#define AW3642_SLAVE_ADDRESS             0x63
#define AW3642_REG_SILICON_REVISION      0x00
#define AW3642_REG_FLASH_CURRENT_CONTROL 0x03
#define AW3642_REG_TORCH_CURRENT_CONTROL 0x05
#define AW3642_REG_ENABLE                0x01
#define AW3642_REG_OCP_FLAGS             0x0A
#define AW3642_REG_OVP_FLAGS             0x0B
#define AW3642_REG_IVFM                  0x02
#define AW3642_ENABLE_STANDBY            0x00
#define AW3642_ENABLE_TORCH              0x08
#define AW3642_ENABLE_FLASH              0x0c
#define AW3642_LED_EN                    0x03
#define AW3642_LEVEL_NUM                 18
#define AW3642_LEVEL_TORCH               4
#define AW3642_HW_TIMEOUT                800
#define AW3642_CHIP_ID                   0x36
#define AW3642_UNDER_VOLTAGE_LOCKOUT     0x02
#define AW3642_OVER_VOLTAGE_PROTECT      0x02
#define AW3642_LED_SHORT                 0x10
#define AW3642_VOUT_SHORT                0x40
#define AW3642_OVER_TEMP_PROTECT         0x04
#define AW3642_IVFM_EN                   0x01
#define AW3642_UVLO_EN                   0x40 /* bit6 1:enable, 0 disable */
#define AW3642_IVFM_VOLTAGE              0x18 /* 3.2v */
#define AW3642_REG_FLASH_TIMER_SET       0x08
#define AW3642_FLASH_TIMEOUT             0x0B /* hardware timeout 800ms */
#define AW3642_MAX_TIMEOUT               680 /* ms */
#define AW3642_TROCH_RAMP_TIME           0x10 /* 1ms(default) */
#define TIMEOUT_MASK                     0xF0
#define IVFM_MASK                        0x84

/* define mutex and work queue */
static DEFINE_MUTEX(aw3642_mutex);
static struct work_struct aw3642_work;
static struct i2c_client *aw3642_i2c_client;

/* platform data */
struct aw3642_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* aw3642 chip data */
struct aw3642_chip_data {
	struct i2c_client *client;
	struct aw3642_platform_data *pdata;
	struct mutex lock;
};

static struct flash_name_info_t g_name_info = {
	.ic_name = "AW3642",
	.module_name = "AWI",
};

static struct flash_error_info_t g_flash_error_info[] = {
	{
		.reg_addr = AW3642_REG_OCP_FLAGS,
		.error_table = {
			{
				AW3642_OVER_TEMP_PROTECT,
				REG_VAL_AND_MASK,
				DSM_FLASH_HOT_DIE_ERROR_NO,
				"flash temperature is too hot",
			}, {
				(AW3642_LED_SHORT | AW3642_VOUT_SHORT),
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash LED or VOUT short",
			}, {
				AW3642_UNDER_VOLTAGE_LOCKOUT,
				REG_VAL_AND_MASK,
				DSM_FLASH_UNDER_VOLTAGE_LOCKOUT_ERROR_NO,
				"flash UVLO",
			},
		},
		.size = 3,
	}, {
		.reg_addr = AW3642_REG_OVP_FLAGS,
		.error_table = {
			{
				AW3642_OVER_VOLTAGE_PROTECT,
				REG_VAL_AND_MASK,
				DSM_FLASH_OPEN_SHOTR_ERROR_NO,
				"flash OVP",

			},
		},
		.size = 1,
	}
};

static struct hrtimer aw3642_timer;
static unsigned int aw3642_timeout_ms;

/* aw3642 operations */
static const int aw3642_current[AW3642_LEVEL_NUM] = {
	47, 94, 141, 188, 283, 378, 471, 566, 659, 754,
	846, 940, 1034, 1128, 1219, 1313, 1407, 1500
};

static const unsigned char aw3642_flash_level[AW3642_LEVEL_NUM] = {
	0x07, 0x0F, 0x17, 0x1F, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
	0x23, 0x27, 0x2B, 0x2F, 0x33, 0x37, 0x3B, 0x3F
};

static int g_aw3642_level;
static int g_use_count;
static bool g_match_id;

static bool aw3642_is_torch(int level)
{
	if (level >= AW3642_LEVEL_TORCH)
		return false;

	return true;
}

static int aw3642_verify_level(int level)
{
	if (level < 0)
		return 0;
	if (level >= AW3642_LEVEL_NUM)
		return AW3642_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int aw3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw3642_chip_data *chip = NULL;

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

static int aw3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw3642_chip_data *chip = NULL;

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
static int aw3642_enable(void)
{
	unsigned char reg;
	unsigned char val;

	(void)flash_clear_error_and_unlock(aw3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	reg = AW3642_REG_ENABLE;
	if (aw3642_is_torch(g_aw3642_level))
		/* torch mode */
		val = AW3642_ENABLE_TORCH | AW3642_LED_EN;
	else
		/* flash mode */
		val = AW3642_ENABLE_FLASH | AW3642_LED_EN;

	return aw3642_write_reg(aw3642_i2c_client, reg, val);
}

/* flashlight disable function */
static int aw3642_disable(void)
{
	(void)flash_clear_error_and_unlock(aw3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	return aw3642_write_reg(aw3642_i2c_client, AW3642_REG_ENABLE,
		AW3642_ENABLE_STANDBY);
}

/* set flashlight level */
static int aw3642_set_level(int level)
{
	unsigned char reg;
	unsigned char val;

	level = aw3642_verify_level(level);
	pr_info("set level %d\n", level);
	g_aw3642_level = level;

	if (aw3642_is_torch(g_aw3642_level)) {
		/* torch mode */
		pr_info("set torch mode\n");
		reg = AW3642_REG_TORCH_CURRENT_CONTROL;
	} else {
		/* flash mode */
		pr_info("set flash mode\n");
		reg = AW3642_REG_FLASH_CURRENT_CONTROL;
	}

	val = aw3642_flash_level[level];
	val = 0x80 | (val & 0x3f); /* set bit */

	return aw3642_write_reg(aw3642_i2c_client, reg, val);
}

static int aw3642_get_flag(void)
{
	int ocp_val;
	int ovp_val;
	unsigned int flag;

	ocp_val = aw3642_read_reg(aw3642_i2c_client, AW3642_REG_OCP_FLAGS);
	if (ocp_val < 0) {
		pr_err("reg ocp error\n");
		return -EINVAL;
	}
	ovp_val = aw3642_read_reg(aw3642_i2c_client, AW3642_REG_OVP_FLAGS);
	if (ovp_val < 0) {
		pr_err("reg ovp error\n");
		return -EINVAL;
	}

	flag = (unsigned int)(ocp_val) | (unsigned int)(ovp_val);
	return flag;
}

/* flashlight init */
int aw3642_init(void)
{
	unsigned char id;
	int val;

	(void)flash_clear_error_and_unlock(aw3642_i2c_client, g_flash_error_info,
		array_size(g_flash_error_info), &g_name_info);

	/* get silicon revision */
	id = aw3642_read_reg(
		aw3642_i2c_client, AW3642_REG_SILICON_REVISION);
	pr_info("AW3642 revision %d\n", id);

	/* disable */
	(void)aw3642_write_reg(aw3642_i2c_client, AW3642_REG_ENABLE,
		AW3642_ENABLE_STANDBY);

	val = aw3642_read_reg(aw3642_i2c_client, AW3642_REG_FLASH_TIMER_SET);
	if (val < 0) {
		val = AW3642_FLASH_TIMEOUT | AW3642_TROCH_RAMP_TIME;
		pr_err("%s, flash timer reg read fail", __func__);
	} else {
		val = ((u8)val & TIMEOUT_MASK) | AW3642_FLASH_TIMEOUT;
	}

	/* set flash ramp time and timeout */
	return aw3642_write_reg(aw3642_i2c_client, AW3642_REG_FLASH_TIMER_SET, (u8)val);
}

/* flashlight uninit */
int aw3642_uninit(void)
{
	return aw3642_disable();
}

static void aw3642_set_flash_on_off(int flash_on)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* flash off */
	if (flash_on != 1) {
		aw3642_disable();
		hrtimer_cancel(&aw3642_timer);
		return;
	}
	/* flash on */
	if (aw3642_is_torch(g_aw3642_level)) {
		/* torch mode */
		aw3642_timeout_ms = 0;
	} else { /* flash mode */
		if (aw3642_timeout_ms == 0 ||
			aw3642_timeout_ms > AW3642_MAX_TIMEOUT) {
			aw3642_timeout_ms = AW3642_MAX_TIMEOUT;
			pr_info("flash update timeout");
		}
	}

	if (aw3642_timeout_ms) {
		hrtimer_cancel(&aw3642_timer);
		s = aw3642_timeout_ms / 1000;
		ns = aw3642_timeout_ms % 1000 * 1000000;
		ktime = ktime_set(s, ns);
		hrtimer_start(&aw3642_timer, ktime, HRTIMER_MODE_REL);
		pr_info("hrtimer start");
	}
	aw3642_enable();
}

/* Timer and work queue */
static void aw3642_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	aw3642_disable();
}

static enum hrtimer_restart aw3642_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3642_work);
	return HRTIMER_NORESTART;
}

/* Flashlight operations */
static int aw3642_ioctl(unsigned int cmd, unsigned long arg)
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
		aw3642_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY %d: %d\n",
			channel, (int)fl_arg->arg);
		aw3642_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF %d: %d\n",
			channel, (int)fl_arg->arg);
		aw3642_set_flash_on_off(fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER %d\n", channel);
		fl_arg->arg = AW3642_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY %d\n", channel);
		fl_arg->arg = AW3642_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = aw3642_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT %d: %d\n",
			channel, (int)fl_arg->arg);
		fl_arg->arg = aw3642_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT %d\n", channel);
		fl_arg->arg = AW3642_MAX_TIMEOUT;
		break;

	case FLASH_IOC_GET_HW_FAULT:
		pr_debug("FLASH_IOC_GET_HW_FAULT %d\n", channel);
		fl_arg->arg = aw3642_get_flag();
		break;

	default:
		pr_err("No such command and arg %d:  %d, %d\n",
			channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3642_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3642_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3642_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw3642_mutex);
	if (set) {
		if (g_use_count == 0)
			ret = aw3642_init();
		g_use_count++;
		pr_debug("Set driver: %d\n", g_use_count);
	} else {
		g_use_count--;
		if (g_use_count == 0)
			ret = aw3642_uninit();
		if (g_use_count < 0)
			g_use_count = 0;
		pr_debug("Unset driver: %d\n", g_use_count);
	}
	mutex_unlock(&aw3642_mutex);

	return ret;
}

static ssize_t aw3642_strobe_store(struct flashlight_arg arg)
{
	aw3642_set_driver(1); /* set driver on */
	aw3642_set_level(arg.level);
	aw3642_timeout_ms = 0;
	aw3642_enable();
	msleep(arg.dur);
	aw3642_disable();
	aw3642_set_driver(0); /* set driver off */

	return 0;
}

static struct flashlight_operations aw3642_ops = {
	aw3642_open,
	aw3642_release,
	aw3642_ioctl,
	aw3642_strobe_store,
	aw3642_set_driver
};

/* I2C device and driver */
static int aw3642_match_id(void)
{
	int id;
	int val;

	id = aw3642_read_reg(
		aw3642_i2c_client, AW3642_REG_SILICON_REVISION);
	pr_info("aw3642 id %d\n", id);
	if (id != AW3642_CHIP_ID) {
		g_match_id = false;
		return -EINVAL;
	}
	g_match_id = true;

	(void)aw3642_disable();

	val = aw3642_read_reg(aw3642_i2c_client, AW3642_REG_IVFM);
	if (val >= 0) {
		val = ((u8)val & IVFM_MASK) | AW3642_IVFM_EN | AW3642_UVLO_EN | AW3642_IVFM_VOLTAGE;
		(void)aw3642_write_reg(aw3642_i2c_client, AW3642_REG_IVFM, (u8)val);
	}

	return 0;
}

static int aw3642_parse_dt(struct device *dev,
	struct aw3642_platform_data *pdata)
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
			strlen(AW3642_NAME), AW3642_NAME) < 0) {
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

static int aw3642_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw3642_chip_data *chip = NULL;
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
	chip = kzalloc(sizeof(struct aw3642_chip_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	i2c_set_clientdata(client, chip);
	aw3642_i2c_client = client;
	aw3642_i2c_client->addr = AW3642_SLAVE_ADDRESS;
	pr_info("aw3642 real i2c addr = 0x%x", aw3642_i2c_client->addr);

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	if (aw3642_match_id()) {
		ret = -ENODEV;
		pr_err("match chip failed\n");
		goto err_out;
	}

	pr_info("i2c probe done\n");

	return 0;

err_out:
	aw3642_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return ret;
}

static int aw3642_i2c_remove(struct i2c_client *client)
{
	struct aw3642_chip_data *chip = NULL;

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
	aw3642_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;

	pr_debug("Remove done\n");

	return 0;
}

static const struct i2c_device_id aw3642_i2c_id[] = {
	{ AW3642_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw3642_i2c_id);

static const struct of_device_id aw3642_i2c_of_match[] = {
	{ .compatible = AW3642_DTNAME_I2C },
	{},
};
MODULE_DEVICE_TABLE(of, aw3642_i2c_of_match);

static struct i2c_driver aw3642_i2c_driver = {
	.driver = {
		.name = AW3642_NAME,
		.of_match_table = aw3642_i2c_of_match,
	},
	.probe = aw3642_i2c_probe,
	.remove = aw3642_i2c_remove,
	.id_table = aw3642_i2c_id,
};

/* Platform device and driver */
static int aw3642_probe(struct platform_device *pdev)
{
	struct aw3642_platform_data *pdata = NULL;
	struct aw3642_chip_data *chip = NULL;
	int err;
	int i;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}
	pr_info("Probe start\n");

	g_aw3642_level = -1;
	if (i2c_add_driver(&aw3642_i2c_driver)) {
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
		err = aw3642_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_parse_dt;
	}

	/* init work queue */
	INIT_WORK(&aw3642_work, aw3642_work_disable);

	/* init timer */
	hrtimer_init(&aw3642_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3642_timer.function = aw3642_timer_func;
	aw3642_timeout_ms = AW3642_MAX_TIMEOUT;

	/* clear usage count */
	g_use_count = 0;

	/* register flashlight device */
	for (i = 0; i < pdata->channel_num; i++)
		if (flashlight_dev_register_by_device_id(
			&pdata->dev_id[i], &aw3642_ops)) {
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
	chip = i2c_get_clientdata(aw3642_i2c_client);
	i2c_set_clientdata(aw3642_i2c_client, NULL);
	if (chip) {
		kfree(chip);
		chip = NULL;
	}

	return err;
}

static int aw3642_remove(struct platform_device *pdev)
{
	int i;
	struct aw3642_platform_data *pdata = NULL;

	if (!pdev) {
		pr_err("pdev is null\n");
		return -ENODEV;
	}

	pr_debug("Remove start\n");

	/* flush work queue */
	flush_work(&aw3642_work);
	i2c_del_driver(&aw3642_i2c_driver);
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

static const struct of_device_id aw3642_of_match[] = {
	{ .compatible = AW3642_DTNAME },
	{},
};
MODULE_DEVICE_TABLE(of, aw3642_of_match);

static struct platform_driver aw3642_platform_driver = {
	.probe = aw3642_probe,
	.remove = aw3642_remove,
	.driver = {
		.name = AW3642_NAME,
		.owner = THIS_MODULE,
		.of_match_table = aw3642_of_match,
	},
};

static int __init flashlight_aw3642_init(void)
{
	int ret;

	pr_info("Init start\n");

	ret = platform_driver_register(&aw3642_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver, ret = %d\n", ret);
		return ret;
	}

	pr_info("Init done\n");

	return 0;
}

static void __exit flashlight_aw3642_exit(void)
{
	pr_debug("Exit start\n");

	platform_driver_unregister(&aw3642_platform_driver);

	pr_debug("Exit done\n");
}

late_initcall(flashlight_aw3642_init);
module_exit(flashlight_aw3642_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flashlight AW3642 Driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

