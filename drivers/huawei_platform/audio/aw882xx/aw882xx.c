/*
 * aw882xx.c   aw882xx codec module
 *
 * Version: v1.0.13
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include <linux/uaccess.h>
#include "aw882xx_reg.h"
#include "aw882xx_misc.h"
#include "aw882xx.h"
#include "smartpakit.h"
#ifdef CONFIG_HUAWEI_DSM_AUDIO
#include "dsm_audio/dsm_audio.h"
#endif

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW882XX_I2C_NAME "aw882xx_smartpa"

#define AW882XX_VERSION "hw_v1.0.13"

#define AW_I2C_RETRIES            5    /* 5 times */
#define AW_I2C_RETRY_DELAY        5    /* 5 ms */
#define AW_READ_CHIPID_RETRIES        5    /* 5 times */
#define AW_READ_CHIPID_RETRY_DELAY    5    /* 5 ms */

#define AW_CHIP_VENDOR (6)
#define AW_CHIP_MODEL "aw882xx"

#define DSM_BUF_SIZE                    (1024)
#define AWINIC_ERROR_INFO_TAG           "[awinic error report]"

static void awinic_dsm_report_by_i2c_error(struct aw882xx *aw882xx);

/******************************************************
 *
 * aw882xx append suffix sound channel information
 *
 ******************************************************/
static void *aw882xx_devm_kstrdup(struct device *dev, const char *buf, unsigned int len)
{
    char *str;

    str = devm_kzalloc(dev, len + 1, GFP_KERNEL);
    if (str == NULL)
        return str;

    memcpy(str, buf, strlen(buf));
    return str;
}

void aw882xx_append_suffix(const char *format, const char **change_name,
    struct aw882xx *aw882xx)
{
    char buf[50];

    if (!aw882xx->chan_info.name_suffix)
        return;

    snprintf(buf, 50, format, *change_name, aw882xx->chan_info.name_suffix);
    *change_name = aw882xx_devm_kstrdup(aw882xx->dev, buf, 50);
    aw_dev_dbg(aw882xx->dev, "%s:change name :%s\n",
        __func__, *change_name);
}



/******************************************************
 *
 * aw882xx i2c write/read
 *
 ******************************************************/
static int aw882xx_i2c_writes(struct aw882xx *aw882xx,
    unsigned char reg_addr, const unsigned char *buf, unsigned int len)
{
    int ret = -1;
    unsigned char *data = NULL;
    
    if(len > 0)
        data = kmalloc(len+1, GFP_KERNEL);
    if (data == NULL) {
        aw_dev_err(aw882xx->dev, "%s: can not allocate memory\n",
            __func__);
        return -ENOMEM;
    }

    data[0] = reg_addr;
    memcpy(&data[1], buf, len);

    ret = i2c_master_send(aw882xx->i2c, data, len+1);
    if (ret < 0) {
        aw_dev_err(aw882xx->dev,
            "%s: i2c master send error\n", __func__);
        awinic_dsm_report_by_i2c_error(aw882xx);
    }

    kfree(data);

    return ret;
}

static int aw882xx_i2c_reads(struct aw882xx *aw882xx,
    unsigned char reg_addr, unsigned char *data_buf, unsigned int data_len)
{
    int ret;
    struct i2c_msg msg[] = {
        [0] = {
            .addr = aw882xx->i2c->addr,
            .flags = 0,
            .len = sizeof(uint8_t),
            .buf = &reg_addr,
            },
        [1] = {
            .addr = aw882xx->i2c->addr,
            .flags = I2C_M_RD,
            .len = data_len,
            .buf = data_buf,
            },
    };

    ret = i2c_transfer(aw882xx->i2c->adapter, msg, ARRAY_SIZE(msg));
    if (ret < 0) {
        pr_err("%s: i2c master send error, ret=%d\n",
            __func__, ret);
        return ret;
    } else if (ret != AW882XX_I2C_READ_MSG_NUM) {
        pr_err("%s: couldn't read registers, return %d bytes\n",
            __func__, ret);
        awinic_dsm_report_by_i2c_error(aw882xx);
        return -1;
    }

    return 0;
}

int aw882xx_i2c_write(struct aw882xx *aw882xx,
    unsigned char reg_addr, unsigned int reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char buf[2];

    buf[0] = (reg_data&0xff00)>>8;
    buf[1] = (reg_data&0x00ff)>>0;

    while (cnt < AW_I2C_RETRIES) {
        ret = aw882xx_i2c_writes(aw882xx, reg_addr, buf, 2);
        if (ret < 0)
            aw_dev_err(aw882xx->dev, "%s: i2c_write cnt=%d error=%d\n",
                __func__, cnt, ret);
        else
            break;
        cnt++;
    }

    return ret;
}

int aw882xx_i2c_read(struct aw882xx *aw882xx,
    unsigned char reg_addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char buf[2];

    while (cnt < AW_I2C_RETRIES) {
        ret = aw882xx_i2c_reads(aw882xx, reg_addr, buf, 2);
        if (ret < 0) {
            aw_dev_err(aw882xx->dev, "%s: i2c_read cnt=%d error=%d\n",
                __func__, cnt, ret);
        } else {
            *reg_data = (buf[0]<<8) | (buf[1]<<0);
            break;
        }
        cnt++;
    }

    return ret;
}

int aw882xx_i2c_read_sigle(struct aw882xx *aw882xx,
    unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char buf[2];

    while (cnt < AW_I2C_RETRIES) {
        ret = aw882xx_i2c_reads(aw882xx, reg_addr, buf, 2);
        if (ret < 0) {
            aw_dev_err(aw882xx->dev, "%s: i2c_read cnt=%d error=%d\n",
                __func__, cnt, ret);
        } else {
            *reg_data = buf[0];
            *(reg_data + 1) = buf[1];
            break;
        }
        cnt++;
    }

    return ret;
}

/******************************************************
 *
 * aw882xx error info report
 *
 ******************************************************/
static void awinic_dsm_report_by_i2c_error(struct aw882xx *aw882xx)
{
    char *report = NULL;

    report = kzalloc(sizeof(char) * DSM_BUF_SIZE, GFP_KERNEL);
    if (!report)
           return;

    /* Splicing Splicing error message */
    sprintf(report, "%s dev_addr = 0x%x, i2c rw error\n",
            AWINIC_ERROR_INFO_TAG, aw882xx->i2c->addr);

    pr_info("%s", report);
#ifdef CONFIG_HUAWEI_DSM_AUDIO
	pr_info("%s: dsm report, %s\n", __func__, report);
	audio_dsm_report_info(AUDIO_SMARTPA, DSM_SMARTPA_I2C_ERR,
		"smartpakit_awinic", report);
#endif
    kfree(report);
}


static void awinic_i2c_handler_irq_dsm_report(struct aw882xx *aw882xx)
{
    char *report = NULL;
    unsigned int reg = 0;

    /* get sysint */
    aw882xx_i2c_read(aw882xx, AW882XX_SYSINT_REG, &reg);

    report = kzalloc(sizeof(char) * DSM_BUF_SIZE, GFP_KERNEL);
    if (report == NULL) {
        pr_err("%s malloc mem failed\n", __func__);
        return;
    }

    sprintf(report, "%s dev_addr = 0x%x, reg[0x02] = 0x%04x",
                     AWINIC_ERROR_INFO_TAG, aw882xx->i2c->addr, reg);
#ifdef CONFIG_HUAWEI_DSM_AUDIO
	pr_info("%s: dsm report, %s\n", __func__, report);
	audio_dsm_report_info(AUDIO_SMARTPA, DSM_SMARTPA_INT_ERR,
		"smartpakit_awinic", report);
#endif
    kfree(report);
}
#if 0
static int aw882xx_i2c_write_bits(struct aw882xx *aw882xx,
    unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
    int ret = -1;
    unsigned int reg_val = 0;

    ret = aw882xx_i2c_read(aw882xx, reg_addr, &reg_val);
    if (ret < 0) {
        aw_dev_err(aw882xx->dev,
            "%s: i2c read error, ret=%d\n", __func__, ret);
        return ret;
    }
    reg_val &= mask;
    reg_val |= reg_data;
    ret = aw882xx_i2c_write(aw882xx, reg_addr, reg_val);
    if (ret < 0) {
        aw_dev_err(aw882xx->dev,
            "%s: i2c read error, ret=%d\n", __func__, ret);
        return ret;
    }

    return 0;
}
#endif

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw882xx_interrupt_setup(struct aw882xx *aw882xx)
{

    aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

    aw882xx_i2c_write(aw882xx, AW882XX_SYSINTM_REG, AW882XX_SYSINTM_DEFAULT);
}

static void aw882xx_interrupt_clear(struct aw882xx *aw882xx)
{
    unsigned int reg_val = 0;

    aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

    aw882xx_i2c_read(aw882xx, AW882XX_SYSST_REG, &reg_val);
    aw_dev_info(aw882xx->dev, "%s: reg SYSST=0x%x\n", __func__, reg_val);

    aw882xx_i2c_read(aw882xx, AW882XX_SYSINT_REG, &reg_val);
    aw_dev_info(aw882xx->dev, "%s: reg SYSINT=0x%x\n", __func__, reg_val);

    aw882xx_i2c_read(aw882xx, AW882XX_SYSINTM_REG, &reg_val);
    aw_dev_info(aw882xx->dev, "%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw882xx_irq(int irq, void *data)
{
    struct aw882xx *aw882xx = data;

    aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

    if (!aw882xx->is_first_poweron) {
        awinic_i2c_handler_irq_dsm_report(aw882xx);
    } else {
        aw882xx->is_first_poweron = false;
    }

    aw882xx_interrupt_clear(aw882xx);

    aw_dev_info(aw882xx->dev, "%s: exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static void aw882xx_parse_gpio_dt(struct aw882xx *aw882xx,
    struct device_node *np)
{
    aw882xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw882xx->reset_gpio < 0) {
        dev_err(aw882xx->dev, "%s: no reset gpio provided, will not HW reset device\n",
            __func__);
    } else {
        dev_info(aw882xx->dev, "%s: reset gpio provided ok\n",
            __func__);
    }
    aw882xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
    if (aw882xx->irq_gpio < 0)
        dev_err(aw882xx->dev, "%s: no irq gpio provided.\n", __func__);
    else
        dev_info(aw882xx->dev, "%s: irq gpio provided ok.\n", __func__);
}

static void aw882xx_parse_channel_dt(struct aw882xx *aw882xx,
    struct device_node *np)
{
    int ret;
    const char *channel_value = NULL;
    struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

    chan_info->channel = AW882XX_CHANNLE_LEFT_MONO;
    chan_info->name_suffix = NULL;
    ret = of_property_read_string(np, "sound-channel", &channel_value);
    if (ret < 0) {
        dev_info(aw882xx->dev,
            "%s:read sound-channel failed,use default\n", __func__);
        return;
    }
    dev_dbg(aw882xx->dev,
        "%s: read sound-channel value is : %s\n",
        __func__, channel_value);

    if (!strcmp(channel_value, "left")) {
        chan_info->name_suffix = "l";
    } else if (!strcmp(channel_value, "right")) {
        chan_info->channel = AW882XX_CHANNLE_RIGHT;
        chan_info->name_suffix = "r";
    } else {
        dev_info(aw882xx->dev, "%s:not stereo channel,use default single track\n",
            __func__);
    }
}

static int aw882xx_parse_dt(struct device *dev, struct aw882xx *aw882xx,
        struct device_node *np)
{
    aw882xx_parse_gpio_dt(aw882xx, np);
    aw882xx_parse_channel_dt(aw882xx, np);

    return 0;
}

int aw882xx_hw_reset(struct aw882xx *aw882xx)
{
    pr_info("%s: enter\n", __func__);

    if (aw882xx == NULL)
	    return -1;
    if (gpio_is_valid(aw882xx->reset_gpio)) {
        gpio_set_value_cansleep(aw882xx->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw882xx->reset_gpio, 1);
        msleep(2);
    } else {
        dev_err(aw882xx->dev, "%s: failed\n", __func__);
    }
    return 0;
}

int aw882xx_hw_reset_deinit(struct aw882xx *aw882xx)
{
    pr_info("%s: enter\n", __func__);

    if (aw882xx == NULL)
	    return -1;
    if (gpio_is_valid(aw882xx->reset_gpio)) {
        gpio_set_value_cansleep(aw882xx->reset_gpio, 0);
    } else {
        dev_err(aw882xx->dev, "%s: failed\n", __func__);
    }
    return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw882xx_read_chipid(struct aw882xx *aw882xx)
{
    int ret = -1;
    unsigned int cnt = 0;
    unsigned int reg = 0;

    while (cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw882xx_i2c_read(aw882xx, AW882XX_ID_REG, &reg);
        if (ret < 0) {
            dev_err(aw882xx->dev,
                "%s: failed to read REG_ID: %d\n",
                __func__, ret);
            return -EIO;
        }
        switch (reg) {
        case AW882XX_ID:
            dev_info(aw882xx->dev, "%s: aw882xx detected\n",
                __func__);
            aw882xx->flags |= AW882XX_FLAG_SKIP_INTERRUPTS;
            aw882xx->flags |= AW882XX_FLAG_START_ON_MUTE;
            aw882xx->chipid = AW882XX_ID;
            dev_info(aw882xx->dev, "%s: aw882xx->flags=0x%x\n",
                __func__, aw882xx->flags);
            return 0;
        default:
            dev_info(aw882xx->dev, "%s: unsupported device revision (0x%x)\n",
                __func__, reg);
            break;
        }
        cnt++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw882xx_reg_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);
    unsigned int databuf[2] = {0};

    if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
        aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);

    return count;
}

static ssize_t aw882xx_reg_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned int reg_val = 0;

    for (i = 0; i < AW882XX_REG_MAX; i++) {
        if (aw882xx_reg_access[i]&REG_RD_ACCESS) {
            aw882xx_i2c_read(aw882xx, i, &reg_val);
            len += snprintf(buf+len, PAGE_SIZE-len,
                "reg:0x%02x=0x%04x\n", i, reg_val);
        }
    }

    return len;
}

static ssize_t aw882xx_rw_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw882xx->reg_addr = (unsigned char)databuf[0];
        aw882xx_i2c_write(aw882xx, databuf[0], databuf[1]);
    } else if (1 == sscanf(buf, "%x", &databuf[0])) {
        aw882xx->reg_addr = (unsigned char)databuf[0];
    }

    return count;
}

static ssize_t aw882xx_rw_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int reg_val = 0;

    if (aw882xx_reg_access[aw882xx->reg_addr] & REG_RD_ACCESS) {
        aw882xx_i2c_read(aw882xx, aw882xx->reg_addr, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len,
            "reg:0x%02x=0x%04x\n", aw882xx->reg_addr, reg_val);
    }
    return len;
}


static ssize_t aw882xx_dsp_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);
    unsigned int databuf[2] = {0};

    int32_t test[10];

    test[6] = 1;
   if (1 == sscanf(buf, "%d", &databuf[0])) {
        aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_NOISE, test, sizeof(int32_t) * 10, aw882xx->chan_info.channel);
    }

    return count;
}

static ssize_t aw882xx_dsp_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct aw882xx *aw882xx = dev_get_drvdata(dev);
    ssize_t len = 0;
    int32_t test[10] = {0};

    aw_read_data_to_dsp(INDEX_PARAMS_ID_RX_F0, test, sizeof(int32_t) * 10, aw882xx->chan_info.channel);

    len += snprintf(buf+len, PAGE_SIZE-len,
                "f0 = %d\n", test[0]);

    return len;
}


static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
    aw882xx_reg_show, aw882xx_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO,
    aw882xx_rw_show, aw882xx_rw_store);

static DEVICE_ATTR(dsp, S_IWUSR | S_IRUGO,
    aw882xx_dsp_show, aw882xx_dsp_store);


static struct attribute *aw882xx_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_rw.attr,
    &dev_attr_dsp.attr,
    NULL
};

static struct attribute_group aw882xx_attribute_group = {
    .attrs = aw882xx_attributes,
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw882xx_i2c_probe(struct i2c_client *i2c,
    const struct i2c_device_id *id)
{
    struct aw882xx *aw882xx = NULL;
    struct device_node *np = i2c->dev.of_node;
    struct aw882xx_chan_info *chan_info = NULL;
    const char *aw882xx_rst = "aw882xx_rst";
    const char *aw882xx_int = "aw882xx_int";
    const char *aw882xx_irq_name = "aw882xx";
    int irq_flags = 0;
    int ret;
    struct smartpa_vendor_info vendor_info;
    
    pr_info("%s: enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw882xx = devm_kzalloc(&i2c->dev, sizeof(struct aw882xx), GFP_KERNEL);
    if (aw882xx == NULL)
        return -ENOMEM;

    aw882xx->dev = &i2c->dev;
    aw882xx->i2c = i2c;
    chan_info = &aw882xx->chan_info;
    i2c_set_clientdata(i2c, aw882xx);
    mutex_init(&aw882xx->lock);

    /* aw882xx rst & int */
    if (np != NULL) {
        ret = aw882xx_parse_dt(&i2c->dev, aw882xx, np);
        if (ret) {
            dev_err(&i2c->dev, "%s: failed to parse device tree node\n",
                __func__);
            goto err_parse_dt;
        }
    } else {
        aw882xx->reset_gpio = -1;
        aw882xx->irq_gpio = -1;
    }

    if (gpio_is_valid(aw882xx->reset_gpio)) {
        aw882xx_append_suffix("%s_%s", &aw882xx_rst, aw882xx);
        ret = devm_gpio_request_one(&i2c->dev, aw882xx->reset_gpio,
            GPIOF_OUT_INIT_LOW, aw882xx_rst);
        if (ret) {
            dev_err(&i2c->dev, "%s: rst request failed\n",
                __func__);
            goto err_reset_gpio_request;
        }
    }

    if (gpio_is_valid(aw882xx->irq_gpio)) {
        aw882xx_append_suffix("%s_%s", &aw882xx_int, aw882xx);
        ret = devm_gpio_request_one(&i2c->dev, aw882xx->irq_gpio,
            GPIOF_DIR_IN, aw882xx_int);
        if (ret) {
            dev_err(&i2c->dev, "%s: int request failed\n",
                __func__);
            goto err_irq_gpio_request;
        }
    }

    /* hardware reset */
    aw882xx_hw_reset(aw882xx);
    aw882xx->is_first_poweron = true;
    
    /* set real address */
    if(i2c->addr == 0x75) {
        i2c->addr = 0x35;
    } else if(i2c->addr == 0x74) {
        i2c->addr = 0x34;
    }
    aw882xx->misc_device.name = "aw882xx_smartpa";

    /* aw882xx chip id */
    ret = aw882xx_read_chipid(aw882xx);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw882xx_read_chipid failed ret=%d\n",
            __func__, ret);
        goto err_id;
    }

    /* aw882xx device name */
    if (i2c->dev.of_node) {
        if (chan_info->name_suffix != NULL) {
            dev_set_name(&i2c->dev, "%s_%s", "aw882xx_smartpa",
                chan_info->name_suffix);
            aw882xx_append_suffix("%s_%s", &aw882xx->misc_device.name, aw882xx);
        }
        else
            dev_set_name(&i2c->dev, "%s", "aw882xx_smartpa");
    } else {
        dev_err(&i2c->dev, "%s failed to set device name: %d\n",
            __func__, ret);
    }

    /* aw882xx irq */
    if (gpio_is_valid(aw882xx->irq_gpio) &&
        !(aw882xx->flags & AW882XX_FLAG_SKIP_INTERRUPTS)) {
        /* register irq handler */
        aw882xx_interrupt_setup(aw882xx);

        /* clear irq */
        aw882xx_interrupt_clear(aw882xx);

        aw882xx_append_suffix("%s_%s", &aw882xx_irq_name, aw882xx);

        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = devm_request_threaded_irq(&i2c->dev,
                gpio_to_irq(aw882xx->irq_gpio),
                NULL, aw882xx_irq, irq_flags,
                aw882xx_irq_name, aw882xx);
        if (ret != 0) {
            aw_dev_err(aw882xx->dev, "failed to request IRQ %d: %d\n",
                gpio_to_irq(aw882xx->irq_gpio), ret);
            goto err_irq;
        }
    } else {
        aw_dev_info(aw882xx->dev, "%s skipping IRQ registration\n",
            __func__);
        /* disable feature support if gpio was invalid */
        aw882xx->flags |= AW882XX_FLAG_SKIP_INTERRUPTS;
    }

    dev_set_drvdata(&i2c->dev, aw882xx);
    ret = sysfs_create_group(&i2c->dev.kobj, &aw882xx_attribute_group);
    if (ret < 0) {
        aw_dev_err(aw882xx->dev, "%s error creating sysfs attr files\n",
            __func__);
        goto err_sysfs;
    }

    aw882xx_misc_init(&aw882xx->misc_device);
    vendor_info.vendor = AW_CHIP_VENDOR;
    vendor_info.chip_model = AW_CHIP_MODEL;
    ret = smartpakit_set_info(&vendor_info);
    if(ret) {
        pr_err("%s: set smartpa info err %d",__func__,ret);
        goto err_sysfs;
    }

    aw_dev_dbg(aw882xx->dev, "%s: probe completed successfully!\n",
        __func__);

    return 0;


err_sysfs:
    devm_free_irq(&i2c->dev, gpio_to_irq(aw882xx->irq_gpio), aw882xx);
err_irq:
err_id:
    aw882xx_hw_reset_deinit(aw882xx);
    if (gpio_is_valid(aw882xx->irq_gpio))
        devm_gpio_free(&i2c->dev, aw882xx->irq_gpio);
err_irq_gpio_request:
    if (gpio_is_valid(aw882xx->reset_gpio))
        devm_gpio_free(&i2c->dev, aw882xx->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
    devm_kfree(&i2c->dev, aw882xx);
    aw882xx = NULL;

    return ret;
}

static int aw882xx_i2c_remove(struct i2c_client *i2c)
{
    struct aw882xx *aw882xx = i2c_get_clientdata(i2c);

    aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

    aw882xx_misc_deinit(&aw882xx->misc_device);

    if (gpio_to_irq(aw882xx->irq_gpio))
        devm_free_irq(&i2c->dev,
            gpio_to_irq(aw882xx->irq_gpio),
            aw882xx);


    if (gpio_is_valid(aw882xx->irq_gpio))
        devm_gpio_free(&i2c->dev, aw882xx->irq_gpio);
    if (gpio_is_valid(aw882xx->reset_gpio))
        devm_gpio_free(&i2c->dev, aw882xx->reset_gpio);

    devm_kfree(&i2c->dev, aw882xx);
    aw882xx = NULL;

    return 0;
}

static const struct i2c_device_id aw882xx_i2c_id[] = {
    { AW882XX_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw882xx_i2c_id);

static struct of_device_id aw882xx_dt_match[] = {
    { .compatible = "awinic,aw882xx_smartpa" },
    { .compatible = "awinic,aw882xx_smartpa_l" },
    { .compatible = "awinic,aw882xx_smartpa_r" },
    { },
};

static struct i2c_driver aw882xx_i2c_driver = {
    .driver = {
        .name = AW882XX_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw882xx_dt_match),
    },
    .probe = aw882xx_i2c_probe,
    .remove = aw882xx_i2c_remove,
    .id_table = aw882xx_i2c_id,
};


static int __init aw882xx_i2c_init(void)
{
    int ret = -1;

    pr_info("%s: aw882xx driver version %s\n", __func__, AW882XX_VERSION);

    ret = i2c_add_driver(&aw882xx_i2c_driver);
    if (ret)
        pr_err("%s: fail to add aw882xx device into i2c\n", __func__);

    return ret;
}
late_initcall_sync(aw882xx_i2c_init);


static void __exit aw882xx_i2c_exit(void)
{
    i2c_del_driver(&aw882xx_i2c_driver);
}
module_exit(aw882xx_i2c_exit);


MODULE_DESCRIPTION("ASoC AW882XX Smart PA Driver");
MODULE_LICENSE("GPL v2");
