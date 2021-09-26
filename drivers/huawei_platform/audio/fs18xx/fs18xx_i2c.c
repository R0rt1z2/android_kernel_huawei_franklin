/**
 * Copyright (c) 2019-2020. All rights reserved.
 *Description: Core Defination For Foursemi Device .
 *Author: Fourier Semiconductor Inc.
 * Create: 2020-02-20 File created.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/i2c-dev.h>
#include "smartpakit.h"
// #include <huawei_platform/charger/huawei_charger.h>
#include "dsm_audio/dsm_audio.h"
#include "fsm_dev.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

static DEFINE_MUTEX(fsm_mutex);

void fsm_mutex_lock(void)
{
    mutex_lock(&fsm_mutex);
}

void fsm_mutex_unlock(void)
{
    mutex_unlock(&fsm_mutex);
}

static ssize_t fsm_misc_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    int ret = 0, retries = 20;
    uint8_t *tmp_data = NULL;
    struct miscdevice *dev = filp->private_data;
    fsm_dev_t *fsm_dev = NULL;

    if (count <= 0) {
        printk("%s count is error\n", __func__);
        return -EINVAL;
    }
    fsm_dev = container_of(dev, fsm_dev_t, fs18xx_misc);
    if(!fsm_dev) {
        printk("%s fsm_dev == NULL\n", __func__);
        return -EINVAL;
    }
    tmp_data = kmalloc(count, GFP_KERNEL);
    if (tmp_data == NULL)
        return -ENOMEM;
    do
    {
        fsm_mutex_lock();
        ret = i2c_master_recv(fsm_dev->client, tmp_data, count);
        fsm_mutex_unlock();
        if(ret != count)
            mdelay(5);
    }
    while((ret - count) && (--retries > 0));

    if(ret) {
        if(copy_to_user(buf, tmp_data, count))
            printk("%s, copy error.", __func__);
    } else {
        printk("%s, reading %zu bytes failed, ret: %d.", __func__, count, ret);
    }
    if(tmp_data) {
        kfree(tmp_data);
        tmp_data = NULL;
    }

    return ret;
}

static ssize_t fsm_misc_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    int ret = 0, retries = 20;
    char *tmp_data = NULL;
    struct miscdevice *dev = filp->private_data;
    fsm_dev_t *fsm_dev = NULL;

    if (count <= 0) {
        printk("%s count is error\n", __func__);
        return -EINVAL;
    }
    fsm_dev = container_of(dev, fsm_dev_t, fs18xx_misc);
    if(!fsm_dev) {
        printk("%s fsm_dev == NULL\n", __func__);
        return -EINVAL;
    }
    tmp_data = kmalloc(count, GFP_KERNEL);
    if (tmp_data == NULL)
        return -ENOMEM;
    if (copy_from_user(tmp_data, buf, count))
    {
        printk("%s, failed to copy from user space\n", __func__);
        kfree(tmp_data);
        tmp_data = NULL;
        return -EFAULT;
    }

    do
    {
        if (fsm_dev->client) {
            fsm_mutex_lock();
            ret = i2c_master_send(fsm_dev->client, tmp_data, count);
            fsm_mutex_unlock();
            ret = (ret == count) ? 0 : ret;
            if(ret)
                mdelay(5);
        }
    }
    while(ret && (--retries > 0));
    if(ret != 0)
        printk("%s, writing %zu bytes failed, ret: %d.", __func__, count, ret);
    
    if(tmp_data) {
        kfree(tmp_data);
        tmp_data = NULL;
    }

    return ret;
}

int fsm_i2c_reg_read(struct i2c_client *i2c, uint8_t reg, uint16_t *pVal)
{
    int ret = 0;
    uint8_t retries = 20;
    uint8_t buffer[2] = {0};
    struct i2c_msg msgs[2];

    if(i2c == NULL || pVal == NULL)
        return -EINVAL;

    // write register address.
    msgs[0].addr = i2c->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    // read register buffer.
    msgs[1].addr = i2c->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 2;
    msgs[1].buf = &buffer[0];

    do
    {
        fsm_mutex_lock();
        ret = i2c_transfer(i2c->adapter, &msgs[0], ARRAY_SIZE(msgs));
        fsm_mutex_unlock();
        if (ret != ARRAY_SIZE(msgs))
            mdelay(5);
    }
    while ((ret != ARRAY_SIZE(msgs)) && (--retries > 0));

    if (ret != ARRAY_SIZE(msgs))
    {
        printk("%s, read transfer error, ret: %d.", __func__, ret);
        return -EIO;
    }

    *pVal = ((buffer[0] << 8) | buffer[1]);

    return 0;
}

static int fsm_i2c_msg_write(struct i2c_client *i2c, uint8_t *buff, uint8_t len)
{
    int ret = 0;
    uint8_t retries = 20;

    struct i2c_msg msgs[] = {
        {
        .addr = i2c->addr,
        .flags = 0,
        .len = len + 1,
        .buf = buff,
        },
    };

    if (!buff)
        return -EINVAL;

    do
    {
        fsm_mutex_lock();
        ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
        fsm_mutex_unlock();
        if (ret != ARRAY_SIZE(msgs))
            mdelay(5);
    }
    while ((ret != ARRAY_SIZE(msgs)) && (--retries > 0));

    if (ret != ARRAY_SIZE(msgs))
    {
        printk("%s, write transfer error, ret: %d.", __func__, ret);
        return -EIO;
    }

    return ret;
}
static int fsm_misc_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static long fsm_misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    struct miscdevice *dev = filp->private_data;
    fsm_dev_t *fsm_dev = NULL;
    
    fsm_dev = container_of(dev, fsm_dev_t, fs18xx_misc);
    if(!fsm_dev) {
        printk("%s fsm_dev == NULL\n", __func__);
        return -EINVAL;
    }
    switch (cmd) {
        case I2C_SLAVE:
        case I2C_SLAVE_FORCE:
            if((arg == FSM_I2CADDR1) ||(arg == FSM_I2CADDR2) ||
                (arg == FSM_I2CADDR3) ||(arg == FSM_I2CADDR4)) {
                fsm_dev->client->addr= arg;
                ret = 0;
            } else {
                printk("%s :wrong i2c address\n",__func__);
                ret = -EINVAL;
            }
            break;
        default:
                printk("%s, bad ioctl parameters %u\n", __func__, cmd);
                ret = -EINVAL;
    }
          return ret;
}

static const struct file_operations fsm_file_ops =
{
    .owner    = THIS_MODULE,
    .open    = fsm_misc_open,
    .unlocked_ioctl = fsm_misc_ioctl,
#if defined(CONFIG_COMPAT)
    .compat_ioctl = fsm_misc_ioctl,
#endif
    .llseek    = no_llseek,
    .read    = fsm_misc_read,
    .write    = fsm_misc_write,
};

static int fsm_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int ret, i;
    uint16_t dev_id;
    struct smartpa_vendor_info vendor_info;
    uint16_t ndev = 0;
    fsm_dev_t *fsm_dev = NULL;

    printk("%s, enter.\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
    {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    for (i = 0; i < FSM_DEV_MAX; i++) {
        dev_id = 0;
        i2c->addr = FSM_DEV_BASE + i;
        ret = fsm_i2c_reg_read(i2c, FS1603_ID_REG, &dev_id);
        printk("%s, addr: 0x%x, device id :0x%x.\n", __func__, i2c->addr, dev_id);
        if (((dev_id & 0xFF00) >> 8) == FS1603_DEV_ID) {
            printk("%s, fs18xx detected.\n", __func__);
            ndev++;
        }
    }
    if (ndev == 0) {
        printk("%s, not foursemi device, return", __func__);
        return -EINVAL;
    }
    vendor_info.vendor = FSM_VENDOR_ID;
    vendor_info.chip_model = "fs18xx";
    smartpakit_set_info(&vendor_info);
    fsm_dev = devm_kzalloc(&i2c->dev, sizeof(fsm_dev_t), GFP_KERNEL);
    if (fsm_dev == NULL)
        return -ENOMEM;

    fsm_dev->client = i2c;
    fsm_dev->dev = &i2c->dev;
    fsm_dev->i2c_addr = i2c->addr;
    fsm_dev->fs18xx_misc.minor = MISC_DYNAMIC_MINOR;
    fsm_dev->fs18xx_misc.name = FSDEV_I2C_NAME;
    fsm_dev->fs18xx_misc.fops = &fsm_file_ops;
    i2c_set_clientdata(i2c, fsm_dev);
     ret = misc_register(&fsm_dev->fs18xx_misc);
     if(ret != 0)
         printk("%s, misc register failed, ret: %d.", __func__, ret);

    printk("%s, i2c probe completed", __func__);

    return ret;
}

static int fsm_i2c_remove(struct i2c_client *i2c)
{
    fsm_dev_t *fsm_dev = i2c_get_clientdata(i2c);
    
    misc_deregister(&fsm_dev->fs18xx_misc);
    mutex_destroy(&fsm_mutex);
    devm_kfree(&i2c->dev, fsm_dev);

    return 0;
}

static const struct i2c_device_id fsm_i2c_id[] =
{
    { FSDEV_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, fsm_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id fsm_match_tbl[] =
{
    { .compatible = "foursemi,fs18xx" },
    { },
};
MODULE_DEVICE_TABLE(of, fsm_match_tbl);
#endif

static struct i2c_driver fsm_i2c_driver =
{
    .driver =
    {
        .name = FSDEV_I2C_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(fsm_match_tbl),
#endif
    },
    .probe =    fsm_i2c_probe,
    .remove =   fsm_i2c_remove,
    .id_table = fsm_i2c_id,
};

static int __init fsm_mod_init(void)
{
    int ret = 0;
    ret = i2c_add_driver(&fsm_i2c_driver);
    return ret;
}

static void __exit fsm_mod_exit(void)
{
    i2c_del_driver(&fsm_i2c_driver);
}

//module_init(fsm_mod_init);
late_initcall_sync(fsm_mod_init);
module_exit(fsm_mod_exit);

MODULE_AUTHOR("FourSemi SW <support@foursemi.com>");
MODULE_DESCRIPTION("FourSemi Smart PA i2c driver");
MODULE_LICENSE("GPL");
