/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:  headfile of adsp_chr_interface.c
 * Author: Shengwei Zhang
 * Create: 2019-03-31
 * History: NA
 */
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <sound/jack.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <securec.h>
#include "adsp_chr.h"

#define HWLOG_TAG adsp_chr

struct adsp_chr_data {
    struct mutex notifier_lock;
};

static struct adsp_chr_data *g_adsp_chr_pdata;

static const struct of_device_id g_adsp_chr_of_match[] = {
    {
        .compatible = "huawei,adsp_chr",
    },
    { },
};

MODULE_DEVICE_TABLE(of, g_adsp_chr_of_match);

static int adsp_chr_notifier_call(unsigned long event, char *str);
char g_envp_hal[ENVP_LENTH + 1] = {0};

static long adsp_chr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret;
    char *wakeInfo = NULL;

    printk("%s: adsp_chr ioctl enter, data addr: %ld\n", __func__, arg);
    if (g_adsp_chr_pdata == NULL) {
        printk("%s: adsp_chr ioctl error: g_adsp_chr_pdata = null\n", __func__);
        return -EBUSY;
    }

    switch (cmd) {
        case ADSP_CHR_REPORT_EVENT:
            wakeInfo = (char *)(uintptr_t)arg;
            copy_from_user(g_envp_hal, wakeInfo, ENVP_LENTH);
            printk("%s: adsp_chr info event(%s)\n", __func__, g_envp_hal);
            ret = adsp_chr_notifier_call(1, g_envp_hal);
            break;
        default:
            printk("%s: unsupport cmd\n", __func__);
            ret = -EINVAL;
            break;
    }

    return (long)ret;
}

static const struct file_operations g_adsp_chr_fops = {
    .owner           = THIS_MODULE,
    .open            = simple_open,
    .unlocked_ioctl  = adsp_chr_ioctl,
    .compat_ioctl    = adsp_chr_ioctl,
};

static struct miscdevice g_adsp_chr_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "adsp_chr",
    .fops  = &g_adsp_chr_fops,
};

static int adsp_chr_notifier_call(unsigned long event, char *str)
{
    int ret;
    char envp_ext0[ENVP_LENTH];
    char *envp_ext[ENVP_EXT_MEMBER] = { envp_ext0, NULL };

    printk("%s: adsp_chr_notifier_call: adsp_chr is %lu, str is %s\n", __func__, event, str);
    mutex_lock(&g_adsp_chr_pdata->notifier_lock);
    ret = snprintf_s(envp_ext0, ENVP_LENTH, (ENVP_LENTH - 1), str);
    if (ret < 0) {
        printk("%s: snprintf_s failed, ret = %d\n", __func__, ret);
        mutex_unlock(&g_adsp_chr_pdata->notifier_lock);
        return ret;
    }

    kobject_uevent_env(&g_adsp_chr_miscdev.this_device->kobj, KOBJ_CHANGE, envp_ext);
    mutex_unlock(&g_adsp_chr_pdata->notifier_lock);

    return 0;
}

static int adsp_chr_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;

    printk("%s: adsp_chr probe enter\n", __func__);
    g_adsp_chr_pdata = devm_kzalloc(dev, sizeof(*g_adsp_chr_pdata), GFP_KERNEL);
    if (g_adsp_chr_pdata == NULL) {
        printk("%s: cannot allocate usb_audio_common data\n", __func__);
        return -ENOMEM;
    }
    mutex_init(&g_adsp_chr_pdata->notifier_lock);

    ret = misc_register(&g_adsp_chr_miscdev);
    if (ret != 0) {
        printk("%s: can't register adsp_chr miscdev, ret:%d\n", __func__, ret);
        goto err_out;
    }
    printk("%s: adsp_chr probe success\n", __func__);

    return 0;

err_out:
    misc_deregister(&g_adsp_chr_miscdev);
    devm_kfree(dev, g_adsp_chr_pdata);
    g_adsp_chr_pdata = NULL;

    return ret;
}

static int adsp_chr_remove(struct platform_device *pdev)
{
    if (g_adsp_chr_pdata != NULL) {
        printk("%s: adsp_chr free\n", __func__);
        devm_kfree(&pdev->dev, g_adsp_chr_pdata);
        g_adsp_chr_pdata = NULL;
    }

    misc_deregister(&g_adsp_chr_miscdev);

    printk("%s: exit\n", __func__);

    return 0;
}

static struct platform_driver g_adsp_chr_driver = {
    .driver = {
        .name           = "adsp_chr",
        .owner          = THIS_MODULE,
        .of_match_table = g_adsp_chr_of_match,
    },
    .probe  = adsp_chr_probe,
    .remove = adsp_chr_remove,
};

static int __init adsp_chr_init(void)
{
    int ret;
    ret = platform_driver_register(&g_adsp_chr_driver);
    if (ret > 0) {
        printk("%s: adsp_chr driver register failed\n", __func__);
        return ret;
    }
    printk("%s: adsp_chr driver register succeed\n", __func__);
    return ret;
}

static void __exit adsp_chr_exit(void)
{
    platform_driver_unregister(&g_adsp_chr_driver);
}

module_init(adsp_chr_init);
module_exit(adsp_chr_exit);

MODULE_DESCRIPTION("adsp_chr control driver");
MODULE_LICENSE("GPL v2");
