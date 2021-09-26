/*
 * antenna_cable_state.c
 *
 * check antenna cable state
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#include <linux/iio/consumer.h>

#define DRV_NAME "antenna_cable"
#define TAG "antenna_cable: "

#define UEVENT_ENVP_LEN 2
#define UEVENT_BUFFER_LEN 50
#define DEBOUNCE_DEFAULT_VALUE 5
#define MAX_ANT_DET_GPIO_NUM 3
#define DEV_ATTR_ACCESS_MODE 0440
#define DEFAULT_VOLTAGE 1350

/* g_func_type 0:adc and gpio function 1:GPIO directly function */
static int g_func_type = 0;
static int g_voltage;
static int g_gpio_count;
static int g_gpio[MAX_ANT_DET_GPIO_NUM] = {0};
static int g_debounce_interval = DEBOUNCE_DEFAULT_VALUE;
static int g_irq[MAX_ANT_DET_GPIO_NUM] = {0};
static struct pinctrl *antenna_pinctrl;
static struct pinctrl_state *pin_state[MAX_ANT_DET_GPIO_NUM] = {};
static char *gpio_state_table[MAX_ANT_DET_GPIO_NUM] = {
	"id0", "id1", "id2"
};
struct iio_channel *cable_adc_channel = NULL;

struct ant_device {
	struct device *dev;
	struct device *dev_adc;
	struct class *class;
	dev_t devno;
	struct delayed_work dwork;
	/* previous cable state */
	unsigned int prev_state;
	/* current cable state */
	unsigned int curr_state;
};

/*
 * the return value is binary of (GPIO0GPIO1...), GPIOX stands for
 * GPIOX level, for example all cables not not connected, return 0.
 */
static int ant_state_get(const struct ant_device *antdev)
{
	unsigned int gpio_val = 0;
	unsigned int temp_val = 0;
	unsigned int i;

	for (i = 0; i < g_gpio_count; i++) {
		temp_val = gpio_get_value(g_gpio[i]);
		gpio_val += (temp_val << i);
	}
	if ((g_voltage > DEFAULT_VOLTAGE) && (gpio_val != 0) && (g_func_type != 1))
		return -1;
	return gpio_val;
}

static void ant_event_report(struct ant_device *antdev)
{
	char *uevent_envp[UEVENT_ENVP_LEN] = {NULL};
	char buf[UEVENT_BUFFER_LEN] = {0};
	int ret;

	if (ant_state_get(antdev)) {
		/* any cable in position, need do sar backoff */
		antdev->curr_state = 1;
		if ((g_voltage > DEFAULT_VOLTAGE) && (g_func_type != 1))
			pr_info(TAG "adc is not in position.\n");
		pr_info(TAG "antenna cable is not in position.\n");
	} else {
		/* no cable in position, restore to default state */
		antdev->curr_state = 0;
		pr_info(TAG "antenna cable is in position.\n");
	}

	/* report uevent if previous state is not equal to current state */
	if ((antdev->prev_state != antdev->curr_state) &&
		(antdev->curr_state == 1)) {
		uevent_envp[1] = NULL;

		snprintf(buf, PAGE_SIZE, "ANTENNA_CABLE_STATE=%u",
			antdev->curr_state);
		if ((g_voltage > DEFAULT_VOLTAGE) && (g_func_type != 1))
			snprintf(buf, PAGE_SIZE, "adc=%d", g_voltage);
		uevent_envp[0] = buf;
		pr_info(TAG "send uevent, %s\n", uevent_envp[0]);

		/* notify the uplayer to do sar backoff or restore */
		ret = kobject_uevent_env(&antdev->dev->kobj,
			KOBJ_CHANGE, uevent_envp);
		if (ret < 0)
			pr_err(TAG "send uevent failed, ret = %d\n", ret);
		else
			pr_info(TAG "send uevent, %s\n", uevent_envp[0]);
	}

			/* save the current state */
		antdev->prev_state = antdev->curr_state;
}

static void ant_det_work_func(struct work_struct *work)
{
	struct ant_device *antdev =
		container_of(work, struct ant_device, dwork.work);

	ant_event_report(antdev);
}

static irqreturn_t ant_pull_in_out_irq(int irq, void *dev_id)
{
	struct ant_device *antdev = dev_id;

	pr_info(TAG "Interrupt occured\n");

	schedule_delayed_work(&antdev->dwork,
		msecs_to_jiffies(g_debounce_interval));

	return IRQ_HANDLED;
}

static const struct of_device_id ant_det_of_match[] = {
	{ .compatible = "huawei,antenna_cable_detect", },
	{ },
};
MODULE_DEVICE_TABLE(of, ant_det_of_match);

static int antenna_pinctrl_init(struct ant_device *di)
{
	int ret;
	unsigned int index = 0;

	pr_info("%s: function start\n", __func__);
	/* get antenna_pinctrl */
	antenna_pinctrl = devm_pinctrl_get(di->dev);
	if (IS_ERR(antenna_pinctrl)) {
		pr_err(TAG "get antenna pinctrl fail\n");
		return -1;
	}

	for (; index < g_gpio_count; index++) {
		pin_state[index] = pinctrl_lookup_state(antenna_pinctrl,
			gpio_state_table[index]);
		if (IS_ERR(pin_state[index])) {
			pr_err(TAG "index:%u, get pin_state fail\n", index);
			return -1;
		}

		ret = pinctrl_select_state(antenna_pinctrl, pin_state[index]);
		if (ret) {
			pr_err(TAG "index:%u, set pin_state fail\n", index);
			return -1;
		}
	}

	return 0;
}

static void ant_dt_parse(struct device *dev)
{
	const struct of_device_id *of_id =
		of_match_device(ant_det_of_match, dev);
	struct device_node *np = dev->of_node;
	int i = 0;
	enum of_gpio_flags flags = 1;

	if (!of_id || !np)
		return;

	if (of_property_read_u32(np, "gpio_count", &g_gpio_count))
		g_gpio_count = 1; /* default 1 antenna */

	for (i = 0; i < g_gpio_count; i++) {
		g_gpio[i] = of_get_gpio_flags(np, i, &flags);
		pr_debug(TAG "g_gpio[%d] = %d\n", i, g_gpio[i]);

		if (!gpio_is_valid(g_gpio[i]))
			return;
	}

	if (of_property_read_u32(np, "debounce-interval", &g_debounce_interval))
		g_debounce_interval = DEBOUNCE_DEFAULT_VALUE;

	if (of_property_read_u32(np, "func_type", &g_func_type)) {
		g_func_type = 0;
		pr_err("%s: get func_type error\n", __func__);
	}
}

static int ant_det_gpio_process(struct ant_device *antdev, int irq, int gpio)
{
	int err;

	/* request a gpio and set the function direction in  */
	err = gpio_request_one(gpio, GPIOF_IN, DRV_NAME);
	if (err) {
		pr_err(TAG "Unable to request GPIO %d, err %d\n", gpio, err);
		return err;
	}

	/* request irq */
	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		err = irq;
		pr_err(TAG "Unable to get irq number for GPIO %d, err %d\n",
			gpio, err);
		return err;
	}

	err = request_threaded_irq(irq, NULL, &ant_pull_in_out_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		DRV_NAME, antdev);
	if (err) {
		gpio_free(gpio);
		pr_err(TAG "Unable to request IRQ %d, err %d\n", irq, err);
		return err;
	}

	pr_info(TAG "Request GPIO %d, IRQ: %d success\n", gpio, irq);
	return 0;
}

static ssize_t ant_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ant_device *antdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ant_state_get(antdev));
}

static struct device_attribute dev_attr_gpio_status =
__ATTR(state, DEV_ATTR_ACCESS_MODE, ant_state_show, NULL);

static void ant_set_gpio_info(struct ant_device *antdev)
{
	int err;
	int i;

	for (i = 0; i < g_gpio_count; i++) {
		err = ant_det_gpio_process(antdev, g_irq[i], g_gpio[i]);
		if (err)
			pr_err(TAG "Process GPIO %d failed\n", g_gpio[i]);
		else
			pr_info(TAG "Process GPIO %d success\n", g_gpio[i]);
	}
}

static int parse_cable_adc_channel_and_range(struct ant_device *di)
{
	int ret;

	/* adc channel */
	cable_adc_channel = devm_kzalloc(di->dev_adc, sizeof(*cable_adc_channel),
		GFP_KERNEL);
	if (!cable_adc_channel)
		return -ENOMEM;

	cable_adc_channel = iio_channel_get(di->dev_adc, "antenna_cable_adc");
	if (IS_ERR(cable_adc_channel)) {
		pr_err("get antenna channel failed!\n");
		return -1;
	}
	pr_err("get adc channel = %d\n", cable_adc_channel->channel->channel);

	ret = iio_read_channel_processed(cable_adc_channel, &g_voltage);
	if (ret < 0 || g_voltage < 0) {
		pr_err("ret = %d, g_voltage = %d\n", ret, g_voltage);
		return -1;
	}

	/* g_voltage * 1500 / 4096 */
	g_voltage = ((unsigned int)g_voltage * 1500) >> 12;
	pr_err("antenna  adc voltage = %d\n", g_voltage);
	return 0;
}

static int ant_dev_probe(struct platform_device *pdev)
{
	int err = -1;
	struct ant_device *antdev = NULL;

	antdev = kzalloc(sizeof(struct ant_device), GFP_KERNEL);
	if (!antdev)
		return -ENOMEM;

	antdev->dev = &pdev->dev;
	antdev->dev_adc = &pdev->dev;

	/* get dts data */
	ant_dt_parse(&pdev->dev);

	if (g_func_type != 1) {
		/* init antenna pinctrl, g_gpio_count needed */
		if (antenna_pinctrl_init(antdev) != 0) {
			pr_err(TAG "antenna pinctrl init fail\n");
			goto exit_free_mem;
		}
		/* init antenna pinctrl, g_gpio_count needed */
		if (parse_cable_adc_channel_and_range(antdev) != 0) {
			pr_err(TAG "antenna pin init fail\n");
			goto exit_free_mem;
		}
	}

	/* init delayed work */
	INIT_DELAYED_WORK(&antdev->dwork, ant_det_work_func);

	ant_set_gpio_info(antdev);

	platform_set_drvdata(pdev, antdev);

	/*
	 * Create directory "/sys/class/antenna_cable/detect"
	 * To get event, up layer will listen the directory
	 */
	antdev->class = class_create(THIS_MODULE, "antenna_cable");
	if (IS_ERR(antdev->class)) {
		pr_err(TAG "create class failed\n");
		goto exit_free_mem;
	}

	err = alloc_chrdev_region(&antdev->devno, 0, 1, DRV_NAME);
	if (err) {
		pr_err(TAG "alloc character device region failed\n");
		goto exit_class_destroy;
	}

	antdev->dev = device_create(antdev->class,
		&pdev->dev, antdev->devno, antdev, "detect");
	if (IS_ERR(antdev->dev)) {
		pr_err(TAG "creat device failed\n");
		goto exit_class_destroy;
	}

	err = device_create_file(antdev->dev, &dev_attr_gpio_status);
	if (err) {
		pr_err(TAG "create file failed\n");
		goto exit_device_destroy;
	}

	pr_info(TAG "huawei antenna cable state detect probe ok\n");
	return 0;

exit_device_destroy:
	device_destroy(antdev->class, antdev->devno);
exit_class_destroy:
	class_destroy(antdev->class);
exit_free_mem:
	kfree(antdev);
	pr_info(TAG "huawei antenna cable state detect probe failed\n");
	return err;
}

static int ant_dev_remove(struct platform_device *pdev)
{
	int i;
	struct ant_device *antdev = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&antdev->dwork);

	device_remove_file(antdev->dev, &dev_attr_gpio_status);
	device_destroy(antdev->class, antdev->devno);
	class_destroy(antdev->class);

	for (i = 0; i < g_gpio_count; i++) {
		(void)free_irq(g_irq[i], antdev);
		gpio_free(g_gpio[i]);
	}

	kfree(antdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ant_dev_driver = {
	.probe = ant_dev_probe,
	.remove = ant_dev_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ant_det_of_match),
	}
};

static int __init ant_det_init(void)
{
	return platform_driver_register(&ant_dev_driver);
}

static void __exit ant_det_exit(void)
{
	platform_driver_unregister(&ant_dev_driver);
}

late_initcall(ant_det_init);
module_exit(ant_det_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("HUAWEI Antenna Cable State Detect Driver");
MODULE_AUTHOR("HUAWEI Inc");
