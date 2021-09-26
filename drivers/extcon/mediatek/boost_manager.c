/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/pm_wakeup.h>
#include <chipset_common/hwpower/hardware_channel/vbus_channel.h>

#if CONFIG_MTK_GAUGE_VERSION == 30
#include <mtk_gauge_time_service.h>
#include <mt-plat/charger_class.h>
#include <linux/alarmtimer.h>
#endif

struct usbotg_boost {
	struct platform_device *pdev;
	struct charger_device *primary_charger;
#if CONFIG_MTK_GAUGE_VERSION == 30
	struct alarm otg_timer;
	struct timespec endtime;
	struct workqueue_struct *boost_workq;
	struct delayed_work kick_work;
	unsigned int polling_interval;
	bool polling_enabled;
	unsigned int user;
	int mode;
#endif
};
static struct usbotg_boost *g_info;

#if CONFIG_MTK_GAUGE_VERSION == 30
static void usbotg_alarm_start_timer(struct usbotg_boost *info)
{
	struct timespec time, time_now;
	ktime_t ktime;

	get_monotonic_boottime(&time_now);
	time.tv_sec = info->polling_interval;
	time.tv_nsec = 0;
	info->endtime = timespec_add(time_now, time);

	ktime = ktime_set(info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->otg_timer, ktime);
}

static void enable_boost_polling(bool poll_en)
{
	if (g_info) {
		if (poll_en) {
			usbotg_alarm_start_timer(g_info);
			g_info->polling_enabled = true;
		} else {
			g_info->polling_enabled = false;
			alarm_try_to_cancel(&g_info->otg_timer);
		}
	}
}

static void usbotg_boost_kick_work(struct work_struct *work)
{
	struct usbotg_boost *usb_boost_manager =
		container_of(work, struct usbotg_boost, kick_work.work);
	struct device *dev = &usb_boost_manager->pdev->dev;

	pr_info("%s\n", __func__);

	charger_dev_kick_wdt(usb_boost_manager->primary_charger);

	if (usb_boost_manager->polling_enabled == true)
		usbotg_alarm_start_timer(usb_boost_manager);

	/* release wakelock */
	pm_relax(dev);
}

static enum alarmtimer_restart
	usbotg_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct usbotg_boost *usb_boost_manager =
		container_of(alarm, struct usbotg_boost, otg_timer);
	struct device *dev = &usb_boost_manager->pdev->dev;

	/* active wakelock */
	pm_stay_awake(dev);

	/* delay 100ms waiting i2c resume */
	queue_delayed_work(usb_boost_manager->boost_workq,
		&usb_boost_manager->kick_work, msecs_to_jiffies(100));

	return ALARMTIMER_NORESTART;
}
#endif

static struct usbotg_boost *mtk_charger_otg_get_dev(void)
{
	if (!g_info) {
		pr_info("g_info is null\n");
		return NULL;
	}

	return g_info;
}

int usb_otg_set_vbus(int is_on)
{
	if (!g_info)
		return -1;

#if CONFIG_MTK_GAUGE_VERSION == 30
	if (is_on) {
		charger_dev_enable_otg(g_info->primary_charger, true);
		charger_dev_set_boost_current_limit(g_info->primary_charger,
			1300000);
		charger_dev_kick_wdt(g_info->primary_charger);
		enable_boost_polling(true);
	} else {
		charger_dev_enable_otg(g_info->primary_charger, false);
		enable_boost_polling(false);
	}
#endif
	return 0;
}

static int mtk_charger_otg_open(unsigned int user, int flag)
{
	struct usbotg_boost *l_dev = mtk_charger_otg_get_dev();

	if (!l_dev)
		return -EINVAL;

	if (l_dev->user == VBUS_CH_NO_OP_USER) {
		l_dev->mode = VBUS_CH_IN_OTG_MODE;
		usb_otg_set_vbus(true);
		pr_info("start reverse_vbus flag=%d\n", flag);
	}

	l_dev->user |= (1 << user);

	pr_info("user=%x open ok\n", l_dev->user);
	return 0;
}

static int mtk_charger_otg_close(unsigned int user, int flag, int force)
{
	struct usbotg_boost *l_dev = mtk_charger_otg_get_dev();

	if (!l_dev)
		return -EINVAL;

	if (force) {
		charger_dev_enable_otg(l_dev->primary_charger, false);
		pr_info("force stop reverse_vbus\n");
		return 0;
	}

	l_dev->user &= (~(unsigned int)(1 << user));

	if (l_dev->user == VBUS_CH_NO_OP_USER) {
		l_dev->mode = VBUS_CH_NOT_IN_OTG_MODE;
		usb_otg_set_vbus(false);
		pr_info("stop reverse_vbus flag=%d\n", flag);
	}

	pr_info("user=%x close ok\n", l_dev->user);
	return 0;
}

static int mtk_charger_otg_get_state(unsigned int user, int *state)
{
	struct usbotg_boost *l_dev = mtk_charger_otg_get_dev();

	if (!l_dev || !state)
		return -EINVAL;

	if (l_dev->user == VBUS_CH_NO_OP_USER)
		*state = VBUS_CH_STATE_CLOSE;
	else
		*state = VBUS_CH_STATE_OPEN;

	return 0;
}

static int mtk_charger_otg_get_mode(unsigned int user, int *mode)
{
	struct usbotg_boost *l_dev = mtk_charger_otg_get_dev();

	if (!l_dev || !mode)
		return -EINVAL;

	*mode = l_dev->mode;
	return 0;
}

static struct vbus_ch_ops mtk_charger_otg_ops = {
	.type_name = "charger",
	.open = mtk_charger_otg_open,
	.close = mtk_charger_otg_close,
	.get_state = mtk_charger_otg_get_state,
	.get_mode = mtk_charger_otg_get_mode,
	.set_switch_mode = NULL,
	.set_voltage = NULL,
	.get_voltage = NULL,
};

static int usbotg_boost_probe(struct platform_device *pdev)
{
	struct usbotg_boost *info = NULL;
	struct device *dev = &pdev->dev;
#if CONFIG_MTK_GAUGE_VERSION == 30
	struct device_node *node = dev->of_node;
#endif

	info = devm_kzalloc(&pdev->dev, sizeof(struct usbotg_boost),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);
	info->pdev = pdev;
#if CONFIG_MTK_GAUGE_VERSION == 30
	info->primary_charger = get_charger_by_name("primary_chg");
#endif
	if (!info->primary_charger) {
		pr_info("%s: get primary charger device failed\n", __func__);
		return -ENODEV;
	}

#if CONFIG_MTK_GAUGE_VERSION == 30
	alarm_init(&info->otg_timer, ALARM_BOOTTIME,
		usbotg_alarm_timer_func);
	if (of_property_read_u32(node, "boost_period",
		(u32 *) &info->polling_interval))
		return -EINVAL;

	info->polling_interval = 30;
	info->boost_workq = create_singlethread_workqueue("boost_workq");
	INIT_DELAYED_WORK(&info->kick_work, usbotg_boost_kick_work);
#endif
	vbus_ch_ops_register(&mtk_charger_otg_ops);
	g_info = info;
	return 0;
}

static int usbotg_boost_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id usb_boost_of_match[] = {
	{.compatible = "mediatek,usb_boost"},
	{},
};

MODULE_DEVICE_TABLE(of, usb_boost_of_match);
static struct platform_driver usb_boost_driver = {
	.remove = usbotg_boost_remove,
	.probe = usbotg_boost_probe,
	.driver = {
		   .name = "mediatek,usb_boost",
		   .of_match_table = usb_boost_of_match,
		   },
};

static int __init usb_boost_init(void)
{
	platform_driver_register(&usb_boost_driver);
	return 0;
}

late_initcall(usb_boost_init);

static void __exit usb_boost_init_cleanup(void)
{
}

module_exit(usb_boost_init_cleanup);



