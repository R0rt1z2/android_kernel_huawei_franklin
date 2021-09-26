/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Description: Contexthub common driver.
 * Author: Huawei
 * Create: 2017-07-21
 */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <securec.h>
#include <securectype.h>

#include "xhub_route.h"
#include "xhub_boot.h"
#include "protocol.h"

#ifdef __LLT_UT__
#define STATIC
#else
#define STATIC static
#endif

int get_contexthub_dts_status(void)
{
	int len = 0;
	struct device_node *node = NULL;
	const char *status = NULL;
	static int ret;
	static int once;

	if (once) {
		pr_info("[%s]status[%d]\n", __func__, ret);
		return ret;
	}

	node = of_find_compatible_node(NULL, NULL, "huawei,contexthub_status");
	if (node != NULL) {
		status = of_get_property(node, "status", &len);
		if (status == NULL) {
			pr_err("[%s]of_get_property status err\n", __func__);
			return -EINVAL;
		}

		if (strstr(status, "disabled")) {
			pr_info("[%s][disabled]\n", __func__);
			ret = -EINVAL;
		}
	}

	once = 1;
	pr_info("[%s][enabled]\n", __func__);
	return ret;
}

int get_ext_contexthub_dts_status(void)
{
	int len = 0;
	struct device_node *node = NULL;
	const char *status = NULL;
	static int ret = -EINVAL;
	static int once;

	if (once) {
		pr_info("[%s]status[%d]\n", __func__, ret);
		return ret;
	}

	node = of_find_compatible_node(NULL, NULL, "huawei,ext_sensorhub_status");
	if (node != NULL) {
		status = of_get_property(node, "status", &len);
		if (status == NULL) {
			pr_err("[%s]of_get_property status err\n", __func__);
			return -EINVAL;
		}

		if (strstr(status, "ok")) {
			pr_info("[%s][disabled]\n", __func__);
			ret = 0;
		}
	}

	once = 1;
	pr_info("[%s][enabled]\n", __func__);
	return ret;
}

STATIC int ctxhub_plt_remove(struct platform_device *pdev)
{
	return 0;
}


STATIC int ctxhub_plt_pm_suspend(struct device *dev)
{
	return 0;
}


STATIC int ctxhub_plt_probe(struct platform_device *pdev)
{
	int ret = get_contexthub_dts_status();
	if (ret) {
		return ret;
	}

	if (false == of_device_is_available(pdev->dev.of_node)) {
		return -ENODEV;
	}

	pr_info("[%s]\n", __func__);
	return 0;
}


STATIC int ctxhub_plt_pm_resume(struct device *dev)
{
	return 0;
}


static const struct of_device_id ctxhub_plt_dev_id[] = {
	{ .compatible = "huawei,smart-plt" },
	{},
};
MODULE_DEVICE_TABLE(of, ctxhub_plt_dev_id);


struct dev_pm_ops ctxhub_plt_pm_ops = {
	.suspend = ctxhub_plt_pm_suspend,
	.resume  = ctxhub_plt_pm_resume,
};


static struct platform_driver ctxhub_plt_platdrv = {
	.driver = {
		.name	= "contexthub-platform",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ctxhub_plt_dev_id),
		.pm = &ctxhub_plt_pm_ops,
	},
	.probe = ctxhub_plt_probe,
	.remove  = ctxhub_plt_remove,
};


STATIC int __init ctxhub_plt_init(void)
{
	return platform_driver_register(&ctxhub_plt_platdrv);
}


STATIC void __exit ctxhub_plt_exit(void)
{
	platform_driver_unregister(&ctxhub_plt_platdrv);
}


late_initcall_sync(ctxhub_plt_init);
module_exit(ctxhub_plt_exit);

