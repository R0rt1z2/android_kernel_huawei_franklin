/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2019. All rights reserved.
 * Description: HHEE driver
 * Creator: security-ap
 * Date: 2017/2/1
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/compiler.h>
#include <linux/interrupt.h>
#include <asm/compiler.h>
#include <linux/debugfs.h>
#include "hhee.h"
#include "hhee_msg.h"

/*check hhee enable*/
static int g_hhee_enable = HHEE_ENABLE;
int hhee_check_enable(void)
{
	return g_hhee_enable;
}
EXPORT_SYMBOL(hhee_check_enable);

#define CPU_MASK         0xF
static void reset_hhee_irq_counters(void)
{
	struct arm_smccc_res res;
	arm_smccc_hvc((unsigned long)HHEE_MONITORLOG_RESET_COUNTERS,
		0ul, 0ul, 0ul, 0ul, 0ul, 0ul, 0ul, &res);
}

void hkip_clean_counters(void)
{
	reset_hkip_irq_counters();
	reset_hhee_irq_counters();
}
static int huawei_hkip_probe(struct platform_device *pdev)
{
	int ret;
#ifndef CONFIG_HKIP_USING_IPI
	int irq;
	struct device *dev = &pdev->dev;
#else
	(void)pdev;
#endif


	pr_info("huawei hkip probe\n");

	if (HHEE_DISABLE == hhee_check_enable())
		return 0;
#ifndef CONFIG_HKIP_USING_IPI
	/*irq num get and register*/
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "get irq failed\n");
		return -ENXIO;
	}
	ret = devm_request_irq(dev, (unsigned int)irq,
						hhee_irq_handle, 0ul, "huawei-hkip", pdev);
	if (ret < 0) {
		dev_err(dev, "devm request irq failed\n");
		return -EINVAL;
	}
	if(cpu_online(CPU_MASK) && (irq_set_affinity(irq, cpumask_of(CPU_MASK)) < 0))
		dev_err(dev, "set affinity failed\n");

#endif
	hhee_module_init();

	ret = hhee_logger_init();
	if (ret < 0) {
		ret = -EINVAL;
		goto err_free_hhee;
	}

#ifdef CONFIG_HUAWEI_HKIP_DEBUG
	ret = hhee_init_debugfs();
	if (ret)
		goto err_free_hhee;
#endif

	ret = hhee_msg_init();
	if (ret)
		goto err_free_hhee;

	pr_info("hhee probe done\n");
	return 0;

err_free_hhee:
	return ret;
}

static int huawei_hkip_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HUAWEI_HKIP_DEBUG
	hhee_cleanup_debugfs();
#endif
	return 0;
}

static const struct of_device_id huawei_hkip_of_match[] = {
	{.compatible = "huawei,huawei-hkip"},
	{},
};

static struct platform_driver huawei_hkip_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "huawei-hkip",
		.of_match_table = of_match_ptr(huawei_hkip_of_match),
	},
	.probe = huawei_hkip_probe,
	.remove = huawei_hkip_remove,
};

static int __init huawei_hhee_cmd_get(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (!strncmp(arg, "false", strlen("false")))
		g_hhee_enable = HHEE_DISABLE;

	pr_err("hkip enable = %d.\n", g_hhee_enable);
	return 0;
}
early_param("hkip_enable", huawei_hhee_cmd_get); /*lint -e528 */

static int __init huawei_hkip_device_init(void)
{
	int ret;

	pr_info("hkip probe init\n");
	ret = platform_driver_register(&huawei_hkip_driver);
	if (ret)
		pr_err("register hhee driver fail\n");

	return ret;
}

static void __exit huawei_hkip_device_exit(void)
{
	platform_driver_unregister(&huawei_hkip_driver);
}

module_init(huawei_hkip_device_init);
module_exit(huawei_hkip_device_exit);

MODULE_DESCRIPTION("huawei hkip driver");
MODULE_ALIAS("huawei hkip module");
MODULE_LICENSE("GPL");
