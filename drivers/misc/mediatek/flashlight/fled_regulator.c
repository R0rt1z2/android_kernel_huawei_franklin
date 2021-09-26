/*
 * fled_regulator.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * fled regulator interface
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <securec.h>
#include <linux/delay.h>

#include "richtek/rt-flashlight.h"
#include "mtk_charger.h"
#include "flashlight-core.h"
#include "flashlight-dt.h"

#define TASK_NAME_LEN 20
#define TASK_NAME "fled_reg_wq"

enum {
	FLED_REGULATOR0,
	FLED_REGULATOR1,
};

enum FLED_REG_MODE {
	FLED_REG_MODE_OFF,
	FLED_REG_MODE_LOW_VOLT,
	FLED_REG_MODE_HIGH_VOLT,
	FLED_REG_MODE_MAX,
};

struct fled_regulator_data {
	struct regulator_desc *desc;
	struct regulator_dev *dev;
	struct flashlight_device *fl_dev;
	struct notifier_block charge_nb;
	struct notifier_block fled_nb;
	struct workqueue_struct *fled_reg_wq;
	struct work_struct fled_charge_work;
	struct work_struct fled_work;
	int charge_enabled; /* 1: Charging, 0: discharging */
	int is_charge_changed; /* 1: Charging status changed, 0: no change */
	int fled_mode; /* the pmic work mode */
	int is_fled_mode_changed; /* 1: fled mode changed, 0: no change */
	const char *channel_name;
	int channel_id;
	int regulator_supported;
	int max_uA;
	int min_uA;
	enum FLED_REG_MODE current_mode; /* current regulator mode */
	enum FLED_REG_MODE new_mode;     /* the mode needed to set if enable the regulator */
	struct mutex mutex_fled_reg;
};

static int fled_regulator_charge_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int ret;
	int enabled;
	struct power_supply *psy = data;
	union power_supply_propval val;
	struct fled_regulator_data *reg_data = container_of(nb,
		struct fled_regulator_data, charge_nb);

	if (!psy || !psy->desc) {
		pr_err("input data error\n");
		return 0;
	}

	if (strcmp(psy->desc->name, "ac") && strcmp(psy->desc->name, "usb")) {
		pr_err("ignore %s\n", psy->desc->name);
		return 0;
	}

	ret = power_supply_get_property(psy,
		POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret) {
		pr_err("get power supply failed\n");
		return 0;
	}
	if (!val.intval) {
		pr_info("delay 1s\n");
		msleep(1000);
	}
	enabled = val.intval ? 1 : 0;
	reg_data->is_charge_changed = (enabled != reg_data->charge_enabled) ? 1 : 0;
	reg_data->charge_enabled  = enabled;

	pr_info("psy_name: %s, val.intval:%d, charge_enabled: %d\n",
		psy->desc->name, val.intval, reg_data->charge_enabled);

	/* queue the charging work */
	queue_work(reg_data->fled_reg_wq, &reg_data->fled_work);

	return 0;
}

static int fled_regulator_callback(struct notifier_block *nb,
	unsigned long mode, void *data)
{
	int cb_id;
	struct fled_regulator_data *reg_data = container_of(nb,
		struct fled_regulator_data, fled_nb);

	if (!data)
		return -EINVAL;

	cb_id = *(int *)data;

	pr_info("mode: %d, id: %d\n", mode, cb_id);

	reg_data->is_fled_mode_changed = (reg_data->fled_mode != mode) ? 1 : 0;
	reg_data->fled_mode = mode;

	queue_work(reg_data->fled_reg_wq, &reg_data->fled_work);

	return 0;
}

static int fled_regulator_fl_dev_init(struct fled_regulator_data *data)
{
	int ret;

	mutex_lock(&data->mutex_fled_reg);
	if (data->fl_dev) {
		ret = 0;
		goto exit;
	}

	data->fl_dev = find_flashlight_by_name(data->channel_name);
	if (!data->fl_dev) {
		pr_err("failed to get flashlight dev:%s\n", data->channel_name);
		ret = -ENODEV;
		goto exit;
	}

	/* set fled driver callback */
	data->fled_nb.notifier_call = fled_regulator_callback;
	ret = flashlight_set_regulator_driver(data->fl_dev, &data->fled_nb);
	if (ret < 0)
		pr_err("set regulator driver failed\n");

exit:
	mutex_unlock(&data->mutex_fled_reg);
	return ret;
}

static int fled_regulator_set_mode(struct flashlight_device *fl_dev,
	enum FLED_REG_MODE mode)
{
	int rc;
	enum flashlight_mode fl_mode;

	pr_info("%s, Enter mode:%d\n", __func__, mode);

	if (mode >= FLED_REG_MODE_MAX) {
		pr_err("Invalid mode: %d\n", mode);
		return -EINVAL;
	}

	switch (mode) {
	case FLED_REG_MODE_LOW_VOLT:
		fl_mode = FLASHLIGHT_MODE_TORCH;
		break;
	case FLED_REG_MODE_HIGH_VOLT:
		fl_mode = FLASHLIGHT_MODE_MIXED;
		break;
	case FLED_REG_MODE_OFF:
		fl_mode = FLASHLIGHT_MODE_OFF;
		break;
	default:
		fl_mode = FLASHLIGHT_MODE_OFF;
		break;
	}

	rc = flashlight_set_mode(fl_dev, fl_mode);
	if (rc < 0)
		pr_err("set mode failed: %d\n", mode);

	return rc;
}

static int fled_regulator_enable(struct regulator_dev *dev)
{
	int rc;
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);

	pr_info("Enter\n");
	if (!reg_data) {
		pr_info("failed\n");
		return -EINVAL;
	}

	rc = fled_regulator_fl_dev_init(reg_data);
	if (rc < 0) {
		pr_err("Init failed\n");
		return -ENODEV;
	}

	mutex_lock(&reg_data->mutex_fled_reg);

	rc = flashlight_is_ready(reg_data->fl_dev);
	if (!rc) {
			reg_data->new_mode = FLED_REG_MODE_LOW_VOLT;
			pr_info("%s, change to low\n", __func__);
	} else {
		pr_info("%s, rc = %d\n", __func__, rc);
	}
	rc = fled_regulator_set_mode(reg_data->fl_dev, reg_data->new_mode);
	if (rc < 0) {
		pr_err("Failed, mode:%d\n", reg_data->new_mode);
		mutex_unlock(&reg_data->mutex_fled_reg);
		return -EFAULT;
	}
	pr_info("current_mode:%d, new_mode:%d\n",
		reg_data->current_mode, reg_data->new_mode);
	reg_data->current_mode = reg_data->new_mode;

	/* notifier the consumer */
	(void)regulator_notifier_call_chain(reg_data->dev,
		REGULATOR_EVENT_VOLTAGE_CHANGE, &reg_data->current_mode);

	mutex_unlock(&reg_data->mutex_fled_reg);

	pr_info("Exit\n");
	return rc;
}

static int fled_regulator_disable(struct regulator_dev *dev)
{
	int rc;
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);

	pr_info("%s, Enter\n", __func__);

	rc = fled_regulator_fl_dev_init(reg_data);
	if (rc < 0) {
		pr_err("Init failed\n");
		return -ENODEV;
	}

	mutex_lock(&reg_data->mutex_fled_reg);
	rc = fled_regulator_set_mode(
		reg_data->fl_dev, FLED_REG_MODE_OFF);
	if (rc < 0) {
		pr_err("set off failed\n");
		mutex_unlock(&reg_data->mutex_fled_reg);
		return -EINVAL;
	}

	reg_data->current_mode = FLED_REG_MODE_OFF;
	mutex_unlock(&reg_data->mutex_fled_reg);

	pr_info("%s, Exit\n", __func__);
	return 0;
}

static int fled_regulator_is_enabled(struct regulator_dev *dev)
{
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);
	pr_info("%s, Enter\n", __func__);
	if (!reg_data) {
		pr_err("Null ptr\n");
		return 0;
	}

	return reg_data->current_mode == FLED_REG_MODE_OFF ? 0 : 1;
}

static int fled_regulator_set_current_limit(struct regulator_dev *dev,
	int min_uA, int max_uA)
{
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);

	if (!reg_data)
		return -EINVAL;

	if (min_uA > max_uA)
		return -EINVAL;

	mutex_lock(&reg_data->mutex_fled_reg);
	if (max_uA < reg_data->max_uA)
		reg_data->max_uA = max_uA;
	if (min_uA > reg_data->min_uA)
		reg_data->max_uA = min_uA;
	mutex_unlock(&reg_data->mutex_fled_reg);

	return 0;
}

static int fled_regulator_get_current_limit(struct regulator_dev *dev)
{
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);
	if (!reg_data) {
		pr_err("%s, get drv data failed\n", __func__);
		return -EINVAL;
	}

	return reg_data->max_uA;
}

static int fled_regulator_set_load(struct regulator_dev *dev, int load_uA)
{
	int rc;
	struct fled_regulator_data *reg_data = rdev_get_drvdata(dev);

	pr_info("Enter load_uA: %d\n", load_uA);
	if (!reg_data) {
		pr_err("get drv data failed\n");
		return -EINVAL;
	}

	if (load_uA >= reg_data->max_uA || load_uA <= reg_data->min_uA)
		return -EINVAL;

	rc = fled_regulator_fl_dev_init(reg_data);
	if (rc < 0) {
		pr_err("Init failed\n");
		return -ENODEV;
	}

	rc = flashlight_set_torch_current(reg_data->fl_dev, load_uA, load_uA);
	if (rc < 0)
		pr_err("set torch cur failed\n");

	rc = flashlight_set_reg_current(reg_data->fl_dev, load_uA, load_uA);
	if (rc < 0)
		pr_err("set reg cur failed\n");

	return rc;
}

/*
 * Description: change the regulator work mode if charging status changed
 * or the low layer pmic work mode changed.
 * return value: 0-mode unchanged, 1-mode changed.
 */
static int fled_regulator_change_mode(struct fled_regulator_data *reg_data)
{
	/* update new mode */
	mutex_lock(&reg_data->mutex_fled_reg);
	if (reg_data->charge_enabled ||
		reg_data->fled_mode == FLASHLIGHT_MODE_TORCH)
		reg_data->new_mode = FLED_REG_MODE_LOW_VOLT;
	else
		reg_data->new_mode = FLED_REG_MODE_HIGH_VOLT;

	pr_info("current_mode: %d, new_mode:%d, fled_mode: %d, charge_enabled: %d",
		reg_data->current_mode,
		reg_data->new_mode,
		reg_data->fled_mode,
		reg_data->charge_enabled);

	/* check the current mode */
	if (reg_data->current_mode == FLED_REG_MODE_OFF ||
		reg_data->current_mode == reg_data->new_mode) {
		pr_info("no need update\n");
		mutex_unlock(&reg_data->mutex_fled_reg);
		return 0;
	}

	if (reg_data->charge_enabled) {
		if (!flashlight_is_ready(reg_data->fl_dev)) {
				reg_data->new_mode = FLED_REG_MODE_LOW_VOLT;
				pr_info("%s, change to low\n", __func__);
		}
		(void)fled_regulator_set_mode(reg_data->fl_dev,
			FLED_REG_MODE_OFF);
		(void)fled_regulator_set_mode(reg_data->fl_dev,
			reg_data->new_mode);
		reg_data->current_mode = reg_data->new_mode;
	}

	mutex_unlock(&reg_data->mutex_fled_reg);

	return 1;
}

static void fled_regulator_work(struct work_struct *work)
{
	int ret;
	struct fled_regulator_data *reg_data =
		container_of(work, struct fled_regulator_data, fled_work);

	pr_info("current_mode:%d, new_mode:%d\n",
		reg_data->current_mode, reg_data->new_mode);

	if (fled_regulator_fl_dev_init(reg_data) < 0) {
		pr_err("dev not create yet\n");
		return;
	}

	if (!reg_data->is_charge_changed &&
		!reg_data->is_fled_mode_changed) {
		pr_info("No status change\n");
		return;
	}

	if (reg_data->is_charge_changed &&
		!reg_data->charge_enabled)
		msleep(1000);

	ret = fled_regulator_change_mode(reg_data);
	if (!ret) {
		pr_info("mode not changed\n");
		return;
	}

	/* notifier the consumer */
	(void)regulator_notifier_call_chain(reg_data->dev,
		REGULATOR_EVENT_VOLTAGE_CHANGE, &reg_data->current_mode);
}

static const struct regulator_ops fled_regulator_ops = {
	.enable = fled_regulator_enable,
	.disable = fled_regulator_disable,
	.is_enabled = fled_regulator_is_enabled,
	.set_current_limit = fled_regulator_set_current_limit,
	.get_current_limit = fled_regulator_get_current_limit,
	.set_load = fled_regulator_set_load,
};

static struct regulator_desc fled_regulators[] = {
	[FLED_REGULATOR0] = {
		.name = "FLED-REG0",
		.id = 0,
		.of_match = of_match_ptr("fled_reg0"),
		.ops = &fled_regulator_ops,
		.type = REGULATOR_CURRENT,
		.owner = THIS_MODULE,
	},
	[FLED_REGULATOR1] = {
		.name = "FLED-REG1",
		.id = 1,
		.of_match = of_match_ptr("fled_reg1"),
		.ops = &fled_regulator_ops,
		.type = REGULATOR_CURRENT,
		.owner = THIS_MODULE,
	},
};

static int fled_regulator_drv_init(struct fled_regulator_data *drv_data)
{
	char task_name[TASK_NAME_LEN];

	if (snprintf_s(task_name, TASK_NAME_LEN, strlen(TASK_NAME) + 1,
		"%s%d", TASK_NAME, drv_data->channel_id) < 0) {
		pr_err("snprintf_s failed\n");
		return -EINVAL;
	}

	pr_info("%s\n", task_name);
	drv_data->fled_reg_wq = create_singlethread_workqueue(task_name);
	if (!drv_data->fled_reg_wq) {
		pr_err("init task failed\n");
		return -EINVAL;
	}

	INIT_WORK(&drv_data->fled_work, fled_regulator_work);
	INIT_WORK(&drv_data->fled_charge_work, fled_regulator_work);

	drv_data->current_mode = FLED_REG_MODE_OFF;
	drv_data->new_mode = FLED_REG_MODE_HIGH_VOLT;
	mutex_init(&drv_data->mutex_fled_reg);

	/* register charge notifier */
	drv_data->charge_nb.notifier_call = fled_regulator_charge_callback;
	(void)power_supply_reg_notifier(&drv_data->charge_nb);

	return 0;
}

static struct regulator_desc *fled_regulator_get_desc(
	struct regulator_desc *desc,
	int desc_size, int channel_id)
{
	int i;

	for (i = 0; i < desc_size; i++) {
		if (channel_id == desc[i].id)
			break;
	}

	if (i >= desc_size) {
		pr_err("find no desc\n");
		return NULL;
	}

	return &desc[i];
}

static int fled_regulator_parse_dt_data(
	const struct device_node *node,
	struct fled_regulator_data *data)
{
	if (!data)
		return -EINVAL;

	if (of_property_read_u32(node, "regulator-channel-id",
			&data->channel_id)) {
		pr_err("failed to get channel-id\n");
		return -ENODEV;
	}

	if (of_property_read_u32(node, "regulator-mode-enable",
			&data->regulator_supported))
		pr_err("failed to get reg supported\n");

	if (!data->regulator_supported) {
		pr_info("not support regulator\n");
		return -ENODEV;
	}

	if (of_property_read_u32(node, "regulator-max-microamp",
			&data->max_uA)) {
		pr_err("failed to get max_uA\n");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "regulator-min-microamp",
			&data->min_uA)) {
		pr_err("failed to get min_uA\n");
		return -EINVAL;
	}

	if (of_property_read_string(node, "regulator-channel-name",
			&data->channel_name)) {
		pr_err("failed to get channel-name\n");
		return -ENODEV;
	}

	return 0;
}

static struct fled_regulator_data *fled_regulator_get_drv_data(
	struct device *dev,
	const struct device_node *node)
{
	int ret;
	struct fled_regulator_data *data = NULL;

	pr_info("Enter\n");
	if (!node) {
		pr_err("Null node\n");
		return NULL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("Out of Memory\n");
		return NULL;
	}

	ret = fled_regulator_parse_dt_data(node, data);
	if (ret) {
		pr_err("parse dt failed\n");
		goto err;
	}

	/* fill desc to data */
	data->desc = fled_regulator_get_desc(fled_regulators,
		ARRAY_SIZE(fled_regulators), data->channel_id);
	if (!data->desc) {
		pr_err("%s, No desc find\n", __func__);
		goto err;
	}

	pr_info("Exit\n");
	return data;

err:
	devm_kfree(dev, data);
	return NULL;
}

static void fled_regulator_put_drv_data(struct device *dev,
	struct fled_regulator_data *drv_data)
{
	if (!dev || !drv_data)
		return;

	devm_kfree(dev, drv_data);
}

static int fled_regulator_probe(struct platform_device *pdev)
{
	int ret;
	struct fled_regulator_data *drv_data = NULL;
	struct device_node *reg_node = NULL;
	struct regulator_config config = {};
	struct device_node *child = NULL;

	pr_info("Enter\n");
	if (!pdev->dev.of_node) {
		pr_err("of_node is NULL\n");
		return -ENODEV;
	}
	reg_node = of_get_child_by_name(pdev->dev.of_node, "fled_regulators");
	if (!reg_node) {
		pr_err("get node failed\n");
		return -ENODEV;
	}

	for_each_child_of_node(reg_node, child) {
		drv_data = fled_regulator_get_drv_data(&pdev->dev, child);
		if (!drv_data)
			continue;

		config.init_data = of_get_regulator_init_data(&pdev->dev,
				child, drv_data->desc);
		if (!config.init_data) {
			pr_err("get init data failed");
			fled_regulator_put_drv_data(&pdev->dev, drv_data);
			continue;
		}
		config.dev = &pdev->dev;
		config.driver_data = drv_data;
		config.of_node = child;

		drv_data->dev = devm_regulator_register(&pdev->dev,
			drv_data->desc, &config);
		if (IS_ERR(drv_data->dev)) {
			pr_err("register reg failed");
			ret = -ENODEV;
			goto err;
		}

		ret = fled_regulator_drv_init(drv_data);
		if (ret < 0) {
			pr_err("drv init failed\n");
			goto err;
		}
	}
	of_node_put(reg_node);
	pr_info("Exit\n");
	return 0;
err:
	of_node_put(reg_node);
	return ret;
}

static const struct platform_device_id fled_reg_platform_ids[] = {
	{
		"fled-regulator",
		0
	}, {
		/* sentinel */
	},
};

MODULE_DEVICE_TABLE(platform, fled_reg_platform_ids);

static const struct of_device_id fled_of_match[] = {
	{
		.compatible = "hw,fled-regulator",
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, mt6359_of_match);

static struct platform_driver fled_regulator_driver = {
	.probe = fled_regulator_probe,
	.driver = {
		.name = "fled-regulator",
		.owner = THIS_MODULE,
		.of_match_table = fled_of_match,
	},
	.id_table = fled_reg_platform_ids,
};

static int __init fled_regulator_init(void)
{
	int ret;

	ret = platform_driver_register(&fled_regulator_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	return 0;
}

static void __exit fled_regulator_exit(void)
{
	platform_driver_unregister(&fled_regulator_driver);
}

subsys_initcall(fled_regulator_init);
module_exit(fled_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FLED Regulator Driver");
