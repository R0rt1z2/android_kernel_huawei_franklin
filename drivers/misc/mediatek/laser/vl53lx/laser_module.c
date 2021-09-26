/*
 * laser_module.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * laser module file
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

/*lint -save -e574*/
#include "laser_module.h"

#define DRIVER_NAME "huawei,vl53l3"

static laser_module_intf_t laser_devices[] = {
	{ "VL53L1", &stmvl53l1_probe, &stmvl53l1_remove, &stmvl53l1_ioctl_handler},
};

static int laser_index = -1;

static long hw_laser_ioctl(void *hw_data, unsigned int cmd, void *p)
{
	int rc = 0;

	laser_info("Enter hw laser ioctl function");

	if (!hw_data) {
		laser_err("parameter error");
		return -EINVAL;
	}

	return rc;
}

static int laser_data_remove(struct i2c_client *client)
{
	int rc = 0;

	if (!client) {
		laser_err("parameter error");
		return -EINVAL;
	}

	if (laser_index >= 0 && laser_index < ARRAY_SIZE(laser_devices))
		rc |= laser_devices[laser_index].data_remove(client);

	return rc;
}

static int laser_data_init(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	/* try sub devices */
	int rc;
	int i;

	if (!client || !id) {
		laser_err("parameter error");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(laser_devices); i++) {
		rc = laser_devices[i].data_init(client, id);
		if (rc == 0)
			break;
	}

	laser_index = i;
	laser_info("sensor index = %d", i);
	return 0;
}

hw_laser_fn_t laser_module_fn = {
	.laser_ioctl = hw_laser_ioctl,
};

static hw_laser_ctrl_t laser_module_ctrl = {
	.func_tbl = &laser_module_fn,
	.data = NULL,
};

static const struct i2c_device_id laser_module_id[] = {
	{ "vl53l3", (unsigned long)&laser_module_ctrl},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(i2c, laser_module_id);

static const struct of_device_id laser_module_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, laser_module_of_match);

static int laser_module_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	if (!client || !id)
		return -EINVAL;

	laser_info("module probe enter");
	rc = laser_data_init(client, id);
	if (rc == 0)
		laser_info("laser_data_init success");
	return rc;
}

static int laser_module_remove(struct i2c_client *client)
{
	int rc = 0;

	if (!client)
		return -EINVAL;

	laser_info("module remove enter");
	rc = laser_data_remove(client);
	return rc;
}

static struct i2c_driver laser_module_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(laser_module_of_match),
	},
	.probe = laser_module_probe,
	.remove = laser_module_remove,
	.id_table = laser_module_id,
};

static int __init laser_module_init(void)
{
	laser_err("%s enter\n", __func__);
	return i2c_add_driver(&laser_module_i2c_driver);
}

static void __exit laser_module_exit(void)
{
	laser_err("%s enter", __func__);
	i2c_del_driver(&laser_module_i2c_driver);
}

module_init(laser_module_init);
module_exit(laser_module_exit);
MODULE_DESCRIPTION("laser driver");
MODULE_LICENSE("GPL v2");
/*lint -restore*/
