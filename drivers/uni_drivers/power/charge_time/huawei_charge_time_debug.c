/*
 * huawei_charge_time_debug.c
 *
 * The remaining time of charging debug.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#include "huawei_charge_time_debug.h"
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/sysfs.h>
#include <linux/module.h>

static struct huawei_chg_time_device *g_chg_time_di;

void huawei_chg_time_output_set(int valid, int sec)
{
	struct huawei_chg_time_device *di = g_chg_time_di;

	if (!di)
		return;

	if (valid)
		di->test_output_val = (int)((unsigned int)sec | VALID_FLAG);
	else
		di->test_output_val = sec;
}

void huawei_chg_time_test_flag(int flag)
{
	struct huawei_chg_time_device *di = g_chg_time_di;

	if (!di)
		return;

	di->test_flag = flag;
}

int huawei_chg_time_test_soc(void)
{
	static int soc;
	struct huawei_chg_time_device *di = g_chg_time_di;

	if (!di)
		return 0;

	if (soc > 100)
		soc = 0;
	return soc++;
}

void huawei_chg_time_flash_test(int type, int soc, int time)
{
	struct huawei_chg_time_device *di = g_chg_time_di;
	struct huawei_chg_time_param *param = NULL;

	if (!di || soc > FULL_SOC || soc < 0)
		return;

	if (type == 0)
		param = &di->flash_param.standard;
	else if (type == 1)
		param = &di->flash_param.fcp;
	else if (type == 2)
		param = &di->flash_param.lvc;
	else if (type == 3)
		param = &di->flash_param.sc;
	else
		return;

	param->step_time[soc] = time;
	ct_info("%s type %d, i[%d] = %d\n", __func__, type, soc, time);

	if (soc == FULL_SOC) {
		param->batt_cycles = di->batt_info.batt_cycles;
		param->temp_high = NORMAL_TEMP;
		param->temp_low = NORMAL_TEMP;
		param->start_soc = 0;
		huawei_chg_time_write_param_to_flash(di);
	}
}

static ssize_t huawei_chg_time_flash_param_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct huawei_chg_time_device *di = dev_get_drvdata(dev);

	if (!di)
		return -EINVAL;

	ct_info("%s()-line=%d\n", __func__, __LINE__);
	huawei_chg_time_read_param_from_flash(di);
	return 0;
}

void huawei_chg_time_clear_flash_data(void)
{
	void *p_buf = NULL;

	p_buf = kzalloc(CHG_DUR_SIZE, GFP_KERNEL);
	if (!p_buf)
		return;

	huawei_chg_time_file_write(p_buf, CHG_DUR_SIZE);
	kfree(p_buf);
}

static ssize_t huawei_chg_time_flash_param_clear(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int status = count;

	if (!buf)
		return -EINVAL;

	ct_info("%s()-line=%d\n", __func__, __LINE__);
	huawei_chg_time_clear_flash_data();

	return status;
}

static DEVICE_ATTR(flash_param, 0644, huawei_chg_time_flash_param_show,
	huawei_chg_time_flash_param_clear);

int huawei_chg_time_debug_init(struct huawei_chg_time_device *di)
{
	int ret;

	if (!di)
		return 0;

	ret = device_create_file(di->dev, &dev_attr_flash_param);
	if (ret)
		ct_err("%s failed to create file\n", __func__);
	else
		g_chg_time_di = di;

	return ret;
}

MODULE_DESCRIPTION("charger time debug driver");
MODULE_LICENSE("GPL v2");
