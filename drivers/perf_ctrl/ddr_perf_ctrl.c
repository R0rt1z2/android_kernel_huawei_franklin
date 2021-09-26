/*
 * ddr_perf_ctrl.c
 *
 * interfaces defined for perf_ctrl
 *
 * Copyright (c) 2016-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/perf_ctrl.h>


void __weak plat_get_ddr_flux(struct ddr_flux *data) {}
void __weak plat_get_ddr_other_vote(int *freq) {}

int perf_ctrl_get_ddr_flux(void __user *uarg)
{
	struct ddr_flux ddr_flux_val = {0};

	if (uarg == NULL)
		return -EINVAL;

	plat_get_ddr_flux(&ddr_flux_val);

	if (copy_to_user(uarg, &ddr_flux_val, sizeof(ddr_flux_val))) {
		pr_err("%s: copy_to_user fail\n", __func__);
		return -EFAULT;
	}

	return 0;
}

int perf_ctrl_get_ddr_other_vote(void __user *uarg)
{
	int max_freq = 0;

	if (uarg == NULL)
		return -EINVAL;

	plat_get_ddr_other_vote(&max_freq);

	if (copy_to_user(uarg, &max_freq, sizeof(int))) {
		pr_err("%s: copy_to_user fail\n", __func__);
		return -EFAULT;
	}

	return 0;
}
