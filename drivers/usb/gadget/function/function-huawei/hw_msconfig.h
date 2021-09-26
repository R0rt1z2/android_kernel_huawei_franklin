/*
 * hw_msconfig.h
 *
 * interface to USB gadget mass config
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

#ifndef HW_MSCONFIG_H
#define HW_MSCONFIG_H

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>

ssize_t autorun_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t size);
ssize_t autorun_show(struct device *dev,
	struct device_attribute *attr, char *buf);

ssize_t luns_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t size);
ssize_t luns_show(struct device *dev,
	struct device_attribute *attr, char *buf);

#endif /* HW_MSCONFIG_H */
