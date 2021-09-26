/*
 * hw_msconfig.c
 *
 * mass storage autorun and lun config driver
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

#include "hw_msconfig.h"

#define MS_STG_SET_LEN         32
#define FSG_MAX_LUNS_HUAWEI    2

/* enable or disable autorun function "enable" & "disable" */
static char g_autorun[MS_STG_SET_LEN] = "enable";
/* "sdcard" & "cdrom,sdcard" & "cdrom" & "sdcard,cdrom" can be used */
static char g_luns[MS_STG_SET_LEN] = "sdcard";

ssize_t autorun_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t size)
{
	if ((size > MS_STG_SET_LEN) || !buff || !dev || !attr) {
		pr_err("%s error\n", __func__);
		return -EINVAL;
	}

	if ((strcmp(buff, "enable") != 0) && (strcmp(buff, "disable") != 0)) {
		pr_err("%s para error '%s'\n", __func__, buff);
		return -EINVAL;
	}

	strlcpy(g_autorun, buff, sizeof(g_autorun));

	return size;
}

ssize_t autorun_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!buf || !dev || !attr) {
		pr_err("%s error\n", __func__);
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", g_autorun);
}

ssize_t luns_store(struct device *dev,
	struct device_attribute *attr, const char *buff, size_t size)
{
	if ((size > MS_STG_SET_LEN) || !buff || !dev || !attr) {
		pr_err("%s error\n", __func__);
		return -EINVAL;
	}

	strlcpy(g_luns, buff, sizeof(g_luns));

	return size;
}

ssize_t luns_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!buf || !dev || !attr) {
		pr_err("%s error\n", __func__);
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", g_luns);
}
