/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: als route source file
 * Author: wangsiwen
 * Create: 2020-10-29
 */

#include <linux/slab.h>
#include <securec.h>
#include <huawei_platform/oeminfo/oeminfo_def.h>
#include "sensor_scp.h"
#include "als_route.h"

ssize_t als_under_tp_calidata_store(int32_t sensor_type, struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	int ret;
	struct oeminfo_info_user *user_info = NULL;
	int als_undertp_calidata[ALS_UNDER_TP_CALDATA_SIZE] = {0};

	if (sensor_type != SENSOR_TYPE_LIGHT || !dev || !attr || !buf ||
		size != (ALS_UNDER_TP_CALDATA_SIZE * sizeof(int))) {
		pr_err("%s : invalid params, sensor_type: %d, size: %zu\n",
			__func__, sensor_type, size);
		return -1;
	}

	ret = memcpy_s(als_undertp_calidata, sizeof(als_undertp_calidata),
		buf, sizeof(als_undertp_calidata));
	if (ret != EOK) {
		pr_err("%s: als_undertp_calidata memcpy_s fail, ret: %d\n",
			__func__, ret);
		return -1;
	}

	pr_info("%s: x = %d, y = %d, width = %d, len = %d\n", __func__,
		als_undertp_calidata[0], als_undertp_calidata[1],
		als_undertp_calidata[2], als_undertp_calidata[3]);
	for (i = 4; i < 29; i++) // memory a[25], b[30]
		pr_info("%s: a[%d] = %d\n", __func__,
			i - 4, als_undertp_calidata[i]);
	for (i = 29; i < ALS_UNDER_TP_CALDATA_SIZE; i++)
		pr_info("%s: b[%d] = %d\n", __func__,
			i - 29, als_undertp_calidata[i]);

	user_info = kzalloc(sizeof(*user_info), GFP_KERNEL);
	if (!user_info)
		return -1;

	user_info->oeminfo_operation = OEMINFO_WRITE;
	user_info->oeminfo_id = OEMINFO_ALS_UNDER_TP_CALIDATA;
	user_info->valid_size = sizeof(als_undertp_calidata);
	ret = memcpy_s(user_info->oeminfo_data, sizeof(user_info->oeminfo_data),
		als_undertp_calidata,
		(sizeof(user_info->oeminfo_data) < user_info->valid_size) ?
		sizeof(user_info->oeminfo_data) : user_info->valid_size);
	if (ret != EOK) {
		pr_err("%s: oeminfo_data memcpy_s fail, ret: %d\n",
			__func__, ret);
		kfree(user_info);
		return -1;
	}
	if (oeminfo_direct_access(user_info)) {
		pr_err("%s: oeminfo_id: %d write fail\n",
			__func__, user_info->oeminfo_id);
		kfree(user_info);
		return -1;
	}
	kfree(user_info);

	return (ssize_t)size;
}

ssize_t als_under_tp_calidata_show(int32_t sensor_type, struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int ret;
	struct oeminfo_info_user *user_info = NULL;
	int als_undertp_calidata[ALS_UNDER_TP_CALDATA_SIZE] = {0};

	if (sensor_type != SENSOR_TYPE_LIGHT || !dev || !attr || !buf) {
		pr_err("%s : invalid params, sensor_type: %d\n",
			__func__, sensor_type);
		return -1;
	}

	user_info = kzalloc(sizeof(*user_info), GFP_KERNEL);
	if (!user_info)
		return -1;

	user_info->oeminfo_operation = OEMINFO_READ;
	user_info->oeminfo_id = OEMINFO_ALS_UNDER_TP_CALIDATA;
	user_info->valid_size = sizeof(als_undertp_calidata);

	if (oeminfo_direct_access(user_info)) {
		pr_err("%s: oeminfo_id: %d read fail\n",
			__func__, user_info->oeminfo_id);
		kfree(user_info);
		return -1;
	}

	ret = memcpy_s(als_undertp_calidata, sizeof(als_undertp_calidata),
		user_info->oeminfo_data, user_info->valid_size);
	if (ret != EOK) {
		pr_err("%s: memcpy_s fail, ret: %d\n", __func__, ret);
		kfree(user_info);
		return -1;
	}
	kfree(user_info);
	ret = snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1,
		"%d", als_undertp_calidata[0]);
	if (ret <= 0) {
		pr_err("%s: write a[0] to buf fail\n", __func__);
		return -1;
	}
	for (i = 1; i < ALS_UNDER_TP_CALDATA_SIZE; i++) {
		ret = snprintf_s(buf, MAX_STR_SIZE, MAX_STR_SIZE - 1, "%s,%d",
			buf, als_undertp_calidata[i]);
		if (ret <= 0) {
			pr_info("%s: write a[%d] to buf fail\n", __func__, i);
			return -1;
		}
	}

	return (ssize_t)ret;
}
