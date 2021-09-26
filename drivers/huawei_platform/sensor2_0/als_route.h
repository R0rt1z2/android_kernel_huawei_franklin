/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: als route header file
 * Author: wangsiwen
 * Create: 2020-10-29
 */

#ifndef __ALS_ROUTE_H__
#define __ALS_ROUTE_H__

#define MAX_STR_SIZE 1024
#define ALS_UNDER_TP_CALDATA_SIZE 59
#define OEMINFO_ALS_UNDER_TP_CALIDATA 16

ssize_t als_under_tp_calidata_store(int32_t tag, struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
ssize_t als_under_tp_calidata_show(int32_t sensor_type, struct device *dev,
	struct device_attribute *attr, char *buf);

#endif
