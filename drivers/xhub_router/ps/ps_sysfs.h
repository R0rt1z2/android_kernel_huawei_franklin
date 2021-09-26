/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: ps sysfs header file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2020-05-25
 */

#ifndef __PS_SYSFS_H__
#define __PS_SYSFS_H__

ssize_t attr_ps_calibrate_show(struct device *dev,
	struct device_attribute *attr, char *buf);
/* temp: buffer to write, length: bytes to write */
int write_ps_offset_to_nv(int *temp, uint16_t length);
ssize_t attr_ps_calibrate_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t attr_set_stop_ps_auto_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
ssize_t attr_set_dt_ps_sensor_stup(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
ssize_t sensors_calibrate_show_ps(int tag, struct device *dev,
	struct device_attribute *attr, char *buf);

#endif
