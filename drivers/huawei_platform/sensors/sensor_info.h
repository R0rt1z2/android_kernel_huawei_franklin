/*
 * sensor_info.h
 *
 * code for sensor debug sysfs
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#ifndef __SENSOR_INFO_H__
#define __SENSOR_INFO_H__
void send_sensor_scp_udfp(unsigned int value);
void get_status_from_ap(void);
#endif
