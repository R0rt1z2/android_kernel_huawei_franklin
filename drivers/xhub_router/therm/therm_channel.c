/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: therm channel source file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2020-05-25
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "xhub_route.h"
#include "xhub_boot.h"
#include "protocol.h"
#include "sensor_config.h"
#include "sensor_detect.h"
#include "therm_channel.h"
#include <securec.h>

void reset_therm_calibrate_data(void)
{
	if (strlen(sensor_chip_info[THERMOMETER]) == 0)
		return;
	hwlog_info("%s\n", __func__);
}

