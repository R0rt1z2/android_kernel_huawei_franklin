/*
 * mtk_leds_common.h
 *
 * lcdkit power function head file for lcd driver
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
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

#ifndef _MTK_LEDS_COMMON_H_
#define _MTK_LEDS_COMMON_H_
#include <linux/leds.h>

#ifdef CONFIG_LCD_KIT_DRIVER
struct led_conf_info {
	int level;
	int led_bits;
	int trans_bits;
	int max_level;
	struct led_classdev cdev;
};
#endif
#endif
