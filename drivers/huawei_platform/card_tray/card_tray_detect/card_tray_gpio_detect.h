 /*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: This program detect the card tray gpio status
 * Author: xubuchao2@huawei.com
 * Create: 2019-11-27
 */
#ifndef _CARD_TRAY_GPIO_DETECT_
#define _CARD_TRAY_GPIO_DETECT_
#include <linux/device.h>

struct card_tray_info {
	struct device *dev;
};

struct card_tray_pinctrl_type {
	int gpio;
	struct pinctrl_state *pinctrl_default;
};

enum gpio_id {
	GPIO_ID0 = 0,
	MAX_GPIO_NUM = 1,
	};

#endif
