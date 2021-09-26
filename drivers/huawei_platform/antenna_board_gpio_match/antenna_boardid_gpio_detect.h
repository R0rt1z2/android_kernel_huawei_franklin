/*
 * huawei antenna boardid gpio detect driver
*/

#ifndef _ANTENNA_BOARDID_GPIO_DETECT
#define _ANTENNA_BOARDID_GPIO_DETECT
#include <linux/device.h>

#define GPIO_COL_INDEX_MAX         3
#define GPIO_ROW_INDEX_MAX         9
#define BOARD_ID_INDEX_MAX         9

#define GPIO_BOARD_INDEX_MASK      0xF
#define ANTENNA_BOARDID_BASE       0x07D0

#define ANTENNA_DETECT_FAIL        0
#define ANTENNA_DETECT_SUCCEED     1

#define DELAY_TIME                 100

enum gpio_id {
	GPIO_ID0 = 0,
	GPIO_ID1 = 1,
	GPIO_ID2 = 2,
	MAX_GPIO_NUM = 3,
};

enum antenna_type {
	ANTENNA_BOARDID_GPIO_STATUS = 0,
	ANTENNA_DETECT_SUCCESS      = 1,
};

struct antenna_device_info {
	struct device *dev;
};

struct antenna_detect_info {
	struct device_attribute attr;
	u8 name;
};


struct antenna_gpio_state {
	char *state_dafault;
	char *state_bias_up;
	char *state_bias_down;
};

struct antenna_pinctrl_type {
	int gpio;
	struct pinctrl_state *pinctrl_up;
	struct pinctrl_state *pinctrl_down;
	struct pinctrl_state *pinctrl_default;
};

#endif
