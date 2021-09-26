/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: headset auto calibration
 * Author: wanggang
 * Create: 2019-11-20
 */

#ifndef __HS_AUTO_CALIBRATION_H__
#define __HS_AUTO_CALIBRATION_H__

#include <linux/device.h>

enum adjust_state {
	DEFAULT = 0,
	REC_JUDGE = DEFAULT,
	ERROR_JUDGE,
	VOL_INTERZONE_READJUST,
	BTN_REPORT,
	BTN_RECORD,
	UNEXCEPTION_HANDLE
};

enum vol_interzone_type {
	KEY_HOOK = 0,
	KEY_VOL_UP,
	KEY_VOL_DOWN,
	KEY_TYPE_MAX
};

struct btn_voltage { // mV
	unsigned int key_media_min_value;
	unsigned int key_media_max_value;
	unsigned int key_vol_up_min_value;
	unsigned int key_vol_up_max_value;
	unsigned int key_vol_down_min_value;
	unsigned int key_vol_down_max_value;
};

enum btn_voltage_def_value { // mV
	KEY_MEDIA_MIN_VALUE = 0,
	KEY_MEDIA_MAX_VALUE = 120,
	KEY_VOL_UP_MIN_VALUE = 121,
	KEY_VOL_UP_MAX_VALUE = 191,
	KEY_VOL_DOWN_MIN_VALUE = 192,
	KEY_VOL_DOWN_MAX_VALUE = 400
};

#ifdef CONFIG_HEADSET_AUTO_CALIB
void headset_auto_calib_init(struct device_node *dev_node);
void headset_auto_calib_reset_interzone(void);
void start_headset_auto_calib(enum adjust_state state, unsigned int vol_value,
	unsigned int *key_code);
#else
static inline void headset_auto_calib_init(struct device_node *dev_node)
{
}

static inline void headset_auto_calib_reset_interzone(void)
{
}

static inline void start_headset_auto_calib(enum adjust_state state,
	unsigned int vol_value, unsigned int *key_code)
{
}
#endif

#endif /* __HS_AUTO_CALIBRATION_H__ */
