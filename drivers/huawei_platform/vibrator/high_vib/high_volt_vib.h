/*
 * high_volt_vib.h
 *
 * code for vibrator
 *
 * Copyright (c) 2020 Huawei Technologies Co., Ltd.
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

#ifndef _LINUX_VIBRATOR_HIGH_H
#define _LINUX_VIBRATOR_HIGH_H

#define HIGH_VIBRATOR  "vibrator"
#define HAPTIC_RESERVE_TIME            25
#define HIGH_VOLT_DEFAULT_PWM_NO       3
#define FULL_PWM_TIME                  100
#define FULL_FLASH_CURRENT             175000
#define FULL_PWM_CLK_FREQ              26000000
#define FULL_PWM_DUTY_RATIO            100
#define HIGH_VOLT_DUTY_RATIO           80
#define HIGH_VB_RTAIO_BATTERY_REF      3400

enum flash_power_mode_t {
	FLASH_PWOER_MODE_OFF,
	FLASH_PWOER_MODE_LOW_VOLT,
	FLASH_PWOER_MODE_HIGH_VOLT,
	FLASH_PWOER_MODE_MAX,
};

struct fled_volt_t {
	int fled_volt;
};

struct high_vib_data_t {
	struct device *dev;
	struct led_classdev vib_dev;
	struct regulator *ldo;
	int vib_n_pin;
	int pwm_no;
	unsigned int duty_ratio;
	unsigned int pwm_freq;
	unsigned int duration;
	enum flash_power_mode_t fled_volt;
	unsigned int battery_ratio;
	int state;
	struct notifier_block flash_notifier_block;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct fled_adapt_work;
	struct mutex lock;
	struct wakeup_source *ws;
};

enum high_volt_haptic_status {
	HIGH_VOLT_HAPTIC_STOP = 0,
	HIGH_VOLT_HAPTIC_START,
	HIGH_VOLT_FULL_PWM_HAPTIC_START,
	HIGH_VOLT_RATIO_PWM_HAPTIC_START,
	HIGH_VOLT_RATIO_TO_RESERVE,
	HIGH_VOLT_HAPTIC_RESERVE,
	HIGH_VOLT_HAPTIC_DEFAULT,
};

#define null_pointer_err_check_ret0(a) \
	do { \
		if (!a) \
			return 0; \
	} while (0)

#define null_pointer_err_check(a) \
	do { \
		if (!a) \
			return; \
	} while (0)

#endif

