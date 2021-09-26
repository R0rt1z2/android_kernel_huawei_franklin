/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: headset auto calibration
 * Author: wanggang
 * Create: 2019-11-20
 */

#include "hs_auto_calib.h"
#include <linux/of.h>
#include "accdet.h"

#define DEFAULT_ERROR 35

static bool g_hs_auto_calib_enable;
static struct btn_voltage g_btn_voltage;
static int g_vol_btn = KEY_VOL_UP;
static int g_vol_record[KEY_TYPE_MAX] = {-1};
static int g_vol_distance = DEFAULT_ERROR;
static int g_vol_error[KEY_TYPE_MAX] = {
	DEFAULT_ERROR,
	DEFAULT_ERROR,
	DEFAULT_ERROR
};

static unsigned int get_property_key_value(struct device_node *dev_node,
	const char *name, unsigned int def_value)
{
	int ret;
	unsigned int value = 0;

	ret = of_property_read_u32(dev_node, name, &value);
	if (ret != 0) {
		value = def_value;
		pr_info("%s %s use default value %u", __func__, name, value);
	}

	return value;
}

void headset_auto_calib_init(struct device_node *dev_node)
{
	const char *status = NULL;
	struct device_node *hs_node = NULL;

	g_hs_auto_calib_enable = false;

	if (dev_node == NULL)
		return;
	hs_node = of_get_child_by_name(dev_node, "hs_auto_calib");
	if (hs_node == NULL)
		return;
	if (of_property_read_string(hs_node, "status", &status) != 0)
		return;
	if (strcmp(status, "ok") != 0)
		return;

	pr_info("%s headset auto calibration enable", __func__);
	g_hs_auto_calib_enable = true;
	g_btn_voltage.key_media_min_value = get_property_key_value(hs_node,
		"key_media_min_value",
		(unsigned int)KEY_MEDIA_MIN_VALUE);
	g_btn_voltage.key_media_max_value = get_property_key_value(hs_node,
		"key_media_max_value",
		(unsigned int)KEY_MEDIA_MAX_VALUE);
	g_btn_voltage.key_vol_up_min_value = get_property_key_value(hs_node,
		"key_vol_up_min_value",
		(unsigned int)KEY_VOL_UP_MIN_VALUE);
	g_btn_voltage.key_vol_up_max_value = get_property_key_value(hs_node,
		"key_vol_up_max_value",
		(unsigned int)KEY_VOL_UP_MAX_VALUE);
	g_btn_voltage.key_vol_down_min_value = get_property_key_value(hs_node,
		"key_vol_down_min_value",
		(unsigned int)KEY_VOL_DOWN_MIN_VALUE);
	g_btn_voltage.key_vol_down_max_value = get_property_key_value(hs_node,
		"key_vol_down_max_value",
		(unsigned int)KEY_VOL_DOWN_MAX_VALUE);
}

void headset_auto_calib_reset_interzone(void)
{
	int i;

	for (i = KEY_VOL_UP; i < KEY_TYPE_MAX; i++) {
		g_vol_record[i] = -1;
		g_vol_error[i] = DEFAULT_ERROR;
	}
	g_vol_btn = KEY_VOL_UP;
}

static int check_headset_type(unsigned int vol_value)
{
	int press_key = KEY_TYPE_MAX;

	if ((vol_value >= g_btn_voltage.key_media_min_value) &&
		(vol_value <= g_btn_voltage.key_media_max_value))
		press_key = KEY_HOOK;
	else if ((vol_value >= g_btn_voltage.key_vol_up_min_value) &&
		(vol_value <= g_btn_voltage.key_vol_up_max_value))
		press_key = KEY_VOL_UP;
	else if ((vol_value >= g_btn_voltage.key_vol_down_min_value) &&
		(vol_value <= g_btn_voltage.key_vol_down_max_value))
		press_key = KEY_VOL_DOWN;
	else
		pr_info("%s volume value %u is not in range",
			__func__, vol_value);

	pr_info("%s press_key %d", __func__, press_key);
	return press_key;
}

static void set_headset_type(unsigned int *key_code, int press_key)
{
	if (press_key == KEY_VOL_UP)
		*key_code = UP_KEY;
	else if (press_key == KEY_VOL_DOWN)
		*key_code = DW_KEY;
	else
		pr_info("%s no need to update key code", __func__);

	pr_info("%s press_key %d,key_code %u", __func__, press_key, *key_code);
}

static void update_vol_error(void)
{
	int temp;
	int vol_distance;

	pr_info("%s begin volume up error %d,volume down error %d", __func__,
		g_vol_error[KEY_VOL_UP], g_vol_error[KEY_VOL_DOWN]);

	temp = g_vol_record[KEY_VOL_DOWN] - g_vol_record[KEY_VOL_UP];
	g_vol_distance = (g_vol_distance > temp) ? g_vol_distance : temp;

	/* Volume distance between down key and up key */
	vol_distance = g_vol_distance / 2;
	/* Volume distance between up key and media key */
	temp = (g_vol_record[KEY_VOL_UP] -
		g_btn_voltage.key_media_max_value) / 2;

	g_vol_error[KEY_VOL_UP] = (vol_distance > temp) ? temp : vol_distance;
	g_vol_error[KEY_VOL_DOWN] = g_vol_distance - g_vol_error[KEY_VOL_UP];

	pr_info("%s end volume up error %d,volume down error %d", __func__,
		g_vol_error[KEY_VOL_UP], g_vol_error[KEY_VOL_DOWN]);
}

static void vol_record_sort(int vol_value)
{
	int temp;

	pr_info("%s begin volume value %d", __func__, vol_value);
	if (vol_value < g_btn_voltage.key_media_max_value) {
		g_vol_record[KEY_HOOK] = vol_value;
		return;
	}

	switch (g_vol_btn) {
	case KEY_VOL_UP:
		g_vol_record[KEY_VOL_UP] = vol_value;
		g_vol_btn = KEY_VOL_DOWN;
		break;
	case KEY_VOL_DOWN:
		if (vol_value > g_vol_record[KEY_VOL_UP]) {
			g_vol_record[KEY_VOL_DOWN] = vol_value;
		} else {
			temp = g_vol_record[KEY_VOL_UP];
			g_vol_record[KEY_VOL_UP] = vol_value;
			g_vol_record[KEY_VOL_DOWN] = temp;
		}
		g_vol_btn = KEY_TYPE_MAX;
		update_vol_error();
		break;
	case KEY_TYPE_MAX:
		if (vol_value < g_vol_record[KEY_VOL_UP])
			g_vol_record[KEY_VOL_UP] = vol_value;
		else if (vol_value > g_vol_record[KEY_VOL_DOWN])
			g_vol_record[KEY_VOL_DOWN] = vol_value;
		update_vol_error();
		break;
	default:
		break;
	}

	pr_info("%s end volume up record %d,volume down record %d", __func__,
		g_vol_record[KEY_VOL_UP], g_vol_record[KEY_VOL_DOWN]);
}

static void readjust_interzone(void)
{
	/* Formula for calculating vol_up_max_value */
	g_btn_voltage.key_vol_up_max_value =
		(unsigned int)((g_vol_record[KEY_VOL_UP] +
		g_vol_record[KEY_VOL_DOWN]) / 2);

	/* Formula for calculating vol_down_min_value */
	g_btn_voltage.key_vol_down_min_value =
		(unsigned int)(g_btn_voltage.key_vol_up_max_value + 1);

	pr_info("interzone readjust to volume up max %u,volume down min %u",
		g_btn_voltage.key_vol_up_max_value,
		g_btn_voltage.key_vol_down_min_value);
}

static bool is_voltage_error(int vol_value)
{
	int i;

	for (i = KEY_VOL_UP; i < g_vol_btn; i++) {
		if (abs(vol_value - g_vol_record[i]) < g_vol_error[i])
			return true;
	}

	return false;
}

static bool is_vol_record_empty(void)
{
	int i;

	for (i = KEY_VOL_UP; i < g_vol_btn; i++) {
		if (g_vol_record[i] != -1)
			return false;
	}
	return true;
}

void start_headset_auto_calib(enum adjust_state state, unsigned int vol_value,
	unsigned int *key_code)
{
	int press_key;

	if (key_code == NULL)
		return;
	if (!g_hs_auto_calib_enable)
		return;
	if (vol_value > g_btn_voltage.key_vol_down_max_value) {
		pr_info("volume value %u is bigger than max value", vol_value);
		return;
	}

	pr_info("%s state %d,volume value %u", __func__, state, vol_value);
	while (true) {
		switch (state) {
		case REC_JUDGE:
			if (is_vol_record_empty())
				state = BTN_RECORD;
			else
				state = ERROR_JUDGE;
			break;
		case ERROR_JUDGE:
			if (is_voltage_error((int)vol_value))
				state = BTN_REPORT;
			else
				state = BTN_RECORD;
			break;
		case BTN_RECORD:
			vol_record_sort((int)vol_value);
			if (g_vol_btn == KEY_TYPE_MAX)
				state = VOL_INTERZONE_READJUST;
			else
				state = BTN_REPORT;
			break;
		case VOL_INTERZONE_READJUST:
			readjust_interzone();
			state = BTN_REPORT;
			break;
		case BTN_REPORT:
			press_key = check_headset_type(vol_value);
			set_headset_type(key_code, press_key);
			return;
		default:
			break;
		}
	}
}

