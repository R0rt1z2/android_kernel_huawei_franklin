/*
 * wireless_charge_ictrl.c
 *
 * wireless charge driver, function as buck iout ctrl
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/power/wireless/wireless_charger.h>

#define HWLOG_TAG wireless_charge_ictrl
HWLOG_REGIST();

static int g_charger_ilim;
static int g_input_current_step;
static int g_input_current_delay;
static int g_input_current_now;
static int g_iin_set_complete_flag;

extern int charger_set_input_current(u32 uA);

void wlc_set_iin_prop(int iin_step, int iin_delay)
{
	g_input_current_delay = iin_delay;
	g_input_current_step = iin_step;
	g_input_current_now = 100; /* generally, default iout is 100mA */

	hwlog_info("[%s] iin_delay = %d iin_step = %d\n",
		__func__, g_input_current_delay, g_input_current_step);
}

static void wlc_set_iin_step_by_step(int iin)
{
	int ret;
	int iin_tmp = g_input_current_now;

	g_iin_set_complete_flag = 0;
	if (iin > g_input_current_now) {
		do {
			ret = charger_set_input_current((u32)iin_tmp * 1000); /* uA */
			if (ret < 0)
				hwlog_err("set iin %dmA failed\n", iin_tmp);
			g_input_current_now = iin_tmp;
			iin_tmp += g_input_current_step;
			if (!g_iin_set_complete_flag)
				msleep(g_input_current_delay);
		} while ((iin_tmp < iin) && !g_iin_set_complete_flag &&
			g_input_current_delay && g_input_current_step);
	}
	if (iin < g_input_current_now) {
		do {
			ret = charger_set_input_current((u32)iin_tmp * 1000); /* uA */
			if (ret < 0)
				hwlog_err("set iin %dmA failed\n", iin_tmp);
			g_input_current_now = iin_tmp;
			iin_tmp -= g_input_current_step;
			if (!g_iin_set_complete_flag)
				msleep(g_input_current_delay);
		} while ((iin_tmp > iin) && !g_iin_set_complete_flag &&
			g_input_current_delay && g_input_current_step);
	}
	if (!g_iin_set_complete_flag) {
		g_input_current_now = iin;
		ret = charger_set_input_current((u32)iin_tmp * 1000); /* uA */
		if (ret < 0)
			hwlog_err("set iin %dmA failed\n", iin);
		g_iin_set_complete_flag = 1;
	}
}

static void wlc_set_iin(int iin)
{
	if (!g_input_current_delay || !g_input_current_step)
		return;

	if (!g_iin_set_complete_flag) {
		g_iin_set_complete_flag = 1;
		/* delay double time for completion */
		msleep(2 * g_input_current_delay);
	}
	wlc_set_iin_step_by_step(iin);
}

void wlc_set_rx_iout_limit(int ilim)
{
	wlc_set_iin(min(ilim, g_charger_ilim));
}

void wlc_ictrl_set_input_current(int iin)
{
	g_charger_ilim = iin;
	wlc_set_iin(min(iin, wireless_charge_get_rx_iout_limit()));
}

int wlc_get_charger_iinlim_regval(void)
{
	return g_input_current_now;
}
