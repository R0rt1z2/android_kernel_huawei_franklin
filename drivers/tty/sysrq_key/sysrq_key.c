/*
 * sysrq_key.c
 *
 * volumedown + volumeup + power for sysrq function.
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#ifdef CONFIG_HANDSET_SYSRQ_RESET
#include "sysrq_key.h"
#include <linux/sysrq.h>
#include <linux/printk.h>
#ifdef CONFIG_FASTBOOT_DUMP
#include <linux/fastboot_dump_reason_api.h>
#endif
#ifdef CONFIG_RAINBOW_RESET_DETECT
#include <linux/rainbow_reset_detect_api.h>
#endif

bool sysrq_down;
int sysrq_alt_use;
int sysrq_alt;

void sysrq_key_init(void)
{
	sysrq_down = false;
	sysrq_alt = 0;
}

bool sysrq_handle_keypress(struct sysrq_state *sysrq,
			   unsigned int code, int value)
{
	switch (code) {
	case KEY_VOLUMEDOWN:
		/* identify volumedown pressed down or not */
		if (value) {
			sysrq_alt = code;
		} else {
			/* volume- release clear syrq_down&syrq_alt state */
			if (sysrq_down && code == sysrq_alt_use)
				sysrq_down = false;
			sysrq_alt = 0;
		}
		break;
	case KEY_VOLUMEUP:
		/* identify volumeup pressed down or not */
		if (value == 1 && sysrq_alt) {
			sysrq_down = true;
			sysrq_alt_use = sysrq_alt;
		} else {
			sysrq_down = false;
			sysrq_alt_use = 0;
		}
		break;
	case KEY_POWER:
		/* identify power pressed down or not */
		if (sysrq_down && value && value != 2) {
			pr_info("trigger system crash by sysrq\n");
#ifdef CONFIG_FASTBOOT_DUMP
			fastboot_dump_m_reason_set(FD_M_NORMALBOOT);
			fastboot_dump_s_reason_set(FD_S_NORMALBOOT_COMBIN_KEY);
			fastboot_dump_s_reason_str_set("Combin_Key");
#endif
#ifdef CONFIG_RAINBOW_RESET_DETECT
			rainbow_reset_detect_m_reason_set(FD_M_NORMAL);
			rainbow_reset_detect_s_reason_set(FD_S_COMBIN_KEY);
			rainbow_reset_detect_s_reason_str_set("Combin_Key");
#endif
			__handle_sysrq('c', true);
		} else {
			sysrq_down = false;
			sysrq_alt_use = 0;
		}
		break;
	default:
		break;
	}
	return sysrq_down;
}

static int __init reboot_reason_print(char *p)
{
	if (p == NULL) {
		pr_err("%s: input null\n", __func__);
		return -EINVAL;
	}
	pr_info("%s: reboot_reason=%s\n", __func__, p);
	return 0;
}
early_param("reboot_reason", reboot_reason_print);
#endif
