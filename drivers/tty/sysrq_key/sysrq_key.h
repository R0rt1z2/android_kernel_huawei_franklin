/*
 * sysrq_key.h
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
#ifndef SYSRQ_KEY_H_
#define SYSRQ_KEY_H_
#include <linux/input.h>
struct sysrq_state {
	struct input_handle handle;
	struct work_struct reinject_work;
	unsigned long key_down[BITS_TO_LONGS(KEY_CNT)];
	unsigned int alt;
	unsigned int alt_use;
	bool active;
	bool need_reinject;
	bool reinjecting;
	/* reset sequence handling */
	bool reset_canceled;
	bool reset_requested;
	unsigned long reset_keybit[BITS_TO_LONGS(KEY_CNT)];
	int reset_seq_len;
	int reset_seq_cnt;
	int reset_seq_version;
	struct timer_list keyreset_timer;
};
bool sysrq_handle_keypress(struct sysrq_state *sysrq,
			   unsigned int code, int value);
void sysrq_key_init(void);
#endif
#endif
