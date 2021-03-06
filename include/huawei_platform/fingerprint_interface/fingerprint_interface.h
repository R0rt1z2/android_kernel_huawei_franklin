/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef FINGERPRINT_INTERFACE_H
#define FINGERPRINT_INTERFACE_H

void ud_fp_on_hbm_completed(void);

void fp_set_lcd_charge_time(int time);

void fp_set_lcd_light_on_time(int time);

void fp_set_cpu_wake_up_time(int time);

#endif
