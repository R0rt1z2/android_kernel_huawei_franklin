/*
 * platform_pd_status.h
 *
 * fusb30X pd status head file
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

#ifndef __FUSB_PLATFORM_PD_STATUS_H_
#define __FUSB_PLATFORM_PD_STATUS_H_

int fusb_get_sink_ibus(void);
bool fusb_get_pd_ready_flag(void);
bool fusb_get_pd_sink_vbus_flag(void);
void fusb_force_source(void);
void fusb_force_sink(void);

#endif // __FUSB_PLATFORM_PD_STATUS_H_
