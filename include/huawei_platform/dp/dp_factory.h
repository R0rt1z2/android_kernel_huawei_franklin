/*
 * dp_factory.h
 *
 * dp factory test driver
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

#ifndef __DP_FACTORY_H__
#define __DP_FACTORY_H__

#include <linux/types.h>

enum dp_manufacture_link_state {
	DP_MANUFACTURE_LINK_CABLE_IN,
	DP_MANUFACTURE_LINK_CABLE_OUT,
	DP_MANUFACTURE_LINK_AUX_FAILED,
	DP_MANUFACTURE_LINK_SAFE_MODE,
	DP_MANUFACTURE_LINK_EDID_FAILED,
	DP_MANUFACTURE_LINK_LINK_FAILED,
	DP_MANUFACTURE_LINK_HPD_NOT_EXISTED,
	DP_MANUFACTURE_LINK_REDUCE_RATE,
	DP_MANUFACTURE_LINK_INVALID_COMBINATIONS, // combinations not support 4k

	DP_MANUFACTURE_LINK_STATE_MAX,
};

void dp_factory_send_event(enum dp_manufacture_link_state event);
bool dp_factory_mode_enable(void);
void dp_factory_get_lane_rate(uint8_t rate, uint8_t lanes,
	uint8_t max_rate, uint8_t max_lanes);
void dp_factory_get_h_v_active(uint16_t hac, uint16_t vac, uint8_t fps);

#endif // __DP_FACTORY_H__