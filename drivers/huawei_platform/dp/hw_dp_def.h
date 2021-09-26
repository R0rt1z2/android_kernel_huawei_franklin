/*
 * hw_dp_def.h
 *
 * hw dp driver head
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

#ifndef __HW_DP_DEF_H__
#define __HW_DP_DEF_H__

#include <linux/of.h>
#include <huawei_platform/log/hw_log.h>
#include <securec.h>

#define DP_LINK_EVENT_BUF_MAX  64
#define DP_FACTORY_H_ACTIVE  3840
#define DP_FACTORY_V_ACTIVE  2160

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)  (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

enum dp_source_mode {
	DIFF_SOURCE = 0,
	SAME_SOURCE,
};

extern uint8_t g_dp_msg_level;
extern bool g_enable_send_event;

/*
 * Message printing priorities:
 * LEVEL 0 KERN_ERR (highest priority)
 * LEVEL 1 KERN_WARNING
 * LEVEL 2 KERN_NOTICE
 * LEVEL 3 KERN_INFO
 * LEVEL 4 KERN_DEBUG (Lowest priority)
 */
#define HW_DP_ERR(msg, ...) \
	do { \
		if (g_dp_msg_level > 0) \
			hwlog_err("%s: "msg, __func__, ## __VA_ARGS__); \
	} while (0)

#define HW_DP_WARNING(msg, ...) \
	do { \
		if (g_dp_msg_level > 1) \
			hwlog_warn("%s: "msg, __func__, ## __VA_ARGS__); \
	} while (0)

#define HW_DP_NOTICE(msg, ...) \
	do { \
		if (g_dp_msg_level > 2) \
			hwlog_info("%s: "msg, __func__, ## __VA_ARGS__); \
	} while (0)

#define HW_DP_INFO(msg, ...)   \
	do { \
		if (g_dp_msg_level > 3) \
			hwlog_info("%s: "msg, __func__, ## __VA_ARGS__); \
	} while (0)

#define HW_DP_DEBUG(msg, ...)  \
	do { \
		if (g_dp_msg_level > 4) \
			hwlog_info("%s: "msg, __func__, ## __VA_ARGS__); \
	} while (0)

#define DP_GET_BOOL_DTS_PROP(v, n, c) \
	do { \
		if (of_property_read_bool(n, #c)) { \
			v = true; \
			HW_DP_INFO("%s configed\n", #c); \
		} else { \
			v = false; \
			HW_DP_INFO("%s not configed\n", #c); \
		} \
	} while (0)

#define DP_GET_U32_DTS_PROP(v, n, c) \
	do { \
		uint32_t out; \
		if (of_property_read_u32(n, #c, &out) == 0) { \
			v = out; \
			HW_DP_INFO("get prop %s:%d\n", #c, out); \
		} else { \
			v = 0; \
			HW_DP_INFO("can not get %s prop\n", #c); \
		} \
	} while (0)

void dp_state_event_report(const char *event);

#endif // __HW_DP_DEF_H__

