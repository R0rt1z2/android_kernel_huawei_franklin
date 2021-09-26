/*
 * camkit_log.h
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: module log define
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

#ifndef CAMKIT_LOG_H
#define CAMKIT_LOG_H

enum camkit_log_module {
	CAMKIT_LOG_DRIVER_ADAPTER = (0x00000001 << 0),
	CAMKIT_LOG_IO_ADAPTER     = (0x00000001 << 1),
	CAMKIT_LOG_I2C            = (0x00000001 << 2),
	CAMKIT_LOG_PROBE_SENSOR   = (0x00000001 << 3),
	CAMKIT_LOG_DRIVER_IMP     = (0x00000001 << 4),
	CAMKIT_LOG_DRIVER_3A      = (0x00000001 << 5),
	CAMKIT_LOG_DRIVER_INTF    = (0x00000001 << 6),
	CAMKIT_LOG_DTS            = (0x00000001 << 7),
};

extern uint32 g_log_mask;

#undef log_dbg
#define log_dbg(mod, fmt, args...) \
	do { \
		if (g_log_mask & mod) \
			pr_info("[%s:%d]" fmt, __func__, __LINE__, ##args); \
	} while (0)

#undef adapt_dbg
#define adapt_dbg(fmt, args...) log_dbg(CAMKIT_LOG_DRIVER_ADAPTER, fmt, ##args)

#undef io_dbg
#define io_dbg(fmt, args...) log_dbg(CAMKIT_LOG_IO_ADAPTER, fmt, ##args)

#undef i2c_dbg
#define i2c_dbg(fmt, args...) log_dbg(CAMKIT_LOG_I2C, fmt, ##args)

#undef probe_dbg
#define probe_dbg(fmt, args...) log_dbg(CAMKIT_LOG_PROBE_SENSOR, fmt, ##args)

#undef drv_dbg
#define drv_dbg(fmt, args...) log_dbg(CAMKIT_LOG_DRIVER_IMP, fmt, ##args)

#undef aaa_dbg
#define aaa_dbg(fmt, args...) log_dbg(CAMKIT_LOG_DRIVER_3A, fmt, ##args)

#undef intf_dbg
#define intf_dbg(fmt, args...) log_dbg(CAMKIT_LOG_DRIVER_INTF, fmt, ##args)

#undef dts_dbg
#define dts_dbg(fmt, args...) log_dbg(CAMKIT_LOG_DTS, fmt, ##args)

#undef log_info
#define log_info(fmt, args...) pr_info("[%s:%d]" fmt, __func__, __LINE__, ##args)

#undef log_err
#define log_err(fmt, args...) pr_err("[%s:%d]" fmt, __func__, __LINE__, ##args)

#endif // CAMKIT_LOG_H
