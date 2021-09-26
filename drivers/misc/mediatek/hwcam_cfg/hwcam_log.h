
/*
 * hwcam_log.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * hwcam log header file
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

#ifndef __HW_ALAN_KERNEL_CAM_LOG_H__
#define __HW_ALAN_KERNEL_CAM_LOG_H__

#include <linux/types.h>

/* #define CAMERA_LOG_DEBUG */

#ifdef CAMERA_LOG_DEBUG
#define cam_debug(fmt, ...) \
	do { \
		printk("[CAMERA]" "DEBUG: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define cam_info(fmt, ...) \
	do { \
		printk("[CAMERA]" "INFO: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define cam_warn(fmt, ...) \
	do { \
		printk("[CAMERA]" "WARN: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define cam_err(fmt, ...) \
	do { \
		printk("[CAMERA]" "ERROR: " fmt "\n", ##__VA_ARGS__); \
	} while (0)
#else /* CAMERA_LOG_DEBUG */
#define cam_debug(fmt, ...)

#define cam_info(fmt, ...) \
	do { \
		printk("[CAMERA]" "INFO: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define cam_warn(fmt, ...) \
	do { \
		printk("[CAMERA]" "WARN: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define cam_err(fmt, ...) \
	do { \
		printk("[CAMERA]" "ERROR: " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#endif /* CAMERA_LOG_DEBUG */

#endif /* __HW_ALAN_KERNEL_CAM_LOG_H__ */