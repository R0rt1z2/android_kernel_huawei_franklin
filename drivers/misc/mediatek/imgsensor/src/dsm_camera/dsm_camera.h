/*
 * dsm_camera.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera dsm config
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

#ifndef __DSM_CAMERA_H
#define __DSM_CAMERA_H

#include <dsm/dsm_pub.h>

#define DSM_CAMERA_BUFF_SIZE (256)

#ifdef CONFIG_HUAWEI_DSM_CAMERA

int camera_dsm_report_info(int err_code, char *fmt, ...);

#else

static inline int camera_dsm_report_info(int err_code, char *fmt, ...)
{
	return 0;
}

#endif

#endif /* __DSM_CAMERA_H */