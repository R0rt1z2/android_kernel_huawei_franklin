/*
 * hwcam_hiview.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * hwcam hiview header file
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

#ifndef HWCAM_HIVIEW_H
#define HWCAM_HIVIEW_H

#include <log/hiview_hievent.h>
#include <dsm/dsm_pub.h>

#define HIVIEW_MAX_IC_NAME_LEN 32
#define HIVIEW_MAX_MODULE_NAME_LEN 32
#define HIVIEW_MAX_CONTENT_LEN 256

struct hwcam_hievent_content {
	int error_no;
	const char *content;
};

struct hwcam_hievent_info {
	int error_no;
	char ic_name[HIVIEW_MAX_IC_NAME_LEN];
	char module_name[HIVIEW_MAX_MODULE_NAME_LEN];
	char content[HIVIEW_MAX_CONTENT_LEN];
};

void hwcam_hiview_get_ic_name(const char *ic_name,
	struct hwcam_hievent_info *cam_info);
void hwcam_hiview_get_module_name(const char *module_name,
	struct hwcam_hievent_info *cam_info);
void hwcam_hiview_get_content(int error_no, const char *error_info,
	struct hwcam_hievent_info *cam_info);
void hwcam_hiview_report(struct hwcam_hievent_info *cam_info);

#endif // HWCAM_HIVIEW_H
