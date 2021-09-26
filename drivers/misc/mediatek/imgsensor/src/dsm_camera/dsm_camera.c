/*
 * dsm_camera.c
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
#include "dsm_camera.h"
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/string.h>
#include "securec.h"

static struct dsm_client *cam_dsm_client;

static struct dsm_dev dsm_camera = {
	.name = "dsm_camera",
	.fops = NULL,
	.buff_size = DSM_CAMERA_BUFF_SIZE,
};

int camera_dsm_report_info(int err_code, char *fmt, ...)
{
	int ret;
	char err_msg[DSM_CAMERA_BUFF_SIZE] = {0};
	va_list args;

	if (!cam_dsm_client) {
		return -EINVAL;
	}
	pr_info("dsm_camera err code: %d begin", err_code);
	va_start(args, fmt);
	ret = vsnprintf_s(err_msg, DSM_CAMERA_BUFF_SIZE, DSM_CAMERA_BUFF_SIZE - 1, fmt, args);
	va_end(args);
	if (ret < 0) {
		pr_err("dsm_camera report info error\n");
		goto EXIT;
	}

	ret = dsm_client_ocuppy(cam_dsm_client);
	if (ret) {
		pr_err("dsm_camera buf is busy\n");
		goto EXIT;
	}

	dsm_client_record(cam_dsm_client, err_msg);
	dsm_client_notify(cam_dsm_client, err_code);

EXIT:
	pr_info("dsm_camera err code: %d end", err_code);
	return ret;
}
EXPORT_SYMBOL(camera_dsm_report_info);

static int camera_dsm_init(void)
{
	if (!cam_dsm_client) {
		cam_dsm_client = dsm_register_client(&dsm_camera);
		if (!cam_dsm_client) {
			return -ENODEV;
		}
	}

	return 0;
}


static void camera_dsm_deinit(void)
{
	if (!cam_dsm_client) {
		return;
	}

	dsm_unregister_client(cam_dsm_client, &dsm_camera);
}

subsys_initcall_sync(camera_dsm_init);
module_exit(camera_dsm_deinit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Huawei camera dsm driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");