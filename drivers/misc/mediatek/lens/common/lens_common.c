/*
 * lens_common.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * lens common interface
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

#include <linux/mutex.h>
#include <linux/delay.h>
#include <lens_common.h>

const char *g_vcm_error_info[VCM_ERROR_TYPE_END] = {
	"(i2c write fail)",
	"(i2c read fail)",
};

int lens_set_setting(struct i2c_client *client,
	const struct i2c_cfg_t *i2c_cfg, unsigned int size)
{
	unsigned int i;
	if (!client || !i2c_cfg) {
		log_err("prams is NULL");
		return -1;
	}

	for (i = 0; i < size; ++i) {
		if (i2c_master_send(client, i2c_cfg[i].setting,
			array_size(i2c_cfg[i].setting)) < 0)
			return -EIO;

		if (i2c_cfg[i].delay > 0)
			mdelay(i2c_cfg[i].delay);
	}
	return 0;
}

void lens_hiview_handle(int error_no, const char *ic_name,
	const char *module_name, vcm_error_type error_type)
{
	errno_t ret;
	const char *error_info = NULL;
	struct hwcam_hievent_info lens_info;

	ret = memset_s(&lens_info, sizeof(lens_info), 0, sizeof(lens_info));
	if (ret != EOK) {
		pr_err("lens_info memset_s fail, ret = %d\n", ret);
		return;
	}

	if (error_type >= VCM_ERROR_TYPE_START &&
		error_type < VCM_ERROR_TYPE_END)
		error_info = g_vcm_error_info[error_type];

	lens_info.error_no = error_no;
	hwcam_hiview_get_ic_name(ic_name, &lens_info);
	hwcam_hiview_get_module_name(module_name, &lens_info);
	hwcam_hiview_get_content(error_no, error_info, &lens_info);
	hwcam_hiview_report(&lens_info);
}
