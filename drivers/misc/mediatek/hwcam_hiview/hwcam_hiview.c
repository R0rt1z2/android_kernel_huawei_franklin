/*
 * hwcam_hiview.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera DFT interface
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

#include <hwcam_hiview.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <securec.h>

#define PFX "[camhiview]"
#define log_inf(fmt, args...) pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define log_err(fmt, args...) pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define loge_if(x) \
	do { \
		int ret = (x); \
		if (ret) \
			log_err("'%s' failed, ret = %d", #x, ret); \
	} while (0)
#define return_on_null(x) \
	do { \
		if (!x) { \
			log_err("invalid params, %s", #x); \
			return; \
		} \
	} while (0)
#define array_size(x) (sizeof(x) / sizeof((x)[0]))

static DEFINE_MUTEX(g_cam_hiview_lock);
static const struct hwcam_hievent_content g_content[] = {
	{ DSM_CAMERA_I2C_ERR, "sensor i2c error" },
	{ DSM_CAMERA_OTP_I2C_ERR, "eeprom i2c error" },
	{ DSM_FLASH_I2C_ERROR_NO, "flash i2c transfer fail" },
	{ DSM_FLASH_HOT_DIE_ERROR_NO, "flash temperature is too hot" },
	{ DSM_FLASH_OPEN_SHOTR_ERROR_NO, "flash ovp, vout or led short" },
	{ DSM_FLASH_UNDER_VOLTAGE_LOCKOUT_ERROR_NO, "flash under voltage lockout threshold" },
	{ DSM_CAMERA_ACTUATOR_INIT_FAIL, "vcm init fail" },
	{ DSM_CAMERA_ACTUATOR_MOVE_FAIL, "vcm set dest code fail" },
	{ DSM_CAMERA_OIS_INIT_FAIL, "ois init fail" },
	{ DSM_CAMPMIC_I2C_ERROR_NO, "pmic i2c error" },
	{ DSM_CAMPMIC_OVER_CURRENT_ERROR_NO, "pmic over current or voltage error" },
	{ DSM_CAMPMIC_UNDER_VOLTAGE_ERROR_NO, "pmic under voltage lockout error" },
};

void hwcam_hiview_get_ic_name(const char *ic_name,
	struct hwcam_hievent_info *cam_info)
{
	return_on_null(ic_name);
	return_on_null(cam_info);

	loge_if(strncpy_s(cam_info->ic_name, sizeof(cam_info->ic_name),
		ic_name, sizeof(cam_info->ic_name) - 1));
}

void hwcam_hiview_get_module_name(const char *module_name,
	struct hwcam_hievent_info *cam_info)
{
	return_on_null(module_name);
	return_on_null(cam_info);

	loge_if(strncpy_s(cam_info->module_name, sizeof(cam_info->module_name),
		module_name, sizeof(cam_info->module_name) - 1));
}

static const char* hwcam_dsm_get_content(int error_no)
{
	unsigned int i;

	for (i = 0; i < array_size(g_content); ++i)
		if (error_no == g_content[i].error_no)
			return g_content[i].content;

	log_inf("not match content");
	return NULL;
}

void hwcam_hiview_get_content(int error_no, const char *error_info,
	struct hwcam_hievent_info *cam_info)
{
	char total_info[HIVIEW_MAX_CONTENT_LEN] = {0};
	const char *dsm_content = NULL;

	return_on_null(cam_info);
	dsm_content = hwcam_dsm_get_content(error_no);
	return_on_null(dsm_content);

	if (snprintf_s(total_info, sizeof(total_info),
		sizeof(total_info) - 1, "Info[ic(%s), module(%s)], ",
		cam_info->ic_name, cam_info->module_name) < 0)
		log_err("total_info snprintf_s fail");

	loge_if(strcat_s(total_info, sizeof(total_info), dsm_content));

	if (error_info)
		loge_if(strcat_s(total_info, sizeof(total_info), error_info));

	loge_if(strncpy_s(cam_info->content, sizeof(cam_info->content),
		total_info, sizeof(cam_info->content) - 1));
}

void hwcam_hiview_report(struct hwcam_hievent_info *cam_info)
{
	struct hiview_hievent *hi_event = NULL;
	return_on_null(cam_info);

	mutex_lock(&g_cam_hiview_lock);
	hi_event = hiview_hievent_create(cam_info->error_no);
	if (!hi_event) {
		log_err("create hievent fail");
		mutex_unlock(&g_cam_hiview_lock);
		return;
	}

	hiview_hievent_put_string(hi_event, "IC_NAME", cam_info->ic_name);
	hiview_hievent_put_string(hi_event, "MODULE_NAME", cam_info->module_name);
	hiview_hievent_put_string(hi_event, "CONTENT", cam_info->content);

	if (hiview_hievent_report(hi_event) <= 0) {
		log_err("report failed");
		hiview_hievent_destroy(hi_event);
		mutex_unlock(&g_cam_hiview_lock);
		return;
	}
	hiview_hievent_destroy(hi_event);
	mutex_unlock(&g_cam_hiview_lock);

	log_inf("report succ");
}
