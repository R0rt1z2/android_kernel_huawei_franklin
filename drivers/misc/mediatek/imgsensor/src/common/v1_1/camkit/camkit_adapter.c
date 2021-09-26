/*
 * camkit_adapter.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camkit interface adapted on the platform
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <securec.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CCU
#include "ccu_imgsensor_if.h"
#endif

#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"
#include "imgsensor.h"

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
#include "imgsensor_ca.h"
#endif

#include "camkit_driver.h"
#include "kd_camkit_define.h"

#include "camkit_adapter.h"
#include "camkit_check_cable.h"

#define PFX "[camkit_adapter]"
#define DEBUG_CAMKIT 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_CAMKIT) \
			pr_info(PFX fmt, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX fmt, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX fmt, ##args)

extern struct IMGSENSOR gimgsensor;

extern void IMGSENSOR_PROFILE_INIT(struct timeval *ptv);
extern void IMGSENSOR_PROFILE(struct timeval *ptv, char *tag);
extern void imgsensor_mutex_init(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_lock(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_unlock(struct IMGSENSOR_SENSOR_INST *psensor_inst);

struct camkit_sensor_index_info_t g_sensor_idx_info[IMGSENSOR_SENSOR_IDX_MAX_NUM];
struct camkit_sensor_power_settings_idx_map_t g_power_setting_map[IMGSENSOR_SENSOR_IDX_MAX_NUM];

struct IMGSENSOR_SENSOR *camkit_get_sensor(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
		idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return NULL;
	else
		return &gimgsensor.sensor[idx];
}

static int camkit_print_mode_info(struct camkit_mode_info *mode_info)
{
	LOG_DBG("pclk: %u", mode_info->pclk);
	LOG_DBG("linelength: %u", mode_info->linelength);
	LOG_DBG("framelength: %u", mode_info->framelength);
	LOG_DBG("startx: %u", mode_info->startx);
	LOG_DBG("starty: %u", mode_info->starty);
	LOG_DBG("grabwindow_width: %u", mode_info->grabwindow_width);
	LOG_DBG("grabwindow_height: %u", mode_info->grabwindow_height);
	LOG_DBG("mipi_settle_dc: %u", mode_info->mipi_data_lp2hs_settle_dc);
	LOG_DBG("max_framerate: %u", mode_info->max_framerate);
	LOG_DBG("mipi_pixel_rate: %u", mode_info->mipi_pixel_rate);
	LOG_DBG("mipi_trail_val: %u", mode_info->mipi_trail_val);

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_setting(
	struct camkit_i2c_reg_setting *i2c_setting)
{
	int len;
	int i;
	struct camkit_i2c_reg *setting = NULL;

	if (i2c_setting == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	if (!i2c_setting->size || i2c_setting->setting == NULL) {
		pr_info("[camkit][%s] setting need't configure?\n", __func__);
		i2c_setting->setting = NULL;
		return ERROR_NONE;
	}

	LOG_DBG("sensor setting size: %u\n", i2c_setting->size);

	len = i2c_setting->size;
	setting = kzalloc(sizeof(struct camkit_i2c_reg) * len, GFP_KERNEL);
	if (!setting) {
		pr_err("[%s] memory not enough\n", __func__);
		i2c_setting->setting = NULL;
		return ENOMEM;
	}

	if (copy_from_user(setting,
		(void *)i2c_setting->setting,
		sizeof(struct camkit_i2c_reg) * len)) {
		pr_err("failed: copy_from_user");
		kfree(setting);
		i2c_setting->setting = NULL;
		return EFAULT;
	}

	i2c_setting->setting = setting;
	for (i = 0; i < len; i++) {
		LOG_DBG("setting[%d].addr: 0x%x\n", i, setting[i].addr);
		LOG_DBG("setting[%d].data: 0x%x\n", i, setting[i].data);
		LOG_DBG("setting[%d].delay: 0x%x\n", i, setting[i].delay);
	}

	LOG_DBG("setting size: %u\n", i2c_setting->size);
	LOG_DBG("setting addr_type: %u\n", i2c_setting->addr_type);
	LOG_DBG("setting data_type: %u\n", i2c_setting->data_type);
	LOG_DBG("setting delay: %u\n", i2c_setting->delay);

	return ERROR_NONE;
}

static uint32 camkit_translate_table_array(
	struct camkit_i2c_reg_table_array *reg_table)
{
	int len;
	int i;
	struct camkit_i2c_reg_table *setting = NULL;

	if (reg_table == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	if (!reg_table->size || !reg_table->setting) {
		pr_info("[camkit][%s] reg table not configure size %d\n", __func__, reg_table->size);
		reg_table->setting = NULL;
		return ERROR_NONE;
	}

	LOG_DBG("setting size: %u\n", reg_table->size);

	len = reg_table->size;
	setting = kzalloc(sizeof(struct camkit_i2c_reg_table) * len, GFP_KERNEL);
	if (!setting) {
		pr_err("[%s] memory not enough\n", __func__);
		reg_table->setting = NULL;
		return ENOMEM;
	}

	if (copy_from_user(setting,
		(void *)reg_table->setting,
		sizeof(struct camkit_i2c_reg_table) * len)) {
		pr_err("failed: copy_from_user");
		kfree(setting);
		reg_table->setting = NULL;
		return EFAULT;
	}

	reg_table->setting = setting;
	for (i = 0; i < len; i++) {
		LOG_DBG("setting[%d].addr: 0x%x\n", i, setting[i].addr);
		LOG_DBG("setting[%d].data: 0x%x\n", i, setting[i].data);
		LOG_DBG("setting[%d] type: %u\n", i, setting[i].data_type);
		LOG_DBG("setting[%d] op: %u\n", i, setting[i].i2c_operation);
		LOG_DBG("setting[%d].delay: 0x%x\n", i, setting[i].delay);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_seamless_scenario_id_map(
	struct camkit_seamless_cfg_t *seamless_cfg)
{
	struct camkit_seamless_scenario_id_t *id_map = NULL;
	uint32 map_size;
	uint32 rc = ERROR_NONE;

	if (!seamless_cfg ||
		seamless_cfg->seamless_support != CAMKIT_SEAMLESS_SUPPORT) {
		LOG_INF("do not support seamless, no need to do translate\n");
		return ERROR_NONE;
	}

	if (!seamless_cfg->scenario_id_map ||
		seamless_cfg->scenario_id_map_size == 0) {
		LOG_ERR("faild to translate seamless scenerio id map\n");
		rc = EFAULT;
		goto clear_seamless_scenario_id_map;
	}

	map_size = seamless_cfg->scenario_id_map_size;
	id_map = kzalloc(sizeof(struct camkit_seamless_scenario_id_t) * map_size, GFP_KERNEL);
	if (!id_map) {
		LOG_ERR("alloc memory failed for seamless id map\n");
		rc = ENOMEM;
		goto clear_seamless_scenario_id_map;
	}

	if (copy_from_user(id_map,
		(void *)seamless_cfg->scenario_id_map,
		sizeof(struct camkit_seamless_scenario_id_t) * map_size)) {
		LOG_ERR("seamless map failed to copy_from_user\n");
		rc = EFAULT;
		goto free_id_map;
	}

	seamless_cfg->scenario_id_map = id_map;
	LOG_INF("success to translate seamless id map\n");
	return ERROR_NONE;

free_id_map:
	kfree(id_map);
clear_seamless_scenario_id_map:
	seamless_cfg->scenario_id_map = NULL;
	seamless_cfg->scenario_id_map_size = 0;
	return rc;
}

static uint32 camkit_translate_seamless_scenario_cfg(
	struct camkit_seamless_cfg_t *seamless_cfg)
{
	uint32 rc = ERROR_NONE;
	struct camkit_seamless_scenario_cfg_t *scenario_cfg = NULL;
	uint32 cfg_size;
	int32 idx;

	if (!seamless_cfg ||
		seamless_cfg->seamless_support != CAMKIT_SEAMLESS_SUPPORT) {
		LOG_INF("do not support seamless, no need to do translate cfg\n");
		return ERROR_NONE;
	}

	if (!seamless_cfg->scenario_cfg ||
		seamless_cfg->scenario_cfg_size == 0) {
		LOG_INF("scenario_cfg is null, may be no need to translate\n");
		rc = ERROR_NONE;
		goto clear_usr_scenario_cfg_ref;
	}

	cfg_size = seamless_cfg->scenario_cfg_size;
	scenario_cfg = kzalloc(
		sizeof(struct camkit_seamless_scenario_cfg_t) * cfg_size, GFP_KERNEL);
	if (!scenario_cfg) {
		LOG_ERR("alloc memory failed for seamless scenario cfg\n");
		rc = ENOMEM;
		goto clear_usr_scenario_cfg_ref;
	}

	if (copy_from_user(scenario_cfg,
		(void *)seamless_cfg->scenario_cfg,
		sizeof(struct camkit_seamless_scenario_cfg_t) * cfg_size)) {
		LOG_ERR("seamless scenario cfg failed to copy_from_user\n");
		rc = EFAULT;
		goto clear_cfg_resource;
	}

	seamless_cfg->scenario_cfg = scenario_cfg;
	for (idx = 0; idx < cfg_size; ++idx) {
		rc = camkit_translate_sensor_setting(&scenario_cfg[idx].seamless_setting);
		if (rc != ERROR_NONE) {
			LOG_ERR("translate sensor setting failed, idx %d\n", idx);
			goto clear_settings_resource;
		}
	}

	LOG_INF("success to translate seamless cfg\n");
	return ERROR_NONE;

clear_settings_resource:
	/* only clear the memory alloced in kernel space */
	for (; idx >= 0; idx--) {
		if (scenario_cfg[idx].seamless_setting.setting) {
			kfree(scenario_cfg[idx].seamless_setting.setting);
			scenario_cfg[idx].seamless_setting.setting = NULL;
		}
	}
clear_cfg_resource:
	kfree(scenario_cfg);
clear_usr_scenario_cfg_ref:
	seamless_cfg->scenario_cfg = NULL;
	seamless_cfg->scenario_cfg_size = 0;
	return rc;
}

static uint32 camkit_translate_stagger_scenario_id_map(
	struct camkit_stagger_cfg_t *stagger_cfg)
{
	struct camkit_stagger_scenario_id_t *id_map = NULL;
	uint32 map_size;
	uint32 rc = ERROR_NONE;

	if (!stagger_cfg ||
		stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("do not support stagger, no need to do translate\n");
		return ERROR_NONE;
	}

	if (!stagger_cfg->scenario_id_map ||
		stagger_cfg->scenario_id_map_size == 0) {
		LOG_ERR("faild to translate stagger scenerio id map\n");
		rc = EFAULT;
		goto clear_stagger_scenario_id_map;
	}

	map_size = stagger_cfg->scenario_id_map_size;
	id_map = kzalloc(sizeof(struct camkit_stagger_scenario_id_t) * map_size, GFP_KERNEL);
	if (!id_map) {
		LOG_ERR("alloc memory failed for stagger id map\n");
		rc = ENOMEM;
		goto clear_stagger_scenario_id_map;
	}

	if (copy_from_user(id_map,
		(void *)stagger_cfg->scenario_id_map,
		sizeof(struct camkit_stagger_scenario_id_t) * map_size)) {
		LOG_ERR("stagger map failed to copy_from_user\n");
		rc = EFAULT;
		goto free_id_map;
	}

	stagger_cfg->scenario_id_map = id_map;
	LOG_INF("success to translate stagger id map\n");
	return ERROR_NONE;

free_id_map:
	kfree(id_map);
clear_stagger_scenario_id_map:
	stagger_cfg->scenario_id_map = NULL;
	stagger_cfg->scenario_id_map_size = 0;
	return rc;
}

static uint32 camkit_translate_stagger_stagger_type_map(
	struct camkit_stagger_cfg_t *stagger_cfg)
{
	struct camkit_stagger_type_id_t *type_map = NULL;
	uint32 map_size;
	uint32 rc = ERROR_NONE;

	if (!stagger_cfg ||
		stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("do not support stagger, no need to do translate\n");
		return ERROR_NONE;
	}

	if (!stagger_cfg->stagger_type_map ||
		stagger_cfg->stagger_type_map_size == 0) {
		LOG_ERR("faild to translate stagger type map\n");
		rc = EFAULT;
		goto clear_stagger_type_map;
	}

	map_size = stagger_cfg->stagger_type_map_size;
	type_map = kzalloc(sizeof(struct camkit_stagger_type_id_t) * map_size, GFP_KERNEL);
	if (!type_map) {
		LOG_ERR("alloc memory failed for stagger type map\n");
		rc = ENOMEM;
		goto clear_stagger_type_map;
	}

	if (copy_from_user(type_map,
		(void *)stagger_cfg->stagger_type_map,
		sizeof(struct camkit_stagger_type_id_t) * map_size)) {
		LOG_ERR("stagger type map failed to copy_from_user\n");
		rc = EFAULT;
		goto free_type_map;
	}

	stagger_cfg->stagger_type_map = type_map;
	LOG_INF("success to translate stagger type map\n");
	return ERROR_NONE;

free_type_map:
	kfree(type_map);
clear_stagger_type_map:
	stagger_cfg->stagger_type_map = NULL;
	stagger_cfg->stagger_type_map_size = 0;
	return rc;
}

static uint32 camkit_translate_stagger_max_expo(
	struct camkit_stagger_cfg_t *stagger_cfg)
{
	struct camkit_stagger_max_expo_t *max_expo_map = NULL;
	uint32 map_size;
	uint32 rc = ERROR_NONE;

	if (!stagger_cfg ||
		stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("do not support stagger, no need to do translate\n");
		return ERROR_NONE;
	}

	if (!stagger_cfg->stagger_max_expo ||
		stagger_cfg->stagger_max_expo_size == 0) {
		LOG_ERR("faild to translate stagger max expo map\n");
		rc = EFAULT;
		goto clear_stagger_max_expo;
	}

	map_size = stagger_cfg->stagger_max_expo_size;
	max_expo_map = kzalloc(sizeof(struct camkit_stagger_max_expo_t) * map_size, GFP_KERNEL);
	if (!max_expo_map) {
		LOG_ERR("alloc memory failed for stagger max expo map\n");
		rc = ENOMEM;
		goto clear_stagger_max_expo;
	}

	if (copy_from_user(max_expo_map,
		(void *)stagger_cfg->stagger_max_expo,
		sizeof(struct camkit_stagger_max_expo_t) * map_size)) {
		LOG_ERR("stagger max expo map failed to copy_from_user\n");
		rc = EFAULT;
		goto free_max_expo_map;
	}

	stagger_cfg->stagger_max_expo = max_expo_map;
	LOG_INF("success to translate stagger max expo map\n");
	return ERROR_NONE;

free_max_expo_map:
	kfree(max_expo_map);
clear_stagger_max_expo:
	stagger_cfg->stagger_max_expo = NULL;
	stagger_cfg->stagger_max_expo_size = 0;
	return rc;
}

static uint32 camkit_translate_sensor_info(
	struct camkit_sensor_info_t *sensor_info)
{
	uint32 ret = ERROR_NONE;
	int i;

	LOG_DBG("sensor_id_reg: %u\n", sensor_info->sensor_id_reg);
	LOG_DBG("sensor_id: %u\n", sensor_info->sensor_id);
	LOG_DBG("checksum_value: %u\n", sensor_info->checksum_value);

	camkit_print_mode_info(&sensor_info->pre);
	camkit_print_mode_info(&sensor_info->cap);
	camkit_print_mode_info(&sensor_info->cap1);
	camkit_print_mode_info(&sensor_info->normal_video);
	camkit_print_mode_info(&sensor_info->hs_video);
	camkit_print_mode_info(&sensor_info->slim_video);
	camkit_print_mode_info(&sensor_info->custom1);
	camkit_print_mode_info(&sensor_info->custom2);
	camkit_print_mode_info(&sensor_info->custom3);
	camkit_print_mode_info(&sensor_info->custom4);
	camkit_print_mode_info(&sensor_info->custom5);

	ret |= camkit_translate_sensor_setting(&sensor_info->id_init_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->init_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->init_burst_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->pre_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->cap_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->cap1_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_video_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->hs_video_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->slim_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom1_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom2_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom3_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom4_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->custom5_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->streamon_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->streamoff_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->test_pattern_on_setting);
	ret |= camkit_translate_sensor_setting(&sensor_info->test_pattern_off_setting);
	ret |= camkit_translate_table_array(&sensor_info->dump_info);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_to_long_ready_settings);
	ret |= camkit_translate_sensor_setting(&sensor_info->normal_to_long_end_settings);
	ret |= camkit_translate_table_array(&sensor_info->fuse_id_info);
	ret |= camkit_translate_table_array(&sensor_info->streamoff_check);
	ret |= camkit_translate_seamless_scenario_id_map(&sensor_info->seamless_cfg);
	ret |= camkit_translate_seamless_scenario_cfg(&sensor_info->seamless_cfg);
	ret |= camkit_translate_stagger_scenario_id_map(&sensor_info->stagger_cfg);
	ret |= camkit_translate_stagger_stagger_type_map(&sensor_info->stagger_cfg);
	ret |= camkit_translate_stagger_max_expo(&sensor_info->stagger_cfg);

	LOG_DBG("ae_shut_delay_frame: %u\n", sensor_info->ae_shut_delay_frame);
	LOG_DBG("ae_sensor_gain_delay_frame: %u\n",
		sensor_info->ae_sensor_gain_delay_frame);
	LOG_DBG("ae_ispGain_delay_frame: %u\n", sensor_info->ae_ispGain_delay_frame);
	LOG_DBG("ihdr_support: %u\n", sensor_info->ihdr_support);
	LOG_DBG("ihdr_le_firstline: %u\n", sensor_info->ihdr_le_firstline);
	LOG_DBG("sensor_mode_num: %u\n", sensor_info->sensor_mode_num);
	LOG_DBG("cap_delay_frame: %u\n", sensor_info->cap_delay_frame);
	LOG_DBG("pre_delay_frame: %u\n", sensor_info->pre_delay_frame);
	LOG_DBG("video_delay_frame: %u\n", sensor_info->video_delay_frame);
	LOG_DBG("hs_video_delay_frame: %u\n", sensor_info->hs_video_delay_frame);
	LOG_DBG("slim_video_delay_frame: %u\n", sensor_info->slim_video_delay_frame);
	LOG_DBG("custom1_delay_frame: %u\n", sensor_info->custom1_delay_frame);
	LOG_DBG("custom2_delay_frame: %u\n", sensor_info->custom2_delay_frame);
	LOG_DBG("custom3_delay_frame: %u\n", sensor_info->custom3_delay_frame);
	LOG_DBG("custom4_delay_frame: %u\n", sensor_info->custom4_delay_frame);
	LOG_DBG("custom5_delay_frame: %u\n", sensor_info->custom5_delay_frame);
	LOG_DBG("isp_driving_current: %u\n", sensor_info->isp_driving_current);
	LOG_DBG("sensor_interface_type: %u\n", sensor_info->sensor_interface_type);
	LOG_DBG("mipi_sensor_type: %u\n", sensor_info->mipi_sensor_type);
	LOG_DBG("mipi_settle_delay_mode: %u\n", sensor_info->mipi_settle_delay_mode);
	LOG_DBG("sensor_output_dataformat: %u\n", sensor_info->sensor_output_dataformat);
	LOG_DBG("mclk: %u\n", sensor_info->mclk);
	LOG_DBG("mipi_lane_num: %u\n", sensor_info->mipi_lane_num);
	LOG_DBG("i2c_addr_table[0]: %u\n", sensor_info->i2c_addr_table[0]);
	LOG_DBG("i2c_addr_table[1]: %u\n", sensor_info->i2c_addr_table[1]);
	LOG_DBG("i2c_addr_table[2]: %u\n", sensor_info->i2c_addr_table[2]);
	LOG_DBG("i2c_addr_table[3]: %u\n", sensor_info->i2c_addr_table[3]);
	LOG_DBG("i2c_addr_table[4]: %u\n", sensor_info->i2c_addr_table[4]);
	LOG_DBG("i2c_speed: %u\n", sensor_info->i2c_speed);
	LOG_DBG("addr_type: %u\n", sensor_info->addr_type);
	LOG_DBG("pdaf_support: %u\n", sensor_info->pdaf_support);
	LOG_DBG("pdaf_support_by_scenario: %u\n", sensor_info->pdaf_support_by_scenario);
	LOG_DBG("need_high_impedance: %u\n", sensor_info->need_high_impedance);
	LOG_DBG("probe_flag: %u\n", sensor_info->probe_flag);

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("binning ratio[%d]: %u\n", i, sensor_info->binning_ratio[i]);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_ctrl(
	struct camkit_sensor_ctrl_t *sensor_ctrl)
{
	if (sensor_ctrl == NULL) {
		pr_err("[camkit][%s] invalid input arg\n", __func__);
		return EFAULT;
	}

	// needn't translate, only dump info now
	LOG_DBG("mirror: %u\n", sensor_ctrl->mirror);
	LOG_DBG("sensor_mode: %u\n", sensor_ctrl->sensor_mode);
	LOG_DBG("shutter: %u\n", sensor_ctrl->shutter);
	LOG_DBG("gain: %u\n", sensor_ctrl->gain);
	LOG_DBG("pclk: %u\n", sensor_ctrl->pclk);
	LOG_DBG("frame_length: %u\n", sensor_ctrl->frame_length);
	LOG_DBG("line_length: %u\n", sensor_ctrl->line_length);
	LOG_DBG("min_frame_length: %u\n", sensor_ctrl->min_frame_length);
	LOG_DBG("dummy_pixel: %u\n", sensor_ctrl->dummy_pixel);
	LOG_DBG("dummy_line: %u\n", sensor_ctrl->dummy_line);
	LOG_DBG("current_fps: %u\n", sensor_ctrl->current_fps);
	LOG_DBG("autoflicker_en: %u\n", sensor_ctrl->autoflicker_en);
	LOG_DBG("test_pattern: %u\n", sensor_ctrl->test_pattern);
	LOG_DBG("current_scenario_id: %d\n", sensor_ctrl->current_scenario_id);
	LOG_DBG("ihdr_en: %u\n", sensor_ctrl->ihdr_en);
	LOG_DBG("i2c_write_id: %u\n", sensor_ctrl->i2c_write_id);
	LOG_DBG("i2c_speed: %u\n", sensor_ctrl->i2c_speed);
	LOG_DBG("addr_type: %d\n", sensor_ctrl->addr_type);

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_output_info(
	struct camkit_sensor_output_info_t *output_info)
{
	int i;

	if (output_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("output_info[%d].full_w: %u\n", i, output_info[i].full_w);
		LOG_DBG("output_info[%d].full_h: %u\n", i, output_info[i].full_h);
		LOG_DBG("output_info[%d].x0_offset: %u\n", i, output_info[i].x0_offset);
		LOG_DBG("output_info[%d].y0_offset: %u\n", i, output_info[i].y0_offset);
		LOG_DBG("output_info[%d].w0_size: %u\n", i, output_info[i].w0_size);
		LOG_DBG("output_info[%d].h0_size: %u\n", i, output_info[i].h0_size);
		LOG_DBG("output_info[%d].scale_w: %u\n", i, output_info[i].scale_w);
		LOG_DBG("output_info[%d].scale_h: %u\n", i, output_info[i].scale_h);
		LOG_DBG("output_info[%d].x1_offset: %u\n", i, output_info[i].x1_offset);
		LOG_DBG("output_info[%d].y1_offset: %u\n", i, output_info[i].y1_offset);
		LOG_DBG("output_info[%d].w1_size: %u\n", i, output_info[i].w1_size);
		LOG_DBG("output_info[%d].h1_size: %u\n", i, output_info[i].h1_size);
		LOG_DBG("output_info[%d].x2_tg_offset: %u\n", i, output_info[i].x2_tg_offset);
		LOG_DBG("output_info[%d].y2_tg_offset: %u\n", i, output_info[i].y2_tg_offset);
		LOG_DBG("output_info[%d].w2_tg_size: %u\n", i, output_info[i].w2_tg_size);
		LOG_DBG("output_info[%d].h2_tg_size: %u\n", i, output_info[i].h2_tg_size);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_pdaf_info(
	struct camkit_sensor_pdaf_info_t *pdaf_info)
{
	int i;

	if (pdaf_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("pdaf_info.i4OffsetX: %u\n", pdaf_info->i4OffsetX);
	LOG_DBG("pdaf_info.i4OffsetY: %u\n", pdaf_info->i4OffsetY);
	LOG_DBG("pdaf_info.i4PitchX: %u\n", pdaf_info->i4PitchX);
	LOG_DBG("pdaf_info.i4PitchY: %u\n", pdaf_info->i4PitchY);
	LOG_DBG("pdaf_info.i4PairNum: %u\n", pdaf_info->i4PairNum);
	LOG_DBG("pdaf_info.i4SubBlkW: %u\n", pdaf_info->i4SubBlkW);
	LOG_DBG("pdaf_info.i4SubBlkH: %u\n", pdaf_info->i4SubBlkH);

	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4PosL); i++) {
		LOG_DBG("pdaf_info.i4PosL[%d][0]: %u\n", i, pdaf_info->i4PosL[i][0]);
		LOG_DBG("pdaf_info.i4PosL[%d][1]: %u\n", i, pdaf_info->i4PosL[i][1]);
	}
	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4PosR); i++) {
		LOG_DBG("pdaf_info.i4PosR[%d][0]: %u\n", i, pdaf_info->i4PosR[i][0]);
		LOG_DBG("pdaf_info.i4PosR[%d][1]: %u\n", i, pdaf_info->i4PosR[i][1]);
	}

	LOG_DBG("pdaf_info.iMirrorFlip: %u\n", pdaf_info->iMirrorFlip);
	LOG_DBG("pdaf_info.i4BlockNumX: %u\n", pdaf_info->i4BlockNumX);
	LOG_DBG("pdaf_info.i4BlockNumY: %u\n", pdaf_info->i4BlockNumY);
	LOG_DBG("pdaf_info.i4LeFirst: %u\n", pdaf_info->i4LeFirst);

	for (i = 0; i < ARRAY_SIZE(pdaf_info->i4Crop); i++) {
		LOG_DBG("pdaf_info.i4Crop[%d][0]: %u\n", i, pdaf_info->i4Crop[i][0]);
		LOG_DBG("pdaf_info.i4Crop[%d][1]: %u\n", i, pdaf_info->i4Crop[i][1]);
	}

	return ERROR_NONE;

}

static uint32 camkit_translate_pdaf_info_scenario (
	struct camkit_sensor_pdaf_info_t *pdaf_info)
{
	uint32 i;
	uint32 j;

	if (pdaf_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("pdaf[%d].i4OffsetX: %u\n", i, pdaf_info[i].i4OffsetX);
		LOG_DBG("pdaf[%d].i4OffsetY: %u\n", i, pdaf_info[i].i4OffsetY);
		LOG_DBG("pdaf[%d].i4PitchX: %u\n", i, pdaf_info[i].i4PitchX);
		LOG_DBG("pdaf[%d].i4PitchY: %u\n", i, pdaf_info[i].i4PitchY);
		LOG_DBG("pdaf[%d].i4PairNum: %u\n", i, pdaf_info[i].i4PairNum);
		LOG_DBG("pdaf[%d].i4SubBlkW: %u\n", i, pdaf_info[i].i4SubBlkW);
		LOG_DBG("pdaf[%d].i4SubBlkH: %u\n", i, pdaf_info[i].i4SubBlkH);
		LOG_DBG("pdaf[%d].iMirrorFlip: %u\n", i, pdaf_info[i].iMirrorFlip);
		LOG_DBG("pdaf[%d].i4BlockNumX: %u\n", i, pdaf_info[i].i4BlockNumX);
		LOG_DBG("pdaf[%d].i4BlockNumY: %u\n", i, pdaf_info[i].i4BlockNumY);
		LOG_DBG("pdaf[%d].i4LeFirst: %u\n", i, pdaf_info[i].i4LeFirst);
		for (j = 0; j < ARRAY_SIZE(pdaf_info[i].i4PosL); j++) {
			LOG_DBG("pdaf_info.i4PosL[%d][0]: %u\n", j, pdaf_info[i].i4PosL[j][0]);
			LOG_DBG("pdaf_info.i4PosL[%d][1]: %u\n", j, pdaf_info[i].i4PosL[j][1]);
		}
		for (j = 0; j < ARRAY_SIZE(pdaf_info[i].i4PosR); j++) {
			LOG_DBG("pdaf_info.i4PosR[%d][0]: %u\n", j, pdaf_info[i].i4PosR[j][0]);
			LOG_DBG("pdaf_info.i4PosR[%d][1]: %u\n", j, pdaf_info[i].i4PosR[j][1]);
		}
		for (j = 0; j < ARRAY_SIZE(pdaf_info[i].i4Crop); j++) {
			LOG_DBG("pdaf_info.i4Crop[%d][0]: %u\n", j, pdaf_info[i].i4Crop[j][0]);
			LOG_DBG("pdaf_info.i4Crop[%d][1]: %u\n", j, pdaf_info[i].i4Crop[j][1]);
		}
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_i2c_idx(int32 *i2c_idx)
{
	uint32 i;
	int32 i2c_index;
	const int32 i2c_dev_id_map[][2] = {
		{ (int32)CAMKIT_I2C_DEV_0, (int32)IMGSENSOR_I2C_DEV_0 },
		{ (int32)CAMKIT_I2C_DEV_1, (int32)IMGSENSOR_I2C_DEV_1 },
		{ (int32)CAMKIT_I2C_DEV_2, (int32)IMGSENSOR_I2C_DEV_2 },
		{ (int32)CAMKIT_I2C_DEV_3, (int32)IMGSENSOR_I2C_DEV_3 },
		{ (int32)CAMKIT_I2C_DEV_4, (int32)IMGSENSOR_I2C_DEV_4 },
		{ (int32)CAMKIT_I2C_DEV_5, (int32)IMGSENSOR_I2C_DEV_5 },
		{ (int32)CAMKIT_I2C_DEV_6, (int32)IMGSENSOR_I2C_DEV_6 },
		{ (int32)CAMKIT_I2C_DEV_7, (int32)IMGSENSOR_I2C_DEV_7 },
	};

	if (!i2c_idx) {
		LOG_ERR("i2c_idx is null\n");
		return EFAULT;
	}

	for (i = 0; i < ARRAY_SIZE(i2c_dev_id_map); i++) {
		i2c_index = *i2c_idx;
		if (i2c_index == i2c_dev_id_map[i][0]) {
			*i2c_idx = i2c_dev_id_map[i][1];
			LOG_DBG("camkit translate i2c idx success, i2c idx from %d to %d\n",
				i2c_index, *i2c_idx);
			return ERROR_NONE;
		}
	}

	LOG_ERR("fail to translate i2c idx, invalid idx %d\n", *i2c_idx);
	return EINVAL;
}

static enum IMGSENSOR_SENSOR_IDX camkit_translate_sensor_index(
	uint32 kit_sensor_id)
{
	uint32 i;
	const sensor_idx_map_t sensor_idx_map[] = {
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN, IMGSENSOR_SENSOR_IDX_MAIN },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB, IMGSENSOR_SENSOR_IDX_SUB },
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN2, IMGSENSOR_SENSOR_IDX_MAIN2 },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB2, IMGSENSOR_SENSOR_IDX_SUB2 },
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN3, IMGSENSOR_SENSOR_IDX_MAIN3 },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB3, IMGSENSOR_SENSOR_IDX_SUB3 },
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN4, IMGSENSOR_SENSOR_IDX_MAIN4 },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB4, IMGSENSOR_SENSOR_IDX_SUB4 },
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN5, IMGSENSOR_SENSOR_IDX_MAIN5 },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB5, IMGSENSOR_SENSOR_IDX_SUB5 },
		{ (uint32)CAMKIT_SENSOR_IDX_MAIN6, IMGSENSOR_SENSOR_IDX_MAIN6 },
		{ (uint32)CAMKIT_SENSOR_IDX_SUB6, IMGSENSOR_SENSOR_IDX_SUB6 },
	};

	for (i = 0; i < ARRAY_SIZE(sensor_idx_map); i++) {
		if (kit_sensor_id == sensor_idx_map[i].camkit_sensor_idx) {
			LOG_DBG("success to tranlate sensor from %u to %d",
				kit_sensor_id,
				sensor_idx_map[i].platform_sensor_idx);
			return sensor_idx_map[i].platform_sensor_idx;
		}
	}

	LOG_ERR("tranlate sensor index failed, do default translate %u to %d\n",
		kit_sensor_id, (enum IMGSENSOR_SENSOR_IDX)kit_sensor_id);

	return (enum IMGSENSOR_SENSOR_IDX)kit_sensor_id;
}

static uint32 camkit_translate_pin_type(int32 *pin_type)
{
	int pin;

	if (pin_type == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	pin = *pin_type;
	switch (pin) {
	case CAMKIT_HW_PIN_NONE:
		*pin_type = (int32)IMGSENSOR_HW_PIN_NONE;
		break;
	case CAMKIT_HW_PIN_PDN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_PDN;
		break;
	case CAMKIT_HW_PIN_RST:
		*pin_type = (int32)IMGSENSOR_HW_PIN_RST;
		break;
	case CAMKIT_HW_PIN_AVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD_EN;
		break;
	case CAMKIT_HW_PIN_AVDD_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD_SEL;
		break;
	case CAMKIT_HW_PIN_DVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD_EN;
		break;
	case CAMKIT_HW_PIN_DVDD_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD_SEL;
		break;
	case CAMKIT_HW_PIN_IOVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_IOVDD_EN;
		break;
	case CAMKIT_HW_PIN_AVDD1_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD1_EN;
		break;
	case CAMKIT_HW_PIN_AFVDD_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AFVDD_EN;
		break;
	case CAMKIT_HW_PIN_AVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD;
		break;
	case CAMKIT_HW_PIN_AVDD1:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AVDD1;
		break;
	case CAMKIT_HW_PIN_DVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DVDD;
		break;
	case CAMKIT_HW_PIN_DOVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_DOVDD;
		break;
	case CAMKIT_HW_PIN_VMCH:
		*pin_type = (int32)IMGSENSOR_HW_PIN_VMCH;
		break;
	case CAMKIT_HW_PIN_AFVDD:
		*pin_type = (int32)IMGSENSOR_HW_PIN_AFVDD;
		break;
	case CAMKIT_HW_PIN_LDO1_PMIC:
		*pin_type = (int32)IMGSENSOR_HW_PIN_LDO1_PMIC;
		break;
	case CAMKIT_HW_PIN_LDO2_PMIC:
		*pin_type = (int32)IMGSENSOR_HW_PIN_LDO2_PMIC;
		break;
	case CAMKIT_HW_PIN_LDO3_PMIC:
		*pin_type = (int32)IMGSENSOR_HW_PIN_LDO3_PMIC;
		break;
	case CAMKIT_HW_PIN_LDO4_PMIC:
		*pin_type = (int32)IMGSENSOR_HW_PIN_LDO4_PMIC;
		break;
	case CAMKIT_HW_PIN_XBUCK1_PMIC:
		*pin_type = (int32)IMGSENSOR_HW_PIN_XBUCK1_PMIC;
		break;
#ifdef MIPI_SWITCH
	case CAMKIT_HW_PIN_MIPI_SWITCH_EN:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MIPI_SWITCH_EN;
		break;
	case CAMKIT_HW_PIN_MIPI_SWITCH_SEL:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL;
		break;
#endif
	case CAMKIT_HW_PIN_MCLK:
		*pin_type = (int32)IMGSENSOR_HW_PIN_MCLK;
		break;
	case CAMKIT_HW_PIN_RST1:
		*pin_type = (int32)IMGSENSOR_HW_PIN_RST1;
		break;
	case CAMKIT_HW_PIN_UNDEF:
		*pin_type = (int32)IMGSENSOR_HW_PIN_UNDEF;
		break;
	case CAMKIT_HW_PIN_5V_BOOST:
		*pin_type = (int32)IMGSENSOR_HW_PIN_5V_BOOST;
		break;
	default:
		*pin_type = (int32)IMGSENSOR_HW_PIN_UNDEF;
		break;
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_pin_value(int32 *pin_val)
{
	int pin_value;

	if (pin_val == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	pin_value = *pin_val;
	switch (pin_value) {
	case CAMKIT_HW_PIN_VALUE_NONE:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	case CAMKIT_HW_PIN_VALUE_LOW:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_0;
		break;
	case CAMKIT_HW_PIN_VALUE_HIGH:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH;
		break;
	case CAMKIT_HW_PIN_VALUE_1000:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1000;
		break;
	case CAMKIT_HW_PIN_VALUE_1050:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1050;
		break;
	case CAMKIT_HW_PIN_VALUE_1100:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1100;
		break;
	case CAMKIT_HW_PIN_VALUE_1200:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1200;
		break;
	case CAMKIT_HW_PIN_VALUE_1210:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1210;
		break;
	case CAMKIT_HW_PIN_VALUE_1220:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1220;
		break;
	case CAMKIT_HW_PIN_VALUE_1250:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1250;
		break;
	case CAMKIT_HW_PIN_VALUE_1500:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1500;
		break;
	case CAMKIT_HW_PIN_VALUE_1800:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_1800;
		break;
	case CAMKIT_HW_PIN_VALUE_2500:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2500;
		break;
	case CAMKIT_HW_PIN_VALUE_2800:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2800;
		break;
	case CAMKIT_HW_PIN_VALUE_2850:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2850;
		break;
	case CAMKIT_HW_PIN_VALUE_2900:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_2900;
		break;
	case CAMKIT_HW_PIN_VALUE_3000:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_3000;
		break;
	case CAMKIT_HW_PIN_VALUE_3300:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_3300;
		break;
	case CAMKIT_HW_PIN_VALUE_5000:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_LEVEL_5000;
		break;
	default:
		*pin_val = (int32)IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_power_info(
	struct camkit_hw_power_info_t *power_info)
{
	uint32 ret = ERROR_NONE;

	if (power_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("power_info.pin_type: %d\n", power_info->pin_type);
	LOG_DBG("power_info.pin_val: %d\n", power_info->pin_val);
	LOG_DBG("power_info.delay: %u\n", power_info->pin_delay);
	ret |= camkit_translate_pin_type(&(power_info->pin_type));
	ret |= camkit_translate_pin_value(&(power_info->pin_val));
	LOG_DBG("translated pin_type: %d\n", power_info->pin_type);
	LOG_DBG("translated pin_val: %d\n", power_info->pin_val);
	LOG_DBG("translated delay: %u\n", power_info->pin_delay);

	return ret;
}

static uint32 camkit_translate_sensor_priv_gain_info(
	struct private_again_info *again_info)
{
	int32 len;
	int32 i;
	//kal_int32 j;
	struct priv_again_map *again_map = NULL;

	if (again_info == NULL) {
		LOG_ERR("invalid input arg");
		return EFAULT;
	}

	LOG_DBG("sensor again info size: %u gain map size: %u",
		again_info->size, (uint32)(sizeof(struct priv_again_map)));

	len = again_info->size;
	if (len <= 0) {
		LOG_ERR("the gain map not configure");
		again_info->again_map = NULL;
		return ERROR_NONE;
	}

	again_map = kzalloc(sizeof(struct priv_again_map) * len, GFP_KERNEL);
	if (!again_map) {
		LOG_ERR("memory not enough");
		again_info->again_map = NULL;
		return ENOMEM;
	}

	if (copy_from_user(again_map,
		(void *)again_info->again_map,
		sizeof(struct priv_again_map) * len)) {
		pr_err("failed: copy_from_user");
		kfree(again_map);
		again_info->again_map = NULL;
		return EFAULT;
	}

	again_info->again_map = again_map;
	for (i = 0; i < len; i++) {
		LOG_DBG("again_map[%d].again: %u", i, again_map[i].again_val);
		LOG_DBG("again_map[%d].size: %u", i, again_map[i].size);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_sensor_aec_map(
	struct aec_ops_map *aec_map)
{
	int32 len;
	int32 i;
	int32 j;
	struct camkit_aec_op *aec_ops = NULL;

	if (aec_map == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("sensor aec map size: %u\n", aec_map->size);

	len = aec_map->size;
	if (len <= 0) {
		pr_err("the aec map need not configure\n");
		aec_map->aec_ops = NULL;
		return ERROR_NONE;
	}
	aec_ops = kzalloc(sizeof(struct camkit_aec_op) * len, GFP_KERNEL);
	if (!aec_ops) {
		pr_err("[%s] memory not enough\n", __func__);
		aec_map->aec_ops = NULL;
		return ENOMEM;
	}

	if (copy_from_user(aec_ops,
		(void *)aec_map->aec_ops,
		sizeof(struct camkit_aec_op) * len)) {
		pr_err("failed: copy_from_user");
		kfree(aec_ops);
		aec_map->aec_ops = NULL;
		return EFAULT;
	}

	aec_map->aec_ops = aec_ops;
	for (i = 0; i < len; i++) {
		LOG_DBG("aec_ops[%d].op_type: %d\n", i, aec_ops[i].op_type);
		LOG_DBG("aec_ops[%d].size: %d\n", i, aec_ops[i].size);
		if (aec_ops[i].size > 0 && aec_ops[i].size < MAX_AEC_REGS) {
			for (j = 0; j < aec_ops[i].size; j++) {
				LOG_DBG("aec_ops[%d].i2c_setting[%d].addr: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].addr);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].val: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].val);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].len: %d\n",
					i, j, aec_ops[i].i2c_setting[j].len);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].mask: 0x%x\n",
					i, j, aec_ops[i].i2c_setting[j].mask);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].mode: %d\n",
					i, j, aec_ops[i].i2c_setting[j].mode);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].addr_type: %d\n",
					i, j, aec_ops[i].i2c_setting[j].addr_type);
				LOG_DBG("aec_ops[%d].i2c_setting[%d].data_type: %d\n",
					i, j, aec_ops[i].i2c_setting[j].data_type);

				LOG_DBG("aec_ops[%d].aec_setting[%d].addr: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].addr);
				LOG_DBG("aec_ops[%d].aec_setting[%d].max_val: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].max_val);
				LOG_DBG("aec_ops[%d].aec_setting[%d].shift: %d\n",
					i, j, aec_ops[i].aec_setting[j].shift);
				LOG_DBG("aec_ops[%d].aec_setting[%d].mask: 0x%x\n",
					i, j, aec_ops[i].aec_setting[j].mask);
				LOG_DBG("aec_ops[%d].aec_setting[%d].addr_type: %d\n",
					i, j, aec_ops[i].aec_setting[j].addr_type);
				LOG_DBG("aec_ops[%d].aec_setting[%d].data_type: %d\n",
					i, j, aec_ops[i].aec_setting[j].data_type);
			}
		}
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_aec_info(
	struct camkit_aec_info_t *aec_info)
{
	uint32 ret = ERROR_NONE;

	if (aec_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	LOG_DBG("aec_info.min_again: %u\n", aec_info->min_again);
	LOG_DBG("aec_info.max_again: %u\n", aec_info->max_again);
	LOG_DBG("aec_info.min_dgain: %u\n", aec_info->min_dgain);
	LOG_DBG("aec_info.max_dgain: %u\n", aec_info->max_dgain);
	LOG_DBG("aec_info.reg_gain_1x: %u\n", aec_info->reg_gain_1x);
	LOG_DBG("aec_info.dgain_decimator: %u\n", aec_info->dgain_decimator);
	LOG_DBG("aec_info.min_linecount: %u\n", aec_info->min_linecount);
	LOG_DBG("aec_info.max_linecount: %u\n", aec_info->max_linecount);
	LOG_DBG("aec_info.vts_offset: %u\n", aec_info->vts_offset);
	LOG_DBG("aec_info.again_type: %u\n", aec_info->again_type);
	LOG_DBG("aec_info.smia_coeff.m0: %u\n", aec_info->smia_coeff.m0);
	LOG_DBG("aec_info.smia_coeff.m1: %u\n", aec_info->smia_coeff.m1);
	LOG_DBG("aec_info.smia_coeff.m2: %u\n", aec_info->smia_coeff.m2);
	LOG_DBG("aec_info.smia_coeff.c0: %u\n", aec_info->smia_coeff.c0);
	LOG_DBG("aec_info.smia_coeff.c1: %u\n", aec_info->smia_coeff.c1);
	LOG_DBG("aec_info.smia_coeff.c2: %u\n", aec_info->smia_coeff.c2);

	ret |= camkit_translate_sensor_priv_gain_info(&aec_info->priv_again_info);
	ret |= camkit_translate_sensor_aec_map(&aec_info->gain_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->expo_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->vts_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->normal2long_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->long2normal_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->awb_gain_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->hdr_m_gain_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->hdr_m_expo_ops_map);
	return ret;
}

static uint32 camkit_translate_vc_info(
	struct camkit_sensor_vc_info_t *vc_info)
{
	uint32 i;

	if (vc_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		LOG_DBG("vc[%d].vc_num: %u\n", i, vc_info[i].vc_num);
		LOG_DBG("vc[%d].vc_pixel_num: %u\n", i, vc_info[i].vc_pixel_num);
		LOG_DBG("vc[%d].mode_select: %u\n", i, vc_info[i].mode_select);
		LOG_DBG("vc[%d].expo_ratio: %u\n", i, vc_info[i].expo_ratio);
		LOG_DBG("vc[%d].od_value: %u\n", i, vc_info[i].od_value);
		LOG_DBG("vc[%d].rg_stats_mode: %u\n", i, vc_info[i].rg_stats_mode);

		LOG_DBG("vc[%d].vc0_id: %u\n", i, vc_info[i].vc0_id);
		LOG_DBG("vc[%d].vc0_data_type: %u\n", i, vc_info[i].vc0_data_type);
		LOG_DBG("vc[%d].vc0_sizeh: %u\n", i, vc_info[i].vc0_sizeh);
		LOG_DBG("vc[%d].vc0_sizev: %u\n", i, vc_info[i].vc0_sizev);
		LOG_DBG("vc[%d].vc1_id: %u\n", i, vc_info[i].vc1_id);
		LOG_DBG("vc[%d].vc1_data_type: %u\n", i, vc_info[i].vc1_data_type);
		LOG_DBG("vc[%d].vc1_sizeh: %u\n", i, vc_info[i].vc1_sizeh);
		LOG_DBG("vc[%d].vc1_sizev: %u\n", i, vc_info[i].vc1_sizev);
		LOG_DBG("vc[%d].vc2_id: %u\n", i, vc_info[i].vc2_id);
		LOG_DBG("vc[%d].vc2_data_type: %u\n", i, vc_info[i].vc2_data_type);
		LOG_DBG("vc[%d].vc2_sizeh: %u\n", i, vc_info[i].vc2_sizeh);
		LOG_DBG("vc[%d].vc2_sizev: %u\n", i, vc_info[i].vc2_sizev);
		LOG_DBG("vc[%d].vc3_id: %u\n", i, vc_info[i].vc3_id);
		LOG_DBG("vc[%d].vc3_data_type: %u\n", i, vc_info[i].vc3_data_type);
		LOG_DBG("vc[%d].vc3_sizeh: %u\n", i, vc_info[i].vc3_sizeh);
		LOG_DBG("vc[%d].vc3_sizev: %u\n", i, vc_info[i].vc3_sizev);
		LOG_DBG("vc[%d].vc4_id: %u\n", i, vc_info[i].vc4_id);
		LOG_DBG("vc[%d].vc4_data_type: %u\n", i, vc_info[i].vc4_data_type);
		LOG_DBG("vc[%d].vc4_sizeh: %u\n", i, vc_info[i].vc4_sizeh);
		LOG_DBG("vc[%d].vc4_sizev: %u\n", i, vc_info[i].vc4_sizev);
		LOG_DBG("vc[%d].vc5_id: %u\n", i, vc_info[i].vc5_id);
		LOG_DBG("vc[%d].vc5_data_type: %u\n", i, vc_info[i].vc5_data_type);
		LOG_DBG("vc[%d].vc5_sizeh: %u\n", i, vc_info[i].vc5_sizeh);
		LOG_DBG("vc[%d].vc5_sizev: %u\n", i, vc_info[i].vc5_sizev);
	}

	return ERROR_NONE;
}

static uint32 camkit_translate_correction_info(
	struct camkit_sensor_correction_t *correction_info)
{
	uint32 ret = ERROR_NONE;

	if (correction_info == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	ret |= camkit_translate_sensor_setting(&correction_info->pdc_config.before_pdc_setting);
	ret |= camkit_translate_sensor_setting(&correction_info->pdc_config.after_pdc_setting);

	LOG_DBG("qsc_apply: %u\n", correction_info->qsc_apply);
	LOG_DBG("eeprom_qsc_addr: %u\n", correction_info->eeprom_qsc_addr);
	LOG_DBG("sensor_qsc_addr: %u\n", correction_info->sensor_qsc_addr);
	LOG_DBG("qsc_len: %u\n", correction_info->qsc_len);
	LOG_DBG("spc_apply: %u\n", correction_info->spc_apply);
	LOG_DBG("eeprom_pdaf_addr: %u\n", correction_info->eeprom_pdaf_addr);
	LOG_DBG("pdaf_len: %u\n", correction_info->pdaf_len);
	LOG_DBG("lsc_start_addr: %u\n", correction_info->lsc_start_addr);
	LOG_DBG("lsc_addr_len: %u\n", correction_info->lsc_addr_len);
	LOG_DBG("rsc_start_addr: %u\n", correction_info->rsc_start_addr);
	LOG_DBG("rsc_addr_len: %u\n", correction_info->rsc_addr_len);
	LOG_DBG("pdc need config: %u\n", correction_info->pdc_config.need_config);

	return ret;
}

static uint32 camkit_translate_high_impedance_info(
	struct camkit_high_impedance_info_t *high_impedance_info)
{
	uint32 ret = ERROR_NONE;
	uint32 i;
	uint32 j;
	uint8 total_num;
	struct camkit_high_impedance_sensor_t *sensor = NULL;

	if (!high_impedance_info) {
		LOG_DBG("high_impedance_info is null\n");
		return ERROR_NONE;
	}

	total_num = high_impedance_info->total_num;
	if (total_num == 0 || total_num > MAX_HIGH_IMPEDANCE_SENSOR_NUM) {
		LOG_DBG("invalid total_num: %u\n", total_num);
		return ERROR_NONE;
	}

	sensor = &high_impedance_info->sensor[0];
	for (i = 0; i < total_num; i++) {
		ret |= camkit_translate_i2c_idx(&sensor[i].i2c_index);
		ret |= camkit_translate_sensor_setting(&sensor[i].high_impedance_setting);
		for (j = 0; j < ARRAY_SIZE(sensor[i].power_info); j++)
			ret |= camkit_translate_power_info(&sensor[i].power_info[j]);

		for (j = 0; j < ARRAY_SIZE(sensor[i].power_down_info); j++)
			ret |= camkit_translate_power_info(&sensor[i].power_down_info[j]);
	}

	for (i = 0; i < total_num; i++) {
		LOG_DBG("high impedance info dump for sensor %u:\n", i);
		LOG_DBG("sensor_index: %d, sensor_name: %s\n",
			sensor[i].sensor_index, sensor[i].sensor_name);
		LOG_DBG("i2c_index: %d, i2c_speed: %u\n",
			sensor[i].i2c_index, sensor[i].i2c_speed);
		LOG_DBG("i2c_addr_table[0]: %u, i2c_addr_table[1]: %u\n",
			sensor[i].i2c_addr_table[0], sensor[i].i2c_addr_table[1]);
	}

	return ret;
}

static uint32 camkit_translate_ext_power_info(struct camkit_extend_hw_power_info_t *ext_power_info)
{
	uint32 ret = ERROR_NONE;
	uint32 i;
	uint32 j;
	uint8 total_num;
	struct camkit_hw_power_info_pair_t *info_pair = NULL;

	if (!ext_power_info) {
		LOG_INF("ext_power_info is null\n");
		return ERROR_NONE;
	}

	total_num = ext_power_info->total_num;
	if (total_num == 0 || total_num > MAX_EXT_POWER_INFO_NUM) {
		LOG_DBG("invalid total_num for ext power info: %u\n", total_num);
		return ERROR_NONE;
	}

	info_pair = &ext_power_info->info_pair[0];
	for (i = 0; i < total_num; i++) {
		for (j = 0; j < ARRAY_SIZE(info_pair[i].power_info); j++)
			ret |= camkit_translate_power_info(&info_pair[i].power_info[j]);

		for (j = 0; j < ARRAY_SIZE(info_pair[i].power_down_info); j++)
			ret |= camkit_translate_power_info(&info_pair[i].power_down_info[j]);
	}

	LOG_INF("translate ext power info done, total num %u\n", total_num);
	return ret;
}

static uint32 camkit_translate_sensor_params(
	struct camkit_params *kit_params)
{
	struct camkit_sensor_params *sensor_params = NULL;
	uint32 ret = ERROR_NONE;
	int i;

	if (kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	sensor_params = kzalloc(sizeof(struct camkit_sensor_params), GFP_KERNEL);
	if (!sensor_params) {
		pr_err("memory not enough\n");
		kit_params->sensor_params = NULL;
		return ENOMEM;
	}

	if (copy_from_user(sensor_params,
		(void *)kit_params->sensor_params,
		sizeof(struct camkit_sensor_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(sensor_params);
		kit_params->sensor_params = NULL;
		return EFAULT;
	}
	kit_params->sensor_params = sensor_params;

	ret |= camkit_translate_sensor_info(&sensor_params->sensor_info);
	ret |= camkit_translate_sensor_ctrl(&sensor_params->sensor_ctrl);
	ret |= camkit_translate_sensor_output_info(sensor_params->output_info);
	ret |= camkit_translate_pdaf_info(&sensor_params->pdaf_info);
	for (i = 0; i < ARRAY_SIZE(sensor_params->power_info); i++) {
		ret |= camkit_translate_power_info(&(sensor_params->power_info[i]));
		ret |= camkit_translate_power_info(&(sensor_params->power_down_info[i]));
	}
	ret |= camkit_translate_aec_info(&sensor_params->aec_info);
	ret |= camkit_translate_vc_info(sensor_params->vc_info);
	ret |= camkit_translate_correction_info(&sensor_params->correction_info);
	ret |= camkit_translate_high_impedance_info(&sensor_params->high_impedance_info);
	ret |= camkit_translate_pdaf_info_scenario(sensor_params->pdaf_info_scenario);
	ret |= camkit_translate_ext_power_info(&sensor_params->ext_power_info);

	return ret;
}

static uint32 camkit_translate_module_params(
	struct camkit_params *kit_params)
{
	struct camkit_module_params *module_params = NULL;
	uint32 ret = ERROR_NONE;

	if (kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	module_params = kzalloc(sizeof(struct camkit_module_params), GFP_KERNEL);
	if (!module_params) {
		pr_err("memory not enough\n");
		kit_params->module_params = NULL;
		return ENOMEM;
	}

	if (copy_from_user(module_params,
		(void *)kit_params->module_params,
		sizeof(struct camkit_module_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(module_params);
		kit_params->module_params = NULL;
		return EFAULT;
	}
	kit_params->module_params = module_params;

	ret |= camkit_translate_sensor_setting(
		&module_params->config_eeprom.attach_setting);
	ret |= camkit_translate_sensor_setting(
		&module_params->config_eeprom.detach_setting);

	LOG_DBG("skip_module_id: %u\n", module_params->skip_module_id);
	LOG_DBG("eeprom_i2c_addr: %u\n", module_params->eeprom_i2c_addr);
	LOG_DBG("module_code_addr: %u\n", module_params->module_code_addr);
	LOG_DBG("module_code: %u\n", module_params->module_code);
	LOG_DBG("lens_type_addr: %u\n", module_params->lens_type_addr);
	LOG_DBG("lens_type: %u\n", module_params->lens_type);
	LOG_DBG("addr_type: %u\n", module_params->addr_type);
	LOG_DBG("data_type: %u\n", module_params->data_type);
	LOG_DBG("sensor_name: %s\n", module_params->sensor_name);

	return ret;
}

static uint32 camkit_fill_sensor_params(struct IMGSENSOR_SENSOR *sensor,
	struct camkit_params *user_kit_params)
{
	struct camkit_params *kit_params = NULL;
	uint32 ret = ERROR_NONE;

	if (sensor == NULL || user_kit_params == NULL) {
		pr_err("invalid input arg\n");
		return EFAULT;
	}

	kit_params = kzalloc(sizeof(struct camkit_params), GFP_KERNEL);
	if (!kit_params) {
		pr_err("memory not enough\n");
		return ENOMEM;
	}

	if (copy_from_user(kit_params, (void *)user_kit_params,
		sizeof(struct camkit_params))) {
		pr_err("failed: copy camkit parameters from user\n");
		kfree(kit_params);
		return EFAULT;
	}
	sensor->kit_params = kit_params;

	ret |= camkit_translate_sensor_params(kit_params);
	ret |= camkit_translate_module_params(kit_params);

	return ret;
}

static void camkit_free_high_impedance_sensor_settings (
	struct camkit_high_impedance_info_t *high_impedance_info)
{
	uint32 i;
	uint8 total_num;
	struct camkit_high_impedance_sensor_t *sensor = NULL;

	if (!high_impedance_info)
		return;

	total_num = high_impedance_info->total_num;
	if (total_num == 0 || total_num > MAX_HIGH_IMPEDANCE_SENSOR_NUM) {
		LOG_DBG("%s no need to free memory\n", __func__);
		return;
	}

	sensor = &high_impedance_info->sensor[0];
	for (i = 0; i < total_num; i++) {
		if (sensor[i].high_impedance_setting.setting) {
			LOG_DBG("free settings memory for sensor %u\n", i);
			kfree(sensor[i].high_impedance_setting.setting);
			sensor[i].high_impedance_setting.setting = NULL;
		}
	}
}

static void camkit_free_pdc_config_setting(
	struct camkit_sensor_correction_t *correction_info)
{
	if (correction_info->pdc_config.before_pdc_setting.setting) {
		kfree(correction_info->pdc_config.before_pdc_setting.setting);
		correction_info->pdc_config.before_pdc_setting.setting = NULL;
	}
	if (correction_info->pdc_config.after_pdc_setting.setting) {
		kfree(correction_info->pdc_config.after_pdc_setting.setting);
		correction_info->pdc_config.after_pdc_setting.setting = NULL;
	}
}

static void camkit_free_seamless_cfg(struct camkit_seamless_cfg_t *seamless_cfg)
{
	uint32 i;
	if (!seamless_cfg ||
		seamless_cfg->seamless_support != CAMKIT_SEAMLESS_SUPPORT) {
		LOG_DBG("no need to free seamless cfg\n");
		return;
	}

	if (seamless_cfg->scenario_id_map) {
		LOG_DBG("free scenario_id_map\n");
		kfree(seamless_cfg->scenario_id_map);
		seamless_cfg->scenario_id_map = NULL;
		seamless_cfg->scenario_id_map_size = 0;
	}

	if (!seamless_cfg->scenario_cfg) {
		LOG_DBG("no need to free seamless scenario cfg\n");
		return;
	}

	for (i = 0; i < seamless_cfg->scenario_cfg_size; ++i) {
		if (seamless_cfg->scenario_cfg[i].seamless_setting.setting) {
			kfree(seamless_cfg->scenario_cfg[i].seamless_setting.setting);
			seamless_cfg->scenario_cfg[i].seamless_setting.setting = NULL;
		}
	}

	kfree(seamless_cfg->scenario_cfg);
	seamless_cfg->scenario_cfg = NULL;
}

static void camkit_free_stagger_cfg(struct camkit_stagger_cfg_t *stagger_cfg)
{
	if (!stagger_cfg ||
		stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("do not support stagger, no need to do translate\n");
		return;
	}

	if (stagger_cfg->scenario_id_map) {
		LOG_DBG("free scenario_id_map\n");
		kfree(stagger_cfg->scenario_id_map);
		stagger_cfg->scenario_id_map = NULL;
		stagger_cfg->scenario_id_map_size = 0;
	}

	if (stagger_cfg->stagger_type_map) {
		LOG_DBG("free stagger_type_map\n");
		kfree(stagger_cfg->stagger_type_map);
		stagger_cfg->stagger_type_map = NULL;
		stagger_cfg->stagger_type_map_size = 0;
	}

	if (stagger_cfg->stagger_max_expo) {
		LOG_DBG("free stagger_max_expo\n");
		kfree(stagger_cfg->stagger_max_expo);
		stagger_cfg->stagger_max_expo = NULL;
		stagger_cfg->stagger_max_expo_size = 0;
	}
}

static int camkit_free_sensor_params(struct camkit_params *kit_params)
{
	struct camkit_sensor_info_t *sensor_info = NULL;
	struct camkit_aec_info_t *aec_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	struct camkit_config_eeprom *config_eeprom = NULL;

	if (kit_params != NULL && kit_params->sensor_params != NULL) {
		sensor_params = kit_params->sensor_params;
		sensor_info = &(sensor_params->sensor_info);
		if (sensor_info->id_init_setting.setting != NULL)
			kfree(sensor_info->id_init_setting.setting);
		if (sensor_info->init_setting.setting != NULL)
			kfree(sensor_info->init_setting.setting);
		if (sensor_info->init_burst_setting.setting != NULL)
			kfree(sensor_info->init_burst_setting.setting);
		if (sensor_info->pre_setting.setting != NULL)
			kfree(sensor_info->pre_setting.setting);
		if (sensor_info->cap_setting.setting != NULL)
			kfree(sensor_info->cap_setting.setting);
		if (sensor_info->cap1_setting.setting != NULL)
			kfree(sensor_info->cap1_setting.setting);
		if (sensor_info->normal_video_setting.setting != NULL)
			kfree(sensor_info->normal_video_setting.setting);
		if (sensor_info->hs_video_setting.setting != NULL)
			kfree(sensor_info->hs_video_setting.setting);
		if (sensor_info->slim_setting.setting != NULL)
			kfree(sensor_info->slim_setting.setting);
		if (sensor_info->custom1_setting.setting != NULL)
			kfree(sensor_info->custom1_setting.setting);
		if (sensor_info->custom2_setting.setting != NULL)
			kfree(sensor_info->custom2_setting.setting);
		if (sensor_info->custom3_setting.setting != NULL)
			kfree(sensor_info->custom3_setting.setting);
		if (sensor_info->custom4_setting.setting != NULL)
			kfree(sensor_info->custom4_setting.setting);
		if (sensor_info->custom5_setting.setting != NULL)
			kfree(sensor_info->custom5_setting.setting);
		if (sensor_info->streamon_setting.setting != NULL)
			kfree(sensor_info->streamon_setting.setting);
		if (sensor_info->streamoff_setting.setting != NULL)
			kfree(sensor_info->streamoff_setting.setting);
		if (sensor_info->test_pattern_on_setting.setting != NULL)
			kfree(sensor_info->test_pattern_on_setting.setting);
		if (sensor_info->test_pattern_off_setting.setting != NULL)
			kfree(sensor_info->test_pattern_off_setting.setting);
		if (sensor_info->normal_to_long_ready_settings.setting != NULL)
			kfree(sensor_info->normal_to_long_ready_settings.setting);
		if (sensor_info->normal_to_long_end_settings.setting != NULL)
			kfree(sensor_info->normal_to_long_end_settings.setting);
		if (sensor_info->streamoff_check.setting != NULL)
			kfree(sensor_info->streamoff_check.setting);

		aec_info = &(sensor_params->aec_info);
		if (aec_info->gain_ops_map.aec_ops != NULL)
			kfree(aec_info->gain_ops_map.aec_ops);
		if (aec_info->expo_ops_map.aec_ops != NULL)
			kfree(aec_info->expo_ops_map.aec_ops);
		if (aec_info->vts_ops_map.aec_ops != NULL)
			kfree(aec_info->vts_ops_map.aec_ops);
		if (aec_info->priv_again_info.again_map != NULL)
			kfree(aec_info->priv_again_info.again_map);
		if (aec_info->normal2long_ops_map.aec_ops != NULL)
			kfree(aec_info->normal2long_ops_map.aec_ops);
		if (aec_info->long2normal_ops_map.aec_ops != NULL)
			kfree(aec_info->long2normal_ops_map.aec_ops);
		if (aec_info->awb_gain_ops_map.aec_ops) {
			kfree(aec_info->awb_gain_ops_map.aec_ops);
			aec_info->awb_gain_ops_map.aec_ops = NULL;
		}
		if (aec_info->hdr_m_gain_ops_map.aec_ops) {
			kfree(aec_info->hdr_m_gain_ops_map.aec_ops);
			aec_info->hdr_m_gain_ops_map.aec_ops = NULL;
		}
		if (aec_info->hdr_m_expo_ops_map.aec_ops) {
			kfree(aec_info->hdr_m_expo_ops_map.aec_ops);
			aec_info->hdr_m_expo_ops_map.aec_ops = NULL;
		}

		camkit_free_pdc_config_setting(&sensor_params->correction_info);

		camkit_free_high_impedance_sensor_settings(&sensor_params->high_impedance_info);

		camkit_free_seamless_cfg(&sensor_info->seamless_cfg);
		camkit_free_stagger_cfg(&sensor_info->stagger_cfg);
		kfree(sensor_params);
		sensor_params = NULL;
	}

	if (kit_params != NULL && kit_params->module_params != NULL) {
		config_eeprom = &kit_params->module_params->config_eeprom;
		if (config_eeprom->attach_setting.setting) {
			kfree(config_eeprom->attach_setting.setting);
			config_eeprom->attach_setting.setting = NULL;
		}
		if (config_eeprom->detach_setting.setting) {
			kfree(config_eeprom->detach_setting.setting);
			config_eeprom->detach_setting.setting = NULL;
		}

		kfree(kit_params->module_params);
		kit_params->module_params = NULL;
	}

	if (kit_params != NULL) {
		kfree(kit_params);
		kit_params = NULL;
	}

	return ERROR_NONE;
}

static bool camkit_match_name_and_index(
	const char *sensor_name, uint32 kit_sensor_index)
{
	size_t name_length;
	enum IMGSENSOR_SENSOR_IDX sensor_index =
		camkit_translate_sensor_index(kit_sensor_index);

	if (sensor_index < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
		sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		LOG_ERR("invalid sensor index %d\n", sensor_index);
		return false;
	}

	name_length = strlen(sensor_name);
	if (name_length == 0) {
		LOG_ERR("sensor name is empty\n");
		return false;
	}

	if (!strncmp(sensor_name, g_sensor_idx_info[sensor_index].sensor_name,
		name_length))
		return true;

	return false;
}

static void camkit_match_impdeance_sensor(
	struct camkit_sensor_params *sensor_params)
{
	uint8 i;
	uint8 total_num;
	uint32 kit_sensor_index;
	char *sensor_name = NULL;
	struct camkit_high_impedance_info_t *high_impedance = NULL;
	struct camkit_high_impedance_sensor_t *sensor = NULL;

	if (!sensor_params) {
		LOG_ERR("sensor_params is null\n");
		return;
	}

	high_impedance = &sensor_params->high_impedance_info;
	/* init not matched as default */
	high_impedance->matched_flag = 0;
	high_impedance->matched_index = 0;

	/* need to do high impedace match: 1, probe ok: 1, same as before */
	if ((sensor_params->sensor_info.need_high_impedance != 1) ||
		(sensor_params->sensor_info.probe_flag != 1)) {
		LOG_DBG("no need to match high impdance sensor\n");
		return;
	}

	total_num = high_impedance->total_num;
	if (total_num == 0 || total_num > MAX_HIGH_IMPEDANCE_SENSOR_NUM) {
		LOG_DBG("invalid total num %u\n", total_num);
		return;
	}

	sensor = &high_impedance->sensor[0];
	for (i = 0; i < total_num; i++) {
		sensor_name = sensor[i].sensor_name;
		kit_sensor_index = sensor[i].sensor_index;
		if (camkit_match_name_and_index(sensor_name, kit_sensor_index)) {
			high_impedance->matched_flag = 1; /* matched ok: 1 */
			high_impedance->matched_index = i;
			LOG_INF("success match the high impedance sensor %s idx %u\n",
				sensor_name, i);
			break;
		}
	}

	return;
}

int32 camkit_sensor_open(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params = {0};
#endif
	struct IMGSENSOR             *pimgsensor   = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	struct sensor_kit_ops          *sensor_ops = psensor->sensor_ops;
	struct camkit_params               *params = psensor->kit_params;

#ifdef CONFIG_MTK_CCU
	struct ccu_sensor_info ccuSensorInfo;
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	struct i2c_client *pi2c_client = NULL;
#endif

	if (sensor_ops && sensor_ops->sensor_open && psensor_inst) {

		/* turn on power */
		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);
		check_camera_btb_gpio_info(psensor_inst->sensor_idx);
		ret = camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_ON);

		if (pimgsensor->imgsensor_oc_irq_enable)
			pimgsensor->imgsensor_oc_irq_enable(psensor->inst.sensor_idx, true);

		if (ret != IMGSENSOR_RETURN_SUCCESS) {
			PK_PR_ERR("[%s]", __func__);
			return ret;
		}

		IMGSENSOR_PROFILE(&psensor_inst->profile_time,
			"kdCISModulePowerOn");

		imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
		(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_OPEN, c_params, &ret_sec);

		} else {
#endif
			ret = sensor_ops->sensor_open(params);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif

		if (ret != ERROR_NONE) {
			// imgsensor_hw_dump(&pimgsensor->hw);
			camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_OFF);
			if (pimgsensor->imgsensor_oc_irq_enable)
				pimgsensor->imgsensor_oc_irq_enable(psensor->inst.sensor_idx, false);

			PK_PR_ERR("sensor_open fail");
		} else {
			psensor_inst->state = IMGSENSOR_STATE_OPEN;
		}

#ifdef CONFIG_MTK_CCU
		ccuSensorInfo.slave_addr =
		    (psensor_inst->i2c_cfg.msg->addr << 1);
		ccuSensorInfo.sensor_name_string =
		    (char *)(params->module_params->sensor_name);
		pi2c_client = psensor_inst->i2c_cfg.pinst->pi2c_client;
		if (pi2c_client)
			ccuSensorInfo.i2c_id = (((struct mt_i2c *)
				i2c_get_adapdata(pi2c_client->adapter))->id);
		else
			ccuSensorInfo.i2c_id = -1;
		ccu_set_sensor_info(sensor_idx, &ccuSensorInfo);
#endif

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time, "sensor_open");
	}

	return ret;
}

uint32 camkit_sensor_get_info(
	struct IMGSENSOR_SENSOR *psensor,
	uint32 scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	uint32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_get_info &&
	    psensor_inst &&
	    sensor_info &&
	    sensor_config_data) {

		imgsensor_mutex_lock(psensor_inst);

		ret = psensor->sensor_ops->sensor_get_info(
			psensor->kit_params,
		    (enum MSDK_SCENARIO_ID_ENUM)(scenario_id),
		    sensor_info,
		    sensor_config_data);

		if (ret != ERROR_NONE)
			PK_PR_ERR("[%s] sensor_get_info failed\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

uint32 camkit_sensor_get_resolution(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	MUINT32 ret = ERROR_NONE;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_get_resolution &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

		ret = psensor->sensor_ops->sensor_get_resolution(
			psensor->kit_params,
			sensor_resolution);
		if (ret != ERROR_NONE)
			PK_PR_ERR("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

uint32 camkit_sensor_control(
	struct IMGSENSOR_SENSOR *psensor,
	enum MSDK_SCENARIO_ID_ENUM scenario_id)
{
	uint32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params;
#endif
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;

	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT image_window;
	MSDK_SENSOR_CONFIG_STRUCT sensor_config_data;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_control &&
	    psensor_inst) {

		IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

		imgsensor_mutex_lock(psensor_inst);

		//psensor_func->psensor_inst = psensor_inst;
		//psensor_func->ScenarioId = ScenarioId;

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	PK_INFO("%s secure state %d", __func__,
		(int)(&gimgsensor)->imgsensor_sec_flag);
	if ((&gimgsensor)->imgsensor_sec_flag) {
		c_params.param0 = (void *)scenario_id;
		c_params.param1 = (void *)&image_window;
		c_params.param2 = (void *)&sensor_config_data;
		ret = imgsensor_ca_invoke_command(
			IMGSENSOR_TEE_CMD_CONTROL, c_params, &ret_sec);
	} else {
#endif
		ret = psensor->sensor_ops->sensor_control(psensor->kit_params,
			scenario_id, &image_window, &sensor_config_data);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	}
#endif
		if (ret != ERROR_NONE)
			PK_PR_ERR("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);

		IMGSENSOR_PROFILE(&psensor_inst->profile_time, "SensorControl");
	}

	return ret;
}

uint32 camkit_sensor_feature_control(
		struct IMGSENSOR_SENSOR *psensor,
		MSDK_SENSOR_FEATURE_ENUM feature_id,
		MUINT8 *feature_param,
		MUINT32 *feature_param_len)
{
	MUINT32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	MINT32 ret_sec = ERROR_NONE;
	struct command_params c_params;
#endif
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;

	if ((&gimgsensor)->camkit_enabled && psensor->sensor_ops &&
		psensor->sensor_ops->sensor_feature_control) {
		imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
			(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			c_params.param0 = (void *)feature_id;
			c_params.param1 = (void *)feature_param;
			c_params.param2 = (void *)feature_param_len;
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_FEATURE_CONTROL, c_params, &ret_sec);
		} else {
#endif
			ret = psensor->sensor_ops->sensor_feature_control(
					psensor->kit_params, feature_id,
					feature_param, feature_param_len);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif
		if (ret != ERROR_NONE)
			PK_PR_ERR("[%s]\n", __func__);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

int32 camkit_sensor_close(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret = ERROR_NONE;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params = {0};
#endif
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST  *psensor_inst = &psensor->inst;

	if (psensor->sensor_ops &&
	    psensor->sensor_ops->sensor_close &&
	    psensor_inst) {

		imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		PK_INFO("%s secure state %d", __func__,
			(int)(&gimgsensor)->imgsensor_sec_flag);
		if ((&gimgsensor)->imgsensor_sec_flag) {
			ret = imgsensor_ca_invoke_command(
				IMGSENSOR_TEE_CMD_CLOSE, c_params, &ret_sec);
		} else {
#endif
			ret = psensor->sensor_ops->sensor_close(psensor->kit_params);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
		}
#endif
		if (ret != ERROR_NONE) {
			PK_PR_ERR("[%s]", __func__);
		} else {
			camkit_hw_power(&pimgsensor->hw,
				psensor,
				IMGSENSOR_HW_POWER_STATUS_OFF);

			psensor_inst->state = IMGSENSOR_STATE_CLOSE;
		}

		if (pimgsensor->imgsensor_oc_irq_enable)
			pimgsensor->imgsensor_oc_irq_enable(psensor->inst.sensor_idx, false);

		imgsensor_mutex_unlock(psensor_inst);
	}

	return ret;
}

static void camkit_get_fuse_id(struct IMGSENSOR_SENSOR *psensor,
			struct camkit_probe_sensor_params *user_params)
{
	int size;
	struct camkit_sensor_params *sensor_params = NULL;
	if (!(psensor->kit_params) || !(psensor->kit_params->sensor_params))
		return;

	sensor_params = psensor->kit_params->sensor_params;
	if (sensor_params->sensor_info.need_get_fuse_id == 0)
		return;

	size = sizeof(user_params->fuse_id);
	camkit_sensor_feature_control(psensor, SENSOR_FEATURE_GET_FUSE_ID,
		user_params->fuse_id, &size);

	return;
}

static int camkit_probe_sensor(struct IMGSENSOR_SENSOR *psensor, struct camkit_probe_sensor_params *user_params)
{
	int32 ret = ERROR_NONE;
	uint32 err = 0;
	uint32 sensor_id = 0;
	uint32 len = sizeof(uint32);
	struct IMGSENSOR *pimgsensor = &gimgsensor;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = &psensor->inst;
	int i;
	struct IMGSENSOR_HW *phw = &pimgsensor->hw;

	IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

	psensor_inst->state = IMGSENSOR_STATE_CLOSE;
	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++)
		if (phw->pdev[i] && phw->pdev[i]->hw_reset)
			(phw->pdev[i]->hw_reset)(phw->pdev[i]->pinstance);

	ret = camkit_hw_power(&pimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_ON);

	if (pimgsensor->imgsensor_oc_irq_enable)
		pimgsensor->imgsensor_oc_irq_enable(psensor->inst.sensor_idx,
			true);

	if (ret != IMGSENSOR_RETURN_SUCCESS)
		return ERROR_SENSOR_CONNECT_FAIL;

	camkit_sensor_feature_control(psensor, SENSOR_FEATURE_CHECK_SENSOR_ID,
		(MUINT8 *)&sensor_id, &len);

	/* not implement this feature ID */
	if (sensor_id == 0 || sensor_id == 0xFFFFFFFF) {
		pr_err("Fail to get sensor ID %x\n", sensor_id);
		err = ERROR_SENSOR_CONNECT_FAIL;
	} else {
		pr_err("Sensor found ID = 0x%x\n", sensor_id);
		err = ERROR_NONE;
	}

	camkit_get_fuse_id(psensor, user_params);

	ret = camkit_hw_power(&pimgsensor->hw, psensor, IMGSENSOR_HW_POWER_STATUS_OFF);

	if (pimgsensor->imgsensor_oc_irq_enable)
		pimgsensor->imgsensor_oc_irq_enable(psensor->inst.sensor_idx,
			false);

	IMGSENSOR_PROFILE(&psensor_inst->profile_time, "CheckIsAlive");

	return err ? -EIO : err;
}

int adopt_camera_hw_probe_sensor(void *pBuf)
{
	uint32 ret;
	static uint32 open_flag;

	struct camkit_probe_sensor_params *user_params = NULL;
	struct IMGSENSOR_SENSOR               *psensor = NULL;
	struct IMGSENSOR_SENSOR_INST      *sensor_inst = NULL;
	struct camkit_params          *user_kit_params = NULL;
	enum IMGSENSOR_SENSOR_IDX sensor_idx;
	struct IMGSENSOR                   *pimgsensor = &gimgsensor;
	struct IMGSENSOR_HW                       *phw = &pimgsensor->hw;

	if (open_flag == 0) {
		pr_err("KKK init hw again\n");
		imgsensor_hw_init(phw);
		open_flag = 1;
	}

	pimgsensor->camkit_enabled = 1;
	pr_info("camkit_enabled = %d\n", pimgsensor->camkit_enabled);

	user_params = (struct camkit_probe_sensor_params *)pBuf;
	if (user_params == NULL) {
		pr_err("[%s] NULL arg\n", __func__);
		return -EFAULT;
	}

	user_kit_params = user_params->kit_params;
	if (user_kit_params == NULL) {
		pr_err("[%s] camkit params from user is NULL\n", __func__);
		return -EFAULT;
	}

	pr_err("sensor index = %d\n", user_params->sensor_idx);
	sensor_idx = camkit_translate_sensor_index(user_params->sensor_idx);
	psensor = camkit_get_sensor(sensor_idx);
	if (psensor == NULL) {
		pr_err("[%s] NULL psensor\n", __func__);
		return -EFAULT;
	}
	psensor->inst.sensor_idx = sensor_idx;

	ret = camkit_fill_sensor_params(psensor, user_kit_params);
	if (ret != ERROR_NONE) {
		camkit_free_sensor_params(psensor->kit_params);
		psensor->kit_params = NULL;
		pr_err("[%s] fill sensor parameters failed\n", __func__);
		return -EFAULT;
	}
	sensor_inst = &psensor->inst;
	imgsensor_mutex_init(sensor_inst);
	imgsensor_i2c_init(&sensor_inst->i2c_cfg,
		imgsensor_custom_config[psensor->inst.sensor_idx].i2c_dev);

	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, true);

	check_camera_btb_gpio_info(psensor->inst.sensor_idx);
	if (camkit_probe_sensor(psensor, user_params)) {
		if (psensor->kit_params && psensor->kit_params->module_params)
			pr_info("sensor[%d]: %s mismatch\n", psensor->inst.sensor_idx,
				psensor->kit_params->module_params->sensor_name);

		imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, false);
		camkit_free_sensor_params(psensor->kit_params);
		psensor->kit_params = NULL;

		return -EFAULT;
	}

	// mark the sensor info for device mount success
	g_sensor_idx_info[psensor->inst.sensor_idx].sensor_index = psensor->inst.sensor_idx;
	ret = strncpy_s(g_sensor_idx_info[psensor->inst.sensor_idx].sensor_name, SENSOR_NAME_LEN,
		psensor->kit_params->module_params->sensor_name, SENSOR_NAME_LEN - 1);
	if (ret != ERROR_NONE) {
		pr_err("strncpy_s fail, ret = %d\n", ret);
	}
	psensor->kit_params->sensor_params->sensor_info.probe_flag = 1;

	pr_info("probe sensor[%d]: %s successfully\n", psensor->inst.sensor_idx,
		psensor->kit_params->module_params->sensor_name);

	user_params->probe_succeed = 1;

	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, false);

	return ERROR_NONE;
}

static enum IMGSENSOR_RETURN camkit_sensor_power_sequence(
	struct IMGSENSOR_HW             *phw,
	enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
	enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
	struct camkit_hw_power_info_t   *ppower_info)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr = NULL;
	struct camkit_hw_power_info_t    *ppwr_info = ppower_info;
	struct IMGSENSOR_HW_DEVICE       *pdev = NULL;
	enum IMGSENSOR_HW_PIN             pin_type;
	enum IMGSENSOR_HW_PIN_STATE       pin_val;

	if (!phw || !ppower_info) {
		pr_err("%s invalid parameter\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}

	psensor_pwr = &phw->sensor_pwr[sensor_idx];
	pin_type = (enum IMGSENSOR_HW_PIN)ppwr_info->pin_type;
	pin_val = (enum IMGSENSOR_HW_PIN_STATE)ppwr_info->pin_val;
	while (pin_type != CAMKIT_HW_PIN_NONE &&
		ppwr_info < ppower_info + CAMKIT_HW_PIN_MAX_NUM) {

		if(pin_type != IMGSENSOR_HW_PIN_UNDEF && psensor_pwr->id[pin_type] != IMGSENSOR_HW_ID_NONE) {
			pdev = phw->pdev[psensor_pwr->id[pin_type]];
			pr_info("%s sensor_idx = %d, pin_type=%d, pin_val=%d, hw_id =%d\n",
				__func__, sensor_idx, pin_type, pin_val, psensor_pwr->id[pin_type]);

			if (pdev && pdev->set)
				pdev->set(pdev->pinstance, sensor_idx, pin_type, pin_val);

			mdelay(ppwr_info->pin_delay);
		}

		ppwr_info++;
		pin_type = (enum IMGSENSOR_HW_PIN)ppwr_info->pin_type;
		pin_val = (enum IMGSENSOR_HW_PIN_STATE)ppwr_info->pin_val;
	}

	/* wait for power stable */
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		mdelay(5);

	return IMGSENSOR_RETURN_SUCCESS;
}

struct camkit_hw_power_info_t* camkit_get_power_info(
	struct camkit_sensor_params *sensor_params,
	uint32 sensor_idx,
	enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	struct camkit_high_impedance_info_t *info = NULL;
	struct camkit_extend_hw_power_info_t *ext_info = NULL;
	uint32 power_setting_idx;

	if (!sensor_params) {
		LOG_ERR("sensor_params is null\n");
		return NULL;
	}

	if (sensor_idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
		sensor_idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		LOG_ERR("%s: invalid sensor idx %u\n", __func__, sensor_idx);
		return NULL;
	}

	ext_info = &sensor_params->ext_power_info;
	power_setting_idx = g_power_setting_map[sensor_idx].power_setting_idx;
	if (g_power_setting_map[sensor_idx].is_map_vaild &&
		power_setting_idx < ext_info->total_num) {
		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
			LOG_INF("%s: success get ext power info, power statue on, idx %u\n",
				__func__, power_setting_idx);
			return ext_info->info_pair[power_setting_idx].power_info;
		}

		LOG_INF("%s: success get ext power info, power statue off, idx %u\n",
			__func__, power_setting_idx);
		return ext_info->info_pair[power_setting_idx].power_down_info;
	}

	info = &sensor_params->high_impedance_info;
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
		if (info->matched_flag == 0)
			return sensor_params->power_info;
		return info->sensor[info->matched_index].power_info;
	}

	if (info->matched_flag == 0)
		return sensor_params->power_down_info;

	return info->sensor[info->matched_index].power_down_info;;
}
enum IMGSENSOR_RETURN camkit_hw_power(
		struct IMGSENSOR_HW *phw,
		struct IMGSENSOR_SENSOR *psensor,
		enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	struct camkit_hw_power_info_t *power_info = NULL;
	enum IMGSENSOR_SENSOR_IDX sensor_idx;
	struct camkit_sensor_params *sensor_params = NULL;

	if (!phw || !psensor || !(psensor->kit_params) ||
		!(psensor->kit_params->sensor_params))
		return IMGSENSOR_RETURN_ERROR;

	sensor_params = psensor->kit_params->sensor_params;
	sensor_idx = psensor->inst.sensor_idx;

	camkit_match_impdeance_sensor(sensor_params);
	power_info = camkit_get_power_info(sensor_params, (uint32)sensor_idx, pwr_status);

	camkit_sensor_power_sequence(
		phw,
		sensor_idx,
		pwr_status,
		power_info);

	return IMGSENSOR_RETURN_SUCCESS;
}

void camkit_get_sensor_power_idx_for_each_sensor(unsigned int sensor_idx)
{
	struct device_node *camera_node = NULL;
	char node_name[64] = {0}; /* compatible node name size is 64 */
	int ret;
	unsigned int power_setting_idx = 0;

	if (sensor_idx > IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		pr_err("%s: invalid sensor idx %u\n", __func__, sensor_idx);
		return;
	}

	ret = sprintf_s(node_name, sizeof(node_name) - 1, "huawei,camera%u", sensor_idx);
	if (ret < 0) {
		pr_err("%s: sprintf_s failed, sensor idx %u\n", __func__, sensor_idx);
		return;
	}

	pr_info("%s: node name %s, sensor idx %u", __func__, node_name, sensor_idx);
	camera_node = of_find_compatible_node(NULL, NULL, node_name);
	if (!camera_node) {
		pr_info("%s: not find node %s\n", __func__, node_name);
		return;
	}

	ret = of_property_read_u32(camera_node, "power-setting-idx", (u32 *)&power_setting_idx);
	if (ret < 0) {
		pr_info("%s: read power-setting-idx failed\n", __func__);
		return;
	}

	if (power_setting_idx > MAX_EXT_POWER_INFO_NUM) {
		pr_err("%s: invalid power settings idx %u\n", __func__, power_setting_idx);
		return;
	}

	g_power_setting_map[sensor_idx].is_map_vaild = true;
	g_power_setting_map[sensor_idx].power_setting_idx = power_setting_idx;
	pr_info("%s success get ext power setting, sensor idx %u, power_setting_idx %u\n",
		__func__, sensor_idx, power_setting_idx);
}

void camkit_get_power_info_from_dts(void)
{
	errno_t ret;
	int rc;
	int total_sensor_idx_num;
	int i;
	unsigned int sensor_idx_array[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {0};
	struct device_node *of_node = NULL;

	pr_info("%s: enter\n", __func__);
	ret = memset_s(g_power_setting_map, sizeof(g_power_setting_map),
		0, sizeof(g_power_setting_map));
	if (ret != EOK) {
		pr_err("%s: power settings map memset failed, ret %d\n", __func__, ret);
		return;
	}

	of_node = of_find_compatible_node(NULL, NULL, "mediatek,camera");
	if (!of_node) {
		pr_info("%s: not support power info extend by dts\n", __func__);
		return;
	}

	total_sensor_idx_num = of_property_count_elems_of_size(of_node,
		"total-sensor-idx", sizeof(u32));
	if (total_sensor_idx_num <= 0 ||
		total_sensor_idx_num > IMGSENSOR_SENSOR_IDX_MAX_NUM) {
		pr_info("%s: invalid total sensor idx num %d\n", __func__, total_sensor_idx_num);
		return;
	}

	rc = of_property_read_u32_array(of_node, "total-sensor-idx",
		sensor_idx_array, total_sensor_idx_num);
	if (rc < 0) {
		pr_err("%s: read total-sensor-idx failed\n", __func__);
		return;
	}

	for (i = 0; i < total_sensor_idx_num; ++i)
		camkit_get_sensor_power_idx_for_each_sensor(sensor_idx_array[i]);
}

int camkit_driver_init(struct IMGSENSOR *img_sensor)
{
	int i;
	int ret;
	struct sensor_kit_ops *sensor_ops = NULL;

	if (!img_sensor) {
		pr_err("%s invalid parameter\n", __func__);
		return 0;
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		sensor_ops = img_sensor->sensor[i].sensor_ops;
		sensor_driver_init(&img_sensor->sensor[i].sensor_ops);
		pr_info("%s set options for camera[%d]\n", __func__, i);
	}

	ret = memset_s(g_sensor_idx_info, sizeof(g_sensor_idx_info), 0, sizeof(g_sensor_idx_info));
	if (ret != 0)
		pr_err("%s: memset fail\n", __func__);

	camkit_get_power_info_from_dts();

	return 0;
}

