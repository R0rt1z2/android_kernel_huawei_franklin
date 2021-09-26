/*
 * camkit_probe_sensor.c
 *
 * Copyright (c) huawei technologies co., ltd. 2020-2020. all rights reserved.
 *
 * Description: probe sensor
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

#include "camkit_probe_sensor.h"
#include "camkit_driver_interface.h"

#include "camkit_driver_impl.h"

// define in kernel/arch/arm64/kernel/vmlinux.lds.s
extern struct customized_driver __start_customized_drv_tag,
	__stop_customized_drv_tag;

struct camkit_sensor g_sensor[CAMKIT_SENSOR_IDX_MAX_NUM];

#if (DRIVER_VERSION == 1)
int eeprom_read_data(unsigned int sensor_id);
#endif

struct camkit_sensor *get_camkit_sensor(const uint32 index)
{
	if (index < CAMKIT_SENSOR_IDX_MIN_NUM ||
		index >= CAMKIT_SENSOR_IDX_MAX_NUM)
		return NULL;
	else
		return &g_sensor[index];
}

static int camkit_print_mode_info(struct camkit_mode_info *mode_info)
{
	probe_dbg("pclk: %u", mode_info->pclk);
	probe_dbg("linelength: %u", mode_info->linelength);
	probe_dbg("framelength: %u", mode_info->framelength);
	probe_dbg("startx: %u", mode_info->startx);
	probe_dbg("starty: %u", mode_info->starty);
	probe_dbg("grabwindow_width: %u", mode_info->grabwindow_width);
	probe_dbg("grabwindow_height: %u", mode_info->grabwindow_height);
	probe_dbg("mipi_settle_dc: %u", mode_info->mipi_data_lp2hs_settle_dc);
	probe_dbg("max_framerate: %u", mode_info->max_framerate);
	probe_dbg("mipi_pixel_rate: %u", mode_info->mipi_pixel_rate);
	probe_dbg("mipi_trail_val: %u", mode_info->mipi_trail_val);

	return ERR_NONE;
}

static uint32 camkit_translate_sensor_setting(
	struct camkit_i2c_reg_setting *i2c_setting)
{
	int len;
	int i;
	struct camkit_i2c_reg *setting = NULL;

	if (i2c_setting == NULL) {
		log_err("[camkit][%s] invalid input arg", __func__);
		return ERR_INVAL;
	}

	if (!i2c_setting->size || i2c_setting->setting == NULL) {
		log_info("[camkit][%s] setting need't configure?", __func__);
		i2c_setting->setting = NULL;
		return ERR_NONE;
	}

	probe_dbg("sensor setting size: %u", i2c_setting->size);

	len = i2c_setting->size;
	setting = kcalloc(len, sizeof(struct camkit_i2c_reg), GFP_KERNEL);
	if (!setting) {
		i2c_setting->setting = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(setting,
		(void *)i2c_setting->setting,
		sizeof(struct camkit_i2c_reg) * len)) {
		log_err("failed: copy_from_user");
		kfree(setting);
		i2c_setting->setting = NULL;
		return ERR_INVAL;
	}

	i2c_setting->setting = setting;
	for (i = 0; i < len; i++) {
		probe_dbg("setting[%d].addr: 0x%x", i, setting[i].addr);
		probe_dbg("setting[%d].data: 0x%x", i, setting[i].data);
		probe_dbg("setting[%d].delay: 0x%x", i, setting[i].delay);
	}

	probe_dbg("setting size: %u", i2c_setting->size);
	probe_dbg("setting addr_type: %u", i2c_setting->addr_type);
	probe_dbg("setting data_type: %u", i2c_setting->data_type);
	probe_dbg("setting delay: %u", i2c_setting->delay);

	return ERR_NONE;
}

static uint32 camkit_translate_table_array(
	struct camkit_i2c_reg_table_array *reg_table)
{
	int len;
	int i;
	struct camkit_i2c_reg_table *setting = NULL;

	if (reg_table == NULL) {
		log_err("[camkit][%s] invalid input arg", __func__);
		return ERR_INVAL;
	}

	if (!reg_table->size || !reg_table->setting) {
		log_info("[camkit][%s] reg table not configure", __func__);
		reg_table->setting = NULL;
		return ERR_NONE;
	}

	probe_dbg("dump setting size: %u", reg_table->size);

	len = reg_table->size;
	setting = kcalloc(len, sizeof(struct camkit_i2c_reg_table), GFP_KERNEL);
	if (!setting) {
		reg_table->setting = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(setting,
		(void *)reg_table->setting,
		sizeof(struct camkit_i2c_reg_table) * len)) {
		log_err("failed: copy_from_user");
		kfree(setting);
		reg_table->setting = NULL;
		return ERR_INVAL;
	}

	reg_table->setting = setting;
	for (i = 0; i < len; i++) {
		probe_dbg("setting[%d].addr: 0x%x", i, setting[i].addr);
		probe_dbg("setting[%d].data: 0x%x", i, setting[i].data);
		probe_dbg("setting[%d] type: %u", i, setting[i].data_type);
		probe_dbg("setting[%d] op: %u", i, setting[i].i2c_operation);
		probe_dbg("setting[%d].delay: 0x%x", i, setting[i].delay);
	}

	return ERR_NONE;
}

static uint32 camkit_translate_sensor_info(
	struct camkit_sensor_info_t *sensor_info)
{
	uint32 ret = ERR_NONE;
	int i;
	int rc = ERR_NONE;

	probe_dbg("sensor_id_reg: %u", sensor_info->sensor_id_reg);
	probe_dbg("sensor_id: %u", sensor_info->sensor_id);
	probe_dbg("checksum_value: %u", sensor_info->checksum_value);

	rc += camkit_print_mode_info(&sensor_info->pre);
	rc += camkit_print_mode_info(&sensor_info->cap);
	rc += camkit_print_mode_info(&sensor_info->cap1);
	rc += camkit_print_mode_info(&sensor_info->normal_video);
	rc += camkit_print_mode_info(&sensor_info->hs_video);
	rc += camkit_print_mode_info(&sensor_info->slim_video);
	rc += camkit_print_mode_info(&sensor_info->custom1);
	rc += camkit_print_mode_info(&sensor_info->custom2);
	rc += camkit_print_mode_info(&sensor_info->custom3);
	rc += camkit_print_mode_info(&sensor_info->custom4);
	rc += camkit_print_mode_info(&sensor_info->custom5);
	if (rc != ERR_NONE)
		log_err("camkit_print_mode_info failed");

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
	ret |= camkit_translate_sensor_setting(
		&sensor_info->test_pattern_on_setting);
	ret |= camkit_translate_sensor_setting(
		&sensor_info->test_pattern_off_setting);
	ret |= camkit_translate_table_array(&sensor_info->dump_info);
	ret |= camkit_translate_sensor_setting(
		&sensor_info->normal_to_long_ready_settings);
	ret |= camkit_translate_sensor_setting(
		&sensor_info->normal_to_long_end_settings);
	ret |= camkit_translate_table_array(&sensor_info->fuse_id_info);

	probe_dbg("ae_shut_delay_frame: %u",
		sensor_info->ae_shut_delay_frame);
	probe_dbg("ae_sensor_gain_delay_frame: %u",
		sensor_info->ae_sensor_gain_delay_frame);
	probe_dbg("ae_isp_gain_delay_frame: %u",
		sensor_info->ae_isp_gain_delay_frame);

	probe_dbg("ihdr_support: %u", sensor_info->ihdr_support);
	probe_dbg("ihdr_le_firstline: %u", sensor_info->ihdr_le_firstline);
	probe_dbg("sensor_mode_num: %u", sensor_info->sensor_mode_num);
	probe_dbg("cap_delay_frame: %u", sensor_info->cap_delay_frame);
	probe_dbg("pre_delay_frame: %u", sensor_info->pre_delay_frame);
	probe_dbg("video_delay_frame: %u", sensor_info->video_delay_frame);

	probe_dbg("hs_video_delay_frame: %u",
		sensor_info->hs_video_delay_frame);
	probe_dbg("slim_video_delay_frame: %u",
		sensor_info->slim_video_delay_frame);

	probe_dbg("custom1_delay_frame: %u", sensor_info->custom1_delay_frame);
	probe_dbg("custom2_delay_frame: %u", sensor_info->custom2_delay_frame);
	probe_dbg("custom3_delay_frame: %u", sensor_info->custom3_delay_frame);
	probe_dbg("custom4_delay_frame: %u", sensor_info->custom4_delay_frame);
	probe_dbg("custom5_delay_frame: %u", sensor_info->custom5_delay_frame);
	probe_dbg("isp_driving_current: %u", sensor_info->isp_driving_current);

	probe_dbg("sensor_interface_type: %u",
		sensor_info->sensor_interface_type);
	probe_dbg("mipi_sensor_type: %u", sensor_info->mipi_sensor_type);
	probe_dbg("mipi_settle_delay_mode: %u",
		sensor_info->mipi_settle_delay_mode);
	probe_dbg("sensor_output_dataformat: %u",
		sensor_info->sensor_output_dataformat);

	probe_dbg("mclk: %u", sensor_info->mclk);
	probe_dbg("mipi_lane_num: %u", sensor_info->mipi_lane_num);

	for (i = 0; i < camkit_array_size(sensor_info->i2c_addr_table); i++)
		probe_dbg("i2c_addr_table[%d]: %u", i, sensor_info->i2c_addr_table[i]);

	probe_dbg("i2c_speed: %u", sensor_info->i2c_speed);
	probe_dbg("addr_type: %u", sensor_info->addr_type);
	probe_dbg("pdaf_support: %u", sensor_info->pdaf_support);

	probe_dbg("pdaf_support_by_scenario: %u",
		sensor_info->pdaf_support_by_scenario);

	probe_dbg("need_high_impedance: %u", sensor_info->need_high_impedance);
	probe_dbg("probe_flag: %u", sensor_info->probe_flag);

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++)
		probe_dbg("binning ratio[%d]: %u", i, sensor_info->binning_ratio[i]);

	return ERR_NONE;
}

static uint32 camkit_translate_sensor_ctrl(
	struct camkit_sensor_ctrl_t *sensor_ctrl)
{
	if (sensor_ctrl == NULL) {
		log_err("[camkit][%s] invalid input arg", __func__);
		return ERR_INVAL;
	}

	// needn't translate, only dump info now
	probe_dbg("mirror: %u", sensor_ctrl->mirror);
	probe_dbg("sensor_mode: %u", sensor_ctrl->sensor_mode);
	probe_dbg("shutter: %u", sensor_ctrl->shutter);
	probe_dbg("gain: %u", sensor_ctrl->gain);
	probe_dbg("pclk: %u", sensor_ctrl->pclk);
	probe_dbg("frame_length: %u", sensor_ctrl->frame_length);
	probe_dbg("line_length: %u", sensor_ctrl->line_length);
	probe_dbg("min_frame_length: %u", sensor_ctrl->min_frame_length);
	probe_dbg("dummy_pixel: %u", sensor_ctrl->dummy_pixel);
	probe_dbg("dummy_line: %u", sensor_ctrl->dummy_line);
	probe_dbg("current_fps: %u", sensor_ctrl->current_fps);
	probe_dbg("autoflicker_en: %u", sensor_ctrl->autoflicker_en);
	probe_dbg("test_pattern: %u", sensor_ctrl->test_pattern);
	probe_dbg("current_scenario_id: %d", sensor_ctrl->current_scenario_id);
	probe_dbg("ihdr_en: %u", sensor_ctrl->ihdr_en);
	probe_dbg("i2c_write_id: %u", sensor_ctrl->i2c_write_id);
	probe_dbg("i2c_speed: %u", sensor_ctrl->i2c_speed);
	probe_dbg("addr_type: %d", sensor_ctrl->addr_type);

	return ERR_NONE;
}

static uint32 camkit_translate_sensor_output_info(
	struct camkit_sensor_output_info_t *output_info)
{
	int i;

	if (output_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	// needn't translate, only dump info now
	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		probe_dbg("out[%d].full_w: %u", i, output_info[i].full_w);
		probe_dbg("out[%d].full_h: %u", i, output_info[i].full_h);
		probe_dbg("out[%d].x0_offset: %u", i, output_info[i].x0_offset);
		probe_dbg("out[%d].y0_offset: %u", i, output_info[i].y0_offset);
		probe_dbg("out[%d].w0_size: %u", i, output_info[i].w0_size);
		probe_dbg("out[%d].h0_size: %u", i, output_info[i].h0_size);
		probe_dbg("out[%d].scale_w: %u", i, output_info[i].scale_w);
		probe_dbg("out[%d].scale_h: %u", i, output_info[i].scale_h);
		probe_dbg("out[%d].x1_offset: %u", i, output_info[i].x1_offset);
		probe_dbg("out[%d].y1_offset: %u", i, output_info[i].y1_offset);
		probe_dbg("out[%d].w1_size: %u", i, output_info[i].w1_size);
		probe_dbg("out[%d].h1_size: %u", i, output_info[i].h1_size);
		probe_dbg("out[%d].x2_tg_offset: %u", i, output_info[i].x2_tg_offset);
		probe_dbg("out[%d].y2_tg_offset: %u", i, output_info[i].y2_tg_offset);
		probe_dbg("out[%d].w2_tg_size: %u", i, output_info[i].w2_tg_size);
		probe_dbg("out[%d].h2_tg_size: %u", i, output_info[i].h2_tg_size);
	}

	return ERR_NONE;
}

static uint32 camkit_translate_pdaf_info(
	struct camkit_sensor_pdaf_info_t *pdaf_info)
{
	int i;

	if (pdaf_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("pdaf_info.offset_x: %u", pdaf_info->offset_x);
	probe_dbg("pdaf_info.offset_y: %u", pdaf_info->offset_y);
	probe_dbg("pdaf_info.pitch_x: %u", pdaf_info->pitch_x);
	probe_dbg("pdaf_info.pitch_y: %u", pdaf_info->pitch_y);
	probe_dbg("pdaf_info.pair_num: %u", pdaf_info->pair_num);
	probe_dbg("pdaf_info.sub_blk_w: %u", pdaf_info->sub_blk_w);
	probe_dbg("pdaf_info.sub_blk_h: %u", pdaf_info->sub_blk_h);

	for (i = 0; i < camkit_array_size(pdaf_info->pos_l); i++) {
		probe_dbg("pd pos_l[%d][0]: %u", i, pdaf_info->pos_l[i][0]);
		probe_dbg("pd pos_l[%d][1]: %u", i, pdaf_info->pos_l[i][1]);
	}
	for (i = 0; i < camkit_array_size(pdaf_info->pos_r); i++) {
		probe_dbg("pd pos_r[%d][0]: %u", i, pdaf_info->pos_r[i][0]);
		probe_dbg("pd pos_r[%d][1]: %u", i, pdaf_info->pos_r[i][1]);
	}

	probe_dbg("pdaf_info.mirror_flip: %u", pdaf_info->mirror_flip);
	probe_dbg("pdaf_info.block_num_x: %u", pdaf_info->block_num_x);
	probe_dbg("pdaf_info.block_num_y: %u", pdaf_info->block_num_y);
	probe_dbg("pdaf_info.lexpo_first: %u", pdaf_info->lexpo_first);

	for (i = 0; i < camkit_array_size(pdaf_info->crop); i++) {
		probe_dbg("pd crop[%d][0]: %u", i, pdaf_info->crop[i][0]);
		probe_dbg("pd crop[%d][1]: %u", i, pdaf_info->crop[i][1]);
	}

	return ERR_NONE;
}

static uint32 camkit_translate_power_info(
	struct camkit_hw_power_info_t *power_info)
{
	uint32 ret = ERR_NONE;

	if (power_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("power_info.pin_type: %d", power_info->pin_type);
	probe_dbg("power_info.pin_val: %d", power_info->pin_val);
	probe_dbg("power_info.delay: %u", power_info->pin_delay);

	return ret;
}

static uint32 camkit_translate_sensor_priv_gain_info(
	struct private_again_info *again_info)
{
	int32 len;
	int32 i;
	struct priv_again_map *again_map = NULL;

	if (again_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("sensor again info size: %u gain map size: %u",
		again_info->size, (uint32)(sizeof(struct priv_again_map)));

	len = again_info->size;
	if (len <= 0) {
		log_err("the gain map not configure");
		again_info->again_map = NULL;
		return ERR_NONE;
	}

	again_map = kzalloc(sizeof(struct priv_again_map) * len, GFP_KERNEL);
	if (!again_map) {
		again_info->again_map = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(again_map,
		(void *)again_info->again_map,
		sizeof(struct priv_again_map) * len)) {
		log_err("failed: copy_from_user");
		kfree(again_map);
		again_info->again_map = NULL;
		return ERR_INVAL;
	}

	again_info->again_map = again_map;
	for (i = 0; i < len; i++) {
		probe_dbg("again_map[%d].again: %u", i, again_map[i].again_val);
		probe_dbg("again_map[%d].size: %u", i, again_map[i].size);
	}

	return ERR_NONE;
}

static uint32 camkit_translate_sensor_aec_map(
	struct aec_ops_map *aec_map)
{
	int32 len;
	int32 i;
	int32 j;
	struct camkit_aec_op *aec_ops = NULL;

	if (aec_map == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("sensor aec map size: %u", aec_map->size);

	len = aec_map->size;
	if (len <= 0) {
		log_err("the aec map need not configure");
		aec_map->aec_ops = NULL;
		return ERR_NONE;
	}
	aec_ops = kzalloc(sizeof(struct camkit_aec_op) * len, GFP_KERNEL);
	if (!aec_ops) {
		aec_map->aec_ops = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(aec_ops,
		(void *)aec_map->aec_ops,
		sizeof(struct camkit_aec_op) * len)) {
		log_err("failed: copy_from_user");
		kfree(aec_ops);
		aec_map->aec_ops = NULL;
		return ERR_INVAL;
	}

	aec_map->aec_ops = aec_ops;
	for (i = 0; i < len; i++) {
		probe_dbg("aec_ops[%d].op_type: %d", i, aec_ops[i].op_type);
		probe_dbg("aec_ops[%d].size: %d", i, aec_ops[i].size);
		if (aec_ops[i].size > 0 && aec_ops[i].size < MAX_AEC_REGS) {
			for (j = 0; j < aec_ops[i].size; j++) {
				probe_dbg("aec_ops[%d].i2c_setting[%d].addr: 0x%x",
					i, j, aec_ops[i].i2c_setting[j].addr);
				probe_dbg("aec_ops[%d].i2c_setting[%d].val: 0x%x",
					i, j, aec_ops[i].i2c_setting[j].val);
				probe_dbg("aec_ops[%d].i2c_setting[%d].len: %d",
					i, j, aec_ops[i].i2c_setting[j].len);
				probe_dbg("aec_ops[%d].i2c_setting[%d].mask: 0x%x",
					i, j, aec_ops[i].i2c_setting[j].mask);
				probe_dbg("aec_ops[%d].i2c_setting[%d].mode: %d",
					i, j, aec_ops[i].i2c_setting[j].mode);
				probe_dbg("aec_ops[%d].i2c_setting[%d].addr_type: %d",
					i, j, aec_ops[i].i2c_setting[j].addr_type);
				probe_dbg("aec_ops[%d].i2c_setting[%d].data_type: %d",
					i, j, aec_ops[i].i2c_setting[j].data_type);

				probe_dbg("aec_ops[%d].aec_setting[%d].addr: 0x%x",
					i, j, aec_ops[i].aec_setting[j].addr);
				probe_dbg("aec_ops[%d].aec_setting[%d].max_val: 0x%x",
					i, j, aec_ops[i].aec_setting[j].max_val);
				probe_dbg("aec_ops[%d].aec_setting[%d].shift: %d",
					i, j, aec_ops[i].aec_setting[j].shift);
				probe_dbg("aec_ops[%d].aec_setting[%d].mask: 0x%x",
					i, j, aec_ops[i].aec_setting[j].mask);
				probe_dbg("aec_ops[%d].aec_setting[%d].addr_type: %d",
					i, j, aec_ops[i].aec_setting[j].addr_type);
				probe_dbg("aec_ops[%d].aec_setting[%d].data_type: %d",
					i, j, aec_ops[i].aec_setting[j].data_type);
			}
		}
	}

	return ERR_NONE;
}

static uint32 camkit_translate_aec_info(
	struct camkit_aec_info_t *aec_info)
{
	uint32 ret = ERR_NONE;

	if (aec_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("aec_info.min_again: %u", aec_info->min_again);
	probe_dbg("aec_info.max_again: %u", aec_info->max_again);
	probe_dbg("aec_info.min_dgain: %u", aec_info->min_dgain);
	probe_dbg("aec_info.max_dgain: %u", aec_info->max_dgain);
	probe_dbg("aec_info.reg_gain_1x: %u", aec_info->reg_gain_1x);
	probe_dbg("aec_info.dgain_decimator: %u", aec_info->dgain_decimator);
	probe_dbg("aec_info.min_linecount: %u", aec_info->min_linecount);
	probe_dbg("aec_info.max_linecount: %u", aec_info->max_linecount);
	probe_dbg("aec_info.vts_offset: %u", aec_info->vts_offset);
	probe_dbg("aec_info.again_type: %u", aec_info->again_type);
	probe_dbg("aec_info.smia_coeff.m0: %u", aec_info->smia_coeff.m0);
	probe_dbg("aec_info.smia_coeff.m1: %u", aec_info->smia_coeff.m1);
	probe_dbg("aec_info.smia_coeff.m2: %u", aec_info->smia_coeff.m2);
	probe_dbg("aec_info.smia_coeff.c0: %u", aec_info->smia_coeff.c0);
	probe_dbg("aec_info.smia_coeff.c1: %u", aec_info->smia_coeff.c1);
	probe_dbg("aec_info.smia_coeff.c2: %u", aec_info->smia_coeff.c2);

	ret |= camkit_translate_sensor_priv_gain_info(&aec_info->priv_again_info);
	ret |= camkit_translate_sensor_aec_map(&aec_info->gain_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->expo_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->vts_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->normal2long_ops_map);
	ret |= camkit_translate_sensor_aec_map(&aec_info->long2normal_ops_map);

	return ret;
}

static uint32 camkit_translate_vc_info(
	struct camkit_sensor_vc_info_t *vc_info)
{
	uint32 i;

	if (vc_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	for (i = 0; i < MAX_OUTPUT_INFO_SIZE; i++) {
		probe_dbg("vc[%d].vc_num: %u", i, vc_info[i].vc_num);
		probe_dbg("vc[%d].vc_pixel_num: %u", i, vc_info[i].vc_pixel_num);
		probe_dbg("vc[%d].mode_select: %u", i, vc_info[i].mode_select);
		probe_dbg("vc[%d].expo_ratio: %u", i, vc_info[i].expo_ratio);
		probe_dbg("vc[%d].od_value: %u", i, vc_info[i].od_value);
		probe_dbg("vc[%d].rg_stats_mode: %u", i, vc_info[i].rg_stats_mode);

		probe_dbg("vc[%d].vc0_id: %u", i, vc_info[i].vc0_id);
		probe_dbg("vc[%d].vc0_data_type: %u", i, vc_info[i].vc0_data_type);
		probe_dbg("vc[%d].vc0_sizeh: %u", i, vc_info[i].vc0_sizeh);
		probe_dbg("vc[%d].vc0_sizev: %u", i, vc_info[i].vc0_sizev);
		probe_dbg("vc[%d].vc1_id: %u", i, vc_info[i].vc1_id);
		probe_dbg("vc[%d].vc1_data_type: %u", i, vc_info[i].vc1_data_type);
		probe_dbg("vc[%d].vc1_sizeh: %u", i, vc_info[i].vc1_sizeh);
		probe_dbg("vc[%d].vc1_sizev: %u", i, vc_info[i].vc1_sizev);
		probe_dbg("vc[%d].vc2_id: %u", i, vc_info[i].vc2_id);
		probe_dbg("vc[%d].vc2_data_type: %u", i, vc_info[i].vc2_data_type);
		probe_dbg("vc[%d].vc2_sizeh: %u", i, vc_info[i].vc2_sizeh);
		probe_dbg("vc[%d].vc2_sizev: %u", i, vc_info[i].vc2_sizev);
		probe_dbg("vc[%d].vc3_id: %u", i, vc_info[i].vc3_id);
		probe_dbg("vc[%d].vc3_data_type: %u", i, vc_info[i].vc3_data_type);
		probe_dbg("vc[%d].vc3_sizeh: %u", i, vc_info[i].vc3_sizeh);
		probe_dbg("vc[%d].vc3_sizev: %u", i, vc_info[i].vc3_sizev);
		probe_dbg("vc[%d].vc4_id: %u", i, vc_info[i].vc4_id);
		probe_dbg("vc[%d].vc4_data_type: %u", i, vc_info[i].vc4_data_type);
		probe_dbg("vc[%d].vc4_sizeh: %u", i, vc_info[i].vc4_sizeh);
		probe_dbg("vc[%d].vc4_sizev: %u", i, vc_info[i].vc4_sizev);
		probe_dbg("vc[%d].vc5_id: %u", i, vc_info[i].vc5_id);
		probe_dbg("vc[%d].vc5_data_type: %u", i, vc_info[i].vc5_data_type);
		probe_dbg("vc[%d].vc5_sizeh: %u", i, vc_info[i].vc5_sizeh);
		probe_dbg("vc[%d].vc5_sizev: %u", i, vc_info[i].vc5_sizev);
	}

	return ERR_NONE;
}

static uint32 camkit_translate_correction_info(
	struct camkit_sensor_correction_t *correction_info)
{
	uint32 ret = ERR_NONE;

	if (correction_info == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	probe_dbg("qsc_apply: %u", correction_info->qsc_apply);
	probe_dbg("eeprom_qsc_addr: %u", correction_info->eeprom_qsc_addr);
	probe_dbg("sensor_qsc_addr: %u", correction_info->sensor_qsc_addr);
	probe_dbg("qsc_len: %u", correction_info->qsc_len);
	probe_dbg("spc_apply: %u", correction_info->spc_apply);
	probe_dbg("eeprom_pdaf_addr: %u", correction_info->eeprom_pdaf_addr);
	probe_dbg("pdaf_len: %u", correction_info->pdaf_len);
	probe_dbg("lsc_start_addr: %u", correction_info->lsc_start_addr);
	probe_dbg("lsc_addr_len: %u", correction_info->lsc_addr_len);
	probe_dbg("rsc_start_addr: %u", correction_info->rsc_start_addr);
	probe_dbg("rsc_addr_len: %u", correction_info->rsc_addr_len);

	return ret;
}

static uint32 camkit_translate_sensor_params(
	struct camkit_params *kit_params)
{
	struct camkit_sensor_params *sensor_params = NULL;
	uint32 ret = ERR_NONE;
	int i;

	if (kit_params == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	sensor_params = kzalloc(sizeof(struct camkit_sensor_params), GFP_KERNEL);
	if (!sensor_params) {
		kit_params->sensor_params = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(sensor_params,
		(void *)kit_params->sensor_params,
		sizeof(struct camkit_sensor_params))) {
		log_err("failed: copy camkit parameters from user");
		kfree(sensor_params);
		kit_params->sensor_params = NULL;
		return ERR_INVAL;
	}
	kit_params->sensor_params = sensor_params;

	ret |= camkit_translate_sensor_info(&sensor_params->sensor_info);
	ret |= camkit_translate_sensor_ctrl(&sensor_params->sensor_ctrl);
	ret |= camkit_translate_sensor_output_info(sensor_params->output_info);
	ret |= camkit_translate_pdaf_info(&sensor_params->pdaf_info);
	for (i = 0; i < camkit_array_size(sensor_params->power_info); i++) {
		ret |= camkit_translate_power_info(
			&(sensor_params->power_info[i]));
		ret |= camkit_translate_power_info(
			&(sensor_params->power_down_info[i]));
	}
	ret |= camkit_translate_aec_info(&sensor_params->aec_info);
	ret |= camkit_translate_vc_info(sensor_params->vc_info);
	ret |= camkit_translate_correction_info(&sensor_params->correction_info);

	return ret;
}

static uint32 camkit_translate_module_params(
	struct camkit_params *kit_params)
{
	struct camkit_module_params *module_params = NULL;

	if (kit_params == NULL) {
		log_err("invalid input arg");
		return ERR_INVAL;
	}

	module_params = kzalloc(sizeof(struct camkit_module_params), GFP_KERNEL);
	if (!module_params) {
		kit_params->module_params = NULL;
		return ERR_NOMEM;
	}

	if (copy_from_user(module_params,
		(void *)kit_params->module_params,
		sizeof(struct camkit_module_params))) {
		log_err("failed: copy camkit parameters from user");
		kfree(module_params);
		kit_params->module_params = NULL;
		return ERR_INVAL;
	}
	kit_params->module_params = module_params;

	probe_dbg("skip_module_id: %u", module_params->skip_module_id);
	probe_dbg("eeprom_i2c_addr: %u", module_params->eeprom_i2c_addr);
	probe_dbg("module_code_addr: %u", module_params->module_code_addr);
	probe_dbg("module_code: %u", module_params->module_code);
	probe_dbg("lens_type_addr: %u", module_params->lens_type_addr);
	probe_dbg("lens_type: %u", module_params->lens_type);
	probe_dbg("addr_type: %u", module_params->addr_type);
	probe_dbg("data_type: %u", module_params->data_type);
	probe_dbg("sensor_name: %s", module_params->sensor_name);

	return ERR_NONE;
}

static uint32 camkit_fill_sensor_params(struct camkit_sensor *sensor,
	struct camkit_params *user_kit_params)
{
	struct camkit_params *kit_params = NULL;
	uint32 ret = ERR_NONE;

	return_err_if_null(sensor);
	return_err_if_null(user_kit_params);

	kit_params = kzalloc(sizeof(struct camkit_params), GFP_KERNEL);
	if (!kit_params)
		return ERR_NOMEM;

	if (copy_from_user(kit_params, (void *)user_kit_params,
		sizeof(struct camkit_params))) {
		log_err("failed: copy camkit parameters from user");
		kfree(kit_params);
		return ERR_INVAL;
	}
	sensor->kit_params = kit_params;

	ret |= camkit_translate_sensor_params(kit_params);
	ret |= camkit_translate_module_params(kit_params);

	return ret;
}

static void camkit_free_i2c_setting(struct camkit_i2c_reg_setting *i2c_setting)
{
	if (i2c_setting->setting != NULL) {
		kfree(i2c_setting->setting);
		i2c_setting->setting = NULL;
	}
}

static void camkit_free_i2c_table(struct camkit_i2c_reg_table_array *i2c_table)
{
	if (i2c_table->setting != NULL) {
		kfree(i2c_table->setting);
		i2c_table->setting = NULL;
	}
}

static void camkit_free_aec_map(struct aec_ops_map *aec_map)
{
	if (aec_map->aec_ops != NULL) {
		kfree(aec_map->aec_ops);
		aec_map->aec_ops = NULL;
	}
}

static int camkit_free_sensor_params(struct camkit_params *kit_params)
{
	struct camkit_sensor_info_t *sensor_info = NULL;
	struct camkit_aec_info_t *aec_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;

	if (kit_params != NULL && kit_params->sensor_params != NULL) {
		sensor_params = kit_params->sensor_params;
		sensor_info = &(sensor_params->sensor_info);

		camkit_free_i2c_setting(&(sensor_info->id_init_setting));
		camkit_free_i2c_setting(&(sensor_info->init_setting));
		camkit_free_i2c_setting(&(sensor_info->init_burst_setting));
		camkit_free_i2c_setting(&(sensor_info->pre_setting));
		camkit_free_i2c_setting(&(sensor_info->cap_setting));
		camkit_free_i2c_setting(&(sensor_info->cap1_setting));
		camkit_free_i2c_setting(&(sensor_info->normal_video_setting));
		camkit_free_i2c_setting(&(sensor_info->hs_video_setting));
		camkit_free_i2c_setting(&(sensor_info->slim_setting));
		camkit_free_i2c_setting(&(sensor_info->custom1_setting));
		camkit_free_i2c_setting(&(sensor_info->custom2_setting));
		camkit_free_i2c_setting(&(sensor_info->custom3_setting));
		camkit_free_i2c_setting(&(sensor_info->custom4_setting));
		camkit_free_i2c_setting(&(sensor_info->custom5_setting));
		camkit_free_i2c_setting(&(sensor_info->streamon_setting));
		camkit_free_i2c_setting(&(sensor_info->streamoff_setting));
		camkit_free_i2c_setting(&(sensor_info->test_pattern_on_setting));
		camkit_free_i2c_setting(&(sensor_info->test_pattern_off_setting));
		camkit_free_i2c_setting(&(sensor_info->normal_to_long_ready_settings));
		camkit_free_i2c_setting(&(sensor_info->normal_to_long_end_settings));
		camkit_free_i2c_table(&(sensor_info->dump_info));
		camkit_free_i2c_table(&(sensor_info->fuse_id_info));

		aec_info = &(sensor_params->aec_info);
		camkit_free_aec_map(&(aec_info->gain_ops_map));
		camkit_free_aec_map(&(aec_info->expo_ops_map));
		camkit_free_aec_map(&(aec_info->vts_ops_map));

		if (aec_info->priv_again_info.again_map != NULL)
			kfree(aec_info->priv_again_info.again_map);
		if (aec_info->normal2long_ops_map.aec_ops != NULL)
			kfree(aec_info->normal2long_ops_map.aec_ops);
		if (aec_info->long2normal_ops_map.aec_ops != NULL)
			kfree(aec_info->long2normal_ops_map.aec_ops);

		kfree(sensor_params);
		sensor_params = NULL;
	}

	if (kit_params != NULL && kit_params->module_params != NULL) {
		kfree(kit_params->module_params);
		kit_params->module_params = NULL;
	}

	if (kit_params != NULL) {
		kfree(kit_params);
		kit_params = NULL;
	}

	return ERR_NONE;
}

static uint32 camkit_get_customized_ops(const uint32 sensor_idx,
	const uint32 sensor_id, struct sensor_kit_ops **ops)
{
	struct customized_driver *customized_drv = &__start_customized_drv_tag;

	do {
		probe_dbg("sensor index: %u, id: 0x%x", sensor_idx, sensor_id);
		if (sensor_idx == customized_drv->sensor_idx &&
			sensor_id == customized_drv->sensor_id) {
			log_info("customized driver found. sensor id = 0x%x", sensor_id);
			return customized_drv->get_ops(ops);
		}
		++customized_drv;
	} while (customized_drv < &__stop_customized_drv_tag);

	return ERR_INVAL;
}

static int32 hwsensor_get_sensor_ops(struct camkit_sensor *sensor)
{
	struct camkit_sensor_params *params = NULL;
	struct camkit_module_params *module_params = NULL;
	uint32 ret;
	uint32 sensor_idx;
	uint32 sensor_id;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(sensor->kit_params->module_params);

	params = sensor->kit_params->sensor_params;
	module_params = sensor->kit_params->module_params;

	sensor_idx = sensor->sensor_idx;
	sensor_id = module_params->match_id;

	log_info("sensor_name:%s", params->sensor_name);
	if (sensor->kit_params->customized_enable) {
		ret = camkit_get_customized_ops(sensor_idx,
			sensor_id, &(sensor->sensor_ops));
		if (ret != ERR_NONE || sensor->sensor_ops == NULL) {
			log_err("get sensor operators fail");
			return ERR_INVAL;
		}
	} else {
		ret = camkit_get_normalized_ops(&(sensor->sensor_ops));
		if (ret != ERR_NONE || sensor->sensor_ops == NULL) {
			log_err("get sensor operators fail");
			return ERR_INVAL;
		}
	}

	log_info("camera[%d]:%s use %s ops", sensor_idx, params->sensor_name,
		sensor->kit_params->customized_enable ? "customized" : "camkit");

	return ERR_NONE;
}

int32 hwsensor_probe_sensor(const uint32 sensor_idx,
	struct camkit_params *user_kit_params)
{
	uint32 ret;
	uint32 rc;
	struct camkit_sensor *kit_sensor = NULL;
	uint32 sensor_id = 0;

	log_info("begin probe sensor[%d]", sensor_idx);
	kit_sensor = get_camkit_sensor(sensor_idx);
	if (kit_sensor == NULL) {
		log_err("invalid sensor index");
		return ERR_INVAL;
	}

	// 1. mark sensor index
	kit_sensor->sensor_idx = sensor_idx;

	// 2. fill sensor camkit parameters with user parameters
	ret = camkit_fill_sensor_params(kit_sensor, user_kit_params);
	if (ret != ERR_NONE) {
		camkit_free_sensor_params(kit_sensor->kit_params);
		kit_sensor->kit_params = NULL;
		log_err("fill sensor parameters failed");
		return ERR_INVAL;
	}

	// 3. fill sensor operators by the flag: customized_enable
	ret = hwsensor_get_sensor_ops(kit_sensor);
	if (ret != ERR_NONE) {
		camkit_free_sensor_params(kit_sensor->kit_params);
		kit_sensor->kit_params = NULL;
		log_err("fill sensor operators failed");
		return ERR_INVAL;
	}

	// 4. power on sensor
	ret = hwsensor_power(kit_sensor, CAMKIT_HW_POWER_STATUS_ON);
	if (ret != ERR_NONE) {
		camkit_free_sensor_params(kit_sensor->kit_params);
		kit_sensor->kit_params = NULL;
		log_err("power on sensor failed");
		return ERR_INVAL;
	}

	// 5. match id
	ret = hwsensor_match_id(sensor_idx, &sensor_id);
	if (sensor_id == 0 || sensor_id == 0xFFFFFFFF) {
		log_err("Fail to match sensor ID %x", sensor_id);
		rc = EIO;
	} else {
		log_info("Sensor found ID = 0x%x", sensor_id);
#if (DRIVER_VERSION == 1)
		(void)eeprom_read_data(sensor_id);
#endif
		rc = ERR_NONE;
	}

	// 6. power down sensor
	(void)hwsensor_power(kit_sensor, CAMKIT_HW_POWER_STATUS_OFF);

	if (rc != ERR_NONE) {
		log_err("sensor[%d]: %s mismatch", sensor_idx,
			kit_sensor->kit_params->module_params->sensor_name);
		camkit_free_sensor_params(kit_sensor->kit_params);
		kit_sensor->kit_params = NULL;
		return ERR_INVAL;
	}

	kit_sensor->probe_success = 1;
	if (strncpy_s(kit_sensor->sensor_name, SENSOR_NAME_LEN,
		kit_sensor->kit_params->module_params->sensor_name,
		SENSOR_NAME_LEN - 1) != EOK)
		log_err("strcpy sensor name fail");

	log_info("probe sensor[%d]: %s successfully", sensor_idx,
		kit_sensor->sensor_name);

	return ERR_NONE;
}
