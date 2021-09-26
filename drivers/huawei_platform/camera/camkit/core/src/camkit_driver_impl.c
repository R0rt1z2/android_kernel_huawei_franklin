/*
 * camkit_driver.c
 *
 * Copyright (c) huawei technologies co., ltd. 2020-2020. all rights reserved.
 *
 * Description: camkit interface
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

#include "camkit_driver_impl.h"

#include <securec.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "camkit_sensor_i2c.h"

#define RETRY_TIMES 2
#define PIX_10_BITS 10
#define PDAF_FEATURE_PARA_LEN 2

DEFINE_SPINLOCK(camkit_lock);

static int32 read_data_from_eeprom(uint16 i2c_addr,
	uint16 addr, uint16 len, uint8 *data)
{
	uint16 i;
	uint16 offset = addr;
	int32 ret;

	for (i = 0; i < len; i++) {
		ret = camkit_read_eeprom(i2c_addr, offset, &data[i]);
		if (ret != ERR_NONE) {
			log_err("read fail, addr: 0x%x", offset);
			return ERR_IO;
		}

		offset++;
	}

	return ERR_NONE;
}

static int32 read_qsc_calib_data_from_eeprom(
	struct camkit_module_params *module_params,
	struct camkit_sensor_params *sensor_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	int32 ret;

	correction_info = &(sensor_params->correction_info);
	if (!correction_info->qsc_apply || !correction_info->eeprom_qsc_addr ||
		!correction_info->qsc_len) {
		log_info("need not read qsc data from eeprom");
		return ERR_NONE;
	}
	if (correction_info->qsc_read_flag) {
		log_info("qsc data had read from eeprom");
		return ERR_NONE;
	}

	correction_info->qsc_buf = kzalloc(correction_info->qsc_len, GFP_KERNEL);
	if (!correction_info->qsc_buf)
		return ERR_NOMEM;

	ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
		correction_info->eeprom_qsc_addr, correction_info->qsc_len,
		correction_info->qsc_buf);
	if (ret != ERR_NONE) {
		log_err("read qsc data from eeprom fail");
		kfree(correction_info->qsc_buf);
		correction_info->qsc_buf = NULL;
		return ERR_IO;
	}
	correction_info->qsc_read_flag = 1;

	return ERR_NONE;
}

static uint16 pdc_addr[][2] = {
	// { start_addr, end_addr }
	{ 0x00C, 0x01C }, { 0x03F, 0x04F }, { 0x072, 0x082 },
	{ 0x0A5, 0x0B5 }, { 0x0D8, 0x0E8 }, { 0x0E9, 0x0F9 },
	{ 0x11C, 0x12C }, { 0x14F, 0x15F }, { 0x182, 0x192 },
	{ 0x1B5, 0x1C5 }, { 0x1C6, 0x1D6 }, { 0x1F9, 0x209 },
	{ 0x22C, 0x23C }, { 0x25F, 0x26F }, { 0x292, 0x2A2 },
	{ 0x2A3, 0x2B3 }, { 0x2D6, 0x2E6 }, { 0x309, 0x319 },
	{ 0x33C, 0x34C }, { 0x36F, 0x37F }, { 0x380, 0x59B },
};

static void extract_valid_data_for_ov48b(uint8 *buf)
{
	uint32 index = 0;
	uint8 *pdc_buf = buf;
	uint32 size = camkit_array_size(pdc_addr);
	uint32 i;
	uint32 j;

	for (i = 0; i < size - 1; i++) {
		// extract a piece of data per 2 pdc addr
		for (j = pdc_addr[i][1]; j >= pdc_addr[i][0]; j = j - 2)
			pdc_buf[index++] = buf[j];
	}

	for (i = pdc_addr[size - 1][0]; i <= pdc_addr[size - 1][1]; i++)
		pdc_buf[index++] = buf[i];
}

static int32 read_spc_calib_data_from_eeprom(
	struct camkit_module_params *module_params,
	struct camkit_sensor_params *sensor_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	int32 ret;

	correction_info = &(sensor_params->correction_info);
	if (!correction_info->spc_apply || !correction_info->eeprom_pdaf_addr ||
		!correction_info->pdaf_len) {
		log_info("need not read spc data from eeprom");
		return ERR_NONE;
	}
	if (correction_info->spc_read_flag) {
		log_info("spc data had read from eeprom");
		return ERR_NONE;
	}

	correction_info->pdaf_buf = kzalloc(correction_info->pdaf_len, GFP_KERNEL);
	if (!correction_info->pdaf_buf)
		return ERR_NOMEM;

	ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
		correction_info->eeprom_pdaf_addr, correction_info->pdaf_len,
		correction_info->pdaf_buf);
	if (ret != ERR_NONE) {
		log_err("read spc data from eeprom fail");
		kfree(correction_info->pdaf_buf);
		correction_info->pdaf_buf = NULL;
		return ERR_IO;
	}
	correction_info->spc_read_flag = 1;

	if (correction_info->spc_type == PDAF_SPC_PDC)
		extract_valid_data_for_ov48b(correction_info->pdaf_buf);

	return ERR_NONE;
}

static void read_calib_data_from_eeprom(struct camkit_params *kit_params)
{
	(void)read_qsc_calib_data_from_eeprom(kit_params->module_params,
		kit_params->sensor_params);
	(void)read_spc_calib_data_from_eeprom(kit_params->module_params,
		kit_params->sensor_params);
}

static int32 write_lrc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	drv_dbg("ENTER");

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len != (correction_info->lsc_addr_len +
		correction_info->rsc_addr_len)) {
		log_err("pdaf correction info configure error, please check");
		return ERR_INVAL;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->lsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->lsc_addr_len);
	if (ret != ERR_NONE) {
		log_err("write lsc fail");
		return ERR_IO;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->rsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf + correction_info->lsc_addr_len,
		correction_info->rsc_addr_len);
	if (ret != ERR_NONE) {
		log_err("write rsc fail");
		return ERR_IO;
	}

	drv_dbg("EXIT");

	return ERR_NONE;
}

static int32 write_pdc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	drv_dbg("ENTER");

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len < correction_info->pdc_len) {
		log_err("pdaf correction info configure error, please check");
		return ERR_INVAL;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->pdc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->pdc_len);
	if (ret != ERR_NONE) {
		log_err("write lsc fail");
		return ERR_IO;
	}

	drv_dbg("EXIT");

	return ERR_NONE;
}

static int32 write_qsc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_module_params *module_params = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	drv_dbg("ENTER");

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	return_err_if_null(kit_params->module_params);
	sensor_params = kit_params->sensor_params;
	module_params = kit_params->module_params;

	correction_info = &(sensor_params->correction_info);

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->sensor_qsc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->qsc_buf, correction_info->qsc_len);
	if (ret != ERR_NONE) {
		log_err("write lsc fail");
		return ERR_IO;
	}

	drv_dbg("EXIT");

	return ERR_NONE;
}

static struct camkit_mode_info *camkit_get_mode_info(
	struct camkit_sensor_params *params, const uint32 scenario_id)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (scenario_id) {
	case CAMKIT_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case CAMKIT_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case CAMKIT_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case CAMKIT_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case CAMKIT_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	return mode_info;
}

static struct camkit_i2c_reg_setting *camkit_get_setting_by_scenario(
	struct camkit_sensor_params *params, const uint32 scenario_id)
{
	struct camkit_i2c_reg_setting *setting = NULL;

	switch (scenario_id) {
	case CAMKIT_SCENARIO_ID_CAMERA_PREVIEW:
		setting = &(params->sensor_info.pre_setting);
		break;
	case CAMKIT_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		setting = &(params->sensor_info.cap_setting);
		break;
	case CAMKIT_SCENARIO_ID_VIDEO_PREVIEW:
		setting = &(params->sensor_info.normal_video_setting);
		break;
	case CAMKIT_SCENARIO_ID_HIGH_SPEED_VIDEO:
		setting = &(params->sensor_info.hs_video_setting);
		break;
	case CAMKIT_SCENARIO_ID_SLIM_VIDEO:
		setting = &(params->sensor_info.slim_setting);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM1:
		setting = &(params->sensor_info.custom1_setting);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM2:
		setting = &(params->sensor_info.custom2_setting);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM3:
		setting = &(params->sensor_info.custom3_setting);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM4:
		setting = &(params->sensor_info.custom4_setting);
		break;
	case CAMKIT_SCENARIO_ID_CUSTOM5:
		setting = &(params->sensor_info.custom5_setting);
		break;
	default:
		setting = &(params->sensor_info.pre_setting);
		break;
	}

	return setting;
}

uint32 camkit_sensor_init(struct camkit_sensor *sensor)
{
	int32 rc;
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	drv_dbg("ENTER");

	return_err_if_null(sensor);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	rc = camkit_sensor_write_setting(&params->sensor_ctrl,
		&params->sensor_info.init_setting);
	if (rc != ERR_NONE) {
		log_err("write initial setting failed");
		return ERR_IO;
	}

	if (params->sensor_info.init_burst_setting.setting != NULL &&
		params->sensor_info.init_burst_setting.size > 0) {
		rc = camkit_sensor_write_block(&params->sensor_ctrl,
			params->sensor_info.init_burst_setting.setting,
			params->sensor_info.init_burst_setting.size,
			params->sensor_info.init_burst_setting.data_type);
		if (rc != ERR_NONE)
			log_err("something abnormal");
	}

	log_info("qsc_apply:%d", params->correction_info.qsc_apply);
	if (params->correction_info.qsc_apply)
		(void)write_qsc_data_to_sensor(kit_params);

	drv_dbg("EXIT");

	return ERR_NONE;
}

uint32 camkit_set_test_pattern(struct camkit_sensor *sensor,
	const uint8 enable)
{
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.test_pattern = enable;
	spin_unlock(&camkit_lock);

	return ERR_NONE;
}

uint32 camkit_dump_reg(struct camkit_sensor *sensor)
{
	int32 rc;
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	drv_dbg("ENTER");

	return_err_if_null(sensor);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	rc = camkit_sensor_i2c_process(&params->sensor_ctrl,
		&params->sensor_info.dump_info);
	if (rc != ERR_NONE)
		log_err("Failed");

	drv_dbg("EXIT");

	return ERR_NONE;
}

static uint32 match_sensor_id(struct camkit_params *params)
{
	int32 rc;
	uint8 i = 0;
	uint8 retry = RETRY_TIMES;
	uint8 size;
	uint16 sensor_id = 0;
	struct camkit_sensor_params *sensor_params = params->sensor_params;
	struct camkit_module_params *module_params = params->module_params;
	struct camkit_sensor_info_t *sensor_info = &sensor_params->sensor_info;
	uint16 expect_id = sensor_info->sensor_id;
	enum camkit_i2c_data_type data_type = sensor_info->sensor_id_dt;
	struct camkit_i2c_reg_setting *setting = &(sensor_info->id_init_setting);

	spin_lock(&camkit_lock);
	/* init i2c config */
	sensor_params->sensor_ctrl.i2c_speed = sensor_info->i2c_speed;
	sensor_params->sensor_ctrl.addr_type = sensor_info->addr_type;
	spin_unlock(&camkit_lock);

	size = camkit_array_size(sensor_info->i2c_addr_table);
	log_info("sensor i2c addr num = %u", size);

	while ((i < size) && (sensor_info->i2c_addr_table[i] != 0xff)) {
		spin_lock(&camkit_lock);
		sensor_params->sensor_ctrl.i2c_write_id =
			sensor_info->i2c_addr_table[i];
		spin_unlock(&camkit_lock);
		do {
			log_info("to match sensor: %s", module_params->sensor_name);

			if (setting->setting && setting->size > 0)
				(void)camkit_sensor_write_setting(&sensor_params->sensor_ctrl,
					setting);

			if (!data_type)
				rc = camkit_sensor_i2c_read(&sensor_params->sensor_ctrl,
					sensor_info->sensor_id_reg,
					&sensor_id, CAMKIT_I2C_WORD_DATA);
			else
				rc = camkit_sensor_i2c_read(&sensor_params->sensor_ctrl,
					sensor_info->sensor_id_reg,
					&sensor_id, data_type);
			if (rc == ERR_NONE && sensor_id == expect_id) {
				log_info("sensor id: 0x%x matched", sensor_id);
				return ERR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}

	log_info("sensor id mismatch, expect:0x%x, real:0x%x",
		expect_id, sensor_id);

	return ERR_IO;
}

static uint32 match_module_id(struct camkit_module_params *params)
{
	int32 rc;
	uint16 module_code = 0;
	uint16 expect_code = params->module_code;
	uint8 i2c_addr = params->eeprom_i2c_addr;
	uint16 module_addr = params->module_code_addr;
	uint16 lens_addr = params->lens_type_addr;
	uint16 expect_lens = params->lens_type;
	uint16 lens_type = 0;
	uint8 retry = RETRY_TIMES;

	if (params->skip_module_id) {
		log_info("not to match module code");
		return ERR_NONE;
	}

	do {
		rc = camkit_i2c_read(i2c_addr, module_addr, params->addr_type,
			&module_code, params->data_type);
		if (rc == ERR_NONE && module_code == expect_code) {
			log_info("module code: 0x%x matched", module_code);
			if (params->lens_type_addr) {
				rc = camkit_i2c_read(i2c_addr, lens_addr, params->addr_type,
					&lens_type, params->data_type);
				if (rc == ERR_NONE && lens_type == expect_lens) {
					log_info("lens type: 0x%x matched", lens_type);
					return ERR_NONE;
				}
			} else {
				return ERR_NONE;
			}
		}
		retry--;
	} while (retry > 0);

	log_info("module code, expect:0x%x, real:0x%x",
		expect_code, module_code);
	log_info("lens type, expect:0x%x, real:0x%x",
		expect_lens, lens_type);

	return ERR_IO;
}

uint32 camkit_match_id(struct camkit_sensor *sensor,
	uint32 *match_id)
{
	uint32 rc;
	struct camkit_params *kit_params = NULL;

	return_err_if_null(sensor);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params);
	return_err_if_null(kit_params->sensor_params);
	return_err_if_null(kit_params->module_params);

	rc = match_sensor_id(kit_params);
	if (rc != ERR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	rc = match_module_id(kit_params->module_params);
	if (rc != ERR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	if (kit_params->sensor_params->sensor_info.need_correction)
		read_calib_data_from_eeprom(kit_params);

	*match_id = kit_params->module_params->match_id;
	log_info("match id ok, sensor id: 0x%x, module: 0x%x, match id: 0x%x",
		kit_params->sensor_params->sensor_info.sensor_id,
		kit_params->module_params->module_code,
		kit_params->module_params->match_id);

	return ERR_NONE;
}

uint32 camkit_open(struct camkit_sensor *sensor)
{
	uint32 sensor_id = 0;
	uint32 rc;
	struct camkit_params *params = NULL;
	struct camkit_sensor_params *sensor_params = NULL;

	log_info("ENTER");

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	params = sensor->kit_params;

	return_err_if_null(params->sensor_params);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->sensor_ops->match_id);
	return_err_if_null(sensor->sensor_ops->sensor_init);

	sensor_params = params->sensor_params;
	rc = sensor->sensor_ops->match_id(sensor, &sensor_id);
	if (rc != ERR_NONE) {
		log_err("probe sensor failed");
		return ERR_IO;
	}
	drv_dbg("sensor probe successfully. sensor_id = 0x%x", sensor_id);

	/* initail sequence write in  */
	rc = sensor->sensor_ops->sensor_init(sensor);
	if (rc != ERR_NONE) {
		log_err("sensor initial failed");
		return ERR_IO;
	}

	spin_lock(&camkit_lock);
	sensor_params->sensor_ctrl.autoflicker_en = FALSE;
	sensor_params->sensor_ctrl.sensor_mode = CAMKIT_MODE_INIT;
	sensor_params->sensor_ctrl.pclk =
		sensor_params->sensor_info.pre.pclk;
	sensor_params->sensor_ctrl.frame_length =
		sensor_params->sensor_info.pre.framelength;
	sensor_params->sensor_ctrl.line_length =
		sensor_params->sensor_info.pre.linelength;
	sensor_params->sensor_ctrl.min_frame_length =
		sensor_params->sensor_info.pre.framelength;
	sensor_params->sensor_ctrl.dummy_pixel = 0;
	sensor_params->sensor_ctrl.dummy_line = 0;
	sensor_params->sensor_ctrl.ihdr_en = FALSE;
	sensor_params->sensor_ctrl.test_pattern = FALSE;
	sensor_params->sensor_ctrl.current_fps =
		sensor_params->sensor_info.pre.max_framerate;
	spin_unlock(&camkit_lock);

	log_info("EXIT");

	return ERR_NONE;
}

uint32 camkit_close(struct camkit_sensor *sensor)
{
	log_info("Enter");
	/* No Need to implement this function */
	return ERR_NONE;
}

uint32 camkit_get_sensor_info(struct camkit_sensor *sensor,
	struct camkit_sensor_info_t *sensor_info)
{
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params->sensor_params);
	return_err_if_null(sensor_info);
	params = kit_params->sensor_params;

	if (memcpy_s((void *)sensor_info,
		sizeof(struct camkit_sensor_info_t),
		(void *)&(params->sensor_info),
		sizeof(struct camkit_sensor_info_t)) != EOK) {
		log_err("memcpy sensor info fail");
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

static uint32 set_setting_by_scenario(struct camkit_sensor_params *params,
	const uint32 scenario_id)
{
	int32 rc;
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_i2c_reg_setting *setting = NULL;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);
	setting = camkit_get_setting_by_scenario(params, scenario_id);
	return_err_if_null(setting);

	spin_lock(&camkit_lock);
	params->sensor_ctrl.sensor_mode = scenario_id;
	params->sensor_ctrl.pclk = mode_info->pclk;
	params->sensor_ctrl.line_length = mode_info->linelength;
	params->sensor_ctrl.frame_length = mode_info->framelength;
	params->sensor_ctrl.min_frame_length = mode_info->framelength;
	params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);

	log_info("Enter scenario_id:%d", scenario_id);
	rc = camkit_sensor_write_setting(&params->sensor_ctrl, setting);
	if (rc != ERR_NONE) {
		log_err("Write sensorMode[%u] settings Failed", scenario_id);
		return ERR_IO;
	}

	return ERR_NONE;
}

uint32 camkit_control(struct camkit_sensor *sensor,
	const uint32 scenario_id)
{
	uint32 ret;
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	drv_dbg("scenario_id = %d", scenario_id);
	spin_lock(&camkit_lock);
	params->sensor_ctrl.current_scenario_id =
		(enum camkit_scenario_type)scenario_id;
	spin_unlock(&camkit_lock);

	ret = set_setting_by_scenario(params, scenario_id);
	if (ret) {
		log_err("set setting fail");
		return ret;
	}

	if (params->sensor_ctrl.pdaf_mode && params->correction_info.spc_apply) {
		if (params->correction_info.spc_type == PDAF_SPC_LRC)
			write_lrc_data_to_sensor(kit_params);
		else if (params->correction_info.spc_type == PDAF_SPC_PDC)
			write_pdc_data_to_sensor(kit_params);
	}

	return ERR_NONE;
}

uint32 camkit_set_video_mode(struct camkit_sensor *sensor,
	const uint16 framerate)
{
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->sensor_ops->set_max_framerate);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	log_info("framerate = %u", framerate);
	if (framerate == 0)
		return ERR_NONE;

	spin_lock(&camkit_lock);
	/* fps set to 298 for anti-flicker */
	if ((framerate == 300) && (params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 298;
	/* fps set to 146 for anti-flicker */
	else if ((framerate == 150) &&
		(params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 146;
	else
		params->sensor_ctrl.current_fps = framerate;
	spin_unlock(&camkit_lock);

	sensor->sensor_ops->set_max_framerate(sensor,
		params->sensor_ctrl.current_fps, 1);

	return ERR_NONE;
}

uint32 camkit_set_auto_flicker(struct camkit_sensor *sensor,
	const uint8 enable, const uint16 framerate)
{
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	log_info("enable = %d, framerate = %u", enable, framerate);
	spin_lock(&camkit_lock);
	if (enable)
		params->sensor_ctrl.autoflicker_en = TRUE;
	else
		params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);

	return ERR_NONE;
}

static uint32 set_max_framerate_by_modeinfo(struct camkit_sensor *sensor,
	struct camkit_sensor_params *params,
	struct camkit_mode_info *mode_info, uint32 framerate)
{
	uint32 frame_length;

	if ((framerate == 0) || (mode_info->linelength == 0))
		return ERR_NONE;

	frame_length = mode_info->pclk / framerate *
		PIX_10_BITS / mode_info->linelength;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.dummy_line = (frame_length > mode_info->framelength) ?
		(frame_length - mode_info->framelength) : 0;
	params->sensor_ctrl.frame_length = mode_info->framelength +
		params->sensor_ctrl.dummy_line;
	params->sensor_ctrl.min_frame_length = params->sensor_ctrl.frame_length;
	spin_unlock(&camkit_lock);

	if (params->sensor_ctrl.frame_length > params->sensor_ctrl.shutter)
		sensor->sensor_ops->set_dummy(sensor);

	return ERR_NONE;
}

uint32 camkit_set_scenario_framerate(struct camkit_sensor *sensor,
	const uint32 scenario_id, const uint32 framerate)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_params *kit_params = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->sensor_ops->set_dummy);
	kit_params = sensor->kit_params;

	return_err_if_null(kit_params->sensor_params);
	params = kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	drv_dbg("scenario_id = %d, framerate = %u", scenario_id, framerate);

	return set_max_framerate_by_modeinfo(sensor,
		params, mode_info, framerate);
}

uint32 camkit_get_scenario_pclk(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pclk)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	*pclk = mode_info->pclk;

	return ERR_NONE;
}

uint32 camkit_get_scenario_period(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *period)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	*period = (mode_info->framelength << 16) + mode_info->linelength;

	return ERR_NONE;
}

uint32 camkit_get_default_framerate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *framerate)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	*framerate = mode_info->max_framerate;

	return ERR_NONE;
}

uint32 camkit_streaming_control(struct camkit_sensor *sensor,
	const uint8 enable)
{
	int32 rc;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	log_info("Enter.enable:%d", enable);
	if (enable)
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamon_setting);
	else
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamoff_setting);

	if (rc != ERR_NONE) {
		log_err("Failed: streaming enable:%d", enable);
		return ERR_IO;
	}
	log_info("Exit.enable:%d", enable);

	return ERR_NONE;
}

uint32 camkit_get_crop_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_output_info_t *win_info)
{
	errno_t err;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	err = memcpy_s((void *)win_info,
		sizeof(struct camkit_sensor_output_info_t),
		(void *)&params->output_info[scenario_id],
		sizeof(struct camkit_sensor_output_info_t));
	if (err != EOK) {
		log_err("memcpy_s win_info failed");
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

uint32 camkit_get_mipi_pixel_rate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *mipi_pixel_rate)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(mipi_pixel_rate);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	*mipi_pixel_rate = mode_info->mipi_pixel_rate;

	return ERR_NONE;
}

uint32 camkit_get_mipi_trail_val(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *mipi_trail_val)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(mipi_trail_val);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	*mipi_trail_val = mode_info->mipi_trail_val;

	return ERR_NONE;
}

uint32 camkit_get_sensor_pixel_rate(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pixel_rate)
{
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(pixel_rate);
	params = sensor->kit_params->sensor_params;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);

	if (mode_info->linelength > CAMKIT_LINGLENGTH_GAP)
		*pixel_rate = (mode_info->pclk /
			(mode_info->linelength - CAMKIT_LINGLENGTH_GAP)) *
			mode_info->grabwindow_width;

	return ERR_NONE;
}

uint32 camkit_get_pdaf_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_pdaf_info_t *pdaf_info)
{
	errno_t err;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(pdaf_info);
	params = sensor->kit_params->sensor_params;

	log_info("get pdaf info scenarioId:%u", scenario_id);

	err = memcpy_s((void *)pdaf_info,
		sizeof(struct camkit_sensor_pdaf_info_t),
		(void *)&params->pdaf_info,
		sizeof(struct camkit_sensor_pdaf_info_t));
	if (err != EOK) {
		log_err("memcpy_s pdaf_info failed, err is %d", err);
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

static uint32 get_pdaf_capacity_by_scenario(
	struct camkit_sensor_params *params,
	const uint32 scenario_id, uint32 *pdaf_support)
{
	struct camkit_mode_info *mode_info = NULL;

	mode_info = camkit_get_mode_info(params, scenario_id);
	return_err_if_null(mode_info);
	*pdaf_support = mode_info->pdaf_support;

	log_info("PDAF_CAPACITY scenarioId:%u, pdaf_support:%u",
		scenario_id, *pdaf_support);

	return ERR_NONE;
}

uint32 camkit_get_pdaf_capacity(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *pdaf_support, uint32 para_len)
{
	uint32 ret;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	return_err_if_null(pdaf_support);
	params = sensor->kit_params->sensor_params;

	if (!params->sensor_info.pdaf_support ||
		para_len < PDAF_FEATURE_PARA_LEN) {
		log_info("sensor not support pdaf or para_len = %u", para_len);
		return ERR_INVAL;
	}

	ret = get_pdaf_capacity_by_scenario(params, scenario_id, pdaf_support);
	if (ret != ERR_NONE)
		log_err("get_pdaf_capacity_by_scenario failed");

	return ERR_NONE;
}

uint32 camkit_get_binning_ratio(struct camkit_sensor *sensor,
	const uint32 scenario_id, uint32 *ratio)
{
	struct camkit_sensor_info_t *sensor_info = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(ratio);
	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	sensor_info = &params->sensor_info;

	if (scenario_id >= 0 && scenario_id < MAX_OUTPUT_INFO_SIZE)
		*ratio = sensor_info->binning_ratio[scenario_id];

	// default ratio is 1
	if (*ratio == 0)
		*ratio = 1;

	log_info("binning type ratio %d", *ratio);
	return ERR_NONE;
}

uint32 camkit_get_vc_info(struct camkit_sensor *sensor,
	const uint32 scenario_id, struct camkit_sensor_vc_info_t *vc_info)
{
	errno_t err;
	struct camkit_sensor_params *params = NULL;
	struct camkit_sensor_vc_info_t *camkit_vc_info = NULL;
	int camkit_vc_info_len;

	return_err_if_null(vc_info);
	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	camkit_vc_info_len = sizeof(struct camkit_sensor_vc_info_t);
	camkit_vc_info = params->vc_info;

	err = memcpy_s((void *)vc_info, camkit_vc_info_len,
		(void *)&camkit_vc_info[scenario_id], camkit_vc_info_len);
	if (err != EOK) {
		log_err("memcpy_s vc_info[%d] failed", scenario_id);
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

uint32 camkit_get_pdaf_regs_data(struct camkit_sensor *sensor,
	uint16 *reg_pairs, const uint32 reg_num)
{
	uint32 i;
	uint32 reg_idx;
	int32 ret;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(reg_pairs);
	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	for (i = 0; i < reg_num; i++) {
		reg_idx = i * 2;  // register pairs: [addr, data, addr, data...]
		ret = camkit_sensor_i2c_read(&params->sensor_ctrl, reg_pairs[reg_idx],
			reg_pairs + reg_idx + 1, CAMKIT_I2C_BYTE_DATA);
		if (ret) {
			log_err("read register fail: 0x%x", reg_pairs[reg_idx]);
			return ERR_IO;
		}
		drv_dbg("[0x%x 0x%x]", reg_pairs[reg_idx], reg_pairs[reg_idx + 1]);
	}

	return ERR_NONE;
}

uint32 camkit_set_pdaf_setting(struct camkit_sensor *sensor,
	uint16 *setting, const uint32 reg_num)
{
	int32 ret;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	ret = camkit_i2c_write_table(&params->sensor_ctrl, setting,
		reg_num * 2, CAMKIT_I2C_BYTE_DATA);
	if (ret != ERR_NONE) {
		log_err("read register fail");
		return ERR_IO;
	}

	return ERR_NONE;
}

uint32 camkit_set_current_fps(struct camkit_sensor *sensor,
	const uint16 framerate)
{
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.current_fps = framerate;
	spin_unlock(&camkit_lock);

	return ERR_NONE;
}

uint32 camkit_set_pdaf_mode(struct camkit_sensor *sensor, const uint16 mode)
{
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.pdaf_mode = mode;
	spin_unlock(&camkit_lock);

	return ERR_NONE;
}

static struct sensor_kit_ops sensor_driver_ops = {
	.sensor_open = camkit_open,
	.sensor_close = camkit_close,
	.match_id = camkit_match_id,
	.sensor_init = camkit_sensor_init,
	.get_sensor_info = camkit_get_sensor_info,
	.control = camkit_control,
	.get_scenario_pclk = camkit_get_scenario_pclk,
	.get_scenario_period = camkit_get_scenario_period,
	.set_test_pattern = camkit_set_test_pattern,
	.dump_reg = camkit_dump_reg,
	.set_auto_flicker = camkit_set_auto_flicker,
	.get_default_framerate = camkit_get_default_framerate,
	.get_crop_info = camkit_get_crop_info,
	.streaming_control = camkit_streaming_control,
	.get_mipi_pixel_rate = camkit_get_mipi_pixel_rate,
	.get_mipi_trail_val = camkit_get_mipi_trail_val,
	.get_sensor_pixel_rate = camkit_get_sensor_pixel_rate,
	.get_pdaf_capacity = camkit_get_pdaf_capacity,
	.get_binning_ratio = camkit_get_binning_ratio,
	.get_pdaf_info = camkit_get_pdaf_info,
	.get_vc_info = camkit_get_vc_info,
	.get_pdaf_regs_data = camkit_get_pdaf_regs_data,
	.set_pdaf_setting = camkit_set_pdaf_setting,
	.set_video_mode = camkit_set_video_mode,
	.set_shutter = camkit_set_shutter,
	.set_gain = camkit_set_gain,
	.set_dummy = camkit_set_dummy,
	.set_max_framerate = camkit_set_max_framerate,
	.set_scenario_framerate = camkit_set_scenario_framerate,
	.set_shutter_frame_length = camkit_set_shutter_frame_length,
	.set_current_fps = camkit_set_current_fps,
	.set_pdaf_mode = camkit_set_pdaf_mode,
};

uint32 camkit_get_normalized_ops(struct sensor_kit_ops **ops)
{
	if (ops != NULL) {
		*ops = &sensor_driver_ops;
	} else {
		log_err("initial sensor ops fail");
		return ERR_INVAL;
	}

	return ERR_NONE;
}
