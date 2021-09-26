/*
 * camkit_driver.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camkit interface
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

#include <securec.h>
#include "camkit_driver.h"
#include <linux/spinlock.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>

#include "kd_imgsensor_errcode.h"
#include "camkit_sensor_i2c.h"

#define RETRY_TIMES 2
#define PIX_10_BITS 10
#define PDAF_FEATURE_PARA_LEN 2

#define PFX "[sensorkit]"
#define DEBUG_SENSOR_KIT 0
#define GC8034_DD_SENSOR_ID 0x8044
#define KEH_DD_SENSOR_ID 0x5035
#define PDAF_CUSTOM3_INDEX 0x07
/* As defined in imgsensor_drv, scenario_ids array size is 16 */
#define SEAMLESS_SCENARIO_ID_SIZE 16
/* offset for expo info in ae_ctrl */
#define EXPO_TABLE_OFFSET 0
#define GAIN_TABLE_OFFSET 1

#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_SENSOR_KIT) \
			pr_info(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define loge_if(x) \
	do { \
		if (x) \
			LOG_ERR("'%s' failed, ret = %d", #x, x); \
	} while (0)

const char* g_driver_error_info[DRIVER_ERROR_TYPE_END] = {
	"(read qsc data from eeprom fail)",
	"(read spc data from eeprom fail)",
	"(write lsc data to sensor fail)",
	"(write rsc data to sensor fail)",
	"(write pdc data to sensor fail)",
	"(write qsc data to sensor fail)",
	"(set init setting fail)",
	"(set resolution setting fail)",
};

DEFINE_SPINLOCK(camkit_lock);

extern void gc8034_otp_function(void);
extern void gc8034_otp_identify(void);
extern void keh_otp_function(void);
extern void keh_otp_identify(void);
extern uint32 pre_shutter; /* shutter be set in previous((N-2)th) frame. */
extern uint32 last_shutter; /* store the last shutter set by hal. */

static uint32 camkit_translate_scenario(enum MSDK_SCENARIO_ID_ENUM scenaio_id)
{
	uint32 index = 0;
	int i;

	/* 0~9 is array index */
	static struct scenaio_index_table index_table[] = {
		{ MSDK_SCENARIO_ID_CAMERA_PREVIEW, 0 },
		{ MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG, 1 },
		{ MSDK_SCENARIO_ID_VIDEO_PREVIEW, 2 },
		{ MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO, 3 },
		{ MSDK_SCENARIO_ID_SLIM_VIDEO, 4 },
		{ MSDK_SCENARIO_ID_CUSTOM1, 5 },
		{ MSDK_SCENARIO_ID_CUSTOM2, 6 },
		{ MSDK_SCENARIO_ID_CUSTOM3, 7 },
		{ MSDK_SCENARIO_ID_CUSTOM4, 8 },
		{ MSDK_SCENARIO_ID_CUSTOM5, 9 },
	};

	for (i = 0; i < CAMKIT_ARRAY_SIZE(index_table); ++i) {
		if (scenaio_id == index_table[i].scenaio_id) {
			index = index_table[i].index;
			break;
		}
	}
	return index;
}
static inline void camkit_hiview_handle(int32 ret, int error_no,
	const char *ic_name, driver_error_type error_type)
{
	if (ret == -EIO)
		camkit_hiview_report(error_no, ic_name, error_type);
}

static struct camkit_mode_info *get_sensor_mode_info(
	struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	return mode_info;
}

static int32 read_data_from_eeprom(uint16 i2c_addr,
	uint16 addr, uint16 len, uint8 *data)
{
	uint16 i;
	uint16 offset = addr;
	int32 ret;

	for (i = 0; i < len; i++) {
		ret = camkit_read_eeprom(i2c_addr, offset, &data[i]);
		if (ret < 0) {
			LOG_ERR("read fail, addr: 0x%x", offset);
			return ret;
		}

		offset++;
	}

	return ERROR_NONE;
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
		LOG_INF("need not read qsc data from eeprom");
		return ERROR_NONE;
	}
	if (correction_info->qsc_read_flag) {
		LOG_INF("qsc data had read from eeprom");
		return ERROR_NONE;
	}

	correction_info->qsc_buf = kzalloc(correction_info->qsc_len, GFP_KERNEL);
	if (!correction_info->qsc_buf) {
		LOG_ERR("memory not enough");
		return -ENOMEM;
	}

	if (correction_info->block_read_support)
		ret = camkit_eeprom_block_read(module_params->eeprom_i2c_addr,
			correction_info->eeprom_qsc_addr, correction_info->qsc_len,
			correction_info->qsc_buf, correction_info->eeprom_i2c_speed);
	else
		ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
			correction_info->eeprom_qsc_addr, correction_info->qsc_len,
			correction_info->qsc_buf);

	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_OTP_I2C_ERR,
			module_params->sensor_name, READ_EEPROM_QSC_DATA_FAILED);
		LOG_ERR("read qsc data from eeprom fail");
		kfree(correction_info->qsc_buf);
		correction_info->qsc_buf = NULL;
		return -EFAULT;
	}
	correction_info->qsc_read_flag = 1;

	return ERROR_NONE;
}

uint16 pdc_addr[][2] = {
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
	uint32 size = sizeof(pdc_addr) / sizeof(pdc_addr[0]);
	uint32 i;
	uint32 j;

	for (i = 0; i < size - 1; i++) {
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
		LOG_INF("need not read spc data from eeprom");
		return ERROR_NONE;
	}
	if (correction_info->spc_read_flag) {
		LOG_INF("spc data had read from eeprom");
		return ERROR_NONE;
	}

	correction_info->pdaf_buf = kzalloc(correction_info->pdaf_len, GFP_KERNEL);
	if (!correction_info->pdaf_buf) {
		LOG_ERR("memory not enough");
		return -ENOMEM;
	}

	if (correction_info->block_read_support)
		ret = camkit_eeprom_block_read(module_params->eeprom_i2c_addr,
			correction_info->eeprom_pdaf_addr, correction_info->pdaf_len,
			correction_info->pdaf_buf, correction_info->eeprom_i2c_speed);
	else
		ret = read_data_from_eeprom(module_params->eeprom_i2c_addr,
			correction_info->eeprom_pdaf_addr, correction_info->pdaf_len,
			correction_info->pdaf_buf);

	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_OTP_I2C_ERR,
			module_params->sensor_name, READ_EEPROM_SPC_DATA_FAILED);
		LOG_ERR("read spc data from eeprom fail");
		kfree(correction_info->pdaf_buf);
		correction_info->pdaf_buf = NULL;
		return -EFAULT;
	}
	correction_info->spc_read_flag = 1;

	if (correction_info->spc_type == PDAF_SPC_PDC)
		extract_valid_data_for_ov48b(correction_info->pdaf_buf);

	return ERROR_NONE;
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

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len != (correction_info->lsc_addr_len +
		correction_info->rsc_addr_len)) {
		LOG_ERR("pdaf correction info configure error, please check");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->lsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->lsc_addr_len);
	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, WRITE_LSC_DATA_FAILED);
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->rsc_start_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf + correction_info->lsc_addr_len,
		correction_info->rsc_addr_len);
	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, WRITE_RSC_DATA_FAILED);
		LOG_ERR("write rsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static int32 write_pdc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;

	correction_info = &(sensor_params->correction_info);
	if (correction_info->pdaf_len < correction_info->pdc_len) {
		LOG_ERR("pdaf correction info configure error, please check");
		return -EFAULT;
	}

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->pdc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->pdaf_buf, correction_info->pdc_len);
	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, WRITE_PDC_DATA_FAILED);
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static int32 write_qsc_data_to_sensor(struct camkit_params *kit_params)
{
	struct camkit_sensor_correction_t *correction_info = NULL;
	struct camkit_module_params *module_params = NULL;
	struct camkit_sensor_params *sensor_params = NULL;
	int32 ret;

	LOG_INF("ENTER");

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameters");
		return -EFAULT;
	}
	sensor_params = kit_params->sensor_params;
	module_params = kit_params->module_params;

	correction_info = &(sensor_params->correction_info);

	ret = camkit_i2c_write_block(sensor_params->sensor_ctrl.i2c_write_id,
		sensor_params->sensor_ctrl.i2c_speed,
		correction_info->sensor_qsc_addr,
		sensor_params->sensor_ctrl.addr_type,
		correction_info->qsc_buf, correction_info->qsc_len);
	if (ret < 0) {
		camkit_hiview_handle(ret, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, WRITE_QSC_DATA_FAILED);
		LOG_ERR("write lsc fail");
		return -EFAULT;
	}

	LOG_INF("EXIT");

	return ERROR_NONE;
}

static uint32 sensor_init(struct camkit_params *kit_params)
{
	int32 rc;
	struct camkit_sensor_params *params = NULL;

	LOG_INF("ENTER\n");

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	params = kit_params->sensor_params;
	rc = camkit_sensor_write_setting(&params->sensor_ctrl,
		&params->sensor_info.init_setting);
	if (rc < 0) {
		camkit_hiview_handle(rc, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, SET_INIT_SETTING_FAILED);
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}

	if (params->sensor_info.init_burst_setting.setting != NULL &&
		params->sensor_info.init_burst_setting.size > 0) {
		rc = camkit_sensor_write_block(&params->sensor_ctrl,
			params->sensor_info.init_burst_setting.setting,
			params->sensor_info.init_burst_setting.size,
			params->sensor_info.init_burst_setting.data_type);
		if (rc < 0)
			LOG_ERR("something abnormal\n");
	}

	LOG_INF("qsc_apply:%d", params->correction_info.qsc_apply);
	if (params->correction_info.qsc_apply)
		(void)write_qsc_data_to_sensor(kit_params);

	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static uint32 set_test_pattern_mode(struct camkit_sensor_params *params,
	bool enable)
{
	spin_lock(&camkit_lock);
	params->sensor_ctrl.test_pattern = enable;
	spin_unlock(&camkit_lock);
	return ERROR_NONE;
}

static uint32 sensor_dump_reg(struct camkit_sensor_params *params)
{
	int32 rc;

	LOG_INF("ENTER\n");
	rc = camkit_sensor_i2c_process(&params->sensor_ctrl,
		&params->sensor_info.dump_info, NULL, 0);
	if (rc < 0)
		LOG_ERR("Failed\n");
	LOG_INF("EXIT\n");
	return ERROR_NONE;
}

static uint32 match_sensor_id(struct camkit_params *params)
{
	int32 rc;
	uint8 i = 0;
	uint8 retry = RETRY_TIMES;
	uint8 size = 0;
	uint16 sensor_id = 0;
	struct camkit_sensor_params *sensor_params = params->sensor_params;
	struct camkit_module_params *module_params = params->module_params;
	struct camkit_sensor_info_t *sensor_info = &sensor_params->sensor_info;
	uint16 expect_id = sensor_info->sensor_id;
	camkit_i2c_data_type data_type = sensor_info->sensor_id_dt;
	struct camkit_i2c_reg_setting *setting = &(sensor_info->id_init_setting);

	spin_lock(&camkit_lock);
	/* init i2c config */
	sensor_params->sensor_ctrl.i2c_speed = sensor_info->i2c_speed;
	sensor_params->sensor_ctrl.addr_type = sensor_info->addr_type;
	spin_unlock(&camkit_lock);

	size = CAMKIT_ARRAY_SIZE(sensor_info->i2c_addr_table);
	LOG_INF("sensor i2c addr num = %u", size);

	while ((i < size) && (sensor_info->i2c_addr_table[i] != 0xff)) {
		spin_lock(&camkit_lock);
		sensor_params->sensor_ctrl.i2c_write_id = sensor_info->i2c_addr_table[i];
		spin_unlock(&camkit_lock);
		do {
			// workaround, TODO
			LOG_INF("sensor name: %s\n", module_params->sensor_name);

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
			if (rc == 0 && sensor_id == expect_id) {
				LOG_INF("sensor id: 0x%x matched", sensor_id);
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}

	LOG_INF("sensor id mismatch, expect:0x%x, real:0x%x",
		expect_id, sensor_id);

	return ERROR_SENSOR_CONNECT_FAIL;
}

static int config_eeprom(struct camkit_config_eeprom *eeprom_param,
	enum eeprom_config_type config_type)
{
	uint32 ret = ERROR_NONE;
	uint32 i2c_idx;
	struct IMGSENSOR_I2C_CFG *i2c_cfg = NULL;
	struct camkit_i2c_reg_setting *setting = NULL;

	if (!eeprom_param) {
		LOG_ERR("param is null\n");
		return EFAULT;
	}

	/* 1: need config */
	if (eeprom_param->need_config != 1) {
		LOG_DBG("no need config eeprom");
		return ret;
	}

	i2c_cfg = kzalloc(sizeof(struct IMGSENSOR_I2C_CFG), GFP_KERNEL);
	if (!i2c_cfg) {
		LOG_ERR("memory alloc failed\n");
		return ENOMEM;
	}
	i2c_idx = eeprom_param->i2c_index;

	if (imgsensor_i2c_init(i2c_cfg, i2c_idx) != IMGSENSOR_RETURN_SUCCESS) {
		LOG_ERR("imgsensor i2c init failed\n");
		ret = EFAULT;
		goto exit_config_eeprom;
	}

	if (config_type == EEPROM_ATTACH_CONFIG)
		setting = &eeprom_param->attach_setting;
	else if (config_type == EEPROM_DETACH_CONFIG)
		setting = &eeprom_param->detach_setting;

	if (setting) {
		ret = camkit_i2c_write_setting(i2c_cfg,
			eeprom_param->i2c_speed,
			eeprom_param->i2c_addr, setting);
		if (ret < 0)
			LOG_ERR("camkit_i2c_write_setting fail\n");
	}

exit_config_eeprom:
	kfree(i2c_cfg);
	return ret;
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
		LOG_INF("not to match module code");
		return ERROR_NONE;
	}

	loge_if(config_eeprom(&params->config_eeprom, EEPROM_ATTACH_CONFIG));

	do {
		rc = camkit_i2c_read(i2c_addr, module_addr, params->addr_type,
			&module_code, params->data_type);
		if (rc == 0 && module_code == expect_code) {
			LOG_INF("module code: 0x%x matched", module_code);
			if (lens_addr == 0)
				break;

			rc = camkit_i2c_read(i2c_addr, lens_addr, params->addr_type,
				&lens_type, params->data_type);
			if (rc == 0 && lens_type == expect_lens) {
				LOG_INF("lens type: 0x%x matched", lens_type);
				break;
			}
		}
		retry--;
	} while (retry > 0);

	loge_if(config_eeprom(&params->config_eeprom, EEPROM_DETACH_CONFIG));

	if (retry > 0)
		return ERROR_NONE;

	if (rc != 0 && params->allow_no_eeprom) {
		LOG_INF("read from eeprom failed but allow no eeprom");
		return ERROR_NONE;
	}

	LOG_ERR("module code, expect:0x%x, real:0x%x",
		expect_code, module_code);
	LOG_ERR("lens type, expect:0x%x, real:0x%x",
		expect_lens, lens_type);

	return ERROR_SENSOR_CONNECT_FAIL;
}


static uint32 match_lcd_model(struct camkit_module_params *params)
{
	struct device_node *np = NULL;
	const char *temp_name = NULL;
	int ret;

	if (!params) {
		 LOG_ERR("camkit_module_params = NULL");
		 return ERROR_NONE;
	}
	if (params->lcd_params.need_lcd_name) {
		np = of_find_compatible_node(NULL, NULL, "huawei,lcd_panel_type");
		if (!np) {
			LOG_ERR("get lcd_config fail, Ignore\n");
			return ERROR_NONE;
		}
		ret = of_property_read_string(np, "lcd_panel_type", &temp_name);
		if (ret < 0) {
			LOG_ERR("get lcd panel-name fail, Ignore\n");
			return ERROR_NONE;
		}
	} else {
		LOG_INF("no need to match LCD name\n");
		return ERROR_NONE;
	}
	LOG_DBG("lcd panel-name =%s\n", temp_name);
	if (strcasecmp(params->lcd_params.lcd_name, temp_name)) {
		LOG_INF("lcd name, expect:%s, real:%s\n",
			params->lcd_params.lcd_name, temp_name);
		return ERROR_SENSOR_CONNECT_FAIL;
	} else {
		LOG_INF("LCD name(%s) is matched\n", temp_name);
		return ERROR_NONE;
	}
}

static uint32 get_match_id(struct camkit_params *kit_params,
	uint32 *match_id)
{
	uint32 rc;

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	rc = match_sensor_id(kit_params);
	if (rc != ERROR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	rc = match_module_id(kit_params->module_params);
	if (rc != ERROR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}
	if (kit_params->sensor_params->sensor_info.sensor_id == GC8034_DD_SENSOR_ID)
		gc8034_otp_identify();
	else if (kit_params->sensor_params->sensor_info.sensor_id == KEH_DD_SENSOR_ID)
		keh_otp_identify();

	rc = match_lcd_model(kit_params->module_params);
	if (rc != ERROR_NONE) {
		*match_id = 0xFFFFFFFF;
		return rc;
	}

	if (kit_params->sensor_params->sensor_info.need_correction)
		read_calib_data_from_eeprom(kit_params);

	*match_id = kit_params->module_params->match_id;
	LOG_INF("match id successfully, sensor id: 0x%x, \
		module code: 0x%x, match id: 0x%x\n",
		kit_params->sensor_params->sensor_info.sensor_id,
		kit_params->module_params->module_code,
		kit_params->module_params->match_id);

	return ERROR_NONE;
}

static uint32 config_high_impedance_settings(
	struct IMGSENSOR_I2C_CFG *i2c_cfg,
	struct camkit_high_impedance_sensor_t *sensor)
{
	int i = 0;
	int32 rc;
	uint8 i2c_write_id;
	int retry = RETRY_TIMES;
	int size;

	if (!i2c_cfg || !sensor) {
		LOG_ERR("%s i2c_cfg or sensor is null\n", __func__);
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	size = CAMKIT_ARRAY_SIZE(sensor->i2c_addr_table);
	while ((i < size) && (sensor->i2c_addr_table[i] != 0xff)) {
		i2c_write_id = sensor->i2c_addr_table[i];
		do {
			rc = camkit_i2c_write_setting(i2c_cfg,
				sensor->i2c_speed, i2c_write_id,
				&sensor->high_impedance_setting);
			if (rc == 0) {
				LOG_INF("high impedance register write id 0x%x speed %u\n",
					i2c_write_id, sensor->i2c_speed);
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}

	LOG_INF("set high impedance fail\n");
	return ERROR_SENSOR_CONNECT_FAIL;
}

static uint32 set_high_impedance(
	struct camkit_high_impedance_info_t *high_impedance)
{
	uint32 ret = ERROR_NONE;
	uint32 i2c_idx;
	struct IMGSENSOR_I2C_CFG *i2c_cfg = NULL;
	struct camkit_high_impedance_sensor_t *sensor = NULL;

	if (!high_impedance) {
		LOG_ERR("high_impedance is null\n");
		return EFAULT;
	}

	sensor = &high_impedance->sensor[high_impedance->matched_index];
	/*
	 * for sensors which set high impedance without need to config registers.
	 * if settings is not configed or settings size is 0,
	 * that means no need to config registers, just return.
	 */
	if (!sensor->high_impedance_setting.setting ||
		sensor->high_impedance_setting.size == 0) {
		LOG_INF("sensor %s no need to config high impedance settings, setting:%p, size:%u\n",
			sensor->sensor_name,
			sensor->high_impedance_setting.setting,
			sensor->high_impedance_setting.size);
		return ERROR_NONE;
	}

	i2c_cfg = kzalloc(sizeof(struct IMGSENSOR_I2C_CFG), GFP_KERNEL);
	if (!i2c_cfg) {
		LOG_ERR("memory alloc failed\n");
		return ENOMEM;
	}
	i2c_idx = sensor->i2c_index;
	if (imgsensor_i2c_init(i2c_cfg, i2c_idx) != IMGSENSOR_RETURN_SUCCESS) {
		LOG_ERR("imgsensor i2c init failed\n");
		ret = EFAULT;
		goto exit_high_impedance;
	}

	ret = config_high_impedance_settings(i2c_cfg, sensor);

exit_high_impedance:
	kfree(i2c_cfg);
	return ret;
}

static uint32 open(struct camkit_params *params)
{
	uint32 sensor_id = 0;
	uint32 rc;
	struct camkit_sensor_params *sensor_params = NULL;

	LOG_INF("ENTER\n");

	if (!params || !params->sensor_params || !params->module_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}

	sensor_params = params->sensor_params;

	if (params->module_params->skip_open_match_id != 1) {
		rc = get_match_id(params, &sensor_id);
		if (rc != ERROR_NONE) {
			LOG_ERR("probe sensor failed\n");
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		LOG_DBG("sensor probe successfully. sensor_id = 0x%x\n", sensor_id);
	}

	if ((sensor_params->sensor_info.need_high_impedance == 1) &&
		(sensor_params->high_impedance_info.matched_flag == 1)) {
		rc = set_high_impedance(&sensor_params->high_impedance_info);
		if (rc != ERROR_NONE)
			LOG_ERR("can not match the high impedance sensor\n");
	}

	/* initail sequence write in  */
	rc = sensor_init(params);
	if (rc != ERROR_NONE) {
		LOG_ERR("init failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	if (sensor_params->sensor_info.sensor_id == GC8034_DD_SENSOR_ID)
		gc8034_otp_function();
	else if (sensor_params->sensor_info.sensor_id == KEH_DD_SENSOR_ID)
		keh_otp_function();
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

	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static uint32 close(struct camkit_params *params)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static void get_setting_info_by_scenario(struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	struct camkit_mode_info **mode_info,
	struct camkit_i2c_reg_setting **setting,
	uint8 *mode)
{
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*mode = CAMKIT_MODE_PREVIEW;
		*mode_info = &(params->sensor_info.pre);
		*setting = &(params->sensor_info.pre_setting);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*mode = CAMKIT_MODE_CAPTURE;
		*mode_info = &(params->sensor_info.cap);
		*setting = &(params->sensor_info.cap_setting);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*mode = CAMKIT_MODE_VIDEO;
		*mode_info = &(params->sensor_info.normal_video);
		*setting = &(params->sensor_info.normal_video_setting);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*mode = CAMKIT_MODE_HIGH_SPEED_VIDEO;
		*mode_info = &(params->sensor_info.hs_video);
		*setting = &(params->sensor_info.hs_video_setting);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*mode = CAMKIT_MODE_SLIM_VIDEO;
		*mode_info = &(params->sensor_info.slim_video);
		*setting = &(params->sensor_info.slim_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*mode = CAMKIT_MODE_CUSTOM1;
		*mode_info = &(params->sensor_info.custom1);
		*setting = &(params->sensor_info.custom1_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*mode = CAMKIT_MODE_CUSTOM2;
		*mode_info = &(params->sensor_info.custom2);
		*setting = &(params->sensor_info.custom2_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*mode = CAMKIT_MODE_CUSTOM3;
		*mode_info = &(params->sensor_info.custom3);
		*setting = &(params->sensor_info.custom3_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*mode = CAMKIT_MODE_CUSTOM4;
		*mode_info = &(params->sensor_info.custom4);
		*setting = &(params->sensor_info.custom4_setting);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*mode = CAMKIT_MODE_CUSTOM5;
		*mode_info = &(params->sensor_info.custom5);
		*setting = &(params->sensor_info.custom5_setting);
		break;
	default:
		LOG_ERR("Error scenario_id setting. scenario_id:%d\n",
			scenario_id);
		*mode = CAMKIT_MODE_PREVIEW;
		*mode_info = &(params->sensor_info.pre);
		*setting = &(params->sensor_info.pre_setting);
		break;
	}
}

static void update_sensor_ctrl(struct camkit_sensor_params *params,
	uint8 mode, struct camkit_mode_info *mode_info)
{
	spin_lock(&camkit_lock);
	params->sensor_ctrl.sensor_mode = mode;
	params->sensor_ctrl.pclk = mode_info->pclk;
	params->sensor_ctrl.line_length = mode_info->linelength;
	params->sensor_ctrl.frame_length = mode_info->framelength;
	params->sensor_ctrl.min_frame_length = mode_info->framelength;
	params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);
}

static uint32 set_setting_by_scenario(struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int32 rc;
	uint8 mode = CAMKIT_MODE_PREVIEW;
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_i2c_reg_setting *setting = NULL;
	struct camkit_sensor_params *params = kit_params->sensor_params;

	LOG_DBG("ENTER\n");

	get_setting_info_by_scenario(params, scenario_id,
		&mode_info, &setting, &mode);
	if (!mode_info || !setting) {
		LOG_ERR("get settings info by scenario failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}

	update_sensor_ctrl(params, mode, mode_info);
	LOG_INF("Enter scenario_id:%d\n", scenario_id);
	rc = camkit_sensor_write_setting(&params->sensor_ctrl, setting);
	if (rc < 0) {
		camkit_hiview_handle(rc, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, SET_RESOLUTION_SETTING_FAILED);
		LOG_ERR("Write sensorMode[%u] settings Failed\n", mode);
		return ERROR_DRIVER_INIT_FAIL;
	}

	LOG_DBG("EXIT\n");
	return ERROR_NONE;
}

static uint32 get_resolution(struct camkit_params *kit_params,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	struct camkit_sensor_params *params = NULL;

	if (!kit_params || !kit_params->sensor_params ||
		!sensor_resolution) {
		LOG_ERR("NULL ptr. params:%pK, sensor_resolution:%pK\n",
			kit_params, sensor_resolution);
		return ERROR_NONE;
	}

	params = kit_params->sensor_params;
	if (sensor_resolution) {
		sensor_resolution->SensorFullWidth =
			params->sensor_info.cap.grabwindow_width;
		sensor_resolution->SensorFullHeight =
			params->sensor_info.cap.grabwindow_height;

		sensor_resolution->SensorPreviewWidth =
			params->sensor_info.pre.grabwindow_width;
		sensor_resolution->SensorPreviewHeight =
			params->sensor_info.pre.grabwindow_height;

		sensor_resolution->SensorVideoWidth =
			params->sensor_info.normal_video.grabwindow_width;
		sensor_resolution->SensorVideoHeight =
			params->sensor_info.normal_video.grabwindow_height;

		sensor_resolution->SensorHighSpeedVideoWidth =
			params->sensor_info.hs_video.grabwindow_width;
		sensor_resolution->SensorHighSpeedVideoHeight =
			params->sensor_info.hs_video.grabwindow_height;

		sensor_resolution->SensorSlimVideoWidth =
			params->sensor_info.slim_video.grabwindow_width;
		sensor_resolution->SensorSlimVideoHeight =
			params->sensor_info.slim_video.grabwindow_height;

		sensor_resolution->SensorCustom1Width =
			params->sensor_info.custom1.grabwindow_width;
		sensor_resolution->SensorCustom1Height =
			params->sensor_info.custom1.grabwindow_height;

		sensor_resolution->SensorCustom2Width =
			params->sensor_info.custom2.grabwindow_width;
		sensor_resolution->SensorCustom2Height =
			params->sensor_info.custom2.grabwindow_height;

		sensor_resolution->SensorCustom3Width =
			params->sensor_info.custom3.grabwindow_width;
		sensor_resolution->SensorCustom3Height =
			params->sensor_info.custom3.grabwindow_height;

		sensor_resolution->SensorCustom4Width =
			params->sensor_info.custom4.grabwindow_width;
		sensor_resolution->SensorCustom4Height =
			params->sensor_info.custom4.grabwindow_height;

		sensor_resolution->SensorCustom5Width =
			params->sensor_info.custom5.grabwindow_width;
		sensor_resolution->SensorCustom5Height =
			params->sensor_info.custom5.grabwindow_height;
	}

	return ERROR_NONE;
}

static uint32 get_mode_info(MSDK_SENSOR_INFO_STRUCT *sensor_info,
	struct camkit_mode_info *mode_info)
{
	sensor_info->SensorGrabStartX = mode_info->startx;
	sensor_info->SensorGrabStartY = mode_info->starty;
	sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			mode_info->mipi_data_lp2hs_settle_dc;

	return ERROR_NONE;
}

static uint32 get_info(struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	struct camkit_sensor_params *params = NULL;
	struct camkit_mode_info *mode_info = NULL;

	if (!kit_params || !kit_params->sensor_params ||
		!sensor_info || !sensor_config_data) {
		LOG_ERR("NULL ptr. sensor_info:%pK, sensor_config_data:%pK\n",
			sensor_info, sensor_config_data);
		return ERROR_INVALID_PARA;
	}

	params = kit_params->sensor_params;

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity =
		SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = params->sensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = params->sensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = params->sensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		params->sensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = params->sensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = params->sensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = params->sensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		params->sensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		params->sensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		params->sensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame =
		params->sensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame =
		params->sensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame =
		params->sensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame =
		params->sensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame =
		params->sensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = params->sensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = params->sensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		params->sensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		params->sensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = params->sensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = params->sensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = params->sensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = params->sensor_info.pdaf_support;
	sensor_info->HDR_Support = params->sensor_info.stagger_cfg.hdr_support;

	sensor_info->SensorMIPILaneNumber = params->sensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = params->sensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	return get_mode_info(sensor_info, mode_info);
}

static int pdc_config(struct camkit_sensor_params *params, enum pdc_config_type config_type)
{
	struct camkit_i2c_reg_setting *setting = NULL;

	if (params->correction_info.pdc_config.need_config != 1) {
		LOG_DBG("no need excute pdc config");
		return 0;
	}

	if (config_type == BEFORE_PDC_CONFIG)
		setting = &params->correction_info.pdc_config.before_pdc_setting;
	else if (config_type == AFTER_PDC_CONFIG)
		setting = &params->correction_info.pdc_config.after_pdc_setting;

	if (!setting) {
		LOG_ERR("pdc config setting is NULL");
		return -1;
	}

	if (camkit_sensor_write_setting(&params->sensor_ctrl,
		setting) < 0) {
		LOG_ERR("camkit sensor write pdc config fail");
		return -1;
	}

	LOG_INF("pdc_config set succ, config type: %d", config_type);
	return 0;
}

static uint32 control(struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	struct camkit_sensor_params *params = NULL;
	uint32 ret;

	if (!kit_params || !kit_params->sensor_params) {
		LOG_ERR("invalid parameter");
		return ERROR_INVALID_PARA;
	}
	params = kit_params->sensor_params;

	LOG_DBG("scenario_id = %d\n", scenario_id);
	spin_lock(&camkit_lock);
	params->sensor_ctrl.current_scenario_id = (camkit_scenario_type)scenario_id;
	spin_unlock(&camkit_lock);

	ret = set_setting_by_scenario(kit_params, scenario_id,
		image_window, sensor_config_data);
	if (ret) {
		LOG_ERR("set setting fail");
		return ret;
	}

	if (params->sensor_ctrl.pdaf_mode && params->correction_info.spc_apply) {
		if (params->correction_info.spc_type == PDAF_SPC_LRC) {
			write_lrc_data_to_sensor(kit_params);
		} else if (params->correction_info.spc_type == PDAF_SPC_PDC ||
			params->correction_info.spc_type == PDAF_SPC_PDC_EXT) {
			pdc_config(params, BEFORE_PDC_CONFIG);
			write_pdc_data_to_sensor(kit_params);
			pdc_config(params, AFTER_PDC_CONFIG);
		}
	}

	return ERROR_NONE;
}

static uint32 set_video_mode(struct camkit_sensor_params *params,
	uint16 framerate)
{
	LOG_INF("framerate = %u\n ", framerate);
	if (framerate == 0)
		return ERROR_NONE;

	spin_lock(&camkit_lock);
	/* fps set to 298 for anti-flicker */
	if ((framerate == 300) && (params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 298;
	/* fps set to 146 for anti-flicker */
	else if ((framerate == 150) && (params->sensor_ctrl.autoflicker_en == TRUE))
		params->sensor_ctrl.current_fps = 146;
	else
		params->sensor_ctrl.current_fps = framerate;
	spin_unlock(&camkit_lock);

	set_max_framerate(params, params->sensor_ctrl.current_fps, 1);

	return ERROR_NONE;
}

static uint32 set_auto_flicker_mode(struct camkit_sensor_params *params,
	bool enable, uint16 framerate)
{
	LOG_INF("enable = %d, framerate = %u\n", enable, framerate);
	spin_lock(&camkit_lock);
	if (enable)
		params->sensor_ctrl.autoflicker_en = TRUE;
	else
		params->sensor_ctrl.autoflicker_en = FALSE;
	spin_unlock(&camkit_lock);

	return ERROR_NONE;
}

static uint32 set_max_framerate_by_modeinfo(
	struct camkit_sensor_params *params,
	struct camkit_mode_info *mode_info, uint32 framerate)
{
	uint32 frame_length;

	if ((framerate == 0) || (mode_info->linelength == 0))
		return ERROR_NONE;

	frame_length = mode_info->pclk / framerate * PIX_10_BITS / mode_info->linelength;

	spin_lock(&camkit_lock);
	params->sensor_ctrl.dummy_line = (frame_length > mode_info->framelength) ?
		(frame_length - mode_info->framelength) : 0;
	params->sensor_ctrl.frame_length = mode_info->framelength +
		params->sensor_ctrl.dummy_line;
	params->sensor_ctrl.min_frame_length = params->sensor_ctrl.frame_length;
	spin_unlock(&camkit_lock);

	if (params->sensor_ctrl.frame_length > params->sensor_ctrl.shutter)
		set_dummy(params);

	return ERROR_NONE;
}

static uint32 set_max_framerate_by_scenario(
	struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32 framerate)
{
	struct camkit_mode_info *mode_info = NULL;

	LOG_DBG("scenario_id = %d, framerate = %u\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &params->sensor_info.pre;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &params->sensor_info.cap;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &params->sensor_info.normal_video;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &params->sensor_info.hs_video;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &params->sensor_info.slim_video;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &params->sensor_info.custom1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &params->sensor_info.custom2;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &params->sensor_info.custom3;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &params->sensor_info.custom4;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &params->sensor_info.custom5;
		break;
	default:  /* coding with  preview scenario by default */
		mode_info = &params->sensor_info.pre;
		LOG_ERR("error scenario_id = %d, use preview scenario\n", scenario_id);
		break;
	}

	return set_max_framerate_by_modeinfo(params, mode_info, framerate);
}

static void get_pclk_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mode_info->pclk;
}

static void get_period_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) =
		(mode_info->framelength << 16) + mode_info->linelength;
}

static uint32 get_default_framerate_by_scenario(
	struct camkit_sensor_params *params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32 *framerate)
{
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = params->sensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = params->sensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = params->sensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = params->sensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = params->sensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = params->sensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = params->sensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = params->sensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = params->sensor_info.custom4.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = params->sensor_info.custom5.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static void check_streamoff(struct camkit_sensor_params *params)
{
	int32 rc;

	LOG_INF("ENTER\n");
	rc = camkit_sensor_i2c_process(&params->sensor_ctrl,
		&params->sensor_info.streamoff_check, NULL, 0);
	if (rc < 0)
		LOG_ERR("check_streamoff Failed\n");
	LOG_INF("EXIT\n");

	return;
}

static uint32 streaming_control(struct camkit_sensor_params *params,
	bool enable)
{
	int32 rc;

	LOG_INF("Enter, sensor_name:%s, enable:%d\n", params->sensor_name, enable);
	if (enable) {
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamon_setting);
	} else {
		rc = camkit_sensor_write_setting(&params->sensor_ctrl,
			&params->sensor_info.streamoff_setting);

		pre_shutter = 0;
		last_shutter = 0;

		if (params->sensor_info.check_streamoff_support)
			check_streamoff(params);
	}

	if (rc < 0) {
		LOG_ERR("Failed enable:%d\n", enable);
		return ERROR_SENSOR_POWER_ON_FAIL;
	}

	return ERROR_NONE;
}

static void get_sensor_crop_info(struct camkit_sensor_params *params,
	uint32 feature, struct SENSOR_WINSIZE_INFO_STRUCT *win_info)
{
	uint32 index;
	errno_t err;
	switch (feature) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		index = 0;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		/* arry 5 is custom1 setting */
		index = 5;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		/* arry 6 is custom2 setting */
		index = 6;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		/* arry 1 is capture setting */
		index = 1;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		/* arry 2 is video setting */
		index = 2;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		/* arry 3 is 120fps setting */
		index = 3;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		/* arry 4 is 30fps setting */
		index = 4;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		/* arry 7 is custom3 setting */
		index = 7;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		/* arry 8 is custom4 setting */
		index = 8;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		/* array 9 is custom5 setting */
		index = 9;
		break;
	default:
		/* arry 0 is preview setting */
		index = 0;
		break;
	}

	err = memcpy_s((void *)win_info,
		sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
		(void *)&params->output_info[index],
		sizeof(params->output_info[index]));
	if (err != EOK) {
		LOG_ERR("memcpy_s win_info failed!\n");
		return;
	}
}

static void get_sensor_mipi_pixel_rate(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mode_info->mipi_pixel_rate;
}

static void get_sensor_pixel_rate(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_mode_info *mode_info = NULL;

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		mode_info = &(params->sensor_info.pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(params->sensor_info.cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(params->sensor_info.normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(params->sensor_info.hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(params->sensor_info.slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(params->sensor_info.custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(params->sensor_info.custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(params->sensor_info.custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(params->sensor_info.custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(params->sensor_info.custom5);
		break;
	default:
		mode_info = &(params->sensor_info.pre);
		break;
	}

	if (mode_info->linelength > CAMKIT_LINGLENGTH_GAP)
		*(uint32 *)(uintptr_t)(*(feature_data + 1)) = (mode_info->pclk /
			(mode_info->linelength - CAMKIT_LINGLENGTH_GAP)) *
			mode_info->grabwindow_width;
}

static void main_camera_get_pdaf_info(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct SET_PD_BLOCK_INFO_T *PDAFinfo = NULL;
	errno_t err;
	LOG_INF("KYM_PDAF SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(uint32)*feature_data);
	PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	case MSDK_SCENARIO_ID_CUSTOM3:
		err = memcpy_s((void *)PDAFinfo, sizeof(struct SET_PD_BLOCK_INFO_T),
			(void *)&params->pdaf_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
		if (err != EOK) {
			LOG_INF("memcpy_s PDAFinfo failed!! err is %d\n", err);
			return;
		}
		break;
	default:
		break;
	}
}

static void get_pdaf_info_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct SET_PD_BLOCK_INFO_T *PDAFinfo = NULL;
	errno_t err;
	uint32 index;
	LOG_INF("KYM_PDAF SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%u\n",
			(uint32)*feature_data);

	PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
	index = camkit_translate_scenario((enum MSDK_SCENARIO_ID_ENUM)(*feature_data));
	if (index >= MAX_OUTPUT_INFO_SIZE) {
		LOG_ERR("invalid index: %u\n", index);
		return;
	}
	err = memcpy_s((void *)PDAFinfo, sizeof(struct SET_PD_BLOCK_INFO_T),
		(void *)&params->pdaf_info_scenario[index],
		sizeof(struct SET_PD_BLOCK_INFO_T));
	if (err != EOK) {
		LOG_INF("memcpy_s PDAF info failed!! err is %d\n", err);
		return;
	}
}

static void get_pdaf_info(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	if (params->sensor_info.pdaf_info_by_scenario)
		get_pdaf_info_by_scenario(params, feature_data);
	else
		main_camera_get_pdaf_info(params, feature_data);
}

static void get_pdaf_capacity_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	uint8 pdaf_support = 0;
	uint8 i;
	struct scenaio_pdaf_table pd_table[] = {
		{ MSDK_SCENARIO_ID_CAMERA_PREVIEW, params->sensor_info.pre.pdaf_support },
		{ MSDK_SCENARIO_ID_VIDEO_PREVIEW, params->sensor_info.normal_video.pdaf_support },
		{ MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG, params->sensor_info.cap.pdaf_support },
		{ MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO, params->sensor_info.hs_video.pdaf_support },
		{ MSDK_SCENARIO_ID_SLIM_VIDEO, params->sensor_info.slim_video.pdaf_support },
		{ MSDK_SCENARIO_ID_CUSTOM1, params->sensor_info.custom1.pdaf_support },
		{ MSDK_SCENARIO_ID_CUSTOM2, params->sensor_info.custom2.pdaf_support },
		{ MSDK_SCENARIO_ID_CUSTOM3, params->sensor_info.custom3.pdaf_support },
		{ MSDK_SCENARIO_ID_CUSTOM4, params->sensor_info.custom4.pdaf_support },
		{ MSDK_SCENARIO_ID_CUSTOM5, params->sensor_info.custom5.pdaf_support },
	};

	for (i = 0; i < CAMKIT_ARRAY_SIZE(pd_table); ++i) {
		if (*feature_data == pd_table[i].scenaio_id) {
			pdaf_support = pd_table[i].pdaf_support;
			break;
		}
	}

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = pdaf_support;

	LOG_INF("PDAF_CAPACITY scenarioId:%llu, pdaf_support:%u\n",
		*feature_data, pdaf_support);
}

static void main_camera_get_pdaf_capacity(unsigned long long *feature_data)
{
	LOG_DBG("PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	case MSDK_SCENARIO_ID_CUSTOM3:
		*(uint32 *)(uintptr_t)(*(feature_data+1)) = 1;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*(uint32 *)(uintptr_t)(*(feature_data + 1)) = 1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	default:
		*(uint32 *)(uintptr_t)(*(feature_data+1)) = 0;
		break;
	}
}

static void get_pdaf_capacity(struct camkit_sensor_params *params,
	unsigned long long *feature_data, uint32 para_len)
{
	if (!params->sensor_info.pdaf_support || para_len < PDAF_FEATURE_PARA_LEN) {
		LOG_INF("sensor not support pdaf or para_len = %u", para_len);
		return;
	}

	if (params->sensor_info.pdaf_support_by_scenario)
		get_pdaf_capacity_by_scenario(params, feature_data);
	else
		main_camera_get_pdaf_capacity(feature_data);
}

static void get_binning_type(struct camkit_sensor_params *params,
	unsigned long long *feature_data, uint32 *feature_param)
{
	struct camkit_sensor_info_t *sensor_info = &params->sensor_info;
	int32 index = 0;

	switch (*(feature_data + 1)) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		index = CAMKIT_SCENARIO_ID_CAMERA_PREVIEW;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		index = MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		index = MSDK_SCENARIO_ID_VIDEO_PREVIEW;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		index = MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		index = MSDK_SCENARIO_ID_SLIM_VIDEO;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		index = MSDK_SCENARIO_ID_CUSTOM1;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		index = MSDK_SCENARIO_ID_CUSTOM2;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		index = MSDK_SCENARIO_ID_CUSTOM3;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		index = MSDK_SCENARIO_ID_CUSTOM4;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		index = MSDK_SCENARIO_ID_CUSTOM5;
		break;
	default:
		index = CAMKIT_SCENARIO_ID_CAMERA_PREVIEW;
		break;
	}

	if (index >= 0 && index < MAX_OUTPUT_INFO_SIZE)
		*feature_param = sensor_info->binning_ratio[index];

	// default ratio is 1
	if (*feature_param == 0)
		*feature_param = 1;

	LOG_INF("binning type ratio %d\n", (*feature_param));
}

static void get_vc_info(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	errno_t err;
	struct SENSOR_VC_INFO_STRUCT *mtk_vc_info = NULL;
	int mtk_vc_info_len = sizeof(struct SENSOR_VC_INFO_STRUCT);
	int camkit_vc_info_len = sizeof(struct camkit_sensor_vc_info_t);
	struct camkit_sensor_vc_info_t *vc_info = params->vc_info;

	if (mtk_vc_info_len != camkit_vc_info_len) {
		LOG_ERR("mtk&camkit vc info struct mismatch");
		return;
	}

	mtk_vc_info = (struct SENSOR_VC_INFO_STRUCT *)
		(uintptr_t) (*(feature_data + 1));

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[0],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[0] failed!\n");
			return;
		}
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[1],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[1] failed!\n");
			return;
		}
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[2],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[2] failed!\n");
			return;
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[5],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[5] failed!\n");
			return;
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[PDAF_CUSTOM3_INDEX],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[3] failed!\n");
			return;
		}
		break;
	default:
		err = memcpy_s((void *)mtk_vc_info,
			sizeof(struct SENSOR_VC_INFO_STRUCT),
			(void *)&vc_info[0],
			mtk_vc_info_len);
		if (err != EOK) {
			LOG_ERR("memcpy_s vc_info[0] failed! at default\n");
			return;
		}
		break;
	}
}

static void get_vc_info2(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct SENSOR_VC_INFO2_STRUCT *mtk_vc_info2 = NULL;
	struct camkit_sensor_vc_info2_t *vc_info2 = params->vc_info2;
	struct camkit_sensor_vc_info2_t *sensor_vc_info = NULL;
	uint32 index;
	int i;

	if (!feature_data || !(feature_data + 1)) {
		LOG_ERR("feature_data is null\n");
		return;
	}
	mtk_vc_info2 = (struct SENSOR_VC_INFO2_STRUCT *)
		(uintptr_t) (*(feature_data + 1));

	index = camkit_translate_scenario((enum MSDK_SCENARIO_ID_ENUM)(*feature_data));
	if (index >= MAX_OUTPUT_INFO_SIZE) {
		LOG_ERR("invalid index: %u\n", index);
		return;
	}
	sensor_vc_info = &vc_info2[index];

	mtk_vc_info2->VC_Num = sensor_vc_info->vc_num;
	mtk_vc_info2->VC_PixelNum = sensor_vc_info->vc_pixel_num;
	mtk_vc_info2->ModeSelect = sensor_vc_info->mode_select;
	mtk_vc_info2->EXPO_Ratio = sensor_vc_info->expo_ratio;
	mtk_vc_info2->ODValue = sensor_vc_info->od_value;
	mtk_vc_info2->RG_STATSMODE = sensor_vc_info->rg_stats_mode;
	mtk_vc_info2->updated = sensor_vc_info->updated;
	for (i = 0; i < sensor_vc_info->vc_num && i < MAX_VC_NUM; i++) {
		mtk_vc_info2->vc_info[i].VC_FEATURE = sensor_vc_info->vc_info[i].vc_feature;
		mtk_vc_info2->vc_info[i].VC_ID = sensor_vc_info->vc_info[i].vc_id;
		mtk_vc_info2->vc_info[i].VC_DataType = sensor_vc_info->vc_info[i].vc_datatype;
		mtk_vc_info2->vc_info[i].VC_SIZEH_PIXEL = sensor_vc_info->vc_info[i].vc_sizeh_pixel;
		mtk_vc_info2->vc_info[i].VC_SIZEV = sensor_vc_info->vc_info[i].vc_sizev;
		mtk_vc_info2->vc_info[i].VC_SIZEH_BYTE = sensor_vc_info->vc_info[i].vc_size_byte;
	}
}

static void get_min_shutter_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	struct camkit_sensor_info_t *sensor_info = &params->sensor_info;
	uint32 index;

	if (!feature_data || !(feature_data + 1) || !(feature_data + 2)) {
		LOG_ERR("feature_data is null\n");
		return;
	}
	*(feature_data + 1) = params->aec_info.min_linecount;
	index = camkit_translate_scenario((enum MSDK_SCENARIO_ID_ENUM)(*(feature_data + 1)));
	if (index < MAX_OUTPUT_INFO_SIZE)
		*(feature_data + 2) = sensor_info->gain_step[index];

	// default gain step is 1
	if (*(feature_data + 2) == 0)
		*(feature_data + 2) = 1;
	LOG_INF("gain step type %u\n", *(feature_data + 2));
}


static void get_pdaf_regs_data(struct camkit_sensor_params *params,
	uint16 *reg_pairs, uint32 reg_num)
{
	uint32 i;
	uint32 reg_idx;
	int32 ret;

	for (i = 0; i < reg_num; i++) {
		reg_idx = i * 2;  // register pairs: [addr, data, addr, data...]
		ret = camkit_sensor_i2c_read(&params->sensor_ctrl, reg_pairs[reg_idx],
			reg_pairs + reg_idx + 1, CAMKIT_I2C_BYTE_DATA);
		if (ret) {
			LOG_ERR("read register fail: 0x%x\n", reg_pairs[reg_idx]);
			return;
		}
		LOG_DBG("[0x%x 0x%x]\n", reg_pairs[reg_idx], reg_pairs[reg_idx + 1]);
	}
}

static void set_pdaf_setting(struct camkit_sensor_params *params,
	uint16 *setting, uint32 reg_num)
{
	int32 ret;
	ret = camkit_i2c_write_table(&params->sensor_ctrl, setting,
		reg_num * 2, CAMKIT_I2C_BYTE_DATA);
	if (ret) {
		LOG_ERR("read register fail\n");
		return;
	}
}

static void get_fuse_id(struct camkit_sensor_params *params,
	uint8 *data_buf, uint32 reg_num)
{
	int32 rc;

	LOG_INF("ENTER\n");
	rc = camkit_sensor_i2c_process(&params->sensor_ctrl,
		&params->sensor_info.fuse_id_info, data_buf, reg_num);
	if (rc < 0)
		LOG_ERR("Failed\n");
	LOG_INF("EXIT\n");
}

static void get_mipi_trail_val(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	if (!params) {
		LOG_ERR("nullptr\n");
		return;
	}

	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			params->sensor_info.cap.mipi_trail_val;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			params->sensor_info.normal_video.mipi_trail_val;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			params->sensor_info.pre.mipi_trail_val;
		break;
	}
	LOG_INF("Mipi trail val: %llu\n", *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
}

static void get_awb_req_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	MUINT32 need_set_awb_gain;
	struct camkit_mode_info *mode_info = NULL;
	mode_info = get_sensor_mode_info(params, feature_data);
	if (!mode_info) {
		LOG_ERR("sensor mode info is NULL, set need_set_awb_gain 0\n");
		need_set_awb_gain = 0; /* default val */
	} else {
		need_set_awb_gain = mode_info->need_set_awb_gain;
	}

	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = need_set_awb_gain;

	LOG_INF("need_set_awb_gain: %u\n", need_set_awb_gain);
}

static u32 mipi_mask_enabled(void)
{
	u32 enabled = 0;
	int ret;
	struct device_node *of_node = of_find_node_by_name(NULL, "product_name_camera");
	if (!of_node) {
		LOG_ERR("not config product_name_camera node\n");
		return 0;
	}

	ret = of_property_read_u32(of_node, "cam-mipi-mask-enabled", &enabled);
	LOG_INF("mipi mask enabled = %u\n",enabled);
	if (ret < 0) {
		LOG_INF("not support mipi mask\n");
		return 0;
	}
	LOG_INF("sensor mipi mask setting value is %d\n", enabled);
	return enabled;
}

static void get_mipi_mask_val(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	int mipi_enable = mipi_mask_enabled();
	if (!mipi_enable) {
		LOG_INF("mipi mask is not enable\n");
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		return;
	}
	if (!params) {
		LOG_ERR("get_mipi_mask_val params is nullptr\n");
		return;
	}
	switch (*feature_data) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
		params->sensor_info.cap.mipi_mask_val;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
		params->sensor_info.normal_video.mipi_mask_val;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
		params->sensor_info.pre.mipi_mask_val;
		break;
	}
	LOG_DBG("Mipi mask val: %llu\n", *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
}

static void get_seamless_scenarios(struct camkit_sensor_params *params,
	unsigned long long *feature_data, uint32 *feature_para_len)
{
	MUINT32 *scenarios = NULL;
	uint32 i = 0;
	uint32 count = 0;
	struct camkit_seamless_cfg_t *seamless_cfg = NULL;
	struct camkit_seamless_scenario_id_t *id_map = NULL;
	camkit_scenario_type src_scenario_id;

	/* as mtk api definition, feature data array size >= 3 */
	if (!feature_para_len || (*feature_para_len < 3)) {
		LOG_ERR("feature data len error\n");
		return;
	}

	if (!feature_data || !(feature_data + 1)) { /* as mtk cmd struct */
		LOG_ERR("feature_data is null\n");
		return;
	}

	src_scenario_id = (camkit_scenario_type)(*feature_data);
	LOG_INF("seamless src scenario id %d\n", src_scenario_id);

	seamless_cfg = &params->sensor_info.seamless_cfg;
	if (seamless_cfg->seamless_support != CAMKIT_SEAMLESS_SUPPORT) {
		LOG_INF("not support to get seamless scenario, just return\n");
		return;
	}

	if (!seamless_cfg->scenario_id_map ||
		seamless_cfg->scenario_id_map_size == 0) {
		LOG_ERR("invalid scenario id map\n");
		return;
	}

	id_map = seamless_cfg->scenario_id_map;
	/* MUINT32 scenerios[SEAMLESS_SCENARIO_ID_SIZE] */
	scenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
	while (i < seamless_cfg->scenario_id_map_size &&
		count < SEAMLESS_SCENARIO_ID_SIZE) {
		if (id_map[i].src_scenario_id == src_scenario_id) {
			scenarios[count] = id_map[i].dst_scenario_id;
			LOG_INF("seamless add dst scenario id %u\n",
				scenarios[count]);
			count++;
		}
		i++;
	}
}

static uint32 set_seamless_setting_by_scenario(
	struct camkit_params *kit_params,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	struct camkit_i2c_reg_setting *fast_seamless_setting)
{
	int32 rc;
	uint8 mode = CAMKIT_MODE_PREVIEW; /* default value */
	struct camkit_mode_info *mode_info = NULL;
	struct camkit_i2c_reg_setting *setting = NULL;
	struct camkit_sensor_params *params = kit_params->sensor_params;

	LOG_INF("enter set_seamless_setting_by_scenario, scenario_id %d\n",
		scenario_id);

	get_setting_info_by_scenario(params, scenario_id,
		&mode_info, &setting, &mode);
	if (!mode_info || !setting) {
		LOG_ERR("get settings info by scenario failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}

	update_sensor_ctrl(params, mode, mode_info);
	if (fast_seamless_setting) {
		LOG_INF("using fast seamless settings\n");
		setting = fast_seamless_setting;
	}
	rc = camkit_sensor_write_setting(&params->sensor_ctrl, setting);
	if (rc < 0) {
		camkit_hiview_handle(rc, DSM_CAMERA_I2C_ERR,
			kit_params->module_params->sensor_name, SET_RESOLUTION_SETTING_FAILED);
		LOG_ERR("Write sensorMode[%u] settings Failed\n", mode);
		return ERROR_DRIVER_INIT_FAIL;
	}

	LOG_INF("success finish seamless settings\n");
	return ERROR_NONE;
}

static struct camkit_i2c_reg_setting *get_seamless_settings_by_scenario(
	struct camkit_seamless_cfg_t *seamless_cfg,
	enum MSDK_SCENARIO_ID_ENUM scenario_id)
{
	uint32 i;

	if (!seamless_cfg) {
		LOG_ERR("seamless_cfg invalid\n");
		return NULL;
	}

	if (!seamless_cfg->scenario_cfg ||
		seamless_cfg->scenario_cfg_size == 0) {
		LOG_INF("seamless scenario cfg settings may not be used, it is ok\n");
		return NULL;
	}

	for (i = 0; i < seamless_cfg->scenario_cfg_size; ++i) {
		if (seamless_cfg->scenario_cfg[i].dst_scenario_id ==
			(camkit_scenario_type)scenario_id) {
			LOG_INF("success find dst scenario seamless settings, i = %u\n", i);
			return &seamless_cfg->scenario_cfg[i].seamless_setting;
		}
	}

	LOG_ERR("failed to find dst scenario seamless settings\n");
	return NULL;
}

static void set_seamless_switch(struct camkit_params *kit_params,
	unsigned long long *feature_data, uint32 *feature_para_len)
{
	struct camkit_sensor_params *params = NULL;
	struct camkit_seamless_cfg_t *seamless_cfg = NULL;
	enum MSDK_SCENARIO_ID_ENUM dst_scenario_id;
	struct camkit_i2c_reg_setting *fast_seamless_settings = NULL;
	uint32 *ae_ctrl = NULL;
	uint32 shutter = 0;
	uint32 gain = 0;

	/* as mtk api definition, feature data array size >= 3 */
	if (!feature_para_len || (*feature_para_len < 3)) {
		LOG_ERR("feature data len error\n");
		return;
	}

	if (!feature_data || !(feature_data + 1)) { /* as mtk cmd struct */
		LOG_ERR("feature_data is null\n");
		return;
	}

	dst_scenario_id = (enum MSDK_SCENARIO_ID_ENUM)(*feature_data);
	params = kit_params->sensor_params;
	seamless_cfg = &params->sensor_info.seamless_cfg;
	if (seamless_cfg->seamless_support != CAMKIT_SEAMLESS_SUPPORT) {
		LOG_INF("not support seamless switch, just return\n");
		return;
	}

	fast_seamless_settings = get_seamless_settings_by_scenario(
		seamless_cfg, dst_scenario_id);

	set_seamless_setting_by_scenario(kit_params, dst_scenario_id,
		fast_seamless_settings);

	/* MUINT32 ae_ctrl[16] */
	ae_ctrl = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
	if (ae_ctrl && (ae_ctrl + GAIN_TABLE_OFFSET)) {
		shutter = *ae_ctrl;
		gain = *(ae_ctrl + GAIN_TABLE_OFFSET);
	}

	if (shutter != 0)
		set_shutter(params, shutter);
	if (gain != 0)
		set_gain(params, (uint16)(gain & 0xFFFF));

	LOG_INF("success set seamless cfg, dst scenario id %d, shutter %u, gain %u\n",
		dst_scenario_id, shutter, gain);
}

static void get_stagger_hdr_capacity_by_scenario(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	uint32 i = 0;
	struct camkit_stagger_cfg_t *stagger_cfg = NULL;
	struct camkit_stagger_type_id_t *stagger_type_map = NULL;
	camkit_scenario_type scenario_id;

	if (!feature_data || !(feature_data + 1)) { /* as mtk cmd struct */
		LOG_ERR("feature_data is null\n");
		return;
	}

	scenario_id = (camkit_scenario_type)(*feature_data);
	stagger_cfg = &params->sensor_info.stagger_cfg;
	if (stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("not support stagger, just return\n");
		return;
	}

	if (!stagger_cfg->stagger_type_map ||
		stagger_cfg->stagger_type_map_size == 0) {
		LOG_ERR("invalid stagger type map\n");
		return;
	}

	stagger_type_map = stagger_cfg->stagger_type_map;
	while (i < stagger_cfg->stagger_type_map_size) {
		if (stagger_type_map[i].scenario_id == scenario_id) {
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				stagger_type_map[i].stagger_hdr_type;
			LOG_INF("stagger hdr capacity scenarioId:%llu, stagger_hdr_type:%u\n",
				*feature_data, stagger_type_map[i].stagger_hdr_type);
			break;
		}
		i++;
	}
	if (i == stagger_cfg->stagger_type_map_size) {
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = CAMKIT_HDR_NONE;
		LOG_INF("default set hdr none:%u\n", *feature_data);
	}
}

static void get_stagger_target_scenarios(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	uint32 i = 0;
	struct camkit_stagger_cfg_t *stagger_cfg = NULL;
	struct camkit_stagger_scenario_id_t *id_map = NULL;
	camkit_scenario_type src_scenario_id;
	camkit_hdr_mode_t hdr_type;

	if (!feature_data || !(feature_data + 1) || !(feature_data + 2)) {
		LOG_ERR("feature_data is null\n");
		return;
	}

	src_scenario_id = (camkit_scenario_type)(*feature_data);
	hdr_type = (camkit_hdr_mode_t)(*(feature_data + 1));
	LOG_INF("stagger src scenario id %d\n", src_scenario_id);

	stagger_cfg = &params->sensor_info.stagger_cfg;
	if (stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("not support stagger, just return\n");
		return;
	}

	if (!stagger_cfg->scenario_id_map ||
		stagger_cfg->scenario_id_map_size == 0) {
		LOG_ERR("invalid scenario id map\n");
		return;
	}

	id_map = stagger_cfg->scenario_id_map;
	while (i < stagger_cfg->scenario_id_map_size) {
		if (id_map[i].src_scenario_id == src_scenario_id &&
			id_map[i].hdr_type == hdr_type) {
			*(feature_data + 2) = id_map[i].dst_scenario_id;
			LOG_INF("stagger dst scenario id %u\n",
				id_map[i].dst_scenario_id);
			break;
		}
		i++;
	}
}

static void get_stagger_max_expo(struct camkit_sensor_params *params,
	unsigned long long *feature_data)
{
	uint32 i = 0;
	struct camkit_stagger_cfg_t *stagger_cfg = NULL;
	struct camkit_stagger_max_expo_t *stagger_max_expo = NULL;
	camkit_vc_feature_t vc_feature;

	if (!feature_data || !(feature_data + 1) || !(feature_data + 2)) {
		LOG_ERR("feature_data is null\n");
		return;
	}

	vc_feature = (camkit_vc_feature_t)(*(feature_data + 1));
	LOG_INF("hdr mode %d\n", vc_feature);

	stagger_cfg = &params->sensor_info.stagger_cfg;
	if (stagger_cfg->hdr_support != CAMKIT_HDR_SUPPORT_STAGGER_FDOL) {
		LOG_INF("not support stagger, just return\n");
		return;
	}

	if (!stagger_cfg->stagger_max_expo ||
		stagger_cfg->stagger_max_expo_size == 0) {
		LOG_ERR("invalid stagger hdr mode\n");
		return;
	}

	stagger_max_expo = stagger_cfg->stagger_max_expo;
	while (i < stagger_cfg->stagger_max_expo_size) {
		if (stagger_max_expo[i].vc_feature == vc_feature) {
			*(feature_data + 2) = stagger_max_expo[i].max_expo;
			LOG_INF("max expo time %u\n",
				stagger_max_expo[i].max_expo);
			break;
		}
		i++;
	}
}


static uint32 feature_control(struct camkit_params *kit_params,
	MSDK_SENSOR_FEATURE_ENUM feature_id,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 rc = ERROR_NONE;
	struct camkit_sensor_params *params = NULL;
	uint16 *feature_return_para_16 = (uint16 *)feature_para;
	uint16 *feature_data_16 = (uint16 *)feature_para;
	uint32 *feature_return_para_32 = (uint32 *)feature_para;
	uint32 *feature_data_32 = (uint32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *win_info = NULL;

	struct SET_SENSOR_AWB_GAIN *set_sensor_awb =
		(struct SET_SENSOR_AWB_GAIN *) feature_para;

	if (!kit_params || !kit_params->sensor_params ||
		!kit_params->module_params ||
		!feature_para || !feature_para_len) {
		LOG_ERR("Fatal null. params:%pK,feature_para:%pK,feature_para_len:%pK\n",
			kit_params, feature_para, feature_para_len);
		return ERROR_NONE;
	}
	params = kit_params->sensor_params;
	LOG_DBG("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = params->aec_info.min_gain;
		*(feature_data + 2) = params->aec_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = params->aec_info.min_iso;
		*(feature_data + 1) = params->aec_info.gain_step;
		*(feature_data + 2) = params->aec_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		get_min_shutter_by_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		get_pclk_by_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		get_period_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = params->sensor_ctrl.line_length;
		*feature_return_para_16 = params->sensor_ctrl.frame_length;
		*feature_para_len = 4; /* return 4 byte data */
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = params->sensor_ctrl.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(params, (uint32)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(params, (uint16)*feature_data);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM */
		/* or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(params, (uint16)*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_match_id(kit_params, feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(params, (bool)*feature_data_16,
			*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(params,
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			(uint32)*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(params,
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(uint32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA. No support\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		(void)set_test_pattern_mode(params, (bool)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = params->sensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&camkit_lock);
		params->sensor_ctrl.current_fps = (uint16)*feature_data_32;
		spin_unlock(&camkit_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		win_info = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data + 1));
		get_sensor_crop_info(params, *feature_data_32, win_info);
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		rc = streaming_control(params, FALSE);
		if (rc != ERROR_NONE)
			LOG_ERR("stream off failed\n");
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(params, (uint32)*feature_data);
		rc = streaming_control(params, TRUE);
		if (rc != ERROR_NONE)
			LOG_ERR("stream on failed\n");
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		get_pdaf_info(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		get_pdaf_capacity(params, feature_data, *feature_para_len);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(params, (uint32)*feature_data,
			(uint32)(*(feature_data + 1)), &(params->aec_info.expo_ops_map));
		break;
	case SENSOR_HUAWEI_FEATURE_DUMP_REG:
		sensor_dump_reg(params);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		get_sensor_mipi_pixel_rate(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		get_sensor_pixel_rate(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		get_binning_type(params, feature_data, feature_return_para_32);
		LOG_INF("binning type ratio %d\n", (*feature_return_para_32));
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (uint16)*feature_data);
		get_vc_info(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("get pdaf setting size %d\n", (*feature_para_len));
		get_pdaf_regs_data(params, feature_data_16,
			(*feature_para_len) / sizeof(uint32));
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		LOG_INF("set pdaf setting size %d\n", (*feature_para_len));
		set_pdaf_setting(params, feature_data_16,
			(*feature_para_len) / sizeof(uint32));
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		params->sensor_ctrl.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_GET_FUSE_ID:
		LOG_INF("Get Fuse id\n");
		get_fuse_id(params, feature_para, *feature_para_len);
		break;
	case SENSOR_FEATURE_GET_MIPI_TRAIL_VAL:
		get_mipi_trail_val(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		get_awb_req_by_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		set_awb_gain(params, set_sensor_awb);
		break;
	case SENSOR_FEATURE_GET_MIPI_MASK_VAL:
		get_mipi_mask_val(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		get_seamless_scenarios(params, feature_data, feature_para_len);
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
		set_seamless_switch(kit_params, feature_data, feature_para_len);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		get_stagger_hdr_capacity_by_scenario(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_VC_INFO2:
		get_vc_info2(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
		get_stagger_target_scenarios(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
		get_stagger_max_expo(params, feature_data);
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		(*(feature_data + 1)) = 1;
		(*(feature_data + 2)) = params->aec_info.vts_offset;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		if (!(feature_data + 1)) {
			LOG_ERR("fatal hdr shutter para\n");
			return ERROR_NONE;
		}
		hdr_set_shutter(params, (uint32)(*feature_data), (uint32)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN:
		if (!(feature_data + 1)) {
			LOG_ERR("fatal hdr gain para\n");
			return ERROR_NONE;
		}
		hdr_set_gain(params, (uint16)(*feature_data), (uint16)(*(feature_data + 1)));
		break;
	default:
		LOG_INF("Not support the feature_id:%d\n", feature_id);
		break;
	}

	return ERROR_NONE;
}

void camkit_hiview_report(int error_no, const char *ic_name,
	driver_error_type error_type)
{
	errno_t ret;
	struct hwcam_hievent_info driver_info;
	const char* error_info = NULL;

	ret = memset_s(&driver_info, sizeof(driver_info),
		0, sizeof(driver_info));
	if (ret != EOK) {
		LOG_ERR("memset_s fail");
		return;
	}

	if (error_type >= DRIVER_ERROR_TYPE_START &&
		error_type < DRIVER_ERROR_TYPE_END)
		error_info = g_driver_error_info[error_type];

	driver_info.error_no = error_no;
	hwcam_hiview_get_ic_name(ic_name, &driver_info);
	hwcam_hiview_get_content(error_no, error_info, &driver_info);
	hwcam_hiview_report(&driver_info);
}

static struct sensor_kit_ops sensor_driver_ops = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

uint32 sensor_driver_init(struct sensor_kit_ops **ops)
{
	LOG_INF("%s ops:%pK\n", __func__, ops);
	if (ops != NULL)
		*ops = &sensor_driver_ops;

	return ERROR_NONE;
}
