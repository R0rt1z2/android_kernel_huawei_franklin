/*
 * gc8054_byd_wkg.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * customized sensor driver
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
#include "camkit_sensor_i2c.h"

#define PFX "[sensorkit]"
#define DEBUG_SENSOR_KIT 1
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_SENSOR_KIT) \
			pr_info(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

#define RETRY_TIMES 2

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
	uint8 i2c_addr = 0xAC;
	uint16 module_addr = 0x0005;

	rc = camkit_i2c_read(i2c_addr, module_addr, CAMKIT_I2C_WORD_ADDR,
		&module_code, CAMKIT_I2C_BYTE_DATA);
	if (rc == ERR_NONE) {
		log_info("deepcamera mismatched, read module code: 0x%x ", module_code);
		return ERR_IO;
	} else {
		log_info("deepcamera do not need to read module code");
		return ERR_NONE;
	}
}

uint32 c585gfi_wn_match_id(struct camkit_sensor *sensor,
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

	*match_id = kit_params->module_params->match_id;
	log_info("match id ok, sensor id: 0x%x, module: 0x%x, match id: 0x%x",
		kit_params->sensor_params->sensor_info.sensor_id,
		kit_params->module_params->module_code,
		kit_params->module_params->match_id);

	return ERR_NONE;
}

static struct sensor_kit_ops c585gfi_wn_ops = {
	.sensor_open = camkit_open,
	.sensor_close = camkit_close,
	.match_id = c585gfi_wn_match_id,
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

uint32 get_c585gfi_wn_ops(struct sensor_kit_ops **ops)
{
	if (ops != NULL) {
		*ops = &c585gfi_wn_ops;
	} else {
		LOG_ERR("get c585gfi_wn_ops operators fail");
		return ERR_INVAL;
	}

	LOG_INF("get c585gfi_wn operators OK");
	return ERR_NONE;
}

register_customized_driver(
	c585gfi_wn,
	CAMKIT_SENSOR_IDX_MAIN2,
	C585GFI_WN_SENSOR_ID,
	get_c585gfi_wn_ops);
