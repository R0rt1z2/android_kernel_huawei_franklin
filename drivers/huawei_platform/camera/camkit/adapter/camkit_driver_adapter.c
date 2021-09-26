/*
 * camkit_driver_adapter.c
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: camkit interface adapted on the platform
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

#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"
#ifdef IMGSENSOR_OC_ENABLE
#include "imgsensor_oc.h"
#endif
#include "imgsensor.h"

#ifdef CONFIG_MTK_CCU
#include "ccu_imgsensor_if.h"
#endif

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
#include "imgsensor_ca.h"
#endif

#include <securec.h>
#include "kd_camkit_define.h"

#include "camkit_driver_adapter.h"
#include "camkit_driver_interface.h"
#include "camkit_probe_sensor.h"

#include "camkit_dts_adapter.h"

extern struct IMGSENSOR gimgsensor;

extern void IMGSENSOR_PROFILE_INIT(struct timeval *ptv);
extern void IMGSENSOR_PROFILE(struct timeval *ptv, char *tag);
extern void imgsensor_mutex_init(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_lock(struct IMGSENSOR_SENSOR_INST *psensor_inst);
extern void imgsensor_mutex_unlock(struct IMGSENSOR_SENSOR_INST *psensor_inst);

struct feature_info {
	MSDK_SENSOR_FEATURE_ENUM feature_id;
	uint32 (*feature_ops)(uint32 sensor_idx,
		uint8 *feature_para, uint32 *feature_para_len);
};

static struct IMGSENSOR_SENSOR *get_image_sensor(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
		idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return NULL;
	else
		return &gimgsensor.sensor[idx];
}

int adopt_camera_enable_camkit_log(void *buf)
{
	uint32 *log_mask = (uint32 *)buf;

	if (log_mask == NULL) {
		log_err("[%s] NULL arg", __func__);
		return ERR_INVAL;
	}

	return hwsensor_set_log_mask(*log_mask);
}

static enum IMGSENSOR_RETURN camkit_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr = NULL;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg = NULL;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info = NULL;
	int i, j;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i])(&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(phw->pdev[i]->pinstance);
	}

	pcust_pwr_cfg = get_sensor_hw_cfg();
	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

		ppwr_info = pcust_pwr_cfg[i].pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0; j < IMGSENSOR_HW_ID_MAX_NUM; j++)
				if (ppwr_info->id == phw->pdev[j]->id)
					break;

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

int adopt_camera_hw_probe_sensor(void *buf)
{
	uint32 ret;
	static uint32 open_flag;

	struct camkit_probe_sensor_params *user_params = NULL;
	struct IMGSENSOR_SENSOR               *psensor = NULL;
	struct IMGSENSOR_SENSOR_INST      *sensor_inst = NULL;
	struct camkit_params          *user_kit_params = NULL;
	struct IMGSENSOR                   *pimgsensor = &gimgsensor;
	struct IMGSENSOR_HW                       *phw = &pimgsensor->hw;

	if (open_flag == 0) {
		log_info("init hw again");
		camkit_hw_init(phw);
		open_flag = 1;
	}

	pimgsensor->camkit_enabled = 1;
	log_info("camkit_enabled = %d", pimgsensor->camkit_enabled);

	user_params = (struct camkit_probe_sensor_params *)buf;
	if (user_params == NULL) {
		log_err("[%s] NULL arg", __func__);
		return -EFAULT;
	}

	user_kit_params = user_params->kit_params;
	if (user_kit_params == NULL) {
		log_err("[%s] camkit params from user is NULL", __func__);
		return -EFAULT;
	}

	log_info("sensor index = %d", user_params->sensor_idx);
	psensor = get_image_sensor(user_params->sensor_idx);
	if (psensor == NULL) {
		log_err("[%s] NULL psensor", __func__);
		return -EFAULT;
	}
	psensor->inst.sensor_idx = user_params->sensor_idx;

	sensor_inst = &psensor->inst;
	imgsensor_mutex_init(sensor_inst);

	imgsensor_i2c_init(&sensor_inst->i2c_cfg,
		get_sensor_i2c_dev(psensor->inst.sensor_idx));
	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, true);

	imgsensor_mutex_lock(&psensor->inst);
	ret = hwsensor_probe_sensor(user_params->sensor_idx, user_kit_params);
	if (ret == ERR_NONE)
		user_params->probe_succeed = 1;
	imgsensor_mutex_unlock(&psensor->inst);

	imgsensor_i2c_filter_msg(&sensor_inst->i2c_cfg, false);

	return ERR_NONE;
}

int32 adopt_sensor_open(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERR_NONE;
	struct command_params c_params = {0};
	struct IMGSENSOR             *pimgsensor   = &gimgsensor;
#endif
	struct IMGSENSOR_SENSOR_INST *psensor_inst = NULL;

#ifdef CONFIG_MTK_CCU
	struct ccu_sensor_info ccu_sinfo;
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	struct i2c_client *pi2c_client = NULL;
#endif

	uint32 sensor_index;

	return_err_if_null(psensor);
	psensor_inst = &psensor->inst;
	sensor_index = (uint32)psensor->inst.sensor_idx;

	IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

	imgsensor_mutex_lock(psensor_inst);

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	PK_INFO("%s secure state %d", __func__,
	(int)(&gimgsensor)->imgsensor_sec_flag);
	if ((&gimgsensor)->imgsensor_sec_flag)
		ret = imgsensor_ca_invoke_command(
			IMGSENSOR_TEE_CMD_OPEN, c_params, &ret_sec);
	else
		ret = hwsensor_open(sensor_index);
#else
	ret = hwsensor_open(sensor_index);
#endif
	if (ret != ERR_NONE)
		log_err("sensor_open fail");
	else
		psensor_inst->state = IMGSENSOR_STATE_OPEN;

#ifdef IMGSENSOR_OC_ENABLE
	if (ret == ERR_NONE)
		imgsensor_oc_interrupt(IMGSENSOR_HW_POWER_STATUS_ON);
#endif

#ifdef CONFIG_MTK_CCU
#if (DRIVER_VERSION > 1)
	ccu_sinfo.slave_addr =
	    (psensor_inst->i2c_cfg.msg->addr << 1);
#else
	ccu_sinfo.slave_addr =
		(psensor_inst->i2c_cfg.pinst->msg->addr << 1);
#endif
	ccu_sinfo.sensor_name_string = hwsensor_get_sensor_name(sensor_index);
	pi2c_client = psensor_inst->i2c_cfg.pinst->pi2c_client;
	if (pi2c_client)
		ccu_sinfo.i2c_id = (((struct mt_i2c *)
			i2c_get_adapdata(pi2c_client->adapter))->id);
	else
		ccu_sinfo.i2c_id = -1;
	ccu_set_sensor_info(sensor_idx, &ccu_sinfo);
#endif

	imgsensor_mutex_unlock(psensor_inst);

	IMGSENSOR_PROFILE(&psensor_inst->profile_time, "sensor_open");

	return ret;
}

static uint32 fill_mtk_sensor_info(uint32 scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	struct camkit_sensor_info_t *kit_sensor_info)
{
	struct camkit_mode_info *mode_info = NULL;

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = kit_sensor_info->sensor_interface_type;
	sensor_info->MIPIsensorType = kit_sensor_info->mipi_sensor_type;
	sensor_info->SettleDelayMode = kit_sensor_info->mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		kit_sensor_info->sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = kit_sensor_info->cap_delay_frame;
	sensor_info->PreviewDelayFrame = kit_sensor_info->pre_delay_frame;
	sensor_info->VideoDelayFrame = kit_sensor_info->video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		kit_sensor_info->hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		kit_sensor_info->slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		kit_sensor_info->slim_video_delay_frame;
	sensor_info->Custom1DelayFrame =
		kit_sensor_info->custom1_delay_frame;
	sensor_info->Custom2DelayFrame =
		kit_sensor_info->custom2_delay_frame;
	sensor_info->Custom3DelayFrame =
		kit_sensor_info->custom3_delay_frame;
	sensor_info->Custom4DelayFrame =
		kit_sensor_info->custom4_delay_frame;
	sensor_info->Custom5DelayFrame =
		kit_sensor_info->custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = kit_sensor_info->isp_driving_current;

	sensor_info->AEShutDelayFrame = kit_sensor_info->ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		kit_sensor_info->ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		kit_sensor_info->ae_isp_gain_delay_frame;
	sensor_info->IHDR_Support = kit_sensor_info->ihdr_support;
	sensor_info->IHDR_LE_FirstLine = kit_sensor_info->ihdr_le_firstline;
	sensor_info->SensorModeNum = kit_sensor_info->sensor_mode_num;
	sensor_info->PDAF_Support = kit_sensor_info->pdaf_support;

	sensor_info->SensorMIPILaneNumber = kit_sensor_info->mipi_lane_num;
	sensor_info->SensorClockFreq = kit_sensor_info->mclk;
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
		mode_info = &(kit_sensor_info->pre);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		mode_info = &(kit_sensor_info->cap);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		mode_info = &(kit_sensor_info->normal_video);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		mode_info = &(kit_sensor_info->hs_video);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		mode_info = &(kit_sensor_info->slim_video);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		mode_info = &(kit_sensor_info->custom1);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		mode_info = &(kit_sensor_info->custom2);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		mode_info = &(kit_sensor_info->custom3);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		mode_info = &(kit_sensor_info->custom4);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		mode_info = &(kit_sensor_info->custom5);
		break;
	default:
		mode_info = &(kit_sensor_info->pre);
		break;
	}

	sensor_info->SensorGrabStartX = mode_info->startx;
	sensor_info->SensorGrabStartY = mode_info->starty;
	sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			mode_info->mipi_data_lp2hs_settle_dc;

	return ERR_NONE;
}

uint32 adopt_sensor_get_info(
	struct IMGSENSOR_SENSOR *psensor,
	uint32 scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	uint32 ret;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = NULL;
	struct camkit_sensor_info_t kit_sensor_info = {0};

	return_err_if_null(psensor);
	return_err_if_null(sensor_info);
	return_err_if_null(sensor_config_data);

	psensor_inst = &psensor->inst;

	imgsensor_mutex_lock(psensor_inst);

	ret = hwsensor_get_sensor_info((uint32)psensor->inst.sensor_idx,
		&kit_sensor_info);
	if (ret != ERR_NONE)
		log_err("get sensor info failed");

	ret = fill_mtk_sensor_info(scenario_id, sensor_info, &kit_sensor_info);
	if (ret != ERR_NONE)
		log_err("fill_mtk_sensor_info failed");

	imgsensor_mutex_unlock(psensor_inst);

	return ret;
}

uint32 adopt_sensor_get_resolution(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	uint32 ret;
	struct IMGSENSOR_SENSOR_INST *psensor_inst = NULL;
	struct camkit_sensor_info_t sensor_info = {0};

	return_err_if_null(psensor);
	return_err_if_null(sensor_resolution);

	psensor_inst = &psensor->inst;

	imgsensor_mutex_lock(psensor_inst);

	ret = hwsensor_get_sensor_info((uint32)psensor->inst.sensor_idx,
		&sensor_info);
	if (ret != ERR_NONE)
		log_err("get sensor resolution failed");

	log_info("cap grabwindow_width:%u", sensor_info.cap.grabwindow_width);
	log_info("cap grabwindow_height:%u", sensor_info.cap.grabwindow_height);

	log_info("pre grabwindow_width:%u", sensor_info.pre.grabwindow_width);
	log_info("pre grabwindow_height:%u", sensor_info.pre.grabwindow_height);

	sensor_resolution->SensorFullWidth =
		sensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		sensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		sensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		sensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		sensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		sensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		sensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		sensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		sensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		sensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		sensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		sensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		sensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		sensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		sensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		sensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width =
		sensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		sensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width =
		sensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =
		sensor_info.custom5.grabwindow_height;

	imgsensor_mutex_unlock(psensor_inst);

	return ret;
}

uint32 adopt_sensor_control(
	struct IMGSENSOR_SENSOR *psensor,
	enum MSDK_SCENARIO_ID_ENUM scenario_id)
{
	uint32 ret;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	int32 ret_sec = ERROR_NONE;
	struct command_params c_params;
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT image_window;
	MSDK_SENSOR_CONFIG_STRUCT sensor_config_data;
#endif
	struct IMGSENSOR_SENSOR_INST *psensor_inst = NULL;

	return_err_if_null(psensor);
	psensor_inst = &psensor->inst;

	IMGSENSOR_PROFILE_INIT(&psensor_inst->profile_time);

	imgsensor_mutex_lock(psensor_inst);

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
		ret = hwsensor_control((uint32)psensor->inst.sensor_idx, scenario_id);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	}
#endif
	if (ret != ERR_NONE)
		log_err("sensor control fail");

	imgsensor_mutex_unlock(psensor_inst);

	IMGSENSOR_PROFILE(&psensor_inst->profile_time, "SensorControl");

	return ret;
}

static uint32 feature_default_ops(uint32 feature_id,
	__attribute__((unused))uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	log_info("feature_id:%d no implement, please check", feature_id);

	return ERR_NONE;
}

#if (DRIVER_VERSION > 1)
static uint32 feature_get_gain_range(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 min_gain = 0;
	uint32 max_gain = 0;

	ret = hwsensor_get_gain_range(sensor_idx, &min_gain, &max_gain);
	if (ret != ERR_NONE)
		log_err("get gain range fail");

	*(feature_data + 1) = min_gain;
	*(feature_data + 2) = max_gain;

	return ERR_NONE;
}

static uint32 feature_get_gain_info(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 min_iso = 0;
	uint32 gain_step = 0;
	uint32 gain_type = 0;

	ret = hwsensor_get_gain_info(sensor_idx, &min_iso, &gain_step, &gain_type);
	if (ret != ERR_NONE)
		log_err("get gain info fail");

	*(feature_data + 0) = min_iso;
	*(feature_data + 1) = gain_step;
	*(feature_data + 2) = gain_type;

	return ERR_NONE;
}

static uint32 feature_get_min_shutter(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 min_shutter = 0;

	ret = hwsensor_get_min_shutter(sensor_idx, &min_shutter);
	if (ret != ERR_NONE)
		log_err("get min shutter fail");

	*(feature_data + 1) = min_shutter;

	return ERR_NONE;
}
#endif

static uint32 feature_get_period(uint32 sensor_idx, uint8 *feature_para,
	uint32 *feature_para_len)
{
	uint32 ret;
	uint16 *feature_data = (uint16 *)feature_para;
	uint16 line_length = 0;
	uint16 frame_length = 0;

	ret = hwsensor_get_period(sensor_idx, &line_length, &frame_length);
	if (ret != ERR_NONE)
		log_err("get min shutter fail");

	*feature_data = line_length;
	*(feature_data + 1) = frame_length;
	*feature_para_len = 4; /* line_length: 2byte, frame_length: 2byte */

	return ERR_NONE;
}

static uint32 feature_get_pclk(uint32 sensor_idx, uint8 *feature_para,
	uint32 *feature_para_len)
{
	uint32 ret;
	uint32 *feature_data = (uint32 *)feature_para;
	uint32 pclk = 0;

	ret = hwsensor_get_pclk(sensor_idx, &pclk);
	if (ret != ERR_NONE)
		log_err("get min shutter fail");

	*feature_data = pclk;
	*feature_para_len = sizeof(pclk);

	return ERR_NONE;
}

static uint32 feature_get_checksum(uint32 sensor_idx, uint8 *feature_para,
	uint32 *feature_para_len)
{
	uint32 ret;
	uint32 *feature_data = (uint32 *)feature_para;
	uint32 checksum = 0;

	ret = hwsensor_get_checksum(sensor_idx, &checksum);
	if (ret != ERR_NONE)
		log_err("get min shutter fail");

	*feature_data = checksum;
	*feature_para_len = 4; /* 4:the size of checksum */

	return ERR_NONE;
}

static uint32 feature_get_scenario_pclk(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 pclk = 0;

	ret = hwsensor_get_scenario_pclk(sensor_idx, scenario_id, &pclk);
	if (ret != ERR_NONE)
		log_err("get scenario pclk fail");

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = pclk;
	log_info("scenario_id: %u pclk: %u", scenario_id, pclk);

	return ERR_NONE;
}

static uint32 feature_get_scenario_period(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 period = 0;

	ret = hwsensor_get_scenario_period(sensor_idx, scenario_id, &period);
	if (ret != ERR_NONE)
		log_err("get scenario period fail");

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = period;
	log_info("scenario_id: %u period: %u", scenario_id, period);

	return ERR_NONE;
}

static uint32 feature_set_shutter(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 shutter = (uint32)*feature_data;

	if (shutter == 0) {
		log_err("shutter value incorrect");
		return ERR_NONE;
	}

	ret = hwsensor_set_shutter(sensor_idx, shutter);
	if (ret != ERR_NONE)
		log_err("set shutter fail");

	log_info("set shutter: %u done", shutter);
	return ERR_NONE;
}

static uint32 feature_set_gain(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint16 gain = (uint16)*feature_data;

	ret = hwsensor_set_gain(sensor_idx, gain);
	if (ret != ERR_NONE)
		log_err("set gain fail");

	log_info("set gain: %u done", gain);
	return ERR_NONE;
}

static uint32 feature_set_video_mode(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint16 framerate = (uint16)*feature_data;

	ret = hwsensor_set_video_mode(sensor_idx, framerate);
	if (ret != ERR_NONE)
		log_err("set video mode fail");

	return ERR_NONE;
}

static uint32 feature_match_id(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	uint32 *match_id = (uint32 *)feature_para;

	ret = hwsensor_match_id(sensor_idx, match_id);
	if (ret != ERR_NONE)
		log_err("match id fail");

	return ERR_NONE;
}

static uint32 feature_set_auto_flicker(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	uint16 *feature_data = (uint16 *)feature_para;
	uint8 enable = *feature_data;
	uint16 framerate = *(feature_data + 1);

	ret = hwsensor_set_auto_flicker(sensor_idx, enable, framerate);
	if (ret != ERR_NONE)
		log_err("set auto flicker fail");

	return ERR_NONE;
}

static uint32 feature_set_current_fps(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	uint32 *feature_data = (uint32 *)feature_para;
	uint16 framerate = (uint16)*feature_data;

	ret = hwsensor_set_current_fps(sensor_idx, framerate);
	if (ret != ERR_NONE)
		log_err("set current fps fail");

	log_info("set current fps: %u done", framerate);
	return ERR_NONE;
}

static uint32 feature_streaming_control(uint32 sensor_idx,
	const uint8 enable)
{
	uint32 ret;

	ret = hwsensor_streaming_control(sensor_idx, enable);
	if (ret != ERR_NONE)
		log_err("streaming control fail: %d", enable);

	return ERR_NONE;
}

static uint32 feature_streaming_suspend(uint32 sensor_idx,
	__attribute__((unused))uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	return feature_streaming_control(sensor_idx, FALSE);
}

static uint32 feature_streaming_resume(uint32 sensor_idx,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 ret;
	ret = feature_set_shutter(sensor_idx, feature_para, feature_para_len);
	if (ret != ERR_NONE)
		log_err("feature_set_shutter failed");
	ret = feature_streaming_control(sensor_idx, TRUE);
	if (ret != ERR_NONE)
		log_err("feature_streaming_control failed");

	return ERR_NONE;
}

static uint32 feature_get_crop_info(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *mtk_win_info =
		(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
	struct camkit_sensor_output_info_t win_info = {0};
	uint32 win_info_len;
	uint32 mtk_win_info_len;

	win_info_len = sizeof(struct camkit_sensor_output_info_t);
	mtk_win_info_len = sizeof(struct SENSOR_WINSIZE_INFO_STRUCT);
	if (win_info_len != mtk_win_info_len) {
		log_err("mtk&camkit win info struct mismatch");
		return ERR_INVAL;
	}

	ret = hwsensor_get_crop_info(sensor_idx, scenario_id, &win_info);
	if (ret != ERR_NONE)
		log_err("get crop info fail");

	if (memcpy_s((void *)mtk_win_info, mtk_win_info_len,
		(void *)&win_info, win_info_len) != EOK) {
		log_err("memcpy_s win info failed");
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

static uint32 feature_get_pdaf_info(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	uint32 scenario_id = (uint32)*feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SET_PD_BLOCK_INFO_T *mtk_pdaf_info =
		(struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));
	struct camkit_sensor_pdaf_info_t pdaf_info = {0};

	uint32 pdaf_info_len;
	uint32 mtk_pdaf_info_len;

	pdaf_info_len = sizeof(struct camkit_sensor_pdaf_info_t);
	mtk_pdaf_info_len = sizeof(struct SET_PD_BLOCK_INFO_T);
	if (pdaf_info_len != mtk_pdaf_info_len) {
		log_err("mtk&camkit pdaf info struct mismatch");
		return ERR_INVAL;
	}

	ret = hwsensor_get_pdaf_info(sensor_idx, scenario_id, &pdaf_info);
	if (ret != ERR_NONE)
		log_err("get pdaf info fail");

	if (memcpy_s((void *)mtk_pdaf_info, mtk_pdaf_info_len,
		(void *)&pdaf_info, pdaf_info_len) != EOK) {
		log_err("memcpy_s pdaf info failed");
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

static uint32 feature_set_test_pattern(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint8 enable = *feature_data;

	ret = hwsensor_set_test_pattern(sensor_idx, enable);
	if (ret != ERR_NONE)
		log_err("set test pattern fail");

	return ERR_NONE;
}

static uint32 feature_dump_reg(uint32 sensor_idx,
	__attribute__((unused))uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;

	ret = hwsensor_dump_reg(sensor_idx);
	if (ret != ERR_NONE)
		log_err("dump reg fail");

	return ERR_NONE;
}

static uint32 feature_get_mipi_pixel_rate(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 mipi_pixel_rate = 0;

	ret = hwsensor_get_mipi_pixel_rate(sensor_idx,
		scenario_id, &mipi_pixel_rate);
	if (ret != ERR_NONE)
		log_err("get mipi pixel rate fail");

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mipi_pixel_rate;
	log_info("scenario: %u mipi_pixel_rate: %u", scenario_id, mipi_pixel_rate);

	return ERR_NONE;
}

static uint32 feature_get_mipi_trail_val(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 mipi_trail_val = 0;

	ret = hwsensor_get_mipi_trail_val(sensor_idx,
		scenario_id, &mipi_trail_val);
	if (ret != ERR_NONE)
		log_err("get mipi trail val fail");

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = mipi_trail_val;
	log_info("scenario: %u mipi_trail_val: %u", scenario_id, mipi_trail_val);

	return ERR_NONE;
}

static uint32 feature_get_sensor_pixel_rate(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 pixel_rate = 0;

	ret = hwsensor_get_sensor_pixel_rate(sensor_idx,
		scenario_id, &pixel_rate);
	if (ret != ERR_NONE)
		log_err("get mipi pixel rate fail");

	*(uint32 *)(uintptr_t)(*(feature_data + 1)) = pixel_rate;
	log_info("scenario: %u pixel_rate: %u", scenario_id, pixel_rate);

	return ERR_NONE;
}

static uint32 feature_set_pdaf_mode(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	uint16 pdaf_mode = (uint16)*feature_para;

	ret = hwsensor_set_pdaf_mode(sensor_idx, pdaf_mode);
	if (ret != ERR_NONE)
		log_err("set pdaf mode fail");

	log_info("set pdaf_mode: %u done", pdaf_mode);
	return ERR_NONE;
}

static uint32 feature_set_pdaf_setting(uint32 sensor_idx,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 ret;
	uint16 *setting = (uint16 *)feature_para;
	uint32 len = (*feature_para_len) / sizeof(uint32);

	ret = hwsensor_set_pdaf_setting(sensor_idx, setting, len);
	if (ret != ERR_NONE)
		log_err("set pdaf setting fail");

	return ERR_NONE;
}

static uint32 feature_get_pdaf_regs_data(uint32 sensor_idx,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 ret;
	uint16 *reg_pairs = (uint16 *)feature_para;
	uint32 num = (*feature_para_len) / sizeof(uint32);

	ret = hwsensor_get_pdaf_regs_data(sensor_idx, reg_pairs, num);
	if (ret != ERR_NONE)
		log_err("get pdaf regs data fail");

	return ERR_NONE;
}

static uint32 feature_get_vc_info(uint32 sensor_idx, uint8 *feature_para,
	__attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SENSOR_VC_INFO_STRUCT *mtk_vc_info = NULL;
	struct camkit_sensor_vc_info_t vc_info = {0};
	int mtk_vc_info_len;
	int camkit_vc_info_len;
	uint32 scenario_id = (uint32)*feature_data;

	mtk_vc_info_len = sizeof(struct SENSOR_VC_INFO_STRUCT);
	camkit_vc_info_len = sizeof(struct camkit_sensor_vc_info_t);

	if (mtk_vc_info_len != camkit_vc_info_len) {
		log_err("mtk&camkit vc info struct mismatch");
		return ERR_INVAL;
	}

	mtk_vc_info = (struct SENSOR_VC_INFO_STRUCT *)
		(uintptr_t) (*(feature_data + 1));

	ret = hwsensor_get_vc_info(sensor_idx, scenario_id, &vc_info);
	if (ret != ERR_NONE)
		log_err("get vc info fail");

	if (memcpy_s((void *)mtk_vc_info, mtk_vc_info_len,
		(void *)&vc_info, camkit_vc_info_len) != EOK) {
		log_err("memcpy_s vc_info failed");
		return ERR_NOMEM;
	}

	return ERR_NONE;
}

static uint32 feature_get_binning_ratio(uint32 sensor_idx,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*(feature_data + 1);
	uint32 *ratio = (uint32 *)feature_para;

	ret = hwsensor_get_binning_ratio(sensor_idx,
		scenario_id, ratio);
	if (ret != ERR_NONE)
		log_err("get binning ratio fail");

	*feature_para_len = 4;

	return ERR_NONE;
}

static uint32 feature_set_shutter_frame_length(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 shutter = (uint32)*feature_data;
	uint32 frame_length = (uint32)*(feature_data + 1);

	ret = hwsensor_set_shutter_frame_length(sensor_idx,
		shutter, frame_length);
	if (ret != ERR_NONE)
		log_err("set shutter frame length fail");

	return ERR_NONE;
}

static uint32 feature_get_default_framerate(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 *framerate = (uint32 *)(uintptr_t)(*(feature_data + 1));

	ret = hwsensor_get_default_framerate(sensor_idx,
		scenario_id, framerate);
	if (ret != ERR_NONE)
		log_err("get default frame rate fail");

	return ERR_NONE;
}

static uint32 feature_get_pdaf_capacity(uint32 sensor_idx,
	uint8 *feature_para, uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 *pdaf_support = (uint32 *)(uintptr_t)(*(feature_data + 1));

	ret = hwsensor_get_pdaf_capacity(sensor_idx,
		scenario_id, pdaf_support, *feature_para_len);
	if (ret != ERR_NONE)
		log_err("get pdaf capacity fail");

	return ERR_NONE;
}

static uint32 feature_set_scenario_framerate(uint32 sensor_idx,
	uint8 *feature_para, __attribute__((unused))uint32 *feature_para_len)
{
	uint32 ret;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	uint32 scenario_id = (uint32)*feature_data;
	uint32 framerate = (uint32)(*(feature_data + 1));

	ret = hwsensor_set_scenario_framerate(sensor_idx,
		scenario_id, framerate);
	if (ret != ERR_NONE)
		log_err("set scenario frame rate fail");

	return ERR_NONE;
}

struct feature_info feature_info_tab[] = {
#if (DRIVER_VERSION > 1)
	{ SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO, feature_get_gain_range },
	{ SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP, feature_get_gain_info },
	{ SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, feature_get_min_shutter },
	{ SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO, feature_default_ops },
#endif
	{ SENSOR_FEATURE_GET_PERIOD, feature_get_period },
	{ SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ, feature_get_pclk },
	{ SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE, feature_get_checksum },
	{
		SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO,
		feature_get_scenario_pclk
	},
	{ SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO, feature_get_scenario_period },

	{ SENSOR_FEATURE_SET_ESHUTTER, feature_set_shutter },
	{ SENSOR_FEATURE_SET_GAIN, feature_set_gain },
	{ SENSOR_FEATURE_SET_VIDEO_MODE, feature_set_video_mode },
	{ SENSOR_FEATURE_CHECK_SENSOR_ID, feature_match_id },
	{ SENSOR_FEATURE_SET_AUTO_FLICKER_MODE, feature_set_auto_flicker },
	{ SENSOR_FEATURE_SET_FRAMERATE, feature_set_current_fps },
	{ SENSOR_FEATURE_SET_STREAMING_SUSPEND, feature_streaming_suspend },
	{ SENSOR_FEATURE_SET_STREAMING_RESUME, feature_streaming_resume },

	{ SENSOR_FEATURE_GET_CROP_INFO, feature_get_crop_info },
	{ SENSOR_FEATURE_GET_PDAF_INFO, feature_get_pdaf_info },
	{ SENSOR_FEATURE_SET_TEST_PATTERN, feature_set_test_pattern },
	{ SENSOR_HUAWEI_FEATURE_DUMP_REG, feature_dump_reg },
	{ SENSOR_FEATURE_GET_MIPI_PIXEL_RATE, feature_get_mipi_pixel_rate },
	{ SENSOR_FEATURE_GET_MIPI_TRAIL_VAL, feature_get_mipi_trail_val },
	{ SENSOR_FEATURE_GET_PIXEL_RATE, feature_get_sensor_pixel_rate },

	{ SENSOR_FEATURE_SET_PDAF, feature_set_pdaf_mode },
	{ SENSOR_FEATURE_SET_PDAF_REG_SETTING, feature_set_pdaf_setting },
	{ SENSOR_FEATURE_GET_PDAF_REG_SETTING, feature_get_pdaf_regs_data },
	{ SENSOR_FEATURE_GET_VC_INFO, feature_get_vc_info },
	{ SENSOR_FEATURE_GET_BINNING_TYPE, feature_get_binning_ratio },
	{
		SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME,
		feature_set_shutter_frame_length
	},
	{
		SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO,
		feature_get_default_framerate
	},
	{ SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY, feature_get_pdaf_capacity },
	{ SENSOR_FEATURE_GET_PDAF_DATA, feature_default_ops },
	{
		SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,
		feature_set_scenario_framerate
	},
	{ SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE, feature_default_ops },
	{ SENSOR_FEATURE_GET_VC_INFO2, feature_default_ops },
	{ SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE, feature_default_ops },
	{ SENSOR_FEATURE_GET_TEMPERATURE_VALUE, feature_default_ops },
	{ SENSOR_FEATURE_SET_HDR, feature_default_ops },
};

static uint32 feature_control(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_FEATURE_ENUM feature_id,
	MUINT8 *feature_para,
	MUINT32 *len)
{
	uint32 sensor_idx = (uint32)psensor->inst.sensor_idx;
	uint32 i;
	uint32 size;

	adapt_dbg("feature_id = %d", feature_id);

	size = camkit_array_size(feature_info_tab);
	for (i = 0; i < size; i++) {
		if (feature_id == feature_info_tab[i].feature_id) {
			feature_info_tab[i].feature_ops(sensor_idx, feature_para, len);
			break;
		}
	}

	if (i >= size)
		log_info("Not implement the feature:%d", feature_id);

	return ERR_NONE;
}

uint32 adopt_sensor_feature_control(
	struct IMGSENSOR_SENSOR *psensor,
	MSDK_SENSOR_FEATURE_ENUM feature_id,
	MUINT8 *feature_param,
	MUINT32 *feature_param_len)
{
	MUINT32 ret;
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	MINT32 ret_sec = ERROR_NONE;
	struct command_params c_params;
#endif
	struct IMGSENSOR_SENSOR_INST *psensor_inst = NULL;

	return_err_if_null(psensor);
	return_err_if_null(feature_param);
	return_err_if_null(feature_param_len);
	psensor_inst = &psensor->inst;

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
		ret = feature_control(psensor, feature_id,
				feature_param, feature_param_len);
#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	}
#endif
	if (ret != ERROR_NONE)
		log_err("feature control fail");

	imgsensor_mutex_unlock(psensor_inst);

	return ret;
}

int32 adopt_sensor_close(struct IMGSENSOR_SENSOR *psensor)
{
	int32 ret;

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	MINT32 ret_sec = ERROR_NONE;
	struct command_params c_params = {0};
	struct IMGSENSOR *pimgsensor = &gimgsensor;
#endif

	struct IMGSENSOR_SENSOR_INST  *psensor_inst = NULL;
	uint32 sensor_index;

	return_err_if_null(psensor);
	psensor_inst = &psensor->inst;
	sensor_index = (uint32)psensor->inst.sensor_idx;

	imgsensor_mutex_lock(psensor_inst);

#ifdef IMGSENSOR_OC_ENABLE
	imgsensor_oc_interrupt(IMGSENSOR_HW_POWER_STATUS_OFF);
#endif

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
	PK_INFO("%s secure state %d", __func__,
		(int)(&gimgsensor)->imgsensor_sec_flag);
	if ((&gimgsensor)->imgsensor_sec_flag)
		ret = imgsensor_ca_invoke_command(
			IMGSENSOR_TEE_CMD_CLOSE, c_params, &ret_sec);
	else
		ret = hwsensor_close(sensor_index);
#else
	ret = hwsensor_close(sensor_index);
#endif
	if (ret != ERROR_NONE)
		log_err("close sensor fail");
	else
		psensor_inst->state = IMGSENSOR_STATE_CLOSE;

	imgsensor_mutex_unlock(psensor_inst);

	return ret;
}
