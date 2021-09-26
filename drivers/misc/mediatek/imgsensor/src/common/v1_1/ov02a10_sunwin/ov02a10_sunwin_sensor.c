/*
 * ov02a10_sunwin_sensor.c
 *
 * ov02a10_sunwin image sensor driver
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#include "ov02a10_sunwin_sensor.h"
#include <linux/spinlock.h>
#include <linux/printk.h>
#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor_i2c.h"
#include <securec.h>

#define RETRY_TIMES 2

/* Modify Following Strings for Debug */
#define PFX "[ov02a10_sunwin]"
#define DEBUG_OV02A10_SUNWIN 0
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_OV02A10_SUNWIN) \
			pr_debug(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) \
		pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) \
		pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

#define REG_GAIN_1X 0x10  // 16x
#define MAX_AGAIN   15.5
#define OV02A10_BASIC_FRAME_LENGTH    1224
#define OV02A10_EXPO_REG_H            0x03
#define OV02A10_EXPO_REG_L            0x04
#define OV02A10_VBLANK_REG_H          0x05
#define OV02A10_VBLANK_REG_L          0x06
#define SENSOR_ANA_GAIN_REG           0x24
#define SENSOR_DIG_GAIN_REG           0x10
#define SENSOR_FRM_CNT_REG            0x01
#define OV02A10_VTS_REG_H             0x4e
#define OV02A10_AEC_CTL_REG           0x01
#define OV02A10_PAGE_SEL_REG          0xfd
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static void set_dummy(void)
{
	kal_uint32 vblank;

	LOG_DBG("ENTER\n");

	vblank = imgsensor.frame_length - OV02A10_BASIC_FRAME_LENGTH;
	/* 1 select page,vts vb reg on paga_1 */
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_PAGE_SEL_REG,
					 0x01, IMGSENSOR_I2C_BYTE_DATA);

	/* 2 If you want to change VTS (fps), you can modify the */
	/* Vblank(P1:0x05&P1:0x06,read_write) */
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_VBLANK_REG_H,
			(vblank >> 8) & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_VBLANK_REG_L,
			vblank & 0xff, IMGSENSOR_I2C_BYTE_DATA);

	/* 3 ov02a10 exposure, analog gain, mirror or flip register, */
	/* must set P1:0x01 value to 0x01,otherwise no effect */
	(void)imgsensor_sensor_i2c_write(&imgsensor, SENSOR_FRM_CNT_REG,
			OV02A10_AEC_CTL_REG, IMGSENSOR_I2C_BYTE_DATA);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_DBG("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	if (!framerate || !imgsensor.line_length) {
		LOG_ERR("Invalid params. framerate=%d, line_length=%d\n",
			framerate, imgsensor.line_length);
		return;
	}
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
		(frame_length > imgsensor.min_frame_length) ?
		frame_length : imgsensor.min_frame_length;

	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter:%d!\n", shutter);

	/* 1. calc the framelength depend on shutter */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	/* 2. calc the to be in the min,max threshold */
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
			imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
			(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;

	/* 3. calc and update framelength to abandon the flicker issue */
	if (imgsensor.autoflicker_en) {
		/* calc fps between 298~305, real fps set to 298 */
		if (realtime_fps >= 298 && realtime_fps <= 305) {
			set_max_framerate(298, 0);
			/* calc fps between 146~150, real fps set to 146 */
		} else if (realtime_fps >= 146 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			set_max_framerate(realtime_fps, 0);
		}
	} else {
		set_max_framerate(realtime_fps, 0);
	}

	/* 4 update shutter */
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_PAGE_SEL_REG,
					 0x01, IMGSENSOR_I2C_BYTE_DATA);

	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_EXPO_REG_H,
				(shutter >> 8) & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_EXPO_REG_L,
				shutter & 0xff, IMGSENSOR_I2C_BYTE_DATA);

	(void)imgsensor_sensor_i2c_write(&imgsensor, SENSOR_FRM_CNT_REG,
				OV02A10_AEC_CTL_REG, IMGSENSOR_I2C_BYTE_DATA);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,
							imgsensor.frame_length);
}

static void set_shutter_frame_length(UINT32 shutter, UINT32 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter:%d!\n", shutter);

	/* if shutter bigger than frame_length, extend frame length */
	spin_lock(&imgsensor_drv_lock);
	if (frame_length > imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
			imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		/* calc fps between 298~305, real fps set to 298 */
		if (realtime_fps >= 298 && realtime_fps <= 305) {
			set_max_framerate(298, 0);
			/* calc fps between 146~150, real fps set to 146 */
		} else if (realtime_fps >= 146 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			set_max_framerate(realtime_fps, 0);
		}
	} else {
		set_max_framerate(realtime_fps, 0);
	}

	/* Update Shutter */
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_PAGE_SEL_REG,
					 0x01, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_EXPO_REG_H,
				(shutter >> 8) & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_EXPO_REG_L,
				shutter & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, SENSOR_FRM_CNT_REG,
				OV02A10_AEC_CTL_REG, IMGSENSOR_I2C_BYTE_DATA);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = (gain * REG_GAIN_1X) / BASEGAIN;
	return (kal_uint16)reg_gain;
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_DBG("set_gain %d\n", gain);
	if (gain < BASEGAIN || gain > (kal_uint16)(MAX_AGAIN * BASEGAIN)) {
		LOG_ERR("Error gain setting\n");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > (kal_uint16)(MAX_AGAIN * BASEGAIN))
			gain = (kal_uint16)(MAX_AGAIN * BASEGAIN);
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	(void)imgsensor_sensor_i2c_write(&imgsensor, OV02A10_PAGE_SEL_REG,
					 0x01, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, SENSOR_ANA_GAIN_REG,
				(reg_gain&0xFF), IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, SENSOR_FRM_CNT_REG,
				OV02A10_AEC_CTL_REG, IMGSENSOR_I2C_BYTE_DATA);
	LOG_DBG("EXIT\n");

	return gain;
}

static kal_uint32 sensor_init(void)
{
	kal_int32 rc = 0;

	LOG_DBG("ENTER\n");

	rc = imgsensor_sensor_write_setting(&imgsensor,
						&imgsensor_info.init_setting);
	if (rc < 0) {
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 set_preview_setting(void)
{
	kal_int32 rc = 0;

	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor,
						&imgsensor_info.pre_setting);
	if (rc < 0) {
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 set_capture_setting(kal_uint16 currefps)
{
	kal_int32 rc = 0;

	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor,
						&imgsensor_info.cap_setting);
	if (rc < 0) {
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 set_normal_video_setting(kal_uint16 currefps)
{
	kal_int32 rc = 0;

	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor,
				&imgsensor_info.normal_video_setting);
	if (rc < 0) {
		LOG_ERR("Failed\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 return_sensor_id(void)
{
	kal_int32 rc = 0;
	kal_uint16 sensor_id = 0;

	rc = imgsensor_sensor_i2c_read(&imgsensor, imgsensor_info.sensor_id_reg,
					&sensor_id, IMGSENSOR_I2C_WORD_DATA);
	if (rc < 0) {
		LOG_ERR("Read id failed.id reg: 0x%x\n",
				imgsensor_info.sensor_id_reg);
		sensor_id = 0xFFFF;
	}
	LOG_INF("sensor_id:0x%x", sensor_id);
	return sensor_id;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = RETRY_TIMES; /* retry 2 time */
	//UINT32 reg_sensor_id = 0;
	/* vendorID offset is 4 in eeprom */
	//UINT32 vendorID_offset = 4;

	LOG_DBG("get_imgsensor_id\n");

	spin_lock(&imgsensor_drv_lock);
	/* init i2c config */
	imgsensor.i2c_speed = imgsensor_info.i2c_speed;
	imgsensor.addr_type = imgsensor_info.addr_type;
	spin_unlock(&imgsensor_drv_lock);

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			/* *sensor_id = imgsensor_convert_sensor_id(
					imgsensor_info.sensor_id,
					reg_sensor_id, vendorID_offset, 0); */
			LOG_INF("sensor_module_product_id:0x%x\n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("id reg: 0x%x, read id: 0x%x, expect id:0x%x\n",
					imgsensor.i2c_write_id, *sensor_id,
					imgsensor_info.sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Check sensor id fail, id reg: 0x%x,read id: 0x%x, expect id:0x%x\n",
					imgsensor.i2c_write_id, *sensor_id,
					imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*
		 * if Sensor ID is not correct,
		 * Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint32 sensor_id = 0;
	kal_uint32 rc = ERROR_NONE;

	LOG_INF("ENTER\n");

	rc = get_imgsensor_id(&sensor_id);
	if (rc != ERROR_NONE) {
		LOG_ERR("probe sensor failed\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	LOG_DBG("sensor probe successfully. sensor_id=0x%x\n", sensor_id);

	rc = sensor_init();
	if (rc != ERROR_NONE) {
		LOG_ERR("init failed\n");
		return rc;
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 rc = ERROR_NONE;

	LOG_INF("ENTER\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	rc = set_preview_setting();
	if (rc != ERROR_NONE) {
		LOG_ERR("preview setting failed\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 rc = ERROR_NONE;

	LOG_INF("ENTER\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	rc = set_capture_setting(imgsensor.current_fps);
	if (rc != ERROR_NONE) {
		LOG_ERR("capture setting failed\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 rc = ERROR_NONE;

	LOG_INF("ENTER\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	rc = set_normal_video_setting(imgsensor.current_fps);
	if (rc != ERROR_NONE) {
		LOG_ERR("normal video setting failed\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT
				*sensor_resolution)
{
	if (sensor_resolution != NULL) {
		sensor_resolution->SensorFullWidth =
				imgsensor_info.cap.grabwindow_width;
		sensor_resolution->SensorFullHeight =
				imgsensor_info.cap.grabwindow_height;

		sensor_resolution->SensorPreviewWidth =
				imgsensor_info.pre.grabwindow_width;
		sensor_resolution->SensorPreviewHeight =
				imgsensor_info.pre.grabwindow_height;

		sensor_resolution->SensorVideoWidth =
				imgsensor_info.normal_video.grabwindow_width;
		sensor_resolution->SensorVideoHeight =
				imgsensor_info.normal_video.grabwindow_height;
	}
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	if (!sensor_info || !sensor_config_data) {
		LOG_ERR("Fatal: NULL ptr. sensor_info:%pK, sensor_config_data:%pK\n",
			sensor_info, sensor_config_data);
		return ERROR_INVALID_PARA;
	}
	LOG_DBG("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;

	sensor_info->SettleDelayMode =
		imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_NA;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 rc = ERROR_NONE;

	LOG_DBG("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		rc = preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		rc = capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		rc = normal_video(image_window, sensor_config_data);
		break;
	default:
		LOG_ERR("Error ScenarioId setting. scenario_id:%d\n",
			scenario_id);
		rc = preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return rc;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		return ERROR_NONE; /* Dynamic frame rate */
	spin_lock(&imgsensor_drv_lock);
	/* fps set to 298 when frame is 300 and auto-flicker enaled */
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 298;
	/* fps set to 146 when frame is 150 and auto-flicker enaled */
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM
	scenario_id, UINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0) {
		LOG_ERR("invalid framerate\n");
		return ERROR_NONE;
	}
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		if (imgsensor_info.pre.linelength == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
				imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (imgsensor_info.normal_video.linelength == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate *
				10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength +
				imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor_info.cap.linelength == 0)
			return ERROR_NONE;

		frame_length = imgsensor_info.cap.pclk / framerate * 10 /
				imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default: /* coding with  preview scenario by default */
		if (imgsensor_info.pre.linelength == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
				imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_ERR("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM
	scenario_id, UINT32 *framerate)
{
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 sensor_dump_reg(void)
{
	kal_int32 rc = 0;

	rc = imgsensor_sensor_i2c_process(&imgsensor,
					  &imgsensor_info.dump_info);
	if (rc < 0)
		LOG_ERR("Failed\n");
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	kal_int32 rc = 0;

	LOG_INF("Enter.enable:%d\n", enable);
	if (enable)
		rc = imgsensor_sensor_write_setting(&imgsensor,
			&imgsensor_info.streamon_setting);
	else
		rc = imgsensor_sensor_write_setting(&imgsensor,
			&imgsensor_info.streamoff_setting);
	if (rc < 0) {
		LOG_ERR("Failed enable:%d\n", enable);
		return ERROR_SENSOR_POWER_ON_FAIL;
	}
	LOG_INF("Exit.enable:%d\n", enable);

	return ERROR_NONE;
}

static kal_uint32 feature_control_ov02a10_sunwin(MSDK_SENSOR_FEATURE_ENUM
	feature_id, UINT8 *feature_para, UINT32 *feature_para_len)
{
	kal_uint32 rc = ERROR_NONE;
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo = 0;
	errno_t tmp = 0;

	if (feature_para == NULL || feature_para_len == NULL) {
		LOG_ERR("null ptr input params\n");
		return ERROR_INVALID_PARA;
	}

	LOG_DBG("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4; /* return 4 byte data */
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4; /* return 4 byte data */
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			LOG_DBG("cap.pclk = %d\n", imgsensor_info.cap.pclk);
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			LOG_DBG("normal_video.pclk = %d\n", imgsensor_info.normal_video.pclk);
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			LOG_DBG("pre.pclk = %d\n", imgsensor_info.pre.pclk);
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter((UINT32)*feature_data);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32)*feature_data,
			(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(
			(BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *feature_data,
			(UINT32) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *feature_data,
			(UINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		break;
	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4; /* return 4 byte data */
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			/* imgsensor_winsize_info arry 1 is capture setting */
			tmp = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (tmp != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 1 failed!");
				return 1;
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			/* imgsensor_winsize_info arry 2 is preview setting */
			tmp = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (tmp != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 2 failed!");
				return 1;
			}
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			/* imgsensor_winsize_info arry 0 is preivew setting */
			tmp = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (tmp != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 0 failed!");
				return 1;
			}
			break;
		}
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		rc = streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter((UINT16)*feature_data);
		rc = streaming_control(KAL_TRUE);
		break;
	case SENSOR_HUAWEI_FEATURE_DUMP_REG:
		sensor_dump_reg();
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor_info.cap.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.cap.pclk /
					(imgsensor_info.cap.linelength -
					IMGSENSOR_LINGLENGTH_GAP))*
					imgsensor_info.cap.grabwindow_width;
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (imgsensor_info.normal_video.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.normal_video.pclk /
					(imgsensor_info.normal_video.linelength -
					IMGSENSOR_LINGLENGTH_GAP))*
					imgsensor_info.normal_video.grabwindow_width;
			}
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			if (imgsensor_info.pre.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
					(imgsensor_info.pre.pclk /
					(imgsensor_info.pre.linelength -
					IMGSENSOR_LINGLENGTH_GAP))*
					imgsensor_info.pre.grabwindow_width;
			}
			break;
		}
		break;
	default:
		LOG_INF("Not support the feature_id:%d\n", feature_id);
		break;
	}

	return rc;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control_ov02a10_sunwin,
	control,
	close
};

UINT32 OV02A10_SUNWIN_SensorInit(struct SENSOR_FUNCTION_STRUCT **pf_func)
{
	/* Check Sensor status here */
	if (pf_func != NULL)
		*pf_func = &sensor_func;
	return ERROR_NONE;
}
