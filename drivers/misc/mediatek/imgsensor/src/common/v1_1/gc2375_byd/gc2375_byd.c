/*
 * gc2375_byd_sensor.c
 *
 * gc2375_byd image sensor driver
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
 */

#include "gc2375_byd.h"
#include <linux/spinlock.h>
#include <linux/printk.h>
#include "kd_imgsensor_errcode.h"
#include "imgsensor_sensor_i2c.h"
#include <securec.h>

static DEFINE_SPINLOCK(imgsensor_drv_lock);

/* Modify Following Strings for Debug */
#define PFX "[gc2375_byd]"
#define DEBUG_GC2375_BYD 1
#define LOG_DBG(fmt, args...) \
	do { \
		if (DEBUG_GC2375_BYD) \
			pr_debug(PFX "%s %d " \
			fmt, __func__, __LINE__, ##args); \
	} while (0)
#define LOG_INF(fmt, args...) \
		pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)
#define LOG_ERR(fmt, args...) \
		pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

#define GC2375_BASIC_FRAME_LENGTH 1224
#define GC2375_MIN_VB 338
#define GC2375_MAX_VB 8191
#define GC2375_VB_REG_H 0x07
#define GC2375_VB_REG_L 0x08
#define GC2375_PAGE_SEL_REG 0xFE
#define GC2375_EXPO_REG_H 0x03
#define GC2375_EXPO_REG_L 0x04
#define GC2375_DGAIN_REG_H 0xb1
#define GC2375_DGAIN_REG_L 0xb2

#define GC2375_MAX_GAIN 8
#define GC2375_GAIN_SETTING_SIZE 4
typedef struct {
	kal_uint16 gain_value;
	struct imgsensor_i2c_reg setting[GC2375_GAIN_SETTING_SIZE];
	kal_uint16 setting_size;
} gc2375_gain_table_t;

/* gain_talbe master be ordered from smallest gain_value to largest */
static gc2375_gain_table_t gain_table[] = {
	{ /* gain = 1x gain_value = gain *64 */
		.gain_value = 64,
		.setting = {
			{ 0x20, 0x0b, 0x00 },
			{ 0x22, 0x0c, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x00, 0x00 },
		},
		.setting_size = 4,
	}, { /* gain = 1.43x gain_value = gain *64 */
		.gain_value = 91,
		.setting = {
			{ 0x20, 0x0c, 0x00 },
			{ 0x22, 0x0e, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x01, 0x00 },
		},
		.setting_size = 4,

	}, { /* gain = 2x gain_value = gain *64 */
		.gain_value = 128,
		.setting = {
			{ 0x20, 0x0c, 0x00 },
			{ 0x22, 0x0e, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x02, 0x00 },
		},
		.setting_size = 4,
	}, { /* gain = 2.84x gain_value = gain *64 */
		.gain_value = 181,
		.setting = {
			{ 0x20, 0x0c, 0x00 },
			{ 0x22, 0x0e, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x03, 0x00 },
		},
		.setting_size = 4,
	}, { /* gain = 3.97x gain_value = gain *64 */
		.gain_value = 254,
		.setting = {
			{ 0x20, 0x0c, 0x00 },
			{ 0x22, 0x0e, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x04, 0x00 },
		},
		.setting_size = 4,
	}, { /* gain = 5.68x gain_value = gain *64 */
		.gain_value = 363,
		.setting = {
			{ 0x20, 0x0e, 0x00 },
			{ 0x22, 0x0e, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x05, 0x00 },
		},
		.setting_size = 4,
	}, { /* gain = 8.14x gain_value = gain *64 */
		.gain_value = 521,
		.setting = {
			{ 0x20, 0x0c, 0x00 },
			{ 0x22, 0x0c, 0x00 },
			{ 0x26, 0x0e, 0x00 },
			{ 0xb6, 0x06, 0x00 },
		},
		.setting_size = 4,
	},
};
static void set_dummy(void)
{
	kal_uint32 vb;
	kal_int32 rc;

	vb = imgsensor.frame_length - GC2375_BASIC_FRAME_LENGTH;
	vb = vb < GC2375_MIN_VB ? GC2375_MIN_VB : vb;
	vb = vb > GC2375_MAX_VB ? GC2375_MAX_VB : vb;
	rc = imgsensor_sensor_i2c_write(&imgsensor, GC2375_PAGE_SEL_REG, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	if (rc < 0) {
		LOG_ERR("wtire sensor page ctrl reg failed.");
	}
	rc = imgsensor_sensor_i2c_write(&imgsensor, GC2375_VB_REG_H, (vb >> 8) & 0x1f, IMGSENSOR_I2C_BYTE_DATA);
	if (rc < 0) {
		LOG_ERR("wtire sensor csictrl capt vb  h reg  failed.");
	}
	rc = imgsensor_sensor_i2c_write(&imgsensor, GC2375_VB_REG_L, vb & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	if (rc < 0) {
		LOG_ERR("wtire sensor csictrl capt vb  l reg  failed.");
	}
	return;
}

static kal_uint32 return_sensor_id(void)
{
	kal_int32 rc = 0;
	kal_uint16 sensor_id = 0;

	rc = imgsensor_sensor_i2c_read(&imgsensor, imgsensor_info.sensor_id_reg,
	                               &sensor_id, IMGSENSOR_I2C_WORD_DATA);
	if (rc < 0) {
		LOG_ERR("Read id failed.id reg: 0x%x\n", imgsensor_info.sensor_id_reg);
		sensor_id = 0xFFFF;
	}
	LOG_INF("sensor_id:0x%x", sensor_id);
	return sensor_id;
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_DBG("framerate = %d, min framelength should enable %d\n",
	        framerate, min_framelength_en);

	if (!framerate || !imgsensor.line_length) {
		LOG_ERR("Invalid params. framerate=%d, line_length=%d.\n",
		        framerate, imgsensor.line_length);
		return;
	}
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	        (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;

	imgsensor.dummy_line =
	        imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
		        imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(UINT32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter:%d!\n", shutter);

	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
	                imgsensor_info.margin)
	          : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

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
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_PAGE_SEL_REG,
	                                 0x00, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_EXPO_REG_H,
	                                 (shutter >> 8) & 0x3f, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_EXPO_REG_L,
	                                 shutter & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
	return;
}

static void set_shutter_frame_length(UINT32 shutter, UINT32 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter:%d!\n", shutter);

	/* if shutter bigger than frame_length, should extend frame length first */
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

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
	                imgsensor_info.margin)
	          : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

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
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_PAGE_SEL_REG,
	                                 0x00, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_EXPO_REG_H,
	                                 (shutter >> 8) & 0x3f, IMGSENSOR_I2C_BYTE_DATA);
	(void)imgsensor_sensor_i2c_write(&imgsensor, GC2375_EXPO_REG_L,
	                                 shutter & 0xff, IMGSENSOR_I2C_BYTE_DATA);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
	return;
}

#define QVALUE 64
static kal_int16 gain_to_reg(kal_uint16 gain,
                             gc2375_gain_table_t *gain_table, kal_uint16 size)
{
	kal_int16 i;
	kal_int16 real_gain;
	kal_int16 digital_gain;
	kal_int32 rc;

	if (!gain_table || !size) {
		LOG_ERR("Fatal err.gain_table:%pK, size:%d.\n",
		        gain_table, size);
		return ERROR_INVALID_PARA;
	}
	for (i = 0; i < size; i++) {
		if (gain < gain_table[i].gain_value) {
			LOG_DBG("Find the target gain setting. i=%d.\n", i);
			break;
		}
	}
	if (i == size) {
		LOG_ERR("Invalid gain. gain:%d", gain);
	}
	real_gain = gain_table[i - 1].gain_value;
	digital_gain = QVALUE * gain / real_gain;
	LOG_DBG("real_gain:%d, digital_gain:%d.enter\n", real_gain, digital_gain);
	rc = imgsensor_sensor_write_table(&imgsensor, gain_table[i - 1].setting,
	                                  gain_table[i - 1].setting_size, IMGSENSOR_I2C_BYTE_DATA);
	if (rc < 0) {
		LOG_ERR("write real_gain failed.\n");
	}
	LOG_DBG("real_gain:%d, digital_gain:%d.exit\n", real_gain, digital_gain);
	/*
	 * GC2375_DGAIN_REG_H:[3:0] digital_gain[9:6];
	 * GC2375_DGAIN_REG_L:[7:2] digital_gain[5:0];
	 */
	rc = imgsensor_sensor_i2c_write(&imgsensor, GC2375_DGAIN_REG_H,
	                                (digital_gain >> 6) & 0x0F, IMGSENSOR_I2C_BYTE_DATA);
	rc |= imgsensor_sensor_i2c_write(&imgsensor, GC2375_DGAIN_REG_L,
	                                 (digital_gain << 2) & 0xFF, IMGSENSOR_I2C_BYTE_DATA);
	if (rc < 0) {
		LOG_ERR("write digital_gain failed.\n");
	}

	return ERROR_NONE;
}
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint32 real_gain = 0;
	kal_uint16 rc;
	LOG_INF("Enter, gain:%d!\n", gain);
	if (gain < BASEGAIN) {
		real_gain = BASEGAIN;
	} else if (gain > GC2375_MAX_GAIN * BASEGAIN) {
		real_gain = GC2375_MAX_GAIN * BASEGAIN;
	} else {
		real_gain = gain;
	}
	/* calc and set again */
	rc = gain_to_reg(real_gain, gain_table, IMGSENSOR_ARRAY_SIZE(gain_table));
	if (rc < 0) {
		LOG_ERR("set gain failed.gain=%d\n", gain);
	}
	LOG_INF("Exit!\n");
	/* calc and set dgain */
	return real_gain;
}

static kal_uint32 sensor_init(void)
{
	kal_int32 rc = 0;
	LOG_DBG("ENTER.\n");

	rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.init_setting);
	if (rc < 0) {
		LOG_ERR("Failed.\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT.\n");

	return ERROR_NONE;
}

static kal_uint32 set_preview_setting(void)
{
	kal_int32 rc = 0;
	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.pre_setting);
	if (rc < 0) {
		LOG_ERR("Failed.\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT.\n");

	return ERROR_NONE;
}

static kal_uint32 set_capture_setting(kal_uint16 currefps)
{
	kal_int32 rc = 0;
	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.cap_setting);
	if (rc < 0) {
		LOG_ERR("Failed.\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT.\n");

	return ERROR_NONE;
}

static kal_uint32 set_normal_video_setting(kal_uint16 currefps)
{
	kal_int32 rc = 0;
	LOG_DBG("ENTER\n");
	rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.normal_video_setting);
	if (rc < 0) {
		LOG_ERR("Failed.\n");
		return ERROR_DRIVER_INIT_FAIL;
	}
	LOG_DBG("EXIT\n");

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 * 	get_imgsensor_id
 *
 * DESCRIPTION
 * 	This function get the sensor ID
 *
 * PARAMETERS
 * 	*sensorID : return the sensor ID
 *
 * RETURNS
 * 	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#define RETRY_TIMES 2
MUINT32 imgsensor_con_sensor_id(MUINT32 imgsensor_sensor_id, MUINT32 sensor_chip_id)
{
	MUINT32 sensor_id;
	kal_uint16 mono_flag = 0;

	imgsensor_sensor_i2c_write(&imgsensor, 0xfe, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfe, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfe, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xf7, 0x01, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xf8, 0x0c, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xf9, 0x42, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfa, 0x88, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfc, 0x9e, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xd4, 0x80, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfe, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xd5, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xf3, 0x20, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_read(&imgsensor, 0xd7, &mono_flag, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xd4, 0x00, IMGSENSOR_I2C_BYTE_DATA);
	imgsensor_sensor_i2c_write(&imgsensor, 0xfc, 0x8e, IMGSENSOR_I2C_BYTE_DATA);
	LOG_INF("mono_flag: 0x%x", mono_flag);

	if ((mono_flag & 0x3) == 0x2) { // 10:GC2375B-WC1X0, 00:GC2375H-WC1X0, 01:GC2375H-WC1XH
		sensor_id = ((sensor_chip_id << 12) & 0x0ffff000) | \
			(((mono_flag & 0x3) << 4) & 0x0ff0);
		return sensor_id;
	} else {
		return sensor_chip_id;
	}
}
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = RETRY_TIMES; /* retry 2 time */
	kal_uint16 tmp_sensor_id;

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
			tmp_sensor_id = return_sensor_id();
			*sensor_id = imgsensor_con_sensor_id(
					imgsensor_info.sensor_id,
					tmp_sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("id reg: 0x%x, read id: 0x%x, expect id: 0x%x\n",
					imgsensor.i2c_write_id,
					*sensor_id,
					imgsensor_info.sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Check sensor id, addr: 0x%x,read id: 0x%x, expect id:0x%x.\n",
			        imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = RETRY_TIMES;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 * 	open
 *
 * DESCRIPTION
 * 	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 * 	None
 *
 * RETURNS
 * 	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint32 sensor_id = 0;
	kal_uint32 rc = ERROR_NONE;
	LOG_INF("ENTER\n");

	rc = get_imgsensor_id(&sensor_id);
	if (rc != ERROR_NONE) {
		LOG_ERR("probe sensor failed.\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	LOG_DBG("sensor probe successfully. sensor_id=0x%x.\n", sensor_id);

	rc = sensor_init();
	if (rc != ERROR_NONE) {
		LOG_ERR("init failed.\n");
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
} /* open */

/*************************************************************************
 * FUNCTION
 * 	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 * 	None
 *
 * RETURNS
 * 	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
} /* close */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 * 	This function start the sensor preview.
 *
 * PARAMETERS
 * 	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 * 	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 rc = ERROR_NONE;
	LOG_INF("E\n");

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
		LOG_ERR("preview setting failed.\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
} /*  preview   */
/*************************************************************************
 * FUNCTION
 * 	capture
 *
 * DESCRIPTION
 * 	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 * 	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
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
		LOG_ERR("capture setting failed.\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
} /* capture() */ /* capture() */

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
		LOG_ERR("normal video setting failed.\n");
		return rc;
	}
	LOG_INF("EXIT\n");

	return ERROR_NONE;
} /*  normal_video   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	if (sensor_resolution != NULL) {
		sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
		sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

		sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
		sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

		sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
		sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	}
	return ERROR_NONE;
} /*  get_resolution  */
static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                           MSDK_SENSOR_INFO_STRUCT *sensor_info,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	if (!sensor_info || !sensor_config_data) {
		LOG_ERR("Fatal: NULL ptr. sensor_info:%pK, sensor_config_data:%pK.\n",
		        sensor_info, sensor_config_data);
		return ERROR_INVALID_PARA;
	}
	LOG_DBG("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;

	sensor_info->SettleDelayMode =
	        imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
	        imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;

	sensor_info->HighSpeedVideoDelayFrame =
	        imgsensor_info.hs_video_delay_frame;

	sensor_info->SlimVideoDelayFrame =
	        imgsensor_info.slim_video_delay_frame;

	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	//sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
	        imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
	        imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->TEMPERATURE_SUPPORT = 1;

	sensor_info->PDAF_Support = PDAF_SUPPORT_NA;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

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
} /*      get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
		LOG_ERR("Error ScenarioId setting. scenario_id:%d.\n", scenario_id);
		rc = preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return rc;
} /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
	{
		return ERROR_NONE;
	}
	spin_lock(&imgsensor_drv_lock);
	/* fps set to 298 when frame is 300 and auto-flicker enaled */
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 298;
	}
	/* fps set to 146 when frame is 150 and auto-flicker enaled */
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 146;
	} else {
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) { /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	} else { /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, UINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0) {
		LOG_ERR("invalid framerate.\n");
		return ERROR_NONE;
	}
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		if (imgsensor_info.pre.linelength == 0) {
			return ERROR_NONE;
		}
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		        (frame_length > imgsensor_info.pre.framelength)
		        ? (frame_length - imgsensor_info.pre.framelength)
		        : 0;

		imgsensor.frame_length =
		        imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (imgsensor_info.normal_video.linelength == 0) {
			return ERROR_NONE;
		}
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		        (frame_length > imgsensor_info.normal_video.framelength)
		        ? (frame_length - imgsensor_info.normal_video.framelength)
		        : 0;

		imgsensor.frame_length =
		        imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor_info.cap.linelength == 0) {
			return ERROR_NONE;
		}

		frame_length = imgsensor_info.cap.pclk /
		               framerate * 10 / imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		        (frame_length > imgsensor_info.cap.framelength)
		        ? (frame_length - imgsensor_info.cap.framelength)
		        : 0;

		imgsensor.frame_length =
		        imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default: /* coding with  preview scenario by default */
		if (imgsensor_info.pre.linelength == 0) {
			return ERROR_NONE;
		}
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		        (frame_length > imgsensor_info.pre.framelength)
		        ? (frame_length - imgsensor_info.pre.framelength)
		        : 0;

		imgsensor.frame_length =
		        imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_DBG("error scenario_id = %d, we use preview scenario\n",
		        scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, UINT32 *framerate)
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

/*************************************************************************
* FUNCTION
*   sensor_dump_reg
*
* DESCRIPTION
*   This function dump some sensor reg
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 sensor_dump_reg(void)
{
	kal_int32 rc = 0;
	rc = imgsensor_sensor_i2c_process(&imgsensor, &imgsensor_info.dump_info);
	if (rc < 0) {
		LOG_ERR("Failed.\n");
	}
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	kal_int32 rc = 0;
	LOG_INF("Enter.enable:%d\n", enable);
	if (enable) {
		rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.streamon_setting);
	} else {
		rc = imgsensor_sensor_write_setting(&imgsensor, &imgsensor_info.streamoff_setting);
	}
	if (rc < 0) {
		LOG_ERR("Failed enable:%d.\n", enable);
		return ERROR_SENSOR_POWER_ON_FAIL;
	}
	LOG_INF("Exit.enable:%d\n", enable);

	return ERROR_NONE;
}

static kal_uint32 feature_control_gc2375_byd(MSDK_SENSOR_FEATURE_ENUM feature_id,
                UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo = 0;
	errno_t rc = 0;

	if (feature_para == NULL || feature_para_len == NULL) {
		LOG_ERR("null ptr input params\n");
		return ERROR_INVALID_PARA;
	}

	LOG_DBG("feature_id = %d.\n", feature_id);
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
		(void)set_gain((UINT16)*feature_data);
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
		        (enum MSDK_SCENARIO_ID_ENUM) * feature_data,
		        (UINT32) * (feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
		        (enum MSDK_SCENARIO_ID_ENUM) * feature_data,
		        (UINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4; /* return 4 byte data */
		break;

	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) * feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		        (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			/* imgsensor_winsize_info arry 1 is capture setting */
			rc = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (rc != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 1 failed!");
				return 1;
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			/* imgsensor_winsize_info arry 2 is preview setting */
			rc = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (rc != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 2 failed!");
				return 1;
			}
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			/* imgsensor_winsize_info arry 0 is preivew setting */
			rc = memcpy_s((void *)wininfo,
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT),
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			if (rc != EOK) {
				LOG_ERR("memcpy_s imgsensor_winsize_info 0 failed!");
				return 1;
			}
			break;
		}
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		(void)streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
		        *feature_data);
		if (*feature_data != 0) {
			set_shutter((UINT32)*feature_data);
		}
		(void)streaming_control(KAL_TRUE);
		break;
	case SENSOR_HUAWEI_FEATURE_DUMP_REG:
		sensor_dump_reg();
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:

		switch (*feature_data) {
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
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor_info.cap.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				        (imgsensor_info.cap.pclk /
				         (imgsensor_info.cap.linelength - IMGSENSOR_LINGLENGTH_GAP)) *
				        imgsensor_info.cap.grabwindow_width;
			}
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (imgsensor_info.normal_video.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				        (imgsensor_info.normal_video.pclk /
				         (imgsensor_info.normal_video.linelength - IMGSENSOR_LINGLENGTH_GAP)) *
				        imgsensor_info.normal_video.grabwindow_width;
			}
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			if (imgsensor_info.pre.linelength > IMGSENSOR_LINGLENGTH_GAP) {
				*(UINT32 *)(uintptr_t)(*(feature_data + 1)) =
				        (imgsensor_info.pre.pclk /
				         (imgsensor_info.pre.linelength - IMGSENSOR_LINGLENGTH_GAP)) *
				        imgsensor_info.pre.grabwindow_width;
			}
			break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
} /*      feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control_gc2375_byd,
	control,
	close
};

UINT32 GC2375_BYD_SensorInit(struct SENSOR_FUNCTION_STRUCT **pf_func)
{
	/* To Do : Check Sensor status here */
	if (pf_func != NULL) {
		*pf_func = &sensor_func;
	}
	return ERROR_NONE;
}
