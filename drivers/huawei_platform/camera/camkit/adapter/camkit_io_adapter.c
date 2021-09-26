/*
 * camkit_io_adapter.c
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * Description: camkit io interface adapted on the platform
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

#include "camkit_io_adapter.h"

#include "imgsensor.h"
#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"

#include "camkit_driver_types.h"

extern struct IMGSENSOR gimgsensor;
extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
		u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
			u16 transfer_length, u16 timing);

struct pin_map {
	uint32 pin;
	enum IMGSENSOR_HW_PIN mtk_pin;
};

static enum IMGSENSOR_HW_PIN adopt_translate_pin_type(int32 pin_type)
{
	enum IMGSENSOR_HW_PIN mtk_pin;
	uint32 i;
	uint32 pin_num;

	struct pin_map pin_table[] = {
		{ CAMKIT_HW_PIN_NONE, IMGSENSOR_HW_PIN_NONE },
		{ CAMKIT_HW_PIN_PDN, IMGSENSOR_HW_PIN_PDN },
		{ CAMKIT_HW_PIN_RST, IMGSENSOR_HW_PIN_RST },
		{ CAMKIT_HW_PIN_AVDD_EN, IMGSENSOR_HW_PIN_AVDD_EN },
		{ CAMKIT_HW_PIN_AVDD_SEL, IMGSENSOR_HW_PIN_AVDD_SEL },
		{ CAMKIT_HW_PIN_DVDD_EN, IMGSENSOR_HW_PIN_DVDD_EN },
		{ CAMKIT_HW_PIN_DVDD_SEL, IMGSENSOR_HW_PIN_DVDD_SEL },
		{ CAMKIT_HW_PIN_IOVDD_EN, IMGSENSOR_HW_PIN_IOVDD_EN },
		{ CAMKIT_HW_PIN_AVDD1_EN, IMGSENSOR_HW_PIN_AVDD1_EN },
		{ CAMKIT_HW_PIN_AFVDD_EN, IMGSENSOR_HW_PIN_AFVDD_EN },
		{ CAMKIT_HW_PIN_AVDD, IMGSENSOR_HW_PIN_AVDD },
		{ CAMKIT_HW_PIN_DVDD, IMGSENSOR_HW_PIN_DVDD },
		{ CAMKIT_HW_PIN_DOVDD, IMGSENSOR_HW_PIN_DOVDD },
		{ CAMKIT_HW_PIN_AFVDD, IMGSENSOR_HW_PIN_AFVDD },
#ifdef MIPI_SWITCH
		{ CAMKIT_HW_PIN_MIPI_SWITCH_EN, IMGSENSOR_HW_PIN_MIPI_SWITCH_EN },
		{ CAMKIT_HW_PIN_MIPI_SWITCH_SEL, IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL },
#endif
		{ CAMKIT_HW_PIN_MCLK, IMGSENSOR_HW_PIN_MCLK },
		{ CAMKIT_HW_PIN_RST1, IMGSENSOR_HW_PIN_RST1 },
		{ CAMKIT_HW_PIN_UNDEF, IMGSENSOR_HW_PIN_UNDEF },
	};

	pin_num = camkit_array_size(pin_table);
	for (i = 0; i < pin_num; i++) {
		if (pin_type == pin_table[i].pin) {
			mtk_pin = pin_table[i].mtk_pin;
			break;
		}
	}

	if (i >= pin_num)
		mtk_pin = IMGSENSOR_HW_PIN_UNDEF;

	return mtk_pin;
}

static enum IMGSENSOR_HW_PIN_STATE adopt_translate_pin_value(int32 pin_val)
{
	enum IMGSENSOR_HW_PIN_STATE mtk_pin_value;

	switch (pin_val) {
	case CAMKIT_HW_PIN_VALUE_NONE:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	case CAMKIT_HW_PIN_VALUE_LOW:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_0;
		break;
	case CAMKIT_HW_PIN_VALUE_HIGH:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH;
		break;
	case CAMKIT_HW_PIN_VALUE_1000:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1000;
		break;
	case CAMKIT_HW_PIN_VALUE_1050:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1050;
		break;
	case CAMKIT_HW_PIN_VALUE_1100:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1100;
		break;
	case CAMKIT_HW_PIN_VALUE_1200:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1200;
		break;
	case CAMKIT_HW_PIN_VALUE_1210:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1210;
		break;
	case CAMKIT_HW_PIN_VALUE_1220:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1220;
		break;
	case CAMKIT_HW_PIN_VALUE_1250:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1250;
		break;
	case CAMKIT_HW_PIN_VALUE_1500:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1500;
		break;
	case CAMKIT_HW_PIN_VALUE_1800:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_1800;
		break;
	case CAMKIT_HW_PIN_VALUE_2500:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_2500;
		break;
	case CAMKIT_HW_PIN_VALUE_2800:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_2800;
		break;
	case CAMKIT_HW_PIN_VALUE_2900:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_2900;
		break;
	case CAMKIT_HW_PIN_VALUE_3000:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_LEVEL_3000;
		break;
	default:
		mtk_pin_value = IMGSENSOR_HW_PIN_STATE_NONE;
		break;
	}

	return mtk_pin_value;
}

static enum IMGSENSOR_SENSOR_IDX adopt_translate_sensor_idx(uint32 sensor_idx)
{
	enum IMGSENSOR_SENSOR_IDX mtk_sensor_idx;

	switch (sensor_idx) {
	case CAMKIT_SENSOR_IDX_MAIN:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN;
		break;
	case CAMKIT_SENSOR_IDX_SUB:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB;
		break;
	case CAMKIT_SENSOR_IDX_MAIN2:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN2;
		break;
	case CAMKIT_SENSOR_IDX_SUB2:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB2;
		break;
	case CAMKIT_SENSOR_IDX_MAIN3:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN3;
		break;
#if (DRIVER_VERSION > 1)
	case CAMKIT_SENSOR_IDX_SUB3:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB3;
		break;
	case CAMKIT_SENSOR_IDX_MAIN4:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN4;
		break;
	case CAMKIT_SENSOR_IDX_SUB4:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB4;
		break;
	case CAMKIT_SENSOR_IDX_MAIN5:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN5;
		break;
	case CAMKIT_SENSOR_IDX_SUB5:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB5;
		break;
	case CAMKIT_SENSOR_IDX_MAIN6:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN6;
		break;
	case CAMKIT_SENSOR_IDX_SUB6:
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_SUB6;
		break;
#endif
	default:
		log_err("sensor idx is wrong, please check");
		mtk_sensor_idx = IMGSENSOR_SENSOR_IDX_MAIN;
		break;
	}

	return mtk_sensor_idx;
}

int32 adopt_sensor_power_sequence(uint32 sensor_idx,
	enum   camkit_power_status pwr_status,
	struct camkit_hw_power_info_t *ppower_info)
{
	struct IMGSENSOR                  *pimgsensor = &gimgsensor;
	struct IMGSENSOR_HW                      *phw = &pimgsensor->hw;
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr = NULL;
	struct camkit_hw_power_info_t    *ppwr_info = ppower_info;
	struct IMGSENSOR_HW_DEVICE       *pdev = NULL;
	enum IMGSENSOR_HW_PIN             pin_type;
	enum IMGSENSOR_HW_PIN_STATE       pin_val;
	enum IMGSENSOR_SENSOR_IDX         mtk_sensor_idx;

	return_err_if_null(phw);
	return_err_if_null(ppower_info);

	mtk_sensor_idx = adopt_translate_sensor_idx(sensor_idx);
	psensor_pwr = &phw->sensor_pwr[mtk_sensor_idx];
	pin_type = adopt_translate_pin_type(ppwr_info->pin_type);
	pin_val = adopt_translate_pin_value(ppwr_info->pin_val);
	while (pin_type != IMGSENSOR_HW_PIN_NONE &&
		ppwr_info < ppower_info + IMGSENSOR_HW_PIN_MAX_NUM) {
		if (pin_type != IMGSENSOR_HW_PIN_UNDEF &&
			psensor_pwr->id[pin_type] != IMGSENSOR_HW_ID_NONE) {
			pdev = phw->pdev[psensor_pwr->id[pin_type]];
			log_info("sensor_idx = %d, pin=%d, pin_val=%d, hw_id =%d\n",
				mtk_sensor_idx, pin_type, pin_val, psensor_pwr->id[pin_type]);

			if (pdev && pdev->set)
				pdev->set(pdev->pinstance, mtk_sensor_idx, pin_type, pin_val);

			mdelay(ppwr_info->pin_delay);
		}

		ppwr_info++;
		pin_type = adopt_translate_pin_type(ppwr_info->pin_type);
		pin_val = adopt_translate_pin_value(ppwr_info->pin_val);
	}

	/* wait for power stable */
	if (pwr_status == CAMKIT_HW_POWER_STATUS_ON)
		mdelay(5);

	return IMGSENSOR_RETURN_SUCCESS;
}

int32 adopt_i2c_read(uint8 *send_data, uint16 send_data_len,
	uint8 *recv_data, uint16 recv_data_len, uint16 i2c_addr)
{
	return iReadRegI2C(send_data, send_data_len,
		recv_data, recv_data_len, i2c_addr);
}

int32 adopt_i2c_write(uint8 *data, uint16 len, uint16 i2c_addr)
{
	return iWriteRegI2C(data, len, i2c_addr);
}

int32 adopt_i2c_set_speed(uint16 i2c_speed)
{
	kdSetI2CSpeed(i2c_speed);
	return ERR_NONE;
}

int32 adopt_i2c_burst_write(uint8 *data, uint32 len,
	uint16 i2c_addr, uint16 len_per_cycle, uint16 i2c_speed)
{
	return iBurstWriteReg_multi(data, len, i2c_addr,
		len_per_cycle, i2c_speed);
}
