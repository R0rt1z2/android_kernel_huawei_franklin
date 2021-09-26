/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "hwpmic.h"
#include <huawei_platform/sensor/hw_pmic.h>
#include <securec.h>

static struct hwpmic_t reg_instance;

static enum IMGSENSOR_RETURN hwpmic_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon) /* follow the api design of MTK */
{
	struct hwpmic_t *hwpmic = (struct hwpmic_t *)pinstance;

	pr_info("%s enter\n", __func__);

	if (!hwpmic) {
		pr_err("hwpmic is NULL");
		return IMGSENSOR_RETURN_ERROR;
	}

	hwpmic->hw_pmic_ctrl = hw_get_pmic_ctrl();

	if (!pcommon)
		pr_err("pcommon is NULL");

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN hwpmic_release(void *pinstance)
{
	pr_info("%s enter\n", __func__);
	(void)pinstance;
	return IMGSENSOR_RETURN_SUCCESS;
}

static int get_pmic_channel(enum IMGSENSOR_HW_PIN pin)
{
	int pmic_channel;

	switch (pin) {
	case IMGSENSOR_HW_PIN_LDO1_PMIC:
		pmic_channel = VOUT_LDO_1;
		break;
	case IMGSENSOR_HW_PIN_LDO2_PMIC:
		pmic_channel = VOUT_LDO_2;
		break;
	case IMGSENSOR_HW_PIN_LDO3_PMIC:
		pmic_channel = VOUT_LDO_3;
		break;
	case IMGSENSOR_HW_PIN_LDO4_PMIC:
		pmic_channel = VOUT_LDO_4;
		break;
	case IMGSENSOR_HW_PIN_XBUCK1_PMIC:
		pmic_channel = VOUT_BUCK_1;
		break;
	default:
		pmic_channel = VOUT_MAX;
		break;
	};

	return pmic_channel;
}

static unsigned int get_pmic_val(enum IMGSENSOR_HW_PIN_STATE pin_val)
{
	unsigned int pmic_val;

	switch(pin_val) {
	case IMGSENSOR_HW_PIN_STATE_LEVEL_0:
		pmic_val = HWPMIC_VOLTAGE_MIN;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1000:
		pmic_val = HWPMIC_VOLTAGE_1000;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1050:
		pmic_val = HWPMIC_VOLTAGE_1050;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1100:
		pmic_val = HWPMIC_VOLTAGE_1100;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1200:
		pmic_val = HWPMIC_VOLTAGE_1200;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1210:
		pmic_val = HWPMIC_VOLTAGE_1210;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1220:
		pmic_val = HWPMIC_VOLTAGE_1220;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1250:
		pmic_val = HWPMIC_VOLTAGE_1250;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1500:
		pmic_val = HWPMIC_VOLTAGE_1500;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_1800:
		pmic_val = HWPMIC_VOLTAGE_1800;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_2500:
		pmic_val = HWPMIC_VOLTAGE_2500;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_2800:
		pmic_val = HWPMIC_VOLTAGE_2800;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_2850:
		pmic_val = HWPMIC_VOLTAGE_2850;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_2900:
		pmic_val = HWPMIC_VOLTAGE_2900;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_3000:
		pmic_val = HWPMIC_VOLTAGE_3000;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_3300:
		pmic_val = HWPMIC_VOLTAGE_3300;
		break;
	case IMGSENSOR_HW_PIN_STATE_LEVEL_5000:
		pmic_val = HWPMIC_VOLTAGE_5000;
		break;
	default:
		pmic_val = HWPMIC_VOLTAGE_MIN;
		break;
	}

	return pmic_val;
}

static enum IMGSENSOR_RETURN hwpmic_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN pin,
	enum IMGSENSOR_HW_PIN_STATE pin_val)
{
	errno_t ret;
	struct hwpmic_t *hwpmic = (struct hwpmic_t *)pinstance;
	struct hw_comm_pmic_cfg_t pmic_config;

	pr_info("%s sensor_idx:%d, pin:%d, pin_val:%u\n",
		__func__, sensor_idx, pin, pin_val);
	if (!hwpmic || !hwpmic->hw_pmic_ctrl ||
		!hwpmic->hw_pmic_ctrl->func_tbl ||
		!hwpmic->hw_pmic_ctrl->func_tbl->pmic_power_cfg) {
		pr_err("input args has NULL.\n");
		return IMGSENSOR_RETURN_ERROR;
	}

	ret = memset_s(&pmic_config, sizeof(pmic_config), 0, sizeof(pmic_config));
	if (ret != EOK)
		pr_err("pmic_info memset_s fail, ret = %d\n", ret);

	pmic_config.pmic_num = MAIN_PMIC;
	pmic_config.pmic_power_type = get_pmic_channel(pin);
	pmic_config.pmic_power_voltage = get_pmic_val(pin_val);

	if (pmic_config.pmic_power_voltage > HWPMIC_VOLTAGE_MIN)
		pmic_config.pmic_power_state = PMIC_POWER_ON;
	else
		pmic_config.pmic_power_state = PMIC_POWER_OFF;

	pr_info("%s sensor_idx:%d, channel:%d, voltage:%u, status:%d\n",
		__func__, sensor_idx, pmic_config.pmic_power_type,
		pmic_config.pmic_power_voltage, pmic_config.pmic_power_state);
	hwpmic->hw_pmic_ctrl->func_tbl->pmic_power_cfg(CAM_PMIC_REQ,
		&pmic_config);

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN hwpmic_dump(void *pinstance)
{
	pr_info("%s enter\n", __func__);
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN hwpmic_reset(void *pinstance)
{
	struct hwpmic_t *hwpmic = (struct hwpmic_t *)pinstance;
	struct hw_comm_pmic_cfg_t pmic_config;
	errno_t ret;

	if (!hwpmic || !hwpmic->hw_pmic_ctrl ||
		!hwpmic->hw_pmic_ctrl->func_tbl ||
		!hwpmic->hw_pmic_ctrl->func_tbl->pmic_reset) {
		pr_err("%s, input args has NULL\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}

	pr_info("%s enter\n", __func__);
	ret = memset_s(&pmic_config, sizeof(pmic_config), 0, sizeof(pmic_config));
	if (ret != EOK)
		pr_err("%s, pmic_info memset_s fail, ret = %d\n", __func__, ret);

	pmic_config.pmic_num = MAIN_PMIC;
	hwpmic->hw_pmic_ctrl->func_tbl->pmic_reset(CAM_PMIC_REQ,
		&pmic_config);
	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_HWPMIC,
	.pinstance = (void *)&reg_instance,
	.init      = hwpmic_init,
	.set       = hwpmic_set,
	.release   = hwpmic_release,
	.dump      = hwpmic_dump,
	.hw_reset  = hwpmic_reset,
};

enum IMGSENSOR_RETURN imgsensor_hw_hwpmic_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	if (!pdevice) {
		pr_err("%s pdevice has NULL\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}
	*pdevice = &device;
	pr_info("%s enter\n", __func__);
	return IMGSENSOR_RETURN_SUCCESS;
}
