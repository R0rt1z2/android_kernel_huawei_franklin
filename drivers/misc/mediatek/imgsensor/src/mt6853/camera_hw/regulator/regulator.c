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

#include "regulator.h"

#ifdef IMGSENSOR_OC_ENABLE
#include <mt-plat/aee.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include "upmu_common.h"

#define OCP_DAEMON_RESTART_MAX_NUM 100 // form hardware design
#endif /* IMGSENSOR_OC_ENABLE */

static bool regulator_status[IMGSENSOR_SENSOR_IDX_MAX_NUM][
	REGULATOR_TYPE_MAX_NUM] = {{false}};
static void check_for_regulator_get(struct REGULATOR *preg,
	struct device *pdevice, unsigned int sensor_index,
	unsigned int regulator_index);
static void check_for_regulator_put(struct REGULATOR *preg,
	unsigned int sensor_index, unsigned int regulator_index);
static struct IMGSENSOR_HW_DEVICE_COMMON *ghw_device_common;

static DEFINE_MUTEX(g_regulator_state_mutex);

static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1250,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2850,
	REGULATOR_VOLTAGE_2900,
	REGULATOR_VOLTAGE_3000,
	REGULATOR_VOLTAGE_3300,
	REGULATOR_VOLTAGE_5000,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{ "vcama" },
	{ "vcama1" },
	{ "vcamd" },
	{ "vcamio" },
	{ "VMCH" },
	{ "vcamaf" },
};

static struct REGULATOR reg_instance;

#ifdef IMGSENSOR_OC_ENABLE
static int vcamio_oc;
static unsigned int camio_ocp_occur_count;
enum IMGSENSOR_RETURN imgsensor_oc_interrupt_enable(
	enum IMGSENSOR_SENSOR_IDX sensor_idx, bool enable)
{
	struct regulator *preg = NULL;
	struct device *pdevice = NULL;

	if (!gimgsensor.hw.common.pplatform_device) {
		pr_err("[regulator] %s param is null\n", __func__);
		return IMGSENSOR_RETURN_ERROR;
	}

	pdevice = &gimgsensor.hw.common.pplatform_device->dev;

	gimgsensor.status.oc = 0;

	if (enable && !vcamio_oc) {
		mdelay(5); /* wait steady */

		preg = regulator_get(pdevice, "vcamio");
		if (preg) {
			if (regulator_is_enabled(preg)) {
				pmic_enable_interrupt(INT_VCAMIO_OC, 1, "camera");
				vcamio_oc = 1; /* ocp occur */
				pr_info(
					"[regulator] %s INT_VCAMIO_OC %d, idx %d\n",
					__func__, enable, sensor_idx);
			}
			regulator_put(preg);
		} else {
			pr_info(
				"[regulator] %s fail to enable INT_VCAMIO_OC idx %d\n",
				__func__, sensor_idx);
		}

		rcu_read_lock();
		reg_instance.pid = current->tgid;
		rcu_read_unlock();
	} else if (!enable && vcamio_oc){
		reg_instance.pid = -1; /* init pid */
		/* Disable interrupt before power off */
		pmic_enable_interrupt(INT_VCAMIO_OC, 0, "camera");
		vcamio_oc = 0;
		pr_info(
			"[regulator] %s INT_VCAMIO_OC %d, idx %d\n",
			__func__, enable, sensor_idx);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static void _vcamio_oc_handler(void)
{
	pmic_enable_interrupt(INT_VCAMIO_OC, 0, "camera");
	gimgsensor.status.oc = 1; /* ocp occur */
	vcamio_oc = 0;

	if (camio_ocp_occur_count >= OCP_DAEMON_RESTART_MAX_NUM) {
		pr_info("ocp daemon restart count is %u >= max num %u, just return\n",
			camio_ocp_occur_count, OCP_DAEMON_RESTART_MAX_NUM);
		return;
	}

	aee_kernel_warning("Imgsensor vcamio OC", "Over current");
	if (reg_instance.pid != -1) { /* -1: init pid */
		force_sig(SIGKILL,
			pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID));
		reg_instance.pid = -1;
		camio_ocp_occur_count++;
	}

	pr_info("[regulator]%s enter vcamio oc %d, ocp deamon restart count %u\n",
		__func__, gimgsensor.status.oc, camio_ocp_occur_count);
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */
	pmic_register_interrupt_callback(INT_VCAMIO_OC, _vcamio_oc_handler);

	gimgsensor.status.oc = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt_enable;
	reg_instance.pid = -1; /* init pid */
	camio_ocp_occur_count = 0;

	return IMGSENSOR_RETURN_SUCCESS;
}
#endif /* IMGSENSOR_OC_ENABLE */

static enum IMGSENSOR_RETURN regulator_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	ghw_device_common = pcommon;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {
		for (type = 0;
			type < REGULATOR_TYPE_MAX_NUM;
			type++) {
			memset(str_regulator_name, 0,
				sizeof(str_regulator_name));
			snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				idx,
				regulator_control[type].pregulator_type);
			preg->pregulator[idx][type] = regulator_get(
					&pcommon->pplatform_device->dev,
					str_regulator_name);
			if (preg->pregulator[idx][type] == NULL)
				PK_INFO("ERROR: regulator[%d][%d]  %s fail!\n",
						idx, type, str_regulator_name);
			else
				regulator_status[idx][type] = true;

			atomic_set(&preg->enable_cnt[idx][type], 0);
		}
	}

#ifdef IMGSENSOR_OC_ENABLE
	imgsensor_oc_init();
#endif

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;


	if (pin > IMGSENSOR_HW_PIN_AFVDD ||
		pin < IMGSENSOR_HW_PIN_AVDD ||
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH) {
		PK_PR_ERR(
			"[regulator]invalid params, pin: %d pin_state: %d\n",
			pin, pin_state);
		return IMGSENSOR_RETURN_ERROR;
	}

	reg_type_offset = REGULATOR_TYPE_VCAMA;
	if (ghw_device_common)
		check_for_regulator_get(preg,
			&ghw_device_common->pplatform_device->dev,
			sensor_idx,
			(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));

	pregulator =
		preg->pregulator[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt =
		&preg->enable_cnt[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			if (regulator_set_voltage(pregulator,
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				PK_PR_ERR(
				  "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				  pin,
				  regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				PK_PR_ERR(
				"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				pin,
				regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				check_for_regulator_put(preg, sensor_idx,
					(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator))
				PK_DBG("[regulator]%d is enabled\n", pin);

			if (regulator_disable(pregulator)) {
				PK_PR_ERR(
					"[regulator]fail to regulator_disable, powertype: %d\n",
					pin);
					check_for_regulator_put(preg, sensor_idx,
						(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
				return IMGSENSOR_RETURN_ERROR;
			}
			check_for_regulator_put(preg, sensor_idx,
				(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
			atomic_dec(enable_cnt);
		}
	} else {
		PK_PR_ERR("regulator == NULL %d %d %d\n",
				reg_type_offset,
				pin,
				IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_dump(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i, j;

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {

		for (i = REGULATOR_TYPE_VCAMA;
		i < REGULATOR_TYPE_MAX_NUM;
		i++) {
			if (preg->pregulator[j][i] &&
				regulator_is_enabled(preg->pregulator[j][i]) &&
				atomic_read(&preg->enable_cnt[j][i]) != 0)
				PK_DBG("index= %d %s = %d\n",
					j,
					regulator_control[i].pregulator_type,
					regulator_get_voltage(
						preg->pregulator[j][i]));
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static void check_for_regulator_get(struct REGULATOR *preg,
	struct device *pdevice, unsigned int sensor_index,
	unsigned int regulator_index)
{
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	if (!preg || !pdevice) {
		pr_err("Fatal: Null ptr.preg:%pK,pdevice:%pK\n", preg, pdevice);
		return;
	}

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
		regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid sensor_idx:%d regulator_idx: %d\n",
			__func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == false) {
		memset(str_regulator_name, 0, sizeof(str_regulator_name));

		snprintf(str_regulator_name,
			sizeof(str_regulator_name),
			"cam%d_%s",
			sensor_index,
			regulator_control[regulator_index].pregulator_type);
		preg->pregulator[sensor_index][regulator_index] =
			regulator_get(pdevice, str_regulator_name);

		if (preg->pregulator[sensor_index][regulator_index] != NULL) {
			regulator_status[sensor_index][regulator_index] = true;
		}
	}

	mutex_unlock(&g_regulator_state_mutex);

	return;
}

static void check_for_regulator_put(struct REGULATOR *preg,
	unsigned int sensor_index, unsigned int regulator_index)
{
	if (!preg) {
		pr_err("Fatal: Null ptr.\n");
		return;
	}

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
		regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid  sensor_idx:%d regulator_idx: %d\n",
			__func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == true) {
		regulator_put(preg->pregulator[sensor_index][regulator_index]);
		regulator_status[sensor_index][regulator_index] = false;
		preg->pregulator[sensor_index][regulator_index] = NULL;
	}

	mutex_unlock(&g_regulator_state_mutex);

	return;
}
static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_REGULATOR,
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.dump      = regulator_dump
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

