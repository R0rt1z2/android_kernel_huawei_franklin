/*
 * rt5112.c
 *
 * debug for pmic sensor
 *
 * Copyright (c) 2019 Huawei Technologies Co., Ltd.
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
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/sensor/hw_pmic.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <hwmanufac/dev_detect/dev_detect.h>
#endif

#define HWLOG_TAG huawei_pmic
HWLOG_REGIST();

/* pmic chip id */
#define REVERSION_ID 0x2A

/* LDO */
#define LDO1_CTL  0x01
#define LDO2_CTL  0x02
#define LDO3_CTL  0x03
#define LDO4_CTL  0x04
#define LDO1_VOUT 0x0D /* 0.6V-3.775V 25mV/step */
#define LDO2_VOUT 0x0E /* 0.6V-3.775V 25mV/step */
#define LDO3_VOUT 0x0F /* 0.6V-3.775V 25mV/step */
#define LDO4_VOUT 0x10 /* 0.6V-3.775V 25mV/step */

/* 600000 -> 3775000 */
#define RT5112_LDO_VOL_MIN 600000
#define RT5112_LDO_VOL_MAX 3775000

/* step 25000 (0.025) */
#define RT5112_LDO_VOL_STEP 25000

/* Buck */
#define BUCK_CTL   0x00
#define BUCK2_CTL  0x06
#define BUCK_VOUT  0x0C /* 0.6V-3.3V 12.5mV/step */
#define BUCK2_VOUT 0x12 /* 0.6V-3.3V 12.5mV/step */
#define RT5112_BUCK_VOL_MIN  600000
#define RT5112_BUCK_VOL_MAX  3300000
#define RT5112_BUCK_VOL_STEP 12500

/* Boost */
#define BOOST_CTL    0x05
#define BOOST_VOUT   0x11 /* 4.5V-5.5V 25mV/step */
#define RT5112_BOOST_ENABLE    0x08
#define RT5112_BOOST_POWER_ON  0x01
#define RT5112_BOOST_POWER_OFF 0x00
#define RT5112_BOOST_VOL_MIN   4500000
#define RT5112_BOOST_VOL_MAX   5500000
#define RT5112_BOOST_VOL_STEP  25000

/* OVP && OCP */
#define RT5112_PMU_STATUS_REG 0x43
#define RT5112_UVP_STATUS_REG 0x15
#define RT5112_OCP_STATUS_REG 0x17
#define RT5112_OVP_STATUS_REG 0x16

/* interrupt */
#define RT5112_INT_STATUS_REG 0x14
#define RT5112_MASK_INTR_REG  0x33
#define RT5112_MASK_BOOST_5V  0x04
#define RT5112_MASK_UV_EVT    0x04
#define RT5112_MASK_FAULT_EVT 0x01
#define RT5112_MASK_OC_EVT    0x04

/* PIN */
#define RT5112_PIN_ENABLE    1
#define RT5112_PIN_DISABLE   0
#define RT5112_POWERON_MASK  0x80
#define RT5112_POWEROFF_MASK 0x7F

/* BUCK_PWM_MODE */
#define RT5112_BUCK_POWERON_MASK  0x81
#define RT5112_BUCK_POWEROFF_MASK 0x7E
#define RT5112_LDO2_CTRL          1
#define RT5112_MAX_LATCH_TIME     500000L
#define RT5112_LATCH_CHECK_TIME   500
#define RT5112_PMIC_LATCH_RESET   1
#define PMIC_STAT_OK              0

/* define mutex for avoid competition */
DEFINE_HW_PMIC_MUTEX(rt5112);

/* rt5112 private pmic struct data */
struct rt5112_pmic_data_t {
	unsigned int enable_pin;
	unsigned int intr_pin;
	unsigned int buck_pwm_mode;
};

struct voltage_map_t {
	int chx_enable;
	int vout_reg;
};

static struct voltage_map_t voltage_map[VOUT_MAX] = {
	{ LDO1_CTL, LDO1_VOUT },
	{ LDO2_CTL, LDO2_VOUT },
	{ LDO3_CTL, LDO3_VOUT },
	{ LDO4_CTL, LDO4_VOUT },
	{ 0, 0, }, /* stub not support LDO5 */
	{ BUCK_CTL, BUCK_VOUT },
};

static struct pmic_error_info_t pmic_error_map[] = {
	{ RT5112_OCP_STATUS_REG, 0x00, 0x04, PMIC_OCP_ERROR, DSM_CAMPMIC_OVER_CURRENT_ERROR_NO },
	{ RT5112_OVP_STATUS_REG, 0x00, 0x04, PMIC_OVP_ERROR, DSM_CAMPMIC_OVER_CURRENT_ERROR_NO },
	{ RT5112_UVP_STATUS_REG, 0x00, 0x05, PMIC_UVP_ERROR, DSM_CAMPMIC_UNDER_VOLTAGE_ERROR_NO },
};

/* static var */
static struct rt5112_pmic_data_t rt5112_pdata;
static struct hw_pmic_i2c_client rt5112_i2c_client;
static int pmic_en_reset;
static int boost_enabled;

static u32 calc_voltage_level(u32 voltage, u32 max, u32 min, u32 step)
{
	hwlog_info("%s enter\n", __func__);
	if ((voltage > max) || (voltage < min) || (step == 0)) {
		hwlog_info("votage %u limit to min\n", voltage);
		return min;
	}

	return (voltage - min) / step;
}

static int rt5122_check_null(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->pmic_i2c_client || !pmic_ctrl->pdata) {
		hwlog_err("%s pmic_ctrl is NULL\n", __func__);
		return -EFAULT;
	}
	if (!pmic_ctrl->pmic_i2c_client->i2c_func_tbl ||
		!pmic_ctrl->pmic_i2c_client->i2c_func_tbl->i2c_read ||
		!pmic_ctrl->pmic_i2c_client->i2c_func_tbl->i2c_write) {
		hwlog_err("%s i2c read write func is NULL\n", __func__);
		return -EFAULT;
	}
	return 0;
}

static int rt5112_boost_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 voltage, int state)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 chx_enable = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (seq_index != VOUT_BOOST) {
		hwlog_err("%s error, seq_index-%d\n", __func__, seq_index);
		return -EPERM;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state %d error\n", __func__, state);
		return -EPERM;
	}

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	i2c_func->i2c_read(i2c_client, RT5112_BOOST_ENABLE, &chx_enable);

	if (state == PMIC_POWER_ON) {
		chx_enable = chx_enable | RT5112_BOOST_POWER_ON;
		hwlog_info("%s BOOST POWER ON : 0x%x\n", __func__, chx_enable);
	} else {
		chx_enable = chx_enable & RT5112_BOOST_POWER_OFF;
		hwlog_info("%s BOOST POWER OFF : 0x%x\n", __func__, chx_enable);
	}

	ret = i2c_func->i2c_write(i2c_client, RT5112_BOOST_ENABLE, chx_enable);
	hwlog_info("%s, bst onoff-%d, reg-%u, ret %d\n", __func__, state, chx_enable, ret);

	return ret;
}

static int rt5112_boost_seq_config_with_flag(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 voltage, int state)
{
	int ret;

	mutex_lock(&pmic_mut_rt5112);
	ret = rt5112_boost_seq_config(pmic_ctrl, seq_index, voltage, state);
	boost_enabled = state;
	mutex_unlock(&pmic_mut_rt5112);

	return ret;
}

static int rt5112_buck_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 volt, int state)
{
	struct rt5112_pmic_data_t *pdata = NULL;
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 chx_enable = 0;
	u8 chx_enable_reg;
	u8 volt_reg;
	u32 volt_level = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (seq_index != VOUT_BUCK_1) {
		hwlog_err("%s index err, seq_index-%u\n", __func__, seq_index);
		return -EFAULT;
	}

	if ((state == PMIC_POWER_ON) &&
		((volt > RT5112_BUCK_VOL_MAX) ||
		(volt < RT5112_BUCK_VOL_MIN))) {
		hwlog_err("%s voltage error, vol-%u\n", __func__, volt);
		return -EFAULT;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state-%u error\n", __func__, state);
		return -1;
	}

	pdata = pmic_ctrl->pdata;
	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	chx_enable_reg = voltage_map[seq_index].chx_enable;
	volt_reg = voltage_map[seq_index].vout_reg;
	i2c_func->i2c_read(i2c_client, chx_enable_reg, &chx_enable);

	if (state == PMIC_POWER_ON) {
		volt_level = calc_voltage_level(volt,
			RT5112_BUCK_VOL_MAX,
			RT5112_BUCK_VOL_MIN,
			RT5112_BUCK_VOL_STEP);
		/* set voltage */
		i2c_func->i2c_write(i2c_client, volt_reg, volt_level);
		if (pdata->buck_pwm_mode == BUCK_PWM_MODE)
			chx_enable = chx_enable | RT5112_BUCK_POWERON_MASK;
		else
			chx_enable = chx_enable | RT5112_POWERON_MASK;
	} else {
		if (pdata->buck_pwm_mode == BUCK_PWM_MODE)
			chx_enable = chx_enable & RT5112_BUCK_POWEROFF_MASK;
		else
			chx_enable = chx_enable & RT5112_POWEROFF_MASK;
	}
	ret = i2c_func->i2c_write(i2c_client, chx_enable_reg, chx_enable);
	hwlog_info("power state-%d, chx_reg-%u, reg-%u, vol-%u, vol_level-%u, ret-%d\n",
		state, chx_enable_reg, volt_reg, volt, volt_level, ret);

	return ret;
}

/* RT5112 ldo set */
static int rt5112_ldo_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 voltage, int state)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 chx_enable = 0;
	u8 chx_enable_reg;
	u8 voltage_reg;
	u32 volt_level;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if ((seq_index < VOUT_LDO_1) || (seq_index > VOUT_LDO_4)) {
		hwlog_err("%s seq_index err, dex-%u\n", __func__, seq_index);
		return -EFAULT;
	}

	if ((state == PMIC_POWER_ON) &&
		((voltage > RT5112_LDO_VOL_MAX) ||
		(voltage < RT5112_LDO_VOL_MIN))) {
		hwlog_err("%s voltage error, vol-%u\n", __func__, voltage);
		return -EFAULT;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state -%d error", __func__, state);
		return -EFAULT;
	}

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	chx_enable_reg = voltage_map[seq_index].chx_enable;
	voltage_reg = voltage_map[seq_index].vout_reg;
	i2c_func->i2c_read(i2c_client, chx_enable_reg, &chx_enable);

	if (state == PMIC_POWER_ON) {
		/* set voltage level */
		volt_level = calc_voltage_level(voltage,
			RT5112_LDO_VOL_MAX,
			RT5112_LDO_VOL_MIN,
			RT5112_LDO_VOL_STEP);
		volt_level = volt_level << 1;
		i2c_func->i2c_write(i2c_client, voltage_reg, volt_level);
		chx_enable = chx_enable | RT5112_POWERON_MASK;
	} else {
		chx_enable = chx_enable & RT5112_POWEROFF_MASK;
	}
	ret = i2c_func->i2c_write(i2c_client, chx_enable_reg, chx_enable);
	hwlog_info("set ldo-%d, enable-%d, voltage-%d\n",
		seq_index, state, voltage);
	return ret;
}

static int rt5112_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index index, u32 volt, int state)
{
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (rt5122_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	mutex_lock(&pmic_mut_rt5112);
	if (index == VOUT_BOOST)
		ret = rt5112_boost_seq_config_with_flag(pmic_ctrl, index, volt, state);
	else if (index == VOUT_BOOST_EN)
		ret = hw_pmic_gpio_boost_enable(pmic_ctrl, state);
	else if (index < VOUT_LDO_5)
		ret = rt5112_ldo_seq_config(pmic_ctrl, index, volt, state);
	else
		ret = rt5112_buck_seq_config(pmic_ctrl, index, volt, state);
	hwlog_info("%s, set power-%d, volt-%d, state to %d, ret = %d\n",
		__func__, index, volt, state, ret);
	mutex_unlock(&pmic_mut_rt5112);
	return ret;
}

static int rt5112_match(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static int rt5112_get_dt_data(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct rt5112_pmic_data_t *pdata = NULL;
	struct device_node *dev_node = NULL;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->pdata || !pmic_ctrl->dev) {
		hwlog_err("%s pmic ctrl is NULL\n", __func__);
		return -EFAULT;
	}

	pdata = pmic_ctrl->pdata;
	dev_node = pmic_ctrl->dev->of_node;

	/* get pmic Enable gpio */
	pdata->enable_pin = of_get_named_gpio(dev_node, "pmic-pin", 0);
	if (pdata->enable_pin <= 0)
		hwlog_err("ERROR %s huawei,pmic-pin %u", __func__,
			pdata->enable_pin);
	if (!gpio_is_valid(pdata->enable_pin))
		hwlog_err("not valid %s huawei,pmic-pin %u", __func__,
			pdata->enable_pin);
	hwlog_info("GET %s huawei,pmic-pin %u", __func__, pdata->enable_pin);

	/* buck pwm forced mode */
	if (of_property_read_u32(dev_node, "mtk,pmic_buck_pwm_mode",
		&pdata->buck_pwm_mode) < 0) {
		pdata->buck_pwm_mode = BUCK_NORMAL_MODE;
		hwlog_info("%s, cannot get buck config, set to default buck mode", __func__);
	}
	hwlog_info("%s pmic_buck_pwm_mode %u", __func__, pdata->buck_pwm_mode);

	return 0;
}

static int rt5112_on(struct hw_pmic_ctrl_t *pmic_ctrl, void *data)
{
	/* check Error registor */
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static int rt5112_off(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	/* Never power off pmic when SOC is running; */
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static void pmic_hw_en_reset(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct rt5112_pmic_data_t *pdata = NULL;

	pdata = pmic_ctrl->pdata;
	if (!pdata) {
		hwlog_err("%s null", __func__);
		return;
	}
	gpio_set_value(pdata->enable_pin, RT5112_PIN_DISABLE);
	usleep_range(900, 1000); /* disable to enable, ic need 900us-1ms delay */
	gpio_set_value(pdata->enable_pin, RT5112_PIN_ENABLE);
	hwlog_err("pmic fault err, set %s", __func__);
}

static void rt5112_reset(struct hw_pmic_ctrl_t *pmic_ctrl, u8 pmu_status)
{
	if (pmu_status == PMIC_STAT_OK ||
		pmic_en_reset != RT5112_PMIC_LATCH_RESET)
		return;

	pmic_hw_en_reset(pmic_ctrl);
	pmic_en_reset = 0;
	mutex_lock(&pmic_mut_rt5112);
	if (!boost_enabled) {
		mutex_unlock(&pmic_mut_rt5112);
		return;
	}
	hwlog_info("%s, pmic err, reset boost power", __func__);
	rt5112_boost_seq_config(pmic_ctrl, VOUT_BOOST, 0, PMIC_POWER_ON);
	mutex_unlock(&pmic_mut_rt5112);
}

static void rt5112_clear_interrupt(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 reg_value = 0;
	u8 pmu_status = 0;

	hwlog_info("%s enter\n", __func__);
	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	/* clean status */
	i2c_func->i2c_read(i2c_client, RT5112_PMU_STATUS_REG, &pmu_status);
	i2c_func->i2c_read(i2c_client, RT5112_OVP_STATUS_REG, &reg_value);
	i2c_func->i2c_read(i2c_client, RT5112_OCP_STATUS_REG, &reg_value);
	i2c_func->i2c_read(i2c_client, RT5112_UVP_STATUS_REG, &reg_value);

	/* clear interrupt */
	i2c_func->i2c_read(i2c_client, RT5112_INT_STATUS_REG, &reg_value);
	/* clear intr by write bit0 to 1 */
	reg_value = reg_value | 0x01;
	i2c_func->i2c_write(i2c_client, RT5112_INT_STATUS_REG, reg_value);
	hwlog_info("%s pmu_status = 0x%x, en = %d", __func__, pmu_status, pmic_en_reset);

	rt5112_reset(pmic_ctrl, pmu_status);
}

static void pmic_fault_err_work(struct work_struct *work)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = NULL;

	if (!work) {
		hwlog_err("work is null\n");
		return;
	}

	pmic_ctrl = container_of(work, struct hw_pmic_ctrl_t,
		pmic_err_work.work);
	if (rt5122_check_null(pmic_ctrl) < 0)
		return;

	pmic_en_reset = RT5112_PMIC_LATCH_RESET;
	rt5112_clear_interrupt(pmic_ctrl);
	pmic_wake_unlock(pmic_ctrl);
}

static void rt5112_shield_boost_interrupt(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;

	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func->i2c_write(i2c_client, RT5112_MASK_INTR_REG,
		RT5112_MASK_BOOST_5V);
	hwlog_info("%s shield ic boost interrupt\n", __func__);
}

static int pmic_check_state_exception(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	static struct irq_err_monitor irq_err;

	hwlog_info("%s enter\n", __func__);
	if (rt5122_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	/* add pmic error dmd */
	pmic_fault_check(pmic_ctrl, RT5112_PMU_STATUS_REG,
		pmic_error_map, array_size(pmic_error_map));

	rt5112_clear_interrupt(pmic_ctrl);
	pmic_fault_reset_check(pmic_ctrl, &irq_err, RT5112_MAX_LATCH_TIME,
		RT5112_LATCH_CHECK_TIME);

	/* shield boost interrupt avoid ic bug */
	if (pmic_ctrl->pmic_info.boost_shield_irq)
		rt5112_shield_boost_interrupt(pmic_ctrl);

	return 0;
}

static int rt5112_exit(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct rt5112_pmic_data_t *pdata = NULL;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->pdata) {
		hwlog_err("%s pmic_ctrl is NULL", __func__);
		return -EFAULT;
	}
	pdata = pmic_ctrl->pdata;
	gpio_free(pdata->enable_pin);
	cancel_delayed_work(&pmic_ctrl->pmic_err_work);
	flush_delayed_work(&pmic_ctrl->pmic_err_work);
	wakeup_source_trash(&pmic_ctrl->pmic_wake_lock);
	hw_pmic_release_intr(pmic_ctrl);
	return 0;
}

static int rt5112_init(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	struct rt5112_pmic_data_t *pdata = NULL;
	u8 chip_id = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (rt5122_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	ret = hw_pmic_setup_intr(pmic_ctrl);
	if (ret < 0)
		hwlog_err("%s setup interrupt failed\n", __func__);

	pdata = pmic_ctrl->pdata;

	hwlog_info("%s power enbale pin = %d\n", __func__, pdata->enable_pin);
	ret = gpio_request(pdata->enable_pin, "pmic-enable-ctrl");
	if (ret < 0) {
		hwlog_err("%s fail request enable pin = %d\n", __func__, ret);
		goto req_failed;
	}

	gpio_direction_output(pdata->enable_pin, RT5112_PIN_ENABLE);
	usleep_range(1000, 1000);

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	ret = i2c_func->i2c_read(i2c_client, REVERSION_ID, &chip_id);
	if (ret < 0) {
		hwlog_err("%s: read CHIP ID failed, ret = %d ", __func__, ret);
		goto read_id_fail;
	}
	hwlog_info("%s chip id = %d, register success\n", __func__, chip_id);
	hwlog_info("%s BOOST 5V SETUP!\n", __func__);
	/* BOOST 5V always on */
	if (rt5112_boost_seq_config_with_flag(pmic_ctrl, VOUT_BOOST, 5000000,
		PMIC_POWER_ON) < 0)
		hwlog_err("%s Boost output 5V failed\n", __func__);

	/* mask boost interrupt */
	if (pmic_ctrl->pmic_info.boost_shield_irq) {
		i2c_func->i2c_write(i2c_client, RT5112_MASK_INTR_REG,
			RT5112_MASK_BOOST_5V);
		hwlog_info("%s shield ic boost interrupt\n", __func__);
	}
	/* clean interrupt */
	rt5112_clear_interrupt(pmic_ctrl);

	INIT_DELAYED_WORK(&pmic_ctrl->pmic_err_work, pmic_fault_err_work);
	wakeup_source_init(&pmic_ctrl->pmic_wake_lock, "rt5112_fault_wake");

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_PMIC);
#endif
	hwlog_info("%s success\n", __func__);
	return 0;

read_id_fail:
	hwlog_err("%s gpio_free:%d\n", __func__, pdata->enable_pin);
	gpio_free(pdata->enable_pin);

req_failed:
	hwlog_err("%s hw_pmic_release_inter\n", __func__);
	hw_pmic_release_intr(pmic_ctrl);
	return ret;
}

static int rt5112_remove(struct i2c_client *client)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = hw_get_pmic_ctrl();
	hwlog_info("%s enter\n", __func__);
	if (!client || !client->adapter)
		return 0;
	rt5112_exit(pmic_ctrl);
	client->adapter = NULL;
	return 0;
}

static struct hw_pmic_fn_t rt5112_func_tbl = {
	.pmic_init = rt5112_init,
	.pmic_exit = rt5112_exit,
	.pmic_on = rt5112_on,
	.pmic_off = rt5112_off,
	.pmic_match = rt5112_match,
	.pmic_get_dt_data = rt5112_get_dt_data,
	.pmic_seq_config = rt5112_seq_config,
	.pmic_check_exception = pmic_check_state_exception,
	.pmic_power_cfg = hw_pmic_power_cfg,
	.pmic_reset = hw_pmic_reset,
};

static struct hw_pmic_ctrl_t rt5112_ctrl = {
	.pmic_i2c_client = &rt5112_i2c_client,
	.func_tbl = &rt5112_func_tbl,
	.hw_pmic_mutex = &pmic_mut_rt5112,
	.pdata = (void *)&rt5112_pdata,
};

static const struct i2c_device_id rt5112_id[] = {
	{ "rt5112", (unsigned long)&rt5112_ctrl },
	{}
};

static const struct of_device_id rt5112_dt_match[] = {
	{ .compatible = "mtk,rt5112" },
	{}
};
MODULE_DEVICE_TABLE(of, rt5112_dt_match);

static struct i2c_driver rt5112_i2c_driver = {
	.probe = hw_pmic_i2c_probe,
	.remove = rt5112_remove,
	.id_table = rt5112_id,
	.driver = {
		.name = "rt5112",
		.of_match_table = rt5112_dt_match,
	},
};

static int __init rt5112_module_init(void)
{
	hwlog_info("%s enter\n", __func__);
	return i2c_add_driver(&rt5112_i2c_driver);
}

static void __exit rt5112_module_exit(void)
{
	hwlog_info("%s enter\n", __func__);
	i2c_del_driver(&rt5112_i2c_driver);
}

subsys_initcall(rt5112_module_init);
module_exit(rt5112_module_exit);

MODULE_DESCRIPTION("RT5112 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

