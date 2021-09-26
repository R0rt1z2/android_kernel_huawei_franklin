/*
 * fan53880.c
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
#include <linux/delay.h>
#include <linux/mutex.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/sensor/hw_pmic.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <hwmanufac/dev_detect/dev_detect.h>
#endif

#define HWLOG_TAG huawei_pmic
HWLOG_REGIST();

/* FAN53880 register define */
#define REVERSION_ID 0x01

/* BUCK/LDO ENABLE */
#define ENABLE_REG 0x09
#define LDO1_ENABLE_SHIFT 0x00
#define LDO2_ENABLE_SHIFT 0x01
#define LDO3_ENABLE_SHIFT 0x02
#define LDO4_ENABLE_SHIFT 0x03
#define BUCK_ENABLE_SHIFT 0x04
#define BUCK_PWM_MODE_MASK 0x40

/* LDO vol set */
#define LDO1_VOUT 0x04 /* 0.6V-3.775V 25mV/step */
#define LDO2_VOUT 0x05 /* 0.6V-3.775V 25mV/step */
#define LDO3_VOUT 0x06 /* 0.6V-3.775V 25mV/step */
#define LDO4_VOUT 0x07 /* 0.6V-3.775V 25mV/step */

/* vol range from 600000 -> 3775000 */
/* step 25000 (0.025mv) */
#define FAN53880_LDO_VOL_MIN     800000
#define FAN53880_LDO_VOL_MAX     3300000
#define FAN53880_LDO_VOL_STEP    25000
#define FAN53880_LDO_VOL_MIN_LEV 0x0F

/* Buck set */
#define BUCK_VOUT 0x02 /* 0.6V-3.3V 12.5mV/step */
#define FAN53880_BUCK_VOL_MIN     600000
#define FAN53880_BUCK_VOL_MAX     3300000
#define FAN53880_BUCK_VOL_STEP    12500
#define FAN53880_BUCK_VOL_MIN_LEV 0x1F

/* Boost set */
#define BOOST_CTL    0x03
#define BOOST_ENABLE 0x0A
#define BOOST_VOUT   0x11

/* Boost vol set range 3V-5.7V 25mV/step */
#define FAN53880_BOOST_VOL_MIN  3000000
#define FAN53880_BOOST_VOL_MAX  5700000
#define FAN53880_BOOST_VOL_STEP 25000

/* satus reg */
#define FAN53880_PMU_STATUS_REG 0x1A
#define FAN53880_OVP_STATUS_REG 0x11
#define FAN53880_OCP_STATUS_REG 0x12
#define FAN53880_UVP_STATUS_REG 0x13

/* Pin config */
#define FAN53880_PIN_ENABLE     1
#define FAN53880_PIN_DISABLE    0
#define FAN53880_POWERON_MASK   0x80
#define FAN53880_POWEROFF_MASK  0x7F
#define FAN53880_MASK_BOOST_5V  0x80
#define FAN53880_MASK_UVP_INT   0x20
#define FAN53880_MASK_BST_IPK_INT 0x80
#define FAN_MAX_LATCH_TIME        50000L
#define FAN_LATCH_CHECK_TIME      50
#define FAN_BOOST_SUSPEND         0x20

/* fan53880 pmic private data struct */
struct fan53880_pmic_data_t {
	unsigned int enable_pin;
	unsigned int ldo2_ctrl;
	unsigned int buck_pwm_mode;
};

struct voltage_map_t {
	int chx_enable;
	int vout_reg;
};

static struct voltage_map_t voltage_map[VOUT_MAX] = {
	{ ENABLE_REG, LDO1_VOUT },
	{ ENABLE_REG, LDO2_VOUT },
	{ ENABLE_REG, LDO3_VOUT },
	{ ENABLE_REG, LDO4_VOUT },
	{ 0, 0, }, /* stub not support LDO5 */
	{ ENABLE_REG, BUCK_VOUT },
};

static struct pmic_error_info_t pmic_error_map[] = {
	{ FAN53880_OCP_STATUS_REG, 0x00, 0xA0, PMIC_OCP_ERROR, DSM_CAMPMIC_OVER_CURRENT_ERROR_NO },
	{ FAN53880_OVP_STATUS_REG, 0x00, 0x00, PMIC_OVP_ERROR, DSM_CAMPMIC_OVER_CURRENT_ERROR_NO },
	{ FAN53880_UVP_STATUS_REG, 0x00, 0x80, PMIC_UVP_ERROR, DSM_CAMPMIC_UNDER_VOLTAGE_ERROR_NO },
};

/* static 53880 var */
static struct fan53880_pmic_data_t fan53880_pdata;
static struct hw_pmic_i2c_client fan53880_i2c_client;
static int boost_enabled;

DEFINE_HW_PMIC_MUTEX(fan53880);

static u32 calc_voltage_level(u32 voltage, u32 max, u32 min, u32 step)
{
	hwlog_info("%s enter\n", __func__);
	if ((voltage > max) || (voltage < min) || (step == 0))
		return min;

	return (voltage - min) / step;
}

static int fan53880_check_null(struct hw_pmic_ctrl_t *pmic_ctrl)
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

static int fan53880_boost_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 voltage, int state)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 chx_enable = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);

	if (seq_index != VOUT_BOOST) {
		hwlog_err("%s seq_index-%d error\n", __func__, seq_index);
		return -EFAULT;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state-%d error\n", __func__, state);
		return -EFAULT;
	}

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	i2c_func->i2c_read(i2c_client, BOOST_ENABLE, &chx_enable);

	if (state == PMIC_POWER_ON)
		chx_enable = chx_enable | 0x01; /* set boost on */
	else
		chx_enable = 0;

	ret = i2c_func->i2c_write(i2c_client, BOOST_ENABLE, chx_enable);
	hwlog_info("%s, power state%d, chx_enable%u\n", __func__,
		state, chx_enable);

	/* vbus need 700us-1ms raise to stable 5v */
	if (state == PMIC_POWER_ON)
		usleep_range(700, 1000);

	return ret;
}

static int fan53880_boost_seq_config_with_flag(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 voltage, int state)
{
	int ret;

	mutex_lock(&pmic_mut_fan53880);
	ret = fan53880_boost_seq_config(pmic_ctrl, seq_index, voltage, state);
	boost_enabled = state;
	mutex_unlock(&pmic_mut_fan53880);

	return ret;
}

static int fan53880_buck_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 volt, int state)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	struct fan53880_pmic_data_t *pdata = NULL;

	u8 chx_enable = 0;
	u8 chx_enable_reg;
	u8 volt_reg;
	u32 volt_level = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (seq_index != VOUT_BUCK_1) {
		hwlog_err("%s set seq_index %u error\n", __func__, seq_index);
		return -EINVAL;
	}

	if ((state == PMIC_POWER_ON) &&
		((volt > FAN53880_BUCK_VOL_MAX) ||
		(volt < FAN53880_BUCK_VOL_MIN))) {
		hwlog_err("%s voltage error, vol-%u\n", __func__, volt);
		return -EINVAL;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state-%u error\n", __func__, state);
		return -EINVAL;
	}

	pdata = pmic_ctrl->pdata;
	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	chx_enable_reg = voltage_map[seq_index].chx_enable;
	volt_reg = voltage_map[seq_index].vout_reg;
	i2c_func->i2c_read(i2c_client, chx_enable_reg, &chx_enable);

	if (state == PMIC_POWER_ON) {
		volt_level = calc_voltage_level(volt,
			FAN53880_BUCK_VOL_MAX,
			FAN53880_BUCK_VOL_MIN,
			FAN53880_BUCK_VOL_STEP);
		volt_level += FAN53880_BUCK_VOL_MIN_LEV;
		/* set voltage */
		i2c_func->i2c_write(i2c_client, volt_reg, volt_level);
		if (pdata->buck_pwm_mode == BUCK_PWM_MODE)
			chx_enable = chx_enable | (0x1 << BUCK_ENABLE_SHIFT) | BUCK_PWM_MODE_MASK;
		else
			chx_enable = chx_enable | (0x1 << BUCK_ENABLE_SHIFT);
	} else {
		/* disable vol */
		chx_enable = chx_enable & (~(0x1 << BUCK_ENABLE_SHIFT));
	}

	ret = i2c_func->i2c_write(i2c_client, chx_enable_reg, chx_enable);
	hwlog_info("%s, set buck state %d, chx_reg %u, reg %u, vol %u, v_level %u\n",
		__func__, state, chx_enable_reg, volt_reg, volt, volt_level);

	return ret;
}

static int fan53880_ldo_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index seq_index, u32 volt, int state)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 chx_enable;
	u8 chx_enable_reg;
	u8 voltage_reg;
	u32 volt_level = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if ((seq_index < VOUT_LDO_1) || (seq_index > VOUT_LDO_4)) {
		hwlog_err("%s seq_index-%u error\n", __func__, seq_index);
		return -EFAULT;
	}

	if ((state == PMIC_POWER_ON) &&
		((volt > FAN53880_LDO_VOL_MAX) ||
		(volt < FAN53880_LDO_VOL_MIN))) {
		hwlog_err("%s voltage error, vol-%u\n", __func__, volt);
		return -EFAULT;
	}

	if ((state != PMIC_POWER_OFF) && (state != PMIC_POWER_ON)) {
		hwlog_err("%s state-%d error", __func__, state);
		return -EFAULT;
	}

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	chx_enable_reg = voltage_map[seq_index].chx_enable;
	voltage_reg = voltage_map[seq_index].vout_reg;
	i2c_func->i2c_read(i2c_client, chx_enable_reg, &chx_enable);

	if (state == PMIC_POWER_ON) {
		volt_level = calc_voltage_level(volt,
			FAN53880_LDO_VOL_MAX,
			FAN53880_LDO_VOL_MIN,
			FAN53880_LDO_VOL_STEP);
		volt_level += FAN53880_LDO_VOL_MIN_LEV;
		/* set voltage */
		i2c_func->i2c_write(i2c_client, voltage_reg, volt_level);
		chx_enable = chx_enable | (0x01 << (unsigned int)seq_index);
	} else {
		/* disable set */
		chx_enable = chx_enable & (~(0x01 << (unsigned int)seq_index));
	}

	ret = i2c_func->i2c_write(i2c_client, chx_enable_reg, chx_enable);
	hwlog_info("%s, power state %d, chx_reg %u, reg %u, vol %u, v_level %u",
		__func__, state, chx_enable_reg, voltage_reg, volt, volt_level);

	return ret;
}

static int fan53880_seq_config(struct hw_pmic_ctrl_t *pmic_ctrl,
	enum pmic_seq_index index, u32 vol, int state)
{
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (fan53880_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	mutex_lock(&pmic_mut_fan53880);

	if (index == VOUT_BOOST)
		ret = fan53880_boost_seq_config_with_flag(pmic_ctrl, index, vol, state);
	else if (index == VOUT_BOOST_EN)
		ret = hw_pmic_gpio_boost_enable(pmic_ctrl, state);
	else if (index < VOUT_LDO_5)
		ret = fan53880_ldo_seq_config(pmic_ctrl, index, vol, state);
	else
		ret = fan53880_buck_seq_config(pmic_ctrl, index, vol, state);

	mutex_unlock(&pmic_mut_fan53880);
	return ret;
}

static int fan53880_match(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static int fan53880_get_dt_data(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct fan53880_pmic_data_t *pdata = NULL;
	struct device_node *dev_node = NULL;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->pdata || !pmic_ctrl->dev) {
		hwlog_err("%s pmic ctrl is NULL\n", __func__);
		return -EFAULT;
	}

	pdata = pmic_ctrl->pdata;
	dev_node = pmic_ctrl->dev->of_node;

	/* PIN Enable gpio */
	pdata->enable_pin = of_get_named_gpio(dev_node, "pmic-pin", 0);
	if (pdata->enable_pin <= 0)
		hwlog_err("ERROR %s huawei,pmic-pin%u", __func__, pdata->enable_pin);
	if (!gpio_is_valid(pdata->enable_pin))
		hwlog_err("not valid %s huawei,pmic-pin%u", __func__, pdata->enable_pin);
	hwlog_info("GET %s huawei,pmic-pin%u", __func__, pdata->enable_pin);

	/* buck pwm forced mode */
	if (of_property_read_u32(dev_node, "mtk,pmic_buck_pwm_mode",
		&pdata->buck_pwm_mode) < 0) {
		pdata->buck_pwm_mode = BUCK_NORMAL_MODE;
		hwlog_info("%s, cannot get buck config, set to default buck mode", __func__);
	}
	hwlog_info("%s pmic_buck_pwm_mode %u", __func__, pdata->buck_pwm_mode);

	return 0;
}

static int fan53880_on(struct hw_pmic_ctrl_t *pmic_ctrl, void *data)
{
	/* check Error registor */
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static int fan53880_off(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	/* Never power off pmic when SOC is running; */
	hwlog_info("%s enter\n", __func__);
	return 0;
}

static void pmic_boost_reset(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	u8 pmu_status = 0;

	mutex_lock(&pmic_mut_fan53880);
	if (boost_enabled == PMIC_POWER_OFF) {
		mutex_unlock(&pmic_mut_fan53880);
		return;
	}

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	/* read pmic suspend status */
	i2c_func->i2c_read(i2c_client, FAN53880_PMU_STATUS_REG, &pmu_status);
	hwlog_info("%s pmu-0x%x, boost-%d\n", __func__, pmu_status, boost_enabled);

	if ((pmu_status & FAN_BOOST_SUSPEND) != FAN_BOOST_SUSPEND) {
		hwlog_info("%s, boost fault recovery\n", __func__);
		mutex_unlock(&pmic_mut_fan53880);
		return;
	}

	/* reset boost en */
	hwlog_info("%s reset boost when boost still exist\n", __func__);
	fan53880_boost_seq_config(pmic_ctrl, VOUT_BOOST, 0, PMIC_POWER_OFF);
	usleep_range(700, 1000); /* vbus need 700us-1ms raise to stable 5v */
	fan53880_boost_seq_config(pmic_ctrl, VOUT_BOOST, 0, PMIC_POWER_ON);
	mutex_unlock(&pmic_mut_fan53880);
}

static void fan53880_pmic_reset(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	pmic_boost_reset(pmic_ctrl);
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
	if (fan53880_check_null(pmic_ctrl) < 0)
		return;

	fan53880_pmic_reset(pmic_ctrl);
	pmic_wake_unlock(pmic_ctrl);
}

static int fan53880_init(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	struct fan53880_pmic_data_t *pdata = NULL;
	u8 chip_id = 0;
	int ret;

	hwlog_info("%s enter\n", __func__);
	if (fan53880_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	ret = hw_pmic_setup_intr(pmic_ctrl);
	if (ret < 0)
		hwlog_err("%s setup interrupt failed\n", __func__);

	pdata = pmic_ctrl->pdata;

	hwlog_info("%s power enbale pin = %d\n", __func__, pdata->enable_pin);
	ret = gpio_request(pdata->enable_pin, "fan53880_pmic_enable");
	if (ret < 0) {
		hwlog_err("%s fail request power-ctrl = %d\n", __func__, ret);
		goto req_failed;
	}

	gpio_direction_output(pdata->enable_pin, FAN53880_PIN_ENABLE);
	usleep_range(1000, 1000);

	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;
	ret = i2c_func->i2c_read(i2c_client, REVERSION_ID, &chip_id);
	if (ret < 0) {
		hwlog_err("%s: read CHIP ID failed, ret = %d\n", __func__, ret);
		goto read_id_fail;
	}
	hwlog_info("%s chip id = %d, register success\n", __func__, chip_id);
	hwlog_info("%s BOOST 5V SETUP!\n", __func__);
	/* BOOST 5V always on */
	if (fan53880_boost_seq_config_with_flag(pmic_ctrl, VOUT_BOOST, 5000000,
		PMIC_POWER_ON) < 0)
		hwlog_err("%s Boost output 5V failed\n", __func__);

	INIT_DELAYED_WORK(&pmic_ctrl->pmic_err_work, pmic_fault_err_work);
	wakeup_source_init(&pmic_ctrl->pmic_wake_lock, "fan53880_fault_wake");

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_PMIC);
#endif
	return 0;

read_id_fail:
	hwlog_err("%s gpio_free:%d\n", __func__, pdata->enable_pin);
	gpio_free(pdata->enable_pin);
req_failed:
	hwlog_err("%s hw_pmic_release_inter\n", __func__);
	hw_pmic_release_intr(pmic_ctrl);
	return ret;
}

static int fan53880_exit(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct fan53880_pmic_data_t *pdata = NULL;

	hwlog_info("%s enter\n", __func__);

	if (!pmic_ctrl || !pmic_ctrl->pdata) {
		hwlog_err("%s pmic_ctrl is NULL\n", __func__);
		return -1;
	}

	pdata = pmic_ctrl->pdata;
	gpio_free(pdata->enable_pin);
	cancel_delayed_work(&pmic_ctrl->pmic_err_work);
	flush_delayed_work(&pmic_ctrl->pmic_err_work);
	wakeup_source_trash(&pmic_ctrl->pmic_wake_lock);
	hw_pmic_release_intr(pmic_ctrl);
	return 0;
}

static int pmic_check_state_exception(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	static struct irq_err_monitor irq_err;

	hwlog_info("%s enter\n", __func__);
	if (fan53880_check_null(pmic_ctrl) < 0)
		return -EFAULT;

	/* add pmic error dmd */
	pmic_fault_check(pmic_ctrl, FAN53880_PMU_STATUS_REG,
		pmic_error_map, array_size(pmic_error_map));

	pmic_fault_reset_check(pmic_ctrl, &irq_err, FAN_MAX_LATCH_TIME,
		FAN_LATCH_CHECK_TIME);

	return 0;
}

static struct hw_pmic_fn_t fan53880_func_tbl = {
	.pmic_init = fan53880_init,
	.pmic_exit = fan53880_exit,
	.pmic_on = fan53880_on,
	.pmic_off = fan53880_off,
	.pmic_match = fan53880_match,
	.pmic_get_dt_data = fan53880_get_dt_data,
	.pmic_seq_config = fan53880_seq_config,
	.pmic_check_exception = pmic_check_state_exception,
	.pmic_power_cfg = hw_pmic_power_cfg,
	.pmic_reset = hw_pmic_reset,
};

static struct hw_pmic_ctrl_t fan53880_ctrl = {
	.pmic_i2c_client = &fan53880_i2c_client,
	.func_tbl = &fan53880_func_tbl,
	.hw_pmic_mutex = &pmic_mut_fan53880,
	.pdata = (void *)&fan53880_pdata,
};

static const struct i2c_device_id fan53880_id[] = {
	{ "fan53880", (unsigned long)&fan53880_ctrl },
	{}
};

static const struct of_device_id fan53880_dt_match[] = {
	{ .compatible = "mtk,fan53880" },
	{}
};
MODULE_DEVICE_TABLE(of, fan53880_dt_match);

static int fan53880_remove(struct i2c_client *client)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = hw_get_pmic_ctrl();
	hwlog_info("%s enter\n", __func__);
	if (!client || !client->adapter)
		return 0;
	client->adapter = NULL;
	fan53880_exit(pmic_ctrl);
	return 0;
}

static struct i2c_driver fan53880_i2c_driver = {
	.probe = hw_pmic_i2c_probe,
	.remove = fan53880_remove,
	.id_table = fan53880_id,
	.driver = {
		.name = "fan53880",
		.of_match_table = fan53880_dt_match,
	},
};

static int __init fan53880_module_init(void)
{
	hwlog_info("%s enter\n", __func__);
	return i2c_add_driver(&fan53880_i2c_driver);
}

static void __exit fan53880_module_exit(void)
{
	hwlog_info("%s enter\n", __func__);
	i2c_del_driver(&fan53880_i2c_driver);
}

subsys_initcall(fan53880_module_init);
module_exit(fan53880_module_exit);

MODULE_DESCRIPTION("FAN53880 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

