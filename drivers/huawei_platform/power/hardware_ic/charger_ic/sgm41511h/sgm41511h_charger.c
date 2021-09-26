/*
 * sgm41511h_charger.c
 *
 * sgm41511h driver
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

#include <sgm41511h_charger.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/raid/pq.h>
#include <linux/power/huawei_charger.h>
#include <linux/power/huawei_mtk_charger.h>
#include <mt-plat/mtk_battery.h>
#include <chipset_common/hwpower/common_module/power_log.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#include <chipset_common/hwpower/common_module/power_devices_info.h>

#define MIN_CURRENT                  1
#define UNIT_CONVERSION              1000
#define MSG_R_LEN                    2
#define MSG_W_LEN                    1
#define GPIO_LOW                     0
#define GPIO_HIGH                    1
#define MIN_CV                       4350
#define I2C_MSG_ONEBYTE              1
#define BUF_LEN                      26
#define RETRY_TIMES                  3
#define DEFAULT_END_OF_CURRENT       150000

struct sgm41511h_device_info *g_sgm41511h_dev;
static bool g_hiz_mode;
static bool g_shutdown_flag;
static bool g_otg_enable_flag;
static int g_safe_time_cnt;
static unsigned int g_safe_pre_time = SAFE_TIME_RESET;
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);

static void sgm41511h_i2c_err_monitor(void)
{
	static int i2c_err_cnt;

	i2c_err_cnt++;
	if (i2c_err_cnt >= I2C_ERR_MAX_COUNT) {
		i2c_err_cnt = I2C_ERR_COUNT_ZERO;
		power_dsm_report_dmd(POWER_DSM_BATTERY, ERROR_CHARGE_I2C_RW,
			"ERROR_CHARGE_I2C_RW");
	}
}

static int sgm41511h_write_block(struct sgm41511h_device_info *di,
	u8 *value, u8 reg, unsigned int num_bytes)
{
	struct i2c_msg msg[MSG_W_LEN];
	int ret;

	if (!di || !di->client || !value) {
		pr_err("di or value is null\n");
		return -EIO;
	}

	*value = reg;

	msg[0].addr = di->client->addr;
	msg[0].flags = 0;
	msg[0].buf = value;
	msg[0].len = num_bytes + MSG_W_LEN;

	ret = i2c_transfer(di->client->adapter, msg, MSG_W_LEN);
	if (ret != MSG_W_LEN) {
		sgm41511h_i2c_err_monitor();
		pr_err("write_block failed REG%x\n", reg);
		if (ret < CHARGE_IC_GOOD)
			return ret;
		else
			return -EIO;
	} else {
		return CHARGE_IC_GOOD;
	}
}

static int sgm41511h_read_block(struct sgm41511h_device_info *di,
	u8 *value, u8 reg, unsigned int num_bytes)
{
	struct i2c_msg msg[MSG_R_LEN];
	u8 buf;
	int ret;

	if (!di || !di->client || !value) {
		pr_err("di or value is null\n");
		return -EIO;
	}

	buf = reg;

	msg[0].addr = di->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &buf;
	msg[0].len = I2C_MSG_ONEBYTE;

	msg[1].addr = di->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = value;
	msg[1].len = num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, MSG_R_LEN);
	if (ret != MSG_R_LEN) {
		pr_err("read_block failed REG%x\n", reg);
		sgm41511h_i2c_err_monitor();
		if (ret < CHARGE_IC_GOOD)
			return ret;
		else
			return -EIO;
	} else {
		return CHARGE_IC_GOOD;
	}
}

static int sgm41511h_write_byte(u8 reg, u8 value)
{
	struct sgm41511h_device_info *di = g_sgm41511h_dev;
	u8 temp_buffer[MSG_R_LEN] = {0};

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return sgm41511h_write_block(di, temp_buffer, reg, I2C_MSG_ONEBYTE);
}

static int sgm41511h_read_byte(u8 reg, u8 *value)
{
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	return sgm41511h_read_block(di, value, reg, I2C_MSG_ONEBYTE);
}

static int sgm41511h_write_mask(u8 reg, u8 mask, u8 shift, u8 value)
{
	int ret;
	u8 val = 0;

	ret = sgm41511h_read_byte(reg, &val);
	if (ret < CHARGE_IC_GOOD)
		return ret;

	val &= ~mask;
	val |= ((value << shift) & mask);

	return sgm41511h_write_byte(reg, val);
}

static int sgm41511h_read_mask(u8 reg, u8 mask, u8 shift, u8 *value)
{
	int ret;
	u8 val = 0;

	ret = sgm41511h_read_byte(reg, &val);
	if (ret < CHARGE_IC_GOOD)
		return ret;

	val &= mask;
	val >>= shift;
	*value = val;

	return CHARGE_IC_GOOD;
}

static int sgm41511h_device_check(unsigned int *dev_id)
{
	u8 reg = 0;
	u8 pn_value;
	int ret;

	ret = sgm41511h_read_byte(SGM41511H_REG_VPRS, &reg);
	if (ret < CHARGE_IC_GOOD)
		return CHARGE_IC_BAD;

	pr_info("device_check REG%x=0x%x\n", SGM41511H_REG_VPRS, reg);

	pn_value = (reg & SGM41511H_REG_VPRS_PN_MASK) >>
		SGM41511H_REG_VPRS_PN_SHIFT;

	if (dev_id)
		*dev_id = pn_value;

	if (pn_value == SGM41511H_VENDOR_ID) {
		pr_info("sgm41511h is good\n");
		return CHARGE_IC_GOOD;
	} else if (pn_value == SGM41512_VENDOR_ID) {
		pr_info("sgm41512 is good\n");
		return CHARGE_IC_GOOD;
	} else if (pn_value == SY6974B_VENDOR_ID) {
		pr_info("sy6974b is good\n");
		return CHARGE_IC_GOOD;
	} else if (pn_value == BQ25601_VENDOR_ID) {
		pr_info("bq25601 is good\n");
		return CHARGE_IC_GOOD;
	}

	pr_err("sgm41511h is bad\n");
	return CHARGE_IC_BAD;
}

static int sgm41511h_get_charging_current(struct charger_device *chg_dev,
	u32 *value)
{
	int ret;
	u8 curr = 0;

	if (!value)
		return -EINVAL;

	ret = sgm41511h_read_mask(SGM41511H_REG_CCC,
		SGM41511H_REG_CCC_ICHG_MASK,
		SGM41511H_REG_CCC_ICHG_SHIFT, &curr);
	if (ret < CHARGE_IC_GOOD) {
		*value = 0;
		pr_err("get charge current failed, ret = %d\n", ret);
		return CHARGE_IC_BAD;
	}

	*value = curr;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_set_charge_current(struct charger_device *chg_dev,
	u32 value)
{
	int val;

	value /= UNIT_CONVERSION;
	val = (value - ICHG_BASE) / ICHG_LSB;

	pr_info("set_charge_current REG%x=0x%x\n", SGM41511H_REG_CCC, val);

	return sgm41511h_write_mask(SGM41511H_REG_CCC,
		SGM41511H_REG_CCC_ICHG_MASK,
		SGM41511H_REG_CCC_ICHG_SHIFT, val);
}

static int sgm41511h_get_input_current(struct charger_device *chg_dev,
	u32 *value)
{
	int ret;
	u8 curr = 0;

	if (!value)
		return -EINVAL;

	ret = sgm41511h_read_mask(SGM41511H_REG_ISC,
		SGM41511H_REG_ISC_IINLIM_MASK,
		SGM41511H_REG_ISC_IINLIM_SHIFT, &curr);
	if (ret < CHARGE_IC_GOOD) {
		*value = 0;
		pr_err("get input current failed, ret = %d\n", ret);
		return CHARGE_IC_BAD;
	}

	*value = curr;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_get_ibus(struct charger_device *chg_dev,
	u32 *value)
{
	int ret;
	u32 curr = 0;

	if (!chg_dev || !value)
		return -EINVAL;
	ret = sgm41511h_get_input_current(chg_dev, &curr);
	if (ret != CHARGE_IC_GOOD)
		return -EINVAL;

	curr = curr * IINLIM_LSB + IINLIM_BASE;
	*value = curr * UNIT_CONVERSION;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_set_input_current(struct charger_device *chg_dev,
	u32 value)
{
	int val;

	value /= UNIT_CONVERSION;
	if (value < IINLIM_BASE)
		value = IINLIM_BASE;

	if (value > AC_IIN_MAX_CURRENT)
		value = AC_IIN_MAX_CURRENT;

	val = (value - IINLIM_BASE) / IINLIM_LSB;
	pr_info("set_input_current REG%x=0x%x\n", SGM41511H_REG_ISC, val);

	return sgm41511h_write_mask(SGM41511H_REG_ISC,
		SGM41511H_REG_ISC_IINLIM_MASK,
		SGM41511H_REG_ISC_IINLIM_SHIFT, val);
}

static int sgm41511h_reused_set_cv(u32 value)
{
	int val;

	if (value < VREG_BASE)
		value = VREG_BASE;

	val = (value - VREG_BASE) / VREG_LSB;

	pr_info("reused_set_cv REG%x=0x%x\n", SGM41511H_REG_CVC, val);
	return sgm41511h_write_mask(SGM41511H_REG_CVC,
		SGM41511H_REG_CVC_VREG_MASK,
		SGM41511H_REG_CVC_VREG_SHIFT, val);
}

static int sgm41511h_get_terminal_voltage(struct charger_device *chg_dev,
	u32 *value)
{
	int ret;
	u8 volt = 0;

	if (!value)
		return -EINVAL;

	ret = sgm41511h_read_mask(SGM41511H_REG_CVC,
		SGM41511H_REG_CVC_VREG_MASK,
		SGM41511H_REG_CVC_VREG_SHIFT, &volt);
	if (ret < CHARGE_IC_GOOD) {
		*value = 0;
		pr_err("get terminal voltage failed, ret = %d\n", ret);
		return CHARGE_IC_BAD;
	}

	*value = volt;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_set_terminal_voltage(struct charger_device *chg_dev,
	u32 value)
{
	value /= UNIT_CONVERSION;

	return sgm41511h_reused_set_cv(value);
}

static int sgm41511h_set_iterm(struct charger_device *chg_dev, u32 value)
{
	int val;

	value /= UNIT_CONVERSION;
	val = (value - ITERM_BASE) / ITERM_LSB;

	pr_info("set iterm REG%x=0x%x\n", SGM41511H_REG_PCTCC, val);
	return sgm41511h_write_mask(SGM41511H_REG_PCTCC,
		SGM41511H_REG_PCTCC_ITERM_MASK,
		SGM41511H_REG_PCTCC_ITERM_SHIFT, val);
}

static int sgm41511h_set_dpm_voltage(struct charger_device *chg_dev, u32 value)
{
	int val;

	value /= UNIT_CONVERSION;

	if (value < VINDPM_BASE)
		value = VINDPM_BASE;

	val = (value - VINDPM_BASE) / VINDPM_LSB;

	pr_info("set_dpm_voltage REG%x=0x%x\n", SGM41511H_REG_BVTRC, val);

	return sgm41511h_write_mask(SGM41511H_REG_BVTRC,
		SGM41511H_REG_BVTRC_VINDPM_MASK,
		SGM41511H_REG_BVTRC_VINDPM_SHIFT, val);
}

static int sgm41511h_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	if (!curr)
		return -EINVAL;

	*curr = MIN_CURRENT;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_is_charging_done(struct charger_device *chg_dev,
	bool *done)
{
	int ret;
	u8 reg = 0;

	if (!done)
		return -EINVAL;

	*done = false;
	ret = sgm41511h_read_byte(SGM41511H_REG_SS, &reg);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("get_charge_state failed\n");
		return CHARGE_IC_BAD;
	}

	if ((reg & SGM41511H_REG_SS_CHRG_STAT_MASK) ==
		SGM41511H_REG_SS_CHRG_STAT_MASK) {
		pr_info("is charging done\n");
		*done = true;
	}

	return CHARGE_IC_GOOD;
}

static int sgm41511h_set_charge_enable(struct charger_device *chg_dev,
	bool enable)
{
	int ret;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!di)
		return -EINVAL;

	ret = sgm41511h_write_mask(SGM41511H_REG_POC,
		SGM41511H_REG_POC_CHG_CONFIG_MASK,
		SGM41511H_REG_POC_CHG_CONFIG_SHIFT, enable);
	if (ret == CHARGE_IC_GOOD)
		di->charge_enabled = enable;

	pr_info("set charge enable = %d, ret = %d\n", enable, ret);
	return ret;
}

static int sgm41511h_plug_in(struct charger_device *chg_dev)
{
	if (!chg_dev)
		return -EINVAL;

	pr_info("plug in set enable true\n");
	return sgm41511h_set_charge_enable(chg_dev, true);
}

static int sgm41511h_plug_out(struct charger_device *chg_dev)
{
	if (!chg_dev)
		return -EINVAL;

	pr_info("plug out set enable false\n");
	return sgm41511h_set_charge_enable(chg_dev, false);
}

static int sgm41511h_set_boost_voltage(u32 voltage)
{
	int val;

	if (voltage <= BOOSTV_STEP1)
		val = REG06_BOOSTV_4P85V;
	else if (voltage <= BOOSTV_STEP2)
		val = REG06_BOOSTV_5V;
	else if (voltage <= BOOSTV_STEP3)
		val = REG06_BOOSTV_5P15V;
	else
		val = REG06_BOOSTV_5P3V;

	pr_info("set_boost_voltage REG%x=0x%x\n", SGM41511H_REG_BVTRC, val);

	return sgm41511h_write_mask(SGM41511H_REG_BVTRC,
		SGM41511H_REG_BVTRC_BOOSTV_MASK,
		SGM41511H_REG_BVTRC_BOOSTV_SHIFT, val);
}

static int sgm41511h_set_otg_enable(struct charger_device *chg_dev, bool enable)
{
	struct sgm41511h_device_info *di = g_sgm41511h_dev;
	int ret;

	if (!di) {
		pr_err("di is null\n");
		return -EINVAL;
	}

	pr_info("sgm41511h_set_otg_enable set otg %d\n", enable);
	ret = sgm41511h_write_mask(SGM41511H_REG_POC,
		SGM41511H_REG_POC_OTG_CONFIG_MASK,
		SGM41511H_REG_POC_OTG_CONFIG_SHIFT, enable);
	if (ret == 0)
		g_otg_enable_flag = enable;

	return ret;
}

static int sgm41511h_disable_otg_usb_plugout(struct charger_device *chg_dev)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	if (!g_otg_enable_flag)
		ret = sgm41511h_set_otg_enable(chg_dev, false);

	return ret;
}

static void sgm41511h_pg_state_dsm(void)
{
	u8 reg = 0;
	int ret;
	static int check_count;

	ret = sgm41511h_read_byte(SGM41511H_REG_SS, &reg);
	if (ret) {
		pr_err("read bytes error\n");
		check_count = 0;
		return;
	}
	if ((!(reg & SGM41511H_REG_SS_PG_STAT_MASK)) &&
		(g_sgm41511h_dev->charge_enabled == true))
		check_count++;
	else
		check_count = 0;

	/* charger state not pg */
	if (check_count >= CH_ENABLE_MAX_COUNT) {
		power_dsm_report_dmd(POWER_DSM_BATTERY,
			ERROR_WEAKSOURCE_STOP_CHARGE,
			"ERROR_WEAKSOURCE_STOP_CHARGE");
		check_count = 0;
	}
}

static void sgm41511h_otg_ocp_dsm(void)
{
	u8 reg = 0;

	if (sgm41511h_read_byte(SGM41511H_REG_F, &reg))
		pr_err("read otg ocp error\n");
	if (reg & SGM41511H_REG_F_FAULT_BOOST_MASK) /* otg ocp happened */
		power_dsm_report_dmd(POWER_DSM_BATTERY,
			ERROR_BOOST_OCP, "ERROR_BOOST_OCP");
}

static void sgm41511h_set_safe_time_enable(bool enable)
{
	int ret;
	int retry;

	for (retry = 0; retry < RETRY_TIMES; retry++) {
		ret = sgm41511h_write_mask(SGM41511H_REG_CTTC,
			SGM41511H_REG_CTTC_EN_TIMER_MASK,
			SGM41511H_REG_CTTC_EN_TIMER_SHIFT, enable);
		if (ret == CHARGE_IC_GOOD)
			break;
	}
	if (ret < CHARGE_IC_GOOD)
		pr_err("set safe time enable fail\n");

	pr_info("set safe time en = %d, retry = %d\n", enable, retry);
}

static void sgm41511h_read_en_timer(void)
{
	int ret;
	u8 regval = 0;

	ret = sgm41511h_read_byte(SGM41511H_REG_CTTC, &regval);
	if (ret < CHARGE_IC_GOOD)
		pr_err("read REG_CTTC_EN_TIMER fail\n");

	pr_info("re-enable safety timer, REG%x=0x%x\n",
		SGM41511H_REG_CTTC, regval);
}

static void sgm41511h_reset_for_safe_time(void)
{
	unsigned int chg_time;

	chg_time = get_charging_time();
	if (chg_time == 0) {
		g_safe_time_cnt = 0;
		g_safe_pre_time = SAFE_TIME_RESET;
	}

	if ((g_safe_time_cnt < ENABLE_TIMES) &&
		((chg_time / g_safe_pre_time) == 1)) {
		g_safe_time_cnt++;
		g_safe_pre_time += SAFE_TIME_RESET;
		sgm41511h_set_safe_time_enable(false);
		mdelay(5); /* delay 5ms for set reg */
		sgm41511h_read_en_timer();
		sgm41511h_set_safe_time_enable(true);
		sgm41511h_read_en_timer();
		pr_info("re-enable safety timer,cnt=%d, chg_time=%d, safe_time=%d\n",
			g_safe_time_cnt, chg_time, g_safe_pre_time);
	}
}

static int sgm41511h_reset_watchdog_timer(struct charger_device *chg_dev)
{
	sgm41511h_reset_for_safe_time();

	if (g_otg_enable_flag == true)
		sgm41511h_otg_ocp_dsm();
	else
		sgm41511h_pg_state_dsm();

	return sgm41511h_write_mask(SGM41511H_REG_POC,
		SGM41511H_REG_POC_WDT_RESET_MASK,
		SGM41511H_REG_POC_WDT_RESET_SHIFT,
		REG01_WDT_RESET);
}

static int sgm41511h_get_register_head(char *reg_head, int size, void *dev_data)
{
	char buff[BUF_LEN] = {0};
	int i;
	int len = 0;
	struct sgm41511h_device_info *di = dev_data;

	if (!reg_head || !di)
		return -EINVAL;

	memset(reg_head, 0, size);
	for (i = 0; i < SGM41511H_REG_NUM; i++) {
		snprintf(buff, sizeof(buff), "Reg[0x%02X] ", i);
		len += strlen(buff);
		if (len < size)
			strncat(reg_head, buff, strlen(buff));
	}
	return 0;
}

static int sgm41511h_hw_dump_register(char *reg_value, int size, void *dev_data)
{
	int i;
	int ret;
	u8 regval;
	char buff[BUF_LEN];
	int len = 0;
	struct sgm41511h_device_info *di = dev_data;

	if (!reg_value || !di)
		return -EINVAL;

	memset(reg_value, 0, size);
	for (i = 0; i < SGM41511H_REG_NUM; i++) {
		ret = sgm41511h_read_byte(i, &regval);
		if (ret < 0) {
			pr_err("dump_register read Reg[%02X] fail\n", i);
			regval = 0;
		}
		snprintf(buff, sizeof(buff), "0x%-8.2x", regval);
		len += strlen(buff);
		if (len < size)
			strncat(reg_value, buff, strlen(buff));
	}

	return 0;
}

static int sgm41511h_set_watchdog_timer(struct charger_device *chg_dev,
	u32 value)
{
	int ret;
	int val;

	if ((value == WDT_BASE) || (g_hiz_mode == true))
		val = REG05_WDT_DISABLE;
	else if (value <= WDT_STEP1)
		val = REG05_WDT_40S;
	else if (value <= WDT_STEP2)
		val = REG05_WDT_80S;
	else
		val = REG05_WDT_160S;

	ret = sgm41511h_write_mask(SGM41511H_REG_CTTC,
		SGM41511H_REG_CTTC_WDT_MASK,
		SGM41511H_REG_CTTC_WDT_SHIFT, val);
	if (ret < CHARGE_IC_GOOD)
		pr_err("set_watchdog_timer write fail\n");

	pr_info("set_watchdog_timer REG%x=0x%x\n", SGM41511H_REG_CTTC, val);

	if (value > WDT_BASE)
		ret = sgm41511h_reset_watchdog_timer(chg_dev);

	return ret;
}

static int sgm41511h_set_charger_hiz(struct charger_device *chg_dev,
	bool enable)
{
	int ret;
	int ret_wdt;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!chg_dev || !di)
		return -EINVAL;

	if (g_shutdown_flag)
		enable = false;

	if (enable == true) {
		ret = sgm41511h_write_mask(SGM41511H_REG_ISC,
			SGM41511H_REG_ISC_EN_HIZ_MASK,
			SGM41511H_REG_ISC_EN_HIZ_SHIFT, true);
		g_hiz_mode = true;

		ret_wdt = sgm41511h_set_watchdog_timer(chg_dev,
			WATCHDOG_TIMER_DISABLE);
		if (ret_wdt)
			pr_err("set_charger_hiz true disable watchdog fail\n");
	} else {
		ret = sgm41511h_write_mask(SGM41511H_REG_ISC,
			SGM41511H_REG_ISC_EN_HIZ_MASK,
			SGM41511H_REG_ISC_EN_HIZ_SHIFT, false);
		g_hiz_mode = false;
		ret_wdt = sgm41511h_set_watchdog_timer(chg_dev, WDT_STEP2);
		if (ret_wdt)
			pr_err("set_charger_hiz false enable watchdog fail\n");
	}

	return ret;
}

static int sgm41511h_enable_eoc(struct charger_device *chg_dev,
	bool enable)
{
	int ret;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!chg_dev || !di)
		return -EINVAL;

	pr_info("sgm41511h_enable_eoc set eoc %d\n", enable);
	ret = sgm41511h_write_mask(SGM41511H_REG_CTTC,
		SGM41511H_REG_CTTC_EN_TERM_MASK,
		SGM41511H_REG_CTTC_EN_TERM_SHIFT, enable);
	if (ret)
		pr_err("sgm41511h_enable_eoc write fail\n");

	return ret;
}

static int sgm41512_inform_psy_changed(void)
{
	int ret;
	union power_supply_propval propval;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!di->psy) {
		di->psy = power_supply_get_by_name("charger");
		if (!di->psy)
			return -ENODEV;
	}

	if (di->chg_type != CHARGER_UNKNOWN)
		propval.intval = 1;
	else
		propval.intval = 0;

	pr_err("usb_online=%d chg-type=%d\n", propval.intval, di->chg_type);
	ret = power_supply_set_property(di->psy,
			POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		pr_err("inform power supply online failed:%d\n", ret);

	propval.intval = di->chg_type;
	ret = power_supply_set_property(di->psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_err("inform power supply charge type failed:%d\n", ret);

	return ret;
}

static int sgm41512_chg_type_det(void)
{
	int ret;
	u8 reg = 0;
	int vbus_stat = 0;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	ret = sgm41511h_read_byte(SGM41511H_REG_SS, &reg);
	if (ret < CHARGE_IC_GOOD)
		return CHARGE_IC_BAD;

	vbus_stat = (reg & SGM41511H_REG_SS_VBUS_STAT_MASK) >>
		SGM41511H_REG_SS_VBUS_STAT_SHIFT;
	switch (vbus_stat) {
	case REG08_VBUS_TYPE_NONE:
		di->chg_type = CHARGER_UNKNOWN;
		break;
	case REG08_VBUS_TYPE_USB:
		di->chg_type = STANDARD_HOST;
		break;
	case REG08_VBUS_TYPE_CDP:
		di->chg_type = CHARGING_HOST;
		break;
	case REG08_VBUS_TYPE_ADAPTER:
		di->chg_type = STANDARD_CHARGER;
		break;
	default:
		di->chg_type = NONSTANDARD_CHARGER;
		break;
	}
#ifdef CONFIG_HUAWEI_PD
	if (pd_dpm_get_pd_finish_flag())
		di->chg_type = STANDARD_CHARGER;
#endif
	pr_info("vbus_stat = %d chg_type=%d\n", vbus_stat, di->chg_type);
	sgm41512_inform_psy_changed();
	return 0;
}

static int sgm41512_enable_chg_type_det(struct charger_device *chg_dev,
	bool enable)
{
	if (enable) {
		Charger_Detect_Init();
		pr_info("Charger_Detect_Init\n");
	} else {
		Charger_Detect_Release();
		pr_info("Charger_Detect_Release\n");
	}
	return 0;
}

static int sgm41511h_is_charging_enable(struct charger_device *chg_dev,
	bool *en)
{
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!di || !en)
		return -EINVAL;

	*en = di->charge_enabled;
	return CHARGE_IC_GOOD;
}

static int sgm41511h_charger_do_event(struct charger_device *chg_dev, u32 event,
	u32 args)
{
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!di)
		return -EINVAL;

	switch (event) {
	case EVENT_EOC:
		pr_info("do eoc event\n");
		charger_dev_notify(di->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		pr_info("do recharge event\n");
		charger_dev_notify(di->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return CHARGE_IC_GOOD;
}

static int sgm41511h_5v_chip_init(struct sgm41511h_device_info *di)
{
	int ret;

	if (!di)
		return -EINVAL;
	/* I2C watchdog timer setting = 80s */
	/* fast charge timer setting = 12h */
	ret = sgm41511h_write_byte(SGM41511H_REG_CTTC, SGM41511H_REG05_INIT);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("REG SGM41511H_REG_CTTC set fail,ret = %d\n", ret);
		return ret;
	}
	/* iprechg = 256ma,iterm current = 128ma */
	ret = sgm41511h_write_byte(SGM41511H_REG_PCTCC, SGM41511H_REG03_INIT);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("REG SGM41511H_REG_PCTCC set fail,ret = %d\n", ret);
		return ret;
	}

	ret = sgm41511h_set_iterm(di->chg_dev, di->eoc);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("sgm41511h_set_iterm set fail\n");
		return ret;
	}

	ret = sgm41511h_write_mask(SGM41511H_REG_MOC,
		SGM41511H_REG_MOC_VDPM_BAT_TRACK_MASK,
		SGM41511H_REG_MOC_VDPM_BAT_TRACK_SHIFT,
		REG07_VDPM_BAT_TRACK_DISABLE);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("REG SGM41511H_REG_MOC set fail,ret = %d\n", ret);
		return ret;
	}

	ret = sgm41511h_write_mask(SGM41511H_REG_BVTRC,
		SGM41511H_REG_BVTRC_OVP_MASK,
		SGM41511H_REG_BVTRC_OVP_SHIFT,
		REG06_OVP_10P5V);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("REG SGM41511H_REG_BVTRC set fail,ret = %d\n", ret);
		return ret;
	}

	ret = sgm41511h_set_dpm_voltage(di->chg_dev, VINDPM_INIT);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("set_dpm_voltage fail,ret = %d\n", ret);
		return ret;
	}

	ret = sgm41511h_set_input_current(di->chg_dev, IINLIM_INIT);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("set_input_current fail,ret = %d\n", ret);
		return ret;
	}

	ret = sgm41511h_set_charge_current(di->chg_dev, ICHG_INIT);
	if (ret < CHARGE_IC_GOOD) {
		pr_info("set_charge_current fail,ret = %d\n", ret);
		return ret;
	}

	gpio_set_value(di->gpio_cd, GPIO_LOW);

	return ret;
}

static struct charger_ops sgm41511h_ops = {
	.plug_in = sgm41511h_plug_in,
	.plug_out = sgm41511h_plug_out,
	.enable = sgm41511h_set_charge_enable,
	.is_enabled = sgm41511h_is_charging_enable,
	.dump_registers = NULL,
	.get_charging_current = sgm41511h_get_charging_current,
	.set_charging_current = sgm41511h_set_charge_current,
	.get_input_current = sgm41511h_get_input_current,
	.set_input_current = sgm41511h_set_input_current,
	.get_constant_voltage = sgm41511h_get_terminal_voltage,
	.set_constant_voltage = sgm41511h_set_terminal_voltage,
	.set_eoc_current = sgm41511h_set_iterm,
	.kick_wdt = sgm41511h_reset_watchdog_timer,
	.set_mivr = sgm41511h_set_dpm_voltage,
	.is_charging_done = sgm41511h_is_charging_done,
	.get_min_charging_current = sgm41511h_get_min_ichg,
	.enable_otg = sgm41511h_set_otg_enable,
	.event = sgm41511h_charger_do_event,
	.enable_hz = sgm41511h_set_charger_hiz,
	.get_ibus_adc = sgm41511h_get_ibus,
	.disable_otg_usb_plugout = sgm41511h_disable_otg_usb_plugout,
	.enable_eoc = sgm41511h_enable_eoc,
};

static struct power_log_ops sgm41511h_log_ops = {
	.dev_name = "sgm41511h",
	.dump_log_head = sgm41511h_get_register_head,
	.dump_log_content = sgm41511h_hw_dump_register,
};

static void sgm41511h_irq_work(struct work_struct *work)
{
	struct sgm41511h_device_info *di = NULL;
	u8 reg = 0;
	int ret;

	if (!work) {
		pr_err("work is null\n");
		return;
	}

	di = container_of(work, struct sgm41511h_device_info, irq_work);
	if (!di) {
		pr_err("di is null\n");
		return;
	}

	msleep(100); /* sleep 100ms */

	ret = sgm41511h_read_byte(SGM41511H_REG_F, &reg);
	if (ret < CHARGE_IC_GOOD)
		pr_err("irq_work read fail\n");

	pr_info("boost_ovp_reg REG%x=0x%x chg_det_enable=%d\n",
			SGM41511H_REG_F, reg, di->chg_det_enable);

	if (di->chg_det_enable)
		sgm41512_chg_type_det();

	if (reg & SGM41511H_REG_F_FAULT_BOOST_MASK)
		pr_info("boost ocp happened\n");

	if (reg & SGM41511H_REG_F_FAULT_WDT_MASK) {
		ret = sgm41511h_5v_chip_init(g_sgm41511h_dev);
		if (ret < CHARGE_IC_GOOD)
			pr_err("sgm41511h_irq_work wdg init fail\n");
	}

	if (di->irq_active == false) {
		di->irq_active = true;
		enable_irq(di->irq_int);
	}
}

static irqreturn_t sgm41511h_interrupt(int irq, void *_di)
{
	struct sgm41511h_device_info *di = _di;

	if (!di) {
		pr_err("di is null\n");
		return -EINVAL;
	}

	if (di->irq_active == true) {
		di->irq_active = false;
		disable_irq_nosync(di->irq_int);
		schedule_work(&di->irq_work);
	} else {
		pr_info("the irq is not enable, do nothing\n");
	}

	return IRQ_HANDLED;
}

static int sgm41511h_gpio_cd_init(struct device_node *np,
	struct sgm41511h_device_info *di)
{
	int ret;

	di->gpio_cd = of_get_named_gpio(np, "gpio_cd", 0);
	pr_info("gpio_cd=%d\n", di->gpio_cd);
	if (!gpio_is_valid(di->gpio_cd)) {
		pr_err("gpio is not valid\n");
		return -EINVAL;
	}
	ret = gpio_request(di->gpio_cd, "charger_cd");
	if (ret < CHARGE_IC_GOOD) {
		pr_err("gpio request fail\n");
		return ret;
	}
	/* set gpio to control CD pin to disable/enable sgm41511h IC */
	ret = gpio_direction_output(di->gpio_cd, GPIO_LOW);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("gpio set output fail\n");
		gpio_free(di->gpio_cd);
		return ret;
	}

	return CHARGE_IC_GOOD;
}

static int sgm41511h_gpio_int_init(struct device_node *np,
	struct sgm41511h_device_info *di)
{
	int ret;

	di->chg_det_enable = of_property_read_bool(np, "charge-detect-enable");
	di->gpio_int = of_get_named_gpio(np, "gpio_int", 0);
	pr_info("gpio_int=%d chg_det_enable=%d\n", di->gpio_int, di->chg_det_enable);

	if (!gpio_is_valid(di->gpio_int)) {
		pr_err("gpio is not valid\n");
		return -EINVAL;
	}
	ret = gpio_request(di->gpio_int, "charger_int");
	if (ret < CHARGE_IC_GOOD) {
		pr_err("gpio request fail\n");
		return ret;
	}
	ret = gpio_direction_input(di->gpio_int);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("gpio set input fail\n");
		gpio_free(di->gpio_int);
		return ret;
	}

	return CHARGE_IC_GOOD;
}

static int sgm41511h_irq_int_init(struct device_node *np,
	struct sgm41511h_device_info *di)
{
	int ret;

	di->irq_int = gpio_to_irq(di->gpio_int);
	if (di->irq_int < 0) {
		pr_err("gpio map to irq fail\n");
		return -EINVAL;
	}
	ret = request_irq(di->irq_int, sgm41511h_interrupt,
		IRQF_TRIGGER_FALLING, "charger_int_irq", di);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("gpio irq request fail\n");
		di->irq_int = -1;
		return ret;
	}
	enable_irq(di->irq_int);
	di->irq_active = true;

	if (di->chg_det_enable)
		sgm41511h_ops.enable_chg_type_det = sgm41512_enable_chg_type_det;

	return CHARGE_IC_GOOD;
}

static int sgm41511h_probe_init(struct device_node *np,
	struct sgm41511h_device_info *di)
{
	int ret;

	if (!np)
		return -EINVAL;

	ret = sgm41511h_gpio_cd_init(np, di);
	if (ret < CHARGE_IC_GOOD)
		goto init_gpio_cd_fail;

	ret = sgm41511h_gpio_int_init(np, di);
	if (ret < CHARGE_IC_GOOD)
		goto init_gpio_int_fail;

	ret = sgm41511h_irq_int_init(np, di);
	if (ret < CHARGE_IC_GOOD)
		goto init_irq_int_fail;

	di->chg_dev = charger_device_register("primary_chg",
		&di->client->dev, di, &sgm41511h_ops, &di->chg_props);
	if (IS_ERR_OR_NULL(di->chg_dev)) {
		ret = PTR_ERR(di->chg_dev);
		pr_err("sgm41511h charge ops register fail\n");
		goto reg_device_fail;
	}

	ret = sgm41511h_5v_chip_init(di);
	if (ret < CHARGE_IC_GOOD) {
		pr_err("sgm41511h chip init fail\n");
		goto init_5v_chip_fail;
	}

	return CHARGE_IC_GOOD;
reg_device_fail:
init_5v_chip_fail:
	free_irq(di->irq_int, di);
init_irq_int_fail:
	gpio_free(di->gpio_int);
init_gpio_int_fail:
	gpio_free(di->gpio_cd);
init_gpio_cd_fail:
	return ret;
}

static int sgm41511h_parser_dts(struct device_node *np,
	struct sgm41511h_device_info *di)
{
	if (!np || !di)
		return -EINVAL;

	if (of_property_read_u32(np, "ieoc", &di->eoc) < 0)
		di->eoc = DEFAULT_END_OF_CURRENT;

	return CHARGE_IC_GOOD;
}

static int sgm41511h_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	unsigned int dev_id = 0;
	struct sgm41511h_device_info *di = NULL;
	struct device_node *np = NULL;
	struct power_devices_info_data *pwr_dev_info = NULL;

	pr_info("sgm41511h probe begin\n");

	if (!client || !client->dev.of_node || !id)
		return -ENODEV;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	g_sgm41511h_dev = di;

	di->dev = &client->dev;
	np = di->dev->of_node;
	di->client = client;
	i2c_set_clientdata(client, di);

	INIT_WORK(&di->irq_work, sgm41511h_irq_work);

	/* check if sgm41511h exist */
	if (sgm41511h_device_check(&dev_id) == CHARGE_IC_BAD) {
		pr_err("sgm41511h not exists\n");
		ret = -EINVAL;
		goto probe_fail;
	}

	ret = sgm41511h_parser_dts(np, di);
	if (ret < CHARGE_IC_GOOD)
		goto probe_fail;

	ret = sgm41511h_probe_init(np, di);
	if (ret < CHARGE_IC_GOOD)
		goto probe_fail;

	ret = sgm41511h_set_boost_voltage(BOOSTV_STEP2);
	if (ret < CHARGE_IC_GOOD)
		pr_err("set sgm41511h boost voltage fail\n");

	sgm41511h_log_ops.dev_data = (void *)di;
	power_log_ops_register(&sgm41511h_log_ops);

	pwr_dev_info = power_devices_info_register();
	if (pwr_dev_info) {
		pwr_dev_info->dev_name = di->dev->driver->name;
		pwr_dev_info->dev_id = dev_id;
		pwr_dev_info->ver_id = 0;
	}
	pr_info("sgm41511h probe end\n");
	return CHARGE_IC_GOOD;

probe_fail:
	kfree(di);
	di = NULL;
	g_sgm41511h_dev = NULL;
	return ret;
}

static void sgm41511h_shutdown(struct i2c_client *client)
{
	int ret;
	u8 part;
	u8 reg = 0;
	struct sgm41511h_device_info *di = g_sgm41511h_dev;

	if (!di)
		return;

	ret = sgm41511h_read_byte(SGM41511H_REG_VPRS, &reg);
	if (ret < CHARGE_IC_GOOD)
		return;

	part = reg & SGM41511H_REG_VPRS_PART_MASK;
	if (part) {
		g_shutdown_flag = true;
		ret = sgm41511h_set_charger_hiz(di->chg_dev, false);
		if (ret < CHARGE_IC_GOOD)
			pr_err("set sgm41511h set hiz fail\n");
	}
}

static int sgm41511h_remove(struct i2c_client *client)
{
	struct sgm41511h_device_info *di = i2c_get_clientdata(client);

	pr_info("remove begin\n");

	if (!di)
		return -ENODEV;

	gpio_set_value(di->gpio_cd, GPIO_HIGH);

	if (di->gpio_cd)
		gpio_free(di->gpio_cd);

	if (di->irq_int)
		free_irq(di->irq_int, di);

	if (di->gpio_int)
		gpio_free(di->gpio_int);

	if (di->gpio_fcp)
		gpio_free(di->gpio_fcp);

	kfree(di);
	di = NULL;

	pr_info("remove end\n");
	return CHARGE_IC_GOOD;
}

static const struct of_device_id sgm41511h_of_match[] = {
	{
		.compatible = "huawei,sgm41511h_charger",
		.data = NULL,
	},
	{},
};

MODULE_DEVICE_TABLE(i2c, sgm41511h_i2c_id);
static const struct i2c_device_id sgm41511h_i2c_id[] = {
	{ "sgm41511h_charger", 0 }, {}
};

static struct i2c_driver sgm41511h_driver = {
	.probe = sgm41511h_probe,
	.remove = sgm41511h_remove,
	.shutdown = sgm41511h_shutdown,
	.id_table = sgm41511h_i2c_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sgm41511h_charger",
		.of_match_table = of_match_ptr(sgm41511h_of_match),
	},
};

static int __init sgm41511h_init(void)
{
	return i2c_add_driver(&sgm41511h_driver);
}

static void __exit sgm41511h_exit(void)
{
	i2c_del_driver(&sgm41511h_driver);
}

module_init(sgm41511h_init);
module_exit(sgm41511h_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("sgm41511h charger module driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
