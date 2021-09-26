/*
 * rt9471_charger.c
 *
 * rt9471 driver
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
#include "rt9471_charger.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/power/huawei_charger.h>
#include <uapi/linux/sched/types.h>
#include "mtk_charger_intf.h"
#include <chipset_common/hwpower/common_module/power_log.h>
#include <chipset_common/hwpower/common_module/power_devices_info.h>

#define BUF_LEN 26
#define MIN_CURRENT 1
#define UNIT_CONVERSION 1000

static struct i2c_client *g_rt9471_i2c;
static int g_hiz_iin_limit_flag;
static bool g_otg_enable_flag;

enum rt9471_stat_idx {
	RT9471_STAT0 = 0,
	RT9471_STAT1,
	RT9471_STAT2,
	RT9471_STAT3,
	RT9471_MAX
};

enum rt9471_irq_idx {
	RT9471_IRQIDX_IRQ0 = 0,
	RT9471_IRQIDX_IRQ1,
	RT9471_IRQIDX_IRQ2,
	RT9471_IRQIDX_IRQ3,
	RT9471_IRQIDX_MAX
};

enum rt9471_ic_stat {
	RT9471_ICSTAT_SLEEP = 0,
	RT9471_ICSTAT_VBUSRDY,
	RT9471_ICSTAT_TRICKLECHG,
	RT9471_ICSTAT_PRECHG,
	RT9471_ICSTAT_FASTCHG,
	RT9471_ICSTAT_IEOC,
	RT9471_ICSTAT_BGCHG,
	RT9471_ICSTAT_CHGDONE,
	RT9471_ICSTAT_CHGFAULT,
	RT9471_ICSTAT_OTG = 15,
	RT9471_ICSTAT_MAX
};

static const char *g_rt9471_ic_stat[RT9471_ICSTAT_MAX] = {
	"hz/sleep", "ready", "trickle-charge", "pre-charge",
	"fast-charge", "ieoc-charge", "background-charge",
	"done", "fault", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "OTG",
};

enum rt9471_mivr_track {
	RT9471_MIVRTRACK_REG = 0,
	RT9471_MIVRTRACK_VBAT_PLUS_200MV,
	RT9471_MIVRTRACK_VBAT_PLUS_250MV,
	RT9471_MIVRTRACK_VBAT_PLUS_300MV,
	RT9471_MIVRTRACK_MAX
};

enum rt9471_port_stat {
	RT9471_PORTSTAT_NOINFO = 0,
	RT9471_PORTSTAT_APPLE_10W = 8,
	RT9471_PORTSTAT_SAMSUNG_10W,
	RT9471_PORTSTAT_APPLE_5W,
	RT9471_PORTSTAT_APPLE_12W,
	RT9471_PORTSTAT_NSDP,
	RT9471_PORTSTAT_SDP,
	RT9471_PORTSTAT_CDP,
	RT9471_PORTSTAT_DCP,
	RT9471_PORTSTAT_MAX
};

struct rt9471_desc {
	const char *rm_name;
	u8 rm_slave_addr;
	u32 acov;
	u32 cust_cv;
	u32 hiz_iin_limit;
	u32 ichg;
	u32 aicr;
	u32 mivr;
	u32 cv;
	u32 ieoc;
	u32 safe_tmr;
	u32 wdt;
	u32 mivr_track;
	bool en_safe_tmr;
	bool en_te;
	bool en_jeita;
	bool ceb_invert;
	bool dis_i2c_tout;
	bool en_qon_rst;
	bool auto_aicr;
	const char *chg_name;
};

/* These default values will be applied if there's no property in dts */
static struct rt9471_desc g_rt9471_default_desc = {
	.rm_name = "rt9471",
	.rm_slave_addr = RT9471_SLAVE_ADDR,
	.acov = RT9471_ACOV_6P5,
	.cust_cv = 0,
	.hiz_iin_limit = 0,
	.ichg = RT9471_ICHG,
	.aicr = RT9471_AICR,
	.mivr = RT9471_MIVR,
	.cv = RT9471_MIVR,
	.ieoc = RT9471_IEOC,
	.safe_tmr = RT9471_SAFETMR,
	.wdt = WDT_MIN,
	.mivr_track = RT9471_MIVRTRACK_REG,
	.en_safe_tmr = true,
	.en_te = true,
	.en_jeita = true,
	.ceb_invert = false,
	.dis_i2c_tout = false,
	.en_qon_rst = true,
	.auto_aicr = true,
	.chg_name = "rt9471",
};

static const u8 g_rt9471_irq_maskall[RT9471_IRQIDX_MAX] = {
	0xFF, 0xFE, 0xF3, 0xE7,
};

static const u32 g_rt9471_wdt[] = {
	0, 40, 80, 160,
};

static const u32 g_rt9471_otgcc[] = {
	500, 1200,
};

static const u8 g_rt9471_val_en_hidden_mode[] = {
	0x69, 0x96,
};

static const u32 g_rt9471_acov_th[] = {
	5800, 6500, 10900, 14000,
};

struct rt9471_chip {
	struct i2c_client *client;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct mutex io_lock;
	struct mutex hidden_mode_lock;
	u32 hidden_mode_cnt;
	u8 dev_id;
	u8 dev_rev;
	u8 chip_rev;
	struct rt9471_desc *desc;
	u32 intr_gpio;
	u32 ceb_gpio;
	int irq;
	bool charge_enabled;
	u8 irq_mask[RT9471_IRQIDX_MAX];
	struct kthread_work irq_work;
	struct kthread_worker irq_worker;
	struct task_struct *irq_worker_task;
};

static const u8 g_rt9471_reg_addr[] = {
	RT9471_REG_OTGCFG,
	RT9471_REG_TOP,
	RT9471_REG_FUNCTION,
	RT9471_REG_IBUS,
	RT9471_REG_VBUS,
	RT9471_REG_PRECHG,
	RT9471_REG_REGU,
	RT9471_REG_VCHG,
	RT9471_REG_ICHG,
	RT9471_REG_CHGTIMER,
	RT9471_REG_EOC,
	RT9471_REG_INFO,
	RT9471_REG_JEITA,
	RT9471_REG_STATUS,
	RT9471_REG_STAT0,
	RT9471_REG_STAT1,
	RT9471_REG_STAT2,
	RT9471_REG_STAT3,
	RT9471_REG_MASK0,
	RT9471_REG_MASK1,
	RT9471_REG_MASK2,
	RT9471_REG_MASK3
};

static int rt9471_read_device(struct i2c_client *client, u32 addr, int len,
	u8 *dst)
{
	return i2c_smbus_read_i2c_block_data(client, addr, len, dst);
}

static int rt9471_write_device(struct i2c_client *client, u32 addr, int len,
	const u8 *src)
{
	return i2c_smbus_write_i2c_block_data(client, addr, len, src);
}

static void rt9471_i2c_err_monitor(void)
{
	static int i2c_err_cnt;

	i2c_err_cnt++;
	if (i2c_err_cnt >= I2C_ERR_MAX_COUNT) {
		i2c_err_cnt = I2C_ERR_COUNT_ZERO;
		power_dsm_report_dmd(POWER_DSM_BATTERY, ERROR_CHARGE_I2C_RW,
			"ERROR_CHARGE_I2C_RW");
	}
}

static int __rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd, u8 data)
{
	int ret;
	/* 1 means len of i2c cmd */
	ret = rt9471_write_device(chip->client, cmd, 1, &data);
	if (ret < 0)
		rt9471_i2c_err_monitor();
	return ret;
}

static int rt9471_i2c_write_byte(struct rt9471_chip *chip, u8 cmd, u8 data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int __rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd,
	u8 *data)
{
	int ret;
	u8 regval = 0;

	/* 1 means len of i2c write_cmd */
	ret = rt9471_read_device(chip->client, cmd, 1, &regval);
	if (ret < 0) {
		rt9471_i2c_err_monitor();
		pr_err("reg0x%02X fail %d\n", cmd, ret);
		return ret;
	}

	pr_debug("reg0x%02X = 0x%02x\n", cmd, regval);
	*data = regval & RT9471_REG_NONE_MASK;
	return 0;
}

static int rt9471_i2c_read_byte(struct rt9471_chip *chip, u8 cmd, u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int __rt9471_i2c_block_write(struct rt9471_chip *chip, u8 cmd,
	u32 len, const u8 *data)
{
	return rt9471_write_device(chip->client, cmd, len, data);
}

static int rt9471_i2c_block_write(struct rt9471_chip *chip, u8 cmd, u32 len,
	const u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_block_write(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt9471_i2c_block_read(struct rt9471_chip *chip, u8 cmd,
	u32 len, u8 *data)
{
	return rt9471_read_device(chip->client, cmd, len, data);
}

static int rt9471_i2c_block_read(struct rt9471_chip *chip, u8 cmd, u32 len,
	u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_block_read(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt9471_i2c_test_bit(struct rt9471_chip *chip, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret;
	u8 data = 0;

	ret = rt9471_i2c_read_byte(chip, cmd, &data);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data &= (1 << shift);
	*is_one = data ? true : false;

	return 0;
}

static int rt9471_i2c_update_bits(struct rt9471_chip *chip, u8 cmd, u8 data,
	u8 mask)
{
	int ret;
	u8 regval = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt9471_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto fail_i2c_err;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __rt9471_i2c_write_byte(chip, cmd, regval);
fail_i2c_err:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt9471_set_bit(struct rt9471_chip *chip, u8 reg, u8 mask)
{
	return rt9471_i2c_update_bits(chip, reg, mask, mask);
}

static inline int rt9471_clr_bit(struct rt9471_chip *chip, u8 reg, u8 mask)
{
	/* 0x00 means clear bit */
	return rt9471_i2c_update_bits(chip, reg, 0x00, mask);
}

static u8 rt9471_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min || step == 0)
		return 0;

	if (target >= max)
		return ((max - min) / step);

	return ((target - min) / step);
}

static u8 rt9471_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size, u32 target)
{
	u32 i;

	if (!tbl || target < tbl[0] || tbl_size == 0)
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return (tbl_size - 1);
}

static u32 rt9471_closest_value(u32 min, u32 max, u32 step, u8 reg_val)
{
	u32 ret_val;

	ret_val = min + (reg_val * step);
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static int rt9471_enable_hidden_mode(struct rt9471_chip *chip, bool en)
{
	int ret;

	mutex_lock(&chip->hidden_mode_lock);

	if (en) {
		if (chip->hidden_mode_cnt == 0) {
			/* enter hidden mode and init hidden regs */
			ret = rt9471_i2c_block_write(chip, RT9471_REG_PASSCODE1,
				ARRAY_SIZE(g_rt9471_val_en_hidden_mode),
				g_rt9471_val_en_hidden_mode);
			if (ret < 0)
				goto hidden_ops_err;
		}
		chip->hidden_mode_cnt++;
	} else {
		if (chip->hidden_mode_cnt == 1) {
			/* exit hidden mode by write 0xA0 to zero */
			ret = rt9471_i2c_write_byte(chip,
				RT9471_REG_PASSCODE1, 0x00);
			if (ret < 0)
				goto hidden_ops_err;
		}
		chip->hidden_mode_cnt--;
	}
	pr_debug("%s en = %d\n", __func__, en);
	goto hidden_unlock;

hidden_ops_err:
	pr_err("%s en = %d fail %d\n", __func__, en, ret);
hidden_unlock:
	mutex_unlock(&chip->hidden_mode_lock);
	return 0;
}

static int __rt9471_get_ic_stat(struct rt9471_chip *chip,
	enum rt9471_ic_stat *stat)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_STATUS, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & RT9471_ICSTAT_MASK) >> RT9471_ICSTAT_SHIFT;
	return 0;
}

static int __rt9471_get_mivr(struct rt9471_chip *chip, u32 *mivr)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VBUS, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_MIVR_MASK) >> RT9471_MIVR_SHIFT;
	*mivr = rt9471_closest_value(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
		RT9471_MIVR_STEP, regval);

	return 0;
}

static int __rt9471_get_ichg(struct rt9471_chip *chip, u32 *ichg)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_ICHG_MASK) >> RT9471_ICHG_SHIFT;
	*ichg = rt9471_closest_value(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
		RT9471_ICHG_STEP, regval);

	return 0;
}

static int __rt9471_get_aicr(struct rt9471_chip *chip, u32 *aicr)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_IBUS, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_AICR_MASK) >> RT9471_AICR_SHIFT;
	*aicr = rt9471_closest_value(RT9471_AICR_MIN, RT9471_AICR_MAX,
		RT9471_AICR_STEP, regval);
	if (*aicr > RT9471_AICR_MIN && *aicr < RT9471_AICR_MAX)
		*aicr -= RT9471_AICR_STEP;

	return ret;
}

static int __rt9471_get_cv(struct rt9471_chip *chip, u32 *cv)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_VCHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_CV_MASK) >> RT9471_CV_SHIFT;
	*cv = rt9471_closest_value(RT9471_CV_MIN, RT9471_CV_MAX, RT9471_CV_STEP,
		regval);

	return ret;
}

static int __rt9471_get_ieoc(struct rt9471_chip *chip, u32 *ieoc)
{
	int ret;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_EOC, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_IEOC_MASK) >> RT9471_IEOC_SHIFT;
	*ieoc = rt9471_closest_value(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
		RT9471_IEOC_STEP, regval);

	return ret;
}

static int __rt9471_is_chg_enabled(struct rt9471_chip *chip, bool *en)
{
	return rt9471_i2c_test_bit(chip, RT9471_REG_FUNCTION,
		RT9471_CHG_EN_SHIFT, en);
}

static int __rt9471_enable_safe_tmr(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_CHGTIMER, RT9471_SAFETMR_EN_MASK);
}

static int __rt9471_enable_te(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_EOC, RT9471_TE_MASK);
}

static int __rt9471_enable_jeita(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_JEITA, RT9471_JEITA_EN_MASK);
}

static int __rt9471_disable_i2c_tout(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_TOP, RT9471_DISI2CTO_MASK);
}

static int __rt9471_enable_qon_rst(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_TOP, RT9471_QONRST_MASK);
}

static int __rt9471_enable_autoaicr(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_IBUS, RT9471_AUTOAICR_MASK);
}

static int __rt9471_enable_hz(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_FUNCTION, RT9471_HZ_MASK);
}

static int __rt9471_enable_otg(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_FUNCTION, RT9471_OTG_EN_MASK);
}

static int __rt9471_enable_chg(struct rt9471_chip *chip, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(chip, RT9471_REG_FUNCTION, RT9471_CHG_EN_MASK);
}

static int __rt9471_set_wdt(struct rt9471_chip *chip, u32 sec)
{
	u8 regval;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= WDT_MIN && sec > 0)
		sec = WDT_MIN;
	regval = rt9471_closest_reg_via_tbl(g_rt9471_wdt,
		ARRAY_SIZE(g_rt9471_wdt), sec);

	pr_info("%s time = %u reg: 0x%02X\n", __func__, sec, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_TOP,
		regval << RT9471_WDT_SHIFT,
		RT9471_WDT_MASK);
}

static int __rt9471_set_ichg(struct rt9471_chip *chip, u32 ichg)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
		RT9471_ICHG_STEP, ichg);

	pr_info("%s ichg = %u reg: 0x%02X\n", __func__, ichg, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_ICHG,
		regval << RT9471_ICHG_SHIFT,
		RT9471_ICHG_MASK);
}

static int __rt9471_set_acov(struct rt9471_chip *chip, u32 vth)
{
	u8 regval;

	regval = rt9471_closest_reg_via_tbl(g_rt9471_acov_th,
		ARRAY_SIZE(g_rt9471_acov_th), vth);

	pr_info("%s vth = %u reg: 0x%02x\n", __func__, vth, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
		regval << RT9471_ACOV_SHIFT,
		RT9471_ACOV_MASK);
}

static int __rt9471_set_aicr(struct rt9471_chip *chip, u32 aicr)
{
	int ret;
	u8 regval;

	regval = rt9471_closest_reg(RT9471_AICR_MIN, RT9471_AICR_MAX,
		RT9471_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < RT9471_AICR_MAX)
		regval += 1;

	pr_info("%s aicr = %u reg: 0x%02X\n", __func__, aicr, regval);

	ret = rt9471_i2c_update_bits(chip, RT9471_REG_IBUS,
		regval << RT9471_AICR_SHIFT,
		RT9471_AICR_MASK);
	/* Store AICR */
	__rt9471_get_aicr(chip, &chip->desc->aicr);
	return ret;
}

static int __rt9471_set_mivr(struct rt9471_chip *chip, u32 mivr)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
		RT9471_MIVR_STEP, mivr);

	pr_info("%s mivr = %u reg: 0x%02x\n", __func__, mivr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
		regval << RT9471_MIVR_SHIFT,
		RT9471_MIVR_MASK);
}

static int rt9471_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret;
	struct rt9471_chip *chip = NULL;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_MAX;

	if (!done)
		return -EINVAL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	*done = false;
	ret = __rt9471_get_ic_stat(chip, &ic_stat);
	if (ret < 0) {
		pr_err("get ic status failed\n");
		return ret;
	}

	if (ic_stat == RT9471_ICSTAT_CHGDONE) {
		pr_info("is charging done\n");
		*done = true;
	}

	return 0;
}

static int rt9471_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	if (!curr)
		return -EINVAL;

	*curr = MIN_CURRENT;
	return 0;
}

static int __rt9471_set_cv(struct rt9471_chip *chip, u32 cv)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_CV_MIN, RT9471_CV_MAX,
		RT9471_CV_STEP, cv);

	pr_info("%s cv = %u reg: 0x%02X\n", __func__, cv, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VCHG,
		regval << RT9471_CV_SHIFT,
		RT9471_CV_MASK);
}

static int __rt9471_set_ieoc(struct rt9471_chip *chip, u32 ieoc)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
		RT9471_IEOC_STEP, ieoc);

	pr_info("%s ieoc = %u reg: 0x%02X\n", __func__, ieoc, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_EOC,
		regval << RT9471_IEOC_SHIFT,
		RT9471_IEOC_MASK);
}

static int __rt9471_set_safe_tmr(struct rt9471_chip *chip, u32 tmr)
{
	u8 regval;

	regval = rt9471_closest_reg(RT9471_SAFETMR_MIN, RT9471_SAFETMR_MAX,
		RT9471_SAFETMR_STEP, tmr);

	pr_info("%s time = %u reg: 0x%02X\n", __func__, tmr, regval);

	return rt9471_i2c_update_bits(chip, RT9471_REG_CHGTIMER,
		regval << RT9471_SAFETMR_SHIFT,
		RT9471_SAFETMR_MASK);
}

static int __rt9471_set_mivrtrack(struct rt9471_chip *chip, u32 mivr_track)
{
	if (mivr_track >= RT9471_MIVRTRACK_MAX)
		mivr_track = RT9471_MIVRTRACK_VBAT_PLUS_300MV;

	pr_info("%s mivrtrack = %u\n", __func__, mivr_track);

	return rt9471_i2c_update_bits(chip, RT9471_REG_VBUS,
		mivr_track << RT9471_MIVRTRACK_SHIFT,
		RT9471_MIVRTRACK_MASK);
}

static int __rt9471_kick_wdt(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return rt9471_set_bit(chip, RT9471_REG_TOP, RT9471_WDTCNTRST_MASK);
}

static int rt9471_detach_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging detach_irq */
	return 0;
}

static int rt9471_rechg_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging rechg_irq */
	return 0;
}

static int rt9471_chg_done_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging chg_done_irq */
	return 0;
}

static int rt9471_bg_chg_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging bg_chg_irq */
	return 0;
}

static int rt9471_ieoc_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging ieoc_irq */
	return 0;
}

static int rt9471_vbus_gd_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging vbus_gd_irq */
	return 0;
}

static int rt9471_chg_batov_irq_handler(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int rt9471_chg_sysov_irq_handler(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int rt9471_chg_tout_irq_handler(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int rt9471_chg_busuv_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging chg_busuv_irq */
	return 0;
}

static int rt9471_chg_threg_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging chg_threg_irq */
	return 0;
}

static int rt9471_chg_aicr_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging chg_aicr_irq */
	return 0;
}

static int rt9471_chg_mivr_irq_handler(struct rt9471_chip *chip)
{
	int ret;
	bool mivr = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT1, RT9471_ST_MIVR_SHIFT,
		&mivr);
	if (ret < 0) {
		pr_err("check stat fail %d\n", ret);
		return ret;
	}
	pr_info("%s mivr = %d\n", __func__, mivr);
	return 0;
}

static int rt9471_sys_short_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging sys_short_irq */
	return 0;
}

static int rt9471_sys_min_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging sys_min_irq */
	return 0;
}

static int rt9471_jeita_cold_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging jeita_cold_irq */
	return 0;
}

static int rt9471_jeita_cool_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging jeita_cool_irq */
	return 0;
}

static int rt9471_jeita_warm_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging jeita_warm_irq */
	return 0;
}

static int rt9471_jeita_hot_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging jeita_hot_irq */
	return 0;
}

static int rt9471_otg_fault_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging otg_fault_irq */
	return 0;
}

static int rt9471_otg_lbp_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging otg_lbp_irq */
	return 0;
}

static int rt9471_otg_cc_irq_handler(struct rt9471_chip *chip)
{
	/* 3rd ic code framework, reserved for debugging otg_cc_irq */
	return 0;
}

static int rt9471_wdt_irq_handler(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return __rt9471_kick_wdt(chip);
}

static int rt9471_vac_ov_irq_handler(struct rt9471_chip *chip)
{
	int ret;
	bool vacov = false;

	ret = rt9471_i2c_test_bit(chip, RT9471_REG_STAT3, RT9471_ST_VACOV_SHIFT,
		&vacov);
	if (ret < 0) {
		pr_err("check stat fail %d\n", ret);
		return ret;
	}
	pr_info("%s vacov = %d\n", __func__, vacov);
	if (vacov) {
		/* Rewrite AICR */
		ret = __rt9471_set_aicr(chip, chip->desc->aicr);
		if (ret < 0)
			pr_err("set aicr fail %d\n", ret);
	}
	return ret;
}

static int rt9471_otp_irq_handler(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return 0;
}

struct irq_mapping_tbl {
	const char *name;
	int (*hdlr)(struct rt9471_chip *chip);
	int num;
};

#define rt9471_irq_mapping(_name, _num) \
{ \
	.name = #_name, \
	.hdlr = rt9471_##_name##_irq_handler, \
	.num = (_num), \
}

static const struct irq_mapping_tbl rt9471_irq_mapping_tbl[] = {
	rt9471_irq_mapping(vbus_gd, RT9471_IRQ_NUM_VBUS_GD),
	rt9471_irq_mapping(detach, RT9471_IRQ_NUM_DETACH),
	rt9471_irq_mapping(rechg, RT9471_IRQ_NUM_RECHG),
	rt9471_irq_mapping(chg_done, RT9471_IRQ_NUM_CHG_DONE),
	rt9471_irq_mapping(bg_chg, RT9471_IRQ_NUM_BG_CHG),
	rt9471_irq_mapping(ieoc, RT9471_IRQ_NUM_IEOC),
	rt9471_irq_mapping(chg_batov, RT9471_IRQ_NUM_CHG_BATOV),
	rt9471_irq_mapping(chg_sysov, RT9471_IRQ_NUM_CHG_SYSOV),
	rt9471_irq_mapping(chg_tout, RT9471_IRQ_NUM_CHG_TOUT),
	rt9471_irq_mapping(chg_busuv, RT9471_IRQ_NUM_CHG_BUGUV),
	rt9471_irq_mapping(chg_threg, RT9471_IRQ_NUM_CHG_THREG),
	rt9471_irq_mapping(chg_aicr, RT9471_IRQ_NUM_CHG_AICR),
	rt9471_irq_mapping(chg_mivr, RT9471_IRQ_NUM_CHG_MIVR),
	rt9471_irq_mapping(sys_short, RT9471_IRQ_NUM_SYS_SHORT),
	rt9471_irq_mapping(sys_min, RT9471_IRQ_NUM_SYS_MIN),
	rt9471_irq_mapping(jeita_cold, RT9471_IRQ_NUM_JEITA_COLD),
	rt9471_irq_mapping(jeita_cool, RT9471_IRQ_NUM_JEITA_COOL),
	rt9471_irq_mapping(jeita_warm, RT9471_IRQ_NUM_JEITA_WARM),
	rt9471_irq_mapping(jeita_hot, RT9471_IRQ_NUM_JEITA_HOT),
	rt9471_irq_mapping(otg_fault, RT9471_IRQ_NUM_OTG_FAULT),
	rt9471_irq_mapping(otg_lbp, RT9471_IRQ_NUM_OTG_LBP),
	rt9471_irq_mapping(otg_cc, RT9471_IRQ_NUM_OTG_CC),
	rt9471_irq_mapping(wdt, RT9471_IRQ_NUM_WDT),
	rt9471_irq_mapping(vac_ov, RT9471_IRQ_NUM_VAC_OV),
	rt9471_irq_mapping(otp, RT9471_IRQ_NUM_OTP),
};

static void rt9471_irq_work_handler(struct kthread_work *work)
{
	int i;
	int irq_num;
	int irq_bit;
	u32 tbl_size = ARRAY_SIZE(rt9471_irq_mapping_tbl);
	u8 evt[RT9471_IRQIDX_MAX] = {0};
	u8 mask[RT9471_IRQIDX_MAX] = {0};
	struct rt9471_chip *chip = NULL;

	pr_info("%s\n", __func__);

	chip = container_of(work, struct rt9471_chip, irq_work);
	if (!chip)
		return;

	if (rt9471_i2c_block_read(chip, RT9471_REG_IRQ0,
		RT9471_IRQIDX_MAX, evt) < 0) {
		pr_err("%s read evt fail\n", __func__);
		goto irq_i2c_err;
	}

	if (rt9471_i2c_block_read(chip, RT9471_REG_MASK0,
		RT9471_IRQIDX_MAX, mask) < 0) {
		pr_err("%s read mask fail\n", __func__);
		goto irq_i2c_err;
	}

	for (i = 0; i < RT9471_IRQIDX_MAX; i++)
		evt[i] &= ~mask[i];
	for (i = 0; i < tbl_size; i++) {
		irq_num = rt9471_irq_mapping_tbl[i].num / BIT_LEN;
		if (irq_num >= RT9471_IRQIDX_MAX)
			continue;
		irq_bit = rt9471_irq_mapping_tbl[i].num % BIT_LEN;
		if (evt[irq_num] & (1 << irq_bit))
			rt9471_irq_mapping_tbl[i].hdlr(chip);
	}
irq_i2c_err:
	enable_irq(chip->irq);
}

static irqreturn_t rt9471_irq_handler(int irq, void *data)
{
	struct rt9471_chip *chip = data;

	if (!chip)
		return IRQ_HANDLED;

	disable_irq_nosync(chip->irq);
	kthread_queue_work(&chip->irq_worker, &chip->irq_work);

	return IRQ_HANDLED;
}

static int rt9471_get_gpio_irq(struct rt9471_chip *chip)
{
	int ret;
	int len;
	char *name = NULL;

	len = strlen(chip->desc->chg_name) + CHG_IRQGPIO_NAME_LEN;
	name = devm_kzalloc(chip->dev, len, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	memset(name, 0, len);
	ret = snprintf(name, len, "%s-irq-gpio", chip->desc->chg_name);
	if (ret < 0)
		goto free_name;

	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_IN, name);
	if (ret < 0) {
		pr_err("gpio request fail %d\n", ret);
		goto free_name;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		pr_err("%s gpio2irq fail %d\n", __func__, chip->irq);
		ret = chip->irq;
		goto free_name;
	}
	pr_info("%s irq = %d\n", __func__, chip->irq);

	/* no need free!! mem will be used by platfrom */
	return 0;
free_name:
	devm_kfree(chip->dev, name);
	name = NULL;
	return ret;
}

static int rt9471_register_irq(struct rt9471_chip *chip)
{
	int ret;
	int len;
	char *name = NULL;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	pr_info("%s\n", __func__);

	ret = rt9471_get_gpio_irq(chip);
	if (ret < 0)
		return ret;

	/* Request IRQ */
	len = strlen(chip->desc->chg_name) + CHG_IRQ_NAME_LEN;
	name = devm_kzalloc(chip->dev, len, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	memset(name, 0, len);
	ret = snprintf(name, len, "%s-irq", chip->desc->chg_name);
	if (ret < 0)
		goto free_name;

	kthread_init_work(&chip->irq_work, rt9471_irq_work_handler);
	kthread_init_worker(&chip->irq_worker);
	chip->irq_worker_task = kthread_run(kthread_worker_fn,
		&chip->irq_worker, chip->desc->chg_name);
	if (IS_ERR(chip->irq_worker_task)) {
		ret = PTR_ERR(chip->irq_worker_task);
		pr_err("kthread run fail %d\n", ret);
		goto free_name;
	}
	sched_setscheduler(chip->irq_worker_task, SCHED_FIFO, &param);

	ret = devm_request_irq(chip->dev, chip->irq, rt9471_irq_handler,
		IRQF_TRIGGER_FALLING, name, chip);
	if (ret < 0) {
		pr_err("request thread irq fail %d\n", ret);
		goto free_name;
	}
	device_init_wakeup(chip->dev, true);

	/* no need free!! mem will be used by platfrom */
	return 0;
free_name:
	devm_kfree(chip->dev, name);
	name = NULL;
	return ret;
}

static int rt9471_init_irq(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
		ARRAY_SIZE(chip->irq_mask),
		chip->irq_mask);
}

static int rt9471_get_irq_number(struct rt9471_chip *chip,
	const char *name)
{
	int i;
	u32 tbl_size = ARRAY_SIZE(rt9471_irq_mapping_tbl);

	if (!name) {
		pr_err("%s null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < tbl_size; i++) {
		if (!strcmp(name, rt9471_irq_mapping_tbl[i].name))
			return rt9471_irq_mapping_tbl[i].num;
	}

	return -EINVAL;
}

static const char *rt9471_get_irq_name(int irq_num)
{
	int i;
	u32 tbl_size = ARRAY_SIZE(rt9471_irq_mapping_tbl);

	for (i = 0; i < tbl_size; i++) {
		if (rt9471_irq_mapping_tbl[i].num == irq_num)
			return rt9471_irq_mapping_tbl[i].name;
	}
	return GET_IRQNAME_FAIL;
}

static inline void rt9471_irq_mask(struct rt9471_chip *chip, int irq_num)
{
	chip->irq_mask[irq_num / BIT_LEN] |= (1 << (irq_num % BIT_LEN));
}

static inline void rt9471_irq_unmask(struct rt9471_chip *chip, int irq_num)
{
	pr_info("%s irq %d, %s\n", __func__, irq_num,
		rt9471_get_irq_name(irq_num));
	chip->irq_mask[irq_num / RT9471_IRQ_GROUP_LEN] &=
		~(1 << (irq_num % BIT_LEN));
}

static int rt9471_gpio_init(struct rt9471_chip *chip, struct device_node *np,
	struct rt9471_desc *desc)
{
	int ret;
	int len;
	char *ceb_name = NULL;

	ret = of_property_read_string(np, "charger_name", &desc->chg_name);
	if (ret)
		pr_err("%s no charger name\n", __func__);
	pr_info("%s name %s\n", __func__, desc->chg_name);

	ret = of_get_named_gpio(np, "rt,intr_gpio", 0);
	if (ret < 0)
		return ret;
	chip->intr_gpio = ret;
	pr_info("%s intr_gpio %u\n", __func__, chip->intr_gpio);

	ret = of_get_named_gpio(np, "rt,ceb_gpio", 0);
	if (ret < 0)
		return ret;
	chip->ceb_gpio = ret;
	pr_info("%s ceb_gpio %u\n", __func__, chip->ceb_gpio);

	len = strlen(desc->chg_name) + CHG_IRQGPIO_NAME_LEN;
	ceb_name = devm_kzalloc(chip->dev, len, GFP_KERNEL);
	if (!ceb_name)
		return -ENOMEM;

	memset(ceb_name, 0, len);
	ret = snprintf(ceb_name, len, "%s-ceb-gpio", chip->desc->chg_name);
	if (ret < 0)
		goto free_ceb_name;

	ret = devm_gpio_request_one(chip->dev, chip->ceb_gpio, GPIOF_DIR_OUT,
		ceb_name);
	if (ret < 0) {
		pr_info("%s gpio request fail %d\n", __func__, ret);
		goto free_ceb_name;
	}

	/* no need free!! mem will be used by platfrom */
	return 0;
free_ceb_name:
	devm_kfree(chip->dev, ceb_name);
	ceb_name = NULL;
	return ret;
}

static void rt9471_chg_para_dt(struct device_node *np, struct rt9471_desc *desc)
{
	/* Charger parameter */
	if (of_property_read_u32(np, "acov", &desc->acov) < 0)
		pr_info("%s no ichg\n", __func__);

	if (of_property_read_u32(np, "custom_cv", &desc->cust_cv) < 0)
		pr_info("%s no custom_cv\n", __func__);

	if (of_property_read_u32(np, "hiz_iin_limit", &desc->hiz_iin_limit) < 0)
		pr_info("%s no hiz_iin_limit\n", __func__);

	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		pr_info("%s no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		pr_info("%s no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		pr_info("%s no mivr\n", __func__);

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		pr_info("%s no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		pr_info("%s no ieoc\n", __func__);

	if (of_property_read_u32(np, "safe-tmr", &desc->safe_tmr) < 0)
		pr_info("%s no safety timer\n", __func__);

	if (of_property_read_u32(np, "wdt", &desc->wdt) < 0)
		pr_info("%s no wdt\n", __func__);

	if (of_property_read_u32(np, "mivr-track", &desc->mivr_track) < 0)
		pr_info("%s no mivr track\n", __func__);
	if (desc->mivr_track >= RT9471_MIVRTRACK_MAX)
		desc->mivr_track = RT9471_MIVRTRACK_VBAT_PLUS_300MV;

	desc->en_te = of_property_read_bool(np, "en-te");
	desc->en_jeita = of_property_read_bool(np, "en-jeita");
	desc->ceb_invert = of_property_read_bool(np, "ceb-invert");
	desc->dis_i2c_tout = of_property_read_bool(np, "dis-i2c-tout");
	desc->en_qon_rst = of_property_read_bool(np, "en-qon-rst");
	desc->auto_aicr = of_property_read_bool(np, "auto-aicr");
}

static int rt9471_parse_dt(struct rt9471_chip *chip)
{
	int ret;
	int irq_num;
	int irq_cnt = 0;
	struct rt9471_desc *desc = NULL;
	struct device_node *np = chip->dev->of_node;
	const char *name = NULL;

	pr_info("%s\n", __func__);

	if (!np) {
		pr_info("%s no device node\n", __func__);
		return -EINVAL;
	}

	/* if device node np is a single rt9471 node without child node,
	 * of_get_child_by_name will return NULL, we init np with of_node.
	 * if device node np is a parent swchg node with two child nodes.
	 * of_get_child_by_name rt9471 will return child node rt9471.
	 */
	np = of_get_child_by_name(chip->dev->of_node, "rt9471");
	if (!np)
		np = chip->dev->of_node;

	chip->desc = &g_rt9471_default_desc;

	desc = devm_kzalloc(chip->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &g_rt9471_default_desc, sizeof(*desc));

	ret = rt9471_gpio_init(chip, np, desc);
	if (ret) {
		devm_kfree(chip->dev, desc);
		desc = NULL;
		return ret;
	}

	rt9471_chg_para_dt(np, desc);

	memcpy(chip->irq_mask, g_rt9471_irq_maskall, RT9471_IRQIDX_MAX);
	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
			irq_cnt, &name);
		if (ret < 0)
			break;
		irq_cnt++;
		irq_num = rt9471_get_irq_number(chip, name);
		if (irq_num >= 0)
			rt9471_irq_unmask(chip, irq_num);
	}

	chip->desc = desc;
	return 0;
}

static int rt9471_sw_workaround(struct rt9471_chip *chip)
{
	int ret;
	u8 regval = 0;

	pr_info("%s\n", __func__);

	ret = rt9471_enable_hidden_mode(chip, true);
	if (ret < 0) {
		pr_err("enter hidden mode fail %d\n", ret);
		return ret;
	}

	ret = rt9471_i2c_read_byte(chip, RT9471_REG_HIDDEN_0, &regval);
	if (ret < 0) {
		pr_err("read HIDDEN_0 fail %d\n", ret);
		goto hidden_mode_exit;
	}
	chip->chip_rev = (regval & RT9471_CHIP_REV_MASK) >>
		RT9471_CHIP_REV_SHIFT;
	pr_info("chip_rev = %d\n", chip->chip_rev);

	/* OTG load transient improvement */
	if (chip->chip_rev <= RT9471_CHIP_VER_3)
		ret = rt9471_i2c_update_bits(chip, RT9471_REG_OTG_HDEN2, 0x10,
			RT9471_REG_OTG_RES_COMP_MASK);

hidden_mode_exit:
	rt9471_enable_hidden_mode(chip, false);
	return ret;
}

static void rt9471_init_para(struct rt9471_chip *chip, struct rt9471_desc *desc)
{
	if (__rt9471_set_acov(chip, desc->acov) < 0)
		pr_err("%s set ac_ov fail\n", __func__);

	if (__rt9471_set_ichg(chip, desc->ichg) < 0)
		pr_err("%s set ichg fail\n", __func__);

	if (__rt9471_set_aicr(chip, desc->aicr) < 0)
		pr_err("%s set aicr fail\n", __func__);

	if (__rt9471_set_mivr(chip, desc->mivr) < 0)
		pr_err("%s set mivr fail\n", __func__);

	if (__rt9471_set_cv(chip, desc->cv) < 0)
		pr_err("%s set cv fail\n", __func__);

	if (__rt9471_set_ieoc(chip, desc->ieoc) < 0)
		pr_err("%s set ieoc fail\n", __func__);

	if (__rt9471_set_safe_tmr(chip, desc->safe_tmr) < 0)
		pr_err("%s set safe tmr fail\n", __func__);

	if (__rt9471_set_mivrtrack(chip, desc->mivr_track) < 0)
		pr_err("%s set mivrtrack fail\n", __func__);

	if (__rt9471_enable_safe_tmr(chip, desc->en_safe_tmr) < 0)
		pr_err("%s en safe tmr fail\n", __func__);

	if (__rt9471_enable_te(chip, desc->en_te) < 0)
		pr_err("%s en te fail\n", __func__);

	if (__rt9471_enable_jeita(chip, desc->en_jeita) < 0)
		pr_err("%s en jeita fail\n", __func__);

	if (__rt9471_disable_i2c_tout(chip, desc->dis_i2c_tout) < 0)
		pr_err("%s dis i2c tout fail\n", __func__);

	if (__rt9471_enable_qon_rst(chip, desc->en_qon_rst) < 0)
		pr_err("%s en qon rst fail\n", __func__);

	if (__rt9471_enable_autoaicr(chip, desc->auto_aicr) < 0)
		pr_err("%s en autoaicr fail\n", __func__);
}

static int rt9471_init_setting(struct rt9471_chip *chip)
{
	int ret;
	struct rt9471_desc *desc = chip->desc;
	u8 evt[RT9471_IRQIDX_MAX] = {0};

	if (!desc)
		return -EINVAL;

	/* Disable WDT during IRQ masked period */
	ret = __rt9471_set_wdt(chip, 0);
	if (ret < 0)
		pr_err("%s set wdt fail %d\n", __func__, ret);

	/* Mask all IRQs */
	ret = rt9471_i2c_block_write(chip, RT9471_REG_MASK0,
		ARRAY_SIZE(g_rt9471_irq_maskall), g_rt9471_irq_maskall);
	if (ret < 0) {
		pr_err("%s mask irq fail %d\n", __func__, ret);
		return ret;
	}
	/* Clear all IRQs */
	ret = rt9471_i2c_block_read(chip, RT9471_REG_IRQ0, RT9471_IRQIDX_MAX,
		evt);
	if (ret < 0) {
		pr_err("%s clear irq fail %d\n", __func__, ret);
		return ret;
	}

	rt9471_init_para(chip, desc);

	ret = rt9471_sw_workaround(chip);
	if (ret < 0)
		pr_err("set sw workaround fail %d\n", ret);

	return 0;
}

static int rt9471_reset_register(struct rt9471_chip *chip)
{
	pr_info("%s\n", __func__);
	return rt9471_set_bit(chip, RT9471_REG_INFO, RT9471_REGRST_MASK);
}

static bool rt9471_check_devinfo(struct rt9471_chip *chip)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, RT9471_REG_INFO);
	if (ret < 0) {
		pr_err("get devinfo fail %d\n", ret);
		return false;
	}
	chip->dev_id = (ret & RT9471_DEVID_MASK) >> RT9471_DEVID_SHIFT;
	if (chip->dev_id != RT9471_DEVID && chip->dev_id != RT9471D_DEVID) {
		pr_err("incorrect devid 0x%02X\n", chip->dev_id);
		return false;
	}

	chip->dev_rev = (ret & RT9471_DEVREV_MASK) >> RT9471_DEVREV_SHIFT;
	pr_info("%s id = 0x%02X, rev = 0x%02X\n", __func__,
		chip->dev_id, chip->dev_rev);
	return true;
}

static void rt9471_dump_addr(struct rt9471_chip *chip)
{
	int i;
	int ret;
	u8 regval = 0;
	u32 tbl_size = ARRAY_SIZE(g_rt9471_reg_addr);

	for (i = 0; i < tbl_size; i++) {
		ret = rt9471_i2c_read_byte(chip, g_rt9471_reg_addr[i],
			&regval);
		if (ret < 0)
			continue;
		pr_info("%s reg0x%02X = 0x%02X\n", __func__,
			g_rt9471_reg_addr[i], regval);
	}
}

static void __rt9471_dump_registers(struct rt9471_chip *chip)
{
	int ret;
	u32 value = 0;
	bool chg_en = false;
	enum rt9471_ic_stat ic_stat = RT9471_ICSTAT_VBUSRDY;
	u8 stats[RT9471_MAX] = {0};

	ret = __rt9471_kick_wdt(chip);
	if (ret)
		pr_err("%s kick wdt fail\n", __func__);
	ret = __rt9471_get_ichg(chip, &value);
	pr_info("ICHG: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = __rt9471_get_aicr(chip, &value);
	pr_info("AICR: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = __rt9471_get_mivr(chip, &value);
	pr_info("MIVR: %umV, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = __rt9471_get_ieoc(chip, &value);
	pr_info("IEOC: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = __rt9471_get_cv(chip, &value);
	pr_info("CV: %umV, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = __rt9471_is_chg_enabled(chip, &chg_en);
	pr_info("CHG_EN: %d, ret: %d\n", chg_en, ret);
	ret = __rt9471_get_ic_stat(chip, &ic_stat);
	if (ic_stat < RT9471_ICSTAT_MAX)
		pr_info("IC_STAT: %s, ret: %d\n", g_rt9471_ic_stat[ic_stat],
			ret);

	ret = rt9471_i2c_block_read(chip, RT9471_REG_STAT0,
		RT9471_MAX, stats);
	if (!ret)
		pr_info("STAT: S0:0x%02X, S1:0x%02X, S2:0x%02X, S3:0x%02X\n",
			stats[RT9471_STAT0], stats[RT9471_STAT1],
			stats[RT9471_STAT2], stats[RT9471_STAT3]);

	if (ic_stat != RT9471_ICSTAT_CHGFAULT)
		return;

	rt9471_dump_addr(chip);
}

static int rt9471_enable_chg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	ret = __rt9471_enable_chg(chip, en);
	if (!ret)
		chip->charge_enabled = en;

	pr_info("set charge enable = %d, ret = %d\n", en, ret);
	return ret;
}

static int rt9471_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	*en = chip->charge_enabled;
	return 0;
}

static void rt9471_set_wdt(bool en)
{
	int ret;
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip || !chip->desc) {
		pr_err("%s get chip info fail\n", __func__);
		return;
	}

	if (en)
		ret = __rt9471_set_wdt(chip, chip->desc->wdt);
	else
		ret = __rt9471_set_wdt(chip, 0); /* 0: disable irq wdt */
	pr_info("%s set wdt as en:%d tm:%d ret:%d\n", __func__,
		en, chip->desc->wdt, ret);
}

static int rt9471_plug_in(struct charger_device *chg_dev)
{
	/* enable WDT when plug in */
	rt9471_set_wdt(true);

	pr_info("plug in set enable true\n");
	return rt9471_enable_chg(chg_dev, true);
}

static int rt9471_plug_out(struct charger_device *chg_dev)
{
	/* disable WDT when plug out */
	rt9471_set_wdt(false);

	pr_info("plug out set enable false\n");
	return rt9471_enable_chg(chg_dev, false);
}

static int rt9471_get_ichg(struct charger_device *chg_dev, u32 *ichg)
{
	struct rt9471_chip *chip = NULL;

	if (!ichg)
		return -EINVAL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	return __rt9471_get_ichg(chip, ichg);
}

static int rt9471_set_ichg(struct charger_device *chg_dev, u32 ichg)
{
	struct rt9471_chip *chip = NULL;

	ichg /= UNIT_CONVERSION;
	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	return __rt9471_set_ichg(chip, ichg);
}

static int rt9471_get_aicr(struct charger_device *chg_dev, u32 *aicr)
{
	struct rt9471_chip *chip = NULL;

	if (!aicr)
		return -EINVAL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	return __rt9471_get_aicr(chip, aicr);
}

static int rt9471_set_aicr(struct charger_device *chg_dev, u32 aicr)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	if (g_hiz_iin_limit_flag == HIZ_IIN_FLAG_TRUE) {
		pr_err("g_hiz_iin_limit_flag,just set 100mA\n");
		aicr = RT9471_AICR_100;
	}

	aicr /= UNIT_CONVERSION;
	return __rt9471_set_aicr(chip, aicr);
}

static int rt9471_get_cv(struct charger_device *chg_dev, u32 *cv)
{
	struct rt9471_chip *chip = NULL;

	if (!cv)
		return -EINVAL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	return __rt9471_get_cv(chip, cv);
}

static int rt9471_set_cv(struct charger_device *chg_dev, u32 cv)
{
	struct rt9471_chip *chip = NULL;
	struct rt9471_desc *desc = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	desc = chip->desc;
	if (!desc)
		return -EINVAL;

	if ((desc->cust_cv > CUST_MIN_CV) && (cv > desc->cust_cv)) {
		pr_info("set cv to custom_cv=%d\n", desc->cust_cv);
		cv = desc->cust_cv;
	}

	cv /= UNIT_CONVERSION;
	return __rt9471_set_cv(chip, cv);
}

static int rt9471_set_ieoc(struct charger_device *chg_dev, u32 uA)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	return __rt9471_set_ieoc(chip, uA / UNIT_CONVERSION);
}

static void rt9471_pg_state_dsm(struct rt9471_chip *chip)
{
	u8 stat[RT9471_IRQIDX_MAX] = {0};
	static int check_count;

	if (rt9471_i2c_block_read(chip, RT9471_REG_STAT0,
		RT9471_IRQIDX_MAX, stat) < 0) {
		check_count = 0;
		return;
	}
	if (!(stat[RT9471_IRQIDX_IRQ0] & RT9471_ST_VBUSGD_MASK) &&
		(chip->charge_enabled == true))
		check_count++;
	else
		check_count = 0;
	if (check_count >= CH_ENABLE_MAX_COUNT) {
		power_dsm_report_dmd(POWER_DSM_BATTERY,
			ERROR_WEAKSOURCE_STOP_CHARGE,
			"ERROR_WEAKSOURCE_STOP_CHARGE");
		check_count = 0;
	}
}

static int rt9471_kick_wdt(struct charger_device *chg_dev)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;
	if (g_otg_enable_flag == false)
		rt9471_pg_state_dsm(chip);
	return __rt9471_kick_wdt(chip);
}

static int rt9471_set_mivr(struct charger_device *chg_dev, u32 mivr)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;
	mivr /= UNIT_CONVERSION;
	return __rt9471_set_mivr(chip, mivr);
}

static int rt9471_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = NULL;
	int ret;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	/* set WDT during OTG */
	rt9471_set_wdt(en);

	ret = __rt9471_enable_otg(chip, en);
	if (ret == 0)
		g_otg_enable_flag = en;
	return ret;
}

static int rt9471_charger_do_event(struct charger_device *chg_dev,
	u32 event, u32 args)
{
	struct rt9471_chip *chip = NULL;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	switch (event) {
	case EVENT_EOC:
		pr_info("do eoc event\n");
		charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		pr_info("do recharge event\n");
		charger_dev_notify(chip->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return 0;
}

static int rt9471_enable_hz(struct charger_device *chg_dev, bool en)
{
	struct rt9471_chip *chip = NULL;
	struct rt9471_desc *desc = NULL;
	int ret;

	chip = i2c_get_clientdata(g_rt9471_i2c);
	if (!chip)
		return -EINVAL;

	desc = chip->desc;
	if (!desc)
		return -EINVAL;

	if (en > 0) {
		ret = __rt9471_enable_hz(chip, en);
	} else {
		ret = __rt9471_enable_hz(chip, en);
		g_hiz_iin_limit_flag = HIZ_IIN_FLAG_FALSE;
	}

	return ret;
}

static int rt9471_get_register_head(char *reg_head, int size, void *dev_data)
{
	char buff[BUF_LEN] = {0};
	int i;
	int len = 0;
	struct rt9471_chip *chip = dev_data;

	if (!reg_head || !chip)
		return -EINVAL;

	memset(reg_head, 0, size);
	for (i = 0; i < ARRAY_SIZE(g_rt9471_reg_addr); i++) {
		snprintf(buff, sizeof(buff), "Reg[0x%02X] ",
			g_rt9471_reg_addr[i]);

		len += strlen(buff);
		if (len < size)
			strncat(reg_head, buff, strlen(buff));
	}
	return 0;
}

static int rt9471_hw_dump_register(char *reg_value, int size, void *dev_data)
{
	int i;
	int ret;
	u8 regval;
	char buff[BUF_LEN] = {0};
	int len = 0;
	struct rt9471_chip *chip = dev_data;

	if (!reg_value || !chip)
		return -EINVAL;

	memset(reg_value, 0, size);
	for (i = 0; i < ARRAY_SIZE(g_rt9471_reg_addr); i++) {
		ret = rt9471_i2c_read_byte(chip, g_rt9471_reg_addr[i], &regval);
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

static struct charger_ops rt9471_ops = {
	.plug_in = rt9471_plug_in,
	.plug_out = rt9471_plug_out,
	.enable = rt9471_enable_chg,
	.is_enabled = rt9471_is_charging_enable,
	.dump_registers = NULL,
	.get_charging_current = rt9471_get_ichg,
	.set_charging_current = rt9471_set_ichg,
	.get_input_current = rt9471_get_aicr,
	.set_input_current = rt9471_set_aicr,
	.get_constant_voltage = rt9471_get_cv,
	.set_constant_voltage = rt9471_set_cv,
	.set_eoc_current = rt9471_set_ieoc,
	.kick_wdt = rt9471_kick_wdt,
	.set_mivr = rt9471_set_mivr,
	.is_charging_done = rt9471_is_charging_done,
	.get_min_charging_current = rt9471_get_min_ichg,
	.enable_otg = rt9471_enable_otg,
	.event = rt9471_charger_do_event,
	.enable_hz = rt9471_enable_hz,
};

static struct power_log_ops rt9471_log_ops = {
	.dev_name = "rt9471",
	.dump_log_head = rt9471_get_register_head,
	.dump_log_content = rt9471_hw_dump_register,
};

static int rt9471_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	struct rt9471_chip *chip = NULL;
	struct power_devices_info_data *pwr_dev_info = NULL;

	pr_info("%s begin\n", __func__);

	if (!client)
		return -EINVAL;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	mutex_init(&chip->hidden_mode_lock);
	chip->hidden_mode_cnt = 0;
	i2c_set_clientdata(client, chip);
	g_rt9471_i2c = client;

	if (!rt9471_check_devinfo(chip)) {
		ret = -ENODEV;
		goto free_chip;
	}

	ret = rt9471_parse_dt(chip);
	if (ret < 0) {
		pr_err("parse dt fail %d\n", ret);
		goto free_chip;
	}

	ret = rt9471_reset_register(chip);
	if (ret < 0)
		pr_err("reset register fail %d\n", ret);

	ret = rt9471_init_setting(chip);
	if (ret < 0) {
		pr_err("init setting fail %d\n", ret);
		goto free_desc;
	}

	ret = rt9471_register_irq(chip);
	if (ret < 0) {
		pr_err("register irq fail %d\n", ret);
		goto free_desc;
	}

	ret = rt9471_init_irq(chip);
	if (ret < 0) {
		pr_err("init irq fail %d\n", ret);
		goto free_desc;
	}

	chip->chg_dev = charger_device_register("primary_chg",
		&client->dev, chip, &rt9471_ops, &chip->chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		pr_err("rt9471 charge ops register fail\n");
		goto free_desc;
	}

	g_hiz_iin_limit_flag = HIZ_IIN_FLAG_FALSE;
	__rt9471_dump_registers(chip);
	rt9471_log_ops.dev_data = (void *)chip;
	power_log_ops_register(&rt9471_log_ops);

	pwr_dev_info = power_devices_info_register();
	if (pwr_dev_info) {
		pwr_dev_info->dev_name = chip->dev->driver->name;
		pwr_dev_info->dev_id = chip->dev_id;
		pwr_dev_info->ver_id = chip->dev_rev;
	}
	pr_info("%s probe end\n", __func__);
	return 0;

free_desc:
	if (!chip->desc && (chip->desc != &g_rt9471_default_desc)) {
		devm_kfree(chip->dev, chip->desc);
		chip->desc = NULL;
	}
free_chip:
	mutex_destroy(&chip->io_lock);
	mutex_destroy(&chip->hidden_mode_lock);
	devm_kfree(chip->dev, chip);
	chip = NULL;
	return ret;
}

static void rt9471_shutdown(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	if (chip)
		rt9471_reset_register(chip);
}

static int rt9471_remove(struct i2c_client *client)
{
	struct rt9471_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		pr_info("%s\n", __func__);
		mutex_destroy(&chip->io_lock);
		mutex_destroy(&chip->hidden_mode_lock);
	}
	return 0;
}

static int rt9471_suspend(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	return 0;
}

static int rt9471_resume(struct device *dev)
{
	struct rt9471_chip *chip = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9471_pm_ops, rt9471_suspend, rt9471_resume);

static const struct of_device_id rt9471_of_device_id[] = {
	{ .compatible = "richtek,rt9471", },
	{ .compatible = "richtek,swchg", },
	{},
};
MODULE_DEVICE_TABLE(of, rt9471_of_device_id);

static const struct i2c_device_id rt9471_i2c_device_id[] = {
	{ "rt9471", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, rt9471_i2c_device_id);

static struct i2c_driver rt9471_i2c_driver = {
	.driver = {
		.name = "rt9471",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt9471_of_device_id),
		.pm = &rt9471_pm_ops,
	},
	.probe = rt9471_probe,
	.shutdown = rt9471_shutdown,
	.remove = rt9471_remove,
	.id_table = rt9471_i2c_device_id,
};

static int __init rt9471_init(void)
{
	return i2c_add_driver(&rt9471_i2c_driver);
}

static void __exit rt9471_exit(void)
{
	i2c_del_driver(&rt9471_i2c_driver);
}

module_init(rt9471_init);
module_exit(rt9471_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("rt9471 charger module driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
