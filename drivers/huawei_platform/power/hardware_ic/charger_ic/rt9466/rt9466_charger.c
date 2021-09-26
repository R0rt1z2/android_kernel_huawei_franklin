/*
 * rt9466_charger.c
 *
 * rt9466 driver
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

#include "rt9466_charger.h"
#include "mtk_charger_intf.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_boot.h>
#include <chipset_common/hwpower/common_module/power_log.h>
#include <chipset_common/hwpower/common_module/power_devices_info.h>

#define BUF_LEN                    26
#define BITS_FOR_BYTE              8
#define RT9466_BUSNUM              1
#define I2C_ACCESS_MAX_RETRY       5
#define IRQ_HANDLER_TABLE_LEN      48
#define SLEW_RATE_IN_OTG_MODE      0x7c
#define SLEW_RATE_NOT_IN_OTG_MODE  0x73
#define HIDDEN_IN_OTG_MODE         0x00
#define HIDDEN_NOT_IN_OTG_MODE     0x0F
#define PERCENT                    100

static struct i2c_client *g_rt9466_i2c;
static const u32 g_rt9466_boost_oc_threshold[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
};

static const u32 g_rt9466_safety_timer[] = {
	4, 6, 8, 10, 12, 14, 16, 20,
};

enum rt9466_irq_idx {
	RT9466_IRQIDX_CHG_STATC = 0,
	RT9466_IRQIDX_CHG_FAULT,
	RT9466_IRQIDX_TS_STATC,
	RT9466_IRQIDX_CHG_IRQ1,
	RT9466_IRQIDX_CHG_IRQ2,
	RT9466_IRQIDX_CHG_IRQ3,
	RT9466_IRQIDX_MAX
};

enum rt9466_irq_stat {
	RT9466_IRQSTAT_CHG_STATC = 0,
	RT9466_IRQSTAT_CHG_FAULT,
	RT9466_IRQSTAT_TS_STATC,
	RT9466_IRQSTAT_MAX
};

static const u8 g_rt9466_irq_maskall[RT9466_IRQIDX_MAX] = {
	0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF
};

struct irq_mapping_tbl {
	const char *name;
	const int id;
};

#define rt9466_irq_mapping(_name, _id) { .name = #_name, .id = _id }
static const struct irq_mapping_tbl g_rt9466_irq_mapping_tbl[] = {
	rt9466_irq_mapping(chg_treg, 4),
	rt9466_irq_mapping(chg_aicr, 5),
	rt9466_irq_mapping(chg_mivr, 6),
	rt9466_irq_mapping(pwr_rdy, 7),
	rt9466_irq_mapping(chg_vsysuv, 12),
	rt9466_irq_mapping(chg_vsysov, 13),
	rt9466_irq_mapping(chg_vbatov, 14),
	rt9466_irq_mapping(chg_vbusov, 15),
	rt9466_irq_mapping(ts_batcold, 20),
	rt9466_irq_mapping(ts_batcool, 21),
	rt9466_irq_mapping(ts_batwarm, 22),
	rt9466_irq_mapping(ts_bathot, 23),
	rt9466_irq_mapping(ts_statci, 24),
	rt9466_irq_mapping(chg_faulti, 25),
	rt9466_irq_mapping(chg_statci, 26),
	rt9466_irq_mapping(chg_tmri, 27),
	rt9466_irq_mapping(chg_batabsi, 28),
	rt9466_irq_mapping(chg_adpbadi, 29),
	rt9466_irq_mapping(chg_rvpi, 30),
	rt9466_irq_mapping(otpi, 31),
	rt9466_irq_mapping(chg_aiclmeasi, 32),
	rt9466_irq_mapping(chg_ichgmeasi, 33),
	rt9466_irq_mapping(wdtmri, 35),
	rt9466_irq_mapping(ssfinishi, 36),
	rt9466_irq_mapping(chg_rechgi, 37),
	rt9466_irq_mapping(chg_termi, 38),
	rt9466_irq_mapping(chg_ieoci, 39),
	rt9466_irq_mapping(adc_donei, 40),
	rt9466_irq_mapping(pumpx_donei, 41),
	rt9466_irq_mapping(bst_batuvi, 45),
	rt9466_irq_mapping(bst_midovi, 46),
	rt9466_irq_mapping(bst_olpi, 47),
};

enum rt9466_charging_status {
	RT9466_CHG_STATUS_READY = 0,
	RT9466_CHG_STATUS_PROGRESS,
	RT9466_CHG_STATUS_DONE,
	RT9466_CHG_STATUS_FAULT,
	RT9466_CHG_STATUS_MAX
};

static const char *g_rt9466_chg_status_name[RT9466_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

static const u8 g_rt9466_val_en_hidden_mode[] = {
	0x49, 0x32, 0xB6, 0x27, 0x48, 0x18, 0x03, 0xE2,
};

enum rt9466_iin_limit_sel {
	RT9466_IIMLMTSEL_PSEL_OTG,
	RT9466_IINLMTSEL_AICR = 2,
	RT9466_IINLMTSEL_LOWER_LEVEL,
};

enum rt9466_adc_sel {
	RT9466_ADC_VBUS_DIV5 = 1,
	RT9466_ADC_VBUS_DIV2,
	RT9466_ADC_VSYS,
	RT9466_ADC_VBAT,
	RT9466_ADC_TS_BAT = 6,
	RT9466_ADC_IBUS = 8,
	RT9466_ADC_IBAT,
	RT9466_ADC_REGN = 11,
	RT9466_ADC_TEMP_JC,
	RT9466_ADC_MAX,
};

/*
 * Unit for each ADC parameter
 * 0 stands for reserved
 * For TS_BAT, the real unit is 0.25.
 * Here we use 25, please remember to divide 100 while showing the value
 */
static const int g_rt9466_adc_unit[RT9466_ADC_MAX] = {
	RT9466_RESERVED,
	RT9466_ADC_UNIT_VBUS_DIV5,
	RT9466_ADC_UNIT_VBUS_DIV2,
	RT9466_ADC_UNIT_VSYS,
	RT9466_ADC_UNIT_VBAT,
	RT9466_RESERVED,
	RT9466_ADC_UNIT_TS_BAT,
	RT9466_RESERVED,
	RT9466_ADC_UNIT_IBUS,
	RT9466_ADC_UNIT_IBAT,
	RT9466_RESERVED,
	RT9466_ADC_UNIT_REGN,
	RT9466_ADC_UNIT_TEMP_JC,
};

static const int g_rt9466_adc_offset[RT9466_ADC_MAX] = {
	0,
	RT9466_ADC_OFFSET_VBUS_DIV5,
	RT9466_ADC_OFFSET_VBUS_DIV2,
	RT9466_ADC_OFFSET_VSYS,
	RT9466_ADC_OFFSET_VBAT,
	0,
	RT9466_ADC_OFFSET_TS_BAT,
	0,
	RT9466_ADC_OFFSET_IBUS,
	RT9466_ADC_OFFSET_IBAT,
	0,
	RT9466_ADC_OFFSET_REGN,
	RT9466_ADC_OFFSET_TEMP_JC,
};

struct rt9466_desc {
	u32 ichg;
	u32 aicr;
	u32 mivr;
	u32 cv;
	u32 ieoc;
	u32 safety_timer;
	u32 ircmp_resistor;
	u32 ircmp_vclamp;
	bool en_te;
	bool en_wdt;
	bool en_irq_pulse;
	bool en_jeita;
	int regmap_represent_slave_addr;
	const char *regmap_name;
	const char *chg_dev_name;
	bool ceb_invert;
};

/* These default values will be applied if there's no property in dts */
static struct rt9466_desc g_rt9466_default_desc = {
	.ichg = RT9466_ICHG,
	.aicr = RT9466_AICR,
	.mivr = RT9466_MIVR,
	.cv = RT9466_CV,
	.ieoc = RT9466_IEOC,
	.safety_timer = RT9466_SAFETMR,
	.ircmp_resistor = IRCMP_RESISTOR,
	.ircmp_vclamp = IRCMP_VCLAMP,
	.en_te = true,
	.en_wdt = true,
	.en_irq_pulse = false,
	.en_jeita = false,
	.regmap_represent_slave_addr = RT9466_SLAVE_ADDR,
	.regmap_name = "rt9466",
	.chg_dev_name = "primary_chg",
	.ceb_invert = false,
};

struct rt9466_info {
	struct i2c_client *client;
	struct mutex i2c_access_lock;
	struct mutex adc_access_lock;
	struct mutex irq_access_lock;
	struct mutex aicr_access_lock;
	struct mutex ichg_access_lock;
	struct mutex pe_access_lock;
	struct mutex hidden_mode_lock;
	struct mutex ieoc_lock;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct rt9466_desc *desc;
	wait_queue_head_t wait_queue;
	int irq;
	int aicr_limit;
	u32 intr_gpio;
	u32 ceb_gpio;
	u8 chip_rev;
	u8 vendor_id;
	u8 irq_flag[RT9466_IRQIDX_MAX];
	u8 irq_stat[RT9466_IRQSTAT_MAX];
	u8 irq_mask[RT9466_IRQIDX_MAX];
	u32 hidden_mode_cnt;
	u32 ieoc;
	bool ieoc_wkard;
	struct work_struct init_work;
};

static const unsigned char g_rt9466_reg_addr[] = {
	RT9466_REG_CORE_CTRL0,
	RT9466_REG_CHG_CTRL1,
	RT9466_REG_CHG_CTRL2,
	RT9466_REG_CHG_CTRL3,
	RT9466_REG_CHG_CTRL4,
	RT9466_REG_CHG_CTRL5,
	RT9466_REG_CHG_CTRL6,
	RT9466_REG_CHG_CTRL7,
	RT9466_REG_CHG_CTRL8,
	RT9466_REG_CHG_CTRL9,
	RT9466_REG_CHG_CTRL10,
	RT9466_REG_CHG_CTRL11,
	RT9466_REG_CHG_CTRL12,
	RT9466_REG_CHG_CTRL13,
	RT9466_REG_CHG_CTRL14,
	RT9466_REG_CHG_CTRL15,
	RT9466_REG_CHG_CTRL16,
	RT9466_REG_CHG_ADC,
	RT9466_REG_CHG_CTRL19,
	RT9466_REG_CHG_CTRL17,
	RT9466_REG_CHG_CTRL18,
	RT9466_REG_DEVICE_ID,
	RT9466_REG_CHG_STAT,
	RT9466_REG_CHG_NTC,
	RT9466_REG_ADC_DATA_H,
	RT9466_REG_ADC_DATA_L,
	RT9466_REG_CHG_STATC,
	RT9466_REG_CHG_FAULT,
	RT9466_REG_TS_STATC,
	RT9466_REG_CHG_STATC_CTRL,
	RT9466_REG_CHG_FAULT_CTRL,
	RT9466_REG_TS_STATC_CTRL,
	RT9466_REG_CHG_IRQ1_CTRL,
	RT9466_REG_CHG_IRQ2_CTRL,
	RT9466_REG_CHG_IRQ3_CTRL,
};

static int rt9466_get_mivr(struct rt9466_info *info, u32 *mivr);
static int rt9466_get_aicr(struct charger_device *chg_dev, u32 *aicr);
static int rt9466_get_ichg(struct charger_device *chg_dev, u32 *ichg);
static int rt9466_set_aicr(struct charger_device *chg_dev, u32 aicr);
static int rt9466_set_ichg(struct charger_device *chg_dev, u32 aicr);
static int rt9466_kick_wdt(struct charger_device *chg_dev);
static int rt9466_enable_charging(struct charger_device *chg_dev, bool en);
static int rt9466_get_ieoc(struct rt9466_info *info, u32 *ieoc);
static void destroy_all_mutex(struct rt9466_info *info);

static int rt9466_device_read(struct i2c_client *client, u32 addr, int leng,
	void *dst)
{
	return i2c_smbus_read_i2c_block_data(client, addr, leng, dst);
}

static int rt9466_device_write(struct i2c_client *client, u32 addr, int leng,
	const void *src)
{
	return i2c_smbus_write_i2c_block_data(client, addr, leng, src);
}

static int __rt9466_i2c_write_byte(struct rt9466_info *info, u8 cmd,
	u8 data)
{
	int ret;
	int retry = 0;

	do {
		ret = rt9466_device_write(info->client, cmd, 1, &data);
		retry++;
		if (ret < 0)
			udelay(10); /* 10us */
	} while ((ret < 0) && (retry < I2C_ACCESS_MAX_RETRY));

	if (ret < 0)
		pr_debug("I2CW[0x%02X] = 0x%02X fail\n", cmd, data);
	else
		pr_debug("I2CW[0x%02X] = 0x%02X\n", cmd, data);

	return ret;
}

static int rt9466_i2c_write_byte(struct rt9466_info *info, u8 cmd, u8 data)
{
	int ret;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9466_i2c_write_byte(info, cmd, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static int __rt9466_i2c_read_byte(struct rt9466_info *info, u8 cmd)
{
	int ret;
	int ret_val = 0;
	int retry = 0;

	do {
		ret = rt9466_device_read(info->client, cmd, 1, &ret_val);
		retry++;
		if (ret < 0)
			udelay(10); /* 10us */
	} while ((ret < 0) && (retry < I2C_ACCESS_MAX_RETRY));

	if (ret < 0) {
		pr_debug("I2CR[0x%02X] fail\n", cmd);
		return ret;
	}

	ret_val = ret_val & 0xFF;
	pr_debug("I2CR[0x%02X] = 0x%02X\n", cmd, ret_val);

	return ret_val;
}

static int rt9466_i2c_read_byte(struct rt9466_info *info, u8 cmd)
{
	int ret;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9466_i2c_read_byte(info, cmd);
	mutex_unlock(&info->i2c_access_lock);

	return (ret < 0) ? ret : (ret & 0xFF);
}

static inline int __rt9466_i2c_block_write(struct rt9466_info *info, u8 cmd,
	u32 leng, const u8 *data)
{
	return rt9466_device_write(info->client, cmd, leng, data);
}

static int rt9466_i2c_block_write(struct rt9466_info *info, u8 cmd, u32 leng,
	const u8 *data)
{
	int ret;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9466_i2c_block_write(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int __rt9466_i2c_block_read(struct rt9466_info *info, u8 cmd,
	u32 leng, u8 *data)
{
	return rt9466_device_read(info->client, cmd, leng, data);
}

static int rt9466_i2c_block_read(struct rt9466_info *info, u8 cmd, u32 leng,
	u8 *data)
{
	int ret;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9466_i2c_block_read(info, cmd, leng, data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static int rt9466_i2c_test_bit(struct rt9466_info *info, u8 cmd, u8 shift,
	bool *is_one)
{
	int ret;
	u8 data;

	ret = rt9466_i2c_read_byte(info, cmd);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = ret & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}

static int rt9466_i2c_update_bits(struct rt9466_info *info, u8 cmd, u8 data,
	u8 mask)
{
	int ret;
	u8 reg_data;

	mutex_lock(&info->i2c_access_lock);
	ret = __rt9466_i2c_read_byte(info, cmd);
	if (ret < 0) {
		mutex_unlock(&info->i2c_access_lock);
		return ret;
	}

	reg_data = ret & 0xFF;
	reg_data &= ~mask;
	reg_data |= (data & mask);
	ret = __rt9466_i2c_write_byte(info, cmd, reg_data);
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static inline int rt9466_set_bit(struct rt9466_info *info, u8 reg, u8 mask)
{
	return rt9466_i2c_update_bits(info, reg, mask, mask);
}

static inline int rt9466_clr_bit(struct rt9466_info *info, u8 reg, u8 mask)
{
	return rt9466_i2c_update_bits(info, reg, 0x00, mask);
}

static inline void rt9466_irq_set_flag(struct rt9466_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq |= mask;
	mutex_unlock(&info->irq_access_lock);
}

static inline void rt9466_irq_clr_flag(struct rt9466_info *info, u8 *irq,
	u8 mask)
{
	mutex_lock(&info->irq_access_lock);
	*irq &= ~mask;
	mutex_unlock(&info->irq_access_lock);
}

static const char *rt9466_get_irq_name(struct rt9466_info *info,
	int irqnum)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_rt9466_irq_mapping_tbl); i++) {
		if (g_rt9466_irq_mapping_tbl[i].id == irqnum)
			return g_rt9466_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt9466_irq_mask(struct rt9466_info *info, int irqnum)
{
	pr_debug("irq = %d, %s\n", irqnum,
		rt9466_get_irq_name(info, irqnum));
	info->irq_mask[irqnum / RT9466_IRQ_COUNT_FOR_ONE_BYTE] |=
		(1 << (irqnum % RT9466_IRQ_COUNT_FOR_ONE_BYTE));
}

static inline void rt9466_irq_unmask(struct rt9466_info *info, int irqnum)
{
	pr_debug("irq = %d, %s\n", irqnum,
		rt9466_get_irq_name(info, irqnum));
	info->irq_mask[irqnum / RT9466_IRQ_COUNT_FOR_ONE_BYTE] &=
		~(1 << (irqnum % RT9466_IRQ_COUNT_FOR_ONE_BYTE));
}

static u8 rt9466_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if ((target < min) || (step == 0))
		return 0;

	if (target >= max)
		return (max - min) / step;

	return (target - min) / step;
}

static u8 rt9466_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
	u32 target)
{
	u32 i;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if ((target >= tbl[i]) && (target < tbl[i + 1]))
			return i;
	}

	return tbl_size - 1;
}

static inline u32 rt9466_closest_value(u32 min, u32 max, u32 step, u8 reg_val)
{
	u32 ret_val = min + reg_val * step;

	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static int rt9466_get_adc(struct rt9466_info *info,
	enum rt9466_adc_sel adc_sel, int *adc_val)
{
	int ret;
	int i;
	u32 aicr = 0;
	u32 ichg = 0;
	bool adc_start = false;
	const int max_wait_times = 6;
	u8 adc_data[RT9466_ADC_REG_LEN] = { 0 };

	mutex_lock(&info->adc_access_lock);

	/* Select ADC to desired channel */
	ret = rt9466_i2c_update_bits(info, RT9466_REG_CHG_ADC,
		adc_sel << RT9466_SHIFT_ADC_IN_SEL, RT9466_MASK_ADC_IN_SEL);
	if (ret < 0) {
		pr_info("select ch to %d fail: %d\n", adc_sel, ret);
		goto out_adc_sel;
	}

	/* Workaround for IBUS & IBAT */
	if (adc_sel == RT9466_ADC_IBUS) {
		mutex_lock(&info->aicr_access_lock);
		ret = rt9466_get_aicr(info->chg_dev, &aicr);
		if (ret < 0) {
			pr_info("get aicr fail\n");
			goto out_unlock_all;
		}
	} else if (adc_sel == RT9466_ADC_IBAT) {
		mutex_lock(&info->ichg_access_lock);
		ret = rt9466_get_ichg(info->chg_dev, &ichg);
		if (ret < 0) {
			pr_info("get ichg fail\n");
			goto out_unlock_all;
		}
	}

	/* Start ADC conversation */
	ret = rt9466_set_bit(info, RT9466_REG_CHG_ADC, RT9466_MASK_ADC_START);
	if (ret < 0) {
		pr_info("start con fail: %d, sel = %d\n", ret, adc_sel);
		goto out_unlock_all;
	}

	for (i = 0; i < max_wait_times; i++) {
		msleep(35); /* 35ms */
		ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_ADC,
			RT9466_SHIFT_ADC_START, &adc_start);
		if ((ret >= 0) && !adc_start)
			break;
	}
	if (i == max_wait_times) {
		pr_info("wait con fail: %d, sel = %d\n", ret, adc_sel);
		ret = -EINVAL;
		goto out_unlock_all;
	}

	mdelay(1); /* 1ms */

	/* Read ADC data high/low byte */
	ret = rt9466_i2c_block_read(info, RT9466_REG_ADC_DATA_H,
		RT9466_ADC_REG_LEN, adc_data);
	if (ret < 0) {
		pr_info("read ADC data fail\n");
		goto out_unlock_all;
	}

	/* Calculate ADC value */
	*adc_val = ((adc_data[0] << BITS_FOR_BYTE) + adc_data[1]) *
	g_rt9466_adc_unit[adc_sel] + g_rt9466_adc_offset[adc_sel];

	pr_info("adc_sel = %d, adc_h = 0x%02X, adc_l = 0x%02X, val = %d\n",
		adc_sel, adc_data[0], adc_data[1], *adc_val);

	ret = 0;

out_unlock_all:
	/* Coefficient of IBUS & IBAT */
	if (adc_sel == RT9466_ADC_IBUS) {
		if (aicr < RT9466_COEF_AICR_MAX)
			*adc_val = *adc_val * RT9466_COEF_AICR_RATE / PERCENT;
		mutex_unlock(&info->aicr_access_lock);
	} else if (adc_sel == RT9466_ADC_IBAT) {
		if ((ichg >= RT9466_COEF_ICHG_MIN_L) &&
			(ichg <= RT9466_COEF_ICHG_MAX_L))
			*adc_val = *adc_val * RT9466_COEF_ICHG_RATE_L / PERCENT;
		else if ((ichg >= RT9466_COEF_ICHG_MIN_H) &&
			(ichg <= RT9466_COEF_ICHG_MAX_H))
			*adc_val = *adc_val * RT9466_COEF_ICHG_RATE_H / PERCENT;
		mutex_unlock(&info->ichg_access_lock);
	}

out_adc_sel:
	mutex_unlock(&info->adc_access_lock);

	return ret;
}

static int rt9466_enable_ilim(struct rt9466_info *info, bool en)
{
	int ret;

	pr_info("ilim: enable = %d\n", en);

	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL3,
			RT9466_MASK_ILIM_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL3,
			RT9466_MASK_ILIM_EN);

	return ret;
}

static int __rt9466_set_aicr(struct rt9466_info *info, u32 aicr)
{
	u8 reg_aicr = rt9466_closest_reg(RT9466_AICR_MIN, RT9466_AICR_MAX,
		RT9466_AICR_STEP, aicr);

	pr_info("aicr = %d, reg = 0x%02X\n", aicr, reg_aicr);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL3,
		reg_aicr << RT9466_SHIFT_AICR, RT9466_MASK_AICR);
}

/* Prevent back boost */
static int rt9466_toggle_cfo(struct rt9466_info *info)
{
	int ret;
	u8 data = 0;

	pr_info("toggle_cfo\n");
	mutex_lock(&info->i2c_access_lock);
	ret = rt9466_device_read(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		pr_info("read cfo fail: %d\n", ret);
		goto out_i2c_io;
	}

	/* CFO off */
	data &= ~RT9466_MASK_CFO_EN;
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		pr_info("cfo off fail: %d\n", ret);
		goto out_i2c_io;
	}

	/* CFO on */
	data |= RT9466_MASK_CFO_EN;
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL2, 1, &data);
	if (ret < 0)
		pr_info("cfo on fail: %d\n", ret);

out_i2c_io:
	mutex_unlock(&info->i2c_access_lock);
	return ret;
}

/* IRQ handlers */
static int rt9466_pwr_rdy_irq_handler(struct rt9466_info *info)
{
	pr_info("pwr_rdy_irq_handler\n");
	return 0;
}

static int rt9466_chg_mivr_irq_handler(struct rt9466_info *info)
{
	int ret;
	bool mivr_act = false;
	int adc_ibus = 0;

	pr_info("chg_mivr_irq_handler\n");

	/* Check whether MIVR loop is active */
	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_STATC,
		RT9466_SHIFT_CHG_MIVR, &mivr_act);
	if (ret < 0) {
		pr_info("read mivr stat failed\n");
		return 0;
	}

	if (!mivr_act) {
		pr_info("mivr loop is not active\n");
		return 0;
	}

	if (strcmp(info->desc->chg_dev_name, "primary_chg") == 0) {
		ret = rt9466_get_adc(info, RT9466_ADC_IBUS, &adc_ibus);
		if (ret < 0) {
			pr_info("get ibus fail\n");
			return ret;
		}
		if (adc_ibus < RT9466_ICHG_MIN) {
			ret = rt9466_toggle_cfo(info);
			return ret;
		}
	}

	return 0;
}

static int rt9466_chg_aicr_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_aicr_irq_handler\n");
	return 0;
}

static int rt9466_chg_treg_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_treg_irq_handler\n");
	return 0;
}

static int rt9466_chg_vsysuv_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_vsysuv_irq_handler\n");
	return 0;
}

static int rt9466_chg_vsysov_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_vsysov_irq_handler\n");
	return 0;
}

static int rt9466_chg_vbatov_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_vbatov_irq_handler\n");
	return 0;
}

static int rt9466_chg_vbusov_irq_handler(struct rt9466_info *info)
{
	int ret;
	bool vbusov = false;
	struct chgdev_notify *noti = &(info->chg_dev->noti);

	pr_info("chg_vbusov_irq_handler\n");
	ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_FAULT,
		RT9466_SHIFT_VBUSOV, &vbusov);
	if (ret < 0)
		return ret;

	noti->vbusov_stat = vbusov;
	pr_info("vbusov = %d\n", vbusov);
	charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	return 0;
}

static int rt9466_ts_bat_cold_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_ts_bat_cool_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_ts_bat_warm_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_ts_bat_hot_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_ts_statci_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_faulti_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_statci_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_tmri_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_tmri_irq_handler\n");
	charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
	return 0;
}

static int rt9466_chg_batabsi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_adpbadi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_rvpi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_otpi_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_otpi_irq_handler\n");
	return 0;
}

static int rt9466_chg_aiclmeasi_irq_handler(struct rt9466_info *info)
{
	pr_info("chg_aiclmeasi_irq_handler\n");
	rt9466_irq_set_flag(info, &info->irq_flag[RT9466_IRQIDX_CHG_IRQ2],
		RT9466_MASK_CHG_AICLMEASI);
	wake_up_interruptible(&info->wait_queue);
	return 0;
}

static int rt9466_chg_ichgmeasi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_wdtmri_irq_handler(struct rt9466_info *info)
{
	int ret;

	pr_info("wdtmri_irq_handler\n");
	ret = rt9466_kick_wdt(info->chg_dev);
	if (ret < 0)
		pr_info("kick wdt fail\n");

	return ret;
}

static int rt9466_ssfinishi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_rechgi_irq_handler(struct rt9466_info *info)
{
	charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
	return 0;
}

static int rt9466_chg_termi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_chg_ieoci_irq_handler(struct rt9466_info *info)
{
	charger_dev_notify(info->chg_dev, CHARGER_DEV_NOTIFY_EOC);
	return 0;
}

static int rt9466_adc_donei_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_pumpx_donei_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_bst_batuvi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_bst_midovi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

static int rt9466_bst_olpi_irq_handler(struct rt9466_info *info)
{
	return 0;
}

typedef int (*rt9466_irq_fptr)(struct rt9466_info *);
static rt9466_irq_fptr g_rt9466_irq_handler_tbl[IRQ_HANDLER_TABLE_LEN] = {
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_chg_treg_irq_handler,
	rt9466_chg_aicr_irq_handler,
	rt9466_chg_mivr_irq_handler,
	rt9466_pwr_rdy_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_chg_vsysuv_irq_handler,
	rt9466_chg_vsysov_irq_handler,
	rt9466_chg_vbatov_irq_handler,
	rt9466_chg_vbusov_irq_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	rt9466_ts_bat_cold_irq_handler,
	rt9466_ts_bat_cool_irq_handler,
	rt9466_ts_bat_warm_irq_handler,
	rt9466_ts_bat_hot_irq_handler,
	rt9466_ts_statci_irq_handler,
	rt9466_chg_faulti_irq_handler,
	rt9466_chg_statci_irq_handler,
	rt9466_chg_tmri_irq_handler,
	rt9466_chg_batabsi_irq_handler,
	rt9466_chg_adpbadi_irq_handler,
	rt9466_chg_rvpi_irq_handler,
	rt9466_chg_otpi_irq_handler,
	rt9466_chg_aiclmeasi_irq_handler,
	rt9466_chg_ichgmeasi_irq_handler,
	NULL,
	rt9466_wdtmri_irq_handler,
	rt9466_ssfinishi_irq_handler,
	rt9466_chg_rechgi_irq_handler,
	rt9466_chg_termi_irq_handler,
	rt9466_chg_ieoci_irq_handler,
	rt9466_adc_donei_irq_handler,
	rt9466_pumpx_donei_irq_handler,
	NULL,
	NULL,
	NULL,
	rt9466_bst_batuvi_irq_handler,
	rt9466_bst_midovi_irq_handler,
	rt9466_bst_olpi_irq_handler,
};

static int rt9466_enable_irqrez(struct rt9466_info *info, bool en)
{
	int ret;

	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL13,
			RT9466_MASK_IRQ_REZ);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL13,
			RT9466_MASK_IRQ_REZ);

	return ret;
}

static int __rt9466_irq_handler(struct rt9466_info *info)
{
	int ret;
	int i;
	int j;
	int tbl_index;
	u8 evt[RT9466_IRQIDX_MAX] = { 0 };
	u8 mask[RT9466_IRQIDX_MAX] = { 0 };
	u8 stat[RT9466_IRQSTAT_MAX] = { 0 };

	/* Read IRQ1, IRQ2 event and skip CHG_IRQ3 */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_IRQ1,
		RT9466_IRQIDX_CHG_IRQ3 - RT9466_IRQIDX_CHG_IRQ1,
		&evt[RT9466_IRQIDX_CHG_IRQ1]);
	if (ret < 0) {
		pr_info("read evt fail: %d\n", ret);
		goto err_read_irq;
	}

	/* Read CHG_STATC, CHG_FAULT, TS_STATC into evt */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC,
		RT9466_IRQIDX_CHG_IRQ1 - RT9466_IRQIDX_CHG_STATC, evt);
	if (ret < 0) {
		pr_info("read stat fail: %d\n", ret);
		goto err_read_irq;
	}

	/* Read mask */
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(mask), mask);
	if (ret < 0) {
		pr_info("read mask fail: %d\n", ret);
		goto err_read_irq;
	}

	/* Store/Update stat */
	memcpy(stat, info->irq_stat, RT9466_IRQSTAT_MAX);

	for (i = 0; i < RT9466_IRQIDX_MAX; i++) {
		evt[i] &= ~mask[i];
		if (i < RT9466_IRQSTAT_MAX) {
			info->irq_stat[i] = evt[i];
			evt[i] ^= stat[i];
		}
		for (j = 0; j < RT9466_IRQ_GROUP_LEN; j++) {
			if (!(evt[i] & (1 << j)))
				continue;
			tbl_index = i * RT9466_IRQ_GROUP_LEN + j;
			if (g_rt9466_irq_handler_tbl[tbl_index])
				g_rt9466_irq_handler_tbl[tbl_index](info);
		}
	}

err_read_irq:
	return ret;
}

static irqreturn_t rt9466_irq_handler(int irq, void *data)
{
	int ret;
	struct rt9466_info *info = (struct rt9466_info *)data;

	ret = __rt9466_irq_handler(info);
	if (ret < 0)
		pr_info("_irq_handler fail\n");
	ret = rt9466_enable_irqrez(info, true);
	if (ret < 0)
		pr_info("en irqrez fail\n");

	return IRQ_HANDLED;
}

static int rt9466_irq_register(struct rt9466_info *info)
{
	int ret;
	int len;
	char *name = NULL;

	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0)
		return 0;

	pr_info("irq_register\n");

	/* request gpio */
	len = strlen(info->desc->chg_dev_name);
	name = devm_kzalloc(info->dev, len + CHG_IRQGPIO_NAME_LEN,
		GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	snprintf(name,  len + CHG_IRQGPIO_NAME_LEN, "%s_irq_gpio",
		info->desc->chg_dev_name);
	ret = devm_gpio_request_one(info->dev, info->intr_gpio, GPIOF_IN, name);
	if (ret < 0) {
		pr_info("gpio request fail\n");
		goto free_name;
	}

	pr_info("irq = %d\n", info->irq);
	info->irq = gpio_to_irq(info->intr_gpio);
	memset(name, 0, len + CHG_IRQGPIO_NAME_LEN);
	/* Request threaded IRQ */
	snprintf(name, len + CHG_IRQ_NAME_LEN, "%s_irq",
		info->desc->chg_dev_name);
	ret = devm_request_threaded_irq(info->dev, info->irq, NULL,
		rt9466_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, name,
		info);
	if (ret < 0) {
		pr_info("request thread irq fail\n");
		goto free_name;
	}
	device_init_wakeup(info->dev, true);

free_name:
	devm_kfree(info->dev, name);
	name = NULL;
	return 0;
}

static inline int rt9466_maskall_irq(struct rt9466_info *info)
{
	return rt9466_i2c_block_write(info, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(g_rt9466_irq_maskall), g_rt9466_irq_maskall);
}

static inline int rt9466_irq_init(struct rt9466_info *info)
{
	pr_info("irq_init\n");
	return rt9466_i2c_block_write(info, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(info->irq_mask), info->irq_mask);
}

static bool rt9466_is_hw_exist(struct rt9466_info *info)
{
	int ret;
	u8 vendor_id;
	u8 chip_rev;

	ret = i2c_smbus_read_byte_data(info->client, RT9466_REG_DEVICE_ID);
	if (ret < 0)
		return false;

	vendor_id = ret & 0xF0;
	chip_rev = ret & 0x0F;
	if (vendor_id != RT9466_VENDOR_ID) {
		pr_info("vendor id is incorrect 0x%02X\n", vendor_id);
		return false;
	}

	ret = i2c_smbus_read_byte_data(info->client, RT9466_REG_CORE_CTRL0);
	if (ret < 0)
		return false;

	if (ret != RT9466_BEGIN_INFO) {
		pr_info("check_begin_info is incorrect 0x%02X\n", ret);
		return false;
	}

	pr_info("0x%02X\n", chip_rev);
	info->vendor_id = vendor_id;
	info->chip_rev = chip_rev;

	return true;
}

static int rt9466_set_safety_timer(struct rt9466_info *info, u32 hr)
{
	u8 reg_st;

	reg_st = rt9466_closest_reg_via_tbl(g_rt9466_safety_timer,
		ARRAY_SIZE(g_rt9466_safety_timer), hr);

	pr_info(" set safe time = %d, reg = 0x%02X\n", hr, reg_st);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL12,
		reg_st << RT9466_SHIFT_WT_FC, RT9466_MASK_WT_FC);
}

static int rt9466_enable_wdt(struct rt9466_info *info, bool en)
{
	int ret;

	pr_info("wdt: enable = %d\n", en);
	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL13,
			RT9466_MASK_WDT_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL13,
			RT9466_MASK_WDT_EN);

	return ret;
}

static int rt9466_select_input_current_limit(struct rt9466_info *info,
	enum rt9466_iin_limit_sel sel)
{
	pr_info("input_current_limit: sel = %d\n", sel);
	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL2,
		sel << RT9466_SHIFT_IINLMTSEL, RT9466_MASK_IINLMTSEL);
}

static int rt9466_enable_hidden_mode(struct rt9466_info *info, bool en)
{
	int ret = 0;

	mutex_lock(&info->hidden_mode_lock);

	if (en) {
		if (info->hidden_mode_cnt == 0) {
			ret = rt9466_i2c_block_write(info,
				RT9466_REG_HIDDEN_MODE,
				ARRAY_SIZE(g_rt9466_val_en_hidden_mode),
				g_rt9466_val_en_hidden_mode);
			if (ret < 0)
				goto i2c_err;
		}
		info->hidden_mode_cnt++;
	} else {
		if (info->hidden_mode_cnt == 1) {
			ret = rt9466_i2c_write_byte(info,
				RT9466_REG_HIDDEN_MODE, 0x00);
			if (ret < 0)
				goto i2c_err;
		}
		info->hidden_mode_cnt--;
	}
	pr_debug("hidden_mode: en = %d\n", en);
	mutex_unlock(&info->hidden_mode_lock);

	return ret;

i2c_err:
	pr_debug("hidden_mode: en = %d fail: %d\n", en, ret);
	mutex_unlock(&info->hidden_mode_lock);
	return ret;
}

static int rt9466_set_iprec(struct rt9466_info *info, u32 iprec)
{
	u8 reg_iprec;

	reg_iprec = rt9466_closest_reg(RT9466_IPREC_MIN, RT9466_IPREC_MAX,
		RT9466_IPREC_STEP, iprec);

	pr_info("iprec = %d, reg = 0x%02X\n", iprec, reg_iprec);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL8,
		reg_iprec << RT9466_SHIFT_IPREC, RT9466_MASK_IPREC);
}

static int rt9466_sw_workaround(struct rt9466_info *info)
{
	int ret;

	pr_info("sw_workaround\n");

	rt9466_enable_hidden_mode(info, true);

	/* Disable TS auto sensing */
	ret = rt9466_clr_bit(info, RT9466_REG_CHG_HIDDEN_CTRL15, 0x01);
	if (ret < 0)
		goto disable_hidden;

	/* Set precharge current to 850mA, only do this in normal boot */
	if (info->chip_rev <= RT9466_CHIP_REV_E3) {
		/* Worst case delay: wait auto sensing */
		msleep(200); /* 200ms */

		if (get_boot_mode() == NORMAL_BOOT) {
			ret = rt9466_set_iprec(info, RT9466_IPREC_MAX);
			if (ret < 0)
				goto disable_hidden;

			/* Increase Isys drop threshold to 2.5A */
			ret = rt9466_i2c_write_byte(info,
				RT9466_REG_CHG_HIDDEN_CTRL7, 0x1c);
			if (ret < 0)
				goto disable_hidden;
		}
	}

	/* Only revision <= E1 needs the following workaround */
	if (info->chip_rev > RT9466_CHIP_REV_E1)
		goto disable_hidden;

	/* ICC: modify sensing node, make it more accurate */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL8, 0x00);
	if (ret < 0)
		goto disable_hidden;

	/* DIMIN level */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL9, 0x86);

disable_hidden:
	rt9466_enable_hidden_mode(info, false);
	return ret;
}

static int rt9466_enable_hz(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	pr_info("hz: enable = %d\n", en);
	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_HZ_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_HZ_EN);

	return ret;
}

/* Reset all registers' value to default */
static int rt9466_reset_chip(struct rt9466_info *info)
{
	int ret;

	pr_info("reset_chip\n");

	/* disable hz before reset chip */
	ret = rt9466_enable_hz(info->chg_dev, false);
	if (ret < 0) {
		pr_debug("disable hz fail\n");
		return ret;
	}

	return rt9466_set_bit(info, RT9466_REG_CORE_CTRL0, RT9466_MASK_RST);
}

static int __rt9466_enable_te(struct rt9466_info *info, bool en)
{
	int ret;

	pr_info("te: enable = %d\n", en);
	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL2,
			RT9466_MASK_TE_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL2,
			RT9466_MASK_TE_EN);

	return ret;
}

static int __rt9466_enable_safety_timer(struct rt9466_info *info,
	bool en)
{
	int ret;

	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL12,
			RT9466_MASK_TMR_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL12,
			RT9466_MASK_TMR_EN);

	return ret;
}

static int __rt9466_set_ieoc(struct rt9466_info *info, u32 ieoc)
{
	int ret;
	u8 reg_ieoc;

	/* IEOC workaround */
	if (info->ieoc_wkard)
		ieoc += IEOC_WORK_AROUND;

	reg_ieoc = rt9466_closest_reg(RT9466_IEOC_MIN, RT9466_IEOC_MAX,
		RT9466_IEOC_STEP, ieoc);

	pr_info("ieoc = %d, reg = 0x%02X\n", ieoc, reg_ieoc);

	ret = rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL9,
		reg_ieoc << RT9466_SHIFT_IEOC, RT9466_MASK_IEOC);
	if (ret < 0)
		pr_info("ret = %d\n", ret);

	/* Store IEOC */
	return rt9466_get_ieoc(info, &info->ieoc);
}

static int rt9466_get_mivr(struct rt9466_info *info, u32 *mivr)
{
	int ret;
	u8 reg_mivr;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL6);
	if (ret < 0)
		return ret;
	reg_mivr = ((ret & RT9466_MASK_MIVR) >> RT9466_SHIFT_MIVR) & 0xFF;

	*mivr = rt9466_closest_value(RT9466_MIVR_MIN, RT9466_MIVR_MAX,
		RT9466_MIVR_STEP, reg_mivr);

	return ret;
}

static int __rt9466_set_mivr(struct rt9466_info *info, u32 mivr)
{
	u8 reg_mivr;

	reg_mivr = rt9466_closest_reg(RT9466_MIVR_MIN, RT9466_MIVR_MAX,
		RT9466_MIVR_STEP, mivr);

	pr_info("mivr = %d, reg = 0x%02X\n", mivr, reg_mivr);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL6,
		reg_mivr << RT9466_SHIFT_MIVR, RT9466_MASK_MIVR);
}

static int rt9466_enable_jeita(struct rt9466_info *info, bool en)
{
	int ret;

	pr_info("jeita: enable = %d\n", en);
	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL16,
			RT9466_MASK_JEITA_EN);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL16,
			RT9466_MASK_JEITA_EN);

	return ret;
}


static int rt9466_get_charging_status(struct rt9466_info *info,
	enum rt9466_charging_status *chg_stat)
{
	int ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_STAT);

	if (ret < 0)
		return ret;

	*chg_stat = (ret & RT9466_MASK_CHG_STAT) >> RT9466_SHIFT_CHG_STAT;

	return ret;
}

static int rt9466_get_ieoc(struct rt9466_info *info, u32 *ieoc)
{
	int ret;
	u8 reg_ieoc;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL9);
	if (ret < 0)
		return ret;

	reg_ieoc = (ret & RT9466_MASK_IEOC) >> RT9466_SHIFT_IEOC;
	*ieoc = rt9466_closest_value(RT9466_IEOC_MIN, RT9466_IEOC_MAX,
		RT9466_IEOC_STEP, reg_ieoc);

	return ret;
}

static inline int __rt9466_is_charging_enable(struct rt9466_info *info,
	bool *en)
{
	return rt9466_i2c_test_bit(info, RT9466_REG_CHG_CTRL2,
		RT9466_SHIFT_CHG_EN, en);
}

static int __rt9466_set_ichg(struct rt9466_info *info, u32 ichg)
{
	u8 reg_ichg = rt9466_closest_reg(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, ichg);

	pr_info("ichg = %d, reg = 0x%02X\n", ichg, reg_ichg);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL7,
		reg_ichg << RT9466_SHIFT_ICHG, RT9466_MASK_ICHG);
}

static int __rt9466_set_cv(struct rt9466_info *info, u32 cv)
{
	u8 reg_cv = rt9466_closest_reg(RT9466_CV_MIN, RT9466_CV_MAX,
		RT9466_CV_STEP, cv);

	pr_info("cv = %d, reg = 0x%02X\n", cv, reg_cv);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL4,
		reg_cv << RT9466_SHIFT_CV, RT9466_MASK_CV);
}

static int rt9466_set_ircmp_resistor(struct rt9466_info *info, u32 uohm)
{
	u8 reg_resistor = rt9466_closest_reg(RT9466_IRCMP_RES_MIN,
		RT9466_IRCMP_RES_MAX, RT9466_IRCMP_RES_STEP, uohm);

	pr_info("resistor = %d, reg = 0x%02X\n", uohm, reg_resistor);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL18,
		reg_resistor << RT9466_SHIFT_IRCMP_RES, RT9466_MASK_IRCMP_RES);
}

static int rt9466_set_ircmp_vclamp(struct rt9466_info *info, u32 uV)
{
	u8 reg_vclamp = rt9466_closest_reg(RT9466_IRCMP_VCLAMP_MIN,
		RT9466_IRCMP_VCLAMP_MAX, RT9466_IRCMP_VCLAMP_STEP, uV);

	pr_info("vclamp = %d, reg = 0x%02X\n", uV, reg_vclamp);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL18,
		reg_vclamp << RT9466_SHIFT_IRCMP_VCLAMP,
		RT9466_MASK_IRCMP_VCLAMP);
}

static int rt9466_enable_irq_pulse(struct rt9466_info *info, bool en)
{
	int ret;

	pr_info("irq pulse: en = %d\n", en);
	if (en)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_IRQ_PULSE);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_IRQ_PULSE);

	return ret;
}

static int rt9466_disable_charger_reset(struct rt9466_info *info, bool disable)
{
	int ret;

	pr_info("charger_reset: enable = %d\n", disable);
	if (disable)
		ret = rt9466_set_bit(info, RT9466_REG_CHG_CTRL19,
			RT9466_MASK_RESET_DISABLE);
	else
		ret = rt9466_clr_bit(info, RT9466_REG_CHG_CTRL19,
			RT9466_MASK_RESET_DISABLE);

	return ret;
}

static int rt9466_get_irq_number(struct rt9466_info *info,
	const char *name)
{
	int i;

	if (!name) {
		pr_debug("get irq number: null name\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(g_rt9466_irq_mapping_tbl); i++) {
		if (!strcmp(name, g_rt9466_irq_mapping_tbl[i].name))
			return g_rt9466_irq_mapping_tbl[i].id;
	}

	return -EINVAL;
}

static void parse_and_init_para(struct rt9466_desc *desc,
	struct device_node *np)
{
	if (of_property_read_u32(np, "regmap_represent_slave_addr",
		&desc->regmap_represent_slave_addr) < 0)
		pr_info("parse_dt: no regmap slave addr\n");

	if (of_property_read_string(np, "regmap_name",
		&(desc->regmap_name)) < 0)
		pr_info("parse_dt: no regmap name\n");

	if (of_property_read_u32(np, "ichg", &desc->ichg) < 0)
		pr_info("parse_dt: no ichg\n");

	if (of_property_read_u32(np, "aicr", &desc->aicr) < 0)
		pr_info("parse_dt: no aicr\n");

	if (of_property_read_u32(np, "mivr", &desc->mivr) < 0)
		pr_info("parse_dt: no mivr\n");

	if (of_property_read_u32(np, "cv", &desc->cv) < 0)
		pr_info("parse_dt: no cv\n");

	if (of_property_read_u32(np, "ieoc", &desc->ieoc) < 0)
		pr_info("parse_dt: no ieoc\n");

	if (of_property_read_u32(np, "safety_timer", &desc->safety_timer) < 0)
		pr_info("parse_dt: no safety timer\n");

	if (of_property_read_u32(np, "ircmp_resistor",
		&desc->ircmp_resistor) < 0)
		pr_info("parse_dt: no ircmp resistor\n");

	if (of_property_read_u32(np, "ircmp_vclamp", &desc->ircmp_vclamp) < 0)
		pr_info("parse_dt: no ircmp vclamp\n");

	desc->en_te = of_property_read_bool(np, "en_te");
	desc->en_wdt = of_property_read_bool(np, "en_wdt");
	desc->en_irq_pulse = of_property_read_bool(np, "en_irq_pulse");
	desc->en_jeita = of_property_read_bool(np, "en_jeita");
	desc->ceb_invert = of_property_read_bool(np, "ceb_invert");
}

static int rt9466_parse_dt(struct rt9466_info *info, struct device *dev)
{
	int ret;
	int irqnum;
	int irq_cnt = 0;
	struct rt9466_desc *desc = NULL;
	struct device_node *np = NULL;
	const char *name = NULL;

	pr_info("parse_dt\n");

	np = of_get_child_by_name(dev->of_node, "rt9466");
	if (!np)
		np = dev->of_node;
	if (!np) {
		pr_info("parse_dt: no device node\n");
		return -EINVAL;
	}

	info->desc = &g_rt9466_default_desc;
	desc = devm_kzalloc(dev, sizeof(struct rt9466_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &g_rt9466_default_desc, sizeof(struct rt9466_desc));

	if (of_property_read_string(np, "charger_name",
		&desc->chg_dev_name) < 0)
		pr_info("parse_dt: no charger name\n");

	info->irq = irq_of_parse_and_map(np, 0);
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "rt,intr_gpio", 0);
	if (ret < 0) {
		pr_info("parse_dt: no rt,intr_gpio\n");
		goto free_desc;
	}
	info->intr_gpio = ret;

	if (strcmp(desc->chg_dev_name, "primary_chg") == 0) {
		ret = of_get_named_gpio(np, "rt,ceb_gpio", 0);
		if (ret < 0) {
			pr_info("parse_dt: no rt,ceb_gpio\n");
			goto free_desc;
		}
		info->ceb_gpio = ret;
	}
#else
	ret = of_property_read_u32(np, "rt,intr_gpio_num", &info->intr_gpio);
	if (ret < 0) {
		pr_info("parse_dt: no rt,intr_gpio_num\n");
		goto free_desc;
	}
	if (strcmp(desc->chg_dev_name, "primary_chg") == 0) {
		ret = of_property_read_u32(np, "rt,ceb_gpio_num",
			&info->ceb_gpio);
		if (ret < 0) {
			pr_info("parse_dt: no rt,ceb_gpio_num\n");
			goto free_desc;
		}
	}
#endif /* (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND)) */

	pr_info("parse_dt: intr/ceb gpio = %d, %d\n", info->intr_gpio,
		info->ceb_gpio);

	/* request ceb gpio for secondary charger */
	if (strcmp(desc->chg_dev_name, "primary_chg") == 0) {
		ret = devm_gpio_request_one(info->dev, info->ceb_gpio,
			GPIOF_DIR_OUT, "rt9466_sec_ceb_gpio");
		if (ret < 0) {
			pr_info("parse_dt: ceb gpio request fail\n");
			goto free_desc;
		}
	}

	parse_and_init_para(desc, np);

	while (true) {
		ret = of_property_read_string_index(np, "interrupt-names",
			irq_cnt, &name);
		if (ret < 0) {
			pr_info("parse_dt: no interrupt-names\n");
			break;
		}
		irq_cnt++;
		irqnum = rt9466_get_irq_number(info, name);
		if (irqnum >= 0)
			rt9466_irq_unmask(info, irqnum);
	}

	info->desc = desc;

	return 0;
free_desc:
	if (!info->desc && (info->desc != &g_rt9466_default_desc)) {
		devm_kfree(info->dev, info->desc);
		info->desc = NULL;
	}

	return ret;
}

static int rt9466_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	pr_info("charging: en = %d\n", en);

	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		ret = rt9466_enable_hz(info->chg_dev, !en);
		if (ret < 0) {
			pr_info("set hz of sec chg fail\n");
			return ret;
		}

		if (info->desc->ceb_invert)
			gpio_set_value(info->ceb_gpio, en);
		else
			gpio_set_value(info->ceb_gpio, !en);
	}

	if (en)
		return rt9466_set_bit(info, RT9466_REG_CHG_CTRL2,
			RT9466_MASK_CHG_EN);
	else
		return rt9466_clr_bit(info, RT9466_REG_CHG_CTRL2,
			RT9466_MASK_CHG_EN);
}

static int rt9466_set_boost_current_limit(struct charger_device *chg_dev,
	u32 current_limit)
{
	u8 reg_ilimit;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	reg_ilimit = rt9466_closest_reg_via_tbl(g_rt9466_boost_oc_threshold,
		ARRAY_SIZE(g_rt9466_boost_oc_threshold), current_limit);

	pr_info("boost ilimit = %d, reg = 0x%02X\n",
		current_limit, reg_ilimit);

	return rt9466_i2c_update_bits(info, RT9466_REG_CHG_CTRL10,
		reg_ilimit << RT9466_SHIFT_BOOST_OC, RT9466_MASK_BOOST_OC);
}

static void otg_en_err(struct rt9466_info *info)
{
	int ret;

	ret = rt9466_enable_wdt(info, false);
	if (ret < 0)
		pr_info("disable wdt fail\n");

	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL4,
		SLEW_RATE_NOT_IN_OTG_MODE);
	if (ret < 0)
		pr_info("recover Low side mos Gate drive speed fail: %d\n",
			ret);
}

static void set_opa_mode(struct rt9466_info *info, bool en)
{
	if (en)
		rt9466_set_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_OPA_MODE);
	else
		rt9466_clr_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_MASK_OPA_MODE);
}

static int rt9466_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	u8 hidden_val;
	u8 lg_slew_rate;
	bool en_otg = false;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	pr_info("otg: en = %d\n", en);
	if (en) {
		hidden_val = HIDDEN_IN_OTG_MODE;
		lg_slew_rate = SLEW_RATE_IN_OTG_MODE;
	} else {
		hidden_val = HIDDEN_NOT_IN_OTG_MODE;
		lg_slew_rate = SLEW_RATE_NOT_IN_OTG_MODE;
	}

	rt9466_enable_hidden_mode(info, true);

	/* Set OTG_OC to 1100000uA, 1100mA */
	ret = rt9466_set_boost_current_limit(chg_dev, 1100000);
	if (ret < 0) {
		pr_info("set current limit fail\n");
		return ret;
	}

	/*
	 * Woraround : slow Low side mos Gate driver slew rate
	 * for decline VBUS noise
	 * reg[0x23] = 0x7c after entering OTG mode
	 * reg[0x23] = 0x73 after leaving OTG mode
	 */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL4,
		lg_slew_rate);
	if (ret < 0) {
		pr_info("set Low side mos Gate drive speed fail: %d\n", ret);
		goto out_disable_hidden;
	}

	if (en && info->desc->en_wdt) {
		ret = rt9466_enable_wdt(info, true);
		if (ret < 0) {
			pr_info("en wdt fail\n");
			goto err_en_otg;
		}
	}

	set_opa_mode(info, en);
	msleep(20); /* 20ms */

	if (en) {
		ret = rt9466_i2c_test_bit(info, RT9466_REG_CHG_CTRL1,
			RT9466_SHIFT_OPA_MODE, &en_otg);
		if ((ret < 0) || !en_otg) {
			pr_info("otg fail: %d\n", ret);
			goto err_en_otg;
		}
	}

	/*
	 * Woraround reg[0x25] = 0x00 after entering OTG mode
	 * reg[0x25] = 0x0F after leaving OTG mode
	 */
	ret = rt9466_i2c_write_byte(info, RT9466_REG_CHG_HIDDEN_CTRL6,
		hidden_val);
	if (ret < 0)
		pr_info("workaroud fail: %d\n", ret);

	if (!en) {
		ret = rt9466_enable_wdt(info, false);
		if (ret < 0)
			pr_info("disable wdt fail\n");
	}
	goto out_disable_hidden;

err_en_otg:
	otg_en_err(info);
	ret = -EIO;
out_disable_hidden:
	rt9466_enable_hidden_mode(info, false);
	return ret;
}

static int rt9466_set_ichg(struct charger_device *chg_dev, u32 ichg)
{
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	return __rt9466_set_ichg(info, ichg);
}

static int rt9466_set_ieoc(struct charger_device *chg_dev, u32 ieoc)
{
	int ret;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	mutex_lock(&info->ichg_access_lock);
	mutex_lock(&info->ieoc_lock);
	ret = __rt9466_set_ieoc(info, ieoc);
	mutex_unlock(&info->ieoc_lock);
	mutex_unlock(&info->ichg_access_lock);

	return ret;
}

static int rt9466_set_aicr(struct charger_device *chg_dev, u32 aicr)
{
	int ret;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	mutex_lock(&info->aicr_access_lock);
	ret = __rt9466_set_aicr(info, aicr);
	mutex_unlock(&info->aicr_access_lock);

	return ret;
}

static int rt9466_set_mivr(struct charger_device *chg_dev, u32 mivr)
{
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	return __rt9466_set_mivr(info, mivr);
}

static int rt9466_set_cv(struct charger_device *chg_dev, u32 cv)
{
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;
	return __rt9466_set_cv(info, cv);
}

static int rt9466_get_ichg(struct charger_device *chg_dev, u32 *ichg)
{
	int ret;
	u8 reg_ichg;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL7);
	if (ret < 0)
		return ret;

	reg_ichg = (ret & RT9466_MASK_ICHG) >> RT9466_SHIFT_ICHG;
	*ichg = rt9466_closest_value(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, reg_ichg);

	return ret;
}

static int rt9466_get_aicr(struct charger_device *chg_dev, u32 *aicr)
{
	int ret;
	u8 reg_aicr;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL3);
	if (ret < 0)
		return ret;

	reg_aicr = (ret & RT9466_MASK_AICR) >> RT9466_SHIFT_AICR;
	*aicr = rt9466_closest_value(RT9466_AICR_MIN, RT9466_AICR_MAX,
		RT9466_AICR_STEP, reg_aicr);

	return ret;
}

static int rt9466_get_cv(struct charger_device *chg_dev, u32 *cv)
{
	int ret;
	u8 reg_cv;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	ret = rt9466_i2c_read_byte(info, RT9466_REG_CHG_CTRL4);
	if (ret < 0)
		return ret;

	reg_cv = (ret & RT9466_MASK_CV) >> RT9466_SHIFT_CV;
	*cv = rt9466_closest_value(RT9466_CV_MIN, RT9466_CV_MAX,
		RT9466_CV_STEP, reg_cv);

	return ret;
}

static int rt9466_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret;
	enum rt9466_charging_status chg_stat = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	ret = rt9466_get_charging_status(info, &chg_stat);

	/* Return is charging done or not */
	switch (chg_stat) {
	case RT9466_CHG_STATUS_READY:
	case RT9466_CHG_STATUS_PROGRESS:
	case RT9466_CHG_STATUS_FAULT:
		*done = false;
		break;
	case RT9466_CHG_STATUS_DONE:
		*done = true;
		break;
	default:
		*done = false;
		break;
	}

	return ret;
}

static int rt9466_kick_wdt(struct charger_device *chg_dev)
{
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	/* Any I2C communication can reset watchdog timer */
	return rt9466_get_charging_status(info, &chg_status);
}

static int __rt9466_enable_auto_sensing(struct rt9466_info *info, bool en)
{
	int ret;
	u8 auto_sense = 0;
	u8 exit_hid = 0x0;

	/* enter hidden mode */
	ret = rt9466_device_write(info->client, RT9466_REG_HIDDEN_MODE,
		ARRAY_SIZE(g_rt9466_val_en_hidden_mode),
		g_rt9466_val_en_hidden_mode);
	if (ret < 0)
		return ret;

	ret = rt9466_device_read(info->client, RT9466_REG_CHG_HIDDEN_CTRL15, 1,
		&auto_sense);
	if (ret < 0) {
		pr_info("read auto sense fail\n");
		goto out;
	}

	if (!en)
		auto_sense &= ~RT9466_AUTO_SENSE_EN; /* clear bit0 */
	else
		auto_sense |= RT9466_AUTO_SENSE_EN; /* set bit0 */
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_HIDDEN_CTRL15, 1,
		&auto_sense);
	if (ret < 0)
		pr_info("auto sense: en = %d fail\n", en);

out:
	return rt9466_device_write(info->client,
			RT9466_REG_HIDDEN_MODE, 1, &exit_hid);
}

/*
 * This function is used in shutdown function
 * Use i2c smbus directly
 */
static int rt9466_sw_reset(struct rt9466_info *info)
{
	int ret;
	u8 evt[RT9466_IRQIDX_MAX] = { 0 };

	/* Register 0x01 ~ 0x10 */
	u8 reg_data[] = {
		0x10, 0x03, 0x23, 0x3C, 0x67, 0x0B, 0x4C, 0xA1,
		0x3C, 0x58, 0x2C, 0x02, 0x52, 0x05, 0x00, 0x10
	};

	pr_info("sw_reset\n");

	/* Disable auto sensing/Enable HZ,ship mode of secondary charger */
	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		mutex_lock(&info->hidden_mode_lock);
		mutex_lock(&info->i2c_access_lock);
		__rt9466_enable_auto_sensing(info, false);
		mutex_unlock(&info->i2c_access_lock);
		mutex_unlock(&info->hidden_mode_lock);

		reg_data[0] = 0x14; /* HZ */
		reg_data[1] = 0x83; /* Shipping mode */
	}

	/* Mask all irq */
	mutex_lock(&info->i2c_access_lock);
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_STATC_CTRL,
		ARRAY_SIZE(g_rt9466_irq_maskall), g_rt9466_irq_maskall);
	if (ret < 0)
		pr_info("sw_reset: mask all irq fail\n");

	/* Read all irq */
	ret = rt9466_device_read(info->client, RT9466_REG_CHG_STATC,
		RT9466_REG_CHG_IRQX_LEN, evt);
	if (ret < 0)
		pr_info("sw_reset: read evt fail\n");

	/* Reset necessary registers */
	ret = rt9466_device_write(info->client, RT9466_REG_CHG_CTRL1,
		ARRAY_SIZE(reg_data), reg_data);
	if (ret < 0)
		pr_info("sw_reset: reset registers fail\n");
	mutex_unlock(&info->i2c_access_lock);

	return ret;
}

static int rt9466_init_setting(struct rt9466_info *info)
{
	int ret;
	struct rt9466_desc *desc = info->desc;
	u8 evt[RT9466_IRQIDX_MAX] = { 0 };

	pr_info("init_setting\n");

	ret = rt9466_maskall_irq(info);
	if (ret < 0) {
		pr_info("init_setting: mask all irq fail\n");
		goto err_init_setting;
	}

	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_STATC, ARRAY_SIZE(evt),
		evt);
	if (ret < 0) {
		pr_info("init_setting: clr evt fail: %d\n", ret);
		goto err_init_setting;
	}

	if (__rt9466_set_ichg(info, desc->ichg) < 0)
		pr_info("init_setting: set ichg fail\n");

	if (__rt9466_set_aicr(info, desc->aicr) < 0)
		pr_info("init_setting: set aicr fail\n");

	if (__rt9466_set_mivr(info, desc->mivr) < 0)
		pr_info("init_setting: set mivr fail\n");

	if (__rt9466_set_cv(info, desc->cv) < 0)
		pr_info("init_setting: set cv fail\n");

	if (__rt9466_set_ieoc(info, desc->ieoc) < 0)
		pr_info("init_setting: set ieoc fail\n");

	if (__rt9466_enable_te(info, desc->en_te) < 0)
		pr_info("init_setting: set te fail\n");

	if (rt9466_set_safety_timer(info, desc->safety_timer) < 0)
		pr_info("init_setting: set fast timer fail\n");

	if (__rt9466_enable_safety_timer(info, true) < 0)
		pr_info("init_setting: enable chg timer fail\n");

	if (rt9466_enable_wdt(info, desc->en_wdt) < 0)
		pr_info("init_setting: set wdt fail\n");

	if (rt9466_enable_jeita(info, desc->en_jeita) < 0)
		pr_info("init_setting: disable jeita fail\n");

	if (rt9466_enable_irq_pulse(info, desc->en_irq_pulse) < 0)
		pr_info("init_setting: set irq pulse fail\n");

	if (rt9466_set_ircmp_resistor(info, desc->ircmp_resistor) < 0)
		pr_info("init_setting: set ircmp resistor fail\n");

	if (rt9466_set_ircmp_vclamp(info, desc->ircmp_vclamp) < 0)
		pr_info("init_setting: set ircmp clamp fail\n");

	if (rt9466_disable_charger_reset(info, true) < 0)
		pr_info("init_setting: disable charger reset fail\n");

	ret = rt9466_sw_workaround(info);
	if (ret < 0) {
		pr_info("init_setting: workaround fail\n");
		return ret;
	}

	if (strcmp(info->desc->chg_dev_name, "secondary_chg") == 0) {
		ret = rt9466_enable_hz(info->chg_dev, true);
		if (ret < 0)
			pr_info("init_setting: hz sec chg fail\n");
	}
err_init_setting:
	return ret;
}

static int rt9466_plug_in(struct charger_device *chg_dev)
{
	pr_info("plug in set enable true\n");
	return rt9466_enable_charging(chg_dev, true);
}

static int rt9466_plug_out(struct charger_device *chg_dev)
{
	pr_info("plug in set enable false\n");
	return rt9466_enable_charging(chg_dev, false);
}

static int rt9466_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);

	if (!info)
		return -EINVAL;

	return __rt9466_is_charging_enable(info, en);
}

static int rt9466_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = rt9466_closest_value(RT9466_ICHG_MIN, RT9466_ICHG_MAX,
		RT9466_ICHG_STEP, 0);

	return 0;
}

static int rt9466_get_register_head(char *reg_head, int size, void *dev_data)
{
	char buff[BUF_LEN] = { 0 };
	int i;
	int len = 0;
	struct rt9466_info *info = dev_data;

	if (!reg_head || !info)
		return -EINVAL;

	memset(reg_head, 0, size);
	for (i = 0; i < ARRAY_SIZE(g_rt9466_reg_addr); i++) {
		snprintf(buff, sizeof(buff), "Reg[0x%02X] ",
			g_rt9466_reg_addr[i]);
		len += strlen(buff);
		if (len < size)
			strncat(reg_head, buff, strlen(buff));
	}

	return 0;
}

static int rt9466_hw_dump_register(char *reg_value, int size, void *dev_data)
{
	int i;
	int ret;
	char buff[BUF_LEN] = { 0 };
	int len = 0;
	struct rt9466_info *info = dev_data;

	if (!reg_value || !info)
		return -EINVAL;

	memset(reg_value, 0, size);
	for (i = 0; i < ARRAY_SIZE(g_rt9466_reg_addr); i++) {
		ret = rt9466_i2c_read_byte(info, g_rt9466_reg_addr[i]);
		if (ret < 0) {
			pr_err("dump_register read Reg[%02X] fail\n", i);
			ret = 0;
		}
		snprintf(buff, sizeof(buff), "0x%-8.2x", (u8)ret);
		len += strlen(buff);
		if (len < size)
			strncat(reg_value, buff, strlen(buff));
	}

	return 0;
}

static int rt9466_dump_register(struct charger_device *chg_dev)
{
	int i;
	int ret;
	u8 chg_stat;
	u32 value = 0;
	bool chg_en = false;
	enum rt9466_charging_status chg_status = RT9466_CHG_STATUS_READY;
	struct rt9466_info *info = i2c_get_clientdata(g_rt9466_i2c);
	u8 chg_ctrl[CHG_CTRL_LEN] = { 0 };

	if (!info)
		return -EINVAL;

	ret = rt9466_get_ichg(chg_dev, &value);
	pr_info("ICHG: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_aicr(chg_dev, &value);
	pr_info("AICR: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_mivr(info, &value);
	pr_info("MIVR: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_is_charging_enable(chg_dev, &chg_en);
	pr_info("CHG_EN: %d, ret: %d\n", chg_en, ret);
	ret = rt9466_get_ieoc(info, &value);
	pr_info("IEOC: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_adc(info, RT9466_ADC_VSYS, &value);
	pr_info("ADC_SYS: %umV, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_adc(info, RT9466_ADC_VBAT, &value);
	pr_info("ADC_VBAT: %umV, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_adc(info, RT9466_ADC_IBAT, &value);
	pr_info("ADC_IBAT: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	ret = rt9466_get_adc(info, RT9466_ADC_IBUS, &value);
	pr_info("ADC_IBUS: %umA, ret: %d\n", value / UNIT_CONVERT, ret);
	chg_stat = rt9466_i2c_read_byte(info, RT9466_REG_CHG_STATC);
	ret = rt9466_i2c_block_read(info, RT9466_REG_CHG_CTRL1,
		CHG_CTRL_LEN, chg_ctrl);
	pr_info("CHG_CTRL1 = 0x%02X, CHG_CTRL2 = 0x%02X, ret: %d\n",
		chg_ctrl[0], chg_ctrl[1], ret);
	ret = rt9466_get_charging_status(info, &chg_status);
	pr_info("CHG_STATUS = %s, CHG_STAT = 0x%02X, ret: %d\n",
		g_rt9466_chg_status_name[chg_status], chg_stat, ret);

	if (chg_status == RT9466_CHG_STATUS_FAULT) {
		for (i = 0; i < ARRAY_SIZE(g_rt9466_reg_addr); i++)
			ret = rt9466_i2c_read_byte(info, g_rt9466_reg_addr[i]);
	}

	return ret;
}

static int rt9466_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static struct charger_ops g_rt9466_chg_ops = {
	.plug_in = rt9466_plug_in,
	.plug_out = rt9466_plug_out,
	.dump_registers = rt9466_dump_register,
	.enable = rt9466_enable_charging,
	.is_enabled = rt9466_is_charging_enable,
	.get_charging_current = rt9466_get_ichg,
	.set_charging_current = rt9466_set_ichg,
	.get_input_current = rt9466_get_aicr,
	.set_input_current = rt9466_set_aicr,
	.get_constant_voltage = rt9466_get_cv,
	.set_constant_voltage = rt9466_set_cv,
	.set_eoc_current = rt9466_set_ieoc,
	.kick_wdt = rt9466_kick_wdt,
	.set_mivr = rt9466_set_mivr,
	.is_charging_done = rt9466_is_charging_done,
	.get_min_charging_current = rt9466_get_min_ichg,
	.enable_otg = rt9466_enable_otg,
	.event = rt9466_do_event,
	.enable_hz = rt9466_enable_hz,
};

static struct power_log_ops rt9466_log_ops = {
	.dev_name = "rt9466",
	.dump_log_head = rt9466_get_register_head,
	.dump_log_content = rt9466_hw_dump_register,
};

void destroy_all_mutex(struct rt9466_info *info)
{
	mutex_destroy(&info->i2c_access_lock);
	mutex_destroy(&info->adc_access_lock);
	mutex_destroy(&info->irq_access_lock);
	mutex_destroy(&info->aicr_access_lock);
	mutex_destroy(&info->ichg_access_lock);
	mutex_destroy(&info->hidden_mode_lock);
	mutex_destroy(&info->pe_access_lock);
	mutex_destroy(&info->ieoc_lock);
}

static void rt9466_init_setting_work_handler(struct work_struct *work)
{
	int ret;
	int retry_cnt = 0;
	struct rt9466_info *info = (struct rt9466_info *)container_of(work,
		struct rt9466_info, init_work);

	do {
		ret = rt9466_select_input_current_limit(info,
			RT9466_IINLMTSEL_AICR);
		if (ret < 0) {
			pr_info("sel ilmtsel fail\n");
			retry_cnt++;
		}
	} while ((retry_cnt < RETRY_MAX_CNT) && (ret < 0));

	msleep(150); /* 150ms */

	retry_cnt = 0;
	do {
		ret = rt9466_enable_ilim(info, false);
		if (ret < 0) {
			pr_info("disable ilim fail\n");
			retry_cnt++;
		}
	} while ((retry_cnt < RETRY_MAX_CNT) && (ret < 0));
}

static void init_info(struct rt9466_info *info, struct i2c_client *client)
{
	mutex_init(&info->i2c_access_lock);
	mutex_init(&info->adc_access_lock);
	mutex_init(&info->irq_access_lock);
	mutex_init(&info->aicr_access_lock);
	mutex_init(&info->ichg_access_lock);
	mutex_init(&info->hidden_mode_lock);
	mutex_init(&info->pe_access_lock);
	mutex_init(&info->ieoc_lock);
	info->client = client;
	info->dev = &client->dev;
	info->aicr_limit = -1;
	info->hidden_mode_cnt = 0;
	info->ieoc_wkard = false;
	info->ieoc = RT9466_IEOC;
	memcpy(info->irq_mask, g_rt9466_irq_maskall, RT9466_IRQIDX_MAX);

	init_waitqueue_head(&info->wait_queue);
	INIT_WORK(&info->init_work, rt9466_init_setting_work_handler);
}

static int irq_register_and_init(struct rt9466_info *info)
{
	int ret;

	ret = rt9466_irq_register(info);
	if (ret < 0) {
		pr_info("probe: irq register fail\n");
		return ret;
	}

	ret = rt9466_irq_init(info);
	if (ret < 0)
		pr_info("probe: irq init fail\n");

	return ret;
}

static int rt9466_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	int ret;
	struct rt9466_info *info = NULL;
	struct power_devices_info_data *pwr_dev_info = NULL;

	pr_info("probe start\n");

	info = devm_kzalloc(&client->dev, sizeof(struct rt9466_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	init_info(info, client);

	if (!rt9466_is_hw_exist(info)) {
		pr_info("probe: no rt9466 exists\n");
		ret = -ENODEV;
		goto err_no_dev;
	}
	i2c_set_clientdata(client, info);
	g_rt9466_i2c = client;
	ret = rt9466_parse_dt(info, &client->dev);
	if (ret < 0) {
		pr_info("probe: parse dt fail\n");
		goto err_parse_dt;
	}

	ret = rt9466_reset_chip(info);
	if (ret < 0) {
		pr_info("probe: reset chip fail\n");
		goto err_reset_chip;
	}

	ret = rt9466_init_setting(info);
	if (ret < 0) {
		pr_info("probe: init setting fail\n");
		goto err_init_setting;
	}

	info->chg_dev = charger_device_register(info->desc->chg_dev_name,
			info->dev, info, &g_rt9466_chg_ops, &info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		ret = PTR_ERR(info->chg_dev);
		goto err_register_chg_dev;
	}

	ret = irq_register_and_init(info);
	if (ret < 0)
		goto err_irq_register;

	gpio_set_value(info->ceb_gpio, 0);
	schedule_work(&info->init_work);

	rt9466_log_ops.dev_data = (void *)info;
	power_log_ops_register(&rt9466_log_ops);

	pwr_dev_info = power_devices_info_register();
	if (pwr_dev_info) {
		pwr_dev_info->dev_name = info->dev->driver->name;
		pwr_dev_info->dev_id = info->vendor_id;
		pwr_dev_info->ver_id = info->chip_rev;
	}
	return ret;

err_irq_register:
	charger_device_unregister(info->chg_dev);
err_register_chg_dev:
err_init_setting:
err_reset_chip:
err_parse_dt:
err_no_dev:
	destroy_all_mutex(info);
	devm_kfree(info->dev, info);
	info = NULL;
	return ret;
}

static int rt9466_remove(struct i2c_client *client)
{
	struct rt9466_info *info = i2c_get_clientdata(client);

	if (info)
		destroy_all_mutex(info);

	return 0;
}

static void rt9466_shutdown(struct i2c_client *client)
{
	int ret;
	struct rt9466_info *info = i2c_get_clientdata(client);

	if (info) {
		ret = rt9466_sw_reset(info);
		if (ret < 0)
			pr_info("shutdown: sw reset fail\n");
	}
}

static int rt9466_suspend(struct device *dev)
{
	struct rt9466_info *info = dev_get_drvdata(dev);

	pr_info("suspend\n");
	if (device_may_wakeup(dev))
		enable_irq_wake(info->irq);

	return 0;
}

static int rt9466_resume(struct device *dev)
{
	struct rt9466_info *info = dev_get_drvdata(dev);

	pr_info("resume\n");
	if (device_may_wakeup(dev))
		disable_irq_wake(info->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rt9466_pm_ops, rt9466_suspend, rt9466_resume);

static const struct i2c_device_id g_rt9466_i2c_id[] = {
	{ "rt9466", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, g_rt9466_i2c_id);

static const struct of_device_id g_rt9466_of_match[] = {
	{ .compatible = "richtek,rt9466", },
	{ .compatible = "richtek,swchg", },
	{},
};
MODULE_DEVICE_TABLE(of, g_rt9466_of_match);

static struct i2c_driver g_rt9466_i2c_driver = {
	.driver = {
		.name = "rt9466",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(g_rt9466_of_match),
		.pm = &rt9466_pm_ops,
	},
	.probe = rt9466_probe,
	.remove = rt9466_remove,
	.shutdown = rt9466_shutdown,
	.id_table = g_rt9466_i2c_id,
};

static int __init rt9466_init(void)
{
	return i2c_add_driver(&g_rt9466_i2c_driver);
}

static void __exit rt9466_exit(void)
{
	i2c_del_driver(&g_rt9466_i2c_driver);
}

module_init(rt9466_init);
module_exit(rt9466_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("rt9466 charger module driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
