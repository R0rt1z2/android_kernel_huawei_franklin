/*
 * scharger_v600.c
 *
 * HI6526 charger driver
 *
 * Copyright (c) 2018-2020 Huawei Technologies Co., Ltd.
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

#include "scharger_v600.h"
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <huawei_platform/power/huawei_charger_common.h>
#include <chipset_common/hwpower/hardware_monitor/water_detect.h>
#include <chipset_common/hwpower/common_module/power_log.h>
#ifdef CONFIG_TCPC_HI6526
#include <huawei_platform/usb/hi6526_tcpc_ops.h>
#endif

#define ILIMIT_RBOOST_CNT 10

struct hi6526_device_info *g_hi6526_dev;
struct hi6526_device_info *g_hi6526_aux_dev;
static const u8 ate_trim_tbl[] = {
	ATE_TRIM_LIST_0, ATE_TRIM_LIST_1, ATE_TRIM_LIST_2, ATE_TRIM_LIST_3,
	ATE_TRIM_LIST_4, ATE_TRIM_LIST_5, ATE_TRIM_LIST_6, ATE_TRIM_LIST_7,
	ATE_TRIM_LIST_8, ATE_TRIM_LIST_9, ATE_TRIM_LIST_10, ATE_TRIM_LIST_11,
	ATE_TRIM_LIST_12, ATE_TRIM_LIST_13, ATE_TRIM_LIST_14, ATE_TRIM_LIST_15,
	ATE_TRIM_LIST_16, ATE_TRIM_LIST_17, ATE_TRIM_LIST_18, ATE_TRIM_LIST_19,
	ATE_TRIM_LIST_20, ATE_TRIM_LIST_21, ATE_TRIM_LIST_22, ATE_TRIM_LIST_23,
	ATE_TRIM_LIST_24, ATE_TRIM_LIST_25, ATE_TRIM_LIST_26, ATE_TRIM_LIST_27,
	ATE_TRIM_LIST_28, ATE_TRIM_LIST_29, ATE_TRIM_LIST_30, ATE_TRIM_LIST_31
};

struct opt_regs common_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0xab, 0xff, 0, 0x63, 0, 0),
	reg_cfg(0x2e2, 0xff, 0, 0x95, 0, 0),
	reg_cfg(0x2e5, 0xff, 0, 0x58, 0, 0),
	reg_cfg(0x293, 0xff, 0, 0xA0, 0, 0),
	reg_cfg(0x284, 0xff, 0, 0x08, 0, 0),
	reg_cfg(0x287, 0xff, 0, 0x1C, 0, 0),
	reg_cfg(0x280, 0xff, 0, 0x0D, 0, 0),
	reg_cfg(0x2ac, 0xff, 0, 0x1e, 0, 0),
	reg_cfg(0x298, 0xff, 0, 0x01, 0, 0),
};

struct opt_regs buck_common_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0xb0, 0xff, 0, 0x01, 0, 0), /* ACR reg reset */
	reg_cfg(0xae, 0xff, 0, 0x10, 0, 0), /* SOH reg reset */
	reg_cfg(0xf1, 0x02, 1, 0x00, 0, 0), /* sc_pulse_mode_en reg reset */
	reg_cfg(0x281, 0xc0, 6, 0x02, 0, 0), /* da_chg_cap3_sel reg reset 0x02 */
	reg_cfg(0x2e9, 0xff, 0, 0x1B, 0, 0),
	reg_cfg(0x2e0, 0xff, 0, 0x3F, 0, 0),
	reg_cfg(0x2f2, 0xff, 0, 0xA6, 0, 0),
	reg_cfg(0x2ed, 0xff, 0, 0x27, 0, 0),
	reg_cfg(0x2d3, 0xff, 0, 0x15, 0, 0),
	reg_cfg(0x2d4, 0xff, 0, 0x97, 0, 0),
	reg_cfg(0x2e1, 0xff, 0, 0x7D, 0, 0),
	reg_cfg(0x2e3, 0xff, 0, 0x22, 0, 0),
	reg_cfg(0x2e4, 0xff, 0, 0x3D, 0, 0),
	reg_cfg(0x2da, 0xff, 0, 0x8C, 0, 0),
	reg_cfg(0x2db, 0xff, 0, 0x94, 0, 0),
	reg_cfg(0x2d6, 0xff, 0, 0x0F, 0, 0),
	reg_cfg(0x2d9, 0xff, 0, 0x11, 0, 0),
	reg_cfg(0x2e7, 0xff, 0, 0x6F, 0, 0),
	reg_cfg(0x2de, 0xff, 0, 0x1E, 0, 0),
	reg_cfg(0x286, 0xff, 0, 0x06, 0, 0),
};

struct opt_regs buck_12v_extra_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0x2e7, 0xff, 0, 0xFF, 0, 0),
};

struct opt_regs otg_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0x2d3, 0xff, 0, 0x08, 0, 0),
	reg_cfg(0x2d4, 0xff, 0, 0x16, 0, 0),
	reg_cfg(0x2e1, 0xff, 0, 0x5A, 0, 0),
	reg_cfg(0x2e3, 0xff, 0, 0x20, 0, 0),
	reg_cfg(0x2e4, 0xff, 0, 0x25, 0, 0),
	reg_cfg(0x2da, 0xff, 0, 0x91, 0, 0),
	reg_cfg(0x2db, 0xff, 0, 0x92, 0, 0),
	reg_cfg(0x2e9, 0xff, 0, 0x1C, 0, 0),
	reg_cfg(0x294, 0xff, 0, 0x01, 0, 0),
	reg_cfg(0x2ee, 0xe7, 0, 0x09, 0, 0),
	reg_cfg(0xf0, 0x0f, 0, 0x06, 0, 0),
	reg_cfg(0x2d9, 0xff, 0, 0x10, 0, 0),
	reg_cfg(0x2de, 0xff, 0, 0X1C, 0, 0),
};

/*
 * chg_limit_v600[CHG_LMT_NUM_V600]: the current limit when setting charger
 *                                   value, unit: mA
 * These arrays are set to replace the tedious "else if" statements,instead,
 * a concise "for" statement is used in this version, the items transferred from
 * the original Macro definition like #define CHG_ILIMIT_1100           1100 in
 * the arrays are the values to be given to the variables in the "for" loop
 */
const int chg_limit_v600[CHG_LMT_NUM_V600] = {
	130,  200,  300,  400,  475,  600,
	700,  800,  825,  1000, 1100, 1200,
	1300, 1400, 1500, 1600, 1700,
	1800, 1900, 2000, 2100, 2200,
	2300, 2400, 2500, 2600, 2700,
	2800, 2900, 3000, 3100
};

#ifndef CONFIG_DIRECT_CHARGER
static int dummy_ops_register2(struct direct_charge_ic_ops *ops)
{
	return 0;
}

static int dummy_ops_register3(struct direct_charge_batinfo_ops *ops)
{
	return 0;
}
#endif

struct hi6526_device_info *get_hi6526_device(void)
{
	return g_hi6526_dev;
}

struct hi6526_device_info *get_hi6526_aux_device(void)
{
	return g_hi6526_aux_dev;
}

void set_boot_weaksource_flag(void)
{

}

static void clr_boot_weaksource_flag(void)
{

}

int hi6526_set_watchdog_timer_execute(struct hi6526_device_info *di, int value);

#define CONFIG_SYSFS_SCHG
#ifdef CONFIG_SYSFS_SCHG
/*
 * There are a numerous options that are configurable on the HI6526
 * that go well beyond what the power_supply properties provide access to.
 * Provide sysfs access to them so they can be examined and possibly modified
 * on the fly.
 */

#define hi6526_sysfs_field(_name, r, f, m, store) \
{							\
	.attr = __ATTR(_name, m, hi6526_sysfs_show, store), \
	.reg = CHG_##r##_REG, .mask = CHG_##f##_MSK, .shift = CHG_##f##_SHIFT, \
}

#define hi6526_sysfs_field_rw(_name, r, f) \
	hi6526_sysfs_field(_name, r, f, 0640, hi6526_sysfs_store)

static ssize_t hi6526_sysfs_show(struct device *dev,
				 struct device_attribute *attr, char *buf);

static ssize_t hi6526_sysfs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count);

struct hi6526_sysfs_field_info {
	struct device_attribute attr;
	u16 reg;
	u8 mask;
	u8 shift;
};

static struct hi6526_sysfs_field_info hi6526_sysfs_field_tbl[] = {
	/* sysfs name reg field in reg */
	hi6526_sysfs_field_rw(en_hiz, HIZ_CTRL, HIZ_ENABLE),
	hi6526_sysfs_field_rw(iinlim, INPUT_SOURCE, ILIMIT),
	hi6526_sysfs_field_rw(reg_addr, NONE, NONE),
	hi6526_sysfs_field_rw(reg_value, NONE, NONE),
	hi6526_sysfs_field_rw(adapter_reg, NONE, NONE),
	hi6526_sysfs_field_rw(adapter_val, NONE, NONE),
};

static struct attribute *hi6526_sysfs_attrs[ARRAY_SIZE(hi6526_sysfs_field_tbl) + 1];

static const struct attribute_group hi6526_sysfs_attr_group = {
	.attrs = hi6526_sysfs_attrs,
};

static void hi6526_sysfs_init_attrs(void)
{
	int i;
	int limit = ARRAY_SIZE(hi6526_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		hi6526_sysfs_attrs[i] = &hi6526_sysfs_field_tbl[i].attr.attr;

	hi6526_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

/* get the current device_attribute from hi6526_sysfs_field_tbl by attr's name */
static struct hi6526_sysfs_field_info *hi6526_sysfs_field_lookup(const char *name)
{
	int i;
	int limit = ARRAY_SIZE(hi6526_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		if (!strncmp(name, hi6526_sysfs_field_tbl[i].attr.attr.name,
			     strlen(name)))
			break;

	if (i >= limit)
		return NULL;

	return &hi6526_sysfs_field_tbl[i];
}

/* 0-success or others-fail */
static ssize_t hi6526_sysfs_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct hi6526_sysfs_field_info *info = NULL;
	struct hi6526_sysfs_field_info *info2 = NULL;
#ifdef CONFIG_SCHARGER_V600_DEBUG
	int ret;
#endif
	u8 v = 0;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	info = hi6526_sysfs_field_lookup(attr->attr.name);
	if (info == NULL)
		return -EINVAL;

	if (!strncmp("reg_addr", attr->attr.name, strlen("reg_addr")))
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", info->reg);

	if (!strncmp(("reg_value"), attr->attr.name, strlen("reg_value"))) {
		info2 = hi6526_sysfs_field_lookup("reg_addr");
		if (info2 == NULL)
			return -EINVAL;
		info->reg = info2->reg;
	}

	if (!strncmp("adapter_reg", attr->attr.name, strlen("adapter_reg")))
		return scnprintf(buf, PAGE_SIZE, "0x%x\n",
				 di->sysfs_fcp_reg_addr);

	if (!strncmp("adapter_val", attr->attr.name, strlen("adapter_val"))) {
#ifdef CONFIG_SCHARGER_V600_DEBUG
		ret = hi6526_fcp_adapter_reg_read(&v, di->sysfs_fcp_reg_addr);
		scharger_inf("sys read fcp adapter reg 0x%x, v 0x%x\n",
			      di->sysfs_fcp_reg_addr, v);
		if (ret)
			return ret;
#endif
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", v);
	}

#ifdef CONFIG_SCHARGER_V600_DEBUG
	ret = hi6526_read_mask(info->reg, info->mask, info->shift, &v);
	scharger_inf("sys read reg 0x%x, v 0x%x\n", info->reg, v);
	if (ret)
		return ret;
#endif
	return scnprintf(buf, PAGE_SIZE, "0x%hhx\n", v);
}

/*
 * set the value for all HI6526 device's node
 * return value: 0-success or others-fail
 */
static ssize_t hi6526_sysfs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct hi6526_sysfs_field_info *info = NULL;
	struct hi6526_sysfs_field_info *info2 = NULL;
	int ret;
	u16 v;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -1;
	}

	info = hi6526_sysfs_field_lookup(attr->attr.name);
	if (info == NULL)
		return -EINVAL;

	ret = kstrtou16(buf, 0, &v);
	if (ret < 0)
		return ret;

	if (!strncmp(("reg_value"), attr->attr.name, strlen("reg_value"))) {
		info2 = hi6526_sysfs_field_lookup("reg_addr");
		if (info2 == NULL)
			return -EINVAL;
		info->reg = info2->reg;
	}
	if (!strncmp(("reg_addr"), attr->attr.name, strlen("reg_addr"))) {
		if (v < HI6526_REG_MAX) {
			info->reg = v;
			return count;
		} else {
			return -EINVAL;
		}
	}

	if (!strncmp(("adapter_reg"), attr->attr.name, strlen("adapter_reg"))) {
		di->sysfs_fcp_reg_addr = (u8)v;
		return count;
	}
	if (!strncmp(("adapter_val"), attr->attr.name, strlen("adapter_val"))) {
		di->sysfs_fcp_reg_val = (u8)v;
#ifdef CONFIG_SCHARGER_V600_DEBUG
		ret = hi6526_fcp_adapter_reg_write(di->sysfs_fcp_reg_val,
						   di->sysfs_fcp_reg_addr);
		scharger_inf("sys write fcp adapter reg 0x%x, v 0x%x\n",
			      di->sysfs_fcp_reg_addr, di->sysfs_fcp_reg_val);

		if (ret)
			return ret;
#endif
		return count;
	}

#ifdef CONFIG_SCHARGER_V600_DEBUG
	ret = hi6526_write_mask(di, info->reg, info->mask, info->shift, (u8)v);
	if (ret)
		return ret;
#endif

	return count;
}

static int hi6526_sysfs_create_group(struct hi6526_device_info *di)
{
	hi6526_sysfs_init_attrs();
	if (!power_sysfs_create_link_group("hw_power", "charger", "HI6526",
					    di->dev, &hi6526_sysfs_attr_group))
		return -1;

	return 0;
}

static void hi6526_sysfs_remove_group(struct hi6526_device_info *di)
{
	(void)power_sysfs_remove_link_group("hw_power", "charger", "HI6526",
					     di->dev, &hi6526_sysfs_attr_group);
}
#else

static int hi6526_sysfs_create_group(struct hi6526_device_info *di)
{
	return 0;
}

static inline void hi6526_sysfs_remove_group(struct hi6526_device_info *di)
{
}
#endif

void hi6526_opt_regs_set(struct hi6526_device_info *di, struct opt_regs *opt, unsigned int len)
{
	unsigned int i;

	if (!opt || !di) {
		scharger_err("%s is NULL!\n", __func__);
		return;
	}
	for (i = 0; i < len; i++) {
		(void)mdelay(opt[i].before);
		(void)hi6526_write_mask(di, opt[i].reg, opt[i].mask,
			opt[i].shift, opt[i].val);
		(void)mdelay(opt[i].after);
	}
}

/* 0-success or others-fail */
static int hi6526_device_check_exec(struct hi6526_device_info *di)
{
	int ret;
	u32 chip_id = 0;
	/* byte length of version 0 chip id is 4 */
	ret = hi6526_read_block(di, CHIP_VERSION_0, (u8 *)(&chip_id), 4);
	if (ret) {
		scharger_err("[%s]:read chip_id fail\n", __func__);
		return CHARGE_IC_BAD;
	}

	scharger_inf("%s, chip id is 0x%x\n", __func__, chip_id);
	return CHARGE_IC_GOOD;
}

/* 0-success or others-fail */
static int hi6526_device_check(struct charger_device *dev)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -1;
	}
	return hi6526_device_check_exec(di);
}

static unsigned int hi6526_get_device_version(struct hi6526_device_info *di)
{
	unsigned int hi6526_version = 0;
	int ret;
	u32 chip_id = 0;
	/* byte length of version 0 chip id is 4 */
	ret = hi6526_read_block(di, CHIP_VERSION_0, (u8 *)(&chip_id), 4);
	if (ret) {
		scharger_err("[%s]:read chip_id fail\n", __func__);
		return CHARGE_IC_BAD;
	}

	if (chip_id == CHIP_ID_6526_V100) {
		/* byte length of version 0 is 2 */
		(void)hi6526_read_block(di, CHIP_VERSION_0,
			(u8 *)(&hi6526_version), 2);
		scharger_inf("%s, chip id is hi6526 v100 [0x%x, 0x%x]\n",
			      __func__, chip_id, hi6526_version);
	} else if (chip_id == CHIP_ID_6526) {
		/* byte length of version 0 is 2 */
		(void)hi6526_read_block(di, CHIP_VERSION_4,
			(u8 *)(&hi6526_version), 2);
		scharger_inf("%s, [0x%x, 0x%x]\n", __func__,
			      chip_id, hi6526_version);
	} else {
		scharger_err("%s, ERROR [0x%x, 0x%x]\n", __func__,
			      chip_id, hi6526_version);
	}

	scharger_inf("%s, version is 0x%x\n", __func__, hi6526_version);
	return hi6526_version;
}

void hi6526_set_anti_reverbst_reset(struct hi6526_device_info *di)
{
	if (di == NULL)
		return;

	(void)hi6526_write_mask(di, CHG_ANTI_REVERBST_REG,
		CHG_ANTI_REVERBST_EN_MSK,
		CHG_ANTI_REVERBST_EN_SHIFT,
		CHG_ANTI_REVERBST_DIS);
	queue_delayed_work(system_power_efficient_wq, &di->reverbst_work,
			   msecs_to_jiffies(REVERBST_DELAY_ON));
}

int hi6526_get_vbus_uvp_state(struct hi6526_device_info *di)
{
	u8 val = 0;
	int ret;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	ret = hi6526_read(di, CHG_IRQ_STATUS0, &val);
	scharger_dbg("%s:0x%x\n", __func__, val);
	if (ret) {
		scharger_err("%s read failed:%d\n", __func__, ret);
		return ret;
	}

	return (int)(!!(val & CHG_VBUS_UVP));
}

int hi6526_set_buck_enable(int enable, void *dev_data)
{
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	scharger_inf("%s:%d\n", __func__, enable);
	if (enable)
		hi6526_set_anti_reverbst_reset(di);

	(void)hi6526_write_mask(di, CHG_BUCK_MODE_CFG, CHG_BUCK_MODE_CFG_MASK,
		CHG_BUCK_MODE_CFG_SHIFT, !!enable);
	if (enable) {
		di->batt_ovp_cnt_30s = 0;
		di->chg_mode = BUCK;
	} else if (di->chg_mode == BUCK) {
		di->chg_mode = NONE;
	}
	(void)hi6526_write_mask(di, CHG_HIZ_CTRL_REG, CHG_HIZ_ENABLE_MSK,
		CHG_HIZ_ENABLE_SHIFT, !enable);
#ifndef CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT
	(void)hi6526_set_batfet_ctrl(di, 0);
#endif
	(void)hi6526_set_watchdog_timer_execute(di, WATCHDOG_TIMER_40_S);
	(void)hi6526_reset_watchdog_timer_execute(di);
	return 0;
}

/*
 * set the bat comp, schargerv100 can't set ir comp due to lx bug
 * return value: 0-success or others-fail
 */
int hi6526_set_bat_comp(int value)
{
	u8 reg;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (value < CHG_IR_COMP_MIN)
		value = CHG_IR_COMP_MIN;
	else if (value > CHG_IR_COMP_MAX)
		value = CHG_IR_COMP_MAX;

	reg = value / CHG_IR_COMP_STEP_15;

	return hi6526_write_mask(di, CHG_IR_COMP_REG, CHG_IR_COMP_MSK,
		CHG_IR_COMP_SHIFT, reg);
}

/*
 * Parameters: value:vclamp mv, schargerv100 can't set vclamp due to lx bug
 * return value:  0-success or others-fail
 */
int hi6526_set_vclamp(int value)
{
	u8 reg;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (value < CHG_IR_VCLAMP_MIN)
		value = CHG_IR_VCLAMP_MIN;
	else if (value > CHG_IR_VCLAMP_MAX)
		value = CHG_IR_VCLAMP_MAX;

	reg = value / CHG_IR_VCLAMP_STEP;
	return hi6526_write_mask(di, CHG_IR_VCLAMP_REG, CHG_IR_VCLAMP_MSK,
		CHG_IR_VCLAMP_SHIFT, reg);
}

int hi6526_set_fast_safe_timer(u32 chg_fastchg_safe_timer)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	return hi6526_write_mask(di, CHG_FASTCHG_TIMER_REG,
		CHG_FASTCHG_TIMER_MSK,
		CHG_FASTCHG_TIMER_SHIFT,
		(u8)chg_fastchg_safe_timer);
}

int hi6526_set_recharge_vol(u8 rechg)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	return hi6526_write_mask(di, CHG_RECHG_REG, CHG_RECHG_MSK,
		CHG_RECHG_SHIFT, rechg);
}

int hi6526_set_precharge_current(int precharge_current)
{
	u8 prechg_limit;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (precharge_current < CHG_PRG_ICHG_MIN)
		precharge_current = CHG_PRG_ICHG_MIN;
	else if (precharge_current > CHG_PRG_ICHG_MAX)
		precharge_current = CHG_PRG_ICHG_MAX;

	prechg_limit =
		(u8)((precharge_current - CHG_PRG_ICHG_MIN) / CHG_PRG_ICHG_STEP);

	return hi6526_write_mask(di, CHG_PRE_ICHG_REG, CHG_PRE_ICHG_MSK,
		CHG_PRE_ICHG_SHIFT, prechg_limit);
}

int hi6526_set_precharge_voltage(u32 pre_vchg)
{
	u8 vprechg;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (pre_vchg <= CHG_PRG_VCHG_2800)
		vprechg = 0; /* 0: set precharge vol <=2800mV */
	else if (pre_vchg > CHG_PRG_VCHG_2800 && pre_vchg <= CHG_PRG_VCHG_3000)
		vprechg = 1; /* 1: set precharge vol <=3000mV  and >2800mV */
	else if (pre_vchg > CHG_PRG_VCHG_3000 && pre_vchg <= CHG_PRG_VCHG_3100)
		vprechg = 2; /* 2: set precharge vol <=3100mV  and >3000mV */
	else if (pre_vchg > CHG_PRG_VCHG_3100 && pre_vchg <= CHG_PRG_VCHG_3200)
		vprechg = 3; /* 3: set precharge vol <=3200mV  and >3100mV */
	else
		vprechg = 0; /* default 2.8V */
	return hi6526_write_mask(di, CHG_PRE_VCHG_REG, CHG_PRE_VCHG_MSK,
		CHG_PRE_VCHG_SHIFT, vprechg);
}

int hi6526_set_batfet_ctrl(struct hi6526_device_info *di, u32 status)
{
	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	return hi6526_write_mask(di, BATFET_CTRL_CFG_REG, BATFET_CTRL_CFG_MSK,
		BATFET_CTRL_CFG_SHIFT, status);
}

static bool hi6526_get_charge_enable(struct hi6526_device_info *di)
{
	u8 charge_state = 0;

	(void)hi6526_read_mask(di, CHG_ENABLE_REG, CHG_EN_MSK,
		CHG_EN_SHIFT, &charge_state);

	if (charge_state)
		return TRUE;
	else
		return FALSE;
}

int hi6526_set_vbus_ovp(struct hi6526_device_info *di, int vbus)
{
	int ret;
	u8 ov_vol;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (vbus < VBUS_VSET_9V)
		ov_vol = 0; /* 0: set ovp to 9V */
	else if (vbus < VBUS_VSET_12V)
		ov_vol = 1; /* 1: set ovp to 12V */
	else
		ov_vol = 2; /* 2: set ovp to default value */

	ret = hi6526_write_mask(di, CHG_OVP_VOLTAGE_REG, CHG_BUCK_OVP_VOLTAGE_MSK,
		CHG_BUCK_OVP_VOLTAGE_SHIFT, ov_vol);

	return ret;
}

int hi6526_set_vbus_uvp_ovp(struct hi6526_device_info *di, int vbus)
{
	int ret0, ret1;
	const u8 uv_vol = 0; /* 0: representing 3.8V */

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret0 = hi6526_set_vbus_ovp(di, vbus);
	ret1 = hi6526_write_mask(di, CHG_UVP_VOLTAGE_REG, CHG_BUCK_UVP_VOLTAGE_MSK,
		CHG_BUCK_UVP_VOLTAGE_SHIFT, uv_vol);
	if (ret0 || ret1) {
		scharger_err("%s:uvp&ovp voltage set failed\n", __func__);
		return -1;
	}
	return 0;
}

int hi6526_set_charge_enable_execute(struct hi6526_device_info *di, bool enable)
{
	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (enable && !di->chg_last_enable)
		hi6526_set_input_current_limit(0);
	else if (!enable && di->chg_last_enable)
		hi6526_set_input_current_limit(1);

	di->chg_last_enable = enable;
	return hi6526_write_mask(di, CHG_ENABLE_REG, CHG_EN_MSK,
		CHG_EN_SHIFT, enable);
}

int hi6526_set_vbus_vset_exec(struct hi6526_device_info *di, u32 value)
{
	u8 data;
	u32 charger_flag = 0;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	scharger_inf("%s:%u v\n", __func__, value);

	/* check charge state, if open, close charge */
	if (hi6526_get_charge_enable(di) == TRUE) {
		hi6526_set_charge_enable_execute(di, CHG_DISABLE);
		charger_flag = 1;
	}
	if (value < VBUS_VSET_9V) {
		di->charger_is_fcp = FCP_FALSE;
		data = 0; /* 0: set vbus to 9V */
	} else if (value < VBUS_VSET_12V) {
		di->charger_is_fcp = FCP_TRUE;
		data = 1; /* 1: set vbus to 12V */
	} else {
		data = 2; /* 2: set vbus to default value */
	}
	di->buck_vbus_set = value;

	/* resume charge state */
	if (charger_flag == 1)
		hi6526_set_charge_enable_execute(di, CHG_ENABLE);

	hi6526_set_vbus_uvp_ovp(di, value);

	return hi6526_write_mask(di, CHG_VBUS_VSET_REG, VBUS_VSET_MSK,
		VBUS_VSET_SHIFT, data);
}

/* Parameters: vbus_set voltage: 5V/9V/12V */
int hi6526_set_vbus_vset(struct charger_device *dev, u32 value)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	return hi6526_set_vbus_vset_exec(di, value);
}

/* Parameters: vbus_vol:5V/9V/12V */
static void hi6526_buck_opt_param(struct hi6526_device_info *di, int vbus_vol)
{
	int ret;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return;
	}

	hi6526_opt_regs_set(di, buck_common_opt_regs,
		ARRAY_SIZE(buck_common_opt_regs));
	switch (vbus_vol) {
	case VBUS_VSET_5V:
		break;
	case VBUS_VSET_9V:
		break;
	case VBUS_VSET_12V:
		hi6526_opt_regs_set(di, buck_12v_extra_opt_regs,
			ARRAY_SIZE(buck_12v_extra_opt_regs));
		break;
	default:
		break;
	}

	if (di->param_dts.r_coul_mohm == R_MOHM_10) {
		ret = hi6526_write_mask(di, IBAT_RES_SEL_REG,
			IBAT_OCP_TH_MASK,
			IBAT_OCP_TH_SHIFT,
			IBAT_OCP_TH_REG_MAX);
		if (ret)
			scharger_err("%s refit ibat ocp limit fail\n", __func__);
	}
}

int hi6526_config_opt_param(struct hi6526_device_info *di, int vbus_vol)
{
	int ret;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	hi6526_buck_opt_param(di, vbus_vol);
	ret = hi6526_set_vbus_vset_exec(di, vbus_vol);
	if (ret != 0)
		scharger_err("%s hi6526_set_vbus_vset fail\n", __func__);
	return 0;
}

int hi6526_set_input_current(struct charger_device *dev, u32 cin_limit)
{
	u8 i;
	int max;
	u8 i_in_limit = 0;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (di->buck_vbus_set < VBUS_VSET_9V)
		max = CHG_ILIMIT_2500;
	else if (di->buck_vbus_set < VBUS_VSET_12V)
		max = CHG_ILIMIT_1400;
	else
		max = CHG_ILIMIT_1200;

	if (di->input_limit_flag) {
		if (di->buck_vbus_set < VBUS_VSET_9V)
			max = CHG_ILIMIT_1100;
		else if (di->buck_vbus_set < VBUS_VSET_12V)
			max = CHG_ILIMIT_600;
		else
			max = CHG_ILIMIT_475;
	}

	di->input_current = cin_limit;

	if (cin_limit > max) {
		scharger_dbg("%s cin_limit %d, max %d, vbus set is %d\n",
			      __func__, cin_limit, max, di->buck_vbus_set);
		cin_limit = max;
	}

	if (cin_limit < CHG_ILIMIT_130)
		i_in_limit = 0;
	else if (cin_limit >= CHG_ILIMIT_1000)
		i_in_limit = cin_limit / CHG_ILIMIT_STEP_100;
	/*
	 * This "for" statement takes place of the previous tedious
	 * "else if" statements. the items in the array chg_lmt_set_v300
	 * are the Macro defined values, like #define CHG_ILIMIT_3200 3200
	 * when cin_limit is in range of:
	 * [CHG_ILIMIT_130, CHG_ILIMIT_200), i_in_limit = 1;
	 * [CHG_ILIMIT_200, CHG_ILIMIT_300), i_in_limit = 2;
	 * [CHG_ILIMIT_300, CHG_ILIMIT_400), i_in_limit = 3;
	 * [CHG_ILIMIT_400, CHG_ILIMIT_475), i_in_limit = 4;
	 * [CHG_ILIMIT_475, CHG_ILIMIT_600), i_in_limit = 5;
	 * [CHG_ILIMIT_600, CHG_ILIMIT_700), i_in_limit = 6;
	 * [CHG_ILIMIT_700, CHG_ILIMIT_800), i_in_limit = 7;
	 * [CHG_ILIMIT_800, CHG_ILIMIT_825), i_in_limit = 8;
	 * [CHG_ILIMIT_825, CHG_ILIMIT_1000), i_in_limit = 9;
	 * 10: number of gears
	 */
	for (i = 1; i < 10; i++)
		if (cin_limit >= chg_limit_v600[i - 1] &&
			cin_limit < chg_limit_v600[i])
			i_in_limit = i;
	scharger_dbg("%s:cin_limit %d ma, reg is set 0x%x\n", __func__,
		      cin_limit, i_in_limit);
	scharger_dbg("%s:flag %d, buck_vbus_set %d\n", __func__,
		      di->input_limit_flag, di->buck_vbus_set);
	di->iin_set = cin_limit;
	return hi6526_write_mask(di, CHG_INPUT_SOURCE_REG, CHG_ILIMIT_MSK,
		CHG_ILIMIT_SHIFT, i_in_limit);
}

void hi6526_set_input_current_limit(int enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;
	struct charger_device *dev = NULL;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return;
	}
	scharger_inf("%s, flag %d, input current %d, vbus vset %d, enable %d\n",
		      __func__, di->input_limit_flag, di->input_current,
		      di->buck_vbus_set, enable);

	di->input_limit_flag = enable;

	if (enable) {
		if (di->buck_vbus_set < VBUS_VSET_9V) {
			if (di->input_current > CHG_ILIMIT_1100)
				(void)hi6526_set_input_current(dev, CHG_ILIMIT_1100);
		} else if (di->buck_vbus_set < VBUS_VSET_12V) {
			if (di->input_current > CHG_ILIMIT_600)
				(void)hi6526_set_input_current(dev, CHG_ILIMIT_600);
		} else {
			if (di->input_current > CHG_ILIMIT_475)
				(void)hi6526_set_input_current(dev, CHG_ILIMIT_475);
		}
	} else {
		(void)hi6526_set_input_current(dev, di->input_current);
	}
}

static int hi6526_get_input_current_set(struct charger_device *dev)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	return di->iin_set;
}

static int hi6526_get_charge_current(struct charger_device *dev, u32 *ichg)
{
	int ret;
	u8 reg = 0;
	int charge_current;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di || !ichg) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = hi6526_read_mask(di, CHG_FAST_CURRENT_REG, CHG_FAST_ICHG_MSK,
		CHG_FAST_ICHG_SHIFT, &reg);
	if (ret) {
		scharger_err("HI6526 read mask fail\n");
		return CHG_FAST_ICHG_00MA;
	}

	charge_current = (reg + 1) * CHG_FAST_ICHG_STEP_100;
	*ichg = charge_current;

	scharger_dbg("charge current is set %d %u\n", charge_current, reg);
	return charge_current;
}

static int hi6526_calc_charge_current_bias(int charge_current)
{
	int i_coul_ma, i_delta_ma, i_bias;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	mutex_lock(&di->ibias_calc_lock);
	/*
	 * if target is less than 1A,need not to calc ibias,need not to minus
	 * bias calculated before
	 */
	if (charge_current < CHG_FAST_ICHG_1000MA) {
		di->last_ichg = charge_current;
		mutex_unlock(&di->ibias_calc_lock);
		return charge_current;
	}
	/*
	 * if target charge current changed, no need to calc ibias,just use bias
	 * calculated before
	 */
	if (di->last_ichg != charge_current) {
		di->last_ichg = charge_current;
		charge_current -= di->i_bias_all;
		mutex_unlock(&di->ibias_calc_lock);
		return charge_current;
	}

	/*
	 * calculate bias with difference between I_coul and last target charge
	 * current
	 */
	i_coul_ma = battery_get_bat_current();
	/*
	 * current from battery_current is negative while charging,change
	 * to positive to calc with charge_current
	 */
	i_coul_ma = -i_coul_ma;
	i_delta_ma = i_coul_ma - charge_current;
	/*
	 * if I_coul is less than last target charge current for more than
	 * 100ma, bias should minus 100ma
	 */
	if (i_delta_ma < -CHG_FAST_ICHG_100MA)
		i_bias = -CHG_FAST_ICHG_100MA;
	/*
	 * if difference between I_coul and last target charge current is less
	 * than 100ma, no need to add bias
	 */
	else if (i_delta_ma < CHG_FAST_ICHG_100MA)
		i_bias = 0;
	else
		/*
		 * if difference between I_coul and last target charge current
		 * is more than 100ma, calc bias with 100 rounding down
		 */
		i_bias = (i_delta_ma / 100) * 100;

	/* update i_bias_all within [0,400] ma */
	di->i_bias_all += i_bias;
	if (di->i_bias_all <= 0)
		di->i_bias_all = 0;
	if (di->i_bias_all >= CHG_FAST_ICHG_400MA)
		di->i_bias_all = CHG_FAST_ICHG_400MA;

	/* update charge current */
	charge_current -= di->i_bias_all;
	scharger_inf("%s:Ichg:%d, i_coul_ma:%d,Ibias:%d,i_bias_all:%d\n",
		__func__, charge_current,
		i_coul_ma, i_bias, di->i_bias_all);
	mutex_unlock(&di->ibias_calc_lock);
	return charge_current;
}

int hi6526_set_charge_current(struct charger_device *dev, u32 charge_current)
{
	u8 i_chg_limit;
	/* Chip limit */
	int max_curr = CHG_FAST_ICHG_2500MA;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	charge_current = hi6526_calc_charge_current_bias(charge_current);

	/* just for wireless 12V charger */
	if ((di->charger_type == CHARGER_TYPE_WIRELESS) &&
		(di->buck_vbus_set == VBUS_VSET_12V)) {
		max_curr = CHG_FAST_ICHG_2800MA;
		scharger_inf("%s max charger current 2.8A\n", __func__);
	}

	if (charge_current > max_curr)
		charge_current = max_curr;

	i_chg_limit = (charge_current / CHG_FAST_ICHG_STEP_100) - 1;

	scharger_dbg("%s:%d, reg is set 0x%x\n", __func__, charge_current,
		      i_chg_limit);
	return hi6526_write_mask(di, CHG_FAST_CURRENT_REG, CHG_FAST_ICHG_MSK,
		CHG_FAST_ICHG_SHIFT, i_chg_limit);
}

/*
 * set the terminal voltage in charging process
 * (v300&v310 scharger's max cv is 4.25V due to lx bug)
 */
int hi6526_set_terminal_voltage(struct charger_device *dev, u32 charge_voltage)
{
	u8 data;
	struct hi6526_device_info *di = g_hi6526_dev;

	scharger_dbg("%s charge_voltage:%d\n", __func__, charge_voltage);

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (charge_voltage < CHG_FAST_VCHG_MIN)
		charge_voltage = CHG_FAST_VCHG_MIN;
	else if (charge_voltage > CHG_FAST_VCHG_MAX)
		charge_voltage = CHG_FAST_VCHG_MAX;

	di->term_vol_mv = charge_voltage;
	/* transfer from code value to actual value */
	data = (u8)((charge_voltage - CHG_FAST_VCHG_MIN) * 1000 /
		    CHG_FAST_VCHG_STEP_16600UV);
	return hi6526_write_mask(di, CHG_FAST_VCHG_REG, CHG_FAST_VCHG_MSK,
		CHG_FAST_VCHG_SHIFT, data);
}

static int hi6526_get_terminal_voltage(struct charger_device *dev)
{
	u8 data = 0;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	(void)hi6526_read_mask(di, CHG_FAST_VCHG_REG, CHG_FAST_VCHG_MSK,
		CHG_FAST_VCHG_SHIFT, &data);
	/* transfer from code value to actual value */
	return (int)((data * CHG_FAST_VCHG_STEP_16600UV +
		     CHG_FAST_VCHG_BASE_UV) / 1000);
}

/*
 * Description: check whether VINDPM or IINDPML
 * return value: TRUE means VINDPM or IINDPM
 *               FALSE means NoT DPM
 */
static int hi6526_check_input_dpm_state(struct charger_device *dev)
{
	u8 reg = 0;
	int ret;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = hi6526_read(di, CHG_R0_REG_STATUE, &reg);
	if (ret < 0) {
		scharger_err("%s err\n", __func__);
		return FALSE;
	}

	if ((reg & CHG_IN_DPM_STATE) == CHG_IN_DPM_STATE) {
		scharger_inf("CHG_STATUS0_REG:%x in vdpm state\n", reg);
		return TRUE;
	}

	return FALSE;
}

/* return value: TRUE means acl, FALSE means NoT acl */
static int hi6526_check_therm_state(void)
{
	u8 reg = 0;
	int ret;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = hi6526_read(di, CHG_R0_REG2_STATUE, &reg);
	if (ret < 0) {
		scharger_err("hi6526_check_input_dpm_state err\n");
		return FALSE;
	}

	if ((reg & CHG_IN_THERM_STATE) == CHG_IN_THERM_STATE)
		return TRUE;
	return FALSE;
}

/* return value: TRUE means acl, FALSE means NoT acl */
static int hi6526_check_input_acl_state(struct charger_device *dev)
{
	u8 reg = 0;
	int ret0, ret1, ret2;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret0 = hi6526_write_mask(di, CHG_ACL_RPT_EN_REG, CHG_ACL_PRT_EN_MASK,
		CHG_ACL_RPT_EN_SHIFT, true);
	ret1 = hi6526_read(di, CHG_R0_REG_STATUE, &reg);
	ret2 = hi6526_write_mask(di, CHG_ACL_RPT_EN_REG, CHG_ACL_PRT_EN_MASK,
		CHG_ACL_RPT_EN_SHIFT, true);
	if (ret0 || ret1 || ret2) {
		scharger_err("%s err\n", __func__);
		return FALSE;
	}

	if ((reg & CHG_IN_ACL_STATE) == CHG_IN_ACL_STATE) {
		scharger_inf("CHG_STATUS0_REG:%x in acl state\n", reg);
		return TRUE;
	}

	return FALSE;
}

static int hi6526_get_charge_state(struct charger_device *dev, unsigned int *state)
{
	u8 reg0 = 0;
	u8 reg1 = 0;
	u8 reg2 = 0;
	int ret0, ret1, ret2;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	*state = 0;
	ret0 = hi6526_read(di, CHG_BUCK_STATUS_REG, &reg0);
	ret1 = hi6526_read(di, CHG_STATUS_REG, &reg1);
	ret2 = hi6526_read(di, WATCHDOG_STATUS_REG, &reg2);
	if (ret0 || ret1 || ret2) {
		scharger_err("[%s]read charge status reg fail\n", __func__);
		return -1;
	}

	if ((reg0 & HI6526_CHG_BUCK_OK) != HI6526_CHG_BUCK_OK)
		*state |= CHAGRE_STATE_NOT_PG;
	if ((reg1 & HI6526_CHG_STAT_CHARGE_DONE) == HI6526_CHG_STAT_CHARGE_DONE)
		*state |= CHAGRE_STATE_CHRG_DONE;
	if ((reg2 & HI6526_WATCHDOG_OK) != HI6526_WATCHDOG_OK)
		*state |= CHAGRE_STATE_WDT_FAULT;

	scharger_inf("%s >>> reg0:0x%x, reg1 0x%x, reg2 0x%x, state 0x%x\n",
		     __func__, reg0, reg1, reg2, *state);
	return 0;
}

static void hi6526_reverbst_delay_work(struct work_struct *work)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL)
		return;

	(void)hi6526_write_mask(di, CHG_ANTI_REVERBST_REG,
		CHG_ANTI_REVERBST_EN_MSK,
		CHG_ANTI_REVERBST_EN_SHIFT,
		CHG_ANTI_REVERBST_EN);
}

/*
 * set the terminal current in charging process
 * (min value is 400ma for scharger ic bug)
 */
int hi6526_set_terminal_current(struct charger_device *dev, u32 term_current)
{
	u8 i_term;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (term_current < CHG_TERM_ICHG_200MA)
		i_term = 0; /* <200mA */
	else if (term_current >= CHG_TERM_ICHG_200MA &&
		term_current < CHG_TERM_ICHG_300MA)
		i_term = 1; /* 200mA */
	else if (term_current >= CHG_TERM_ICHG_300MA &&
		term_current < CHG_TERM_ICHG_400MA)
		i_term = 2; /* 300mA */
	else
		i_term = 3; /* default 400mA */

	/* for chip bug */
	i_term = 3; /* default 400mA */

	scharger_dbg("term_current: %d, term current reg is set 0x%x\n",
		      term_current, i_term);
	return hi6526_write_mask(di, CHG_TERM_ICHG_REG, CHG_TERM_ICHG_MSK,
		CHG_TERM_ICHG_SHIFT, i_term);
}

int hi6526_set_charge_enable(struct charger_device *dev, bool enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	return hi6526_set_charge_enable_execute(di, enable);
}

int hi6526_set_otg_current_execute(u32 value)
{
	u8 reg;
	unsigned int temp_current_ma;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	temp_current_ma = value;

	if (temp_current_ma < BOOST_LIM_MIN || temp_current_ma > BOOST_LIM_MAX)
		scharger_inf("set otg current %dmA is out of range!\n", value);
	if (temp_current_ma < BOOST_LIM_500)
		reg = 0; /* 0: marking (0, BOOST_LIM_500) */
	else if (temp_current_ma >= BOOST_LIM_500 && temp_current_ma <
		 BOOST_LIM_1000)
		reg = 0; /* 0: marking [BOOST_LIM_500, BOOST_LIM_1000) */
	else if (temp_current_ma >= BOOST_LIM_1000 && temp_current_ma <
		 BOOST_LIM_1500)
		reg = 1; /* 1: marking [BOOST_LIM_1000, BOOST_LIM_1500) */
	else if (temp_current_ma >= BOOST_LIM_1500 && temp_current_ma <
		 BOOST_LIM_2000)
		reg = 3; /* default 400mA */
	else
		reg = 3; /* default 400mA */

	scharger_dbg("otg current reg is set 0x%x\n", reg);
	return hi6526_write_mask(di, CHG_OTG_REG0, CHG_OTG_LIM_MSK,
		CHG_OTG_LIM_SHIFT, reg);
}

int hi6526_set_otg_current(struct charger_device *dev, u32 value)
{
	return hi6526_set_otg_current_execute(value);
}

static int hi6526_otg_switch_mode_execute(int enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}
	scharger_inf("%s enable %d\n", __func__, enable);
	return 0;
}

static int hi6526_otg_switch_mode(struct charger_device *dev, int enable)
{
	return hi6526_otg_switch_mode_execute(enable);
}

int hi6526_set_otg_enable_exeute(bool enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	mutex_lock(&di->otg_lock);
	scharger_inf("%s %d\n", __func__, enable);

	if (enable) {
		hi6526_opt_regs_set(di, otg_opt_regs, ARRAY_SIZE(otg_opt_regs));
		hi6526_set_charge_enable_execute(di, CHG_DISABLE);
	} else {
		hi6526_otg_switch_mode_execute(0);
	}

	(void)hi6526_write_mask(di, CHG_OTG_CFG_REG, CHG_OTG_EN_MSK,
		CHG_OTG_EN_SHIFT, enable);
	if (!enable)
		mdelay(50); /* 50: for chip set requirement */

	(void)hi6526_write_mask(di, CHG_OTG_CFG_REG_0, CHG_OTG_MODE_MSK,
		CHG_OTG_MODE_SHIFT, enable);

	if (!enable)
		/* Set optimization parameters to buck mode */
		hi6526_buck_opt_param(di, VBUS_VSET_5V);

	mutex_unlock(&di->otg_lock);
	return 0;
}

int hi6526_set_otg_enable(struct charger_device *dev, bool enable)
{
	return hi6526_set_otg_enable_exeute(enable);
}

int hi6526_set_term_enable(struct charger_device *dev, bool enable)
{
	int vbatt_mv, term_mv, chg_state, dpm, acl, therm;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (di->force_set_term_flag == CHG_STAT_ENABLE) {
		scharger_inf("Charger is in the production line test phase!\n");
		return 0;
	}

	if (enable) {
		dpm = hi6526_check_input_dpm_state(dev);
		acl = hi6526_check_input_acl_state(dev);
		therm = hi6526_check_therm_state();
		vbatt_mv = battery_get_bat_voltage();
		term_mv = hi6526_get_terminal_voltage(dev);
		chg_state = (dpm || acl || therm);
		if (chg_state || (vbatt_mv < (term_mv - 100))) { /* ref vol 100 */
			scharger_inf("%s enable:%d, %d, but in dpm or acl or thermal state\n",
				     __func__, enable, chg_state);
			enable = 0;
		} else {
			scharger_inf("%s enable:%d\n", __func__, enable);
		}
	}

	return hi6526_write_mask(di, CHG_EN_TERM_REG, CHG_EN_TERM_MSK,
		CHG_EN_TERM_SHIFT, !!enable);
}

/*
 * Parameters:   enable:terminal enable or not
 *               0&1:dbc control. 2:original charger procedure
 * return value:  0-success or others-fail
 */
static int hi6526_force_set_term_enable(struct charger_device *dev, int enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	scharger_inf("%s enable:%d\n", __func__, enable);

	if ((enable == 0) || (enable == 1)) {
		di->force_set_term_flag = CHG_STAT_ENABLE;
	} else {
		di->force_set_term_flag = CHG_STAT_DISABLE;
		return 0;
	}
	return hi6526_write_mask(di, CHG_EN_TERM_REG, CHG_EN_TERM_MSK,
		CHG_EN_TERM_SHIFT, (u8)enable);
}

/*
 * return value: charger state:
 *               1:Charge Termination Done
 *               0:Not Charging and buck is closed;Pre-charge;Fast-charg;
 *              -1:error
 */
static int hi6526_get_charger_state(struct charger_device *dev)
{
	int ret;
	u8 data = 0;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	ret = hi6526_read(di, CHG_STATUS_REG, &data);
	if (ret) {
		scharger_err("[%s]:read charge status reg fail\n", __func__);
		return -1;
	}

	data &= HI6526_CHG_STAT_MASK;
	data = data >> HI6526_CHG_STAT_SHIFT;
	switch (data) {
	case 0: /* not charging state */
	case 1: /* not charging state */
	case 2: /* not charging state */
		ret = 0; /* not charging */
		break;
	case 3: /* charging state */
		ret = 1; /* charging */
		break;
	default:
		scharger_err("get charger state fail\n");
		break;
	}

	return ret;
}

static int hi6526_db_value_dump(struct hi6526_device_info *di,
	char *reg_value, int size)
{
	int vbus = 0;
	int ibat = 0;
	int vusb = 0;
	int vout = 0;
	int temp = 0;
	int ibus = 0;
	char buff[BUF_LEN] = { 0 };
	int i;
	int val_array[HI6526_DBG_VAL_SIZE] = {0};
	int len = 0;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	if (hi6526_get_vbus_mv(NULL, (unsigned int *)&vbus))
		scharger_err("%s get vbus value failed!\n", __func__);
	if (hi6526_get_ibat(&ibat, NULL))
		scharger_err("%s get ibat value failed!\n", __func__);
	if (hi6526_get_vusb(&vusb))
		scharger_err("%s get vusb value failed!\n", __func__);
	if (hi6526_get_vout(&vout, di))
		scharger_err("%s get vout value failed!\n", __func__);
	val_array[5] = hi6526_get_vbat(NULL);
	hi6526_get_ibus_ma_exec(di, &ibus);
	if (hi6526_get_chip_temp(&temp, NULL))
		scharger_err("%s get chip temp failed!\n", __func__);

	if (di->chg_mode == LVC)
		snprintf(buff, sizeof(buff), "%s", "LVC    ");
	else if (di->chg_mode == SC)
		snprintf(buff, sizeof(buff), "%s", "SC     ");
	else
		snprintf(buff, sizeof(buff), "%s", "BUCK   ");

	len += strlen(buff);
	if (len < size)
		strncat(reg_value, buff, strlen(buff));

	val_array[0] = ibus; /* ibus site */
	val_array[1] = vbus;
	val_array[2] = ibat;
	val_array[3] = vusb;
	val_array[4] = vout;
	val_array[6] = di->i_bias_all;
	val_array[7] = temp;

	for (i = 0; i < HI6526_DBG_VAL_SIZE; i++) {
		snprintf(buff, sizeof(buff), "%-7.2d", val_array[i]);
		len += strlen(buff);
		if (len < size)
			strncat(reg_value, buff, strlen(buff));
	}

	return len;
}

/* print the register value in charging process */
static int hi6526_dump_reg_value(void *dev, char *reg_value,
	int size)
{
	u8 reg_val = 0;
	int i;
	int ret = 0;
	char buff[BUF_LEN] = { 0 };
	int len;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	len = hi6526_db_value_dump(di, reg_value, size);

	for (i = 0; i < (PAGE1_NUM); i++) {
		ret = ret || hi6526_read(di, PAGE1_BASE + i, &reg_val);
		snprintf(buff, sizeof(buff), "0x%-7x", reg_val);
		len += strlen(buff);
		if (len < size)
			strncat(reg_value, buff, strlen(buff));
	}

	if (ret)
		scharger_err("[%s]ret %d\n", __func__, ret);
	return 0;
}

/* print the register head in charging process */
static int hi6526_reg_head(void *dev, char *reg_head, int size)
{
	const int str_num = HI6526_DBG_VAL_SIZE + 1;
	char buff[BUF_LEN] = { 0 };
	int i;
	int len = 0;

	const static char *type_str[] = {
		"mode ",
		"  Ibus   ",
		"Vbus   ",
		"Ibat   ",
		"Vusb   ",
		"Vout   ",
		"Vbat   ",
		"Ibias  ",
		"Temp   ",

	};

	for (i = 0; i < str_num; i++) {
		snprintf(buff, sizeof(buff), "%s", type_str[i]);
		len += strlen(buff);
		if (len < size)
			strncat(reg_head, buff, strlen(buff));
	}
	memset(buff, 0, sizeof(buff));

	for (i = 0; i < (PAGE1_NUM); i++) {
		snprintf(buff, sizeof(buff), "R[0x%3x] ", PAGE1_BASE + i);
		len += strlen(buff);
		if (len < size)
			strncat(reg_head, buff, strlen(buff));
	}

	return 0;
}

static int hi6526_register_head(char *buffer, int size, void *dev_data)
{
	return  hi6526_reg_head(dev_data, buffer, size);
}

static int hi6526_dump_reg(char *buffer, int size, void *dev_data)
{
	return hi6526_dump_reg_value(dev_data, buffer, size);
}

static int hi6526_set_batfet_disable(struct charger_device *dev, int disable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	scharger_dbg("%s >>> disable:%d\n", __func__, disable);
	return hi6526_set_batfet_ctrl(di, !disable);
}

int hi6526_set_watchdog_timer_execute(struct hi6526_device_info *di, int value)
{
	u8 val;
	u8 dog_time = (u8)value;

	/*
	 * val(0,2,3,4,5,6,7): mask values according to
	 * chip set the concrete values could be checked in
	 * nManager or <pmic_interface.h>
	 */
	if (dog_time >= WATCHDOG_TIMER_80_S)
		val = 7;
	else if (dog_time >= WATCHDOG_TIMER_40_S)
		val = 6;
	else if (dog_time >= WATCHDOG_TIMER_20_S)
		val = 5;
	else if (dog_time >= WATCHDOG_TIMER_10_S)
		val = 4;
	else if (dog_time >= WATCHDOG_TIMER_02_S)
		val = 3;
	else if (dog_time >= WATCHDOG_TIMER_01_S)
		val = 2;
	else
		val = 0;

	scharger_dbg("watch dog timer is %u,the register value is set %u\n",
		     dog_time, val);
	hi6526_reset_watchdog_timer_execute(di);
	return hi6526_write_mask(di, WATCHDOG_CTRL_REG, WATCHDOG_TIMER_MSK,
		WATCHDOG_TIMER_SHIFT, val);
}

int hi6526_set_watchdog_timer(struct charger_device *dev, int value)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	return hi6526_set_watchdog_timer_execute(di, value);
}

int hi6526_set_watchdog_timer_ms(int ms, void *dev_data)
{
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;
	return hi6526_set_watchdog_timer_execute(di, ms / 1000); /* micro second to second */
}

/* set the charger hiz close watchdog */
static int hi6526_set_charger_hiz(struct charger_device *dev, bool enable)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}
	return hi6526_set_buck_enable(!enable, di);
}

/*
 * Description: check voltage of DN/DP
 * return value: 1:water intrused/ 0:water not intrused/ -ENOMEM:hi6526 is not
 * initializied
 */
static int hi6526_is_water_intrused(void *dev_data)
{
	int dp_res;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (!di->param_dts.dp_th_res)
		return 0;

	dp_res = hi6526_get_dp_res(di);
	if (dp_res > di->param_dts.dp_th_res) {
		scharger_inf("D+ water intrused\n");
		return 1;
	}

	return 0;
}

static int hi6526_soft_vbatt_ovp_protect(struct charger_device *dev)
{
	struct hi6526_device_info *di = g_hi6526_dev;
	int vbatt_mv, vbatt_max;

	if (di == NULL)
		return -1;
	vbatt_mv = battery_get_bat_voltage();
	vbatt_max = huawei_get_vbat_max();
	if (vbatt_mv >= min(CHG_VBATT_SOFT_OVP_MAX, chg_vbatt_cv_103(vbatt_max))) {
		di->batt_ovp_cnt_30s++;
		if (di->batt_ovp_cnt_30s == CHG_VBATT_SOFT_OVP_CNT) {
			hi6526_set_charger_hiz(dev, TRUE);
			scharger_err("%s:vbat:%d,cv_mv:%d,ovp_cnt:%u,shutdown buck\n",
				     __func__, vbatt_mv, di->term_vol_mv,
				     di->batt_ovp_cnt_30s);
			di->batt_ovp_cnt_30s = 0;
		}
	} else {
		di->batt_ovp_cnt_30s = 0;
	}
	return 0;
}

/*
 * limit buck current to 470ma according to rboost count
 * Return: 0: do nothing; 1:limit buck current 470ma
 */
static int hi6526_rboost_buck_limit(struct charger_device *dev)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (di->reverbst_cnt > ILIMIT_RBOOST_CNT) {
		set_boot_weaksource_flag();
		scharger_inf("%s:rboost cnt:%d\n", __func__, di->reverbst_cnt);
		return 1;
	}
	di->reverbst_cnt = 0;
	return 0;
}

int hi6526_dpm_init(void)
{
	struct hi6526_device_info *di = g_hi6526_dev;
	int ret, ret0, ret1;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	/* set dpm mode auto */
	ret0 = hi6526_write_mask(di, CHG_DPM_MODE_REG, CHG_DPM_MODE_MSK,
		CHG_DPM_MODE_SHIFT, CHG_DPM_MODE_AUTO);

	/* set dpm voltage sel 90% vbus */
	ret1 = hi6526_write_mask(di, CHG_DPM_SEL_REG, CHG_DPM_SEL_MSK,
		CHG_DPM_SEL_SHIFT, CHG_DPM_SEL_DEFAULT);

	ret = (ret0 || ret1);
	return ret;
}

/* ibat resisitance val for ADC */
int hi6526_ibat_res_sel(struct hi6526_device_info *di)
{
	int ret;
	u8 val = 0;

	if (!di) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	if (di->param_dts.r_coul_mohm == R_MOHM_2)
		val = 0;
	else if ((di->param_dts.r_coul_mohm == R_MOHM_5) || (di->param_dts.r_coul_mohm == R_MOHM_10))
		val = 1;
	else
		scharger_err("%s: %d mohm, not expected\n", __func__, di->param_dts.r_coul_mohm);

	ret = hi6526_write_mask(di, IBAT_RES_SEL_REG, IBAT_RES_SEL_MASK,
		IBAT_RES_SEL_SHIFT, val);

	return ret;
}

void hi6526_vusb_uv_det_enable(u8 enable)
{
	int ret;
	u8 value = 0;

	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return;
	}

	if (di->hi6526_version != CHIP_VERSION_V100)
		return;

	ret = hi6526_write_mask(di, VUSB_UV_DET_ENB_REG, VUSB_UV_DET_ENB_MASK,
		VUSB_UV_DET_ENB_SHIFT, !enable);
	if (ret)
		scharger_err("%s:reg write failed!\n", __func__);

#ifdef CONFIG_TCPC_HI6526
	(void)hi6526_tcpc_set_vusb_uv_det_sts(!enable);
#endif

	ret = hi6526_read(di, VUSB_UV_DET_ENB_REG, &value);
	if (ret)
		scharger_err("%s:reg read failed!\n", __func__);
	scharger_dbg("%s:%u, reg read back 0x%x!\n", __func__, enable, value);
}

static int hi6526_stop_charge_config(struct charger_device *dev)
{
	int ret;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}

	ret = hi6526_set_vbus_vset(dev, VBUS_VSET_5V);
	hi6526_vusb_uv_det_enable(1);

	di->is_weaksource = WEAKSOURCE_FALSE;
	di->reverbst_cnt = 0;

	return ret;
}

/* for loadswitch ops */
int hi6526_dummy_fun_1_execute(void *dev_data)
{
	return 0;
}

int hi6526_dummy_fun_1(struct charger_device *dev)
{
	return hi6526_dummy_fun_1_execute(NULL);
}

int hi6526_dummy_fun_2_execute(u32 val)
{
	return 0;
}

int hi6526_dummy_fun_2(struct charger_device *dev, u32 val)
{
	return hi6526_dummy_fun_2_execute(val);
}

static int cv_pre_trim_cali_value_get(int trim_a, u8 src_val, u8 *des_val)
{
	int index = 0;
	int i, index_tmp;
	int limit = ARRAY_SIZE(ate_trim_tbl);

	if (des_val == NULL)
		return -1;

	if (trim_a < TRIM_A_MIN || trim_a > TRIM_A_MAX) {
		scharger_err("%s trim_a=%d,error!\n", __func__, trim_a);
		return -1;
	}

	for (i = 0; i < limit; i++) {
		if (ate_trim_tbl[i] == src_val) {
			index = i;
			break;
		}
	}
	if (i >= limit) {
		scharger_err("%s ate trim value err!\n", __func__);
		return -1;
	}
	index_tmp = index + trim_a;
	if (index_tmp < 0 || index_tmp >= limit) {
		scharger_err("%s pre trim value err! index=%d, a=%d, tmp=%d\n",
			     __func__, index, trim_a, index_tmp);
		return -1;
	}
	*des_val = ate_trim_tbl[index_tmp];
	return 0;
}

int hi6526_set_pretrim_code(struct charger_device *dev, int val)
{
	int ret;
	u8 ate_code = 0;
	u8 val_ate_trim, pre_trim_tmp, pre_trim_code;
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -1;
	}

	scharger_inf("%s hi6526_version=0x%x!\n", __func__, di->hi6526_version);
	if (di->hi6526_version == CHIP_VERSION_V100)
		return -1;

	ret = hi6526_efuse_read(EFUSE2, EFUSE_BYTE7, &ate_code, 0);
	scharger_inf("%s val_ate=0x%x, ret1=%d!\n", __func__, ate_code, ret);
	if (ret)
		return -1;

	val_ate_trim = (ate_code & CV_ATE_TRIM_MASK) >> CV_ATE_TRIM_SHITF;
	scharger_inf("%s val_ate_trim=0x%x!\n", __func__, val_ate_trim);
	ret = cv_pre_trim_cali_value_get(val, val_ate_trim, &pre_trim_tmp);
	if (ret) {
		scharger_err("%s calculate trim err!\n", __func__);
		return -1;
	}
	pre_trim_code =
		(pre_trim_tmp << CV_ATE_TRIM_SHITF) |
					(ate_code & (~CV_ATE_TRIM_MASK));
	scharger_inf("%s pre_trim_code=0x%x!\n", __func__, pre_trim_code);
	ret = hi6526_efuse_write(EFUSE2, EFUSE_BYTE7, pre_trim_code, 1);
	if (ret)
		return -1;

	return 0;
}

struct charger_ops hi6526_dummy_ops = {
	.chip_init                   = hi6526_chip_init,
	.get_dieid                   = hi6526_get_dieid_str,
	.dev_check                   = hi6526_device_check,
	.set_input_current           = hi6526_set_input_current,
	.set_charging_current        = hi6526_set_charge_current,
	.get_charging_current        = hi6526_get_charge_current,
	.set_constant_voltage        = hi6526_set_terminal_voltage,
	.get_terminal_voltage        = hi6526_get_terminal_voltage,
	.set_eoc_current             = hi6526_set_terminal_current,
	.enable_termination          = hi6526_set_term_enable,
	.set_force_term_enable       = hi6526_force_set_term_enable,
	.enable                      = hi6526_set_charge_enable,
	.get_charge_state            = hi6526_get_charge_state,
	.get_charger_state           = hi6526_get_charger_state,
	.set_watchdog_timer          = hi6526_set_watchdog_timer,
	.kick_wdt                    = hi6526_reset_watchdog_timer,
	.set_batfet_disable          = hi6526_set_batfet_disable,
	.enable_hz                   = hi6526_set_charger_hiz,
	.get_ibus_adc                = hi6526_get_ibus_ma,
	.get_vbus_adc                = hi6526_get_vbus_mv,
	.get_vbat_sys                = NULL,
	.check_input_dpm_state       = hi6526_check_input_dpm_state,
	.check_input_vdpm_state      = hi6526_check_input_dpm_state,
	.check_input_idpm_state      = hi6526_check_input_acl_state,
	.set_mivr                    = hi6526_dummy_fun_2,
	.set_vbus_vset               = hi6526_set_vbus_vset,
	.set_uvp_ovp                 = hi6526_dummy_fun_1,
	.soft_vbatt_ovp_protect      = hi6526_soft_vbatt_ovp_protect,
	.rboost_buck_limit           = hi6526_rboost_buck_limit,
	.stop_charge_config          = hi6526_stop_charge_config,
	.enable_otg                  = hi6526_set_otg_enable,
	.set_boost_current_limit     = hi6526_set_otg_current,
	.set_otg_switch_mode_enable  = hi6526_otg_switch_mode,
	.dump_registers              = NULL,
	.get_iin_set                 = hi6526_get_input_current_set,
	.get_vbat                    = hi6526_get_vbat,
	.get_vusb                    = hi6526_get_vusb_force,
	.set_pretrim_code            = hi6526_set_pretrim_code,
	.get_dieid_for_nv            = hi6526_efuse_get_dieid,
};

#ifdef CONFIG_TCPC_HI6526
static int hi6526_tcpc_read_block(u16 reg, u8 *value, unsigned int num_bytes)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di)
		di = g_hi6526_aux_dev;
	return hi6526_read_block(di, reg, value, num_bytes);
}

static int hi6526_tcpc_write_block(u16 reg, u8 *value, unsigned int num_bytes)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di)
		di = g_hi6526_aux_dev;
	return hi6526_write_block(di, reg, value, num_bytes);
}

static struct hi6526_tcpc_reg_ops hi6526_tcpc_reg_ops = {
	.block_read                  = hi6526_tcpc_read_block,
	.block_write                 = hi6526_tcpc_write_block,
};
#endif /* CONFIG_TCPC_HI6526 */

#ifdef CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT
static int hi6526_plug_in(struct charger_device *chg_dev)
{
	struct chip_init_crit init_crit;

	scharger_inf("plug in set enable true\n");
	init_crit.vbus = ADAPTER_5V;
	init_crit.charger_type = CHARGER_TYPE_STANDARD;
	hi6526_chip_init(chg_dev, &init_crit);
	hi6526_set_charge_enable(chg_dev, true);
	return 0;
}

static int hi6526_plug_out(struct charger_device *chg_dev)
{
	scharger_inf("plug out set enable false\n");
	return 0;
}

static int hi6526_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct hi6526_device_info *di = get_hi6526_device();

	if (!di || !en)
		return -EINVAL;

	scharger_inf("%s:chg_last_enable %d\n", __func__, di->chg_last_enable);

	*en = di->chg_last_enable;
	return 0;
}

#ifdef CONFIG_CHARGER_DUMP_REG_OPS_SUPPORT
static int hi6526_hw_dump_register(struct charger_device *chg_dev,
	char *reg_value, int size)
{
	return 0;
}
#endif /* CONFIG_CHARGER_DUMP_REG_OPS_SUPPORT */

#ifdef CONFIG_CHARGER_GET_REG_HEAD_OPS_SUPPORT
static int hi6526_get_register_head(struct charger_device *chg_dev,
	char *reg_head, int size)
{
	return 0;
}
#endif /* CONFIG_CHARGER_GET_REG_HEAD_OPS_SUPPORT */

static int hi6526_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret;
	unsigned int state = 0;

	if (!done)
		return -EINVAL;

	*done = false;
	ret = hi6526_get_charge_state(NULL, &state);
	if (ret < 0) {
		scharger_err("get ic status failed\n");
		return ret;
	}

	if (state & CHAGRE_STATE_CHRG_DONE) {
		scharger_inf("is charging done\n");
		*done = true;
	}

	return 0;
}

static int hi6526_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	if (!curr)
		return -EINVAL;

	*curr = 0;
	return 0;
}

static int hi6526_charger_do_event(struct charger_device *chg_dev,
	u32 event, u32 args)
{
	struct hi6526_device_info *chip = g_hi6526_dev;

	if (!chip) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	switch (event) {
	case PLAT_EVENT_EOC:
		scharger_inf("do eoc event\n");
		charger_dev_notify(chip->chg_dev, PLAT_CHARGER_DEV_NOTIFY_EOC);
		break;
	case PLAT_EVENT_RECHARGE:
		scharger_inf("do recharge event\n");
		charger_dev_notify(chip->chg_dev, PLAT_CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return 0;
}

static int hi6526_get_ibus_ua(struct charger_device *dev, u32 *ibus)
{
	int ret;

	if (!ibus)
		return -EINVAL;

	ret = hi6526_get_ibus_ma(dev, ibus);
	if (ret < 0)
		return ret;

	*ibus *= 1000;

	return 0;
}

static int hi6526_set_input_current_ua(struct charger_device *dev, u32 cin_limit_ua)
{
	return hi6526_set_input_current(dev, cin_limit_ua / 1000);
}

static int hi6526_get_input_current_set_ua(struct charger_device *dev, u32 *ua)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (!di || !ua) {
		scharger_err("%s hi6526_device_info is NULL\n", __func__);
		return -ENOMEM;
	}

	*ua = di->iin_set * 1000;

	 return 0;
}

static int hi6526_get_charge_current_ua(struct charger_device *dev, u32 *ichg)
{
	int ret;

	if (!ichg)
		return -EINVAL;

	ret = hi6526_get_charge_current(dev, ichg);
	if (ret <= CHG_FAST_ICHG_00MA)
		return -1;

	*ichg *= 1000;

	return 0;
}

static int hi6526_set_charge_current_uv(struct charger_device *dev, u32 charge_current_uv)
{
	return hi6526_set_charge_current(dev, charge_current_uv / 1000);
}

static int hi6526_set_terminal_voltage_uv(struct charger_device *dev, u32 charge_voltage_uv)
{
	return hi6526_set_terminal_voltage(dev, charge_voltage_uv / 1000);
}

static int hi6526_get_terminal_voltage_uv(struct charger_device *dev, u32 *uv)
{
	int ret;

	if (!uv)
		return -EINVAL;

	ret = hi6526_get_terminal_voltage(dev);
	if (ret < 0)
		return ret;

	*uv = (u32)ret * 1000;

	return 0;
}

static int hi6526_set_terminal_current_ua(struct charger_device *dev, u32 term_current_ua)
{
	return hi6526_set_terminal_current(dev, term_current_ua / 1000);
}

struct charger_ops hi6526_ops = {
	.chip_init                   = hi6526_chip_init,
	.set_vbus_vset               = hi6526_set_vbus_vset,
	.plug_in                     = hi6526_plug_in,
	.plug_out                    = hi6526_plug_out,
	.enable                      = hi6526_set_charge_enable,
	.is_enabled                  = hi6526_is_charging_enable,
	.get_ibus_adc                = hi6526_get_ibus_ua,
	.get_vbus_adc                = hi6526_get_vbus_mv,
	.get_vbat                    = hi6526_get_vbat,
	.get_vusb                    = hi6526_get_vusb_force,
	.get_charge_state            = hi6526_get_charge_state,
	.dump_registers              = NULL,
#ifdef CONFIG_CHARGER_DUMP_REG_OPS_SUPPORT
	.dump_reg                    = hi6526_hw_dump_register,
#endif
#ifdef CONFIG_CHARGER_GET_REG_HEAD_OPS_SUPPORT
	.get_reg_head                = hi6526_get_register_head,
#endif
	.get_charging_current        = hi6526_get_charge_current_ua,
	.set_charging_current        = hi6526_set_charge_current_uv,
	.get_input_current           = hi6526_get_input_current_set_ua,
	.set_input_current           = hi6526_set_input_current_ua,
	.get_constant_voltage        = hi6526_get_terminal_voltage_uv,
	.set_constant_voltage        = hi6526_set_terminal_voltage_uv,
	.set_eoc_current             = hi6526_set_terminal_current_ua,
	.enable_termination          = hi6526_set_term_enable,
	.set_force_term_enable       = hi6526_force_set_term_enable,
	.kick_wdt                    = hi6526_reset_watchdog_timer,
	.set_mivr                    = NULL,
	.is_charging_done            = hi6526_is_charging_done,
	.get_min_charging_current    = hi6526_get_min_ichg,
	.enable_otg                  = hi6526_set_otg_enable,
	.event                       = hi6526_charger_do_event,
	.enable_hz                   = hi6526_set_charger_hiz,
};
#endif /* CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT */

static struct water_detect_ops hi6526_water_detect_ops = {
	.type_name                   = "usb_dp_dn",
	.is_water_intruded           = hi6526_is_water_intrused,
};

static struct power_log_ops hi6526_log_ops = {
	.dev_name                    = "hi6526",
	.dump_log_head               = hi6526_register_head,
	.dump_log_content            = hi6526_dump_reg,
};

static struct power_log_ops hi6526_log_aux_ops = {
	.dev_name                    = "hi6526_aux",
	.dump_log_head               = hi6526_register_head,
	.dump_log_content            = hi6526_dump_reg,
};

static struct fcp_protocol_ops hi6526_fcp_protocol_ops = {
	.chip_name                   = "scharger_v600",
	.reg_read                    = hi6526_fcp_reg_read_block,
	.reg_write                   = hi6526_fcp_reg_write_block,
	.detect_adapter              = hi6526_fcp_detect_adapter,
	.soft_reset_master           = hi6526_fcp_master_reset,
	.soft_reset_slave            = hi6526_fcp_adapter_reset,
	.get_master_status           = hi6526_fcp_read_switch_status,
	.stop_charging_config        = hi6526_fcp_stop_charge_config,
	.is_accp_charger_type        = hi6526_is_fcp_charger_type,
};

static struct scp_protocol_ops hi6526_scp_protocol_ops = {
	.chip_name                   = "scharger_v600",
	.reg_read                    = hi6526_scp_reg_read_block,
	.reg_write                   = hi6526_scp_reg_write_block,
	.reg_multi_read              = hi6526_fcp_adapter_reg_read_block,
	.detect_adapter              = hi6526_scp_detect_adapter,
	.soft_reset_master           = hi6526_scp_chip_reset,
	.soft_reset_slave            = hi6526_scp_adaptor_reset,
	.pre_init                    = hi6526_pre_init,
};

static struct dc_ic_ops hi6526_lvc_ops = {
	.dev_name                    = "hi6526",
	.ic_init                     = hi6526_dummy_fun_1_execute,
	.ic_exit                     = hi6526_switch_to_buck_mode,
	.ic_enable                   = hi6526_lvc_enable,
	.is_ic_close                 = hi6526_lvc_is_close,
	.get_ic_id                   = hi6526_get_loadswitch_id,
	.config_ic_watchdog          = hi6526_set_watchdog_timer_ms,
	.kick_ic_watchdog            = hi6526_reset_watchdog_timer_execute,
	.set_ic_buck_enable          = hi6526_set_buck_enable,
};

static struct dc_ic_ops hi6526_sc_ops = {
	.dev_name                    = "hi6526",
	.ic_init                     = hi6526_dummy_fun_1_execute,
	.ic_exit                     = hi6526_switch_to_buck_mode,
	.ic_enable                   = hi6526_sc_enable,
	.is_ic_close                 = hi6526_sc_is_close,
	.get_ic_id                   = hi6526_get_switchcap_id,
	.config_ic_watchdog          = hi6526_set_watchdog_timer_ms,
	.kick_ic_watchdog            = hi6526_reset_watchdog_timer_execute,
	.set_ic_buck_enable          = hi6526_set_buck_enable,
};

static struct dc_batinfo_ops hi6526_batinfo_ops = {
	.init                        = hi6526_dummy_fun_1_execute,
	.exit                        = hi6526_dummy_fun_1_execute,
	.get_bat_btb_voltage         = hi6526_get_vbat_execute,
	.get_bat_package_voltage     = hi6526_get_vbat_execute,
	.get_vbus_voltage            = hi6526_get_vbus_mv2,
	.get_bat_current             = hi6526_get_ibat,
	.get_ic_ibus                 = hi6526_batinfo_get_ibus_ma,
	.get_ic_temp                 = hi6526_get_chip_temp,
	.get_ic_vout                 = hi6526_get_vout,
};

static struct dc_ic_ops hi6526_sc_aux_ops = {
	.dev_name                    = "hi6526",
	.ic_init                     = hi6526_dummy_fun_1_execute,
	.ic_exit                     = hi6526_switch_to_buck_mode,
	.ic_enable                   = hi6526_sc_enable,
	.is_ic_close                 = hi6526_sc_is_close,
	.get_ic_id                   = hi6526_get_switchcap_id,
	.config_ic_watchdog          = hi6526_set_watchdog_timer_ms,
	.kick_ic_watchdog            = hi6526_reset_watchdog_timer_execute,
	.set_ic_buck_enable          = hi6526_set_buck_enable,
};

static struct dc_batinfo_ops hi6526_batinfo_aux_ops = {
	.init                        = hi6526_dummy_fun_1_execute,
	.exit                        = hi6526_dummy_fun_1_execute,
	.get_bat_btb_voltage         = hi6526_get_vbat_execute,
	.get_bat_package_voltage     = hi6526_get_vbat_execute,
	.get_vbus_voltage            = hi6526_get_vbus_mv2,
	.get_bat_current             = hi6526_get_ibat,
	.get_ic_ibus                 = hi6526_batinfo_get_ibus_ma,
	.get_ic_temp                 = hi6526_get_chip_temp,
	.get_ic_vout                 = hi6526_get_vout,
};

static void hi6526_fcp_scp_ops_register(void)
{
	int i;
	int ret[RET_SIZE_8] = {0};

	/* if chip support fcp, register fcp adapter ops */
	if (hi6526_is_support_fcp() == 0)
		(void)fcp_protocol_ops_register(&hi6526_fcp_protocol_ops);

	/* if chip support scp,register scp adapter ops */
	if (hi6526_is_support_scp() == 0) {
#ifdef CONFIG_DIRECT_CHARGER
		/*
		 * ret[0]: reserved
		 * ret[1]: lvc ops reg
		 * ret[2]: sc ops reg
		 * ret[3]: sc batinfo ops reg
		 * ret[4]: lvc batinfo ops reg
		 */
		ret[0] = 0;
		ret[1] = dc_ic_ops_register(LVC_MODE, CHARGE_IC_TYPE_MAIN,
			&hi6526_lvc_ops);
		ret[2] = dc_ic_ops_register(SC_MODE, CHARGE_IC_TYPE_MAIN,
			&hi6526_sc_ops);
		ret[3] = dc_batinfo_ops_register(SC_MODE, CHARGE_IC_TYPE_MAIN,
			&hi6526_batinfo_ops);
		ret[4] = dc_batinfo_ops_register(LVC_MODE, CHARGE_IC_TYPE_MAIN,
			&hi6526_batinfo_ops);
#else
		ret[0] = 0;
		ret[1] = dummy_ops_register2(&hi6526_lvc_ops);
		ret[2] = dummy_ops_register2(&hi6526_sc_ops);
		ret[3] = dummy_ops_register3(&hi6526_batinfo_ops);
#endif

		for (i = 0; i < RET_SIZE_8; i++) {
			if (ret[i])
				scharger_err("register scp adapter ops failed i %d!\n", i);
		}

#ifdef CONFIG_DIRECT_CHARGER
		scp_protocol_ops_register(&hi6526_scp_protocol_ops);
#endif /* CONFIG_DIRECT_CHARGER */
	}
}

/*
 * schedule or cancel check work based on charger type
 *        USB/NON_STD/BC_USB: schedule work
 *                   REMOVED: cancel work
 */
static void hi6526_plugout_check_process(int type)
{
	struct hi6526_device_info *di = g_hi6526_dev;

	if (di == NULL)
		return;
	switch (type) {
	case CHARGER_TYPE_SDP:
	case CHARGER_TYPE_DCP:
	case CHARGER_TYPE_CDP:
	case CHARGER_TYPE_UNKNOWN:
		(void)hi6526_write_mask(di, CHG_IRQ_MASK_0,
			CHG_IRQ_VBUS_UVP_MSK,
			CHG_IRQ_VBUS_UVP_SHIFT,
			IRQ_VBUS_UVP_UNMASK);
		break;

	case CHARGER_TYPE_NONE:
		di->reverbst_cnt = 0;
		di->reverbst_begin = 0;
		(void)hi6526_write_mask(di, CHG_IRQ_MASK_0,
			CHG_IRQ_VBUS_UVP_MSK,
			CHG_IRQ_VBUS_UVP_SHIFT,
			IRQ_VBUS_UVP_MASK);
		clr_boot_weaksource_flag();
		break;
	default:
		break;
	}
}

/* respond the charger_type events from USB PHY */
static int hi6526_usb_notifier_call(struct notifier_block *usb_nb,
				    unsigned long event, void *data)
{
	int type = (int)event;

	scharger_inf("%s:notifier event %d\n", __func__, type);
	hi6526_plugout_check_process(type);
	return NOTIFY_OK;
}

#ifdef CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT
static int hi6526_charger_device_register(struct i2c_client *client,
	struct hi6526_device_info *di)
{
	/*
	 * if ic_role NOT CHARGE_IC_TYPE_MAIN, just ignore
	 */
	if (di->param_dts.ic_role != CHARGE_IC_TYPE_MAIN)
		return 0;

	/*
	 * if hi6526 v600 buck disabled, just ignore
	 */
	if (!di->param_dts.enable_v600_buck)
		return 0;

	di->chg_props.alias_name = "schargerv600";
	di->chg_dev = charger_device_register("primary_chg",
		&client->dev, di, &hi6526_ops, &di->chg_props);
	if (IS_ERR_OR_NULL(di->chg_dev)) {
		scharger_err("scharger_v600 charge ops register fail\n");
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT */

static int hi6526_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct hi6526_device_info *di = NULL;
	struct device_node *np = NULL;
	struct chip_init_crit init_crit;

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}
	if (hi6526_lock_mutex_init(di)) {
		ret = -EINVAL;
		goto hi6526_init_mutex_dev_irq_fail;
	}
	di->dev = &client->dev;
	np = di->dev->of_node;
	di->client = client;
	di->i2c_reg_page = 0;
	i2c_set_clientdata(client, di);

	ret = hi6526_device_check_exec(di);
	if (ret == CHARGE_IC_BAD)
		goto hi6526_init_mutex_dev_irq_fail;

	hi6526_parse_dts(np, di);

	INIT_WORK(&di->irq_work, hi6526_irq_work);
	INIT_DELAYED_WORK(&di->reverbst_work, hi6526_reverbst_delay_work);
	INIT_DELAYED_WORK(&di->dc_ucp_work, hi6526_dc_ucp_delay_work);
	INIT_DELAYED_WORK(&di->dbg_work, hi6526_dbg_work);

	ret = hi6526_irq_init(di, np);
	if (ret) {
		scharger_err("%s, hi6526_irq_init failed\n", __func__);
		goto hi6526_init_mutex_dev_irq_fail;
	}
	di->usb_nb.notifier_call = hi6526_usb_notifier_call;
	if (ret < 0) {
		scharger_err("charger_type_notifier_register failed\n");
		goto hi6526_register_notifier_fail;
	}
	di->hi6526_version = hi6526_get_device_version(di);
	hi6526_vbat_lv_handle(di);
	/* use for main charger only */
	if (di->param_dts.ic_role == CHARGE_IC_TYPE_MAIN) {
		g_hi6526_dev = di;
		hi6526_fcp_protocol_ops.dev_data = di;
		hi6526_scp_protocol_ops.dev_data = di;
		hi6526_lvc_ops.dev_data = di;
		hi6526_sc_ops.dev_data = di;
		hi6526_batinfo_ops.dev_data = di;
		hi6526_log_ops.dev_data = di;
		di->term_vol_mv = hi6526_get_terminal_voltage(NULL);
		hi6526_buck_opt_param(di, VBUS_VSET_5V);
		wakeup_source_init(&di->hi6526_wake_lock, "hi6526_adc_wakelock");
		(void)water_detect_ops_register(&hi6526_water_detect_ops);
		ret = power_log_ops_register(&hi6526_log_ops);
		if (ret)
			scharger_err("power log ops register fail\n");
		ret = hi6526_sysfs_create_group(di);
		if (ret) {
			scharger_err("create sysfs entries failed!\n");
			goto hi6526_create_sysfs_fail;
		}
		hi6526_fcp_scp_ops_register();
		init_crit.vbus = ADAPTER_5V;
		init_crit.charger_type = 0;
		ret = hi6526_chip_init(NULL, &init_crit);
		if (ret)
			scharger_err("hi6526 chip init failed\n");
	} else if (di->param_dts.ic_role == CHARGE_IC_TYPE_AUX) {
		g_hi6526_aux_dev = di;
		hi6526_sc_aux_ops.dev_data = di;
		hi6526_batinfo_aux_ops.dev_data = di;
		hi6526_log_aux_ops.dev_data = di;

		wakeup_source_init(&di->hi6526_wake_lock, "hi6526_adc_wakelock_aux");
		(void)hi6526_ibat_res_sel(di);
		(void)hi6526_set_charge_enable_execute(di, CHG_DISABLE);
		(void)hi6526_config_opt_param(di, VBUS_VSET_12V);
		(void)hi6526_set_vbus_uvp_ovp(di, VBUS_VSET_12V);
		dc_ic_ops_register(SC_MODE, CHARGE_IC_TYPE_AUX,
			&hi6526_sc_aux_ops);
		dc_batinfo_ops_register(SC_MODE, CHARGE_IC_TYPE_AUX,
			&hi6526_batinfo_aux_ops);
		ret = power_log_ops_register(&hi6526_log_aux_ops);
		if (ret)
			scharger_err("power log ops register fail\n");
	}
	hi6526_opt_regs_set(di, common_opt_regs, ARRAY_SIZE(common_opt_regs));
	hi6526_unmask_all_irq(di);

#ifdef CONFIG_TCPC_HI6526
	(void)hi6526_tcpc_reg_ops_register(client, &hi6526_tcpc_reg_ops);
	(void)hi6526_tcpc_irq_gpio_register(client, di->gpio_int);
	(void)hi6526_tcpc_notify_chip_version(di->hi6526_version);
#endif

#ifdef CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT
	ret = hi6526_charger_device_register(client, di);
	if (ret)
		goto hi6526_create_sysfs_fail;
#endif /* CONFIG_HI6526_PRIMARY_CHARGER_SUPPORT */

	scharger_inf("%s_%d success!\n", __func__, di->param_dts.ic_role);
	return 0;

hi6526_create_sysfs_fail:
	if (di->param_dts.ic_role == CHARGE_IC_TYPE_MAIN)
		hi6526_sysfs_remove_group(di);
	if (ret < 0)
		scharger_err("charger_type_notifier_unregister failed\n");
hi6526_register_notifier_fail:
	free_irq(di->irq_int, di);
	gpio_free(di->gpio_int);
hi6526_init_mutex_dev_irq_fail:
	g_hi6526_dev = NULL;

	return ret;
}

static int hi6526_remove(struct i2c_client *client)
{
	struct hi6526_device_info *di = i2c_get_clientdata(client);

	if (di == NULL)
		return -1;
	if (di->param_dts.ic_role == CHARGE_IC_TYPE_MAIN)
		hi6526_sysfs_remove_group(di);

	if (di->irq_int)
		free_irq(di->irq_int, di);
	if (di->gpio_int)
		gpio_free(di->gpio_int);
	g_hi6526_dev = NULL;
	return 0;
}

static void hi6526_shutdown(struct i2c_client *client)
{
	struct hi6526_device_info *di = NULL;
	if (!client) {
		scharger_err("%s: invalid param\n", __func__);
		return;
	}
	di = i2c_get_clientdata(client);
	if (di == NULL)
		return;

	(void)hi6526_global_reset(di);
}

MODULE_DEVICE_TABLE(i2c, HI6526);
const static struct of_device_id hi6526_of_match[] = {
	{
		.compatible = "huawei,hi6526_charger",
		.data = NULL,
	}, {

	},
};

static const struct i2c_device_id hi6526_i2c_id[] = {
	{ "hi6526_charger", 0 }, { }
};

static struct i2c_driver hi6526_driver = {
	.probe    = hi6526_probe,
	.remove   = hi6526_remove,
	.shutdown = hi6526_shutdown,
	.id_table = hi6526_i2c_id,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "hi6526_charger",
		.of_match_table = of_match_ptr(hi6526_of_match),
	},
};

static int __init hi6526_init(void)
{
	int ret;

	ret = i2c_add_driver(&hi6526_driver);
	if (ret)
		scharger_err("%s: i2c_add_driver error!!!\n", __func__);
	scharger_inf("%s:!\n", __func__);
	return ret;
}

static void __exit hi6526_exit(void)
{
	i2c_del_driver(&hi6526_driver);
}

module_init(hi6526_init);
module_exit(hi6526_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HI6526 charger driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
