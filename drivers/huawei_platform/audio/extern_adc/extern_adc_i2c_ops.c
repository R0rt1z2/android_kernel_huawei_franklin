/*
 * extern_adc_i2c_ops.c
 *
 * extern adc i2c driver
 *
 * Copyright (c) 2020 Huawei Technologies Co., Ltd.
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

#include "extern_adc.h"

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <huawei_platform/log/hw_log.h>

#ifdef CONFIG_HUAWEI_DSM_AUDIO_MODULE
#define CONFIG_HUAWEI_DSM_AUDIO
#endif

#ifdef CONFIG_HUAWEI_DSM_AUDIO
#include <dsm_audio/dsm_audio.h>
#endif

#define HWLOG_TAG extern_adc
HWLOG_REGIST();

#ifdef CONFIG_HUAWEI_DSM_AUDIO
#define DSM_BUF_SIZE DSM_AUDIO_BUF_SIZE
#endif

#define RETRY_TIMES                3
#define EXT_ADC_I2C_READ           0
#define EXT_ADC_I2C_WRITE          1
#define REG_PRINT_NUM              4

#define I2C_STATUS_B64 64

/* 0 i2c device not init completed, 1 init completed */
static bool extern_adc_i2c_probe_skip[EXTERN_ADC_ID_MAX] = {
	0, 0, 0, 0, 0, 0, 0, 0 };

static inline int extern_adc_check_i2c_regmap_cfg(
	struct extern_adc_i2c_priv *i2c_priv)
{
	if ((i2c_priv == NULL) || (i2c_priv->regmap_cfg == NULL) ||
		(i2c_priv->regmap_cfg->regmap == NULL))
		return -EINVAL;
	return 0;
}

static inline int extern_adc_regmap_read(struct regmap *regmap,
	unsigned int reg_addr, unsigned int *value)
{
	int ret;
	int i;

	for (i = 0; i < RETRY_TIMES; i++) {
		ret = regmap_read(regmap, reg_addr, value);
		if (ret == 0)
			break;

		mdelay(1);
	}
	return ret;
}

static inline int extern_adc_regmap_write(struct regmap *regmap,
	unsigned int reg_addr, unsigned int value)
{
	int ret;
	int i;

	for (i = 0; i < RETRY_TIMES; i++) {
		ret = regmap_write(regmap, reg_addr, value);
		if (ret == 0)
			break;

		mdelay(1);
	}
	return ret;
}

static inline int extern_adc_regmap_update_bits(struct regmap *regmap,
	unsigned int reg_addr, unsigned int mask, unsigned int value)
{
	int ret;
	int i;

	for (i = 0; i < RETRY_TIMES; i++) {
		ret = regmap_update_bits(regmap, reg_addr, mask, value);
		if (ret == 0)
			break;

		mdelay(1);
	}
	return ret;
}

static int extern_adc_regmap_xorw(struct regmap *regmap,
	unsigned int reg_addr, unsigned int mask, unsigned int read_addr)
{
	int ret;
	unsigned int value = 0;

	ret = extern_adc_regmap_read(regmap, read_addr, &value);
	if (ret < 0) {
		hwlog_info("%s: read reg 0x%x failed, ret = %d\n",
			__func__, read_addr, ret);
		return ret;
	}

#ifndef CONFIG_FINAL_RELEASE
	hwlog_debug("%s: read reg 0x%x = 0x%x\n", __func__, read_addr, value);
#endif

	value ^= mask;
#ifndef CONFIG_FINAL_RELEASE
	hwlog_debug("%s: after xor 0x%x, write reg 0x%x = 0x%x\n", __func__,
		mask, reg_addr, value);
#endif

	ret += extern_adc_regmap_write(regmap, reg_addr, value);
	return ret;
}

static int extern_adc_regmap_complex_write(struct extern_adc_regmap_cfg *cfg,
	unsigned int reg_addr, unsigned int mask, unsigned int value)
{
	int ret;

	if (cfg == NULL)
		return -EINVAL;

	if ((mask ^ cfg->value_mask) == 0)
		ret = extern_adc_regmap_write(cfg->regmap, reg_addr, value);
	else
		ret = extern_adc_regmap_update_bits(cfg->regmap, reg_addr,
			mask, value);

	return ret;
}

static void extern_adc_delay(unsigned int delay, int index, int num)
{
	if (delay == 0)
		return;
	if ((index < 0) || (num < 0))
		return;

	if (index == num)
		msleep(delay);
	else
		mdelay(delay);
}

#ifdef CONFIG_HUAWEI_DSM_AUDIO
static void extern_adc_append_dsm_report(char *dst, char *fmt, ...)
{
	va_list args;
	unsigned int buf_len;
	char tmp_str[DSM_BUF_SIZE] = {0};

	if ((dst == NULL) || (fmt == NULL)) {
		hwlog_err("%s, dst or src is NULL\n", __func__);
		return;
	}

	va_start(args, fmt);
	vscnprintf(tmp_str, (unsigned long)DSM_BUF_SIZE, (const char *)fmt,
		args);
	va_end(args);

	buf_len = DSM_BUF_SIZE - strlen(dst) - 1;
	if (strlen(dst) < DSM_BUF_SIZE - 1)
		strncat(dst, tmp_str, buf_len);
}
#endif

static void extern_adc_dsm_report_by_i2c_error(const char *model, int id,
	int flag, int errno, struct i2c_err_info *info)
{
	char *report = NULL;

	unused(report);
	if (errno == 0)
		return;

#ifdef CONFIG_HUAWEI_DSM_AUDIO
	report = kzalloc(sizeof(char) * DSM_BUF_SIZE, GFP_KERNEL);
	if (report == NULL)
		return;

	extern_adc_append_dsm_report(report, "%s_%d i2c ", model, id);

	if (flag == EXT_ADC_I2C_READ) /* read i2c error */
		extern_adc_append_dsm_report(report, "read ");
	else /* flag write i2c error == 1 */
		extern_adc_append_dsm_report(report, "write ");

	extern_adc_append_dsm_report(report, "errno %d", errno);

	if (info != NULL)
		extern_adc_append_dsm_report(report,
			" %u fail times of %u all times, err_details is 0x%lx",
			info->err_count, info->regs_num, info->err_details);

	audio_dsm_report_info(AUDIO_CODEC, DSM_CODEC_SSI_READ_ONCE,
			"%s", report);
	hwlog_info("%s: dsm report, %s\n", __func__, report);
	kfree(report);
#endif
}

static void extern_adc_i2c_dsm_report(struct extern_adc_i2c_priv *i2c_priv,
	int flag, int errno, struct i2c_err_info *info)
{
	if (i2c_priv->probe_completed == 0)
		return;
	extern_adc_dsm_report_by_i2c_error(i2c_priv->chip_model,
		(int)i2c_priv->chip_id, flag, errno, info);
}

static int extern_adc_i2c_dump_regs_read(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl *reg)
{
	struct regmap *regmap = NULL;
	unsigned int reg_addr;
	unsigned int value = 0;
	int ret = 0;
	int ret_once;
	int j;

	regmap = i2c_priv->regmap_cfg->regmap;
	reg_addr = reg->addr;

	/* reg->mask  is dump regs num */
	for (j = 0; j < (int)reg->mask; j++) {
		ret_once = extern_adc_regmap_read(regmap, reg_addr, &value);
		extern_adc_i2c_dsm_report(i2c_priv, EXT_ADC_I2C_READ, ret_once,
			NULL);

		if (ret_once < 0)
			ret += ret_once;

		hwlog_info("%s: adc %u, r reg 0x%x = 0x%x\n", __func__,
			i2c_priv->chip_id, reg_addr, value);

		reg_addr++;
	}
	return ret;
}

static int extern_adc_i2c_dump_reg_ops(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl_sequence *sequence)
{
	unsigned int reg_addr;
	unsigned int ctl_value;
	unsigned int ctl_type;
	int ret = 0;
	int ret_once;
	int i;
	struct regmap *regmap = NULL;

	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if ((sequence == NULL) || (sequence->num == 0) ||
		(sequence->regs == NULL))
		return 0;

	regmap = i2c_priv->regmap_cfg->regmap;

	for (i = 0; i < (int)sequence->num; i++) {
		reg_addr = sequence->regs[i].addr;
		ctl_value = sequence->regs[i].value;
		ctl_type = sequence->regs[i].ctl_type;

		if (ctl_type == EXTERN_DAC_REG_CTL_TYPE_W) {
			ret_once = extern_adc_regmap_write(regmap, reg_addr,
				ctl_value);
			extern_adc_i2c_dsm_report(i2c_priv, EXT_ADC_I2C_WRITE,
				ret_once, NULL);

			ret += ret_once;
#ifndef CONFIG_FINAL_RELEASE
			hwlog_info("%s: adc %u, w reg 0x%x 0x%x\n", __func__,
				i2c_priv->chip_id, reg_addr, ctl_value);
#endif
		} else if (ctl_type == EXTERN_DAC_REG_CTL_TYPE_DELAY) {
			if (ctl_value > 0) /* delay time units: msecs */
				msleep(ctl_value);
		} else if (ctl_type == EXTERN_DAC_REG_CTL_TYPE_DUMP) {
			ret += extern_adc_i2c_dump_regs_read(i2c_priv,
					&sequence->regs[i]);
		} else { /* EXTERN_DAC_REG_CTL_TYPE_R or other types */
			continue;
		}
	}

	return ret;
}

static int extern_adc_i2c_dump_regs(struct extern_adc_i2c_priv *i2c_priv)
{
	if (i2c_priv == NULL) {
		hwlog_err("%s: i2c_priv NULL\n", __func__);
		return -EINVAL;
	}

	hwlog_info("%s: adc %u, dump regs\n", __func__, i2c_priv->chip_id);
	return extern_adc_i2c_dump_reg_ops(i2c_priv,
		i2c_priv->dump_regs_sequence);
}

static int extern_adc_i2c_ctrl_read_reg(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl *reg)
{
	int ret;

	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	ret = extern_adc_regmap_read(i2c_priv->regmap_cfg->regmap, reg->addr,
		&reg->value);
	extern_adc_i2c_dsm_report(i2c_priv, EXT_ADC_I2C_READ, ret, NULL);
	if (ret < 0)
		hwlog_debug("%s: regmap_read failed ret = %d\n", __func__, ret);

	return ret;
}

static bool extern_adc_is_need_write_regs(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl_sequence *seq)
{
	struct extern_adc_reg_ctl_sequence *old = NULL;
	unsigned int regs_size;

	if (i2c_priv == NULL)
		return false;

	if (i2c_priv->record_wirten_seq == NULL)
		return true;

	old = i2c_priv->record_wirten_seq;
	regs_size = sizeof(struct extern_adc_reg_ctl) * seq->num;

	if ((old->num == seq->num) &&
		(memcmp(old->regs, seq->regs, regs_size) == 0))
		return false;

	return true;
}

static int extern_adc_i2c_reg_node_ops(struct extern_adc_regmap_cfg *cfg,
	struct extern_adc_reg_ctl *reg, int index, unsigned int reg_num)
{
	int ret = 0;
	int value;

	switch (reg->ctl_type) {
	case EXTERN_DAC_REG_CTL_TYPE_DELAY:
		if (reg->delay > 0)
			msleep(reg->delay);
		break;
	case EXTERN_DAC_PARAM_NODE_TYPE_SKIP:
	case EXTERN_DAC_REG_CTL_TYPE_DUMP:
		break;
	case EXTERN_DAC_PARAM_NODE_TYPE_REG_RXORW:
		hwlog_info("%s: rworw node %d/%u\n", __func__, index, reg_num);
		ret = extern_adc_regmap_xorw(cfg->regmap, reg->addr,
			reg->mask, reg->value);
		break;
	case EXTERN_DAC_REG_CTL_TYPE_R:
		ret = extern_adc_regmap_read(cfg->regmap, reg->addr, &value);
#ifndef CONFIG_FINAL_RELEASE
		hwlog_info("%s: read node %d/%u, reg[0x%x]:0x%x, ret:%d\n",
			__func__, index, reg_num, reg->addr, value, ret);
#endif
		break;
	case EXTERN_DAC_REG_CTL_TYPE_W:
		extern_adc_delay(reg->delay, index, (int)reg_num - 1);
		ret = extern_adc_regmap_complex_write(cfg, reg->addr,
			reg->mask, reg->value);
#ifndef CONFIG_FINAL_RELEASE
		hwlog_info("%s: w node %d/%u,reg[0x%x]:0x%x,ret:%d delay:%d\n",
			__func__, index, reg_num,
			reg->addr, reg->value, ret, reg->delay);
#endif
		break;
	default:
		hwlog_err("%s: invalid argument\n", __func__);
		break;
	}
	return ret;
}

static void extern_adc_i2c_get_i2c_err_info(struct i2c_err_info *info,
	unsigned int index)
{
	if (index < I2C_STATUS_B64)
		info->err_details |= ((unsigned long int)1 << index);

	info->err_count++;
}

static void extern_adc_i2c_set_record_regs(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl_sequence *seq)
{
	unsigned int size;

	if (i2c_priv == NULL) {
		hwlog_err("%s: i2c_priv invalid argument\n", __func__);
		return;
	}

	if (i2c_priv->record_wirten_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->record_wirten_seq->regs);

	extern_adc_kfree_ops(i2c_priv->record_wirten_seq);

	size = sizeof(struct extern_adc_reg_ctl_sequence);
	i2c_priv->record_wirten_seq = kzalloc(size, GFP_KERNEL);
	if (i2c_priv->record_wirten_seq == NULL)
		return;

	size = sizeof(struct extern_adc_reg_ctl) * seq->num;

	i2c_priv->record_wirten_seq->regs = kzalloc(size, GFP_KERNEL);
	if (i2c_priv->record_wirten_seq->regs == NULL) {
		extern_adc_kfree_ops(i2c_priv->record_wirten_seq);
		return;
	}
	i2c_priv->record_wirten_seq->num = seq->num;
	memcpy(i2c_priv->record_wirten_seq->regs, seq->regs, size);
}

static int extern_adc_i2c_ctrl_write_regs(struct extern_adc_i2c_priv *i2c_priv,
	struct extern_adc_reg_ctl_sequence *seq)
{
	struct extern_adc_regmap_cfg *cfg = NULL;
	int ret = 0;
	int i;
	int errno = 0;
	struct i2c_err_info info = {
		.regs_num    = 0,
		.err_count   = 0,
		.err_details = 0,
	};

	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if ((seq == NULL) || (seq->num == 0)) {
		hwlog_err("%s: reg node is invalid\n", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: chip model %s, chip_id %u, node num %u\n",
		__func__, i2c_priv->chip_model, i2c_priv->chip_id, seq->num);
#endif

	if (!extern_adc_is_need_write_regs(i2c_priv, seq)) {
		hwlog_info("%s: adc%d not need re-write the same regs\n",
			__func__, i2c_priv->chip_id);
		return 0;
	}

	cfg = i2c_priv->regmap_cfg;

	for (i = 0; i < (int)(seq->num); i++) {
		/* regmap node */
		ret = extern_adc_i2c_reg_node_ops(cfg, &(seq->regs[i]),
				i, seq->num);
		if (ret < 0) {
			hwlog_err("%s: ctl %d, reg 0x%x w/r 0x%x err, ret %d\n",
				__func__, i, seq->regs[i].addr,
				seq->regs[i].value, ret);
			extern_adc_i2c_get_i2c_err_info(&info, (unsigned int)i);
			errno = ret;
		}
	}
	info.regs_num = seq->num;
	extern_adc_i2c_dsm_report(i2c_priv, EXT_ADC_I2C_WRITE, errno, &info);
	extern_adc_i2c_set_record_regs(i2c_priv, seq);
	return ret;
}

static int extern_adc_init_regs_seq(struct extern_adc_i2c_priv *i2c_priv)
{
	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->init_regs_seq != NULL)
		return extern_adc_i2c_ctrl_write_regs(i2c_priv,
			i2c_priv->init_regs_seq);

	return 0;
}

static int  extern_adc_poweron(struct extern_adc_i2c_priv *i2c_priv)
{
	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->poweron_regs_seq != NULL)
		return extern_adc_i2c_ctrl_write_regs(i2c_priv,
			i2c_priv->poweron_regs_seq);

	return 0;
}

static int  extern_adc_poweroff(struct extern_adc_i2c_priv *i2c_priv)
{
	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->poweroff_regs_seq != NULL)
		return extern_adc_i2c_ctrl_write_regs(i2c_priv,
			i2c_priv->poweroff_regs_seq);

	return 0;
}

struct extern_adc_i2c_ctl_ops adc_i2c_ops = {
	.dump_regs  = extern_adc_i2c_dump_regs,
	.power_on  = extern_adc_poweron,
	.power_off = extern_adc_poweroff,
	.write_regs = extern_adc_i2c_ctrl_write_regs,
	.read_reg = extern_adc_i2c_ctrl_read_reg,
};

static bool extern_adc_i2c_is_reg_in_special_range(unsigned int reg_addr,
	int num, unsigned int *reg)
{
	int i;

	if ((num == 0) || (reg == NULL)) {
		hwlog_err("%s: invalid arg\n", __func__);
		return false;
	}

	for (i = 0; i < num; i++) {
		if (reg[i] == reg_addr)
			return true;
	}
	return false;
}

static struct extern_adc_regmap_cfg *extern_adc_i2c_get_regmap_cfg(
	struct device *dev)
{
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	if (dev == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return NULL;
	}
	i2c_priv = dev_get_drvdata(dev);
	if ((i2c_priv == NULL) || (i2c_priv->regmap_cfg == NULL)) {
		hwlog_err("%s: regmap_cfg invalid argument\n", __func__);
		return NULL;
	}
	return i2c_priv->regmap_cfg;
}

static bool extern_adc_i2c_writeable_reg(struct device *dev, unsigned int reg)
{
	struct extern_adc_regmap_cfg *cfg = NULL;

	cfg = extern_adc_i2c_get_regmap_cfg(dev);
	if (cfg == NULL)
		return false;
	/* config writable or unwritable, can not config in dts at same time */
	if (cfg->num_writeable > 0)
		return extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_writeable, cfg->reg_writeable);
	if (cfg->num_unwriteable > 0)
		return !extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_unwriteable, cfg->reg_unwriteable);

	return true;
}

static bool extern_adc_i2c_readable_reg(struct device *dev, unsigned int reg)
{
	struct extern_adc_regmap_cfg *cfg = NULL;

	cfg = extern_adc_i2c_get_regmap_cfg(dev);
	if (cfg == NULL)
		return false;
	/* config readable or unreadable, can not config in dts at same time */
	if (cfg->num_readable > 0)
		return extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_readable, cfg->reg_readable);
	if (cfg->num_unreadable > 0)
		return !extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_unreadable, cfg->reg_unreadable);

	return true;
}

static bool extern_adc_i2c_volatile_reg(struct device *dev, unsigned int reg)
{
	struct extern_adc_regmap_cfg *cfg = NULL;

	cfg = extern_adc_i2c_get_regmap_cfg(dev);
	if (cfg == NULL)
		return false;
	/* config volatile or unvolatile, can not config in dts at same time */
	if (cfg->num_volatile > 0)
		return extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_volatile, cfg->reg_volatile);
	if (cfg->num_unvolatile > 0)
		return !extern_adc_i2c_is_reg_in_special_range(reg,
			cfg->num_unvolatile, cfg->reg_unvolatile);

	return true;
}

static void extern_adc_i2c_free_reg_ctl(
	struct extern_adc_reg_ctl_sequence *reg_ctl)
{
	if (reg_ctl == NULL)
		return;

	extern_adc_kfree_ops(reg_ctl->regs);
	kfree(reg_ctl);
	reg_ctl = NULL;
}

static void extern_adc_print_regs_info(const char *seq_str,
	struct extern_adc_reg_ctl_sequence *reg_ctl)
{
	unsigned int print_node_num;
	unsigned int i;
	struct extern_adc_reg_ctl *reg = NULL;

	if (reg_ctl == NULL)
		return;

	print_node_num =
		(reg_ctl->num < REG_PRINT_NUM) ? reg_ctl->num : REG_PRINT_NUM;

	/* only print two registers */
	for (i = 0; i < print_node_num; i++) {
		reg = &(reg_ctl->regs[i]);
		hwlog_info("%s: %s reg_%d=0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			__func__, seq_str, i, reg->addr, reg->mask, reg->value,
			reg->delay, reg->ctl_type);
	}
}

static int extern_adc_i2c_parse_reg_ctl(
	struct extern_adc_reg_ctl_sequence **reg_ctl, struct device_node *node,
	const char *ctl_str)
{
	struct extern_adc_reg_ctl_sequence *ctl = NULL;
	int count = 0;
	int val_num;
	int ret;

	if ((node == NULL) || (ctl_str == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
	if (ctl == NULL)
		return -ENOMEM;

	ret = extern_adc_get_prop_of_u32_array(node, ctl_str,
		(u32 **)&ctl->regs, &count);
	if ((count <= 0) || (ret < 0)) {
		hwlog_err("%s: get %s failed or count is 0\n",
			__func__, ctl_str);
		ret = -EFAULT;
		goto parse_reg_ctl_err_out;
	}

	val_num = sizeof(struct extern_adc_reg_ctl) / sizeof(unsigned int);
	if ((count % val_num) != 0) {
		hwlog_err("%s: count %d %% val_num %d != 0\n",
			__func__, count, val_num);
		ret = -EFAULT;
		goto parse_reg_ctl_err_out;
	}

	ctl->num = (unsigned int)(count / val_num);
	*reg_ctl = ctl;
	return 0;

parse_reg_ctl_err_out:
	extern_adc_i2c_free_reg_ctl(ctl);
	return ret;
}

static int extern_adc_i2c_parse_dt_reg_ctl(struct i2c_client *i2c,
	struct extern_adc_reg_ctl_sequence **reg_ctl, const char *seq_str)
{
	int ret;

	if (!of_property_read_bool(i2c->dev.of_node, seq_str)) {
		hwlog_debug("%s: %s not existed, skip\n", seq_str, __func__);
		return 0;
	}
	ret = extern_adc_i2c_parse_reg_ctl(reg_ctl,
		i2c->dev.of_node, seq_str);
	if (ret < 0) {
		hwlog_err("%s: parse %s failed\n", __func__, seq_str);
		goto parse_dt_reg_ctl_err_out;
	}

	extern_adc_print_regs_info(seq_str, *reg_ctl);

parse_dt_reg_ctl_err_out:
	return ret;
}

static int extern_adc_parse_dts_sequence(struct i2c_client *i2c,
	struct extern_adc_i2c_priv *i2c_priv)
{
	int ret;

	ret = extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->version_regs_seq, "version_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->dump_regs_sequence, "dump_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->poweron_regs_seq, "poweron_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->init_regs_seq, "init_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->poweroff_regs_seq, "poweroff_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&i2c_priv->pga_volume_seq, "pga_volume_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&(i2c_priv->kctl_seqs[EXTERN_DAC_KCTL_TDM_MODE]),
		"tdm_mode_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&(i2c_priv->kctl_seqs[EXTERN_DAC_KCTL_DATA_SEL]),
		"data_sel_regs");
	ret += extern_adc_i2c_parse_dt_reg_ctl(i2c,
		&(i2c_priv->kctl_seqs[EXTERN_DAC_KCTL_MUTE_CTL]),
		"mute_ctl_regs");

	return ret;
}

static unsigned int extern_adc_i2c_get_reg_value_mask(int val_bits)
{
	unsigned int mask;

	if (val_bits == EXTERN_ADC_REG_VALUE_B16)
		mask = EXTERN_ADC_REG_VALUE_M16;
	else if (val_bits == EXTERN_ADC_REG_VALUE_B24)
		mask = EXTERN_ADC_REG_VALUE_M24;
	else if (val_bits == EXTERN_ADC_REG_VALUE_B32)
		mask = EXTERN_ADC_REG_VALUE_M32;
	else /* SMARTPAKIT_REG_VALUE_B8 or other */
		mask = EXTERN_ADC_REG_VALUE_M8;

	return mask;
}

static void extern_adc_i2c_free_regmap_cfg(struct extern_adc_regmap_cfg *cfg)
{
	if (cfg == NULL)
		return;

	extern_adc_kfree_ops(cfg->reg_writeable);
	extern_adc_kfree_ops(cfg->reg_unwriteable);
	extern_adc_kfree_ops(cfg->reg_readable);
	extern_adc_kfree_ops(cfg->reg_unreadable);
	extern_adc_kfree_ops(cfg->reg_volatile);
	extern_adc_kfree_ops(cfg->reg_unvolatile);
	extern_adc_kfree_ops(cfg->reg_defaults);

	kfree(cfg);
	cfg = NULL;
}

static int extern_adc_i2c_parse_reg_defaults(struct device_node *node,
	struct extern_adc_regmap_cfg *cfg_info)
{
	const char *reg_defaults_str = "reg_defaults";

	return extern_adc_get_prop_of_u32_array(node, reg_defaults_str,
		(u32 **)&cfg_info->reg_defaults, &cfg_info->num_defaults);
}

static int extern_adc_i2c_parse_special_regs_range(struct device_node *node,
	struct extern_adc_regmap_cfg *cfg_info)
{
	const char *reg_writeable_str   = "reg_writeable";
	const char *reg_unwriteable_str = "reg_unwriteable";
	const char *reg_readable_str    = "reg_readable";
	const char *reg_unreadable_str  = "reg_unreadable";
	const char *reg_volatile_str    = "reg_volatile";
	const char *reg_unvolatile_str  = "reg_unvolatile";
	int ret;

	cfg_info->num_writeable   = 0;
	cfg_info->num_unwriteable = 0;
	cfg_info->num_readable    = 0;
	cfg_info->num_unreadable  = 0;
	cfg_info->num_volatile    = 0;
	cfg_info->num_unvolatile  = 0;
	cfg_info->num_defaults    = 0;

	ret = extern_adc_get_prop_of_u32_array(node, reg_writeable_str,
		&cfg_info->reg_writeable, &cfg_info->num_writeable);
	ret += extern_adc_get_prop_of_u32_array(node, reg_unwriteable_str,
		&cfg_info->reg_unwriteable, &cfg_info->num_unwriteable);
	ret += extern_adc_get_prop_of_u32_array(node, reg_readable_str,
		&cfg_info->reg_readable, &cfg_info->num_readable);
	ret += extern_adc_get_prop_of_u32_array(node, reg_unreadable_str,
		&cfg_info->reg_unreadable, &cfg_info->num_unreadable);
	ret += extern_adc_get_prop_of_u32_array(node, reg_volatile_str,
		&cfg_info->reg_volatile, &cfg_info->num_volatile);
	ret += extern_adc_get_prop_of_u32_array(node, reg_unvolatile_str,
		&cfg_info->reg_unvolatile, &cfg_info->num_unvolatile);
	ret += extern_adc_i2c_parse_reg_defaults(node, cfg_info);
	return ret;
}

static int extern_adc_i2c_parse_remap_cfg(struct device_node *node,
	struct extern_adc_regmap_cfg **cfg)
{
	struct extern_adc_regmap_cfg *cfg_info = NULL;
	const char *reg_bits_str     = "reg_bits";
	const char *val_bits_str     = "val_bits";
	const char *cache_type_str   = "cache_type";
	const char *max_register_str = "max_register";
	int ret;

	cfg_info = kzalloc(sizeof(*cfg_info), GFP_KERNEL);
	if (cfg_info == NULL)
		return -ENOMEM;

	ret = of_property_read_u32(node, reg_bits_str,
		(u32 *)&cfg_info->cfg.reg_bits);
	if (ret < 0) {
		hwlog_err("%s: get reg_bits failed\n", __func__);
		ret = -EFAULT;
		goto parse_remap_err_out;
	}

	ret = of_property_read_u32(node, val_bits_str,
		(u32 *)&cfg_info->cfg.val_bits);
	if (ret < 0) {
		hwlog_err("%s: get val_bits failed\n", __func__);
		ret = -EFAULT;
		goto parse_remap_err_out;
	}
	cfg_info->value_mask = extern_adc_i2c_get_reg_value_mask(
		cfg_info->cfg.val_bits);

	ret = of_property_read_u32(node, cache_type_str,
		(u32 *)&cfg_info->cfg.cache_type);
	if ((ret < 0) || (cfg_info->cfg.cache_type > REGCACHE_FLAT)) {
		hwlog_err("%s: get cache_type failed\n", __func__);
		ret = -EFAULT;
		goto parse_remap_err_out;
	}

	ret = of_property_read_u32(node, max_register_str,
		&cfg_info->cfg.max_register);
	if (ret < 0) {
		hwlog_err("%s: get max_register failed\n", __func__);
		ret = -EFAULT;
		goto parse_remap_err_out;
	}

	ret = extern_adc_i2c_parse_special_regs_range(node, cfg_info);
	if (ret < 0)
		goto parse_remap_err_out;

	*cfg = cfg_info;
	return 0;

parse_remap_err_out:
	extern_adc_i2c_free_regmap_cfg(cfg_info);
	return ret;
}

static int extern_adc_i2c_regmap_init(struct i2c_client *i2c,
	struct extern_adc_i2c_priv *i2c_priv)
{
	const char *regmap_cfg_str = "regmap_cfg";
	struct extern_adc_regmap_cfg *cfg = NULL;
	struct device_node *node = NULL;
	int ret;
	int val_num;

	node = of_get_child_by_name(i2c->dev.of_node, regmap_cfg_str);
	if (node == NULL) {
		hwlog_debug("%s: regmap_cfg not existed, skip\n", __func__);
		return 0;
	}

	ret = extern_adc_i2c_parse_remap_cfg(node, &i2c_priv->regmap_cfg);
	if (ret < 0)
		return ret;

	cfg = i2c_priv->regmap_cfg;
	val_num = sizeof(struct reg_default) / sizeof(unsigned int);
	if (cfg->num_defaults > 0) {
		if ((cfg->num_defaults % val_num) != 0) {
			hwlog_err("%s: reg_defaults %d%%%d != 0\n",
				__func__, cfg->num_defaults, val_num);
			ret = -EFAULT;
			goto regmap_init_err_out;
		}
	}

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: regmap_cfg get w%d,%d,r%d,%d,v%d,%d,default%d\n",
		__func__, cfg->num_writeable, cfg->num_unwriteable,
		cfg->num_readable, cfg->num_unreadable, cfg->num_volatile,
		cfg->num_unvolatile, cfg->num_defaults / val_num);
#endif

	/* set num_reg_defaults */
	if (cfg->num_defaults > 0) {
		cfg->num_defaults /= val_num;
		cfg->cfg.reg_defaults = cfg->reg_defaults;
		cfg->cfg.num_reg_defaults = (unsigned int)cfg->num_defaults;
	}

	cfg->cfg.writeable_reg = extern_adc_i2c_writeable_reg;
	cfg->cfg.readable_reg  = extern_adc_i2c_readable_reg;
	cfg->cfg.volatile_reg  = extern_adc_i2c_volatile_reg;

	cfg->regmap = regmap_init_i2c(i2c, &cfg->cfg);
	if (IS_ERR(cfg->regmap)) {
		hwlog_err("%s: regmap_init_i2c regmap failed\n", __func__);
		ret = -EFAULT;
		goto regmap_init_err_out;
	}
	return 0;
regmap_init_err_out:
	extern_adc_i2c_free_regmap_cfg(i2c_priv->regmap_cfg);
	return ret;
}

static int extern_adc_i2c_regmap_deinit(struct extern_adc_i2c_priv *i2c_priv)
{
	struct extern_adc_regmap_cfg *cfg = NULL;

	if ((i2c_priv == NULL) || (i2c_priv->regmap_cfg == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}
	cfg = i2c_priv->regmap_cfg;

	regmap_exit(cfg->regmap);
	cfg->regmap = NULL;
	extern_adc_i2c_free_regmap_cfg(cfg);
	return 0;
}

static int extern_adc_check_chip_info_valid(
	struct extern_adc_i2c_priv *i2c_priv)
{
	if (i2c_priv == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->chip_id >= EXTERN_ADC_ID_MAX) {
		hwlog_err("%s: invalid chip_id %u\n", __func__,
			i2c_priv->chip_id);
		return -EFAULT;
	}

	return 0;
}

static int extern_adc_i2c_parse_dt_chip(struct i2c_client *i2c,
	struct extern_adc_i2c_priv *i2c_priv)
{
	const char *chip_id_str      = "chip_id";
	const char *chip_model_str   = "chip_model";
	int ret;

	ret = extern_adc_get_prop_of_u32_type(i2c->dev.of_node, chip_id_str,
		&i2c_priv->chip_id, true);
	ret += extern_adc_get_prop_of_str_type(i2c->dev.of_node, chip_model_str,
		&i2c_priv->chip_model);
	if (ret < 0)
		goto parse_dt_err_out;

	ret = extern_adc_check_chip_info_valid(i2c_priv);
	if (ret < 0)
		goto parse_dt_err_out;

	return 0;

parse_dt_err_out:
	return ret;
}

static void extern_adc_i2c_free(struct extern_adc_i2c_priv *i2c_priv)
{
	if (i2c_priv == NULL) {
		hwlog_err("%s: i2c_priv invalid argument\n", __func__);
		return;
	}

	if (i2c_priv->regmap_cfg != NULL)
		extern_adc_i2c_regmap_deinit(i2c_priv);

	if (i2c_priv->init_regs_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->init_regs_seq->regs);

	if (i2c_priv->poweron_regs_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->poweron_regs_seq->regs);

	if (i2c_priv->poweroff_regs_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->poweroff_regs_seq->regs);

	if (i2c_priv->dump_regs_sequence != NULL)
		extern_adc_kfree_ops(i2c_priv->dump_regs_sequence->regs);

	if (i2c_priv->version_regs_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->version_regs_seq->regs);

	if (i2c_priv->record_wirten_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->record_wirten_seq->regs);

	if (i2c_priv->pga_volume_seq != NULL)
		extern_adc_kfree_ops(i2c_priv->pga_volume_seq->regs);

	extern_adc_kfree_ops(i2c_priv->init_regs_seq);
	extern_adc_kfree_ops(i2c_priv->poweron_regs_seq);
	extern_adc_kfree_ops(i2c_priv->poweroff_regs_seq);
	extern_adc_kfree_ops(i2c_priv->dump_regs_sequence);
	extern_adc_kfree_ops(i2c_priv->version_regs_seq);
	extern_adc_kfree_ops(i2c_priv->record_wirten_seq);
	extern_adc_kfree_ops(i2c_priv->pga_volume_seq);

	kfree(i2c_priv);
}

static void extern_adc_i2c_reset_priv_data(
	struct extern_adc_i2c_priv *i2c_priv)
{
	if (i2c_priv == NULL)
		return;

	i2c_priv->probe_completed = 0;
}

static int extern_adc_i2c_read_chip_version(
	struct extern_adc_i2c_priv *i2c_priv)
{
	struct regmap *regmap = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;
	unsigned int reg_addr;
	unsigned int value = 0;
	int ret;
	int i;
	unsigned int num;

	if (extern_adc_check_i2c_regmap_cfg(i2c_priv) < 0) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->version_regs_seq == NULL)
		return 0;

	seq = i2c_priv->version_regs_seq;
	num = seq->num;
	regmap = i2c_priv->regmap_cfg->regmap;

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: adc %u, seq num %u read version\n",
		__func__, i2c_priv->chip_id, seq->num);
#endif
	for (i = 0; i < (int)num; i++) {
		regs = &(seq->regs[i]);
		reg_addr = regs->addr;
		hwlog_info("%s: adc %u, seq num %u  read version reg:0x%x\n",
			__func__, i2c_priv->chip_id, seq->num, reg_addr);
		ret = extern_adc_regmap_read(regmap, reg_addr, &value);
		extern_adc_i2c_dsm_report(i2c_priv, EXT_ADC_I2C_READ, ret,
			NULL);
		if (ret < 0) {
			hwlog_err("%s: %s read version regs:0x%x failed, %d\n",
				__func__, i2c_priv->chip_model, reg_addr, ret);
			return ret;
		}
		hwlog_info("%s: adc %u, r reg 0x%x = 0x%x\n", __func__,
			i2c_priv->chip_id, reg_addr, value);

		if (value != regs->chip_version) {
			hwlog_err("%s: adc %u, r reg 0x%x = 0x%x , 0x%x\n",
				__func__, i2c_priv->chip_id, reg_addr,
				value, regs->chip_version);
			return -EINVAL;
		}
	}
	return ret;
}

static int extern_adc_i2c_before_probe_check(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	if (extern_adc_get_misc_init_flag() == 0) {
		hwlog_info("%s: this driver need probe_defer\n", __func__);
		return -EPROBE_DEFER;
	}

	if (i2c == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	hwlog_info("%s: device %s, addr=0x%x, flags=0x%x\n", __func__,
		id->name, i2c->addr, i2c->flags);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		hwlog_err("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int extern_adc_i2c_parse_dt_init(struct i2c_client *i2c,
	struct extern_adc_i2c_priv *i2c_priv)
{
	int ret;

	ret = extern_adc_parse_dts_sequence(i2c, i2c_priv);
	if (ret < 0)
		return ret;

	/* int regmap */
	ret = extern_adc_i2c_regmap_init(i2c, i2c_priv);
	if (ret < 0)
		return ret;

	ret = extern_adc_i2c_read_chip_version(i2c_priv);
	if (ret < 0)
		return ret;

	/* register i2c device to extern adc device */
	ret = extern_adc_register_i2c_device(i2c_priv);
	if (ret < 0)
		return ret;

	ret = extern_adc_init_regs_seq(i2c_priv);
	if (ret < 0)
		return ret;

	return 0;
}

static int extern_adc_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	int ret;
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	ret = extern_adc_i2c_before_probe_check(i2c, id);
	if (ret < 0)
		return ret;

	i2c_priv = kzalloc(sizeof(*i2c_priv), GFP_KERNEL);
	if (i2c_priv == NULL)
		return -ENOMEM;

	extern_adc_i2c_reset_priv_data(i2c_priv);
	ret = extern_adc_i2c_parse_dt_chip(i2c, i2c_priv);
	if (ret < 0)
		goto probe_err_out;

	if (extern_adc_i2c_probe_skip[i2c_priv->chip_id]) {
		hwlog_info("%s: chip_id = %u has been probed success, skip\n",
			__func__, i2c_priv->chip_id);
		ret = 0;
		goto skip_probe;
	}

	i2c_priv->dev = &i2c->dev;
	i2c_priv->i2c = i2c;
	i2c_set_clientdata(i2c, i2c_priv);
	dev_set_drvdata(&i2c->dev, i2c_priv);
	ret = extern_adc_i2c_parse_dt_init(i2c, i2c_priv);
	if (ret < 0)
		goto probe_err_out;

	extern_adc_register_i2c_ctl_ops(&adc_i2c_ops);
	i2c_priv->probe_completed = 1;
	extern_adc_i2c_probe_skip[i2c_priv->chip_id] = true;
	hwlog_info("%s: end success\n", __func__);
	return 0;

probe_err_out:
	hwlog_err("%s: end failed\n", __func__);
skip_probe:
	extern_adc_i2c_free(i2c_priv);
	return ret;
}

static int extern_adc_i2c_remove(struct i2c_client *i2c)
{
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	hwlog_info("%s: remove\n", __func__);
	if (i2c == NULL) {
		pr_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	i2c_priv = i2c_get_clientdata(i2c);
	if (i2c_priv == NULL) {
		hwlog_err("%s: i2c_priv invalid\n", __func__);
		return -EINVAL;
	}

	i2c_set_clientdata(i2c, NULL);
	dev_set_drvdata(&i2c->dev, NULL);

	/* deregister i2c device */
	extern_adc_deregister_i2c_device(i2c_priv);
	extern_adc_deregister_i2c_ctl_ops();

	extern_adc_i2c_free(i2c_priv);
	return 0;
}

static void extern_adc_i2c_shutdown(struct i2c_client *i2c)
{
}

#ifdef CONFIG_PM
static int extern_adc_i2c_suspend(struct device *dev)
{
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	if (dev == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	i2c_priv = dev_get_drvdata(dev);
	if (i2c_priv == NULL)
		return 0;

	if ((i2c_priv->regmap_cfg != NULL) &&
		(i2c_priv->regmap_cfg->regmap != NULL) &&
		(i2c_priv->regmap_cfg->cfg.cache_type == REGCACHE_RBTREE))
		regcache_cache_only(i2c_priv->regmap_cfg->regmap, (bool)true);

	return 0;
}

static int extern_adc_i2c_resume(struct device *dev)
{
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	if (dev == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	i2c_priv = dev_get_drvdata(dev);
	if (i2c_priv == NULL)
		return 0;

	if ((i2c_priv->regmap_cfg != NULL) &&
		(i2c_priv->regmap_cfg->regmap != NULL) &&
		(i2c_priv->regmap_cfg->cfg.cache_type == REGCACHE_RBTREE)) {
		regcache_cache_only(i2c_priv->regmap_cfg->regmap, (bool)false);
		regcache_sync(i2c_priv->regmap_cfg->regmap);
	}

	return 0;
}
#else
#define extern_adc_i2c_suspend NULL /* function pointer */
#define extern_adc_i2c_resume  NULL /* function pointer */
#endif /* CONFIG_PM */

static const struct dev_pm_ops extern_adc_i2c_pm_ops = {
	.suspend = extern_adc_i2c_suspend,
	.resume  = extern_adc_i2c_resume,
};

static const struct i2c_device_id extern_adc_i2c_id[] = {
	{ "extern_adc_i2c", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, extern_adc_i2c_id);

static const struct of_device_id extern_adc_i2c_match[] = {
	{ .compatible = "huawei,extern_adc_i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, extern_adc_i2c_match);

static struct i2c_driver extern_adc_i2c_driver = {
	.driver = {
		.name           = "extern_adc_i2c",
		.owner          = THIS_MODULE,
		.pm             = &extern_adc_i2c_pm_ops,
		.of_match_table = of_match_ptr(extern_adc_i2c_match),
	},
	.probe    = extern_adc_i2c_probe,
	.remove   = extern_adc_i2c_remove,
	.shutdown = extern_adc_i2c_shutdown,
	.id_table = extern_adc_i2c_id,
};

static int __init extern_adc_i2c_init(void)
{
	return i2c_add_driver(&extern_adc_i2c_driver);
}

static void __exit extern_adc_i2c_exit(void)
{
	i2c_del_driver(&extern_adc_i2c_driver);
}

fs_initcall(extern_adc_i2c_init);
module_exit(extern_adc_i2c_exit);

MODULE_DESCRIPTION("extern adc i2c driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_LICENSE("GPL v2");

