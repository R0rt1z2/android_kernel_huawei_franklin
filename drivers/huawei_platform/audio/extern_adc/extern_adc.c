/*
 * extern_adc.c
 *
 * extern adc driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>
#include <huawei_platform/log/hw_log.h>
#include <dsm/dsm_pub.h>

#ifdef CONFIG_HUAWEI_DSM_AUDIO_MODULE
#define CONFIG_HUAWEI_DSM_AUDIO
#endif
#ifdef CONFIG_HUAWEI_DSM_AUDIO
#include <dsm_audio/dsm_audio.h>
#endif

#define HWLOG_TAG extern_adc
HWLOG_REGIST();

#define ADC_POWERON     1
#define ADC_POWEROFF    0

#define BYTE_LEN    8

typedef int (*extern_adc_ops_fun)(struct extern_adc_i2c_priv *priv);

static struct extern_adc_priv *extern_adc_priv_data = NULL;

/* misc device 0 not init completed, 1 init completed */
static int extern_adc_init_flag = 0;

#define EXT_ADC_PGA_VOLUME_SINGLE(xname, chip_id, pga_volum_index) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
  .info = snd_extern_adc_volume_info, \
  .get = snd_extern_adc_get_volume, .put = snd_extern_adc_put_volume, \
  .private_value = (chip_id) | ((pga_volum_index) << 16)}


#define EXT_ADC_KCTL_SINGLE(xname, chip_id, kctl_index) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
  .info = snd_extern_adc_kctl_info, \
  .get = snd_extern_adc_get_kctl, .put = snd_extern_adc_put_kctl, \
  .private_value = (chip_id) | ((kctl_index) << 16)}

struct extern_adc_priv *extern_adc_get_misc_priv(void)
{
	return extern_adc_priv_data;
}

int extern_adc_get_misc_init_flag(void)
{
	return extern_adc_init_flag;
}

static void extern_adc_map_i2c_addr_to_chip_id(
	struct extern_adc_i2c_priv *i2c_priv, unsigned int id)
{
	unsigned int addr;
	unsigned char chip_id;

	if (i2c_priv->i2c == NULL)
		return;

	chip_id = (unsigned char)id;
	addr = i2c_priv->i2c->addr;

	if (addr < EXTERN_ADC_I2C_ADDR_ARRAY_MAX)
		extern_adc_priv_data->i2c_addr_to_pa_index[addr] = chip_id;
}

int extern_adc_register_i2c_device(struct extern_adc_i2c_priv *i2c_priv)
{
	unsigned int str_length;

	if ((extern_adc_priv_data == NULL) || (i2c_priv == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (i2c_priv->chip_id >= extern_adc_priv_data->adc_num) {
		hwlog_err("%s: error, chip_id %u >= adc_num %u\n", __func__,
			i2c_priv->chip_id, extern_adc_priv_data->adc_num);
		extern_adc_priv_data->chip_register_failed = true;
		return -EINVAL;
	}

	if (extern_adc_priv_data->i2c_priv[i2c_priv->chip_id] != NULL) {
		hwlog_err("%s: chip_id reduplicated error\n", __func__);
		extern_adc_priv_data->chip_register_failed = true;
		return -EINVAL;
	}

	i2c_priv->priv_data = (void *)extern_adc_priv_data;
	extern_adc_priv_data->i2c_num++;
	extern_adc_priv_data->i2c_priv[i2c_priv->chip_id] = i2c_priv;

	str_length = (strlen(i2c_priv->chip_model) < EXTERN_ADC_ID_MAX) ?
		strlen(i2c_priv->chip_model) : (EXTERN_ADC_NAME_MAX - 1);
	strncpy(extern_adc_priv_data->chip_model_list[i2c_priv->chip_id],
		i2c_priv->chip_model, str_length);

	extern_adc_map_i2c_addr_to_chip_id(i2c_priv, i2c_priv->chip_id);

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: i2c_priv registered, success\n", __func__);
#endif

	return 0;
}

int extern_adc_deregister_i2c_device(struct extern_adc_i2c_priv *i2c_priv)
{
	int i;
	unsigned int chip_id;

	if ((extern_adc_priv_data == NULL) || (i2c_priv == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < (int)extern_adc_priv_data->adc_num; i++) {
		if (extern_adc_priv_data->i2c_priv[i] == NULL)
			continue;
		chip_id = extern_adc_priv_data->i2c_priv[i]->chip_id;
		if (i2c_priv->chip_id != chip_id)
			continue;

		extern_adc_map_i2c_addr_to_chip_id(i2c_priv,
			EXTERN_ADC_INVALID_PA_INDEX);
		i2c_priv->priv_data = NULL;

		extern_adc_priv_data->i2c_num--;
		extern_adc_priv_data->i2c_priv[i] = NULL;
		hwlog_info("%s: i2c_priv deregistered, success\n", __func__);
		break;
	}

	return 0;
}

void extern_adc_register_i2c_ctl_ops(struct extern_adc_i2c_ctl_ops *ops)
{
	if ((extern_adc_priv_data == NULL) || (ops == NULL)) {
		hwlog_err("%s: ioctl_ops register failed\n", __func__);
		return;
	}

	if (extern_adc_priv_data->ioctl_ops != NULL) {
#ifndef CONFIG_FINAL_RELEASE
		hwlog_debug("%s: ioctl_ops had registered, skip\n", __func__);
#endif
		return;
	}

	extern_adc_priv_data->ioctl_ops = ops;
#ifndef CONFIG_FINAL_RELEASE
	hwlog_debug("%s: ioctl_ops registered, success\n", __func__);
#endif
}

void extern_adc_deregister_i2c_ctl_ops(void)
{
	if (extern_adc_priv_data == NULL) {
		hwlog_err("%s: ioctl_ops deregister failed\n", __func__);
		return;
	}

	if (extern_adc_priv_data->i2c_num != 0) {
		hwlog_debug("%s: skip deregister ioctl_ops\n", __func__);
		return;
	}

	extern_adc_priv_data->ioctl_ops = NULL;
	hwlog_info("%s: ioctl_ops deregistered, success\n", __func__);
}

static int extern_adc_dump_chips(struct extern_adc_priv *adc_priv)
{
	struct extern_adc_i2c_priv *i2c_priv_p = NULL;
	int ret = 0;
	int i;

	if (adc_priv == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if ((adc_priv->ioctl_ops == NULL) ||
		(adc_priv->ioctl_ops->dump_regs == NULL)) {
		hwlog_err("%s: i2c dump_regs ops is NULL\n", __func__);
		return -ECHILD;
	}

	mutex_lock(&adc_priv->dump_regs_lock);
	for (i = 0; i < (int)adc_priv->adc_num; i++) {
		i2c_priv_p = adc_priv->i2c_priv[i];
		ret = adc_priv->ioctl_ops->dump_regs(i2c_priv_p);
		if (ret < 0)
			break;
	}
	mutex_unlock(&adc_priv->dump_regs_lock);

	return ret;
}

static void extern_adc_get_model_from_i2c_cfg(char *dst,
	struct extern_adc_priv *adc_priv, unsigned int dst_len)
{
	char report_tmp[EXTERN_ADC_NAME_MAX] = {0};
	unsigned int model_len;
	unsigned int tmp_len;
	int i;

	if ((dst == NULL) || (adc_priv == NULL) || (adc_priv->adc_num == 0))
		return;

	for (i = 0; i < adc_priv->adc_num; i++) {
		if (i < (adc_priv->adc_num - 1))
			snprintf(report_tmp,
				(unsigned long)EXTERN_ADC_NAME_MAX,
				"%s_", adc_priv->chip_model_list[i]);
		else
			snprintf(report_tmp,
				(unsigned long)EXTERN_ADC_NAME_MAX,
				"%s", adc_priv->chip_model_list[i]);

		model_len = dst_len - (strlen(dst) + 1);
		tmp_len = strlen(report_tmp);
		model_len = (tmp_len < model_len) ? tmp_len : model_len;
		strncat(dst, report_tmp, model_len);
	}
}

static int get_extern_adc_info_from_dts_config(struct extern_adc_priv *adc_priv,
	struct extern_adc_info *info)
{
	size_t chip_model_len;
	unsigned int i;

	if ((adc_priv == NULL) || (info == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	info->adc_num = adc_priv->i2c_num;
	for (i = 0; i < EXTERN_ADC_ID_MAX; i++) {
		chip_model_len = strlen(adc_priv->chip_model_list[i]);
		chip_model_len = (chip_model_len < EXTERN_ADC_NAME_MAX) ?
			chip_model_len : (EXTERN_ADC_NAME_MAX - 1);
		strncpy(info->chip_model_list[i],
			adc_priv->chip_model_list[i], chip_model_len);
	}

	extern_adc_get_model_from_i2c_cfg(info->chip_model,
		adc_priv, EXTERN_ADC_NAME_MAX);
	hwlog_info("%s:adc chip_model:%s\n", __func__, info->chip_model);

	return 0;
}

static int extern_adc_get_info(struct extern_adc_priv *adc_priv)
{
	struct extern_adc_info info;

#ifndef CONFIG_FINAL_RELEASE
	pr_info("%s: enter\n", __func__);
#endif
	if (adc_priv == NULL) {
		pr_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	memset(&info, 0, sizeof(info));
	if (get_extern_adc_info_from_dts_config(adc_priv, &info) < 0)
		return -EINVAL;

	hwlog_info("%s: adc_num=%u, chip_model=%s\n",
		__func__, info.adc_num, info.chip_model);

	if (info.adc_num != adc_priv->adc_num)
		hwlog_info("%s: NOTICE I2C_NUM %u != ADC_NUM %u\n", __func__,
			info.adc_num, adc_priv->adc_num);

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: enter end\n", __func__);
#endif
	return 0;
}

static int extern_adc_write_regs_to_multi_chips(
	struct extern_adc_priv *adc_priv, extern_adc_ops_fun ops_fun,
	unsigned int status)
{
	struct extern_adc_i2c_priv *i2c_priv_p = NULL;
	int ret = 0;
	int i;

	if (adc_priv == NULL || ops_fun == NULL) {
		hwlog_err("%s: write_regs is invalid\n", __func__);
		return -ECHILD;
	}

	mutex_lock(&adc_priv->i2c_ops_lock);
	for (i = 0; i < (int)adc_priv->adc_num; i++) {
		i2c_priv_p = adc_priv->i2c_priv[i];
		if (i2c_priv_p == NULL) {
			hwlog_err("%s: i2c_priv is NULL\n", __func__);
			continue;
		}

		ret = ops_fun(i2c_priv_p);
		if (ret < 0)
			break;

		adc_priv->power_status[i] = status;
	}
	mutex_unlock(&adc_priv->i2c_ops_lock);

	return ret;
}

#ifdef CONFIG_NEED_WRITE_SINGLE_ADC
static int extern_write_regs_to_single_adc(
	struct extern_adc_priv *adc_priv, extern_adc_ops_fun ops_fun,
	unsigned int id, unsigned int status)
{
	struct extern_adc_i2c_priv *i2c_priv = NULL;
	unsigned int chip_id = 0;
	int ret = 0;

	if (adc_priv == NULL || ops_fun == NULL) {
		hwlog_err("%s: write_regs is invalid\n", __func__);
		return -ECHILD;
	}

	if (adc_priv->adc_num >= (id + 1))
		chip_id = id;

	mutex_lock(&adc_priv->i2c_ops_lock);
	i2c_priv = adc_priv->i2c_priv[chip_id];
	if (i2c_priv == NULL) {
		hwlog_err("%s: i2c_priv is NULL\n", __func__);
		goto single_adc_exit;
	}

	ret = ops_fun(i2c_priv);
	if (ret < 0)
		goto single_adc_exit;

	adc_priv->power_status[i] = status;
single_adc_exit:
	mutex_unlock(&adc_priv->i2c_ops_lock);
	return ret;
}
#endif

int extern_adc_get_prop_of_u32_array(struct device_node *node,
	const char *propname, u32 **value, int *num)
{
	u32 *array = NULL;
	int ret;
	int count = 0;

	if ((node == NULL) || (propname == NULL) || (value == NULL) ||
		(num == NULL)) {
		pr_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_bool(node, propname))
		count = of_property_count_elems_of_size(node,
			propname, (int)sizeof(u32));

	if (count == 0) {
		hwlog_debug("%s: %s not existed, skip\n", __func__, propname);
		return 0;
	}

	array = kzalloc(sizeof(u32) * count, GFP_KERNEL);
	if (array == NULL)
		return -ENOMEM;

	ret = of_property_read_u32_array(node, propname, array,
		(size_t)(long)count);
	if (ret < 0) {
		hwlog_err("%s: get %s array failed\n", __func__, propname);
		ret = -EFAULT;
		goto get_prop_err_out;
	}

	*value = array;
	*num = count;
	return 0;

get_prop_err_out:
	extern_adc_kfree_ops(array);
	return ret;
}

int extern_adc_get_prop_of_u32_type(struct device_node *node,
	const char *key_str, unsigned int *type, bool is_requisite)
{
	int ret = 0;

	if ((node == NULL) || (key_str == NULL) || (type == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	if (!of_property_read_bool(node, key_str)) {
		if (is_requisite)
			ret = -EINVAL;
		hwlog_debug("%s: %s is not config in dts\n", __func__, key_str);
		return ret;
	}

	ret = of_property_read_u32(node, key_str, type);
	if (ret < 0)
		hwlog_err("%s: get %s error\n", __func__, key_str);

	return ret;
}

int extern_adc_get_prop_of_str_type(struct device_node *node,
	const char *key_str, const char **dst)
{
	int ret = 0;

	if ((node == NULL) || (key_str == NULL) || (dst == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}
	if (of_property_read_bool(node, key_str)) {
		ret = of_property_read_string(node, key_str, dst);
		if (ret < 0) {
			hwlog_err("%s: get %s failed\n", __func__, key_str);
			return ret;
		}
#ifndef CONFIG_FINAL_RELEASE
		hwlog_debug("%s: %s=%s\n", __func__, key_str, *dst);
#endif
	} else {
		hwlog_debug("%s: %s not existed, skip\n", __func__, key_str);
	}

	return ret;
}

static int extern_adc_get_fist_bit_from_mask(unsigned int mask)
{
	unsigned int size = sizeof(mask) * BYTE_LEN;
	int i;

	for (i = 0; i < size; i++) {
		if ((mask >> i) & 0x1)
			break;
	}

	return i;
}

static int extern_adc_check_adc_volume_info_valid(unsigned long private_value)
{
	int index = private_value & 0xff;
	int pga_reg_index = (private_value >> 16) & 0xff;

	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;

	if (extern_adc_priv_data == NULL) {
		hwlog_info("%s: extern_adc_priv_data is NULL\n", __func__);
		return -EINVAL;
	}

	if (index >= EXTERN_ADC_ID_MAX) {
		hwlog_info("%s: index %d is invaild\n", __func__, index);
		return -EINVAL;
	}

	i2c_priv = extern_adc_priv_data->i2c_priv[index];
	if (i2c_priv == NULL) {
		hwlog_info("%s: i2c_priv[%d] is NULL\n", __func__, index);
		return -EINVAL;
	}

	seq = i2c_priv->pga_volume_seq;
	if (seq == NULL) {
		hwlog_err("%s: seq is NULL\n", __func__);
		return -EINVAL;
	}

	if (pga_reg_index >= seq->num ||
		pga_reg_index >= EXTERN_DAC_PGA_VOLUME_MAX) {
		hwlog_err("%s: index %d is invaild\n", __func__, pga_reg_index);
		return -EINVAL;
	}

	return 0;
}

static int snd_extern_adc_volume_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int chip_id;
	int pga_reg_index;
	int ret;

	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	if (kcontrol == NULL || uinfo == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return -1;
	}

	ret = extern_adc_check_adc_volume_info_valid(kcontrol->private_value);
	if (ret < 0) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 0;
		return 0;
	}

	chip_id = kcontrol->private_value & 0xff;
	pga_reg_index = (kcontrol->private_value >> 16) & 0xff;
	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->pga_volume_seq;
	regs = &(seq->regs[pga_reg_index]);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = regs->min_value;
	uinfo->value.integer.max = regs->max_value;
	return 0;
}

static int snd_extern_adc_get_volume(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	unsigned int chip_id;
	unsigned int pga_reg_index;
	unsigned int shif_bit;
	int ret;
	struct extern_adc_reg_ctl pga_regs;
	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return -1;
	}

	ret = extern_adc_check_adc_volume_info_valid(kcontrol->private_value);
	if (ret < 0) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	chip_id = kcontrol->private_value & 0xff;
	pga_reg_index = (kcontrol->private_value >> 16) & 0xff;
	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->pga_volume_seq;
	regs = &(seq->regs[pga_reg_index]);
	mutex_lock(&extern_adc_priv_data->i2c_ops_lock);
	pga_regs.addr = regs->addr;
	pga_regs.mask = regs->mask;
	pga_regs.delay = regs->delay;
	pga_regs.ctl_type = EXTERN_DAC_REG_CTL_TYPE_R;
	shif_bit = extern_adc_get_fist_bit_from_mask(pga_regs.mask);

	if (extern_adc_priv_data->ioctl_ops == NULL ||
		extern_adc_priv_data->ioctl_ops->read_reg == NULL) {
		hwlog_err("%s: ioctl_ops is NULL\n", __func__);
		ucontrol->value.integer.value[0] =
			extern_adc_priv_data->pag_volume[chip_id][pga_reg_index];
		goto exit_get_volume;
	}

	ret = extern_adc_priv_data->ioctl_ops->read_reg(i2c_priv, &pga_regs);
	if (ret < 0) {
		pr_err("%s: read adc:%d, %d, reg failed",
			__func__, chip_id, pga_reg_index);
		ucontrol->value.integer.value[0] =
			extern_adc_priv_data->pag_volume[chip_id][pga_reg_index];
	} else {
		ucontrol->value.integer.value[0] =
			(pga_regs.value & pga_regs.mask) >> shif_bit;
	}
exit_get_volume:
	mutex_unlock(&extern_adc_priv_data->i2c_ops_lock);
	return 0;
}

static int snd_extern_adc_put_volume(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	unsigned int chip_id;
	unsigned int pga_reg_index;
	unsigned int shif_bit;
	int ret;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	struct extern_adc_reg_ctl_sequence pga_seq;
	struct extern_adc_reg_ctl pga_regs;
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return 0;
	}
#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: chip_id %u, pga_reg_index %u\n", __func__,
		chip_id, pga_reg_index);
#endif
	ret = extern_adc_check_adc_volume_info_valid(kcontrol->private_value);
	if (ret < 0)
		return 0;

	chip_id = kcontrol->private_value & 0xff;
	pga_reg_index = (kcontrol->private_value >> 16) & 0xff;
	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->pga_volume_seq;
	regs = &(seq->regs[pga_reg_index]);

	mutex_lock(&extern_adc_priv_data->i2c_ops_lock);
	pga_regs.addr = regs->addr;
	pga_regs.mask = regs->mask;
	pga_regs.delay = regs->delay;

	shif_bit = extern_adc_get_fist_bit_from_mask(pga_regs.mask);

	pga_regs.value = ucontrol->value.integer.value[0] << shif_bit;
	pga_regs.ctl_type = EXTERN_DAC_REG_CTL_TYPE_W;

	pga_seq.num = 1;
	pga_seq.regs = &pga_regs;
	if (extern_adc_priv_data->ioctl_ops == NULL ||
		extern_adc_priv_data->ioctl_ops->write_regs == NULL) {
		hwlog_err("%s: ioctl_ops is NULL\n", __func__);
		goto exit_put_volume;
	}
	extern_adc_priv_data->ioctl_ops->write_regs(i2c_priv, &pga_seq);

exit_put_volume:
	extern_adc_priv_data->pag_volume[chip_id][pga_reg_index] =
		ucontrol->value.integer.value[0];
	mutex_unlock(&extern_adc_priv_data->i2c_ops_lock);
	return 0;
}

#define EXTERN_ADC_PGA_VOLUME_KCONTROLS \
	EXT_ADC_KCTL_SINGLE("EXT_ADC0_LEFT_PGA_VOLUME", 0, 0), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC0_RIGHT_PGA_VOLUME", 0, 1), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC1_LEFT_PGA_VOLUME", 1, 0), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC1_RIGHT_PGA_VOLUME", 1, 1), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC2_LEFT_PGA_VOLUME", 2, 0), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC2_RIGHT_PGA_VOLUME", 2, 1), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC3_LEFT_PGA_VOLUME", 3, 0), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC3_RIGHT_PGA_VOLUME", 3, 1), \

static int extern_adc_check_adc_kctl_info_valid(unsigned long private_value)
{
	int chip_id = private_value & 0xff;
	int kctl_index = (private_value >> 16) & 0xff;

	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;

	if (extern_adc_priv_data == NULL) {
		hwlog_info("%s: priv_data is NULL, index:%d, kctl_index:%d\n",
			__func__, chip_id, kctl_index);
		return -EINVAL;
	}

	if (chip_id >= EXTERN_ADC_ID_MAX) {
		hwlog_info("%s: index %d is invaild\n", __func__, chip_id);
		return -EINVAL;
	}

	if (kctl_index >= EXTERN_DAC_KCTL_MAX) {
		hwlog_err("%s: kctl_index %d is invaild\n",
			__func__, kctl_index);
		return -EINVAL;
	}

	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	if (i2c_priv == NULL) {
		hwlog_info("%s: i2c_priv[%d] is NULL\n", __func__, chip_id);
		return -EINVAL;
	}

	seq = i2c_priv->kctl_seqs[kctl_index];
	if (seq == NULL) {
		hwlog_err("%s: seq is NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int snd_extern_adc_kctl_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int chip_id;
	int kctl_index;
	int ret;

	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	if (kcontrol == NULL || uinfo == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return -1;
	}

	ret = extern_adc_check_adc_kctl_info_valid(kcontrol->private_value);
	if (ret < 0) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 0;
		return 0;
	}

	chip_id = kcontrol->private_value & 0xff;
	kctl_index = (kcontrol->private_value >> 16) & 0xff;
	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->kctl_seqs[kctl_index];
	/* regs num is alway is 1 */
	regs = seq->regs;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = regs->min_value;
	uinfo->value.integer.max = regs->max_value;
	return 0;
}

static int snd_extern_adc_get_kctl(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	unsigned int chip_id;
	unsigned int kctl_index;
	unsigned int shif_bit;
	int ret;
	struct extern_adc_reg_ctl pga_regs;
	struct extern_adc_i2c_priv *i2c_priv = NULL;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return -1;
	}

	ret = extern_adc_check_adc_kctl_info_valid(kcontrol->private_value);
	if (ret < 0) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	chip_id = kcontrol->private_value & 0xff;
	kctl_index = (kcontrol->private_value >> 16) & 0xff;
	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->kctl_seqs[kctl_index];
	/* regs num is alway is 1 */
	regs = seq->regs;

	mutex_lock(&extern_adc_priv_data->i2c_ops_lock);
	pga_regs.addr = regs->addr;
	pga_regs.mask = regs->mask;
	pga_regs.delay = regs->delay;
	pga_regs.ctl_type = EXTERN_DAC_REG_CTL_TYPE_R;
	shif_bit = extern_adc_get_fist_bit_from_mask(pga_regs.mask);

	if (extern_adc_priv_data->ioctl_ops == NULL ||
		extern_adc_priv_data->ioctl_ops->read_reg == NULL) {
		hwlog_err("%s: ioctl_ops is NULL\n", __func__);
		ucontrol->value.integer.value[0] =
			extern_adc_priv_data->kctl_status[chip_id][kctl_index];
		goto exit_get_kctl;
	}

	ret = extern_adc_priv_data->ioctl_ops->read_reg(i2c_priv, &pga_regs);
	if (ret < 0) {
		hwlog_err("%s: read kctl:%d reg failed", __func__, kctl_index);
		ucontrol->value.integer.value[0] =
			extern_adc_priv_data->kctl_status[chip_id][kctl_index];
	} else {
		hwlog_info("%s: read kctl:%d reg:0x%x value:0x%x mask:0x%x",
			__func__, kctl_index, pga_regs.addr,
			pga_regs.value, pga_regs.mask);
		ucontrol->value.integer.value[0] =
			(pga_regs.value & pga_regs.mask) >> shif_bit;
	}
exit_get_kctl:
	mutex_unlock(&extern_adc_priv_data->i2c_ops_lock);
	return 0;
}

static int snd_extern_adc_put_kctl(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	unsigned int chip_id;
	unsigned int kctl_index;
	unsigned int shif_bit;
	int ret;
	struct extern_adc_reg_ctl_sequence *seq = NULL;
	struct extern_adc_reg_ctl *regs = NULL;

	struct extern_adc_reg_ctl_sequence pga_seq;
	struct extern_adc_reg_ctl pga_regs;
	struct extern_adc_i2c_priv *i2c_priv = NULL;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return 0;
	}

	ret = extern_adc_check_adc_kctl_info_valid(kcontrol->private_value);
	if (ret < 0)
		return 0;

	chip_id = kcontrol->private_value & 0xff;
	kctl_index = (kcontrol->private_value >> 16) & 0xff;

	i2c_priv = extern_adc_priv_data->i2c_priv[chip_id];
	seq = i2c_priv->kctl_seqs[kctl_index];
	/* regs num is alway is 1 */
	regs = seq->regs;

	mutex_lock(&extern_adc_priv_data->i2c_ops_lock);
	pga_regs.addr = regs->addr;
	pga_regs.mask = regs->mask;
	pga_regs.delay = regs->delay;
	shif_bit = extern_adc_get_fist_bit_from_mask(pga_regs.mask);

	pga_regs.value = ucontrol->value.integer.value[0] << shif_bit;
	pga_regs.ctl_type = EXTERN_DAC_REG_CTL_TYPE_W;
	pga_seq.num = 1;
	pga_seq.regs = &pga_regs;

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: chip_id %u,index %u,reg:0x%x,value:0x%x,mask:0x%x\n",
		__func__, chip_id, kctl_index, pga_regs.addr,
		pga_regs.value, pga_regs.mask);
#endif

	if (extern_adc_priv_data->ioctl_ops == NULL ||
		extern_adc_priv_data->ioctl_ops->write_regs == NULL) {
		hwlog_err("%s: ioctl_ops is NULL\n", __func__);
		goto exit_put_kctl;
	}
	extern_adc_priv_data->ioctl_ops->write_regs(i2c_priv, &pga_seq);

exit_put_kctl:
	extern_adc_priv_data->kctl_status[chip_id][kctl_index] =
		ucontrol->value.integer.value[0];
	mutex_unlock(&extern_adc_priv_data->i2c_ops_lock);
	return 0;
}

#define EXTERN_ADC_PGA_KCTL_KCONTROLS \
	EXT_ADC_KCTL_SINGLE("EXT_ADC0_TDM_MODE", 0, EXTERN_DAC_KCTL_TDM_MODE), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC1_TDM_MODE", 1, EXTERN_DAC_KCTL_TDM_MODE), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC2_TDM_MODE", 2, EXTERN_DAC_KCTL_TDM_MODE), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC3_TDM_MODE", 3, EXTERN_DAC_KCTL_TDM_MODE), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC0_DATA_SEL", 0, EXTERN_DAC_KCTL_DATA_SEL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC1_DATA_SEL", 1, EXTERN_DAC_KCTL_DATA_SEL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC2_DATA_SEL", 2, EXTERN_DAC_KCTL_DATA_SEL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC3_DATA_SEL", 3, EXTERN_DAC_KCTL_DATA_SEL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC0_MUTE_CTL", 0, EXTERN_DAC_KCTL_MUTE_CTL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC1_MUTE_CTL", 1, EXTERN_DAC_KCTL_MUTE_CTL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC2_MUTE_CTL", 2, EXTERN_DAC_KCTL_MUTE_CTL), \
	EXT_ADC_KCTL_SINGLE("EXT_ADC3_MUTE_CTL", 3, EXTERN_DAC_KCTL_MUTE_CTL), \

static const char * const adc_control_text[] = { "OFF", "ON" };

static const struct soc_enum adc_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(adc_control_text), adc_control_text),
};

static int adc_control_status_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = ADC_POWERON;
	int i;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return 0;
	}

	if (extern_adc_priv_data == NULL) {
		hwlog_err("%s: extern_adc_priv_data is NULL\n", __func__);
		return 0;
	}

	for (i = 0; i < (int)extern_adc_priv_data->adc_num; i++) {
		if (extern_adc_priv_data->power_status[i] == ADC_POWEROFF) {
			ret = ADC_POWEROFF;
			break;
		}
	}
	ucontrol->value.integer.value[0] = ret;

	return 0;
}

static int adc_control_status_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret;

	if (kcontrol == NULL || ucontrol == NULL) {
		hwlog_err("%s: input pointer is null", __func__);
		return 0;
	}

	if (extern_adc_priv_data == NULL) {
		hwlog_err("%s: extern_adc_priv_data is NULL\n", __func__);
		return 0;
	}

	ret = ucontrol->value.integer.value[0];

	if (extern_adc_priv_data->ioctl_ops == NULL ||
		extern_adc_priv_data->ioctl_ops->power_on == NULL ||
		extern_adc_priv_data->ioctl_ops->power_off == NULL) {
		hwlog_err("%s: ioctl_ops is NULL\n", __func__);
		return 0;
	}

	if (ret == ADC_POWERON) {
		hwlog_info("%s: set adc poweron\n", __func__);
		extern_adc_write_regs_to_multi_chips(extern_adc_priv_data,
			extern_adc_priv_data->ioctl_ops->power_on, ADC_POWERON);
	} else {
		hwlog_info("%s: set adc poweroff\n", __func__);
		extern_adc_write_regs_to_multi_chips(extern_adc_priv_data,
			extern_adc_priv_data->ioctl_ops->power_off,
			ADC_POWEROFF);
	}

	return ret;
}

#define EXTERN_ADC_POWER_KCONTROLS \
	SOC_ENUM_EXT("EXTERN_ADCS_POWER_CONTROL", adc_control_enum[0], \
		adc_control_status_get, adc_control_status_set),\


static const struct snd_kcontrol_new snd_controls[] = {
	EXTERN_ADC_PGA_VOLUME_KCONTROLS
	EXTERN_ADC_PGA_KCTL_KCONTROLS
	EXTERN_ADC_POWER_KCONTROLS
};

int extern_adc_add_kcontrol(struct snd_soc_codec *codec)
{
	if (codec == NULL) {
		hwlog_err("%s: ecodec parameter is NULL\n", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: enter\n", __func__);
#endif

	return snd_soc_add_codec_controls(codec, snd_controls,
		ARRAY_SIZE(snd_controls));
}
EXPORT_SYMBOL_GPL(extern_adc_add_kcontrol);

static int extern_adc_check_dt_info_valid(struct extern_adc_priv *adc_priv)
{
	int ret = 0;

	if (adc_priv->adc_num >= EXTERN_ADC_ID_MAX) {
		hwlog_err("%s: adc_num %u invalid\n", __func__,
			adc_priv->adc_num);
		ret = -EINVAL;
	}

	return ret;
}

static int extern_adc_parse_dt_info(struct platform_device *pdev,
	struct extern_adc_priv *adc_priv)
{
	const char *adc_num = "adc_num";
	int ret;

	if ((pdev == NULL) || (adc_priv == NULL)) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	ret = extern_adc_get_prop_of_u32_type(pdev->dev.of_node,
		adc_num, &adc_priv->adc_num, true);
	ret += extern_adc_check_dt_info_valid(adc_priv);

	return ret;
}

static void extern_adc_free(struct extern_adc_priv *adc_priv)
{
	extern_adc_kfree_ops(adc_priv);
}

static void extern_adc_reset_priv_data(struct extern_adc_priv *adc_priv)
{
	int i;
	int len;

	if (adc_priv == NULL)
		return;

	adc_priv->chip_register_failed = false;

	len = (strlen(EXTERN_ADC_NAME_INVALID) < EXTERN_ADC_NAME_MAX) ?
		strlen(EXTERN_ADC_NAME_INVALID) : (EXTERN_ADC_NAME_MAX - 1);

	memset(adc_priv->i2c_addr_to_pa_index, EXTERN_ADC_INVALID_PA_INDEX,
		sizeof(adc_priv->i2c_addr_to_pa_index));
	for (i = 0; i < EXTERN_ADC_ID_MAX; i++) {
		strncpy(adc_priv->chip_model_list[i],
			EXTERN_ADC_NAME_INVALID, len);
		adc_priv->chip_model_list[i][len] = '\0';
	}
}

static ssize_t extern_adc_regs_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (buf == NULL)
		return 0;

	if (extern_adc_priv_data == NULL || extern_adc_init_flag == 0)
		return 0;

	extern_adc_get_info(extern_adc_priv_data);
	extern_adc_dump_chips(extern_adc_priv_data);
	return snprintf(buf, PAGE_SIZE, "%s\n", "dump extern adc regs");
}


static DEVICE_ATTR(extern_adc_regs_info, 0660, extern_adc_regs_info_show, NULL);

static struct attribute *extern_adc_attributes[] = {
	&dev_attr_extern_adc_regs_info.attr,
	NULL
};

static const struct attribute_group extern_adc_attr_group = {
	.attrs = extern_adc_attributes,
};

static int extern_adc_probe(struct platform_device *pdev)
{
	struct extern_adc_priv *adc_priv = NULL;
	int ret;

#ifndef CONFIG_FINAL_RELEASE
	hwlog_info("%s: enter\n", __func__);
#endif

	if (pdev == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	adc_priv = kzalloc(sizeof(*adc_priv), GFP_KERNEL);
	if (adc_priv == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, adc_priv);
	extern_adc_reset_priv_data(adc_priv);

	ret = extern_adc_parse_dt_info(pdev, adc_priv);
	if (ret < 0) {
		hwlog_err("%s: parse dt_info failed, %d\n", __func__, ret);
		goto probe_err_out;
	}

	/* init ops lock */
	mutex_init(&adc_priv->dump_regs_lock);
	mutex_init(&adc_priv->i2c_ops_lock);

	extern_adc_priv_data = adc_priv;
	extern_adc_init_flag = 1; /* 1: init success */

	ret = sysfs_create_group(&(pdev->dev.kobj), &extern_adc_attr_group);
	if (ret < 0)
		hwlog_err("failed to register sysfs\n");

	hwlog_info("%s: end success\n", __func__);
	return 0;

probe_err_out:
	extern_adc_free(adc_priv);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int extern_adc_remove(struct platform_device *pdev)
{
	struct extern_adc_priv *adc_priv = NULL;

	if (pdev == NULL) {
		hwlog_err("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	adc_priv = platform_get_drvdata(pdev);
	if (adc_priv == NULL) {
		hwlog_err("%s: adc_priv invalid argument\n", __func__);
		return -EINVAL;
	}
	platform_set_drvdata(pdev, NULL);
	extern_adc_free(adc_priv);
	extern_adc_priv_data = NULL;

	return 0;
}

static void extern_adc_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id extern_adc_match[] = {
	{ .compatible = "huawei,extern_adc", },
	{},
};
MODULE_DEVICE_TABLE(of, extern_adc_match);

static struct platform_driver extern_adc_driver = {
	.driver = {
		.name           = "extern_adc",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(extern_adc_match),
	},
	.probe    = extern_adc_probe,
	.remove   = extern_adc_remove,
	.shutdown = extern_adc_shutdown,
};

static int __init extern_adc_init(void)
{
	int ret;

	ret = platform_driver_register(&extern_adc_driver);
	if (ret != 0)
		hwlog_err("%s: platform_driver_register failed, %d\n",
			__func__, ret);

	return ret;
}

static void __exit extern_adc_exit(void)
{
	platform_driver_unregister(&extern_adc_driver);
}

fs_initcall(extern_adc_init);
module_exit(extern_adc_exit);

MODULE_DESCRIPTION("extern_adc driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_LICENSE("GPL v2");

