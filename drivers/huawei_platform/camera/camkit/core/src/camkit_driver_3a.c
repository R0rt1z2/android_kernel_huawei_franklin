/*
 * camkit_driver.c
 *
 * Copyright (c) huawei technologies co., ltd. 2020-2020. all rights reserved.
 *
 * Description: image sensor driver
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

#include <linux/spinlock.h>
#include <linux/delay.h>

#include <securec.h>

#include "camkit_driver_impl.h"
#include "camkit_sensor_i2c.h"

#define PLATFORM_BASE_GAIN 64
#define CAMKIT_PIX_10_BITS 10
#define SENSOR_LONG_SHUTTER_LINELENGTH (0x16F0 * 10)

extern spinlock_t camkit_lock;
uint32 pre_shutter; /* shutter be set in previous((N-2)th) frame. */
uint32 last_shutter; /* store the last shutter set by hal. */

static uint16 real_gain_to_register_gain(
	struct camkit_aec_info_t *aec_info, uint16 real_gain)
{
	struct smia_again_coeff *sima_coeff = NULL;
	uint16 reg_gain = aec_info->reg_gain_1x;
	int32 x;
	int32 y;

	sima_coeff = &(aec_info->smia_coeff);
	if (real_gain < aec_info->min_again)
		real_gain = aec_info->min_again;
	else if (real_gain > aec_info->max_again)
		real_gain = aec_info->max_again;

	x = sima_coeff->c0 * PLATFORM_BASE_GAIN - sima_coeff->c1 * real_gain;
	y = sima_coeff->m1 * real_gain - sima_coeff->m0 * PLATFORM_BASE_GAIN;
	if (y != 0)
		reg_gain = x / y;

	return reg_gain;
}

static uint16 register_gain_to_real_again(
	struct camkit_aec_info_t *aec_info, uint16 reg_gain)
{
	struct smia_again_coeff *sima_coeff = NULL;
	uint16 gain = aec_info->min_again;
	int32 x;
	int32 y;

	sima_coeff = &(aec_info->smia_coeff);
	x = (sima_coeff->m0 * reg_gain + sima_coeff->c0) * PLATFORM_BASE_GAIN;
	y = sima_coeff->m1 * reg_gain + sima_coeff->c1;

	if (y != 0)
		gain = x / y;

	return gain;
}

static uint16 calc_register_dgain(struct camkit_aec_info_t *aec_info,
	uint16 real_gain, uint16 real_again)
{
	uint16 reg_dgain;

	if (real_again != 0 && (real_gain > aec_info->max_again ||
		aec_info->again_type == CAMKIT_AGAIN_OV13855))
		reg_dgain = real_gain * aec_info->dgain_decimator / real_again;
	else
		reg_dgain = aec_info->min_dgain;

	if (reg_dgain < aec_info->min_dgain)
		reg_dgain = aec_info->min_dgain;
	else if  (reg_dgain > aec_info->max_dgain)
		reg_dgain = aec_info->max_dgain;

	aaa_dbg("digital reg gain %u", reg_dgain);

	return reg_dgain;
}

static uint32 camkit_calc_gain(struct camkit_sensor_params *params,
	struct camkit_aec_info_t *aec_info, uint16 platform_gain,
	struct aec_ctrl_cfg *aec_ctrl)
{
	uint16 reg_again;
	uint16 reg_dgain;
	uint16 real_again;

	aaa_dbg("[%s] 3A platform_gain %u", params->sensor_name, platform_gain);

	// transfer real gain to sensor register gain
	reg_again = real_gain_to_register_gain(aec_info, platform_gain);
	aaa_dbg("[%s] sensor reg_again 0x%x", params->sensor_name, reg_again);
	aec_ctrl->again = reg_again;
	aec_ctrl->again_valid = true;

	// in order to calculate digital gain, need to recalculate
	// the analog again for accuracy
	real_again = register_gain_to_real_again(aec_info, reg_again);
	aaa_dbg("[%s] real_again %u", params->sensor_name, real_again);

	reg_dgain = calc_register_dgain(aec_info, platform_gain, real_again);
	aaa_dbg("[%s] sensor reg_dgain 0x%x", params->sensor_name, reg_dgain);
	aec_ctrl->dgain = reg_dgain;
	aec_ctrl->dgain_valid = true;

	return ERR_NONE;
}

static void camkit_fill_aec_item(struct camkit_aec_op *aec_ops, uint32 val,
	uint8 item_valid, struct sensor_setting_cfg *setting_cfg,
	uint16 *reg_count)
{
	int i;
	uint16 shift;
	struct camkit_aec_cfg *aec_setting = aec_ops->aec_setting;

	if ((aec_ops == NULL) || (aec_ops->aec_setting == NULL) ||
		(setting_cfg == NULL) || (reg_count == NULL)) {
		log_err("invalid arguments");
		return;
	}

	if (!item_valid)
		return;

	for (i = 0; i < aec_ops->size; i++) {
		if (*reg_count >= MAX_I2C_REG_NUM) {
			log_info("regs too long, may not be filled");
			return;
		}

		setting_cfg->i2c_setting[*reg_count].addr = aec_setting[i].addr;
		if (aec_setting[i].shift >= 0) {
			shift = (uint16)aec_setting[i].shift;
			setting_cfg->i2c_setting[*reg_count].val =
				(val << shift) & aec_setting[i].max_val;
		} else {
			shift = (uint16)(-aec_setting[i].shift);
			setting_cfg->i2c_setting[*reg_count].val =
				(val >> shift) & aec_setting[i].max_val;
		}
		setting_cfg->i2c_setting[*reg_count].addr_type =
			aec_setting[i].addr_type;
		setting_cfg->i2c_setting[*reg_count].data_type =
			aec_setting[i].data_type;
		setting_cfg->i2c_setting[*reg_count].i2c_mode = SENSOR_WRITE;
		setting_cfg->i2c_setting[*reg_count].delay = 0;

		aaa_dbg("addr = 0x%x; val = 0x%x",
			aec_setting[i].addr,
			setting_cfg->i2c_setting[*reg_count].val);

		*reg_count = *reg_count + 1;
	}
}

static void camkit_fill_i2c_item(struct camkit_i2c_cfg *i2c_setting,
	uint32 size, struct sensor_setting_cfg *setting_cfg,
	uint16 *reg_count)
{
	uint32 i;

	for (i = 0; i < size; i++) {
		if (*reg_count >= MAX_I2C_REG_NUM) {
			log_info("regs too large, may not be filled");
			return;
		}

		setting_cfg->i2c_setting[*reg_count].addr =
			i2c_setting[i].addr;
		setting_cfg->i2c_setting[*reg_count].val =
			i2c_setting[i].val;
		setting_cfg->i2c_setting[*reg_count].addr_type =
			i2c_setting[i].addr_type;
		setting_cfg->i2c_setting[*reg_count].data_type =
			i2c_setting[i].data_type;
		setting_cfg->i2c_setting[*reg_count].i2c_mode = i2c_setting[i].mode;
		setting_cfg->i2c_setting[*reg_count].delay = i2c_setting[i].delay;
		aaa_dbg("addr = 0x%x; val = 0x%x",
			i2c_setting[i].addr, i2c_setting[i].val);

		*reg_count = *reg_count + 1;
	}
}

static void camkit_fill_aec_array(struct aec_ops_map *aec_map,
	struct aec_ctrl_cfg *aec_ctrl, struct sensor_setting_cfg *setting_cfg)
{
	int16  i;
	uint16 reg_count = 0;
	int32  index;
	struct camkit_i2c_cfg *camkit_i2c_setting = NULL;
	struct camkit_aec_op *aec_op = NULL;
	uint32 camkit_i2c_size;

	aaa_dbg("aec_map.size: %d", aec_map->size);
	for (index = 0; index < aec_map->size; index++) {
		switch (aec_map->aec_ops[index].op_type) {
		case SENSOR_AEC_OP_GROUPON:
		case SENSOR_AEC_OP_GROUPOFF:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_VTS:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->frame_length,
				aec_ctrl->fl_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_SHIFT:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->shift,
				aec_ctrl->shift_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_LC:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->line_count,
				aec_ctrl->lc_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_AGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->again,
				aec_ctrl->again_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_DGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op, aec_ctrl->dgain,
				aec_ctrl->dgain_valid, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_CTRL:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_LONG_EXPO_MODE_CONFIG:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		default:
			break;
		}
	}

	for (i = 0; i < reg_count; i++) {
		aaa_dbg("Addr: 0x%X", setting_cfg->i2c_setting[i].addr);
		aaa_dbg("Data: 0x%X", setting_cfg->i2c_setting[i].val);
	}

	setting_cfg->size = reg_count;
	setting_cfg->delay = 0;
}

static uint32 camkit_calc_gc_gain(struct camkit_sensor_params *params,
	struct camkit_aec_info_t *aec_info, uint16 platform_gain,
	struct aec_ctrl_cfg *aec_ctrl)
{
	uint16 reg_dgain;
	uint16 real_again;
	uint16 i;
	uint16 gain;

	struct private_again_info *gc_again_info = &(aec_info->priv_again_info);

	if (gc_again_info->size == 0 ||
		gc_again_info->again_map == NULL) {
		log_err("gc gain table not configure?");
		return ERR_NONE;
	}
	if (gc_again_info->again_map[0].again_val <
		aec_info->min_gain) {
		log_err("gc gain table configured incorrectly");
		return ERR_NONE;
	}

	aaa_dbg("3A platform_gain %u", platform_gain);
	gain = platform_gain;  // gain = real_gain * 64

	if (gain < aec_info->min_gain)
		gain = aec_info->min_gain;
	else if (gain > aec_info->max_gain)
		gain = aec_info->max_gain;

	// get gain index from gain table
	for (i = 0; i < gc_again_info->size; i++) {
		if (gain < gc_again_info->again_map[i].again_val)
			break;
	}

	if (i == 0) {
		log_err("min gain configured incorrectly");
		return ERR_NONE;
	}
	if (i == gc_again_info->size)
		log_err("max gain configured incorrectly");

	aec_ctrl->again = i - 1;   // i is the gain index, not reg gain
	aec_ctrl->again_valid = true;
	aaa_dbg("gc sensor gain index %u", i - 1);

	// in order to calculate digital gain, need to recalculate
	// the analog again for accuracy
	real_again = gc_again_info->again_map[i - 1].again_val;
	if (real_again == 0) {
		log_err("gc gain table not configure?");
		return ERR_NONE;
	}
	aaa_dbg("real_again %u", real_again);
	reg_dgain = gain * aec_info->dgain_decimator / real_again;
	aaa_dbg("sensor reg_dgain 0x%x", reg_dgain);
	aec_ctrl->dgain = reg_dgain;
	aec_ctrl->dgain_valid = true;

	return ERR_NONE;
}

static void camkit_fill_gc_gain_array(struct camkit_aec_info_t *aec_info,
	struct aec_ctrl_cfg *aec_ctrl, struct sensor_setting_cfg *setting_cfg)
{
	int16  i;
	uint16 reg_count = 0;
	int32  index;
	struct camkit_i2c_cfg *camkit_i2c_setting = NULL;
	uint32 camkit_i2c_size;
	struct camkit_i2c_reg *again_setting = NULL;
	struct camkit_aec_op *aec_op = NULL;

	uint32 again_idx = aec_ctrl->again;
	struct private_again_info *gc_again_info = &(aec_info->priv_again_info);
	struct aec_ops_map *aec_map = &(aec_info->gain_ops_map);

	if (aec_map->size <= 0 || again_idx >= gc_again_info->size) {
		log_err("invalid parameters");
		return;
	}

	aaa_dbg("aec_map.size: %d", aec_map->size);
	for (index = 0; index < aec_map->size; index++) {
		switch (aec_map->aec_ops[index].op_type) {
		case SENSOR_AEC_OP_GROUPON:
		case SENSOR_AEC_OP_GROUPOFF:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_AGAIN_TABLE:
			again_setting = gc_again_info->again_map[again_idx].setting;
			camkit_i2c_size = gc_again_info->again_map[again_idx].size;
			for (i = 0; i < camkit_i2c_size; i++) {
				if (reg_count >= MAX_AEC_REGS) {
					log_info("out of range, the will not be filled");
					break;
				}
				setting_cfg->i2c_setting[reg_count].addr =
					again_setting[i].addr;
				setting_cfg->i2c_setting[reg_count].val =
					again_setting[i].data;
				setting_cfg->i2c_setting[reg_count].addr_type =
					gc_again_info->again_map[again_idx].addr_type;
				setting_cfg->i2c_setting[reg_count].data_type =
					gc_again_info->again_map[again_idx].data_type;
				reg_count = reg_count + 1;
			}
			break;

		case SENSOR_AEC_OP_DGAIN:
			aec_op = &aec_map->aec_ops[index];
			camkit_fill_aec_item(aec_op,
				aec_ctrl->dgain, aec_ctrl->dgain_valid,
				setting_cfg, &reg_count);
			break;

		case SENSOR_AEC_OP_CTRL:
			camkit_i2c_setting =
				aec_map->aec_ops[index].i2c_setting;
			camkit_i2c_size = aec_map->aec_ops[index].size;
			camkit_fill_i2c_item(camkit_i2c_setting,
				camkit_i2c_size, setting_cfg, &reg_count);
			break;

		default:
			log_err("unsupported operator type: %u",
				aec_map->aec_ops[index].op_type);
			break;
		}
	}

	for (i = 0; i < reg_count; i++) {
		aaa_dbg("Addr: 0x%x", setting_cfg->i2c_setting[i].addr);
		aaa_dbg("Data: 0x%x", setting_cfg->i2c_setting[i].val);
	}

	setting_cfg->size = reg_count;
	setting_cfg->delay = 0;
}

// for gc sensor, such as: gc2375, gc5035 and gc8034 and so on
static uint16 set_gc_gain(struct camkit_sensor_params *params,
	uint16 gain)
{
	int32 ret;
	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *gain_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;

	unsigned long flags;
	uint16 i;

	return_0_if_null(params);
	aec_info = &(params->aec_info);
	gain_ops_map = &(aec_info->gain_ops_map);

	(void)memset_s(&setting_cfg, sizeof(setting_cfg), 0, sizeof(setting_cfg));
	(void)memset_s(&aec_ctrl, sizeof(aec_ctrl), 0, sizeof(aec_ctrl));

	ret = camkit_calc_gc_gain(params, aec_info, gain, &aec_ctrl);
	if (ret != ERR_NONE)
		log_err("camkit_calc_gc_gain failed");

	camkit_fill_gc_gain_array(aec_info, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret != ERR_NONE) {
			log_err("write gain failed, gain: %u", gain);
			return ERR_IO;
		}
	}

	spin_lock_irqsave(&camkit_lock, flags);
	params->sensor_ctrl.gain = gain;
	spin_unlock_irqrestore(&camkit_lock, flags);

	return ERR_NONE;
}

uint32 camkit_set_gain(struct camkit_sensor *sensor, const uint16 gain)
{
	int32 ret;
	struct camkit_sensor_params *params = NULL;
	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *gain_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;

	unsigned long flags;
	uint16 i;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	if (!params) {
		log_err("invalid input parameters");
		return ERR_NONE;
	}

	aec_info = &(params->aec_info);
	if (aec_info->again_type == CAMKIT_AGAIN_GC)
		return set_gc_gain(params, gain);

	gain_ops_map = &(aec_info->gain_ops_map);

	(void)memset_s(&setting_cfg, sizeof(setting_cfg), 0, sizeof(setting_cfg));
	(void)memset_s(&aec_ctrl, sizeof(aec_ctrl), 0, sizeof(aec_ctrl));

	ret = camkit_calc_gain(params, aec_info, gain, &aec_ctrl);
	if (ret != ERR_NONE)
		log_err("camkit_calc_gain failed");

	camkit_fill_aec_array(gain_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret != ERR_NONE) {
			log_err("write gain failed, gain: %u", gain);
			return ERR_IO;
		}
	}

	spin_lock_irqsave(&camkit_lock, flags);
	params->sensor_ctrl.gain = gain;
	spin_unlock_irqrestore(&camkit_lock, flags);

	return ERR_NONE;
}

static void camkit_calc_vblank(struct camkit_aec_info_t *aec_info,
	uint32 *frame_length)
{
	uint32 base_fl = aec_info->base_fl;
	uint16 min_vblank = aec_info->min_vblank;
	uint16 max_vblank = aec_info->max_vblank;
	uint32 vblank = *frame_length - base_fl;

	vblank = (vblank < min_vblank) ? min_vblank : vblank;
	vblank = (vblank > max_vblank) ? max_vblank : vblank;

	aaa_dbg("base_fl = %u, vblank [%u-%u]: %u",
		base_fl, min_vblank, max_vblank, vblank);

	*frame_length = vblank;
}

uint32 camkit_set_dummy(struct camkit_sensor *sensor)
{
	int32 ret;
	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *vts_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;
	uint16 i;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	aec_info = &(params->aec_info);
	vts_ops_map = &(aec_info->vts_ops_map);

	(void)memset_s(&setting_cfg, sizeof(setting_cfg), 0, sizeof(setting_cfg));
	(void)memset_s(&aec_ctrl, sizeof(aec_ctrl), 0, sizeof(aec_ctrl));

	aec_ctrl.frame_length = params->sensor_ctrl.frame_length;
	if (aec_info->vblank_flag)
		camkit_calc_vblank(aec_info, &aec_ctrl.frame_length);
	aec_ctrl.fl_valid = true;

	camkit_fill_aec_array(vts_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	for (i = 0; i < setting_cfg.size; i++) {
		ret = camkit_sensor_i2c_write(&params->sensor_ctrl,
			setting_cfg.i2c_setting[i].addr,
			setting_cfg.i2c_setting[i].val,
			setting_cfg.i2c_setting[i].data_type);
		if (ret != ERR_NONE) {
			log_err("write fl failed, frame_length: 0x%x",
				params->sensor_ctrl.frame_length);
			return ERR_INVAL;
		}
	}

	return ERR_NONE;
}

uint32 camkit_set_max_framerate(struct camkit_sensor *sensor,
	uint16 framerate, uint8 min_framelength_en)
{
	uint32 frame_length;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	sensor_ctrl = &(params->sensor_ctrl);
	if (!framerate || !sensor_ctrl->line_length) {
		log_err("Invalid params. framerate=%u, line_length=%u",
			framerate, sensor_ctrl->line_length);
		return ERR_INVAL;
	}

	frame_length = sensor_ctrl->pclk / framerate *
		CAMKIT_PIX_10_BITS / sensor_ctrl->line_length;

	spin_lock(&camkit_lock);
	sensor_ctrl->frame_length = frame_length;
	if (frame_length < sensor_ctrl->min_frame_length)
		sensor_ctrl->frame_length = sensor_ctrl->min_frame_length;
	if (frame_length > params->aec_info.max_frame_length)
		sensor_ctrl->frame_length = params->aec_info.max_frame_length;
	if (min_framelength_en)
		sensor_ctrl->min_frame_length = sensor_ctrl->frame_length;
	spin_unlock(&camkit_lock);

	sensor->sensor_ops->set_dummy(sensor);

	return ERR_NONE;
}

uint32 camkit_set_shutter(struct camkit_sensor *sensor, const uint32 shutter)
{
	uint32 frame_length;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	sensor_ctrl = &(params->sensor_ctrl);
	spin_lock(&camkit_lock);
	if (shutter > sensor_ctrl->min_frame_length -
		params->aec_info.vts_offset)
		sensor_ctrl->frame_length =
			shutter + params->aec_info.vts_offset;
	else
		sensor_ctrl->frame_length =
			sensor_ctrl->min_frame_length;

	frame_length = sensor_ctrl->frame_length;
	spin_unlock(&camkit_lock);

	sensor->sensor_ops->set_shutter_frame_length(sensor,
		shutter, frame_length);

	return ERR_NONE;
}

/*
 * for long shutter mode which needs to stream off, such as s5k3l6
 */
static enum sensor_shutter_mode get_shutter_mode(uint32 in_shutter,
		uint32 *out_shutter, struct camkit_aec_info_t *aec_info)
{
	uint32 cur_shutter; /* shutter needed to set in current(Nth) frame. */
	enum sensor_shutter_mode shutter_mode;
	uint32 normal_max_shutter = aec_info->max_linecount;

	if (last_shutter == 0)
		last_shutter = in_shutter;

	if (pre_shutter == 0)
		pre_shutter = in_shutter;

	/* 1. update cur_shutter to "shutter" or "last_shutter" */
	if (in_shutter > normal_max_shutter &&
		last_shutter <= normal_max_shutter) {
		/*
		 * before change to long shutter, set a normal shutter firstly,
		 * actully long shutter will be set next time
		 */
		cur_shutter = last_shutter;  // set last shutter
	} else if (in_shutter <= normal_max_shutter &&
		   last_shutter > normal_max_shutter) {
		/*
		 * before change to normal shutter, set a long shutter firstly,
		 * actully normal shutter will be set next time
		 */
		cur_shutter = last_shutter;  /* set last shutter */
	} else {
		cur_shutter = in_shutter;    /* set current shutter */
	}
	aaa_dbg("cur_shutter first is %u", cur_shutter);
	/* 2. update shutter_mode base on cur_shutter and previous shutter */
	if (cur_shutter > normal_max_shutter &&
		pre_shutter <= normal_max_shutter) {
		/*
		 * the shutter to be set this time is long,
		 * but last is normal, the mode is NORMAL2LONG
		 */
		cur_shutter = last_shutter;
		log_info("NORMAL2LONG");
		shutter_mode = SHUTTER_NORMAL2LONG;
	} else if (cur_shutter <= normal_max_shutter &&
		   pre_shutter > normal_max_shutter) {
		/*
		 * the shutter to be set this time is normal,
		 * but last is long, the mode is LONG2NORMAL
		 */
		log_info("LONG2NORMAL");
		cur_shutter = last_shutter;
		shutter_mode = SHUTTER_LONG2NORMAL;
	} else {
		/*
		 * the shutter to be set this time is normal,
		 * and last is normal too, the mode is NORMAL
		 */
		aaa_dbg("NORMAL");
		shutter_mode = SHUTTER_DEFAULT;
	}
	aaa_dbg("cur_shutter second is %u", cur_shutter);
	*out_shutter = cur_shutter;
	pre_shutter = cur_shutter;
	last_shutter = in_shutter;

	return shutter_mode;
}

/*
 * for long shutter mode which needs to stream off, such as s5k3l6
 */
static uint32 camkit_calc_shutter_by_mode(struct camkit_sensor_params *params,
	uint32 in_shutter, enum sensor_shutter_mode shutter_mode,
	struct aec_ops_map **expo_ops_map)
{
	uint32 out_shutter;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;
	struct camkit_aec_info_t *aec_info = NULL;

	if (params == NULL || expo_ops_map == NULL) {
		log_err("input parameters is unavailable, return default\n");
		return in_shutter;
	}

	sensor_ctrl = &(params->sensor_ctrl);
	aec_info = &(params->aec_info);
	/* default shutter */
	out_shutter = in_shutter;
	/* default expo ops map */
	*expo_ops_map = &(aec_info->expo_ops_map);

	switch (shutter_mode) {
	case SHUTTER_NORMAL2LONG:
		*expo_ops_map = &(aec_info->normal2long_ops_map);
		out_shutter = in_shutter * sensor_ctrl->line_length / SENSOR_LONG_SHUTTER_LINELENGTH;
		log_info("SHUTTER_NORMAL2LONG shutter: %u\n", in_shutter);
		break;
	case SHUTTER_LONG2NORMAL:
		*expo_ops_map = &(aec_info->long2normal_ops_map);
		log_info("SHUTTER_LONG2NORMAL shutter: %u\n", in_shutter);
		break;
	default:
		log_info("Default Mode\n");
		break;
	}

	if (out_shutter > aec_info->max_linecount)
		out_shutter = aec_info->max_linecount;
	else if (out_shutter < aec_info->min_linecount)
		out_shutter = aec_info->min_linecount;

	return out_shutter;
}

static uint32 camkit_get_expo_info_samsung_type(struct camkit_sensor_params *params,
	uint32 in_shutter,
	struct aec_ops_map **out_expo_map)
{
	uint32 shutter;
	enum sensor_shutter_mode shutter_mode;
	struct camkit_aec_info_t *aec_info = &(params->aec_info);

	shutter_mode = get_shutter_mode(in_shutter, &shutter, aec_info);
	shutter = camkit_calc_shutter_by_mode(params, shutter, shutter_mode, out_expo_map);
	aaa_dbg("shutter_mode: %d, shutter: %u\n", shutter_mode, shutter);
	return shutter;
}

static int32 camkit_write_setting_cfg(struct camkit_sensor_ctrl_t *sensor_ctrl,
	struct sensor_setting_cfg *setting_cfg)
{
	int32 i;
	int32 ret = 0;
	struct sensor_i2c_cfg *cfg = setting_cfg->i2c_setting;

	for (i = 0; i < setting_cfg->size; i++) {
		switch (cfg[i].i2c_mode) {
		case SENSOR_WRITE:
			ret = camkit_sensor_i2c_write(sensor_ctrl,
				cfg[i].addr,
				cfg[i].val,
				cfg[i].data_type);
			if (cfg[i].delay > 0)
				msleep(cfg[i].delay);
			break;
		case SENSOR_POLL:
			ret = camkit_sensor_i2c_poll(sensor_ctrl, cfg[i].addr,
				cfg[i].val, cfg[i].data_type,
				cfg[i].delay);
			break;
		default:
			log_err("Not support yet, mode：%d", cfg[i].i2c_mode);
			break;
		}

		aaa_dbg("addr: 0x%x, val: 0x%x, type:%u, mode: %d, delay: %u\n",
			cfg[i].addr, cfg[i].val, cfg[i].data_type, cfg[i].i2c_mode, cfg[i].delay);

		if (ret < 0) {
			log_err("write setting cfg failed, mode：%d", cfg[i].i2c_mode);
			return ret;
		}
	}

	return ret;
}

static uint32 camkit_abandon_flicker(struct camkit_sensor_ctrl_t *sensor_ctrl)
{
	uint16 realtime_fps;

	if (!sensor_ctrl->autoflicker_en)
		return ERR_NONE;

	if (sensor_ctrl->line_length == 0 ||
		sensor_ctrl->frame_length == 0)
		return ERR_NONE;

	realtime_fps = sensor_ctrl->pclk / sensor_ctrl->line_length *
		CAMKIT_PIX_10_BITS / sensor_ctrl->frame_length;

	/*
	 * in order to abandon flicker, set fps to 298 while fps
	 * between 298~305, set fps 146 while fps between 147~150
	 */
	if (realtime_fps >= 298 && realtime_fps <= 305) {
		realtime_fps = 298;
		sensor_ctrl->frame_length = sensor_ctrl->pclk /
			realtime_fps * CAMKIT_PIX_10_BITS /
			sensor_ctrl->line_length;
	} else if (realtime_fps >= 147 && realtime_fps <= 150) {
		realtime_fps = 146;
		sensor_ctrl->frame_length = sensor_ctrl->pclk /
			realtime_fps * CAMKIT_PIX_10_BITS /
			sensor_ctrl->line_length;
	}

	return ERR_NONE;
}

uint32 camkit_set_shutter_frame_length(struct camkit_sensor *sensor,
	uint32 shutter, uint32 frame_length)
{
	int32 ret;

	struct camkit_aec_info_t *aec_info = NULL;
	struct aec_ops_map *expo_ops_map = NULL;
	struct sensor_setting_cfg setting_cfg;
	struct aec_ctrl_cfg aec_ctrl;
	struct camkit_sensor_ctrl_t *sensor_ctrl = NULL;
	unsigned long flags;
	uint16 shift = 0;
	struct camkit_sensor_params *params = NULL;

	return_err_if_null(sensor);
	return_err_if_null(sensor->sensor_ops);
	return_err_if_null(sensor->kit_params);
	return_err_if_null(sensor->kit_params->sensor_params);
	params = sensor->kit_params->sensor_params;

	log_info("input shutter is %u", shutter);

	sensor_ctrl = &(params->sensor_ctrl);
	aec_info = &(params->aec_info);
	expo_ops_map = &(aec_info->expo_ops_map);

	(void)memset_s(&setting_cfg, sizeof(setting_cfg), 0, sizeof(setting_cfg));
	(void)memset_s(&aec_ctrl, sizeof(aec_ctrl), 0, sizeof(aec_ctrl));

	/*
	 * 1. confirm the shutter whether it is
	 * in the min and max line count threshold
	 */
	if (shutter < aec_info->min_linecount)
		shutter = aec_info->min_linecount;

	if (aec_info->lexpo_type == SENSOR_LONG_EXPO_SONY) {
		while (shutter > aec_info->max_linecount) {
			shift++;
			if (shift > aec_info->max_shift) {
				shutter = aec_info->max_linecount;
				shift = aec_info->max_shift;
				break;
			}
			shutter >>= 1;
		}
	} else if (aec_info->lexpo_type == SENSOR_LONG_EXPO_SAMSUNG) {
		shutter = camkit_get_expo_info_samsung_type(params, shutter, &expo_ops_map);
		frame_length = shutter + aec_info->vts_offset;
	} else {
		if (shutter > aec_info->max_linecount)
			shutter = aec_info->max_linecount;
	}

	aec_ctrl.line_count = shutter;
	aec_ctrl.lc_valid = true;

	aec_ctrl.shift = shift;
	aec_ctrl.shift_valid = true;

	/* 2. calc frame length depend on shutter */
	aaa_dbg("[%s] lock", params->sensor_name);
	spin_lock(&camkit_lock);
	aaa_dbg("[%s] lock in", params->sensor_name);
	if (frame_length < sensor_ctrl->min_frame_length)
		frame_length = sensor_ctrl->min_frame_length;
	if (shutter > frame_length - aec_info->vts_offset || shift > 0)
		sensor_ctrl->frame_length = shutter + aec_info->vts_offset;
	else
		sensor_ctrl->frame_length = frame_length;

	if (sensor_ctrl->frame_length > aec_info->max_frame_length)
		sensor_ctrl->frame_length = aec_info->max_frame_length;

	/* 3. calc and update framelength to abandon the flicker issue */
	(void)camkit_abandon_flicker(sensor_ctrl);

	aec_ctrl.frame_length = sensor_ctrl->frame_length;
	if (aec_info->vblank_flag)
		camkit_calc_vblank(aec_info, &aec_ctrl.frame_length);
	aec_ctrl.fl_valid = true;

	aaa_dbg("[%s] lock down", params->sensor_name);
	spin_unlock(&camkit_lock);
	aaa_dbg("[%s] unlock", params->sensor_name);

	aaa_dbg("[%s] shutter = 0x%x, fl = 0x%x", params->sensor_name,
		shutter, aec_ctrl.frame_length);

	camkit_fill_aec_array(expo_ops_map, &aec_ctrl, &setting_cfg);

	// write registers to sensor
	ret = camkit_write_setting_cfg(sensor_ctrl, &setting_cfg);
	if (ret < 0) {
		log_err("write shutter failed, shutter: %u\n", shutter);
		return ret;
	}

	spin_lock_irqsave(&camkit_lock, flags);
	sensor_ctrl->shutter = shutter;
	spin_unlock_irqrestore(&camkit_lock, flags);

	return ERR_NONE;
}
