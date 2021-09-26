/*
 * cs35l41.h -- CS35L41 Misc Audio Amplifier SoC Driver
 *
 * Copyright 2018 Cirrus Logic, Inc.
 *
 * Author: Brian Austin <brian.austin@cirrus.com>
 *         David Rhodes <david.rhodes@cirrus.com>
 *	   James Schulman <james.schulman@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define DEBUG
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/workqueue.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "cs35axx_data.h"
#include "cs35axx.h"
#ifdef CONFIG_PM
#include <linux/suspend.h>
#endif
#ifdef CONFIG_HUAWEI_DSM_AUDIO
#include "dsm_audio/dsm_audio.h"
#endif
#include <linux/power_supply.h>
#include <chipset_common/hwpower/common_module/power_supply_interface.h>
#include "smartpakit.h"

#define CS35L41_MAGIC_NUMBER	0x112233

#define CS35L41_SPK_DAC_VOLUME			_IOWR(CS35L41_MAGIC_NUMBER, 1,\
							void *)
#define CS35L41_SPK_POWER_ON			_IOWR(CS35L41_MAGIC_NUMBER, 2,\
							void *)
#define CS35L41_SPK_POWER_OFF			_IOWR(CS35L41_MAGIC_NUMBER, 3,\
							void *)
#define CS35L41_SPK_DSP_BYPASS			_IOWR(CS35L41_MAGIC_NUMBER, 4,\
							void *)
#define CS35L41_SPK_SWITCH_CONFIGURATION	_IOWR(CS35L41_MAGIC_NUMBER, 5,\
							void *)
#define CS35L41_SPK_SWITCH_CALIBRATION		_IOWR(CS35L41_MAGIC_NUMBER, 6,\
							void *)
#define CS35L41_SPK_GET_R0			_IOWR(CS35L41_MAGIC_NUMBER, 7,\
							void *)
#define CS35L41_SPK_GET_F0			_IOWR(CS35L41_MAGIC_NUMBER, 8,\
							void *)
#define CS35L41_SPK_GET_CAL_STRUCT		_IOWR(CS35L41_MAGIC_NUMBER, 9,\
							void *)
#define CS35L41_SPK_SET_CAL_STRUCT		_IOWR(CS35L41_MAGIC_NUMBER, 10,\
							void *)
#define CS35L41_SPK_SET_AMBIENT			_IOWR(CS35L41_MAGIC_NUMBER, 11,\
							void *)
#define CS35L41_SPK_SET_R0		_IOWR(CS35L41_MAGIC_NUMBER, 12,\
							void *)
							
#define CS35L41_SPK_SWITCH_FIRMWARE		_IOWR(CS35L41_MAGIC_NUMBER, 13,\
							void *)
#define CS35L41_SPK_GET_R0_REALTIME   _IOWR(CS35L41_MAGIC_NUMBER, 14,\
							void *)
#define CS35L41_SPK_SET_VPBR     _IOWR(CS35L41_MAGIC_NUMBER, 15,\
							void *)

#ifdef CONFIG_COMPAT
#define CS35L41_SPK_DAC_VOLUME_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 1,\
							compat_uptr_t)
#define CS35L41_SPK_POWER_ON_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 2,\
							compat_uptr_t)
#define CS35L41_SPK_POWER_OFF_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 3,\
							compat_uptr_t)
#define CS35L41_SPK_DSP_BYPASS_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 4,\
							compat_uptr_t)
#define CS35L41_SPK_SWITCH_CONFIGURATION_COMPAT	_IOWR(CS35L41_MAGIC_NUMBER, 5,\
							compat_uptr_t)
#define CS35L41_SPK_SWITCH_CALIBRATION_COMPAT	_IOWR(CS35L41_MAGIC_NUMBER, 6,\
							compat_uptr_t)
#define CS35L41_SPK_GET_R0_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 7,\
							compat_uptr_t)
#define CS35L41_SPK_GET_F0_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 8,\
							compat_uptr_t)
#define CS35L41_SPK_GET_CAL_STRUCT_COMPAT	_IOWR(CS35L41_MAGIC_NUMBER, 9,\
							compat_uptr_t)
#define CS35L41_SPK_SET_CAL_STRUCT_COMPAT	_IOWR(CS35L41_MAGIC_NUMBER, 10,\
							compat_uptr_t)
#define CS35L41_SPK_SET_AMBIENT_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 11,\
							compat_uptr_t)
#define CS35L41_SPK_SET_R0_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 12,\
							compat_uptr_t)
#define CS35L41_SPK_SWITCH_FIRMWARE_COMPAT		_IOWR(CS35L41_MAGIC_NUMBER, 13,\
							compat_uptr_t)
#define CS35L41_SPK_GET_R0_REALTIME_COMPAT   _IOWR(CS35L41_MAGIC_NUMBER, 14,\
							compat_uptr_t)
#define CS35L41_SPK_SET_VPBR_COMPAT     _IOWR(CS35L41_MAGIC_NUMBER, 15,\
							compat_uptr_t)
#endif

//this macro for check dsp tuning, if the same as before, don't reload again
#define CHECK_DSP_TUNING
#define MBOX_MODE


#ifdef ESD_TEST
//log realtime tempereature
#define CS35L41_ESD_DELAY	2000
#define CS35L41_ESD_STAGE	CS35L41_SP_FRAME_TX_SLOT

#define CS35L41_ESD_STAGE1_MASK	(1 << 24)
#define CS35L41_ESD_STAGE2_MASK	(1 << 25)
#define CS35L41_ESD_STAGE3_MASK	(1 << 26)
#define CS35L41_ESD_STAGE4_MASK	(1 << 27)
#define CS35L41_ESD_STAGE5_MASK	(1 << 28)
#define CS35L41_ESD_STAGE6_MASK	(1 << 29)

#define CS35L41_ESD_STAGE_FINAL_MASK	(0x3f << 24)
#define CS35L41_ESD_STAGE_FINAL			(0x3c << 24)
#endif

#define SMARTPA_VENDOR 3 //CS
#define SMARTPA_MODEL "cs35axx"


struct cs35l41_cal_data {
	uint32_t status;
	uint32_t rdc;
	uint32_t temp;
	uint32_t checksum;
};

struct cs35l41_private {
	struct cs35l41_platform_data pdata;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	struct mutex lock;
	int num_supplies;
	unsigned int revid;
	unsigned int num_algos;
	struct cs35l41_algo_info algo_info[CS35L41_NUM_ALGOS_MAX + 1];
	struct list_head coeff_desc_head;
	struct miscdevice misc_dev;
#ifdef MBOX_MODE
	struct cs35l41_cal_data calib_data;
#endif
	int irq;
	unsigned int cspl_cmd;
	bool halo_booted;
	int box;
	int scene;
	bool diag_en;
	bool amp_en;
	bool esd_detect_en;
	/* GPIO for /RST */
	int reset_gpio;
	struct completion mbox_cmd;
#ifdef ESD_TEST
	struct delayed_work esd_work;
#endif
#ifdef CONFIG_PM
struct notifier_block pm_nb;
#endif

};



static const char * const cs35l41_supplies[] = {
	"VA",
	"VP",
};

static const unsigned char cs35l41_bst_k1_table[4][5] = {
	{0x24, 0x32, 0x32, 0x4F, 0x57},
	{0x24, 0x32, 0x32, 0x4F, 0x57},
	{0x40, 0x32, 0x32, 0x4F, 0x57},
	{0x40, 0x32, 0x32, 0x4F, 0x57}
};

static const unsigned char cs35l41_bst_k2_table[4][5] = {
	{0x24, 0x49, 0x66, 0xA3, 0xEA},
	{0x24, 0x49, 0x66, 0xA3, 0xEA},
	{0x48, 0x49, 0x66, 0xA3, 0xEA},
	{0x48, 0x49, 0x66, 0xA3, 0xEA}
};

static const unsigned char cs35l41_bst_slope_table[4] = {
					0x75, 0x6B, 0x3B, 0x28};

static const struct reg_sequence cs35l41_pup_patch[] = {
	{0x00000040, 0x00000055},
	{0x00000040, 0x000000AA},
	{0x00002084, 0x002F1AA0},
	{0x00000040, 0x000000CC},
	{0x00000040, 0x00000033},
};

static const struct reg_sequence cs35l41_pdn_patch[] = {
	{0x00000040, 0x00000055},
	{0x00000040, 0x000000AA},
	{0x00002084, 0x002F1AA3},
	{0x00000040, 0x000000CC},
	{0x00000040, 0x00000033},
};

static const struct reg_sequence cs35l41_spk_power_on_patch[] = {
	{CS35L41_AMP_GAIN_CTRL,	0x00000213},
	{CS35L41_SP_ENABLES,	0x00030003},
	//brownout prevention config
	{CS35L41_VPBR_CFG,	0x02AA111B},
	{CS35L41_PWR_CTRL3,	0x01001010},
	{CS35L41_PWR_CTRL2,	0x00003721},
};

static const struct reg_sequence cs35l41_spk_power_on_patch_low_temp[] = {
	{CS35L41_AMP_GAIN_CTRL,	0x00000213},
	{CS35L41_SP_ENABLES,	0x00030003},
	{CS35L41_VPBR_CFG,	0x02A8341B},
	{CS35L41_PWR_CTRL3,	0x01001010},
	{CS35L41_PWR_CTRL2,	0x00003721},
};

static const struct reg_sequence cs35l41_mpu_unlock_patch[] = {
	{0x2BC3140, 0x00005555},
	{0x2BC3140, 0x0000AAAA},
	{0x2BC6008, 0x00000000},
	{0x2BC3000, 0x00000003},
	{0x2BC3004, 0x00000001},
	{0x2BC3008, 0xFFFFFFFF},
	{0x2BC300C, 0xFFFFFFFF},
	{0x2BC3014, 0xFFFFFFFF},
	{0x2BC3018, 0xFFFFFFFF},
	{0x2BC301C, 0xFFFFFFFF},
	{0x2BC3020, 0xFFFFFFFF},
	{0x2BC3024, 0xFFFFFFFF},
	{0x2BC302C, 0xFFFFFFFF},
	{0x2BC3030, 0xFFFFFFFF},
	{0x2BC3034, 0xFFFFFFFF},
	{0x2BC3038, 0xFFFFFFFF},
	{0x2BC303C, 0xFFFFFFFF},
	{0x2BC3044, 0xFFFFFFFF},
	{0x2BC3048, 0xFFFFFFFF},
	{0x2BC304C, 0xFFFFFFFF},
	{0x2BC3050, 0xFFFFFFFF},
	{0x2BC3054, 0xFFFFFFFF},
	{0x2BC305C, 0xFFFFFFFF},
	{0x2BC3140, 0x0000DEAD},
	{0x2BC3140, 0x0000DEAD},
};

static const struct reg_sequence cs35l41_clear_stream_arb_patch[] = {
	/* Disable and clear masters */
	{0x2bc5000,	0}, {0x2bc5004,	0}, {0x2bc5008,	0},
	{0x2bc5010,	0}, {0x2bc5014,	0}, {0x2bc5018,	0},
	{0x2bc5020,	0}, {0x2bc5024,	0}, {0x2bc5028,	0},
	{0x2bc5030,	0}, {0x2bc5034,	0}, {0x2bc5038,	0},
	{0x2bc5040,	0}, {0x2bc5044,	0}, {0x2bc5048,	0},
	{0x2bc5050,	0}, {0x2bc5054,	0}, {0x2bc5058,	0},
	{0x2bc5060,	0}, {0x2bc5064,	0}, {0x2bc5068,	0},
	{0x2bc5070,	0}, {0x2bc5074,	0}, {0x2bc5078,	0},
	/* clear stream arbiter channel configs */
	{0x2bc5200,	0}, {0x2bc5208,	0},
	{0x2bc5210,	0}, {0x2bc5218,	0},
	{0x2bc5220,	0}, {0x2bc5228,	0},
	{0x2bc5230,	0}, {0x2bc5238,	0},
	{0x2bc5240,	0}, {0x2bc5248,	0},
	{0x2bc5250,	0}, {0x2bc5258,	0},
	{0x2bc5260,	0}, {0x2bc5268,	0},
	{0x2bc5270,	0}, {0x2bc5278,	0},
	{0x2bc5400,	0}, {0x2bc5408,	0},
	{0x2bc5410,	0}, {0x2bc5418,	0},
	{0x2bc5420,	0}, {0x2bc5428,	0},
	{0x2bc5430,	0}, {0x2bc5438,	0},
	{0x2bc5440,	0}, {0x2bc5448,	0},
	{0x2bc5450,	0}, {0x2bc5458,	0},
	{0x2bc5460,	0}, {0x2bc5468,	0},
	{0x2bc5470,	0}, {0x2bc5478,	0},
	/* clear stream arbiter interrupt registers */
	{0x2bc5600,	0}, {0x2bc5604,	0},
	{0x2bc5610,	0}, {0x2bc5614,	0},
	{0x2bc5620,	0}, {0x2bc5624,	0},
	{0x2bc5630,	0}, {0x2bc5634,	0},
	{0x2bc5640,	0}, {0x2bc5644,	0},
	{0x2bc5650,	0}, {0x2bc5654,	0},
	{0x2bc5660,	0}, {0x2bc5664,	0},
	{0x2bc5670,	0}, {0x2bc5674,	0},
};

static int cs35l41_firmware_parse(struct cs35l41_private *cs35l41, const struct firmware *fw);
static void cs35l41_coeff_file_load(const struct firmware *fw, void *context);

/* CONFIG_PM */
#ifdef CONFIG_PM
static int cs35l41_runtime_suspend(struct cs35l41_private *cs35l41)
{

	if (cs35l41->amp_en) {
		cs35l41->esd_detect_en = false;
	}
	return 0;
}
static int cs35l41_runtime_resume(struct cs35l41_private *cs35l41)
{
	if (cs35l41->amp_en) {
		cs35l41->esd_detect_en = true;
	}
	return 0;

}

static int pm_event_handler(struct notifier_block *notifier,
                       unsigned long pm_event, void *unused)
{
	struct cs35l41_private *cs35l41 = container_of(notifier,
									struct cs35l41_private,
									pm_nb);

    switch (pm_event) {
    case PM_SUSPEND_PREPARE:
           cs35l41_runtime_suspend(cs35l41);
           return NOTIFY_DONE;
    case PM_POST_SUSPEND:
           cs35l41_runtime_resume(cs35l41);
           return NOTIFY_DONE;
    }
    return NOTIFY_DONE;
}
#endif /* CONFIG_PM */


#ifdef MBOX_MODE

static bool cs35l41_is_csplmboxsts_correct(enum cs35l41_cspl_mboxcmd cmd,
                                                                        enum cs35l41_cspl_mboxstate sts)
{
              switch (cmd) {
              case CSPL_MBOX_CMD_NONE:
              case CSPL_MBOX_CMD_UNKNOWN_CMD:
                           return true;
              case CSPL_MBOX_CMD_PAUSE:
                           return (sts == CSPL_MBOX_STS_PAUSED);
              case CSPL_MBOX_CMD_RESUME:
                           return (sts == CSPL_MBOX_STS_RUNNING);
              case CSPL_MBOX_CMD_REINIT:
                           return (sts == CSPL_MBOX_STS_RUNNING);
              case CSPL_MBOX_CMD_STOP_PRE_REINIT:
                           return (sts == CSPL_MBOX_STS_RDY_FOR_REINIT);
              default:
                           return false;
              }
}

static int cs35l41_set_csplmboxcmd(struct cs35l41_private *cs35l41,
                                                          enum cs35l41_cspl_mboxcmd cmd)
{
              int                       ret = 0;
              unsigned int      sts, i;
              bool                    ack = false;

              /* Reset DSP sticky bit */
              regmap_write(cs35l41->regmap, CS35L41_IRQ2_STATUS2,
                                1 << CS35L41_CSPL_MBOX_CMD_DRV_SHIFT);

              /* Reset AP sticky bit */
              regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS2,
                                1 << CS35L41_CSPL_MBOX_CMD_FW_SHIFT);

              /*
              * Set mailbox cmd
              */
              /* Unmask DSP INT */
              regmap_update_bits(cs35l41->regmap, CS35L41_IRQ2_MASK2,
                                            1 << CS35L41_CSPL_MBOX_CMD_DRV_SHIFT, 0);
              regmap_write(cs35l41->regmap, CS35L41_CSPL_MBOX_CMD_DRV, cmd);

              /* Poll for DSP ACK */
              for (i = 0; i < 5; i++) {
                   usleep_range(1000, 1010);
                   ret = regmap_read(cs35l41->regmap, CS35L41_IRQ1_STATUS2, &sts);
                   if (ret < 0) {
                                 dev_err(cs35l41->dev, "regmap_read failed (%d)\n", ret);
                                 continue;
                   }
                   if (sts & (1 << CS35L41_CSPL_MBOX_CMD_FW_SHIFT)) {
                                 dev_dbg(cs35l41->dev,
                                               "%u: Received ACK in EINT for mbox cmd (%d)\n",
                                               i, cmd);
                                 regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS2,
                                      1 << CS35L41_CSPL_MBOX_CMD_FW_SHIFT);
                                 ack = true;
                                 break;
                   }
              }

              if (!ack) {
                           dev_err(cs35l41->dev,
                                         "Timout waiting for DSP to set mbox cmd\n");
                           ret = -ETIMEDOUT;
              }

              /* Mask DSP INT */
              regmap_update_bits(cs35l41->regmap, CS35L41_IRQ2_MASK2,
                                            1 << CS35L41_CSPL_MBOX_CMD_DRV_SHIFT,
                                            1 << CS35L41_CSPL_MBOX_CMD_DRV_SHIFT);

              if (regmap_read(cs35l41->regmap,
                                         CS35L41_CSPL_MBOX_STS, &sts) < 0) {
                           dev_err(cs35l41->dev, "Failed to read %u\n",
                                         CS35L41_CSPL_MBOX_STS);
                           ret = -EACCES;
              }

              if (!cs35l41_is_csplmboxsts_correct(cmd, (enum cs35l41_cspl_mboxstate)sts)) {
                           dev_err(cs35l41->dev,
                                         "Failed to set mailbox(cmd: %u, sts: %u)\n", cmd, sts);
                           ret = -ENOMSG;
              }

              return ret;
}

#endif
static unsigned int cs35l41_dsp_reg(struct cs35l41_private *cs35l41,
			const char *coeff_name, const unsigned char block_type)
{
	struct cs35l41_coeff_desc *coeff_desc = NULL;

	list_for_each_entry(coeff_desc, &cs35l41->coeff_desc_head, list) {
		if (strcmp(coeff_desc->name, coeff_name))
			continue;
		if (coeff_desc->block_type != block_type)
			continue;

		return coeff_desc->reg;
	}

	/* return an identifiable register that is known to be read-only */
	return CS35L41_DEVID;
}
static int low_temperature_state_get(void)
{
	struct power_supply * batt_psy;
	int state_ret = LOW_TEMP_STATE_DEFAULT;
	int temp = 0;
	int vol = 0;
	
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("batt_psy get failed\n");
		return LOW_TEMP_STATE_OUT;
	}
	temp = power_supply_return_int_property_value_with_psy(batt_psy,
		POWER_SUPPLY_PROP_TEMP) / 10;
	vol = power_supply_return_int_property_value_with_psy(batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW);
	power_supply_put(batt_psy);
	pr_info("battery temp %d vol %d\n", temp ,vol);
	
	if(temp <= LOW_TEMP_STATE_IN_TEMP) {
			if(vol <= LOW_TEMP_STATE_STATE2_VOLTAGE)
					state_ret = LOW_TEMP_STATE_IN_STAGE2;
			else if(vol <= LOW_TEMP_STATE_STATE1_VOLTAGE)
					state_ret = LOW_TEMP_STATE_IN_STAGE1;
	} else {
		state_ret = LOW_TEMP_STATE_OUT;
		
	}
	pr_info("low temperature state in %d\n", state_ret);
	return state_ret;
}
static int cs35l41_spk_power_on(struct cs35l41_private *cs35l41)
{
	int ret, val, i;
	int dsp_timeout = CS35L41_DSP_TIMEOUT_COUNT;
	u32 dsp_scratch[4];
	int low_temperature_state = LOW_TEMP_STATE_DEFAULT;

	regmap_write(cs35l41->regmap, CS35L41_DSP1_CTRL_BASE +
		     CS35L41_HALO_CCM_CORE_CONTROL,
		     CS35L41_DSP1_EN_MASK);

	regmap_write(cs35l41->regmap, CS35L41_ASP_TX1_SRC, 0x32);

	regmap_multi_reg_write_bypassed(cs35l41->regmap, cs35l41_pup_patch,
					ARRAY_SIZE(cs35l41_pup_patch));

	low_temperature_state = low_temperature_state_get();
	if (low_temperature_state == LOW_TEMP_STATE_IN_STAGE1 ||
				low_temperature_state == LOW_TEMP_STATE_IN_STAGE2) {
					regmap_multi_reg_write(cs35l41->regmap, 
							cs35l41_spk_power_on_patch_low_temp,
							ARRAY_SIZE(cs35l41_spk_power_on_patch_low_temp));
	} else {
			regmap_multi_reg_write(cs35l41->regmap, cs35l41_spk_power_on_patch,
							ARRAY_SIZE(cs35l41_spk_power_on_patch));
	}

	regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL1,
			   CS35L41_GLOBAL_EN_MASK,
			   1 << CS35L41_GLOBAL_EN_SHIFT);

#ifdef MBOX_MODE
		cs35l41_set_csplmboxcmd(cs35l41, CSPL_MBOX_CMD_RESUME);
#endif

	while (dsp_timeout > 0) {
		usleep_range(10000, 10100);

		ret = regmap_read(cs35l41->regmap,
				  cs35l41_dsp_reg(cs35l41, "HALO_STATE",
					CS35L41_XM_UNPACKED_TYPE), &val);
		if (ret) {
			dev_err(cs35l41->dev, "Failed to read DSP status\n");
			return ret;
		}

		if (val == CS35L41_HALO_STATE_RUNNING)
			break;

		dsp_timeout--;
	}

	if (dsp_timeout == 0) {
		dev_dbg(cs35l41->dev, "DSP status = %d\n", val);
		for (i = 0; i < ARRAY_SIZE(dsp_scratch); i++) {
				regmap_read(cs35l41->regmap, 
							CS35L41_DSP1_CTRL_BASE + HALO_SCRATCH1 + 8 * i, dsp_scratch + i);
				dev_info(cs35l41->dev, "show DSP SCRATCH : dsp_scratch[%d] = 0x%08x\n", i , dsp_scratch[i]);
		}
		return -ETIMEDOUT;
	}

	cs35l41->amp_en = true;

	return 0;
}

static int cs35l41_spk_power_down(struct cs35l41_private *cs35l41)
{
	int i;
	unsigned int val;
	bool pdn = false;

#ifdef MBOX_MODE
	if (cs35l41->scene != -1)
		cs35l41_set_csplmboxcmd(cs35l41, CSPL_MBOX_CMD_PAUSE);
#endif

	regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL1,
			CS35L41_GLOBAL_EN_MASK, 0);

	for (i = 0; i < 200; i++) {
		regmap_read(cs35l41->regmap, CS35L41_IRQ1_STATUS1, &val);
		if (val & CS35L41_PDN_DONE_MASK) {
			pdn = true;
			break;
		}
		usleep_range(1000, 1010);
	}

	if (!pdn)
		dev_warn(cs35l41->dev, "PDN failed\n");

	regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
			 CS35L41_PDN_DONE_MASK);

	regmap_multi_reg_write(cs35l41->regmap, cs35l41_pdn_patch,
			      ARRAY_SIZE(cs35l41_pdn_patch));

	regmap_write(cs35l41->regmap, CS35L41_SP_ENABLES, 0);

	/*regmap_write(cs35l41->regmap, CS35L41_DSP1_CTRL_BASE +
		     CS35L41_HALO_CCM_CORE_CONTROL, 0);*/

	cs35l41->amp_en = false;

	return 0;
}

static int cs35l41_coeff_init(struct cs35l41_private *cs35l41)
{
	struct regmap *regmap = cs35l41->regmap;
	struct device *dev = cs35l41->dev;
	struct cs35l41_coeff_desc *coeff_desc = NULL;
	unsigned int reg = CS35L41_XM_FW_ID;
	unsigned int val;
	int ret, i;

	ret = regmap_read(regmap, CS35L41_XM_NUM_ALGOS, &val);
	if (ret) {
		dev_err(dev, "Failed to read number of algorithms\n");
		return ret;
	}

	if (val > CS35L41_NUM_ALGOS_MAX) {
		dev_err(dev, "Invalid number of algorithms\n");
		return -EINVAL;
	}
	cs35l41->num_algos = val + 1;

	for (i = 0; i < cs35l41->num_algos; i++) {
		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_ID_OFFSET,
				&cs35l41->algo_info[i].id);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d ID\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_REV_OFFSET,
				&cs35l41->algo_info[i].rev);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d revision\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_XM_BASE_OFFSET,
				&cs35l41->algo_info[i].xm_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_XM_SIZE_OFFSET,
				&cs35l41->algo_info[i].xm_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_SIZE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_YM_BASE_OFFSET,
				&cs35l41->algo_info[i].ym_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS35L41_ALGO_YM_SIZE_OFFSET,
				&cs35l41->algo_info[i].ym_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_SIZE\n", i);
			return ret;
		}

		list_for_each_entry(coeff_desc,
			&cs35l41->coeff_desc_head, list) {

			if (coeff_desc->parent_id != cs35l41->algo_info[i].id)
				continue;

			switch (coeff_desc->block_type) {
			case CS35L41_XM_UNPACKED_TYPE:
				coeff_desc->reg = CS35L41_DSP1_XMEM_UNPACK24_0
					+ cs35l41->algo_info[i].xm_base * 4
					+ coeff_desc->block_offset * 4;
				break;
			case CS35L41_YM_UNPACKED_TYPE:
				coeff_desc->reg = CS35L41_DSP1_YMEM_UNPACK24_0
					+ cs35l41->algo_info[i].ym_base * 4
					+ coeff_desc->block_offset * 4;
				break;
			}

			dev_info(dev, "Found control %s at 0x%08X\n",
					coeff_desc->name, coeff_desc->reg);
		}

		/* system algo. contains one extra register (num. algos.) */
		if (i)
			reg += CS35L41_ALGO_ENTRY_SIZE;
		else
			reg += (CS35L41_ALGO_ENTRY_SIZE + 4);
	}

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read list terminator\n");
		return ret;
	}

	if (val != CS35L41_ALGO_LIST_TERM) {
		dev_err(dev, "Invalid list terminator: 0x%X\n", val);
		return -EINVAL;
	}

	return 0;
}

static int cs35l41_raw_write(struct cs35l41_private *cs35l41, unsigned int reg,
		const void *val, size_t val_len, size_t limit)
{
	int ret, i;

	/* split "val" into smaller writes not to exceed "limit" in length */
	for (i = 0; i < val_len; i += limit) {
		ret = regmap_raw_write(cs35l41->regmap, (reg + i), (val + i),
				(val_len - i) > limit ? limit : (val_len - i));
		if (ret)
			break;
	}

	return ret;
}
static int cs35l41_coeff_file_parse(struct cs35l41_private *cs35l41,
			const struct firmware *fw)
{
	struct device *dev = cs35l41->dev;
	unsigned int pos = CS35L41_WT_FILE_HEADER_SIZE;
	unsigned int block_offset, block_type, block_length;
	unsigned int algo_id, algo_rev, reg;
	char mem_str[64];
	int ret = -EINVAL;
	int i;
	int burst_multiple = 4;
	int to_write;
	dev_info(dev, "COEFF File Parse...\n");

	if (memcmp(fw->data, "WMDR", 4)) {
		dev_err(dev, "Failed to recognize coefficient file\n");
		goto err_rls_fw;
	}

	while (pos < fw->size) {
		block_offset = fw->data[pos]
				+ (fw->data[pos + 1] << 8);
		pos += CS35L41_WT_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos]
				+ (fw->data[pos + 1] << 8);
		pos += CS35L41_WT_DBLK_TYPE_SIZE;

		algo_id = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS35L41_WT_ALGO_ID_SIZE;

		algo_rev = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS35L41_WT_ALGO_REV_SIZE;

		/* sample rate is not used here */
		pos += CS35L41_WT_SAMPLE_RATE_SIZE;

		block_length = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS35L41_WT_DBLK_LENGTH_SIZE;

		for (i = 0; i < cs35l41->num_algos; i++)
			if (algo_id == cs35l41->algo_info[i].id)
				break;
		if (i == cs35l41->num_algos &&
			((block_type == CS35L41_XM_UNPACKED_TYPE) ||
			(block_type == CS35L41_YM_UNPACKED_TYPE) ||
			(block_type == CS35L41_XM_PACKED_TYPE) ||
			(block_type == CS35L41_YM_PACKED_TYPE))) {
			dev_err(dev, "Invalid algo. ID: 0x%06X\n",
				algo_id);
			ret = -EINVAL;
			goto err_rls_fw;
		}

		switch (block_type) {
		case CS35L41_XM_UNPACKED_TYPE:
			reg = CS35L41_DSP1_XMEM_UNPACK24_0 + block_offset
					+ cs35l41->algo_info[i].xm_base * 4;
			strcpy(mem_str, "XM_UNPACKED");
			break;
		case CS35L41_YM_UNPACKED_TYPE:
			reg = CS35L41_DSP1_YMEM_UNPACK24_0 + block_offset
					+ cs35l41->algo_info[i].ym_base * 4;
			strcpy(mem_str, "YM_UNPACKED");
			break;
		case CS35L41_XM_PACKED_TYPE:
			reg = ((CS35L41_DSP1_XMEM_PACK_0 +
				cs35l41->algo_info[i].xm_base * 3) & ~0x3) +
				block_offset;
			strcpy(mem_str, "XM_PACKED");
			burst_multiple = 12;
			break;
		case CS35L41_YM_PACKED_TYPE:
			reg = ((CS35L41_DSP1_YMEM_PACK_0 +
				cs35l41->algo_info[i].ym_base * 3) & ~0x3) +
				block_offset;
			strcpy(mem_str, "YM_PACKED");
			burst_multiple = 12;
			break;
		default:
			goto next_block;
		}

		/*if (((algo_rev >> 8) & CS35L41_ALGO_REV_MASK)
				!= (cs35l41->algo_info[i].rev
					& CS35L41_ALGO_REV_MASK)) {
			dev_err(dev, "Invalid algo. rev.: %d.%d.%d\n",
				(algo_rev & 0xFF000000) >> 24,
				(algo_rev & 0xFF0000) >> 16,
				(algo_rev & 0xFF00) >> 8);
			ret = -EINVAL;
			goto err_rls_fw;
		}*/
		to_write = (CS35L41_MAX_WLEN - (CS35L41_MAX_WLEN % burst_multiple));
		if (block_length <= to_write) {
				ret = cs35l41_raw_write(cs35l41, reg, &fw->data[pos],
						block_length, block_length);
		} else {
				ret = cs35l41_raw_write(cs35l41, reg, &fw->data[pos],
						block_length, to_write);
		}

		if (ret) {
			dev_err(dev,
				"Failed to write %s memory\n", mem_str);
			goto err_rls_fw;
		}
		else
			dev_info(dev, "COEFF Write to %s (block_offset = %d, block_length = %d\n",
				 mem_str, block_offset, block_length);


next_block:
		pos += ((block_length + 3) & ~0x03);
	}
#ifdef MBOX_MODE
	ret = regmap_write(cs35l41->regmap,
	  cs35l41_dsp_reg(cs35l41, "CAL_STATUS",
		CS35L41_XM_UNPACKED_TYPE), cs35l41->calib_data.status);
	ret |= regmap_write(cs35l41->regmap,
	  cs35l41_dsp_reg(cs35l41, "CAL_CHECKSUM",
		CS35L41_XM_UNPACKED_TYPE), cs35l41->calib_data.checksum);
	ret = regmap_write(cs35l41->regmap,
	  cs35l41_dsp_reg(cs35l41, "CAL_R",
		CS35L41_XM_UNPACKED_TYPE), cs35l41->calib_data.rdc);
	if (ret)
		dev_err(cs35l41->dev, "Failed to set ReDC\n");
	else
		dev_info(cs35l41->dev, "Sync ReDC 0x%x to dsp before tuning reinit\n", cs35l41->calib_data.rdc);

#endif

err_rls_fw:
	release_firmware(fw);

	return ret;
}

static void cs35l41_coeff_file_load(const struct firmware *fw, void *context)
{
	struct cs35l41_private *cs35l41 = (struct cs35l41_private *)context;
	int ret;

	if (fw) {
		regmap_write(cs35l41->regmap, CS35L41_DSP1_CTRL_BASE +
		     CS35L41_HALO_CCM_CORE_CONTROL,
		     CS35L41_DSP1_RESET_MASK);

		ret = cs35l41_coeff_file_parse(cs35l41, fw);
		if (ret)
			return;

		dev_info(cs35l41->dev, "Firmware revision %d.%d.%d\n",
				(cs35l41->algo_info[0].rev & 0xFF0000) >> 16,
				(cs35l41->algo_info[0].rev & 0xFF00) >> 8,
				cs35l41->algo_info[0].rev & 0xFF);

		regmap_multi_reg_write_bypassed(cs35l41->regmap,
					cs35l41_clear_stream_arb_patch,
					ARRAY_SIZE(cs35l41_clear_stream_arb_patch));

		regmap_multi_reg_write_bypassed(cs35l41->regmap,
					cs35l41_mpu_unlock_patch,
					ARRAY_SIZE(cs35l41_mpu_unlock_patch));
	}
}

static int cs35l41_algo_parse(struct cs35l41_private *cs35l41,
		const unsigned char *data)
{
	struct cs35l41_coeff_desc *coeff_desc = NULL;
	unsigned int pos = 0;
	unsigned int algo_id, algo_desc_length, coeff_count;
	unsigned int block_offset, block_type, block_length;
	unsigned char algo_name_length;
	int i;

	/* record algorithm ID */
	algo_id = *(data + pos)
			+ (*(data + pos + 1) << 8)
			+ (*(data + pos + 2) << 16)
			+ (*(data + pos + 3) << 24);
	pos += CS35L41_ALGO_ID_SIZE;

	/* skip past algorithm name */
	algo_name_length = *(data + pos);
	pos += ((algo_name_length / 4) * 4) + 4;

	/* skip past algorithm description */
	algo_desc_length = *(data + pos)
			+ (*(data + pos + 1) << 8);
	pos += ((algo_desc_length / 4) * 4) + 4;

	/* record coefficient count */
	coeff_count = *(data + pos)
			+ (*(data + pos + 1) << 8)
			+ (*(data + pos + 2) << 16)
			+ (*(data + pos + 3) << 24);
	pos += CS35L41_COEFF_COUNT_SIZE;

	for (i = 0; i < coeff_count; i++) {
		block_offset = *(data + pos)
				+ (*(data + pos + 1) << 8);
		pos += CS35L41_COEFF_OFFSET_SIZE;

		block_type = *(data + pos)
				+ (*(data + pos + 1) << 8);
		pos += CS35L41_COEFF_TYPE_SIZE;

		block_length = *(data + pos)
				+ (*(data + pos + 1) << 8)
				+ (*(data + pos + 2) << 16)
				+ (*(data + pos + 3) << 24);
		pos += CS35L41_COEFF_LENGTH_SIZE;

		coeff_desc = devm_kzalloc(cs35l41->dev, sizeof(*coeff_desc),
				GFP_KERNEL);
		if (!coeff_desc)
			return -ENOMEM;

		coeff_desc->parent_id = algo_id;
		coeff_desc->block_offset = block_offset;
		coeff_desc->block_type = block_type;

		memcpy(coeff_desc->name, data + pos + 1, *(data + pos));
		coeff_desc->name[*(data + pos)] = '\0';

		list_add(&coeff_desc->list, &cs35l41->coeff_desc_head);

		pos += block_length;
	}

	return 0;
}

static int cs35l41_firmware_parse(struct cs35l41_private *cs35l41,
			const struct firmware *fw)
{
	struct device *dev = cs35l41->dev;
	unsigned int pos = CS35L41_FW_FILE_HEADER_SIZE, reg;
	unsigned int block_offset, block_type, block_length;
	char mem_str[64];
	int ret = -EINVAL;
	int burst_multiple = 4;
	int to_write;

	if (memcmp(fw->data, "WMFW", 4)) {
		dev_err(dev, "Failed to recognize firmware file\n");
		return ret;
	}

	while (pos < fw->size) {
		block_offset = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16);
		pos += CS35L41_FW_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos];
		pos += CS35L41_FW_DBLK_TYPE_SIZE;

		block_length = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS35L41_FW_DBLK_LENGTH_SIZE;

		switch (block_type) {
		case CS35L41_PM_PACKED_TYPE:
			reg = CS35L41_DSP1_PMEM_0 + block_offset * 5;
			strcpy(mem_str, "PM_PACKED");
			burst_multiple = 20;
			break;
		case CS35L41_XM_PACKED_TYPE:
			reg = CS35L41_DSP1_XMEM_PACK_0 + block_offset * 3;
			strcpy(mem_str, "XM_PACKED");
			burst_multiple = 12;
			break;
		case CS35L41_YM_PACKED_TYPE:
			reg = CS35L41_DSP1_YMEM_PACK_0 + block_offset * 3;
			strcpy(mem_str, "YM_PACKED");
			burst_multiple = 12;
			break;
		case CS35L41_ALGO_INFO_TYPE:
			ret = cs35l41_algo_parse(cs35l41, &fw->data[pos]);
			if (ret) {
				dev_err(dev,
					"Failed to parse algorithm: %d\n", ret);
				return ret;
			}
			/* Fall through */
		default:
			goto skip_reg_write;
		}

		to_write = (CS35L41_MAX_WLEN - (CS35L41_MAX_WLEN % burst_multiple));
		if (block_length <= to_write) {
				ret = cs35l41_raw_write(cs35l41, reg, &fw->data[pos],
						block_length, block_length);
		} else {
				ret = cs35l41_raw_write(cs35l41, reg, &fw->data[pos],
						block_length, to_write);
		}
		if (ret) {
			dev_err(dev,
				"Failed to write %s memory\n", mem_str);
			return ret;
		}
		else
			dev_info(dev, "FW Write to %s (block_offset = %d, block_length = %d\n",
				 mem_str, block_offset, block_length);

skip_reg_write:
		pos += block_length;
	}

	ret = cs35l41_coeff_init(cs35l41);


	return ret;
}

static void cs35l41_firmware_load(const struct firmware *fw, void *context)
{
	struct cs35l41_private *cs35l41 = (struct cs35l41_private *)context;
	struct device *dev = cs35l41->dev;
	int ret;

	if (!fw) {
		dev_err(dev, "Failed to request firmware file\n");
		return;
	}

	ret = cs35l41_firmware_parse(cs35l41, fw);
	if (ret) {
		dev_err(dev, "Failed to load firmware file, retry load once\n");
		ret = cs35l41_firmware_parse(cs35l41, fw);
		if(ret) {
				dev_err(dev, "Failed to load firmware file finally\n");
			goto err_rls_fw;
		}
	}
	cs35l41->halo_booted = true;

	err_rls_fw:
		release_firmware(fw);

	return;
}

static int cs35l41_live_coeff_load(struct cs35l41_private *cs35l41,
					const char *filename)
{
	struct device *dev = cs35l41->dev;
	int timeout, state, ret;
	const struct firmware *coeff_file = NULL;

	/* Send STOP_PRE_REINIT command and poll for response */
	regmap_write(cs35l41->regmap, CS35L41_CSPL_MBOX_CMD_DRV,
			CSPL_MBOX_CMD_STOP_PRE_REINIT);
	timeout = 100;
	do {
		dev_info(dev, "waiting for REINIT ready...\n");
		usleep_range(1000, 1500);
		regmap_read(cs35l41->regmap, CS35L41_CSPL_MBOX_STS,
				&state);
	} while ((state != CSPL_MBOX_STS_RDY_FOR_REINIT) &&
			--timeout > 0);

	if (timeout == 0) {
		dev_err(dev, "Timeout waiting for REINIT ready\n");
		return -ETIME;
	}

	ret = request_firmware(&coeff_file, filename, dev);

	if (ret != 0) {
		dev_err(dev, "Failed to request '%s'\n", filename);
		return ret;
	}
	else {
		ret = cs35l41_coeff_file_parse(cs35l41, coeff_file);
		if (ret) {
			dev_err(dev, "Failed to load '%s'\n", filename);
			return ret;
		}
	}

	/* Send REINIT command and poll for response */
	regmap_write(cs35l41->regmap, CS35L41_CSPL_MBOX_CMD_DRV,
			CSPL_MBOX_CMD_REINIT);
	timeout = 100;
	do {
		dev_info(dev, "waiting for REINIT done...\n");
		usleep_range(1000, 1500);
		regmap_read(cs35l41->regmap, CS35L41_CSPL_MBOX_STS,
				&state);
	} while ((state != CSPL_MBOX_STS_RUNNING) &&
			--timeout > 0);

	if (timeout == 0) {
		dev_err(dev, "Timeout waiting for REINIT ready\n");
		return -ETIME;
	}
	dev_info(dev, "live tuning %s switch success \n", filename);
	return 0;
}

static irqreturn_t cs35l41_irq(int irq, void *data)
{
	struct cs35l41_private *cs35l41 = data;
	unsigned int status[4];
	unsigned int masks[4];
	unsigned int i;
	int ret;
	unsigned int die_temp, spk_temp;
	dev_crit(cs35l41->dev, "cs35l41 one interrupt triggered\n");
	for (i = 0; i < ARRAY_SIZE(status); i++) {
		regmap_read(cs35l41->regmap,
			    CS35L41_IRQ1_STATUS1 + (i * CS35L41_REGSTRIDE),
			    &status[i]);
		regmap_read(cs35l41->regmap,
			    CS35L41_IRQ1_MASK1 + (i * CS35L41_REGSTRIDE),
			    &masks[i]);
	}

	/* Check to see if unmasked bits are active */
	if (!(status[0] & ~masks[0]) && !(status[1] & ~masks[1]) &&
		!(status[2] & ~masks[2]) && !(status[3] & ~masks[3]))
		return IRQ_NONE;

	if (status[1] & (1 << CS35L41_CSPL_MBOX_CMD_FW_SHIFT)) {
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS2,
			     1 << CS35L41_CSPL_MBOX_CMD_FW_SHIFT);
		complete(&cs35l41->mbox_cmd);
	}


	/*
	 * The following interrupts require a
	 * protection release cycle to get the
	 * speaker out of Safe-Mode.
	 */
	if (status[0] & CS35L41_AMP_SHORT_ERR) {
		dev_crit(cs35l41->dev, "Amp short error\n");
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_AMP_SHORT_ERR);
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_AMP_SHORT_ERR_RLS,
					CS35L41_AMP_SHORT_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_AMP_SHORT_ERR_RLS, 0);
	}

	if (status[0] & CS35L41_TEMP_WARN_RISE) {
		regmap_read(cs35l41->regmap, CS35L41_DTEMP_EN, &die_temp);
		regmap_read(cs35l41->regmap, cs35l41_dsp_reg(cs35l41, "CSPL_TEMPERATURE", CS35L41_XM_UNPACKED_TYPE), &spk_temp);
		dev_crit(cs35l41->dev, "Over temperature warning rise, die temp=0x%x, spk_temp=0x%x\n", die_temp, spk_temp);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
						CS35L41_TEMP_WARN_RISE);
	}
	if (status[0] & CS35L41_TEMP_WARN_FALL) {
		regmap_read(cs35l41->regmap, CS35L41_DTEMP_EN, &die_temp);
		regmap_read(cs35l41->regmap, cs35l41_dsp_reg(cs35l41, "CSPL_TEMPERATURE", CS35L41_XM_UNPACKED_TYPE), &spk_temp);
		dev_crit(cs35l41->dev, "Over temperature warning fall, die temp=0x%x, spk_temp=0x%x\n", die_temp, spk_temp);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_TEMP_WARN_FALL);
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_TEMP_WARN_ERR_RLS,
					CS35L41_TEMP_WARN_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_TEMP_WARN_ERR_RLS, 0);
		//over temp error release
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_TEMP_ERR_RLS,
					CS35L41_TEMP_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_TEMP_ERR_RLS, 0);
	}


	if (status[0] & CS35L41_TEMP_ERR) {
		regmap_read(cs35l41->regmap, CS35L41_DTEMP_EN, &die_temp);
		regmap_read(cs35l41->regmap, cs35l41_dsp_reg(cs35l41, "CSPL_TEMPERATURE", CS35L41_XM_UNPACKED_TYPE), &spk_temp);
		dev_crit(cs35l41->dev, "Over temperature error, die temp=0x%x, spk_temp=0x%x\n", die_temp, spk_temp);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_TEMP_ERR);
	}

	if (status[0] & CS35L41_BST_OVP_ERR) {
		dev_crit(cs35l41->dev, "VBST Over Voltage error\n");
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK, 0);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_BST_OVP_ERR);
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_OVP_ERR_RLS,
					CS35L41_BST_OVP_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_OVP_ERR_RLS, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK,
					CS35L41_BST_EN_DEFAULT <<
					CS35L41_BST_EN_SHIFT);
	}

	if (status[0] & CS35L41_BST_DCM_UVP_ERR) {
		dev_crit(cs35l41->dev, "DCM VBST Under Voltage Error\n");
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK, 0);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_BST_DCM_UVP_ERR);
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_UVP_ERR_RLS,
					CS35L41_BST_UVP_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_UVP_ERR_RLS, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK,
					CS35L41_BST_EN_DEFAULT <<
					CS35L41_BST_EN_SHIFT);
	}

	if (status[0] & CS35L41_BST_SHORT_ERR) {
		dev_crit(cs35l41->dev, "LBST error: powering off!\n");
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK, 0);
		regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS1,
					CS35L41_BST_SHORT_ERR);
		regmap_write(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_SHORT_ERR_RLS,
					CS35L41_BST_SHORT_ERR_RLS);
		regmap_update_bits(cs35l41->regmap, CS35L41_PROTECT_REL_ERR_IGN,
					CS35L41_BST_SHORT_ERR_RLS, 0);
		regmap_update_bits(cs35l41->regmap, CS35L41_PWR_CTRL2,
					CS35L41_BST_EN_MASK,
					CS35L41_BST_EN_DEFAULT <<
					CS35L41_BST_EN_SHIFT);
	}

	if (status[3] & CS35L41_OTP_BOOT_DONE) {
		regmap_update_bits(cs35l41->regmap, CS35L41_IRQ1_MASK4,
				CS35L41_OTP_BOOT_DONE, CS35L41_OTP_BOOT_DONE);
	}
	
	if (status[2] & CS35L41_REF_CLK_FALL_MISSING) {
		dev_crit(cs35l41->dev, "Reference clock falling missing!\n");
		ret = regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS3,
						CS35L41_REF_CLK_FALL_MISSING);
		if (ret) {
				dev_crit(cs35l41->dev, "Failed to write CS35L41_IRQ1_STATUS3\n");
		}
	}
	if (status[2] & CS35L41_REF_CLK_RISE_MISSING) {
		dev_crit(cs35l41->dev, "Reference clock rising missing!\n");
		ret = regmap_write(cs35l41->regmap, CS35L41_IRQ1_STATUS3,
				CS35L41_REF_CLK_RISE_MISSING);
		if (ret) {
				dev_crit(cs35l41->dev, "Failed to write CS35L41_IRQ1_STATUS3\n");
		}
	}

	return IRQ_HANDLED;
}

static int cs35l41_boost_config(struct cs35l41_private *cs35l41,
		int boost_ind, int boost_cap, int boost_ipk)
{
	int ret;
	unsigned char bst_lbst_val, bst_cbst_range, bst_ipk_scaled;
	struct regmap *regmap = cs35l41->regmap;
	struct device *dev = cs35l41->dev;

	switch (boost_ind) {
	case 1000:	/* 1.0 uH */
		bst_lbst_val = 0;
		break;
	case 1200:	/* 1.2 uH */
		bst_lbst_val = 1;
		break;
	case 1500:	/* 1.5 uH */
		bst_lbst_val = 2;
		break;
	case 2200:	/* 2.2 uH */
		bst_lbst_val = 3;
		break;
	default:
		dev_err(dev, "Invalid boost inductor value: %d nH\n",
				boost_ind);
		return -EINVAL;
	}

	switch (boost_cap) {
	case 0 ... 19:
		bst_cbst_range = 0;
		break;
	case 20 ... 50:
		bst_cbst_range = 1;
		break;
	case 51 ... 100:
		bst_cbst_range = 2;
		break;
	case 101 ... 200:
		bst_cbst_range = 3;
		break;
	default:	/* 201 uF and greater */
		bst_cbst_range = 4;
	}

	ret = regmap_update_bits(regmap, CS35L41_BSTCVRT_COEFF,
			CS35L41_BST_K1_MASK,
			cs35l41_bst_k1_table[bst_lbst_val][bst_cbst_range]
				<< CS35L41_BST_K1_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost K1 coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS35L41_BSTCVRT_COEFF,
			CS35L41_BST_K2_MASK,
			cs35l41_bst_k2_table[bst_lbst_val][bst_cbst_range]
				<< CS35L41_BST_K2_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost K2 coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS35L41_BSTCVRT_SLOPE_LBST,
			CS35L41_BST_SLOPE_MASK,
			cs35l41_bst_slope_table[bst_lbst_val]
				<< CS35L41_BST_SLOPE_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost slope coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS35L41_BSTCVRT_SLOPE_LBST,
			CS35L41_BST_LBST_VAL_MASK,
			bst_lbst_val << CS35L41_BST_LBST_VAL_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost inductor value\n");
		return ret;
	}

	if ((boost_ipk < 1600) || (boost_ipk > 4500)) {
		dev_err(dev, "Invalid boost inductor peak current: %d mA\n",
				boost_ipk);
		return -EINVAL;
	}
	bst_ipk_scaled = ((boost_ipk - 1600) / 50) + 0x10;

	ret = regmap_update_bits(regmap, CS35L41_BSTCVRT_PEAK_CUR,
			CS35L41_BST_IPK_MASK,
			bst_ipk_scaled << CS35L41_BST_IPK_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost inductor peak current\n");
		return ret;
	}

	return 0;
}

static int cs35l41_apply_of_data(struct cs35l41_private *cs35l41)
{
	struct classh_cfg *classh = &cs35l41->pdata.classh_config;
	int ret;

	/* Set Platform Data */
	/* Required */
	if (cs35l41->pdata.bst_ipk &&
			cs35l41->pdata.bst_ind && cs35l41->pdata.bst_cap) {
		ret = cs35l41_boost_config(cs35l41, cs35l41->pdata.bst_ind,
					cs35l41->pdata.bst_cap,
					cs35l41->pdata.bst_ipk);
		if (ret) {
			dev_err(cs35l41->dev, "Error in Boost DT config\n");
			return ret;
		}
	} else {
		dev_err(cs35l41->dev, "Incomplete Boost component DT config\n");
		return -EINVAL;
	}

	/* Optional */
	if (cs35l41->pdata.sclk_frc)
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_FORMAT,
				CS35L41_SCLK_FRC_MASK,
				cs35l41->pdata.sclk_frc <<
				CS35L41_SCLK_FRC_SHIFT);

	if (cs35l41->pdata.lrclk_frc)
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_FORMAT,
				CS35L41_LRCLK_FRC_MASK,
				cs35l41->pdata.lrclk_frc <<
				CS35L41_LRCLK_FRC_SHIFT);

	if (cs35l41->pdata.amp_gain_zc)
		regmap_update_bits(cs35l41->regmap, CS35L41_AMP_GAIN_CTRL,
				CS35L41_AMP_GAIN_ZC_MASK,
				cs35l41->pdata.amp_gain_zc <<
				CS35L41_AMP_GAIN_ZC_SHIFT);

	if (cs35l41->pdata.bst_vctrl)
		regmap_update_bits(cs35l41->regmap, CS35L41_BSTCVRT_VCTRL1,
				CS35L41_BST_CTL_MASK, cs35l41->pdata.bst_vctrl);

	if (cs35l41->pdata.temp_warn_thld)
		regmap_update_bits(cs35l41->regmap, CS35L41_DTEMP_WARN_THLD,
				CS35L41_TEMP_THLD_MASK,
				cs35l41->pdata.temp_warn_thld);

	if (cs35l41->pdata.dout_hiz <= CS35L41_ASP_DOUT_HIZ_MASK &&
	    cs35l41->pdata.dout_hiz >= 0)
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_HIZ_CTRL,
				CS35L41_ASP_DOUT_HIZ_MASK,
				cs35l41->pdata.dout_hiz);

	if (cs35l41->pdata.ng_enable) {
		regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH1_CFG,
				CS35L41_NG_ENABLE_MASK,
				CS35L41_NG_ENABLE_MASK);
		regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH2_CFG,
				CS35L41_NG_ENABLE_MASK,
				CS35L41_NG_ENABLE_MASK);

		if (cs35l41->pdata.ng_pcm_thld) {
			regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH1_CFG,
				CS35L41_NG_THLD_MASK,
				cs35l41->pdata.ng_pcm_thld);
			regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH2_CFG,
				CS35L41_NG_THLD_MASK,
				cs35l41->pdata.ng_pcm_thld);
		}

		if (cs35l41->pdata.ng_delay) {
			regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH1_CFG,
				CS35L41_NG_DELAY_MASK,
				cs35l41->pdata.ng_delay <<
				CS35L41_NG_DELAY_SHIFT);
			regmap_update_bits(cs35l41->regmap,
				CS35L41_MIXER_NGATE_CH2_CFG,
				CS35L41_NG_DELAY_MASK,
				cs35l41->pdata.ng_delay <<
				CS35L41_NG_DELAY_SHIFT);
		}
	}

	if (classh->classh_algo_enable) {
		if (classh->classh_bst_override)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_BSTCVRT_VCTRL2,
					CS35L41_BST_CTL_SEL_MASK,
					CS35L41_BST_CTL_SEL_REG);
		if (classh->classh_bst_max_limit)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_BSTCVRT_VCTRL2,
					CS35L41_BST_LIM_MASK,
					classh->classh_bst_max_limit <<
					CS35L41_BST_LIM_SHIFT);
		if (classh->classh_mem_depth)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_CLASSH_CFG,
					CS35L41_CH_MEM_DEPTH_MASK,
					classh->classh_mem_depth <<
					CS35L41_CH_MEM_DEPTH_SHIFT);
		if (classh->classh_headroom)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_CLASSH_CFG,
					CS35L41_CH_HDRM_CTL_MASK,
					classh->classh_headroom <<
					CS35L41_CH_HDRM_CTL_SHIFT);
		if (classh->classh_release_rate)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_CLASSH_CFG,
					CS35L41_CH_REL_RATE_MASK,
					classh->classh_release_rate <<
					CS35L41_CH_REL_RATE_SHIFT);
		if (classh->classh_wk_fet_delay)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_WKFET_CFG,
					CS35L41_CH_WKFET_DLY_MASK,
					classh->classh_wk_fet_delay <<
					CS35L41_CH_WKFET_DLY_SHIFT);
		if (classh->classh_wk_fet_thld)
			regmap_update_bits(cs35l41->regmap,
					CS35L41_WKFET_CFG,
					CS35L41_CH_WKFET_THLD_MASK,
					classh->classh_wk_fet_thld <<
					CS35L41_CH_WKFET_THLD_SHIFT);
	}

	if (cs35l41->pdata.asp_config.asp_fmt)
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_FORMAT,
				CS35L41_ASP_FMT_MASK,
				cs35l41->pdata.asp_config.asp_fmt <<
				CS35L41_ASP_FMT_SHIFT);

	if (cs35l41->pdata.asp_config.asp_rx_width) {
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_FORMAT,
				CS35L41_ASP_WIDTH_RX_MASK,
				cs35l41->pdata.asp_config.asp_rx_width <<
				CS35L41_ASP_WIDTH_RX_SHIFT);
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_FORMAT,
				CS35L41_ASP_WIDTH_TX_MASK,
				cs35l41->pdata.asp_config.asp_rx_width <<
				CS35L41_ASP_WIDTH_TX_SHIFT);
		}
	if (cs35l41->pdata.asp_config.asp_rx_wl) {
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_RX_WL,
				CS35L41_ASP_RX_WL_MASK,
				cs35l41->pdata.asp_config.asp_rx_wl <<
				CS35L41_ASP_RX_WL_SHIFT);
		regmap_update_bits(cs35l41->regmap, CS35L41_SP_TX_WL,
				CS35L41_ASP_TX_WL_MASK,
				cs35l41->pdata.asp_config.asp_rx_wl <<
				CS35L41_ASP_TX_WL_SHIFT);
		
		}
	if (cs35l41->pdata.asp_config.asp_srate)
		regmap_update_bits(cs35l41->regmap, CS35L41_GLOBAL_CLK_CTRL,
				CS35L41_GLOBAL_FS_MASK,
				cs35l41->pdata.asp_config.asp_srate <<
				CS35L41_GLOBAL_FS_SHIFT);

	if ((cs35l41->pdata.pll_refclk_freq & CS35L41_VALID_PDATA) |
	    (cs35l41->pdata.pll_refclk_sel & CS35L41_VALID_PDATA)) {
		regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_PLL_OPENLOOP_MASK,
				1 << CS35L41_PLL_OPENLOOP_SHIFT);

		if (cs35l41->pdata.pll_refclk_freq & CS35L41_VALID_PDATA)
			regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_REFCLK_FREQ_MASK,
				cs35l41->pdata.pll_refclk_freq <<
				CS35L41_REFCLK_FREQ_SHIFT);

		regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_PLL_CLK_EN_MASK,
				0 << CS35L41_PLL_CLK_EN_SHIFT);


		if (cs35l41->pdata.pll_refclk_freq & CS35L41_VALID_PDATA)
			regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_PLL_CLK_SEL_MASK,
				cs35l41->pdata.pll_refclk_sel);

		regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_PLL_OPENLOOP_MASK,
				0 << CS35L41_PLL_OPENLOOP_SHIFT);

		regmap_update_bits(cs35l41->regmap, CS35L41_PLL_CLK_CTRL,
				CS35L41_PLL_CLK_EN_MASK,
				1 << CS35L41_PLL_CLK_EN_SHIFT);
	}

	return 0;
}

static const struct cs35l41_otp_map_element_t *cs35l41_find_otp_map(u32 otp_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs35l41_otp_map_map); i++) {
		if (cs35l41_otp_map_map[i].id == otp_id)
			return &cs35l41_otp_map_map[i];
	}

	return NULL;
}

static int cs35l41_otp_unpack(void *data)
{
	struct cs35l41_private *cs35l41 = data;
	u32 otp_mem[32];
	u32 otp_ctrl_mem[9];
	u32 ymem_log[32];
	int i;
	int bit_offset, word_offset;
	unsigned int bit_sum = 8;
	u32 otp_val, otp_id_reg;
	const struct cs35l41_otp_map_element_t *otp_map_match = NULL;
	const struct cs35l41_otp_packed_element_t *otp_map = NULL;
	int ret;
	int retry = 3;

	ret = regmap_read(cs35l41->regmap, CS35L41_OTPID, &otp_id_reg);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Read OTP ID failed\n");
		return -EINVAL;
	}

	otp_map_match = cs35l41_find_otp_map(otp_id_reg);

	if (otp_map_match == NULL) {
		dev_err(cs35l41->dev, "OTP Map matching ID %d not found\n",
				otp_id_reg);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ymem_log); i++) {
		regmap_read(cs35l41->regmap,
				CS35L41_DSP1_YMEM_UNPACK32_0 + 4 * i, ymem_log + i);
		dev_dbg(cs35l41->dev, "regmap_read:ymem_log[%d] = 0x%08x\n", i, ymem_log[i]);
	}

	otp_map = otp_map_match->map;

	bit_offset = otp_map_match->bit_offset;
	word_offset = otp_map_match->word_offset;

	ret = regmap_write(cs35l41->regmap, CS35L41_TEST_KEY_CTL, 0x00000055);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Write Unlock key failed 1/2\n");
		return -EINVAL;
	}
	ret = regmap_write(cs35l41->regmap, CS35L41_TEST_KEY_CTL, 0x000000AA);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Write Unlock key failed 2/2\n");
		return -EINVAL;
	}

	for (i = 0; i < otp_map_match->num_elements; i++) {
		dev_dbg(cs35l41->dev, "YMEM to trim, [%d], bitoffset= %d, word_offset=%d, bit_sum mod 32=%d\n",
					i, bit_offset, word_offset, bit_sum % 32);
		if (bit_offset + otp_map[i].size - 1 >= 32) {
			otp_val = (ymem_log[word_offset] &
					GENMASK(31, bit_offset)) >>
					bit_offset;
			otp_val |= (ymem_log[++word_offset] &
					GENMASK(bit_offset +
						otp_map[i].size - 33, 0)) <<
					(32 - bit_offset);
			bit_offset += otp_map[i].size - 32;
		} else {

			otp_val = (ymem_log[word_offset] &
				GENMASK(bit_offset + otp_map[i].size - 1,
					bit_offset)) >>	bit_offset;
			bit_offset += otp_map[i].size;
		}
		bit_sum += otp_map[i].size;

		if (bit_offset == 32) {
			bit_offset = 0;
			word_offset++;
		}

		if (otp_map[i].reg != 0) {
			retry = 3;
			while (retry --) {
				ret = regmap_update_bits(cs35l41->regmap,
							otp_map[i].reg,
							GENMASK(otp_map[i].shift +
								otp_map[i].size - 1,
							otp_map[i].shift),
							otp_val << otp_map[i].shift);
				if (ret < 0) {
					dev_err(cs35l41->dev, "Write OTP val failed, retry once\n");
				} else {
					dev_dbg(cs35l41->dev, "Write OTP val success\n");
					break;
				}
			}
			if ((retry == 0) && (ret < 0)) {
#ifdef CONFIG_HUAWEI_DSM_AUDIO
					audio_dsm_report_info(AUDIO_SMARTPA, DSM_SMARTPA_I2C_ERR, "I2C error, Result:%d, Function:%s, Line:%d\n", ret, __func__, __LINE__);
#endif
				dev_err(cs35l41->dev, "Write OTP val failed finally\n");
				return -EINVAL;
			}
		}
	}
	ret = regmap_write(cs35l41->regmap, CS35L41_TEST_KEY_CTL, 0x000000CC);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Write Lock key failed 1/2\n");
		return -EINVAL;
	}
	ret = regmap_write(cs35l41->regmap, CS35L41_TEST_KEY_CTL, 0x00000033);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Write Lock key failed 2/2\n");
		return -EINVAL;
	}

	return 0;
}

static int cs35l41_irq_gpio_config(struct cs35l41_private *cs35l41)
{
	struct irq_cfg *irq_gpio_cfg1 = &cs35l41->pdata.irq_config1;
	struct irq_cfg *irq_gpio_cfg2 = &cs35l41->pdata.irq_config2;
	int irq_pol = IRQF_TRIGGER_NONE;

	if (irq_gpio_cfg1->is_present) {
		if (irq_gpio_cfg1->irq_pol_inv)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO1_CTRL1,
						CS35L41_GPIO_POL_MASK,
						CS35L41_GPIO_POL_MASK);
		if (irq_gpio_cfg1->irq_out_en)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO1_CTRL1,
						CS35L41_GPIO_DIR_MASK,
						0);
		if (irq_gpio_cfg1->irq_src_sel)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO_PAD_CONTROL,
						CS35L41_GPIO1_CTRL_MASK,
						irq_gpio_cfg1->irq_src_sel <<
						CS35L41_GPIO1_CTRL_SHIFT);
	}

	if (irq_gpio_cfg2->is_present) {
		if (irq_gpio_cfg2->irq_pol_inv)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO2_CTRL1,
						CS35L41_GPIO_POL_MASK,
						CS35L41_GPIO_POL_MASK);
		if (irq_gpio_cfg2->irq_out_en)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO2_CTRL1,
						CS35L41_GPIO_DIR_MASK,
						0);
		if (irq_gpio_cfg2->irq_src_sel)
			regmap_update_bits(cs35l41->regmap,
						CS35L41_GPIO_PAD_CONTROL,
						CS35L41_GPIO2_CTRL_MASK,
						irq_gpio_cfg2->irq_src_sel <<
						CS35L41_GPIO2_CTRL_SHIFT);
	}

	if (irq_gpio_cfg2->irq_src_sel ==
			(CS35L41_GPIO_CTRL_ACTV_LO | CS35L41_VALID_PDATA))
		irq_pol = IRQF_TRIGGER_FALLING;
	else if (irq_gpio_cfg2->irq_src_sel ==
			(CS35L41_GPIO_CTRL_ACTV_HI | CS35L41_VALID_PDATA))
		irq_pol = IRQF_TRIGGER_RISING;

	return irq_pol;
}

static int cs35l41_handle_of_data(struct device *dev,
				struct cs35l41_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int val;
	int ret;
	struct device_node *sub_node = NULL;
	struct classh_cfg *classh_config = &pdata->classh_config;
	struct irq_cfg *irq_gpio1_config = &pdata->irq_config1;
	struct irq_cfg *irq_gpio2_config = &pdata->irq_config2;
	struct asp_cfg *asp_config = &pdata->asp_config;

	if (!np)
		return 0;

	pdata->right_channel = of_property_read_bool(np,
					"cirrus,right-channel-amp");
	pdata->sclk_frc = of_property_read_bool(np,
					"cirrus,sclk-force-output");
	pdata->lrclk_frc = of_property_read_bool(np,
					"cirrus,lrclk-force-output");
	pdata->amp_gain_zc = of_property_read_bool(np,
					"cirrus,amp-gain-zc");

	if (of_property_read_u32(np, "cirrus,temp-warn_threshold", &val) >= 0)
		pdata->temp_warn_thld = val | CS35L41_VALID_PDATA;

	ret = of_property_read_u32(np, "cirrus,boost-ctl-millivolt", &val);
	if (ret >= 0) {
		if (val < 2550 || val > 11000) {
			dev_err(dev,
				"Invalid Boost Voltage %u mV\n", val);
			return -EINVAL;
		}
		pdata->bst_vctrl = ((val - 2550) / 100) + 1;
	}

	ret = of_property_read_u32(np, "cirrus,boost-peak-milliamp", &val);
	if (ret >= 0)
		pdata->bst_ipk = val;

	ret = of_property_read_u32(np, "cirrus,boost-ind-nanohenry", &val);
	if (ret >= 0)
		pdata->bst_ind = val;

	ret = of_property_read_u32(np, "cirrus,boost-cap-microfarad", &val);
	if (ret >= 0)
		pdata->bst_cap = val;

	ret = of_property_read_u32(np, "cirrus,asp-sdout-hiz", &val);
	if (ret >= 0)
		pdata->dout_hiz = val;
	else
		pdata->dout_hiz = -1;

	pdata->ng_enable = of_property_read_bool(np,
					"cirrus,noise-gate-enable");
	if (of_property_read_u32(np, "cirrus,noise-gate-threshold", &val) >= 0)
		pdata->ng_pcm_thld = val | CS35L41_VALID_PDATA;
	if (of_property_read_u32(np, "cirrus,noise-gate-delay", &val) >= 0)
		pdata->ng_delay = val | CS35L41_VALID_PDATA;

	sub_node = of_get_child_by_name(np, "cirrus,classh-internal-algo");
	classh_config->classh_algo_enable = sub_node ? true : false;

	if (classh_config->classh_algo_enable) {
		classh_config->classh_bst_override =
			of_property_read_bool(sub_node,
				"cirrus,classh-bst-overide");

		ret = of_property_read_u32(sub_node,
					   "cirrus,classh-bst-max-limit",
					   &val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			classh_config->classh_bst_max_limit = val;
		}

		ret = of_property_read_u32(sub_node, "cirrus,classh-mem-depth",
					   &val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			classh_config->classh_mem_depth = val;
		}

		ret = of_property_read_u32(sub_node,
					"cirrus,classh-release-rate", &val);
		if (ret >= 0)
			classh_config->classh_release_rate = val;

		ret = of_property_read_u32(sub_node, "cirrus,classh-headroom",
					   &val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			classh_config->classh_headroom = val;
		}

		ret = of_property_read_u32(sub_node,
					"cirrus,classh-wk-fet-delay", &val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			classh_config->classh_wk_fet_delay = val;
		}

		ret = of_property_read_u32(sub_node,
					"cirrus,classh-wk-fet-thld", &val);
		if (ret >= 0)
			classh_config->classh_wk_fet_thld = val;
	}
	of_node_put(sub_node);

	/* GPIO1 Pin Config */
	sub_node = of_get_child_by_name(np, "cirrus,gpio-config1");
	irq_gpio1_config->is_present = sub_node ? true : false;
	if (irq_gpio1_config->is_present) {
		irq_gpio1_config->irq_pol_inv = of_property_read_bool(sub_node,
						"cirrus,gpio-polarity-invert");
		irq_gpio1_config->irq_out_en = of_property_read_bool(sub_node,
						"cirrus,gpio-output-enable");
		ret = of_property_read_u32(sub_node, "cirrus,gpio-src-select",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			irq_gpio1_config->irq_src_sel = val;
		}
	}
	of_node_put(sub_node);

	/* GPIO2 Pin Config */
	sub_node = of_get_child_by_name(np, "cirrus,gpio-config2");
	irq_gpio2_config->is_present = sub_node ? true : false;
	if (irq_gpio2_config->is_present) {
		irq_gpio2_config->irq_pol_inv = of_property_read_bool(sub_node,
						"cirrus,gpio-polarity-invert");
		irq_gpio2_config->irq_out_en = of_property_read_bool(sub_node,
						"cirrus,gpio-output-enable");
		ret = of_property_read_u32(sub_node, "cirrus,gpio-src-select",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			irq_gpio2_config->irq_src_sel = val;
		}
	}
	of_node_put(sub_node);

	ret = of_property_read_u32(np, "cirrus,pll-refclk-sel",
				&val);
	if (ret >= 0) {
		val |= CS35L41_VALID_PDATA;
		pdata->pll_refclk_sel = val;
	}

	ret = of_property_read_u32(np, "cirrus,pll-refclk-freq",
				&val);
	if (ret >= 0) {
		val |= CS35L41_VALID_PDATA;
		pdata->pll_refclk_freq = val;
	}

	/* ASP Config */
	sub_node = of_get_child_by_name(np, "cirrus,asp-config");
	if (sub_node) {
		ret = of_property_read_u32(sub_node, "cirrus,asp-rx-wl",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			asp_config->asp_rx_wl = val;
		}

		ret = of_property_read_u32(sub_node, "cirrus,asp-rx-width",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			asp_config->asp_rx_width = val;
		}

		ret = of_property_read_u32(sub_node, "cirrus,asp-fmt",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			asp_config->asp_fmt = val;
		}

		ret = of_property_read_u32(sub_node, "cirrus,asp-srate",
					&val);
		if (ret >= 0) {
			val |= CS35L41_VALID_PDATA;
			asp_config->asp_srate = val;
		}
	}
	of_node_put(sub_node);

	return 0;
}

static int cs35l41_recovery(struct cs35l41_private *cs35l41);


#ifdef ESD_TEST
static int cs35l41_detect_esd_reset(struct cs35l41_private *cs35l41)
{
	int val, ret;

	regcache_cache_bypass(cs35l41->regmap, true);
	regmap_read(cs35l41->regmap, CS35L41_ESD_STAGE, &val);
	regcache_cache_bypass(cs35l41->regmap, false);
	
	if ((val & CS35L41_ESD_STAGE_FINAL_MASK) != CS35L41_ESD_STAGE_FINAL) {
		dev_err(cs35l41->dev, "ESD reset condition found, attempting recovery...\n");
		ret = cs35l41_recovery(cs35l41);
		if (ret < 0) {
			dev_err(cs35l41->dev, "ESD recovery failed!\n");
			return ret;
		}
		dev_err(cs35l41->dev, "ESD recovery success.\n");
	}
	return 0;
}

static void cs35l41_esd_work(struct work_struct *work)
{
	struct cs35l41_private *cs35l41 = container_of(work,
									struct cs35l41_private,
									esd_work.work);
	int ret;

	while(cs35l41->esd_detect_en) {
		mutex_lock(&cs35l41->lock);
		ret = cs35l41_detect_esd_reset(cs35l41);
		mutex_unlock(&cs35l41->lock);
		if (ret < 0) {
			dev_err(cs35l41->dev, "Failed to resolve ESD reset\n");
		}
		msleep(CS35L41_ESD_DELAY);
	}
}
#endif

static long cs35l41_ioctl(struct file *f, unsigned int cmd,
				 void __user *arg)
{
	struct miscdevice *dev = f->private_data;
	struct cs35l41_private *cs35l41;
	int ret = 0, val, box, scene;
	struct cs35l41_cal_data cal_data = {0};
	char *config = NULL;
	const struct firmware *fw = NULL;

	cs35l41 = container_of(dev, struct cs35l41_private, misc_dev);
	dev_info(cs35l41->dev, "ioctl cmd: %d", cmd);
	mutex_lock(&cs35l41->lock);

	if (copy_from_user(&val, arg, sizeof(val))) {
		dev_err(cs35l41->dev, "copy from user failed\n");
		ret = -EFAULT;
		goto exit;
	}

	switch (cmd) {
	case CS35L41_SPK_DAC_VOLUME:
		break;
	case CS35L41_SPK_POWER_ON:
		dev_info(cs35l41->dev, "cs35axx power on");
#ifdef ESD_TEST
		regcache_cache_bypass(cs35l41->regmap, true);
		regmap_read(cs35l41->regmap, CS35L41_ESD_STAGE, &val);
		regcache_cache_bypass(cs35l41->regmap, false);

		if ((val & CS35L41_ESD_STAGE_FINAL_MASK) != CS35L41_ESD_STAGE_FINAL) {
			cs35l41->amp_en = true;
			ret = cs35l41_detect_esd_reset(cs35l41);
			if (ret < 0) {
				dev_err(cs35l41->dev, "Failed to resolve ESD reset\n");
				break;
			}
		}
		else
			ret = cs35l41_spk_power_on(cs35l41);

		cs35l41->esd_detect_en = true;

		schedule_delayed_work(&cs35l41->esd_work,
					msecs_to_jiffies(CS35L41_ESD_DELAY));
#else
		ret = cs35l41_spk_power_on(cs35l41);
#endif
		break;
	case CS35L41_SPK_POWER_OFF:
		dev_info(cs35l41->dev, "cs35axx power off");
#ifdef ESD_TEST
		cs35l41->esd_detect_en = false;
#endif
		ret = cs35l41_spk_power_down(cs35l41);
		break;
	case CS35L41_SPK_DSP_BYPASS:
		if (val)
			regmap_write(cs35l41->regmap, CS35L41_DAC_PCM1_SRC,
					CS35L41_INPUT_SRC_ASPRX1);
		else
			regmap_write(cs35l41->regmap, CS35L41_DAC_PCM1_SRC,
					CS35L41_INPUT_DSP_TX1);
		break;
	case CS35L41_SPK_SWITCH_CALIBRATION:
		dev_info(cs35l41->dev, "cs35axx calib");
		box = val / 100 -1;
		if ((box > BOX_NUM -1) || (box < 0))
			box = GV_BOX_NUM;
		scene = val % 100;
		if ((scene >= CONFIG_NUM) || (scene < 0)) {
			dev_err(cs35l41->dev, "scene out of range, config = %d, use default\n", val);
			scene = 0;
		}
		//calibration start while scene != 0, else calibration stop
		if (scene && cs35l41->halo_booted) {
			cs35l41->calib_data.status=0;
			cs35l41->calib_data.checksum=0;
			cs35l41->calib_data.rdc=0;
			cs35l41->box = box;
			cs35l41->scene = scene;
			config = CS35L41_COEFF0_NAME[cs35l41->box][cs35l41->scene];
			dev_dbg(cs35l41->dev, "calibration load config = %s\n", config);
			if (cs35l41->amp_en)
					ret = cs35l41_live_coeff_load(cs35l41, config);
				else {
					ret = request_firmware(&fw, config, cs35l41->dev);
	
					if (ret != 0)
						dev_err(cs35l41->dev,
							"Failed to request '%s'\n", config);
					else
						cs35l41_coeff_file_load(fw, cs35l41);
				}
		break;
		}
		//Fall through
	case CS35L41_SPK_SWITCH_CONFIGURATION:
		dev_info(cs35l41->dev, "cs35axx set tuning, val = %d", val);
		//if tuing number out of range, use default tuning
		box = val / 100 -1;
		if ((box > BOX_NUM -1) || (box < 0))
			box = GV_BOX_NUM;
		scene = val % 100;
		if (scene >= CONFIG_NUM || scene < 0) {
			dev_err(cs35l41->dev, "scene out of range, config = %d, use default\n", val);
			scene = 0;
		}
		#ifdef CHECK_DSP_TUNING
		if (scene == cs35l41->scene) {
			dev_dbg(cs35l41->dev, "keep DSP same tuning as before, config = %s\n", CS35L41_COEFF0_NAME[box][scene]);
			break;
		}
		#endif

		dev_dbg(cs35l41->dev, "load DSP tuning, old config = %d, new config tuning = %s\n",
			cs35l41->scene, CS35L41_COEFF0_NAME[box][scene]);

		cs35l41->box = box;
		if (low_temperature_state_get() == LOW_TEMP_STATE_IN_STAGE2 &&
				(scene == CS35L41_CONFIG_RING ||
					scene == CS35L41_CONFIG_RING_HS_SPK)) {
			cs35l41->scene = CS35L41_CONFIG_LOWTEMP;
		} else {
				cs35l41->scene = scene;
		}
		config = CS35L41_COEFF0_NAME[cs35l41->box][cs35l41->scene];
		if (cs35l41->halo_booted) {
			regmap_update_bits(cs35l41->regmap,
									CS35L41_AMP_DIG_VOL_CTRL,
									CS35L41_AMP_VOL_RAMP_MASK <<
									CS35L41_AMP_VOL_RAMP_SHIFT,
									CS35L41_AMP_VOL_RAMP_1MS_6DB <<
									CS35L41_AMP_VOL_RAMP_SHIFT);
			regmap_update_bits(cs35l41->regmap,
									CS35L41_AMP_DIG_VOL_CTRL,
									CS35L41_AMP_VOL_PCM_MASK <<
									CS35L41_AMP_VOL_PCM_SHIFT,
									CS35L41_AMP_VOL_PCM_MUTE <<
									CS35L41_AMP_VOL_PCM_SHIFT);						
			usleep_range(17000, 18000);
			if (cs35l41->amp_en)
				ret = cs35l41_live_coeff_load(cs35l41, config);
			else {
				ret = request_firmware(&fw, config, cs35l41->dev);

				if (ret != 0)
					dev_err(cs35l41->dev,
						"Failed to request '%s'\n", config);
				else
					cs35l41_coeff_file_load(fw, cs35l41);
			}
			regmap_update_bits(cs35l41->regmap,
									CS35L41_AMP_DIG_VOL_CTRL,
									CS35L41_AMP_VOL_RAMP_MASK <<
									CS35L41_AMP_VOL_RAMP_SHIFT,
									CS35L41_AMP_VOL_RAMP_1MS_6DB <<
									CS35L41_AMP_VOL_RAMP_SHIFT);
			regmap_update_bits(cs35l41->regmap,
									CS35L41_AMP_DIG_VOL_CTRL,
									CS35L41_AMP_VOL_PCM_MASK <<
									CS35L41_AMP_VOL_PCM_SHIFT,
									CS35L41_AMP_VOL_PCM_DEFAULT <<
									CS35L41_AMP_VOL_PCM_SHIFT);		
		} else {
			dev_err(cs35l41->dev, "Load firmware before loading configuration\n");
			ret = -EINVAL;
		}
		break;
	case CS35L41_SPK_SWITCH_FIRMWARE:
		if (val == 0) {
			config = CS35L41_FW_NAME;
			cs35l41->diag_en = false;
		}
		else {
			config = CS35L41_FW_DIAG_NAME;
			cs35l41->diag_en = true;
		}

		cs35l41->halo_booted = false;

		regmap_write(cs35l41->regmap, CS35L41_DSP1_CTRL_BASE +
		     		CS35L41_HALO_CCM_CORE_CONTROL, 0);

		ret = request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					config,
					cs35l41->dev,
					GFP_KERNEL, cs35l41,
					cs35l41_firmware_load);

		if (ret < 0)
			cs35l41->diag_en = false;
		break;
	case CS35L41_SPK_GET_R0:
		ret = regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_R",
				CS35L41_XM_UNPACKED_TYPE), &val);
		if (ret) {
			dev_err(cs35l41->dev, "Failed to read R0\n");
		} else {
			ret = copy_to_user(arg, (void *)&val, sizeof(val));
			dev_info(cs35l41->dev, "Read R0: 0x%x\n", val);
		}
		break;
	case CS35L41_SPK_GET_F0:
		ret = regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "DIAG_F0",
				CS35L41_XM_UNPACKED_TYPE), &val);
		if (ret) {
			dev_err(cs35l41->dev, "Failed to read F0\n");
		} else {
			ret = copy_to_user(arg, (void *)&val, sizeof(val));
			dev_info(cs35l41->dev, "Read F0: 0x%x\n", val);
		}
		break;
	case CS35L41_SPK_GET_CAL_STRUCT:
		ret = regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_STATUS",
				CS35L41_XM_UNPACKED_TYPE), &cal_data.status);
		ret |= regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_R",
				CS35L41_XM_UNPACKED_TYPE), &cal_data.rdc);
		ret |= regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_AMBIENT",
				CS35L41_XM_UNPACKED_TYPE), &cal_data.temp);
		ret |= regmap_read(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_CHECKSUM",
				CS35L41_XM_UNPACKED_TYPE), &cal_data.checksum);

		if (ret) {
			dev_err(cs35l41->dev, "Failed to read cal struct\n");
		} else {
			ret = copy_to_user(arg, (void *)&cal_data,
						sizeof(cal_data));
			dev_info(cs35l41->dev, "Read cal struct:\n");
			dev_info(cs35l41->dev, "\tStatus: %d\n",
						cal_data.status);
			dev_info(cs35l41->dev, "\tRDC: 0x%x\n",
						cal_data.rdc);
			dev_info(cs35l41->dev, "\tAmbient: %d\n",
						cal_data.temp);
			dev_info(cs35l41->dev, "\tChecksum: 0x%x\n",
						cal_data.checksum);
		}
		#ifdef MBOX_MODE
		//buffer calib result to private data, write it to DSP after reload tuning
		cs35l41->calib_data = cal_data;
		#endif
		break;
	case CS35L41_SPK_SET_CAL_STRUCT:
		if (copy_from_user(&cal_data, arg, sizeof(cal_data))) {
			dev_err(cs35l41->dev, "copy cal data from user fail\n");
			ret = -EFAULT;
			goto exit;
		}
		dev_info(cs35l41->dev, "Received cal struct:\n");
		dev_info(cs35l41->dev, "\tStatus: %d\n",
					cal_data.status);
		dev_info(cs35l41->dev, "\tRDC: 0x%x\n",
					cal_data.rdc);
		dev_info(cs35l41->dev, "\tAmbient: %d\n",
					cal_data.temp);
		dev_info(cs35l41->dev, "\tChecksum: 0x%x\n",
					cal_data.checksum);
		ret = regmap_write(cs35l41->regmap,
			  	cs35l41_dsp_reg(cs35l41, "CAL_STATUS",
							CS35L41_XM_UNPACKED_TYPE), cal_data.status);
		ret |= regmap_write(cs35l41->regmap,
			  	cs35l41_dsp_reg(cs35l41, "CAL_R",
							CS35L41_XM_UNPACKED_TYPE), cal_data.rdc);
		ret |= regmap_write(cs35l41->regmap,
			  	cs35l41_dsp_reg(cs35l41, "CAL_AMBIENT",
							CS35L41_XM_UNPACKED_TYPE), cal_data.temp);
		ret |= regmap_write(cs35l41->regmap,
			  	cs35l41_dsp_reg(cs35l41, "CAL_CHECKSUM",
							CS35L41_XM_UNPACKED_TYPE), cal_data.checksum);

		if (ret)
			dev_err(cs35l41->dev, "Failed to set cal struct\n");
		break;
	case CS35L41_SPK_SET_AMBIENT:
		if (val) {
			ret = regmap_write(cs35l41->regmap,
			  cs35l41_dsp_reg(cs35l41, "CAL_AMBIENT",
				CS35L41_XM_UNPACKED_TYPE), val);
			if (ret)
				dev_err(cs35l41->dev,
					"Failed to set ambient temperature\n");
			else
				dev_info(cs35l41->dev,
					"Set ambient temp %d degrees C\n", val);
		}
		break;
	case CS35L41_SPK_SET_R0:
		dev_info(cs35l41->dev, "CS35AXX_SPK_SET_R0, value = 0x%x\n", val);
		if (val) {
			cal_data.status = 1;
			cal_data.checksum = val + cal_data.status;
			cal_data.rdc= val;
		} else {			
			//default calib rdc 5.32
			cal_data.status=1;
			cal_data.checksum=0x1cb9;
			cal_data.rdc=0x1cb8;
		}
		#ifdef MBOX_MODE
		//buffer calib result to private data , write it to DSP after reload tuning
			cs35l41->calib_data = cal_data;
		#endif
		break;
	case CS35L41_SPK_GET_R0_REALTIME:
		ret = regmap_read(cs35l41->regmap,
			cs35l41_dsp_reg(cs35l41, "CSPL_TEMPERATURE",
			CS35L41_XM_UNPACKED_TYPE), &val);
		if (ret) {
			dev_err(cs35l41->dev, "Failed to read CSPL_TEMPERATURE\n");
		} else {
			ret = copy_to_user(arg, (void *)&val, sizeof(val));
			dev_info(cs35l41->dev, "Read CSPL_TEMPERATURE: 0x%x\n", val);
		}
		break;
	default:
		dev_err(cs35l41->dev, "Invalid IOCTL, command = %d\n", cmd);
		ret = -EINVAL;
	}

exit:
	mutex_unlock(&cs35l41->lock);

	return ret;
}

static long cs35l41_unlocked_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	return cs35l41_ioctl(f, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long cs35l41_compat_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{

	struct miscdevice *dev = f->private_data;
	struct cs35l41_private *cs35l41;
	unsigned int cmd64;

	cs35l41 = container_of(dev, struct cs35l41_private, misc_dev);

	switch (cmd) {
	case CS35L41_SPK_DAC_VOLUME_COMPAT:
		cmd64 = CS35L41_SPK_DAC_VOLUME;
		break;
	case CS35L41_SPK_POWER_ON_COMPAT:
		cmd64 = CS35L41_SPK_POWER_ON;
		break;
	case CS35L41_SPK_POWER_OFF_COMPAT:
		cmd64 = CS35L41_SPK_POWER_OFF;
		break;
	case CS35L41_SPK_DSP_BYPASS_COMPAT:
		cmd64 = CS35L41_SPK_DSP_BYPASS;
		break;
	case CS35L41_SPK_SWITCH_CONFIGURATION_COMPAT:
		cmd64 = CS35L41_SPK_SWITCH_CONFIGURATION;
		break;
	case CS35L41_SPK_SWITCH_CALIBRATION_COMPAT:
		cmd64 = CS35L41_SPK_SWITCH_CALIBRATION;
		break;
	case CS35L41_SPK_GET_R0_COMPAT:
		cmd64 = CS35L41_SPK_GET_R0;
		break;
	case CS35L41_SPK_GET_F0_COMPAT:
		cmd64 = CS35L41_SPK_GET_F0;
		break;
	case CS35L41_SPK_GET_CAL_STRUCT_COMPAT:
		cmd64 = CS35L41_SPK_GET_CAL_STRUCT;
		break;
	case CS35L41_SPK_SET_CAL_STRUCT_COMPAT:
		cmd64 = CS35L41_SPK_SET_CAL_STRUCT;
		break;
	case CS35L41_SPK_SET_AMBIENT_COMPAT:
		cmd64 = CS35L41_SPK_SET_AMBIENT;
		break;
	case CS35L41_SPK_SET_R0_COMPAT:
		cmd64 = CS35L41_SPK_SET_R0;
		break;
	case CS35L41_SPK_SWITCH_FIRMWARE_COMPAT:
		cmd64 = CS35L41_SPK_SWITCH_FIRMWARE;
		break;
	case CS35L41_SPK_GET_R0_REALTIME_COMPAT:
		cmd64 = CS35L41_SPK_GET_R0_REALTIME;
		break;
	default:
		dev_err(cs35l41->dev, "Invalid IOCTL 32, command = %d\n", cmd);
		return -EINVAL;
	}

	return cs35l41_ioctl(f, cmd64, compat_ptr(arg));
}
#endif

static const struct reg_sequence cs35l41_reva0_errata_patch[] = {
	{0x00000040,			0x00005555},
	{0x00000040,			0x0000AAAA},
	{0x00003854,			0x05180240},
	{CS35L41_VIMON_SPKMON_RESYNC,	0x00000000},
	{CS35L41_OTP_TRIM_30,		0x9091A1C8},
	{0x00003014,			0x0200EE0E},
	{CS35L41_BSTCVRT_DCM_CTRL,	0x00000051},
	{0x00000054,			0x00000004},
	{CS35L41_IRQ1_DB3,		0x00000000},
	{CS35L41_IRQ2_DB3,		0x00000000},
	{CS35L41_DSP1_YM_ACCEL_PL0_PRI,	0x00000000},
	{CS35L41_DSP1_XM_ACCEL_PL0_PRI,	0x00000000},
	{CS35L41_ASP_CONTROL4,		0x00000000},
	{0x00000040,			0x0000CCCC},
	{0x00000040,			0x00003333},
};


static const struct reg_sequence cs35l41_revb0_errata_patch[] = {
	{0x00000040,                    0x00005555},
	{0x00000040,                    0x0000AAAA},
	{CS35L41_BSTCVRT_DCM_CTRL,      0x00000051},
	{CS35L41_DSP1_YM_ACCEL_PL0_PRI, 0x00000000},
	{CS35L41_DSP1_XM_ACCEL_PL0_PRI, 0x00000000},
	{0x00000040,			0x0000CCCC},
	{0x00000040,			0x00003333},
};

static struct regmap_config cs35l41_regmap_i2c = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS35L41_LASTREG,
	.reg_defaults = cs35l41_reg,
	.num_reg_defaults = ARRAY_SIZE(cs35l41_reg),
	.volatile_reg = cs35l41_volatile_reg,
	.readable_reg = cs35l41_readable_reg,
	.precious_reg = cs35l41_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

static const struct file_operations cs35l41_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cs35l41_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cs35l41_compat_ioctl,
#endif
};

static int cs35axx_init(struct cs35l41_private *cs35l41)
{
	int ret;
	u32 regid, reg_revid, mtl_revid, int_status, chipid_match;
	int timeout = 100;
	int irq_pol = 0;
	int gpio_irq = 0;
	
	do {
		if (timeout == 0) {
			dev_err(cs35l41->dev,
				"Timeout waiting for OTP_BOOT_DONE\n");
			ret = -EBUSY;
			goto err;
		}
		usleep_range(1000, 1100);
		regmap_read(cs35l41->regmap, CS35L41_IRQ1_STATUS4, &int_status);
		timeout--;
	} while (!(int_status & CS35L41_OTP_BOOT_DONE));

#ifdef ESD_TEST
	regcache_cache_bypass(cs35l41->regmap, true);
	regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
					   CS35L41_ESD_STAGE1_MASK,
					   CS35L41_ESD_STAGE_FINAL &
					   CS35L41_ESD_STAGE1_MASK);
	regcache_cache_bypass(cs35l41->regmap, false);
#endif

	regmap_read(cs35l41->regmap, CS35L41_IRQ1_STATUS3, &int_status);
	if (int_status & CS35L41_OTP_BOOT_ERR) {
		dev_err(cs35l41->dev, "OTP Boot error\n");
		ret = -EINVAL;
		goto err;
	}

	ret = regmap_read(cs35l41->regmap, CS35L41_DEVID, &regid);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Get Device ID failed\n");
		goto err;
	}

	ret = regmap_read(cs35l41->regmap, CS35L41_REVID, &reg_revid);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Get Revision ID failed\n");
		goto err;
	}

	mtl_revid = reg_revid & CS35L41_MTLREVID_MASK;

	/* CS35L41 will have even MTLREVID
	*  CS35L41R will have odd MTLREVID
	*/
	chipid_match = (mtl_revid % 2) ? CS35L41R_CHIP_ID : CS35L41_CHIP_ID;
	if (regid != chipid_match) {
		dev_err(cs35l41->dev, "CS35L41 Device ID (%X). Expected ID %X\n",
			regid, chipid_match);
		ret = -ENODEV;
		goto err;
	}
	irq_pol = cs35l41_irq_gpio_config(cs35l41);
#ifdef ESD_TEST
		regcache_cache_bypass(cs35l41->regmap, true);
		regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
						   CS35L41_ESD_STAGE2_MASK,
						   CS35L41_ESD_STAGE_FINAL &
						   CS35L41_ESD_STAGE2_MASK);
		regcache_cache_bypass(cs35l41->regmap, false);
#endif

	if (cs35l41->irq >= 0) {
			dev_info(cs35l41->dev, "IRQ GPIO already configured; skipping configuration\n");
			goto skip_irq_config;
	}
	gpio_irq = of_get_named_gpio(cs35l41->dev->of_node, "irq-gpio", 0);
	if (gpio_is_valid(gpio_irq)) {
		ret = devm_gpio_request_one(cs35l41->dev, gpio_irq,
						GPIOF_DIR_IN, "cs35l41_irq");
		if (ret < 0) {
				dev_err(cs35l41->dev, "%s: GPIO %d request INT error\n", __func__, gpio_irq);
				goto err;
		}
		cs35l41->irq = gpio_to_irq(gpio_irq);
		dev_dbg(cs35l41->dev, "CS35L41 irq = %d\n", cs35l41->irq);
		ret = devm_request_threaded_irq(cs35l41->dev, cs35l41->irq, NULL,
					cs35l41_irq, IRQF_ONESHOT | irq_pol,
					"cs35l41", cs35l41);
		if (ret < 0) {
			dev_err(cs35l41->dev, "request_irq %d failed, %d\n", cs35l41->irq, ret);
			goto err;
		}
	} else {
		dev_err(cs35l41->dev, "request_irq_gpio failed, %d\n", ret);
	}
skip_irq_config:
	/* Set interrupt masks for critical errors */
	regmap_write(cs35l41->regmap, CS35L41_IRQ1_MASK1,
			CS35L41_INT1_MASK_DEFAULT);
	regmap_read(cs35l41->regmap, CS35L41_IRQ1_MASK1, &int_status);
	dev_info(cs35l41->dev, "%s: CS35AXX_IRQ1_MASK1 0x%x \n", __func__, int_status);
	//unmask ref clk missing  interrupt
	regmap_write(cs35l41->regmap, CS35L41_IRQ1_MASK3,
			0xffff867f);
	regmap_read(cs35l41->regmap, CS35L41_IRQ1_MASK3, &int_status);
	dev_info(cs35l41->dev, "%s: CS35AXX_IRQ1_MASK3 0x%x \n", __func__, int_status);
	switch (reg_revid) {
	case CS35L41_REVID_A0:
		ret = regmap_multi_reg_write(cs35l41->regmap,
				cs35l41_reva0_errata_patch,
				ARRAY_SIZE(cs35l41_reva0_errata_patch));
		if (ret < 0) {
			dev_err(cs35l41->dev,
				"Failed to apply A0 errata patch %d\n", ret);
			goto err;
		}
		break;
	case CS35L41_REVID_B0:
		ret = regmap_multi_reg_write(cs35l41->regmap,
								cs35l41_revb0_errata_patch,
								ARRAY_SIZE(cs35l41_revb0_errata_patch));
  		if (ret < 0) {
			dev_err(cs35l41->dev,
				"Failed to apply B0 errata patch %d\n", ret);
		}                   
		break;
	}

#ifdef ESD_TEST
		regcache_cache_bypass(cs35l41->regmap, true);
		regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
						   CS35L41_ESD_STAGE3_MASK,
						   CS35L41_ESD_STAGE_FINAL &
						   CS35L41_ESD_STAGE3_MASK);
		regcache_cache_bypass(cs35l41->regmap, false);
#endif

	cs35l41->revid = reg_revid;

	ret = cs35l41_otp_unpack(cs35l41);
	if (ret < 0) {
		dev_err(cs35l41->dev, "OTP Unpack failed\n");
		goto err;
	}
	dev_dbg(cs35l41->dev, "OTP Unpack success\n");
	regmap_write(cs35l41->regmap, CS35L41_DAC_PCM1_SRC,
					CS35L41_INPUT_DSP_TX1);
	regmap_write(cs35l41->regmap, CS35L41_DSP1_RX5_SRC,
					CS35L41_INPUT_SRC_VPMON);
	regmap_write(cs35l41->regmap, CS35L41_DSP1_RX6_SRC,
					CS35L41_INPUT_SRC_CLASSH);
	regmap_write(cs35l41->regmap, CS35L41_DSP1_RX7_SRC,
					CS35L41_INPUT_SRC_TEMPMON);
	regmap_write(cs35l41->regmap, CS35L41_DSP1_RX8_SRC,
					CS35L41_INPUT_SRC_RSVD);

	ret = cs35l41_apply_of_data(cs35l41);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Apply OF data failed\n");
		goto err;
	}

#ifdef ESD_TEST
		regcache_cache_bypass(cs35l41->regmap, true);
		regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
						   CS35L41_ESD_STAGE4_MASK,
						   CS35L41_ESD_STAGE_FINAL &
						   CS35L41_ESD_STAGE4_MASK);
		regcache_cache_bypass(cs35l41->regmap, false);
#endif
	
	dev_info(cs35l41->dev, "Cirrus Logic CS35AXX, Revision: %02X\n", reg_revid);

err:
	return ret;
}

#ifdef ESD_TEST
static int cs35l41_recovery(struct cs35l41_private *cs35l41)
{
	const struct firmware *fw = NULL;
	int ret, scene, box, i, val;
	bool amp_en;
	char *config = NULL;

	amp_en = cs35l41->amp_en;
	box = cs35l41->box;
	scene = cs35l41->scene;

	cs35l41->halo_booted = false;
	cs35l41->amp_en = false;
	cs35l41->diag_en = false;
	cs35l41->box = -1;
	cs35l41->scene = -1;

	dev_err(cs35l41->dev, "CS35L41 RECOVERY MODE\n");

	/* Default to RESUME cmd */
	cs35l41->cspl_cmd = (unsigned int)CSPL_MBOX_CMD_RESUME;

	for (i = 0; i < CS35L41_MAX_CACHE_REG; i++) {
		regcache_write(cs35l41->regmap, cs35l41_reg[i].reg,
						cs35l41_reg[i].def);

		if (cs35l41_reg[i].reg == CS35L41_TEMP_CAL2)
			break;
	}

	regmap_write(cs35l41->regmap, CS35L41_SFT_RESET,
				 CS35L41_SFT_RESET_VAL);

	msleep(5);

	ret = cs35axx_init(cs35l41);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Failed to initialize CS35AXX amplifier: %d\n", ret);
		return ret;
	}

	ret = request_firmware(&fw, CS35L41_FW_NAME, cs35l41->dev);
	if (ret != 0) {
		dev_err(cs35l41->dev,
				 "Failed to request '%s'\n", CS35L41_FW_NAME);
		return -EINVAL;
	}

	ret = cs35l41_firmware_parse(cs35l41, fw);
	if (ret) {
		dev_err(cs35l41->dev, "Failed to load firmware file, retry load once\n");
		ret = cs35l41_firmware_parse(cs35l41, fw);
		if (ret) {
			dev_err(cs35l41->dev, "Failed to load firmware file finally\n");
			return -EINVAL;
		}
	}

#ifdef ESD_TEST
	regcache_cache_bypass(cs35l41->regmap, true);
	regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
					   CS35L41_ESD_STAGE5_MASK,
					   CS35L41_ESD_STAGE_FINAL &
					   CS35L41_ESD_STAGE5_MASK);
	regcache_cache_bypass(cs35l41->regmap, false);
#endif


	cs35l41->halo_booted = true;

	if (amp_en) {
		ret = cs35l41_spk_power_on(cs35l41);
	}
	if (scene >= 0) {
		config = CS35L41_COEFF0_NAME[box][scene];
		if (cs35l41->halo_booted) {
			if (cs35l41->amp_en)
				ret = cs35l41_live_coeff_load(cs35l41, config);
			else {
				ret = request_firmware(&fw, config, cs35l41->dev);
				if (ret != 0)
					dev_err(cs35l41->dev,
							 "Failed to request '%s'\n", config);
				else
					cs35l41_coeff_file_load(fw, cs35l41);
			}
		} else {
			dev_err(cs35l41->dev, "Load firmware before loading configuration\n");
			ret = -EINVAL;
		}
	}

#ifdef ESD_TEST
	regcache_cache_bypass(cs35l41->regmap, true);
	regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
					   CS35L41_ESD_STAGE6_MASK,
					   CS35L41_ESD_STAGE_FINAL &
					   CS35L41_ESD_STAGE6_MASK);
	regmap_read(cs35l41->regmap, CS35L41_ESD_STAGE, &val);
	regcache_cache_bypass(cs35l41->regmap, false);
#endif

	if (ret == 0) {
		if ((val & CS35L41_ESD_STAGE_FINAL_MASK) == CS35L41_ESD_STAGE_FINAL)
			dev_err(cs35l41->dev, "CS35AXX RECOVERY SUCCESS\n");
		else {
			dev_err(cs35l41->dev,
				"CS35L41 ESD recovery did not finish (%x)\n", val);
			return -EINVAL;
		}
	}

	cs35l41->box = box;
	cs35l41->scene = scene;

	return ret;
}
#endif

static int cs35axx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cs35l41_private *cs35l41 = NULL;
	struct device *dev = &client->dev;
	struct cs35l41_platform_data *pdata = dev_get_platdata(dev);
	const struct regmap_config *regmap_config = &cs35l41_regmap_i2c;
	int ret;
	u32 i;
	struct smartpa_vendor_info vendor_info;
	printk(KERN_ERR "cs35axx_i2c_probe begin");

	cs35l41 = devm_kzalloc(dev,
			       sizeof(struct cs35l41_private),
			       GFP_KERNEL);
	if (cs35l41 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, cs35l41);
	cs35l41->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(cs35l41->regmap)) {
		ret = PTR_ERR(cs35l41->regmap);
		dev_err(cs35l41->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	cs35l41->dev = dev;
	cs35l41->irq = client->irq;
	cs35l41->halo_booted = false;
	cs35l41->amp_en = false;
	cs35l41->diag_en = false;
	cs35l41->esd_detect_en = false;
	cs35l41->box = -1;
	cs35l41->scene = -1;

	mutex_init(&cs35l41->lock);

	INIT_LIST_HEAD(&cs35l41->coeff_desc_head);
	

	init_completion(&cs35l41->mbox_cmd);

#ifdef ESD_TEST
	INIT_DELAYED_WORK(&cs35l41->esd_work, cs35l41_esd_work);
#endif

	/* Default to RESUME cmd */
	cs35l41->cspl_cmd = (unsigned int)CSPL_MBOX_CMD_RESUME;

	for (i = 0; i < ARRAY_SIZE(cs35l41_supplies); i++)
		cs35l41->supplies[i].supply = cs35l41_supplies[i];

	cs35l41->num_supplies = ARRAY_SIZE(cs35l41_supplies);

	ret = devm_regulator_bulk_get(cs35l41->dev, cs35l41->num_supplies,
					cs35l41->supplies);
	if (ret != 0) {
		dev_err(cs35l41->dev,
			"Failed to request core supplies: %d\n",
			ret);
		return ret;
	}

	if (pdata) {
		cs35l41->pdata = *pdata;
	} else if (cs35l41->dev->of_node) {
		ret = cs35l41_handle_of_data(cs35l41->dev, &cs35l41->pdata);
		if (ret != 0)
			return ret;
	} else {
		ret = -EINVAL;
		goto err;
	}

	ret = regulator_bulk_enable(cs35l41->num_supplies, cs35l41->supplies);
	if (ret != 0) {
		dev_err(cs35l41->dev,
			"Failed to enable core supplies: %d\n", ret);
		return ret;
	}
	cs35l41->reset_gpio = of_get_named_gpio(cs35l41->dev->of_node, "reset-gpio", 0);
	if (!gpio_is_valid(cs35l41->reset_gpio)) {
			dev_err(cs35l41->dev, "%s: failed to get reset gpio\n", __func__);
			ret = -EINVAL;
			goto err;
	} else {
		ret = devm_gpio_request_one(cs35l41->dev,
						cs35l41->reset_gpio,
						GPIOF_OUT_INIT_LOW,
						 "cs35l41 reset");
		if (ret) {
				dev_err(cs35l41->dev,
						"%s: failed to request reset gpio\n",
						__func__);
				goto err;
		}
		dev_info(cs35l41->dev, "%s: get reset gpio is %d\n",
					__func__, cs35l41->reset_gpio);
		gpio_direction_output(cs35l41->reset_gpio, 0);
		usleep_range(2000, 2100);
		gpio_direction_output(cs35l41->reset_gpio, 1);
	}

	ret = cs35axx_init(cs35l41);
	if (ret < 0) {
		dev_err(cs35l41->dev, "Failed to initialize CS35AXX amplifier: %d\n", ret);
		goto err;
	}

	ret = request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					CS35L41_FW_NAME,
					cs35l41->dev,
					GFP_KERNEL, cs35l41,
					cs35l41_firmware_load);
	if (ret < 0)
		dev_err(cs35l41->dev, "Preload HALO firmware failed\n");

#ifdef ESD_TEST
	regcache_cache_bypass(cs35l41->regmap, true);
	regmap_update_bits(cs35l41->regmap, CS35L41_ESD_STAGE,
					   CS35L41_ESD_STAGE_FINAL_MASK,
					   CS35L41_ESD_STAGE_FINAL);
	regcache_cache_bypass(cs35l41->regmap, false);
#endif

/* CONFIG_PM */
#ifdef CONFIG_PM
	cs35l41->pm_nb.notifier_call = pm_event_handler;
	cs35l41->pm_nb.priority = 0;
	ret = register_pm_notifier(&cs35l41->pm_nb);
	if (ret) {
		dev_err(cs35l41->dev, "Failed to register PM notifier.\n");
	}
#endif /* CONFIG_PM */

	vendor_info.vendor = SMARTPA_VENDOR;
	vendor_info.chip_model = SMARTPA_MODEL;
	ret = smartpakit_set_info(&vendor_info);
	if (ret) {
			pr_err("%s: set info err %d", __func__, ret);
			goto err;
	}
	cs35l41->misc_dev.minor = MISC_DYNAMIC_MINOR;
	cs35l41->misc_dev.name = "cs35axx";
	cs35l41->misc_dev.fops = &cs35l41_fops;

	return misc_register(&cs35l41->misc_dev);
err:
	regulator_bulk_disable(cs35l41->num_supplies, cs35l41->supplies);
	return ret;
}

static int cs35l41_i2c_remove(struct i2c_client *client)
{
	struct cs35l41_private *cs35l41 = i2c_get_clientdata(client);

	gpio_direction_output(cs35l41->reset_gpio, 0);
	regmap_write(cs35l41->regmap, CS35L41_IRQ1_MASK1, 0xFFFFFFFF);
	regulator_bulk_disable(cs35l41->num_supplies, cs35l41->supplies);
	misc_deregister(&cs35l41->misc_dev);
	mutex_destroy(&cs35l41->lock);

	return 0;
}

static const struct i2c_device_id cs35l41_id_i2c[] = {
	{"cs35l40", 0},
	{"cs35l41", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs35l41_id_i2c);

static const struct of_device_id cs35l41_of_match[] = {
	{.compatible = "cirrus,cs35l40"},
	{.compatible = "cirrus,cs35axx"},
	{},
};
MODULE_DEVICE_TABLE(of, cs35l41_of_match);

static struct i2c_driver cs35axx_i2c_driver = {
	.driver = {
		.name		= "cs35axx",
		.of_match_table = cs35l41_of_match,
	},
	.id_table	= cs35l41_id_i2c,
	.probe		= cs35axx_i2c_probe,
	.remove		= cs35l41_i2c_remove,
};
static int __init cs35axx_i2c_init(void)
{
	int ret = -1;
	pr_info("%s: cs35axx driver init", __func__);
	ret = i2c_add_driver(&cs35axx_i2c_driver);
	if (ret)
			pr_err("%s: cs35axx driver add failed", __func__);
			
	return ret;	
}

//late init call due to smartpakit requirement
late_initcall_sync(cs35axx_i2c_init);
static void __exit cs35axx_i2c_exit(void)
{
	i2c_del_driver(&cs35axx_i2c_driver);
}

module_exit(cs35axx_i2c_exit);
MODULE_DESCRIPTION("Misc I2C CS35L41 driver");
MODULE_AUTHOR("James Schulman, Cirrus Logic Inc, <james.schulman@cirrus.com>");
MODULE_LICENSE("GPL");
