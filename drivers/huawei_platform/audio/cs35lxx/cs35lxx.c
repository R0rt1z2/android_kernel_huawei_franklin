/*
 * cs35lxx.c  --  CS35LXX Misc driver
 *
 * Copyright 2020 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif


#include "cs35lxx_platform.h"
#include "cs35lxx.h"
/* MTK platform header */
#include <mtk-sp-spk-amp.h>

#include "smartpakit.h"
#define VENDOR_ID_CIRRUS 3

/*
 * Some fields take zero as a valid value so use a high bit flag that won't
 * get written to the device to mark those.
 */
#define CS35LXX_VALID_PDATA 0x80000000

#define CS35LXX_IOCTL_STATUS_OK 0
#define CS35LXX_CSPL_HANDSHAKE_MAX_TRY_TIME 5

static const char * const cs35lxx_supplies[] = {
	"VA",
	"VP",
};

struct cs35lxx_cspl {
	struct workqueue_struct *calib_wq;
	struct work_struct calib_work;
	atomic_t calib_monitor;

	struct workqueue_struct *diag_wq;
	struct work_struct diag_work;
	atomic_t diag_monitor;

	struct workqueue_struct *set_cal_struct_wq;
	struct work_struct set_cal_struct_work;
	atomic_t set_cal_struct_monitor;

	struct workqueue_struct *bypass_wq;
	struct work_struct bypass_work;
	atomic_t bypass_monitor;

	int cspl_cmd_type;
	struct cs35lxx_calib_cmd calib_param;
	struct cs35lxx_r0_cmd r0_param;
	struct cs35lxx_diagnostics_cmd diag_param;
	struct cs35lxx_set_scene_cmd   scene_param;
	uint32_t *dsp_recv_buffer;
	uint32_t ambient_temperature;
	bool cspl_ready;
	int dsp_bypass;
};

struct cs35lxx_private {
	struct device *dev;
	struct cs35lxx_platform_data pdata;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	int num_supplies;
	int chip_version;
	int rev_id;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct mutex lock;
	struct miscdevice misc_dev;
	struct cs35lxx_cspl cspl;
};

static struct reg_default cs35lxx_reg[] = {
	{CS35LXX_TESTKEY_CTRL,			0x00000000},
	{CS35LXX_USERKEY_CTL,			0x00000000},
	{CS35LXX_OTP_CTRL1,			0x00002460},
	{CS35LXX_OTP_CTRL2,			0x00000000},
	{CS35LXX_OTP_CTRL3,			0x00000000},
	{CS35LXX_OTP_CTRL4,			0x00000000},
	{CS35LXX_OTP_CTRL5,			0x00000000},
	{CS35LXX_PAC_CTL1,			0x00000004},
	{CS35LXX_PAC_CTL2,			0x00000000},
	{CS35LXX_PAC_CTL3,			0x00000000},
	{CS35LXX_PWR_CTRL1,			0x00000000},
	{CS35LXX_PWR_CTRL2,			0x00003321},
	{CS35LXX_PWR_CTRL3,			0x01000010},
	{CS35LXX_CTRL_OVRRIDE,			0x00000002},
	{CS35LXX_AMP_OUT_MUTE,			0x00000000},
	{CS35LXX_OTP_TRIM_STATUS,		0x00000000},
	{CS35LXX_DISCH_FILT,			0x00000000},
	{CS35LXX_PROTECT_REL_ERR,		0x00000000},
	{CS35LXX_PAD_INTERFACE,			0x00000038},
	{CS35LXX_PLL_CLK_CTRL,			0x00000010},
	{CS35LXX_GLOBAL_CLK_CTRL,		0x00000003},
	{CS35LXX_ADC_CLK_CTRL,			0x00000000},
	{CS35LXX_SWIRE_CLK_CTRL,		0x00000000},
	{CS35LXX_SP_SCLK_CLK_CTRL,		0x00000000},
	{CS35LXX_MDSYNC_EN,			0x00000000},
	{CS35LXX_MDSYNC_TX_ID,			0x00000000},
	{CS35LXX_MDSYNC_PWR_CTRL,		0x00000000},
	{CS35LXX_MDSYNC_DATA_TX,		0x00000000},
	{CS35LXX_MDSYNC_TX_STATUS,		0x00000002},
	{CS35LXX_MDSYNC_RX_STATUS,		0x00000000},
	{CS35LXX_MDSYNC_ERR_STATUS,		0x00000000},
	{CS35LXX_BSTCVRT_VCTRL1,		0x00000000},
	{CS35LXX_BSTCVRT_VCTRL2,		0x00000001},
	{CS35LXX_BSTCVRT_PEAK_CUR,		0x0000004A},
	{CS35LXX_BSTCVRT_SFT_RAMP,		0x00000003},
	{CS35LXX_BSTCVRT_COEFF,			0x00002424},
	{CS35LXX_BSTCVRT_SLOPE_LBST,		0x00005800},
	{CS35LXX_BSTCVRT_SW_FREQ,		0x00010000},
	{CS35LXX_BSTCVRT_DCM_CTRL,		0x00002001},
	{CS35LXX_BSTCVRT_DCM_MODE_FORCE,	0x00000000},
	{CS35LXX_BSTCVRT_OVERVOLT_CTRL,		0x00000130},
	{CS35LXX_VPI_LIMIT_MODE,		0x00000000},
	{CS35LXX_VPI_LIMIT_MINMAX,		0x00003000},
	{CS35LXX_VPI_VP_THLD,			0x00101010},
	{CS35LXX_VPI_TRACK_CTRL,		0x00000000},
	{CS35LXX_VPI_TRIG_MODE_CTRL,		0x00000000},
	{CS35LXX_VPI_TRIG_STEPS,		0x00000000},
	{CS35LXX_VI_SPKMON_FILT,		0x00000003},
	{CS35LXX_VI_SPKMON_GAIN,		0x00000909},
	{CS35LXX_VI_SPKMON_IP_SEL,		0x00000000},
	{CS35LXX_DTEMP_WARN_THLD,		0x00000002},
	{CS35LXX_DTEMP_STATUS,			0x00000000},
	{CS35LXX_VPVBST_FS_SEL,			0x00000001},
	{CS35LXX_VPVBST_VP_CTRL,		0x000001C0},
	{CS35LXX_VPVBST_VBST_CTRL,		0x000001C0},
	{CS35LXX_ASP_TX_PIN_CTRL,		0x00000028},
	{CS35LXX_ASP_RATE_CTRL,			0x00090000},
	{CS35LXX_ASP_FORMAT,			0x00000002},
	{CS35LXX_ASP_FRAME_CTRL,		0x00180018},
	{CS35LXX_ASP_TX1_TX2_SLOT,		0x00010000},
	{CS35LXX_ASP_TX3_TX4_SLOT,		0x00030002},
	{CS35LXX_ASP_TX5_TX6_SLOT,		0x00050004},
	{CS35LXX_ASP_TX7_TX8_SLOT,		0x00070006},
	{CS35LXX_ASP_RX1_SLOT,			0x00000000},
	{CS35LXX_ASP_RX_TX_EN,			0x00000000},
	{CS35LXX_ASP_RX1_SEL,			0x00000008},
	{CS35LXX_ASP_TX1_SEL,			0x00000018},
	{CS35LXX_ASP_TX2_SEL,			0x00000019},
	{CS35LXX_ASP_TX3_SEL,			0x00000028},
	{CS35LXX_ASP_TX4_SEL,			0x00000029},
	{CS35LXX_ASP_TX5_SEL,			0x00000020},
	{CS35LXX_ASP_TX6_SEL,			0x00000000},
	{CS35LXX_SWIRE_P1_TX1_SEL,		0x00000018},
	{CS35LXX_SWIRE_P1_TX2_SEL,		0x00000019},
	{CS35LXX_SWIRE_P2_TX1_SEL,		0x00000028},
	{CS35LXX_SWIRE_P2_TX2_SEL,		0x00000029},
	{CS35LXX_SWIRE_P2_TX3_SEL,		0x00000020},
	{CS35LXX_SWIRE_DP1_FIFO_CFG,		0x0000001B},
	{CS35LXX_SWIRE_DP2_FIFO_CFG,		0x0000001B},
	{CS35LXX_SWIRE_DP3_FIFO_CFG,		0x0000001B},
	{CS35LXX_SWIRE_PCM_RX_DATA,		0x00000000},
	{CS35LXX_SWIRE_FS_SEL,			0x00000001},
	{CS35LXX_AMP_DIG_VOL_CTRL,		0x00008000},
	{CS35LXX_VPBR_CFG,			0x02AA1905},
	{CS35LXX_VBBR_CFG,			0x02AA1905},
	{CS35LXX_VPBR_STATUS,			0x00000000},
	{CS35LXX_VBBR_STATUS,			0x00000000},
	{CS35LXX_OVERTEMP_CFG,			0x00000001},
	{CS35LXX_AMP_ERR_VOL,			0x00000000},
	{CS35LXX_CLASSH_CFG,			0x000B0405},
	{CS35LXX_CLASSH_FET_DRV_CFG,		0x00000111},
	{CS35LXX_NG_CFG,			0x00000033},
	{CS35LXX_AMP_GAIN_CTRL,			0x00000273},
	{CS35LXX_PWM_MOD_IO_CTRL,		0x00000000},
	{CS35LXX_PWM_MOD_STATUS,		0x00000000},
	{CS35LXX_DAC_MSM_CFG,			0x00000000},
	{CS35LXX_AMP_SLOPE_CTRL,		0x00000B00},
	{CS35LXX_AMP_PDM_VOLUME,		0x00000000},
	{CS35LXX_AMP_PDM_RATE_CTRL,		0x00000000},
	{CS35LXX_PDM_CH_SEL,			0x00000000},
	{CS35LXX_AMP_NG_CTRL,			0x0000212F},
	{CS35LXX_PDM_HIGHFILT_CTRL,		0x00000000},
	{CS35LXX_PAC_INT0_CTRL,			0x00000001},
	{CS35LXX_PAC_INT1_CTRL,			0x00000001},
	{CS35LXX_PAC_INT2_CTRL,			0x00000001},
	{CS35LXX_PAC_INT3_CTRL,			0x00000001},
	{CS35LXX_PAC_INT4_CTRL,			0x00000001},
	{CS35LXX_PAC_INT5_CTRL,			0x00000001},
	{CS35LXX_PAC_INT6_CTRL,			0x00000001},
	{CS35LXX_PAC_INT7_CTRL,			0x00000001},
};

static bool cs35lxx_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case CS35LXX_SW_RESET:
		case CS35LXX_SW_REV:
		case CS35LXX_HW_REV:
		case CS35LXX_TESTKEY_CTRL:
		case CS35LXX_USERKEY_CTL:
		case CS35LXX_OTP_MEM30:
		case CS35LXX_OTP_CTRL1:
		case CS35LXX_OTP_CTRL2:
		case CS35LXX_OTP_CTRL3:
		case CS35LXX_OTP_CTRL4:
		case CS35LXX_OTP_CTRL5:
		case CS35LXX_PAC_CTL1:
		case CS35LXX_PAC_CTL2:
		case CS35LXX_PAC_CTL3:
		case CS35LXX_DEVICE_ID:
		case CS35LXX_FAB_ID:
		case CS35LXX_REV_ID:
		case CS35LXX_PWR_CTRL1:
		case CS35LXX_PWR_CTRL2:
		case CS35LXX_PWR_CTRL3:
		case CS35LXX_CTRL_OVRRIDE:
		case CS35LXX_AMP_OUT_MUTE:
		case CS35LXX_OTP_TRIM_STATUS:
		case CS35LXX_DISCH_FILT:
		case CS35LXX_PROTECT_REL_ERR:
		case CS35LXX_PAD_INTERFACE:
		case CS35LXX_PLL_CLK_CTRL:
		case CS35LXX_GLOBAL_CLK_CTRL:
		case CS35LXX_ADC_CLK_CTRL:
		case CS35LXX_SWIRE_CLK_CTRL:
		case CS35LXX_SP_SCLK_CLK_CTRL:
		case CS35LXX_TST_FS_MON0:
		case CS35LXX_MDSYNC_EN:
		case CS35LXX_MDSYNC_TX_ID:
		case CS35LXX_MDSYNC_PWR_CTRL:
		case CS35LXX_MDSYNC_DATA_TX:
		case CS35LXX_MDSYNC_TX_STATUS:
		case CS35LXX_MDSYNC_RX_STATUS:
		case CS35LXX_MDSYNC_ERR_STATUS:
		case CS35LXX_BSTCVRT_VCTRL1:
		case CS35LXX_BSTCVRT_VCTRL2:
		case CS35LXX_BSTCVRT_PEAK_CUR:
		case CS35LXX_BSTCVRT_SFT_RAMP:
		case CS35LXX_BSTCVRT_COEFF:
		case CS35LXX_BSTCVRT_SLOPE_LBST:
		case CS35LXX_BSTCVRT_SW_FREQ:
		case CS35LXX_BSTCVRT_DCM_CTRL:
		case CS35LXX_BSTCVRT_DCM_MODE_FORCE:
		case CS35LXX_BSTCVRT_OVERVOLT_CTRL:
		case CS35LXX_BST_TST_MANUAL:
		case CS35LXX_BST_ANA2_TEST:
		case CS35LXX_VPI_LIMIT_MODE:
		case CS35LXX_VPI_LIMIT_MINMAX:
		case CS35LXX_VPI_VP_THLD:
		case CS35LXX_VPI_TRACK_CTRL:
		case CS35LXX_VPI_TRIG_MODE_CTRL:
		case CS35LXX_VPI_TRIG_STEPS:
		case CS35LXX_VI_SPKMON_FILT:
		case CS35LXX_VI_SPKMON_GAIN:
		case CS35LXX_VI_SPKMON_IP_SEL:
		case CS35LXX_DTEMP_WARN_THLD:
		case CS35LXX_DTEMP_STATUS:
		case CS35LXX_VPVBST_FS_SEL:
		case CS35LXX_VPVBST_VP_CTRL:
		case CS35LXX_VPVBST_VBST_CTRL:
		case CS35LXX_ASP_TX_PIN_CTRL:
		case CS35LXX_ASP_RATE_CTRL:
		case CS35LXX_ASP_FORMAT:
		case CS35LXX_ASP_FRAME_CTRL:
		case CS35LXX_ASP_TX1_TX2_SLOT:
		case CS35LXX_ASP_TX3_TX4_SLOT:
		case CS35LXX_ASP_TX5_TX6_SLOT:
		case CS35LXX_ASP_TX7_TX8_SLOT:
		case CS35LXX_ASP_RX1_SLOT:
		case CS35LXX_ASP_RX_TX_EN:
		case CS35LXX_ASP_RX1_SEL:
		case CS35LXX_ASP_TX1_SEL:
		case CS35LXX_ASP_TX2_SEL:
		case CS35LXX_ASP_TX3_SEL:
		case CS35LXX_ASP_TX4_SEL:
		case CS35LXX_ASP_TX5_SEL:
		case CS35LXX_ASP_TX6_SEL:
		case CS35LXX_SWIRE_P1_TX1_SEL:
		case CS35LXX_SWIRE_P1_TX2_SEL:
		case CS35LXX_SWIRE_P2_TX1_SEL:
		case CS35LXX_SWIRE_P2_TX2_SEL:
		case CS35LXX_SWIRE_P2_TX3_SEL:
		case CS35LXX_SWIRE_DP1_FIFO_CFG:
		case CS35LXX_SWIRE_DP2_FIFO_CFG:
		case CS35LXX_SWIRE_DP3_FIFO_CFG:
		case CS35LXX_SWIRE_PCM_RX_DATA:
		case CS35LXX_SWIRE_FS_SEL:
		case CS35LXX_AMP_DIG_VOL_CTRL:
		case CS35LXX_VPBR_CFG:
		case CS35LXX_VBBR_CFG:
		case CS35LXX_VPBR_STATUS:
		case CS35LXX_VBBR_STATUS:
		case CS35LXX_OVERTEMP_CFG:
		case CS35LXX_AMP_ERR_VOL:
		case CS35LXX_CLASSH_CFG:
		case CS35LXX_CLASSH_FET_DRV_CFG:
		case CS35LXX_NG_CFG:
		case CS35LXX_AMP_GAIN_CTRL:
		case CS35LXX_PWM_MOD_IO_CTRL:
		case CS35LXX_PWM_MOD_STATUS:
		case CS35LXX_DAC_MSM_CFG:
		case CS35LXX_AMP_SLOPE_CTRL:
		case CS35LXX_AMP_PDM_VOLUME:
		case CS35LXX_AMP_PDM_RATE_CTRL:
		case CS35LXX_PDM_CH_SEL:
		case CS35LXX_AMP_NG_CTRL:
		case CS35LXX_PDM_HIGHFILT_CTRL:
		case CS35LXX_INT1_STATUS:
		case CS35LXX_INT2_STATUS:
		case CS35LXX_INT3_STATUS:
		case CS35LXX_INT4_STATUS:
		case CS35LXX_INT1_RAW_STATUS:
		case CS35LXX_INT2_RAW_STATUS:
		case CS35LXX_INT3_RAW_STATUS:
		case CS35LXX_INT4_RAW_STATUS:
		case CS35LXX_INT1_MASK:
		case CS35LXX_INT2_MASK:
		case CS35LXX_INT3_MASK:
		case CS35LXX_INT4_MASK:
		case CS35LXX_INT1_EDGE_LVL_CTRL:
		case CS35LXX_INT3_EDGE_LVL_CTRL:
		case CS35LXX_PAC_INT_STATUS:
		case CS35LXX_PAC_INT_RAW_STATUS:
		case CS35LXX_PAC_INT_FLUSH_CTRL:
		case CS35LXX_PAC_INT0_CTRL:
		case CS35LXX_PAC_INT1_CTRL:
		case CS35LXX_PAC_INT2_CTRL:
		case CS35LXX_PAC_INT3_CTRL:
		case CS35LXX_PAC_INT4_CTRL:
		case CS35LXX_PAC_INT5_CTRL:
		case CS35LXX_PAC_INT6_CTRL:
		case CS35LXX_PAC_INT7_CTRL:
			return true;
		default:
			if (reg >= CS35LXX_PAC_PMEM_WORD0 &&
				reg <= CS35LXX_PAC_PMEM_WORD1023)
				return true;
			else
				return false;
	}
}

static bool cs35lxx_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case CS35LXX_TESTKEY_CTRL:
		case CS35LXX_USERKEY_CTL:
		case CS35LXX_TST_FS_MON0:
			return true;
		default:
			return false;
	}
}

static bool cs35lxx_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case CS35LXX_SW_RESET:
		case CS35LXX_SW_REV:
		case CS35LXX_HW_REV:
		case CS35LXX_TESTKEY_CTRL:
		case CS35LXX_USERKEY_CTL:
		case CS35LXX_DEVICE_ID:
		case CS35LXX_FAB_ID:
		case CS35LXX_REV_ID:
		case CS35LXX_INT1_STATUS:
		case CS35LXX_INT2_STATUS:
		case CS35LXX_INT3_STATUS:
		case CS35LXX_INT4_STATUS:
		case CS35LXX_INT1_RAW_STATUS:
		case CS35LXX_INT2_RAW_STATUS:
		case CS35LXX_INT3_RAW_STATUS:
		case CS35LXX_INT4_RAW_STATUS:
		case CS35LXX_INT1_MASK:
		case CS35LXX_INT2_MASK:
		case CS35LXX_INT3_MASK:
		case CS35LXX_INT4_MASK:
		case CS35LXX_INT1_EDGE_LVL_CTRL:
		case CS35LXX_INT3_EDGE_LVL_CTRL:
		case CS35LXX_PAC_INT_STATUS:
		case CS35LXX_PAC_INT_RAW_STATUS:
		case CS35LXX_PAC_INT_FLUSH_CTRL:
			return true;
		default:
			if (reg >= CS35LXX_PAC_PMEM_WORD0 &&
				reg <= CS35LXX_PAC_PMEM_WORD1023)
				return true;
			else
				return false;
	}
}

static const struct reg_sequence cs35lxx_spk_power_on_patch[] = {
	{CS35LXX_AMP_GAIN_CTRL, 0x00000233},
	{CS35LXX_ASP_TX1_TX2_SLOT, 0x00000002},
	{CS35LXX_ASP_RX1_SLOT, 0x00000000},
	{CS35LXX_ASP_RX_TX_EN, 0x00010003},
	{CS35LXX_PWR_CTRL2, 0x00003721},
};


static const struct reg_sequence cs35lxx_spk_errata_revb1_patch[] = {
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_UNLOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_UNLOCK2 },
	{ 0x0000394c,		0x028764b7 },
	{ 0x00002d10,		0x0002c01c },
	{ 0x00007418,		0x909001c8 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_LOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_LOCK2 },
};


static int cs35lxx_spk_power_on(struct cs35lxx_private *cs35lxx)
{
	//apply errata writes
	if(cs35lxx->rev_id == CS35LXX_REV_B1)
		regmap_multi_reg_write(cs35lxx->regmap, cs35lxx_spk_errata_revb1_patch,
					 ARRAY_SIZE(cs35lxx_spk_errata_revb1_patch));

	regmap_multi_reg_write(cs35lxx->regmap, cs35lxx_spk_power_on_patch,
			     ARRAY_SIZE(cs35lxx_spk_power_on_patch));

	regmap_update_bits(cs35lxx->regmap, CS35LXX_PWR_CTRL1,
			  CS35LXX_GLOBAL_EN_MASK,
			  1 << CS35LXX_GLOBAL_EN_SHIFT);

	usleep_range(1000, 1100);

	return 0;
}

static int cs35lxx_spk_power_off(struct cs35lxx_private *cs35lxx)
{
	regmap_update_bits(cs35lxx->regmap, CS35LXX_PWR_CTRL1,
			  CS35LXX_GLOBAL_EN_MASK,
			  0 << CS35LXX_GLOBAL_EN_SHIFT);

	usleep_range(1000, 1100);

	regmap_update_bits(cs35lxx->regmap, CS35LXX_PWR_CTRL2, 0x01, 0);

	return 0;
}

static int cs35lxx_boost_inductor(struct cs35lxx_private *cs35lxx, int inductor)
{
	regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_COEFF,
			  CS35LXX_BSTCVRT_K1_MASK, 0x3C);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_COEFF,
			  CS35LXX_BSTCVRT_K2_MASK,
			  0x3C << CS35LXX_BSTCVRT_K2_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_SW_FREQ,
			  CS35LXX_BSTCVRT_CCMFREQ_MASK, 0x00);

	switch (inductor) {
		case 1000: /* 1 uH */
			regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_SLOPE_LBST,
					  CS35LXX_BSTCVRT_SLOPE_MASK,
					  0x75 << CS35LXX_BSTCVRT_SLOPE_SHIFT);
			regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_SLOPE_LBST,
					  CS35LXX_BSTCVRT_LBSTVAL_MASK, 0x00);
			break;
		case 1200: /* 1.2 uH */
			regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_SLOPE_LBST,
					  CS35LXX_BSTCVRT_SLOPE_MASK,
					  0x6B << CS35LXX_BSTCVRT_SLOPE_SHIFT);
			regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_SLOPE_LBST,
					  CS35LXX_BSTCVRT_LBSTVAL_MASK, 0x01);
			break;
		default:
			dev_err(cs35lxx->dev, "%s Invalid Inductor Value %d uH\n",
				__func__, inductor);
			return -EINVAL;
	}

	return 0;
}

static int cs35lxx_probe(struct cs35lxx_private *cs35lxx)
{
	int ret;

	if ((cs35lxx->rev_id == CS35LXX_REV_A0) && cs35lxx->pdata.dcm_mode) {
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_DCM_CTRL,
				  CS35LXX_DCM_AUTO_MASK,
				  CS35LXX_DCM_AUTO_MASK);

		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_UNLOCK1);
		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_UNLOCK2);

		regmap_update_bits(cs35lxx->regmap, CS35LXX_BST_TST_MANUAL,
				  CS35LXX_BST_MAN_IPKCOMP_MASK,
				  0 << CS35LXX_BST_MAN_IPKCOMP_SHIFT);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BST_TST_MANUAL,
				  CS35LXX_BST_MAN_IPKCOMP_EN_MASK,
				  CS35LXX_BST_MAN_IPKCOMP_EN_MASK);

		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
				CS35LXX_TEST_LOCK1);
		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
				CS35LXX_TEST_LOCK2);
	}

	if (cs35lxx->pdata.amp_pcm_inv)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_AMP_DIG_VOL_CTRL,
				  CS35LXX_AMP_PCM_INV_MASK,
				  CS35LXX_AMP_PCM_INV_MASK);

	if (cs35lxx->pdata.multi_amp_mode)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_ASP_TX_PIN_CTRL,
				  CS35LXX_ASP_TX_HIZ_MASK,
				  CS35LXX_ASP_TX_HIZ_MASK);

	if (cs35lxx->pdata.imon_pol_inv)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_VI_SPKMON_FILT,
				  CS35LXX_IMON_POL_MASK, 0);

	if (cs35lxx->pdata.vmon_pol_inv)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_VI_SPKMON_FILT,
				  CS35LXX_VMON_POL_MASK, 0);

	if (cs35lxx->pdata.bst_vctl)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_VCTRL1,
				  CS35LXX_BSTCVRT_CTL_MASK,
				  cs35lxx->pdata.bst_vctl);

	if (cs35lxx->pdata.bst_vctl_sel)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_VCTRL2,
				  CS35LXX_BSTCVRT_CTL_SEL_MASK,
				  cs35lxx->pdata.bst_vctl_sel);

	if (cs35lxx->pdata.bst_ipk)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_PEAK_CUR,
				  CS35LXX_BST_IPK_MASK,
				  cs35lxx->pdata.bst_ipk);

	if (cs35lxx->pdata.boost_ind) {
		ret = cs35lxx_boost_inductor(cs35lxx, cs35lxx->pdata.boost_ind);
		if (ret < 0) {
			dev_err(cs35lxx->dev,
				"Boost inductor config failed(%d)\n", ret);
			return ret;
		}
	}

	if (cs35lxx->pdata.temp_warn_thld)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_DTEMP_WARN_THLD,
				  CS35LXX_TEMP_THLD_MASK,
				  cs35lxx->pdata.temp_warn_thld);

	if (cs35lxx->pdata.irq_drv_sel)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PAD_INTERFACE,
				  CS35LXX_INT_DRV_SEL_MASK,
				  cs35lxx->pdata.irq_drv_sel <<
				  CS35LXX_INT_DRV_SEL_SHIFT);

	//if (cs35lxx->pdata.irq_gpio_sel)
	//	regmap_update_bits(cs35lxx->regmap, CS35LXX_PAD_INTERFACE,
	//			  CS35LXX_INT_GPIO_SEL_MASK,
	//			  cs35lxx->pdata.irq_gpio_sel <<
	//			  CS35LXX_INT_GPIO_SEL_SHIFT);

	/*
	 * Rev B0 has 2 versions
	 * L36 is 10V
	 * L37 is 12V
	 * If L36 we need to clamp some values for safety
	 * after probe has setup dt values. We want to make
	 * sure we dont miss any values set in probe
	 */
	if (cs35lxx->chip_version == CS35LXX_10V_L36) {
		regmap_update_bits(cs35lxx->regmap,
				  CS35LXX_BSTCVRT_OVERVOLT_CTRL,
				  CS35LXX_BST_OVP_THLD_MASK,
				  CS35LXX_BST_OVP_THLD_11V);

		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_UNLOCK1);
		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_UNLOCK2);

		regmap_update_bits(cs35lxx->regmap, CS35LXX_BST_ANA2_TEST,
				  CS35LXX_BST_OVP_TRIM_MASK,
				  CS35LXX_BST_OVP_TRIM_11V <<
				  CS35LXX_BST_OVP_TRIM_SHIFT);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_VCTRL2,
				  CS35LXX_BST_CTRL_LIM_MASK,
				  1 << CS35LXX_BST_CTRL_LIM_SHIFT);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_BSTCVRT_VCTRL1,
				  CS35LXX_BSTCVRT_CTL_MASK,
				  CS35LXX_BST_CTRL_10V_CLAMP);
		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_LOCK1);
		regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
			    CS35LXX_TEST_LOCK2);
	}

	/*
	 * RevA and B require the disabling of
	 * SYNC_GLOBAL_OVR when GLOBAL_EN = 0.
	 * Just turn it off from default
	 */
	regmap_update_bits(cs35lxx->regmap, CS35LXX_CTRL_OVRRIDE,
			  CS35LXX_SYNC_GLOBAL_OVR_MASK,
			  0 << CS35LXX_SYNC_GLOBAL_OVR_SHIFT);

	// Apply ASP Config
	if (cs35lxx->pdata.asp_config.asp_fmt)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_ASP_FORMAT,
				  CS35LXX_ASP_FMT_MASK,
				  cs35lxx->pdata.asp_config.asp_fmt << CS35LXX_ASP_FMT_SHIFT);

	if (cs35lxx->pdata.asp_config.asp_rx_width)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_ASP_FRAME_CTRL,
				  CS35LXX_ASP_RX_WIDTH_MASK,
				  cs35lxx->pdata.asp_config.asp_rx_width <<
				  CS35LXX_ASP_RX_WIDTH_SHIFT);

	if (cs35lxx->pdata.asp_config.asp_tx_width)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_ASP_FRAME_CTRL,
				  CS35LXX_ASP_TX_WIDTH_MASK,
				  cs35lxx->pdata.asp_config.asp_tx_width <<
				  CS35LXX_ASP_TX_WIDTH_SHIFT);

	if (cs35lxx->pdata.asp_config.asp_sample_rate)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_GLOBAL_CLK_CTRL,
				  CS35LXX_GLOBAL_FS_MASK,
				  cs35lxx->pdata.asp_config.asp_sample_rate <<
				  CS35LXX_GLOBAL_FS_SHIFT);

	if (cs35lxx->pdata.asp_config.asp_sclk_rate)
		regmap_update_bits(cs35lxx->regmap, CS35LXX_ASP_TX_PIN_CTRL,
				  CS35LXX_SCLK_FREQ_MASK,
				  cs35lxx->pdata.asp_config.asp_sclk_rate);

	// Apply PLL Config
	if ((cs35lxx->pdata.pll_refclk_freq & CS35LXX_VALID_PDATA) |
	   (cs35lxx->pdata.pll_refclk_sel & CS35LXX_VALID_PDATA)) {
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_PLL_OPENLOOP_MASK,
				  1 << CS35LXX_PLL_OPENLOOP_SHIFT);

		if (cs35lxx->pdata.pll_refclk_freq & CS35LXX_VALID_PDATA)
			regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_REFCLK_FREQ_MASK,
				  cs35lxx->pdata.pll_refclk_freq << CS35LXX_REFCLK_FREQ_SHIFT);

		regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_PLL_REFCLK_EN_MASK,
				  0 << CS35LXX_PLL_REFCLK_EN_SHIFT);
		if (cs35lxx->pdata.pll_refclk_freq & CS35LXX_VALID_PDATA)
			regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_PLL_CLK_SEL_MASK,
				  cs35lxx->pdata.pll_refclk_sel);

		regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_PLL_OPENLOOP_MASK,
				  0 << CS35LXX_PLL_OPENLOOP_SHIFT);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PLL_CLK_CTRL,
				  CS35LXX_PLL_REFCLK_EN_MASK,
				  1 << CS35LXX_PLL_REFCLK_EN_SHIFT);
	}
	return 0;
}

static struct regmap_config cs35lxx_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = CS35LXX_PAC_PMEM_WORD1023,
	.reg_defaults = cs35lxx_reg,
	.num_reg_defaults = ARRAY_SIZE(cs35lxx_reg),
	.precious_reg = cs35lxx_precious_reg,
	.volatile_reg = cs35lxx_volatile_reg,
	.readable_reg = cs35lxx_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static irqreturn_t cs35lxx_irq(int irq, void *data)
{
	struct cs35lxx_private *cs35lxx = data;
	unsigned int status[4];
	unsigned int masks[4];
	int ret = IRQ_NONE;

	/* ack the irq by reading all status registers */
	regmap_bulk_read(cs35lxx->regmap, CS35LXX_INT1_STATUS, status,
			 ARRAY_SIZE(status));

	regmap_bulk_read(cs35lxx->regmap, CS35LXX_INT1_MASK, masks,
			 ARRAY_SIZE(masks));

	/* Check to see if unmasked bits are active */
	if (!(status[0] & ~masks[0]) && !(status[1] & ~masks[1]) &&
		!(status[2] & ~masks[2]) && !(status[3] & ~masks[3])) {
		return IRQ_NONE;
	}

	/*
	 * The following interrupts require a
	 * protection release cycle to get the
	 * speaker out of Safe-Mode.
	 */
	if (status[2] & CS35LXX_AMP_SHORT_ERR) {
		dev_crit(cs35lxx->dev, "Amp short error\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_AMP_SHORT_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_AMP_SHORT_ERR_RLS,
				  CS35LXX_AMP_SHORT_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_AMP_SHORT_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT3_STATUS,
				  CS35LXX_AMP_SHORT_ERR,
				  CS35LXX_AMP_SHORT_ERR);
		ret = IRQ_HANDLED;
	}

	if (status[0] & CS35LXX_TEMP_WARN) {
		dev_crit(cs35lxx->dev, "Over temperature warning\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_WARN_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_WARN_ERR_RLS,
				  CS35LXX_TEMP_WARN_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_WARN_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT1_STATUS,
				  CS35LXX_TEMP_WARN, CS35LXX_TEMP_WARN);
		ret = IRQ_HANDLED;
	}

	if (status[0] & CS35LXX_TEMP_ERR) {
		dev_crit(cs35lxx->dev, "Over temperature error\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, CS35LXX_TEMP_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT1_STATUS,
				  CS35LXX_TEMP_ERR, CS35LXX_TEMP_ERR);
		ret = IRQ_HANDLED;
	}

	if (status[0] & CS35LXX_BST_OVP_ERR) {
		dev_crit(cs35lxx->dev, "VBST Over Voltage error\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, CS35LXX_TEMP_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_TEMP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT1_STATUS,
				  CS35LXX_BST_OVP_ERR, CS35LXX_BST_OVP_ERR);
		ret = IRQ_HANDLED;
	}

	if (status[0] & CS35LXX_BST_DCM_UVP_ERR) {
		dev_crit(cs35lxx->dev, "DCM VBST Under Voltage Error\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_UVP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_UVP_ERR_RLS,
				  CS35LXX_BST_UVP_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_UVP_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT1_STATUS,
				  CS35LXX_BST_DCM_UVP_ERR,
				  CS35LXX_BST_DCM_UVP_ERR);
		ret = IRQ_HANDLED;
	}

	if (status[0] & CS35LXX_BST_SHORT_ERR) {
		dev_crit(cs35lxx->dev, "LBST SHORT error!\n");
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_SHORT_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_SHORT_ERR_RLS,
				  CS35LXX_BST_SHORT_ERR_RLS);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_PROTECT_REL_ERR,
				  CS35LXX_BST_SHORT_ERR_RLS, 0);
		regmap_update_bits(cs35lxx->regmap, CS35LXX_INT1_STATUS,
				  CS35LXX_BST_SHORT_ERR,
				  CS35LXX_BST_SHORT_ERR);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int cs35lxx_handle_of_data(struct i2c_client *i2c_client,
				struct cs35lxx_platform_data *pdata)
{
	struct device_node *np = i2c_client->dev.of_node;
	struct cs35lxx_vpbr_cfg *vpbr_config = &pdata->vpbr_config;
	struct asp_cfg *asp_config = &pdata->asp_config;
	struct device_node *vpbr_node, *asp_node;
	unsigned int val;
	int ret;

	if (!np)
		return 0;

	ret = of_property_read_u32(np, "cirrus,boost-ctl-millivolt", &val);
	if (!ret) {
		if (val < 2550 || val > 12000) {
			dev_err(&i2c_client->dev,
				"Invalid Boost Voltage %d mV\n", val);
			return -EINVAL;
		}
		pdata->bst_vctl = (((val - 2550) / 100) + 1) << 1;
	} else {
		dev_err(&i2c_client->dev,
			"Unable to find required parameter 'cirrus,boost-ctl-millivolt'");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "cirrus,boost-ctl-select", &val);
	if (!ret)
		pdata->bst_vctl_sel = val | CS35LXX_VALID_PDATA;

	ret = of_property_read_u32(np, "cirrus,boost-peak-milliamp", &val);
	if (!ret) {
		if (val < 1600 || val > 4500) {
			dev_err(&i2c_client->dev,
				"Invalid Boost Peak Current %u mA\n", val);
			return -EINVAL;
		}

		pdata->bst_ipk = (val - 1600) / 50 + 0xF;
	} else {
		dev_err(&i2c_client->dev,
			"Unable to find required parameter 'cirrus,boost-peak-milliamp'");
		return -EINVAL;
	}

	pdata->multi_amp_mode = of_property_read_bool(np,
					"cirrus,multi-amp-mode");

	pdata->dcm_mode = of_property_read_bool(np,
					"cirrus,dcm-mode-enable");

	pdata->amp_pcm_inv = of_property_read_bool(np,
					"cirrus,amp-pcm-inv");

	pdata->imon_pol_inv = of_property_read_bool(np,
					"cirrus,imon-pol-inv");

	pdata->vmon_pol_inv = of_property_read_bool(np,
					"cirrus,vmon-pol-inv");

	if (of_property_read_u32(np, "cirrus,temp-warn-threshold", &val) >= 0)
		pdata->temp_warn_thld = val | CS35LXX_VALID_PDATA;

	if (of_property_read_u32(np, "cirrus,boost-ind-nanohenry", &val) >= 0) {
		pdata->boost_ind = val;
	} else {
		dev_err(&i2c_client->dev, "Inductor not specified.\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "cirrus,irq-drive-select", &val) >= 0)
		pdata->irq_drv_sel = val | CS35LXX_VALID_PDATA;

	if (of_property_read_u32(np, "cirrus,irq-gpio-select", &val) >= 0)
		pdata->irq_gpio_sel = val | CS35LXX_VALID_PDATA;

	/* VPBR Config */
	vpbr_node = of_get_child_by_name(np, "cirrus,vpbr-config");
	vpbr_config->is_present = vpbr_node ? true : false;
	if (vpbr_config->is_present) {
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-en",
					 &val) >= 0)
			vpbr_config->vpbr_en = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-thld",
					 &val) >= 0)
			vpbr_config->vpbr_thld = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-atk-rate",
					 &val) >= 0)
			vpbr_config->vpbr_atk_rate = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-atk-vol",
					 &val) >= 0)
			vpbr_config->vpbr_atk_vol = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-max-attn",
					 &val) >= 0)
			vpbr_config->vpbr_max_attn = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-wait",
					 &val) >= 0)
			vpbr_config->vpbr_wait = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-rel-rate",
					 &val) >= 0)
			vpbr_config->vpbr_rel_rate = val;
		if (of_property_read_u32(vpbr_node, "cirrus,vpbr-mute-en",
					 &val) >= 0)
			vpbr_config->vpbr_mute_en = val;
	}
	of_node_put(vpbr_node);

	// PLL Config
	ret = of_property_read_u32(np, "cirrus,pll-refclk-sel", &val);
	if (ret >= 0) {
		val |= CS35LXX_VALID_PDATA;
		pdata->pll_refclk_sel = val;
	}

	ret = of_property_read_u32(np, "cirrus,pll-refclk-freq", &val);
	if (ret >= 0) {
		val |= CS35LXX_VALID_PDATA;
		pdata->pll_refclk_freq = val;
	}

	// ASP Config
	asp_node = of_get_child_by_name(np, "cirrus,asp-config");
	if (asp_node) {
		ret = of_property_read_u32(asp_node, "cirrus,asp-rx-width", &val);
		if (ret >= 0) {
			val |= CS35LXX_VALID_PDATA;
			asp_config->asp_rx_width = val;
		}

		ret = of_property_read_u32(asp_node, "cirrus,asp-tx-width", &val);
		if (ret >= 0) {
			val |= CS35LXX_VALID_PDATA;
			asp_config->asp_tx_width = val;
		}

		ret = of_property_read_u32(asp_node, "cirrus,asp-fmt", &val);
		if (ret >= 0) {
			val |= CS35LXX_VALID_PDATA;
			asp_config->asp_fmt = val;
		}

		ret = of_property_read_u32(asp_node, "cirrus,asp-sample-rate", &val);
		if (ret >= 0) {
			val |= CS35LXX_VALID_PDATA;
			asp_config->asp_sample_rate = val;
		}

		ret = of_property_read_u32(asp_node, "cirrus,asp-sclk-rate", &val);
		if (ret >= 0) {
			val |= CS35LXX_VALID_PDATA;
			asp_config->asp_sclk_rate = val;
		}
	}
	of_node_put(asp_node);

	return 0;
}

static int cs35lxx_pac(struct cs35lxx_private *cs35lxx)
{
	int ret, count;
	unsigned int val;

	if (cs35lxx->rev_id == CS35LXX_REV_A0)
		return 0;

	/*
	 * Magic code for internal PAC
	 */
	regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
		    CS35LXX_TEST_UNLOCK1);
	regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
		    CS35LXX_TEST_UNLOCK2);

	usleep_range(9500, 10500);

	regmap_write(cs35lxx->regmap, CS35LXX_PAC_CTL1,
		    CS35LXX_PAC_RESET);
	regmap_write(cs35lxx->regmap, CS35LXX_PAC_CTL3,
		    CS35LXX_PAC_MEM_ACCESS);
	regmap_write(cs35lxx->regmap, CS35LXX_PAC_PMEM_WORD0,
		    CS35LXX_B0_PAC_PATCH);

	regmap_write(cs35lxx->regmap, CS35LXX_PAC_CTL3,
		    CS35LXX_PAC_MEM_ACCESS_CLR);
	regmap_write(cs35lxx->regmap, CS35LXX_PAC_CTL1,
		    CS35LXX_PAC_ENABLE_MASK);

	usleep_range(9500, 10500);

	ret = regmap_read(cs35lxx->regmap, CS35LXX_INT4_STATUS, &val);
	if (ret < 0) {
		dev_err(cs35lxx->dev, "Failed to read int4_status %d\n", ret);
		return ret;
	}

	count = 0;
	while (!(val & CS35LXX_MCU_CONFIG_CLR)) {
		usleep_range(100, 200);
		count++;

		ret = regmap_read(cs35lxx->regmap, CS35LXX_INT4_STATUS,
				  &val);
		if (ret < 0) {
			dev_err(cs35lxx->dev, "Failed to read int4_status %d\n",
				ret);
			return ret;
		}

		if (count >= 100)
			return -EINVAL;
	}

	regmap_write(cs35lxx->regmap, CS35LXX_INT4_STATUS,
		    CS35LXX_MCU_CONFIG_CLR);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_PAC_CTL1,
			  CS35LXX_PAC_ENABLE_MASK, 0);

	regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
		    CS35LXX_TEST_LOCK1);
	regmap_write(cs35lxx->regmap, CS35LXX_TESTKEY_CTRL,
		    CS35LXX_TEST_LOCK2);

	return 0;
}

static void cs35lxx_apply_vpbr_config(struct cs35lxx_private *cs35lxx)
{
	struct cs35lxx_platform_data *pdata = &cs35lxx->pdata;
	struct cs35lxx_vpbr_cfg *vpbr_config = &pdata->vpbr_config;

	regmap_update_bits(cs35lxx->regmap, CS35LXX_PWR_CTRL3,
			  CS35LXX_VPBR_EN_MASK,
			  vpbr_config->vpbr_en <<
			  CS35LXX_VPBR_EN_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_THLD_MASK,
			  vpbr_config->vpbr_thld <<
			  CS35LXX_VPBR_THLD_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_MAX_ATTN_MASK,
			  vpbr_config->vpbr_max_attn <<
			  CS35LXX_VPBR_MAX_ATTN_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_ATK_VOL_MASK,
			  vpbr_config->vpbr_atk_vol <<
			  CS35LXX_VPBR_ATK_VOL_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_ATK_RATE_MASK,
			  vpbr_config->vpbr_atk_rate <<
			  CS35LXX_VPBR_ATK_RATE_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_WAIT_MASK,
			  vpbr_config->vpbr_wait <<
			  CS35LXX_VPBR_WAIT_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_REL_RATE_MASK,
			  vpbr_config->vpbr_rel_rate <<
			  CS35LXX_VPBR_REL_RATE_SHIFT);
	regmap_update_bits(cs35lxx->regmap, CS35LXX_VPBR_CFG,
			  CS35LXX_VPBR_MUTE_EN_MASK,
			  vpbr_config->vpbr_mute_en <<
			  CS35LXX_VPBR_MUTE_EN_SHIFT);
}

static int cs35lxx_send_data_to_dsp(struct cs35lxx_private *cs35lxx)
{
	int ret = 0;
	uint32_t *dsp_send_buffer = NULL;

	switch (cs35lxx->cspl.cspl_cmd_type) {
		case CSPL_CMD_SCENE:
			dsp_send_buffer = kmalloc(sizeof(struct cs35lxx_set_scene_cmd), GFP_KERNEL);
			if (!dsp_send_buffer) {
				dev_err(cs35lxx->dev, "Failed to allocate memory for buffer\n");
				return -ENOMEM;
			}

			dsp_send_buffer[0] = cs35lxx->cspl.scene_param.command;
			dsp_send_buffer[1] = cs35lxx->cspl.scene_param.data.scene;
			dsp_send_buffer[2] = cs35lxx->cspl.scene_param.data.empty1;
			dsp_send_buffer[3] = cs35lxx->cspl.scene_param.data.empty2;
			dsp_send_buffer[4] = cs35lxx->cspl.scene_param.data.empty3;

			ret = mtk_spk_send_ipi_buf_to_dsp(dsp_send_buffer,
											  (sizeof(struct cs35lxx_set_scene_cmd)));
			if (ret) {
				dev_err(cs35lxx->dev, "Failed send buffer to the DSP %d\n", ret);
				goto exit;
			}
			break;
		case CSPL_CMD_CALIBRATION:
			dsp_send_buffer = kmalloc(sizeof(struct cs35lxx_calib_cmd), GFP_KERNEL);
			if (!dsp_send_buffer) {
				dev_err(cs35lxx->dev, "Failed to allocate memory for buffer\n");
				return -ENOMEM;
			}

			dsp_send_buffer[0] = cs35lxx->cspl.calib_param.command;
			dsp_send_buffer[1] = cs35lxx->cspl.calib_param.data.temperature;
			dsp_send_buffer[2] = cs35lxx->cspl.calib_param.data.rdc;
			dsp_send_buffer[3] = cs35lxx->cspl.calib_param.data.status;
			dsp_send_buffer[4] = cs35lxx->cspl.calib_param.data.checksum;

			ret = mtk_spk_send_ipi_buf_to_dsp(dsp_send_buffer,
											  (sizeof(struct cs35lxx_calib_cmd)));
			if (ret) {
				dev_err(cs35lxx->dev, "Failed send buffer to the DSP %d\n", ret);
				goto exit;
			}
			break;
		case CSPL_CMD_DIAGNOSTICS:
			dsp_send_buffer = kmalloc(sizeof(struct cs35lxx_diagnostics_cmd), GFP_KERNEL);
			if (!dsp_send_buffer) {
				dev_err(cs35lxx->dev, "Failed to allocate memory for buffer\n");
				return -ENOMEM;
			}
			dsp_send_buffer[0] = cs35lxx->cspl.diag_param.command;
			dsp_send_buffer[1] = cs35lxx->cspl.diag_param.data.temperature;
			dsp_send_buffer[2] = cs35lxx->cspl.diag_param.data.f0;
			dsp_send_buffer[3] = cs35lxx->cspl.diag_param.data.status;

			ret = mtk_spk_send_ipi_buf_to_dsp(dsp_send_buffer,
											  (sizeof(struct cs35lxx_diagnostics_cmd)));
			if (ret) {
				dev_err(cs35lxx->dev, "Failed send buffer to the DSP %d\n", ret);
				goto exit;
			}
			break;
		default:
			dev_err(cs35lxx->dev, "Unsupported command\n");
			ret = -EINVAL;
			break;
	}

exit:
	if (dsp_send_buffer != NULL)
		kfree(dsp_send_buffer);

	return ret;
}

static int cs35lxx_receive_data_from_dsp(struct cs35lxx_private *cs35lxx)
{
	int ret = 0;
	uint32_t data_length = 0;

	if (!cs35lxx->cspl.dsp_recv_buffer) {
		dev_err(cs35lxx->dev, "Failed to allocate memory for buffer\n");
		return -ENOMEM;
	}

	switch (cs35lxx->cspl.cspl_cmd_type) {
		case CSPL_CMD_HAND_SHAKE:
			ret = mtk_spk_recv_ipi_buf_from_dsp((int8_t*)cs35lxx->cspl.dsp_recv_buffer,
												sizeof(struct cs35lxx_calib_cmd),
												&data_length);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to read buffer from the DSP\n");
				return -EFAULT;
			}

			cs35lxx->cspl.calib_param.command = cs35lxx->cspl.dsp_recv_buffer[0];

			break;
		case CSPL_CMD_CALIBRATION:
			ret = mtk_spk_recv_ipi_buf_from_dsp((int8_t*)cs35lxx->cspl.dsp_recv_buffer,
												sizeof(struct cs35lxx_calib_cmd),
												&data_length);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to read buffer from the DSP\n");
				return -EFAULT;
			}

			cs35lxx->cspl.calib_param.command = cs35lxx->cspl.dsp_recv_buffer[0];
			cs35lxx->cspl.calib_param.data.temperature = cs35lxx->cspl.dsp_recv_buffer[1];
			cs35lxx->cspl.calib_param.data.rdc = cs35lxx->cspl.dsp_recv_buffer[2];
			cs35lxx->cspl.calib_param.data.status = cs35lxx->cspl.dsp_recv_buffer[3];
			cs35lxx->cspl.calib_param.data.checksum = cs35lxx->cspl.dsp_recv_buffer[4];

			break;
		case CSPL_CMD_R0:
			ret = mtk_spk_recv_ipi_buf_from_dsp((int8_t*)cs35lxx->cspl.dsp_recv_buffer,
												sizeof(struct cs35lxx_r0_cmd),
												&data_length);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to read buffer from the DSP\n");
				return -EFAULT;
			}

			cs35lxx->cspl.r0_param.command = cs35lxx->cspl.dsp_recv_buffer[0];
			cs35lxx->cspl.r0_param.data.r0 = cs35lxx->cspl.dsp_recv_buffer[2];

			break;
		case CSPL_CMD_DIAGNOSTICS:
			ret = mtk_spk_recv_ipi_buf_from_dsp((int8_t*)cs35lxx->cspl.dsp_recv_buffer,
												sizeof(struct cs35lxx_diagnostics_cmd),
												&data_length);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to read buffer from the DSP\n");
				return -EFAULT;
			}

			cs35lxx->cspl.diag_param.command = cs35lxx->cspl.dsp_recv_buffer[0];
			cs35lxx->cspl.diag_param.data.f0 = cs35lxx->cspl.dsp_recv_buffer[2];
			cs35lxx->cspl.diag_param.data.status = cs35lxx->cspl.dsp_recv_buffer[3];
			break;
		default:
			dev_err(cs35lxx->dev, "Unsupported command\n");
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int cs35lxx_get_calib_struct(struct cs35lxx_private *cs35lxx)
{
	int ret;

	cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_CALIBRATION;
	ret = cs35lxx_receive_data_from_dsp(cs35lxx);
	if (ret) {
		dev_err(cs35lxx->dev, "Failed to read calib struct from the DSP\n");
		return ret;
	}
	if (cs35lxx->cspl.calib_param.command != CSPL_CMD_GET_CALIBRATION_PARAM) {
		dev_err(cs35lxx->dev, "Command does not match\n");
		return -EINVAL;
	}

	dev_info(cs35lxx->dev, "Read cal struct:\n");
	dev_info(cs35lxx->dev, "\tStatus: %d\n", cs35lxx->cspl.calib_param.data.status);
	dev_info(cs35lxx->dev, "\tRDC: 0x%x\n", cs35lxx->cspl.calib_param.data.rdc);
	dev_info(cs35lxx->dev, "\tAmbient: %d\n", cs35lxx->cspl.calib_param.data.temperature);
	dev_info(cs35lxx->dev, "\tChecksum: 0x%x\n", cs35lxx->cspl.calib_param.data.checksum);

	return ret;
}

static void cs35lxx_set_cal_struct(struct work_struct *wk)
{
	struct cs35lxx_private *cs35lxx;
	int i;
	int ret;
	int try_times = CS35LXX_CSPL_HANDSHAKE_MAX_TRY_TIME;

	cs35lxx = container_of(container_of(wk, struct cs35lxx_cspl,
						   set_cal_struct_work),
						   struct cs35lxx_private, cspl);

	// CSPL handshake
	//cs35lxx->cspl.cspl_ready = false;
	while (atomic_read(&cs35lxx->cspl.set_cal_struct_monitor)) {
		for (i = 0; i < try_times && (!cs35lxx->cspl.cspl_ready); i++) {
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_HAND_SHAKE;
			ret = cs35lxx_receive_data_from_dsp(cs35lxx);
			if (ret == 0 && cs35lxx->cspl.calib_param.command == CSPL_CMD_LIBARAY_READY) {
				dev_info(cs35lxx->dev, "CSPL is ready, set tuning to %d, \n",
							cs35lxx->cspl.scene_param.data.scene);
				//CSPL defult tuning is "MUSIC" when aurisys manager created
				//here CSPL is ready, load the tuning before set cal_struct
				if(cs35lxx->cspl.scene_param.data.scene != MUSIC) {
					cs35lxx->cspl.scene_param.command = CSPL_CMD_SET_SCENE;
					cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_SCENE;
					ret = cs35lxx_send_data_to_dsp(cs35lxx);
					usleep_range(10000, 10100);
				}
				cs35lxx->cspl.cspl_ready = true;
				break;
			}
			usleep_range(200000, 200100);
		}

		if (cs35lxx->cspl.cspl_ready) {
			cs35lxx->cspl.calib_param.command = CSPL_CMD_SET_CALIBRATION_PARAM;
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_CALIBRATION;
			dev_info(cs35lxx->dev, "calib struct to be sent to CSPL:\n");
			dev_info(cs35lxx->dev, "\tStatus: %d\n", cs35lxx->cspl.calib_param.data.status);
			dev_info(cs35lxx->dev, "\tRDC: 0x%x\n", cs35lxx->cspl.calib_param.data.rdc);
			dev_info(cs35lxx->dev, "\tAmbient: %d\n", cs35lxx->cspl.calib_param.data.temperature);
			dev_info(cs35lxx->dev, "\tChecksum: 0x%x\n", cs35lxx->cspl.calib_param.data.checksum);

			ret = cs35lxx_send_data_to_dsp(cs35lxx);
		}

		atomic_set(&cs35lxx->cspl.set_cal_struct_monitor, 0);
	}
}

static int cs35lxx_get_r0(struct cs35lxx_private *cs35lxx)
{
	int ret;

	cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_R0;
	ret = cs35lxx_receive_data_from_dsp(cs35lxx);
	if (ret) {
		dev_err(cs35lxx->dev, "Failed to read r0 from the DSP\n");
		return ret;
	}
	if (cs35lxx->cspl.r0_param.command != CSPL_CMD_GET_R0) {
		dev_err(cs35lxx->dev, "Command does not match\n");
		return -EINVAL;
	}

	dev_info(cs35lxx->dev, "Read r0 struct:\n");
	dev_info(cs35lxx->dev, "\tT_realtime: 0x%x\n", cs35lxx->cspl.r0_param.data.r0);

	return ret;
}

static void cs35lxx_calibration_start(struct work_struct *wk)
{
	struct cs35lxx_private *cs35lxx;
	int i;
	int ret;
	int try_times = CS35LXX_CSPL_HANDSHAKE_MAX_TRY_TIME;

	cs35lxx = container_of(container_of(wk, struct cs35lxx_cspl, calib_work),
						   struct cs35lxx_private, cspl);

	// CSPL handshake
	//cs35lxx->cspl.cspl_ready = false;
	while (atomic_read(&cs35lxx->cspl.calib_monitor)) {
		for (i = 0; i < try_times && (!cs35lxx->cspl.cspl_ready); i++) {
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_HAND_SHAKE;
			ret = cs35lxx_receive_data_from_dsp(cs35lxx);
			if (ret == 0 && cs35lxx->cspl.calib_param.command == CSPL_CMD_LIBARAY_READY) {
				dev_info(cs35lxx->dev, "CSPL is ready\n");
				cs35lxx->cspl.cspl_ready = true;
				break;
			}
			usleep_range(200000, 200100);
		}

		if (cs35lxx->cspl.cspl_ready) {
			cs35lxx->cspl.calib_param.command = CSPL_CMD_START_CALIBRATION;
			cs35lxx->cspl.calib_param.data.temperature = cs35lxx->cspl.ambient_temperature;
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_CALIBRATION;
			ret = cs35lxx_send_data_to_dsp(cs35lxx);
		}

		atomic_set(&cs35lxx->cspl.calib_monitor, 0);
	}
}

static int cs35lxx_calibration_stop(struct cs35lxx_private *cs35lxx)
{
	cs35lxx->cspl.calib_param.command = CSPL_CMD_STOP_CALIBRATION;
	cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_CALIBRATION;

	return cs35lxx_send_data_to_dsp(cs35lxx);
}

static void cs35lxx_diagnostics_start(struct work_struct *wk)
{
	struct cs35lxx_private *cs35lxx;
	int i;
	int ret;
	int try_times = CS35LXX_CSPL_HANDSHAKE_MAX_TRY_TIME;

	cs35lxx = container_of(container_of(wk, struct cs35lxx_cspl, diag_work),
						   struct cs35lxx_private, cspl);

	// CSPL handshake
	//cs35lxx->cspl.cspl_ready = false;
	while (atomic_read(&cs35lxx->cspl.diag_monitor)) {
		for (i = 0; i < try_times && (!cs35lxx->cspl.cspl_ready); i++) {
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_HAND_SHAKE;
			ret = cs35lxx_receive_data_from_dsp(cs35lxx);
			if (ret == 0 && cs35lxx->cspl.diag_param.command == CSPL_CMD_LIBARAY_READY) {
				dev_info(cs35lxx->dev, "CSPL is ready\n");
				cs35lxx->cspl.cspl_ready = true;
				break;
			}
			usleep_range(200000, 200100);
		}

		if ( cs35lxx->cspl.cspl_ready) {
			cs35lxx->cspl.diag_param.command = CSPL_CMD_START_DIAGNOSTICS;
			cs35lxx->cspl.diag_param.data.temperature = cs35lxx->cspl.ambient_temperature;
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_DIAGNOSTICS;
			ret = cs35lxx_send_data_to_dsp(cs35lxx);
		}

		atomic_set(&cs35lxx->cspl.diag_monitor, 0);
	}
}

static int cs35lxx_diagnostics_stop(struct cs35lxx_private *cs35lxx)
{
	cs35lxx->cspl.diag_param.command = CSPL_CMD_STOP_DIAGNOSTICS;
	cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_DIAGNOSTICS;

	return cs35lxx_send_data_to_dsp(cs35lxx);
}

static int cs35lxx_get_f0(struct cs35lxx_private *cs35lxx)
{
	int ret;

	cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_DIAGNOSTICS;
	ret = cs35lxx_receive_data_from_dsp(cs35lxx);
	if (ret) {
		dev_err(cs35lxx->dev, "Failed to read f0 from the DSP\n");
		return ret;
	}
	if (cs35lxx->cspl.diag_param.command != CSPL_CMD_GET_F0) {
		dev_err(cs35lxx->dev, "Command does not match\n");
		return -EINVAL;
	}

	dev_info(cs35lxx->dev, "Read F0 struct:\n");
	dev_info(cs35lxx->dev, "\tStatus: %d\n", cs35lxx->cspl.diag_param.data.status);
	dev_info(cs35lxx->dev, "\tF0: 0x%x\n", cs35lxx->cspl.diag_param.data.f0);
	dev_info(cs35lxx->dev, "\tAmbient: %d\n", cs35lxx->cspl.diag_param.data.temperature);

	return ret;
}

static void cs35lxx_bypass(struct work_struct *wk)
{
	struct cs35lxx_private *cs35lxx;
	int i;
	int ret;
	int try_times = CS35LXX_CSPL_HANDSHAKE_MAX_TRY_TIME;

	cs35lxx = container_of(container_of(wk, struct cs35lxx_cspl,
						   bypass_work),
						   struct cs35lxx_private, cspl);

	// CSPL handshake
	while (atomic_read(&cs35lxx->cspl.bypass_monitor)) {
		for (i = 0; i < try_times && (!cs35lxx->cspl.cspl_ready); i++) {
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_HAND_SHAKE;
			ret = cs35lxx_receive_data_from_dsp(cs35lxx);
			if (ret == 0 && cs35lxx->cspl.calib_param.command == CSPL_CMD_LIBARAY_READY) {
				dev_info(cs35lxx->dev, "CSPL is ready\n");
				cs35lxx->cspl.cspl_ready = true;
				break;
			}
			usleep_range(200000, 200100);
		}

		if (cs35lxx->cspl.cspl_ready) {
			if (cs35lxx->cspl.dsp_bypass == 1)
				cs35lxx->cspl.calib_param.command = CSPL_CMD_ENABLE_DSPBYPASS;
			else
				cs35lxx->cspl.calib_param.command = CSPL_CMD_DISABLE_DSPBYPASS;
			cs35lxx->cspl.cspl_cmd_type = CSPL_CMD_CALIBRATION;
			ret = cs35lxx_send_data_to_dsp(cs35lxx);
		}

		atomic_set(&cs35lxx->cspl.bypass_monitor, 0);
	}
}

static long cs35lxx_ioctl(struct file *f, unsigned int cmd, void __user *arg)
{
	struct miscdevice *dev = f->private_data;
	struct cs35lxx_private *cs35lxx;
	int ret = CS35LXX_IOCTL_STATUS_OK;
	int val = 0;

	cs35lxx = container_of(dev, struct cs35lxx_private, misc_dev);

	mutex_lock(&cs35lxx->lock);

	if (copy_from_user(&val, arg, sizeof(val))) {
		dev_err(cs35lxx->dev, "copy from user failed\n");
		ret = -EFAULT;
		goto exit;
	}

	switch (cmd) {
		case CS35LXX_SPK_DAC_VOLUME:
			break;
		case CS35LXX_SPK_POWER_ON:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_POWER_ON\n");
			ret = cs35lxx_spk_power_on(cs35lxx);
			cs35lxx->cspl.scene_param.data.scene = val;
			break;
		case CS35LXX_SPK_POWER_OFF:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_POWER_OFF\n");
			ret = cs35lxx_spk_power_off(cs35lxx);
			cs35lxx->cspl.cspl_ready = false;
			break;
		case CS35LXX_SPK_DSP_BYPASS:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_DSP_BYPASS, val = %d\n", val);
			if (val == 1) {
				cs35lxx->cspl.calib_param.command = CSPL_CMD_ENABLE_DSPBYPASS;
				cs35lxx->cspl.dsp_bypass = 1;
				//bypass for AT loopback test, gain set to analog 13dB, digital -0.5dB
				regmap_write(cs35lxx->regmap, CS35LXX_AMP_DIG_VOL_CTRL, 0x0000BFE0);
				regmap_write(cs35lxx->regmap, CS35LXX_AMP_GAIN_CTRL, 0x000001B3);
			} else if (val == 0) {
				cs35lxx->cspl.calib_param.command = CSPL_CMD_DISABLE_DSPBYPASS;
				cs35lxx->cspl.dsp_bypass = 0;
				//disable bypass, back to normal gain
				regmap_write(cs35lxx->regmap, CS35LXX_AMP_DIG_VOL_CTRL, 0x00008000);
				regmap_write(cs35lxx->regmap, CS35LXX_AMP_GAIN_CTRL, 0x00000233);
			} else {
				dev_err(cs35lxx->dev, "Unsupported value %d\n", val);
				cs35lxx->cspl.dsp_bypass = 0;
				ret = -EFAULT;
				goto exit;
			}
			atomic_set(&cs35lxx->cspl.bypass_monitor, 1);
			queue_work(cs35lxx->cspl.bypass_wq, &cs35lxx->cspl.bypass_work);
			break;
		case CS35LXX_SPK_SWITCH_CALIBRATION:
			break;
		case CS35LXX_SPK_SWITCH_CONFIGURATION:
			break;
		case CS35LXX_SPK_GET_R0:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_GET_R0\n");
			ret = cs35lxx_get_calib_struct(cs35lxx);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to get calib struct from dsp\n");
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((uint32_t *) arg,
							 &cs35lxx->cspl.calib_param.data.rdc,
							 sizeof(uint32_t))) {
				dev_err(cs35lxx->dev, "copy to user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			break;
		case CS35LXX_SPK_GET_F0:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_GET_F0\n");
			ret = cs35lxx_get_f0(cs35lxx);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to get f0 from dsp\n");
				ret = -EFAULT;
				goto exit;
			}
			dev_info(cs35lxx->dev, "F0 = %d\n", cs35lxx->cspl.diag_param.data.f0);
			if (copy_to_user((uint32_t *) arg,
							 &cs35lxx->cspl.diag_param.data.f0,
							 sizeof(uint32_t))) {
				dev_err(cs35lxx->dev, "copy to user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			break;
		case CS35LXX_SPK_GET_CAL_STRUCT:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_GET_CAL_STRUCT\n");
			ret = cs35lxx_get_calib_struct(cs35lxx);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to get calib struct from dsp\n");
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((struct cs35lxx_calib_data *)arg,
							  &cs35lxx->cspl.calib_param.data,
							  sizeof(struct cs35lxx_calib_data))) {
				dev_err(cs35lxx->dev, "copy to user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			break;
		case CS35LXX_SPK_SET_CAL_STRUCT:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_SET_CAL_STRUCT\n");
			if (copy_from_user(cs35lxx->cspl.dsp_recv_buffer, arg,
							   sizeof(struct cs35lxx_calib_data))) {
				dev_err(cs35lxx->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			dev_info(cs35lxx->dev, "Received struct cs35lxx_calib_data:\n");
			dev_info(cs35lxx->dev, "\tAmbient: %d\n", cs35lxx->cspl.dsp_recv_buffer[0]);
			dev_info(cs35lxx->dev, "\tRDC: 0x%x\n", cs35lxx->cspl.dsp_recv_buffer[1]);
			dev_info(cs35lxx->dev, "\tStatus: %d\n", cs35lxx->cspl.dsp_recv_buffer[2]);
			dev_info(cs35lxx->dev, "\tChecksum: 0x%x\n", cs35lxx->cspl.dsp_recv_buffer[3]);

			cs35lxx->cspl.calib_param.command = CSPL_CMD_SET_CALIBRATION_PARAM;
			cs35lxx->cspl.calib_param.data.temperature = cs35lxx->cspl.dsp_recv_buffer[0];
			cs35lxx->cspl.calib_param.data.rdc = cs35lxx->cspl.dsp_recv_buffer[1];
			cs35lxx->cspl.calib_param.data.status = cs35lxx->cspl.dsp_recv_buffer[2];
			cs35lxx->cspl.calib_param.data.checksum = cs35lxx->cspl.dsp_recv_buffer[3];

			atomic_set(&cs35lxx->cspl.set_cal_struct_monitor, 1);
			queue_work(cs35lxx->cspl.set_cal_struct_wq, &cs35lxx->cspl.set_cal_struct_work);
			break;
		case CS35LXX_SPK_SET_AMBIENT:
			dev_info(cs35lxx->dev, "copy from user val = %d\n", val);
			cs35lxx->cspl.ambient_temperature = val;
			ret = CS35LXX_IOCTL_STATUS_OK;
			break;
		case CS35LXX_SPK_SET_R0:
			break;
		case CS35LXX_SPK_SWITCH_FIRMWARE:
			break;
		case CS35LXX_SPK_GET_R0_REALTIME:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_GET_R0_REALTIME\n");
			ret = cs35lxx_get_r0(cs35lxx);
			if (ret) {
				dev_err(cs35lxx->dev, "Failed to get calib struct from dsp\n");
				ret = -EFAULT;
				goto exit;
			}
			if (copy_to_user((uint32_t *) arg,
							 &cs35lxx->cspl.r0_param.data.r0,
							 sizeof(uint32_t))) {
				dev_err(cs35lxx->dev, "copy to user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			break;
		case CS35LXX_SPK_SET_DEFAULT_CALIB:
			break;
		case CS35LXX_SPK_GET_CALIB_STATE:
			break;
		case CS35LXX_SPK_START_CALIBRATION:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_START_CALIBRATION\n");
			atomic_set(&cs35lxx->cspl.calib_monitor, 1);
			queue_work(cs35lxx->cspl.calib_wq, &cs35lxx->cspl.calib_work);
			break;
		case CS35LXX_SPK_STOP_CALIBRATION:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_STOP_CALIBRATION\n");
			cancel_work_sync(&cs35lxx->cspl.calib_work);
			flush_workqueue(cs35lxx->cspl.calib_wq);
			ret = cs35lxx_calibration_stop(cs35lxx);
			break;
		case CS35LXX_SPK_START_DIAGNOSTICS:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_START_DIAGNOSTICS\n");
			atomic_set(&cs35lxx->cspl.diag_monitor, 1);
			queue_work(cs35lxx->cspl.diag_wq, &cs35lxx->cspl.diag_work);
			break;
		case CS35LXX_SPK_STOP_DIAGNOSTICS:
			dev_info(cs35lxx->dev, "CS35LXX_SPK_STOP_DIAGNOSTICS\n");
			cancel_work_sync(&cs35lxx->cspl.diag_work);
			flush_workqueue(cs35lxx->cspl.diag_wq);
			ret = cs35lxx_diagnostics_stop(cs35lxx);
			break;
		default:
			dev_err(cs35lxx->dev, "Invalid IOCTL, command = %d\n", cmd);
			return -EINVAL;
	}

exit:
	mutex_unlock(&cs35lxx->lock);

	return ret;
}

static long cs35lxx_unlocked_ioctl(struct file *f, unsigned int cmd,
				unsigned long arg)
{
	return cs35lxx_ioctl(f, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long cs35lxx_compat_ioctl(struct file *f, unsigned int cmd,
				unsigned long arg)
{
	struct miscdevice *dev = f->private_data;
	struct cs35lxx_private *cs35lxx;
	unsigned int cmd64;

	cs35lxx = container_of(dev, struct cs35lxx_private, misc_dev);

	switch (cmd) {
		case CS35LXX_SPK_DAC_VOLUME_COMPAT:
			cmd64 = CS35LXX_SPK_DAC_VOLUME;
			break;
		case CS35LXX_SPK_POWER_ON_COMPAT:
			cmd64 = CS35LXX_SPK_POWER_ON;
			break;
		case CS35LXX_SPK_POWER_OFF_COMPAT:
			cmd64 = CS35LXX_SPK_POWER_OFF;
			break;
		case CS35LXX_SPK_DSP_BYPASS_COMPAT:
			cmd64 = CS35LXX_SPK_DSP_BYPASS;
			break;
		case CS35LXX_SPK_SWITCH_CONFIGURATION_COMPAT:
			cmd64 = CS35LXX_SPK_SWITCH_CONFIGURATION;
			break;
		case CS35LXX_SPK_SWITCH_CALIBRATION_COMPAT:
			cmd64 = CS35LXX_SPK_SWITCH_CALIBRATION;
			break;
		case CS35LXX_SPK_GET_R0_COMPAT:
			cmd64 = CS35LXX_SPK_GET_R0;
			break;
		case CS35LXX_SPK_GET_F0_COMPAT:
			cmd64 = CS35LXX_SPK_GET_F0;
			break;
		case CS35LXX_SPK_GET_CAL_STRUCT_COMPAT:
			cmd64 = CS35LXX_SPK_GET_CAL_STRUCT;
			break;
		case CS35LXX_SPK_SET_CAL_STRUCT_COMPAT:
			cmd64 = CS35LXX_SPK_SET_CAL_STRUCT;
			break;
		case CS35LXX_SPK_SET_AMBIENT_COMPAT:
			cmd64 = CS35LXX_SPK_SET_AMBIENT;
			break;
		case CS35LXX_SPK_SWITCH_FIRMWARE_COMPAT:
			cmd64 = CS35LXX_SPK_SWITCH_FIRMWARE;
			break;
		case CS35LXX_SPK_GET_R0_REALTIME_COMPAT:
			cmd64 = CS35LXX_SPK_GET_R0_REALTIME;
			break;
		case CS35LXX_SPK_SET_DEFAULT_CALIB_COMPAT:
			cmd64 = CS35LXX_SPK_SET_DEFAULT_CALIB;
			break;
		case CS35LXX_SPK_GET_CALIB_STATE_COMPAT:
			cmd64 = CS35LXX_SPK_GET_CALIB_STATE;
			break;
		case CS35LXX_SPK_START_CALIBRATION_COMPAT:
			cmd64 = CS35LXX_SPK_START_CALIBRATION;
			break;
		case CS35LXX_SPK_STOP_CALIBRATION_COMPAT:
			cmd64 = CS35LXX_SPK_STOP_CALIBRATION;
			break;
		case CS35LXX_SPK_START_DIAGNOSTICS_COMPAT:
			cmd64 = CS35LXX_SPK_START_DIAGNOSTICS;
			break;
		case CS35LXX_SPK_STOP_DIAGNOSTICS_COMPAT:
			cmd64 = CS35LXX_SPK_STOP_DIAGNOSTICS;
			break;
		default:
			dev_err(cs35lxx->dev, "Invalid IOCTL, command = %d\n", cmd);
			return -EINVAL;
	}

	return cs35lxx_ioctl(f, cmd64, compat_ptr(arg));
}
#endif

static const struct reg_sequence cs35lxx_reva0_errata_patch[] = {
	{ CS35LXX_TESTKEY_CTRL,		CS35LXX_TEST_UNLOCK1 },
	{ CS35LXX_TESTKEY_CTRL,		CS35LXX_TEST_UNLOCK2 },
	/* Errata Writes */
	{ CS35LXX_OTP_CTRL1,		0x00002060 },
	{ CS35LXX_OTP_CTRL2,		0x00000001 },
	{ CS35LXX_OTP_CTRL1,		0x00002460 },
	{ CS35LXX_OTP_CTRL2,		0x00000001 },
	{ 0x00002088,			0x012A1838 },
	{ 0x00003014,			0x0100EE0E },
	{ 0x00003008,			0x0008184A },
	{ 0x00007418,			0x509001C8 },
	{ 0x00007064,			0x0929A800 },
	{ 0x00002D10,			0x0002C01C },
	{ 0x0000410C,			0x00000A11 },
	{ 0x00006E08,			0x8B19140C },
	{ 0x00006454,			0x0300000A },
	{ CS35LXX_AMP_NG_CTRL,		0x000020EF },
	{ 0x00007E34,			0x0000000E },
	{ 0x0000410C,			0x00000A11 },
	{ 0x00007410,			0x20514B00 },
	/* PAC Config */
	{ CS35LXX_CTRL_OVRRIDE,		0x00000000 },
	{ CS35LXX_PAC_INT0_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT1_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT2_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT3_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT4_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT5_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT6_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT7_CTRL,	0x00860001 },
	{ CS35LXX_PAC_INT_FLUSH_CTRL,	0x000000FF },
	{ CS35LXX_TESTKEY_CTRL,		CS35LXX_TEST_LOCK1 },
	{ CS35LXX_TESTKEY_CTRL,		CS35LXX_TEST_LOCK2 },
};

static const struct reg_sequence cs35lxx_revb0_errata_patch[] = {
	{ CS35LXX_TESTKEY_CTRL,	CS35LXX_TEST_UNLOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_UNLOCK2 },
	{ 0x00007064,		0x0929A800 },
	{ 0x00007850,		0x00002FA9 },
	{ 0x00007854,		0x0003F1D5 },
	{ 0x00007858,		0x0003F5E3 },
	{ 0x0000785C,		0x00001137 },
	{ 0x00007860,		0x0001A7A5 },
	{ 0x00007864,		0x0002F16A },
	{ 0x00007868,		0x00003E21 },
	{ 0x00007848,		0x00000001 },
	{ 0x00003854,		0x05180240 },
	{ 0x00007418,		0x509001C8 },
	{ 0x0000394C,		0x028764BD },
	{ CS35LXX_TESTKEY_CTRL,	CS35LXX_TEST_LOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_LOCK2 },
};
static const struct reg_sequence cs35lxx_revb1_errata_patch[] = {
	{ CS35LXX_TESTKEY_CTRL,	CS35LXX_TEST_UNLOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_UNLOCK2 },
	{ 0x00007064,		0x0929A800 },
	{ 0x00007850,		0x00002FA9 },
	{ 0x00007854,		0x0003F1D5 },
	{ 0x00007858,		0x0003F5E3 },
	{ 0x0000785C,		0x00001137 },
	{ 0x00007860,		0x0001A7A5 },
	{ 0x00007864,		0x0002F16A },
	{ 0x00007868,		0x00003E21 },
	{ 0x00007848,		0x00000001 },
	{ CS35LXX_TESTKEY_CTRL,	CS35LXX_TEST_LOCK1 },
	{ CS35LXX_TESTKEY_CTRL, CS35LXX_TEST_LOCK2 },
};


static const struct file_operations cs35lxx_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cs35lxx_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cs35lxx_compat_ioctl,
#endif
};

static int cs35lxx_i2c_probe(struct i2c_client *i2c_client,
				const struct i2c_device_id *id)
{
	struct cs35lxx_private *cs35lxx;
	struct device *dev = &i2c_client->dev;
	struct cs35lxx_platform_data *pdata = dev_get_platdata(dev);
	int ret, chip_irq_pol, i;
	int irq_pol = IRQF_TRIGGER_LOW;
	u32 reg_id, reg_revid, l37_id_reg;
	struct smartpa_vendor_info vendor_info;
	int irq_gpio = 0;

	cs35lxx = devm_kzalloc(dev, sizeof(struct cs35lxx_private), GFP_KERNEL);
	if (!cs35lxx)
		return -ENOMEM;

	cs35lxx->dev = dev;

	i2c_set_clientdata(i2c_client, cs35lxx);
	cs35lxx->regmap = devm_regmap_init_i2c(i2c_client, &cs35lxx_regmap);
	if (IS_ERR(cs35lxx->regmap)) {
		ret = PTR_ERR(cs35lxx->regmap);
		dev_err(dev, "regmap_init() failed: %d\n", ret);
		goto err;
	}

	cs35lxx->num_supplies = ARRAY_SIZE(cs35lxx_supplies);
	for (i = 0; i < ARRAY_SIZE(cs35lxx_supplies); i++)
		cs35lxx->supplies[i].supply = cs35lxx_supplies[i];

	ret = devm_regulator_bulk_get(dev, cs35lxx->num_supplies,
				    cs35lxx->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	if (pdata) {
		cs35lxx->pdata = *pdata;
	} else {
		pdata = devm_kzalloc(dev, sizeof(struct cs35lxx_platform_data),
				    GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		if (i2c_client->dev.of_node) {
			ret = cs35lxx_handle_of_data(i2c_client, pdata);
			if (ret != 0)
				return ret;
		}

		cs35lxx->pdata = *pdata;
	}

	ret = regulator_bulk_enable(cs35lxx->num_supplies, cs35lxx->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		return ret;
	}

	/* returning NULL can be an option if in stereo mode */
	cs35lxx->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(cs35lxx->reset_gpio)) {
		ret = PTR_ERR(cs35lxx->reset_gpio);
		cs35lxx->reset_gpio = NULL;
		if (ret == -EBUSY) {
			dev_info(dev, "Reset line busy, assuming shared reset\n");
		} else {
			dev_err(dev, "Failed to get reset GPIO: %d\n", ret);
			goto err_disable_regs;
		}
	}

	if (cs35lxx->reset_gpio) {
		gpiod_set_value_cansleep(cs35lxx->reset_gpio, 1);
		usleep_range(2000, 2100);
		gpiod_set_value_cansleep(cs35lxx->reset_gpio, 0);
		usleep_range(1000, 1100);
	}

	usleep_range(2000, 2100);

	/* initialize amplifier */
	ret = regmap_read(cs35lxx->regmap, CS35LXX_SW_RESET, &reg_id);
	if (ret < 0) {
		dev_err(dev, "Get Device ID failed %d\n", ret);
		goto err;
	}

	if (reg_id != CS35LXX_CHIP_ID) {
		dev_err(dev, "Device ID (%X). Expected ID %X\n", reg_id,
			CS35LXX_CHIP_ID);
		ret = -ENODEV;
		goto err;
	}

	ret = regmap_read(cs35lxx->regmap, CS35LXX_REV_ID, &reg_revid);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Get Revision ID failed %d\n", ret);
		goto err;
	}

	cs35lxx->rev_id = reg_revid >> 8;

	ret = regmap_read(cs35lxx->regmap, CS35LXX_OTP_MEM30, &l37_id_reg);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Failed to read otp_id Register %d\n",
			ret);
		return ret;
	}

	if ((l37_id_reg & CS35LXX_OTP_REV_MASK) == CS35LXX_OTP_REV_L37)
		cs35lxx->chip_version = CS35LXX_12V_L37;
	else
		cs35lxx->chip_version = CS35LXX_10V_L36;

	switch (cs35lxx->rev_id) {
		case CS35LXX_REV_B1:
			ret = cs35lxx_pac(cs35lxx);
			if (ret < 0) {
				dev_err(dev, "Failed to Trim OTP %d\n", ret);
				goto err;
			}

			ret = regmap_register_patch(cs35lxx->regmap,
					cs35lxx_revb1_errata_patch,
					ARRAY_SIZE(cs35lxx_revb1_errata_patch));
			if (ret < 0) {
				dev_err(dev, "Failed to apply B0 errata patch %d\n",
					ret);
				goto err;
			}
			break;
		case CS35LXX_REV_B0:
			ret = cs35lxx_pac(cs35lxx);
			if (ret < 0) {
				dev_err(dev, "Failed to Trim OTP %d\n", ret);
				goto err;
			}

			ret = regmap_register_patch(cs35lxx->regmap,
					cs35lxx_revb0_errata_patch,
					ARRAY_SIZE(cs35lxx_revb0_errata_patch));
			if (ret < 0) {
				dev_err(dev, "Failed to apply B0 errata patch %d\n",
					ret);
				goto err;
			}
			break;
		case CS35LXX_REV_A0:
		default:
			ret = regmap_register_patch(cs35lxx->regmap,
					cs35lxx_reva0_errata_patch,
					ARRAY_SIZE(cs35lxx_reva0_errata_patch));
			if (ret < 0) {
				dev_err(dev, "Failed to apply A0 errata patch %d\n",
					ret);
				goto err;
			}
			break;
	}

	mutex_init(&cs35lxx->lock);

	if (pdata->vpbr_config.is_present)
		cs35lxx_apply_vpbr_config(cs35lxx);

	cs35lxx->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(cs35lxx->irq_gpio)) {
		ret = PTR_ERR(cs35lxx->irq_gpio);
		cs35lxx->irq_gpio = NULL;
		if (ret == -EBUSY) {
			dev_info(dev, "Reset line busy, assuming shared reset\n");
		} else {
			dev_err(dev, "Failed to get irq GPIO: %d\n", ret);
			goto err_disable_regs;
		}
	}

	if (cs35lxx->irq_gpio) {
		irq_gpio = gpiod_to_irq(cs35lxx->irq_gpio);
		dev_info(dev, "gpiod_to_irq ret: %d\n", irq_gpio);

		irq_pol = IRQF_TRIGGER_LOW;
	}

	switch (irq_pol) {
		case IRQF_TRIGGER_FALLING:
		case IRQF_TRIGGER_LOW:
			chip_irq_pol = 0;
			break;
		case IRQF_TRIGGER_RISING:
		case IRQF_TRIGGER_HIGH:
			chip_irq_pol = 1;
			break;
		default:
			dev_err(cs35lxx->dev, "Invalid IRQ polarity: %d\n", irq_pol);
			ret = -EINVAL;
			goto err;
	}

	regmap_update_bits(cs35lxx->regmap, CS35LXX_PAD_INTERFACE,
			  CS35LXX_INT_POL_SEL_MASK,
			  chip_irq_pol << CS35LXX_INT_POL_SEL_SHIFT);

	ret = devm_request_threaded_irq(dev, irq_gpio, NULL, cs35lxx_irq,
					IRQF_ONESHOT | irq_pol, "cs35lxx",
					cs35lxx);
	if (ret != 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		goto err;
	}

	regmap_update_bits(cs35lxx->regmap, CS35LXX_PAD_INTERFACE,
			  CS35LXX_INT_OUTPUT_EN_MASK, 1);

	/* Set interrupt masks for critical errors */
	regmap_write(cs35lxx->regmap, CS35LXX_INT1_MASK,
		    CS35LXX_INT1_MASK_DEFAULT);
	regmap_write(cs35lxx->regmap, CS35LXX_INT3_MASK,
		    CS35LXX_INT3_MASK_DEFAULT);

	ret = cs35lxx_probe(cs35lxx);
	if (ret < 0) {
		dev_err(cs35lxx->dev, "cs35lxx probe failed\n");
		goto err;
	}

	// Initialise calibration work
	cs35lxx->cspl.calib_wq = create_singlethread_workqueue("cs35lxx_calibration");
	INIT_WORK(&cs35lxx->cspl.calib_work, cs35lxx_calibration_start);

	// Initialise diagnostics work
	cs35lxx->cspl.diag_wq = create_singlethread_workqueue("cs35lxx_diagnostics");
	INIT_WORK(&cs35lxx->cspl.diag_work, cs35lxx_diagnostics_start);

	// Initialise set_cal_struct work
	cs35lxx->cspl.set_cal_struct_wq = create_singlethread_workqueue("cs35lxx_set_cal_struct");
	INIT_WORK(&cs35lxx->cspl.set_cal_struct_work, cs35lxx_set_cal_struct);

	// Initialise bypass work
	cs35lxx->cspl.bypass_wq = create_singlethread_workqueue("cs35lxx_bypass");
	INIT_WORK(&cs35lxx->cspl.bypass_work, cs35lxx_bypass);

	cs35lxx->cspl.dsp_recv_buffer = kmalloc(sizeof(struct cs35lxx_calib_cmd), GFP_KERNEL);
	if (!cs35lxx->cspl.dsp_recv_buffer) {
		dev_err(cs35lxx->dev, "Failed to allocate dsp recv buffer\n");
		return -ENOMEM;
	}
	cs35lxx->cspl.cspl_ready = false;

	dev_info(&i2c_client->dev, "Cirrus Logic CS35L%d, Revision: %02X\n",
		 cs35lxx->chip_version, reg_revid >> 8);

	cs35lxx->misc_dev.minor = MISC_DYNAMIC_MINOR;
	cs35lxx->misc_dev.name = "cs35lxx";
	cs35lxx->misc_dev.fops = &cs35lxx_fops;

	ret = misc_register(&cs35lxx->misc_dev);
	if (ret < 0) {
		dev_err(dev, "Register misc driver failed %d\n", ret);
		goto err;
	}

	dev_info(dev, "Register misc driver successful\n");

	vendor_info.vendor = VENDOR_ID_CIRRUS;
	vendor_info.chip_model = "cs35lxx";
	ret = smartpakit_set_info(&vendor_info);
	if (ret != 0) {
		dev_err(dev, "cs35lxx failed to smartpakit_set_info: %d\n", ret);
		goto err;
	}

	return 0;

err:
	gpiod_set_value_cansleep(cs35lxx->reset_gpio, 0);

err_disable_regs:
	regulator_bulk_disable(cs35lxx->num_supplies, cs35lxx->supplies);
	return ret;
}

static int cs35lxx_i2c_remove(struct i2c_client *client)
{
	struct cs35lxx_private *cs35lxx = i2c_get_clientdata(client);

	/* Reset interrupt masks for device removal */
	regmap_write(cs35lxx->regmap, CS35LXX_INT1_MASK,
		    CS35LXX_INT1_MASK_RESET);
	regmap_write(cs35lxx->regmap, CS35LXX_INT3_MASK,
		    CS35LXX_INT3_MASK_RESET);

	if (cs35lxx->reset_gpio)
		gpiod_set_value_cansleep(cs35lxx->reset_gpio, 0);

	if (cs35lxx->cspl.dsp_recv_buffer)
		kfree(cs35lxx->cspl.dsp_recv_buffer);

	regulator_bulk_disable(cs35lxx->num_supplies, cs35lxx->supplies);

	misc_deregister(&cs35lxx->misc_dev);

	return 0;
}
static const struct of_device_id cs35lxx_of_match[] = {
	{.compatible = "cirrus,cs35lxx"},
	{},
};
MODULE_DEVICE_TABLE(of, cs35lxx_of_match);

static const struct i2c_device_id cs35lxx_id[] = {
	{"cs35lxx", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs35lxx_id);

static struct i2c_driver cs35lxx_i2c_driver = {
	.driver = {
		.name = "cs35lxx",
		.of_match_table = cs35lxx_of_match,
	},
	.id_table = cs35lxx_id,
	.probe = cs35lxx_i2c_probe,
	.remove = cs35lxx_i2c_remove,
};

static int __init cs35lxx_i2c_init(void)
{
	int ret  = -1;
	pr_info("%s: cs35lxx driver init", __func__);
	ret = i2c_add_driver(&cs35lxx_i2c_driver);
	if (ret)
		pr_err("%s: cs35lxx driver add failed", __func__);

	return ret;
}

// late init call due to smartpakit requirement
late_initcall_sync(cs35lxx_i2c_init);
static void __exit cs35lxx_i2c_exit(void)
{
	i2c_del_driver(&cs35lxx_i2c_driver);
}
module_exit(cs35lxx_i2c_exit);

MODULE_DESCRIPTION("Misc CS35LXX driver");
MODULE_AUTHOR("Qi Zhou <qizhou@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
