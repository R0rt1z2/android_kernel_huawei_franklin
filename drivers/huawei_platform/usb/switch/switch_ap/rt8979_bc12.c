/*
 * rt8979_bc12.c
 *
 * driver file for switch chip
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#include "rt8979_bc12.h"
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/upmu_common.h>
#include "mtk_charger_intf.h"
#include <chipset_common/hwpower/common_module/power_i2c.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/usb/switch/switch_fsa9685.h>
#ifdef CONFIG_TCPC_HI6526
#include <huawei_platform/usb/hi6526_tcpc_ops.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#endif

#define HWLOG_TAG rt8979_bc12
HWLOG_REGIST();

#define RT8979_IRQ_ADC_CHG      7
#define RT8979_IRQ_RSV_ATTACH   6
#define RT8979_IRQ_VBUS_CHANGE  5
#define RT8979_IRQ_DEV_CHANGE   4
#define RT8979_IRQ_DETACH       1
#define RT8979_IRQ_ATTACH       0

struct rt8979_desc {
	const char *chg_name;
	bool auto_switch;
	bool scp;
	bool dcd;
	u32 dcd_timeout;
	u32 intb_watchdog;
	u32 ovp_sel;
};

enum rt8979_usbsw_state {
	RT8979_USBSW_CHG = 0,
	RT8979_USBSW_USB,
};

enum rt8979_reg_addr {
	RT8979_REG_DEVICE_ID = 0x01,
	RT8979_REG_MUIC_CTRL1,
	RT8979_REG_INT1,
	RT8979_REG_INT_MASK1,
	RT8979_REG_ADC,
	RT8979_REG_TIMING_SET1,
	RT8979_REG_DETACH_CTRL,
	RT8979_REG_DEVICE_TYPE1,
	RT8979_REG_DEVICE_TYPE2,
	RT8979_REG_DEVICE_TYPE3,
	RT8979_REG_MANUAL_SW1,
	RT8979_REG_MANUAL_SW2,
	RT8979_REG_TIMING_SET2,
	RT8979_REG_MUIC_CTRL2,
	RT8979_REG_DEVICE_TYPE4,
	RT8979_REG_MUIC_CTRL3,
	RT8979_REG_MUIC_CTRL4,
	RT8979_REG_MUIC_STATUS1,
	RT8979_REG_MUIC_STATUS2,
	RT8979_REG_RESET = 0x19,
	RT8979_REG_SCP_CMD = 0x44,
	RT8979_REG_PASS_CODE = 0xA0,
	RT8979_REG_TM_MUIC1,
	RT8979_REG_MUIC_OPT1 = 0xA4,
};

static const u8 rt8979_reg_addrs[] = {
	RT8979_REG_DEVICE_ID,
	RT8979_REG_MUIC_CTRL1,
	RT8979_REG_INT_MASK1,
	RT8979_REG_ADC,
	RT8979_REG_TIMING_SET1,
	RT8979_REG_DETACH_CTRL,
	RT8979_REG_DEVICE_TYPE1,
	RT8979_REG_DEVICE_TYPE2,
	RT8979_REG_DEVICE_TYPE3,
	RT8979_REG_MANUAL_SW1,
	RT8979_REG_MANUAL_SW2,
	RT8979_REG_TIMING_SET2,
	RT8979_REG_MUIC_CTRL2,
	RT8979_REG_DEVICE_TYPE4,
	RT8979_REG_MUIC_CTRL3,
	RT8979_REG_MUIC_CTRL4,
	RT8979_REG_MUIC_STATUS1,
	RT8979_REG_MUIC_STATUS2,
	RT8979_REG_RESET,
	RT8979_REG_MUIC_OPT1,
};

/* ovp voltage table: 6.2V 6.8V 11.5V 14.5V */
static const u32 ovp_sel_tbl[] = { 6200000, 6800000, 11500000, 14500000 };
/* dcd timeout table: 300ms 600ms 900ms 1200ms */
static const u32 dcd_timeout_tbl[] = { 300, 600, 900, 1200 };
/* intb watchdog table: 0ms 250ms 500ms 1000ms */
static const u32 intb_watchdog_tbl[] = { 0, 250, 500, 1000 };

static struct rt8979_desc rt8979_default_desc = {
	.chg_name = "usb_switch",
	.auto_switch = true,
	.scp = false,
	.dcd = true,
	.dcd_timeout = 600, /* 600ms */
	.intb_watchdog = 0, /* disabled */
	.ovp_sel = 6800000, /* 6.8V */
};

static inline int __rt8979_i2c_read_byte(struct fsa9685_device_info *chip, u8 cmd, u8 *data)
{
	int ret;
	u8 regval = 0;

	ret = i2c_smbus_read_i2c_block_data(chip->client, cmd, 1, &regval);
	if (ret < 0) {
		hwlog_info("%s reg0x%02X fail %d\n", __func__, cmd, ret);
		return ret;
	}

	hwlog_info("%s reg0x%02X = 0x%02X\n", __func__, cmd, regval);
	*data = regval & 0xFF;
	return 0;
}

static int rt8979_i2c_read_byte(struct fsa9685_device_info *chip, u8 cmd, u8 *data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt8979_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __rt8979_i2c_write_byte(struct fsa9685_device_info *chip, u8 cmd, u8 data)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(chip->client, cmd, 1, &data);
	if (ret < 0)
		hwlog_info("%s reg0x%02X = 0x%02X fail %d\n", __func__, cmd, data, ret);
	else
		hwlog_info("%s reg0x%02X = 0x%02X\n", __func__, cmd, data);

	return ret;
}

static int rt8979_i2c_write_byte(struct fsa9685_device_info *chip, u8 cmd, u8 data)
{
	int ret;

	mutex_lock(&chip->io_lock);
	ret = __rt8979_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static int rt8979_i2c_update_bits(struct fsa9685_device_info *chip, u8 cmd, u8 data, u8 mask)
{
	int ret;
	u8 regval = 0;

	mutex_lock(&chip->io_lock);
	ret = __rt8979_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __rt8979_i2c_write_byte(chip, cmd, regval);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt8979_set_bit(struct fsa9685_device_info *chip, u8 cmd, u8 mask)
{
	return rt8979_i2c_update_bits(chip, cmd, mask, mask);
}

static inline int rt8979_clr_bit(struct fsa9685_device_info *chip, u8 cmd, u8 mask)
{
	return rt8979_i2c_update_bits(chip, cmd, 0x00, mask);
}

static inline u8 rt8979_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size, u32 target)
{
	u32 i;

	if (target <= tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 rt8979_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val = min + (regval * step);

	if (val > max)
		val = max;

	return val;
}

static int rt8979_enable_auto_switch(struct fsa9685_device_info *chip, bool en)
{
	hwlog_info("%s en = %d\n", __func__, en);
	return (en ? rt8979_set_bit : rt8979_clr_bit)
		(chip, RT8979_REG_MUIC_CTRL1, RT8979_SW_OPEN_MASK);
}

static int rt8979_enable_scp(struct fsa9685_device_info *chip, bool en)
{
	int ret;

	hwlog_info("%s en = %d\n", __func__, en);
	if (en) {
		/* set REG_SCP 0x01 as default value */
		ret = rt8979_i2c_write_byte(chip, RT8979_REG_SCP_CMD, 0x01);
		if (ret < 0)
			return ret;
	}
	return (en ? rt8979_set_bit : rt8979_clr_bit)
		(chip, RT8979_REG_MUIC_CTRL2, RT8979_SCP_EN_MASK);
}

static int rt8979_enable_dcd(struct fsa9685_device_info *chip, bool en)
{
	hwlog_info("%s en = %d\n", __func__, en);
	return (en ? rt8979_set_bit : rt8979_clr_bit)
		(chip, RT8979_REG_MUIC_CTRL2, RT8979_DCD_TIMEOUT_EN_MASK);
}

static int rt8979_set_dcd_timeout(struct fsa9685_device_info *chip, u32 tout)
{
	u8 regval;

	regval = rt8979_closest_reg_via_tbl(dcd_timeout_tbl,
		ARRAY_SIZE(dcd_timeout_tbl), tout);

	hwlog_info("%s tout = %d 0x%02X\n", __func__, tout, regval);

	return rt8979_i2c_update_bits(chip, RT8979_REG_TIMING_SET2,
		regval << RT8979_DCD_TIMEOUT_SHIFT, RT8979_DCD_TIMEOUT_MASK);
}


static int rt8979_set_intb_watchdog(struct fsa9685_device_info *chip, u32 wdt)
{
	u8 regval;

	regval = rt8979_closest_reg_via_tbl(intb_watchdog_tbl,
		ARRAY_SIZE(intb_watchdog_tbl), wdt);

	hwlog_info("%s wdt = %d 0x%02X\n", __func__, wdt, regval);

	return rt8979_i2c_update_bits(chip, RT8979_REG_TIMING_SET2,
		regval << RT8979_INTB_WATCHDOG_SHIFT, RT8979_INTB_WATCHDOG_MASK);
}

static int rt8979_set_ovp_sel(struct fsa9685_device_info *chip, u32 ovp)
{
	u8 regval;

	regval = rt8979_closest_reg_via_tbl(ovp_sel_tbl, ARRAY_SIZE(ovp_sel_tbl), ovp);

	hwlog_info("%s ovp = %d 0x%02X\n", __func__, ovp, regval);

	return rt8979_i2c_update_bits(chip, RT8979_REG_MUIC_CTRL3,
		regval << RT8979_OVP_SEL_SHIFT, RT8979_OVP_SEL_MASK);
}

static int __rt8979_enable_bc12(struct fsa9685_device_info *chip, bool en)
{
	hwlog_info("%s en = %d\n", __func__, en);
	return (en ? rt8979_set_bit : rt8979_clr_bit)
		(chip, RT8979_REG_MUIC_OPT1, RT8979_USBCHGEN_MASK);
}

#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
static bool rt8979_is_vbus_gd(struct fsa9685_device_info *chip)
{
	int ret;
	u8 regval = 0;
	bool vbus_gd = false;

	ret = rt8979_i2c_read_byte(chip, RT8979_REG_MUIC_STATUS1, &regval);
	if (ret < 0)
		hwlog_info("%s fail %d\n", __func__, ret);
	else
		vbus_gd = !(regval & RT8979_VBUS_UVLO_MASK) &&
			!(regval & RT8979_VBUS_OVP_MASK);

	hwlog_info("%s vbus_gd = %d\n", __func__, vbus_gd);

	return vbus_gd;
}

void rt8979_set_usbsw_state(int state)
{
	hwlog_info("%s state = %d\n", __func__, state);

	if (state == RT8979_USBSW_CHG)
		Charger_Detect_Init();
	else
		Charger_Detect_Release();
}

static inline int rt8979_toggle_bc12(struct fsa9685_device_info *chip)
{
	int ret;
	u8 regval = 0;
	u8 bc12_dis[2] = {0};
	u8 bc12_en[2] = {0};
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = bc12_dis,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = bc12_en,
		},
	};

	mutex_lock(&chip->io_lock);

	ret = i2c_smbus_read_i2c_block_data(client, RT8979_REG_MUIC_OPT1, 1, &regval);
	if (ret < 0) {
		hwlog_err("%s read reg fail %d\n", __func__, ret);
		goto out;
	}

	ret = i2c_smbus_write_byte_data(client, RT8979_REG_PASS_CODE, 0x38);
	if (ret < 0) {
		hwlog_err("%s enter testmode fail %d\n", __func__, ret);
		goto out;
	}

	ret = i2c_smbus_write_byte_data(client, RT8979_REG_TM_MUIC1, 0x10);
	if (ret < 0) {
		hwlog_err("%s enable manual DCD fail %d\n", __func__, ret);
		goto out;
	}

	/* bc12 disable and then enable */
	bc12_dis[0] = bc12_en[0] = RT8979_REG_MUIC_OPT1;
	bc12_dis[1] = regval & ~RT8979_USBCHGEN_MASK;
	bc12_en[1] = regval | RT8979_USBCHGEN_MASK;
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		hwlog_err("%s bc12 dis/en fail %d\n", __func__, ret);
		goto out;
	}

	usleep_range(35000, 36000); /* delay 35ms for bc12 operate */
out:
	ret = i2c_smbus_write_byte_data(client, RT8979_REG_TM_MUIC1, 0x00);
	if (ret < 0)
		hwlog_err("%s disable manual DCD fail %d\n", __func__, ret);
	ret = i2c_smbus_write_byte_data(client, RT8979_REG_PASS_CODE, 0x00);
	if (ret < 0)
		hwlog_err("%s leave testmode fail %d\n", __func__, ret);

	mutex_unlock(&chip->io_lock);
	return (ret < 0) ? ret : 0;
}

static int rt8979_bc12_en_kthread(void *data)
{
	int ret;
	int i;
	int en;
	struct fsa9685_device_info *chip = data;
	const int max_wait_cnt = 200; /* wait DCP ready */

	hwlog_info("%s\n", __func__);

	if (!chip)
		return -1;
wait:
	wait_event(chip->bc12_en_req, atomic_read(&chip->bc12_en_req_cnt) > 0 ||
		kthread_should_stop());
	if (atomic_read(&chip->bc12_en_req_cnt) <= 0 && kthread_should_stop()) {
		hwlog_info("%s bye bye\n", __func__);
		return 0;
	}
	atomic_dec(&chip->bc12_en_req_cnt);

	mutex_lock(&chip->bc12_en_lock);
	en = chip->bc12_en_buf[chip->bc12_en_buf_idx];
	chip->bc12_en_buf[chip->bc12_en_buf_idx] = -1;
	if (en == -1) {
		chip->bc12_en_buf_idx = 1 - chip->bc12_en_buf_idx;
		en = chip->bc12_en_buf[chip->bc12_en_buf_idx];
		chip->bc12_en_buf[chip->bc12_en_buf_idx] = -1;
	}
	mutex_unlock(&chip->bc12_en_lock);

	hwlog_info("%s en = %d\n", __func__, en);
	if (en == -1)
		goto wait;

	__pm_stay_awake(chip->bc12_en_ws);

	if (en) {
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt; i++) {
			if (is_usb_rdy())
				break;
			hwlog_info("%s CDP block\n", __func__);
			if (!atomic_read(&chip->chg_type_det)) {
				hwlog_info("%s bc12 dis\n", __func__);
				goto relax_and_wait;
			}
			usleep_range(100000, 110000); /* wait 100ms each cycle */
		}
		if (i == max_wait_cnt)
			hwlog_info("%s CDP timeout\n", __func__);
		else
			hwlog_info("%s CDP free\n", __func__);
	}
	rt8979_set_usbsw_state(en ? RT8979_USBSW_CHG : RT8979_USBSW_USB);
	if (en)
		ret = rt8979_toggle_bc12(chip);
	else
		ret = __rt8979_enable_bc12(chip, en);
	if (ret < 0)
		hwlog_info("%s en = %d fail %d\n", __func__, en, ret);
relax_and_wait:
	__pm_relax(chip->bc12_en_ws);
	goto wait;

	return 0;
}

static void rt8979_enable_bc12(struct fsa9685_device_info *chip, bool en)
{
	hwlog_info("%s en = %d\n", __func__, en);

	mutex_lock(&chip->bc12_en_lock);
	chip->bc12_en_buf[chip->bc12_en_buf_idx] = en;
	chip->bc12_en_buf_idx = 1 - chip->bc12_en_buf_idx;
	mutex_unlock(&chip->bc12_en_lock);
	atomic_inc(&chip->bc12_en_req_cnt);
	wake_up(&chip->bc12_en_req);
}

#ifdef CONFIG_TCPC_CLASS
static int rt8979_bc12_pre_process(struct fsa9685_device_info *chip)
{
	if (atomic_read(&chip->chg_type_det))
		rt8979_enable_bc12(chip, true);

	return 0;
}
#endif /* CONFIG_TCPC_CLASS */

static void rt8979_inform_psy_dwork_handler(struct work_struct *work)
{
	int ret;
	union power_supply_propval propval = { .intval = 0 };
	struct fsa9685_device_info *chip = container_of(work, struct fsa9685_device_info, psy_dwork.work);
	bool attach = false;
	enum charger_type chg_type = CHARGER_UNKNOWN;

	mutex_lock(&chip->bc12_lock);
	attach = chip->attach;
	chg_type = chip->chg_type;
	mutex_unlock(&chip->bc12_lock);

	hwlog_info("%s attach = %d, type = %d\n", __func__,
		attach, chg_type);

	/* Get chg type det power supply */
	if (!chip->psy)
		chip->psy = power_supply_get_by_name("charger");
	if (!chip->psy) {
		hwlog_info("%s get power supply fail\n", __func__);
		mod_delayed_work(system_wq, &chip->psy_dwork, msecs_to_jiffies(1000));
		return;
	}

	propval.intval = attach;
	ret = power_supply_set_property(chip->psy, POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		hwlog_info("%s psy online fail %d\n", __func__, ret);

	propval.intval = chg_type;
	ret = power_supply_set_property(chip->psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		hwlog_info("%s psy type fail %d\n", __func__, ret);
}

static inline bool rt8979_tcpc_nonstd_retry_flag(void)
{
	bool nonstd_retry_flag = true;

#ifdef CONFIG_TCPC_HI6526
	/*
	 * if hi6526 PD supported and pd_finish_flag set,
	 * then should skip NONSTANDARD_CHARGER retry
	 */
	if (hi6526_get_tcpc_dev())
		nonstd_retry_flag = (pd_dpm_get_pd_finish_flag() == false);
	hwlog_info("%s nonstd_retry_flag %d\n", __func__, nonstd_retry_flag);
#endif
	return nonstd_retry_flag;
}

static int rt8979_bc12_post_process(struct fsa9685_device_info *chip)
{
	int ret = 0;
	bool attach = false;
	bool inform_psy = true;
	u8 regval = 0;

	attach = atomic_read(&chip->chg_type_det);
	if (chip->attach == attach && !chip->sdp_retried &&
		!chip->nstd_retrying) {
		hwlog_info("%s attach %d is the same\n", __func__, attach);
		inform_psy = !attach;
		goto out;
	}
	chip->attach = attach;
	hwlog_info("%s attach = %d\n", __func__, attach);

	if (!attach)
		goto out_novbus;

	ret = rt8979_i2c_read_byte(chip, RT8979_REG_MUIC_STATUS2, &regval);
	if (ret < 0)
		goto out_novbus;
	else
		chip->us = (regval & RT8979_USB_STATUS_MASK) >>
			RT8979_USB_STATUS_SHIFT;

	switch (chip->us) {
	case RT8979_USB_STATUS_NOVBUS:
		goto out_novbus;
	case RT8979_USB_STATUS_UNDER_GOING:
		return 0;
	case RT8979_USB_STATUS_SDP:
		chip->chg_type = STANDARD_HOST;
		if (chip->sdp_retried < 2) {
			chip->sdp_retried++;
			rt8979_enable_bc12(chip, true);
			return 0;
		}
		break;
	case RT8979_USB_STATUS_DCP:
		chip->chg_type = STANDARD_CHARGER;
		rt8979_dcp_accp_init();
		break;
	case RT8979_USB_STATUS_CDP:
		chip->chg_type = CHARGING_HOST;
		break;
	case RT8979_USB_STATUS_SDP_NSTD:
	default:
		chip->chg_type = NONSTANDARD_CHARGER;
		break;
	}
	ret = rt8979_i2c_read_byte(chip, RT8979_REG_MUIC_STATUS1, &regval);
	if (ret < 0)
		goto out_novbus;
	if (regval & RT8979_DCDT_MASK)
		chip->chg_type = NONSTANDARD_CHARGER;
	if (chip->chg_type == NONSTANDARD_CHARGER &&
		rt8979_tcpc_nonstd_retry_flag()) {
		inform_psy = !chip->nstd_retrying;
		chip->nstd_retrying = true;
		rt8979_set_dcd_timeout(chip, 1200);
		rt8979_enable_bc12(chip, true);
		goto out_nstd_retrying;
	}
	goto out;
out_novbus:
	chip->us = RT8979_USB_STATUS_NOVBUS;
	chip->chg_type = CHARGER_UNKNOWN;
out:
	chip->sdp_retried = 0;
	chip->nstd_retrying = false;
	rt8979_set_dcd_timeout(chip, chip->desc->dcd_timeout);
	if ((chip->chg_type != STANDARD_CHARGER) || (!chip->fcp_support && !chip->scp_support))
		rt8979_enable_bc12(chip, false); /* release usb_phy to soc */
out_nstd_retrying:
	if (inform_psy)
		schedule_delayed_work(&chip->psy_dwork, 0);

	return ret;
}
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */

int rt8979_enable_chg_type_det(struct fsa9685_device_info *chip, bool en)
{
	int ret;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
#ifdef CONFIG_TCPC_CLASS
	hwlog_info("%s en = %d\n", __func__, en);

	mutex_lock(&chip->bc12_lock);
	atomic_set(&chip->chg_type_det, en);
	ret = (en ? rt8979_bc12_pre_process : rt8979_bc12_post_process)(chip);
	mutex_unlock(&chip->bc12_lock);
	if (ret < 0)
		hwlog_info("%s en bc12 fail %d\n", __func__, ret);
#endif /* CONFIG_TCPC_CLASS */
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	return ret;
}

static int rt8979_dump_registers(struct fsa9685_device_info *chip)
{
	int ret;
	int i;
	u8 regval = 0;

	for (i = 0; i < ARRAY_SIZE(rt8979_reg_addrs); i++) {
		ret = rt8979_i2c_read_byte(chip, rt8979_reg_addrs[i], &regval);
		if (ret < 0)
			continue;
		hwlog_info("%s reg0x%02X = 0x%02X\n", __func__,
			rt8979_reg_addrs[i], regval);
	}

	return 0;
}

static int rt8979_adc_chg_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);
	return 0;
}

static int rt8979_reserved_attach_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);
	return 0;
}

static int rt8979_vbus_change_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);

#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	if (rt8979_is_vbus_gd(chip)) {
		mutex_lock(&chip->bc12_lock);
		rt8979_bc12_post_process(chip);
		hwlog_info("%s usb_status = %d\n", __func__, chip->us);
		mutex_unlock(&chip->bc12_lock);
	}
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */

	return 0;
}

static int rt8979_device_change_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);
	return 0;
}

static int rt8979_detach_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);
	return 0;
}

static int rt8979_attach_irq_handler(struct fsa9685_device_info *chip)
{
	hwlog_info("%s\n", __func__);
	return 0;
}

struct irq_mapping_tbl {
	const char *name;
	int (*hdlr)(struct fsa9685_device_info *chip);
	u8 num;
};

#define rt8979_irq_mapping(_name, _num) \
	{.name = #_name, .hdlr = rt8979_##_name##_irq_handler, .num = _num}

static const struct irq_mapping_tbl rt8979_irq_mapping_tbl[] = {
	rt8979_irq_mapping(adc_chg, RT8979_IRQ_ADC_CHG),
	rt8979_irq_mapping(reserved_attach, RT8979_IRQ_RSV_ATTACH),
	rt8979_irq_mapping(vbus_change, RT8979_IRQ_VBUS_CHANGE),
	rt8979_irq_mapping(device_change, RT8979_IRQ_DEV_CHANGE),
	rt8979_irq_mapping(detach, RT8979_IRQ_DETACH),
	rt8979_irq_mapping(attach, RT8979_IRQ_ATTACH),
};

static irqreturn_t rt8979_irq_handler(int irq, void *data)
{
	int ret;
	int i;
	u8 int_data = 0;
	u8 mask_data = 0;
	struct fsa9685_device_info *chip = data;

	ret = rt8979_i2c_read_byte(chip, RT8979_REG_INT1, &int_data);
	if (ret < 0)
		goto out;
	ret = rt8979_i2c_read_byte(chip, RT8979_REG_INT_MASK1, &mask_data);
	if (ret < 0)
		goto out;

	int_data &= ~mask_data;

	for (i = 0; i < ARRAY_SIZE(rt8979_irq_mapping_tbl); i++) {
		if (int_data & BIT(rt8979_irq_mapping_tbl[i].num))
			rt8979_irq_mapping_tbl[i].hdlr(chip);
	}
out:
	return IRQ_HANDLED;
}

int rt8979_reset(struct fsa9685_device_info *chip)
{
	int ret;

	if (!chip)
		return -1;

	hwlog_info("%s\n", __func__);
	mutex_lock(&chip->io_lock);
	ret = __rt8979_i2c_write_byte(chip, RT8979_REG_RESET, RT8979_RESET_MASK);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt8979_get_irq_number(struct fsa9685_device_info *chip, const char *name)
{
	int i;

	if (!name) {
		hwlog_info("%s null name\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(rt8979_irq_mapping_tbl); i++) {
		if (!strcmp(name, rt8979_irq_mapping_tbl[i].name))
			return rt8979_irq_mapping_tbl[i].num;
	}

	return -EINVAL;
}

static inline const char *rt8979_get_irq_name(int irqnum)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rt8979_irq_mapping_tbl); i++) {
		if (rt8979_irq_mapping_tbl[i].num == irqnum)
			return rt8979_irq_mapping_tbl[i].name;
	}

	return "not found";
}

static inline void rt8979_irq_unmask(struct fsa9685_device_info *chip, int irqnum)
{
	hwlog_info("%s irq %d, %s\n", __func__, irqnum,
		rt8979_get_irq_name(irqnum));
	chip->irq_mask &= ~BIT(irqnum);
}

static int rt8979_parse_dt(struct fsa9685_device_info *chip)
{
	int ret;
	int irqcnt;
	int irqnum;
	struct device_node *np = chip->dev->of_node;
	struct rt8979_desc *desc = NULL;
	const char *name = NULL;

	hwlog_info("%s\n", __func__);

	chip->desc = &rt8979_default_desc;

	if (!np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}

	desc = devm_kmemdup(chip->dev, &rt8979_default_desc,
		sizeof(rt8979_default_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	chip->desc = desc;

	ret = of_get_named_gpio(np, "fairchild_fsa9685,gpio-intb", 0);
	if (ret < 0)
		hwlog_info("%s no rt,intr_gpio %d\n", __func__, ret);
	else
		chip->intr_gpio = ret;

	hwlog_info("%s intr_gpio = %u\n", __func__, chip->intr_gpio);

	desc->auto_switch = of_property_read_bool(np, "auto_switch");
	desc->scp = of_property_read_bool(np, "scp");
	desc->dcd = of_property_read_bool(np, "dcd");

	ret = of_property_read_u32(np, "dcd_timeout", &desc->dcd_timeout);
	if (ret < 0)
		hwlog_info("%s no dcd_timeout %d\n", __func__, ret);

	ret = of_property_read_u32(np, "intb_watchdog", &desc->intb_watchdog);
	if (ret < 0)
		hwlog_info("%s no intb_watchdog %d\n", __func__, ret);

	ret = of_property_read_u32(np, "ovp_sel", &desc->ovp_sel);
	if (ret < 0)
		hwlog_info("%s no ovp_sel %d\n", __func__, ret);

	chip->irq_mask = 0xFF;
	for (irqcnt = 0; ; irqcnt++) {
		ret = of_property_read_string_index(np, "interrupt-names", irqcnt, &name);
		if (ret < 0)
			break;
		irqnum = rt8979_get_irq_number(chip, name);
		if (irqnum >= 0)
			rt8979_irq_unmask(chip, irqnum);
	}

	return 0;
}

static int rt8979_init_setting(struct fsa9685_device_info *chip)
{
	int ret;
	struct rt8979_desc *desc = chip->desc;

	hwlog_info("%s\n", __func__);
	if (!desc)
		return -1;

	ret = rt8979_enable_auto_switch(chip, desc->auto_switch);
	if (ret < 0)
		hwlog_info("%s enable auto_switch fail %d\n", __func__, ret);

	ret = rt8979_enable_scp(chip, desc->scp);
	if (ret < 0)
		hwlog_info("%s enable scp fail %d\n", __func__, ret);

	ret = rt8979_enable_dcd(chip, desc->dcd);
	if (ret < 0)
		hwlog_info("%s enable dcd fail %d\n", __func__, ret);

	ret = rt8979_set_dcd_timeout(chip, desc->dcd_timeout);
	if (ret < 0)
		hwlog_info("%s set dcd_timeout fail %d\n", __func__, ret);

	ret = rt8979_set_intb_watchdog(chip, desc->intb_watchdog);
	if (ret < 0)
		hwlog_info("%s set intb_watchdog fail %d\n", __func__, ret);

	ret = rt8979_set_ovp_sel(chip, desc->ovp_sel);
	if (ret < 0)
		hwlog_info("%s set ovp_sel fail %d\n", __func__, ret);

	ret = __rt8979_enable_bc12(chip, false);
	if (ret < 0)
		hwlog_info("%s dis bc12 fail %d\n", __func__, ret);

	return 0;
}

static int rt8979_register_irq(struct fsa9685_device_info *chip)
{
	int ret;

	hwlog_info("%s\n", __func__);

	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_DIR_IN,
		devm_kasprintf(chip->dev, GFP_KERNEL,
		"rt8979_intr_gpio.%s", dev_name(chip->dev)));
	if (ret < 0) {
		hwlog_info("%s gpio request fail %d\n",
			__func__, ret);
		return ret;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		hwlog_info("%s gpio2irq fail %d\n",
			__func__, chip->irq);
		return chip->irq;
	}
	hwlog_info("%s irq = %d\n", __func__, chip->irq);

	/* Request threaded IRQ */
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
		rt8979_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		devm_kasprintf(chip->dev, GFP_KERNEL, "rt8979_irq.%s",
		dev_name(chip->dev)), chip);
	if (ret < 0) {
		hwlog_info("%s request threaded irq fail %d\n",
			__func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);

	return ret;
}

int rt8979_init_irq(struct fsa9685_device_info *chip)
{
	int ret;

	hwlog_info("%s\n", __func__);

	ret = rt8979_i2c_write_byte(chip, RT8979_REG_INT_MASK1,
		chip->irq_mask);
	if (ret < 0)
		return ret;
	return rt8979_clr_bit(chip, RT8979_REG_MUIC_CTRL1,
		RT8979_INT_MASK_MASK);
}

static void rt8979_mutex_init(struct fsa9685_device_info *chip)
{
	mutex_init(&chip->io_lock);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	mutex_init(&chip->bc12_lock);
	mutex_init(&chip->bc12_en_lock);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	mutex_init(&chip->hidden_mode_lock);
}

static void rt8979_mutex_destroy(struct fsa9685_device_info *chip)
{
	mutex_destroy(&chip->io_lock);
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	mutex_destroy(&chip->bc12_lock);
	mutex_destroy(&chip->bc12_en_lock);
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	mutex_destroy(&chip->hidden_mode_lock);
}

static void rt8979_info_init(struct fsa9685_device_info *chip)
{
	chip->hidden_mode_cnt = 0;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	INIT_DELAYED_WORK(&chip->psy_dwork, rt8979_inform_psy_dwork_handler);
	atomic_set(&chip->chg_type_det, 0);
	chip->attach = false;
	chip->us = RT8979_USB_STATUS_NOVBUS;
	chip->chg_type = CHARGER_UNKNOWN;
	chip->sdp_retried = 0;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
}

void rt8979_re_init(struct fsa9685_device_info *chip)
{
	(void)rt8979_init_setting(chip);
	(void)rt8979_init_irq(chip);
}

int rt8979_init_early(struct fsa9685_device_info *chip)
{
	int ret;

	if (!chip)
		return -1;

	hwlog_info("%s +\n", __func__);

	rt8979_mutex_init(chip);
	rt8979_info_init(chip);

	ret = rt8979_reset(chip);
	if (ret < 0)
		hwlog_info("%s reset fail %d\n", __func__, ret);

	ret = rt8979_parse_dt(chip);
	if (ret < 0)
		hwlog_info("%s parse dt fail %d\n", __func__, ret);

	ret = rt8979_init_setting(chip);
	if (ret < 0) {
		hwlog_info("%s init fail %d\n", __func__, ret);
		goto init_fail;
	}

	hwlog_info("%s -\n", __func__);
	return 0;

init_fail:
	rt8979_mutex_destroy(chip);
	return ret;
}

int rt8979_init_later(struct fsa9685_device_info *chip)
{
	int ret = 0;

	hwlog_info("%s +\n", __func__);

#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	chip->bc12_en_ws = wakeup_source_register(devm_kasprintf(chip->dev,
		GFP_KERNEL, "rt8979_bc12_en_ws.%s", dev_name(chip->dev)));
	if (!(chip->bc12_en_ws)) {
		hwlog_err("%s wake source register fail\n", __func__);
		goto kthread_run_fail;
	}
	chip->bc12_en_buf[0] = chip->bc12_en_buf[1] = -1;
	chip->bc12_en_buf_idx = 0;
	atomic_set(&chip->bc12_en_req_cnt, 0);
	init_waitqueue_head(&chip->bc12_en_req);
	chip->bc12_en_kthread = kthread_run(rt8979_bc12_en_kthread, chip, "%s",
		devm_kasprintf(chip->dev, GFP_KERNEL,
		"rt8979_bc12_en_kthread.%s", dev_name(chip->dev)));
	if (IS_ERR_OR_NULL(chip->bc12_en_kthread)) {
		ret = PTR_ERR(chip->bc12_en_kthread);
		hwlog_err("%s kthread run fail %d\n", __func__, ret);
		goto kthread_run_fail;
	}
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */

	ret = rt8979_register_irq(chip);
	if (ret < 0) {
		hwlog_err("%s register irq fail %d\n", __func__, ret);
		goto irq_fail;
	}

	ret = rt8979_init_irq(chip);
	if (ret < 0) {
		hwlog_err("%s init irq fail %d\n", __func__, ret);
		goto irq_fail;
	}

	rt8979_dump_registers(chip);
	hwlog_info("%s -\n", __func__);
	return 0;

irq_fail:
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	cancel_delayed_work_sync(&chip->psy_dwork);
	if (chip->psy)
		power_supply_put(chip->psy);

	kthread_stop(chip->bc12_en_kthread);
	wakeup_source_unregister(chip->bc12_en_ws);
kthread_run_fail:
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	rt8979_mutex_destroy(chip);
	return ret;
}

