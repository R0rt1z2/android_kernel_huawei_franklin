/*
 * tcpc_hi6526.c
 *
 * Hi6526 TCPC Driver.
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/sched/clock.h>
#include <linux/sched/rt.h>
#include <linux/version.h>
#include <uapi/linux/sched/types.h>

#include <securec.h>

#include "include/pd_dbg_info.h"
#include "include/tcpci.h"
#include "hi6526.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "[tcpc_hi6526]"

#define TCPC_IRQ_SCHED_DELAY_TIME 10000

struct hi6526_tcpc_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct hi6526_tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_delayed_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source irq_wakelock;

	atomic_t poll_count;
	struct delayed_work	poll_work;

	unsigned int chip_version;

	int irq_gpio;
	int irq;
	int low_power_mode;

	uint8_t pd_dbg_rdata_cfg; /* for pd fsm state */
	uint8_t vbus_detect; /* record vbus detect state */

	int otg_vbst_ori; /* OTG Power Source */
	int poll_cc_status;
	bool fr_swap_status;
#ifdef CONFIG_WORKAROUND_FF_CONER_CHIP_BIST_MODE_ERROR
	struct delayed_work bist_mode_2_stop_work;
#endif
};

static int hi6526_tcpc_set_cc(struct hi6526_tcpc_device *tcpc, int pull);

static int hi6526_tcpc_read8(struct hi6526_tcpc_device *tcpc, u8 reg)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	return hi6526_tcpc_i2c_read8(chip->client, reg);
}

static int hi6526_tcpc_write8(struct hi6526_tcpc_device *tcpc, u8 reg, u8 data)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	return hi6526_tcpc_i2c_write8(chip->client, reg, data);
}

static inline void hi6526_tcpc_command(struct hi6526_tcpc_device *tcpc, uint8_t cmd)
{
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_COMMAND, cmd);
}

static void hi6526_tcpc_da_vbus_en_status(struct hi6526_tcpc_device *tcpc)
{
	int ret;
	int da_vbus_en;
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	if (chip->pd_dbg_rdata_cfg != PD_DBG_RDATA_VBUS_STATUS) {
		(void)hi6526_tcpc_i2c_write8(chip->client, REG_PD_DBG_RDATA_CFG,
			PD_DBG_RDATA_VBUS_STATUS);
		chip->pd_dbg_rdata_cfg = PD_DBG_RDATA_VBUS_STATUS;
	}

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_DBG_RDATA);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return;
	}

	da_vbus_en = (u32)ret & DA_VBUS_5V_EN_STATUS;
	D("da_vbus_en: 0x%x--%s\n", (u32)ret, da_vbus_en ? "Y" : "N");
}

#ifdef CONFIG_WORKAROUND_FF_CONER_CHIP_BIST_MODE_ERROR
#define BIST_MODE_2_STOP_TIME_MS 45
static void bist_mode_2_stop_work_fn(struct work_struct *work)
{
	struct hi6526_tcpc_chip *chip = container_of(work,
			struct hi6526_tcpc_chip, bist_mode_2_stop_work.work);

	(void)hi6526_tcpc_i2c_write8(chip->client, REG_PD_VDM_ENABLE, 0);
	(void)hi6526_tcpc_i2c_write8(chip->client, REG_PD_VDM_ENABLE, 1);
}
#endif

static void hi6526_tcpc_enable_vbus(struct i2c_client *client)
{
	uint8_t reg;
	int ret;

	/* dis force disch */
	ret = hi6526_tcpc_i2c_read8(client, TCPC_V10_REG_POWER_CTRL);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return;
	}

	reg = (uint8_t)ret;
	reg &= ~(TCPC_V10_REG_POWER_CTRL_FORCE_DISCHARGE_DISCONNECT);
	(void)hi6526_tcpc_i2c_write8(client, TCPC_V10_REG_POWER_CTRL, reg);
}

static void hi6526_tcpc_disable_vbus(struct i2c_client *client)
{
	uint8_t reg;
	int ret;

	/* en force disch */
	ret = hi6526_tcpc_i2c_read8(client, TCPC_V10_REG_POWER_CTRL);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return;
	}

	reg = (uint8_t)ret;
	reg |= (TCPC_V10_REG_POWER_CTRL_FORCE_DISCHARGE_DISCONNECT);
	(void)hi6526_tcpc_i2c_write8(client, TCPC_V10_REG_POWER_CTRL, reg);
	mdelay(50);
}

static int hi6526_tcpc_source_vbus(struct hi6526_tcpc_device *tcpc, uint8_t type,
	int mv, int ma)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	D("type: 0x%x\n", type);
	if (type == TCP_VBUS_CTRL_REMOVE) {
		hi6526_tcpc_disable_vbus(chip->client);
		return 0;
	}

	if (type == TCP_VBUS_CTRL_TYPEC) {
		if (mv != TCPC_VBUS_SOURCE_0V)
			hi6526_tcpc_enable_vbus(chip->client);
		else
			hi6526_tcpc_disable_vbus(chip->client);

		return 0;
	}

	switch (type) {
	case TCP_VBUS_CTRL_HRESET:
	case TCP_VBUS_CTRL_PR_SWAP:
	case TCP_VBUS_CTRL_REQUEST:
	case TCP_VBUS_CTRL_STANDBY:
	case TCP_VBUS_CTRL_STANDBY_UP:
	case TCP_VBUS_CTRL_STANDBY_DOWN:
		if (mv != TCPC_VBUS_SOURCE_0V) {
			D("PD source vbus %dmV %dmA\n", mv, ma);
			hi6526_tcpc_enable_vbus(chip->client);
		} else {
			hi6526_tcpc_disable_vbus(chip->client);
		}
		break;

	case TCP_VBUS_CTRL_PD_HRESET:
	case TCP_VBUS_CTRL_PD_PR_SWAP:
	case TCP_VBUS_CTRL_PD_REQUEST:
	case TCP_VBUS_CTRL_PD_STANDBY:
	case TCP_VBUS_CTRL_PD_STANDBY_UP:
	case TCP_VBUS_CTRL_PD_STANDBY_DOWN:
		if (mv != TCPC_VBUS_SOURCE_0V) {
			D("PD_DETECT source vbus %dmV %dmA\n", mv, ma);
			hi6526_tcpc_enable_vbus(chip->client);
		} else {
			hi6526_tcpc_disable_vbus(chip->client);
		}
		break;

	default:
		break;
	}

	return 0;
}

static void hi6526_tcpc_set_vbus_detect(struct hi6526_tcpc_device *tcpc, bool enable)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	uint8_t reg;
	int ret;

	if (chip->vbus_detect == enable) {
		D("vbus_detect already %s\n", enable ? "enabled" : "disabled");
		return;
	}

	chip->vbus_detect = enable;
	ret = hi6526_tcpc_i2c_read8(chip->client,
			TCPC_V10_REG_POWER_CTRL);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return;
	}
	reg = (uint8_t)ret;

	if (enable) {
		hi6526_tcpc_command(tcpc, TCPM_CMD_ENABLE_VBUS_DETECT);

		/* enable AutoDischargeDisconnect */
		reg |= TCPC_V10_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT;
	} else {
		hi6526_tcpc_command(tcpc, TCPM_CMD_DISABLE_VBUS_DETECT);

		/* disable AutoDischargeDisconnect */
		reg &= ~TCPC_V10_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT;
	}

	(void)hi6526_tcpc_i2c_write8(chip->client, TCPC_V10_REG_POWER_CTRL, reg);
}

static int __hi6526_tcpc_pd_fsm_reset(struct i2c_client *client)
{
	uint8_t reg;
	int ret;

	ret = hi6526_tcpc_i2c_read8(client, REG_PD_VDM_CFG_1);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return ret;
	}

	reg = (uint8_t)ret;
	reg |= PD_TC_ALL_RESET;
	(void)hi6526_tcpc_i2c_write8(client, REG_PD_VDM_CFG_1, reg);

	ret = hi6526_tcpc_i2c_read8(client, REG_PD_VDM_CFG_1);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error return %d\n", ret);
		return ret;
	}

	reg = (uint8_t)ret;
	reg &= ~PD_TC_ALL_RESET;
	(void)hi6526_tcpc_i2c_write8(client, REG_PD_VDM_CFG_1, reg);

	return 0;
}

static void hi6526_tcpc_pd_fsm_reset(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

#ifdef CONFIG_WORKAROUND_FF_CONER_CHIP_BIST_MODE_ERROR
	cancel_delayed_work_sync(&chip->bist_mode_2_stop_work);
#endif
	(void)__hi6526_tcpc_pd_fsm_reset(chip->client);
}

static void __get_tcpc_fsm_state(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	u8 rdata;
	s32 ret;

	const char * const tc_cu_st[16] = {
		"IDLE",
		"Unattached.SNK",
		"Attachwait.SNK",
		"Attached.SNK",
		"Unattached.SRC",
		"Attachwait.SRC",
		"Attached.SRC",
		"DegbuAccessory.SNK",
		"Orientation.Snk",
		"Attached.AudioAcc",
		"Try.SNK",
		"TryWait.SRC",
	};

	if (chip->pd_dbg_rdata_cfg != PD_DBG_RDATA_MACHINE_STATUS) {
		(void)hi6526_tcpc_i2c_write8(chip->client,
			REG_PD_DBG_RDATA_CFG, PD_DBG_RDATA_MACHINE_STATUS);
		chip->pd_dbg_rdata_cfg = PD_DBG_RDATA_MACHINE_STATUS;
	}

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_DBG_RDATA);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return;
	}

	rdata = (u8)(u32)ret;
	I("pd_dbg_rdata 0x%02x\n", rdata);

	if (rdata & TCPC_DBG_ACC)
		D("DebugAccessory.Snk\n");

	if (rdata & TCPC_PD_STATE)
		D("PD Attached\n");
	else
		D("PD Unattached or AttachWait\n");

	if (rdata & TCPC_TYPEC_STATE)
		D("TypeC Attached\n");
	else
		D("TypeC Unattached or AttachWait\n");

	I("tc_cu_st %s\n", tc_cu_st[rdata & 0xf]);
}

static void hi6526_tcpc_pd_fsm_state(struct hi6526_tcpc_device *tcpc)
{
	__get_tcpc_fsm_state(tcpc);
}

static void hi6526_tcpc_init_alert_mask(struct hi6526_tcpc_device *tcpc)
{
	uint16_t mask;
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	mask = TCPC_V10_REG_ALERT_CC_STATUS | TCPC_V10_REG_ALERT_POWER_STATUS;
#ifdef CONFIG_USB_POWER_DELIVERY_SUPPORT
	/* Need to handle RX overflow */
	mask |= TCPC_V10_REG_ALERT_TX_SUCCESS   |
		TCPC_V10_REG_ALERT_TX_DISCARDED |
		TCPC_V10_REG_ALERT_TX_FAILED    |
		TCPC_V10_REG_ALERT_RX_HARD_RST  |
		TCPC_V10_REG_ALERT_RX_STATUS    |
		TCPC_V10_REG_RX_OVERFLOW;
#endif
	mask |= TCPC_REG_ALERT_FAULT;
	(void)hi6526_tcpc_i2c_write16(chip->client,
				      TCPC_V10_REG_ALERT_MASK, mask);
}

static void hi6526_tcpc_init_power_status_mask(struct hi6526_tcpc_device *tcpc)
{
	const uint8_t mask = TCPC_V10_REG_POWER_STATUS_VBUS_PRES;

	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_POWER_STATUS_MASK, mask);
}

static void hi6526_tcpc_init_fault_mask(struct hi6526_tcpc_device *tcpc)
{
	const uint8_t mask = TCPC_V10_REG_FAULT_STATUS_VCONN_OV |
			     TCPC_V10_REG_FAULT_STATUS_VCONN_OC;

	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_FAULT_STATUS_MASK, mask);
}

static int hi6526_tcpci_init(struct hi6526_tcpc_device *tcpc, bool sw_reset)
{
	if (sw_reset)
		D("sw_reset true!!!\n");

	/*
	 * UFP Both RD setting
	 * DRP = 0, RpVal = 0 (Default), Rd, Rd
	 */
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_ROLE_CTRL,
		TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, CC_RD, CC_RD));

	tcpci_alert_status_clear(tcpc, 0xffffffff);

	hi6526_tcpc_init_power_status_mask(tcpc);
	hi6526_tcpc_init_alert_mask(tcpc);
	hi6526_tcpc_init_fault_mask(tcpc);

	return 0;
}

static void hi6526_tcpc_fr_swap(struct hi6526_tcpc_device *tcpc, bool enable)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	int ret;
	u8 rdata;
	u16 mask;

	if (chip->otg_vbst_ori == 0) {
		D("Inside Otg\n");
		return;
	}

	if (chip->fr_swap_status == enable)
		return;

	chip->fr_swap_status = enable;

	/* alert mask: fr_swap */
	ret = hi6526_tcpc_i2c_read8(chip->client, TCPC_V10_REG_ALERT_MASK);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return;
	}

	mask = (u16)(u32)ret;
	if (enable)
		mask |= TCPC_V10_REG_ALERT_FR_SWAP;
	else
		mask &= ~TCPC_V10_REG_ALERT_FR_SWAP;
	(void)hi6526_tcpc_i2c_write16(chip->client,
				      TCPC_V10_REG_ALERT_MASK, mask);

	/* Switch */
	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_VDM_CFG_0);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return;
	}

	rdata = (u8)(u32)ret;
	if (enable)
		rdata |= PD_DA_FRS_ENABLE;
	else
		rdata &= ~PD_DA_FRS_ENABLE;
	(void)hi6526_tcpc_i2c_write8(chip->client, REG_PD_VDM_CFG_0, rdata);

	/* FR_Swap Detect en */
	ret = hi6526_tcpc_i2c_read8(chip->client, REG_TCPC_CFG_REG_1);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return;
	}

	rdata = (u8)(u32)ret;
	if (enable)
		rdata |= PD_FRS_DETECT;
	else
		rdata &= ~PD_FRS_DETECT;
	(void)hi6526_tcpc_i2c_write8(chip->client, REG_TCPC_CFG_REG_1, rdata);
}

static void hi6526_tcpc_sc_buck_ctrl(struct hi6526_tcpc_device *tcpc, bool en)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	u8 rdata;
	s32 ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_SC_BUCK_EN);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return;
	}

	rdata = (u8)(u32)ret;
	if (en)
		rdata &= ~BIT_SC_BUCK_EN;
	else
		rdata |= BIT_SC_BUCK_EN;
	(void)hi6526_tcpc_i2c_write8(chip->client, REG_SC_BUCK_EN, rdata);
}

uint8_t hi6526_tcpc_get_cc_from_analog_ch(struct hi6526_tcpc_device *tcpc)
{
	int ret;
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	if (chip->pd_dbg_rdata_cfg != PD_DBG_RDATA_CC_STATUS) {
		(void)hi6526_tcpc_i2c_write8(chip->client,
			REG_PD_DBG_RDATA_CFG, PD_DBG_RDATA_CC_STATUS);
		chip->pd_dbg_rdata_cfg = PD_DBG_RDATA_CC_STATUS;
	}

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_DBG_RDATA);
	if (ret < 0) {
		E("i2c read8 error ret %d\n", ret);
		return 0;
	}

	return (uint8_t)(u32)ret;
}

#define MAX_CC_CHECK_CNT 3
static int hi6526_tcpc_cc_is_realdetach(struct hi6526_tcpc_device *tcpc)
{
	u8 rdata;
	int i;

	for (i = 0; i < MAX_CC_CHECK_CNT; i++) {
		rdata = hi6526_tcpc_get_cc_from_analog_ch(tcpc);
		D("dump debug-CC-status 0x%x\n", rdata);
		if (rdata != 0)
			break;
	}

	return (rdata == 0x0);
}

static bool hi6526_tcpc_vusb_uv_det_mask(void)
{
	return !!hi6526_tcpc_get_vusb_uv_det_sts();
}

static bool hi6526_tcpc_chip_v610(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	return chip->chip_version == CHIP_VERSION_V610;
}

static int hi6526_tcpc_tcpc_deinit(struct hi6526_tcpc_device *tcpc_dev)
{
	bool charger_type_pd = usb_typec_charger_type_pd();

	hi6526_tcpc_set_cc(tcpc_dev, TYPEC_CC_DRP);
	if (charger_type_pd)
		hi6526_tcpc_set_cc(tcpc_dev, TYPEC_CC_OPEN);

	return 0;
}

static int hi6526_tcpc_alert_status_clear(struct hi6526_tcpc_device *tcpc,
	uint32_t mask)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	uint16_t mask_t1;

	mask_t1 = (uint16_t) mask;
	if (mask_t1)
		(void)hi6526_tcpc_i2c_write16(chip->client,
					      TCPC_V10_REG_ALERT, mask_t1);
	return 0;
}

static int hi6526_tcpc_fault_status_clear(struct hi6526_tcpc_device *tcpc,
	uint8_t status)
{
	return hi6526_tcpc_write8(tcpc, TCPC_V10_REG_FAULT_STATUS, status);
}

static int hi6526_tcpc_get_alert_status(struct hi6526_tcpc_device *tcpc,
	uint32_t *alert)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	int ret;

	ret = hi6526_tcpc_i2c_read16(chip->client, TCPC_V10_REG_ALERT);
	if (ret < 0) {
		E("i2c read16 error ret %d\n", ret);
		return ret;
	}
	*alert = (uint16_t)ret;

	if (chip->poll_cc_status) {
		D("poll_cc_status, cc_status_change\n");
		*alert |= TCPC_REG_ALERT_CC_STATUS;
		chip->poll_cc_status = 0;
	}

	return 0;
}

static void enable_auto_discharge_disconnect(struct hi6526_tcpc_chip *chip)
{
	uint8_t reg;
	int ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, TCPC_V10_REG_POWER_CTRL);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return;
	}

	reg = (uint8_t)(unsigned int)ret;
	reg |= TCPC_V10_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT;
	(void)hi6526_tcpc_i2c_write8(chip->client,
				     TCPC_V10_REG_POWER_CTRL, reg);
}

static int hi6526_tcpc_get_power_status(struct hi6526_tcpc_device *tcpc,
	uint16_t *pwr_status)
{
	uint8_t ps_reg;
	int ret;
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_POWER_STATUS);
	if (ret < 0)
		return ret;

	ps_reg = (uint8_t)(unsigned int)ret;

	*pwr_status = 0;

	if (ps_reg & TCPC_V10_REG_POWER_STATUS_SINK_VBUS)
		*pwr_status |= TCPC_REG_POWER_STATUS_SINK_VBUS;

	if (ps_reg & TCPC_V10_REG_POWER_STATUS_VCONN_PRES)
		*pwr_status |= TCPC_REG_POWER_STATUS_VCONN_PRES;

	if (ps_reg & TCPC_V10_REG_POWER_STATUS_VBUS_PRES_DET)
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES_DET;

	if (ps_reg & TCPC_V10_REG_POWER_STATUS_SRC_VBUS)
		*pwr_status |= TCPC_REG_POWER_STATUS_SRC_VBUS;

	if (chip->vbus_detect) {
		if (ps_reg & TCPC_V10_REG_POWER_STATUS_VBUS_PRES) {
			*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
			enable_auto_discharge_disconnect(chip);
		}
	}

	I("pwr_status 0x%x\n", *pwr_status);
	return 0;
}

static int hi6526_tcpc_get_fault_status(struct hi6526_tcpc_device *tcpc,
	uint8_t *status)
{
	int ret;

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_FAULT_STATUS);
	if (ret < 0)
		return ret;

	*status = (uint8_t)ret;

	return 0;
}

#define TYPEC_CC_VOLT_ACT_AS_SINK (1 << 2)

static int hi6526_tcpc_get_cc(struct hi6526_tcpc_device *tcpc, int *cc1, int *cc2)
{
	int ret;
	uint8_t status, role_ctrl, cc_role;
	bool act_as_sink = false;
	bool act_as_drp = false;

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_CC_STATUS);
	if (ret < 0)
		return ret;
	status = (uint8_t)ret;
	I("cc status reg: 0x%x\n", status);

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_ROLE_CTRL);
	if (ret < 0)
		return ret;
	role_ctrl = (uint8_t)ret;

	if (status & TCPC_V10_REG_CC_STATUS_DRP_TOGGLING) {
		*cc1 = TYPEC_CC_DRP_TOGGLING;
		*cc2 = TYPEC_CC_DRP_TOGGLING;
		return 0;
	}

	*cc1 = TCPC_V10_REG_CC_STATUS_CC1(status);
	*cc2 = TCPC_V10_REG_CC_STATUS_CC2(status);

	act_as_drp = TCPC_V10_REG_ROLE_CTRL_DRP & role_ctrl;
	if (act_as_drp) {
		act_as_sink = TCPC_V10_REG_CC_STATUS_DRP_RESULT(status);
	} else {
		cc_role =  TCPC_V10_REG_CC_STATUS_CC1(role_ctrl);
		if (cc_role == TYPEC_CC_RP)
			act_as_sink = false;
		else
			act_as_sink = true;
	}

	/*
	 * If status is not open, then OR in termination to convert to
	 * enum tcpm_cc_voltage_status.
	 */
	if ((*cc1 != TYPEC_CC_VOLT_OPEN) && act_as_sink)
		*cc1 = (unsigned int)(*cc1) | TYPEC_CC_VOLT_ACT_AS_SINK;

	if ((*cc2 != TYPEC_CC_VOLT_OPEN) && act_as_sink)
		*cc2 = (unsigned int)(*cc2) | TYPEC_CC_VOLT_ACT_AS_SINK;

	return 0;
}

static int hi6526_tcpc_set_cc(struct hi6526_tcpc_device *tcpc, int pull)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	uint8_t data;
	unsigned int rp_lvl;

	rp_lvl = TYPEC_CC_PULL_GET_RP_LVL((unsigned int)pull);
	pull = TYPEC_CC_PULL_GET_RES((unsigned int)pull);
	if (pull == TYPEC_CC_DRP) {
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(1, rp_lvl,
						      TYPEC_CC_RD, TYPEC_CC_RD);
		D("DRP set role ctrl 0x%x\n", data);
		(void)hi6526_tcpc_i2c_write8(chip->client,
					     TCPC_V10_REG_ROLE_CTRL, data);

		hi6526_tcpc_command(tcpc, TCPM_CMD_LOOK_CONNECTION);
	} else {
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(0, rp_lvl,
				(unsigned int)pull, (unsigned int)pull);
		D("set role ctrl 0x%x\n", data);
		(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
	}

	return 0;
}

static int hi6526_tcpc_set_polarity_cc(struct hi6526_tcpc_device *tcpc)
{
	uint8_t data;
	int ret;

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if ((uint8_t)ret & TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT)
		data = 0x4;
	else
		data = 0x1;
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);

	return 0;
}

static int hi6526_tcpc_set_polarity(struct hi6526_tcpc_device *tcpc, int polarity)
{
	uint8_t data;
	int ret;

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (ret < 0)
		return ret;
	data = (uint8_t)ret;

	data &= ~TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT;
	data |= polarity ? TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT : 0;
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);

	return 0;
}

static int hi6526_tcpc_set_vconn(struct hi6526_tcpc_device *tcpc, int enable)
{
	uint8_t data;
	int ret;

	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_POWER_CTRL);
	if (ret < 0)
		return ret;
	data = (uint8_t)ret;

	data &= ~TCPC_V10_REG_POWER_CTRL_VCONN;
	data |= enable ? TCPC_V10_REG_POWER_CTRL_VCONN : 0;
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_POWER_CTRL, data);

	usb_typec_set_vconn(enable);

	return 0;
}

static void hi6526_tcpc_snk_unattach_by_vbus_bypass(
	struct hi6526_tcpc_device *tcpc_dev, bool en)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc_dev);
	int ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_CDR_CFG_0);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return;
	}

	if (en)
		ret = (uint8_t)ret | MASK_SNK_VBUS_DETECT;
	else
		ret = (uint8_t)ret & (~MASK_SNK_VBUS_DETECT);

	(void)hi6526_tcpc_i2c_write8(chip->client, REG_PD_CDR_CFG_0, ret);
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int hi6526_tcpc_set_msg_header(struct hi6526_tcpc_device *tcpc,
	int power_role, int data_role, uint8_t rev)
{
	uint8_t msg_header =
		TCPC_V10_REG_MSG_HDR_INFO_SET((unsigned int)data_role,
			(unsigned int)power_role, rev);

	D("msg_header 0x%02x\n", msg_header);
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_MSG_HDR_INFO, msg_header);
	return 0;
}

static int hi6526_tcpc_set_rx_enable(struct hi6526_tcpc_device *tcpc, uint8_t enable)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	uint8_t reg;
	int ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, TCPC_V10_REG_RX_DETECT);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return ret;
	}

	reg = (uint8_t)(unsigned int)ret;
	D("RX_DETECT 0x%x, enable 0x%x\n", reg, enable);

	reg = enable;
	ret = hi6526_tcpc_i2c_write8(chip->client, TCPC_V10_REG_RX_DETECT, reg);

	return ret;
}

/*
 * Receiving or Received
 */
static int hi6526_tcpc_rx_busy(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	int ret;

	if (chip->pd_dbg_rdata_cfg != PD_DBG_RDATA_RX_TX_STATUS) {
		(void)hi6526_tcpc_i2c_write8(chip->client,
			REG_PD_DBG_RDATA_CFG, PD_DBG_RDATA_RX_TX_STATUS);
		chip->pd_dbg_rdata_cfg = PD_DBG_RDATA_RX_TX_STATUS;
	}

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_PD_DBG_RDATA);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return ret;
	}

	if ((uint8_t)ret & PD_PHY_RX_STAT) {
		D("Receiving\n");
		return 1;
	}

	ret = hi6526_tcpc_i2c_read16(chip->client, TCPC_V10_REG_ALERT);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read16 error ret %d\n", ret);
		return ret;
	}

	if ((uint16_t)ret & TCPC_REG_ALERT_RX_STATUS) {
		D("Received\n");
		return 1;
	}

	return 0;
}

/*
 * TCPCI 2.0: 0x30 ~ 0x4F
 * Offset: Size, Info
 * 0x30: 1Byte, CNT
 * 0x31: 1Byte, Type
 * 0x32: 2Byte, Header
 * 0x34: CNT Byte, Payload
 */
static int hi6526_tcpc_get_message(struct hi6526_tcpc_device *tcpc, uint32_t *payload,
	uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	int rv;
	uint8_t type, cnt;
	uint8_t buf[8] = {0};
	const uint16_t alert_rx = TCPC_V10_REG_ALERT_RX_STATUS |
				  TCPC_V10_REG_RX_OVERFLOW;

	rv = hi6526_tcpc_block_read(TCPC_V10_REG_RX_BYTE_CNT, 8, buf);
	if (rv) {
		E("hi6526_tcpc_block_read error rv %d\n", rv);
		return rv;
	}

	cnt = buf[0]; /* 1st Byte: CNT */
	type = buf[1]; /* 2nd Byte: Type */
	*msg_head = *(uint16_t *)&buf[2]; /* 3rid&4th Byte: Header */
	payload[0] = *(uint32_t *)&buf[4]; /* 5th Byte: Payload */

	if (cnt > 7) {
		cnt -= 7; /* MSG_HDR */
		/* First DW payload Read with Header: the last 4 of 8Byte */
		rv = hi6526_tcpc_block_read(TCPC_V10_REG_RX_DATA + 4, cnt,
				(uint8_t *)(payload + 1));
		if (rv) {
			E("hi6526_tcpc_block_read error rv %d\n", rv);
			return rv;
		}
	}

	*frame_type = (enum tcpm_transmit_type) type;

	/* Read complete, clear RX status alert bit */
	tcpci_alert_status_clear(tcpc, alert_rx);

	return rv;
}

static int hi6526_tcpc_set_bist_test_mode(struct hi6526_tcpc_device *tcpc, bool en)
{
	uint8_t data;
	int ret;

	/* receive mode */
	ret = hi6526_tcpc_read8(tcpc, TCPC_V10_REG_TCPC_CTRL);
	if (ret < 0)
		return ret;

	data = (uint8_t)ret;
	data &= ~TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE;
	data |= en ? TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE : 0;
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_TCPC_CTRL, data);

	return 0;
}

/*
 * TCPCI 2.0: 0x30 ~ 0x4F
 * Offset: Size, Info
 * 0x51: 1Byte, CNT
 * 0x52: 2Byte, Header
 * 0x54: CNT Byte, Payload
 */
#define TRANSMIT_MAX_SIZE	(1 + sizeof(uint16_t) + sizeof(uint32_t) * 7)
static int hi6526_tcpc_transmit(struct hi6526_tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	int rv;
	int data_cnt, packet_cnt;
	uint8_t temp[TRANSMIT_MAX_SIZE];
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	errno_t ret;

	if (type < TCPC_TX_HARD_RESET) {
		data_cnt = sizeof(uint32_t) * PD_HEADER_CNT(header);
		packet_cnt = data_cnt + sizeof(uint16_t);

		temp[0] = (uint8_t)(uint32_t)packet_cnt;
		/* 2nd Byte:Header */
		ret = memcpy_s(temp + 1, TRANSMIT_MAX_SIZE - 1,
			       (uint8_t *)&header, 2);
		if (ret != EOK)
			E("memcpy_s header fail!\n");

		if (data_cnt > 0) {
			/* 3rd Byte:Payload */
			ret = memcpy_s(temp + 3, TRANSMIT_MAX_SIZE - 3,
				       (uint8_t *)data, data_cnt);
			if (ret != EOK)
				E("memcpy_s data fail!\n");
		}

		/* Fill Tx header and payload */
		rv = hi6526_tcpc_block_write(TCPC_V10_REG_TX_BYTE_CNT,
					     packet_cnt + 1, (uint8_t *)temp);
		if (rv) {
			E("hi6526_tcpc_block_write error rv %d\n", rv);
			return rv;
		}
#ifdef CONFIG_WORKAROUND_FF_CONER_CHIP_BIST_MODE_ERROR
	} else if (type == TCPC_TX_BIST_MODE_2) {
		D("transmit BIST Mode 2\n");
		schedule_delayed_work(&chip->bist_mode_2_stop_work,
			msecs_to_jiffies(BIST_MODE_2_STOP_TIME_MS));
#endif
	}

	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_TRANSMIT,
		TCPC_V10_REG_TRANSMIT_SET((unsigned int)type));
	return 0;
}

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
static int hi6526_tcpc_retransmit(struct hi6526_tcpc_device *tcpc)
{
	(void)hi6526_tcpc_write8(tcpc, TCPC_V10_REG_TRANSMIT,
		TCPC_V10_REG_TRANSMIT_SET(TCPC_TX_SOP));
	return 0;
}
#endif

#endif /* CONFIG_USB_POWER_DELIVERY_SUPPORT */

static struct tcpc_ops hi6526_tcpc_ops = {
	.init                        = hi6526_tcpci_init,
	.alert_status_clear          = hi6526_tcpc_alert_status_clear,
	.fault_status_clear          = hi6526_tcpc_fault_status_clear,
	.get_alert_status            = hi6526_tcpc_get_alert_status,
	.get_power_status            = hi6526_tcpc_get_power_status,
	.get_fault_status            = hi6526_tcpc_get_fault_status,
	.get_cc                      = hi6526_tcpc_get_cc,
	.set_cc                      = hi6526_tcpc_set_cc,
	.set_polarity                = hi6526_tcpc_set_polarity,
	.set_vconn                   = hi6526_tcpc_set_vconn,
	.source_vbus                 = hi6526_tcpc_source_vbus,
	.deinit                      = hi6526_tcpc_tcpc_deinit,

#ifdef CONFIG_USB_POWER_DELIVERY_SUPPORT
	.set_msg_header              = hi6526_tcpc_set_msg_header,
	.set_rx_enable               = hi6526_tcpc_set_rx_enable,
	.check_rx_busy               = hi6526_tcpc_rx_busy,
	.get_message                 = hi6526_tcpc_get_message,
	.set_bist_test_mode          = hi6526_tcpc_set_bist_test_mode,
	.transmit                    = hi6526_tcpc_transmit,
#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	.retransmit                  = hi6526_tcpc_retransmit,
#endif
#endif
	.set_vbus_detect             = hi6526_tcpc_set_vbus_detect,
	.reset_pd_fsm                = hi6526_tcpc_pd_fsm_reset,
	.tcpc_pd_fsm_state           = hi6526_tcpc_pd_fsm_state,
	.snk_unattach_by_vbus_bypass = hi6526_tcpc_snk_unattach_by_vbus_bypass,
	.tcpc_cc_is_realdetach       = hi6526_tcpc_cc_is_realdetach,
	.set_fr_swap                 = hi6526_tcpc_fr_swap,
	.set_polarity_cc             = hi6526_tcpc_set_polarity_cc,
	.da_vbus_en_status           = hi6526_tcpc_da_vbus_en_status,
	.sc_buck_ctrl                = hi6526_tcpc_sc_buck_ctrl,
	.vusb_uv_det_mask            = hi6526_tcpc_vusb_uv_det_mask,
	.chip_version_v610           = hi6526_tcpc_chip_v610,
};

static void hi6526_tcpc_wdt_rst_mask_typec(struct i2c_client *client)
{
	int ret = hi6526_tcpc_i2c_read8(client, REG_PD_DBG_CFG_1);
	uint8_t reg_val;

	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
	} else {
		reg_val = (uint8_t)ret | MASK_WDT_RST_PD_BIT;
		(void)hi6526_tcpc_i2c_write8(client, REG_PD_DBG_CFG_1, reg_val);
	}

	ret = hi6526_tcpc_i2c_read8(client, REG_PD_DBG_CFG_1);
	if (ret < 0)
		 E("hi6526_tcpc_i2c_read8 error again ret %d\n", ret);
}

static void hi6526_tcpc_poll_ctrl(struct hi6526_tcpc_chip *chip)
{
	cancel_delayed_work_sync(&chip->poll_work);

	if (atomic_read(&chip->poll_count) == 0) {
		atomic_inc(&chip->poll_count);
		cpu_idle_poll_ctrl(true);
	}

	schedule_delayed_work(&chip->poll_work, msecs_to_jiffies(40));
}

static void hi6526_tcpc_poll_work(struct work_struct *work)
{
	struct hi6526_tcpc_chip *chip = container_of(
			work, struct hi6526_tcpc_chip, poll_work.work);

	if (atomic_dec_and_test(&chip->poll_count))
		cpu_idle_poll_ctrl(false);
}

void hi6526_tcpc_poll_cc_status(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);

	chip->poll_cc_status = 1;
	__pm_stay_awake(&chip->irq_wakelock);
	kthread_queue_delayed_work(&chip->irq_worker, &chip->irq_work, 0);
}

static void hi6526_tcpc_cc_abnormal_irq_handler(struct hi6526_tcpc_chip *chip)
{
	uint8_t reg_val;
	int32_t ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_IRQ_FLAG_5);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return;
	}

	reg_val = (uint8_t)(uint32_t)ret;
	if (reg_val & BIT_IRQ_MASK_CC_OVP) {
		E("CC OVP\n");
		(void)hi6526_tcpc_i2c_write8(chip->client, REG_IRQ_FLAG_5,
					     BIT_IRQ_MASK_CC_OVP);
		usb_typec_cc_ovp_dmd_report();
	}
}

static void hi6526_tcpc_irq_work_handler(struct kthread_work *work)
{
	struct hi6526_tcpc_chip *chip =
		container_of(work, struct hi6526_tcpc_chip, irq_work.work);
	int ret;
	int gpio_val;

	__pm_stay_awake(&chip->irq_wakelock);

	hi6526_tcpc_poll_ctrl(chip);

	/* make sure I2C bus had resumed */
	down(&chip->suspend_lock);

	do {
		hi6526_tcpc_cc_abnormal_irq_handler(chip);
		ret = hi6526_tcpci_alert(chip->tcpc);
		if (ret) {
			I("alert handle\n");
			break;
		}

		gpio_val = gpio_get_value(chip->irq_gpio);
	} while (gpio_val == 0);

	up(&chip->suspend_lock);

	if (ret && (gpio_get_value(chip->irq_gpio) == 0)) {
		I("Sched tcpc irq work after 10 sec\n");
		kthread_queue_delayed_work(&chip->irq_worker, &chip->irq_work,
			msecs_to_jiffies(TCPC_IRQ_SCHED_DELAY_TIME));
	}

	__pm_relax(&chip->irq_wakelock);
}

static irqreturn_t hi6526_tcpc_intr_handler(int irq, void *data)
{
	struct hi6526_tcpc_chip *chip = data;

	__pm_stay_awake(&chip->irq_wakelock);
	/* 0:immediately */
	kthread_queue_delayed_work(&chip->irq_worker, &chip->irq_work, 0);

	return IRQ_HANDLED;
}

static void __hi6526_tcpc_init_abnormal_irq(struct hi6526_tcpc_chip *chip)
{
	uint8_t reg_val;
	int32_t ret;

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_IRQ_MASK5);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return;
	}

	reg_val = (uint8_t)(unsigned int)ret;

	if (reg_val & BIT_IRQ_MASK_CC_OVP)
		reg_val &= ~BIT_IRQ_MASK_CC_OVP;

	(void)hi6526_tcpc_i2c_write8(chip->client, REG_IRQ_MASK5, reg_val);
}

static void __hi6526_tcpc_init_alert(struct hi6526_tcpc_chip *chip)
{
	uint8_t reg_u8;
	int ret;

	/* Clear Alert Mask & Status */
	(void)hi6526_tcpc_i2c_write16(chip->client, TCPC_V10_REG_ALERT_MASK, 0);
	(void)hi6526_tcpc_i2c_write16(chip->client, TCPC_V10_REG_ALERT, 0xffff);

	ret = hi6526_tcpc_i2c_read8(chip->client, REG_IRQ_MASK);
	if (ret < 0) {
		E("hi6526_tcpc_i2c_read8 error ret %d\n", ret);
		return;
	}

	reg_u8 = (uint8_t)(unsigned int)ret;
	D("REG_IRQ_MASK 0x%x\n", reg_u8);

	if (reg_u8 & BIT_IRQ_MASK_GLB) {
		D("global irq unmask\n");
		reg_u8 &= ~BIT_IRQ_MASK_GLB;
		(void)hi6526_tcpc_i2c_write8(chip->client,
					     REG_IRQ_MASK, reg_u8);
	}
	if (reg_u8 & BIT_IRQ_MASK_SRC) {
		D("global irq source unmask\n");
		reg_u8 &= ~BIT_IRQ_MASK_SRC;
		(void)hi6526_tcpc_i2c_write8(chip->client,
					     REG_IRQ_MASK, reg_u8);
	}
	if (reg_u8 & BIT_IRQ_MASK_PD) {
		reg_u8 &= ~BIT_IRQ_MASK_PD;
		(void)hi6526_tcpc_i2c_write8(chip->client,
					     REG_IRQ_MASK, reg_u8);
	}
}

static int hi6526_tcpc_init_alert(struct hi6526_tcpc_device *tcpc)
{
	struct hi6526_tcpc_chip *chip = hi6526_tcpc_get_dev_data(tcpc);
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	int ret = 0;

	__hi6526_tcpc_init_abnormal_irq(chip);
	__hi6526_tcpc_init_alert(chip);

	I("Name = %s\n", chip->tcpc_desc->name);
	chip->irq = gpio_to_irq(chip->irq_gpio);

	kthread_init_worker(&chip->irq_worker);
	chip->irq_worker_task = kthread_run(kthread_worker_fn,
			&chip->irq_worker, chip->tcpc_desc->name);
	if (IS_ERR((void *)chip->irq_worker_task)) {
		E("Could not create tcpc task\n");
		ret = -EINVAL;
		goto out;
	}

	sched_setscheduler(chip->irq_worker_task, SCHED_FIFO, &param);
	kthread_init_delayed_work(&chip->irq_work,
				  hi6526_tcpc_irq_work_handler);
	wakeup_source_init(&chip->irq_wakelock, "hi6526_tcpc_irq_wakelock");

	ret = request_irq(chip->irq, hi6526_tcpc_intr_handler,
			  (IRQF_SHARED | IRQF_TRIGGER_FALLING |
			  IRQF_NO_SUSPEND | IRQF_NO_THREAD),
			  chip->tcpc_desc->name, chip);
	if (ret) {
		E("Request_irq error ret %d\n", ret);
		goto out;
	}

	ret = enable_irq_wake(chip->irq);
	if (ret)
		E("Enable_irq_wake error ret %d\n", ret);
out:
	return ret;
}

static void hi6526_tcpc_get_otg_vbst_type(struct hi6526_tcpc_chip *chip)
{
	chip->otg_vbst_ori = usb_typec_otg_pwr_src();
	D("Otg power Source: %s\n", chip->otg_vbst_ori ? "5V_bst" : "Hi65xx");
}

static int hi6526_tcpc_parse_dt(struct hi6526_tcpc_chip *chip,
	struct device *dev)
{
	chip->irq_gpio = hi6526_get_irq_gpio();
	D("irq_gpio %d\n", chip->irq_gpio);
	return 0;
}

static void hi6526_tcpc_parse_rp_lvl(struct device_node *np,
	struct tcpc_desc *desc)
{
	u32 val = 0;

	desc->rp_lvl = TYPEC_CC_RP_DFT;
	if (of_property_read_u32(np, "tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case 0: /* RP Default */
			desc->rp_lvl = TYPEC_CC_RP_DFT;
			break;
		case 1: /* RP 1.5V */
			desc->rp_lvl = TYPEC_CC_RP_1_5;
			break;
		case 2: /* RP 3.0V */
			desc->rp_lvl = TYPEC_CC_RP_3_0;
			break;
		default:
			break;
		}
	}
}

static int hi6526_tcpc_tcpcdev_init(struct hi6526_tcpc_chip *chip,
	struct device *dev)
{
	struct tcpc_desc *desc = NULL;
	struct device_node *np = dev->of_node;
	u32 val = 0;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (of_property_read_u32(np, "tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	D("role_def %u\n", desc->role_def);

	hi6526_tcpc_parse_rp_lvl(np, desc);

	D("rp_level %u\n", desc->rp_lvl);

	if (of_property_read_string(np, "tcpc,name",
			(char const **)&desc->name) < 0) {
		I("desc_name dts parse fail!\n");
		desc->name = "type_c_port0";
	}
	D("desc_name: %s\n", desc->name);

	chip->tcpc_desc = desc;

	chip->tcpc = hi6526_tcpc_device_register(dev, desc, &hi6526_tcpc_ops, chip);
	if (IS_ERR_OR_NULL(chip->tcpc))
		return -EINVAL;

	hi6526_usb_typec_register_tcpc_device(chip->tcpc);

	return 0;
}

static int hi6526_tcpc_check_revision(struct i2c_client *client)
{
	int ret;

	ret = hi6526_tcpc_i2c_read16(client, TCPC_V10_REG_VID);
	if (ret < 0) {
		dev_err(&client->dev, "read chip ID fail\n");
		return -EIO;
	}

	I("chip ver: 0x%x\n", (uint32_t)ret);
	return ret;
}

static void check_printk_performance(void)
{
	int i;
	u64 t1, t2;
	u32 nsrem;

#ifdef CONFIG_HI6526_PD_DBG_INFO
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		hi6526_pd_dbg_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		hi6526_pd_dbg_info("pd_dbg_info : t2-t1 = %lu\n",
				   (unsigned long)nsrem / 1000);
	}
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("pr_info : t2-t1 = %lu\n", (unsigned long)nsrem / 1000);
	}
#else
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("t2-t1 = %lu\n", (unsigned long)nsrem / 1000);
		WARN_ON(nsrem > 100 * 1000);
	}
#endif /* CONFIG_HI6526_PD_DBG_INFO */
}

static int hi6526_tcpc_probe(struct platform_device *pdev)
{
	struct hi6526_tcpc_chip *chip = NULL;
	struct i2c_client *client = NULL;
	int ret;

	D("+\n");
	ret = hi6526_usb_typec_register_pd_dpm();
	if (ret == -EBUSY)
		return 0;
	else if (ret)
		return ret;

	check_printk_performance();

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		E("alloc chip failed\n");
		return -ENOMEM;
	}

	client = hi6526_get_i2c_client();
	if (!client) {
		E("no client\n");
		devm_kfree(&pdev->dev, chip);
		return -ENODEV;
	}

	chip->client = client;
	chip->chip_version = hi6526_tcpc_get_chip_version();

	ret = hi6526_tcpc_check_revision(client);
	if (ret < 0) {
		E("I2C Communicate %d\n", ret);
		return ret;
	}

	hi6526_tcpc_get_otg_vbst_type(chip);

	ret = hi6526_tcpc_parse_dt(chip, &pdev->dev);
	if (ret)
		return ret;

	hi6526_tcpc_wdt_rst_mask_typec(chip->client);

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	sema_init(&chip->suspend_lock, 1);
	INIT_DELAYED_WORK(&chip->poll_work, hi6526_tcpc_poll_work);
#ifdef CONFIG_WORKAROUND_FF_CONER_CHIP_BIST_MODE_ERROR
	INIT_DELAYED_WORK(&chip->bist_mode_2_stop_work,
			  bist_mode_2_stop_work_fn);
#endif
	ret = hi6526_tcpc_tcpcdev_init(chip, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "tcpc dev init fail\n");
		return ret;
	}

	ret = hi6526_tcpc_init_alert(chip->tcpc);
	if (ret < 0) {
		E("tcpc init alert fail\n");
		goto err_irq_init;
	}

	chip->vbus_detect = true;
	/* start typec state machine! */
	hi6526_tcpc_schedule_init_work(chip->tcpc);

	D("-\n");
	return 0;

err_irq_init:
	hi6526_tcpc_device_unregister(chip->dev, chip->tcpc);
	return ret;
}

static int hi6526_tcpc_remove(struct platform_device *pdev)
{
	struct hi6526_tcpc_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->poll_work);
		hi6526_tcpc_device_unregister(chip->dev, chip->tcpc);
	}

	return 0;
}

static void hi6526_tcpc_shutdown(struct platform_device *pdev)
{
	struct hi6526_tcpc_chip *chip = platform_get_drvdata(pdev);
	struct hi6526_tcpc_device *tcpc = NULL;

	if (!chip)
		return;

	tcpc = chip->tcpc;
	if (chip->irq)
		disable_irq(chip->irq);

	hi6526_tcpm_shutdown(tcpc);
}

#ifdef CONFIG_PM
static int hi6526_tcpc_suspend(struct device *dev)
{
	struct hi6526_tcpc_chip *chip = NULL;

	D("+\n");
	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	down(&chip->suspend_lock);
	enable_irq_wake(chip->irq);
	D("-\n");

	return 0;
}

static int hi6526_tcpc_resume(struct device *dev)
{
	struct hi6526_tcpc_chip *chip = NULL;

	D("+\n");
	chip = dev_get_drvdata(dev);
	if (!chip)
		return 0;

	up(&chip->suspend_lock);
	disable_irq_wake(chip->irq);
	D("-\n");

	return 0;
}

static const struct dev_pm_ops hi6526_tcpc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hi6526_tcpc_suspend,
				hi6526_tcpc_resume)
};
#define HI6526_TCPC_PM_OPS	(&hi6526_tcpc_pm_ops)
#else
#define HI6526_TCPC_PM_OPS	(NULL)
#endif /* CONFIG_PM */

static const struct of_device_id hi6526_tcpc_match_table[] = {
	{ .compatible = "huawei,hi6526_tcpc", },
	{},
};

static struct platform_driver hi6526_tcpc_driver = {
	.driver = {
		.name           = "hi6526_tcpc",
		.owner          = THIS_MODULE,
		.of_match_table = hi6526_tcpc_match_table,
		.pm             = HI6526_TCPC_PM_OPS,
	},
	.probe    = hi6526_tcpc_probe,
	.remove   = hi6526_tcpc_remove,
	.shutdown = hi6526_tcpc_shutdown,
};

static int __init hi6526_tcpc_init(void)
{
	return platform_driver_register(&hi6526_tcpc_driver);
}

static void __exit hi6526_tcpc_exit(void)
{
	platform_driver_unregister(&hi6526_tcpc_driver);
}

late_initcall_sync(hi6526_tcpc_init);
module_exit(hi6526_tcpc_exit);

MODULE_DESCRIPTION("Hi6526 Type-C Port Controller Driver");
MODULE_LICENSE("GPL");

