/*
 * scharger_v600_direct_charge.c
 *
 * HI6526 charger direct charge inner api
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
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

struct opt_regs g_lvc_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0x2d3, 0xff, 0, 0x15, 0, 0),
	reg_cfg(0x2d4, 0xff, 0, 0x97, 0, 0),
	reg_cfg(0x2e1, 0xff, 0, 0x7D, 0, 0),
	reg_cfg(0x2e3, 0xff, 0, 0x22, 0, 0),
	reg_cfg(0x2e4, 0xff, 0, 0x3D, 0, 0),
	reg_cfg(0x2da, 0xff, 0, 0x8C, 0, 0),
	reg_cfg(0x2db, 0xff, 0, 0x94, 0, 0),
	reg_cfg(0x2d6, 0xff, 0, 0x0F, 0, 0),
	reg_cfg(0x2d9, 0xff, 0, 0x11, 0, 0),
	reg_cfg(0x2e7, 0xff, 0, 0xBF, 0, 0),
	reg_cfg(0x2de, 0xff, 0, 0x1F, 0, 0),
	reg_cfg(0x286, 0xff, 0, 0x06, 0, 0),
	reg_cfg(0x2b0, 0xff, 0, 0xE2, 0, 0),
	reg_cfg(0x2b1, 0xff, 0, 0x16, 0, 0),
	reg_cfg(0x2b7, 0xff, 0, 0x4D, 0, 0),
	reg_cfg(0x2b8, 0xff, 0, 0x67, 0, 0),
	reg_cfg(0x2b3, 0xff, 0, 0x1F, 0, 0),
	reg_cfg(0x2b6, 0xff, 0, 0x14, 0, 0),
	reg_cfg(0x2c6, 0xff, 0, 0x13, 0, 0),
	reg_cfg(0x2b5, 0xff, 0, 0x78, 0, 0),
	reg_cfg(0x2bb, 0xff, 0, 0xF3, 0, 0),
	reg_cfg(0x2c4, 0xff, 0, 0x4E, 0, 0),
	reg_cfg(0x2c5, 0xff, 0, 0x65, 0, 0),
	reg_cfg(0x2bc, 0xff, 0, 0x57, 0, 0),
	reg_cfg(0x2a3, 0xff, 0, 0x83, 0, 0),
	reg_cfg(0x2a4, 0xff, 0, 0x02, 0, 0),
};

struct opt_regs g_lvc_opt_regs_after_enabled[] = {
	reg_cfg(0x2bb, 0xff, 0, 0xF7, 20, 0),
};

struct opt_regs g_sc_opt_regs[] = {
	/* reg, mask, shift, val, before,after */
	reg_cfg(0x2a6, 0x07, 0, 0x07, 0, 0),
	reg_cfg(0x2d3, 0xff, 0, 0x15, 0, 0),
	reg_cfg(0x2d4, 0xff, 0, 0x97, 0, 0),
	reg_cfg(0x2e1, 0xff, 0, 0x7D, 0, 0),
	reg_cfg(0x2e3, 0xff, 0, 0x22, 0, 0),
	reg_cfg(0x2e4, 0xff, 0, 0x3D, 0, 0),
	reg_cfg(0x2da, 0xff, 0, 0x8C, 0, 0),
	reg_cfg(0x2db, 0xff, 0, 0x94, 0, 0),
	reg_cfg(0x2d6, 0xff, 0, 0x0F, 0, 0),
	reg_cfg(0x2d9, 0xff, 0, 0x11, 0, 0),
	reg_cfg(0x2e7, 0xff, 0, 0xBF, 0, 0),
	reg_cfg(0x2de, 0xff, 0, 0X1F, 0, 0),
	reg_cfg(0x286, 0xff, 0, 0x06, 0, 0),
	reg_cfg(0x2bd, 0xff, 0, 0xD9, 0, 0),
	reg_cfg(0x2be, 0xff, 0, 0x09, 0, 0),
	reg_cfg(0x2bf, 0xff, 0, 0x09, 0, 0),
	reg_cfg(0x2c0, 0xff, 0, 0x09, 0, 0),
	reg_cfg(0x2c1, 0xff, 0, 0x10, 0, 0),
	reg_cfg(0x2b0, 0xff, 0, 0XE2, 0, 0),
	reg_cfg(0x2b1, 0xff, 0, 0X16, 0, 0),
	reg_cfg(0x2b7, 0xff, 0, 0x7D, 0, 0),
	reg_cfg(0x2b8, 0xff, 0, 0X67, 0, 0),
	reg_cfg(0x2b3, 0xff, 0, 0X1F, 0, 0),
	reg_cfg(0x2b6, 0xff, 0, 0x14, 0, 0),
	reg_cfg(0x2c6, 0xff, 0, 0xF3, 0, 0),
	reg_cfg(0x2b5, 0xff, 0, 0xF8, 0, 0),
	reg_cfg(0x2c5, 0xff, 0, 0x53, 0, 0),
	reg_cfg(0x2a3, 0xff, 0, 0x83, 0, 0),
	reg_cfg(0x2a4, 0xff, 0, 0x02, 0, 0),
	reg_cfg(0x2bc, 0xff, 0, 0x57, 0, 0),
	reg_cfg(0x2bb, 0xff, 0, 0XE3, 0, 0),
};

struct opt_regs g_sc_opt_regs_after_enabled[] = {
	reg_cfg(0x2bb, 0xff, 0, 0xE7, 20, 0),
};

static int hi6526_record_chip_track(struct hi6526_device_info *di)
{
	int ret;
	u8 ilimit = 0;
	unsigned int index;

	if (di == NULL)
		return -1;
	index = di->dbg_index++;
	if (di->dbg_index == INFO_LEN)
		di->dbg_index = 0;
	di->dbg_info[index].ts_nsec = local_clock();
	ret = hi6526_get_adc_value(di, CHG_ADC_CH_IBUS_REF,
		&(di->dbg_info[index].ibus_ref));
	if (ret) {
		scharger_err("[%s]get ibus_ref fail,ret:%d\n", __func__, ret);
		return -1;
	}

	ret = hi6526_get_adc_value(di, CHG_ADC_CH_IBUS, &(di->dbg_info[index].ibus));
	if (ret) {
		scharger_err("[%s]get ibus_data fail,ret:%d\n", __func__, ret);
		return -1;
	}

	ret = hi6526_get_adc_value(di, CHG_ADC_CH_VBUS, &(di->dbg_info[index].vbus));
	if (ret) {
		scharger_err("[%s]get CHG_ADC_CH_VBUS fail,ret:%d\n",
			     __func__, ret);
		return -1;
	}

	ret = hi6526_get_adc_value(di, CHG_ADC_CH_VUSB, &(di->dbg_info[index].vusb));
	if (ret) {
		scharger_err("[%s]get CHG_ADC_CH_VUSB fail,ret:%d\n",
			     __func__, ret);
		return -1;
	}

	ret = hi6526_get_adc_value(di, CHG_ADC_CH_IBAT, &(di->dbg_info[index].ibat));
	if (ret) {
		scharger_err("[%s]get CHG_ADC_CH_IBAT fail,ret:%d\n", __func__, ret);
		return -1;
	}
	ret = hi6526_get_adc_value(di, CHG_ADC_CH_VBAT, &(di->dbg_info[index].vbat));
	if (ret) {
		scharger_err("[%s]get CHG_ADC_CH_VBAT fail,ret:%d\n",
			     __func__, ret);
		return -1;
	}

	ret = hi6526_get_adc_value(di, CHG_ADC_CH_VOUT, &(di->dbg_info[index].vout));
	if (ret) {
		scharger_err("[%s]get CHG_ADC_CH_VOUT fail,ret:%d\n",
			     __func__, ret);
		return -1;
	}

	if ((di->dbg_info[index].ibus == IBUS_INVALID_VAL) ||
		(power_platform_get_charger_type() == CHARGER_REMOVED))
		di->dbg_info[index].ibus = 0;

	if (di->chg_mode != LVC && di->chg_mode != SC) {
		(void)hi6526_read_mask(di, CHG_INPUT_SOURCE_REG,
			CHG_ILIMIT_MSK,
			CHG_ILIMIT_SHIFT, &ilimit);
		if (ilimit < CHG_ILIMIT_600MA)
			di->dbg_info[index].ibus = di->dbg_info[index].ibus /
				CHG_IBUS_DIV;
	}

	return ret;
}

static size_t printk_time(u64 ts_nsec, char *buf, int len_buf)
{
	int temp;
	unsigned long rem_nsec;
	/* 1000000000: nano seconds to seconds */
	rem_nsec = do_div(ts_nsec, 1000000000);
	/* 1000: nano seconds to micro seconds */
	temp = snprintf_s(buf, (unsigned long)BUF_LEN, (unsigned long)BUF_LEN - 1,
			  "[%5lu.%06lus]", (unsigned long)ts_nsec, rem_nsec / 1000);
	if (temp >= 0)
		return (unsigned int)temp;
	else
		return 0;
}

static void hi6526_dbg_printk(struct hi6526_device_info *di)
{
	int index;
	int count = INFO_LEN;
	char time_log[BUF_LEN] = "";
	size_t tlen;
	char *ptime_log = NULL;

	if (di == NULL)
		return;

	index = di->dbg_index;

	while (count--) {
		ptime_log = time_log;
		tlen = printk_time(di->dbg_info[index].ts_nsec, ptime_log, BUF_LEN);

		scharger_err("%s_%d <%s> vusb:%d, vbus:%d, ibus:%d, ibat:%d, vout:%d, vbat:%d, ibus_ref:%d\n",
			__func__, di->param_dts.ic_role, time_log, di->dbg_info[index].vusb,
			di->dbg_info[index].vbus,
			di->dbg_info[index].ibus,
			di->dbg_info[index].ibat,
			di->dbg_info[index].vout,
			di->dbg_info[index].vbat,
			di->dbg_info[index].ibus_ref);
		index++;
		if (index == INFO_LEN)
			index = 0;
	}
}

void hi6526_dbg_work(struct work_struct *work)
{
	struct hi6526_device_info *di = container_of(work, struct hi6526_device_info, dbg_work.work);

	if (di == NULL)
		return;

	if (di->param_dts.dbg_check_en != ENABLE_DBG_CHECK)
		return;

	(void)hi6526_record_chip_track(di);
	if (di->dbg_work_stop)
		di->delay_cunt++;
	else
		di->delay_cunt = 0;

	if (di->delay_cunt > DELAY_TIMES) {
		if (di->abnormal_happened)
			hi6526_dbg_printk(di);
		di->abnormal_happened = 0;
		di->dbg_work_stop = 0;
	} else {
		queue_delayed_work(system_power_efficient_wq, &di->dbg_work,
				   msecs_to_jiffies(DBG_WORK_TIME));
	}
}

void hi6526_dc_ucp_delay_work(struct work_struct *work)
{
	u32 ibus = 0;
	struct hi6526_device_info *di = container_of(work, struct hi6526_device_info, dc_ucp_work.work);

	if (!di || (di->chg_mode != SC && di->chg_mode != LVC)) {
		di->ucp_ibus_abn_cnt = 0;
		return;
	}

	hi6526_get_ibus_ma_exec(di, &ibus);
	if (di->ucp_work_first_run && ibus > IBUS_OCP_START_VAL)
		di->ucp_work_first_run = 0;

	if (ibus < IBUS_ABNORMAL_VAL && !di->ucp_work_first_run)
		di->ucp_ibus_abn_cnt++;
	else
		di->ucp_ibus_abn_cnt = 0;

	if (di->ucp_ibus_abn_cnt >= IBUS_ABNORMAL_CNT) {
		scharger_inf("%s:role:%d, cnt %d, ibus %d, chg_mode %d\n", __func__, di->param_dts.ic_role,
			di->wdg_ibus_abn_cnt, ibus, di->chg_mode);
		di->ucp_ibus_abn_cnt = 0;
		di->dc_ibus_ucp_happened = 1;
		hi6526_direct_charge_fault_handle(di, IRQ_IBUS_DC_UCP_MASK);
		(void)hi6526_lvc_enable(0, di);
		(void)hi6526_sc_enable(0, di);
		return;
	}

	di->dc_ibus_ucp_happened = 0;
	queue_delayed_work(system_power_efficient_wq, &di->dc_ucp_work,
			   msecs_to_jiffies(IBUS_ABNORMAL_TIME));
}

int hi6526_switch_to_buck_mode(void *dev_data)
{
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}
	di->dc_ibus_ucp_happened = 0;
	return 0;
}

static void hi6526_lvc_opt_regs(struct hi6526_device_info *di)
{
	hi6526_opt_regs_set(di, g_lvc_opt_regs, ARRAY_SIZE(g_lvc_opt_regs));
}

void hi6526_sc_opt_regs(struct hi6526_device_info *di)
{
	hi6526_opt_regs_set(di, g_sc_opt_regs, ARRAY_SIZE(g_sc_opt_regs));
}

static void hi6526_after_direct_charger(struct hi6526_device_info *di,
					int enable)
{
	if (enable) {
		if (di->chg_mode == LVC) {
			hi6526_opt_regs_set(di, g_lvc_opt_regs_after_enabled,
				ARRAY_SIZE(g_lvc_opt_regs_after_enabled));
		} else if (di->chg_mode == SC) {
			hi6526_opt_regs_set(di, g_sc_opt_regs_after_enabled,
				ARRAY_SIZE(g_sc_opt_regs_after_enabled));
		} else {
			scharger_err("%s chg mode %d error!\n", __func__,
				     di->chg_mode);
			return;
		}
		di->dbg_work_stop = 0;
		di->abnormal_happened = 0;
		di->ucp_work_first_run = 1;
		queue_delayed_work(system_power_efficient_wq, &di->dc_ucp_work,
				   msecs_to_jiffies(IBUS_ABNORMAL_TIME));
		queue_delayed_work(system_power_efficient_wq, &di->dbg_work,
				   msecs_to_jiffies(DBG_WORK_TIME));
	} else {
		mutex_lock(&di->adc_conv_lock);
		di->chg_mode = NONE;
		di->ucp_work_first_run = 0;
		di->dbg_work_stop = 1;
		(void)hi6526_adc_loop_enable(di, CHG_ADC_DIS);
		mutex_unlock(&di->adc_conv_lock);
	}
}

static void hi6526_dc_set_ibat_regulator(struct hi6526_device_info *di, int ma)
{
	u8 reg_val;

	if (ma < DC_REGULATOR_IBAT_MIN)
		ma = DC_REGULATOR_IBAT_MIN;
	if (ma > DC_REGULATOR_IBAT_MAX)
		ma = DC_REGULATOR_IBAT_MAX;

	reg_val = (ma - DC_REGULATOR_IBAT_MIN) / DC_REGULATOR_IBAT_SETP;
	(void)hi6526_write(di, DC_REGULATOR_IBAT_REG, reg_val);
}

static void hi6526_dc_set_vbat_regulator(struct hi6526_device_info *di, int mv)
{
	u8 reg_val;

	if (mv < DC_REGULATOR_VBAT_MIN)
		mv = DC_REGULATOR_VBAT_MIN;
	if (mv > DC_REGULATOR_VBAT_MAX)
		mv = DC_REGULATOR_VBAT_MAX;

	reg_val = (mv - DC_REGULATOR_VBAT_MIN) / DC_REGULATOR_VBAT_SETP;
	(void)hi6526_write(di, DC_REGULATOR_VBAT_REG, reg_val);
}

static void hi6526_dc_set_vout_regulator(struct hi6526_device_info *di, int mv)
{
	u8 reg_val;

	if (mv < DC_REGULATOR_VOUT_MIN)
		mv = DC_REGULATOR_VOUT_MIN;
	if (mv > DC_REGULATOR_VOUT_MAX)
		mv = DC_REGULATOR_VOUT_MAX;

	reg_val = (mv - DC_REGULATOR_VOUT_MIN) / DC_REGULATOR_VOUT_SETP;
	(void)hi6526_write(di, DC_REGULATOR_VOUT_REG, reg_val);
}

static void hi6526_dc_set_ibus_regulator(struct hi6526_device_info *di, int ma)
{
	u8 reg_val;

	if (ma < DC_REGULATOR_IBUS_MIN)
		ma = DC_REGULATOR_IBUS_MIN;
	if (ma > DC_REGULATOR_IBUS_MAX)
		ma = DC_REGULATOR_IBUS_MAX;

	reg_val = (ma - DC_REGULATOR_IBUS_MIN) / DC_REGULATOR_IBUS_SETP;
	(void)hi6526_write_mask(di, DC_REGULATOR_IBUS_REG, DC_REGULATOR_IBUS_MASK,
		DC_REGULATOR_IBUS_SHIFT, reg_val);
}

static void hi6526_dc_vusb2vbus_drpo_en(struct hi6526_device_info *di, int en)
{
	(void)hi6526_write_mask(di, DC_VUSB2VBUS_DRPO_REG, DC_VUSB2VBUS_DRPO_MASK,
		DC_VUSB2VBUS_DRPO_SHIFT, en);
}

static void hi6526_set_dc_regulator(struct hi6526_device_info *di, enum chg_mode_state chg_mode)
{
	struct dc_regulator_info *regulator = NULL;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return;
	}
	if (chg_mode == LVC) {
		regulator = &(di->param_dts.lvc_regulator);
	} else if (chg_mode == SC) {
		regulator = &(di->param_dts.sc_regulator);
	} else {
		scharger_err("%s charge mode err %d!\n", __func__, chg_mode);
		regulator = &(di->param_dts.lvc_regulator);
	}

	scharger_inf(
		"%s, chg_mode %d, ibat %d ma, vbat %d mv, vout %d mv, ibus %d ma!\n",
		__func__, chg_mode, regulator->ibat,
		regulator->vbat, regulator->vout, regulator->ibus);

	hi6526_dc_set_ibat_regulator(di, regulator->ibat);
	hi6526_dc_set_vbat_regulator(di, regulator->vbat);
	hi6526_dc_set_vout_regulator(di, regulator->vout);
	hi6526_dc_set_ibus_regulator(di, regulator->ibus);
}

int hi6526_lvc_enable(int enable, void *dev_data)
{
	u8 lvc_mode = 0;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}
	(void)hi6526_read_mask(di, LVC_CHG_MODE_REG, LVC_CHG_MODE_MASK,
		LVC_CHG_MODE_SHIFT, &lvc_mode);

	if (!enable && di->chg_mode != LVC && (!lvc_mode))
		return 0;

	scharger_inf("[%s] %d\n", __func__, enable);

	if (enable) {
		di->chg_mode = LVC;
		hi6526_lvc_opt_regs(di);
		hi6526_set_dc_regulator(di, LVC);
		hi6526_dc_vusb2vbus_drpo_en(di, di->param_dts.lvc_vusb2vbus_drop_en);
	}
	(void)hi6526_write_mask(di, LVC_CHG_MODE_FLAG_REG, LVC_CHG_MODE_FLAG_MASK,
		LVC_CHG_MODE_FLAG_SHIFT, !!enable);
	(void)hi6526_write_mask(di, LVC_CHG_MODE_REG, LVC_CHG_MODE_MASK,
		LVC_CHG_MODE_SHIFT, !!enable);
	(void)hi6526_write_mask(di, LVC_CHG_EN_REG, LVC_CHG_EN_MASK,
		LVC_CHG_EN_SHIFT, !!enable);

	hi6526_after_direct_charger(di, enable);

	return 0;
}

int hi6526_sc_enable(int enable, void *dev_data)
{
	u8 sc_mode = 0;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (di == NULL) {
		scharger_err("%s hi6526_device_info is NULL!\n", __func__);
		return -ENOMEM;
	}
	(void)hi6526_read_mask(di, SC_CHG_MODE_REG, SC_CHG_MODE_MASK,
		SC_CHG_MODE_SHIFT, &sc_mode);

	if (!enable && di->chg_mode != SC && (!sc_mode))
		return 0;

	scharger_inf("[%s] %d role:%d\n", __func__, enable, di->param_dts.ic_role);

	if (enable) {
		di->chg_mode = SC;
		hi6526_sc_opt_regs(di);
		hi6526_set_dc_regulator(di, SC);
		hi6526_dc_vusb2vbus_drpo_en(di, di->param_dts.sc_vusb2vbus_drop_en);
	}
	(void)hi6526_write_mask(di, SC_CHG_MODE_FLAG_REG, SC_CHG_MODE_FLAG_MASK,
		SC_CHG_MODE_FLAG_SHIFT, !!enable);
	(void)hi6526_write_mask(di, SC_CHG_MODE_REG, SC_CHG_MODE_MASK,
		SC_CHG_MODE_SHIFT, !!enable);
	(void)hi6526_write_mask(di, SC_CHG_EN_REG, SC_CHG_EN_MASK,
		SC_CHG_EN_SHIFT, !!enable);

	hi6526_after_direct_charger(di, enable);

	return 0;
}

int hi6526_lvc_is_close(void *dev_data)
{
	u8 val = 0;
	int ret, is_close;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	ret = hi6526_read(di, LVC_CHG_EN_REG, &val);
	if (!ret && (val & LVC_CHG_EN_MASK))
		is_close = 0;
	else
		is_close = 1;
	scharger_dbg("[%s] reg 0x%x, is_close %d\n", __func__, val, is_close);
	return is_close;
}

int hi6526_sc_is_close(void *dev_data)
{
	u8 val = 0;
	int ret, is_close;
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	ret = hi6526_read(di, SC_CHG_EN_REG, &val);
	if (!ret && (val & SC_CHG_EN_MASK))
		is_close = 0;
	else
		is_close = 1;
	scharger_dbg("[%s] reg 0x%x, is_close %d\n", __func__, val, is_close);
	return is_close;
}

int hi6526_batinfo_get_ibus_ma(int *vbus_mv, void *dev_data)
{
	struct hi6526_device_info *di = (struct hi6526_device_info *)dev_data;

	if (!di || !vbus_mv) {
		scharger_err("[%s] input device or vbus_mv is NULL\n", __func__);
		return -ENOMEM;
	}
	hi6526_get_ibus_ma_exec(di, (u32 *)vbus_mv);
	return 0;
}

int hi6526_get_loadswitch_id(void *dev_data)
{
	return LOADSWITCH_SCHARGERV600;
}

int hi6526_get_switchcap_id(void *dev_data)
{
	return SWITCHCAP_SCHARGERV600;
}
