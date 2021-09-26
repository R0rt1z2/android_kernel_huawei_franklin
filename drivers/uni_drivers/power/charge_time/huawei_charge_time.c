/*
 * huawei_charge_time.c
 *
 * Calculate the remaining time of charging.
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
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

#include "huawei_charge_time.h"
#include "huawei_charge_time_debug.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include "securec.h"
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/sched/clock.h>
#include <chipset_common/hwpower/common_module/power_ui_ne.h>
#include <chipset_common/hwpower/common_module/power_temp.h>
#include <chipset_common/hwpower/common_module/power_debug.h>
#include <chipset_common/hwpower/common_module/power_interface.h>
#include <huawei_platform/hwpower/common_module/power_platform.h>
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <chipset_common/hwpower/coul/coul_interface.h>
#include <chipset_common/hwpower/common_module/power_supply.h>

struct huawei_chg_time_device *g_huawei_chg_time_info;

static u64 get_current_time(void)
{
	return (local_clock() / NSEC_PRE_SECOND);
}

static int huawei_chg_time_file_read(void *buf, u32 buf_size)
{
	int flags;
	struct file *fd = NULL;
	mm_segment_t old_fs;
	int count;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	flags = O_RDWR | O_CREAT;
	fd = filp_open(CHG_TIME_PATH, flags, 0644);
	if (IS_ERR(fd)) {
		set_fs(old_fs);
		ct_err("%s()-line=%d, fd:%d\n", __func__, __LINE__, fd);
		return -1;
	}

#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
	count = kernel_read(fd, pos, buf, buf_size);
#else
	count = kernel_read(fd, buf, buf_size, &pos);
#endif

	filp_close(fd, NULL);
	set_fs(old_fs);

	if (count != buf_size)
		ct_err("%s()-line=%d, count :%d, size %d\n",
			__func__, __LINE__, count, buf_size);

	return count;
}

int huawei_chg_time_file_write(void *buf, u32 buf_size)
{
	int flags;
	struct file *fd = NULL;
	mm_segment_t old_fs;
	int count;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	flags = O_RDWR | O_CREAT;
	fd = filp_open(CHG_TIME_PATH, flags, 0644);
	if (IS_ERR(fd)) {
		set_fs(old_fs);
		ct_err("%s()-line=%d, fd:%d\n", __func__, __LINE__, fd);
		return -1;
	}

#if (KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE)
	count = kernel_write(fd, buf, buf_size, pos);
#else
	count = kernel_write(fd, buf, buf_size, &pos);
#endif

	filp_close(fd, NULL);
	set_fs(old_fs);

	if (count != buf_size)
		ct_err("%s()-line=%d, count :%d, size %d\n",
					__func__, __LINE__, count, buf_size);

	return count;
}

static void huawei_chg_time_sub_param_printk(
	struct huawei_chg_time_param *type_param)
{
	int i, j;
	char buff[PARAM_SIZE] = {0};
	int offset = 0;
	int ret;
	const int str_len = PARAM_SIZE - 1;
	int soc;

	if (!type_param) {
		ct_err("%s()-line=%d error\n", __func__, __LINE__);
		return;
	}

	ct_info("%s start %d,TH %d,TL %d,fcc %d,cycles %d\n", __func__,
		type_param->start_soc, type_param->temp_high, type_param->temp_low,
		type_param->fcc, type_param->batt_cycles);

	for (i = 0; i < ROW; i++) {
		for (j = 0; j < COL; j++) {
			soc = i * ROW + j + 1;
			ret = snprintf_s(buff + offset, str_len - strlen(buff),
				str_len - strlen(buff) - 1, "[%d(%d)]", soc,
				type_param->step_time[soc]);
			if (ret < 0)
				ct_err(" %s, ret %d, i %d, j %d error! %s\n", __func__, ret, i,
					j, buff);
			else
				offset += ret;
		}
		ct_info(" %s, %s\n", __func__, buff);
		if (memset_s(buff, PARAM_SIZE, 0, PARAM_SIZE) != EOK)
			ct_err("%s():%d:memset_s fail!\n", __func__, __LINE__);

		offset = 0;
	}
}

static void huawei_chg_time_param_printk(struct huawei_chg_time_info *param)
{
	ct_info("standard:\n");
	huawei_chg_time_sub_param_printk(&param->standard);

	ct_info("fcp:\n");
	huawei_chg_time_sub_param_printk(&param->fcp);

	ct_info("lvc:\n");
	huawei_chg_time_sub_param_printk(&param->lvc);

	ct_info("sc:\n");
	huawei_chg_time_sub_param_printk(&param->sc);
}

int huawei_chg_time_read_param_from_flash(
	struct huawei_chg_time_device *di)
{
	int cnt;
	size_t size = sizeof(struct huawei_chg_time_info);

	cnt = huawei_chg_time_file_read(&di->flash_param, size);
	if (cnt < 0) {
		ct_err("%s()-line=%d fail\n", __func__, __LINE__);
		return -1;
	}

	ct_info("%s\n", __func__);
	huawei_chg_time_param_printk(&di->flash_param);

	return 0;
}

void huawei_chg_time_write_param_to_flash(
	struct huawei_chg_time_device *di)
{
	size_t size = sizeof(struct huawei_chg_time_info);

	huawei_chg_time_param_printk(&di->flash_param);

	huawei_chg_time_file_write(&di->flash_param, size);
}

static bool is_charge_full(struct huawei_chg_time_device *di)
{
	if (di->batt_info.soc == FULL_SOC)
		return 1;
	else
		return 0;
}

static bool is_direct_charge(enum charge_type_enum charge_type)
{
	if (charge_type == CHG_SC || charge_type == CHG_LVC)
		return 1;
	else
		return 0;
}

static int huawei_chg_time_param_select(struct huawei_chg_time_device *di,
	int charger_type)
{
	ct_dbg("%s charger_type: %d\n", __func__, charger_type);

	switch (charger_type) {
	case CHG_5V2A:
		di->ref_curve = &di->flash_param.standard;
		ct_dbg("%s charger_type: CHG_5V2A\n", __func__);
		break;
	case CHG_FCP:
		di->ref_curve = &di->flash_param.fcp;
		ct_dbg("%s charger_type: CHG_FCP\n", __func__);
		break;
	case CHG_LVC:
		if (di->chg_info.cc_cable_detect_ok)
			di->ref_curve = &di->flash_param.lvc;
		else
			di->ref_curve = &di->flash_param.lvc_none_standard;
		ct_dbg("%s charger_type: CHG_LVC\n", __func__);
		break;
	case CHG_SC:
		if (di->chg_info.cc_cable_detect_ok)
			di->ref_curve = &di->flash_param.sc;
		else
			di->ref_curve = &di->flash_param.sc_none_standard;
		ct_dbg("%s charger_type: CHG_SC\n", __func__);
		break;
	default:
		di->ref_curve = NULL;
		ct_dbg("%s charger_type: %d\n", __func__, charger_type);
		return -1;
	}
	return 0;
}

static void huawei_chg_time_updata_flash(struct huawei_chg_time_device *di)
{
	int i;

	ct_info("%s charge_type %s[%d], start soc %d, soc %d\n", __func__,
		charge_type_str_enum[di->chg_info.charge_type],
		di->chg_info.charge_type, di->start_capacity, di->batt_info.soc);

	if (di->start_capacity < 0 || di->start_capacity > FULL_SOC) {
		ct_err("%s start_capacity %d error\n", __func__, di->start_capacity);
		return;
	}

	ct_info("%s ref curve :\n", __func__);
	huawei_chg_time_sub_param_printk(di->ref_curve);
	ct_info("%s cur curve :\n", __func__);
	huawei_chg_time_sub_param_printk(&di->cur_curve);

	ct_info("%s cur curve temp_high %d, temp_low %d\n", __func__,
		di->cur_curve.temp_high, di->cur_curve.temp_low);
	if (di->cur_curve.temp_high < TEMP_HIGH &&
		di->cur_curve.temp_low > TEMP_LOW) {
		if (di->ref_curve) {
			di->ref_curve->batt_cycles = di->batt_info.batt_cycles;
			di->ref_curve->start_soc = di->start_capacity;
			di->ref_curve->fcc = di->batt_info.fcc;
			di->ref_curve->temp_high = di->cur_curve.temp_high;
			di->ref_curve->temp_low = di->cur_curve.temp_low;
			for (i = FULL_SOC; i > di->start_capacity; i--) {
				di->ref_curve->step_time[i] = di->cur_curve.step_time[i];
				di->ref_curve->volt[i] = di->cur_curve.volt[i];
			}
		}
		huawei_chg_time_write_param_to_flash(di);
	}
}

static int self_study_condition_not_match(
			struct huawei_chg_time_device *di, u64 timestemp)
{
	/*
	 * start soc is larger than the reference curve soc,
	 * and battery cycle is within 5 times, then not update the curve
	 */
	if (di->ref_curve->start_soc != 0 &&
		di->start_capacity > di->ref_curve->start_soc &&
		di->batt_info.batt_cycles < (di->ref_curve->batt_cycles + CYCLE_TH)) {
		ct_info("%s start:%d ref st %d,cyc %d,ref cyc %d\n", __func__,
			di->start_capacity, di->ref_curve->start_soc,
			di->batt_info.batt_cycles, di->ref_curve->batt_cycles);
		return -1;
	}

	/*
	 * start soc after 40%, the first 5 minutes will not be updated to
	 * avoid excessive current deviation due to internal resistance.
	 */
	if (is_direct_charge(di->chg_info.charge_type) &&
		(di->start_timestemp + MIN_START_TH) > timestemp &&
		di->start_capacity > SOC_START_TH) {
		ct_info("%s start:%ld now %ld,start_capacity %d\n", __func__,
			di->start_timestemp, timestemp, di->start_capacity);
		return -1;
	}

	return 0;
}

static void huawei_chg_time_self_study(struct huawei_chg_time_device *di)
{
	u64 timestemp = get_current_time();

	ct_dbg("%s  +\n", __func__);

	if (di->charge_type_change) {
		di->cur_curve.start_soc = di->start_capacity;
		di->start_timestemp = timestemp;
		di->cur_curve.fcc = di->batt_info.fcc;
		di->cur_curve.batt_cycles = di->batt_info.batt_cycles;
		di->cur_curve.temp_low = di->cur_curve.temp_high =
			di->batt_info.batt_temp;
	}

	if (self_study_condition_not_match(di, timestemp)) {
		ct_info("%s self study condition not match\n", __func__);
		return;
	}

	if (di->batt_info.soc == (di->pre_capacity + 1)) {
		if (di->pre_capacity == di->start_capacity && di->start_capacity > 2) {
			/* Discard the first value */
		} else {
			di->cur_curve.step_time[di->batt_info.soc] =
				timestemp - di->pre_timestemp;
			di->cur_curve.volt[di->batt_info.soc] = di->batt_info.volt_mv;

			ct_info("%s soc:%d now %ld,pre %ld, dur %d\n", __func__,
				di->batt_info.soc, timestemp, di->pre_timestemp,
				di->cur_curve.step_time[di->batt_info.soc]);
			/* for debug */
			huawei_chg_time_sub_param_printk(&di->cur_curve);
			if (is_charge_full(di)) {
				ct_info("%s soc is full\n", __func__);
				huawei_chg_time_updata_flash(di);
			}
			/* update maximum and minimum temperatures */
			if (di->cur_curve.temp_high < di->batt_info.batt_temp)
				di->cur_curve.temp_high = di->batt_info.batt_temp;
			if (di->cur_curve.temp_low > di->batt_info.batt_temp)
				di->cur_curve.temp_low = di->batt_info.batt_temp;
		}

		di->pre_timestemp = timestemp;
	}

	ct_dbg("%s  -\n", __func__);
}

static void huawei_chg_time_update(struct huawei_chg_time_device *di)
{
	union power_supply_propval value;
	struct power_supply *psy = NULL;
	enum power_supply_property psp = POWER_SUPPLY_PROP_CHARGE_TIME_REMAINING;
	int ret;

	value.intval = di->remaining_duration_with_valid;

	if (di->test_output_val)
		value.intval = di->test_output_val;

	if (di->pre_remaining_duration_with_valid == value.intval)
		return;

	psy = power_supply_get_by_name(BATTERY_NAME);
	if (!psy) {
		ct_err("%s power supply %s not exist\n", __func__, BATTERY_NAME);
		return;
	}

	ret = power_supply_set_property(psy, psp, &value);
	if (ret) {
		ct_err("%s power supply set CHARGE_TIME_REMAINING error %d\n",
			__func__, ret);
		return;
	}
	ct_info("%s %d\n", __func__, di->remaining_duration);
	power_supply_changed(psy);
}

static void huawei_chg_time_adjust_by_current(struct huawei_chg_time_device *di)
{
	int i;
	int ref_current, delt_ma;
	int compensation_time, durations;

	if (!di->param_dts.adjust_by_current)
		return;

	if (is_direct_charge(di->chg_info.charge_type)) {
		i = di->batt_info.soc;
		durations = di->remaining_duration;

		ref_current = di->batt_info.fcc * SEC_PRE_HOUR /
			di->ref_curve->step_time[i] / FULL_SOC;
		delt_ma = di->batt_info.curr_ma - ref_current;
		/* pre 500mA add 1min */
		compensation_time = delt_ma / 500 * 60;
		durations -= compensation_time;

		ct_info("%s fcc %dmAh,time[%d] %d,cur %dmA\n", __func__,
			di->batt_info.fcc, i, di->ref_curve->step_time[i], ref_current);
		ct_info("%s cur %d,ref %d, delt %dmA, comp %d, dur %d\n", __func__,
			di->batt_info.curr_ma, ref_current, delt_ma, compensation_time,
			durations);
	}
}

static void huawei_chg_time_adjust_by_fcc(struct huawei_chg_time_device *di)
{
	int delt_time;
	int remaining_duration = di->remaining_duration;
	int delt_fcc = di->batt_info.fcc - di->ref_curve->fcc;

	if (!di->param_dts.adjust_by_fcc)
		return;

	if (di->batt_info.soc > ADJUST_FCC_SOC)
		return;

	delt_time = delt_fcc * SEC_PRE_HOUR / ADJUST_FCC_CURRENT;

	remaining_duration += delt_time;
	ct_dbg("%s fcc %d,soc %d,ref %d,delt %d,delt T %d,dur %d,duration %d\n",
		__func__, di->batt_info.fcc, di->batt_info.soc, di->ref_curve->fcc,
		delt_fcc, delt_time, di->remaining_duration, remaining_duration);

	di->remaining_duration = remaining_duration;
}

static void huawei_chg_time_adjust_by_temperature(
	struct huawei_chg_time_device *di)
{
	if (!di->param_dts.adjust_by_temp)
		return;
}

static void huawei_chg_time_adjust_by_volt(struct huawei_chg_time_device *di)
{
	if (!di->param_dts.adjust_by_volt)
		return;
}

static int huawei_chg_time_step_check(struct huawei_chg_time_device *di, int soc)
{
	int pre_step, pos_step;
	int cur_step = di->ref_curve->step_time[soc];

	/* step time is normal */
	if (cur_step < STEP_TIME_MAX || soc >= SOC_FULL || soc <= 0)
		return cur_step;

	pre_step = di->ref_curve->step_time[soc - 1];
	pos_step = di->ref_curve->step_time[soc + 1];

	if (pre_step > 0 && pre_step < STEP_TIME_MAX &&
			pos_step > 0 && pos_step < STEP_TIME_MAX)
		cur_step = (pre_step + pos_step) / 2;

	ct_warn("%s soc %d, ori step %d, cur step %d\n", __func__,
			soc, di->ref_curve->step_time[soc], cur_step);

	return cur_step;
}

static void huawei_chg_time_calc_param(struct huawei_chg_time_device *di)
{
	int i;
	int durations = 0;
	int cur_step;

	ct_dbg("%s  +\n", __func__);

	di->pre_remaining_duration_with_valid = di->remaining_duration_with_valid;

	if (!di->ref_curve) {
		ct_info("%s no reference curve\n", __func__);
		di->remaining_duration = INVALID;
		di->remaining_duration_with_valid = INVALID;
		goto out;
	}

	for (i = FULL_SOC; i > di->batt_info.soc; i--) {
		if (di->ref_curve->step_time[i] <= 0) {
			di->remaining_duration = INVALID;
			ct_info("%s durations %d, i %d, time %d\n", __func__,
				di->remaining_duration, i, di->ref_curve->step_time[i]);
			goto out;
		} else {
			cur_step = huawei_chg_time_step_check(di, i);
			ct_info("%s durations %d, step_time[%d] = %d\n",
				__func__, durations, i, cur_step);
			durations += cur_step;
		}
	}
	di->remaining_duration = durations;

	huawei_chg_time_adjust_by_volt(di);
	huawei_chg_time_adjust_by_current(di);
	huawei_chg_time_adjust_by_fcc(di);
	huawei_chg_time_adjust_by_temperature(di);

	/* Add valid flag for framework to check */
	di->remaining_duration_with_valid =
		(int)((unsigned int)di->remaining_duration | VALID_FLAG);

out:
	ct_info("%s durations sec:%d, code:0x%x\n", __func__,
		di->remaining_duration, di->remaining_duration_with_valid);

	huawei_chg_time_update(di);

	ct_dbg("%s  -\n", __func__);
}

static void huawei_chg_time_get_charger_type(struct huawei_chg_time_device *di)
{
	enum charge_type_enum charge_type = CHG_OTHERS;
	unsigned int type = power_platform_get_charger_type();

	ct_dbg("%s type %d\n", __func__, type);

	if (direct_charge_get_super_charging_flag()) {
		/*
		 * when direct charger change to buck charger, flag still be true
		 */
		ct_dbg("%s supre charger succ\n", __func__);
		if (di->chg_info.direct_charge_type == SC_MODE) {
			charge_type = CHG_SC;
			di->chg_info.direct_charge_done = false;
		} else if (di->chg_info.direct_charge_type == LVC_MODE) {
			charge_type = CHG_LVC;
			di->chg_info.direct_charge_done = false;
		} else {
			/*
			 * when direct charger done and change to buck charger,
			 * charge type still be direct charger.
			 */
			ct_dbg("%s direct charger done, charge type %d\n", __func__,
				di->chg_info.charge_type);
			di->chg_info.direct_charge_done = true;
			return;
		}
	} else {
		if (type == CHARGER_TYPE_STANDARD)
			charge_type = CHG_5V2A;
		else if (type == CHARGER_TYPE_USB)
			charge_type = CHG_USB;
		else if (type == CHARGER_TYPE_FCP)
			charge_type = CHG_FCP;
		else if (type == CHARGER_REMOVED)
			charge_type = CHG_NONE;
		else
			charge_type = CHG_OTHERS;
	}

	if (charge_type != di->chg_info.charge_type) {
		ct_info("%s type %d is %s,pre charge_type %s[%d]\n", __func__, type,
			charge_type_str_enum[charge_type],
			charge_type_str_enum[di->chg_info.charge_type],
			di->chg_info.charge_type);

		di->charge_type_change = 1;
	}

	di->chg_info.charge_type = charge_type;
}

static void huawei_chg_time_get_charge_info(struct huawei_chg_time_device *di)
{
	int ret;
	bool pre_direct_charger_done = di->chg_info.direct_charge_done;

	huawei_chg_time_get_charger_type(di);

	if (is_direct_charge(di->chg_info.charge_type)) {
		ret = direct_charge_get_info(CC_CABLE_DETECT_OK,
			&di->chg_info.cc_cable_detect_ok);
		if (ret) {
			ct_err("%s, get cc_cable_detect fail %d\n", __func__, ret);
			di->chg_info.cc_cable_detect_ok = 1;
		}

		ct_info("%s, get cc_cable_detect_ok is %d\n", __func__,
			di->chg_info.cc_cable_detect_ok);

		if (pre_direct_charger_done == false &&
			di->chg_info.direct_charge_done == true) {
			di->chg_info.direct_charge_done_soc = di->batt_info.soc;
			ct_dbg("%s, direct charge done at %d\n", __func__,
				di->batt_info.soc);
		}
	}

	ct_dbg("%s charge_type %d\n", __func__, di->chg_info.charge_type);
}

static void huawei_chg_time_get_batt_info(struct huawei_chg_time_device *di)
{
	union power_supply_propval value;
	struct power_supply *psy = NULL;
	int soc;

	psy = power_supply_get_by_name(BATTERY_NAME);
	if (psy == NULL)
		return;

	/* percentage never be NULL because of input parameters check */
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value))
		soc = value.intval;
	if (di->test_flag)
		di->batt_info.soc = huawei_chg_time_test_soc();
	else
		di->batt_info.soc = soc;

	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL, &value))
		di->batt_info.fcc = value.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value))
		di->batt_info.volt_mv = value.intval / MICRO2MINI;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_AVG, &value))
		di->batt_info.curr_ma = value.intval / MICRO2MINI;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &value))
		di->batt_info.batt_temp = value.intval / 10;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CYCLE_COUNT, &value))
		di->batt_info.batt_cycles = value.intval;

	ct_dbg("%s, fcc %d, soc %d, vol %d, cur %d, temp %d, cyc %d\n", __func__,
		di->batt_info.fcc, di->batt_info.soc, di->batt_info.volt_mv,
		di->batt_info.curr_ma, di->batt_info.batt_temp,
		di->batt_info.batt_cycles);
	/* soc check */
	if (di->batt_info.soc < 0 || di->batt_info.soc > FULL_SOC)
		di->batt_info.soc = 0;
}

static void huawei_charge_time_calc_work(struct work_struct *work)
{
	int ret;
	struct huawei_chg_time_device *di = container_of(work,
		struct huawei_chg_time_device, charge_time_work.work);
	u64 timestemp = get_current_time();

	di->work_running = 1;
	di->remaining_duration_with_valid = 0;

	huawei_chg_time_get_charge_info(di);
	huawei_chg_time_get_batt_info(di);
	ct_dbg("%s, pre soc :%d, soc %d\n", __func__, di->pre_capacity,
		di->batt_info.soc);

	if (di->charge_type_change) {
		di->start_capacity = di->batt_info.soc;
		di->pre_timestemp = timestemp;
		di->start_timestemp = timestemp;
		di->remaining_duration = 0;

		ret = huawei_chg_time_read_param_from_flash(di);
		if (ret < 0)
			goto next;

		/* reset current curve */
		if (memset_s(&di->cur_curve, sizeof(struct huawei_chg_time_param), 0,
			sizeof(struct huawei_chg_time_param)) != EOK)
			ct_err("%s():%d:memset_s fail!\n", __func__, __LINE__);

		ct_info("%s charge type change: soc %d, timestep %ld\n", __func__,
			di->batt_info.soc, di->start_timestemp);
	}

	if (di->chg_info.charge_type == CHG_NONE ||
		di->chg_info.charge_type == CHG_USB) {
		di->work_running = 0;
		ct_info("%s, charge type is usb/none, exit work!\n", __func__);
		return;
	}

	ret = huawei_chg_time_param_select(di, di->chg_info.charge_type);
	if (ret) {
		di->work_running = 0;
		return;
	}

	huawei_chg_time_self_study(di);
	huawei_chg_time_calc_param(di);
	di->pre_capacity = di->batt_info.soc;

	di->charge_type_change = 0;
	if (is_charge_full(di)) {
		di->work_running = 0;
		return;
	}
	ct_dbg("%s -\n", __func__);
next:
	queue_delayed_work(system_power_efficient_wq, &di->charge_time_work,
		msecs_to_jiffies(CHG_TYPE_DETECT_TIME));
}

static int huawei_charger_event_rcv(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct huawei_chg_time_device *di =
		container_of(nb, struct huawei_chg_time_device, nb);

	switch (event) {
	case POWER_NE_START_CHARGING:
		ct_info("%s event: POWER_NE_START_CHARGING\n", __func__);
		if (di->work_running)
			return 0;
		di->work_running = 1;
		queue_delayed_work(system_power_efficient_wq, &di->charge_time_work,
			msecs_to_jiffies(1000));
		break;
	case POWER_NE_STOP_CHARGING:
		ct_info("%s event: POWER_NE_STOP_CHARGING\n", __func__);
		di->work_running = 0;
		di->remaining_duration = 0;
		break;
	case POWER_NE_SUSPEND_CHARGING:
		ct_info("%s event: POWER_NE_SUSPEND_CHARGING\n", __func__);
		di->remaining_duration = 0;
		di->work_running = 0;
		break;
	default:
		ct_err("%s default run.\n", __func__);
		break;
	}

	return 0;
}
static int huawei_chg_time_dc_status_notifier_call(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct huawei_chg_time_device *di =
		container_of(nb, struct huawei_chg_time_device, direct_charger_nb);

	if (!di) {
		ct_err("%s di is null\n", __func__);
		return NOTIFY_OK;
	}
	switch (event) {
	case POWER_NE_DC_LVC_CHARGING:
		di->chg_info.direct_charge_type = LVC_MODE;
		break;
	case POWER_NE_DC_SC_CHARGING:
		di->chg_info.direct_charge_type = SC_MODE;
		break;
	case POWER_NE_DC_STOP_CHARGE:
	default:
		di->chg_info.direct_charge_type = UNDEFINED_MODE;
		break;
	}
	ct_info("%s direct charge type is %d\n", __func__,
		di->chg_info.direct_charge_type);

	return NOTIFY_OK;
}

static void parse_dts(struct device_node *np, struct huawei_chg_time_device *di)
{
	int ret;

	ret = of_property_read_u32(np, "adjust_by_volt",
		(u32 *)&(di->param_dts.adjust_by_volt));
	if (ret)
		ct_err("get adjust_by_volt fail\n");

	ret = of_property_read_u32(np, "adjust_by_current",
		(u32 *)&(di->param_dts.adjust_by_current));
	if (ret)
		ct_err("get adjust_by_current fail\n");

	ret = of_property_read_u32(np, "adjust_by_fcc",
		(u32 *)&(di->param_dts.adjust_by_fcc));
	if (ret)
		ct_err("get adjust_by_fcc fail\n");

	ret = of_property_read_u32(np, "adjust_by_temp",
		(u32 *)&(di->param_dts.adjust_by_temp));
	if (ret)
		ct_err("get adjust_by_temp fail\n");
}

static int huawei_chg_time_probe(struct platform_device *pdev)
{
	struct huawei_chg_time_device *di = NULL;
	struct device_node *np = NULL;
	int ret;

	di = (struct huawei_chg_time_device *)devm_kzalloc(&pdev->dev, sizeof(*di),
		GFP_KERNEL);
	if (!di)
		return -1;

	di->dev = &pdev->dev;
	platform_set_drvdata(pdev, di);
	np = di->dev->of_node;

	parse_dts(np, di);

	di->nb.notifier_call = huawei_charger_event_rcv;
	ret = power_event_bnc_register(POWER_BNT_CHARGING, &di->nb);
	if (ret)
		goto event_err;

	di->direct_charger_nb.notifier_call =
		huawei_chg_time_dc_status_notifier_call;
	ret = power_event_bnc_register(POWER_BNT_DC, &di->direct_charger_nb);
	if (ret) {
		ct_err("%s register notifier failed\n", __func__);
		goto nf_err;
	}

	INIT_DELAYED_WORK(&di->charge_time_work, huawei_charge_time_calc_work);

	ret = huawei_chg_time_debug_init(di);
	if (ret) {
		ct_err("failed to create file\n");
		goto dbg_err;
	}

	g_huawei_chg_time_info = di;

	ct_err("%s succ\n", __func__);
	return 0;

dbg_err:
	ret = power_event_bnc_unregister(POWER_BNT_DC, &di->direct_charger_nb);
	if (ret)
		ct_err("%s direct charge unregister fail\n", __func__);
nf_err:
	ret = power_event_bnc_unregister(POWER_BNT_CHARGING, &di->nb);
	if (ret)
		ct_err("%s huawei_unregister_notifier fail\n", __func__);
event_err:
	return -1;
}

static int huawei_chg_time_remove(struct platform_device *pdev)
{
	struct huawei_chg_time_device *di = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, di);
	g_huawei_chg_time_info = NULL;

	return 0;
}

static const struct of_device_id huawei_chg_time_of_match[] = {
	{.compatible = "huawei,charger_time", .data = NULL}, {},
};

MODULE_DEVICE_TABLE(of, huawei_chg_time_of_match);

static struct platform_driver huawei_chg_time_driver = {
	.probe = huawei_chg_time_probe,
	.remove = huawei_chg_time_remove,
	.driver = {
		.name = "charger-time",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(huawei_chg_time_of_match),
	},
};

static int __init huawei_chg_time_init(void)
{
	return platform_driver_register(&huawei_chg_time_driver);
}

static void __exit huawei_chg_time_exit(void)
{
	platform_driver_unregister(&huawei_chg_time_driver);
}
late_initcall(huawei_chg_time_init);
module_exit(huawei_chg_time_exit);

MODULE_DESCRIPTION("charger time remaining driver");
MODULE_LICENSE("GPL v2");
