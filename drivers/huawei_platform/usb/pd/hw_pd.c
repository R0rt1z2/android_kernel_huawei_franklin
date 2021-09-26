/************************************************************
*
* Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
* FileName: hw_typec.c
* Author: suoandajie(00318894)       Version : 0.1      Date:  2016-5-9
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*  Description:    .c file for type-c core layer which is used to handle
*                  pulic logic management for different chips and to
*                  provide interfaces for exteranl modules.
*  Version:
*  Function List:
*  History:
*  <author>  <time>   <version >   <desc>
***********************************************************/

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/usb/class-dual-role.h>
#include <linux/version.h>
#include <ana_hs_kit/ana_hs.h>
#include <chipset_common/hwpower/common_module/power_sysfs.h>
#include <chipset_common/hwpower/hardware_channel/vbus_channel.h>
#include <chipset_common/hwpower/hardware_ic/boost_5v.h>
#include <chipset_common/hwpower/hardware_monitor/water_detect.h>
#include <huawei_platform/audio/ana_hs_extern_ops.h>
#include <huawei_platform/charger/huawei_charger.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/power/direct_charger/direct_charger.h>
#include <huawei_platform/usb/hw_pd_dev.h>
#include <huawei_platform/usb/switch/switch_fsa9685.h>
#include <tcpci_typec.h>
#include <tcpm.h>
#ifdef CONFIG_TCPC_HI6526
#include <huawei_platform/usb/hi6526_tcpc_ops.h>
#endif

static int g_otg_channel;
#define CC_SHORT_DEBOUNCE 50 /* ms */
#define QUICK_CHARGE_ICON_THRESHOLD 18000000 /* mw */

struct pd_dpm_info *g_pd_di;
static bool g_pd_cc_orientation;
static int g_pd_cc_orientation_factory;
static bool g_pd_optional_max_power_status;
#ifdef CONFIG_TCPC_HI6526
static bool g_pd_high_voltage_charging_status;
#endif
static bool g_pd_quick_charger_flag;
static struct device *typec_dev;
static int g_pd_dpm_typec_state = PD_DPM_USB_TYPEC_DETACHED;

struct cc_check_ops* g_cc_check_ops;
static struct pd_dpm_ops *g_ops;

static void *g_client;
static unsigned int abnormal_cc_detection;
static unsigned int abnormal_cc_interval = PD_DPM_CC_CHANGE_INTERVAL;
int support_dp = 1;
int moisture_detection_by_cc_enable;
static unsigned int g_cc_abnormal_dmd_report_enable;
int support_analog_audio = 1;
static bool g_pd_cc_moisture_status;
static bool g_pd_cc_moisture_happened;
static int cc_moisture_status_report;
/* 0: disabled 1: only for SC; 2: for SC and LVC */
static unsigned int cc_dynamic_protect;
static struct delayed_work cc_short_work;

/* for audio analog hs drivers to check */
static int pd_dpm_analog_hs_state;

void pd_dpm_set_cc_mode(int mode);

#define PD_DPM_MAX_OCP_COUNT 1000
#define OCP_DELAY_TIME 5000
#define DISABLE_INTERVAL 50
#define GET_IBUS_INTERVAL 1000
#define MMI_PD_TIMES 3
#define MMI_PD_IBUS_MIN 300

#ifndef HWLOG_TAG
#define HWLOG_TAG huawei_pd
HWLOG_REGIST();
#endif

int g_cc1_short_flag = 0;
int g_cc2_short_flag = 0;

void mtk_set_cc1_cc2_status(bool cc1_status, bool cc2_status)
{
	g_cc1_short_flag = (int)cc1_status;
	g_cc2_short_flag = (int)cc2_status;
}

void mtk_get_cc1_cc2_status(int *cc1_status, int *cc2_status)
{
	*cc1_status = g_cc1_short_flag;
	*cc2_status = g_cc2_short_flag;
}

static struct abnomal_change_info abnomal_change[] = {
	{PD_DPM_ABNORMAL_CC_CHANGE, true, 0, 0, {0}, {0}, {0}, {0}},
	{PD_DPM_UNATTACHED_VBUS_ONLY, true, 0, 0, {0}, {0}, {0}, {0}},
};

static void pd_dpm_cc_moisture_flag_restore(struct work_struct *work)
{
	hwlog_err("%s %d,%d\n", __func__,
		  g_pd_cc_moisture_happened,
		  g_pd_cc_moisture_status);
	if (!g_pd_cc_moisture_happened)
		g_pd_cc_moisture_status = false;
}

static int pd_dpm_handle_fb_event(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct fb_event *fb_event = data;
	int *blank = fb_event->data;

	switch (event) {
	case FB_EVENT_BLANK:
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			if ((g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_NONE) ||
				(g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_DETACHED) ||
				(g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_AUDIO_DETACHED)) {
				hwlog_err("%s set pd to drp\n", __func__);
				pd_dpm_set_cc_mode(PD_DPM_CC_MODE_DRP);
			}

			if (cc_moisture_status_report == 0)
				break;

			g_pd_cc_moisture_happened = false;
			/* 120000: delay 120s to restore the flag */
			queue_delayed_work(g_pd_di->usb_wq,
				&g_pd_di->cc_moisture_flag_restore_work,
				msecs_to_jiffies(120000));
			break;
		case FB_BLANK_POWERDOWN:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block pd_dpm_handle_fb_notifier = {
	.notifier_call = pd_dpm_handle_fb_event,
};

static void init_fb_notification(void)
{
	fb_register_client(&pd_dpm_handle_fb_notifier);
}

int pd_dpm_ops_register(struct pd_dpm_ops *ops, void *client)
{
	int ret = 0;

	if (g_ops) {
		hwlog_err("pd_dpm ops register fail! g_ops busy\n");
		return -EBUSY;
	}

	if (ops != NULL) {
		g_ops = ops;
		g_client = client;
	} else {
		hwlog_err("pd_dpm ops register fail!\n");
		ret = -EPERM;
	}
	return ret;
}

int pd_dpm_disable_pd(bool disable)
{
	hwlog_info("%s\n", __func__);

	if (!g_ops) {
		hwlog_err("%s g_ops is NULL\n", __func__);
		return -EPERM;
	}

	if (!g_ops->pd_dpm_disable_pd) {
		hwlog_err("%s pd_dpm_disable_pd is NULL\n", __func__);
		return -EPERM;
	}

	return g_ops->pd_dpm_disable_pd(g_client, disable);
}

void pd_dpm_set_cc_mode(int mode)
{
	static int cur_mode = PD_DPM_CC_MODE_DRP;

	hwlog_info("%s cur_mode = %d, new mode = %d\n",
		__func__, cur_mode, mode);
	if (!g_ops || !g_ops->pd_dpm_set_cc_mode || cur_mode == mode)
		return;

	g_ops->pd_dpm_set_cc_mode(mode);
	cur_mode = mode;
}

/*
 * bugfix: For Hi65xx
 * DRP lost Cable(without adapter) plugin during Wireless.
 */
void pd_pdm_enable_drp(void)
{
	if (g_ops && g_ops->pd_dpm_enable_drp)
		g_ops->pd_dpm_enable_drp(PD_PDM_RE_ENABLE_DRP);
}

void pd_dpm_reinit_chip(void)
{
	if (g_ops && g_ops->pd_dpm_reinit_chip)
		g_ops->pd_dpm_reinit_chip(g_client);
}

bool pd_dpm_get_hw_dock_svid_exist(void)
{
	if (g_ops && g_ops->pd_dpm_get_hw_dock_svid_exist)
		return g_ops->pd_dpm_get_hw_dock_svid_exist(g_client);

	return false;
}
int pd_dpm_notify_direct_charge_status(bool dc)
{
	hwlog_err("%s,%d", __func__, __LINE__);
	if (g_ops && g_ops->pd_dpm_notify_direct_charge_status) {
		hwlog_err("%s,%d", __func__, __LINE__);
		return g_ops->pd_dpm_notify_direct_charge_status(g_client, dc);
	}

	return false;
}

bool pd_dpm_get_pd_finish_flag(void)
{
	if (g_pd_di)
		return g_pd_di->pd_finish_flag;

	return false;
}

int cc_check_ops_register(struct cc_check_ops* ops)
{
	int ret = 0;

	if (g_cc_check_ops) {
		hwlog_err("cc_check ops register fail! g_cc_check_ops busy\n");
		return -EBUSY;
	}

	if (ops != NULL)
	{
		g_cc_check_ops = ops;
	}
	else
	{
		hwlog_err("cc_check ops register fail!\n");
		ret = -EPERM;
	}
	return ret;
}

static int direct_charge_cable_detect(void)
{
	if (!g_pd_di)
		return -1;

	if (g_pd_di->cur_typec_state != PD_DPM_TYPEC_ATTACHED_CUSTOM_SRC)
		return -1;

	return 0;
}

static struct dc_cable_ops cable_detect_ops = {
	.detect = direct_charge_cable_detect,
};
bool pd_dpm_get_cc_orientation(void)
{
	hwlog_info("%s cc_orientation =%d\n", __func__, g_pd_cc_orientation);
	return g_pd_cc_orientation;
}

static void pd_dpm_set_cc_orientation(bool cc_orientation)
{
	hwlog_info("%s cc_orientation =%d\n", __func__, cc_orientation);
	g_pd_cc_orientation = cc_orientation;
}

void pd_dpm_get_typec_state(int *typec_state)
{
	hwlog_info("%s = %d\n", __func__, g_pd_dpm_typec_state);
	*typec_state = g_pd_dpm_typec_state;
}

/* for analog audio driver polling */
static void pd_dpm_set_typec_state(int typec_state)
{
	hwlog_info("%s typec_state = %d\n", __func__, typec_state);
	g_pd_dpm_typec_state = typec_state;

	if ((g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_NONE) ||
		(g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_DETACHED) ||
		(g_pd_dpm_typec_state == PD_DPM_USB_TYPEC_AUDIO_DETACHED)) {
		hwlog_info("%s report detach, start res detect\n", __func__);
		ana_hs_fsa4480_stop_ovp_detect(ANA_HS_TYPEC_DEVICE_DETACHED);
		ana_hs_fsa4480_start_res_detect(ANA_HS_TYPEC_DEVICE_DETACHED);
	} else {
		hwlog_info("%s report attach, stop res detect\n", __func__);
		ana_hs_fsa4480_stop_res_detect(ANA_HS_TYPEC_DEVICE_ATTACHED);
		ana_hs_fsa4480_start_ovp_detect(ANA_HS_TYPEC_DEVICE_ATTACHED);
	}
}

static ssize_t pd_dpm_cc_orientation_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	/* 3 means cc is abnormally grounded */
	if (g_pd_cc_orientation_factory == CC_ORIENTATION_FACTORY_SET)
		return scnprintf(buf, PAGE_SIZE, "%s\n", "3");

	return scnprintf(buf, PAGE_SIZE, "%s\n", pd_dpm_get_cc_orientation()? "2" : "1");
}

static ssize_t pd_dpm_pd_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int retrys = 0;

	hwlog_info("%s  = %d\n", __func__, pd_dpm_get_pd_finish_flag());
	if (pd_dpm_get_pd_finish_flag()) {
		while (retrys++ < MMI_PD_TIMES) {
			hwlog_info("%s, ibus = %d\n", __func__,
				get_charger_ibus_curr());
			if (get_charger_ibus_curr() >= MMI_PD_IBUS_MIN)
				return scnprintf(buf, PAGE_SIZE, "%s\n", "0");
			msleep(GET_IBUS_INTERVAL);
		}
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", "1");
}

static ssize_t pd_dpm_cc_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int ret;
	unsigned int cc1 = 0;
	unsigned int cc2 = 0;
	unsigned int cc = 0;

	ret = pd_dpm_get_cc_state_type(&cc1, &cc2);
	if (ret == 0)
		cc = ((cc1 & PD_DPM_CC_STATUS_MASK) |
			(cc2 << PD_DPM_CC2_STATUS_OFFSET)) &
			PD_DPM_BOTH_CC_STATUS_MASK;

	return scnprintf(buf, PAGE_SIZE, "%d\n", cc);
}

static DEVICE_ATTR(cc_orientation, S_IRUGO, pd_dpm_cc_orientation_show, NULL);
static DEVICE_ATTR(pd_state, S_IRUGO, pd_dpm_pd_state_show, NULL);
static DEVICE_ATTR(cc_state, S_IRUGO, pd_dpm_cc_state_show, NULL);
static struct attribute *pd_dpm_ctrl_attributes[] = {
	&dev_attr_cc_orientation.attr,
	&dev_attr_pd_state.attr,
	&dev_attr_cc_state.attr,
	NULL,
};

static const struct attribute_group pd_dpm_attr_group = {
	.attrs = pd_dpm_ctrl_attributes,
};

static struct device *pd_dpm_create_group(void)
{
	return power_sysfs_create_group("hw_typec", "typec",
		&pd_dpm_attr_group);
}

bool pd_dpm_get_optional_max_power_status(void)
{
	hwlog_info("%s status =%d\n", __func__, g_pd_optional_max_power_status);
	return g_pd_optional_max_power_status;
}

void pd_dpm_set_optional_max_power_status(bool status)
{
	hwlog_info("%s status =%d\n", __func__, status);
	g_pd_optional_max_power_status = status;
}

bool pd_dpm_get_high_voltage_charging_status(void)
{
#ifdef CONFIG_TCPC_HI6526
	hwlog_info("%s status = %d\n", __func__,
		   g_pd_high_voltage_charging_status);
	return g_pd_high_voltage_charging_status;
#else
	return false;
#endif
}

void pd_dpm_set_high_voltage_charging_status(bool status)
{
#ifdef CONFIG_TCPC_HI6526
	hwlog_info("%s status = %d\n", __func__, status);
	g_pd_high_voltage_charging_status = status;
#endif
}

bool pd_dpm_get_ctc_cable_flag(void)
{
	if (g_pd_di)
		return g_pd_di->ctc_cable_flag;

	return false;
}

bool pd_dpm_check_cc_vbus_short(void)
{
	int cc1_status = 0;
	int cc2_status = 0;

#ifdef CONFIG_TCPC_HI6526
	/*
	 * Get result from chip module directly,
	 * Only for HI6526_PD at present.
	 */
	if (g_ops && g_ops->pd_dpm_check_cc_vbus_short) {
		hwlog_info("Hi6526 PD\n");
		return g_ops->pd_dpm_check_cc_vbus_short();
	}
#endif

	mtk_get_cc1_cc2_status(&cc1_status, &cc2_status);

	hwlog_info("%s: cc1:%d, cc2:%d\n", __func__, cc1_status, cc2_status);

	if (cc1_status || cc2_status)
		return true;

	return false;
}

/*
 * Function:       pd_dpm_get_cc_state_type
 * Description:   get cc1 and cc2 state
 *                             open    56k    22k    10k
 *                     cc1    00       01     10     11
 *                     cc2    00       01     10     11
 * Input:           cc1: value of cc1  cc2: value of cc2
 * Output:         cc1: value of cc1  cc2: value of cc2
 * Return:         success: 0   fail: -1
*/
int pd_dpm_get_cc_state_type(unsigned int *cc1, unsigned int *cc2)
{
	unsigned int cc;

	if (!g_ops || !g_ops->pd_dpm_get_cc_state)
		return -1;

	cc = g_ops->pd_dpm_get_cc_state();
	*cc1 = cc & PD_DPM_CC_STATUS_MASK;
	*cc2 = (cc >> PD_DPM_CC2_STATUS_OFFSET) & PD_DPM_CC_STATUS_MASK;

	return 0;
}

bool pd_dpm_get_cc_moisture_status(void)
{
	return g_pd_cc_moisture_status;
}

void pd_dpm_handle_abnomal_change(int event)
{
	int i;
	char dsm_buf[PD_DPM_CC_DMD_BUF_SIZE] = {0};
	int time_diff;
	unsigned int time_diff_index;
	unsigned int flag;
	struct timespec64 ts64_interval;
	struct timespec64 ts64_now;
	struct timespec64 ts64_sum;

	struct timespec64 ts64_dmd_interval;
	struct timespec64 ts64_dmd_now;
	struct timespec64 ts64_dmd_sum;

	int* change_counter = &abnomal_change[event].change_counter;
	int change_counter_threshold = PD_DPM_CC_CHANGE_COUNTER_THRESHOLD;
	int* dmd_counter = &abnomal_change[event].dmd_counter;
	int dmd_counter_threshold = PD_DPM_CC_DMD_COUNTER_THRESHOLD;
	ts64_interval.tv_sec = 0;
	ts64_interval.tv_nsec = abnormal_cc_interval * NSEC_PER_MSEC;
	ts64_dmd_interval.tv_sec = PD_DPM_CC_DMD_INTERVAL;
	ts64_dmd_interval.tv_nsec = 0;

	ts64_now = current_kernel_time64();
	if (abnomal_change[event].first_enter) {
		abnomal_change[event].first_enter = false;
	} else {
		ts64_sum = timespec64_add_safe(abnomal_change[event].ts64_last, ts64_interval);
		if (ts64_sum.tv_sec == TIME_T_MAX) {
			hwlog_err("%s time overflow happend\n",__func__);
			*change_counter = 0;
		} else if (timespec64_compare(&ts64_sum, &ts64_now) >= 0) {
			++*change_counter;
			hwlog_info("%s change_counter = %d,\n",__func__, *change_counter);

			time_diff = (ts64_now.tv_sec - abnomal_change[event].ts64_last.tv_sec) *
				PD_DPM_CC_CHANGE_MSEC +
				(ts64_now.tv_nsec - abnomal_change[event].ts64_last.tv_nsec) /
				NSEC_PER_MSEC;
			time_diff_index = time_diff /
				(PD_DPM_CC_CHANGE_INTERVAL / PD_DPM_CC_CHANGE_BUF_SIZE);
			if (time_diff_index >= PD_DPM_CC_CHANGE_BUF_SIZE)
				time_diff_index = PD_DPM_CC_CHANGE_BUF_SIZE - 1;
			++abnomal_change[event].change_data[time_diff_index];
		} else {
			*change_counter = 0;
			memset(abnomal_change[event].change_data, 0, PD_DPM_CC_CHANGE_BUF_SIZE);
		}
	}

	if (*change_counter >= change_counter_threshold) {
		hwlog_err("%s change_counter hit\n",__func__);

		if (cc_moisture_status_report) {
			g_pd_cc_moisture_happened = true;
			g_pd_cc_moisture_status = true;
		}

		pd_dpm_set_cc_mode(PD_DPM_CC_MODE_UFP);

		for (i = 0; i < PD_DPM_CC_CHANGE_BUF_SIZE; i++)
			abnomal_change[event].dmd_data[i] += abnomal_change[event].change_data[i];
		*change_counter = 0;
		memset(abnomal_change[event].change_data, 0, PD_DPM_CC_CHANGE_BUF_SIZE);
		++*dmd_counter;

		if (moisture_detection_by_cc_enable) {
			hwlog_err("%s moisture_detected\n",__func__);
			flag = WD_NON_STBY_MOIST;
			power_event_bnc_notify(POWER_BNT_WD, POWER_NE_WD_REPORT_UEVENT, &flag);
		}
	}

	if ((*dmd_counter >= dmd_counter_threshold) && g_cc_abnormal_dmd_report_enable) {
		*dmd_counter = 0;

		ts64_dmd_now = current_kernel_time64();
		ts64_dmd_sum = timespec64_add_safe(abnomal_change[event].ts64_dmd_last, ts64_dmd_interval);
		if (ts64_dmd_sum.tv_sec == TIME_T_MAX) {
			hwlog_err("%s time overflow happend when add 24 hours\n",__func__);
		} else if (timespec64_compare(&ts64_dmd_sum, &ts64_dmd_now) < 0) {
			snprintf(dsm_buf, PD_DPM_CC_DMD_BUF_SIZE - 1, "cc abnormal is triggered:");
			for (i = 0; i < PD_DPM_CC_CHANGE_BUF_SIZE; i++)
				snprintf(dsm_buf + strlen(dsm_buf), PD_DPM_CC_DMD_BUF_SIZE - 1, " %d", abnomal_change[event].dmd_data[i]);
			snprintf(dsm_buf + strlen(dsm_buf), PD_DPM_CC_DMD_BUF_SIZE - 1, "\n");

			power_dsm_report_dmd(POWER_DSM_BATTERY, ERROR_NO_WATER_CHECK_IN_USB, dsm_buf);

			abnomal_change[event].ts64_dmd_last = ts64_dmd_now;
		} else {
			hwlog_info("error: cc abnormal happend within 24 hour, do not report\n");
		}
	}

	abnomal_change[event].ts64_last =  ts64_now;
}

enum cur_cap pd_dpm_get_cvdo_cur_cap(void)
{
	enum cur_cap cap;

	cap = (g_pd_di->cable_vdo & CABLE_CUR_CAP_MASK) >> CABLE_CUR_CAP_SHIFT;
	hwlog_info("%s, cur_cap = %d\n", __func__, cap);

	return cap;
}

static bool pd_dpm_is_cc_protection(void)
{
	/* cc_dynamic_protect-0: disabled 1: only SC; 2: for SC and LVC */
	if (!cc_dynamic_protect)
		return false;

#ifdef CONFIG_DIRECT_CHARGER
	if (direct_charge_in_charging_stage() != DC_IN_CHARGING_STAGE)
		return false;
#endif
	if (!pd_dpm_check_cc_vbus_short())
		return false;

	hwlog_info("cc short\n");
	return true;
}

static void pd_dpm_cc_short_action(void)
{
#ifdef CONFIG_DIRECT_CHARGER
	unsigned int notifier_type;
	int mode = direct_charge_get_working_mode();

	/* cc_dynamic_protect-0: disabled 1: only SC; 2: for SC and LVC */
	if (mode == SC_MODE)
		notifier_type = POWER_ANT_SC_FAULT;
	else if (mode == LVC_MODE && cc_dynamic_protect == 2)
		notifier_type = POWER_ANT_LVC_FAULT;

	power_event_anc_notify(notifier_type, POWER_NE_DC_FAULT_CC_SHORT, NULL);
#endif /* CONFIG_DIRECT_CHARGER */
	hwlog_info("cc_short_action\n");
}

static void pd_dpm_cc_short_work(struct work_struct *work)
{
	hwlog_info("cc_short_work\n");
	if (!pd_dpm_is_cc_protection())
		return;

	pd_dpm_cc_short_action();
}

void pd_dpm_cc_dynamic_protect(void)
{
	if (!pd_dpm_is_cc_protection())
		return;

	mod_delayed_work(system_wq, &cc_short_work,
		msecs_to_jiffies(CC_SHORT_DEBOUNCE));
}

#define VBUS_VOL_9000MV 9000
#define PD_POWER_CHARGE_18W 18000000
static void pd_dpm_sink_voltage_status_update(struct pd_dpm_info *di,
	const struct pd_dpm_vbus_state *vbus_state)
{
#ifdef CONFIG_TCPC_HI6526
	bool high_voltage_charging = false;
	bool high_power_charging = false;

	/* only support Hi6526 PD */
	if (!hi6526_get_tcpc_dev())
		return;

	if (vbus_state->mv == 0) {
		if (di->pd_finish_flag) {
			hwlog_info("%s : Disable\n", __func__);
			pd_dpm_set_high_voltage_charging_status(false);
		}
		return;
	}

	di->pd_source_vbus = false;

	hwlog_info("%s : Sink %d mV, %d mA\n", __func__,
		   vbus_state->mv, vbus_state->ma);

	if ((vbus_state->mv * vbus_state->ma) >= PD_POWER_CHARGE_18W) {
		hwlog_info("%s : over 18w\n", __func__);
		high_power_charging = true;
	}

	if (vbus_state->mv >= VBUS_VOL_9000MV) {
		hwlog_info("%s : over 9V\n", __func__);
		high_voltage_charging = true;
	}

	hwlog_info("%s : %d uW\n", __func__,
		   vbus_state->mv * vbus_state->ma);

	/* update max power status & high voltage status */
	pd_dpm_set_optional_max_power_status(high_power_charging);
	pd_dpm_set_high_voltage_charging_status(high_voltage_charging);
#endif
}

void pd_dpm_report_pd_sink_vbus(struct pd_dpm_info *di, void *data)
{
	struct pd_dpm_vbus_state *vbus_state = data;

	mutex_lock(&di->sink_vbus_lock);
	if (vbus_state->vbus_type & TCP_VBUS_CTRL_PD_DETECT){
		di->ctc_cable_flag = true;
		di->pd_finish_flag = true;
		hw_usb_psy_force_change_type_to_ac();
		if ((vbus_state->mv * vbus_state->ma) >= QUICK_CHARGE_ICON_THRESHOLD) {
			charge_send_icon_uevent(ICON_TYPE_QUICK);
			g_pd_quick_charger_flag = true;
		}
	}
	pd_dpm_sink_voltage_status_update(di, vbus_state);
	mutex_unlock(&di->sink_vbus_lock);
}

bool pd_dpm_get_quick_charger_flag(void)
{
	if (g_pd_di->ctc_cable_flag && g_pd_di->pd_finish_flag && g_pd_quick_charger_flag)
		return true;

	return false;
}

bool pd_dpm_use_extra_otg_channel(void)
{
	return g_otg_channel > 0;
}

static void pd_dpm_source_voltage_status_update(struct pd_dpm_info *di,
	const struct pd_dpm_vbus_state *vbus_state)
{
#ifdef CONFIG_TCPC_HI6526
	if (!hi6526_get_tcpc_dev())
		return;

	if (vbus_state->mv == 0) {
		hwlog_info("%s : Disable\n", __func__);
		pd_dpm_set_high_voltage_charging_status(false);
	} else {
		di->pd_source_vbus = true;
		hwlog_info("%s : Source %d mV, %d mA\n", __func__,
			   vbus_state->mv, vbus_state->ma);
	}
#endif
}

void pd_dpm_report_pd_source_vbus(struct pd_dpm_info *di, void *data)
{
	struct pd_dpm_vbus_state *vbus_state = data;

	mutex_lock(&di->sink_vbus_lock);
	if (vbus_state->vbus_type & TCP_VBUS_CTRL_PD_DETECT) {
		di->ctc_cable_flag = true;
		di->pd_finish_flag = true;
	}

	if (vbus_state->mv == 0) {
		hwlog_info("%s : Disable\n", __func__);
		if (g_otg_channel)
			vbus_ch_close(VBUS_CH_USER_PD, VBUS_CH_TYPE_BOOST_GPIO, true, false);
	} else {
		hwlog_info("%s : Enable Source %d mV, %d mA\n", __func__, vbus_state->mv, vbus_state->ma);
		if (g_otg_channel)
			vbus_ch_open(VBUS_CH_USER_PD, VBUS_CH_TYPE_BOOST_GPIO, true);
	}
	pd_dpm_source_voltage_status_update(di, vbus_state);
	mutex_unlock(&di->sink_vbus_lock);
}

static void pd_dpm_analog_hs_plug_state(int *usb_event, int *event_id,
	bool *notify_audio, bool state)
{
	if (state) {
		*notify_audio = true;
		pd_dpm_analog_hs_state = 1;
		*event_id = ANA_HS_PLUG_IN;
		*usb_event = PD_DPM_USB_TYPEC_AUDIO_ATTACHED;
		hwlog_info("%s ATTACHED_AUDIO\n", __func__);
	} else {
		if (pd_dpm_analog_hs_state == 1) {
			*usb_event = PD_DPM_USB_TYPEC_AUDIO_DETACHED;
			*notify_audio = true;
			pd_dpm_analog_hs_state = 0;
			*event_id = ANA_HS_PLUG_OUT;
			hwlog_info("%s AUDIO UNATTACHED\n", __func__);
		} else {
			*usb_event = PD_DPM_USB_TYPEC_DETACHED;
		}
	}
}

static void pd_dpm_dr_swap(void *data)
{
	int usb_event;
	struct pd_dpm_swap_state *swap_state = data;

	if (swap_state->new_role == PD_ROLE_DFP)
		usb_event = PD_DPM_USB_TYPEC_HOST_ATTACHED;
	else
		usb_event = PD_DPM_USB_TYPEC_DEVICE_ATTACHED;

	pd_dpm_set_typec_state(usb_event);
}
/* for audio analog hs drivers to check usb state */
int pd_dpm_get_analog_hs_state(void)
{
	hwlog_info("%s: state is %d\n", __func__, pd_dpm_analog_hs_state);
	return pd_dpm_analog_hs_state;
}

static void pd_dpm_notify_ana_hs(int new_state)
{
	if (!ana_hs_support_usb_sw())
		return;

	if (new_state == PD_DPM_TYPEC_ATTACHED_AUDIO) {
		hwlog_info("%s: ATTACHED_AUDIO\n", __func__);
		pd_dpm_analog_hs_state = ANA_HS_PLUG_IN;
		ana_hs_plug_handle(ANA_HS_PLUG_IN);
		return;
	}
	if ((new_state == PD_DPM_TYPEC_UNATTACHED) &&
		(pd_dpm_analog_hs_state == ANA_HS_PLUG_IN)) {
		hwlog_info("%s: AUDIO UNATTACHED\n", __func__);
		pd_dpm_analog_hs_state = ANA_HS_PLUG_OUT;
		ana_hs_plug_handle(ANA_HS_PLUG_OUT);
	}
}

static uint8_t pd_dpm_get_no_rpsrc_state(void)
{
#ifdef CONFIG_TCPC_HI6526
	return get_no_rpsrc_state() || hi6526_get_no_rpsrc_state();
#else
	return get_no_rpsrc_state();
#endif
}

int pd_dpm_handle_pe_event(unsigned long event, void *data)
{
	int usb_event = PD_DPM_USB_TYPEC_NONE;
	struct pd_dpm_typec_state *typec_state = NULL;
	bool notify_audio = false;
	int event_id = ANA_HS_PLUG_OUT;

	if (!g_pd_di) {
		hwlog_err("%s g_pd_di is null\n", __func__);
		return -1;
	}

	switch (event) {
	case PD_DPM_PE_ABNORMAL_CC_CHANGE_HANDLER:
		if (abnormal_cc_detection &&
			(direct_charge_get_stage_status() <
			DC_STAGE_CHARGE_INIT))
			pd_dpm_handle_abnomal_change(
				PD_DPM_ABNORMAL_CC_CHANGE);

		return 0;

	case PD_DPM_PE_EVT_TYPEC_STATE:
		{
			if(!data || !g_pd_di) {
				hwlog_info("%s data is null\r\n", __func__);
				return -1;
			}

			typec_state = data;
			if (!support_analog_audio && (typec_state->new_state == PD_DPM_TYPEC_ATTACHED_AUDIO)) {
				hwlog_err("%s does not support analog audio\n", __func__);
				return -1;
			}
			g_pd_cc_orientation_factory =
				CC_ORIENTATION_FACTORY_UNSET;
			switch (typec_state->new_state) {
			case PD_DPM_TYPEC_ATTACHED_SNK:
				usb_event = PD_DPM_USB_TYPEC_DEVICE_ATTACHED;
				hwlog_info("%s ATTACHED_SINK\r\n", __func__);
				break;

			case PD_DPM_TYPEC_ATTACHED_SRC:
				usb_event = PD_DPM_USB_TYPEC_HOST_ATTACHED;
				hwlog_info("%s ATTACHED_SOURCE\r\n", __func__);
				break;
			case PD_DPM_TYPEC_UNATTACHED:
				pd_dpm_set_optional_max_power_status(false);
				g_pd_di->ctc_cable_flag = false;
				g_pd_di->pd_finish_flag = false;
				g_pd_quick_charger_flag = false;
				/* the sequence can not change, would affect sink_vbus command */
				pd_dpm_analog_hs_plug_state(&usb_event,
					&event_id, &notify_audio, false);
				hwlog_info("%s UNATTACHED\r\n", __func__);
				g_pd_cc_orientation_factory =
					CC_ORIENTATION_FACTORY_SET;
				g_pd_di->cable_vdo = 0;
				break;
			case PD_DPM_TYPEC_ATTACHED_AUDIO:
				pd_dpm_analog_hs_plug_state(&usb_event,
					&event_id, &notify_audio, true);
				hwlog_info("%s ATTACHED_audio\r\n", __func__);
				break;
			case PD_DPM_TYPEC_ATTACHED_DBGACC_SNK:
			case PD_DPM_TYPEC_ATTACHED_CUSTOM_SRC:
				usb_event = PD_DPM_USB_TYPEC_DEVICE_ATTACHED;
				hwlog_info("%s ATTACHED_CUSTOM_SRC\r\n",
					__func__);
				break;
			case PD_DPM_TYPEC_ATTACHED_DEBUG:
				break;
			case PD_DPM_TYPEC_ATTACHED_NORP_SRC:
				g_pd_cc_orientation_factory =
					CC_ORIENTATION_FACTORY_SET;
				usb_event = PD_DPM_USB_TYPEC_DEVICE_ATTACHED;
				hwlog_info("%s TYPEC_ATTACHED_NORP_SRC\r\n", __func__);
				if (pd_dpm_get_no_rpsrc_state() == 0)
					pd_dpm_handle_abnomal_change(PD_DPM_UNATTACHED_VBUS_ONLY);
				break;
			default:
				hwlog_info("%s can not detect typec state\r\n", __func__);
				break;
			}

			g_pd_di->cur_typec_state = typec_state->new_state;
			pd_dpm_set_typec_state(usb_event);

			if (typec_state->polarity == 1)
				pd_dpm_set_cc_orientation(true);
			else
				pd_dpm_set_cc_orientation(false);

			if (notify_audio)
				ana_hs_plug_handle(event_id);

			pd_dpm_notify_ana_hs(typec_state->new_state);
		}
		break;

	case PD_DPM_PE_EVT_PD_STATE:
		{
			struct pd_dpm_pd_state *pd_state = data;
			switch (pd_state->connected) {
			case PD_CONNECT_PE_READY_SNK:
			case PD_CONNECT_PE_READY_SRC:
				break;
			default:
				break;
			}
		}
		break;
	case PD_DPM_PE_EVT_SINK_VBUS:
		pd_dpm_report_pd_sink_vbus(g_pd_di, data);
		break;

	case PD_DPM_PE_EVT_SOURCE_VBUS:
		pd_dpm_report_pd_source_vbus(g_pd_di, data);
		break;
	case PD_DPM_PE_EVT_DIS_VBUS_CTRL:
		if(g_pd_di->pd_finish_flag == true) {
			pd_dpm_set_optional_max_power_status(false);
			pd_dpm_set_high_voltage_charging_status(false);
			mutex_lock(&g_pd_di->sink_vbus_lock);
			g_pd_di->pd_finish_flag = false;
			mutex_unlock(&g_pd_di->sink_vbus_lock);
		}
		break;
	case PD_DPM_PE_EVT_SOURCE_VCONN:
		if (!data) {
			break;
		}

		if (*(int *)data) {
			boost_5v_enable(ENABLE, BOOST_CTRL_PD_VCONN);
			hwlog_info("%s  enable vconn\n", __func__);
		} else {
			boost_5v_enable(DISABLE, BOOST_CTRL_PD_VCONN);
			hwlog_info("%s  disable vconn\n", __func__);
		}
		break;
	case PD_DPM_PE_EVT_DR_SWAP:
		pd_dpm_dr_swap(data);
		break;
	case PD_DPM_PE_EVT_PR_SWAP:
		break;
	case PD_DPM_PE_CABLE_VDO:
		if (!data)
			return 0;

		memcpy(&g_pd_di->cable_vdo, data, sizeof(g_pd_di->cable_vdo));
		hwlog_info("%s cable_vdo=%u\n", __func__, g_pd_di->cable_vdo);
		break;
	default:
		hwlog_info("%s  unkonw event \r\n", __func__);
		break;
	};

	return 0;
}
EXPORT_SYMBOL_GPL(pd_dpm_handle_pe_event);

static int pd_dpm_parse_dt(struct pd_dpm_info *info,
	struct device *dev)
{
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;
	// default name
	info->tcpc_name = "type_c_port0";
	if (of_property_read_u32(np,"moisture_detection_by_cc_enable", &moisture_detection_by_cc_enable)) {
		hwlog_err("get moisture_detection_by_cc_enable fail!\n");
		moisture_detection_by_cc_enable = 0;
	}
	hwlog_info("moisture_detection_by_cc_enable = %d!\n", moisture_detection_by_cc_enable);

	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), np,
		"cc_abnormal_dmd_report_enable", &g_cc_abnormal_dmd_report_enable, 1);

	if (of_property_read_u32(np, "support_analog_audio", &support_analog_audio)) {
		hwlog_err("get support_analog_audio fail!\n");
		support_analog_audio = 1;
	}
	hwlog_info("support_analog_audio = %d!\n", support_analog_audio);

	if (of_property_read_u32(np, "cc_moisture_status_report",
		&cc_moisture_status_report)) {
		hwlog_err("get cc_moisture_status_report fail\n");
		cc_moisture_status_report = 0;
	}
	hwlog_info("cc_moisture_status_report = %d\n",
		cc_moisture_status_report);

	(void)power_dts_read_u32(power_dts_tag(HWLOG_TAG), np,
		"otg_channel", &g_otg_channel, 0);
	hwlog_info("g_otg_channel = %d\n", g_otg_channel);

	return 0;
}

static int pd_dpm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pd_dpm_info *di = NULL;
	hwlog_info("%s +\n", __func__);

	di = devm_kzalloc(&pdev->dev,sizeof(*di), GFP_KERNEL);
	if (!di) {
		hwlog_err("%s: alloc dev failed\n", __func__);
		return -ENOMEM;
	}

	di->dev = &pdev->dev;

	g_pd_di = di;
	mutex_init(&di->sink_vbus_lock);

	if (of_property_read_u32(di->dev->of_node, "abnormal_cc_detection", &abnormal_cc_detection))
	{
		hwlog_err("get abnormal_cc_detection fail!\n");
	} else {
		hwlog_info("abnormal_cc_detection = %d \n", abnormal_cc_detection);
	}

	if (of_property_read_u32(di->dev->of_node, "cc_dynamic_protect",
		&cc_dynamic_protect))
		cc_dynamic_protect = 0;
	hwlog_info("cc_short_dynamic = %d\n", cc_dynamic_protect);

	if (of_property_read_u32(di->dev->of_node, "abnormal_cc_interval", &abnormal_cc_interval)) {
		hwlog_err("get abnormal_cc_interval fail!\n");
		abnormal_cc_interval = PD_DPM_CC_CHANGE_INTERVAL;
	}
	hwlog_info("abnormal_cc_interval= %d \n", abnormal_cc_interval);

	if (abnormal_cc_detection)
		init_fb_notification();

	typec_dev = pd_dpm_create_group();

	mutex_init(&di->usb_lock);

	di->usb_wq = create_workqueue("pd_dpm_usb_wq");
	INIT_DELAYED_WORK(&di->cc_moisture_flag_restore_work,
		pd_dpm_cc_moisture_flag_restore);
	INIT_DELAYED_WORK(&cc_short_work, pd_dpm_cc_short_work);

	platform_set_drvdata(pdev, di);

	pd_dpm_parse_dt(di, &pdev->dev);

	dc_cable_ops_register(&cable_detect_ops);

	hwlog_info("%s - probe ok\n", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(pd_dpm_probe);

static const struct of_device_id pd_dpm_callback_match_table[] = {
	{.compatible = "huawei,pd_dpm",},
	{},
};

static struct platform_driver pd_dpm_callback_driver = {
	.probe		= pd_dpm_probe,
	.remove		= NULL,
	.driver		= {
		.name	= "huawei,pd_dpm",
		.owner	= THIS_MODULE,
		.of_match_table = pd_dpm_callback_match_table,
	}
};

static int __init pd_dpm_init(void)
{
	hwlog_info("%s \n", __func__);

	return platform_driver_register(&pd_dpm_callback_driver);
}

static void __exit pd_dpm_exit(void)
{
	platform_driver_unregister(&pd_dpm_callback_driver);
}

module_init(pd_dpm_init);
module_exit(pd_dpm_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("huawei pd dpm");
MODULE_AUTHOR("SuoAnDaJie<suoandajie@huawei.com>");
