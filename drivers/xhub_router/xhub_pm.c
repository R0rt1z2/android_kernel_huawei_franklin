/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub pm module
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/suspend.h>
#include <linux/fb.h>
#include <linux/pm_wakeup.h>
#include <securec.h>
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "xhub_route.h"
#include "protocol.h"
#include "sensor_config.h"
#include "sensor_detect.h"
#include "xhub_pm.h"
#include "xhub_recovery.h"

uint32_t need_reset_io_power;
int g_xhub_wdt_irq = -1;
sys_status_t iom3_sr_status = ST_WAKEUP;
int key_state;
int iom3_power_state = ST_POWERON;
struct ipc_debug ipc_debug_info;

static struct sensor_status sensor_status_backup;
static DEFINE_MUTEX(mutex_pstatus);

static char *sys_status_t_str[] = {
	[ST_SCREENON] = "ST_SCREENON",
	[ST_SCREENOFF] = "ST_SCREENOFF",
	[ST_SLEEP] = "ST_SLEEP",
	[ST_WAKEUP] = "ST_WAKEUP",
};

int get_iomcu_power_state(void)
{
	return iom3_power_state;
}
EXPORT_SYMBOL(get_iomcu_power_state);

static inline void clean_ipc_debug_info(void)
{
	memset(&ipc_debug_info, 0, sizeof(ipc_debug_info));
}

static inline void print_ipc_debug_info(void)
{
	int i;

	for (i = TAG_BEGIN; i < TAG_END; ++i) {
		if (ipc_debug_info.event_cnt[i])
			hwlog_info("event_cnt[%d]: %d\n", i,
			ipc_debug_info.event_cnt[i]);
	}
	if (ipc_debug_info.pack_error_cnt)
		hwlog_err("pack_err_cnt: %d\n", ipc_debug_info.pack_error_cnt);
}

int tell_ap_status_to_mcu(int ap_st)
{
	read_info_t pkg_mcu;
	write_info_t winfo;

	if ((ap_st >= ST_BEGIN) && (ap_st < ST_END)) {
		pkt_sys_statuschange_req_t pkt;

		winfo.tag = TAG_SYS;
		winfo.cmd = CMD_SYS_STATUSCHANGE_REQ;
		winfo.wr_len = sizeof(pkt) - sizeof(pkt.hd);
		pkt.status = ap_st;
		winfo.wr_buf = &pkt.status;

		if (likely((ap_st >= ST_SCREENON) && (ap_st <= ST_WAKEUP))) {
			hwlog_info("------------>tell mcu ap in status %s\n",
				sys_status_t_str[ap_st]);
			iom3_sr_status = ap_st;
		} else {
			hwlog_info("------------>tell mcu ap in status %d\n",
				ap_st);
		}
		return write_customize_cmd(&winfo,
			(ap_st == ST_SLEEP) ? (&pkg_mcu) : NULL, true);
	} else {
		hwlog_err("error status %d in %s\n", ap_st, __func__);
		return -EINVAL;
	}
}

void update_current_app_status(uint8_t tag, uint8_t cmd)
{
	mutex_lock(&mutex_pstatus);
	if ((cmd == CMD_CMN_OPEN_REQ) || (cmd == CMD_CMN_INTERVAL_REQ))
		i_power_status.app_status[tag] = 1;
	else if (cmd == CMD_CMN_CLOSE_REQ)
		i_power_status.app_status[tag] = 0;
	mutex_unlock(&mutex_pstatus);
}

static void check_current_app(void)
{
	int i;
	int flag = 0;

	mutex_lock(&mutex_pstatus);
	for (i = 0; i < TAG_END; i++) {
		if (i_power_status.app_status[i])
			flag++;
	}
	if (flag > 0) {
		hwlog_info("total %d app running after ap suspend\n", flag);
		i_power_status.power_status = SUB_POWER_ON;
		flag = 0;
	} else {
		hwlog_info("iomcu will power off after ap suspend\n");
		i_power_status.power_status = SUB_POWER_OFF;
	}
	mutex_unlock(&mutex_pstatus);
}

static int xhub_pm_suspend(struct device *dev)
{
	int ret = 0;

	hwlog_info("%s+\n", __func__);
	if (iom3_sr_status != ST_SLEEP) {
		ret = tell_ap_status_to_mcu(ST_SLEEP);
		iom3_power_state = ST_SLEEP;
		check_current_app();
		clean_ipc_debug_info();
	}
	hwlog_info("%s-\n", __func__);
	return ret;
}

static int xhub_pm_resume(struct device *dev)
{
	hwlog_info("%s+\n", __func__);
	print_ipc_debug_info();
	tell_ap_status_to_mcu(ST_WAKEUP);

	iom3_power_state = ST_WAKEUP;
	hwlog_info("%s-\n", __func__);
	return 0;
}

const static struct of_device_id xhub_io_supply_ids[] = {
	{ .compatible = "huawei,xhub_io" },
	{}
};
MODULE_DEVICE_TABLE(of, xhub_io_supply_ids);

static int xhub_io_driver_probe(struct platform_device *pdev)
{
	uint32_t val = 0;
	struct device_node *power_node = NULL;

	if (!of_match_device(xhub_io_supply_ids, &pdev->dev)) {
		hwlog_err("[%s,%d]: match fail !\n",
			__func__, __LINE__);
		return -ENODEV;
	}
	g_xhub_wdt_irq = platform_get_irq(pdev, 0);
	if (g_xhub_wdt_irq < 0) {
		pr_err("[%s] platform_get_irq err\n", __func__);
		return -ENXIO;
	}
	power_node = of_find_node_by_name(NULL, "xhub_io_power");
	if (!power_node) {
		hwlog_err("%s failed to find dts node xhub_io_power\n",
			__func__);
	} else {
		if (of_property_read_u32(power_node, "need-reset", &val)) {
			hwlog_err("%s failed to find property need-reset.\n",
				__func__);
		} else {
			need_reset_io_power = val;
			hwlog_info("%s property need-reset is %d.\n",
				__func__, val);
		}
	}
	hwlog_info("%s: success!\n", __func__);
	return 0;
}

static bool should_be_processed_when_sr(int sensor_tag)
{
	bool ret = true; /* can be closed default */

	switch (sensor_tag) {
	case TAG_PS:
	case TAG_STEP_COUNTER:
	case TAG_SIGNIFICANT_MOTION:
	case TAG_PHONECALL:
	case TAG_CONNECTIVITY:
	case TAG_FP:
	case TAG_FP_UD:
	case TAG_MAGN_BRACKET:
	case TAG_DROP:
	case TAG_EXT_HALL:
		ret = false;
		break;

	default:
		break;
	}

	return ret;
}

void disable_sensors_when_suspend(void)
{
	int tag;

	memset(&sensor_status_backup, 0, sizeof(sensor_status_backup));
	memcpy(&sensor_status_backup,
		&sensor_status, sizeof(sensor_status_backup));
	for (tag = TAG_SENSOR_BEGIN; tag < TAG_SENSOR_END; ++tag) {
		if ((sensor_status_backup.status[tag] ||
			sensor_status_backup.opened[tag]) &&
			!(hifi_supported == 1 &&
			(sensor_status.batch_cnt[tag] > 1))) {
			if (should_be_processed_when_sr(tag))
				xhub_sensor_enable(tag, false);
		}
	}
}

void enable_sensors_when_resume(void)
{
	int tag = 0;
	interval_param_t delay_param = {
		.period = sensor_status_backup.delay[tag],
		.batch_count = 1,
		.mode = AUTO_MODE,
		.reserved[0] = TYPE_STANDARD /* for step counter only */
	};
	for (tag = TAG_SENSOR_BEGIN;
		tag < TAG_SENSOR_END; ++tag) {
		if ((sensor_status_backup.status[tag] ||
			sensor_status_backup.opened[tag]) &&
			!(hifi_supported == 1 &&
			(sensor_status.batch_cnt[tag] > 1))) {
			if (should_be_processed_when_sr(tag)) {
				if (sensor_status_backup.opened[tag] &&
					(sensor_status.opened[tag] == 0))
					xhub_sensor_enable(tag, true);
				if (sensor_status_backup.status[tag]) {
					delay_param.period =
						sensor_status.status[tag] ?
						sensor_status.delay[tag] :
						sensor_status_backup.delay[tag];
					delay_param.batch_count =
						sensor_status.status[tag] ?
						sensor_status.batch_cnt[tag] :
						sensor_status_backup.batch_cnt[tag];
					xhub_sensor_setdelay(tag, &delay_param);
				} else if ((sensor_status_backup.status[tag] == 0) &&
					(tag == TAG_ALS || tag == TAG_ALS1 || tag == TAG_ALS2) &&
					sensor_status_backup.opened[tag]) {
					hwlog_info("ALS set delay when backup_status =0 && backup_opened = 1\n");
					delay_param.period = 0;
					delay_param.batch_count = 1;
					xhub_sensor_setdelay(tag, &delay_param);
				}
			}
		}
	}
}

static int xhub_fb_notifier(struct notifier_block *nb,
	unsigned long action, void *data)
{
	if (!data)
		return NOTIFY_OK;
	switch (action) {
	case FB_EVENT_BLANK: /* change finished */
	{
		struct fb_event *event = data;
		int *blank = event->data;

		if (registered_fb[0] !=
			event->info) { /* only main screen on/off info send to hub */
			hwlog_err("%s, not main screen info, just return\n",
				__func__);
			return NOTIFY_OK;
		}
		switch (*blank) {
		case FB_BLANK_UNBLANK: /* screen on */
			tell_ap_status_to_mcu(ST_SCREENON);
			sync_time_to_xhub();
			break;

		case FB_BLANK_POWERDOWN: /* screen off */
			tell_ap_status_to_mcu(ST_SCREENOFF);
			sync_time_to_xhub();
			sensor_redetect_enter();
			break;

		default:
			hwlog_err("unknown---> lcd unknown in %s\n", __func__);
			break;
		}
		break;
	}
	default:
		break;
	}

	return NOTIFY_OK;
}

static int xhub_pm_notify(struct notifier_block *nb,
	unsigned long mode, void *_unused)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE: /* suspend */
		hwlog_info("suspend in %s\n", __func__);
		disable_sensors_when_suspend();
		break;

	case PM_POST_SUSPEND: /* resume */
		hwlog_info("resume in %s\n", __func__);
		enable_sensors_when_resume();
		break;

	case PM_HIBERNATION_PREPARE: /* Going to hibernate */
	case PM_POST_HIBERNATION: /* Hibernation finished */
	case PM_RESTORE_PREPARE: /* Going to restore a saved image */
	case PM_POST_RESTORE: /* Restore failed */
	default:
		break;
	}

	return 0;
}

static struct notifier_block fb_notify = {
	.notifier_call = xhub_fb_notifier,
};

void set_pm_notifier(void)
{
	pm_notifier(xhub_pm_notify, 0);
	fb_register_client(&fb_notify);
}

const static struct dev_pm_ops xhub_io_pm_ops = {
	.suspend = xhub_pm_suspend,
	.resume = xhub_pm_resume,
};

static struct platform_driver xhub_io_driver = {
	.probe = xhub_io_driver_probe,
	.driver = {
		   .name = "xhub_io_driver",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(xhub_io_supply_ids),
		   .pm = &xhub_io_pm_ops,
	},
};

int xhub_io_driver_init(void)
{
	int ret;

	hwlog_info("[%s] ++", __func__);

	ret = platform_driver_register(&xhub_io_driver);
	if (ret) {
		hwlog_err("%s: platform_device_register(xhub_io_driver) failed, ret:%d.\n",
			__func__, ret);
		return ret;
	}
	hwlog_info("[%s] --", __func__);
	return 0;
}
