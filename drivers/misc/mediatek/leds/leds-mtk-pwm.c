// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 */

#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#ifdef CONFIG_LCD_KIT_DRIVER
#include "mtk_leds_common.h"
#include "lcd_kit_drm_panel.h"
#endif
#include "leds-mtk-pwm.h"
#include "sensor_para.h"
#ifdef CONFIG_HW_ZEROHUNG
#include "chipset_common/hwzrhung/hung_wp_screen.h"
#endif

#define CONFIG_LEDS_BRIGHTNESS_CHANGED
#ifdef CONFIG_LCD_KIT_DRIVER
#define LCD_DEFAULT_BRIGHTNESS 255
#endif
/****************************************************************************
 * variables
 ***************************************************************************/
#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME " %s(%d) :" fmt, __func__, __LINE__

#ifdef CONFIG_LCD_KIT_DRIVER
struct mtk_pwm_leds_info;

static int led_level_set(struct led_classdev *led_cdev,
					 enum led_brightness brightness);

struct pwm_led_debug_info {
	unsigned long long current_t;
	unsigned long long last_t;
	char buffer[4096];
	int count;
};

struct pwm_led_desp {
	int index;
	char name[16];
};

struct pwm_leds_desp_info {
	int lens;
	struct pwm_led_desp *leds[0];
};

struct led_pwm_info {
	struct pwm_device *pwm;
	struct led_pwm config;
	unsigned long long duty;
};

struct mtk_pwm_led_data {
	struct pwm_led_desp desp;
	struct led_conf_info conf;
	int last_level;
	int brightness;
	struct led_pwm_info info;
	struct mtk_pwm_leds_info	*parent;
	struct pwm_led_debug_info debug;
	struct work_struct work;
};

struct mtk_pwm_leds_info {
	struct device *dev;
	int			nums;
	struct mtk_pwm_led_data leds[0];
};

static DEFINE_MUTEX(leds_mutex);
static struct pwm_leds_desp_info *leds_info;

int mtk_pwm_leds_get_satus(void)
{
	struct mtk_panel_info *panel_info = NULL;
	int ret = 0;

	panel_info = lcm_get_panel_info();
	if (panel_info == NULL) {
		pr_notice("panel_info is NULL\n");
		return ret;
	}
	switch (panel_info->bl_ic_ctrl_mode) {
	case I2C_ONLY_MODE:
		ret = 1; /* 0:not support mtk led pwm  1:support mtk led pwm */
		break;
	case MTK_PWM_HIGH_I2C_MODE:
		ret = 1;
		break;
	case MTK_AAL_I2C_MODE:
		ret = 1;
		break;
	case PWM_ONLY_MODE:
		ret = 1;
		break;
	case PWM_I2C_MODE:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static BLOCKING_NOTIFIER_HEAD(mtk_leds_chain_head);

int mtk_pwm_leds_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mtk_leds_chain_head, nb);
}
EXPORT_SYMBOL_GPL(mtk_pwm_leds_register_notifier);

int mtk_pwm_leds_unregister_notifier(struct notifier_block *nb)

{
	return blocking_notifier_chain_unregister(&mtk_leds_chain_head, nb);
}
EXPORT_SYMBOL_GPL(mtk_pwm_leds_unregister_notifier);

int mtk_pwm_leds_call_notifier(unsigned long action, void *data)
{
	return blocking_notifier_call_chain(&mtk_leds_chain_head, action, data);
}
EXPORT_SYMBOL_GPL(mtk_pwm_leds_call_notifier);

static int call_notifier(int event, struct mtk_pwm_led_data *led_dat)
{
	int err;

	err = mtk_pwm_leds_call_notifier(event, &led_dat->conf);
	if (err)
		pr_info("notifier_call_chain error\n");
	return err;
}

static void led_debug_log(struct mtk_pwm_led_data *s_led,
		int level, int mappingLevel)
{
	unsigned long cur_time_mod = 0;
	unsigned long long cur_time_display = 0;

	s_led->debug.current_t = sched_clock();
	cur_time_display = s_led->debug.current_t;
	cur_time_mod = do_div(cur_time_display, 1000000000);

	sprintf(s_led->debug.buffer + strlen(s_led->debug.buffer),
		"T:%lld.%ld,L:%d L:%d map:%d    ",
		cur_time_display, cur_time_mod/1000000,
		s_led->conf.cdev.brightness, level, mappingLevel);

	s_led->debug.count++;

	if (level == 0 || s_led->debug.count >= 5 ||
		(s_led->debug.current_t - s_led->debug.last_t) > 1000000000) {
		pr_info("%s", s_led->debug.buffer);
		s_led->debug.count = 0;
		s_led->debug.buffer[strlen("[Light] Set directly ") +
			strlen(s_led->conf.cdev.name)] = '\0';
	}

	s_led->debug.last_t = sched_clock();
}

static int getLedDespIndex(char *name)
{
	int i = 0;

	while (i < leds_info->lens) {
		if (!strcmp(name, leds_info->leds[i]->name))
			return i;
		i++;
	}
	return -1;
}

static void __led_pwm_set(struct led_pwm_info *led_info)
{
	int new_duty = led_info->duty;

	mutex_lock(&leds_mutex);

	pwm_config(led_info->pwm, new_duty, led_info->config.pwm_period_ns);
	if (new_duty == 0)
		pwm_disable(led_info->pwm);
	else
		pwm_enable(led_info->pwm);

	mutex_unlock(&leds_mutex);
}

static  void mt_lcd_kit_set_backlgiht(struct mtk_pwm_led_data *led_dat,
	int level)
{
	unsigned int bl_max_level;
	struct mtk_panel_info *panel_info = NULL;
	unsigned int max;
	unsigned long long duty;

	if (led_dat == NULL) {
		pr_notice("led_dat is NULL\n");
		return;
	}
	panel_info = lcm_get_panel_info();
	if (panel_info == NULL) {
		pr_notice("panel_info is NULL\n");
		return;
	}

	if (panel_info->bl_ic_ctrl_mode == MTK_AAL_I2C_MODE) {
		pr_notice("AAL set backlight\n");
		return;
	}

	level = min(level, led_dat->conf.max_level);
	if (level == led_dat->conf.level)
		return;
	bl_max_level = led_dat->conf.cdev.max_brightness;

	if (level != 0 && level > bl_max_level)
		level = bl_max_level;

	led_dat->conf.level = level;
	max = led_dat->conf.cdev.max_brightness;
	duty = led_dat->info.config.pwm_period_ns;
	if (disp_info->quickly_sleep_out.support) {
		if (disp_info->quickly_sleep_out.panel_on_tag)
			lcd_kit_disp_on_check_delay();
	}
	switch (panel_info->bl_ic_ctrl_mode) {
	case I2C_ONLY_MODE:
		lcd_kit_bl_ic_set_backlight(level);
		break;
	case MTK_PWM_HIGH_I2C_MODE:
		duty *= max;
		do_div(duty, max);
		if (led_dat->info.config.active_low)
			duty = led_dat->info.config.pwm_period_ns - duty;
		led_dat->info.duty = duty;
		lcd_kit_bl_ic_set_backlight(level);
		__led_pwm_set(&led_dat->info);
		break;
	case PWM_ONLY_MODE:
		duty *= level;
		do_div(duty, max);
		if (led_dat->info.config.active_low)
			duty = led_dat->info.config.pwm_period_ns - duty;
		led_dat->info.duty = duty;
		__led_pwm_set(&led_dat->info);
		break;
	case PWM_I2C_MODE:
		duty *= level;
		do_div(duty, max);
		if (led_dat->info.config.active_low)
			duty = led_dat->info.config.pwm_period_ns - duty;
		led_dat->info.duty = duty;
		lcd_kit_bl_ic_set_backlight(level);
		__led_pwm_set(&led_dat->info);
		break;
	default:
		break;
	}
}

static int led_level_pwm_set(struct mtk_pwm_led_data *led_dat,
				int brightness)
{
	mt_lcd_kit_set_backlgiht(led_dat, brightness);
#ifdef CONFIG_HW_ZEROHUNG
		hung_wp_screen_setbl(brightness);
#endif
	send_sensor_scp_brightness(brightness);
	return 0;
}

int mt_pwm_leds_brightness_set(char *name, int level)
{
	struct mtk_pwm_led_data *led_dat = NULL;
	int index;
	int led_Level;
	struct mtk_panel_info *panel_info = NULL;
	unsigned int max;
	unsigned long long duty;

	index = getLedDespIndex(name);
	if (index < 0) {
		pr_notice("can not find leds by led_desp %s", name);
		return -1;
	}
	led_dat = container_of(leds_info->leds[index],
		struct mtk_pwm_led_data, desp);
	led_Level = (((led_dat->conf.cdev.max_brightness) * level +
		(((1 << led_dat->conf.trans_bits) - 1) / 2)) /
		((1 << led_dat->conf.trans_bits) - 1));
	led_Level = min(led_Level, led_dat->conf.max_level);
	if (led_Level == led_dat->conf.level)
		return 0;

	panel_info = lcm_get_panel_info();
	if (panel_info == NULL) {
		pr_notice("panel_info is NULL\n");
		return 0;
	}
	max = led_dat->conf.cdev.max_brightness;
	duty = led_dat->info.config.pwm_period_ns;

	/* quickly_sleep_out */
	if (led_Level > 0 && disp_info->quickly_sleep_out.support) {
		if (disp_info->quickly_sleep_out.panel_on_tag)
			lcd_kit_disp_on_check_delay();
	}

	switch (panel_info->bl_ic_ctrl_mode) {
	case I2C_ONLY_MODE:
		break;
	case MTK_PWM_HIGH_I2C_MODE:
		duty *= max;
		do_div(duty, max);
		if (led_dat->info.config.active_low)
			duty = led_dat->info.config.pwm_period_ns - duty;
		led_dat->info.duty = duty;
		__led_pwm_set(&led_dat->info);
		break;
	case PWM_ONLY_MODE:
		duty *= led_Level;
		do_div(duty, max);
		if (led_dat->info.config.active_low)
			duty = led_dat->info.config.pwm_period_ns - duty;
		led_dat->info.duty = duty;
		__led_pwm_set(&led_dat->info);
		led_dat->conf.level = led_Level;
		break;
	case MTK_AAL_I2C_MODE:
		lcd_kit_bl_ic_set_backlight(led_Level);
		led_dat->conf.level = led_Level;
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(mt_pwm_leds_brightness_set);

void mtk_led_work(struct work_struct *work)
{
	struct mtk_pwm_led_data *led_data =
	    container_of(work, struct mtk_pwm_led_data, work);

	led_level_pwm_set(led_data, led_data->conf.level);
}

#ifndef CONFIG_MTK_AAL_SUPPORT
static int led_pwm_disable(struct led_pwm_info *led_info)
{

	mutex_lock(&leds_mutex);

	pwm_config(led_info->pwm, 0, led_info->config.pwm_period_ns);
	pwm_disable(led_info->pwm);

	mutex_unlock(&leds_mutex);

	return 0;
}
#endif

static int led_level_set(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	int trans_level = 0;

	struct led_conf_info *led_conf =
		container_of(led_cdev, struct led_conf_info, cdev);
	struct mtk_pwm_led_data *led_dat =
		container_of(led_conf, struct mtk_pwm_led_data, conf);

	if (led_dat->brightness == brightness)
		return 0;

	led_dat->brightness = brightness;
#ifdef CONFIG_LCD_KIT_DRIVER
	trans_level = (
		(((1 << led_dat->conf.trans_bits) - 1) * brightness +
		((led_dat->conf.cdev.max_brightness) / 2)) /
		(led_dat->conf.cdev.max_brightness));
#else
	trans_level = (
		(((1 << led_dat->conf.trans_bits) - 1) * brightness
		+ (((1 << led_dat->conf.led_bits) - 1) / 2))
		/ ((1 << led_dat->conf.led_bits) - 1));
#endif
	led_debug_log(led_dat, brightness, trans_level);

#ifdef MET_USER_EVENT_SUPPORT
	if (enable_met_backlight_tag())
		output_met_backlight_tag(brightness);
#endif

#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	call_notifier(1, led_dat);
#endif
#ifdef CONFIG_MTK_AAL_SUPPORT
	disp_pq_notify_backlight_changed(trans_level);
#ifdef CONFIG_LCD_KIT_DRIVER
	led_level_pwm_set(led_dat, brightness);
	led_dat->last_level = brightness;
#endif
#else
	led_level_pwm_set(led_dat, brightness);
	led_dat->last_level = brightness;
#endif
	return 0;

}

int mt_pwm_leds_setmaxbrightness(char *name, int percent, bool enable)
{
	struct mtk_pwm_led_data *led_dat;
	int max_l = 0, index = -1, limit_l = 0, cur_l = 0;

	index = getLedDespIndex(name);
	if (index < 0) {
		pr_notice("can not find leds by led_desp %s", name);
		return -1;
	}
	led_dat = container_of(leds_info->leds[index],
		struct mtk_pwm_led_data, desp);

	max_l = led_dat->conf.cdev.max_brightness;
	limit_l = (percent * max_l) / 100;
	pr_info("before: name: %s, percent : %d, limit_l : %d, enable: %d",
		leds_info->leds[index]->name, percent, limit_l, enable);
	if (enable) {
		led_dat->conf.max_level = limit_l;
		cur_l = min(led_dat->last_level, limit_l);
	} else if (!enable) {
		led_dat->conf.max_level = max_l;
		cur_l = led_dat->last_level;
	}
#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	call_notifier(3, led_dat);
#endif

	if (led_dat->conf.cdev.brightness != 0)
		led_level_pwm_set(led_dat, cur_l);

	pr_info("after: name: %s, cur_l : %d, max_level : %d",
		led_dat->conf.cdev.name, cur_l, led_dat->conf.max_level);
	return 0;

}
EXPORT_SYMBOL(mt_pwm_leds_setmaxbrightness);

static int led_data_init(struct device *dev, struct mtk_pwm_led_data *s_led)
{
	int ret;

	s_led->conf.cdev.default_trigger = s_led->info.config.default_trigger;
	s_led->conf.cdev.max_brightness = s_led->info.config.max_brightness;
	s_led->conf.cdev.flags = LED_CORE_SUSPENDRESUME;
	s_led->conf.cdev.brightness_set_blocking = led_level_set;
	s_led->brightness = s_led->conf.cdev.max_brightness;
	s_led->conf.level = s_led->conf.cdev.max_brightness;
	s_led->last_level = s_led->conf.cdev.max_brightness;
	ret = devm_led_classdev_register(dev, &(s_led->conf.cdev));
	if (ret < 0) {
		pr_notice("led class register fail!");
		return ret;
	}
	pr_info("%s devm_led_classdev_register ok! ", s_led->conf.cdev.name);

	INIT_WORK(&s_led->work, mtk_led_work);
	sprintf(s_led->debug.buffer + strlen(s_led->debug.buffer),
		"[Light] Set %s directly ", s_led->conf.cdev.name);
	return 0;

}

static int led_pwm_config_add(struct device *dev,
		struct mtk_pwm_led_data *s_led, struct device_node *leds_np)
{
	struct pwm_args pargs;
	int ret = 0;

	if (leds_np != NULL)
		s_led->info.pwm = devm_of_pwm_get(dev, leds_np,
			s_led->info.config.name);
	else
		s_led->info.pwm = devm_pwm_get(dev, s_led->info.config.name);
	if (IS_ERR(s_led->info.pwm)) {
		ret = PTR_ERR(s_led->info.pwm);
		if (ret != -EPROBE_DEFER) {
			pr_notice("unable to request PWM for %s, err_code: %d\n",
				s_led->info.config.name, ret);
			goto err;
		}
	}

	pwm_apply_args(s_led->info.pwm);
	pwm_get_args(s_led->info.pwm, &pargs);

	s_led->info.config.pwm_period_ns = pargs.period;
	if (!s_led->info.config.pwm_period_ns && (pargs.period > 0))
		s_led->info.config.pwm_period_ns = pargs.period;
	pr_info("set led pwm OK, info.config.pwm_period_ns = %d!",
		s_led->info.config.pwm_period_ns);

	return ret;

 err:
	pr_notice("add pwm failed!\n");
	ret = -ENOMEM;
	return ret;

}

static int mtk_leds_parse_dt(struct device *dev,
		struct mtk_pwm_leds_info *m_leds)
{
	struct device_node *child;
	struct mtk_pwm_led_data *s_led;
	int ret = 0, num = 0, level = 102;
	const char *state;

	if (!dev->of_node) {
		pr_notice("Error load dts: node not exist!\n");
		return ret;
	}

	for_each_available_child_of_node(dev->of_node, child) {

		s_led = &(m_leds->leds[num]);
		ret = of_property_read_string(child, "label",
			&(s_led->conf.cdev.name));
		if (ret) {
			pr_info("Fail to read label property");
			goto out_led_dt;
		}
		ret = of_property_read_string(child, "pwm-names",
			&(s_led->info.config.name));
		if (ret) {
			pr_info("Fail to read pwm-names property");
			goto out_led_dt;
		}
		ret = of_property_read_string(child, "default-trigger",
			&(s_led->info.config.default_trigger));
		if (ret) {
			pr_info("Fail to read default-trigger property");
			s_led->info.config.default_trigger = NULL;
		}
		ret = of_property_read_u8(child, "active-low",
			&(s_led->info.config.active_low));
		if (ret)
			pr_info("Fail to read active-low property\n");
		ret = of_property_read_u32(child,
			"led-bits", &(s_led->conf.led_bits));
		if (ret) {
			pr_info("No led-bits, use default value 8");
			s_led->conf.led_bits = 8;
		}
		ret = of_property_read_u32(child,
			"max-brightness", &(s_led->conf.max_level));
		if (ret) {
			pr_info("No max-brightness, use default value 255");
			s_led->conf.max_level = (1 << s_led->conf.led_bits) - 1;
		}
		s_led->info.config.max_brightness =
			(1 << s_led->conf.led_bits) - 1;
		ret = of_property_read_u32(child,
			"trans-bits", &(s_led->conf.trans_bits));
		if (ret) {
			pr_info("No trans-bits, use default value 10");
			s_led->conf.trans_bits = 10;
		}
		if (!strcmp(s_led->conf.cdev.name, "lcd-backlight")) {
			s_led->conf.cdev.max_brightness =
				lcm_get_panel_backlight_max_level();
			if (s_led->conf.cdev.max_brightness == 0) {
				s_led->conf.cdev.max_brightness =
					LCD_DEFAULT_BRIGHTNESS;
			} else {
				s_led->info.config.max_brightness =
					s_led->conf.cdev.max_brightness;
				s_led->conf.max_level =
					s_led->conf.cdev.max_brightness;
			}
		}
		ret = of_property_read_string(child, "default-state", &state);
		if (!ret) {
			if (!strcmp(state, "half"))
				level = s_led->info.config.max_brightness / 2;
			else if (!strcmp(state, "on"))
				level = s_led->info.config.max_brightness;
			else
				level = s_led->conf.level = 0;
		}
		if (mtk_pwm_leds_get_satus() == 0) {
			pr_info("pwm led is not support\n");
			ret = -EINVAL;
			goto out_led_dt;
		}
		pr_info("parse %s(%d) leds dt: %s, %s, %d, %d, %d\n",
			s_led->conf.cdev.name, num, s_led->info.config.name,
			s_led->info.config.default_trigger,
			s_led->info.config.active_low,
			s_led->info.config.max_brightness,
			s_led->conf.led_bits);
		s_led->desp.index = num;
		strncpy(s_led->desp.name, s_led->conf.cdev.name,
			strlen(s_led->conf.cdev.name));
		s_led->desp.index = num;
		leds_info->leds[num] = &s_led->desp;
		s_led->conf.cdev.brightness = level;
		ret = led_data_init(dev, s_led);
		if (ret)
			goto out_led_dt;
		led_pwm_config_add(dev, s_led, child);
		led_level_set(&s_led->conf.cdev, level);
		num++;
	}
	m_leds->nums = num;
	pr_info("load dts ok!");
	return ret;
out_led_dt:
	pr_notice("Error load dts node!\n");
	of_node_put(child);
	return ret;
}

static int mtk_leds_probe(struct platform_device *pdev)
{

	struct device *dev = &pdev->dev;
	struct mtk_pwm_leds_info *m_leds;
	int ret, nums;

	pr_info("probe begain +++");

	nums = of_get_child_count(dev->of_node);
	pr_info("Load dts node nums: %d", nums);
	m_leds = devm_kzalloc(dev, (sizeof(struct mtk_pwm_leds_info) +
		(sizeof(struct mtk_pwm_led_data) * (nums))), GFP_KERNEL);
	if (!m_leds)
		goto err;

	leds_info = devm_kzalloc(dev, (sizeof(struct pwm_leds_desp_info) +
		sizeof(struct pwm_led_desp *) * (nums)),
		GFP_KERNEL);
	leds_info->lens = nums;
	if (!leds_info) {
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(pdev, m_leds);
	m_leds->dev = dev;
	ret = mtk_leds_parse_dt(&(pdev->dev), m_leds);
	if (ret) {
		pr_notice("Failed to parse devicetree!\n");
		goto err;
	}

	pr_info("probe end ---");
	return 0;
 err:
	pr_notice("Failed to probe!\n");
	ret = -ENOMEM;
	return ret;
}

static int mtk_leds_remove(struct platform_device *pdev)
{
	int i;
	struct mtk_pwm_leds_info *m_leds = dev_get_platdata(&pdev->dev);

	if (!m_leds)
		return 0;
	for (i = 0; i < m_leds->nums; i++) {
		if (!m_leds->leds[i].parent)
			continue;
		led_classdev_unregister(&m_leds->leds[i].conf.cdev);
		cancel_work_sync(&m_leds->leds[i].work);
		m_leds->leds[i].parent = NULL;
	}
	kfree(m_leds);
	m_leds = NULL;

	return 0;
}

static void mtk_leds_shutdown(struct platform_device *pdev)
{
	int i;
	struct mtk_pwm_leds_info *m_leds = dev_get_platdata(&pdev->dev);

	pr_info("Turn off backlight\n");

	for (i = 0; m_leds && i < m_leds->nums; i++) {
		if (!&(m_leds->leds[i]))
			continue;
#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
		call_notifier(2, &(m_leds->leds[i]));
#ifdef CONFIG_MTK_AAL_SUPPORT
		continue;
#else
		led_pwm_disable(&(m_leds->leds[i].info));
#endif
#else
		led_pwm_disable(&(m_leds->leds[i].info));
#endif
	}

}

static const struct of_device_id of_mtk_pwm_leds_match[] = {
	{ .compatible = "mediatek,pwm-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_mtk_pwm_leds_match);

static struct platform_driver mtk_pwm_leds_driver = {
	.driver = {
		   .name = "mtk-pwm-leds",
		   .owner = THIS_MODULE,
		   .of_match_table = of_mtk_pwm_leds_match,
		   },
	.probe = mtk_leds_probe,
	.remove = mtk_leds_remove,
	.shutdown = mtk_leds_shutdown,

};

static int __init mtk_pwm_leds_init(void)
{
	int ret;

	pr_info("Leds init\n");
	ret = platform_driver_register(&mtk_pwm_leds_driver);

	if (ret) {
		pr_info("driver register error: %d\n", ret);
		return ret;
	}

	return ret;
}

static void __exit mtk_pwm_leds_exit(void)
{
	platform_driver_unregister(&mtk_pwm_leds_driver);
}

late_initcall(mtk_pwm_leds_init);
module_exit(mtk_pwm_leds_exit);
#else
struct mtk_leds_info;

static int led_level_set(struct led_classdev *led_cdev,
					 enum led_brightness brightness);

struct led_debug_info {
	unsigned long long current_t;
	unsigned long long last_t;
	char buffer[4096];
	int count;
};

struct led_desp {
	int index;
	char name[16];
};

struct leds_desp_info {
	int lens;
	struct led_desp *leds[0];
};

struct led_pwm_info {
	struct pwm_device *pwm;
	struct led_pwm config;
	unsigned long long duty;
};

struct mtk_led_data {
	struct led_desp desp;
	struct led_conf_info	conf;
	int last_level;
	int brightness;
	struct led_pwm_info info;
	struct mtk_leds_info	*parent;
	struct led_debug_info debug;
	struct work_struct work;
};

struct mtk_leds_info {
	struct device *dev;
	int			nums;
	struct mtk_led_data leds[0];
};

static DEFINE_MUTEX(leds_mutex);
struct leds_desp_info *leds_info;
static BLOCKING_NOTIFIER_HEAD(mtk_leds_chain_head);

int mtk_leds_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mtk_leds_chain_head, nb);
}
EXPORT_SYMBOL_GPL(mtk_leds_register_notifier);

int mtk_leds_unregister_notifier(struct notifier_block *nb)

{
	return blocking_notifier_chain_unregister(&mtk_leds_chain_head, nb);
}
EXPORT_SYMBOL_GPL(mtk_leds_unregister_notifier);

int mtk_leds_call_notifier(unsigned long action, void *data)
{
	return blocking_notifier_call_chain(&mtk_leds_chain_head, action, data);
}
EXPORT_SYMBOL_GPL(mtk_leds_call_notifier);


static int call_notifier(int event, struct mtk_led_data *led_dat)
{
	int err;

	err = mtk_leds_call_notifier(event, &led_dat->conf);
	if (err)
		pr_info("notifier_call_chain error\n");
	return err;
}

/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/

static void led_debug_log(struct mtk_led_data *s_led,
		int level, int mappingLevel)
{
	unsigned long cur_time_mod = 0;
	unsigned long long cur_time_display = 0;

	s_led->debug.current_t = sched_clock();
	cur_time_display = s_led->debug.current_t;
	cur_time_mod = do_div(cur_time_display, 1000000000);

	sprintf(s_led->debug.buffer + strlen(s_led->debug.buffer),
		"T:%lld.%ld,L:%d L:%d map:%d    ",
		cur_time_display, cur_time_mod/1000000,
		s_led->conf.cdev.brightness, level, mappingLevel);

	s_led->debug.count++;

	if (level == 0 || s_led->debug.count >= 5 ||
		(s_led->debug.current_t - s_led->debug.last_t) > 1000000000) {
		pr_info("%s", s_led->debug.buffer);
		s_led->debug.count = 0;
		s_led->debug.buffer[strlen("[Light] Set directly ") +
			strlen(s_led->conf.cdev.name)] = '\0';
	}

	s_led->debug.last_t = sched_clock();
}


static int getLedDespIndex(char *name)
{
	int i = 0;

	while (i < leds_info->lens) {
		if (!strcmp(name, leds_info->leds[i]->name))
			return i;
		i++;
	}
	return -1;
}


/****************************************************************************
 * driver functions
 ***************************************************************************/

static void __led_pwm_set(struct led_pwm_info *led_info)
{
	int new_duty = led_info->duty;

	mutex_lock(&leds_mutex);

	pwm_config(led_info->pwm, new_duty, led_info->config.pwm_period_ns);
	if (new_duty == 0)
		pwm_disable(led_info->pwm);
	else
		pwm_enable(led_info->pwm);

	mutex_unlock(&leds_mutex);
}

static int led_level_pwm_set(struct mtk_led_data *led_dat,
				int brightness)
{
	unsigned int max;
	unsigned long long duty;

	brightness = min(brightness, led_dat->conf.max_level);
	if (brightness == led_dat->conf.level)
		return 0;

	led_dat->conf.level = brightness;
	max = (1 << led_dat->conf.trans_bits) - 1;
	duty = led_dat->info.config.pwm_period_ns;
	duty *= brightness;
	do_div(duty, max);

	if (led_dat->info.config.active_low)
		duty = led_dat->info.config.pwm_period_ns - duty;

	led_dat->info.duty = duty;

	__led_pwm_set(&led_dat->info);

	return 0;
}

int mt_leds_brightness_set(char *name, int level)
{
	struct mtk_led_data *led_dat;
	int index;

	index = getLedDespIndex(name);
	if (index < 0) {
		pr_notice("can not find leds by led_desp %s", name);
		return -1;
	}
	led_dat = container_of(leds_info->leds[index],
		struct mtk_led_data, desp);
	led_level_pwm_set(led_dat, level);
	led_dat->conf.level = level;

	return 0;
}
EXPORT_SYMBOL(mt_leds_brightness_set);

void mtk_led_work(struct work_struct *work)
{
	struct mtk_led_data *led_data =
	    container_of(work, struct mtk_led_data, work);

	led_level_pwm_set(led_data, led_data->conf.level);
}


#ifndef CONFIG_MTK_AAL_SUPPORT
static int led_pwm_disable(struct led_pwm_info *led_info)
{

	mutex_lock(&leds_mutex);

	pwm_config(led_info->pwm, 0, led_info->config.pwm_period_ns);
	pwm_disable(led_info->pwm);

	mutex_unlock(&leds_mutex);

	return 0;
}
#endif

static int led_level_set(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	int trans_level = 0;

	struct led_conf_info *led_conf =
		container_of(led_cdev, struct led_conf_info, cdev);
	struct mtk_led_data *led_dat =
		container_of(led_conf, struct mtk_led_data, conf);

	if (led_dat->brightness == brightness)
		return 0;

	led_dat->brightness = brightness;

	trans_level = (
		(((1 << led_dat->conf.trans_bits) - 1) * brightness
		+ (((1 << led_dat->conf.led_bits) - 1) / 2))
		/ ((1 << led_dat->conf.led_bits) - 1));

	led_debug_log(led_dat, brightness, trans_level);

#ifdef MET_USER_EVENT_SUPPORT
	if (enable_met_backlight_tag())
		output_met_backlight_tag(brightness);
#endif

#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	call_notifier(1, led_dat);
#endif
#ifdef CONFIG_MTK_AAL_SUPPORT
		disp_pq_notify_backlight_changed(trans_level);
#else
	led_level_pwm_set(led_dat, trans_level);
	led_dat->last_level = trans_level;

#endif
	return 0;

}


/****************************************************************************
 * add API for temperature control
 ***************************************************************************/

int setMaxBrightness(char *name, int percent, bool enable)
{
	struct mtk_led_data *led_dat;
	int max_l = 0, index = -1, limit_l = 0, cur_l = 0;

	index = getLedDespIndex(name);
	if (index < 0) {
		pr_notice("can not find leds by led_desp %s", name);
		return -1;
	}
	led_dat = container_of(leds_info->leds[index],
		struct mtk_led_data, desp);

	max_l = (1 << led_dat->conf.trans_bits) - 1;
	limit_l = (percent * max_l) / 100;
	pr_info("before: name: %s, percent : %d, limit_l : %d, enable: %d",
		leds_info->leds[index]->name, percent, limit_l, enable);
	if (enable) {
		led_dat->conf.max_level = limit_l;
		cur_l = min(led_dat->last_level, limit_l);
	} else if (!enable) {
		led_dat->conf.max_level = max_l;
		cur_l = led_dat->last_level;
	}
#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
	call_notifier(3, led_dat);
#endif

	if (led_dat->conf.cdev.brightness != 0)
		led_level_pwm_set(led_dat, cur_l);

	pr_info("after: name: %s, cur_l : %d, max_level : %d",
		led_dat->conf.cdev.name, cur_l, led_dat->conf.max_level);
	return 0;

}
EXPORT_SYMBOL(setMaxBrightness);

static int led_data_init(struct device *dev, struct mtk_led_data *s_led)
{
	int ret;

	s_led->conf.cdev.default_trigger = s_led->info.config.default_trigger;
	s_led->conf.cdev.max_brightness = s_led->info.config.max_brightness;
	s_led->conf.cdev.flags = LED_CORE_SUSPENDRESUME;
	s_led->conf.cdev.brightness_set_blocking = led_level_set;
	s_led->brightness = (1 << s_led->conf.trans_bits) - 1;
	s_led->conf.level = s_led->brightness;
	s_led->last_level = s_led->brightness;
	ret = devm_led_classdev_register(dev, &(s_led->conf.cdev));
	if (ret < 0) {
		pr_notice("led class register fail!");
		return ret;
	}
	pr_info("%s devm_led_classdev_register ok! ", s_led->conf.cdev.name);

	INIT_WORK(&s_led->work, mtk_led_work);
	sprintf(s_led->debug.buffer + strlen(s_led->debug.buffer),
		"[Light] Set %s directly ", s_led->conf.cdev.name);
	return 0;

}

static int led_pwm_config_add(struct device *dev,
		struct mtk_led_data *s_led, struct device_node *leds_np)
{
	struct pwm_args pargs;
	int ret = 0;

	if (leds_np != NULL)
		s_led->info.pwm = devm_of_pwm_get(dev, leds_np,
			s_led->info.config.name);
	else
		s_led->info.pwm = devm_pwm_get(dev, s_led->info.config.name);
	if (IS_ERR(s_led->info.pwm)) {
		ret = PTR_ERR(s_led->info.pwm);
		if (ret != -EPROBE_DEFER) {
			pr_notice("unable to request PWM for %s, err_code: %d\n",
				s_led->info.config.name, ret);
			goto err;
		}
	}

	pwm_apply_args(s_led->info.pwm);
	pwm_get_args(s_led->info.pwm, &pargs);

	s_led->info.config.pwm_period_ns = pargs.period;
	if (!s_led->info.config.pwm_period_ns && (pargs.period > 0))
		s_led->info.config.pwm_period_ns = pargs.period;

	pr_info("set led pwm OK! info.config.pwm_period_ns = %d!",
		s_led->info.config.pwm_period_ns);
	return ret;

 err:
	pr_notice("add pwm failed!\n");
	ret = -ENOMEM;
	return ret;

}

static int mtk_leds_parse_dt(struct device *dev,
		struct mtk_leds_info *m_leds)
{
	struct device_node *child;
	struct mtk_led_data *s_led;
	int ret = 0, num = 0, level = 102;
	const char *state;

	if (!dev->of_node) {
		pr_notice("Error load dts: node not exist!\n");
		return ret;
	}

	for_each_available_child_of_node(dev->of_node, child) {

		s_led = &(m_leds->leds[num]);
		ret = of_property_read_string(child, "label",
			&(s_led->conf.cdev.name));
		if (ret) {
			pr_info("Fail to read label property");
			goto out_led_dt;
		}
		ret = of_property_read_string(child, "pwm-names",
			&(s_led->info.config.name));
		if (ret) {
			pr_info("Fail to read pwm-names property");
			goto out_led_dt;
		}
		ret = of_property_read_string(child, "default-trigger",
			&(s_led->info.config.default_trigger));
		if (ret) {
			pr_info("Fail to read default-trigger property");
			s_led->info.config.default_trigger = NULL;
		}
		ret = of_property_read_u8(child, "active-low",
			&(s_led->info.config.active_low));
		if (ret)
			pr_info("Fail to read active-low property\n");
		ret = of_property_read_u32(child,
			"led-bits", &(s_led->conf.led_bits));
		if (ret) {
			pr_info("No led-bits, use default value 8");
			s_led->conf.led_bits = 8;
		}
		ret = of_property_read_u32(child,
			"max-brightness", &(s_led->info.config.max_brightness));
		if (ret) {
			pr_info("No max-brightness, use default value 255");
		s_led->info.config.max_brightness =
			(1 << s_led->conf.led_bits) - 1;
		}
		ret = of_property_read_u32(child,
			"trans-bits", &(s_led->conf.trans_bits));
		if (ret) {
			pr_info("No trans-bits, use default value 10");
			s_led->conf.trans_bits = 10;
		}
		s_led->conf.max_level = (1 << s_led->conf.trans_bits) - 1;
		ret = of_property_read_string(child, "default-state", &state);
		if (!ret) {
			if (!strcmp(state, "half"))
				level = s_led->info.config.max_brightness / 2;
			else if (!strcmp(state, "on"))
				level = s_led->info.config.max_brightness;
			else
				level = s_led->conf.level = 0;
		}
		pr_info("parse %s(%d) leds dt: %s, %s, %d, %d, %d\n",
			s_led->conf.cdev.name, num, s_led->info.config.name,
			s_led->info.config.default_trigger,
			s_led->info.config.active_low,
			s_led->info.config.max_brightness,
			s_led->conf.led_bits);
		s_led->desp.index = num;
		strncpy(s_led->desp.name, s_led->conf.cdev.name,
			strlen(s_led->conf.cdev.name));
		s_led->desp.index = num;
		leds_info->leds[num] = &s_led->desp;
		s_led->conf.cdev.brightness = level;
		ret = led_data_init(dev, s_led);
		if (ret)
			goto out_led_dt;
		led_pwm_config_add(dev, s_led, child);
		led_level_set(&s_led->conf.cdev, level);
		num++;
	}
	m_leds->nums = num;
	pr_info("load dts ok!");
	return ret;
out_led_dt:
	pr_notice("Error load dts node!\n");
	of_node_put(child);
	return ret;
}


/****************************************************************************
 * driver functions
 ***************************************************************************/

static int mtk_leds_probe(struct platform_device *pdev)
{

	struct device *dev = &pdev->dev;
	struct mtk_leds_info *m_leds;
	int ret, nums;

	pr_info("probe begain +++");

	nums = of_get_child_count(dev->of_node);
	pr_info("Load dts node nums: %d", nums);
	m_leds = devm_kzalloc(dev, (sizeof(struct mtk_leds_info) +
		(sizeof(struct mtk_led_data) * (nums))), GFP_KERNEL);
	if (!m_leds)
		goto err;

	leds_info = devm_kzalloc(dev, (sizeof(struct leds_desp_info) +
		sizeof(struct led_desp *) * (nums)),
		GFP_KERNEL);
	leds_info->lens = nums;
	if (!leds_info) {
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(pdev, m_leds);
	m_leds->dev = dev;
	ret = mtk_leds_parse_dt(&(pdev->dev), m_leds);
	if (ret) {
		pr_notice("Failed to parse devicetree!\n");
		goto err;
	}

	pr_info("probe end ---");
	return 0;
 err:
	pr_notice("Failed to probe!\n");
	ret = -ENOMEM;
	return ret;
}

static int mtk_leds_remove(struct platform_device *pdev)
{
	int i;
	struct mtk_leds_info *m_leds = dev_get_platdata(&pdev->dev);

	if (!m_leds)
		return 0;
	for (i = 0; i < m_leds->nums; i++) {
		if (!m_leds->leds[i].parent)
			continue;
		led_classdev_unregister(&m_leds->leds[i].conf.cdev);
		cancel_work_sync(&m_leds->leds[i].work);
		m_leds->leds[i].parent = NULL;
	}
	kfree(m_leds);
	m_leds = NULL;

	return 0;
}

static void mtk_leds_shutdown(struct platform_device *pdev)
{
	int i;
	struct mtk_leds_info *m_leds = dev_get_platdata(&pdev->dev);

	pr_info("Turn off backlight\n");

	for (i = 0; m_leds && i < m_leds->nums; i++) {
		if (!&(m_leds->leds[i]))
			continue;
#ifdef CONFIG_LEDS_BRIGHTNESS_CHANGED
		call_notifier(2, &(m_leds->leds[i]));
#ifdef CONFIG_MTK_AAL_SUPPORT
		continue;
#else
		led_pwm_disable(&(m_leds->leds[i].info));
#endif
#else
		led_pwm_disable(&(m_leds->leds[i].info));
#endif
	}

}

static const struct of_device_id of_mtk_pwm_leds_match[] = {
	{ .compatible = "mediatek,pwm-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_mtk_pwm_leds_match);

static struct platform_driver mtk_pwm_leds_driver = {
	.driver = {
		   .name = "mtk-pwm-leds",
		   .owner = THIS_MODULE,
		   .of_match_table = of_mtk_pwm_leds_match,
		   },
	.probe = mtk_leds_probe,
	.remove = mtk_leds_remove,
	.shutdown = mtk_leds_shutdown,

};

static int __init mtk_leds_init(void)
{
	int ret;

	pr_info("Leds init\n");
	ret = platform_driver_register(&mtk_pwm_leds_driver);

	if (ret) {
		pr_info("driver register error: %d\n", ret);
		return ret;
	}

	return ret;
}

static void __exit mtk_leds_exit(void)
{
	platform_driver_unregister(&mtk_pwm_leds_driver);
}

/* delay leds init, for (1)display has delayed to use clock upstream.
 * (2)to fix repeat switch battary and power supply caused BL KE issue,
 * battary calling bl .shutdown whitch need to call disp_pwm and display
 * function and they not yet probe.
 */
late_initcall(mtk_leds_init);
module_exit(mtk_leds_exit);
#endif
MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK Disp PWM Backlight Driver");
MODULE_LICENSE("GPL");


