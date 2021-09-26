/*
 * high_volt_vib.c
 *
 * code for vibrator
 *
 * Copyright (c) 2020 Huawei Technologies Co., Ltd.
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
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <mt-plat/mtk_pwm.h>
#include <mt-plat/mtk_pwm_hal_pub.h>
#include <securec.h>
#include <huawei_platform/hwpower/common_module/power_platform.h>
#include <high_volt_vib.h>

static struct regulator *flash_vib_id;
static struct wakeup_source high_lock;
static int flash_regulator_event(struct notifier_block *nb,
	unsigned long event, void *data);
static void high_vib_auto_pwm_freq_adapt(struct high_vib_data_t *vdata,
	int flash_volt_state);

static struct notifier_block flash_notifier_block = {
	.notifier_call = flash_regulator_event,
};

struct pwm_spec_config hv_pwm_config = {
	.pwm_no = HIGH_VOLT_DEFAULT_PWM_NO,
	.mode = PWM_MODE_OLD,
	.clk_div = CLK_DIV1, // support 1/2/4/8/16/32/64/128
	.clk_src = PWM_CLK_OLD_MODE_BLOCK, // 26M
	.pmic_pad = 0,
	.PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
	.PWM_MODE_OLD_REGS.GUARD_VALUE = GUARD_FALSE,
	.PWM_MODE_OLD_REGS.GDURATION = 0,
	.PWM_MODE_OLD_REGS.WAVE_NUM = 0, // 65535
	.PWM_MODE_OLD_REGS.DATA_WIDTH = 520, // 1300--20k   520--50k
	.PWM_MODE_OLD_REGS.THRESH = 520, // 1040 -- 416
};

static void high_vibrator_ldo_ctrl(struct led_classdev *cdev,
	enum led_brightness state)
{
	struct high_vib_data_t *vdata = container_of(cdev,
		struct high_vib_data_t, vib_dev);
	if (!vdata)
		return;
	pr_info("high_vibrator ldo ctrl\n");
}

static void high_vib_flash_power_onoff(struct high_vib_data_t *vdata, bool status)
{
	static bool pre_flash_power_satus = false;

	if (!flash_vib_id)
		return;

	if (pre_flash_power_satus == status)
		return;
	pre_flash_power_satus = status;
	if (status) {
		regulator_set_load(flash_vib_id, FULL_FLASH_CURRENT);
		regulator_enable(flash_vib_id);
		pr_info("%s set flash power on\n", __func__);
	} else {
		regulator_disable(flash_vib_id);
		pr_info("%s set flash power off\n", __func__);
	}
}
static void high_volt_duty_ratio_calc(struct high_vib_data_t *vdata)
{
	uint16_t pwm_width;
	uint16_t pwm_thresh;
	static int pre_duty_ratio = 0;

	if (pre_duty_ratio == vdata->duty_ratio)
		return;
	if (vdata->pwm_freq == 0)
		return;
	pre_duty_ratio = vdata->duty_ratio;
	pwm_width = FULL_PWM_CLK_FREQ / vdata->pwm_freq;
	pwm_thresh = pwm_width * vdata->duty_ratio / 100;
	pr_info("get pwm_width = %u, pwm_thre = %u\n", pwm_width, pwm_thresh);
	hv_pwm_config.PWM_MODE_OLD_REGS.DATA_WIDTH = pwm_width;
	hv_pwm_config.PWM_MODE_OLD_REGS.THRESH = pwm_thresh;
}

static void high_vib_pwm_power_cfg(struct high_vib_data_t *vdata, bool status)
{
	static bool pre_pwm_status = false;

	pr_info("%s start, status = %u, state = %d\n", __func__, status, vdata->state);
	if (status == pre_pwm_status)
		return;
	if (status) {
		high_volt_duty_ratio_calc(vdata);
		pwm_set_spec_config(&hv_pwm_config);
	} else {
		mt_pwm_disable(hv_pwm_config.pwm_no, 0);
	}
	pr_info("%s end\n", __func__);
	pre_pwm_status = status;
}

static void high_vib_n_ctrl_power_cfg(struct high_vib_data_t *vdata, bool status)
{
	static bool pre_n_ctrl_status = false;

	if (status == pre_n_ctrl_status)
		return;
	gpio_set_value(vdata->vib_n_pin, status);
	pre_n_ctrl_status = status;
}

static void high_vib_stop_haptic(struct high_vib_data_t *vdata)
{
	high_vib_pwm_power_cfg(vdata, false);
	high_vib_n_ctrl_power_cfg(vdata, false);
}

static void high_vib_reverse_haptic(struct high_vib_data_t *vdata)
{
	high_vib_pwm_power_cfg(vdata, false);
	udelay(1000);
	high_vib_n_ctrl_power_cfg(vdata, true);
}

static void high_vib_start_haptic(struct high_vib_data_t *vdata)
{
	high_vib_n_ctrl_power_cfg(vdata, false);
	high_vib_pwm_power_cfg(vdata, false);
	high_vib_pwm_power_cfg(vdata, true);
}

static void high_vib_haptic_duty_ratio_config(struct high_vib_data_t *vdata,
	uint32_t duty_ratio)
{
	vdata->duty_ratio = duty_ratio;
	high_vib_start_haptic(vdata);
}

static enum hrtimer_restart high_vibrator_timer_func(struct hrtimer *timer)
{
	struct high_vib_data_t *vdata =
		container_of(timer, struct high_vib_data_t, timer);

	pr_info("%s enter\n", __func__);
	queue_work(system_power_efficient_wq, &vdata->vibrator_work);
	return HRTIMER_NORESTART;
}

static void high_vib_ctrl_haptic_cfg(struct high_vib_data_t *vdata, int val)
{
	int dura_time;

	if (val == 0 && vdata->state != HIGH_VOLT_HAPTIC_STOP) {
		if (hrtimer_active(&vdata->timer))
			hrtimer_cancel(&vdata->timer);
		high_vib_reverse_haptic(vdata);
		hrtimer_start(&vdata->timer, ktime_set(HAPTIC_RESERVE_TIME / 1000,
			(HAPTIC_RESERVE_TIME % 1000) * 1000000), HRTIMER_MODE_REL);
		vdata->state = HIGH_VOLT_HAPTIC_RESERVE;
	} else if (val == 1 && vdata->state == 0 && vdata->duration != 0) {
		high_vib_flash_power_onoff(vdata, true);
		if (hrtimer_active(&vdata->timer))
			hrtimer_cancel(&vdata->timer);
		__pm_wakeup_event(&high_lock,
			jiffies_to_msecs(vdata->duration + HAPTIC_RESERVE_TIME));
		if (vdata->duration > FULL_PWM_TIME) {
			dura_time = FULL_PWM_TIME;
			high_vib_haptic_duty_ratio_config(vdata, FULL_PWM_DUTY_RATIO);
			vdata->state = HIGH_VOLT_RATIO_PWM_HAPTIC_START;
		} else {
			dura_time = vdata->duration;
			high_vib_haptic_duty_ratio_config(vdata, FULL_PWM_DUTY_RATIO);
			vdata->state = HIGH_VOLT_HAPTIC_RESERVE;
		}
		hrtimer_start(&vdata->timer, ktime_set(dura_time / 1000,
			(dura_time % 1000) * 1000000), HRTIMER_MODE_REL);
	}
}

static void high_vibrator_work_routine(struct work_struct *work)
{
	struct high_vib_data_t *vdata =
		container_of(work, struct high_vib_data_t, vibrator_work);

	int dura;

	pr_info("%s enter\n", __func__);
	null_pointer_err_check(vdata);
	mutex_lock(&vdata->lock);
	// here stop vibrator haptic
	switch (vdata->state) {
	case HIGH_VOLT_FULL_PWM_HAPTIC_START:
		pr_info("start vibrator haptic, duration = %d\n", vdata->duration);
		vdata->state = HIGH_VOLT_HAPTIC_RESERVE;
		break;
	case HIGH_VOLT_HAPTIC_STOP:
		vdata->state = HIGH_VOLT_HAPTIC_STOP;
		pr_info("stop vibrator haptic\n");
		high_vib_stop_haptic(vdata);
		high_vib_flash_power_onoff(vdata, false);
		break;
	case HIGH_VOLT_RATIO_PWM_HAPTIC_START:
		vdata->state = HIGH_VOLT_RATIO_TO_RESERVE;
		dura = vdata->duration - FULL_PWM_TIME;
		pr_info("start dupty haptic, duration = %d\n", dura);
		high_vib_auto_pwm_freq_adapt(vdata, vdata->fled_volt);
		if (hrtimer_active(&vdata->timer))
			hrtimer_cancel(&vdata->timer);
		hrtimer_start(&vdata->timer, ktime_set(dura / 1000,
			(dura % 1000) * 1000000), HRTIMER_MODE_REL);
		break;
	case HIGH_VOLT_RATIO_TO_RESERVE:
	case HIGH_VOLT_HAPTIC_RESERVE:
		vdata->state = HIGH_VOLT_HAPTIC_STOP;
		high_vib_reverse_haptic(vdata);
		if (hrtimer_active(&vdata->timer))
			hrtimer_cancel(&vdata->timer);
		hrtimer_start(&vdata->timer, ktime_set(HAPTIC_RESERVE_TIME / 1000,
			(HAPTIC_RESERVE_TIME % 1000) * 1000000), HRTIMER_MODE_REL);
		pr_info("reverse vibrator haptic\n");
		break;
	default:
		pr_info("default vibrator haptic\n");
		break;
	}
	mutex_unlock(&vdata->lock);
}

static ssize_t high_vib_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	static int power_state = 0;
	struct led_classdev *cdev = NULL;
	struct high_vib_data_t *vdata = NULL;

	pr_info("%s\n", __func__);
	if (!buf || !dev)
		return 0;
	cdev = dev_get_drvdata(dev);
	null_pointer_err_check_ret0(cdev);
	vdata = container_of(cdev, struct high_vib_data_t, vib_dev);
	null_pointer_err_check_ret0(vdata);

	if (power_state == 0) {
		high_vib_flash_power_onoff(vdata, true);
		power_state = 1;
	} else if (power_state == 1) {
		high_vib_flash_power_onoff(vdata, 0);
		power_state = 0;
	}

	pr_info("%s\n", __func__);
	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%d\n", power_state);
}

static ssize_t high_vib_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("%s\n", __func__);
	return count;
}

static ssize_t high_vib_duration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ktime_t time_rem;
	s64 time_ms = 0;
	struct led_classdev *cdev = NULL;
	struct high_vib_data_t *vdata = NULL;

	if (!buf || !dev)
		return 0;
	pr_info("%s\n", __func__);
	cdev = dev_get_drvdata(dev);
	null_pointer_err_check_ret0(cdev);
	vdata = container_of(cdev, struct high_vib_data_t, vib_dev);
	null_pointer_err_check_ret0(vdata);

	if (hrtimer_active(&vdata->timer)) {
		time_rem = hrtimer_get_remaining(&vdata->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%lld\n", time_ms);
}

static ssize_t high_vib_duration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = NULL;
	struct high_vib_data_t *vdata = NULL;
	int rc;

	if (!dev || !buf)
		return 0;

	cdev = dev_get_drvdata(dev);
	null_pointer_err_check_ret0(cdev);
	vdata = container_of(cdev, struct high_vib_data_t, vib_dev);
	null_pointer_err_check_ret0(vdata);

	rc = kstrtouint(buf, 0, &vdata->duration);
	pr_info("set duration time = %d, rc=%d\n", vdata->duration, rc);
	return count;
}

static ssize_t high_vib_activate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = NULL;
	struct high_vib_data_t *vdata = NULL;

	if (!buf || !dev)
		return 0;
	cdev = dev_get_drvdata(dev);
	null_pointer_err_check_ret0(cdev);
	vdata = container_of(cdev, struct high_vib_data_t, vib_dev);
	null_pointer_err_check_ret0(vdata);

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%d\n", vdata->state);
}

static ssize_t high_vib_activate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct led_classdev *cdev = NULL;
	struct high_vib_data_t *vdata = NULL;
	unsigned int val = 0;

	if (!buf || !dev)
		return 0;
	cdev = dev_get_drvdata(dev);
	null_pointer_err_check_ret0(cdev);
	vdata = container_of(cdev, struct high_vib_data_t, vib_dev);
	null_pointer_err_check_ret0(vdata);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	pr_info("%s: value=%d\n", __func__, val);
	mutex_lock(&vdata->lock);
	high_vib_ctrl_haptic_cfg(vdata, val);
	mutex_unlock(&vdata->lock);
	return count;
}

static DEVICE_ATTR(state, 0664, high_vib_state_show, high_vib_state_store);
static DEVICE_ATTR(duration, 0664, high_vib_duration_show, high_vib_duration_store);
static DEVICE_ATTR(activate, 0664, high_vib_activate_show, high_vib_activate_store);

static struct attribute *high_vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	NULL
};

static struct attribute_group high_vibrator_attribute_group = {
	.attrs = high_vibrator_attributes
};

static int high_vibrator_register_led_classdev(struct high_vib_data_t *vdata)
{
	struct led_classdev *cdev = &vdata->vib_dev;

	cdev->name = "vibrator";
	cdev->flags = LED_CORE_SUSPENDRESUME;
	cdev->brightness_set = high_vibrator_ldo_ctrl;
	cdev->default_trigger = "transient";

	return devm_led_classdev_register(vdata->dev, cdev);
}

static const struct of_device_id high_vibrator_match[] = {
	{.compatible = "high_volt,vibrator"},
	{},
};

MODULE_DEVICE_TABLE(of, high_vibrator_match);

static int high_volt_init_configs(struct high_vib_data_t *vdata,
	struct device_node *temp)
{
	int ret = 0;

	vdata->vib_n_pin = of_get_named_gpio(temp, "n_power-gpio", 0);
	if (vdata->vib_n_pin < 0) {
		pr_err("%s: no reset gpio provide\n", __func__);
		return -EINVAL;
	}
	ret = of_property_read_u32(temp, "pwm_no", &vdata->pwm_no);
	if (vdata->pwm_no < 0) {
		pr_err("%s: no pwm_no cfg\n", __func__);
		return -EINVAL;
	}
	ret = of_property_read_u32(temp, "pwm_freq", &vdata->pwm_freq);
	if (vdata->pwm_freq < 0) {
		pr_err("%s: get pwm_freq err\n", __func__);
		return -EINVAL;
	}

	pr_info("%s get n_gpio = %d, pwm_no = %u\n", __func__, vdata->vib_n_pin,
		vdata->pwm_no);
	hv_pwm_config.pwm_no = vdata->pwm_no;
	return ret;
}

static void high_vib_auto_pwm_freq_adapt(struct high_vib_data_t *vdata,
	int flash_volt_state)
{
	int battery_volt;

	switch (flash_volt_state) {
	case FLASH_PWOER_MODE_LOW_VOLT:
		battery_volt = power_platform_get_battery_voltage();
		pr_info("%s, battery_volt = %d\n", __func__, battery_volt);
		if (battery_volt <= 0)
			return;
		vdata->battery_ratio = HIGH_VB_RTAIO_BATTERY_REF * 100 / battery_volt;
		if (vdata->battery_ratio > FULL_PWM_DUTY_RATIO)
			vdata->battery_ratio = FULL_PWM_DUTY_RATIO;
		high_vib_haptic_duty_ratio_config(vdata, vdata->battery_ratio);
		break;
	case FLASH_PWOER_MODE_HIGH_VOLT:
		vdata->battery_ratio = HIGH_VOLT_DUTY_RATIO;
		high_vib_haptic_duty_ratio_config(vdata, vdata->battery_ratio);
		break;
	default:
		break;
	}
}

static void fled_adapt_delayed_work(struct work_struct *work)
{
	struct high_vib_data_t *vdata =
		container_of(work, struct high_vib_data_t, fled_adapt_work);

	null_pointer_err_check(vdata);
	mutex_lock(&vdata->lock);

	if (vdata->state == HIGH_VOLT_RATIO_TO_RESERVE)
		high_vib_auto_pwm_freq_adapt(vdata, vdata->fled_volt);

	mutex_unlock(&vdata->lock);
}

static int flash_regulator_event(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct fled_volt_t *tmp_fled_volt = NULL;
	struct high_vib_data_t *vdata = NULL;

	pr_info("%s Enter\n", __func__);

	if (event != REGULATOR_EVENT_VOLTAGE_CHANGE) {
		pr_info("regulator case is not available\n");
		return 0;
	}
	if (!data)
		return 0;
	vdata = container_of(nb, struct high_vib_data_t, flash_notifier_block);
	if (!vdata) {
		pr_err("%s vdata null\n", __func__);
		return 0;
	}
	tmp_fled_volt = data;

	vdata->fled_volt = tmp_fled_volt->fled_volt;
	queue_work(system_power_efficient_wq, &vdata->fled_adapt_work);
	pr_info("%s Exit test = %d\n", __func__, tmp_fled_volt->fled_volt);
	return 0;
}

static int high_vibrator_probe(struct platform_device *pdev)
{
	struct high_vib_data_t *vdata = NULL;
	int ret;
	struct device_node *temp = NULL;

	return 0;
	pr_info("%s in\n", __func__);
	if (!pdev)
		return -EINVAL;

	temp = pdev->dev.of_node;
	if (!of_match_node(high_vibrator_match, pdev->dev.of_node)) {
		dev_err(&pdev->dev, "dev node is not match. exiting.\n");
		return -ENODEV;
	}

	vdata = devm_kzalloc(
		&pdev->dev, sizeof(struct high_vib_data_t), GFP_KERNEL);
	if (!vdata)
		return -ENOMEM;

	vdata->dev = &pdev->dev;
	wakeup_source_init(&high_lock, "high_volt");
	/* register led classdev, use "transient" as default trigger */
	flash_vib_id = regulator_get(vdata->dev, "high_volt");
	if (!flash_vib_id)
		pr_err("%s get regulator fail\n", __func__);
	ret = high_vibrator_register_led_classdev(vdata);
	if (ret) {
		pr_err("failed to register led classdev\n");
		return ret;
	}
	ret = high_volt_init_configs(vdata, temp);
	if (ret) {
		pr_err("failed to init config\n");
		return ret;
	}
	ret = sysfs_create_group(&vdata->vib_dev.dev->kobj,
		&high_vibrator_attribute_group);
	if (ret)
		pr_err("unable create vibrator's min_timeout\n");
	hrtimer_init(&vdata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vdata->timer.function = high_vibrator_timer_func;
	INIT_WORK(&vdata->vibrator_work, high_vibrator_work_routine);
	INIT_WORK(&vdata->fled_adapt_work, fled_adapt_delayed_work);
	mutex_init(&vdata->lock);
	vdata->flash_notifier_block.notifier_call = flash_regulator_event;
	vdata->battery_ratio = HIGH_VOLT_DUTY_RATIO;
	vdata->fled_volt = FLASH_PWOER_MODE_HIGH_VOLT;
	ret = regulator_register_notifier(flash_vib_id, &vdata->flash_notifier_block);
	if (ret < 0)
		pr_err("%s regulator register notify fail\n", __func__);
	platform_set_drvdata(pdev, vdata);
	pr_info("%s init ok\n", __func__);
	return 0;
}

static int high_vibrator_remove(struct platform_device *pdev)
{
	struct high_vib_data_t *vdata = NULL;

	vdata = dev_get_drvdata(&pdev->dev);
	if (vdata == NULL) {
		pr_err("%s:failed to get drvdata\n", __func__);
		return -ENODEV;
	}

	sysfs_remove_group(&vdata->vib_dev.dev->kobj, &high_vibrator_attribute_group);

	return 0;
}

static struct platform_driver high_vibrator_driver = {
	.probe = high_vibrator_probe,
	.remove = high_vibrator_remove,
	.driver = {
		.name = "vibrator",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(high_vibrator_match),
	},
};

static int __init high_vibrator_init(void)
{
	return platform_driver_register(&high_vibrator_driver);
}

static void __exit high_vibrator_exit(void)
{
	platform_driver_unregister(&high_vibrator_driver);
}

late_initcall(high_vibrator_init);
module_exit(high_vibrator_exit);

MODULE_DESCRIPTION("high vibrator driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
