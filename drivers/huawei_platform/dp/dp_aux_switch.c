/*
 * dp_aux_switch.c
 *
 * dp aux switch driver
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

#include <huawei_platform/dp/dp_aux_switch.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <hw_dp_def.h>
#include <ana_hs_kit/ana_hs.h>

#define HWLOG_TAG dp_aux_switch
HWLOG_REGIST();

#define INVALID_GPIO   (-1)
#define SET_GPIO_HIGH  1
#define SET_GPIO_LOW   0

struct dp_aux_switch_priv {
	uint32_t ch_polarity;
	uint32_t aux_gpio;
	uint32_t uart_gpio;
	uint32_t dp_switch_gpio; // for M platform, dp_switch always needed
	uint32_t dp_power_control_gpio;
	bool from_fsa44xx;
	bool with_uart;
	bool aux_and_dp_same_switch;
	bool dp_switch_and_polarity_inverse;
	bool need_define_notifier_call;
	bool need_dp_power_control;
};

static struct dp_aux_switch_priv aux_switch_priv;

static void gpio_output_value_set(uint32_t gpio, uint32_t value)
{
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, value);
		HW_DP_INFO("set gpio %d %d\n", gpio, value);
	} else {
		HW_DP_ERR("invalid gpio %d\n", gpio);
	}
}

static void dp_switch_op(uint32_t value)
{
	int dp_switch_value = value;

	if (aux_switch_priv.aux_and_dp_same_switch)
		return;

	if (aux_switch_priv.dp_switch_and_polarity_inverse)
		dp_switch_value = (value == 0) ? 1 : 0;
	gpio_output_value_set(aux_switch_priv.dp_switch_gpio, dp_switch_value);
}

void dp_aux_switch_op(uint32_t value)
{
	dp_switch_op(value);
	if (aux_switch_priv.from_fsa44xx) {
		aux_switch_priv.ch_polarity = value;
		return;
	}
	gpio_output_value_set(aux_switch_priv.aux_gpio, value);
}
EXPORT_SYMBOL_GPL(dp_aux_switch_op);

void dp_aux_uart_switch_enable(void)
{
	if (aux_switch_priv.need_dp_power_control)
		gpio_output_value_set(aux_switch_priv.dp_power_control_gpio,
			SET_GPIO_HIGH);
	if (aux_switch_priv.from_fsa44xx) {
		// SBU bypass switch
		if (aux_switch_priv.ch_polarity) {
			HW_DP_INFO("dp plug in cross\n");
			ana_hs_plug_handle(DP_PLUG_IN_CROSS);
		} else {
			HW_DP_INFO("dp plug in\n");
			ana_hs_plug_handle(DP_PLUG_IN);
		}
	}

	if (aux_switch_priv.with_uart)
		gpio_output_value_set(aux_switch_priv.uart_gpio, SET_GPIO_HIGH);
}
EXPORT_SYMBOL_GPL(dp_aux_uart_switch_enable);

void dp_aux_uart_switch_disable(void)
{
	if (aux_switch_priv.from_fsa44xx) {
		// ENN H, EN1/EN2 00
		HW_DP_INFO("dp plug out\n");
		ana_hs_plug_handle(DP_PLUG_OUT);
	}

	if (aux_switch_priv.with_uart)
		gpio_output_value_set(aux_switch_priv.uart_gpio, SET_GPIO_LOW);
	if (aux_switch_priv.need_dp_power_control)
		gpio_output_value_set(aux_switch_priv.dp_power_control_gpio,
			SET_GPIO_LOW);
}
EXPORT_SYMBOL_GPL(dp_aux_uart_switch_disable);

bool is_need_define_notifier_call(void)
{
	return aux_switch_priv.need_define_notifier_call;
}
EXPORT_SYMBOL_GPL(is_need_define_notifier_call);

static void gpio_output_init(uint32_t gpio, const char *label, int value)
{
	int ret;

	if (!gpio_is_valid(gpio)) {
		HW_DP_ERR("invalid gpio %d\n", gpio);
		return;
	}

	ret = gpio_request(gpio, label);
	if (ret < 0)
		HW_DP_ERR("request gpio %d failed %d\n", gpio, ret);
	else
		gpio_direction_output(gpio, value);
}

static void dp_aux_switch_gpio_init(struct device_node *np)
{
	DP_GET_BOOL_DTS_PROP(aux_switch_priv.from_fsa44xx, np,
		aux_switch_from_fsa44xx);
	if (aux_switch_priv.from_fsa44xx)
		return;

	aux_switch_priv.aux_gpio = of_get_named_gpio(np, "dp-aux-gpio", 0);
	HW_DP_INFO("get aux switch gpio %d\n", aux_switch_priv.aux_gpio);
	gpio_output_init(aux_switch_priv.aux_gpio, "dp_aux_gpio", SET_GPIO_LOW);
}

static void dp_switch_gpio_init(struct device_node *np)
{
	DP_GET_BOOL_DTS_PROP(aux_switch_priv.aux_and_dp_same_switch, np,
		aux_and_dp_same_switch);
	if (aux_switch_priv.aux_and_dp_same_switch)
		return;

	aux_switch_priv.dp_switch_gpio = of_get_named_gpio(np,
		"dp-switch-gpio", 0);
	HW_DP_INFO("get dp switch gpio:%d\n", aux_switch_priv.dp_switch_gpio);
	gpio_output_init(aux_switch_priv.dp_switch_gpio, "dp_switch_gpio",
		SET_GPIO_LOW);
}

static void dp_uart_gpio_init(struct device_node *np)
{
	DP_GET_BOOL_DTS_PROP(aux_switch_priv.with_uart, np, aux_switch_with_uart);
	if (!aux_switch_priv.with_uart)
		return;

	aux_switch_priv.uart_gpio = of_get_named_gpio(np, "uart-gpio", 0);
	HW_DP_INFO("get uart gpio:%d\n", aux_switch_priv.uart_gpio);
	gpio_output_init(aux_switch_priv.uart_gpio, "uart_gpio", SET_GPIO_LOW);
}

static void dp_power_control_gpio_init(struct device_node *np)
{
	DP_GET_BOOL_DTS_PROP(aux_switch_priv.need_dp_power_control, np,
		need_dp_power_control);
	if (!aux_switch_priv.need_dp_power_control)
		return;

	aux_switch_priv.dp_power_control_gpio = of_get_named_gpio(np,
		"dp-power-control-gpio", 0);
	HW_DP_INFO("get aux switch gpio %d\n",
		aux_switch_priv.dp_power_control_gpio);
	gpio_output_init(aux_switch_priv.dp_power_control_gpio,
		"dp_power_control_gpio", SET_GPIO_LOW);
}

static int dp_aux_switch_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	aux_switch_priv.from_fsa44xx = false;
	aux_switch_priv.with_uart = false;
	aux_switch_priv.need_define_notifier_call = false;
	aux_switch_priv.need_dp_power_control = false;
	aux_switch_priv.aux_and_dp_same_switch = false;
	aux_switch_priv.dp_switch_and_polarity_inverse = false;
	aux_switch_priv.ch_polarity = 0;
	aux_switch_priv.aux_gpio = INVALID_GPIO;
	aux_switch_priv.uart_gpio = INVALID_GPIO;
	aux_switch_priv.dp_switch_gpio = INVALID_GPIO;
	aux_switch_priv.dp_power_control_gpio = INVALID_GPIO;

	if (!np) {
		HW_DP_ERR("device node not found\n");
		return -EINVAL;
	}

	DP_GET_BOOL_DTS_PROP(aux_switch_priv.need_define_notifier_call, np,
		register_usb_dp_notifier);
	DP_GET_BOOL_DTS_PROP(aux_switch_priv.dp_switch_and_polarity_inverse, np,
		dp_switch_and_polarity_inverse);

	dp_uart_gpio_init(np);
	dp_switch_gpio_init(np);
	dp_aux_switch_gpio_init(np);
	dp_power_control_gpio_init(np);

	return 0;
}

static int dp_aux_switch_remove(struct platform_device *pdev)
{
	if (!aux_switch_priv.aux_and_dp_same_switch) {
		if (gpio_is_valid(aux_switch_priv.dp_switch_gpio)) {
			gpio_free((unsigned int)aux_switch_priv.dp_switch_gpio);
			aux_switch_priv.dp_switch_gpio = INVALID_GPIO;
		}
	}

	if (!aux_switch_priv.from_fsa44xx) {
		if (gpio_is_valid(aux_switch_priv.aux_gpio)) {
			gpio_free((unsigned int)aux_switch_priv.aux_gpio);
			aux_switch_priv.aux_gpio = INVALID_GPIO;
		}
	}

	if (aux_switch_priv.with_uart) {
		if (gpio_is_valid(aux_switch_priv.uart_gpio)) {
			gpio_free((unsigned int)aux_switch_priv.uart_gpio);
			aux_switch_priv.uart_gpio = INVALID_GPIO;
		}
	}

	return 0;
}

static const struct of_device_id dp_aux_switch_match[] = {
	{ .compatible = "huawei,dp_aux_switch", },
	{},
};

static struct platform_driver dp_aux_switch_driver = {
	.driver = {
		.name = "dp_aux_switch",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dp_aux_switch_match),
	},
	.probe  = dp_aux_switch_probe,
	.remove = dp_aux_switch_remove,
};

static int __init dp_aux_switch_init(void)
{
	int ret;

	HW_DP_INFO("enter\n");
	ret = platform_driver_register(&dp_aux_switch_driver);
	if (ret < 0) {
		HW_DP_ERR("register driver failed %d\n", ret);
		return ret;
	}

	HW_DP_INFO("success\n");
	return 0;
}

static void __exit dp_aux_switch_exit(void)
{
	HW_DP_INFO("enter\n");
	platform_driver_unregister(&dp_aux_switch_driver);
}

module_init(dp_aux_switch_init);
module_exit(dp_aux_switch_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei dp aux driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
