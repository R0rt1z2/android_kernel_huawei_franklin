/*
 * msdc_mmc_sdsim.c
 *
 * add nano sd functions.
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

#include "msdc_mmc_sdsim.h"
#include <mt6360/inc/mt6360_ldo.h>
#include <msdc_io.h>
#include <mt-plat/mtk_ccci_common.h>
#include <mmc/core/core.h>
#include <mmc/core/card.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#define MUX_SDSIM_LOG_TAG "[MUX_SDSIM][mmc1]"

enum sdsim_gpio_type {
	GPIO_SD_FUNCTION0_CLK_NP = 0,
	GPIO_SD_FUNCTION1_CLK_NP,
	GPIO_SD_FUNCTION0_CMD_NP,
	GPIO_SD_FUNCTION1_CMD_PU,
	GPIO_SD_FUNCTION0_DATA0_NP,
	GPIO_SD_FUNCTION1_DATA0_PU,
	GPIO_SD_FUNCTION0_DATA1_NP,
	GPIO_SD_FUNCTION1_DATA1_PU,
	GPIO_SD_FUNCTION0_DATA2_NP,
	GPIO_SD_FUNCTION1_DATA2_PU,
	GPIO_SD_FUNCTION0_DATA3_NP,
	GPIO_SD_FUNCTION1_DATA3_PU,
	GPIO_SIM_FUNTION0_CLK_NP,
	GPIO_SIM_FUNTION1_CLK_PD,
	GPIO_SIM_FUNTION0_RST_NP,
	GPIO_SIM_FUNTION1_RST_PD,
	GPIO_SIM_FUNTION0_DATA_NP,
	GPIO_SIM_FUNTION1_DATA_PU,
	GPIO_SDSIM_NUM
};

struct sdsim_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpio_ctrl;
};

struct pinctrl *g_pinctrl_storage;
struct semaphore g_sem_mux_sdsim_detect;
static struct sdsim_gpio_attr g_sdsim_gpios[GPIO_SDSIM_NUM] = {
	[GPIO_SD_FUNCTION0_CLK_NP]   = {"sd_clk_function0_np", false, NULL},
	[GPIO_SD_FUNCTION1_CLK_NP]   = {"sd_clk_function1_np", false, NULL},
	[GPIO_SD_FUNCTION0_CMD_NP]   = {"sd_cmd_np", false, NULL},
	[GPIO_SD_FUNCTION1_CMD_PU]   = {"sd_cmd_pu", false, NULL},
	[GPIO_SD_FUNCTION0_DATA0_NP] = {"sd_data0_np", false, NULL},
	[GPIO_SD_FUNCTION1_DATA0_PU] = {"sd_data0_pu", false, NULL},
	[GPIO_SD_FUNCTION0_DATA1_NP] = {"sd_data1_np", false, NULL},
	[GPIO_SD_FUNCTION1_DATA1_PU] = {"sd_data1_pu", false, NULL},
	[GPIO_SD_FUNCTION0_DATA2_NP] = {"sd_data2_np", false, NULL},
	[GPIO_SD_FUNCTION1_DATA2_PU] = {"sd_data2_pu", false, NULL},
	[GPIO_SD_FUNCTION0_DATA3_NP] = {"sd_data3_np", false, NULL},
	[GPIO_SD_FUNCTION1_DATA3_PU] = {"sd_data3_pu", false, NULL},
	[GPIO_SIM_FUNTION0_CLK_NP]   = {"sim_clk_function0_np", false, NULL},
	[GPIO_SIM_FUNTION1_CLK_PD]   = {"sim_clk_function1_pd", false, NULL},
	[GPIO_SIM_FUNTION0_RST_NP]   = {"sim_rst_function0_np", false, NULL},
	[GPIO_SIM_FUNTION1_RST_PD]   = {"sim_rst_function1_pd", false, NULL},
	[GPIO_SIM_FUNTION0_DATA_NP]  = {"sim_data_function0_np", false, NULL},
	[GPIO_SIM_FUNTION1_DATA_PU]  = {"sim_data_function1_pu", false, NULL},
};

/*
 * sd_sim_group_io
 * NUMBER     FUNCTION0        FUNCTION1
 * GPIO51       GPIO           SD_CLK
 * GPIO52       GPIO           SD_CMD
 * GPIO53       GPIO           SD_DATA3
 * GPIO54       GPIO           SD_DATA0
 * GPIO55       GPIO           SD_DATA2
 * GPIO56       GPIO           SD_DATA1
 * GPIO45       GPIO           SIM2_CLK
 * GPIO46       GPIO           SIM2_RST
 * GPIO47       GPIO           SIM2_DATA
 */
int g_gpio_number_for_sd_clk   = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sd_cmd   = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sd_data0 = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sd_data1 = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sd_data2 = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sd_data3 = SDSIM_GPIO_DEFAULT_NUMBER;
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
int g_gpio_number_for_sim_clk  = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sim_rst  = SDSIM_GPIO_DEFAULT_NUMBER;
int g_gpio_number_for_sim_data = SDSIM_GPIO_DEFAULT_NUMBER;
#endif
static int g_switch_gpio_number = SWITCH_GPIO_DEFAULT_NUMBER;
static int g_switch_sd_side_value;
int g_sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_UNDETECTED;
struct semaphore g_sem_mux_sdsim_detect;
struct msdc_host *g_host_from_sd_module;

#define SIMHOTPLUG_SIM_CARD_IN 1
#define STATUS_SIM 5
#define STATUS_SD 6
#define STATUS_NO_CARD 7

void sdsim_set_current_sd_clk_gpios(int sd_clk_gpio)
{
	g_gpio_number_for_sd_clk = sd_clk_gpio;
}

void sdsim_set_current_sd_cmd_gpios(int sd_cmd_gpio)
{
	g_gpio_number_for_sd_cmd = sd_cmd_gpio;
}

void sdsim_set_current_sd_data0_gpios(int sd_data0_gpio)
{
	g_gpio_number_for_sd_data0 = sd_data0_gpio;
}

void sdsim_set_current_sd_data1_gpios(int sd_data1_gpio)
{
	g_gpio_number_for_sd_data1 = sd_data1_gpio;
}

void sdsim_set_current_sd_data2_gpios(int sd_data2_gpio)
{
	g_gpio_number_for_sd_data2 = sd_data2_gpio;
}

void sdsim_set_current_sd_data3_gpios(int sd_data3_gpio)
{
	g_gpio_number_for_sd_data3 = sd_data3_gpio;
}

void sdsim_set_current_sim_gpios(int sim_clk_gpio, int sim_rst_gpio,
				 int sim_data_gpio)
{
	g_gpio_number_for_sim_clk  = sim_clk_gpio;
	g_gpio_number_for_sim_rst  = sim_rst_gpio;
	g_gpio_number_for_sim_data = sim_data_gpio;
}

int sdsim_get_current_detect_status(void)
{
	return g_sd_sim_detect_status_current;
}

enum sdsim_result sdsim_switch_gpio_change(int set_side_status)
{
	int err;
	int side_value;

	if (set_side_status != SWITCH_GPIO_SD_SIDE &&
		set_side_status != SWITCH_GPIO_SIM_SIDE) {
		pr_err("%s %s argument set_side_status=%d is invalid\n",
		       MUX_SDSIM_LOG_TAG, __func__, set_side_status);
		return SDSIM_PARAM_INVALID;
	}

	/* dts "switch-sd-side" value:
	 * 1: switch high level to SD, switch low level to SIM
	 * 0: switch low level to SD, switch high level to SIM
	 */
	if (g_switch_sd_side_value == 1) {
		if (set_side_status == SWITCH_GPIO_SD_SIDE)
			side_value = 1;
		if (set_side_status == SWITCH_GPIO_SIM_SIDE)
			side_value = 0;
	} else {
		if (set_side_status == SWITCH_GPIO_SD_SIDE)
			side_value = 0;
		if (set_side_status == SWITCH_GPIO_SIM_SIDE)
			side_value = 1;
	}
	pr_info("%s %s side_value = %d\n",
		MUX_SDSIM_LOG_TAG, __func__, side_value);

	if (g_switch_gpio_number != SWITCH_GPIO_DEFAULT_NUMBER) {
		err = gpio_request(g_switch_gpio_number, "switch_gpio_nums");
		if (err < 0) {
			pr_err("%s %s gpio_request fail,gpio_num=%d,err=%d\n",
				MUX_SDSIM_LOG_TAG, __func__,
				g_switch_gpio_number, err);
			return SDSIM_GPIO_REQUEST_FAIL;
		}
		err = gpio_direction_output(g_switch_gpio_number, side_value);
		if (err) {
			pr_err("%s %s gpio_direc set fail,gpio_num=%d, "
			       "side_value=%d, err=%d\n", MUX_SDSIM_LOG_TAG,
			       __func__, g_switch_gpio_number, err);
			gpio_free(g_switch_gpio_number);
			return SDSIM_SET_DIRECT_OUT_FAIL;
		}
		gpio_set_value(g_switch_gpio_number, side_value);
		gpio_free(g_switch_gpio_number);
	}
	return SDSIM_OK;
}

void sdsim_set_switch_default_gpio(int switch_gpio_num)
{
	g_switch_gpio_number = switch_gpio_num;
}

void sdsim_set_switch_sd_side_value(int switch_sd_side_num)
{
	g_switch_sd_side_value = switch_sd_side_num;
}


#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
static enum sdsim_result
sdsim_set_gpio_direction(int gpio_num, const char *gpio_name,
			 int direction, int power_level)
{
	/*
	 * direction: 0 = input, 1 = output;
	 * power_level: 0 = lowlevel; 1 = highlevel
	 */
	int err;
	enum sdsim_result ret = SDSIM_OK;

	if (gpio_num == SDSIM_GPIO_DEFAULT_NUMBER) {
		pr_err("%s argument gpionumber=%d is invalid, return\n",
		       MUX_SDSIM_LOG_TAG, __func__, gpio_num);
		return SDSIM_PARAM_INVALID;
	}

	if (!gpio_name) {
		pr_err("%s %s argument gpio_name is null, return\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return SDSIM_PARAM_INVALID;
	}

	if (direction != 0 && direction != 1) {
		pr_err("%s %s argument direction=%d is invalid, return\n",
			MUX_SDSIM_LOG_TAG, __func__, direction);
		return SDSIM_PARAM_INVALID;
	}

	if (power_level != 0 && power_level != 1) {
		pr_err("%s %s argument power_level=%d is invalid, return\n",
			MUX_SDSIM_LOG_TAG, __func__, power_level);
		return SDSIM_PARAM_INVALID;
	}

	err = gpio_request(gpio_num, gpio_name);
	if (err < 0) {
		pr_err("%s %s gpio_request failed, gpio_num=%d, gpio_name = %s,"
		       "err=%d\n", MUX_SDSIM_LOG_TAG, __func__, gpio_num,
		       gpio_name, err);
		return SDSIM_GPIO_REQUEST_FAIL;
	}

	if (!direction) {
		err = gpio_direction_input(gpio_num);
		if (err) {
			ret = SDSIM_SET_DIRECT_IN_FAIL;
			goto end;
		}
	} else {
		err = gpio_direction_output(gpio_num, power_level);
		if (err) {
			ret = SDSIM_SET_DIRECT_OUT_FAIL;
			goto end;
		}
		gpio_set_value(gpio_num, power_level);
	}

end:
	if (err)
		pr_err("%s %s set gpio%d(%s) %s(%s) fail err=%d ret=%d ",
			MUX_SDSIM_LOG_TAG, __func__, gpio_num, gpio_name,
			direction ? "output" : "input",
			power_level ? "high" : "low", err, ret);
	gpio_free(gpio_num);
	return ret;
}
#endif

enum sdsim_result sdsim_gpio_probe(struct device *dev)
{
	enum sdsim_result ret = SDSIM_OK;
	int result;
	int i;

	if (dev == NULL) {
		ret = SDSIM_PARAM_INVALID;
		pr_err("%s %s argument:dev is invalid\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return ret;
	}

	g_pinctrl_storage = devm_pinctrl_get(dev);
	if (IS_ERR(g_pinctrl_storage)) {
		result = PTR_ERR(g_pinctrl_storage);
		pr_err("%s %s Cannot find g_pinctrl_storage, result: %d\n",
				MUX_SDSIM_LOG_TAG, __func__, result);
		ret = SDSIM_GET_PINCTL_FAIL;
		return ret;
	}
	for (i = 0; i < ARRAY_SIZE(g_sdsim_gpios); i++) {
		g_sdsim_gpios[i].gpio_ctrl =
			pinctrl_lookup_state(g_pinctrl_storage,
					     g_sdsim_gpios[i].name);
		if (IS_ERR(g_sdsim_gpios[i].gpio_ctrl)) {
			result = PTR_ERR(g_sdsim_gpios[i].gpio_ctrl);
			pr_err("%s %s pinctrl lookup state %s fail, result: %d\n",
				MUX_SDSIM_LOG_TAG, __func__,
				g_sdsim_gpios[i].name, result);
			ret = SDSIM_GET_GPIOCTL_ERROR;
			return ret;
		}
		g_sdsim_gpios[i].gpio_prepare = true;
	}
	return ret;
}

enum sdsim_result msdc_sdsim_gpio_probe(struct device *dev,
					struct msdc_host *host)
{
	enum sdsim_result ret = SDSIM_OK;

	if (host == NULL) {
		pr_err("%s %s argument:host is invalid\n",
			MUX_SDSIM_LOG_TAG, __func__);
		ret = SDSIM_PARAM_INVALID;
		return ret;
	}
	if (dev == NULL) {
		pr_err("%s %s [msdc%d] argument:dev is invalid\n",
			MUX_SDSIM_LOG_TAG, __func__, host->id);
		ret = SDSIM_PARAM_INVALID;
		return ret;
	}
	if (host->mux_sdsim == 1) {
		ret = sdsim_gpio_probe(dev);
		if (ret != SDSIM_OK)
			pr_err("%s %s gpio parse error\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return ret;
	}

	return ret;
}

enum sdsim_result msdc_sem_init(struct msdc_host *host)
{
	enum sdsim_result ret = SDSIM_OK;

	if (host == NULL) {
		pr_err("%s %s argument:host is invalid\n",
			MUX_SDSIM_LOG_TAG, __func__);
		ret = SDSIM_PARAM_INVALID;
		return ret;
	}

	if (host->mux_sdsim == 1)
		sema_init(&g_sem_mux_sdsim_detect, 1);

	return ret;
}

enum sdsim_result msdc_sdsim_set_sd_host(struct msdc_host *host)
{
	enum sdsim_result ret = SDSIM_OK;

	if (host == NULL) {
		pr_err("%s %s argument:host is invalid\n",
			MUX_SDSIM_LOG_TAG, __func__);
		ret = SDSIM_PARAM_INVALID;
		return ret;
	}
	g_host_from_sd_module = host;

	return ret;
}

static struct msdc_host *msdc_sdsim_get_sd_host(void)
{
	return g_host_from_sd_module;
}

static enum sdsim_result sdsim_gpio_select(enum sdsim_gpio_type type)
{
	enum sdsim_result ret = SDSIM_OK;
	int result;

	if (type < 0 || type >= GPIO_SDSIM_NUM) {
		pr_err("%s %s, error, invaild gpio type %d\n",
				MUX_SDSIM_LOG_TAG, __func__, type);
		return SDSIM_PARAM_INVALID;
	}

	if (!g_sdsim_gpios[type].gpio_prepare) {
		pr_err("%s %s, error, gpio type %d not prepared\n",
				MUX_SDSIM_LOG_TAG, __func__, type);
		return SDSIM_GET_GPIO_CTRL_INFO_ERROR;
	}

	result = pinctrl_select_state(g_pinctrl_storage,
				      g_sdsim_gpios[type].gpio_ctrl);
	if (result) {
		pr_err("%s %s, error, can not set gpio type %d, result %d\n",
		       MUX_SDSIM_LOG_TAG, __func__, type, result);
		return SDSIM_PINCTL_SELECT_STATE_FAIL;
	}
	return ret;
}

static void config_sdsim_mode_sd_normal_detect(void)
{
	sdsim_gpio_select(GPIO_SD_FUNCTION1_CLK_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION1_CMD_PU);
	sdsim_gpio_select(GPIO_SD_FUNCTION1_DATA0_PU);
	sdsim_gpio_select(GPIO_SD_FUNCTION1_DATA1_PU);
	sdsim_gpio_select(GPIO_SD_FUNCTION1_DATA2_PU);
	sdsim_gpio_select(GPIO_SD_FUNCTION1_DATA3_PU);
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
	/* sim2 gpio set np and input, this will not affect nano sd. */
	sdsim_gpio_select(GPIO_SIM_FUNTION0_CLK_NP);
	sdsim_gpio_select(GPIO_SIM_FUNTION0_RST_NP);
	sdsim_gpio_select(GPIO_SIM_FUNTION0_DATA_NP);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_clk,
				       "sim_clk_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_rst,
				       "sim_rst_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_data,
				       "sim_data_gpio_nums", 0, 0);
#endif
}

static void config_sdsim_mode_sd_idle_detect(void)
{
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CLK_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CMD_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA0_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA1_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA2_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA3_NP);
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
	/* sim2 gpio set np and input, this will not affect nano sd. */
	sdsim_gpio_select(GPIO_SIM_FUNTION0_CLK_NP);
	sdsim_gpio_select(GPIO_SIM_FUNTION0_RST_NP);
	sdsim_gpio_select(GPIO_SIM_FUNTION0_DATA_NP);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_clk,
				       "sim_clk_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_rst,
				       "sim_rst_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sim_data,
				       "sim_data_gpio_nums", 0, 0);
#endif
}

static void config_sdsim_mode_sim_normal_detect(void)
{
	/* sim2 gpio set*/
	sdsim_gpio_select(GPIO_SIM_FUNTION1_CLK_PD);
	sdsim_gpio_select(GPIO_SIM_FUNTION1_RST_PD);
	sdsim_gpio_select(GPIO_SIM_FUNTION1_DATA_PU);
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
	/* nano sd gpio set np and input, this will not affect sim. */
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CLK_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CMD_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA0_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA1_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA2_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA3_NP);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_clk,
				       "sd_clk_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_cmd,
				       "sd_cmd_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data0,
				       "sd_data0_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data1,
				       "sd_data1_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data2,
				       "sd_data2_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data3,
				       "sd_data3_gpio_nums", 0, 0);
#endif
}

static void config_sdsim_mode_sim_idle_detect(void)
{
	/* sim2 gpio set*/
	sdsim_gpio_select(GPIO_SIM_FUNTION1_CLK_PD);
	sdsim_gpio_select(GPIO_SIM_FUNTION1_RST_PD);
	sdsim_gpio_select(GPIO_SIM_FUNTION1_DATA_PU);
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
	/* nano sd gpio set np and input, this will not affect sim. */
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CLK_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_CMD_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA0_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA1_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA2_NP);
	sdsim_gpio_select(GPIO_SD_FUNCTION0_DATA3_NP);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_clk,
				       "sd_clk_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_cmd,
				       "sd_cmd_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data0,
				       "sd_data0_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data1,
				       "sd_data1_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data2,
				       "sd_data2_gpio_nums", 0, 0);
	(void)sdsim_set_gpio_direction(g_gpio_number_for_sd_data3,
				       "sd_data3_gpio_nums", 0, 0);
#endif
}

/* driect connection solustion */
/* SDSIM_MODE_GPIO_DETECT mode is not used currently, reserved */
/*
 * GpioMode/GpioNum  Func  PullType Driver SlewRate GpioDirection OutputValue
 *
 * SDSIM_MODE_GPIO_DETECT
 * NA
 *
 * SDSIM_MODE_SD_NORMAL
 * GPIO_51        Function1  NP    8mA      -    output     -
 * GPIO_52        Function1  PU    8mA      -    output     -
 * GPIO_53        Function1  PU    8mA      -    output     -
 * GPIO_54        Function1  PU    8mA      -    output     -
 * GPIO_55        Function1  PU    8mA      -    output     -
 * GPIO_56        Function1  PU    8mA      -    output     -
 * GPIO_45        Function0  NP    4mA      -    input      -
 * GPIO_46        Function0  NP    4mA      -    input      -
 * GPIO_47        Function0  NP    4mA      -    input      -
 *
 * SDSIM_MODE_SD_IDLE
 * GPIO_51        Function0  NP    8mA      -    input      -
 * GPIO_52        Function0  NP    8mA      -    input      -
 * GPIO_53        Function0  NP    8mA      -    input      -
 * GPIO_54        Function0  NP    8mA      -    input      -
 * GPIO_55        Function0  NP    8mA      -    input      -
 * GPIO_56        Function0  NP    8mA      -    input      -
 * GPIO_45        Function0  NP    4mA      -    input      -
 * GPIO_46        Function0  NP    4mA      -    input      -
 * GPIO_47        Function0  NP    4mA      -    input      -
 *
 * SDSIM_MODE_SIM_IDLE
 * GPIO_51        Function0  NP    8mA      -    input      -
 * GPIO_52        Function0  NP    8mA      -    input      -
 * GPIO_53        Function0  NP    8mA      -    input      -
 * GPIO_54        Function0  NP    8mA      -    input      -
 * GPIO_55        Function0  NP    8mA      -    input      -
 * GPIO_56        Function0  NP    8mA      -    input      -
 * GPIO_45        Function0  NP    4mA      -      -        -
 * GPIO_46        Function0  NP    4mA      -      -        -
 * GPIO_47        Function0  NP    4mA      -      -        -
 *
 * SDSIM_MODE_SIM_NORMAL
 * GPIO_51        Function0  NP    8mA      -    input      -
 * GPIO_52        Function0  NP    8mA      -    input      -
 * GPIO_53        Function0  NP    8mA      -    input      -
 * GPIO_54        Function0  NP    8mA      -    input      -
 * GPIO_55        Function0  NP    8mA      -    input      -
 * GPIO_56        Function0  NP    8mA      -    input      -
 * GPIO_45        Function1  PD    4mA      -       -       -
 * GPIO_46        Function1  PD    4mA      -       -       -
 * GPIO_47        Function1  PU    4mA      -       -       -
 */
int sdsim_config_sdsim_gpio_mode(enum sdsim_gpio_mode gpio_mode)
{
	pr_info("%s %s config gpio mode =%d\n",
		MUX_SDSIM_LOG_TAG, __func__, gpio_mode);
	if (gpio_mode == SDSIM_MODE_GPIO_DETECT)
		/* set sd_idle as gpio detect */
		config_sdsim_mode_sd_idle_detect();
	else if (gpio_mode == SDSIM_MODE_SD_NORMAL)
		config_sdsim_mode_sd_normal_detect();
	else if (gpio_mode == SDSIM_MODE_SD_IDLE)
		config_sdsim_mode_sd_idle_detect();
	else if (gpio_mode == SDSIM_MODE_SIM_NORMAL)
		config_sdsim_mode_sim_normal_detect();
	else if (gpio_mode == SDSIM_MODE_SIM_IDLE)
		config_sdsim_mode_sim_idle_detect();
	return 0;
}
/*
 * temp value for work around after modem set raw register here,
 * but ap can't set same value again
 */
static void sdsim_fix_sim_3_0_change_sd_1_8(struct msdc_host *host)
{
	int ret = 0;

	if (!host->mmc->supply.vqmmc ||
	    !host->mmc->supply.vmmc ||
	    !host->mmc->supply.vsim2) {
		pr_err("%s %s supply is NULL\n", MUX_SDSIM_LOG_TAG, __func__);
		return;
	}
	/* IO power up with 1.8V */
	msdc_ldo_power(LDO_POWER_ON, host->mmc->supply.vqmmc,
		       VOL_1800, &host->power_io);
	/* VMCH keep power off. VSIM2 power up with 1.8V */
	mmc_card_ldo_power(host, LDO_POWER_ON);
}

static int sdsim_detect_cmd_status(int current_module, int detect_result)
{
	int result = STATUS_PLUG_IN;
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
	struct msdc_host *host = msdc_sdsim_get_sd_host();
#endif

	if (!detect_result) {
		g_sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SD;
		pr_info("%s %s SD is inserted and detected now(CMD1 success),"
			" go with SD\n", MUX_SDSIM_LOG_TAG, __func__);
		if (current_module == MODULE_SD)
			return STATUS_PLUG_IN;
		else if (current_module == MODULE_SIM)
			return STATUS_PLUG_OUT;
	} else {
#ifdef CONFIG_MSDC_SDSIM_DIRECT_CONNECT
		msdc_ldo_power(LDO_POWER_ON, host->mmc->supply.vqmmc,
			       VOL_3000, &host->power_io);
		pr_info("%s %s VMC still on with 3.0V for SIM",
			MUX_SDSIM_LOG_TAG, __func__);
#endif
		sdsim_switch_gpio_change(SWITCH_GPIO_SIM_SIDE);
		sdsim_config_sdsim_gpio_mode(SDSIM_MODE_SIM_NORMAL);
		g_sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SIM;
		pr_info("%s %s SIM is inserted and detected now(CMD1 fail),"
			" go with SIM\n", MUX_SDSIM_LOG_TAG, __func__);
		if (current_module == MODULE_SD)
			return STATUS_PLUG_OUT;
		else if (current_module == MODULE_SIM)
			return STATUS_PLUG_IN;
	}

	return result;
}

static int sdsim_send_cmd_detect(struct msdc_host *host, int current_module)
{
	int detect_result;
	int result;

	mmc_claim_host(host->mmc);

	sdsim_fix_sim_3_0_change_sd_1_8(host);
	sdsim_config_sdsim_gpio_mode(SDSIM_MODE_SD_NORMAL);
	sdsim_switch_gpio_change(SWITCH_GPIO_SD_SIDE);

	msleep(SLEEP_MS_TIME_FOR_DETECT_UNSTABLE);
	pr_info("%s %s enter CMD1-RESPONSE STATUS detect stage "
		 "after sleep 20 ms\n", MUX_SDSIM_LOG_TAG, __func__);
	detect_result = mmc_detect_sd_or_mmc(host->mmc);

	msleep(SLEEP_MS_TIME_FOR_DETECT_UNSTABLE);
	pr_debug("%s %s enter OVER-ALL STATUS detect stage after sleep 20 ms\n",
		 MUX_SDSIM_LOG_TAG, __func__);
	result = sdsim_detect_cmd_status(current_module, detect_result);

	mmc_release_host(host->mmc);

	return result;
}

static int sim_plug_in(void)
{
	int result;
	struct msdc_host *host = msdc_sdsim_get_sd_host();

	if (!host) {
		pr_err("%s %s host=NULL,current_module=%d,This is Error,"
			" maybe MODULE_SIM,just return STATUS_PLUG_IN and"
			" let SIM go on\n", MUX_SDSIM_LOG_TAG, __func__,
			MODULE_SIM);
		/* This is sim */
		return STATUS_PLUG_IN;
	}

        mmc_claim_host(host->mmc);
	down(&g_sem_mux_sdsim_detect);

	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SD) {
		pr_info("%s %s MODULE_SIM find SD card already detected,"
			"just return STATUS_PLUG_OUT and do nothing\n",
			MUX_SDSIM_LOG_TAG, __func__);
		result = STATUS_PLUG_OUT;
		up(&g_sem_mux_sdsim_detect);
                mmc_release_host(host->mmc);
		return result;
	}

	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SIM) {
		pr_info("%s %s current_module=%d now re-entered but same"
			" detect status, just return STATUS_PLUG_IN and"
			" do nothing\n",
			MUX_SDSIM_LOG_TAG, __func__, MODULE_SIM);
		result = STATUS_PLUG_IN;
		up(&g_sem_mux_sdsim_detect);
                mmc_release_host(host->mmc);
		return result;
	}

	result = sdsim_send_cmd_detect(host, MODULE_SIM);
	up(&g_sem_mux_sdsim_detect);
        mmc_release_host(host->mmc);
	return result;
}

static int sd_plug_in(struct msdc_host *host)
{
	int result = 0;

	if (!host) {
		pr_err("%s %s host=NULL,should be MODULE_SD,current_module=%d,"
			"This is Error, just return STATUS_PLUG_IN.\n",
			MUX_SDSIM_LOG_TAG, __func__, MODULE_SD);
		/* This is sim */
		return STATUS_PLUG_IN;
	}

	if (host->id != 1) {
		pr_info("%s %s for those non DWMMC_SD_ID device,"
			"just return former status value\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return STATUS_PLUG_IN;
	}

	if (!host->mux_sdsim) {
		pr_info("%s %s for device mux_sdsim not enabled,"
			"just return former status value\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return STATUS_PLUG_IN;
	}

        mmc_claim_host(host->mmc);
	down(&g_sem_mux_sdsim_detect);

	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SD) {
		pr_info("%s %s current_module=%d now re-entered "
			"but same detect status,"
			"just return STATUS_PLUG_IN and do nothing\n",
			 MUX_SDSIM_LOG_TAG, __func__, MODULE_SD);
		result = STATUS_PLUG_IN;
		up(&g_sem_mux_sdsim_detect);
                mmc_release_host(host->mmc);
		return result;
	}
	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SIM) {
		pr_info("%s %s MODULE_SD find SIM card already "
			"detected,just return STATUS_PLUG_OUT and do nothing\n",
			MUX_SDSIM_LOG_TAG, __func__);
		result = STATUS_PLUG_OUT;
		up(&g_sem_mux_sdsim_detect);
                mmc_release_host(host->mmc);
		return result;
	}
	result = sdsim_send_cmd_detect(host, MODULE_SD);
	up(&g_sem_mux_sdsim_detect);
        mmc_release_host(host->mmc);
	return result;
}

/*
 * For plug out event,just update
 * g_sd_sim_detect_status_current here
 */
static int sd_sim_plug_out(void)
{
	down(&g_sem_mux_sdsim_detect);
	struct msdc_host *host = msdc_sdsim_get_sd_host();

	mmc_power_off(host->mmc);
	if (SD_SIM_DETECT_STATUS_UNDETECTED !=
		g_sd_sim_detect_status_current) {
		sdsim_config_sdsim_gpio_mode(SDSIM_MODE_GPIO_DETECT);
		g_sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_UNDETECTED;
	}
	pr_info("%s %s For plug out event,update "
		"g_sd_sim_detect_status_current here\n",
		MUX_SDSIM_LOG_TAG, __func__);
	sdsim_switch_gpio_change(SWITCH_GPIO_SD_SIDE);

	up(&g_sem_mux_sdsim_detect);

	return STATUS_PLUG_OUT;
}

int sd_sim_detect_run(void *host, int status,
		      int current_module, int need_sleep)
{
	pr_info("%s %s argument list: status = %d current_module = %d "
	       "g_sd_sim_detect_status_current = %d need_sleep = %d\n",
		MUX_SDSIM_LOG_TAG, __func__, status, current_module,
		g_sd_sim_detect_status_current, need_sleep);

	if ((host == NULL) && (current_module == MODULE_SIM))
		host = msdc_sdsim_get_sd_host();

	if (!((struct msdc_host *)host)->mux_sdsim) {
		pr_info("%s %s mux_sdsim not enabled, just return\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return STATUS_PLUG_IN;
	}

	if (current_module == MODULE_SIM) {
		might_sleep();
		down_write(&((struct msdc_host *)host)->suspend_rwsem);
		if (((struct msdc_host *)host)->suspend_flag == MSDC_NMC_SUSPEND) {
			status = STATUS_CARD_SUSPENDED;
			pr_info("%s %s suspend_flag=1.\n",
				MUX_SDSIM_LOG_TAG, __func__);
			goto out;
		}
	}

	int cd_state = get_cd_state(host);

	if (cd_state == STATUS_PLUG_OUT) {
		status = sd_sim_plug_out();
		goto out;
	}

	if (status == STATUS_PLUG_OUT) {
		status = sd_sim_plug_out();
		goto out;
	}

	if (current_module == MODULE_SIM) {
		status = sim_plug_in();
		goto out;
	}

	if (current_module == MODULE_SD) {
		status = sd_plug_in(host);
		goto out;
	}
out:
	if (current_module == MODULE_SIM) {
		might_sleep();
		up_write(&((struct msdc_host *)host)->suspend_rwsem);
	}
	return status;
}
EXPORT_SYMBOL(sd_sim_detect_run);

static char *detect_status_to_string(void)
{
	switch (g_sd_sim_detect_status_current) {
	case SD_SIM_DETECT_STATUS_UNDETECTED:
		return "DETECT_STATUS_UNDETECTED";

	case SD_SIM_DETECT_STATUS_SD:
		return "DETECT_STATUS_SD";

	case SD_SIM_DETECT_STATUS_SIM:
		return "DETECT_STATUS_SIM";

	case SD_SIM_DETECT_STATUS_ERROR:
	default:
		return "DETECT_STATUS_ERROR";
	}
}

static void notify_sim1_card_in(void)
{
	int ret = 0;
	int32_t sim_card_state = SIMHOTPLUG_SIM_CARD_IN;
	/* for notify sim this will develop later*/
	/*ret = exec_ccci_kern_func_by_md_id(MD_SYS1, MD_DISPLAY_DYNAMIC_MIPI,
	 *	(char *)&sim_card_state, sizeof(sim_card_state));
	 */
	if (ret)
		pr_err("%s %s exec_ccci_kern_func_by_md_id msg fail\n",
			MUX_SDSIM_LOG_TAG, __func__);

	pr_info("%s %s Send msg pass\n", MUX_SDSIM_LOG_TAG, __func__);
}

void notify_sim_while_sd_success(struct mmc_host *mmc)
{
	struct msdc_host *host = NULL;

	down(&g_sem_mux_sdsim_detect);

	host = mmc_priv(mmc);

	pr_info("%s %s enter g_sd_sim_detect_status_current = %d\n",
	       MUX_SDSIM_LOG_TAG, __func__, g_sd_sim_detect_status_current);

	if (!host->mux_sdsim) {
		pr_info("%s %s for device mux_sdsim not enabled,just return\n",
			MUX_SDSIM_LOG_TAG, __func__);
		up(&g_sem_mux_sdsim_detect);
		return;
	}

	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SD)
		notify_sim1_card_in();
	else
		pr_info("%s %s SD card init success now, "
			"g_sd_sim_detect_status_current = %d but not "
			"SD_SIM_DETECT_STATUS_SD,no need retry with SIM, "
			"just return\n", MUX_SDSIM_LOG_TAG, __func__,
			g_sd_sim_detect_status_current);

	up(&g_sem_mux_sdsim_detect);
}

void notify_sim_while_sd_fail(struct mmc_host *mmc)
{
	struct msdc_host *host = NULL;

	down(&g_sem_mux_sdsim_detect);

	host = mmc_priv(mmc);

	pr_err("%s %s enter\n", MUX_SDSIM_LOG_TAG, __func__);

	if (!host->mux_sdsim) {
		pr_info("%s %s for device mux_sdsim not enabled,just return\n",
			MUX_SDSIM_LOG_TAG, __func__);
		up(&g_sem_mux_sdsim_detect);
		return;
	}

	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SD) {
		pr_info("%s %s SD card init fail now,retry go with SIM\n",
			MUX_SDSIM_LOG_TAG, __func__);
		sdsim_switch_gpio_change(SWITCH_GPIO_SIM_SIDE);
		sdsim_config_sdsim_gpio_mode(SDSIM_MODE_SIM_NORMAL);
		g_sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SIM;
		notify_sim1_card_in();
	} else {
		pr_info("%s %s SD card init fail now, "
			"g_sd_sim_detect_status_current = %d, but not "
			"SD_SIM_DETECT_STATUS_SD,no need retry with SIM,"
			"just return\n", MUX_SDSIM_LOG_TAG, __func__,
			g_sd_sim_detect_status_current);
	}

	up(&g_sem_mux_sdsim_detect);
}

int get_card1_type(void)
{
	unsigned char status = STATUS_SIM;
	struct msdc_host *host = msdc_sdsim_get_sd_host();

	if (!host->mux_sdsim) {
		pr_info("%s %s mux_sdsim not enabled, just return\n",
			MUX_SDSIM_LOG_TAG, __func__);
		return STATUS_SIM;
	}
#ifdef CONFIG_MMC_DW_MUX_SDSIM
	if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SIM) {
		status = STATUS_SIM;
	} else if (g_sd_sim_detect_status_current == SD_SIM_DETECT_STATUS_SD) {
		status = STATUS_SD;
	} else {
		pr_info("%s %s sd_sim_detect: %s, as STATUS_NO_CARD\n",
			MUX_SDSIM_LOG_TAG, __func__, detect_status_to_string());
		status = STATUS_NO_CARD;
	}
#endif
	return status;
}
EXPORT_SYMBOL(get_card1_type);

