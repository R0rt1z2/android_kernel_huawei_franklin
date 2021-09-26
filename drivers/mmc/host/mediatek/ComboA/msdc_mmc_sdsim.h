/*
 * msdc_mmc_sdsim.h
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

#ifndef _MSDC_MMC_SDSIM_
#define _MSDC_MMC_SDSIM_

#include <linux/semaphore.h>
#include <mtk_sd.h>

#define MUX_SDSIM_LOG_TAG "[MUX_SDSIM][mmc1]"
enum sdsim_gpio_mode {
	SDSIM_MODE_GPIO_DETECT = 0,  /*gpio detect mode for detect sd or sim*/
	SDSIM_MODE_SD_NORMAL = 1,    /*sd normal mode*/
	SDSIM_MODE_SD_IDLE = 2,      /*sd idle/lowpower mode*/
	SDSIM_MODE_SIM_NORMAL = 3,   /*sim normal mode*/
	SDSIM_MODE_SIM_IDLE = 4,     /*sim idle/lowpower mode*/
};

enum sdsim_detect_status {
	SD_SIM_DETECT_STATUS_UNDETECTED = 0,
	SD_SIM_DETECT_STATUS_SD,
	SD_SIM_DETECT_STATUS_SIM,
	SD_SIM_DETECT_STATUS_ERROR,
};

enum sdsim_result {
	SDSIM_OK = 0,
	SDSIM_PARAM_INVALID,
	SDSIM_MALLOC_FAIL,
	SDSIM_GET_PINCTL_FAIL,
	SDSIM_GET_GPIOCTL_ERROR,
	SDSIM_GPIO_REQUEST_FAIL,
	SDSIM_SET_DIRECT_OUT_FAIL,
	SDSIM_SET_DIRECT_IN_FAIL,
	SDSIM_GET_GPIO_CTRL_INFO_ERROR,
	SDSIM_PINCTL_SELECT_STATE_FAIL,
	SDSIM_UNKNOW,
	SDSIM_MAX = 0xff,
};

#define MODULE_SD  0
#define MODULE_SIM  1

/*
 * status=1 means plug out;
 * status=0 means plug in;
 */
#define STATUS_PLUG_IN 0
#define STATUS_PLUG_OUT 1
#define STATUS_CARD_SUSPENDED 2

#define MSDC_SWITCH_GPIO_NUMBER 4
#define MSDC_CHAR_MAX_LENGTH    32

#define SWITCH_GPIO_SD_SIDE 1
#define SWITCH_GPIO_SIM_SIDE 0

#define SLEEP_MS_TIME_FOR_DETECT_UNSTABLE   20
#define SWITCH_GPIO_DEFAULT_NUMBER  0xFFFF
#define SDSIM_GPIO_DEFAULT_NUMBER  0xFFFF

void sdsim_set_switch_default_gpio(int switch_gpio_num);
enum sdsim_result sdsim_gpio_probe(struct device *dev);

void sdsim_set_current_sd_clk_gpios(int sd_clk_gpio);
void sdsim_set_current_sd_cmd_gpios(int sd_cmd_gpio);
void sdsim_set_current_sd_data0_gpios(int sd_data0_gpio);
void sdsim_set_current_sd_data1_gpios(int sd_data1_gpio);
void sdsim_set_current_sd_data2_gpios(int sd_data2_gpio);
void sdsim_set_current_sd_data3_gpios(int sd_data3_gpio);
void sdsim_set_current_sim_gpios(int sim_clk_gpio, int sim_rst_gpio,
				 int sim_data_gpio);
void sdsim_set_switch_sd_side_value(int switch_sd_side_num);
enum sdsim_result msdc_sdsim_set_sd_host(struct msdc_host *host);
enum sdsim_result msdc_sdsim_gpio_probe(struct device *dev,
					struct msdc_host *host);
int sdsim_get_current_detect_status(void);
enum sdsim_result msdc_sem_init(struct msdc_host *host);
int sdsim_config_sdsim_gpio_mode(enum sdsim_gpio_mode gpio_mode);
enum sdsim_result sdsim_switch_gpio_change(int set_side_status);

/*
 * Description: while sd/sim plug in or out, gpio_cd detect pin is actived,
 *   we need call this sd_sim_detect_run function to make sure
 *   sd or sim which is inserted
 * @host: MODULE_SD use host argu as input, while MODULE_SIM just use NULL
 * @status: use STATUS_PLUG_IN or STATUS_PLUG_OUT by gpio_cd detect pin's value
 * @current_module: sd or sim which module is calling this function
 * return value: return STATUS_PLUG_IN or STATUS_PLUG_OUT,
 * just tell current_module sd or sim is inserted or not,
 * and current_module can update gpio_cd detect pin value by this return value
 */
extern int sd_sim_detect_run(void *host, int status,
			     int current_module, int need_sleep);
extern int get_card1_type(void);
void notify_sim_while_sd_fail(struct mmc_host *mmc);
void notify_sim_while_sd_success(struct mmc_host *mmc);

#endif

