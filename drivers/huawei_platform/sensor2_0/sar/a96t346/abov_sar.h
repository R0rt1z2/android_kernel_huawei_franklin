/*
 * abov_sar.h
 *
 * code for sensor abov sar
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#ifndef ABOV_SAR_H
#define ABOV_SAR_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "hf_manager.h"

/*  I2C Registers Addr */
#define ABOV_IRQSTAT_REG                0x04
#define ABOV_VERSION_REG                0x01
#define ABOV_MODELNO_REG                0x02
#define ABOV_VENDOR_ID_REG              0x03
#define ABOV_SOFTRESET_REG              0x06
#define ABOV_CTRL_MODE_REG              0x07
#define ABOV_CTRL_CHANNEL_REG           0x08
#define ABOV_CH0_DIFF_MSB_REG           0x1C
#define ABOV_CH0_DIFF_LSB_REG           0x1D
#define ABOV_CH1_DIFF_MSB_REG           0x1E
#define ABOV_CH1_DIFF_LSB_REG           0x1F
#define ABOV_CH0_CAP_MSB_REG            0x20
#define ABOV_CH0_CAP_LSB_REG            0x21
#define ABOV_CH1_CAP_MSB_REG            0x22
#define ABOV_CH1_CAP_LSB_REG            0x23
#define ABOV_REF_CAP_MSB_REG            0x24
#define ABOV_REF_CAP_LSB_REG            0x25
#define ABOV_CTRL_TH_LEVEL_REG          0x2B
#define ABOV_RECALI_REG                 0xFB
#define ABOV_RESET_CHIP                 0x10

/* enable body stat */
#define ABOV_TCHCMPSTAT_TCHSTAT0_FLAG   0x01
#define ABOV_TCHCMPSTAT_TCHSTAT1_FLAG   0x02
#define ABOV_CH0_CH1_MASK               0x07
#define ABOV_CTRL_MODE_REG_ACTIVATE     0x00
#define ABOV_CTRL_MODE_REG_SLEEP        0x01
#define ABOV_CTRL_MODE_REG_STOP         0x02


#define C_I2C_FIFO_SIZE                 8
#define ABOV_CHAR_MASK                  0xff
#define I2C_MASK_FLAG                   0x00ff
#define ABOV_FOUR_BYTE_MASK             0xffff

#define ABOV_ONE_BYTE                   1
#define ABOV_TWO_BYTE                   2
#define ABOV_HEX                        10

#define ABOV_CHIP_DETECTED              1
#define ABOV_CHIP_IS_PRELOAD            2
#define ABOV_REG_VAL_BUF_SIZE           2
#define ABOV_DIF_OFF_BUF_SIZE           2
#define ABOV_CHANNL_NUM                 2
#define ABOV_DATA_SIZE                  3
#define ABOV_CHECK_SUM_LEN              6
#define ABOV_SHIFT_8_BIT                8
#define ABOV_RETRY_TIMES                5
#define ABOV_FW_UPDATE_SIZE             32
#define ABOV_FW_NAME_SIZE               32
#define ABOV_FW_BUF_SIZE                36
#define ABOV_UPDATE_COUNT_VALID         2
#define ABOV_TRY_TIMES                  3
#define ABOV_SLEEP_5_MS                 5
#define ABOV_MODENO_INDEX               1
#define ABOV_VERSION_INDEX              5
#define ABOV_CK_H_INDEX                 8
#define ABOV_CK_L_INDEX                 9
#define MAX_NUM_STATUS_BITS             2
#define ABOV_FW_UPDATE_FAILED           0
#define ABOV_FW_UPDATE_SUCCESS          1
#define MAX_CALI_OFFSET                 65535
#define MIN_CALI_OFFSET                 0

enum {
	ABOV_IRQ_ENABLE,
	ABOV_IRQ_DISABLE,
};

enum {
	FAR_TEST,
	NEAR_TEST,
	UNKNOW_TEST,
};

enum {
	PRODUCT_DEFAULT,
	PRODUCT_OK,
	PRODUCT_DM,
	PRODUCT_TH,
	PRODUCT_WN,
};

struct abov_reg_data {
	unsigned char reg;
	unsigned char val;
};

struct abov_platform_data {
	int i2c_reg_num;
	struct abov_reg_data *pi2c_reg;
	unsigned int irq_gpio;
	const char *fw_name;

	int (*get_is_nirq_low)(unsigned int irq_gpio);
	int (*init_platform_hw)(void);
	void (*exit_platform_hw)(void);
};

/* specific platform data settings */
struct abov_plat {
	struct abov_platform_data *hw;
};

typedef struct {
	int type;

	int near_diff;
	int near_offset;
	int far_diff;
	int far_offset;
} sar_cali_priv;

struct abov_dev_t {
	struct device *pdev;
	struct delayed_work dworker;
	struct abov_platform_data *board;
	struct mutex mutex;
	struct hf_device hf_dev;

	void *bus;
	void *pdevice;
	int read_flag;
	int irq;
	int irq_timeout;
	char irq_disabled;
	u8 use_irq_timer;
	u8 read_reg;

	struct work_struct ps_notify_work;
	struct notifier_block ps_notif;
	bool ps_is_present;
	bool loading_fw;
	struct work_struct fw_update_work;

	int (*init)(struct abov_dev_t *pdev);
	int (*refresh_status)(struct abov_dev_t *pdev);
	int (*get_nirq_low)(unsigned int irq_gpio);
	void (*status_func)(struct abov_dev_t *pdev);
};

int sar_data_report(int32_t value[3]);
int situation_flush_report(int handle);
int hw_get_sensor_dts_config(void);

static int abov_tk_fw_mode_enter(struct i2c_client *client);
static int abov_sar_interrupt_init(struct abov_dev_t *pdev);

#endif  // ABOV_SAR_H
