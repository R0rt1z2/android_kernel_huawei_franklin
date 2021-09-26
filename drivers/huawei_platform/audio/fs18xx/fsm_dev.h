/**
 * Copyright (c) 2018 Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *Description: Core Defination For Foursemi Device .
 *Author: Fourier Semiconductor Inc.
 * Create: 2020-02-20 File created.
 */

#ifndef __FSM_DEV_H__
#define __FSM_DEV_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>

#define FS1603_ID_REG           0x03
/* device id */
#define FS1603_DEV_ID           0x05
#define FSM_VENDOR_ID         (7)
#define FSM_DEV_BASE            0x34
#define FSDEV_I2C_NAME        "fs18xx"

#define FSM_DEV_MAX             (4)

#define STEREO_COEF_LEN    (10)

#define FSM_I2CADDR1        0x34
#define FSM_I2CADDR2        0x35
#define FSM_I2CADDR3        0X36
#define FSM_I2CADDR4        0x37

struct fsm_dev {
    struct i2c_client	*client;
    struct miscdevice	fs18xx_misc;
    struct device *dev;
    uint8_t i2c_addr;
};
typedef struct fsm_dev fsm_dev_t;
#endif
