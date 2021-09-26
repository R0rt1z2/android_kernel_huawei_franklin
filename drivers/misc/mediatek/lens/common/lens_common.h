/*
 * lens_common.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * lens common header file
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

#ifndef LENS_COMMON_H
#define LENS_COMMON_H

#include <linux/i2c.h>
#include <securec.h>
#include "../../hwcam_hiview/hwcam_hiview.h"

#ifndef array_size
#define array_size(x) (sizeof(x) / sizeof((x)[0]))
#endif
#ifndef log_info
#define log_info(fmt, args...) pr_info("%s %d " fmt, __func__, __LINE__, ##args)
#endif
#ifndef log_err
#define log_err(fmt, args...) pr_err("%s %d " fmt, __func__, __LINE__, ##args)
#endif
#define I2C_LEN 2
#define MAX_PRODUCT_LEN 32

extern const char *g_product_name;
struct i2c_cfg_t {
	char setting[I2C_LEN];
	unsigned int delay;
};

struct product_lens_cfg_map_t {
	char product_name[MAX_PRODUCT_LEN];
	const struct i2c_cfg_t *i2c_cfg_map;
	unsigned int cfg_size;
};

typedef enum {
	VCM_ERROR_TYPE_START = 0,
	VCM_WRITE_I2C_FAIL = VCM_ERROR_TYPE_START,
	VCM_READ_I2C_FAIL,
	VCM_ERROR_TYPE_END,
} vcm_error_type;

int lens_set_setting(struct i2c_client *client,
	const struct i2c_cfg_t *i2c_cfg, unsigned int size);
void lens_hiview_handle(int error_no, const char *ic_name,
	const char *module_name, vcm_error_type error_type);

#endif /* LENS_COMMON_H */
