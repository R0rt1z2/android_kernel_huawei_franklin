/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef SCP_ERR_INFO_H
#define SCP_ERR_INFO_H


#include <linux/types.h>        /* for uint32_t */

/******************************************************************************
* The following definitions are used in error_info::case_id.
******************************************************************************/
typedef enum err_case_id_t {
	ERR_CASE_ACC_GYR_INIT     = 926006001,
	ERR_CASE_ACC_INIT         = 926006002,
	ERR_CASE_GYR_INIT         = 926006003,
	ERR_CASE_MAG_INIT         = 926006004,
	ERR_CASE_ALS_PS_INIT      = 926006005,
	ERR_CASE_ALS_INIT         = 926006006,
	ERR_CASE_PS_INIT          = 926006007,
	ERR_CASE_BARO_INIT        = 926006008,
	ERR_CASE_I2C              = 926006009,
	ERR_CASE_SPI              = 926006010,
	ERR_CASE_DEV_CHECK        = 926006011,
	ERR_CASE_UNKNOWN          = 0xffffffff
} err_case_id_t;

/******************************************************************************
* The following definitions are used in error_info::sensor_id.
******************************************************************************/
typedef enum err_sensor_id_t {
	ERR_SENSOR_ACC_GYR        = 0x00000001,
	ERR_SENSOR_ACC            = 0x00000002,
	ERR_SENSOR_GYR            = 0x00000003,
	ERR_SENSOR_MAG            = 0x00000004,
	ERR_SENSOR_ALS_PS         = 0x00000005,
	ERR_SENSOR_ALS            = 0x00000006,
	ERR_SENSOR_PS             = 0x00000007,
	ERR_SENSOR_BARO           = 0x00000008,
	ERR_SENSOR_I2C            = 0x00000009,
	ERR_SENSOR_SPI            = 0x0000000a,
	ERR_SENSOR_UNKNOWN        = 0xffffffff
} err_sensor_id_t;

/******************************************************************************
 ******************************************************************************/
#define ERR_MAX_CONTEXT_LEN     32


/******************************************************************************
 * SCP side uses the data types err_case_id_t and err_sensor_id_t which can be
 * regarded as uint32_t.
 ******************************************************************************/
struct error_info {
	uint32_t case_id;
	uint32_t sensor_id;
	char context[ERR_MAX_CONTEXT_LEN];
};


__attribute__((weak)) void report_hub_dmd(uint32_t case_id, uint32_t sensor_id,
						char *context);


#endif  // SCP_ERR_INFO_H

