/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: cap prox channel source file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2020-05-25
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif
#include "xhub_route.h"
#include "xhub_boot.h"
#include "protocol.h"
#include "sensor_config.h"
#include "sensor_detect.h"
#include "cap_prox_channel.h"
#include <securec.h>

union sar_calibrate_data sar_calibrate_datas;
union sar_calibrate_data g_sar_calibrate_datas;

int send_cap_prox1_calibrate_data_to_mcu(void)
{
	if (strlen(sensor_chip_info[CAP_PROX1]) == 0) {
		hwlog_info("is not overseas phone,cap_prox sensor is not in board\n");
		return 0;
	}

	if (read_calibrate_data_from_nv(CAP_PROX1_CALIDATA_NV_NUM,
		CAP_PROX1_CALIDATA_NV_SIZE, CAP_PROX1_NV_NAME))
		return -1;

	/* send to mcu */
	if (memcpy_s(&g_sar_calibrate_datas, sizeof(g_sar_calibrate_datas),
		user_info.nv_data, sizeof(g_sar_calibrate_datas)) != EOK)
		return -1;
	if (send_calibrate_data_to_mcu(TAG_CAP_PROX1,
		SUB_CMD_SET_OFFSET_REQ, (const void *)&g_sar_calibrate_datas,
		sizeof(g_sar_calibrate_datas), false))
		return -1;
	return 0;
}

int send_cap_prox_calibrate_data_to_mcu(void)
{
	if (strlen(sensor_chip_info[CAP_PROX]) == 0) {
		hwlog_info("is not overseas phone,cap_prox sensor is not in board\n");
		return 0;
	}

	if (read_calibrate_data_from_nv(CAP_PROX_CALIDATA_NV_NUM,
		CAP_PROX_CALIDATA_NV_SIZE, "Csensor"))
		return -1;

	if (memcpy_s(&sar_calibrate_datas, sizeof(sar_calibrate_datas),
		user_info.nv_data, sizeof(sar_calibrate_datas)) != EOK)
		return -1;
	if (!strncmp(sensor_chip_info[CAP_PROX], "huawei,semtech-sx9323",
			strlen("huawei,semtech-sx9323")))
		hwlog_info("sx9323:offset1=%d offset2=%d diff1=%d diff2=%d len:%ld\n",
			sar_calibrate_datas.semtech_cali_data.offset[0],
			sar_calibrate_datas.semtech_cali_data.offset[1],
			sar_calibrate_datas.semtech_cali_data.diff[0],
			sar_calibrate_datas.semtech_cali_data.diff[1],
			sizeof(sar_calibrate_datas));
	else if (!strncmp(sensor_chip_info[CAP_PROX], "huawei,abov-a96t3x6",
			strlen("huawei,abov-a96t3x6")))
		hwlog_info("a96t3x6:offset1=%d offset2=%d diff1=%d diff2=%d len:%ld\n",
			sar_calibrate_datas.abov_cali_data.offset[0],
			sar_calibrate_datas.abov_cali_data.offset[1],
			sar_calibrate_datas.abov_cali_data.diff[0],
			sar_calibrate_datas.abov_cali_data.diff[1],
			sizeof(sar_calibrate_datas));

	if (send_calibrate_data_to_mcu(TAG_CAP_PROX,
		SUB_CMD_SET_OFFSET_REQ,
		(const void *)&sar_calibrate_datas,
		sizeof(sar_calibrate_datas), false))
		return -1;
	return 0;
}

void reset_cap_prox_calibrate_data(void)
{
	if (strlen(sensor_chip_info[CAP_PROX]))
		send_calibrate_data_to_mcu(TAG_CAP_PROX,
			SUB_CMD_SET_OFFSET_REQ, &sar_calibrate_datas,
			sizeof(sar_calibrate_datas), true);
}

void reset_cap_prox1_calibrate_data(void)
{
	if (strlen(sensor_chip_info[CAP_PROX1]))
		send_calibrate_data_to_mcu(TAG_CAP_PROX1,
			SUB_CMD_SET_OFFSET_REQ, &sar_calibrate_datas,
			sizeof(sar_calibrate_datas), true);
}

