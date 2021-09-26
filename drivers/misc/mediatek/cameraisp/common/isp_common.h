/*
 * isp_common.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * isp common header file
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

#ifndef ISP_COMMON_H
#define ISP_COMMON_H

#include <securec.h>
#include "../../hwcam_hiview/hwcam_hiview.h"

#define CAM_ISP_DSM_SKIP_FRAME_NUM 3
#define MAX_REPORT_NUM 1 /* allow report 1 times, when open camera every time */
#define MIPI_ERR_CNT 2
#ifndef array_size
#define array_size(x) (sizeof(x) / sizeof((x)[0]))
#endif

enum csi_index_enum {
	CSIA,
	CSIB,
	CSIC,
	MAX_CSI_INDEX,
};

struct mipi_err_info_t {
	unsigned int err_status;
	unsigned int err_mask;
	unsigned int frame_cnt;
	unsigned int is_stream_on;
};

struct mipi_err_cnt_t {
	unsigned int err_cnt[MAX_CSI_INDEX];
	unsigned int report_num[MAX_CSI_INDEX];
};

void isp_mipi_err_cnt_clear(void);
bool is_reach_max_report_num(unsigned int csi_index);
void isp_hiview_report(int error_no, unsigned int csi_index,
	struct mipi_err_info_t *mipi_err_info);

#endif /* ISP_COMMON_H */
