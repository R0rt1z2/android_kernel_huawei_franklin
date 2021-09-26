/*
 * isp_common.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * isp common interface
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

#include "dsm_camera.h"
#include "isp_common.h"

static struct mipi_err_cnt_t g_mipi_err_cnt;

void isp_mipi_err_cnt_clear(void)
{
	if (memset_s(&g_mipi_err_cnt, sizeof(g_mipi_err_cnt),
		0, sizeof(g_mipi_err_cnt)) != EOK) {
		pr_err("%s, mipi_err_cnt memset_s fail", __func__);
		return;
	}
}

bool is_reach_max_report_num(unsigned int csi_index)
{
	if (csi_index >= MAX_CSI_INDEX)
		return false;

	if (g_mipi_err_cnt.report_num[csi_index] >= MAX_REPORT_NUM) {
		pr_debug("%s, csi%u has reatch max report num", __func__, csi_index);
		return true;
	}

	return false;
}

void isp_hiview_report(int error_no, unsigned int csi_index,
	struct mipi_err_info_t *mipi_err_info)
{
	if (!mipi_err_info)
		return;

	if (csi_index >= MAX_CSI_INDEX)
		return;

	if (mipi_err_info->is_stream_on ||
		mipi_err_info->frame_cnt < CAM_ISP_DSM_SKIP_FRAME_NUM)
		return;

	g_mipi_err_cnt.err_cnt[csi_index]++;
	if (g_mipi_err_cnt.err_cnt[csi_index] < MIPI_ERR_CNT)
		return;

	(void)camera_dsm_report_info(error_no,
		"err_status[0x%x], err_mask[0x%x], frame_cnt[%d]:%d, check RFI",
		mipi_err_info->err_status, mipi_err_info->err_mask,
		csi_index, mipi_err_info->frame_cnt);

	g_mipi_err_cnt.report_num[csi_index]++;
	g_mipi_err_cnt.err_cnt[csi_index] = 0;
}
