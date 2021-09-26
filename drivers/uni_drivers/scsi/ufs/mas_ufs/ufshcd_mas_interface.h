/*
 * ufshcd_mas_interface.h
 *
 * The mas interface for ufshcd.c
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __UFS_MAS_INTERFACE_H__
#define __UFS_MAS_INTERFACE_H__

#include "ufshcd.h"

/* define scmd group number */
#define TZ_GROUP_NUM 0x1F

static inline int ufshcd_vops_get_device_info(struct ufs_hba *hba,
					      struct ufs_dev_desc *dev_desc,
					      uint8_t *buf)
{
	if (hba->vops && hba->vops->get_device_info)
		return hba->vops->get_device_info(hba, dev_desc, buf);

	return 0;
}

u64 read_utr_doorbell(struct ufs_hba *hba);
#endif
