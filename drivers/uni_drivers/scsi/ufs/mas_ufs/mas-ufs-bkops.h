/*
 * mas-ufs-bkops.h
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
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

#ifndef MAS_UFS_BKOPS_H
#define MAS_UFS_BKOPS_H
#include "ufshcd.h"

#define MAS_UFS_MODEL_ANY "MAS_UFS_MODEL_ANY"
#define MAS_UFS_REV_ANY "MAS_UFS_REV_ANY"

struct mas_ufs_bkops_id {
	uint16_t manufacturer_id;
	char *ufs_model;
	char *ufs_rev;
	struct list_head p;
};

typedef int(ufs_bkops_query_fn)(struct ufs_hba *hba, u32 *status);
typedef int(ufs_bkops_start_fn)(struct ufs_hba *hba);
typedef int(ufs_bkops_stop_fn)(struct ufs_hba *hba);
struct ufs_dev_bkops_ops {
	ufs_bkops_query_fn *ufs_bkops_query;
	ufs_bkops_start_fn *ufs_bkops_start;
	ufs_bkops_stop_fn *ufs_bkops_stop;
};

struct device_model_para {
	char *compatible;
	char *model;
	char *rev;
};

int mas_ufs_manual_bkops_config(struct scsi_device *sdev);
void mas_ufs_populate_mgc_dt(struct ufs_hba *hba);
#endif
