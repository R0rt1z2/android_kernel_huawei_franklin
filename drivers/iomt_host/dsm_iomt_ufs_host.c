/*
 * file_name
 * dsm_iomt_ufs_host.c
 * description
 * stat io latency scatter at driver level,
 * show it in kernel log and host attr node.
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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

#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_host.h>
#include <linux/iomt_host/dsm_iomt_host.h>
#ifdef CONFIG_HUAWEI_UFS_DSM
#include "dsm_ufs.h"
#endif
#include "ufshcd.h"

static char *ufs_host_name(void *host);

unsigned char iomt_host_cmd_dir(struct scsi_cmnd *cmd)
{
	if (cmd->sc_data_direction == DMA_FROM_DEVICE)
		return IOMT_DIR_READ;
	else if (cmd->sc_data_direction == DMA_TO_DEVICE)
		return IOMT_DIR_WRITE;
	else
		return IOMT_DIR_OTHER;
}

void iomt_host_latency_cmd_init(struct scsi_cmnd *cmd,
	struct Scsi_Host *scsi_host)
{
	cmd->iomt_start_time.ticks = 0;
	cmd->iomt_start_time.ktime = 0;
	cmd->iomt_start_time.type = IOMT_LATNECY_LOW_ACC;
	if (iomt_info_from_host(scsi_host))
		cmd->iomt_start_time.type =
			iomt_info_from_host(scsi_host)->latency_scatter.acc_type;
}

void iomt_host_latency_cmd_start_once(struct scsi_cmnd *cmd)
{
	if (cmd->iomt_start_time.ticks == 0)
		cmd->iomt_start_time.ticks = jiffies;

	if ((cmd->iomt_start_time.type == IOMT_LATNECY_HIGH_ACC) &&
		(cmd->iomt_start_time.ktime == 0))
		cmd->iomt_start_time.ktime = ktime_get();
}

void iomt_host_latency_cmd_start(struct scsi_cmnd *cmd)
{
	cmd->iomt_start_time.ticks = jiffies;

	if (cmd->iomt_start_time.type == IOMT_LATNECY_HIGH_ACC)
		cmd->iomt_start_time.ktime = ktime_get();
}

void iomt_host_latency_cmd_end(struct Scsi_Host *scsi_host,
	struct scsi_cmnd *cmd)
{
	struct iomt_host_io_timeout_dsm *io_timeout_dsm = NULL;
	struct iomt_host_info *iomt_host_info = NULL;
	unsigned int i;
	unsigned char dir;

	if (scsi_host->iomt_host_info == NULL)
		return;

	iomt_host_info = iomt_info_from_host(scsi_host);

	dir = iomt_host_cmd_dir(cmd);

	i = iomt_host_stat_latency(iomt_host_info, &(cmd->iomt_start_time), dir);

	if (!(iomt_host_info->latency_scatter.latency_stat_enable))
		return;

	if (unlikely(i == IOMT_LATENCY_INVALID_INDEX))
		return;

	if (unlikely(i >= iomt_host_info->io_timeout_dsm.judge_slot)) {
		io_timeout_dsm = &(iomt_host_info->io_timeout_dsm);
		io_timeout_dsm->opcode = 0;
		io_timeout_dsm->op_arg = 0;
		io_timeout_dsm->data_dir = dir;

		io_timeout_dsm->block_ticks =
			(u16)(jiffies - cmd->iomt_start_time.ticks);

		if (cmd) {
			io_timeout_dsm->opcode = cmd->cmnd[0];
			/* 1 block = 8 sector = 4096 Byte */
			io_timeout_dsm->op_arg = cmd->request->__sector >> 3;
			io_timeout_dsm->block_ticks |=
				(((unsigned int)(cmd->request->__data_len >> 12)) <<
				IOMT_OP_BLOCK_SHIFT);
		}
	}
}

static void iomt_ufs_io_timeout_dsm_action(
	struct Scsi_Host *host)
{
#ifdef CONFIG_HUAWEI_UFS_DSM
	unsigned int opcode;
	unsigned int op_arg;
	unsigned int block_ticks;
	struct ufs_hba *hba = NULL;
	struct iomt_host_info *iomt_host_info = NULL;

	if (host != NULL && host->hostdata != NULL &&
		host->iomt_host_info != NULL &&
		iomt_info_from_host(host)->latency_scatter.latency_stat_enable) {
		hba = shost_priv(host);
		iomt_host_info = iomt_info_from_host(host);
		opcode = iomt_host_info->io_timeout_dsm.opcode;
		op_arg = iomt_host_info->io_timeout_dsm.op_arg;
		block_ticks = iomt_host_info->io_timeout_dsm.block_ticks;
		DSM_UFS_LOG(hba, DSM_UFS_IO_TIMEOUT,
			"%s:%s(%d) ufs io timeout, judge slot = %u, "
			"opcode = 0x%x, lba = 0x%x, blocks = %u, ticks = %u, dir = %u",
			__func__, ufs_host_name(host), hba->manufacturer_id,
			iomt_host_info->io_timeout_dsm.judge_slot, opcode, op_arg,
			block_ticks >> IOMT_OP_BLOCK_SHIFT,
			(unsigned int)((unsigned short)block_ticks),
			iomt_host_info->io_timeout_dsm.data_dir);
	}
#endif
	return;
}

static void *ufs_host_to_iomt(void *scsi_host)
{
	return (void *)(((struct Scsi_Host *)scsi_host)->iomt_host_info);
}

static ssize_t io_latency_ufs_show_checker(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct Scsi_Host *host = class_to_shost(dev);

	if (host->iomt_host_info != NULL)
		return io_latency_scatter_show(
			(struct iomt_host_info *)host->iomt_host_info,
			buf);
	return 0;
}

static ssize_t io_latency_ufs_store_checker(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct Scsi_Host *host = class_to_shost(dev);

	if (host->iomt_host_info != NULL)
		return io_latency_scatter_store(
			(struct iomt_host_info *)host->iomt_host_info,
			buf, count);
	return -EINVAL;
}

static DEVICE_ATTR(io_latency_scatter, 0644,
	io_latency_ufs_show_checker,
	io_latency_ufs_store_checker);

static void ufs_host_create_latency_attr(void *iomt_host_info)
{
	struct Scsi_Host * host = NULL;

	struct iomt_host_latency_stat *latency_scatter =
		&(((struct iomt_host_info *)iomt_host_info)->latency_scatter);

	host = (struct Scsi_Host *)latency_scatter->host;
	if (host == NULL) {
		pr_err("%s:Invalid host info\n", __func__);
		return;
	}

	if (device_create_file(&host->shost_dev, &dev_attr_io_latency_scatter))
		dev_err(&host->shost_dev, "Failed to create io_latency_scatter sysfs entry\n");
}
static void ufs_host_delete_latency_attr(void *iomt_host_info)
{
	struct Scsi_Host * host = NULL;

	struct iomt_host_latency_stat *latency_scatter =
		&(((struct iomt_host_info *)iomt_host_info)->latency_scatter);

	host = (struct Scsi_Host *)latency_scatter->host;

	if (host == NULL) {
		pr_err("%s:Invalid host info\n", __func__);
		return;
	}

	device_remove_file(&host->shost_dev, &dev_attr_io_latency_scatter);
}

static void ufs_host_io_timeout_dsm(void *iomt_host_info)
{
	struct Scsi_Host * host = NULL;

	struct iomt_host_latency_stat *latency_scatter =
		&(((struct iomt_host_info *)iomt_host_info)->latency_scatter);

	host = (struct Scsi_Host *)latency_scatter->host;

	if (host == NULL) {
		pr_err("%s:Invalid host info\n", __func__);
		return;
	}

	iomt_ufs_io_timeout_dsm_action(host);
}

static char *ufs_host_name(void *host)
{
	return (char *)dev_name(&((struct Scsi_Host *)host)->shost_dev);
}

static unsigned int ufs_host_blk_size(void *host)
{
	return  512;
}

static ssize_t rw_size_ufs_show_checker(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct Scsi_Host *host = class_to_shost(dev);

	if (host->iomt_host_info != NULL)
		return rw_size_scatter_show(
			(struct iomt_host_info *)host->iomt_host_info,
			buf);
	return 0;
}

static DEVICE_ATTR(rw_size_scatter, 0444,
	rw_size_ufs_show_checker,
	NULL);

static void ufs_host_create_rw_size_attr(void *iomt_host_info)
{
	struct Scsi_Host * host = NULL;

	struct iomt_host_rw_size_stat *rw_size_scatter =
		&(((struct iomt_host_info *)iomt_host_info)->rw_size_scatter);

	host = (struct Scsi_Host *)rw_size_scatter->host;

	if (host == NULL) {
		pr_err("%s:Invalid host info\n", __func__);
		return;
	}

	if (device_create_file(&host->shost_dev, &dev_attr_rw_size_scatter))
		dev_err(&host->shost_dev, "Failed to create rw_size_scatter sysfs entry\n");
}

static void ufs_host_delete_rw_size_attr(void *iomt_host_info)
{
	struct Scsi_Host * host = NULL;

	struct iomt_host_rw_size_stat *rw_size_scatter =
		&(((struct iomt_host_info *)iomt_host_info)->rw_size_scatter);

	host = (struct Scsi_Host *)rw_size_scatter->host;

	if (host == NULL) {
		pr_err("%s:Invalid host info\n", __func__);
		return;
	}

	device_remove_file(&host->shost_dev, &dev_attr_rw_size_scatter);
}

static const struct iomt_host_ops iomt_ufs_host_ops = {
	.host_to_iomt_func = ufs_host_to_iomt,
	.host_name_func = ufs_host_name,
	.create_latency_attr_func = ufs_host_create_latency_attr,
	.delete_latency_attr_func = ufs_host_delete_latency_attr,
	.dsm_func = ufs_host_io_timeout_dsm,
	.host_blk_size_fun = ufs_host_blk_size,
	.create_rw_size_attr_func = ufs_host_create_rw_size_attr,
	.delete_rw_size_attr_func = ufs_host_delete_rw_size_attr,
};

void dsm_iomt_ufs_host_init(struct Scsi_Host *host)
{
	dsm_iomt_host_init((void *)host, &iomt_ufs_host_ops);
}

void dsm_iomt_ufs_host_exit(struct Scsi_Host *host)
{
	dsm_iomt_host_exit((void *)host, &iomt_ufs_host_ops);
}
