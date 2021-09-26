/*
 * rpmb_driver.c
 *
 * function for rpmb
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "rpmb_driver.h"
#include <securec.h>

#include <linux/mmc/card.h> /* for struct mmc_card */
#include <linux/rpmb.h>
#include <linux/mmc/sd.h>

#ifdef CONFIG_MTK_UFS_SUPPORT
#include "ufs-mtk.h"
#endif
#include <mt-plat/mtk_boot.h>
#include "core.h"
#include "card.h"
#include "mmc_ops.h"
#include "mtk_sd.h"
#include "tc_ns_log.h"
#include "queue.h"

#define IOC_CMD_0                0
#define IOC_CMD_1                1
#define IOC_CMD_2                2
#define STORAGE_IOC_MAX_RPMB_CMD 3
#define RPMB_EMMC_CID_SIZE       32
#define RPMB_CTRL_MAGIC          0x5A5A5A5A
#define RPMB_REQ                 1        /* RPMB request mark */
#define RPMB_RESP                (1 << 1) /* RPMB response mark */
#define RPMB_PROGRAM_KEY         0x1      /* Program RPMB Authentication Key */
#define RPMB_GET_WRITE_COUNTER   0x2      /* Read RPMB write counter */
#define RPMB_WRITE_DATA          0x3      /* Write data to RPMB partition */
#define RPMB_READ_DATA           0x4      /* Read data from RPMB partition */
#define RPMB_RESULT_READ         0x5      /* Read result request  (Internal) */

struct emmc_rpmb_blk_data {
	spinlock_t lock;
	struct device *parent;
	struct gendisk *disk;
	struct mmc_queue queue;
	struct list_head part;
	uint32_t flags;
	uint32_t usage;
	uint32_t read_only;
	uint32_t part_type;
	uint32_t reset_done;
	uint32_t part_curr; // keep curr partition
	struct device_attribute force_ro;
	struct device_attribute power_ro_lock;
	int32_t area_type;
};

static int32_t emmc_rpmb_switch(struct mmc_card *card,
	struct emmc_rpmb_blk_data *md)
{
	int32_t ret;
	struct emmc_rpmb_blk_data *main_md = NULL;

	if (card == NULL)
		return -1;

	main_md = dev_get_drvdata(&card->dev);
	if (main_md == NULL)
		return -1;

	if (main_md->part_curr == md->part_type)
		return 0;

#if defined(CONFIG_MTK_EMMC_CQ_SUPPORT) || defined(CONFIG_MTK_EMMC_HW_CQ)
	if (mmc_card_cmdq(card)) {
		ret = mmc_cmdq_disable(card);
		if (ret) {
			tloge("CQ disabled failed!!! ret: 0x%x\n", ret);
			return ret;
		}
	}
#endif

	if (mmc_card_mmc(card)) {
		uint8_t cfg = card->ext_csd.part_config;

		cfg &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		cfg |= md->part_type;

		ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_PART_CONFIG,
			cfg, card->ext_csd.part_time);
		if (ret)
			return ret;

		card->ext_csd.part_config = cfg;
	}

#if defined(CONFIG_MTK_EMMC_CQ_SUPPORT) || defined(CONFIG_MTK_EMMC_HW_CQ)
	/* enable cmdq at user partition */
	if (!mmc_card_cmdq(card) && (md->part_type <= 0)) {
		ret = mmc_cmdq_enable(card);
		if (ret)
			tloge("%s enable CMDQ error %d, so just work without\n",
				mmc_hostname(card->host), ret);
	}
#endif

#if defined(CONFIG_MTK_EMMC_HW_CQ)
	card->part_curr = md->part_type;
#endif
	main_md->part_curr = md->part_type;
	return 0;
}

#define RPMB_BLOCK_SIZE 512
static void set_sbc(__u16 blks, __u16 type, u8 req_type,
	struct mmc_command *sbc)
{
	sbc->opcode = MMC_SET_BLOCK_COUNT;
	sbc->arg = blks;
	if ((req_type == RPMB_REQ && type == RPMB_WRITE_DATA) ||
		type == RPMB_PROGRAM_KEY)
		sbc->arg |= 1 << 31;
	sbc->flags = MMC_RSP_R1 | MMC_CMD_AC;
}

static void rpmb_send_req_cmd(struct mmc_card *card,
	struct storage_blk_ioc_rpmb_data *storage_data,
	__u16 blks, __u16 type, struct mmc_request *request)
{
	request->cmd->opcode = MMC_WRITE_MULTIPLE_BLOCK;
	request->data->flags |= MMC_DATA_WRITE;
	if (type == RPMB_RESULT_READ) {
		/* this is the step2 for write data cmd and write key cmd */
		sg_copy_from_buffer(request->data->sg, 1,
			storage_data->data[IOC_CMD_1].buf, RPMB_BLOCK_SIZE * blks);
	} else {
		/* this is step 1 for read data and read counter */
		sg_copy_from_buffer(request->data->sg, 1,
			storage_data->data[IOC_CMD_0].buf, RPMB_BLOCK_SIZE * blks);
	}
	mmc_set_data_timeout(request->data, card);
	mmc_wait_for_req(card->host, request);
}

static void resp_get_sg(struct storage_blk_ioc_rpmb_data *storage_data,
	__u16 blks, __u16 type, struct scatterlist *sg)
{
	bool read_type = (type == RPMB_READ_DATA) ||
		(type == RPMB_GET_WRITE_COUNTER);
	bool write_type = (type == RPMB_WRITE_DATA) ||
		(type == RPMB_PROGRAM_KEY);
	if (read_type) {
		if (storage_data->data[IOC_CMD_1].buf != NULL)
			sg_copy_to_buffer(sg, 1, storage_data->data[IOC_CMD_1].buf,
				RPMB_BLOCK_SIZE * blks);
		else
			tloge("ivalid data1buff, is null\n");
	} else if (write_type) {
		if (storage_data->data[IOC_CMD_2].buf != NULL)
			sg_copy_to_buffer(sg, 1, storage_data->data[IOC_CMD_2].buf,
				RPMB_BLOCK_SIZE * blks);
		else
			tloge("ivalid data1buff, is null\n");
	} else {
		/* do nothing */
		tloge("invalid reqtype %d\n", type);
	}
}

static void rpmb_send_resp_cmd(struct mmc_card *card,
	struct storage_blk_ioc_rpmb_data *storage_data,
	__u16 blks, __u16 type, struct mmc_request *request)
{
	request->cmd->opcode = MMC_READ_MULTIPLE_BLOCK;
	request->data->flags |= MMC_DATA_READ;
	mmc_set_data_timeout(request->data, card);
	mmc_wait_for_req(card->host, request);
	resp_get_sg(storage_data, blks, type, request->data->sg);
}

static int emmc_rpmb_send_command(struct mmc_card *card,
	struct storage_blk_ioc_rpmb_data *storage_data,
	__u16 blks, __u16 type, u8 req_type)
{
	struct mmc_command cmd = {0};
	struct mmc_command sbc = {0};
	struct mmc_data data = {0};
	struct mmc_request request = {NULL};
	struct scatterlist sg;
	u8 *transfer_buf = NULL;

	if (blks == 0) {
		tloge("Invalid blks: 0\n");
		return -EINVAL;
	}

	set_sbc(blks, type, req_type, &sbc);
	request.sbc = &sbc;

	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	request.cmd = &cmd;

	data.blksz = RPMB_BLOCK_SIZE;
	data.blocks = blks;
	data.sg = &sg;
	data.sg_len = 1;
	request.data = &data;

	request.stop = NULL;

	transfer_buf = kzalloc(RPMB_BLOCK_SIZE * blks, GFP_KERNEL);
	if (transfer_buf == NULL)
		return -ENOMEM;

	sg_init_one(&sg, transfer_buf, RPMB_BLOCK_SIZE * blks);

	if (req_type == RPMB_REQ)
		rpmb_send_req_cmd(card, storage_data, blks, type, &request);
	else
		rpmb_send_resp_cmd(card, storage_data, blks, type, &request);

	kfree(transfer_buf);

	if (cmd.error)
		return cmd.error;
	else if (data.error)
		return data.error;
	else
		return 0;
}

static int emmc_rpmb_cmd_proc(struct mmc_card *card, unsigned short type,
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	int err = 0;

	/* STEP 1: send request to RPMB partition */
	if (type == RPMB_WRITE_DATA) {
		err = emmc_rpmb_send_command(card, storage_data,
			storage_data->data[IOC_CMD_0].blocks, type, RPMB_REQ);
	} else {
		/* assemble the frame */
		storage_data->data[IOC_CMD_0].blocks = storage_data->data[IOC_CMD_1].blocks;
		err = emmc_rpmb_send_command(card, storage_data,
			1, type, RPMB_REQ);
	}
	if (err) {
		tloge("step 1, request failed err-%d\n", err);
		goto out;
	}

	/* STEP 2: check write result. Only for WRITE_DATA or Program key */
	if (type == RPMB_WRITE_DATA || type == RPMB_PROGRAM_KEY) {
		err = emmc_rpmb_send_command(card, storage_data,
			1, RPMB_RESULT_READ, RPMB_REQ);
		if (err) {
			tloge("step 2, request result failed err-%d\n", err);
			goto out;
		}
	}

	/* STEP 3: get response from RPMB partition */
	if (type == RPMB_READ_DATA)
		err = emmc_rpmb_send_command(card, storage_data,
			storage_data->data[IOC_CMD_0].blocks, type, RPMB_RESP);
	else
		err = emmc_rpmb_send_command(card, storage_data, 1,
			type, RPMB_RESP);
	if (err)
		tloge("step 3, response failed err-%d\n", err);

out:
	return err;
}

static int rpmb_operation_emmc(enum rpmb_op_type operation,
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	struct emmc_rpmb_blk_data *part_md = NULL;
	int ret;
	struct emmc_rpmb_blk_data *md = NULL;

	struct mmc_card *card = get_card_from_mtk_msdc_host();
	if (card == NULL)
		return -1;

	md = dev_get_drvdata(&card->dev);
	if (md == NULL)
		return -1;

	list_for_each_entry(part_md, &md->part, part) {
		if (part_md->part_type == EXT_CSD_PART_CONFIG_ACC_RPMB)
			break;
	}

	if (part_md->part_type != EXT_CSD_PART_CONFIG_ACC_RPMB)
		return -1;

	mmc_get_card(card);
	ret = emmc_rpmb_switch(card, part_md);
	if (ret) {
		tloge("emmc switch to rpmb failed ret-%x\n", ret);
		goto error;
	}

	switch (operation) {
	case RPMB_OP_RD:
		ret = emmc_rpmb_cmd_proc(card, RPMB_READ_DATA, storage_data);
		break;
	case RPMB_OP_WR_CNT:
		ret = emmc_rpmb_cmd_proc(card, RPMB_GET_WRITE_COUNTER,
			storage_data);
		break;
	case RPMB_OP_WR_DATA:
		ret = emmc_rpmb_cmd_proc(card, RPMB_WRITE_DATA, storage_data);
		break;
	default:
		tloge("receive an unknown operation %d\n", operation);
		goto error;
	}
	if (ret)
		tloge("emmc rpmb cmd proc failed ret-%x\n", ret);

error:
	ret = emmc_rpmb_switch(card, dev_get_drvdata(&card->dev));
	if (ret)
		tloge("emmc switch to main failed ret-%x\n", ret);

	mmc_put_card(card);

	return ret;
}

#ifdef CONFIG_MTK_UFS_SUPPORT
static int rpmb_req_read_data_ufs(
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	struct rpmb_data data;
	struct rpmb_dev *rawdev_ufs_rpmb = NULL;
	int ret;
	uint16_t blk_cnt;

	rawdev_ufs_rpmb = ufs_mtk_rpmb_get_raw_dev();

	blk_cnt = storage_data->data[1].blocks;
	tlogd("rpmb read data ufs, blk_cnt: %u\n", blk_cnt);

	data.req_type = RPMB_READ_DATA;
	data.icmd.nframes = 1;
	data.icmd.frames = (struct rpmb_frame *)storage_data->data[IOC_CMD_0].buf;

	/*
	 * We need to fill-in block_count by ourselves for UFS case.
	 */
	data.icmd.frames->block_count = cpu_to_be16(blk_cnt);

	data.ocmd.nframes = blk_cnt;
	data.ocmd.frames = (struct rpmb_frame *)storage_data->data[IOC_CMD_1].buf;

	ret = rpmb_cmd_req(rawdev_ufs_rpmb, &data);
	if (ret != 0)
		tloge("rpmb req ufs error, ret:0x%x\n", ret);

	tlogd("result 0x%x\n", cpu_to_be16(data.ocmd.frames->result));

	return ret;
}

static int rpmb_req_write_data_ufs(
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	struct rpmb_data data;
	struct rpmb_dev *rawdev_ufs_rpmb = NULL;
	int ret;
	uint16_t blk_cnt;

	rawdev_ufs_rpmb = ufs_mtk_rpmb_get_raw_dev();

	blk_cnt = storage_data->data[IOC_CMD_0].blocks;

	tlogd("blk_cnt: %d\n", blk_cnt);

	/*
	 * Alloc output frame to avoid overwriting input frame
	 * buffer provided by TEE
	 */
	data.ocmd.frames = kzalloc(sizeof(struct rpmb_frame), 0);
	if (data.ocmd.frames == NULL)
		return RPMB_ALLOC_ERROR;

	data.ocmd.nframes = 1;

	data.req_type = RPMB_WRITE_DATA;
	data.icmd.nframes = blk_cnt;
	data.icmd.frames = (struct rpmb_frame *)storage_data->data[IOC_CMD_0].buf;

	ret = rpmb_cmd_req(rawdev_ufs_rpmb, &data);
	if (ret)
		tloge("rpmb_req write_data_ufs error, ret:0x%x\n", ret);

	/*
	 * Microtrust TEE will check write counter in the first frame,
	 * thus we copy response frame to the first frame.
	 */
	if (storage_data->data[IOC_CMD_2].buf == NULL) {
		ret = -1;
		goto free;
	}

	ret = memcpy_s(storage_data->data[IOC_CMD_2].buf,
		storage_data->data[2].buf_bytes,
		data.ocmd.frames, sizeof(*(data.ocmd.frames)));
	if (ret != EOK)
		tloge("frames cpoy fail, ret:0x%x", ret);

	tlogd("result 0x%x\n", cpu_to_be16(data.ocmd.frames->result));

free:
	kfree(data.ocmd.frames);

	return ret;
}

static int rpmb_req_get_wc_ufs(u8 *key, u32 *wc,
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	struct rpmb_data data;
	struct rpmb_dev *rawdev_ufs_rpmb = NULL;
	int ret;

	tlogd("rpmb_req_get_wc_ufs start!!!\n");

	rawdev_ufs_rpmb = ufs_mtk_rpmb_get_raw_dev();

	/*
	 * Initial frame buffers
	 */
	data.icmd.frames = (struct rpmb_frame *)storage_data->data[IOC_CMD_0].buf;
	data.ocmd.frames = (struct rpmb_frame *)storage_data->data[IOC_CMD_1].buf;

	/*
	 * Prepare frame contents.
	 * Input frame (in view of device) only needs nonce
	 */
	data.req_type = RPMB_GET_WRITE_COUNTER;
	data.icmd.nframes = 1;

	/* Output frame (in view of device) */
	data.ocmd.nframes = 1;
	ret = rpmb_cmd_req(rawdev_ufs_rpmb, &data);
	if (ret != 0)
		tloge("rpmb_req_get_wc_ufs error!!! ret:0x%x\n", ret);

	tlogd("end\n");

	return ret;
}

static int rpmb_operation_ufs(enum rpmb_op_type operation,
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	int ret;

	switch (operation) {
	case RPMB_OP_RD:
		ret = rpmb_req_read_data_ufs(storage_data);
		break;
	case RPMB_OP_WR_CNT:
		ret = rpmb_req_get_wc_ufs(NULL, NULL, storage_data);
		break;
	case RPMB_OP_WR_DATA:
		ret = rpmb_req_write_data_ufs(storage_data);
		break;
	default:
		tloge("receive an unknown command id %d.\n", operation);
		break;
	}

	return ret;
}
#endif

int rpmb_ioctl_cmd(enum func_id id, enum rpmb_op_type operation,
	struct storage_blk_ioc_rpmb_data *storage_data)
{
	int ret = 0;
	int boot_type;

	if (storage_data == NULL)
		return -1;

	boot_type = get_boot_type();
	if (boot_type == BOOTDEV_SDMMC)
		ret = rpmb_operation_emmc(operation, storage_data);
#ifdef CONFIG_MTK_UFS_SUPPORT
	else if (boot_type == BOOTDEV_UFS)
		ret = rpmb_operation_ufs(operation, storage_data);
#endif
	return ret;
}
