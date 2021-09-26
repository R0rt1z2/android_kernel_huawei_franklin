#include <asm/unaligned.h>
#include <linux/of.h>
#include <linux/version.h>

#include "ufshcd.h"
#include "mas_ufs.h"
#ifdef CONFIG_MAS_UFS_MANUAL_BKOPS
#include "mas-ufs-bkops.h"
#endif
#ifndef CONFIG_SCSI_UFS_QCOM
#include "ufstt.h"
#endif

#ifdef CONFIG_MAS_ORDER_PRESERVE
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
extern int mas_blk_order_debug_en(void);
#endif

int ufshcd_custom_upiu_order(struct utp_upiu_req *ucd_req_ptr,
				     struct request *req,
				     struct scsi_cmnd *scmd,
				     struct ufs_hba *hba)
{
	unsigned int wo_nr;

	if (unlikely(!req || !req->q))
		return 0;

	if (scsi_is_order_cmd(scmd)) {
		wo_nr = blk_req_get_order_nr(req, true);
		if (wo_nr) {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
			if (mas_blk_order_debug_en())
				pr_err("cmd = 0x%x, order_nr = %u \r\n",
						scmd->cmnd[0], wo_nr);
#endif
			/* CDB 12-15 for Command Order */
			ucd_req_ptr->sc.cdb[12] = (unsigned char)(wo_nr);
			ucd_req_ptr->sc.cdb[13] = (unsigned char)(wo_nr >> 8);
			ucd_req_ptr->sc.cdb[14] = (unsigned char)(wo_nr >> 16);
			ucd_req_ptr->sc.cdb[15] = (unsigned char)(wo_nr >> 24);
		}
	}

	return 0;
}

void ufshcd_read_vendor_feature(struct ufs_hba *hba, u8 *desc_buf)
{
	uint32_t vendor_feature;

	if (desc_buf[UNIT_DESC_PARAM_LEN] < DEVICE_DESC_PARAM_FEATURE)
		return;

#define VENDOR_FEATURE_ORDER BIT(2)
	vendor_feature = get_unaligned_be32(&desc_buf[DEVICE_DESC_PARAM_FEATURE]);
	if (vendor_feature & VENDOR_FEATURE_ORDER) {
		hba->host->order_enabled = 1;
		dev_info(hba->dev, "support order feature\n");
	}
}
#endif /* CONFIG_MAS_ORDER_PRESERVE */

#ifdef CONFIG_MAS_MQ_USING_CP
/* UPIU Command Priority */
#define UPIU_CMD_PRIO		0x04
void mas_ufshcd_prepare_req_desc_hdr(struct ufshcd_lrb *lrbp, u32 *upiu_flags)
{
	if (unlikely(lrbp->cmd && lrbp->cmd->request && req_hoq(lrbp->cmd->request)))
		*upiu_flags |= UPIU_TASK_ATTR_HEADQ;
	if (unlikely(lrbp->cmd && lrbp->cmd->request && req_cp(lrbp->cmd->request)))
		*upiu_flags |= UPIU_CMD_PRIO;
#ifdef CONFIG_MAS_UFSTT
	if (unlikely(lrbp->cmd && lrbp->cmd->cmnd[0] == UFSTT_READ_BUFFER && is_ufstt_batch_mode()))
		*upiu_flags |= UPIU_CMD_PRIO;
#endif
}
#endif

void mas_ufshcd_slave_config(struct request_queue *q, struct scsi_device *sdev)
{
#ifdef CONFIG_MAS_ORDER_PRESERVE
	blk_queue_order_enable(q, sdev->host->order_enabled);
#endif
#ifdef CONFIG_MAS_UNISTORE_PRESERVE
	mas_blk_queue_unistore_enable(q, sdev->host->unistore_enable);
#endif
#ifdef CONFIG_MAS_UFS_MANUAL_BKOPS
	mas_ufs_manual_bkops_config(sdev);
#endif
}

static unsigned char *sync_cache_buffer;
int ufshcd_send_scsi_sync_cache_init(void)
{
	if (likely(!sync_cache_buffer)) {
		sync_cache_buffer = kzalloc((size_t)SCSI_SENSE_BUFFERSIZE, GFP_KERNEL);
		if (unlikely(!sync_cache_buffer))
			return -ENOMEM;
	}

	return 0;
}

void ufshcd_send_scsi_sync_cache_deinit(void)
{
	if (likely(sync_cache_buffer)) {
		kfree(sync_cache_buffer);
		sync_cache_buffer = NULL;
	}
}

static int ufshcd_trylock_hostlock(struct ufs_hba *hba, unsigned long* flags)
{
	int locked = 0;
	unsigned int trycount = 100000;

	do {
		locked = spin_trylock_irqsave(hba->host->host_lock, *flags);
		if (locked)
			break;
		udelay(10);
	} while (--trycount);

	return locked;
}

static void ufshcd_sync_cache_setup_lrbp(struct ufshcd_lrb *lrbp,
		struct scsi_cmnd *cmd, int tag)
{
	lrbp->cmd = cmd;
	lrbp->sense_bufflen = SCSI_SENSE_BUFFERSIZE;
	lrbp->sense_buffer = cmd->sense_buffer;
	lrbp->task_tag = tag;
	lrbp->lun = ufshcd_scsi_to_upiu_lun((unsigned int)cmd->device->lun);
	lrbp->intr_cmd = false;
	lrbp->command_type = UTP_CMD_TYPE_SCSI;
}

static void __ufshcd_print_doorbell(struct ufs_hba *hba, u32 tm_doorbell,
		u32 tr_doorbell, char *s)
{
	dev_err(hba->dev, "wait door bell clean %s:tm_doorbell:0x%x, "
			  "tr_doorbell:0x%x\n", s, tm_doorbell, tr_doorbell);
}

/* caller need to hold host_lock */
int __ufshcd_wait_for_doorbell_clr(struct ufs_hba *hba)
{
	u32 tr_doorbell, tm_doorbell, wait_timeout_us;
	int ret = 0;
	ktime_t start = ktime_get();

	wait_timeout_us = 2 * USEC_PER_SEC;
	tm_doorbell = ufshcd_readl(hba, REG_UTP_TASK_REQ_DOOR_BELL);
	tr_doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
	if (!tm_doorbell && !tr_doorbell)
		return 0;

	__ufshcd_print_doorbell(hba, tm_doorbell, tr_doorbell, "begin");

	/*
	 * Wait for all the outstanding tasks/transfer requests.
	 * Verify by checking the doorbell registers are clear.
	 */
	do {
		tm_doorbell = ufshcd_readl(hba, REG_UTP_TASK_REQ_DOOR_BELL);
		tr_doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
		if (ktime_to_us(ktime_sub(ktime_get(), start)) > wait_timeout_us) {
			ret = -EIO;
			dev_err(hba->dev, "wait doorbell clean timeout\n");
			break;
		}
	} while (tm_doorbell || tr_doorbell);

	__ufshcd_print_doorbell(hba, tm_doorbell, tr_doorbell, "end");

	return ret;
}

#define SYNC_CACHE_WAIT_DELAY 50
static int ufshcd_sync_cache_irq_safe(struct ufs_hba *hba,
					struct scsi_cmnd *cmd,	long timeout)
{
	int tag;
	unsigned long flags;
	struct ufshcd_lrb *lrbp = NULL;
	volatile u32 ie_value = 0;
	volatile u32 tr_doorbell = 0;
	bool clk_gate_suspended = false;

	scsi_block_requests(hba->host);
	pm_runtime_get_sync(hba->dev);
	spin_lock_irqsave(hba->host->host_lock, flags);

	if (ufshcd_is_clkgating_allowed(hba)) {
		if (hba->clk_gating.state != CLKS_ON)
			goto unlock;
		clk_gate_suspended = hba->clk_gating.is_suspended;
		hba->clk_gating.is_suspended = true;
	}

	if (__ufshcd_wait_for_doorbell_clr(hba)) {
		dev_err(hba->dev, "wait doorbell clear timeout\n");
		goto restore_clk_gate;
	}

	ie_value = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);
	if (ie_value)
		ufshcd_writel(hba, 0, REG_INTERRUPT_ENABLE);
	tag = (int)ffz(hba->lrb_in_use);
	if (tag >= hba->nutrs)
		goto restore_ie;

	__set_bit(tag, &hba->lrb_in_use);
	lrbp = &hba->lrb[tag];
	ufshcd_sync_cache_setup_lrbp(lrbp, cmd, tag);
	if (ufshcd_compose_upiu(hba, lrbp))
		goto restore_ie;

	/* Make sure descriptors are ready before ringing the doorbell */
	wmb();
	/* issue command to the controller */
	__set_bit(tag, &hba->outstanding_reqs);

	ufshcd_writel(hba, 1 << (unsigned int)tag, REG_UTP_TRANSFER_REQ_DOOR_BELL);

	/* Make sure that doorbell is committed immediately */
	wmb();
	while (timeout) {

		tr_doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);

		if (!(tr_doorbell & (1U << (unsigned int)tag))) {
			hba->outstanding_reqs ^= (1UL << (unsigned int)tag);
			goto scsi_cmd_deinit;
		}
		udelay(SYNC_CACHE_WAIT_DELAY);
		timeout -= SYNC_CACHE_WAIT_DELAY;
	}
scsi_cmd_deinit:
	lrbp->cmd = NULL;
	__clear_bit(tag, &hba->lrb_in_use);

restore_ie:
	if (ie_value)
		ufshcd_writel(hba, ie_value, REG_INTERRUPT_ENABLE);
restore_clk_gate:
	if (ufshcd_is_clkgating_allowed(hba))
		hba->clk_gating.is_suspended = clk_gate_suspended;
unlock:
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	pm_runtime_put_autosuspend(hba->dev);
	scsi_unblock_requests(hba->host);

	return 0;
}

#define SYNC_CACHE_TIMEOUT (5 * USEC_PER_SEC)
static int ufshcd_send_scsi_sync_cache_on_specified_disk(
	struct ufs_hba *hba, struct scsi_device *sdp)
{
	int ret;
	struct scsi_cmnd cmnd;
	unsigned char cmd[10] = {0};

	cmd[0] = SYNCHRONIZE_CACHE;
	memset(&cmnd, 0, sizeof(cmnd));
	ufshcd_compose_scsi_cmd(&cmnd, sdp, cmd, sync_cache_buffer, DMA_NONE, NULL, 0, 0);
	ret = ufshcd_sync_cache_irq_safe(hba, &cmnd, SYNC_CACHE_TIMEOUT);

	dev_err(hba->dev, "UFS:<%s> Emergency sync cache lun=%llu ret = %d\n",
		__func__, sdp->lun, ret);
	return ret;
}

int ufshcd_direct_flush_test(struct request_queue *q)
{
	int ret;

	pr_err("%s %d++\n", __func__, __LINE__);
	ret = ufshcd_direct_flush(q->queuedata);
	pr_err("%s %d--\n", __func__, __LINE__);
	return ret;
}

/* This function is called with irqsave */
int ufshcd_direct_flush(struct scsi_device *sdev)
{
	int ret;
	unsigned long flags;
	struct Scsi_Host *host = NULL;
	struct ufs_hba *hba = NULL;

	if (!sdev ) {
		pr_err("%s, sdev is null!\n", __func__);
		return -ENODEV;
	}
	host = sdev->host;
	hba = shost_priv(host);
	if (!hba ) {
		pr_err( "%s, hba is null!\n", __func__);
		return -ENODEV;
	}

	if (hba->is_sys_suspended || hba->pm_op_in_progress){
		dev_err(hba->dev, "%s sys has suspended!\n", __func__);
		return 0;
	}
	if (!ufshcd_trylock_hostlock(hba, &flags)) {
		dev_err(hba->dev, "%s, can't get the hostlock!\n", __func__);
		return -EIO;
	}
	ret = scsi_device_get(sdev);
	if (!ret && !scsi_device_online(sdev)) {
		dev_err(hba->dev, "%s, scsi_device_get error or device not online, %d\n", __func__, ret);
		ret = -ENODEV;
	}
	if (hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL) {
		dev_err(hba->dev, "%s, ufshcd_state = %d\n",
			__func__, hba->ufshcd_state);
		ret = SCSI_MLQUEUE_HOST_BUSY;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	if (ret)
		goto out;

	ret = ufshcd_send_scsi_sync_cache_on_specified_disk(hba, sdev);
out:
	scsi_device_put(sdev);
	return ret;
}

void ufshcd_dump_status(
	struct Scsi_Host *host, enum blk_dump_scene dump_type)
{
	struct ufs_hba *hba = shost_priv(host);
	struct ufshcd_lrb *lrbp = NULL;
	int tag;

#ifdef CONFIG_MAS_BLK
	dev_err(hba->dev, "ufshcd: lrb_in_use = 0x%lx\n", hba->lrb_in_use);
	if (dump_type != BLK_DUMP_PANIC)
		return;

	for_each_set_bit(tag, &hba->outstanding_reqs, hba->nutrs) {
		lrbp = &hba->lrb[tag];
		dev_err(hba->dev,
			"UPIU[%d] - issue time %lld - complete time %lld\n",
			tag, ktime_to_us(lrbp->issue_time_stamp),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0))
			ktime_to_us(lrbp->compl_time_stamp));
#else
			ktime_to_us(lrbp->complete_time_stamp));
#endif
	}
#endif
}

int ufshcd_send_scsi_request_sense(struct ufs_hba *hba,
		struct scsi_device *sdp, unsigned int timeout, bool eh_handle)
{
	int ret;
	unsigned char *buffer = NULL;
	unsigned char *dma_buf = NULL;
	struct scatterlist sglist;
	struct scsi_cmnd cmnd;
	unsigned char cmd[6] = {REQUEST_SENSE,	 0, 0, 0,
				SCSI_SENSE_BUFFERSIZE, 0};

	buffer = kzalloc((size_t)SCSI_SENSE_BUFFERSIZE, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto out;
	}

	dma_buf = kzalloc((size_t)PAGE_SIZE, GFP_KERNEL);
	if (!dma_buf) {
		ret = -ENOMEM;
		goto free_buffer;
	}

	sg_init_one(&sglist, dma_buf, (unsigned int)PAGE_SIZE);
	memset(&cmnd, 0, sizeof(cmnd));
	ufshcd_compose_scsi_cmd(&cmnd, sdp, cmd, buffer, DMA_FROM_DEVICE,
				&sglist, 1, (unsigned int)PAGE_SIZE);

	ret = ufshcd_queuecommand_directly(hba, &cmnd, timeout, eh_handle);
	if (ret)
		dev_err(hba->dev, "%s: failed with err %d\n", __func__, ret);

	kfree(dma_buf);

free_buffer:
	kfree(buffer);
out:
	return ret;
}

int ufshcd_send_scsi_ssu(struct ufs_hba *hba,
				struct scsi_device *sdp,
				unsigned char *cmd,
				unsigned int timeout,
				struct scsi_sense_hdr *sshdr)
{
	int ret;
	struct scsi_cmnd cmnd;
	unsigned char *buffer;

	buffer = kzalloc((size_t)SCSI_SENSE_BUFFERSIZE, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto out;
	}

	memset(&cmnd, 0, sizeof(cmnd));
	ufshcd_compose_scsi_cmd(&cmnd, sdp, cmd, buffer, DMA_NONE, NULL, 0, 0);
	ret = ufshcd_queuecommand_directly(hba, &cmnd, timeout, true);
	if (ret)
		dev_err(hba->dev, "%s: failed with err %d\n", __func__, ret);

	kfree(buffer);
out:
	return ret;
}

#define LAST_LUN 2
/*
 * Sync cache for already knowed fixed lun (0-4)
 * If failed, then failed, skip SCSI layer means skip exception handler
 */
int ufshcd_send_scsi_sync_cache(struct ufs_hba *hba, struct scsi_device *sdp)
{
	int ret = 0;
	unsigned int i;
	unsigned char *buffer = NULL;
	struct scsi_cmnd cmnd;
	struct scsi_device *psdev = NULL;
	unsigned char cmd[10] = {0};

	if (!sdp)
		return -ENODEV;

	cmd[0] = SYNCHRONIZE_CACHE;

	psdev = kzalloc(sizeof(struct scsi_device), GFP_KERNEL);
	if (!psdev) {
		buffer = NULL;
		ret = -ENOMEM;
		goto out;
	}

	buffer = kzalloc((size_t)SCSI_SENSE_BUFFERSIZE, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto out;
	}

	memset(&cmnd, 0, sizeof(cmnd));

	/* find scsi_host by device well know's host member */
	psdev->host = sdp->host;

#ifdef CONFIG_SCSI_UFS_QCOM
	for (i = 0; i < hba->lu_num; i++) {
#else
	for (i = LAST_LUN; i > 0; i--) {
#endif
#ifdef CONFIG_SCSI_UFS_LUN_PROTECT
		/*
		 * LU0/LU1/LU2 are write-protected, and sync-cache cannot be
		 * executed.
		 */
		if ((i == 0) || (i == 1) || (i == 2))
			continue;
#endif
#ifdef CONFIG_SCSI_UFS_QCOM
		if (hba->lu_wp[i].lu_wp != UFS_LU_NO_WP)
			continue;
#endif
		psdev->lun = i;
		ufshcd_compose_scsi_cmd(&cmnd, psdev, cmd, buffer, DMA_NONE,
					NULL, 0, 0);
		ret = ufshcd_queuecommand_directly(
			hba, &cmnd, (unsigned int)QUUEUE_CMD_TIMEOUT, true);
		if (ret) {
			dev_err(hba->dev, "%s: failed for lun %llu, ret = %d\n",
				__func__, psdev->lun, ret);
			goto out;
		}
	}

out:
	kfree(psdev);
	kfree(buffer);

	return ret;
}

#ifdef CONFIG_HYPERHOLD_CORE
int ufshcd_get_health_info(struct scsi_device *sdev,
	u8 *pre_eol_info, u8 *life_time_est_a, u8 *life_time_est_b)
{
	int ret;
	struct ufs_hba *hba = NULL;
	u8 buff[QUERY_DESC_HEALTH_MAX_SIZE];

	if ((!sdev) || (!pre_eol_info) || (!life_time_est_a) ||
		(!life_time_est_b))
		return -EFAULT;

	hba = shost_priv(sdev->host);
	if (!hba)
		return -EFAULT;

	ret = ufshcd_read_health_desc(hba, buff, QUERY_DESC_HEALTH_MAX_SIZE);
	if (ret) {
		dev_err(hba->dev, "%s: Failed getting device health info\n",
			__func__);
		return ret;
	}

	*pre_eol_info = buff[HEALTH_DEVICE_DESC_PARAM_PREEOL];
	*life_time_est_a = buff[HEALTH_DEVICE_DESC_PARAM_LIFETIMEA];
	*life_time_est_b = buff[HEALTH_DEVICE_DESC_PARAM_LIFETIMEB];

	return 0;
}
#endif /* CONFIG_HYPERHOLD_CORE */

#ifdef CONFIG_MAS_UFS_MANUAL_BKOPS
void mas_ufs_populate_mgc_dt(struct ufs_hba *hba)
{
	struct device_node *child_np = NULL;
	struct device_model_para model_para;
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	unsigned int man_id;
	int ret;
	int is_white;
	struct mas_ufs_bkops_id *bkops_id = NULL;

	INIT_LIST_HEAD(&hba->bkops_whitelist);
	INIT_LIST_HEAD(&hba->bkops_blacklist);
	for_each_child_of_node (np, child_np) {
		ret = of_property_read_string(child_np, "compatible",
			(const char **)(&(model_para.compatible)));
		if (ret) {
			dev_err(hba->dev, "check the compatible %s\n", child_np->name);
			continue;
		} else {
			if (!strcmp("white", model_para.compatible)) {
				is_white = 1;
			} else if (!strcmp("black", model_para.compatible)) {
				is_white = 0;
			} else {
				dev_err(hba->dev, "check the compatible %s\n", child_np->name);
				continue;
			}
		}
		ret = of_property_read_u32(child_np, "manufacturer_id", &man_id);
		if (ret) {
#ifdef CONFIG_MAS_DEBUG_FS
			dev_err(hba->dev, "check the manufacturer_id %s\n", child_np->name);
#endif
			continue;
		}

		ret = of_property_read_string(
			child_np, "model",(const char **)(&(model_para.model)));
		if (ret) {
			dev_err(hba->dev, "check the model %s\n", child_np->name);
			continue;
		}

		ret = of_property_read_string(
			child_np, "rev", (const char **)(&(model_para.rev)));
		if (ret) {
			dev_err(hba->dev, "check the rev %s\n", child_np->name);
			continue;
		}

		bkops_id = devm_kzalloc(hba->dev, sizeof(*bkops_id), GFP_KERNEL);
		if (!bkops_id) {
			dev_err(hba->dev, "%s %d Failed to alloc bkops_id\n",
				__func__, __LINE__);
			return;
		}

		bkops_id->manufacturer_id = man_id;
		bkops_id->ufs_model = model_para.model;
		bkops_id->ufs_rev = model_para.rev;
		INIT_LIST_HEAD(&bkops_id->p);
		if (is_white)
			list_add(&bkops_id->p, &hba->bkops_whitelist);
		else
			list_add(&bkops_id->p, &hba->bkops_blacklist);
	}
}
#endif /* CONFIG_MAS_UFS_MANUAL_BKOPS */

void ufshcd_mas_mq_init(struct Scsi_Host *host)

{
	host->use_blk_mq = 1;
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_BUSY_IDLE_ENABLE);
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_IO_LATENCY_WARNING);
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_FLUSH_REDUCING);
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_UNMAP_IN_SOFTIRQ);
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_SCSI_QUIESCE_IN_LLD);
	if (!host->use_blk_mq)
		return;
	host->queue_quirk_flag |= SHOST_QUIRK(SHOST_QUIRK_MAS_UFS_MQ);
	if (host->queue_quirk_flag & SHOST_QUIRK(SHOST_QUIRK_MAS_UFS_MQ)) {
		host->queue_quirk_flag |=
		    SHOST_QUIRK(SHOST_QUIRK_DRIVER_TAG_ALLOC);
		host->nr_hw_queues = 1;
		host->mq_queue_depth = 192;
		host->mq_reserved_queue_depth = 64;
		host->mq_high_prio_queue_depth = 64;
		host->can_queue =
		    host->mq_queue_depth * (int)host->nr_hw_queues;
	}
}
