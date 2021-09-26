/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#endif
#include "mdla.h"
#include "mdla_hw_reg.h"
#include "mdla_ion.h"
#include "mdla_trace.h"
#include "mdla_debug.h"
#include "mdla_util.h"
#include "mdla_power_ctrl.h"
#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
#include "apusys_power.h"
#endif

static struct platform_device *mdlactlPlatformDevice;

static void mdla_cfg_write_with_mdlaid(u32 mdlaid, u32 value, u32 offset)
{
	iowrite32(value, mdla_reg_control[mdlaid].apu_mdla_config_top + offset);
}


static void mdla_reg_write_with_mdlaid(u32 mdlaid, u32 value, u32 offset)
{
	iowrite32(value,
		mdla_reg_control[mdlaid].apu_mdla_cmde_mreg_top + offset);
}

#define mdla_cfg_set_with_mdlaid(mdlaid, mask, offset) \
	mdla_cfg_write_with_mdlaid(mdlaid,\
	mdla_cfg_read_with_mdlaid(mdlaid, offset) | (mask), (offset))


int mdla_dts_map(struct platform_device *pdev)
{
	struct resource *apu_mdla_command; /* IO mem resources */
	struct resource *apu_mdla_config; /* IO mem resources */
	struct resource *apu_mdla_biu; /* IO mem resources */
	struct resource *apu_mdla_gsm; /* IO mem resources */
	struct resource *apu_conn; /* IO mem resources */
	//struct resource *infracfg_ao; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct device_node *node;

	int rc = 0;
	int i;

	mdlactlPlatformDevice = pdev;

	dev_info(dev, "Device Tree Probing\n");

	for (i = 0; i < mdla_max_num_core; i++) {
		/* Get iospace for MDLA Config */
		apu_mdla_config = platform_get_resource(pdev,
			IORESOURCE_MEM, i+(i*2));
		if (!apu_mdla_config) {
			mdla_drv_debug("invalid address\n");
			return -ENODEV;
		}

		/* Get iospace for MDLA Command */
		apu_mdla_command = platform_get_resource(pdev,
			IORESOURCE_MEM, i+1+(i*2));
		if (!apu_mdla_command) {
			dev_info(dev, "invalid address\n");
			return -ENODEV;
		}

		/* Get iospace for MDAL PMU */
		apu_mdla_biu = platform_get_resource(pdev,
			IORESOURCE_MEM, i+2+(i*2));
		if (!apu_mdla_biu) {
			dev_info(dev, "apu_mdla_biu address\n");
			return -ENODEV;
		}

		mdla_reg_control[i].apu_mdla_config_top = ioremap_nocache(
				apu_mdla_config->start,
				apu_mdla_config->end -
				apu_mdla_config->start + 1);
		if (!mdla_reg_control[i].apu_mdla_config_top) {
			dev_info(dev, "mtk_mdla: Could not allocate iomem\n");
			rc = -EIO;
			return rc;
		}

		mdla_reg_control[i].apu_mdla_cmde_mreg_top = ioremap_nocache(
				apu_mdla_command->start,
				apu_mdla_command->end -
				apu_mdla_command->start + 1);
		if (!mdla_reg_control[i].apu_mdla_cmde_mreg_top) {
			dev_info(dev, "mtk_mdla: Could not allocate iomem\n");
			rc = -EIO;
			return rc;
		}

		mdla_reg_control[i].apu_mdla_biu_top = ioremap_nocache(
				apu_mdla_biu->start,
				apu_mdla_biu->end - apu_mdla_biu->start + 1);
		if (!mdla_reg_control[i].apu_mdla_biu_top) {
			dev_info(dev, "mtk_mdla: Could not allocate iomem\n");
			rc = -EIO;
			return rc;
		}

		dev_info(dev, "mdla_config_top at 0x%08lx mapped to 0x%08lx\n",
				(unsigned long __force)apu_mdla_config->start,
				(unsigned long __force)apu_mdla_config->end);

		dev_info(dev, "mdla_command at 0x%08lx mapped to 0x%08lx\n",
				(unsigned long __force)apu_mdla_command->start,
				(unsigned long __force)apu_mdla_command->end);

		dev_info(dev, "mdla_biu_top at 0x%08lx mapped to 0x%08lx\n",
				(unsigned long __force)apu_mdla_biu->start,
				(unsigned long __force)apu_mdla_biu->end);

	}


	/* Get iospace GSM */
	apu_mdla_gsm = platform_get_resource(pdev,
		IORESOURCE_MEM,
		3);
	if (!apu_mdla_gsm) {
		dev_info(dev, "apu_gsm address\n");
		return -ENODEV;
	}

	/* Get iospace APU CONN */
	apu_conn = platform_get_resource(pdev,
		IORESOURCE_MEM,
		4);
	if (!apu_conn) {
		mdla_drv_debug("apu_conn address\n");
		return -ENODEV;
	}

	apu_mdla_gsm_top = ioremap_nocache(apu_mdla_gsm->start,
			apu_mdla_gsm->end - apu_mdla_gsm->start + 1);
	if (!apu_mdla_gsm_top) {
		dev_info(dev, "mtk_mdla: Could not allocate iomem\n");
		rc = -EIO;
		return rc;
	}
	apu_mdla_gsm_base = (void *) apu_mdla_gsm->start;
	pr_info("%s: apu_mdla_gsm_top: %p, apu_mdla_gsm_base: %p\n",
		__func__, apu_mdla_gsm_top, apu_mdla_gsm_base);

	apu_conn_top = ioremap_nocache(apu_conn->start,
			apu_conn->end - apu_conn->start + 1);
	if (!apu_conn_top) {
		mdla_drv_debug("mtk_mdla: Could not allocate apu_conn_top\n");
		rc = -EIO;
		return rc;
	}

	dev_info(dev, "apu_mdla_gsm at 0x%08lx mapped to 0x%08lx\n",
			(unsigned long __force)apu_mdla_gsm->start,
			(unsigned long __force)apu_mdla_gsm->end);

	dev_info(dev, "apu_conn_top at 0x%08lx mapped to 0x%08lx\n",
			(unsigned long __force)apu_conn->start,
			(unsigned long __force)apu_conn->end);

	node = pdev->dev.of_node;
	if (!node) {
		dev_info(dev, "get mdla device node err\n");
		return rc;
	}

	for (i = 0; i < mdla_max_num_core; i++) {
		mdla_irqdesc[i].irq  = irq_of_parse_and_map(node, i);
		if (!mdla_irqdesc[i].irq) {
			dev_info(dev, "get mdla irq: %d failed\n", i);
			return rc;
		}
		rc = request_irq(mdla_irqdesc[i].irq, mdla_irqdesc[i].handler,
				IRQF_TRIGGER_HIGH, DRIVER_NAME, dev);

		if (rc) {
			dev_info(dev, "mtk_mdla[%d]: Could not allocate interrupt %d.\n",
					i, mdla_irqdesc[i].irq);
		}
		dev_info(dev, "request_irq %d done\n", mdla_irqdesc[i].irq);
	}

	return 0;
}

void mdla_reset(unsigned int core, int res)
{
	const char *str = mdla_get_reason_str(res);
	unsigned long flags;

	/*use power down==>power on apis insted bus protect init*/
	mdla_drv_debug("%s: MDLA RESET: %s(%d)\n", __func__,
		str, res);

	spin_lock_irqsave(&mdla_devices[core].hw_lock, flags);
	mdla_cfg_write_with_mdlaid(core, 0xffffffff, MDLA_CG_CLR);

	mdla_reg_write_with_mdlaid(core, MDLA_IRQ_MASK & ~(MDLA_IRQ_SWCMD_DONE),
		MREG_TOP_G_INTP2);

	/* for DCM and CG */
	mdla_reg_write_with_mdlaid(core, cfg_eng0, MREG_TOP_ENG0);
	mdla_reg_write_with_mdlaid(core, cfg_eng1, MREG_TOP_ENG1);
	mdla_reg_write_with_mdlaid(core, cfg_eng2, MREG_TOP_ENG2);
	/*TODO, 0x0 after verification*/
	mdla_reg_write_with_mdlaid(core, cfg_eng11, MREG_TOP_ENG11);

#ifdef CONFIG_MTK_MDLA_ION
	mdla_cfg_set_with_mdlaid(core, MDLA_AXI_CTRL_MASK, MDLA_AXI_CTRL);
	mdla_cfg_set_with_mdlaid(core, MDLA_AXI_CTRL_MASK, MDLA_AXI1_CTRL);
#endif
	spin_unlock_irqrestore(&mdla_devices[core].hw_lock, flags);

	mdla_profile_reset(core, str);//TODO, to confirm multi mdla settings

}

int mdla_zero_skip_detect(unsigned int core_id)
{
	u32 debug_if_0, debug_if_2, it_front_c_invalid;

	debug_if_0 =
		mdla_reg_read_with_mdlaid(core_id, MREG_DEBUG_IF_0);
	debug_if_2 =
		mdla_reg_read_with_mdlaid(core_id, MREG_DEBUG_IF_2);
	it_front_c_invalid =
		mdla_reg_read_with_mdlaid(core_id, MREG_IT_FRONT_C_INVALID);

	if (debug_if_0 == 0x6) {
		if ((debug_if_2 == it_front_c_invalid) ||
			(debug_if_2 == (it_front_c_invalid/2))) {
			mdla_timeout_debug("core:%d, %s: match zero skip issue\n",
				core_id,
				__func__);
			mdla_devices[core_id].mdla_zero_skip_count++;
			return -1;
		}
	}
	return 0;
}

static inline u32 mdla_get_data(void *base_kva, u32 offset)
{
	return (*(u32 *)(base_kva + offset));
}

static inline void mdla_set_data(void *base_kva, u32 offset, u32 val)
{
	(*(u32 *)(base_kva + offset)) = val;
}

void mdla_del_free_command(struct command_entry *ce)
{
	struct cmd_tmp *cb;
	struct list_head *tmp, *next;

	if (unlikely(ce->cmd_list_head == NULL))
		return;
	if (unlikely(ce->cmd_tmp_size >= ce->count))
		return;
	list_for_each_safe(tmp, next, ce->cmd_list_head) {
		cb = list_entry(tmp, struct cmd_tmp, node);
		list_del(&cb->node);
		kfree(cb);
	}
	kfree(ce->cmd_list_head);
}

void mdla_split_command(struct command_entry *ce)
{
	size_t i, j;
	u32 cur_len = 0;
	u32 tid = 0;
	u32 size = ce->cmd_tmp_size;
	u32 cmd_count = ce->count;
	struct cmd_tmp *cb;
	void *stop_addr;
	void *first_addr;
	void *pre_addr;

	if (ce->cmd_list_head == NULL)
		return;
	if (!list_empty(ce->cmd_list_head))
		return;
	if (ce->cmd_tmp_size >= ce->count)
		return;
	if (unlikely(ce->cmdbuf == NULL))
		return;
	apusys_mem_invalidate(ce->cmdbuf);
	for (i = 1; i <= cmd_count; cur_len = 0) {
		for (j = i; j <= cmd_count; ++j) {
			cur_len++;
			stop_addr = ce->kva + (j - 1) * 0x1C0;
			if (cur_len >= size &&
				(mdla_get_data(stop_addr, 0x70) & 0x1000))
				break;
		}

		cb = kzalloc(sizeof(*cb), GFP_KERNEL);
		if (!cb)
			return;

		cb->index = i;
		cb->size = cur_len;
		tid = i + cur_len - 1;
		list_add_tail(&cb->node, ce->cmd_list_head);

		stop_addr = ce->kva + (tid - 1) * 0x1C0;
		mdla_set_data(
			stop_addr, 0x154,
			mdla_get_data(stop_addr, 0x154) | 0x1000000);
		first_addr = ce->kva + (cb->index - 1) * 0x1C0;
		mdla_set_data(first_addr, 0x15C,
			(mdla_get_data(first_addr, 0x15C) & 0xFFFEFFFF));

		if (tid > cb->index &&
			!(mdla_get_data(first_addr, 0x70) & 0x1000)) {
			first_addr = ce->kva + (cb->index) * 0x1C0;
			mdla_set_data(first_addr, 0x15C,
				(mdla_get_data(
					first_addr, 0x15C) & 0xFFFEFFFF));
		}
		pre_addr = ce->kva + (tid - 1) * 0x1C0;

		mdla_set_data(pre_addr, 0x15C,
			mdla_get_data(pre_addr, 0x15C) & 0xFFFF7FFF);

		pre_addr = ce->kva + (tid - 2) * 0x1C0;
		if (tid > 1 && !(mdla_get_data(pre_addr, 0x70) & 0x1000)) {
			mdla_set_data(
				pre_addr, 0x154,
				mdla_get_data(pre_addr, 0x154) | 0x1000000);
			mdla_set_data(pre_addr, 0x15C,
				mdla_get_data(pre_addr, 0x15C) & 0xFFFF7FFF);
		}
		i += cur_len;
	}
	if (likely(ce->cmdbuf != NULL))
		apusys_mem_flush(ce->cmdbuf);
}

static inline struct mdla_scheduler *mdla_get_scheduler(unsigned int core_id)
{
	return (core_id >= MTK_MDLA_MAX_NUM) ?
		NULL : mdla_devices[core_id].sched;
}
static void mdla_issue_ce_cdma_error(
	uint32_t core_id,
	struct command_entry *ce,
	uint32_t cmda1_golden,
	uint32_t cmda2_golden)
{
	uint32_t cdma1, cdma2;

	cdma1 = mdla_reg_read_with_mdlaid(
			core_id, MREG_TOP_G_CDMA1);

	cdma2 = mdla_reg_read_with_mdlaid(
			core_id, MREG_TOP_G_CDMA2);

	if (cdma1 != cmda1_golden) {
		ce->state |= (1 << CE_ISSUE_ERROR1);
		ce_func_trace(ce, F_ISSUE|1);
	}
	if (cdma2 != cmda2_golden) {
		ce->state |= (1 << CE_ISSUE_ERROR2);
		ce_func_trace(ce, F_ISSUE|2);
	}
}

/*
 * Enqueue one CE and start scheduler
 * resume = 0: add tail
 * resume = 1: add begin
 */
void mdla_enqueue_ce_2_1(
	unsigned int core_id,
	struct command_entry *ce,
	uint32_t resume)
{
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);

	/* ce == NULL will occur when mdla_normal_dual_cmd_issue call it */
	if (unlikely(sched == NULL || ce == NULL))
		return;

	if (resume == 0) {
		list_add_tail(&ce->node, &(sched->ce_list[ce->priority]));
		ce->state |= (1 << CE_QUEUE);
	} else if (resume > 0) {
		list_add(&ce->node, &(sched->ce_list[ce->priority]));
		ce->state |= (1 << CE_QUEUE_RESUME);
	}
	ce_func_trace(ce, (F_ENQUEUE|resume));
}

/*
 * Dequeue a prioritized CE from active CE queue, handle the context switch of
 * the original processing_ce, and set the prioritized CE as processing_ce.
 *
 * NOTE: sched->lock should be acquired by caller
 */
struct command_entry *mdla_dequeue_ce_2_1(unsigned int core_id)
{
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);
	struct command_entry *new_ce = NULL;
	int16_t i = 0;

	if (unlikely(sched == NULL))
		return NULL;

	for (i = PRIORITY_LEVEL - 1; i >= 0; i--) {
		/* get one CE from the active CE queue */
		new_ce = list_first_entry_or_null(
					&(sched->ce_list[i]),
					struct command_entry,
					node);
		if (new_ce != NULL) {
			/* remove prioritized CE from active CE queue */
			list_del(&new_ce->node);
			new_ce->state |= (1 << CE_DEQUE);
			ce_func_trace(new_ce, F_DEQUEUE);
			return new_ce;
		}
	}

	return new_ce;
}

void mdla_preempt_ce_2_1(unsigned int core_id, struct command_entry *high_ce)
{
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);
	struct command_entry *low_ce = sched->pro_ce;
	uint64_t deadline =
		get_jiffies_64() + msecs_to_jiffies(mdla_timeout);

	sched->pro_ce->req_end_t = sched_clock();
	sched->pro_ce->state |= (1 << CE_PREEMPTED);
	mdla_preemption_times++;
	sched->enqueue_ce(core_id, low_ce, 1);
	low_ce->deadline_t = deadline;
	high_ce->state |= (1 << CE_PREEMPTING);
	sched->pro_ce = high_ce;
}

/*
 * Consume the processing_ce:
 * 1. get the finished command id and tile id from HW RGs
 * 2. check whether this batch completed
 *
 * Return value:
 * 1. return CE_DONE if all the batches in the CE completed.
 * 2. return CE_RUN if a batch completed, and we can go on to issue the next.
 * 3. return CE_NONE if the batch is still under processing.
 *
 * NOTE: sched->lock should be acquired by caller
 */
unsigned int mdla_process_ce_2_1(unsigned int core_id)
{
	unsigned long flags;
	unsigned int ret = CE_NONE;
	struct command_entry *ce;
	struct cmd_tmp *cb;
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);
	u32 fin_cid;

	if (unlikely(sched == NULL))
		return ret;

	spin_lock_irqsave(&mdla_devices[core_id].hw_lock, flags);
	fin_cid = mdla_reg_read_with_mdlaid(core_id, MREG_TOP_G_FIN3);
	spin_unlock_irqrestore(&mdla_devices[core_id].hw_lock, flags);

	ce = sched->pro_ce;
	if (unlikely(ce == NULL))
		return ret;

	ce->fin_cid = fin_cid;
	if (unlikely(fin_cid < ce->wish_fin_cid)) {
		ce->irq_state |= IRQ_TWICE;
		ce_func_trace(ce, F_CMDDONE_CE_FIN3ERROR);
		return ret;
	}

	/* clear event id after this event is done */
	spin_lock_irqsave(&mdla_devices[core_id].hw_lock, flags);
#ifdef __APUSYS_MDLA_PMU_SUPPORT__
	/* handle PMU */
	pmu_reg_save(core_id, (u16)sched->pro_ce->priority);
#endif
	mdla_reg_write_with_mdlaid(core_id, 1, MREG_TOP_G_CDMA4);
	spin_unlock_irqrestore(&mdla_devices[core_id].hw_lock, flags);

	if (ce->cmd_list_head != NULL) {
		if (likely(!list_empty(ce->cmd_list_head))) {
			cb = list_first_entry(ce->cmd_list_head,
						struct cmd_tmp, node);
			list_del(&cb->node);
			kfree(cb);
		}
	}

	/* all command done for command-based scheduling */
	if (fin_cid >= ce->count) {
		ret = CE_DONE;
		ce->state |= (1 << CE_DONE);
	} else {
		ret = CE_SCHED;
		ce->state |= (1 << CE_SCHED);
	}
	return ret;
}

/*
 * Issue the processing_ce to HW engine
 * NOTE: sched->lock should be acquired by caller
 */
void mdla_issue_ce_2_1(unsigned int core_id)
{
	dma_addr_t addr;
	u32 nr_cmd_to_issue;
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);
	struct command_entry *ce;
	struct cmd_tmp *cb;
	unsigned long flags;
	u32 irq_status;

	if (unlikely(sched == NULL))
		return;

	ce = sched->pro_ce;
	if (unlikely(ce == NULL))
		return;

	if (ce->poweron_t == 0) {
		ce->poweron_t = sched_clock();
		ce->req_start_t = ce->poweron_t;
	}

	addr = ce->mva + ((dma_addr_t)ce->fin_cid) * MREG_CMD_SIZE;
	nr_cmd_to_issue = ce->count - ce->fin_cid;
	if (ce->cmd_list_head != NULL) {
		if (!list_empty(ce->cmd_list_head)) {
			cb = list_first_entry(ce->cmd_list_head,
						struct cmd_tmp, node);
			nr_cmd_to_issue = cb->size;
		}
	}
	ce->wish_fin_cid = ce->fin_cid + nr_cmd_to_issue;

	spin_lock_irqsave(&mdla_devices[core_id].hw_lock, flags);

	irq_status =
		mdla_reg_read_with_mdlaid(core_id, MREG_TOP_G_INTP0);
	if (likely(irq_status&MDLA_IRQ_CDMA_FIFO_EMPTY)) {
		uint64_t deadline =
			get_jiffies_64() + msecs_to_jiffies(mdla_timeout);

#ifdef __APUSYS_MDLA_PMU_SUPPORT__
		/* reset pmu and set register */
		pmu_set_reg(core_id, (u16)ce->priority);
#endif
		ce->state |= (1 << CE_RUN);
		/* update deadline value */
		ce->deadline_t = deadline;

		if (likely(ce->context_callback != NULL))
			ce->context_callback(
				APUSYS_DEVICE_MDLA,
				core_id, ce->ctx_id);

		/* set command address */
		mdla_reg_write_with_mdlaid(core_id,
			addr, MREG_TOP_G_CDMA1);

		/* set command number */
		mdla_reg_write_with_mdlaid(core_id,
			nr_cmd_to_issue, MREG_TOP_G_CDMA2);

		/* trigger engine */
		mdla_reg_write_with_mdlaid(
			core_id, ce->csn, MREG_TOP_G_CDMA3);

		if (unlikely(mdla_timeout_dbg)) {
			mdla_issue_ce_cdma_error(
				core_id,
				ce, addr,
				nr_cmd_to_issue);
		} else
			ce_func_trace(ce, F_ISSUE);
	} else {
		if ((ce->irq_state & IRQ_N_EMPTY_IN_SCHED) == 0)
			ce->irq_state |= IRQ_NE_ISSUE_FIRST;
		ce->irq_state |= IRQ_N_EMPTY_IN_ISSUE;
		ce->state |= (1 << CE_SKIP);
		if (in_interrupt()) {
			ce->irq_state |= IRQ_IN_IRQ;
			ce_func_trace(ce, F_ISSUE|3);
		} else {
			ce->irq_state |= IRQ_NOT_IN_IRQ;
			ce_func_trace(ce, F_ISSUE|4);
		}
	}
	spin_unlock_irqrestore(&mdla_devices[core_id].hw_lock, flags);
}

/*
 * Set the status of completed CE as CE_FIN
 * NOTE: sched->lock should be acquired by caller
 */
void mdla_complete_ce_2_1(unsigned int core_id)
{
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);

	if (unlikely(sched == NULL))
		return;

	sched->pro_ce->req_end_t = sched_clock();
	sched->pro_ce->state |= (1 << CE_FIN);

	ce_func_trace(sched->pro_ce, F_COMPLETE);

	complete(&sched->pro_ce->swcmd_done_wait);
	sched->pro_ce = NULL;
}

void mdla_normal_dual_cmd_issue(
	uint32_t core_id,
	uint64_t dual_cmd_id)
{
	/* This platform don't support SMP cmd */
	mdla_cmd_debug("This platform doesn't support dual cmd\n");
}

/*
 * Scheduler to process the current CE, and pick the next one to issue
 * NOTE: mdla_scheduler would be invoked in ISR
 */
irqreturn_t mdla_scheduler_2_1(unsigned int core_id)
{
	struct mdla_scheduler *sched = mdla_get_scheduler(core_id);
	unsigned long flags;
	unsigned int status;
	struct mdla_dev *mdla_info = &mdla_devices[core_id];
	u32 irq_status = 0;
	u32 cdma4 = 0;
	struct command_entry *new_ce;

	/* clear intp0 to avoid irq fire twice */
	spin_lock_irqsave(&mdla_info->hw_lock, flags);
	irq_status =
		mdla_reg_read_with_mdlaid(core_id, MREG_TOP_G_INTP0);
	mdla_reg_write_with_mdlaid(
		core_id,
		MDLA_IRQ_SWCMD_DONE | MDLA_IRQ_PMU_INTE,
		MREG_TOP_G_INTP0);
	cdma4 = mdla_reg_read_with_mdlaid(core_id, MREG_TOP_G_CDMA4);
	spin_unlock_irqrestore(&mdla_info->hw_lock, flags);

	/* interrupt error check start */
	if (unlikely(sched == NULL)) {
		mdla_info->error_bit |= IRQ_NO_SCHEDULER;
		goto end;
	}

	spin_lock_irqsave(&sched->lock, flags);
	if (unlikely(sched->pro_ce == NULL)) {
		mdla_info->error_bit |= IRQ_NO_PROCESSING_CE;
		goto unlock;
	}

	if (unlikely(sched->pro_ce->state & (1 << CE_FAIL))) {
		mdla_info->error_bit |= IRQ_TIMEOUT;
		ce_func_trace(sched->pro_ce, F_TIMEOUT|1);
		goto unlock;
	}

	if (unlikely(time_after64(
		get_jiffies_64(),
		sched->pro_ce->deadline_t)
		)) {
		sched->pro_ce->state |= (1 << CE_TIMEOUT);
		ce_func_trace(sched->pro_ce, F_TIMEOUT);
		goto unlock;
	}
	if (unlikely((irq_status&MDLA_IRQ_SWCMD_DONE) == 0)) {
		ce_func_trace(sched->pro_ce, F_INIRQ_ERROR);
		goto unlock;
	}

	if (unlikely(cdma4 != (sched->pro_ce->csn))) {
		sched->pro_ce->irq_state |= IRQ_RECORD_ERROR;
		ce_func_trace(sched->pro_ce, F_INIRQ_CDMA4ERROR);
		goto unlock;
	}
	/* interrupt error check end */

	sched->pro_ce->state |= (1 << CE_SCHED);

	/* process the current CE */
	status = sched->process_ce(core_id);

	if (status == CE_DONE) {
		sched->complete_ce(core_id);
	} else if (status == CE_NONE) {
		/* nothing to do but wait for the engine completed */
		goto unlock;
	}
	/* get the next CE to be processed */
	new_ce = sched->dequeue_ce(core_id);

	if (new_ce != NULL) {
		if (sched->pro_ce != NULL) {
			sched->preempt_ce(core_id, new_ce);
			sched->issue_ce(core_id);
		} else {
			sched->pro_ce = new_ce;
			sched->issue_ce(core_id);
		}
	} else {
		if (sched->pro_ce != NULL) {
			/* Issue next cmd batch */
			sched->issue_ce(core_id);
		}
	}

unlock:
	spin_unlock_irqrestore(&sched->lock, flags);
end:
	return IRQ_HANDLED;
}

void dump_timeout_debug_info(int core_id)
{
	int i;

	for (i = 0x0000; i < 0x1000; i += 4)
		mdla_timeout_all_debug("apu_mdla_config_top+%04X: %08X\n",
				i, mdla_cfg_read_with_mdlaid(core_id, i));
	for (i = 0x0000; i < 0x1000; i += 4)
		mdla_timeout_debug("apu_mdla_cmde_mreg_top+%04X: %08X\n",
				i, mdla_reg_read_with_mdlaid(core_id, i));

}

void mdla_dump_reg(int core_id)
{
	mdla_timeout_debug("mdla_timeout\n");
	// TODO: too many registers, dump only debug required ones.
	dump_reg_cfg(core_id, MDLA_CG_CON);
	dump_reg_cfg(core_id, MDLA_SW_RST);
	dump_reg_cfg(core_id, MDLA_MBIST_MODE0);
	dump_reg_cfg(core_id, MDLA_MBIST_MODE1);
	dump_reg_cfg(core_id, MDLA_MBIST_CTL);
	dump_reg_cfg(core_id, MDLA_MBIST_DEFAULT_DELSEL);
	dump_reg_cfg(core_id, MDLA_RP_RST);
	dump_reg_cfg(core_id, MDLA_RP_CON);
	dump_reg_cfg(core_id, MDLA_AXI_CTRL);
	dump_reg_cfg(core_id, MDLA_AXI1_CTRL);

	dump_reg_top(core_id, MREG_TOP_G_REV);
	dump_reg_top(core_id, MREG_TOP_G_INTP0);
	dump_reg_top(core_id, MREG_TOP_G_INTP1);
	dump_reg_top(core_id, MREG_TOP_G_INTP2);
	dump_reg_top(core_id, MREG_TOP_G_CDMA0);
	dump_reg_top(core_id, MREG_TOP_G_CDMA1);
	dump_reg_top(core_id, MREG_TOP_G_CDMA2);
	dump_reg_top(core_id, MREG_TOP_G_CDMA3);
	dump_reg_top(core_id, MREG_TOP_G_CDMA4);
	dump_reg_top(core_id, MREG_TOP_G_CDMA5);
	dump_reg_top(core_id, MREG_TOP_G_CDMA6);
	dump_reg_top(core_id, MREG_TOP_G_CUR0);
	dump_reg_top(core_id, MREG_TOP_G_CUR1);
	dump_reg_top(core_id, MREG_TOP_G_FIN0);
	dump_reg_top(core_id, MREG_TOP_G_FIN1);
	dump_reg_top(core_id, MREG_TOP_G_FIN3);
	dump_reg_top(core_id, MREG_TOP_G_IDLE);

	/* for DCM and CG */
	dump_reg_top(core_id, MREG_TOP_ENG0);
	dump_reg_top(core_id, MREG_TOP_ENG1);
	dump_reg_top(core_id, MREG_TOP_ENG2);
	dump_reg_top(core_id, MREG_TOP_ENG11);
	dump_timeout_debug_info(core_id);
}
