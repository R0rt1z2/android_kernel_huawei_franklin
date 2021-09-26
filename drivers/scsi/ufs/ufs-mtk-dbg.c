/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mt-plat/mtk_gpt.h>
#include <linux/io.h>
#include <linux/scatterlist.h>

#include "ufs-mtk-dbg.h"
#include "ufs.h"
#include <mt-plat/mtk_boot.h>


#ifndef FPGA_PLATFORM
#ifdef UPMU_READY
#include <mt-plat/upmu_common.h>
#endif
#endif

#define MAX_NAME_LEN 32
#ifdef CONFIG_MTK_UFS_DEBUG
#define MAX_UFS_CMD_HLIST_ENTRY_CNT (500)
/* max dump size is 40KB whitch can be adjusted */
#define UFS_AEE_BUFFER_SIZE (100 * 1024)

struct ufs_cmd_hlist_struct ufs_cmd_hlist[MAX_UFS_CMD_HLIST_ENTRY_CNT];
int ufs_cmd_ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
int ufs_cmd_cnt;
static spinlock_t ufs_mtk_cmd_dump_lock;
static int ufs_mtk_is_cmd_dump_lock_init;
static atomic_t cmd_hist_enabled;
char ufs_aee_buffer[UFS_AEE_BUFFER_SIZE];

#ifndef CONFIG_MAS_BOOTDEVICE
struct __bootdevice {
	char product_name[MAX_NAME_LEN + 1];
	sector_t size;
	unsigned int manfid;
};

static struct __bootdevice bootdevice;
#endif

static void ufs_mtk_dbg_dump_feature(struct ufs_hba *hba, struct seq_file *m)
{
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "-- Crypto Features ----\n");
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "Features          = 0x%x\n",
		hba->crypto_feature);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FDE           = 0x%x\n",
		UFS_CRYPTO_HW_FDE);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FDE Encrypted = 0x%x\n",
		UFS_CRYPTO_HW_FDE_ENCRYPTED);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FBE           = 0x%x\n",
		UFS_CRYPTO_HW_FBE);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, " HW-FBE Encrypted = 0x%x\n",
		UFS_CRYPTO_HW_FBE_ENCRYPTED);
	UFS_DEVINFO_PROC_MSG(m, hba->dev, "-----------------------\n");
}
#endif


void ufs_mtk_dbg_stop_trace(struct ufs_hba *hba)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	atomic_set(&cmd_hist_enabled, 0);
	pr_info("ufsdbg: cmd history off\n");
#endif
}

void ufs_mtk_dbg_start_trace(struct ufs_hba *hba)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	atomic_set(&cmd_hist_enabled, 1);
	pr_info("ufsdbg: cmd history on\n");
#endif
}

void ufs_mtk_dbg_add_trace(struct ufs_hba *hba,
	enum ufs_trace_event event, u32 tag,
	u8 lun, u32 transfer_len, sector_t lba, u8 opcode,
	unsigned long long ppn, u32 region, u32 subregion, u32 resv)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int ptr;
	unsigned long flags;

	if (!ufs_mtk_is_cmd_dump_lock_init) {
		spin_lock_init(&ufs_mtk_cmd_dump_lock);
		ufs_mtk_is_cmd_dump_lock_init = 1;

		atomic_set(&cmd_hist_enabled, 1);
	}

	if (!atomic_read(&cmd_hist_enabled)
		&& event != UFS_TRACE_DEBUG_PROC)
		return;

	spin_lock_irqsave(&ufs_mtk_cmd_dump_lock, flags);

	ufs_cmd_ptr++;

	if (ufs_cmd_ptr >= MAX_UFS_CMD_HLIST_ENTRY_CNT)
		ufs_cmd_ptr = 0;

	ptr = ufs_cmd_ptr;

	ufs_cmd_hlist[ptr].pid = current->pid;
	ufs_cmd_hlist[ptr].event = event;
	ufs_cmd_hlist[ptr].tag = tag;
	ufs_cmd_hlist[ptr].transfer_len = transfer_len;
	ufs_cmd_hlist[ptr].lun = lun;
	ufs_cmd_hlist[ptr].lba = lba;
	ufs_cmd_hlist[ptr].opcode = opcode;
	ufs_cmd_hlist[ptr].time = sched_clock();
	ufs_cmd_hlist[ptr].duration = 0;
	ufs_cmd_hlist[ptr].rq = NULL;
	ufs_cmd_hlist[ptr].cpu = smp_processor_id();
#if defined(CONFIG_UFSHPB)
	ufs_cmd_hlist[ptr].ppn = ppn;
	ufs_cmd_hlist[ptr].region = region;
	ufs_cmd_hlist[ptr].subregion = subregion;
	ufs_cmd_hlist[ptr].resv = resv;
#endif

	/* keep request pointer to dig out block layer status */
	if ((event == UFS_TRACE_SEND) || (event == UFS_TRACE_COMPLETED)) {
		if (hba->lrb[tag].cmd && hba->lrb[tag].cmd->request) {
			ufs_cmd_hlist[ptr].rq =
				hba->lrb[tag].cmd->request;
			ufs_cmd_hlist[ptr].crypted =
				hba->lrb[tag].crypto_en;
			ufs_cmd_hlist[ptr].keyslot =
				ufs_cmd_hlist[ptr].crypted ?
				hba->lrb[tag].crypto_cfgid : 0;
		}
	}

	if (event == UFS_TRACE_COMPLETED) {
		if (ufs_cmd_ptr == 0)
			ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
		else
			ptr = ufs_cmd_ptr - 1;

		while (1) {
			if (ufs_cmd_hlist[ptr].tag == tag) {
				ufs_cmd_hlist[ufs_cmd_ptr].duration =
					sched_clock() - ufs_cmd_hlist[ptr].time;
				break;
			}

			ptr--;
			if (ptr < 0)
				ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;


			if (ufs_cmd_ptr == 0) {
				if (ptr == (MAX_UFS_CMD_HLIST_ENTRY_CNT - 1))
					break;
			} else {
				if (ptr == ufs_cmd_ptr - 1)
					break;
			}
		}

		/* Over 1 second, record performance warning */
		if (ufs_cmd_hlist[ufs_cmd_ptr].duration >= 1000000000) {
			/*
			 * op code[31:24]
			 * length(4KB)[23:16]
			 * duration(ms)[15:0]
			 */
			ufshcd_update_reg_hist(&hba->ufs_stats.perf_warn,
			    (u32) ((opcode << 24) |
			    (((transfer_len >> 12) & 0xFF) << 16) |
			    (ufs_cmd_hlist[ufs_cmd_ptr].duration / 1000000)));
		}
	}

	ufs_cmd_cnt++;

	spin_unlock_irqrestore(&ufs_mtk_cmd_dump_lock, flags);
#endif
}

void ufs_mtk_dbg_dump_trace(char **buff, unsigned long *size,
	u32 latest_cnt, struct seq_file *m)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int ptr;
	int dump_cnt;
	unsigned long flags;

	if (!ufs_mtk_is_cmd_dump_lock_init) {
		spin_lock_init(&ufs_mtk_cmd_dump_lock);
		ufs_mtk_is_cmd_dump_lock_init = 1;
	}

	spin_lock_irqsave(&ufs_mtk_cmd_dump_lock, flags);

	if (ufs_cmd_cnt > MAX_UFS_CMD_HLIST_ENTRY_CNT)
		dump_cnt = MAX_UFS_CMD_HLIST_ENTRY_CNT;
	else
		dump_cnt = ufs_cmd_cnt;

	if (latest_cnt)
		dump_cnt = min_t(u32, latest_cnt, dump_cnt);

	ptr = ufs_cmd_ptr;

	SPREAD_PRINTF(buff, size, m,
		"[ufs] CMD History: req_cnt=%d, real_cnt=%d, ptr=%d\n",
		latest_cnt, dump_cnt, ptr);

	while (dump_cnt > 0) {

		if ((ufs_cmd_hlist[ptr].event >= UFS_TRACE_UIC_SEND) &&
			(ufs_cmd_hlist[ptr].event <=
			UFS_TRACE_UIC_CMPL_PWR_CTRL)) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-u(%d),%5d,%2d,0x%2X,arg1=0x%X,arg2=0x%X,arg3=0x%X,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,       /* command */
				ufs_cmd_hlist[ptr].tag,          /* argument1 */
				ufs_cmd_hlist[ptr].transfer_len, /* argument2 */
				(u32)ufs_cmd_hlist[ptr].lba,     /* argument3 */
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_REG_TOGGLE) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-g(%d),%5d,state=%d,on=%d,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].tag,          /* state */
				ufs_cmd_hlist[ptr].transfer_len, /* on or off */
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_TM_SEND ||
			ufs_cmd_hlist[ptr].event == UFS_TRACE_TM_COMPLETED) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-t(%d),%5d,%2d,tm=0x%x,t=%2d,lun=0x%x,data=0x%x,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				ufs_cmd_hlist[ptr].transfer_len,
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_DEV_SEND ||
			ufs_cmd_hlist[ptr].event == UFS_TRACE_DEV_COMPLETED) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-d(%d),%5d,%2d,0x%2x,t=%2d,lun=0x%x,idn=0x%x,idx=0x%x,sel=0x%x,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				(u8)ufs_cmd_hlist[ptr].lba & 0xFF,
				(u8)(ufs_cmd_hlist[ptr].lba >> 8) & 0xFF,
				(u8)(ufs_cmd_hlist[ptr].lba >> 16) & 0xFF,
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else if (ufs_cmd_hlist[ptr].event == UFS_TRACE_GENERIC) {

			SPREAD_PRINTF(buff, size, m,
				"%3d-u(%d),%5d,%2d,G,arg1=0x%X,arg2=%d,arg3=%d,%llu\n",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].tag,          /* argument1 */
				ufs_cmd_hlist[ptr].transfer_len, /* argument2 */
				(u32)ufs_cmd_hlist[ptr].lba,     /* argument3 */
				(u64)ufs_cmd_hlist[ptr].time
				);

		} else {
			SPREAD_PRINTF(buff, size, m,
				"%3d-r(%d),%5d,%2d,0x%2x,t=%2d,lun=0x%x,crypt:%d,%d,lba=0x%llx,len=%6d,%llu,\t%llu",
				ptr,
				ufs_cmd_hlist[ptr].cpu,
				ufs_cmd_hlist[ptr].pid,
				ufs_cmd_hlist[ptr].event,
				ufs_cmd_hlist[ptr].opcode,
				ufs_cmd_hlist[ptr].tag,
				ufs_cmd_hlist[ptr].lun,
				ufs_cmd_hlist[ptr].crypted,
				ufs_cmd_hlist[ptr].keyslot,
				(long long int)ufs_cmd_hlist[ptr].lba,
				ufs_cmd_hlist[ptr].transfer_len,
				(u64)ufs_cmd_hlist[ptr].time,
				(u64)ufs_cmd_hlist[ptr].duration
				);
#if defined(CONFIG_UFSHPB)
			if (ufs_cmd_hlist[ptr].opcode == READ_16) {
				SPREAD_PRINTF(buff, size, m,
					",\tppn=0x%llx,\tregion=0x%x,\tsubregion=0x%x,\tresv=0x%x",
					ufs_cmd_hlist[ptr].ppn,
					ufs_cmd_hlist[ptr].region,
					ufs_cmd_hlist[ptr].subregion,
					ufs_cmd_hlist[ptr].resv
					);
			}
			if (ufs_cmd_hlist[ptr].opcode == UFSHPB_WRITE_BUFFER) {
				SPREAD_PRINTF(buff, size, m,
					",\tlba=0x%llx,\tbuf_len=0x%x",
					ufs_cmd_hlist[ptr].ppn,
					ufs_cmd_hlist[ptr].region
					);
			}
#endif
			SPREAD_PRINTF(buff, size, m, "\n");
		}

		dump_cnt--;

		ptr--;

		if (ptr < 0)
			ptr = MAX_UFS_CMD_HLIST_ENTRY_CNT - 1;
	}

	spin_unlock_irqrestore(&ufs_mtk_cmd_dump_lock, flags);
#endif
}

void ufs_mtk_dme_cmd_log(struct ufs_hba *hba, struct uic_command *ucmd,
	enum ufs_trace_event event)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	u32 cmd;

	if (event == UFS_TRACE_UIC_SEND)
		cmd = ucmd->command;
	else
		cmd = ufshcd_readl(hba, REG_UIC_COMMAND);

	ufs_mtk_dbg_add_trace(hba, event,
		ufshcd_readl(hba, REG_UIC_COMMAND_ARG_1),
		0,
		ufshcd_readl(hba, REG_UIC_COMMAND_ARG_2),
		ufshcd_readl(hba, REG_UIC_COMMAND_ARG_3),
		cmd, 0, 0, 0, 0);
#endif
}

void ufs_mtk_dbg_hang_detect_dump(void)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int boot_type;

	boot_type = get_boot_type();
	if (boot_type != BOOTDEV_UFS)
		return;

	/*
	 * do not touch host to get unipro or mphy information via
	 * dme commands during exception handling since interrupt
	 * or preemption may be disabled.
	 */
	ufshcd_print_host_state(ufs_mtk_hba, 0, NULL, NULL, NULL);

	ufs_mtk_dbg_dump_trace(NULL, NULL,
		ufs_mtk_hba->nutrs + ufs_mtk_hba->nutrs / 2, NULL);

	ufshcd_print_all_err_hist(ufs_mtk_hba, NULL, NULL, NULL);
#endif
}



void ufs_mtk_dbg_proc_dump(struct seq_file *m)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	ufs_mtk_dbg_dump_feature(ufs_mtk_hba, m);

	/*
	 * do not touch host to get unipro or mphy information via
	 * dme commands during exception handling since interrupt
	 * or preemption may be disabled.
	 */
	ufshcd_print_host_state(ufs_mtk_hba, 0, m, NULL, NULL);

	ufs_mtk_dbg_dump_trace(NULL, NULL,
		MAX_UFS_CMD_HLIST_ENTRY_CNT, m);

	ufshcd_print_all_err_hist(ufs_mtk_hba, m, NULL, NULL);
#endif
}

void get_ufs_aee_buffer(unsigned long *vaddr, unsigned long *size)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	unsigned long free_size = UFS_AEE_BUFFER_SIZE;
	char *buff;

	if (ufs_mtk_hba == NULL) {
		pr_info("====== Null ufs_mtk_hba, dump skipped ======\n");
		return;
	}

	buff = ufs_aee_buffer;
	ufshcd_print_host_state(ufs_mtk_hba, 0, NULL, &buff, &free_size);
	ufs_mtk_dbg_dump_trace(&buff, &free_size,
		MAX_UFS_CMD_HLIST_ENTRY_CNT, NULL);

	ufshcd_print_all_err_hist(ufs_mtk_hba,
		NULL, &buff, &free_size);

	/* retrun start location */
	*vaddr = (unsigned long)ufs_aee_buffer;
	*size = UFS_AEE_BUFFER_SIZE - free_size;
#endif
}
EXPORT_SYMBOL(get_ufs_aee_buffer);

static int ufsdbg_dump_health_desc(struct seq_file *file)
{
#ifdef CONFIG_MTK_UFS_DEBUG
	int err = 0;
	int buff_len = QUERY_DESC_HEALTH_MAX_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_MAX_SIZE];
	int i;

	pm_runtime_get_sync(ufs_mtk_hba->dev);
	err = ufshcd_read_health_desc(ufs_mtk_hba, desc_buf, buff_len);
	pm_runtime_put_sync(ufs_mtk_hba->dev);


	if (err) {
		seq_printf(file, "Reading Health Descriptor failed. err = %d\n",
			err);
		goto out;
	}

	for (i = 0; i < QUERY_DESC_HEALTH_MAX_SIZE; i++) {
		seq_printf(file,
			"Health Descriptor[0x%x] = 0x%x\n",
			i,
			(u8)desc_buf[i]);
	}

	seq_printf(file,
		"Health Descriptor[offset 0x02]: %s = 0x%x\n",
		"bPreEOLInfo",
		(u8)desc_buf[2]);

	seq_printf(file,
		"Health Descriptor[offset 0x03]: %s = 0x%x\n",
		"bDeviceLifeTimeEstA",
		(u8)desc_buf[3]);

	seq_printf(file,
		"Health Descriptor[offset 0x04]: %s = 0x%x\n",
		"bDeviceLifeTimeEstB",
		(u8)desc_buf[4]);

out:
	return err;
#else
	return 0;
#endif
}

static char cmd_buf[256];

static int ufs_help_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "\n===============[ufs_help]================\n");

	seq_printf(m, "\n   Commands dump: echo %x [host_id] > ufs_debug\n",
		UFS_CMDS_DUMP);

	seq_printf(m, "\n   Get Power Mode Status: echo %x [host_id] > ufs_debug\n",
		UFS_GET_PWR_MODE);

	seq_printf(m, "\n   Dump Health Descriptors: echo %x [host_id] > ufs_debug\n",
		UFS_DUMP_HEALTH_DESCRIPTOR);

	seq_puts(m, "\n   NOTE: All input data is Hex number!\n");

	seq_puts(m, "\n=============================================\n\n");

	return 0;
}

/* ========== driver proc interface =========== */
static int ufs_debug_proc_show(struct seq_file *m, void *v)
{
	unsigned long cmd;

	seq_printf(m, "ufsdbg: debug command: %s\n", cmd_buf);

	if (kstrtoul(cmd_buf, 10, &cmd))
		cmd = UFS_CMDS_DUMP;

	cmd_buf[0] = '\0';

	if (cmd == UFS_CMDS_DUMP) {
		/*
		 * Default print cmd history for aee:
		 * JE/NE/ANR/EE/SWT/system api dump
		 */
		seq_puts(m, "==== UFS Debug Info ====\n");
		ufs_mtk_dbg_proc_dump(m);
	} else if (cmd == UFS_GET_PWR_MODE) {
		seq_puts(m, "(1:FAST 2:SLOW 4:FAST_AUTO 5:SLOW_AUTO 7:UNCHANGE)\n");
		seq_printf(m, "Power Mode: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.pwr_tx,
			ufs_mtk_hba->pwr_info.pwr_rx);
		seq_printf(m, "Gear: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.gear_tx,
			ufs_mtk_hba->pwr_info.gear_rx);
		seq_printf(m, "HS Rate: 0x%x (1:HS_A 2:HS_B)\n",
			ufs_mtk_hba->pwr_info.hs_rate);
		seq_printf(m, "Lanes: tx 0x%x rx 0x%x\n",
			ufs_mtk_hba->pwr_info.lane_tx,
			ufs_mtk_hba->pwr_info.lane_rx);
	} else if (cmd == UFS_DUMP_HEALTH_DESCRIPTOR) {
		ufsdbg_dump_health_desc(m);
	}

	return 0;
}

static ssize_t ufs_debug_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *data)
{
	unsigned long op = UFS_CMD_UNKNOWN;
	bool handled = false;

	if (count == 0 || count > 255)
		return -EINVAL;

	if (copy_from_user(cmd_buf, buf, count))
		return -EINVAL;

	/*
	 * Let's handle simple commands here.
	 * Simple command can be executed immediately
	 * after command is written and do not need further
	 * "read" or "cat" anymore.
	 */
	cmd_buf[count] = '\0';
	if (kstrtoul(cmd_buf, 10, &op))
		return -EINVAL;

	if (op == UFS_CMD_HIST_BEGIN) {
		atomic_set(&cmd_hist_enabled, 1);
		pr_info("ufsdbg: cmd history on\n");
		handled = true;
	} else if (op == UFS_CMD_HIST_STOP) {
		atomic_set(&cmd_hist_enabled, 0);
		pr_info("ufsdbg: cmd history off\n");
		handled = true;
	}

	if (ufs_mtk_hba)
		ufs_mtk_dbg_add_trace(ufs_mtk_hba,
			UFS_TRACE_DEBUG_PROC,
			op, 0, atomic_read(&cmd_hist_enabled),
			0, 0, 0, 0, 0, 0);

	if (handled)
		cmd_buf[0] = '\0';

	return count;
}

static int ufs_perf_proc_show(struct seq_file *m, void *v)
{
	struct ufs_mtk_host *host = m->private;

	seq_puts(m, "UFS Performance Mode\n");
	seq_printf(m, "  - supported: %d\n", ufs_mtk_perf_is_supported(host));
	seq_printf(m, "  - enabled: %d\n", host->perf_en);
	seq_printf(m, "  - mode: %d\n", host->perf_mode);
	seq_printf(m, "  - crypto_vcore_opp: %d\n", host->crypto_vcore_opp);

	return 0;
}

static ssize_t ufs_perf_proc_write(struct file *file, const char *ubuf,
				   size_t count, loff_t *data)
{
	struct ufs_mtk_host *host = PDE_DATA(file->f_mapping->host);
	unsigned long op = UFS_CMD_UNKNOWN;
	char cmd[16] = {0};
	loff_t buff_pos = 0;
	int ret = 0, last_mode;

	ret = simple_write_to_buffer(cmd, 15, &buff_pos, ubuf, count);
	if (ret < 0) {
		dev_info(host->hba->dev, "%s: failed to read user data\n",
			__func__);
		return -EINVAL;
	}

	cmd[ret] = '\0';
	if (kstrtoul(cmd, 10, &op))
		return -EINVAL;

	last_mode = host->perf_mode;

	if (op == PERF_AUTO) {
		host->perf_mode = PERF_AUTO;
		ret = 0;
	} else if (op == PERF_FORCE_ENABLE &&
			host->perf_mode != PERF_FORCE_ENABLE) {
		host->perf_mode = PERF_FORCE_ENABLE;
		ret = ufs_mtk_perf_setup_crypto_clk(host, true);
	} else if (op == PERF_FORCE_DISABLE &&
			host->perf_mode != PERF_FORCE_DISABLE) {
		host->perf_mode = PERF_FORCE_DISABLE;
		ret = ufs_mtk_perf_setup_crypto_clk(host, false);
	} else
		return -EINVAL;

	if (ret)
		host->perf_mode = last_mode;

	return count;
}


static int ufs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_debug_proc_show, inode->i_private);
}

static const struct file_operations ufs_proc_fops = {
	.open = ufs_proc_open,
	.write = ufs_debug_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ufs_help_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_help_proc_show, inode->i_private);
}

static const struct file_operations ufs_help_fops = {
	.open = ufs_help_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ufs_perf_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_perf_proc_show, PDE_DATA(inode));
}

static const struct file_operations ufs_perf_fops = {
	.open = ufs_perf_proc_open,
	.write = ufs_perf_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifndef USER_BUILD_KERNEL
#define PROC_PERM		0660
#else
#define PROC_PERM		0440
#endif

#define UFS_PROC_SHOW(name, fmt, args...) \
static int ufs_##name##_show(struct seq_file *m, void *v) \
{ \
	if (ufs_mtk_hba) \
		seq_printf(m, fmt, args); \
	return 0; \
} \
static int ufs_##name##_open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, ufs_##name##_show, inode->i_private); \
} \
static const struct file_operations name##_fops = { \
	.open = ufs_##name##_open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

#ifndef CONFIG_MAS_BOOTDEVICE
/*These two TYPES are requested by HUIWEI normalized project*/
#define  EMMC_TYPE      0
#define  UFS_TYPE       1
#define BOOTTYPE UFS_TYPE

static int ufs_cid_show(struct seq_file *m, void *v)
{
	u32 cid[4];
	int i;

	if (ufs_mtk_hba) {
		memcpy(cid, (u32 *)&ufs_mtk_hba->unique_number, sizeof(cid));
		for (i = 0; i < 3; i++)
			cid[i] = be32_to_cpu(cid[i]);
		cid[3] = (((cid[3]) & 0xffff) << 16) | (((cid[3]) >> 16) & 0xffff);
		seq_printf(m, "%08x%08x%08x%08x\n", cid[0], cid[1], cid[2], cid[3]);
	}
	return 0;
}
static int ufs_cid_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_cid_show, inode->i_private);
}
static const struct file_operations cid_fops = {
	.open = ufs_cid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void set_bootdevice_manfid(unsigned int manfid)
{
 	bootdevice.manfid = manfid;
}

unsigned int get_bootdevice_manfid(void)
{
 	return bootdevice.manfid;
}

static int ufs_manfid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%06x\n", bootdevice.manfid);
	return 0;
}

static int ufs_manfid_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_manfid_show, inode->i_private);
}

static const struct file_operations manfid_fops = {
	.open		= ufs_manfid_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void set_bootdevice_size(sector_t size)
{
 	bootdevice.size = size;
}

sector_t get_bootdevice_size()
{
 	return bootdevice.size;
}

static int ufs_size_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%llu\n", (unsigned long long)bootdevice.size);
	return 0;
}

static int ufs_size_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_size_show, inode->i_private);
}

static const struct file_operations size_fops = {
	.open		= ufs_size_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ufs_name_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", "mtk-platform");
	return 0;
}

static int ufs_name_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_name_show, inode->i_private);
}

static const struct file_operations name_fops = {
	.open = ufs_name_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void set_bootdevice_product_name(char *product_name)
{
 	strlcpy(bootdevice.product_name,
 		product_name,
 		sizeof(bootdevice.product_name));
}

void get_bootdevice_product_name(char* product_name, u32 len)
{
 	strlcpy(product_name, bootdevice.product_name, len); /* [false alarm] */
}

static int ufs_product_name_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", bootdevice.product_name);
	return 0;
}

static int ufs_product_name_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_product_name_show, inode->i_private);
}

static const struct file_operations product_name_fops = {
	.open		= ufs_product_name_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ufs_fw_version_show(struct seq_file *m, void *v)
{
	if (ufs_mtk_hba && ufs_mtk_hba->card)
		seq_printf(m, "%s\n", ufs_mtk_hba->card->prl);

	return 0;
}
static int ufs_fw_version_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_fw_version_show, inode->i_private);
}
static const struct file_operations fw_version_fops = {
	.open = ufs_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int ufs_pre_eol_info_show(struct seq_file *m, void *v)
{
	int err;
	int buff_len = QUERY_DESC_HEALTH_MAX_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_MAX_SIZE];

	if (ufs_mtk_hba) {
		pm_runtime_get_sync(ufs_mtk_hba->dev);
		err = ufshcd_read_health_desc(ufs_mtk_hba, desc_buf, buff_len);
		pm_runtime_put_sync(ufs_mtk_hba->dev);
		if (err) {
			seq_printf(m, "Reading Health Descriptor failed. err = %d\n",
				err);
			return err;
		}
		seq_printf(m, "0x%02x\n", (u8)desc_buf[2]);
	}
	return 0;
}
static int ufs_pre_eol_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_pre_eol_info_show, inode->i_private);
}
static const struct file_operations pre_eol_info_fops = {
	.open = ufs_pre_eol_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ufs_life_time_est_typ_a_show(struct seq_file *m, void *v)
{
	int err;
	int buff_len = QUERY_DESC_HEALTH_MAX_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_MAX_SIZE];

	if (ufs_mtk_hba) {
		pm_runtime_get_sync(ufs_mtk_hba->dev);
		err = ufshcd_read_health_desc(ufs_mtk_hba, desc_buf, buff_len);
		pm_runtime_put_sync(ufs_mtk_hba->dev);
		if (err) {
			seq_printf(m, "Reading Health Descriptor failed. err = %d\n",
				err);
			return err;
		}
		seq_printf(m, "0x%02x\n", (u8)desc_buf[3]);
	}
	return 0;
}
static int ufs_life_time_est_typ_a_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_life_time_est_typ_a_show, inode->i_private);
}
static const struct file_operations life_time_est_typ_a_fops = {
	.open = ufs_life_time_est_typ_a_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ufs_life_time_est_typ_b_show(struct seq_file *m, void *v)
{
	int err;
	int buff_len = QUERY_DESC_HEALTH_MAX_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_MAX_SIZE];

	if (ufs_mtk_hba) {
		pm_runtime_get_sync(ufs_mtk_hba->dev);
		err = ufshcd_read_health_desc(ufs_mtk_hba, desc_buf, buff_len);
		pm_runtime_put_sync(ufs_mtk_hba->dev);
		if (err) {
			seq_printf(m, "Reading Health Descriptor failed. err = %d\n",
				err);
			return err;
		}
		seq_printf(m, "0x%02x\n", (u8)desc_buf[4]);
	}
	return 0;
}
static int ufs_life_time_est_typ_b_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_life_time_est_typ_b_show, inode->i_private);
}
static const struct file_operations life_time_est_typ_b_fops = {
	.open = ufs_life_time_est_typ_b_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

UFS_PROC_SHOW(type, "%d\n", BOOTTYPE);

static const char * const ufs_proc_list[] = {
	"cid",
	"type",
	"manfid",
	"size",
	"product_name",
	"rev",
	"pre_eol_info",
	"life_time_est_typ_a",
	"life_time_est_typ_b",
	"name",
};
static const struct file_operations *proc_fops_list[] = {
	&cid_fops,
	&type_fops,
	&manfid_fops,
	&size_fops,
	&product_name_fops,
	&fw_version_fops,
	&pre_eol_info_fops,
	&life_time_est_typ_a_fops,
	&life_time_est_typ_b_fops,
	&name_fops,
};

int ufs_debug_proc_init_bootdevice(void)
{
	struct proc_dir_entry *prEntry;
	struct proc_dir_entry *bootdevice_dir;
	int i, num;

	bootdevice_dir = proc_mkdir("bootdevice", NULL);

	if (!bootdevice_dir) {
		pr_notice("[%s]: failed to create /proc/bootdevice\n",
			__func__);
		return -1;
	}

	num = ARRAY_SIZE(ufs_proc_list);
	for (i = 0; i < num; i++) {
		prEntry = proc_create(ufs_proc_list[i], 0,
			bootdevice_dir, proc_fops_list[i]);
		if (prEntry)
			continue;
		pr_notice(
			"[%s]: failed to create /proc/bootdevice/%s\n",
			__func__, ufs_proc_list[i]);
	}

	return 0;
}
#endif

int ufs_mtk_debug_proc_init(struct ufs_hba *hba)
{
	struct ufs_mtk_host *host;
	struct proc_dir_entry *prEntry;
	kuid_t uid;
	kgid_t gid;

	if (!hba || !hba->priv) {
		pr_info("%s: NULL host, exiting\n", __func__);
		return -EINVAL;
	}

	host = hba->priv;

	uid = make_kuid(&init_user_ns, 0);
	gid = make_kgid(&init_user_ns, 1001);

	prEntry = proc_create("ufs_debug", PROC_PERM, NULL, &ufs_proc_fops);

	if (prEntry)
		proc_set_user(prEntry, uid, gid);
	else
		pr_info("%s: failed to create /proc/ufs_debug\n", __func__);

	prEntry = proc_create("ufs_help", PROC_PERM, NULL, &ufs_help_fops);

	if (!prEntry)
		pr_info("%s: failed to create /proc/ufs_help\n", __func__);

	/* allow write permission in all builds */
	prEntry = proc_create_data("ufs_perf", 0660, NULL, &ufs_perf_fops,
				   host);

	if (prEntry)
		proc_set_user(prEntry, uid, gid);
	else
		pr_info("%s: failed to create /proc/ufs_perf\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(ufs_mtk_debug_proc_init);
