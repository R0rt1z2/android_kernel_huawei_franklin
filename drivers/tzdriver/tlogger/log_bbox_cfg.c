/*
 * log_bbox_cfg.c
 *
 * for bbox log cfg api define
 *
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
 */
#include "log_cfg_api.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <device/bbox_pub.h>
#include <linux/io.h>
#include <securec.h>
#include "tc_ns_log.h"
#include "tlogger.h"

#define EXCEPT_MODULE_INDEX 17
#define EXCEPT_LEVER_INDEX  25
#define EXCEPT_ENCODE_INDEX 28
#define EXCEPT_REPORT_INDEX 30

#define UINT64_MAX (u64)(~((u64)0)) /* 0xFFFFFFFFFFFFFFFF */

/* Host 0x1 ,DEVICE 0x2 */
#define B_DEVICE   0x2
/* Code : recover 0x1, Exception  0x2, recovered 0x3 */
#define B_EXCEP    0x2

/* Emergency priority */
#define TEEOS_MODID \
	(B_DEVICE << EXCEPT_REPORT_INDEX | B_EXCEP << EXCEPT_ENCODE_INDEX | \
	BBOX_CRITICAL << EXCEPT_LEVER_INDEX | BBOX_TEEOS << EXCEPT_MODULE_INDEX) \

#define TEEOS_MODID_END TEEOS_MODID

/* Minor priority */
#define TEEOS_TA_MODID \
	(B_DEVICE << EXCEPT_REPORT_INDEX | B_EXCEP << EXCEPT_ENCODE_INDEX | \
	BBOX_MINOR << EXCEPT_LEVER_INDEX | BBOX_TEEOS << EXCEPT_MODULE_INDEX) \

#define TEEOS_TA_MODID_END TEEOS_TA_MODID

struct teeos_bbox_common_info {
	void __iomem *vaddr;
	excep_time b_time;
};

static const u64 g_current_core_id = BBOX_TEEOS;
struct bbox_module_result g_mem_info = {0};
struct teeos_bbox_common_info g_common_info = {0};

static void tee_fn_dump(const struct bbox_dump_ops_info *info,
	bbox_dump_done_ops fn_ops)
{
	struct bbox_dump_done_ops_info dump_done_info = {0};

	if (!info) {
		tloge("invalid info param\n");
		return;
	}
	if (!fn_ops) {
		tloge("invalid fndone param, core:0x%x\n", info->coreid);
		return;
	}
	if (info->coreid != BBOX_TEEOS) {
		tloge("invalid param, core:0x%x\n", info->coreid);
		return;
	}

	tloge("====================================\n");
	tloge("devid:      [%d]\n", info->devid);
	tloge("excepid:    [0x%x]\n", info->excepid);
	tloge("exce_tpye:  [0x%x]\n", info->etype);
	tloge("tv_sec:     [0x%llx]\n", info->time.tv_sec);
	tloge("tv_usec:    [0x%llx]\n", info->time.tv_usec);
	tloge("====================================\n");

	dump_done_info.devid = info->devid;
	dump_done_info.coreid = info->coreid;
	dump_done_info.excepid = info->excepid;
	dump_done_info.etype = info->etype;
	dump_done_info.time.tv_sec = info->time.tv_sec;
	dump_done_info.time.tv_usec = info->time.tv_usec;

	fn_ops(&dump_done_info);
}

/*
 * @brief       : get the current utc time
 * @param [in]  : excep_time *time
 * @return      : NA
 */
static void tee_log_get_systime(excep_time *time)
{
	struct timeval tv = {0};

	do_gettimeofday(&tv);
	time->tv_sec = (u64)tv.tv_sec;
	time->tv_usec = (u64)tv.tv_usec;
}

static u32 get_log_mem_len(void)
{
	if (g_mem_info.log_len > BBOX_MODULE_CTRL_BLOCK_SIZE)
		return (g_mem_info.log_len - BBOX_MODULE_CTRL_BLOCK_SIZE);

	return 0;
}

static void init_bbox_except_ctrl(struct bbox_module_exception_ctrl *ctrl, u32 excepid)
{
	ctrl->e_clock.tv_sec = g_common_info.b_time.tv_sec;
	ctrl->e_clock.tv_usec = g_common_info.b_time.tv_usec;
	ctrl->e_excepid = excepid;
	ctrl->e_block_offset = BBOX_MODULE_CTRL_BLOCK_SIZE;
	ctrl->e_block_len = get_log_mem_len();
	ctrl->e_info_len = get_log_mem_len();
}

static void init_bbox_header(struct bbox_module_ctrl *ctrl_header, u32 excepid)
{
	ctrl_header->magic = BBOX_MODULE_MAGIC;
	ctrl_header->e_block_num = 1;

	init_bbox_except_ctrl(&ctrl_header->block[0], excepid);
}

static void report_log_exception(u32 excepid)
{
	struct bbox_report_info tee_report_info = {0};

	if (!g_mem_info.log_addr || !g_mem_info.log_len) {
		tloge("invalid phys addr,len:%u for report system\n", g_mem_info.log_len);
		return;
	}

	g_common_info.vaddr = ioremap(g_mem_info.log_addr, g_mem_info.log_len);
	if (!g_common_info.vaddr) {
		tloge("vaddr map error for report system\n");
		return;
	}

	tee_log_get_systime(&g_common_info.b_time);

	init_bbox_header((struct bbox_module_ctrl *)g_common_info.vaddr, excepid);

	tee_report_info.devid = 0;
	tee_report_info.excepid = excepid;
	tee_report_info.time.tv_sec = g_common_info.b_time.tv_sec;
	tee_report_info.time.tv_usec = g_common_info.b_time.tv_usec;

	(void)bbox_exception_report(&tee_report_info); /* tee log with time already */
}

void report_log_system_error(void)
{
	report_log_exception((u32)TEEOS_MODID);
}

void report_log_system_panic(void)
{
}

void ta_crash_report_log(void)
{
	report_log_exception((u32)TEEOS_TA_MODID);
}

static int tee_bbox_register_core(void)
{
	int ret;
	struct bbox_module_info s_module_ops = {0};

	s_module_ops.coreid = g_current_core_id;
	s_module_ops.ops_dump = tee_fn_dump;
	s_module_ops.ops_reset = NULL;

	ret = bbox_register_module(&s_module_ops, &g_mem_info);
	if (ret)
		tloge("register bbox module failed\n");

	return ret;
}

void unregister_log_exception(void)
{
	int ret;

	ret = bbox_unregister_exception((u32)TEEOS_MODID);
	if (ret)
		tloge("unregister bbox exception error\n");
}

static int init_bbox_excep_info(struct bbox_exception_info *info, bool is_ta_crash)
{
	errno_t ret;
	const char tee_module_name[] = "BBOX_TEEOS";
	const char *tee_module_desc = NULL;

	if (is_ta_crash) {
		tee_module_desc = "Teeos ta crash";

		info->e_excepid = (u32)TEEOS_TA_MODID;
		info->e_excepid_end = (u32)TEEOS_TA_MODID_END;
		info->e_process_priority = BBOX_MINOR;
		info->e_reboot_priority = BBOX_REBOOT_NO;
	} else {
		tee_module_desc = "Teeos system crash";

		info->e_excepid = (u32)TEEOS_MODID;
		info->e_excepid_end = (u32)TEEOS_MODID_END;
		info->e_process_priority = BBOX_CRITICAL;
		/* This func is not support */
		info->e_reboot_priority = BBOX_REBOOT_WAIT;
	}

	info->e_reset_core_mask = BBOX_COREID_MASK(BBOX_TEEOS);
	info->e_notify_core_mask = BBOX_COREID_MASK(BBOX_TEEOS);
	info->e_reentrant = (u8)BBOX_REENTRANT_ALLOW;
	info->e_exce_type = TEE_EXCEPTION;
	info->e_from_core = BBOX_TEEOS;

	ret = memcpy_s(info->e_from_module, BBOX_MODULE_NAME_LEN,
		tee_module_name, strlen(tee_module_name));
	if (ret) {
		tloge("memcpy module name failed\n");
		return ret;
	}

	ret = memcpy_s(info->e_desc, BBOX_EXCEPTIONDESC_MAXLEN,
		tee_module_desc, strlen(tee_module_desc));
	if (ret) {
		tloge("memcpy module desc failed\n");
		return ret;
	}
	return 0;
}

static int register_tee_exception(bool is_ta_crash)
{
	int ret;
	struct bbox_exception_info info = {0};

	(void)memset_s(&info, sizeof(info), 0, sizeof(info));
	ret = init_bbox_excep_info(&info, is_ta_crash);
	if (ret)
		return ret;

	ret = bbox_register_exception(&info); /* return value 0 is error */
	if (!ret) {
		tloge("register exception mem failed\n");
		return -1;
	}

	return 0;
}

int register_log_exception(void)
{
	int ret;

	/* register teeos system exception */
	ret = register_tee_exception(false);
	if (ret) {
		tloge("register teeos exception failed\n");
		return -1;
	}

	/* register ta exception */
	ret = register_tee_exception(true);
	if (ret) {
		tloge("register ta exception failed\n");
		return -1;
	}

	return 0;
}

static int check_bbox_log_mem(void)
{
	if (!g_mem_info.log_addr || !g_mem_info.log_len) {
		tloge("invalid phys addr or len\n");
		return -1;
	}

	/* log header is 512 bit */
	if (g_mem_info.log_addr > (UINT64_MAX - BBOX_MODULE_CTRL_BLOCK_SIZE)) {
		tloge("log add's data is too large\n");
		return -1;
	}

	if (g_mem_info.log_len <= BBOX_MODULE_CTRL_BLOCK_SIZE) {
		tloge("log len is too smaller\n");
		return -1;
	}

	return 0;
}

/* Register log memory */
int register_log_mem(u64 *addr, u32 *len)
{
	int ret;
	u64 mem_addr;
	u32 mem_len;

	if (!addr || !len) {
		tloge("check addr or len is failed\n");
		return -1;
	}

	ret = tee_bbox_register_core();
	if (ret)
		return ret;

	ret = check_bbox_log_mem();
	if (ret)
		return ret;

	mem_addr = g_mem_info.log_addr; /* return is paddr */
	mem_addr += BBOX_MODULE_CTRL_BLOCK_SIZE; /* log header is 512 bit */
	mem_len = g_mem_info.log_len - BBOX_MODULE_CTRL_BLOCK_SIZE;

	ret = register_mem_to_teeos(mem_addr, mem_len, false);
	if (ret)
		return ret;

	*addr = mem_addr;
	*len = mem_len;
	return ret;
}

int *map_log_mem(u64 mem_addr, u32 mem_len)
{
	return (int *)ioremap(mem_addr, mem_len);
}

void unmap_log_mem(int *log_buffer)
{
	iounmap((void __iomem*)log_buffer);
	bbox_unregister_module(g_current_core_id);
}

#define ROOT_UID                0
#define SYSTEM_GID              0
void get_log_chown(uid_t *user, gid_t *group)
{
	if (!user || !group) {
		tloge("user or group buffer is null\n");
		return;
	}

	*user = ROOT_UID;
	*group = SYSTEM_GID;
}
