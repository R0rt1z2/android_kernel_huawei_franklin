/*
 * track.c
 *
 * record the irq and task data. (kernel run data recorder.)
 *
 * Copyright (c) 2013-2020 Huawei Technologies Co., Ltd.
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_HUAWEI_KERNEL_DEBUG
#include <linux/fs.h>
#endif
#include <securec.h>
#include <ringbuffer.h>
#include "track.h"

#define BB_TRACK_MEM_SIZE      (512 * 1024)

struct ap_hook g_ap_hook = {0};
static atomic_t g_ap_hook_on[AP_HK_MAX] = { ATOMIC_INIT(0) };
static struct ap_percpu_buffer_info *g_hook_percpu_buffer[AP_HK_PERCPU_TAG] = { NULL };
static unsigned char *g_hook_buffer_addr[AP_HK_MAX] = { 0 };
static const char *g_hook_trace_pattern[AP_HK_MAX] = {
	"irq_switch::ktime,slice,vec,dir",
	"task_switch::ktime,stack,pid,comm",
};
char g_ringbuff_irq[BB_TRACK_MEM_SIZE] = {0};
char g_ringbuff_task[BB_TRACK_MEM_SIZE] = {0};

/*
 * Description: Record kernel task traces
 * Input:       Pre_task: current task task structure pointer, next_task: next task task structure pointer
 * Other:       added to kernel/sched/core.c
 */
void task_switch_hook(const void *pre_task, void *next_task)
{
	/* Record the timestamp, cpu_id, next_task, task name, and the loop buffer corresponding to the cpu */
	struct task_struct *task = next_task;
	struct task_info_ap info;
	u8 cpu;

	if (!pre_task || !next_task) {
		pr_err("%s() error:pre_task or next_task is NULL\n", __func__);
		return;
	}

	/* hook is not installed */
	if (!atomic_read(&g_ap_hook_on[AP_HK_TASK]))
		return;

	info.clock = (u64)ktime_get_boottime();
	cpu = (u8)smp_processor_id();
	info.pid = (u32)task->pid; /* process identifier */
	/* the executable file name that the process is running */
	(void)strncpy(info.comm, task->comm, sizeof(task->comm) - 1);
	info.comm[TASK_COMM_LEN - 1] = '\0';
	info.stack = (uintptr_t)task->stack;

	ap_ringbuffer_write((struct ap_ringbuffer_s *)g_hook_percpu_buffer[AP_HK_TASK]->percpu_addr[cpu], (u8 *)&info);
}
EXPORT_SYMBOL(task_switch_hook);

/*
 * Description: Interrupt track record
 * Input:       dir: 0 interrupt entry, 1 interrupt exit, new_vec: current interrupt
 */
void irq_trace_hook(unsigned int dir, unsigned int old_vec, unsigned int new_vec)
{
	/* Record time stamp, cpu_id, interrupt number, interrupt in and out direction */
	struct hook_irq_info_ap info;
	u8 cpu;

	/* hook is not installed */
	if (!atomic_read(&g_ap_hook_on[AP_HK_IRQ]))
		return;

	info.clock = (u64)ktime_get_boottime();
	info.jiff = jiffies_64;

	cpu = (u8)smp_processor_id();
	info.dir = (u8)dir;
	info.irq = (u32)new_vec;

	ap_ringbuffer_write((struct ap_ringbuffer_s *)g_hook_percpu_buffer[AP_HK_IRQ]->percpu_addr[cpu], (u8 *)&info);
}
EXPORT_SYMBOL(irq_trace_hook);

int percpu_buffer_init(struct ap_percpu_buffer_info *buffer_info, u32 ratio[][AP_HOOK_CPU_NUMBERS], /* HERE:8 */
				u32 cpu_num, u32 fieldcnt, const char *keys, u32 gap)
{
	unsigned int i;
	int ret;
	struct ap_percpu_buffer_info *buffer = buffer_info;

	if (!keys) {
		pr_err("[%s], argument keys is NULL\n", __func__);
		return -EINVAL;
	}

	if (!buffer) {
		pr_err("[%s], buffer info is null\n", __func__);
		return -EINVAL;
	}

	if (!buffer->buffer_addr) {
		pr_err("[%s], buffer_addr is NULL\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < cpu_num; i++) {
		pr_err("[%s], ratio[%u][%u] = [%u]\n", __func__, (cpu_num - 1), i, ratio[cpu_num - 1][i]);
		buffer->percpu_length[i] = (buffer->buffer_size - cpu_num * gap) /
					   PERCPU_TOTAL_RATIO * ratio[cpu_num - 1][i];

		if (i == 0)
			buffer->percpu_addr[0] = buffer->buffer_addr;
		else
			buffer->percpu_addr[i] = buffer->percpu_addr[i - 1] + buffer->percpu_length[i - 1] + gap;

		pr_err(
		       "[%s], [%u]: percpu_addr [0x%pK], percpu_length [0x%x], fieldcnt [%u]\n",
		       __func__, i, buffer->percpu_addr[i],
		       buffer->percpu_length[i], fieldcnt);

		ret = ap_ringbuffer_init((struct ap_ringbuffer_s *)
					 buffer->percpu_addr[i],
					 buffer->percpu_length[i], fieldcnt,
					 keys);
		if (ret) {
			pr_err("[%s], cpu [%u] ringbuffer init failed!\n", __func__, i);
			return ret;
		}
	}
	return 0;
}

int hook_percpu_buffer_init(struct ap_percpu_buffer_info *buffer_info,
				unsigned char *addr, u32 size, u32 fieldcnt,
				enum hook_type_ap hk, u32 ratio[][AP_HOOK_CPU_NUMBERS])
{
	u64 min_size;
	u32 cpu_num = num_possible_cpus();

	pr_info("[%s], num_online_cpus [%u] !\n", __func__, num_online_cpus());

	if (IS_ERR_OR_NULL(addr) || IS_ERR_OR_NULL(buffer_info)) {
		pr_err(
		       "[%s], buffer_info [0x%pK] buffer_addr [0x%pK], buffer_size [0x%x]\n",
		       __func__, buffer_info, addr, size);
		return -EINVAL;
	}

	min_size = cpu_num * (sizeof(struct ap_ringbuffer_s) + PERCPU_TOTAL_RATIO * (u64)(unsigned int)fieldcnt);
	if (size < (u32)min_size) {
		g_hook_buffer_addr[hk] = 0;
		g_hook_percpu_buffer[hk] = buffer_info;
		g_hook_percpu_buffer[hk]->buffer_addr = 0;
		g_hook_percpu_buffer[hk]->buffer_size = 0;
		pr_err(
		       "[%s], buffer_info [0x%pK] buffer_addr [0x%pK], buffer_size [0x%x]\n",
		       __func__, buffer_info, addr, size);
		return 0;
	}

	if (hk >= AP_HK_PERCPU_TAG) {
		pr_err("[%s], hook_type [%d] is invalid!\n", __func__, hk);
		return -EINVAL;
	}

	pr_info("[%s], buffer_addr [0x%pK], buffer_size [0x%x]\n",
	       __func__, addr, size);

	g_hook_buffer_addr[hk] = addr;
	g_hook_percpu_buffer[hk] = buffer_info;
	g_hook_percpu_buffer[hk]->buffer_addr = addr;
	g_hook_percpu_buffer[hk]->buffer_size = size;

	return percpu_buffer_init(buffer_info, ratio, cpu_num,
				  fieldcnt, g_hook_trace_pattern[hk], 0);
}

int irq_buffer_init(struct ap_percpu_buffer_info *buffer_info, unsigned char *addr, unsigned int size)
{
	unsigned int irq_record_ratio[AP_HOOK_CPU_NUMBERS][AP_HOOK_CPU_NUMBERS] = {
	{ 16, 0, 0, 0, 0, 0, 0, 0 },
	{ 8, 8, 0, 0, 0, 0, 0, 0 },
	{ 8, 4, 4, 0, 0, 0, 0, 0 },
	{ 8, 4, 2, 2, 0, 0, 0, 0 },
	{ 8, 4, 2, 1, 1, 0, 0, 0 },
	{ 8, 4, 1, 1, 1, 1, 0, 0 },
	{ 6, 4, 2, 1, 1, 1, 1, 0 },
	{ 6, 4, 1, 1, 1, 1, 1, 1 }
	};

	return hook_percpu_buffer_init(buffer_info, addr, size,
				       sizeof(struct hook_irq_info_ap), AP_HK_IRQ,
				       irq_record_ratio);
}

int task_buffer_init(struct ap_percpu_buffer_info *buffer_info, unsigned char *addr, unsigned int size)
{
	unsigned int task_record_ratio[AP_HOOK_CPU_NUMBERS][AP_HOOK_CPU_NUMBERS] = {
	{ 16, 0, 0, 0, 0, 0, 0, 0 },
	{ 8, 8, 0, 0, 0, 0, 0, 0 },
	{ 8, 4, 4, 0, 0, 0, 0, 0 },
	{ 8, 4, 2, 2, 0, 0, 0, 0 },
	{ 4, 4, 4, 2, 2, 0, 0, 0 },
	{ 4, 4, 2, 2, 2, 2, 0, 0 },
	{ 4, 4, 2, 2, 2, 1, 1, 0 },
	{ 4, 2, 2, 2, 2, 2, 1, 1 }
	};

	return hook_percpu_buffer_init(buffer_info, addr, size,
				       sizeof(struct task_info_ap), AP_HK_TASK,
				       task_record_ratio);
}

static int __init track_init(void)
{
	int ret;

	g_ap_hook.hook_buffer_addr[AP_HK_IRQ] = g_ringbuff_irq;
	g_ap_hook.hook_buffer_addr[AP_HK_TASK] = g_ringbuff_task;

	pr_info("[%s], irq_buffer_init start!\n", __func__);
	ret = irq_buffer_init(&g_ap_hook.hook_percpu_buffer[AP_HK_IRQ],
		g_ap_hook.hook_buffer_addr[AP_HK_IRQ], BB_TRACK_MEM_SIZE);
	if (ret) {
		pr_err("[%s], irq_buffer_init failed!\n", __func__);
		return ret;
	}

	atomic_set(&g_ap_hook_on[AP_HK_IRQ], 1);

	pr_info("[%s], task_buffer_init start!\n", __func__);
	ret = task_buffer_init(&g_ap_hook.hook_percpu_buffer[AP_HK_TASK],
		g_ap_hook.hook_buffer_addr[AP_HK_TASK], BB_TRACK_MEM_SIZE);
	if (ret) {
		pr_err("[%s], task_buffer_init failed!\n", __func__);
		return ret;
	}

	atomic_set(&g_ap_hook_on[AP_HK_TASK], 1);

	return 0;
}

static void __exit track_exit(void)
{
	;
}

subsys_initcall_sync(track_init);
module_exit(track_exit);

#ifdef CONFIG_HUAWEI_KERNEL_DEBUG
#define PATH_MAXLEN 128
#define FILE_LIMIT  0660
int rdr_savebuf2fs(const char *logpath, const char *filename,
				const void *buf, u32 len)
{
	int ret;
	int flags;
	struct file *fp = NULL;
	char path[PATH_MAXLEN];

	if (logpath == NULL || filename == NULL || buf == NULL || len <= 0) {
		pr_err("invalid  parameter. path:%s, name:%s buf:%llx len:0x%x\n",
		     logpath, filename, buf, len);
		ret = -1;
		goto end;
	}

	(void)snprintf(path, PATH_MAXLEN, "%s/%s", logpath, filename);

	flags = O_CREAT | O_RDWR | O_TRUNC;
	fp = filp_open(path, flags, FILE_LIMIT);
	if (IS_ERR(fp)) {
		pr_err("%s():create file %s err. fp=0x%llx\n", __func__, path, fp);
		ret = -1;
		goto end;
	}
	vfs_llseek(fp, 0L, SEEK_END);
	ret = vfs_write(fp, buf, len, &(fp->f_pos));
	if (ret != len) {
		pr_err("%s():write file %s exception with ret %d\n",
			     __func__, path, ret);
		goto close;
	}

	vfs_fsync(fp, 0);
close:
	filp_close(fp, NULL);
end:
	return ret;
}
void track_save_to_storage(void)
{
	const char *logpath = "/data/log";
	int len;

	len = rdr_savebuf2fs(logpath, "irq", g_ringbuff_irq, BB_TRACK_MEM_SIZE);
	if (len != BB_TRACK_MEM_SIZE) {
		pr_err("write file irq fail\n");
		return;
	}

	len = rdr_savebuf2fs(logpath, "task", g_ringbuff_task, BB_TRACK_MEM_SIZE);
	if (len != BB_TRACK_MEM_SIZE) {
		pr_err("write file task fail\n");
		return;
	}
}
#endif
