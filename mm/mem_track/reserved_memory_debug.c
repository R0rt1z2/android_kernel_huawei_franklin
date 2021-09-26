/*
 * mm/mem_track/reserved_memory_debug.c  reserved memory debug info
 *
 * Copyright(C) 2020 Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <asm/sections.h>
#include <linux/err.h>
#include <linux/sizes.h>
#include <linux/of_reserved_mem.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#define RESERVED_MEMORY	"reserved_memory"

/* find rmem is in mem_array, mem_array may be cma_array\dynamic_array */
static int __find_reserved_mem_type(struct reserved_mem *rmem,
	const char **mem_array, int array_len)
{
	int ret = 0;
	int i;

	/* look for mem_array reserve memory node */
	for (i = 0; i < array_len; i++) {
		if (!strcmp(rmem->name, mem_array[i])) {
			ret = 1;
			break;
		}
	}
	return ret;
}

static int dt_memory_reserved_debug_show(struct seq_file *m, void *private)
{
	struct reserved_mem *dt_mem_resrved = NULL;
	int reserved_mem_cnt = 0;
	const char **d_reserved_array = NULL;
	int dynamic_mem_cnt = 0;
	const char **cma_mem_reserved_array = NULL;
	int cma_mem_cnt = 0;
	struct reserved_mem *rmem = NULL;
	int i;
	int cma;
	int dynamic;

	dt_mem_resrved = get_reserved_memory(&reserved_mem_cnt);
	if (dt_mem_resrved == NULL)
		return 0;

	d_reserved_array = get_dynamic_reserved_memory(&dynamic_mem_cnt);
	if (d_reserved_array == NULL)
		return 0;

	cma_mem_reserved_array = get_cma_reserved_memory(&cma_mem_cnt);
	if (cma_mem_reserved_array == NULL)
		return 0;

	seq_printf(m, "  num [start            ....              end]      [size]       [d/s]     [cma]       [ name ]\n");

	for (i = 0; i < reserved_mem_cnt; i++) {
		rmem = &(dt_mem_resrved[i]);

		dynamic = __find_reserved_mem_type(rmem,
			d_reserved_array, dynamic_mem_cnt);
		cma = __find_reserved_mem_type(rmem,
			cma_mem_reserved_array, cma_mem_cnt);

		seq_printf(m, "%4d: ", i);
		seq_printf(m, "[0x%016llx..0x%016llx]  %8lluKB   %8s %8s        %8s\n",
			(unsigned long long)rmem->base,
			(unsigned long long)(rmem->base + rmem->size - 1),
			(unsigned long long)rmem->size / SZ_1K,
			(dynamic == 1)?"d":"s",
			(cma == 1)?"Y":"N",
			rmem->name);
	}
	return 0;
}

static int dt_memory_reserved_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dt_memory_reserved_debug_show, inode->i_private);
}

static const struct file_operations dt_memory_reserved_debug_fops = {
	.open = dt_memory_reserved_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

struct vmlinux_dump_info {
	char *segment_start;
	char *segment_end;
	const char *name;
};

static struct vmlinux_dump_info vmlinux_info[] = {
	{_text, _etext, "vmlinux_text"},
	{_data, _end, "vmlinux_data"},
	{__bss_start, __bss_stop, "vmlinux_bss"},
	{__start_rodata, __inittext_begin, "vmlinux_rodata"},
	{__inittext_begin, __inittext_end, "vmlinux_inittext"},
	{__initdata_begin, __initdata_end, "vmlinux_initdata"},
};


static int vmlinux_memory_debug_show(struct seq_file *m, void *private)
{
	int i;

	seq_printf(m, "  num [start            ....              end]      [size]             [ name ]\n");

	for (i = 0; i < ARRAY_SIZE(vmlinux_info); i++) {
		seq_printf(m, "%4d: ", i);
		seq_printf(m, "[0x%016llx..0x%016llx]  %8lluKB          %8s\n",
			vmlinux_info[i].segment_start,
			vmlinux_info[i].segment_end - 1,
			(vmlinux_info[i].segment_end - vmlinux_info[i].segment_start) / SZ_1K,
			vmlinux_info[i].name);
	}

	return 0;
}

static int vmlinux_memory_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, vmlinux_memory_debug_show, inode->i_private);
}

static const struct file_operations vmlinux_memory_debug_fops = {
	.open = vmlinux_memory_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int __init dt_memory_reserved_init_debugfs(void)
{
	struct dentry *root = NULL;

	root = debugfs_create_dir(RESERVED_MEMORY, NULL);
	if (!root)
		return -ENXIO;

	debugfs_create_file("dt_reserved_memory", S_IRUGO, root,
		NULL, &dt_memory_reserved_debug_fops);

	debugfs_create_file("vmlinux_segment", S_IRUGO, root,
		NULL, &vmlinux_memory_debug_fops);
	return 0;
}
__initcall(dt_memory_reserved_init_debugfs);

