/*
 * kernel_dump.c
 *
 * memory/register proc-fs dump implementation
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
 *
 */

#include <asm/pgtable.h>
#include <linux/mm_types.h>
#include <linux/memblock.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <asm/memory.h>
#include <kdump.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/rculist.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "securec.h"

extern int pcpu_base_size;
extern struct mm_struct init_mm;
extern struct page *mem_map;
extern char _text[];
extern char _end[];

#ifdef CONFIG_RANDOMIZE_BASE
extern s64 phystart_addr;
#endif
#ifdef CONFIG_KERNELDUMP_KO_DBG
extern struct list_head modules;
#endif

struct kernel_dump_cb *g_kdump_cb = NULL;

static struct table_extra {
	u64 extra_mem_phy_base;
	u64 extra_mem_virt_base;
	u64 extra_mem_size;
} g_tbl_extra_mem[MAX_EXTRA_MEM] = { {0, 0} };

static unsigned int extra_index;
static DEFINE_RAW_SPINLOCK(g_kdump_lock);
unsigned long long g_resize_addr = 0;
unsigned long long g_himntn_value = 0;
unsigned int g_skp_flag = 0;
extern unsigned long get_kdump_logbuf_start(void);
extern char *pbuff_kdump;
unsigned long long g_kdump_addr;
extern void show_pte(unsigned long addr);
void skp_save_kdump_file(void);

static struct kernel_dump_cb *g_kernel_dump_cb = NULL;

struct memblock_type *g_yd_print_mb_cb = NULL;
static unsigned int g_dataready_flag = DATA_NOT_READY;


int rdr_wait_partition(const char *path, int timeouts)
{
	struct kstat m_stat;
	int timeo;

	if (path == NULL) {
		pr_err("invalid  parameter path\n");
		return -1;
	}

	timeo = timeouts;

	if (strncmp(path, "/data", strlen("/data")) == 0) {
		while (vfs_stat(path, &m_stat) != 0) {
			current->state = TASK_INTERRUPTIBLE;
			(void)schedule_timeout(HZ / 10);    /* wait for 1/10 second */
			if (timeouts-- < 0) {
				pr_err("%d:rdr:wait partiton[%s] fail. use [%d]'s . skip!\n", __LINE__, path, timeo);
				return -1;
			}
		}
	}

	return 0;
}


static int kdump_main_thread_body(void *arg)
{
	long jiffies_time;
	int ret;

	while (!kthread_should_stop()) {
		pr_err("====wait for fs ready start ====\n");
		while (rdr_wait_partition(KDUMP_WAIT_DONE_PATH, RDR_WAIT_PARTITION_TIME) != 0)
			;
		pr_err("====wait for fs ready end ====\n");
		skp_save_kdump_file();

		pr_err("====save done ====\n");

		return 0;
	}
}

/*
 * Description:    create /proc/data-ready
 * Return:         0:success;-1:fail
 */
static int __init kdump_datasave_init(void)
{
	struct task_struct *kdump_main = NULL;

	kdump_main = kthread_run(kdump_main_thread_body, NULL, "huawei_dump");
	if (kdump_main == NULL) {
		pr_err("create thread kdump main thread faild\n");
		return -1;
	}

	return 0;
}

module_init(kdump_datasave_init);


u32 checksum32(u32 *addr, u32 count)
{
	u64 sum = 0;
	u32 i;

	while (count > sizeof(u32) - 1) {
		/*  This is the inner loop */
		sum += *(addr++);
		count -= sizeof(u32);
	}

	if (count > 0) {
		u32 left = 0;

		i = 0;
		while (i <= count) {
			*((u8 *)&left + i) = *((u8 *)addr + i);
			i++;
		}

		sum += left;
	}

	while (sum >> KDUMP_CRC_OFFSET)
		sum = (sum & 0xffffffff) + (sum >> KDUMP_CRC_OFFSET);

	return (~sum);
}

static int resizecb_proc_para_check(struct file *file, char __user *buffer, loff_t *data)
{
	if (!file || !buffer || !data)
		return -1;

	return 0;
}

static int check_himntn(unsigned int himntn_id)
{
	if (g_himntn_value & (0x1 << himntn_id)) {
		pr_err("himntn is enable\n");
		return 1;
	}

	return 0;
}

ssize_t resizecb_read_proc(struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	ssize_t ret = -EINVAL;
	char tmp;

	if (resizecb_proc_para_check(file, buffer, data))
		return ret;

	if (count < RESIZE_FLAG_MAX)
		return ret;

	if (*data)
		return 0;

	tmp = (char)(check_himntn(HIMNTN_ID_KDUMP_SWITCH) + '0');

	ret = simple_read_from_buffer(buffer, count, data, &tmp, sizeof(char));

	pr_info("%s():%d:output arg [%c],%d\n", __func__, __LINE__, tmp, tmp);

	return ret;
}

/*
 * Function:       dataready_write_proc
 * Description:    write /proc/resize-cb, for get the status of data_partition
 * Input:          file;buffer;count;data
 * Output:         NA
 * Return:         >0:success;other:fail
 */
ssize_t resizecb_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	ssize_t ret = -EINVAL;
	char tmp;

	/* buffer must be '1' or '0', so count<=2 */
	if (count > RESIZE_FLAG_MAX)
		return ret;

	if (resizecb_proc_para_check(file, (char __user *)buffer, data))
		return ret;

	/* should ignore character '\n' */
	if (copy_from_user(&tmp, buffer, sizeof(tmp)))
		return -EFAULT;

	pr_err("%s():%d:input arg [%c],%d\n", __func__, __LINE__, tmp, tmp);

	if (tmp == '1') {
		g_kdump_cb->resize_flag = SKP_DUMP_RESIZE_SUCCESS_FLAG;
		g_kdump_cb->crc = 0;
		g_kdump_cb->crc = checksum32((u32 *)g_kdump_cb, sizeof(struct kernel_dump_cb));
		pr_err("%s():%d resize success\n", __func__, __LINE__);
	} else if (tmp == '0') {
		g_kdump_cb->resize_flag = SKP_DUMP_RESIZE_FAIL;
		g_kdump_cb->crc = 0;
		g_kdump_cb->crc = checksum32((u32 *)g_kdump_cb, sizeof(struct kernel_dump_cb));
		pr_err("%s():%d resize fail\n", __func__, __LINE__);
	} else {
		pr_err("%s():%d:input arg invalid[%c]\n", __func__, __LINE__, tmp);
	}
	return 1;
}

/*
 * Function:       dataready_info_show
 * Description:    show g_dataready_flag
 * Input:          struct seq_file *m, void *v
 * Output:         NA
 * Return:         0:success;other:fail
 */
static int resizecb_info_show(struct seq_file *m, void *v)
{
	pr_err("%x\n", g_kdump_cb->resize_flag);
	return 0;
}

/*
 * Function:       dataready_open
 * Description:    open /proc/data-ready
 * Input:          inode;file
 * Output:         NA
 * Return:         0:success;other:fail
 */
static int resizecb_open(struct inode *inode, struct file *file)
{
	if (!file)
		return -EFAULT;

	return single_open(file, resizecb_info_show, NULL);
}


static const struct file_operations resizecb_proc_fops = {
	.open		= resizecb_open,
	.write		= resizecb_write_proc,
	.read		= resizecb_read_proc,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void kernel_dump_printcb(struct memblock_type *print_mb_cb, struct kernel_dump_cb *cb)
{
	unsigned int i;

	pr_info("cb->page is 0x%lx\n", (uintptr_t)(cb->page));
	pr_info("cb->page_shift is 0x%x\n", cb->page_shift);
	pr_info("cb->struct_page_size is 0x%x\n", cb->struct_page_size);
	pr_info("cb->phys_offset is 0x%llx\n", cb->phys_offset);
	pr_info("cb->page_offset is 0x%llx\n", cb->page_offset);
	pr_info("cb->pfn_offset is 0x%llx\n", cb->pfn_offset);
	pr_info("cb->ttbr is ttbr:%llx\n", cb->ttbr);
	pr_info("cb->mb_cb is 0x%lx\n", (uintptr_t)(cb->mb_cb));
	pr_info("cb->section_size is 0x%llx\n", cb->section_size);
	pr_info("cb->pmd_size is 0x%llx\n", cb->pmd_size);
	pr_info("cb->mbr_size is 0x%llx\n", cb->mbr_size);
	pr_info("cb->mbr_cnt is 0x%llx\n", print_mb_cb->cnt);
	pr_info("cb->mb_cb_region is 0x%x\n", cb->mb_cb_region);
	pr_info("cb->mb_cb_cnt is 0x%x\n", cb->mb_cb_cnt);
	pr_info("cb->kern_map_offset is 0x%llx\n", (unsigned long long)(cb->kern_map_offset));
	for (i = 0; i < print_mb_cb->cnt; i++) {
		pr_info("print_mb_cb->regions base is 0x%llx\n", (print_mb_cb->regions + i)->base);
		pr_info("print_mb_cb->regions size is 0x%llx\n", (print_mb_cb->regions + i)->size);
	}
}


int kernel_dump_init(void)
{
	unsigned int i, j;
	unsigned long long k;
	phys_addr_t mem_ret;
	struct kernel_dump_cb *cb = NULL;
	struct memblock_type *print_mb_cb = NULL;
	struct proc_dir_entry *proc_dir_entry = NULL;

	pr_err("[%s:%d]:start\n]", __func__, __LINE__);

	k = get_kdump_logbuf_start();
	pr_err("[%s:%d]:start %llx %x\n]", __func__, __LINE__, k, sizeof(struct kernel_dump_cb));

	cb = (struct kernel_dump_cb *)(get_kdump_logbuf_start());

	cb->magic = KERNELDUMP_CB_MAGIC;
	cb->page_shift = PAGE_SHIFT;
	cb->struct_page_size = sizeof(struct page);
#ifdef CONFIG_RANDOMIZE_BASE
	cb->phys_offset = phystart_addr;
	cb->kernel_offset = kimage_vaddr;
#else
	cb->phys_offset = PHYS_OFFSET;
	cb->kernel_offset = KIMAGE_VADDR;
#endif
	cb->page_offset = PAGE_OFFSET;
	cb->extra_mem_phy_base[0] = virt_to_phys(_text);
	cb->extra_mem_virt_base[0] = (u64)(uintptr_t)_text;
	cb->extra_mem_size[0] = ALIGN((u64)(uintptr_t)_end - (u64)(uintptr_t)_text, PAGE_SIZE);
	cb->extra_mem_phy_base[1] = virt_to_phys(pcpu_base_addr); /* per cpu info */
	cb->extra_mem_virt_base[1] = (u64)(uintptr_t)pcpu_base_addr; /* per cpu info */
	cb->extra_mem_size[1] = (u64)ALIGN(pcpu_base_size, PAGE_SIZE)*CONFIG_NR_CPUS;
	/* first and second has been processed above, loop begins at 2 */
	for (i = 2, j = 0; i < MAX_EXTRA_MEM && j < extra_index; i++, j++) {
		cb->extra_mem_phy_base[i] = g_tbl_extra_mem[j].extra_mem_phy_base;
		cb->extra_mem_virt_base[i] = g_tbl_extra_mem[j].extra_mem_virt_base;
		cb->extra_mem_size[i] = g_tbl_extra_mem[j].extra_mem_size;
	}
	extra_index = i;

	pr_info("_text:0x%pK _end:0x%pK\n", _text, _end);

#ifdef CONFIG_FLATMEM
	cb->page = mem_map;
	cb->pfn_offset = PHYS_PFN_OFFSET;
	cb->section_size = 0;
#elif defined CONFIG_SPARSEMEM_VMEMMAP
	cb->page = vmemmap;
	cb->pfn_offset = 0;
	cb->pmd_size = PMD_SIZE;
	cb->section_size = 1UL << SECTION_SIZE_BITS;
#else
#error "Configurations other than CONFIG_PLATMEM and CONFIG_SPARSEMEM_VMEMMAP are not supported"
#endif
#ifdef CONFIG_64BIT
	/* Subtract the base address that TTBR1 maps */
	cb->kern_map_offset = (UL(0xffffffffffffffff) << VA_BITS);
#else
	cb->kern_map_offset = 0;
#endif
	cb->ttbr = virt_to_phys(init_mm.pgd);
	cb->mb_cb = (struct memblock_type *)(uintptr_t)virt_to_phys(&memblock.memory);
	cb->mb_cb_region = virt_to_phys(((struct memblock_type *)&memblock.memory)->regions);
	cb->mb_cb_cnt = ((struct memblock_type *)&memblock.memory)->cnt;
	print_mb_cb = &memblock.memory;
	g_yd_print_mb_cb = print_mb_cb;
	cb->mbr_size = sizeof(struct memblock_region);
	cb->text_kaslr_offset = (uintptr_t)_text - (KIMAGE_VADDR + TEXT_OFFSET);
	mem_ret = memblock_start_of_DRAM();

	cb->linear_kaslr_offset = __phys_to_virt(mem_ret) - PAGE_OFFSET;
	cb->resize_flag = SKP_DUMP_RESIZE_FAIL;
	cb->skp_fastbootdump_flag    = 0;
	cb->resize_addr = 0;
	cb->skp_fastbootflash_flag = 0;

	kernel_dump_printcb(print_mb_cb, cb);

	pr_err("[%s:%d]:pbuff_kdump %llx\n]", __func__, __LINE__, pbuff_kdump);
	show_pte(pbuff_kdump);

	g_kdump_cb = cb;

	cb->crc = 0;
	cb->crc = checksum32((u32 *)cb, sizeof(struct kernel_dump_cb));

	pr_err("%s: kdump enable ,proc create\n", __func__);
	proc_dir_entry = proc_create(RESIZE_RESULT_NAME, RESIZE_PROC_RIGHT, NULL, &resizecb_proc_fops);
	if (!proc_dir_entry) {
		pr_err("proc_create RESIZE_RESULT_NAME fail\n");
		return -1;
	}

	return 0;
err:
	return -1;
}

u64 skp_resiz_addr_get(void)
{
	return g_resize_addr;
}

unsigned int skp_flag_get(void)
{
	return g_skp_flag;
}

/*
 * Description : Copy the content of a file to another file
 */
int skp_rdr_copy_file_apend(const char *dst, const char *src)
{
	struct sky_file_info info;
	char buf[SZ_4K / SZ_4];
	long cnt;
	int ret = 0;
	struct memdump *tmp = NULL;
	long kdump_size;
	long resize_addr = skp_resiz_addr_get();
	mm_segment_t oldfs;

	if (!dst || !src) {
		pr_err("rdr:%s():dst or src is NULL\n", __func__);
		return -1;
	}
	pr_err("rdr:%s():dst=%s or src=%s\n", __func__, dst, src);

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	info.fdsrc = -1;
	info.fddst = -1;
	info.seek_return = -1;

	info.fdsrc = sys_open(src, O_RDONLY, FILE_LIMIT);
	if (info.fdsrc < 0) {
		pr_err("rdr:%s():open %s failed, return [%ld]\n", __func__, src, info.fdsrc);
		ret = -1;
		goto out;
	}

	info.fddst = sys_open(dst, O_CREAT | O_WRONLY, FILE_LIMIT);
	if (info.fddst < 0) {
		pr_err("rdr:%s():open %s failed, return [%ld]\n", __func__, dst, info.fddst);
		sys_close((unsigned int)info.fdsrc);
		ret = -1;
		goto out;
	}

	info.seek_value = resize_addr;
	pr_err("%s():%d:seek value[%lx]\n", __func__, __LINE__, info.seek_value);

	/* Offset address read, seek to resize addr */
	info.seek_return = sys_lseek((unsigned int)info.fdsrc, info.seek_value, SEEK_SET);
	if (info.seek_return < 0) {
		pr_err("%s():%d:lseek fail[%ld]\n", __func__, __LINE__, info.seek_return);
		ret = -1;
		goto close;
	}

	/* read out the head of kdump,find the size to save */
	cnt = sys_read((unsigned int)info.fdsrc, buf, SZ_4K / SZ_4);
	if (cnt <= 0) {
		pr_err("rdr:%s():read %s failed, return [%ld]\n", __func__, src, cnt);
		ret = -1;
		goto close;
	}

	tmp = (struct memdump *)buf;

	pr_err("%s():%d:seek value[%lx] %s\n", __func__, __LINE__, tmp->size, tmp->name);

	/* add SZ_4K, for size less than 4K condition */
	kdump_size = (long)(tmp->size + SZ_4K / SZ_4 + KDUMP_SKP_DATASAVE_OFFSET);
	if (kdump_size > KDUMP_MAX_SIZE) {
		pr_err("%s():%d:kdump_size=%ld error\n", __func__, __LINE__, kdump_size);
		ret = -1;
		goto close;
	}

	pr_err("%s():%d:seek value[%lx]\n", __func__, __LINE__, tmp->size);

	/* Offset address read, seek to resize addr again */
	info.seek_return = (long)sys_lseek((unsigned int)info.fdsrc, info.seek_value, SEEK_SET);
	if (info.seek_return < 0) {
		pr_err("%s():%d:lseek fail[%ld]\n", __func__, __LINE__, info.seek_return);
		ret = -1;
		goto close;
	}

	/* start to dump flash to data partiton */
	while (kdump_size > 0 && kdump_size <= KDUMP_MAX_SIZE) {
		cnt = sys_read((unsigned int)info.fdsrc, buf, SZ_4K / SZ_4);
		if (cnt == 0)
			break;
		if (cnt < 0) {
			pr_err("rdr:%s():read %s failed, return [%ld]\n", __func__, src, cnt);
			ret = -1;
			goto close;
		}

		cnt = sys_write((unsigned int)info.fddst, buf, SZ_4K / SZ_4);
		if (cnt <= 0) {
			pr_err("rdr:%s():write %s failed, return [%ld]\n", __func__, dst, cnt);
			ret = -1;
			goto close;
		}

		kdump_size = kdump_size - (SZ_4K / SZ_4);
	}

close:
	sys_close((unsigned int)info.fdsrc);
	sys_close((unsigned int)info.fddst);
out:
	set_fs(oldfs);
	return ret;
}

static int skp_rdr_create_file(const char *name)
{
	long fd = -1;

	if (strncmp(name, "/data", strlen("/data")) != 0) {
		pr_err("invalid input path name\n");
		return -1;
	}

	fd = sys_open(name, O_CREAT | O_RDONLY, FILE_LIMIT);
	if (fd < 0) {
		pr_err("create file[%s] fail,ret:%ld\n", name, fd);
		return -1;
	}
	return 0;
}

void skp_save_kdump_file(void)
{
	int ret;
	unsigned int lk_skp_flag = 0;

	lk_skp_flag = skp_flag_get();

	pr_err("[%s], set kdump_gzip flag 0x%x\n", __func__, lk_skp_flag);

	if (lk_skp_flag == SKP_KDUMP_SAVE_SUCCESS_FLAG) {
		ret = skp_rdr_copy_file_apend(KDUMP_DEST_ADDR, KDUMP_SRC_ADDR);
		if (ret == -1) {
			pr_err("[%s], set kdump_gzip flag\n", __func__);
			return;
		}
		skp_rdr_create_file(KDUMPDONE_DEST_ADDR);
	}
}

u64 skp_abort(void)
{
	return g_kernel_dump_cb->magic;
}

void skp_swapper_tasks(void)
{
	struct task_struct *g = NULL;
	struct task_struct *p = NULL;

	for_each_process_thread(g, p) {
		pr_err("task struct addr is %llx , comm %s\n", p, p->comm);
	}
}

static int early_parse_skpdump_cmdline(char *p)
{
	int ret;
	char *ptr = NULL;

	if (p == NULL)
		return -1;

	ptr = p;

	while (*ptr != 0) {
		if (*ptr == '+')
			*ptr = ' ';

		ptr++;
	}

	ret = sscanf_s(p, "%llx %x", &g_resize_addr, &g_skp_flag);
	if (ret != 2) { /* resize_addr and skp_flag should be both correct, so ret is 2 */
		pr_err("sscanf parse error\n");
		return -1;
	}

	pr_err("g_resize_addr is 0x%llx, g_skp_flag is 0x%x\n", g_resize_addr, g_skp_flag);
}

early_param("skpdump", early_parse_skpdump_cmdline);

static int early_parse_himntn_cmdline(char *p)
{
	int ret;

	if (p == NULL)
		return -1;

	ret = sscanf_s(p, "%llx", &g_himntn_value);
	if (ret != 1) {
		pr_err("sscanf parse error\n");
		return -1;
	}

	pr_err("g_himntn_value is 0x%llx\n", g_himntn_value);
}

early_param("HIMNTN", early_parse_himntn_cmdline);

early_initcall(kernel_dump_init);

