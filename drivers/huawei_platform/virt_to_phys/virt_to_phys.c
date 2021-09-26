/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2013-2019. All rights reserved.
 * Description: virtual address to physical address
 * Author: Copyright (C) 2013 Hisilicon
 * Create: 2013-12-22
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/mm_types.h>
#include <linux/rwsem.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/pgtable-hwdef.h>
/*lint -e451*/
#include <asm/current.h>
/*lint +e451*/
#include <linux/thread_info.h>
#include <linux/kernel_stat.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/io.h>
#include <asm/pgalloc.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <linux/sysfs.h>

static unsigned long g_virt_addr;
static unsigned int g_virt_len;
static unsigned int g_pid;

enum {
	ARGS_0,
	ARGS_1,
	ARGS_2,
	ARGS_3,
};

static ssize_t show_phys_pte(char *buf, ssize_t size, struct mm_struct *mm,
	unsigned long addr)
{
	pgd_t *pgd = NULL;
	ssize_t temp = size;

	size += scnprintf(buf + size, PAGE_SIZE - size,
		"pgd = %p\n", mm->pgd);
	pgd = pgd_offset(mm, addr);
	size += scnprintf(buf + size, PAGE_SIZE - size,
		"[%08lx] *pgd=%08llx", addr, (long long)pgd_val(*pgd));

	do {
		pud_t *pud = NULL;
		pmd_t *pmd = NULL;
		pte_t *pte = NULL;

		if (pgd_none(*pgd))
			break;

		if (pgd_bad(*pgd)) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"(bad)");
			break;
		}

		pud = pud_offset(pgd, addr);
		if (PTRS_PER_PUD != 1)
			size += scnprintf(buf + size, PAGE_SIZE - size,
				", *pud=%08llx",
				(long long)pud_val(*pud));

		if (pud_none(*pud))
			break;

		if (pud_bad(*pud)) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"(bad)");
			break;
		}

		pmd = pmd_offset(pud, addr);
		if (PTRS_PER_PMD != 1)
			size += scnprintf(buf + size, PAGE_SIZE - size,
				", *pmd=%08llx",
				(long long)pmd_val(*pmd));

		if (pmd_none(*pmd))
			break;

		if (pmd_bad(*pmd)) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"(bad)");
			break;
		}

		/* We must not map this if we have highmem enabled */
		if (PageHighMem(pfn_to_page(pmd_val(*pmd) >> PAGE_SHIFT)))
			break;

		pte = pte_offset_map(pmd, addr);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			", *pte=%08llx",
			(long long)pte_val(*pte));
		pte_unmap(pte);
	} while (0);

	size += scnprintf(buf + size, PAGE_SIZE - size, "\n");

	return size - temp;
}

static ssize_t show_phys_addr(struct kobject *dev, struct kobj_attribute *attr,
	char *buf)
{
	ssize_t size;
	unsigned long va_addr = g_virt_addr;
	struct task_struct *tgt_task = NULL;

	if (!buf) {
		pr_err("[%s]:%d buf is null\n", __func__, __LINE__);
		return -EINVAL;
	}

	size = scnprintf(buf, PAGE_SIZE, "%u, 0x%lx, %u\n",
		g_pid, g_virt_addr, g_virt_len);

	if (g_pid == 0)
		tgt_task = current;
	else
		tgt_task = pid_task(find_vpid(g_pid), PIDTYPE_PID);

	if (!tgt_task) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"[virt_to_phys] tgt_task is NULL\n");
		return -EINVAL;
	}

	if (!find_vma(tgt_task->mm, va_addr)) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"[virt_to_phys] cannot find vma\n");
		return -EINVAL;
	}
	size += scnprintf(buf + size, PAGE_SIZE - size,
		"[virt_to_phys]:");

	size += show_phys_pte(buf, size, tgt_task->mm, va_addr);

	return size;
}

static ssize_t store_virt_addr(struct kobject *dev, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned int args = 0;
	unsigned int argl;
	unsigned int len = count;
	const char *head = buf;
	char *p = NULL;

	if (!buf) {
		pr_err("[%s]:%d buf is null\n", __func__, __LINE__);
		return -EINVAL;
	}

	while ((len > 0) && (args < ARGS_3)) {
		p = memchr(head, ' ', len);
		argl = p ? (p - head) : 0;

		switch (args) {
		case ARGS_0:
			if (sscanf(head, "%u", &g_pid) != 1)
				return -EINVAL;
			break;
		case ARGS_1:
			if (sscanf(head, "%lx", &g_virt_addr) != 1)
				return -EINVAL;
			break;
		case ARGS_2:
			if (sscanf(head, "%u", &g_virt_len) != 1)
				return -EINVAL;
			break;
		default:
			pr_err("[%s]:%d invalid args\n", __func__, __LINE__);
		}

		head = head + argl + 1;
		len = len - argl - 1;
		args++;
	}

	return count;
}

static struct kobj_attribute g_kvirt_attr = __ATTR(virt_to_phys, 0640,
	show_phys_addr, store_virt_addr);

static int __init virt_to_phys_init(void)
{
	int error = sysfs_create_file(kernel_kobj, &g_kvirt_attr.attr);

	if (error)
		pr_debug("failed to create module virt_to_phys node\n");
	return 0;
}
module_init(virt_to_phys_init);

static void __exit virt_to_phys_exit(void)
{
	sysfs_remove_file(kernel_kobj, &g_kvirt_attr.attr);
}
module_exit(virt_to_phys_exit);

MODULE_AUTHOR("cuiyong1@huawei.com>");
MODULE_DESCRIPTION("VIRT TO PHYS MODULE");
MODULE_LICENSE("GPL v2");