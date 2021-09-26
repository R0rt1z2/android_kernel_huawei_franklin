/*
 * cfi_test.c
 *
 * test the basic function of cfi
 *
 * Copyright (c) 2017-2019 Huawei Technologies Co., Ltd.
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
#include <linux/printk.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/kallsyms.h>

#define ROP_NAME_SIZE 8
#define LIMIT_LINE    40

static struct proc_dir_entry *cfi_entry;
static struct proc_dir_entry *cfi_call_entry;
static struct proc_dir_entry *cfi_attack_entry;
/* roptest_impl and roptest_bad func addr offset */
static long cfi_func_offset;

/* set addr_offset by ecall */
void set_func_offset(unsigned int offset)
{
	cfi_func_offset = offset;
}

static void get_func_offset(void)
{
	unsigned long roptest_impl_addr;
	unsigned long roptest_bad_addr;

	roptest_impl_addr =  kallsyms_lookup_name("roptest_impl");
	roptest_bad_addr =  kallsyms_lookup_name("roptest_bad");
	cfi_func_offset = roptest_impl_addr - roptest_bad_addr;
	pr_err("roptest get_func_offset 0x%lx\n", cfi_func_offset);
}

static noinline void roptest_bad(const char *name)
{
	pr_err("roptest_bad \"%s\"\n", name);
}

static noinline void roptest_good(const char *name)
{
	pr_err("roptest_good \"%s\"\n", name);
}

static void roptest_impl(const char *name)
{
	if (!strncmp(name, "roptest", strlen("roptest")))
		roptest_good(name);
	else
		roptest_bad(name);
}

typedef void (*openfunc) (const char *name);

struct roptest_struct {
	char name[ROP_NAME_SIZE];
	openfunc func;
};

static struct roptest_struct roptest = {
	.name = "roptest",
	.func = roptest_impl,
};

#define RECORD_SIZE 7
struct cfi_violation_record {
	void *callee;
	void *retaddr;
};

struct cfi_violation_record cfi_violations[RECORD_SIZE];

void __ifcc_logger(void *callee)
{
	if (*((uint32_t *)callee - 1) != CONFIG_HUAWEI_CFI_TAG) {
		void *retaddr = __builtin_return_address(0);
		long i = (uintptr_t)callee % RECORD_SIZE;

		cfi_violations[i].callee = callee;
		cfi_violations[i].retaddr = retaddr;
	}
}
EXPORT_SYMBOL(__ifcc_logger);

void __efistub___ifcc_logger(void *callee)
{
	__ifcc_logger(callee);
}
EXPORT_SYMBOL(__efistub___ifcc_logger);

static ssize_t testcall_read(struct file *file, char __user *buf,
			     size_t size, loff_t *ppos)
{
	pr_debug("roptest: hint: roptest.func = %pK\n", roptest.func);
	roptest.func(roptest.name);
	return 0;
}

static ssize_t testattack_read(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	/*
	 * Use objdump or System.map to get the function address.
	 * I use relative address rather than abs addr,
	 * because sometime there's relocation.
	 */
	roptest.func = (openfunc)((char *)roptest_impl - cfi_func_offset);
	return 0;
}

static int nop_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int nop_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations testcall_fops = {
	.open = nop_open,
	.read = testcall_read,
	.release = nop_release,
};

static const struct file_operations testattack_fops = {
	.open = nop_open,
	.read = testattack_read,
	.release = nop_release,
};

static int proccfi_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int proccfi_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t proccfi_read(struct file *file, char __user *buf,
			    size_t size, loff_t *ppos)
{
	char kbuf[RECORD_SIZE * LIMIT_LINE] = {0};
	int len = 0;
	long i;

	if (*ppos != 0)
		return 0;
	if (size < sizeof(kbuf))
		return -ENOMEM;

	for (i = 0; i < RECORD_SIZE; i++) {
		if (cfi_violations[i].callee == NULL)
			continue;
		len += snprintf(kbuf + len, sizeof(kbuf) - len, "%pK %pK\n",
				cfi_violations[i].callee,
				cfi_violations[i].retaddr);
		if ((len >= sizeof(kbuf)) || (len < 0))
			return -ENOMEM;
	}

	if (copy_to_user(buf, kbuf, len))
		return -ENOMEM;
	*ppos += len;
	return len;
}

static const struct file_operations proccfi_fops = {
	.open  = proccfi_open,
	.read  = proccfi_read,
	.release = proccfi_release,
};

static int __init cfi_test_init(void)
{
	get_func_offset();
	cfi_entry = proc_create("cfi", 0444, NULL, &proccfi_fops);
	if (!cfi_entry) {
		pr_err("proc_create cfi fail\n");
		goto free_cfi_entry;
	}

	cfi_call_entry = proc_create("cfitest-call", 0444, NULL,
				     &testcall_fops);
	if (!cfi_call_entry) {
		pr_err("proc_create cfitest-call fail\n");
		goto free_cfi_call_entry;
	}

	cfi_attack_entry = proc_create("cfitest-attack", 0444, NULL,
				       &testattack_fops);
	if (!cfi_attack_entry) {
		pr_err("proc_create cfitest-attack fail\n");
		goto free_cfi_attack_entry;
	}
	return 0;

free_cfi_attack_entry:
	proc_remove(cfi_call_entry);
	cfi_call_entry = NULL;
free_cfi_call_entry:
	proc_remove(cfi_entry);
	cfi_entry = NULL;
free_cfi_entry:
	return -ENOMEM;
}

static void __exit cfi_test_exit(void)
{
	if (cfi_attack_entry) {
		proc_remove(cfi_attack_entry);
		cfi_attack_entry = NULL;
	}

	if (cfi_call_entry) {
		proc_remove(cfi_call_entry);
		cfi_call_entry = NULL;
	}

	if (cfi_entry) {
		proc_remove(cfi_entry);
		cfi_entry = NULL;
	}
}

module_init(cfi_test_init);
module_exit(cfi_test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Test the basic function of cfi");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
