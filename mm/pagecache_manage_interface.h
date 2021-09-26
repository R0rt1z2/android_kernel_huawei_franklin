#ifndef __HUAWEI_PCH_MANAGE_INTERFACE_H__
#define __HUAWEI_PCH_MANAGE_INTERFACE_H__
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/version.h>

extern void __cfi_pch_timeout_work(struct work_struct *work);
extern void pch_timeout_work(struct work_struct *work);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0))
extern void __cfi_pch_timeout_timer(unsigned long data);
extern void pch_timeout_timer(unsigned long data);
#else
extern void __cfi_pch_timeout_timer(struct timer_list *data);
extern void pch_timeout_timer(struct timer_list *data);
#endif

extern int __cfi_pch_emui_bootstat_proc_show(struct seq_file *m, void *v);
extern int pch_emui_bootstat_proc_show(struct seq_file *m, void *v);
extern int __cfi_pch_emui_bootstat_proc_open(struct inode *p_inode, struct file *p_file);
extern int pch_emui_bootstat_proc_open(struct inode *p_inode, struct file *p_file);
extern ssize_t __cfi_pch_emui_bootstat_proc_write(struct file *p_file,
		const char __user *userbuf, size_t count, loff_t *ppos);
extern ssize_t pch_emui_bootstat_proc_write(struct file *p_file,
		const char __user *userbuf, size_t count, loff_t *ppos);
extern int __init __cfi_huawei_page_cache_manage_init(void);
extern int __init huawei_page_cache_manage_init(void);
extern void __exit __cfi_huawei_page_cache_manage_exit(void);
extern void __exit huawei_page_cache_manage_exit(void);

#endif /* __HUAWEI_PCH_MANAGE_INTERFACE_H__ */

