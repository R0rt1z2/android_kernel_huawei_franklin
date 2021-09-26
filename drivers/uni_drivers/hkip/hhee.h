/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2019. All rights reserved.
 * Description: hhee internal header
 * Creator: security-ap
 * Date: 2017/2/1
 */

#ifndef __HKIP_HHEE_H__
#define __HKIP_HHEE_H__

#include <asm/compiler.h>
#include <linux/arm-smccc.h>
#include <linux/interrupt.h>
#include <uni/hkip/hkip_hvc.h>

/*struct for logbuf*/
struct circular_buffer {
	unsigned long size; /* Indicates the total size of the buffer */
	unsigned long start; /* Starting point of valid data in buffer */
	unsigned long end; /* First character which is empty (can be written to) */
	unsigned long overflow; /* Indicator whether buffer has overwritten itself */
	unsigned long virt_log_addr; /*Indicator the virtual addr of buffer*/
	unsigned long virt_log_size; /*Indicator the max size of buffer*/
	unsigned int inited;        /*Indicator the status of buffer*/
	unsigned int logtype;       /*Indicator the type of buffer*/
	char *buf;
};

/*enum for logtype*/
enum ltype {
	CRASH_LOG,
	PMF_LOG,
};

irqreturn_t hhee_irq_handle(int irq, void *data);
int hhee_logger_init(void);
#ifdef CONFIG_HUAWEI_HKIP_DEBUG
int hhee_init_debugfs(void);
void hhee_cleanup_debugfs(void);
#endif
#ifdef CONFIG_HKIP_MODULE_ALLOC
void hhee_module_init(void);
#else
static inline void hhee_module_init(void){};
#endif
void reset_hkip_irq_counters(void);

ssize_t hhee_copy_logs(char __user *buf, size_t count,
		       loff_t *offp, int logtpye);
#endif
