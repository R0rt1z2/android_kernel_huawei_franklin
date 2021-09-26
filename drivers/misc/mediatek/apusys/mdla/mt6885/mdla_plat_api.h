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

#ifndef _MDLA_PLAT_API_H_
#define _MDLA_PLAT_API_H_
#include <linux/interrupt.h>
#include <linux/device.h>
void mdla_reset(unsigned int core, int res);
int mdla_zero_skip_detect(unsigned int core_id);
int mdla_run_command_codebuf_check(struct command_entry *ce);

int mdla_dts_map(struct platform_device *pdev);
void mdla_dump_reg(int core_id);

void mdla_del_free_command(struct command_entry *ce);
void mdla_split_command(struct command_entry *ce);
void mdla_clear_swcmd_wait_bit(void *base_kva, u32 cid);
irqreturn_t mdla_scheduler(unsigned int core_id);
#endif
