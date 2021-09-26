/*
 * trans_rtg.h
 *
 * trans rtg thread header
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
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

#ifndef TRANS_RTG_H
#define TRANS_RTG_H

#include <linux/sched.h>
#include <linux/types.h>

#define DEFAULT_TRANS_DEPTH (-100)
#define DEFAULT_MAX_THREADS (-100)

#ifdef CONFIG_HW_RTG_FRAME
void set_trans_config(int depth, int max_threads);
void trans_rtg_sched_enqueue(struct task_struct *task,
	struct task_struct *from, unsigned int type);
void trans_rtg_sched_dequeue(struct task_struct *task, unsigned int type);
#else
void set_trans_config(int depth, int max_threads) { }
void trans_rtg_sched_enqueue(struct task_struct *task,
	struct task_struct *from, unsigned int type) { }
void trans_rtg_sched_dequeue(struct task_struct *task, unsigned int type) { }
#endif

#endif
