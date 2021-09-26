/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2020. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Provide external call interfaces of Process reclaim information.
 */
#ifndef __PROCESS_RECLAIM_INFO_H
#define __PROCESS_RECLAIM_INFO_H

extern bool process_reclaim_need_abort(struct mm_walk *walk);
extern struct reclaim_result *process_reclaim_result_cache_alloc(gfp_t gfp);
extern void process_reclaim_result_cache_free(struct reclaim_result *result);
extern int process_reclaim_result_read(
	struct seq_file *m, struct pid_namespace *ns,
	struct pid *pid, struct task_struct *tsk);
extern void process_reclaim_result_write(
	struct task_struct *task, unsigned nr_reclaimed,
	unsigned nr_writedblock, s64 elapsed);
#endif /* __PROCESS_RECLAIM_INFO_H */
