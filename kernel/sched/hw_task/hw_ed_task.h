/*
 * hw_ed_task.h
 *
 * huawei early detection task header file
 *
 * Copyright (c) 2019, Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HW_ED_TASK_H__
#define __HW_ED_TASK_H__

#ifdef CONFIG_HW_ED_TASK
#define EARLY_DETECTION_TASK_WAITING_DURATION 11500000
#define EARLY_DETECTION_TASK_RUNNING_DURATION 120000000
#define EARLY_DETECTION_NEW_TASK_RUNNING_DURATION 100000000
bool early_detection_notify(struct rq *rq, u64 wallclock);
void clear_ed_task(struct task_struct *p, struct rq *rq);
void migrate_ed_task(struct task_struct *p, struct rq *src_rq,
		     struct rq *dest_rq, u64 wallclock);
void note_task_waking(struct task_struct *p, u64 wallclock);
#else
bool early_detection_notify(struct rq *rq, u64 wallclock);
{
	return false;
}
void clear_ed_task(struct task_struct *p, struct rq *rq)
{
}
void migrate_ed_task(struct task_struct *p, struct rq *src_rq,
		     struct rq *dest_rq, u64 wallclock)
{
}
void note_task_waking(struct task_struct *p, u64 wallclock)
{
}
#endif

#endif
