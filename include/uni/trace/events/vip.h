/*
 * vip.h
 *
 * vip sched trace events
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

#ifdef CONFIG_HUAWEI_SCHED_VIP
TRACE_EVENT(sched_find_vip_cpu_each,

	TP_PROTO(struct task_struct *task, int cpu, int has_vip,
		unsigned int cpu_vip_prio, bool reserve, int high_irq,
		unsigned long cap_orig, unsigned long wake_util,
		int prefer),

	TP_ARGS(task, cpu, has_vip, cpu_vip_prio, reserve, high_irq,
		cap_orig, wake_util, prefer),

	TP_STRUCT__entry(
		__field( pid_t,		pid		)
		__field( unsigned int,	task_vip_prio	)
		__field( int,		cpu		)
		__field( pid_t,		curr		)
		__field( int,		cpus_allowed	)
		__field( int,		has_rt		)
		__field( int,		has_vip		)
		__field( unsigned int,	cpu_vip_prio	)
		__field( bool,		reserve		)
		__field( int,		high_irq	)
		__field( unsigned long,	cap_orig	)
		__field( unsigned long,	wake_util	)
		__field( int,		prefer		)
	),

	TP_fast_assign(
		__entry->pid		= task->pid;
		__entry->task_vip_prio	= task->vip_prio;
		__entry->cpu		= cpu;
		__entry->curr		= cpu_curr(cpu)->pid;
		__entry->cpus_allowed 	= cpu_curr(cpu)->nr_cpus_allowed;
		__entry->has_rt		= cpu_rq(cpu)->rt.rt_nr_running;
		__entry->has_vip	= has_vip;
		__entry->cpu_vip_prio	= cpu_vip_prio;
		__entry->reserve	= reserve;
		__entry->high_irq	= high_irq;
		__entry->cap_orig	= cap_orig;
		__entry->wake_util	= wake_util;
		__entry->prefer		= prefer;
	),

	TP_printk("task=%d(%u) cpu=%d curr=%d(%d) nr_rt=%d vip=%d(%u) reserve=%d "
		  "irq=%d cap=%lu util=%lu prefer=%d",
		__entry->pid, __entry->task_vip_prio, __entry->cpu, __entry->curr,
		__entry->cpus_allowed, __entry->has_rt,
		__entry->has_vip, __entry->cpu_vip_prio, __entry->reserve,
		__entry->high_irq, __entry->cap_orig, __entry->wake_util,
		__entry->prefer)
);

TRACE_EVENT(sched_find_vip_cpu,

	TP_PROTO(struct task_struct *task, int preferred_cpu,
		bool favor_larger_capacity, int target_cpu),

	TP_ARGS(task, preferred_cpu, favor_larger_capacity, target_cpu),

	TP_STRUCT__entry(
		__array( char,	comm,	TASK_COMM_LEN	)
		__field( pid_t,		pid		)
		__field( unsigned int,	vip_prio	)
		__field( int,		preferred_cpu	)
		__field( int,		target_cpu	)
		__field( bool,		favor_larger_capacity	)
	),

	TP_fast_assign(
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid		= task->pid;
		__entry->vip_prio	= task->vip_prio;
		__entry->preferred_cpu	= preferred_cpu;
		__entry->target_cpu	= target_cpu;
		__entry->favor_larger_capacity	= favor_larger_capacity;
	),

	TP_printk("pid=%d comm=%s prio=%u preferred=%d favor_larger=%d target=%d",
		__entry->pid, __entry->comm, __entry->vip_prio,
		__entry->preferred_cpu, __entry->favor_larger_capacity,
		__entry->target_cpu)
);
#endif

TRACE_EVENT(sched_pick_next_vip,

	TP_PROTO(struct task_struct *task),

	TP_ARGS(task),

	TP_STRUCT__entry(
		__array( char,	comm,	TASK_COMM_LEN	)
		__field( pid_t,		pid		)
		__field( unsigned int,	vip_prio	)
	),

	TP_fast_assign(
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid		= task->pid;
		__entry->vip_prio	= task->vip_prio;
	),

	TP_printk("pid=%d comm=%s prio=%u",
		__entry->pid, __entry->comm, __entry->vip_prio)
);
