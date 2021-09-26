/* SPDX-License-Identifier: GPL-2.0 */
/*
 * rtg.h
 *
 * rtg sched trace events
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


#ifdef CONFIG_HW_RTG
#include <securec.h>

/* note msg size is less than TASK_COMM_LEN */
TRACE_EVENT(find_rtg_cpu,

	TP_PROTO(struct task_struct *p, const struct cpumask *perferred_cpumask, char *msg, int cpu),

	TP_ARGS(p, perferred_cpumask, msg, cpu),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__bitmask(cpus,	num_possible_cpus())
		__array(char, msg, TASK_COMM_LEN)
		__field(int, cpu)
	),

	TP_fast_assign(
		__entry->pid = p->pid;
		memcpy_s(__entry->comm, TASK_COMM_LEN, p->comm, TASK_COMM_LEN);
		__assign_bitmask(cpus, cpumask_bits(perferred_cpumask), num_possible_cpus());
		memcpy_s(__entry->msg, TASK_COMM_LEN, msg, min((size_t)TASK_COMM_LEN, strlen(msg)+1));
		__entry->cpu = cpu;
	),

	TP_printk("comm=%s pid=%d perferred_cpus=%s reason=%s target_cpu=%d",
		__entry->comm, __entry->pid, __get_bitmask(cpus), __entry->msg, __entry->cpu)
);

TRACE_EVENT(sched_rtg_task_each,

	TP_PROTO(unsigned int id, unsigned int nr_running, struct task_struct *task),

	TP_ARGS(id, nr_running, task),

	TP_STRUCT__entry(
		__field( unsigned int,	id		)
		__field( unsigned int,	nr_running	)
		__array( char,	comm,	TASK_COMM_LEN	)
		__field( pid_t,		pid		)
		__field( int,		prio		)
		__bitmask(allowed, num_possible_cpus()	)
		__field( int,		cpu		)
		__field( int,		state		)
		__field( bool,		on_rq		)
		__field( int,		on_cpu		)
	),

	TP_fast_assign(
		__entry->id		= id;
		__entry->nr_running	= nr_running;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid		= task->pid;
		__entry->prio		= task->prio;
		__assign_bitmask(allowed, cpumask_bits(&task->cpus_allowed), num_possible_cpus());
		__entry->cpu		= task_cpu(task);
		__entry->state		= task->state;
		__entry->on_rq		= task->on_rq;
		__entry->on_cpu		= task->on_cpu;
	),

	TP_printk("comm=%s pid=%d prio=%d allowed=%s cpu=%d state=%s%s on_rq=%d on_cpu=%d",
		__entry->comm, __entry->pid, __entry->prio, __get_bitmask(allowed), __entry->cpu,
		__entry->state & (TASK_REPORT_MAX) ?
		__print_flags(__entry->state & (TASK_REPORT_MAX), "|",
				{ TASK_INTERRUPTIBLE, "S" },
				{ TASK_UNINTERRUPTIBLE, "D" },
				{ __TASK_STOPPED, "T" },
				{ __TASK_TRACED, "t" },
				{ EXIT_DEAD, "X" },
				{ EXIT_ZOMBIE, "Z" },
				{ TASK_DEAD, "x" },
				{ TASK_WAKEKILL, "K"},
				{ TASK_WAKING, "W"}) : "R",
		__entry->state & TASK_STATE_MAX ? "+" : "",
		__entry->on_rq, __entry->on_cpu)
);

TRACE_EVENT(sched_rtg_valid_normalized_util,

	TP_PROTO(unsigned int id, unsigned int nr_running,
		 const struct cpumask *rtg_cpus, unsigned int valid),

	TP_ARGS(id, nr_running, rtg_cpus, valid),

	TP_STRUCT__entry(
		__field( unsigned int,	id		)
		__field(unsigned int,	nr_running	)
		__bitmask(cpus,	num_possible_cpus()	)
		__field( unsigned int,	valid		)
	),

	TP_fast_assign(
		__entry->id		= id;
		__entry->nr_running	= nr_running;
		__assign_bitmask(cpus, cpumask_bits(rtg_cpus), num_possible_cpus());
		__entry->valid		= valid;
	),

	TP_printk("id=%d nr_running=%d cpus=%s valid=%d",
		__entry->id, __entry->nr_running,
		__get_bitmask(cpus), __entry->valid)
);

#ifdef CONFIG_HW_CGROUP_RTG
TRACE_EVENT(rtg_update_preferred_cluster,

	TP_PROTO(struct related_thread_group *grp, u64 total_demand),

	TP_ARGS(grp, total_demand),

	TP_STRUCT__entry(
		__field(int,		id)
		__field(u64,		total_demand)
		__field(bool,		skip_min)
	),

	TP_fast_assign(
		__entry->id			= grp->id;
		__entry->total_demand		= total_demand;
		__entry->skip_min		= grp->skip_min;
	),

	TP_printk("group_id %d total_demand %llu skip_min %d",
			__entry->id, __entry->total_demand,
			__entry->skip_min)
);
#endif

#endif

#ifdef CONFIG_HW_RTG_FRAME
/*
 * Tracepoint for rtg sched
 */
DECLARE_EVENT_CLASS(sched_rtg_template,

	TP_PROTO(struct task_struct *p, struct task_struct *from, int type, int op),

	TP_ARGS(p, from, type, op),

	TP_STRUCT__entry(
		__array(char, comm, TASK_COMM_LEN)
		__field(pid_t, pid)
		__field(int, target_cpu)
		__field(int, prio)
#ifdef CONFIG_HUAWEI_VIP_COMMON
		__field(int, vip_prio)
#endif
		__field(int, min_util)
		__field(pid_t, trans_from)
		__field(int, from_prio)
#ifdef CONFIG_HUAWEI_VIP_COMMON
		__field(int, from_vip_prio)
#endif
		__field(int, from_min_util)
		__field(int, trans_type)
		__field(int, op)
	),

	TP_fast_assign(
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->pid		= p->pid;
		__entry->target_cpu	= task_cpu(p);
		__entry->prio		= p->prio;
#ifdef CONFIG_HUAWEI_VIP_COMMON
		__entry->vip_prio	= p->vip_prio;
#endif
#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
		__entry->min_util	= p->util_req.min_util;
#else
		__entry->min_util	= -1;
#endif
		__entry->trans_from	= (from == NULL) ? 0 : from->pid;
		__entry->from_prio	= (from == NULL) ? 0 : from->prio;
#ifdef CONFIG_HUAWEI_VIP_COMMON
		__entry->from_vip_prio	= (from == NULL) ? 0 : from->vip_prio;
#endif
#ifdef CONFIG_SCHED_TASK_UTIL_CLAMP
		__entry->from_min_util	= (from == NULL) ? 0 : from->util_req.min_util;
#else
		__entry->from_min_util	= -1;
#endif
		__entry->trans_type	= type;
		__entry->op		= op;
	),

#ifdef CONFIG_HUAWEI_VIP_COMMON
	TP_printk("comm=%s pid=%d target_cpu=%03d prio=%d vip_prio=%d"
		" min_util=%d trans_from=%d from_prio=%d from_vip_prio=%d"
		" from_min_util=%d trans_type=%d op=%d",
		__entry->comm, __entry->pid, __entry->target_cpu,
		__entry->prio, __entry->vip_prio,
		__entry->min_util, __entry->trans_from,
		__entry->from_prio, __entry->from_vip_prio,
		__entry->from_min_util, __entry->trans_type, __entry->op)
#else
	TP_printk("comm=%s pid=%d target_cpu=%03d prio=%d"
		" min_util=%d trans_from=%d from_prio=%d"
		" from_min_util=%d trans_type=%d op=%d",
		__entry->comm, __entry->pid, __entry->target_cpu,
		__entry->prio, __entry->min_util, __entry->trans_from,
		__entry->from_prio, __entry->from_min_util,
		__entry->trans_type, __entry->op)
#endif
);

DEFINE_EVENT(sched_rtg_template, sched_rtg,
	TP_PROTO(struct task_struct *p, struct task_struct *from, int type, int op),
	TP_ARGS(p, from, type, op));

#endif
