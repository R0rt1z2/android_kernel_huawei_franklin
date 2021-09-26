/*
 * huawei_eas.h
 *
 * huawei eas trace events
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

/*
 * Tracepoint for find busiest group
 */
TRACE_EVENT(sched_find_busiest_group,

	TP_PROTO(int idle, int group_type, const struct cpumask *cpus,
		 unsigned int fbg_flag),

	TP_ARGS(idle, group_type, cpus, fbg_flag),

	TP_STRUCT__entry(
		__field(int,		idle)
		__field(int,		group_type)
		__bitmask(cpumask,	num_possible_cpus())
		__field(unsigned int,	fbg_flag)
	),

	TP_fast_assign(
		__entry->idle		= idle;
		__entry->group_type	= group_type;
		__assign_bitmask(cpumask, cpumask_bits(cpus),
				 num_possible_cpus());
		__entry->fbg_flag	= fbg_flag;
	),

	TP_printk("idle=%d group_type=%d cpus=%s flag=%#x",
		  __entry->idle, __entry->group_type,
		  __get_bitmask(cpumask),
		  __entry->fbg_flag)
);

/*
 * Tracepoint for find new ilb
 */
TRACE_EVENT(sched_huawei_find_new_ilb,

	TP_PROTO(int type, int ilb),

	TP_ARGS(type, ilb),

	TP_STRUCT__entry(
		__field(int, type)
		__field(int, ilb)
	),

	TP_fast_assign(
		__entry->type	= type;
		__entry->ilb	= ilb;
	),

	TP_printk("type=%d ilb=%d", __entry->type, __entry->ilb)
);

/*
 * Tracepoint for load balancing:
 */
#if NR_CPUS > 32
#error "Unsupported NR_CPUS for lb tracepoint."
#endif
TRACE_EVENT(sched_load_balance,

	TP_PROTO(int cpu, int idle, int balance,
		unsigned long group_mask, int busiest_nr_running,
		long imbalance, unsigned int env_flags, int ld_moved,
		unsigned int balance_interval, int active_balance),

	TP_ARGS(cpu, idle, balance, group_mask, busiest_nr_running,
		imbalance, env_flags, ld_moved, balance_interval,
		active_balance),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(int,		idle)
		__field(int,		balance)
		__field(unsigned long,	group_mask)
		__field(int,		busiest_nr_running)
		__field(long,		imbalance)
		__field(unsigned int,	env_flags)
		__field(int,		ld_moved)
		__field(unsigned int,	balance_interval)
		__field(int,		active_balance)
	),

	TP_fast_assign(
		__entry->cpu                    = cpu;
		__entry->idle                   = idle;
		__entry->balance                = balance;
		__entry->group_mask             = group_mask;
		__entry->busiest_nr_running     = busiest_nr_running;
		__entry->imbalance              = imbalance;
		__entry->env_flags              = env_flags;
		__entry->ld_moved               = ld_moved;
		__entry->balance_interval       = balance_interval;
		__entry->active_balance		= active_balance;
	),

	TP_printk("cpu=%d state=%d balance=%d group=%#lx busy_nr=%d"
		  " imbalance=%ld flags=%#x ld_moved=%d bal_int=%d"
		  " active_balance=%d",
		  __entry->cpu, __entry->idle, __entry->balance,
		  __entry->group_mask, __entry->busiest_nr_running,
		  __entry->imbalance, __entry->env_flags, __entry->ld_moved,
		  __entry->balance_interval, __entry->active_balance)
);


TRACE_EVENT(sched_load_balance_nohz_kick,

	TP_PROTO(int cpu, int kick_cpu, int cpu_overutil),

	TP_ARGS(cpu, kick_cpu, cpu_overutil),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	cpu_nr)
		__field(unsigned long,	misfit_task_load)
		__field(int,		cpu_overutil)
		__field(int,		kick_cpu)
		__field(unsigned long,	nohz_flags)
	),

	TP_fast_assign(
		__entry->cpu			= cpu;
		__entry->cpu_nr			= cpu_rq(cpu)->nr_running;
		__entry->misfit_task_load	= cpu_rq(cpu)->misfit_task_load;
		__entry->cpu_overutil		= cpu_overutil;
		__entry->kick_cpu		= kick_cpu;
		__entry->nohz_flags		= *nohz_flags(kick_cpu);
	),

	TP_printk("cpu=%d nr_run=%u misfit_task_load=%lu overutilized=%d"
		  " kick_cpu=%d nohz_flags=0x%lx",
		  __entry->cpu, __entry->cpu_nr, __entry->misfit_task_load,
		  __entry->cpu_overutil, __entry->kick_cpu,
		  __entry->nohz_flags)

);

TRACE_EVENT(sched_load_balance_sg_stats,

	TP_PROTO(unsigned long sg_cpus, int group_type, unsigned int idle_cpus,
		 unsigned int sum_nr_running, unsigned long group_load,
		 unsigned long group_capacity, unsigned long group_util,
		 int group_no_capacity, unsigned long load_per_task,
		 unsigned long misfit_load, unsigned long busiest),

	TP_ARGS(sg_cpus, group_type, idle_cpus, sum_nr_running, group_load,
		group_capacity, group_util, group_no_capacity, load_per_task,
		misfit_load, busiest),

	TP_STRUCT__entry(
		__field(unsigned long,		group_mask)
		__field(int,			group_type)
		__field(unsigned int,		group_idle_cpus)
		__field(unsigned int,		sum_nr_running)
		__field(unsigned long,		group_load)
		__field(unsigned long,		group_capacity)
		__field(unsigned long,		group_util)
		__field(int,			group_no_capacity)
		__field(unsigned long,		load_per_task)
		__field(unsigned long,		misfit_task_load)
		__field(unsigned long,		busiest)
	),

	TP_fast_assign(
		__entry->group_mask			= sg_cpus;
		__entry->group_type			= group_type;
		__entry->group_idle_cpus		= idle_cpus;
		__entry->sum_nr_running			= sum_nr_running;
		__entry->group_load			= group_load;
		__entry->group_capacity			= group_capacity;
		__entry->group_util			= group_util;
		__entry->group_no_capacity		= group_no_capacity;
		__entry->load_per_task			= load_per_task;
		__entry->misfit_task_load		= misfit_load;
		__entry->busiest			= busiest;
	),

	TP_printk("sched_group=%#lx type=%d idle_cpus=%u sum_nr_run=%u"
		  " group_load=%lu capacity=%lu util=%lu no_capacity=%d"
		  " lpt=%lu misfit_tload=%lu busiest_group=%#lx",
		  __entry->group_mask, __entry->group_type,
		  __entry->group_idle_cpus, __entry->sum_nr_running,
		  __entry->group_load, __entry->group_capacity,
		  __entry->group_util, __entry->group_no_capacity,
		  __entry->load_per_task, __entry->misfit_task_load,
		  __entry->busiest)
);

TRACE_EVENT(sched_load_balance_stats,

	TP_PROTO(unsigned long busiest, int bgroup_type,
		 unsigned long bavg_load, unsigned long bload_per_task,
		 unsigned long local, int lgroup_type,
		 unsigned long lavg_load, unsigned long lload_per_task,
		 unsigned long sds_avg_load, unsigned long imbalance),

	TP_ARGS(busiest, bgroup_type, bavg_load, bload_per_task,
		local, lgroup_type, lavg_load, lload_per_task,
		sds_avg_load, imbalance),

	TP_STRUCT__entry(
		__field(unsigned long,		busiest)
		__field(int,			bgp_type)
		__field(unsigned long,		bavg_load)
		__field(unsigned long,		blpt)
		__field(unsigned long,		local)
		__field(int,			lgp_type)
		__field(unsigned long,		lavg_load)
		__field(unsigned long,		llpt)
		__field(unsigned long,		sds_avg)
		__field(unsigned long,		imbalance)
	),

	TP_fast_assign(
		__entry->busiest			= busiest;
		__entry->bgp_type			= bgroup_type;
		__entry->bavg_load			= bavg_load;
		__entry->blpt				= bload_per_task;
		__entry->bgp_type			= bgroup_type;
		__entry->local				= local;
		__entry->lgp_type			= lgroup_type;
		__entry->lavg_load			= lavg_load;
		__entry->llpt				= lload_per_task;
		__entry->sds_avg			= sds_avg_load;
		__entry->imbalance			= imbalance;
	),

	TP_printk("busiest_group=%#lx busiest_type=%d busiest_avg_load=%ld"
		  " busiest_lpt=%ld local_group=%#lx local_type=%d"
		  " local_avg_load=%ld local_lpt=%ld domain_avg_load=%ld"
		  " imbalance=%ld",
		  __entry->busiest, __entry->bgp_type,
		  __entry->bavg_load, __entry->blpt,
		  __entry->local, __entry->lgp_type,
		  __entry->lavg_load, __entry->llpt,
		  __entry->sds_avg, __entry->imbalance)
);

TRACE_EVENT(sched_load_balance_skip_tasks,

	TP_PROTO(int scpu, int dcpu, int grp_type, int pid,
		 unsigned long h_load, unsigned long task_util,
		 unsigned long affinity),

	TP_ARGS(scpu, dcpu, grp_type, pid, h_load, task_util, affinity),

	TP_STRUCT__entry(
		__field(int,		scpu)
		__field(unsigned long,	src_util_cum)
		__field(int,		grp_type)
		__field(int,		dcpu)
		__field(unsigned long,	dst_util_cum)
		__field(int,		pid)
		__field(unsigned long,	affinity)
		__field(unsigned long,	task_util)
		__field(unsigned long,	h_load)
	),

	TP_fast_assign(
		__entry->scpu			= scpu;
		__entry->src_util_cum		= cpu_util(scpu);
		__entry->grp_type		= grp_type;
		__entry->dcpu			= dcpu;
		__entry->dst_util_cum		= cpu_util(dcpu);
		__entry->pid			= pid;
		__entry->affinity		= affinity;
		__entry->task_util		= task_util;
		__entry->h_load			= h_load;
	),

	TP_printk("source_cpu=%d util=%lu group_type=%d dest_cpu=%d util=%lu"
		  " pid=%d affinity=%#lx task_util=%lu task_h_load=%lu",
		  __entry->scpu, __entry->src_util_cum, __entry->grp_type,
		  __entry->dcpu, __entry->dst_util_cum, __entry->pid,
		  __entry->affinity, __entry->task_util, __entry->h_load)
);

