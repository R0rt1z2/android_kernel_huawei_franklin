/*
 * rtg.h
 *
 * related thread group trace events
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
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__assign_bitmask(cpus, cpumask_bits(perferred_cpumask), num_possible_cpus());
		memcpy(__entry->msg, msg, min((size_t)TASK_COMM_LEN, strlen(msg) + 1));
		__entry->cpu = cpu;
	),

	TP_printk("comm=%s pid=%d perferred_cpus=%s reason=%s target_cpu=%d",
		__entry->comm, __entry->pid, __get_bitmask(cpus), __entry->msg, __entry->cpu)
);
