/*
 *  Copyright (C)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM cpufreq_schedutil

#if !defined(_TRACE_CPUFREQ_SCHEDUTIL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CPUFREQ_SCHEDUTIL_H

#include <linux/tracepoint.h>

TRACE_EVENT(cpufreq_schedutil_boost,
	    TP_PROTO(const char *s),
	    TP_ARGS(s),
	    TP_STRUCT__entry(
		    __string(s, s)
	    ),
	    TP_fast_assign(
		    __assign_str(s, s);
	    ),
	    TP_printk("%s", __get_str(s))
);

TRACE_EVENT(cpufreq_schedutil_unboost,
	    TP_PROTO(const char *s),
	    TP_ARGS(s),
	    TP_STRUCT__entry(
		    __string(s, s)
	    ),
	    TP_fast_assign(
		    __assign_str(s, s);
	    ),
	    TP_printk("%s", __get_str(s))
);

TRACE_EVENT(cpufreq_schedutil_eval_target,
	    TP_PROTO(unsigned int cpu,
		     unsigned long util,
		     unsigned long rtg_util,
		     unsigned long cap,
		     unsigned long max,
		     unsigned int load,
		     unsigned int curr,
		     unsigned int iowait_boost,
		     unsigned int rtg_freq,
		     unsigned int min_util_freq,
		     unsigned int target),
	    TP_ARGS(cpu, util, rtg_util, cap, max, load, curr, iowait_boost,
		    rtg_freq, min_util_freq, target),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned long,	util)
		    __field(unsigned long,	rtg_util)
		    __field(unsigned long,	cap)
		    __field(unsigned long,	max)
		    __field(unsigned int,	load)
		    __field(unsigned int,	curr)
		    __field(unsigned int,	iowait_boost)
		    __field(unsigned int,	rtg_freq)
		    __field(unsigned int,	min_util_freq)
		    __field(unsigned int,	target)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->util = util;
		    __entry->rtg_util = rtg_util;
		    __entry->cap = cap;
		    __entry->max = max;
		    __entry->load = load;
		    __entry->curr = curr;
		    __entry->iowait_boost = iowait_boost;
		    __entry->rtg_freq = rtg_freq;
		    __entry->min_util_freq = min_util_freq;
		    __entry->target = target;
	    ),
	    TP_printk("cpu=%u util=%lu rtg_util=%lu cap=%lu max=%lu load=%u "
		      "cur=%u iowait_boost=%u, rtg_freq=%u, min_util_freq=%u target=%u",
		      __entry->cpu, __entry->util, __entry->rtg_util,
		      __entry->cap, __entry->max, __entry->load, __entry->curr,
		      __entry->iowait_boost, __entry->rtg_freq,
		      __entry->min_util_freq, __entry->target)
);

TRACE_EVENT(cpufreq_schedutil_get_util,
	    TP_PROTO(unsigned int cpu,
		     unsigned long util,
		     unsigned long max,
		     unsigned long cpu_util,
		     unsigned long top,
		     unsigned long pred,
		     unsigned long max_pred,
		     unsigned long cpu_pred,
		     unsigned int iowait,
		     unsigned int flag,
		     unsigned int ed,
		     unsigned int od),
	    TP_ARGS(cpu, util, max, cpu_util, top, pred, max_pred, cpu_pred,
		    iowait, flag, ed, od),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned long,	util)
		    __field(unsigned long,	max)
		    __field(unsigned long,	cpu_util)
		    __field(unsigned long,	top)
		    __field(unsigned long,	pred)
		    __field(unsigned long,	max_pred)
		    __field(unsigned long,	cpu_pred)
		    __field(unsigned int,	iowait)
		    __field(unsigned int,	flag)
		    __field(unsigned int,	ed)
		    __field(unsigned int,	od)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->util = util;
		    __entry->max = max;
		    __entry->cpu_util = cpu_util;
		    __entry->top = top;
		    __entry->pred = pred;
		    __entry->max_pred = max_pred;
		    __entry->cpu_pred = cpu_pred;
		    __entry->iowait = iowait;
		    __entry->flag = flag;
		    __entry->ed = ed;
		    __entry->od = od;
	    ),
	    TP_printk("cpu=%u util=%lu max=%lu cpu_util=%lu top=%lu "
		      "pred=%lu,%lu,%lu iowait=%u flag=%u ed=%u od=%u",
		      __entry->cpu, __entry->util, __entry->max,
		      __entry->cpu_util, __entry->top, __entry->pred,
		      __entry->max_pred, __entry->cpu_pred,
		      __entry->iowait, __entry->flag, __entry->ed, __entry->od)
);

TRACE_EVENT(cpufreq_schedutil_notyet,
	    TP_PROTO(unsigned int cpu,
		     const char *reason,
		     unsigned long long delta,
		     unsigned int curr,
		     unsigned int target),
	    TP_ARGS(cpu, reason, delta, curr, target),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __string(reason, reason)
		    __field(unsigned long long,	delta)
		    __field(unsigned int,	curr)
		    __field(unsigned int,	target)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __assign_str(reason, reason);
		    __entry->delta = delta;
		    __entry->curr = curr;
		    __entry->target = target;
	    ),
	    TP_printk("cpu=%u reason=%s delta_ns=%llu curr=%u target=%u",
		      __entry->cpu, __get_str(reason),
		      __entry->delta, __entry->curr, __entry->target)
);

TRACE_EVENT(cpufreq_schedutil_already,
	    TP_PROTO(unsigned int cpu,
		     unsigned int curr,
		     unsigned int target),
	    TP_ARGS(cpu, curr, target),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	curr)
		    __field(unsigned int,	target)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->curr = curr;
		    __entry->target = target;
	    ),
	    TP_printk("cpu=%u curr=%u target=%u",
		      __entry->cpu, __entry->curr, __entry->target)
);

TRACE_EVENT(cpufreq_schedutil_choose_freq,
	    TP_PROTO(unsigned int tl,
		     unsigned int loadadjfreq,
		     unsigned int freq),
	    TP_ARGS(tl, loadadjfreq, freq),
	    TP_STRUCT__entry(
		    __field(unsigned int, tl)
		    __field(unsigned int, loadadjfreq)
		    __field(unsigned int, freq)
	    ),
	    TP_fast_assign(
		    __entry->tl = tl;
		    __entry->loadadjfreq =loadadjfreq;
		    __entry->freq =freq;
	    ),
	    TP_printk("tl=%u, loadadjfreq=%u, freq=%u", __entry->tl,
		      __entry->loadadjfreq, __entry->freq)
);

#ifdef CONFIG_HW_CPU_FREQ
TRACE_EVENT(cpufreq_hw_target_index,
	    TP_PROTO(unsigned int cpu,
		     unsigned int cur,
		     unsigned int target,
		     unsigned int table_freq,
		     unsigned int real_freq,
		     unsigned int delay,
		     unsigned int changed,
		     unsigned int *freqs),
	    TP_ARGS(cpu, cur, target, table_freq, real_freq,
		    delay, changed, freqs),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	cur)
		    __field(unsigned int,	target)
		    __field(unsigned int,	table_freq)
		    __field(unsigned int,	real_freq)
		    __field(unsigned int,	delay)
		    __field(unsigned int,	changed)
		    __dynamic_array(unsigned int,   freqs, delay)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->cur = cur;
		    __entry->target = target;
		    __entry->table_freq = table_freq;
		    __entry->real_freq = real_freq;
		    __entry->delay = delay;
		    __entry->changed = changed;
		    memcpy(__get_dynamic_array(freqs), freqs, delay * sizeof(unsigned int));
	    ),
	    TP_printk("cpu=%u cur=%u target=%u table_freq=%u real_freq=%u " \
		      "delay=%u changed=%u freqs=%s",
		      __entry->cpu, __entry->cur, __entry->target,
		      __entry->table_freq, __entry->real_freq,
		      __entry->delay, __entry->changed,
		      __print_array(__get_dynamic_array(freqs), __entry->delay, sizeof(unsigned int)))
);

TRACE_EVENT(cpufreq_hw_min_max_ipi,
	    TP_PROTO(unsigned int cpu,
		     unsigned int cur,
		     unsigned int table_freq,
		     unsigned int cur_min,
		     unsigned int target_min,
		     unsigned int cur_max,
		     unsigned int target_max),
	    TP_ARGS(cpu, cur, table_freq, cur_min, target_min,
		    cur_max, target_max),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	cur)
		    __field(unsigned int,	table_freq)
		    __field(unsigned int,	cur_min)
		    __field(unsigned int,	target_min)
		    __field(unsigned int,	cur_max)
		    __field(unsigned int,	target_max)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->cur = cur;
		    __entry->table_freq = table_freq;
		    __entry->cur_min = cur_min;
		    __entry->target_min = target_min;
		    __entry->cur_max = cur_max;
		    __entry->target_max = target_max;
	    ),
	    TP_printk("cpu=%u cur=%u table_freq=%u cur_min=%u " \
		      "target_min=%u cur_max=%u target_max=%u",
		      __entry->cpu, __entry->cur, __entry->table_freq,
		      __entry->cur_min, __entry->target_min,
		      __entry->cur_max, __entry->target_max)
);

TRACE_EVENT(cpufreq_hw_out_of_sync,
	    TP_PROTO(unsigned int cpu,
		     unsigned int cur,
		     unsigned int target),
	    TP_ARGS(cpu, cur, target),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	cur)
		    __field(unsigned int,	target)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->cur = cur;
		    __entry->target = target;
	    ),
	    TP_printk("cpu=%u cur=%u target=%u",
		      __entry->cpu, __entry->cur, __entry->target)
);

TRACE_EVENT(cpufreq_hw_set_min_max,
	    TP_PROTO(struct task_struct *tsk,
		     unsigned int cluster,
		     unsigned int min,
		     unsigned int max),
	    TP_ARGS(tsk, cluster, min, max),
	    TP_STRUCT__entry(
		    __array(char,  comm,   TASK_COMM_LEN)
		    __field(unsigned int,	cluster)
		    __field(unsigned int,	min)
		    __field(unsigned int,	max)
	    ),
	    TP_fast_assign(
		    memcpy(__entry->comm, tsk->comm, TASK_COMM_LEN);
		    __entry->cluster = cluster;
		    __entry->min = min;
		    __entry->max = max;
	    ),
	    TP_printk("comm=%s cluster=%u min=%u max=%u", __entry->comm,
		      __entry->cluster, __entry->min, __entry->max)
);

TRACE_EVENT(cpufreq_limit_sspm_thermal,
	    TP_PROTO(unsigned int cpu,
		     unsigned int max_idx,
		     unsigned int max),
	    TP_ARGS(cpu, max_idx, max),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	max_idx)
		    __field(unsigned int,	max)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->max_idx = max_idx;
		    __entry->max = max;
	    ),
	    TP_printk("cpu=%u max_idx=%d max=%d",
		      __entry->cpu, __entry->max_idx, __entry->max)
);

TRACE_EVENT(ppm_dlpt_limit,
	    TP_PROTO(unsigned int cpu,
		     unsigned int max_idx,
		     unsigned int max),
	    TP_ARGS(cpu, max_idx, max),
	    TP_STRUCT__entry(
		    __field(unsigned int,	cpu)
		    __field(unsigned int,	max_idx)
		    __field(unsigned int,	max)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->max_idx = max_idx;
		    __entry->max = max;
	    ),
	    TP_printk("cpu=%u max_idx=%d max=%d",
		      __entry->cpu, __entry->max_idx, __entry->max)
);
#endif /* CONFIG_HW_CPU_FREQ */
#endif /* _TRACE_CPUFREQ_SCHEDUTIL_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
