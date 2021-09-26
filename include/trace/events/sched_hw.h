/*
 * Tracepoint for schedtune_freqboost
 */
TRACE_EVENT(sched_tune_freqboost,

	TP_PROTO(const char *name, int freq_boost),

	TP_ARGS(name, freq_boost),

	TP_STRUCT__entry(
		__array(char,	name,	TASK_COMM_LEN)
		__field(int,		freq_boost)
	),

	TP_fast_assign(
		memcpy(__entry->name, name, TASK_COMM_LEN);
		__entry->freq_boost	= freq_boost;
	),

	TP_printk("name=%s freq_boost=%d", __entry->name, __entry->freq_boost)
);

/*
 * Tracepoint for schedtune_freqboostgroup_update
 */
TRACE_EVENT(sched_tune_freqboostgroup_update,

	TP_PROTO(int cpu, int variation, int max_boost),

	TP_ARGS(cpu, variation, max_boost),

	TP_STRUCT__entry(
		__field(int,	cpu)
		__field(int,	variation)
		__field(int,	max_boost)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->variation	= variation;
		__entry->max_boost	= max_boost;
	),

	TP_printk("cpu=%d variation=%d max_freq_boost=%d",
		__entry->cpu, __entry->variation, __entry->max_boost)
);

TRACE_EVENT(walt_window_rollover,

	TP_PROTO(int cpu),

	TP_ARGS(cpu),

	TP_STRUCT__entry(
		__field(int,	cpu)
	),

	TP_fast_assign(
		__entry->cpu	= cpu;
	),

	TP_printk("cpu=%d", __entry->cpu)
);


#ifdef CONFIG_HW_EAS_TRACE

extern unsigned long cpu_util(int cpu);

TRACE_EVENT(sched_cpu_util,

	TP_PROTO(int cpu, long new_util, u64 irqload, int high_irqload,
		 unsigned int capacity_curr, unsigned int capacity,
		 unsigned int capacity_orig, unsigned int capacity_min),

	TP_ARGS(cpu, new_util, irqload, high_irqload, capacity_curr,
		capacity, capacity_orig, capacity_min),

	TP_STRUCT__entry(
		__field(unsigned int, cpu)
		__field(unsigned int, nr_running)
		__field(long, cpu_util)
		__field(long, new_util)
		__field(unsigned int, capacity_curr)
		__field(unsigned int, capacity)
		__field(unsigned int, capacity_orig)
		__field(unsigned int, capacity_min)
		__field(int, idle_state)
		__field(u64, irqload)
		__field(int, online)
		__field(int, isolated)
		__field(int, reserved)
		__field(int, high_irqload)
	),

	TP_fast_assign(
		__entry->cpu			= cpu;
		__entry->nr_running		= cpu_rq(cpu)->nr_running;
		__entry->cpu_util		= cpu_util(cpu);
		__entry->new_util		= new_util;
		__entry->capacity_curr		= capacity_curr;
		__entry->capacity		= capacity;
		__entry->capacity_orig		= capacity_orig;
		__entry->capacity_min		= capacity_min;
		__entry->idle_state		= idle_get_state_idx(cpu_rq(cpu));
		__entry->irqload		= irqload;
		__entry->online			= cpu_online(cpu);
		__entry->isolated		= cpu_isolated(cpu);
		__entry->reserved		= is_reserved(cpu);
		__entry->high_irqload		= high_irqload;
	),

	TP_printk("cpu=%d nr_running=%d cpu_util=%ld new_util=%ld"
		  " capacity_curr=%u capacity=%u"
		  " capacity_orig=%u capacity_min=%u"
		  " idle_state=%d irqload=%llu online=%d isolated=%d"
		  " reserved=%d high_irq_load=%d",
		  __entry->cpu, __entry->nr_running, __entry->cpu_util,
		  __entry->new_util, __entry->capacity_curr,
		  __entry->capacity, __entry->capacity_orig,
		  __entry->capacity_min, __entry->idle_state,
		  __entry->irqload, __entry->online, __entry->isolated,
		  __entry->reserved, __entry->high_irqload)
);

/*
 * Tracepoint for eas_store
 */
TRACE_EVENT(eas_attr_store,

	TP_PROTO(const char *name, int value),

	TP_ARGS(name, value),

	TP_STRUCT__entry(
		__array( char,	name,	TASK_COMM_LEN	)
		__field( int,		value		)
	),

	TP_fast_assign(
		memcpy(__entry->name, name, TASK_COMM_LEN);
		__entry->value		= value;
	),

	TP_printk("name=%s value=%d", __entry->name, __entry->value)
);

/*
 * Tracepoint for boost_write
 */
TRACE_EVENT(sched_tune_boost,

	TP_PROTO(const char *name, int boost),

	TP_ARGS(name, boost),

	TP_STRUCT__entry(
		__array( char,	name,	TASK_COMM_LEN	)
		__field( int,		boost		)
	),

	TP_fast_assign(
		memcpy(__entry->name, name, TASK_COMM_LEN);
		__entry->boost		= boost;
	),

	TP_printk("name=%s boost=%d", __entry->name, __entry->boost)
);

/*
 * Tracepoint for sched group energy
 */
TRACE_EVENT(sched_group_energy,

	TP_PROTO(int cpu, const struct cpumask *cpus,
		 int idle_idx, int cap_idx, unsigned long util,
		 int busy_energy, int idle_energy),

	TP_ARGS(cpu, cpus, idle_idx, cap_idx, util, busy_energy, idle_energy),

	TP_STRUCT__entry(
		__field( int,	cpu	)
		__bitmask(cpumask, num_possible_cpus())
		__field( int,	idle_idx	)
		__field( int,	cap_idx	)
		__field( unsigned long,	group_util	)
		__field( int,	busy_energy	)
		__field( int,	idle_energy	)
	),

	TP_fast_assign(
		__entry->cpu	= cpu;
		__assign_bitmask(cpumask, cpumask_bits(cpus),
				num_possible_cpus());
		__entry->idle_idx	= idle_idx;
		__entry->cap_idx	= cap_idx;
		__entry->group_util	= util;
		__entry->busy_energy	= busy_energy;
		__entry->idle_energy	= idle_energy;
	),

	TP_printk("cpu=%d sg_cpus=%s idle_idx=%d cap_idx=%d group_util=%lu sg_busy_energy=%d sg_idle_energy=%d",
		__entry->cpu, __get_bitmask(cpumask), __entry->idle_idx,
		__entry->cap_idx, __entry->group_util, __entry->busy_energy, __entry->idle_energy)
);

#endif

#if defined(CONFIG_HW_EAS_TRACE) || defined(CONFIG_HW_PERFHUB_TRACE)
/*
 * Tracepoint for sched_setaffinity
 */
TRACE_EVENT(sched_set_affinity,

	TP_PROTO(struct task_struct *p, const struct cpumask *mask),

	TP_ARGS(p, mask),

	TP_STRUCT__entry(
		__array(   char,	comm,	TASK_COMM_LEN	)
		__field(   pid_t,	pid			)
		__bitmask( cpus,	num_possible_cpus()	)
	),

	TP_fast_assign(
		__entry->pid = p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__assign_bitmask(cpus, cpumask_bits(mask), num_possible_cpus());
	),

	TP_printk("comm=%s pid=%d cpus=%s",
		__entry->comm, __entry->pid, __get_bitmask(cpus))
);

#endif
