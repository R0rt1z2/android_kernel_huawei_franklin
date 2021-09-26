TRACE_EVENT(IPA_boost,
	TP_PROTO(pid_t pid, struct thermal_zone_device *tz, int boost,
		 unsigned int boost_timeout),

	TP_ARGS(pid, tz, boost, boost_timeout),

	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(int, tz_id)
		__field(int, boost)
		__field(unsigned int, boost_timeout)
	),

	TP_fast_assign(
		__entry->pid = pid;
		__entry->tz_id = tz->id;
		__entry->boost = boost;
		__entry->boost_timeout = boost_timeout;
	),

	TP_printk("pid=%d tz_id=%d boost=%d timeout=%u",
		__entry->pid,
		__entry->tz_id,
		__entry->boost,
		__entry->boost_timeout)
);
