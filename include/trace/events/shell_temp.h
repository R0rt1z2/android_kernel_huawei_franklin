#undef TRACE_SYSTEM
#define TRACE_SYSTEM shell_temp
#if !defined(_SHELL_TEMP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SHELL_TEMP_H

#include <linux/tracepoint.h>

TRACE_EVENT(calc_shell_temp,
	TP_PROTO(struct thermal_zone_device *tz, int i, int j, int coef, int temp, long sum),
	TP_ARGS(tz, i, j, coef, temp, sum),

	TP_STRUCT__entry(
	__string(thermal_zone, tz->type)
	__field(int, i)
	__field(int, j)
	__field(int, coef)
	__field(int, temp)
	__field(long, sum)
	),

	TP_fast_assign(
	__assign_str(thermal_zone, tz->type);
	__entry->i = i;
	__entry->j = j;
	__entry->coef = coef;
	__entry->temp = temp;
	__entry->sum = sum;
	),

	TP_printk("%s: sensor=%d time=%d coef=%d temp=%d sum=%ld",
	   __get_str(thermal_zone), __entry->i, __entry->j,
	   __entry->coef, __entry->temp, __entry->sum)
);

TRACE_EVENT(shell_temp,
	TP_PROTO(struct thermal_zone_device *tz, int temp),
	TP_ARGS(tz, temp),

	TP_STRUCT__entry(
	__string(thermal_zone, tz->type)
	__field(int, temp)
	),

	TP_fast_assign(
	__assign_str(thermal_zone, tz->type);
	__entry->temp = temp;
	),

	TP_printk("%s: shell temp = %d", __get_str(thermal_zone), __entry->temp)
);

TRACE_EVENT(shell_temp_read_succ,
	TP_PROTO(struct thermal_zone_device *tz, int temp, const char *passage_name,
        u32 sw_curr_temp, u32 qcom_temp, int para1),
	TP_ARGS(tz, temp, passage_name, sw_curr_temp, qcom_temp, para1),

	TP_STRUCT__entry(
	__string(thermal_zone, tz->type)
	__field(int, temp)
	__field(const char *, passage_name)
	__field(u32, sw_curr_temp)
	__field(u32, qcom_temp)
	__field(int, para1)
	),

	TP_fast_assign(
	__assign_str(thermal_zone, tz->type);
	__entry->temp = temp;
	__entry->passage_name = passage_name;
	__entry->sw_curr_temp = sw_curr_temp;
	__entry->qcom_temp = qcom_temp;
	__entry->para1 = para1;
	),

	TP_printk("SUCC %s: shell temp = %d; passage_name: %s; id: %d; sensor_type: %d; para1: %d;",
		__get_str(thermal_zone),
		__entry->temp,
		__entry->passage_name,
		__entry->sw_curr_temp,
		__entry->qcom_temp,
		__entry->para1
		)
);

TRACE_EVENT(shell_temp_read_fail,
	TP_PROTO(struct thermal_zone_device *tz, int temp, int rt_value, const char *passage_name,
        u32 id, u32 sensor_type, int para1),
	TP_ARGS(tz, temp, rt_value, passage_name, id, sensor_type, para1),

	TP_STRUCT__entry(
	__string(thermal_zone, tz->type)
	__field(int, temp)
	__field(int, rt_value)
	__field(const char *, passage_name)
	__field(u32, id)
	__field(u32, sensor_type)
	__field(int, para1)
	),

	TP_fast_assign(
	__assign_str(thermal_zone, tz->type);
	__entry->temp = temp;
	__entry->rt_value = rt_value;
	__entry->passage_name = passage_name;
	__entry->id = id;
	__entry->sensor_type = sensor_type;
	__entry->para1 = para1;
	),

	TP_printk("FAIL %s: shell temp = %d; rt_value = %d; passage_name: %s; id: %d; sensor_type: %d; para1: %d;",
		__get_str(thermal_zone),
		__entry->temp,
		__entry->rt_value,
		__entry->passage_name,
		__entry->id,
		__entry->sensor_type,
		__entry->para1
		)
);

TRACE_EVENT(handle_invalid_temp,
	TP_PROTO(struct thermal_zone_device *tz, int i, int invalid_temp, int temperature),
	TP_ARGS(tz, i, invalid_temp, temperature),

	TP_STRUCT__entry(
	__string(thermal_zone, tz->type)
	__field(int, i)
	__field(int, invalid_temp)
	__field(int, temperature)
	),

	TP_fast_assign(
	__assign_str(thermal_zone, tz->type);
	__entry->i = i;
	__entry->invalid_temp = invalid_temp;
	__entry->temperature = temperature;
	),

	TP_printk("handle_invalid_temp %s: idx = %d; rt_value = %d; [hisi_shell->index].temp: %d;",
        __get_str(thermal_zone), __entry->i, __entry->invalid_temp, __entry->temperature)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
