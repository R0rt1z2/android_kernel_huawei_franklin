/*
 * Huawei Top Task File
 *
 * Copyright (c) 2019-2020 Huawei Technologies Co., Ltd.
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

static inline unsigned int task_load_freq(struct task_struct *p)
{
	return p->ravg.load_avg;
}

static inline u8 curr_table(struct rq *rq)
{
	return rq->curr_table;
}

static inline u8 prev_table(struct rq *rq)
{
	return 1 - rq->curr_table;
}

static inline int reversed_index(int index)
{
	if (unlikely(index >= NUM_LOAD_INDICES))
		index = NUM_LOAD_INDICES - 1;
	return NUM_LOAD_INDICES - 1 - index;
}

/*
 * Let lower index means higher load, so we can use find_next_bit
 * to get the next highest load.
 *
 * Note: max load is limited as:
 *       walt_ravg_window / NUM_LOAD_INDICES * (NUM_LOAD_INDICES - 1)
 */
static int load_to_index(u32 load)
{
	return reversed_index(load / SCHED_LOAD_GRANULE);
}

static unsigned long index_to_load(int index)
{
	return reversed_index(index) * SCHED_LOAD_GRANULE;
}

static unsigned long __top_task_load(struct rq *rq, u8 table)
{
	struct top_task_entry *top_task_table = rq->top_tasks[table];
	int top_index = rq->top_task_index[table];

	/*
	 * Top_index might be NUM_LOAD_INDICES.
	 * Check before use it as array index.
	 */
	if (top_index >= NUM_LOAD_INDICES)
		return 0;

	/*
	 * Just consider no top task load when the top task is
	 * not a preferidle one. It will take some cost to find
	 * the top preferidle task, which may not be very necessary.
	 */
	if (!top_task_table[top_index].preferidle_count)
		return 0;

	return index_to_load(top_index);
}

static unsigned long top_task_load(struct rq *rq)
{
	return max(__top_task_load(rq, curr_table(rq)),
		   __top_task_load(rq, prev_table(rq)));
}

unsigned long top_task_util(struct rq *rq)
{
	unsigned long capacity = capacity_orig_of(cpu_of(rq));

	unsigned long load = top_task_load(rq);
	unsigned long util = (load << SCHED_CAPACITY_SHIFT) / walt_ravg_window;

	return (util >= capacity) ? capacity : util;
}

/*
 * Limit overflow case.
 * For top_task_table[index].count, increase and decrease
 * for any index are always in pair.
 * For top_task_table[index].preferidle_count, we may lose
 * +1 or -1 sometimes so it's not accurate. Disallow overflow
 * will be helpful.
 */
static inline bool inc_top_task_count(u8 *count)
{
	bool will_overflow = (*count == (u8)(~0));

	if (!will_overflow)
		(*count)++;

	return will_overflow;
}

static inline bool dec_top_task_count(u8 *count)
{
	bool will_overflow = (*count == 0);

	if (!will_overflow)
		(*count)--;

	return will_overflow;
}

static u32 add_top_task_load(struct rq *rq, u8 table,
				u32 load, bool prefer_idle)
{
	struct top_task_entry *top_task_table = rq->top_tasks[table];
	int index = load_to_index(load);
	u32 delta_load = 0;

	if (index == ZERO_LOAD_INDEX)
		return delta_load;

	inc_top_task_count(&top_task_table[index].count);

	/*
	 * if 0->1.
	 * This means it's the first task that fill this load index.
	 * Set the bit in top_task_bitmap and check if top_task_index
	 * needs to be updated.
	 */
	if (top_task_table[index].count == 1) {
		__set_bit(index, rq->top_tasks_bitmap[table]);

		if (index < rq->top_task_index[table]) {
			delta_load = (rq->top_task_index[table] - index)
					* SCHED_LOAD_GRANULE;
			rq->top_task_index[table] = index;
		}
	}

	if (prefer_idle)
		inc_top_task_count(&top_task_table[index].preferidle_count);

	return delta_load;
}

static void del_top_task_load(struct rq *rq, u8 table,
				u32 load, bool prefer_idle)
{
	struct top_task_entry *top_task_table = rq->top_tasks[table];
	int index = load_to_index(load);

	if (index == ZERO_LOAD_INDEX)
		return;

	dec_top_task_count(&top_task_table[index].count);

	/*
	 * if 1->0.
	 * This means currently there's no task has this load index.
	 * Clear the bit in top_task_bitmap and update top_task_index
	 * if necessary.
	 */
	if (top_task_table[index].count == 0) {
		__clear_bit(index, rq->top_tasks_bitmap[table]);

		/* Always clear preferidle_count when we are sure there's no task */
		top_task_table[index].preferidle_count = 0;

		if (index == rq->top_task_index[table]) {
			rq->top_task_index[table] = find_next_bit(
				rq->top_tasks_bitmap[table], NUM_LOAD_INDICES, index);
		}
	}

	if (prefer_idle)
		dec_top_task_count(&top_task_table[index].preferidle_count);

	return;
}

#define TOP_TASK_IGNORE_UTIL 80
#define TOP_TASK_IGNORE_LOAD \
	((u64)walt_ravg_window * TOP_TASK_IGNORE_UTIL / SCHED_CAPACITY_SCALE)

static void add_top_task(struct task_struct *p, struct rq *rq)
{
	u32 delta_load;
	u32 load = task_load_freq(p);

	if (exiting_task(p) || is_idle_task(p))
		return;

	if (load < TOP_TASK_IGNORE_LOAD)
		return;

	p->ravg.curr_load = load;

	delta_load = add_top_task_load(rq, rq->curr_table, load, schedtune_top_task(p) > 0);

	if (delta_load > TOP_TASK_IGNORE_LOAD &&
	    top_task_util(rq) > capacity_curr_of(cpu_of(rq)))
		sugov_mark_util_change(cpu_of(rq), ADD_TOP_TASK);

	trace_walt_update_top_task(rq, p);
}

static void
migrate_top_task(struct task_struct *p, struct rq *src_rq, struct rq *dest_rq)
{
	bool is_top_task = schedtune_top_task(p) > 0;

	add_top_task_load(dest_rq, curr_table(dest_rq), p->ravg.curr_load, is_top_task);
	add_top_task_load(dest_rq, prev_table(dest_rq), p->ravg.prev_load, is_top_task);

	del_top_task_load(src_rq, curr_table(src_rq), p->ravg.curr_load, is_top_task);
	del_top_task_load(src_rq, prev_table(src_rq), p->ravg.prev_load, is_top_task);

	trace_walt_update_top_task(src_rq, p);
	trace_walt_update_top_task(dest_rq, p);
}

void top_task_exit(struct task_struct *p, struct rq *rq)
{
	/*
	 * We'd better not clear p in top task here, unless we do walt update
	 * before top_task_exit. For now, leave exit task as if it was still
	 * on that cpu.
	 */
	return;
}

static inline void clear_top_tasks_table(struct top_task_entry *table)
{
	memset(table, 0, NUM_LOAD_INDICES * sizeof(struct top_task_entry));
}

static inline void clear_top_tasks_bitmap(unsigned long *bitmap)
{
	memset(bitmap, 0, BITS_TO_LONGS(NUM_LOAD_INDICES) * sizeof(unsigned long));
}

static void rollover_top_task_table(struct rq *rq, int nr_full_windows)
{
	u8 curr_table = rq->curr_table;
	u8 prev_table = 1 - curr_table;

	/* Clear prev table */
	clear_top_tasks_table(rq->top_tasks[prev_table]);
	clear_top_tasks_bitmap(rq->top_tasks_bitmap[prev_table]);
	rq->top_task_index[prev_table] = ZERO_LOAD_INDEX;

	/* Also clear curr table if last window is empty */
	if (nr_full_windows) {
		rq->top_task_index[curr_table] = ZERO_LOAD_INDEX;
		clear_top_tasks_table(rq->top_tasks[curr_table]);
		clear_top_tasks_bitmap(rq->top_tasks_bitmap[curr_table]);
	}

	/* Finally exchange rq's curr_table & prev_table pointer.
	 * This means let curr to be prev, and let an empty one to be curr.
	 *
	 * new tables:               curr                prev
	 *                            |                   |
	 * old tables:   prev_table(must be cleared)  curr_table
	 */
	rq->curr_table = prev_table;
}

static void rollover_top_task_load(struct task_struct *p, int nr_full_windows)
{
	int curr_load = 0;

	if (!nr_full_windows)
		curr_load = p->ravg.curr_load;

	p->ravg.prev_load = curr_load;
	p->ravg.curr_load = 0;
}

static void top_task_load_update_history(struct rq *rq, struct task_struct *p,
				     u32 runtime, int samples, int event)
{
	u32 *hist;
	int ridx, widx;
	u32 max = 0, avg, load_avg;
	u64 sum = 0;
	unsigned int hist_size, stats_policy;

	hist = &p->ravg.load_sum_history[0];
	hist_size = rq->top_task_hist_size;
	stats_policy = rq->top_task_stats_policy;

	/* Clear windows out of hist_size */
	widx = RAVG_HIST_SIZE_MAX - 1;
	for (; widx >= hist_size; --widx)
		hist[widx] = 0;

	/* Push new 'runtime' value onto stack */
	ridx = widx - samples;
	for (; ridx >= 0; --widx, --ridx) {
		hist[widx] = hist[ridx];
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	for (widx = 0; widx < samples && widx < hist_size; widx++) {
		hist[widx] = runtime;
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	p->ravg.load_sum = 0;

	if (stats_policy == WINDOW_STATS_RECENT) {
		load_avg = runtime;
	} else if (stats_policy == WINDOW_STATS_MAX) {
		load_avg = max;
	} else {
		avg = div64_u64(sum, hist_size);
		if (stats_policy == WINDOW_STATS_AVG)
			load_avg = avg;
		else
			load_avg = max(avg, runtime);
	}

	p->ravg.load_avg = load_avg;

	return;
}

static void update_top_task_load(struct task_struct *p, struct rq *rq,
				int event, u64 wallclock, bool account_busy)
{
	u64 mark_start = p->ravg.mark_start;
	u64 delta, window_start = rq->window_start;
	int nr_full_windows;
	u32 window_size = walt_ravg_window;

	/* No need to bother updating task load for exiting tasks
	 * or the idle task. */
	if (exiting_task(p) || is_idle_task(p))
		return;

	delta = window_start - mark_start;
	nr_full_windows = div64_u64(delta, window_size);

	if (!account_busy) {
		/* Complete last non-empty window */
		top_task_load_update_history(rq, p, p->ravg.load_sum, 1, event);

		/* Push empty windows into history if wanted */
		if (nr_full_windows && rq->top_task_stats_empty_window)
			top_task_load_update_history(rq, p, 0, nr_full_windows, event);

		return;
	}

	/* Temporarily rewind window_start to first window boundary
	 * after mark_start. */
	window_start -= (u64)nr_full_windows * (u64)window_size;

	/* Process (window_start - mark_start) first */
	p->ravg.load_sum += scale_exec_time(window_start - mark_start, rq);

	/* Push new sample(s) into task's load history */
	top_task_load_update_history(rq, p, p->ravg.load_sum, 1, event);
	if (nr_full_windows)
		top_task_load_update_history(rq, p, scale_exec_time(window_size, rq),
			       nr_full_windows, event);

	/* Roll window_start back to current to process any remainder
	 * in current window. */
	window_start += (u64)nr_full_windows * (u64)window_size;

	/* Process (wallclock - window_start) next */
	mark_start = window_start;
	p->ravg.load_sum += scale_exec_time(wallclock - mark_start, rq);
}
