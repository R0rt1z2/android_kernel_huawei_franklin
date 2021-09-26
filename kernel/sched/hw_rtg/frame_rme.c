/*
 * iaware_rtg.c
 *
 * rtg ioctl entry
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

#include "include/iaware_rtg.h"
#include <linux/atomic.h>
#include <linux/cred.h>
#include <linux/sched/task.h>
#include <linux/sched/topology.h>
#include <trace/events/sched.h>

#include "include/frame.h"
#include "include/frame_timer.h"
#include "include/proc_state.h"
#include "include/set_rtg.h"


extern atomic_t g_act_state;
extern atomic_t g_rtg_pid;
extern atomic_t g_rtg_tid;

atomic_t g_fps_state = ATOMIC_INIT(INIT_STATE);
static struct rme_fps_data g_fps_data[MAX_BUF_SIZE] = {
	{
		.deadline_margin = -3,
		.input_margin = -12,
		.animation_margin = -10,
		.traversal_margin = -10,
		.input_period = 3,
		.animation_period = 4,
		.traversal_period = 4,
		.default_util = 600,
		.frame_rate = DEFAULT_FRAME_RATE,
		.min_prev_util = FRAME_DEFAULT_MIN_PREV_UTIL,
		.max_prev_util = FRAME_DEFAULT_MAX_PREV_UTIL,
	},
	{
		.deadline_margin = 0,
		.input_margin = 0,
		.animation_margin = 0,
		.traversal_margin = 0,
		.input_period = 0,
		.animation_period = 0,
		.traversal_period = 0,
		.default_util = 0,
		.frame_rate = 0,
		.min_prev_util = 0,
		.max_prev_util = 0,
	}
};

static int set_min_util(const struct rtg_data_head *rtg_head, int min_util)
{
	struct frame_info *frame_info = NULL;

	if ((rtg_head == NULL) || (rtg_head->tid != current->pid))
		return -FRAME_ERR_PID;

	frame_info = lookup_frame_info_by_task(current);
	if (!frame_info)
		return -FRAME_ERR_PID;

	if (atomic_read(&frame_info->act_state) == ACTIVITY_BEGIN)
		return -FRAME_IS_ACT;

	set_frame_min_util(frame_info, min_util, false);
	return SUCC;
}

static int set_margin(const struct rtg_data_head *rtg_head, int margin)
{
	struct frame_info *frame_info = NULL;

	if ((rtg_head == NULL) || (rtg_head->tid != current->pid))
		return -FRAME_ERR_PID;

	frame_info = lookup_frame_info_by_task(current);
	if (!frame_info)
		return -FRAME_ERR_PID;

	if (atomic_read(&frame_info->act_state) == ACTIVITY_BEGIN)
		return -FRAME_IS_ACT;

	set_frame_margin(frame_info, margin);
	return SUCC;
}

static int set_min_util_and_margin(const struct rtg_data_head *rtg_head,
			int min_util, int margin)
{
	struct frame_info *frame_info = NULL;

	if ((rtg_head == NULL) || (rtg_head->tid != current->pid))
		return -FRAME_ERR_PID;

	frame_info = lookup_frame_info_by_task(current);
	if (!frame_info)
		return -FRAME_ERR_PID;

	if (atomic_read(&frame_info->act_state) == ACTIVITY_BEGIN)
		return -FRAME_IS_ACT;

	set_frame_min_util_and_margin(frame_info, min_util, margin);
	return SUCC;
}

int ctrl_rme_state(int freq_type)
{
	/* if rme set frame_freq_type, translated margin to freq_type */
	switch (freq_type) {
	case FRAME_INPUT: {
		freq_type = g_fps_data[PING].input_margin;
		break;
	}
	case FRAME_ANIMATION: {
		freq_type = g_fps_data[PING].animation_margin;
		break;
	}
	case FRAME_TRAVERSAL: {
		freq_type = g_fps_data[PING].traversal_margin;

		break;
	}
	default:
		break;
	}

	return freq_type;
}

long ctrl_set_min_util(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct proc_state_data state_data;

	if (uarg == NULL)
		return -INVALID_ARG;

	if (copy_from_user(&state_data, uarg, sizeof(state_data))) {
		pr_err("[AWARE_RTG] CMD_ID_SET_MIN_UTIL copy data failed\n");
		return -INVALID_ARG;
	}
	return set_min_util(&(state_data.head), state_data.frame_freq_type);
}

long ctrl_set_margin(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct proc_state_data state_data;
	int margin;

	if (uarg == NULL)
		return -INVALID_ARG;

	if (copy_from_user(&state_data, uarg, sizeof(state_data))) {
		pr_err("[AWARE_RTG] CMD_ID_SET_MARGIN copy data failed\n");
		return -INVALID_ARG;
	}

	margin = ctrl_rme_state(state_data.frame_freq_type);
	return set_margin(&(state_data.head), margin);
}

long ctrl_set_min_util_and_margin(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct min_util_margin_data state_data;
	int util = 0;

	if (uarg == NULL)
		return -INVALID_ARG;

	if (copy_from_user(&state_data, uarg, sizeof(state_data))) {
		pr_err("[RME][RMEKERNEL] CMD_ID_SET_MIN_UTIL_AND_MARGIN copy data failed\n");
		return -INVALID_ARG;
	}

	if (state_data.min_util == 1) /* 1 means use util */
		util = g_fps_data[PING].default_util;
	return set_min_util_and_margin(&(state_data.head),
		util, (state_data.margin + g_fps_data[PING].deadline_margin));
}

long ctrl_set_rme_margin(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct rme_fps_data fps_data;
	const int min_margin = -16;

	if (current_uid().val != SYSTEM_SERVER_UID) {
		pr_err("[RME][RMEKERNEL]  Invalid uid\n");
		return -INVALID_ARG;
	}
	if (uarg == NULL)
		return -INVALID_ARG;
	if (copy_from_user(&fps_data, uarg, sizeof(fps_data))) {
		pr_err("[RME][RMEKERNEL]  CMD_ID_SET_RME_MARGIN copy data failed\n");
		return -INVALID_ARG;
	}

	if ((fps_data.deadline_margin < min_margin) ||
	    (fps_data.input_margin < min_margin) ||
	    (fps_data.animation_margin < min_margin) ||
	    (fps_data.traversal_margin < min_margin) ||
	    (fps_data.input_period < 0) || /* peroid above 0 */
	    (fps_data.animation_period < 0) || /* peroid above 0 */
	    (fps_data.traversal_period < 0) || /* peroid above 0 */
	    (fps_data.default_util >= 1024)) { /* util below 1024 */
		pr_err("[RME][RMEKERNEL] CMD_ID_SET_RME_MARGIN Para Invalid\n");
		return -INVALID_ARG;
	}

	if (atomic_read(&g_fps_state) != INIT_STATE)
		return SUCC;
	atomic_set(&g_fps_state, WRITING);
	if (atomic_read(&g_fps_state) != WRITING)
		return SUCC;

	/* check and set frame_rate */
	if (g_fps_data[PONG].frame_rate == fps_data.frame_rate) {
		atomic_set(&g_fps_state, INIT_STATE);
		return SUCC;
	}

	g_fps_data[PONG] = fps_data;
	atomic_set(&g_fps_state, INIT_STATE);

	return SUCC;
}

long ctrl_get_rme_margin(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct rme_fps_data margin_data;

	if (uarg == NULL)
		return -INVALID_ARG;
	if (copy_from_user(&margin_data, uarg, sizeof(margin_data))) {
		pr_err("[RME][RMEKERNEL] CMD_ID_GET_RME_MARGIN copy data failed\n");
		return -INVALID_ARG;
	}

	if (atomic_read(&g_fps_state) != INIT_STATE)
		return SUCC;
	atomic_set(&g_fps_state, READING);
	if (atomic_read(&g_fps_state) != READING)
		return SUCC;

	if ((g_fps_data[PONG].frame_rate != 0) &&
	    (g_fps_data[PONG].frame_rate != g_fps_data[PING].frame_rate))
		g_fps_data[PING] = g_fps_data[PONG];

	if (margin_data.frame_rate != g_fps_data[PING].frame_rate)
		margin_data = g_fps_data[PING];
	else /* notify do not need to update para */
		margin_data.frame_rate = 0;

	atomic_set(&g_fps_state, INIT_STATE);
	if (copy_to_user(uarg, &margin_data, sizeof(margin_data))) {
		pr_err("[RME][RMEKERNEL] CMD_ID_GET_RME_MARGIN send data failed\n");
		return -INVALID_ARG;
	}

	return SUCC;
}
