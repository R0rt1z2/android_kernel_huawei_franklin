/*
 *
 * (C) COPYRIGHT 2011-2017 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 */

#ifndef MALI_KBASE_CONFIG_MT6885_H
#define MALI_KBASE_CONFIG_MT6885_H

#include "linux/gmc.h"
#include "linux/gmc_storage.h"
struct kbase_mtk_device_data {
#ifdef CONFIG_GPU_GMC_GENERIC
	struct gmc_device kbase_gmc_device;
	struct workqueue_struct *gmc_workqueue;
	int gmc_cancel;
#endif
};

struct kbase_mtk_ctx_data {

/* Add other context data here */

	/* BASE_DEBUG_FENCE_TIMEOUT: timer and timer_started */
	struct hrtimer fence_wait_timer;
	int timer_started;

#ifdef CONFIG_GPU_GMC_GENERIC
	bool set_pt_flag;
#endif
};
#endif