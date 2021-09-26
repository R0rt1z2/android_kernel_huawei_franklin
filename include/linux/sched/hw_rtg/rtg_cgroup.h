/*
 * rtg_cgroup.h
 *
 * Default cgroup rtg declaration
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
#ifndef HW_CGROUP_RTG_H
#define HW_CGROUP_RTG_H

#ifdef CONFIG_HW_CGROUP_RTG

void _do_update_preferred_cluster(struct related_thread_group *grp);
void do_update_preferred_cluster(struct related_thread_group *grp);

#endif

#endif
