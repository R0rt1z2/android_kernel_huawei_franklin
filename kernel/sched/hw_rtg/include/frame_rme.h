/*
 * frame_rme.h
 *
 * trans rtg thread header
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

#ifndef HW_FRAME_RME_H
#define HW_FRAME_RME_H

#ifdef CONFIG_HW_RTG_FRAME_RME
int ctrl_rme_state(int freq_type);

long ctrl_set_min_util(unsigned long arg);
long ctrl_set_margin(unsigned long arg);
long ctrl_set_min_util_and_margin(unsigned long arg);
long ctrl_set_rme_margin(unsigned long arg);
long ctrl_get_rme_margin(unsigned long arg);
#else
static inline long ctrl_set_min_util(unsigned long arg)
{
	return -EFAULT;
}
static inline long ctrl_set_margin(unsigned long arg)
{
	return -EFAULT;
}
static inline long ctrl_set_min_util_and_margin(unsigned long arg)
{
	return -EFAULT;
}
static inline long ctrl_set_rme_margin(unsigned long arg)
{
	return -EFAULT;
}
static inline long ctrl_get_rme_margin(unsigned long arg)
{
	return -EFAULT;
}
#endif /* CONFIG_HW_RTG_FRAME_RME */

#endif /* HW_FRAME_RME_H */
