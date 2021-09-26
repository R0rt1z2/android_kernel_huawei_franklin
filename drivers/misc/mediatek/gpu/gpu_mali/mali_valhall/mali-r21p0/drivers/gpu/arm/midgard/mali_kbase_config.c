/*
 *
 * (C) COPYRIGHT 2011-2015,2017 ARM Limited. All rights reserved.
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



#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config_defaults.h>
#ifdef CONFIG_GPU_AI_FENCE_INFO
#include <linux/gpu_hook.h>
#endif

static struct kbase_device *kbase_dev;

int kbasep_platform_device_init(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;
	kbase_dev = kbdev;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_init_func)
		return platform_funcs_p->platform_init_func(kbdev);

	return 0;
}

void kbasep_platform_device_term(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_term_func)
		platform_funcs_p->platform_term_func(kbdev);
}

#ifdef CONFIG_GPU_AI_FENCE_INFO
static int mali_kbase_report_fence_info(struct kbase_fence_info *fence)
{
	if (kbase_dev == NULL)
		return -EINVAL;

	if (kbase_dev->dev_data.game_pid != fence->game_pid)
		kbase_dev->dev_data.signaled_seqno = 0;

	kbase_dev->dev_data.game_pid = fence->game_pid;
	fence->signaled_seqno = kbase_dev->dev_data.signaled_seqno;

	return 0;
}

int perf_ctrl_get_gpu_fence(void __user *uarg)
{
	struct kbase_fence_info gpu_fence;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&gpu_fence, uarg, sizeof(struct kbase_fence_info))) {
		pr_err("%s copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	ret = mali_kbase_report_fence_info(&gpu_fence);
	if (ret != 0) {
		pr_err("get_gpu_fence mali fail, ret=%d\n", ret);
		return -EFAULT;
	}

	if (copy_to_user(uarg, &gpu_fence, sizeof(struct kbase_fence_info))) {
		pr_err("%s copy_to_user fail\n", __func__);
		return -EFAULT;
	}

	return 0;
}

/** The number of counter blocks that always present in the gpu.
 * - Job Manager
 * - Tiler
 *  */
#define ALWAYS_PRESENT_NUM_OF_HWCBLK_PER_GPU 2
/* Get performance counter raw dump blocks
 * The blocks include Job manager,Tiler,L2,Shader core
 **/
static unsigned int mali_kbase_get_hwc_buffer_size(void)
{
	struct kbase_gpu_props *kprops = NULL;
	unsigned int num_l2;
	unsigned int num_cores;

	if (kbase_dev == NULL)
		return 0;

	kprops = &kbase_dev->gpu_props;
	/* Number of L2 slice blocks */
	num_l2 = kprops->props.l2_props.num_l2_slices;
	/* Number of shader core blocks. coremask without the leading zeros
	 * Even coremask is not successive, the memory should reserve for dump`
	 */
	num_cores = fls64(kprops->props.coherency_info.group[0].core_mask);

	return ALWAYS_PRESENT_NUM_OF_HWCBLK_PER_GPU + num_l2 + num_cores;
}

int perf_ctrl_get_gpu_buffer_size(void __user *uarg)
{
	unsigned int gpu_buffer_size;

	if (uarg == NULL)
		return -EINVAL;

	gpu_buffer_size = mali_kbase_get_hwc_buffer_size();
	if (copy_to_user(uarg, &gpu_buffer_size, sizeof(unsigned int))) {
		pr_err("%s: copy_to_user fail\n", __func__);
		return -EFAULT;
	}

	return 0;
}

#ifdef CONFIG_MALI_LAST_BUFFER
/*
 * This function get lb_enable flag from AI freq schedule service
 * When enter games, AI freq will set lb_enable=1 to enable LB
 * When exit games, AI freq will set lb_enable=0 to bypass LB
 */
int perf_ctrl_enable_gpu_lb(void __user *uarg)
{
	unsigned int enable = 0;

	if (uarg == NULL || kbase_dev == NULL)
		return -EINVAL;

	if (copy_from_user(&enable, uarg, sizeof(unsigned int))) {
		pr_err("[Mali gpu]%s: Get LB enable fail\n", __func__);
		return -EFAULT;
	}
	if (enable != 0 && enable != 1) {
		pr_err("[Mali gpu]%s: Invalid LB parameters\n", __func__);
		return -EINVAL;
	}
	kbase_dev->dev_data.game_scene = enable;
	if (kbase_dev->dev_data.lb_enable == enable)
		return 0;
	mali_kbase_enable_lb(kbase_dev, enable);

	return 0;
}
#endif
#endif
