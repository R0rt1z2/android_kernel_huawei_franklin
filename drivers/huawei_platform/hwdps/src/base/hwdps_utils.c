/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for
 *              operations about some tools.
 * Create: 2020-06-16
 */

#include "inc/base/hwdps_utils.h"
#include <linux/err.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/limits.h>
#include <linux/mm.h>
#include <linux/path.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <securec.h>
#include <huawei_platform/hwdps/hwdps_limits.h>

#define EXE_MAX_ARG_STRLEN 256
#define MAX_JSON_STR_LEN 256
#define HEX_STR_PER_BYTE 2
#define HWDPS_PERMISSION_LEN 8
#define PROCESS_INIT_TAG "init="
#define PROCESS_INIT_TAG_LEN 5

void hwdps_utils_free_package_info(struct hwdps_package_info_t *pinfo)
{
	if (!pinfo)
		return;
	kzfree(pinfo->app_policy);
	pinfo->app_policy = NULL;
}

static bool hwdps_utils_exe_check_without_check_para(const s8 *exe_path,
	s32 exe_path_len, pid_t pid)
{
	bool is_matched = false;
	s8 *buf = NULL;
	s8 *path = NULL;
	struct file *exe_file = NULL;
	struct mm_struct *mm = NULL;
	struct task_struct *task = NULL;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		hwdps_pr_err("Failed to lookup running task! pid %llu\n", pid);
		return is_matched;
	}
	get_task_struct(task);
	rcu_read_unlock();
	mm = get_task_mm(task);
	if (mm) {
		buf = (s8 *)__get_free_page(GFP_KERNEL);
		if (buf == NULL)
			goto done_mmput;
		exe_file = get_mm_exe_file(mm);
		if (exe_file == NULL)
			goto done_free;
		path = file_path(exe_file, buf, PAGE_SIZE);
		if (path == NULL || IS_ERR(path))
			goto done_put;
		if (strncmp(path, exe_path, exe_path_len) == 0)
			is_matched = true;
		else
			hwdps_pr_err("arg dismatch %s\n", path);
	} else {
		put_task_struct(task);
		return is_matched;
	}
done_put:
	fput(exe_file);
done_free:
	free_page((uintptr_t)buf);
done_mmput:
	mmput(mm); /* only when mm is not null would this branch take affect */
	put_task_struct(task);
	return is_matched;
}

bool hwdps_utils_exe_check(pid_t pid, const s8 *exe_path, s32 exe_path_len)
{
	if ((exe_path == NULL) || (exe_path_len > EXE_MAX_ARG_STRLEN) ||
		(exe_path_len <= 0))
		return false;

	return hwdps_utils_exe_check_without_check_para(exe_path,
		exe_path_len, pid);
}

s32 hwdps_utils_copy_package_info(struct hwdps_package_info_t *dst,
	struct hwdps_package_info_t *src)
{
	/* policy len max is HWDPS_POLICY_RULESET_MAX 256 */
	if (!dst || !src)
		return -EINVAL;
	dst->app_policy_len = src->app_policy_len;
	if (dst->app_policy_len == 0)
		return -EINVAL;
	dst->app_policy = kmalloc(dst->app_policy_len + 1, GFP_KERNEL);
	if (!dst->app_policy)
		goto cleanup;
	if (memcpy_s(dst->app_policy, dst->app_policy_len, src->app_policy,
		src->app_policy_len) != EOK)
		goto cleanup;
	dst->app_policy[dst->app_policy_len] = '\0';
	return 0;

cleanup:
	hwdps_pr_err("Failed to allocate memory\n");
	hwdps_utils_free_package_info(dst);
	return -ENOMEM;
}

static s32 hwdps_utils_copy_policy_from_user(
	struct hwdps_package_info_t *kpinfo,
	struct hwdps_package_info_t *upinfo,
	bool copy_extinfo)
{
	if (!copy_extinfo)
		return 0;
	if (!upinfo->app_policy ||
		(kpinfo->app_policy_len < HWDPS_POLICY_RULESET_MIN) ||
		(kpinfo->app_policy_len > HWDPS_POLICY_RULESET_MAX)) {
		hwdps_pr_err("app_policy_len %lu\n", kpinfo->app_policy_len);
		return -EINVAL;
	}
	kpinfo->app_policy = kzalloc(kpinfo->app_policy_len + 1,
		GFP_KERNEL);
	if (!kpinfo->app_policy)
		return -ENOMEM;
	if (copy_from_user(kpinfo->app_policy, upinfo->app_policy,
		kpinfo->app_policy_len) != 0) {
		hwdps_pr_err("Failed while copying from user space\n");
		return -EFAULT;
	}
	kpinfo->app_policy[kpinfo->app_policy_len] = '\0';
	return 0;
}

s32 hwdps_utils_copy_package_info_from_user(struct hwdps_package_info_t *kpinfo,
	struct hwdps_package_info_t *upinfo, bool copy_extinfo)
{
	s32 ret;

	if (!kpinfo || !upinfo)
		return -EINVAL;
	/* Pre set NULL is to avoid freeing user memory under err cases. */
	kpinfo->app_policy = NULL;
	/* Copy policy from user space. */
	ret = hwdps_utils_copy_policy_from_user(kpinfo, upinfo, copy_extinfo);
	if (ret != 0)
		goto cleanup;
	return 0;

cleanup:
	hwdps_utils_free_package_info(kpinfo);
	return ret;
}

s32 hwdps_utils_copy_packages_from_user(struct hwdps_package_info_t *kpackages,
	struct hwdps_package_info_t *upackages, u32 package_count)
{
	s32 ret;
	u32 i;

	if (!kpackages || !upackages)
		return -EFAULT;
	if ((package_count <= 0) || (package_count > SYNC_PKG_CNT_MAX))
		return -EINVAL;
	for (i = 0; i < package_count; i++) {
		if (memcpy_s(&kpackages[i],
			sizeof(struct hwdps_package_info_t), &upackages[i],
			sizeof(struct hwdps_package_info_t)) != EOK)
			return -EINVAL;
		ret = hwdps_utils_copy_package_info_from_user(&kpackages[i],
			&upackages[i], true);
		if (ret != 0)
			return ret;
		if (kpackages[i].app_policy) {
			hwdps_pr_debug("%dth pkg info policy %s\n", i,
				kpackages[i].app_policy);
		}
	}
	return ret;
}

static s8 *hwdps_utils_get_json_start_str(const s8 *str, const s8 *key)
{
	s8 *pc_start_buf = NULL;

	/* no need to check key */
	if (!str || (strlen(str) > MAX_JSON_STR_LEN) ||
		(strlen(str) == 0))
		return NULL;
	pc_start_buf = strstr((s8 *)str, key);
	if (!pc_start_buf) {
		hwdps_pr_err("Faild get label \"%s\" content from json\n", str);
		return NULL;
	}
	return pc_start_buf;
}

s8 *hwdps_utils_get_json_str(const s8 *str, const s8 *key, u32 *out_len)
{
	u32 ui_conten_len;
	u32 len;
	s8 ch;
	s8 *pc_content = NULL;
	s8 *pc_start_buf = NULL;
	s8 *pc_end_buf = NULL;
	s8 *pc_key_str_end = (s8 *)"\"";
	u32 ui_offset = 0;

	if (!str || !key)
		return NULL;

	pc_start_buf = hwdps_utils_get_json_start_str(str, key);

	if (!pc_start_buf)
		return NULL;
	len = strlen(key);

	/* buf + len + 1 must be inner str */
	ch = *(pc_start_buf + len + 1); /* one indicates size of end */

	/* Continue until no space and :, multy : is also valid. */
	while ((ch == ' ') || (ch == '\n') || (ch == ':')) {
		ui_offset++;
		if (strlen(pc_start_buf) < len + ui_offset)
			return NULL;
		ch = *(pc_start_buf + len + ui_offset);
	}
	if (ch != '\"')
		goto error;
	/* Json formart err if " doesn't after : skip starting " of content. */
	ui_offset++;
	pc_start_buf += (len + ui_offset); /* Set start point at content. */
	pc_end_buf = strstr(pc_start_buf, pc_key_str_end);
	if (!pc_end_buf) {
		hwdps_pr_err("Faild to get label \"%s\" from json\n", key);
		goto error;
	}
	/* get content */
	ui_conten_len = pc_end_buf - pc_start_buf;
	if ((ui_conten_len > strlen(pc_start_buf)) || (ui_conten_len == 0)) {
		hwdps_pr_err("json data len error %d\n", ui_conten_len);
		goto error;
	}
	pc_content = kzalloc(ui_conten_len + 1, GFP_KERNEL);

	if (!pc_content)
		goto error;
	if (memcpy_s(pc_content, ui_conten_len, pc_start_buf, ui_conten_len) !=
		EOK) {
		kzfree(pc_content);
		goto error;
	}
	*out_len = ui_conten_len;
	return pc_content;
error:
	hwdps_pr_err("get json str err as %s end\n", str);
	return NULL;
}
