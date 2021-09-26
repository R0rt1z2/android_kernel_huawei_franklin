/*
 * cpufreq_hw.h
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
#ifndef __CPUFREQ_HW_H__
#define __CPUFREQ_HW_H__

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
#include <linux/uidgid.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#endif

struct governor_user_attr {
	const char *name;
	uid_t uid;
	gid_t gid;
	mode_t mode;
};
#define SYSTEM_UID (uid_t)1000
#define SYSTEM_GID (uid_t)1000

#define INVALID_ATTR \
	{.name = NULL, .uid = (uid_t)(-1), .gid = (uid_t)(-1), .mode = 0000}

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
static inline void gov_sysfs_set_attr(unsigned int cpu, char *gov_name,
					struct governor_user_attr *attrs)
{
	char *buf = NULL;
	int i = 0, gov_dir_len, gov_attr_len;
	long ret;
	mm_segment_t fs = 0;

	if (!capable(CAP_SYS_ADMIN))
		return;

	buf = kzalloc(PATH_MAX, GFP_KERNEL);
	if (!buf)
		return;

	gov_dir_len = scnprintf(buf, PATH_MAX,
				"/sys/devices/system/cpu/cpu%u/cpufreq/%s/",
				cpu, gov_name);

	fs = get_fs(); /*lint !e501*/
	set_fs(KERNEL_DS); /*lint !e501*/

	while (attrs[i].name) {
		gov_attr_len = scnprintf(buf + gov_dir_len,
					 PATH_MAX - gov_dir_len, attrs[i].name);

		if (gov_dir_len + gov_attr_len >= PATH_MAX) {
			i++;
			continue;
		}
		buf[gov_dir_len + gov_attr_len] = '\0';

		ret = sys_chown(buf, attrs[i].uid, attrs[i].gid);
		if (ret)
			pr_debug("chown fail:%s ret=%ld\n", buf, ret);

		ret = sys_chmod(buf, attrs[i].mode);
		if (ret)
			pr_debug("chmod fail:%s ret=%ld\n", buf, ret);
		i++;
	}

	set_fs(fs);
	kfree(buf);

	return;
}
#else
static inline void gov_sysfs_set_attr(unsigned int cpu, char *gov_name,
					struct governor_user_attr *attrs)
{
}
#endif

#endif
