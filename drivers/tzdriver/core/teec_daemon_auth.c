/*
 * teec_daemon_auth.c
 *
 * function for teecd or hidl process auth
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "teec_daemon_auth.h"
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/err.h>
#include <securec.h>

#include "tc_ns_log.h"

#ifdef CONFIG_CMS_CAHASH_AUTH
/* hash for: teecd/var/teecd0 */
static unsigned char g_teecd_auth_hash[SHA256_DIGEST_LENTH] = {
	0x8a, 0xfa, 0xd0, 0xd3,
	0xaa, 0xa1, 0x21, 0xb2,
	0x69, 0x2c, 0xe4, 0x99,
	0x54, 0xac, 0x99, 0x99,
	0x13, 0xe1, 0x49, 0x83,
	0x48, 0x8c, 0x2f, 0x95,
	0x51, 0xc5, 0x3d, 0xd4,
	0x86, 0xc0, 0x40, 0x12,
};
#elif defined (CONFIG_CLOUDSERVER_TEECD_AUTH)
/* hash for: teecd/usr/bin/teecd0 */
static unsigned char g_teecd_auth_hash[SHA256_DIGEST_LENTH] = {
	0x62, 0xd9, 0x07, 0xe4,
	0xd4, 0xf8, 0x5e, 0x4e,
	0xe4, 0xe7, 0xa1, 0x46,
	0x6d, 0x80, 0x03, 0x60,
	0x28, 0x4a, 0xaa, 0xe6,
	0xf1, 0xf9, 0x8d, 0x6c,
	0x12, 0x2e, 0xee, 0x21,
	0x6b, 0x1f, 0xd4, 0xad,
};
#else
/* hash for: teecd/vendor/bin/teecd0 */
static unsigned char g_teecd_auth_hash[SHA256_DIGEST_LENTH] = {
	0xc5, 0x6e, 0x2b, 0x89,
	0xce, 0x9e, 0xeb, 0x63,
	0xe7, 0x42, 0xfb, 0x2b,
	0x9d, 0x48, 0xff, 0x52,
	0xb2, 0x2f, 0xa7, 0xd5,
	0x87, 0xc6, 0x1f, 0x95,
	0x84, 0x5c, 0x0e, 0x96,
	0x9e, 0x18, 0x81, 0x51,
};
#endif

static unsigned char g_teecd_calc_hash[SHA256_DIGEST_LENTH] = {0};
static bool g_teecd_hash_calced = false;
DEFINE_MUTEX(g_hash_calc_lock);

static int check_teecd_path_access(void)
{
	unsigned char digest[SHA256_DIGEST_LENTH] = {0};

	if (calc_path_hash(false, digest, SHA256_DIGEST_LENTH)) {
		tloge("calc path hash failed\n");
		return -EFAULT;
	}

	if (memcmp(digest, g_teecd_auth_hash, SHA256_DIGEST_LENTH)) {
		tloge("check teecd process path failed \n");
		return -EACCES;
	}

	if (check_proc_selinux_access(current, "u:r:tee:s0")) {
		tloge("check teecd seclabel failed\n");
		return -EACCES;
	}

	return 0;
}

static int calc_teecd_process_hash(void)
{
	mutex_lock(&g_hash_calc_lock);

	if (g_teecd_hash_calced) {
		mutex_unlock(&g_hash_calc_lock);
		return 0;
	}

	if (memset_s(g_teecd_calc_hash,
		sizeof(g_teecd_calc_hash), 0x00,
		sizeof(g_teecd_calc_hash)) != EOK) {
		tloge("memset failed!\n");
		mutex_unlock(&g_hash_calc_lock);
		return -EFAULT;
	}

	g_teecd_hash_calced = (current->mm != NULL &&
		calc_task_hash(g_teecd_calc_hash,
		(uint32_t)SHA256_DIGEST_LENTH, current) == EOK);
	if (!g_teecd_hash_calced) {
		tloge("calc libteec hidl hash failed\n");
		mutex_unlock(&g_hash_calc_lock);
		return -EFAULT;
	}

	mutex_unlock(&g_hash_calc_lock);

	return 0;
}

static int check_teecd_code_hash(void)
{
	unsigned char digest[SHA256_DIGEST_LENTH] = {0};

	if (!g_teecd_hash_calced)
		return 0;

	if (calc_task_hash(digest, (uint32_t)SHA256_DIGEST_LENTH, current)) {
		tloge("calc task hash failed!\n");
		return -EACCES;
	}

	if (memcmp(digest, g_teecd_calc_hash, SHA256_DIGEST_LENTH)) {
		tloge("compare teecd hash error!\n");
		return -EACCES;
	}

	return EOK;
}

int check_teecd_access(void)
{
	if (check_teecd_path_access())
		return -EACCES;

	if (calc_teecd_process_hash()) {
		tloge("calc hidl process hash failed\n");
		return -EFAULT;
	}

	if (check_teecd_code_hash())
		return -EACCES;

	return EOK;
}
