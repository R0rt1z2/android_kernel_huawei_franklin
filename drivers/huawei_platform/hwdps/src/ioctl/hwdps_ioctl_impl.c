/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This file contains the function required for init_user and
 *              package management.
 * Create: 2020-06-16
 */

#include "inc/ioctl/hwdps_ioctl_impl.h"
#include <securec.h>
#include <huawei_platform/hwdps/hwdps_ioctl.h>
#include "inc/base/hwdps_utils.h"
#include "inc/data/hwdps_data.h"
#include "inc/data/hwdps_packages.h"
#include "inc/policy/hwdps_policy.h"

static hwdps_result_t get_result(s32 ret)
{
	switch (ret) {
	case 0:
		return HWDPS_SUCCESS;
	case -EINVAL:
		return HWDPS_ERR_INVALID_ARGS;
	case -ENOMEM:
		return HWDPS_ERR_NO_MEMORY;
	default:
		return HWDPS_ERR_INTERNAL;
	}
}

void hwdps_sync_installed_packages(
	struct hwdps_sync_packages_t *sync_installed_packages)
{
	s32 ret;
	u32 i;

	if (!sync_installed_packages)
		return;

	hwdps_data_write_lock();
	for (i = 0; i < sync_installed_packages->package_count; i++) {
		ret = hwdps_packages_insert(
			&sync_installed_packages->packages[i]);
		if (ret != 0)
			break;
	}

	hwdps_data_write_unlock();
	sync_installed_packages->ret = get_result(ret);
}

void hwdps_install_package(struct hwdps_install_package_t *install_package)
{
	s32 ret;

	if (!install_package)
		return;

	hwdps_data_write_lock();
	ret = hwdps_packages_insert(&install_package->pinfo);
	hwdps_data_write_unlock();
	install_package->ret = get_result(ret);
}

void hwdps_uninstall_package(
	struct hwdps_uninstall_package_t *uninstall_package)
{
	if (!uninstall_package)
		return;

	hwdps_data_write_lock();
	hwdps_packages_delete(&uninstall_package->pinfo);
	hwdps_data_write_unlock();
	uninstall_package->ret = HWDPS_SUCCESS;
}
