/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Define type struct and function about ioctl.
 * Create: 2020-06-16
 */

#ifndef _HWDPS_IOCTL_H
#define _HWDPS_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#include <huawei_platform/hwdps/hwdps_error.h>
#include <huawei_platform/hwdps/hwdps_limits.h>
#else
#include <stdbool.h>
#include <sys/ioctl.h>
#include "hwdps_error.h"
#include "hwdps_limits.h"
#endif

/* device name */
#ifndef HWDPS_DEVICE_NAME
#define HWDPS_DEVICE_NAME "hwdps"
#endif

/* user space shared data structures */
struct hwdps_package_info_t {
	s32 appid; /* in: unique package identifier  */
	s8 *app_policy;
	u64 app_policy_len;
};

struct hwdps_sync_packages_t {
	struct hwdps_package_info_t *packages; /* list of installed packages */
	u32 package_count; /* in: number of packages */
	hwdps_result_t ret; /* out: return code */
};

struct hwdps_install_package_t {
	struct hwdps_package_info_t pinfo; /* package to install */
	hwdps_result_t ret; /* out: return code */
};

struct hwdps_uninstall_package_t {
	struct hwdps_package_info_t pinfo; /* package to uninstall */
	hwdps_result_t ret; /* out: return code */
};

/* magic number */
#define HWDPS_MAGIC 0x2001

/* ioctl commands */
#define HWDPS_SYNC_INSTALLED_PACKAGES _IOWR( \
	HWDPS_MAGIC, 14, struct hwdps_sync_packages_t *)
#define HWDPS_INSTALL_PACKAGE _IOWR(HWDPS_MAGIC, 15, \
	struct hwdps_install_package_t *)
#define HWDPS_UNINSTALL_PACKAGE _IOWR(HWDPS_MAGIC, 16, \
	struct hwdps_uninstall_package_t *)

void hwdps_sync_installed_packages(
	struct hwdps_sync_packages_t *sync_packages);

void hwdps_install_package(struct hwdps_install_package_t *install_package);

void hwdps_uninstall_package(
	struct hwdps_uninstall_package_t *uninstall_package);

#endif
