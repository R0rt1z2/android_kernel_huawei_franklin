/*
 * cfi_report.c
 *
 * test the basic function of cfi
 *
 * Copyright (c) 2017-2019 Huawei Technologies Co., Ltd.
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
#include <chipset_common/security/saudit.h>
#ifdef CONFIG_HUAWEI_VENDOR_EXCEPTION
#include <huawei_platform/vendor_exception/vendor_exception.h>
#else
#include <linux/bug.h>
#endif

void __cfi_report(void)
{
	pr_err("CFI exception detected\n");
	saudit_log(CFI, STP_RISK, 0, NULL);

#ifdef CONFIG_HUAWEI_VENDOR_EXCEPTION
	VENDOR_EXCEPTION(VENDOR_MODID_AP_S_CFI, 0, 0);
#else
	BUG();
#endif
}
EXPORT_SYMBOL(__cfi_report);

void __efistub___cfi_report(void)
{
	__cfi_report();
}
EXPORT_SYMBOL(__efistub___cfi_report);
