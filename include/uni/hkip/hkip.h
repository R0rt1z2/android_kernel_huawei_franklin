/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: HKIP hooks
 * Creator: security-ap
 * Date: 2017/10/15
 */

#ifndef LINUX_HKIP_H
#define LINUX_HKIP_H

#include <asm/alternative.h>
#include <asm/cpufeature.h>
#include <linux/types.h>
#include <uni/hkip/hkip_hvc.h>

static inline bool hhee_is_present(void)
{
	bool ret = false;
#ifdef CONFIG_HUAWEI_HKIP
	if (hhee_check_enable() == HHEE_ENABLE)
		ret = true;
#endif
	return ret;
}

u32 hkip_hvc2(u32, unsigned long);
u32 hkip_hvc3(u32, unsigned long, unsigned long);
u32 hkip_hvc4(u32, unsigned long, unsigned long, unsigned long);

#ifdef CONFIG_HUAWEI_HKIP
int hkip_register_xo(const uintptr_t base, size_t size);
#else
static inline int hkip_register_xo(const uintptr_t base, size_t size)
{
	return 0;
}
#endif

#endif /* LINUX_HKIP_H */
