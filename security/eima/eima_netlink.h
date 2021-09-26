/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2018-2020. All rights reserved.
 * Description: eima_netlink.h for eima netlink communication in kernel
 * Create: 2018-10-24
 */

#ifndef _LINUX_EIMA_NETLINK_H
#define _LINUX_EIMA_NETLINK_H

#if IS_ENABLED(CONFIG_HUAWEI_EIMA_ACCESS_CONTROL)
extern int eima_netlink_init(void);
extern void eima_netlink_destroy(void);
#else
static inline int eima_netlink_init(void)
{
	return 0;
}
static inline void eima_netlink_destroy(void) { }

#endif

#endif
