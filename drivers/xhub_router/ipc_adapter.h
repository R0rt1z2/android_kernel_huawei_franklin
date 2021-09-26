/*
 *  Sensor Hub driver
 *
 * Copyright (C) 2012 Huawei, Inc.
 * Author: qindiwen <qindiwen@huawei.com>
 *
 */

#ifndef __HW_IPC_ADAPTER_H__
#define __HW_IPC_ADAPTER_H__
#include "scp_ipi.h"
#include "scp_helper.h"

#define HW_CUST_IPC

int ipc_adapter_send(const char *buf, unsigned int length);

#endif /* __HW_IPC_ADAPTER_H__ */
