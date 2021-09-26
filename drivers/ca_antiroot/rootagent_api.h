/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2020. All rights reserved.
 * Description: rootagent API
 * Create: 2016-04-01
 */
#ifndef _ROOTAGENT_API_H_
#define _ROOTAGENT_API_H_

#include "rootagent_common.h"

/* TA UID */
#define UUID_RM_TA TEE_SERVICE_ANTIROOT

#define UNAME_LENTH 20

/*
 * the offset of buffer which tansport physical address to TA,
 * such as start or end address of kcode
 */
#define OFFSET_KCODE_ADDR_START    0
#define OFFSET_KCODE_ADDR_END      1
#define OFFSET_SYSCALL_ADDR_START  2
#define OFFSET_SYSCALL_NUM         3
#define OFFSET_SEHOOKS_NUM         4
#define OFFSET_SEHOOKS_ADDR_START  5
#define REAL_OFFSET_LENGTH         1

#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
#define OFFSET_RODATA_ADDR_START   (OFFSET_SEHOOKS_ADDR_START + g_se_hooks_num)
#define OFFSET_RODATA_ADDR_END     (OFFSET_RODATA_ADDR_START + 1)
#define RODATA_OCCUPY_LENGTH       2
#endif

/*
 * @Function: root_monitor_tee_init
 *
 * @brief: This function initial TEE environment
 * (context, session), create two share memory,
 * one is used to read, and anther is used to write.
 *
 * @attention: This function should be invoked first.
 *
 * @param: void
 *
 * @retval: see teek_client_constants.h
 *
 * @depend on: teek_client_api.c teek_root_monitor_mem.c
 *
 */
TEEC_Result root_monitor_tee_init(void);

/*
 * @Function:  setting_config
 *
 * @brief: This function set the white information
 * which is used to do check operations
 * into the TEE environment
 *
 * @attention: When Linux kernel start up,
 * The function should be invoked
 * after root_monitor_tee_init, but before other functions.
 *
 * @param: TEE_RM_CONFIG, see teek_root_monitor_common.h
 *
 * @retval: see teek_client_constants.h
 *
 * @depend on: teek_client_api.c
 *
 */
TEEC_Result setting_config(const struct ragent_config *config,
			const struct ragent_rootproc *root_proc);

/*
 * @Function:  request_challenge
 *
 * @brief: This function request
 * a challenge generated
 * by TEE environment,
 * According to the challenge,
 * we do corresponding scanning and reply.
 *
 * @attention: null
 *
 * @param: void
 *
 * @retval: see teek_client_constants.h
 *
 * @depend on: teek_client_api.c teek_root_monitor_mem.c
 *
 */
TEEC_Result request_challenge(struct ragent_challenge *ichallenge);

/*
 * @Function:  feedback_response
 *
 * @brief: This function send the result of scanning to TEE,
 * and TEE compare the result with white information.
 *
 * @attention: This function should be invoked after request_challenge.
 *
 * @param: void
 *
 * @retval: see teek_client_constants.h
 *
 * @depend on: teek_client_api.c teek_root_monitor_mem.c
 *
 */
TEEC_Result feedback_response(const struct ragent_response *response,
			const struct ragent_rootproc *root_proc);

/* This function request challenge from antiroot TA */
TEEC_Result eima_request_challenge(void);

/* This function send result to antiroot TA */
TEEC_Result eima_send_response(int type, const struct m_list_msg *response);

/* This function finalize the resource create by tee_init */
void root_monitor_finalize(void);

/* upload the information of rodata to stp */
#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
extern void rodata_stp_upload(void);
#else
static inline void rodata_stp_upload(void)
{
	return;
}
#endif

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG
/*
 * @Function: tee_status_check
 * @brief: This function can create abnormal
 * to check the tee status
 * @retrun 0 trigger check OK, other trigger check failed
 */
int tee_status_check(int cmd_item);
#endif
#endif

#endif
