/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: the eima_fs.h for EMUI Integrity Measurement Architecture(EIMA)
 * Create: 2017-12-20
 */

#ifndef _EIMA_FS_H_
#define _EIMA_FS_H_

#include "eima_agent_api.h"

#define EIMA_POLICY_USECASE_NAME     "default"

/* judge the Running environment of eima */
#ifdef _EIMA_RUN_ON_QEMU
#define EIMA_POLICY_PATH_TEECD        "/bin/teecd"
#else
#define EIMA_POLICY_PATH_TEECD        "/vendor/bin/teecd"
#endif

#define EIMA_POLICY_PATH_SYSTEM_TEECD "/system/bin/system_teecd"
#define EIMA_POLICY_PATH_DEBUGGERD    "/system/bin/debuggerd"
#define EIMA_POLICY_PATH_TOMBSTONED   "/system/bin/tombstoned"

#ifdef CONFIG_HUAWEI_ENG_EIMA
#define EIMA_POLICY_PATH_HELLO        "/data/log/hello"
#endif

#define EIMA_PIF_TLV_TYPE_SEQ    0x30
#define EIMA_PIF_TLV_TYPE_STRING 0x13
#define EIMA_PIF_TLV_TYPE_ENUM   0x0a
#define EIMA_PIF_TLV_TYPE_INT    0x02

#define EIMA_MAX_POLICY_CNT      2
#define EIMA_PIF_SIGNATURE_LEN   256

typedef struct {
	uint8_t file[EIMA_NAME_STRING_LEN + 1];
	uint8_t type;
	uint8_t exception;
	uint8_t sent_baseline;
	uint8_t rev[1]; /* Reserve memory for byte alignment */
	pid_t target_pid;
} integrity_targets_t;

typedef struct {
	uint8_t usecase_name[EIMA_NAME_STRING_LEN + 1];
	uint8_t target_count;
	uint8_t rev[3]; /* Reserve memory for byte alignment */
	integrity_targets_t targets[EIMA_MAX_MEASURE_OBJ_CNT];
} policy_t;

typedef struct {
	uint8_t version;
	uint8_t policy_count;
	uint8_t rev[2]; /* Reserve memory for byte alignment */
	policy_t policy[EIMA_MAX_POLICY_CNT];
	uint32_t sign_algo;
	uint8_t signature[EIMA_PIF_SIGNATURE_LEN];
} pif_struct_t;

struct file *eima_file_open(const char *path, int flag, umode_t mode);

void eima_file_close(struct file *filp);

int __init eima_fs_init(void);
void eima_fs_exit(void);

extern pif_struct_t g_eima_pif;

#endif /* _EIMA_FS_H_ */
