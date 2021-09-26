/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: eima_agent_api.h for api of eima agent.
 * Create: 2017-12-20
 */

#ifndef _EIMA_AGENT_API_H_
#define _EIMA_AGENT_API_H_
#include <securec.h>

#define EIMA_NAME_STRING_LEN 511

#define EIMA_MAX_MEASURE_OBJ_CNT 10 /* Maximum number of targets per policy metric */
#define EIMA_HASH_DIGEST_SIZE 32 /* sha256 */

/* Measurement result */
struct m_entry {
	uint8_t type; /* 1 for EIMA_STATIC; 0 for EIMA_DYNAMIC */
	uint8_t hash_len; /* The length of the hash */
	uint8_t hash[EIMA_HASH_DIGEST_SIZE]; /* Hash result storage array */
	uint16_t fn_len; /* Length of file name, eg: system/bin/teecd */
	uint8_t fn[EIMA_NAME_STRING_LEN + 1]; /* File name string to store array */
};

/* Message to save measurement results */
struct m_list_msg {
	uint8_t usecase[EIMA_NAME_STRING_LEN + 1]; /* Reserved */
	uint32_t num; /* Number of metrics */
	struct m_entry m_list[EIMA_MAX_MEASURE_OBJ_CNT]; /* Measure target array */
};

/* The type of message sent to the agent */
enum eima_msg_type {
	EIMA_MSG_TRUSTLIST,    /* Trustlist */
	EIMA_MSG_BASELINE,     /* Baseline */
	EIMA_MSG_RUNTIME_INFO, /* Dynamic metric */
};

/* Measure the type of target */
enum eima_measure_type {
	EIMA_DYNAMIC, /* dynamic */
	EIMA_STATIC,  /* Static */
};

/*
 * the interface Eima sends a message to the agent.
 * Input Value:
 *    type: Trustlist, baseline, or dynamic metrics
 *    mlist_msg: Message entity
 */

/*
 * char *get_hash_error_file_path(void)
 * get the tampered file path when the measurement of one process failed.
 * Return Value:
 *    one c style string, either "" or "/path/to/file"
 */
#ifdef CONFIG_TEE_ANTIROOT_CLIENT
extern int send_eima_data(int type, const struct m_list_msg *mlist_msg);

extern char *get_hash_error_file_path(void);
#else
static inline int send_eima_data(int type, const struct m_list_msg *mlist_msg)
{
	return 0;
}

static inline char *get_hash_error_file_path(void)
{
	return NULL;
}

static inline uint32_t get_tee_status(void)
{
	return 0;
}
#endif

#endif /* _EIMA_AGENT_API_H_ */
