/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: the eima_queue.h for eima add template entry and destroy
 *     measurements list.
 * Create: 2017-12-20
 */

#ifndef _EIMA_QUEUE_H_
#define _EIMA_QUEUE_H_

#include "eima_agent_api.h"

#define EIMA_STATIC_MEASURE_PCR_IDX 11  /* TPM PCR Index for static EIMA */
#define EIMA_DYNAMIC_MEASURE_PCR_IDX 12 /* TPM PCR Index for dynamic EIMA */

extern int g_eima_used_tpm;

struct eima_template_entry {
	u8 digest[EIMA_HASH_DIGEST_SIZE]; /* measurement hash */
	uint8_t fn[EIMA_NAME_STRING_LEN + 1];
	u8 type;
	u8 rev[3]; /* reserve memory for byte alignment */
};

struct eima_queue_entry {
	struct hlist_node hnext; /* place in hash collision list */
	struct list_head later;  /* place in ima_measurements list */
	struct eima_template_entry entry;
};

extern struct list_head g_eima_measurements_list; /* list of all measurements */

#define EIMA_HASH_BITS 9
#define EIMA_MEASURE_HTABLE_SIZE (1 << EIMA_HASH_BITS)

struct eima_h_table {
	atomic_long_t count; /* number of stored measurements in the list */
	atomic_long_t violations;
	struct hlist_head queue[EIMA_MEASURE_HTABLE_SIZE];
};

int eima_add_template_entry(struct eima_template_entry *entry);
void eima_destroy_measurements_list(void);

#endif /* _EIMA_QUEUE_H_ */
