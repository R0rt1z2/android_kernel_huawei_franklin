/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: the eima_queue.c for eima add template entry and destroy
 *     measurements list.
 * Create: 2017-12-20
 */

#include "eima_queue.h"

#include <linux/hash.h>
#include <linux/module.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <linux/tpm.h>
#include <linux/version.h>

#include "eima_agent_api.h"
#include "eima_utils.h"

LIST_HEAD(g_eima_measurements_list); /* list of all measurements */

/*lint -save -e651*/
/* key: inode (before secure-hashing a file) */
static struct eima_h_table g_eima_htable = {
	.count = ATOMIC_LONG_INIT(0),
	.violations = ATOMIC_LONG_INIT(0),
	.queue[0 ... EIMA_MEASURE_HTABLE_SIZE - 1] = HLIST_HEAD_INIT
};
/*lint -restore*/

/*
 * mutex protects atomicity of extending measurement list
 * and extending the TPM PCR aggregate. Since tpm_extend can take
 * long (and the tpm driver uses a mutex), we can't use the spinlock.
 */
static DEFINE_MUTEX(eima_extend_list_mutex);

/* lookup up the digest value in the hash table, and return the entry */
static struct eima_queue_entry *eima_lookup_digest_entry(u8 *digest_value,
							size_t digest_len)
{
	struct eima_queue_entry *qe = NULL;
	struct eima_queue_entry *ret = NULL;
	unsigned int key;
	int rc;

	if ((digest_value == NULL) || (digest_len == 0)) {
		eima_error("the parameter is null");
		return NULL;
	}

	key = hash_long(*digest_value, EIMA_HASH_BITS); // get eima hash key
	rcu_read_lock();
	hlist_for_each_entry_rcu(qe, &g_eima_htable.queue[key], hnext) {
		rc = memcmp(qe->entry.digest, digest_value, digest_len);
		if (rc == 0) {
			ret = qe;
			break;
		}
	}
	rcu_read_unlock();
	return ret;
}

/*
 * Add template entry to measurement list and hash table.
 * (Called with eima_extend_list_mutex held.)
 */
/*lint -save -e429*/
static int eima_add_digest_entry(struct eima_template_entry *entry)
{
	struct eima_queue_entry *qe = NULL;
	unsigned int key;
	int s_ret;

	qe = kmalloc(sizeof(*qe), GFP_KERNEL);
	if (qe == NULL) {
		eima_error("out of memory error creating queue entry");
		return -ENOMEM;
	}

	s_ret = memcpy_s(&qe->entry, sizeof(struct eima_template_entry),
		entry, sizeof(struct eima_template_entry));
	if (s_ret != EOK) {
		kfree(qe);
		eima_error("memcpy_s failed, s_ret = %d", s_ret);
		return s_ret;
	}

	INIT_LIST_HEAD(&qe->later);

	key = hash_long(*(entry->digest), EIMA_HASH_BITS); // get eima hash key

	mutex_lock(&eima_extend_list_mutex);

	list_add_tail_rcu(&qe->later, &g_eima_measurements_list);

	hlist_add_head_rcu(&qe->hnext, &g_eima_htable.queue[key]);

	mutex_unlock(&eima_extend_list_mutex);

	atomic_long_inc(&g_eima_htable.count);

	return s_ret;
}

/*lint -restore*/
static int eima_pcr_extend(const u8 *hash, int hash_len, u8 type)
{
	int ret = 0;
	int pcr_idx;

	if (!g_eima_used_tpm)
		return ret;

	if (hash_len == 0) {
		eima_error("the parameter is null");
		return -1;
	}

	if (type == EIMA_STATIC)
		pcr_idx = EIMA_STATIC_MEASURE_PCR_IDX;
	else
		pcr_idx = EIMA_DYNAMIC_MEASURE_PCR_IDX;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0)
	ret = tpm_pcr_extend(NULL, pcr_idx, hash);
#else
	ret = tpm_pcr_extend(TPM_ANY_NUM, pcr_idx, hash);
#endif
	if (ret != 0)
		eima_error("Error Communicating to TPM chip, result: %d", ret);

	return ret;
}

/*
 * Add template entry to the measurement list and hash table,
 * and extend the pcr.
 */
int eima_add_template_entry(struct eima_template_entry *entry)
{
	u8 digest[EIMA_HASH_DIGEST_SIZE] = {0};
	int result;
	int tpm_result;
	int s_ret;

	if (entry == NULL) {
		eima_error("Bad expected parameter");
		return -1;
	}

	s_ret = memcpy_s(digest, sizeof(digest), entry->digest, EIMA_HASH_DIGEST_SIZE);
	if (s_ret != EOK) {
		eima_error("memcpy_s failed, s_ret = %d", s_ret);
		return s_ret;
	}

	if (eima_lookup_digest_entry(digest, sizeof(digest))) {
		result = -EEXIST;
		goto out;
	}

	result = eima_add_digest_entry(entry);
	if (result != EOK) {
		eima_error("the result of eima add digest entry = %d", result);
		goto out;
	}

	eima_debug("eima add template entry, name %s, type %d",
		entry->fn, entry->type);

	tpm_result = eima_pcr_extend(digest, sizeof(digest), entry->type);
	/* wait mobile office to decide the next operation */
	if (tpm_result != 0) {
		eima_error("the result of eima pcr extend = %d", tpm_result);
		goto out;
	}

out:
	return result;
}

void eima_destroy_measurements_list(void)
{
	struct eima_queue_entry *pos = NULL;
	struct eima_queue_entry *n = NULL;
	struct hlist_node *m = NULL;
	int i;

	mutex_lock(&eima_extend_list_mutex);
	for (i = 0; i < EIMA_MEASURE_HTABLE_SIZE; i++) {
		hlist_for_each_entry_safe(pos, m, &g_eima_htable.queue[i],
					hnext)
			hlist_del(&pos->hnext);
	}

	list_for_each_entry_safe(pos, n, &g_eima_measurements_list, later) {
		list_del(&pos->later);
		kfree(pos);
	}
	mutex_unlock(&eima_extend_list_mutex);

	eima_debug("destroyed the eima measurement list!");
}
