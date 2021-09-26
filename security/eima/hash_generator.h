/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: the hash_generator.h for EMUI Integrity Measurement
 *     Architecture(EIMA)
 * Create: 2017-12-20
 */

#ifndef _HEADER_HASH_GENERATOR
#define _HEADER_HASH_GENERATOR

#include <crypto/hash.h>

/* Struct encapsulating the hash returned by the hash generator. */
struct t_hash {
	ssize_t len; /* Length of hash */
	u8 *hash; /* Bytes of the calculated hash digest */
	const char *alg; /* Algorithm used to calculate hash */
};

/*
 * This struct contains state information for hashing operations that span
 * several function invocations. It should never be used directly, use
 * hash_handle_init() to gain a valid instances of this struct.
 */
struct hashgen_handle {
	/* Container to store hash digest and algorithm information */
	struct t_hash *hash;
	/* Shash algorithm descriptor, for internal use only */
	struct crypto_shash *tfm;
	/* Working struct for shash algorithm, for internal use only */
	struct shash_desc *desc;
};

/*
 * Allocate and initialize a new @c hashgen_handle struct. The hash
 * generator will use the hash algorithm @c alg for its operation.
 * param:
 *     alg: Hash algorithm to be used, must be a valid shash
 *         algorithm string
 * return: Pointer to the initialized handle
 */
struct hashgen_handle *hash_handle_init(const char *alg);

/*
 * Add the data to the hashing algorithm.
 * input value:
 *     handle: Hash generator handle to operate on
 *     data: Data to add
 *     len: Length of the data
 * return value: 0 on success, non-zero otherwise
 */
int hash_handle_update(struct hashgen_handle *handle, u8 *data, size_t len);

/*
 * Finalize the hash algorithm in @c handle and retrieve the digest.
 * The digest will be stored in the @c hash struct element of the handle.
 * input value:
 *     handle: Handle to finalize
 * return: 0 on success, non-zero otherwise
 */
int hash_handle_final(struct hashgen_handle *handle);

/*
 * Free all memory allocated by a hash generator handle.
 * The provider pointer will be invalid when this function returns.
 * param handle: Handle to free
 */
void hash_handle_free(struct hashgen_handle *handle);

#endif
