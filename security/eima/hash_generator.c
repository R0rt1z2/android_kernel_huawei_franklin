/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: the hash_generator.c for EMUI Integrity Measurement
 *     Architecture(EIMA)
 * Create: 2017-12-20
 */

#include "hash_generator.h"

#include <crypto/hash.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>

#include "eima_utils.h"

#define PRINT_DKM_INFO_TIMES 2
#define PRINT_ROW_SIZE 32

/*
 * Allocate memory for a shash description struct. The memory required by this
 * struct is actual larger than the size of the struct itself. If must be
 * appended by a specific amount of bytes which is returned by the
 * crypto_shash_descsize function. This additional memory is used by the
 * hash algorithm as additional working space.
 * input value:
 *     tfm: Configured and allocated hash algorithm
 * return: Pointer to the allocated description struct
 */
static struct shash_desc *__alloc_shash_desc(struct crypto_shash *tfm)
{
	struct shash_desc *rv = NULL;

	rv = kmalloc(sizeof(*rv) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (rv == NULL)
		eima_error("Unable alloc shash_desc");

	return rv;
}

/*
 * Free the memory of a shash description struct.
 * desc: Description struct to free
 */
static void __free_hash_desc(struct shash_desc *desc)
{
	if (desc != NULL)
		kfree(desc);
}

/*
 * Test if the provided hash generator handle is already initialized or not.
 * input value:
 *     handle: Handle to test
 * return value: true if initialized, false otherwise
 */
static bool __handle_is_init(struct hashgen_handle *handle)
{
	if (handle == NULL)
		return false;

	if (handle->tfm == NULL)
		return false;

	if (handle->hash == NULL)
		return false;

	return true;
}

static struct t_hash *alloc_t_hash(const char *alg)
{
	struct t_hash *hash = NULL;
	struct crypto_shash *tfm = NULL;

	hash = kmalloc(sizeof(*hash), GFP_KERNEL);
	if (hash == NULL) {
		eima_error("Could not allocate memory for t_hash");
		goto fn_error;
	}

	hash->alg = alg;

	tfm = crypto_alloc_shash(hash->alg, 0, 0);
	if (IS_ERR(tfm)) {
		eima_error("Unable to allocate hash memory (%ld)",
			PTR_ERR(tfm));
		goto fn_error_free;
	}

	hash->len = crypto_shash_digestsize(tfm);

	hash->hash = kzalloc(hash->len, GFP_KERNEL);
	if (hash->hash == NULL) {
		eima_error("Could not allocate memory for hash digest storage");
		goto fn_error_free_tfm;
	}

	crypto_free_shash(tfm);
	return hash;

fn_error_free_tfm:
	crypto_free_shash(tfm);

fn_error_free:
	kfree(hash);

fn_error:
	return NULL;
}

static void free_t_hash(struct t_hash *hash)
{
	if (hash->hash != NULL)
		kfree(hash->hash);

	kfree(hash);
}

struct hashgen_handle *hash_handle_init(const char *alg)
{
	int ret;
	struct hashgen_handle *handle = NULL;

	if (alg == NULL) {
		eima_error("Bad params");
		return NULL;
	}
	eima_debug("Hash generator - handle init - using algorithm '%s'", alg);

	handle = kmalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL) {
		eima_error("Could not allocate memory for hashgen_handle");
		goto fn_error;
	}

	/* Allocate memory for the specified shash algorithm */
	handle->tfm = crypto_alloc_shash(alg, 0, 0);
	if (IS_ERR(handle->tfm)) {
		eima_error("Unable to allocate hash memory (%ld)",
			PTR_ERR(handle->tfm));
		goto fn_error_free_handle;
	}

	handle->hash = alloc_t_hash(alg);
	if (handle->hash == NULL)
		goto fn_error_free_tfm;

	/* Allocate memory for the shash description struct */
	handle->desc = __alloc_shash_desc(handle->tfm);
	if (handle->desc == NULL) {
		eima_error("Could not allocate shash_desc");
		goto fn_error_free_hash;
	}

	handle->desc->tfm = handle->tfm;
	handle->desc->flags = CRYPTO_TFM_REQ_MAY_SLEEP;

	/* Initialize hash algorithm */
	ret = crypto_shash_init(handle->desc);
	if (ret < 0) {
		eima_error("crypto_shash_init error: %d", ret);
		goto fn_error_free;
	}

	return handle;

fn_error_free:
	__free_hash_desc(handle->desc);

fn_error_free_hash:
	free_t_hash(handle->hash);

fn_error_free_tfm:
	crypto_free_shash(handle->tfm);

fn_error_free_handle:
	kfree(handle);

fn_error:
	return NULL;
}

int hash_handle_update(struct hashgen_handle *handle, u8 *data, size_t len)
{
	if (__handle_is_init(handle) && (data != NULL)) {
		/* Calculate and retrieve the hash digest */
		crypto_shash_update(handle->desc, data, len);
		return 0;
	} else {
		return -1;
	}
}

int hash_handle_final(struct hashgen_handle *handle)
{
	if (__handle_is_init(handle)) {
		/* Calculate and retrieve the hash digest */
		crypto_shash_final(handle->desc, handle->hash->hash);
		return 0;
	} else {
		return -1;
	}
}

void hash_handle_free(struct hashgen_handle *handle)
{
	if (handle != NULL) {
		__free_hash_desc(handle->desc);
		crypto_free_shash(handle->tfm);
		free_t_hash(handle->hash);
		kfree(handle);
	}
}

