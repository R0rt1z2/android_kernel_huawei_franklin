/* Copyright (C) 2020-2023 huawei Software Operations
 * Description: ufs-inline driver file
 */
#ifndef _UFS_INLINE_H
#define _UFS_INLINE_H

#include "ufshcd.h"

#if defined(CONFIG_SCSI_UFS_ENHANCED_INLINE_CRYPTO)
/* use #0~29 key index */
#define MAX_CRYPTO_KEY_INDEX 30
#define TEMP_BUFF_LENGTH 13
#define UFS_CCI_KEY_LEN 65
/* hash algorithm typical seed */
#define UFS_INLINE_HASH_SEED 131
#define UFS_INLINE_HASH_MASK 0xFFFFFFFF
#define UFS_VOLATE_DEFAULT_VALUE 0

#define UFS_SLOT_NUM_MAX 31
#define UFS_DUN_MASK 0xFFFFFFFF
#define UFS_UPPER_LOW_OFFSET 32
#define UFS_HEADER_DWORD_0 0
#define UFS_HEADER_DWORD_1 1
#define UFS_HEADER_DWORD_3 2
#define UFS_HEADER_DWORD_12 12

/* 102: 1xx stand for MTK;  2 stand fo version2.0 */
#define UFS_INLINE_VERSION 102

enum ufs_inline_debug_state {
	DEBUG_LOG_OFF,
	DEBUG_LOG_ON,
	DEBUG_CRYPTO_OFF,
	DEBUG_CRYPTO_ON,
};

#define DEBUG_DTS_INLINE_OFF    0
#define DEBUG_DTS_INLINE_ON     1

bool ufshcd_support_inline_encrypt(const struct ufs_hba *hba);
void ufs_inline_crypto_attr(struct ufs_hba *hba);
#ifdef CONFIG_HUAWEI_KERNEL_DEBUG
void ufs_inline_crypto_debug_init(struct ufs_hba *hba);
#endif
void ufshcd_prepare_req_desc_uie(const struct ufs_hba *hba,
			struct ufshcd_lrb *lrbp);
int ufshcd_hba_uie_init(const struct ufs_hba *hba);
#else
static inline __attribute__((unused)) bool ufshcd_support_inline_encrypt(
		const struct ufs_hba *hba __attribute__((unused)))
{
	return 0;
}

static inline __attribute__((unused)) void ufs_inline_crypto_attr(
		struct ufs_hba *hba __attribute__((unused)))
{
	return;
}
static inline __attribute__((unused)) void ufs_inline_crypto_debug_init(
		struct ufs_hba *hba __attribute__((unused)))
{
	return;
}

static inline __attribute__((unused)) void ufshcd_prepare_req_desc_uie(
		const struct ufs_hba *hba __attribute__((unused)),
		struct ufshcd_lrb *lrbp __attribute__((unused)))
{
	return;
}
static inline __attribute__((unused)) int ufshcd_hba_uie_init(
		const struct ufs_hba *hba __attribute__((unused)))
{
	return 0;
}
#endif /* end CONFIG_SCSI_UFS_ENHANCED_INLINE_CRYPTO */
#endif /* !_UFS_INLINE_H */
