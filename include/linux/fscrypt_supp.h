/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fscrypt_supp.h
 *
 * Do not include this file directly. Use fscrypt.h instead!
 */
#ifndef _LINUX_FSCRYPT_H
#error "Incorrect include of linux/fscrypt_supp.h!"
#endif

#ifndef _LINUX_FSCRYPT_SUPP_H
#define _LINUX_FSCRYPT_SUPP_H

#include <linux/mm.h>
#include <linux/slab.h>
#include <crypto/aead.h>

/*
 * fscrypt superblock flags
 */
#define FS_CFLG_OWN_PAGES (1U << 1)

#ifndef CONFIG_FS_UNI_ENCRYPTION
#define FS_KEY_DERIVATION_NONCE_SIZE	16
#else
#define FS_KEY_DERIVATION_NONCE_SIZE		64
#endif
#define FS_KEY_DERIVATION_TAG_SIZE	16
#define FS_KEY_DERIVATION_IV_SIZE	16
#define FS_KEY_DERIVATION_CIPHER_SIZE	80 /* nonce + tag */

/*
 * fscrypt_info - the "encryption key" for an inode
 *
 * When an encrypted file's key is made available, an instance of this struct is
 * allocated and stored in ->i_crypt_info.  Once created, it remains until the
 * inode is evicted.
 */
struct fscrypt_info {

	/* The actual crypto transform used for encryption and decryption */
	struct crypto_skcipher *ci_ctfm;

	/*
	 * Cipher for ESSIV IV generation.  Only set for CBC contents
	 * encryption, otherwise is NULL.
	 */
	struct crypto_cipher *ci_essiv_tfm;
	struct crypto_aead *ci_gtfm;

	/*
	 * Encryption mode used for this inode.  It corresponds to either
	 * ci_data_mode or ci_filename_mode, depending on the inode type.
	 */
	struct fscrypt_mode *ci_mode;

	/*
	 * If non-NULL, then this inode uses a master key directly rather than a
	 * derived key, and ci_ctfm will equal ci_master_key->mk_ctfm.
	 * Otherwise, this inode uses a derived key.
	 */
	struct fscrypt_master_key *ci_master_key;

	/* fields from the fscrypt_context */
	u8 ci_format;
	u8 ci_data_mode;
	u8 ci_filename_mode;
	u8 ci_flags;
	u8 ci_status;
	atomic_t ci_count;
	spinlock_t ci_lock;
	u8 ci_master_key_descriptor[FS_KEY_DESCRIPTOR_SIZE];
	u8 ci_nonce[FS_KEY_DERIVATION_NONCE_SIZE];
	u8 ci_raw_key[FS_MAX_KEY_SIZE];
	u8 ci_hw_enc_flag;
#ifdef CONFIG_FS_UNI_ENCRYPTION
	void *ci_key;
	int ci_key_len;
	int ci_key_index;
#endif
};

static inline void *fscrypt_ci_key(struct inode *inode)
{
#if IS_ENABLED(CONFIG_FS_UNI_ENCRYPTION)
	return inode->i_crypt_info->ci_key;
#else
	return NULL;
#endif
}

static inline int fscrypt_ci_key_len(struct inode *inode)
{
#if IS_ENABLED(CONFIG_FS_UNI_ENCRYPTION)
	return inode->i_crypt_info->ci_key_len;
#else
	return 0;
#endif
}

static inline int fscrypt_ci_key_index(struct inode *inode)
{
#if IS_ENABLED(CONFIG_FS_UNI_ENCRYPTION)
	return inode->i_crypt_info->ci_key_index;
#else
	return -1;
#endif
}

/*
 * crypto operations for filesystems
 */
struct fscrypt_operations {
	unsigned int flags;
	const char *key_prefix;
	int (*get_context)(struct inode *, void *, size_t);
	int (*set_context)(struct inode *, const void *, size_t, void *);
	bool (*dummy_context)(struct inode *);
#ifdef CONFIG_FS_UNI_ENCRYPTION
	bool (*is_inline_encrypted)(struct inode *);
#endif
	bool (*empty_dir)(struct inode *);
	unsigned int max_namelen;
	int (*get_keyinfo)(struct inode *, void *, int *);
#ifdef CONFIG_HWDPS
	int (*set_hwdps_attr)(struct inode *, const void *, size_t, void *);
	int (*set_hwdps_flags)(struct inode *, void *, u32 *);
	int (*update_hwdps_attr)(struct inode *, const void *, size_t, void *);
	int (*get_hwdps_attr)(struct inode *, void *, size_t, struct page *);
	int (*get_hwdps_flags)(struct inode *, void *, u32 *);
#endif
	int (*is_file_sdp_encrypted)(struct inode *);
	int (*get_sdp_context)(struct inode *, void *, size_t, void *);
	int (*set_sdp_context)(struct inode *, const void *, size_t, void *);
	int (*update_sdp_context)(struct inode *, const void *, size_t, void *);
	int (*update_context)(struct inode *, const void *, size_t, void *);
	int (*get_sdp_encrypt_flags)(struct inode *, void *, u32 *);
	int (*set_sdp_encrypt_flags)(struct inode *, void *, u32 *);
};

struct fscrypt_ctx {
	union {
		struct {
			struct page *bounce_page;	/* Ciphertext page */
			struct page *control_page;	/* Original page  */
		} w;
		struct {
			struct bio *bio;
			struct work_struct work;
		} r;
		struct list_head free_list;	/* Free list */
	};
	u8 flags;				/* Flags */
};

static inline bool fscrypt_has_encryption_key(const struct inode *inode)
{
	return (inode->i_crypt_info != NULL);
}

static inline bool fscrypt_dummy_context_enabled(struct inode *inode)
{
	return inode->i_sb->s_cop->dummy_context &&
		inode->i_sb->s_cop->dummy_context(inode);
}

/* crypto.c */
extern void fscrypt_enqueue_decrypt_work(struct work_struct *);
extern struct fscrypt_ctx *fscrypt_get_ctx(const struct inode *, gfp_t);
extern void fscrypt_release_ctx(struct fscrypt_ctx *);
extern struct page *fscrypt_encrypt_page(const struct inode *, struct page *,
						unsigned int, unsigned int,
						u64, gfp_t);
extern int fscrypt_decrypt_page(const struct inode *, struct page *, unsigned int,
				unsigned int, u64);

static inline struct page *fscrypt_control_page(struct page *page)
{
	return ((struct fscrypt_ctx *)page_private(page))->w.control_page;
}

extern void fscrypt_restore_control_page(struct page *);

/* policy.c */
extern int fscrypt_ioctl_set_policy(struct file *, const void __user *);
extern int fscrypt_ioctl_get_policy(struct file *, void __user *);
extern int fscrypt_has_permitted_context(struct inode *, struct inode *);
extern int fscrypt_inherit_context(struct inode *, struct inode *,
					void *, bool);
#ifdef CONFIG_HWDPS
extern int hwdps_get_context(struct inode *);
extern int hwdps_update_context(struct inode *inode, uid_t new_uid);
#endif
extern int fscrypt_set_bio_ctx(struct inode *inode, struct bio *bio);
extern int fscrypt_key_payload(struct bio_crypt_ctx *ctx,
				const unsigned char **key);
extern int fscrypt_is_hw_encrypt(const struct inode *inode);
extern int fscrypt_is_sw_encrypt(const struct inode *inode);

/* keyinfo.c */
extern int fscrypt_set_gcm_key(struct crypto_aead *, const u8 *);
extern int fscrypt_derive_gcm_key(struct crypto_aead *,
				const u8 *, u8 *, u8 *, int);
extern struct key *fscrypt_request_key(const u8 *, const u8 *, int);
extern int fscrypt_get_encryption_info(struct inode *);
extern void fscrypt_put_encryption_info(struct inode *);
extern void *fscrypt_crypt_info_act(void *ci, int act);
extern int file_protect_set_gcm_key(struct crypto_aead *tfm,
	const u8 *derive_key, u32 derive_key_len);
extern int file_protect_derive_gcm_key(struct crypto_aead *tfm,
	const u8 *src, u32 src_len, u8 *dst, u32 dst_len, u8 *iv, u32 iv_len,
	int enc);
/* fname.c */
extern int fscrypt_setup_filename(struct inode *, const struct qstr *,
				int lookup, struct fscrypt_name *);

static inline void fscrypt_free_filename(struct fscrypt_name *fname)
{
	kfree(fname->crypto_buf.name);
}

extern int fscrypt_fname_alloc_buffer(const struct inode *, u32,
				struct fscrypt_str *);
extern void fscrypt_fname_free_buffer(struct fscrypt_str *);
extern int fscrypt_fname_disk_to_usr(struct inode *, u32, u32,
			const struct fscrypt_str *, struct fscrypt_str *);

#define FSCRYPT_FNAME_MAX_UNDIGESTED_SIZE	32

/* Extracts the second-to-last ciphertext block; see explanation below */
#define FSCRYPT_FNAME_DIGEST(name, len)	\
	((name) + round_down((len) - FS_CRYPTO_BLOCK_SIZE - 1, \
			     FS_CRYPTO_BLOCK_SIZE))

#define FSCRYPT_FNAME_DIGEST_SIZE	FS_CRYPTO_BLOCK_SIZE

/**
 * fscrypt_digested_name - alternate identifier for an on-disk filename
 *
 * When userspace lists an encrypted directory without access to the key,
 * filenames whose ciphertext is longer than FSCRYPT_FNAME_MAX_UNDIGESTED_SIZE
 * bytes are shown in this abbreviated form (base64-encoded) rather than as the
 * full ciphertext (base64-encoded).  This is necessary to allow supporting
 * filenames up to NAME_MAX bytes, since base64 encoding expands the length.
 *
 * To make it possible for filesystems to still find the correct directory entry
 * despite not knowing the full on-disk name, we encode any filesystem-specific
 * 'hash' and/or 'minor_hash' which the filesystem may need for its lookups,
 * followed by the second-to-last ciphertext block of the filename.  Due to the
 * use of the CBC-CTS encryption mode, the second-to-last ciphertext block
 * depends on the full plaintext.  (Note that ciphertext stealing causes the
 * last two blocks to appear "flipped".)  This makes accidental collisions very
 * unlikely: just a 1 in 2^128 chance for two filenames to collide even if they
 * share the same filesystem-specific hashes.
 *
 * However, this scheme isn't immune to intentional collisions, which can be
 * created by anyone able to create arbitrary plaintext filenames and view them
 * without the key.  Making the "digest" be a real cryptographic hash like
 * SHA-256 over the full ciphertext would prevent this, although it would be
 * less efficient and harder to implement, especially since the filesystem would
 * need to calculate it for each directory entry examined during a search.
 */
struct fscrypt_digested_name {
	u32 hash;
	u32 minor_hash;
	u8 digest[FSCRYPT_FNAME_DIGEST_SIZE];
};

/**
 * fscrypt_match_name() - test whether the given name matches a directory entry
 * @fname: the name being searched for
 * @de_name: the name from the directory entry
 * @de_name_len: the length of @de_name in bytes
 *
 * Normally @fname->disk_name will be set, and in that case we simply compare
 * that to the name stored in the directory entry.  The only exception is that
 * if we don't have the key for an encrypted directory and a filename in it is
 * very long, then we won't have the full disk_name and we'll instead need to
 * match against the fscrypt_digested_name.
 *
 * Return: %true if the name matches, otherwise %false.
 */
static inline bool fscrypt_match_name(const struct fscrypt_name *fname,
				      const u8 *de_name, u32 de_name_len)
{
	if (unlikely(!fname->disk_name.name)) {
		const struct fscrypt_digested_name *n =
			(const void *)fname->crypto_buf.name;
		if (WARN_ON_ONCE(fname->usr_fname->name[0] != '_'))
			return false;
		if (de_name_len <= FSCRYPT_FNAME_MAX_UNDIGESTED_SIZE)
			return false;
		return !memcmp(FSCRYPT_FNAME_DIGEST(de_name, de_name_len),
			       n->digest, FSCRYPT_FNAME_DIGEST_SIZE);
	}

	if (de_name_len != fname->disk_name.len)
		return false;
	return !memcmp(de_name, fname->disk_name.name, fname->disk_name.len);
}

/* bio.c */
extern void fscrypt_decrypt_bio(struct bio *);
extern void fscrypt_enqueue_decrypt_bio(struct fscrypt_ctx *ctx,
					struct bio *bio);
extern void fscrypt_pullback_bio_page(struct page **, bool);
extern int fscrypt_zeroout_range(const struct inode *, pgoff_t, sector_t,
				 unsigned int);

/* hooks.c */
extern int fscrypt_file_open(struct inode *inode, struct file *filp);
extern int __fscrypt_prepare_link(struct inode *inode, struct inode *dir,
				  struct dentry *dentry);
extern int __fscrypt_prepare_rename(struct inode *old_dir,
				    struct dentry *old_dentry,
				    struct inode *new_dir,
				    struct dentry *new_dentry,
				    unsigned int flags);
extern int __fscrypt_prepare_lookup(struct inode *dir, struct dentry *dentry,
				    struct fscrypt_name *fname);
extern int __fscrypt_prepare_symlink(struct inode *dir, unsigned int len,
				     unsigned int max_len,
				     struct fscrypt_str *disk_link);
extern int __fscrypt_encrypt_symlink(struct inode *inode, const char *target,
				     unsigned int len,
				     struct fscrypt_str *disk_link);
extern const char *fscrypt_get_symlink(struct inode *inode, const void *caddr,
				       unsigned int max_size,
				       struct delayed_call *done);

#endif	/* _LINUX_FSCRYPT_SUPP_H */
