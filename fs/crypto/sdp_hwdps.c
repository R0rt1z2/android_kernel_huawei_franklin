/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Implementation of entrance to hwdps and sdp when open.
 * Create: 2020-06-16
 */

#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include <huawei_platform/hwdps/hwdps_fs_hooks.h>
#include "keyinfo_sdp.h"

int f2fs_get_crypt_keyinfo(struct inode *inode, void *fs_data, int *flag)
{
	int res = 0;
#ifdef F2FS_FS_SDP_ENCRYPTION
	u32 sdpflag = 0;
#endif
	if (!inode)
		return -EINVAL;
	if (!flag) /* fs_data can be null so not check it */
		return -EINVAL;

	/* 0 for getting original ce crypt info, otherwise be 1 */
	*flag = 0;
#ifdef CONFIG_HWDPS
	if (!inode->i_crypt_info || (inode->i_crypt_info &&
		((inode->i_crypt_info->ci_hw_enc_flag &
		HWDPS_ENABLE_FLAG) != 0))) {
		res = hwdps_get_context(inode);
		if (res == -EOPNOTSUPP) {
			goto get_sdp_encryption_info;
		} else if (res != 0) {
			*flag = 1; /* enabled */
			return -EACCES;
		}
		*flag = 1; /* enabled */
		return 0;
	}
get_sdp_encryption_info:
#endif

#ifdef F2FS_FS_SDP_ENCRYPTION
	if (!S_ISREG(inode->i_mode))
		return 0;
	down_write(&inode->i_sdp_sem);
	res = inode->i_sb->s_cop->get_sdp_encrypt_flags(inode, fs_data,
		&sdpflag);
	if (res != 0) {
		sdp_pr_err(" %s get flag error:%d\n", __func__, res);
		goto unlock;
	}

	/* get sdp crypt info */
	if (f2fs_inode_is_sdp_encrypted(sdpflag)) {
		*flag = 1; /* enabled */
		res = f2fs_get_sdp_crypt_info(inode, fs_data);
		if (res != 0)
			sdp_pr_err("%s:ino%lu get keyinfo failed res:%d\n",
				__func__, inode->i_ino, res);
	}
unlock:
	up_write(&inode->i_sdp_sem);
	return res;
#endif
	return res;
}
