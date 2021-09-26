/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: define entrance to hwdps and sdp when open
 * Create: 2020-06-16
 */

#ifndef _SDP_HWDPS_H
#define _SDP_HWDPS_H

/*
 * This function is called by keyinfo.c to generate i_crypt_info
 */
int f2fs_get_crypt_keyinfo(struct inode *inode, void *fs_data, int *flag);

#endif
