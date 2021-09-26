/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description: Implementation of the userspace process guideline. It reads
 *     the code segments of the target process and all linked libraries. The
 *     contents are hashed and collected as @c result_entry struct instances.
 *     Additionally the guideline reads the flags on all hashed memory
 *     segments and searches for differences between PTE flags of the same
 *     segment.
 * Create: 2017-12-20
 */

#ifndef _DKM_H_
#define _DKM_H_

#include <linux/sched.h>

typedef struct {
	struct mm_struct *mm;
	struct file *target_file;
} process_msg_t;

/*
 * Description: This function reads the code segments of the target process
 *    and all linked libraries
 * input value:
 *    target_proc is the target product struct
 *    measure_libs is the process linked libraries number
 *    hash is the output parameter to store the contents hash value
 *    hash_len is the hash length
 *    hlen is the output parameter actual storage length
 * output value:
 *    0: success
 *    others: failed
 */
int measure_process(struct task_struct *target_proc, int measure_libs,
		char *hash, int hash_len, int *hlen);

#define HASH_ALG "sha256"
#define HASH_ALG_384 "sha384"
#define EIMA_SHA384_DIGEST_SIZE 48 /* sha384 */

#endif
