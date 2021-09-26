/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: the rootagent.h for kernel space root scan pause and resume.
 * Create: 2016-5-19
 */

#ifndef ROOT_AGENT_H
#define ROOT_AGENT_H

#include <linux/types.h>

enum tee_rs_mask {
	ROOTSTATE_BIT   = 0,            /* 0    on */
	/* read from fastboot */
	OEMINFO_BIT,                    /* 1    on */
	FBLOCK_YELLOW_BIT,              /* 2    on */
	FBLOCK_RED_BIT,                 /* 3    on */
	FBLOCK_ORANGE_BIT,              /* 4    on */
	/* dy scan result */
	KERNELCODEBIT   = 6,            /* 6    on */
	SYSTEMCALLBIT,                  /* 7    on */
	ROOTPROCBIT,                    /* 8    on */
	SESTATUSBIT,                    /* 9    on */
	SEHOOKBIT       = 10,           /* 10   on */
	SEPOLICYBIT,                    /* 11   off */
	PROCINTERBIT,                   /* 12   off */
	FRAMINTERBIT,                   /* 13   off */
	INAPPINTERBIT,                  /* 14   off */
	NOOPBIT        = 15,            /* 15   on */
	ITIMEOUTBIT,                    /* 16   on */
	EIMABIT,                        /* 17   on */
	SETIDBIT,                       /* 18   on */
	CHECKFAILBIT,                   /* 19   on */
	RODATABIT,                      /* 20   on */
};

/*
 * get_tee_status - provide the tee status for caller
 *
 * return the tee status, 0 for not root, other for rooted
 */
uint32_t get_tee_status(void);

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
/*
 * pause_measurement - this function can pause tee measurement for livepatch
 *
 * return 0 for pause OK, other for pause failed
 */
int pause_measurement(void);
/*
 * resume_measurement - this function can resume tee measurement for livepatch
 *
 * return 0 for resume OK, other for resume failed
 */
int resume_measurement(void);
#else
static inline int pause_measurement(void)
{
	return 0;
}

static inline int resume_measurement(void)
{
	return 0;
}
#endif

#endif
