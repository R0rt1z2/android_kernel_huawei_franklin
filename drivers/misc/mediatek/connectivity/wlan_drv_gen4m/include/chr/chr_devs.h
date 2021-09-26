/*
 * chr_devs.h
 *
 * chr device node include
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __CHR_DEVS_H__
#define __CHR_DEVS_H__

#include <linux/skbuff.h>
#include <linux/types.h>

int32_t chr_exception(u_int32_t errno);
int32_t chr_exception_para(u_int32_t chr_errno, uint8_t *chr_ptr, u_int16_t chr_len);
int32_t chr_exception_para_q(u_int32_t chr_errno, u_int16_t chr_flag, uint8_t *chr_ptr, u_int16_t chr_len);
int32_t chr_miscdevs_init(void);
void chr_miscdevs_exit(void);

#define CHR_EXCEPTION(errno) chr_exception(errno)
#define CHR_EXCEPTION_P(chr_errno, chr_ptr, chr_len) chr_exception_para(chr_errno, chr_ptr, chr_len)
#define CHR_EXCEPTION_Q(chr_errno, chr_flag, chr_ptr, chr_len) chr_exception_para_q(chr_errno, chr_flag, chr_ptr, chr_len)

#define CHR_DEV_KMSG_PLAT "chrKmsgPlat"
#define CHR_LOG_ENABLE 1
#define CHR_LOG_DISABLE 0
#define CHR_ERRNO_QUEUE_MAX_LEN 20

#ifdef CHR_DEBUG
#define chr_dbg(s, args...)                                                 \
    do {                                                                    \
        /*lint -e515*/                                                      \
        /*lint -e516*/                                                      \
        printk(KERN_INFO KBUILD_MODNAME ":D]chr %s]" s, __func__, ##args); \
        /*lint +e515*/                                                      \
        /*lint +e516*/                                                      \
    } while (0)
#else
#define chr_dbg(s, args...)
#endif
#define chr_err(s, args...)                                               \
    do {                                                                  \
        /*lint -e515*/                                                    \
        /*lint -e516*/                                                    \
        printk(KERN_ERR KBUILD_MODNAME ":E]chr %s]" s, __func__, ##args); \
        /*lint +e515*/                                                    \
        /*lint +e516*/                                                    \
    } while (0)
#define chr_warning(s, args...)                                               \
    do {                                                                      \
        /*lint -e515*/                                                        \
        /*lint -e516*/                                                        \
        printk(KERN_WARNING KBUILD_MODNAME ":W]chr %s]" s, __func__, ##args); \
        /*lint +e515*/                                                        \
        /*lint +e516*/                                                        \
    } while (0)
#define chr_info(s, args...)                                                \
    do {                                                                    \
        /*lint -e515*/                                                      \
        /*lint -e516*/                                                      \
        printk(KERN_INFO KBUILD_MODNAME ":I]chr %s]" s, __func__, ##args); \
        /*lint +e515*/                                                      \
        /*lint +e516*/                                                      \
    } while (0)

#define CHR_MAGIC          'C'
#define CHR_MAX_NR         2
#define chr_ERRNO_WRITE_NR 1
#define CHR_ERRNO_WRITE    _IOW(CHR_MAGIC, 1, int32_t)
#define CHR_ERRNO_ASK      _IOW(CHR_MAGIC, 2, int32_t)

enum CHR_ID_ENUM {
    CHR_WIFI = 909,
    CHR_BT = 913,
    CHR_GNSS = 910,
    CHR_ENUM
};

enum return_type {
    CHR_SUCC = 0,
    CHR_EFAIL,
};

 typedef enum {
     CHR_DEVICE = 0x0,
     CHR_HOST = 0x1,
 } CHR_REPORT_FLAGS_ENUM;

typedef struct {
    wait_queue_head_t errno_wait;
    struct sk_buff_head errno_queue;
    struct semaphore errno_sem;
} chr_event;

typedef struct {
    u_int32_t errno;
    u_int16_t errlen;
    u_int16_t flag : 1;
    u_int16_t resv : 15;
} chr_dev_exception_stru_para;

typedef struct {
    u_int32_t chr_errno;
    u_int16_t chr_len;
    uint8_t *chr_ptr;
} chr_host_exception_stru;

#define MAC_ADDRESS_LEN  6
typedef struct {
    uint8_t srcBssid[MAC_ADDRESS_LEN];
    uint8_t srcFreq;
    int8_t srcRssi;
    uint8_t dstBssid[MAC_ADDRESS_LEN];
    uint8_t dstFreq;
    int8_t dstRssi;
    uint8_t roamMode;
    uint8_t roamStage;
    uint8_t roamResult;
    uint8_t triggerType;
    uint32_t roamTime;
} wifi_roam_fail; // 909009102

typedef struct {
    uint32_t airTxop;
    int16_t temp;
    uint8_t isPingPong;
    uint8_t isReducePwr;
    uint8_t baCnt;
    uint8_t resv[3];
} temperature_protect_info; // 909003005

typedef struct {
    uint8_t ucTxNss;
    uint8_t ucRxNss;
}  MimoToSiso; // 909002038 related event
#endif
