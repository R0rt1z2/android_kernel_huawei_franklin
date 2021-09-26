/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef SEC_ERROR_H
#define SEC_ERROR_H

/**************************************************************************
 *  COMPILE ASSERT
 **************************************************************************/
#define COMPILE_ASSERT(condition) ((void)sizeof(char[1 - 2*!!!(condition)]))

#define SEC_OK                                  0x0000

/* IMAGE CIPHER */
#define ERR_IMAGE_CIPHER_KEY_ERR                0x1000
#define ERR_IMAGE_CIPHER_IMG_NOT_FOUND          0x1001
#define ERR_IMAGE_CIPHER_READ_FAIL              0x1002
#define ERR_IMAGE_CIPHER_WRONG_OPERATION        0x1003
#define ERR_IMAGE_CIPHER_DEC_TEST_ERROR         0x1004
#define ERR_IMAGE_CIPHER_ENC_TEST_ERROR         0x1005
#define ERR_IMAGE_CIPHER_HEADER_NOT_FOUND       0x1006
#define ERR_IMAGE_CIPHER_DEC_Fail               0x1007

/* SW AES */
#define ERR_AES_KEY_SIZE_ERR                    0x2000
#define ERR_AES_ALLOCATE_CTX_ERR                0x2001
#define ERR_AES_ILEN_SHOULD_EQUAL_OLEN          0x2002
#define ERR_AES_DATA_NOT_MULTIPLE_OF_BLOCK_SIZE 0x2004
#define ERR_AES_KEY_NOT_FOUND                   0x2005

/* ROM INFO */
#define ERR_ROM_INFO_ALLOCATE_BUF_FAIL          0x3000
#define ERR_ROM_INFO_MTD_OPEN_FAIL              0x3001
#define ERR_ROM_INFO_MTD_READ_FAIL              0x3002
#define ERR_ROM_INFO_MTD_NOT_FOUND              0x3003
#define ERR_ROM_INFO_RESET_FAIL                 0x3004
#define ERR_ROM_INFO_MOD_READ_FAIL              0x3005
#define ERR_ROM_INFO_ID_INVALID                 0x3006
#define ERR_INFO_MTD_NUM_INVALID                0x3007
#define ERR_INFO_PART_NOT_FOUND                 0x3008
#define ERR_INFO_OVER_MAX_PART_COUNT            0x3009

/* SW RSA */
#define ERR_RSA_KEY_NOT_FOUND                   0x4000
#define ERR_RSA_WRONG_SIGNATURE_LEN             0x4001
#define ERR_RSA_SIGNATURE_VERIFY_FAIL           0x4002

/* SW HASH */
#define ERR_HASH_WRONG_HASH_LEN                 0x5000

/* HW AES */
#define ERR_KER_CRYPTO_INVALID_MODE             0x6000
#define ERR_HACC_MODE_INVALID                   0x6001
#define ERR_HACC_KEY_INVALID                    0x6002
#define ERR_HACC_DATA_UNALIGNED                 0x6003
#define ERR_HACC_SEED_LEN_ERROR                 0x6004
#define ERR_HACC_ENC_FAIL                       0x6005
#define ERR_HACC_DEC_FAIL                       0x6006
#define ERR_HACC_HW_WRAP_KEY_NOT_INIT           0x6007
#define ERR_HACC_SW_KEY_NOT_INIT                0x6008
#define ERR_SBOOT_HACC_INIT_FAIL                0x6009
#define ERR_SBOOT_HACC_LOCK_FAIL                0x600A
#define ERR_HACC_ENABLE_CLK_FAIL                0x600B
#define ERR_HACC_UNKNOWN_USER                   0x600C
#define ERR_HACC_OPEN_SECURE_CONNECTION_FAIL    0x6010
#define ERR_HACC_REQUEST_SECURE_SERVICE_FAIL    0x6011
#define ERR_HACC_ALLOCATE_BUFFER_FAIL           0x6012
#define ERR_HACC_MCMAP_BUFFER_FAIL              0x6013
#define ERR_HACC_NOTIFY_TO_TRUSTLET_FAIL        0x6014
#define ERR_HACC_NOTIFY_FROM_TRUSTLET_FAIL      0x6015
#define ERR_HACC_CLOSE_SECURE_CONNECTION_FAIL   0x6016
#define ERR_HACC_CFG_INVALID                    0x6017

/* SEC CFG */
#define ERR_SEC_CFG_ALLOCATE_BUF_FAIL           0x7000
#define ERR_SEC_CFG_MTD_OPEN_FAIL               0x7001
#define ERR_SEC_CFG_MTD_READ_FAIL               0x7002
#define ERR_SEC_CFG_MTD_NOT_FOUND               0x7003
#define ERR_SEC_CFG_END_PATTERN_NOT_EXIST       0x7004
#define ERR_SEC_CFG_STATUS_INVALID              0x7005
#define ERR_SEC_CFG_INVALID_ID                  0x7006
#define ERR_SEC_CFG_INVALID_END_PATTERN         0x7007
#define ERR_SEC_CFG_MARK_INCOMPLETE_FAIL        0x7008
#define ERR_SEC_CFG_RESET_FAIL                  0x7009
#define ERR_SEC_CFG_IMG_NOT_FOUND               0x700A
#define ERR_SEC_CFG_IS_FULL                     0x700B
#define ERR_SEC_CFG_VERSION_INVALID             0x700C
#define ERR_SEC_CFG_EXT_REGION_SPACE_OVERFLOW   0x700D
#define ERR_SEC_CFG_MAGIC_INVALID               0x700E
#define ERR_SEC_CFG_EXT_REGION_SELF_COPY_FAIL   0x700F
#define ERR_SEC_CFG_EXT_REGION_OFFSET_INVALID   0x7010
#define ERR_SEC_CFG_EXT_REGION_SIZE_CHANGE      0x7011

/* SEC BOOT UPDATE */
#define ERR_SBOOT_UPDATE_IMG_NOT_FOUND_IN_SECCFG 0x8000
#define ERR_SBOOT_UPDATE_IMG_NOT_FOUND_IN_MTD   0x8001
#define ERR_SBOOT_UPDATE_IMG_OPEN_FAIL          0x8002
#define ERR_SBOOT_UPDATE_IMG_READ_FAIL          0x8003
#define ERR_SBOOT_UPDATE_SEC_CFG_FAIL           0x8004
#define ERR_SBOOT_UPDATE_SEC_RO_FAIL            0x8005
#define ERR_SBOOT_UPDATE_CANNOT_ROLLBACK_VER    0x8006
#define ERR_SBOOT_UPDATE_SEC_VER_NOT_FOUND      0x8007
#define ERR_SBOOT_UPDATE_CUST_NAME_MISMATCH     0x8008
#define ERR_SBOOT_UPDATE_IMG_INVALID            0x8009
#define ERR_SBOOT_UPDATE_CUST_NAME_CANNOT_BE_NULL 0x800A

/* SEC BOOT LIBRARY */
#define SEC_SBOOT_INFO_PART_NOT_FOUND           0x9000
#define SEC_SBOOT_OPEN_SEC_DRV_FAIL             0x9001
#define SEC_SBOOT_SEC_DRV_IOCTL_FAIL            0x9002
#define SEC_SBOOT_INFO_PART_WRITE_OPEN_FAIL     0x9003
#define SEC_SBOOT_INFO_PART_WRITE_FAIL          0x9004
#define SEC_SBOOT_INFO_INIT_FAIL                0x9005
#define SEC_SBOOT_STATUE_QUERY_FAIL             0x9006
#define SEC_SBOOT_NOT_ENABLED                   0x9007
#define SEC_SUSBDL_STATUE_QUERY_FAIL            0x9008
#define SEC_SUSBDL_NOT_ENABLED                  0x9009
#define SEC_SBOOT_MARK_STATUS_FAIL              0x900A
#define SEC_SBOOT_NOT_INIT_YET                  0x900B
#define SEC_SBOOT_NOTIFY_DRIVER_FAIL            0x900C
#define SEC_SBOOT_INVALID_IMG_ATTR              0x900D

/* MTD / USIF  */
#define ERR_MTD_INFO_NOT_FOUND                  0xA000
#define ERR_MTD_PART_COUNT_INVALID              0xA001
#define ERR_MTD_PART_NOT_FOUND                  0xA002
#define ERR_MTD_PART_READ_FAIL                  0xA003
#define ERR_MTD_PART_WRITE_FAIL                 0xA004
#define ERR_MTD_PART_ADJUST_OFFSET_FAIL         0xA005
#define ERR_MTD_PART_READ_MEMINFO_FAIL          0xA006
#define ERR_MTD_PART_INVALID_MEMINFO_FAIL       0xA007
#define ERR_MTD_NOT_SUPPORT_READ_YAFFS2         0xA008
#define ERR_USIF_PART_READ_FAIL                 0xA009
#define ERR_USIF_PART_WRITE_FAIL                0xA00A
#define ERR_USIF_PROC_READ_FAIL                 0xA00B
#define ERR_USIF_PROC_RN_NOT_FOUND              0xA00C
#define ERR_MTD_NOT_SUPPORT_WRITE_YAFFS2        0xA00D
#define ERR_USIF_NOT_SUPPORT_WRITE_YAFFS2       0xA00E
#define ERR_USIF_NOT_SUPPORT_READ_YAFFS2        0xA00F
#define ERR_GPT_PART_NAME_IS_NULL               0xA010
#define ERR_GPT_PART_NAME_NOT_FOUND             0xA011



/* SEC BOOT CHECK */
#define ERR_SBOOT_CHECK_IMG_NOT_FOUND_IN_SECCFG 0xB000
#define ERR_SBOOT_CHECK_IMG_NOT_FOUND_IN_MTD    0xB001
#define ERR_SBOOT_CHECK_IMG_OPEN_FAIL           0xB002
#define ERR_SBOOT_CHECK_IMG_READ_FAIL           0xB003
#define ERR_SBOOT_CHECK_SEC_CFG_FAIL            0xB004
#define ERR_SBOOT_CHECK_IMG_VERIFY_FAIL         0xB005
#define ERR_SBOOT_CHECK_INVALID_IMAGE_OFFSET    0xB006
#define ERR_SBOOT_CHECK_QUERY_ENABLED_FAIL      0xB007
#define ERR_SBOOT_CHECK_PART_INVALID_STATUS     0xB008
#define ERR_SBOOT_CHECK_MD_HDR_MAGIC_ERROR      0xB009
#define ERR_SBOOT_CHECK_MD_NAME_INVLAID         0xB00A
#define ERR_SBOOT_CHECK_MD_VER_CANNOT_ROLLBACK  0xB00B
#define ERR_SBOOT_CHECK_INVALID_IMG_MAGIC_NUM   0xB00C
#define ERR_SBOOT_CHECK_INVALID_MODEM           0xB00D
#define ERR_SBOOT_CHECK_FL_NAME_INVLAID         0xB00E
#define ERR_SBOOT_CHECK_FL_VER_CANNOT_ROLLBACK  0xB00F
#define ERR_SBOOT_CHECK_INVALID_IMG_TYPE        0xB010

/* META */
#define ERR_META_NOT_CORRECT_MODE               0xC000
#define ERR_NVRAM_DATA_NOT_ALIGNED              0xC001
#define ERR_NVRAM_ENC_IOCTL_FAIL                0xC002
#define ERR_NVRAM_DEC_IOCTL_FAIL                0xC002
#define ERR_NVRAM_CIPHER_UT_FAIL                0xC003

/* YAFFS2 COMMON */
#define ERR_YAFFS2_PART_READ_FAIL               0xD000

/* FILE SYSTEM */
#define ERR_FS_ANDROID_SEC_LIST_NOT_SPECIFY     0xE000
#define ERR_FS_SECRO_SEC_LIST_NOT_SPECIFY       0xE001
#define ERR_FS_SEC_LIST_NOT_SPECIFY             0xE002
#define ERR_FS_READ_SEC_LIST_FAIL               0xE003
#define ERR_FS_SIGN_LENGTH_INVALID              0xE004
#define ERR_FS_READ_BUF_IS_NULL                 0xE005
#define ERR_FS_OPEN_SEC_FILE_FAIL               0xE006
#define ERR_FS_READ_SEC_FILE_FAIL               0xE007
#define ERR_FS_READ_BUF_ALLOCATE_FAIL           0xE008
#define ERR_FS_READ_SIZE_FAIL                   0xE009
#define ERR_FS_UNSUPPORT_IMAGE_NAME             0xE00A
#define ERR_FS_SEC_LIST_NOT_SIGNED              0xE00B
#define ERR_FS_READ_MODEM_FAIL                  0xE00C
#define ERR_FS_MD_BIN_NOT_SPECIFY               0xE00D
#define ERR_FS_SECRO_OPEN_FAIL                  0xE00E
#define ERR_FS_SECRO_READ_SIZE_CANNOT_BE_ZERO   0xE00F
#define ERR_FS_SECRO_READ_FAIL                  0xE010
#define ERR_FS_SECRO_AP_INVALID                 0xE011
#define ERR_FS_SECRO_MD_INVALID                 0xE012
#define ERR_FS_SECRO_READ_WRONG_SIZE            0xE013

/* SIGN FORMAT */
#define ERR_SIGN_FORMAT_HASH_SIZE_WRONG         0xE100
#define ERR_SIGN_FORMAT_MAGIC_WRONG             0xE101
#define ERR_SIGN_FORMAT_GENERATE_HASH_FAIL      0xE102
#define ERR_SIGN_FORMAT_EXT_MAGIC_WRONG         0xE103
#define ERR_SIGN_FORMAT_EXT_HDR_MAGIC_WRONG     0xE104
#define ERR_SIGN_FORMAT_EXT_TYPE_NOT_SUPPORT    0xE105
#define ERR_SIGN_FORMAT_EXT_HDR_NOT_FOUND       0xE106
#define ERR_SIGN_FORMAT_CAL_HASH_BY_CHUNK_FAIL  0xE107

/* SECRO IMAGE */
#define ERR_SECROIMG_MTD_NOT_FOUND              0xF000
#define ERR_SECROIMG_HACC_IS_LOCK                0xF001
#define ERR_SECROIMG_HACC_INIT_FAIL              0xF002
#define ERR_SECROIMG_DECRYPT_INVALID            0xF003
#define ERR_SECROIMG_PART_NOT_FOUND             0xF004
#define ERR_SECROIMG_INVALID_IMG_LEN            0xF005
#define ERR_SECROIMG_ALLOCATE_BUF_FAIL          0xF006
#define ERR_SECROIMG_IS_EMPTY                   0xF007
#define ERR_SECROIMG_MD_BUF_NOT_ENOUGH          0xF008
#define ERR_SECROIMG_HACC_AP_DECRYPT_FAIL        0xF009
#define ERR_SECROIMG_HACC_MD_DECRYPT_FAIL        0xF00A
#define ERR_SECROIMG_INVALID_BUF_LEN            0xF00B
#define ERR_SECROIMG_LEN_INCONSISTENT_WITH_PL   0xF00C
#define ERR_SECROIMG_HASH_CHECK_FAIL            0xF00D
#define ERR_SECROIMG_EMPTY_MD_INFO_STR          0xF00E
#define ERR_SECROIMG_MD_INFO_NOT_EXIST          0xF00F
#define ERR_SECROIMG_NEITHER_V3_NOR_V5_FORMAT   0xF010
#define ERR_SECROIMG_V5_HASH_CHECK_FAIL         0xF011
#define ERR_SECROIMG_V3_OFFSET_NOT_INIT         0xF012






#endif				/* SEC_ERROR_H */
