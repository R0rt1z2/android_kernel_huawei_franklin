/*
 * oeminfo.h
 *
 * direct character-device access to oeminfo device header file.
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef OEMINFO_H
#define OEMINFO_H

#define OEMINFO_VERSION 0x09
#define OEMINFO_PAGE_SIZE 512
#define OEMINFO_PAGES_IN_BLOCK 32
#define OEMINFO_BLOCK_SIZE (OEMINFO_PAGE_SIZE * OEMINFO_PAGES_IN_BLOCK)
#define PART_OEMINFO "/dev/block/by-name/oeminfo"
#define OEMINFO_HEAD_NOT_FOUND 0
#define OEMINFO_FOUND_ERROR (-1)
#define OEMINFO_READ_HEAD_ERROR (-2)
#define OEMINFO_READ_SIZE_ERROR (-3)
#define OEMINFO_READ_DATA_ERROR (-4)
#define OEMINFO_NOT_EMMC_ERROR  (-5)
#define OEMINFO_4K_MIN_INDEX 0 /* 4K area base index id 0 */
#define OEMINFO_4K_MAX_INDEX 328 /* 4K base  1-328,total 328 nums */
#define OEMINFO_4K_NUM 328 /* 4K total 328 nums */
#define OEMINFO_8K_MIN_INDEX 500 /* 8K area base index id 500 */
#define OEMINFO_8K_MAX_INDEX 600 /* 8K base  501-600,total 100 nums */
#define OEMINFO_8K_NUM 100  /* 8K total 100 nums */

/*
 * each oeminfo subpart can only contain
 * no more than 750 nums(1~750, 751~1500, 1501~2250)
 */
#define OEMINFO_SUBPART_MAX_INDEX 750

#define OEMINFO_4K_AREA_SIZE (4 * 1024) /* 4k oeminfo area */
#define OEMINFO_8K_AREA_SIZE (8 * 1024) /* 8k oeminfo area */
#define OEMINFO_4K_BASE_OFFSET 0
#define OEMINFO_8K_BASE_OFFSET (OEMINFO_4K_BASE_OFFSET + \
	(OEMINFO_4K_NUM * OEMINFO_4K_AREA_SIZE))

#define SUBPART_NONEWP_AREA 0
#define SUBPART_ROOTWP_AREA 1
#define SUBPART_PWRWP_AREA 2
 /* 0 ~ 8M area is none wp */
#define OEMINFO_NONEWP_REGION_OFFSET (0 * 1024 * 1024)
 /* 8 ~ 16M area is root risk wp */
#define OEMINFO_ROOTWP_REGION_OFFSET (8 * 1024 * 1024)
 /* 16 ~ 24M area is power up wp */
#define OEMINFO_PWRWP_REGION_OFFSET (16 * 1024 * 1024)
#define OEMINFO_BACKUP_REGION_OFFSET (4 * 1024 * 1024)

/*
 * the first 512(none reused header size) or 64(reused header size) byte
 * is used for the header of one id,
 * when want get the header, the offset is 0, and when
 * want read the valid data, the offset will greater than 512
 * so the value from 1 to 64 can be used to mark error
 */
#define OEMINFO_OFFSET_ERR 1
#define OMEINFO_PARTITION_LEN (32 * 1024 * 1024)
#define OEMINFOACCESSDATA _IOWR('N', 26, struct oeminfo_info_user)

#define OEMINFO_MAGIC1 0x5F4D454F
#define OEMINFO_MAGIC2 0x4F464E49
#define OEMINFO_MAX_AGE (1000000000L)

struct oeminfo_hdr_page_t {
	unsigned int magic_number1;
	unsigned int magic_number2;
	unsigned int version;
	unsigned int info_type; /* of oeminfo_info_type_enum_type */
	unsigned int total_blck; /* how many blocks to save this type */
	unsigned int total_byte; /* how many byte to save this type */
	unsigned int oeminfo_age; /* mark the valid region */
};

#define LENTH_FOR_FUTURE_EXPAND 36

struct oeminfo_reused_hdr_page {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int ver;
	unsigned int id;
	unsigned int subid;
	unsigned int len; /* how many byte to save int this subid. */
	unsigned int age; /* mark the valid region */
	char reserved[LENTH_FOR_FUTURE_EXPAND]; /* reserved for future expand */
};

#endif
