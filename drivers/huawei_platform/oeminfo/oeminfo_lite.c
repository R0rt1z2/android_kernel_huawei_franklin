/*
 * oeminfo.c
 *
 * direct character-device access to oeminfo device
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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "oeminfo/oeminfo_def.h"
#ifdef OEMINFO_WP_FEATURE
	#include "oeminfo_v8.h"
	#include "oeminfo_v6_to_v8.h"
#elif OEMINFO_LITE_FEATURE
	#include "oeminfo_lite.h"
	#include "oeminfo_v6_to_lite.h"
#endif

static struct semaphore oeminfo_sem;
static struct class *oeminfo_class;
static unsigned int dev_major_num;

enum oeminfo_region_valid_t {
	OEMINFO_USING_REGION_VALID,
	OEMINFO_BACKUP_REGION_VALID,
	OEMINFO_REGION_ERROR,
	OEMINFO_REGION_MAX,
};

enum oeminfo_operation_t {
	OEMINFO_GET_VALID_REGION_OFFSET,
	OEMINFO_GET_INVALID_REGION_OFFSET,
	OEMINFO_OPERATION_MAX,
};

/* static func, all parameters have been checked or assured already */
static inline unsigned int oeminfo_index_adapt(int *index)
{
	if (*index <= 0)
		return 0;
#ifdef OEMINFO_WP_FEATURE
	if (*index >= ARRAY_SIZE(g_oeminfoV8Table))
		return 0;
	*index = g_oeminfoV8Table[*index].oeminfoV8IdIndex;
#elif OEMINFO_LITE_FEATURE
	if (*index >= ARRAY_SIZE(g_oeminfoLiteTable))
		return 0;
	*index = g_oeminfoLiteTable[*index].oeminfoLiteIdIndex;
#endif
	return 1;
}

static unsigned int judge_index_valid(int index)
{
	int subpart;
	int subindex;

	if (index <= OEMINFO_4K_MIN_INDEX) {
		pr_err("%s: index = %d is invalid\n", __func__, index);
		return 0;
	}
	subpart = index / OEMINFO_SUBPART_MAX_INDEX;
	subindex = index % OEMINFO_SUBPART_MAX_INDEX;
	if (subpart >= SUBPART_NONEWP_AREA &&
		subpart <= SUBPART_PWRWP_AREA) {
		if ((subindex > OEMINFO_4K_MIN_INDEX &&
			subindex <= OEMINFO_4K_MAX_INDEX) ||
			(subindex > OEMINFO_8K_MIN_INDEX &&
			subindex <= OEMINFO_8K_MAX_INDEX))
			return 1;
	}
	pr_err("%s: index = %d is invalid\n", __func__, index);
	return 0;
}

static unsigned int get_oeminfo_size(int index)
{
	unsigned int size = 0;
	int subpart;
	int subindex;

	if (index <= OEMINFO_4K_MIN_INDEX) {
		pr_err("%s: index = %d is invalid\n", __func__, index);
		return size;
	}
	subpart = index / OEMINFO_SUBPART_MAX_INDEX;
	subindex = index % OEMINFO_SUBPART_MAX_INDEX;

	if (subpart >= SUBPART_NONEWP_AREA &&
		subpart <= SUBPART_PWRWP_AREA) {
		if (subindex > OEMINFO_4K_MIN_INDEX &&
			subindex <= OEMINFO_4K_MAX_INDEX)
			size  = OEMINFO_4K_AREA_SIZE;
		else if (subindex > OEMINFO_8K_MIN_INDEX &&
			subindex <= OEMINFO_8K_MAX_INDEX)
			size = OEMINFO_8K_AREA_SIZE;
		else
			pr_err("%s: can't find index: %d\n", __func__, index);
	} else {
		pr_err("%s: index = %d is invalid\n", __func__, index);
	}
	return size;
}

/* 1 means not exceed boundary, 0 means exceed boundary or index invalid */
static int is_not_exceed_boundary(int index, unsigned int size)
{
	/* total size of oeminfo */
	unsigned int t_size;
	/* remain size of data total_size - OEMINFO_PAGE_SIZE */
	unsigned int r_size;

	t_size = get_oeminfo_size(index);
	if (t_size == 0) {
		pr_err("%s: index = %d is invalid\n", __func__, index);
		return 0;
	}
	r_size = t_size - OEMINFO_PAGE_SIZE;
	if (size <= r_size)
		return 1;
	return 0;
}

static unsigned int get_region_offset(int subpart)
{
	switch (subpart) {
	case SUBPART_NONEWP_AREA:
		return OEMINFO_NONEWP_REGION_OFFSET;
	case SUBPART_ROOTWP_AREA:
		return OEMINFO_ROOTWP_REGION_OFFSET;
	case SUBPART_PWRWP_AREA:
		return OEMINFO_PWRWP_REGION_OFFSET;
	default:
		return OEMINFO_OFFSET_ERR;
	}
}

static unsigned int get_oeminfo_base_offset(int index)
{
	int subpart;
	int subindex;
	unsigned int offset;
	unsigned int partoffset;

	if (index <= OEMINFO_4K_MIN_INDEX) {
		pr_err("%s: index = %d is invalid\n", __func__, index);
		return OEMINFO_OFFSET_ERR;
	}
	subpart = index / OEMINFO_SUBPART_MAX_INDEX;
	partoffset = get_region_offset(subpart);
	if (partoffset == OEMINFO_OFFSET_ERR) {
		pr_err("%s: index = %d is invalid\n", __func__, index);
		return OEMINFO_OFFSET_ERR;
	}
	subindex = index % OEMINFO_SUBPART_MAX_INDEX;
	if (subindex > OEMINFO_4K_MIN_INDEX &&
		subindex <= OEMINFO_4K_MAX_INDEX) {
		offset  = partoffset + OEMINFO_4K_BASE_OFFSET +
			OEMINFO_4K_AREA_SIZE * (subindex - 1);
		return offset;
	}
	if (subindex > OEMINFO_8K_MIN_INDEX &&
		subindex <= OEMINFO_8K_MAX_INDEX) {
		offset = partoffset + OEMINFO_8K_BASE_OFFSET +
			OEMINFO_8K_AREA_SIZE *
			(subindex - OEMINFO_8K_MIN_INDEX - 1);
		return offset;
	}
	pr_err("%s: invalid index = %d\n", __func__, index);
	return OEMINFO_OFFSET_ERR;
}

/* static func, all parameters have been checked or assured already */
static int oeminfo_write(const char *ptn_name, unsigned long to,
	unsigned int len, const unsigned char *buf)
{
	int ret = 0;
	int fd;
	mm_segment_t oldfs = get_fs();

	set_fs(get_ds());
	fd = sys_open(ptn_name, O_RDWR, 0);
	if (fd < 0) {
		pr_err("%s: open oeminfo block device failed, and fd = %x!\n",
			__func__, fd);
		ret = -ENODEV;
		goto open_err;
	}
	ret = sys_lseek((unsigned int)fd, to, SEEK_SET);
	if (ret < 0) {
		pr_err("%s: seek error, read flash from = %llu, len = %u\n",
			__func__, to, len);
		ret = -EIO;
		goto out_close;
	}
	ret = sys_write((unsigned int)fd, (char *)buf, len);
	if (ret < 0) {
		pr_err("%s: write error, read flash from = %llu, len = %u\n",
			__func__, to, len);
		ret = -EIO;
		goto out_close;
	}
	ret = sys_fsync((unsigned int)fd);
	if (ret < 0) {
		pr_err("%s: sync error, read flash from = %llu, len = %u\n",
			__func__, to, len);
		ret = -EIO;
		goto out_close;
	} else {
		ret = 0;
	}
out_close:
	sys_close((unsigned int)fd);
open_err:
	set_fs(oldfs);
	return ret;
}

/* static func, all parameters have been checked or assured already */
static int oeminfo_read(const char *ptn_name, unsigned long from,
	unsigned int len, unsigned char *buf)
{
	int ret = 0;
	int fd;
	mm_segment_t oldfs = get_fs();

	set_fs(get_ds());
	fd = sys_open(ptn_name, O_RDONLY, 0);
	if (fd < 0) {
		pr_err("%s: open oeminfo block device failed, and fd = %d\n",
			__func__, fd);
		ret = -ENODEV;
		goto open_err;
	}
	ret = sys_lseek((unsigned int)fd, from, SEEK_SET);
	if (ret < 0) {
		pr_err("%s: seek error, read flash from = %llu, len = %u\n",
			__func__, from, len);
		ret = -EIO;
		goto out_close;
	}
	ret = sys_read((unsigned int)fd, (char *)buf, len);
	if (ret < 0) {
		pr_err("%s: read error, read flash from = %llu, len = %u\n",
			__func__, from, len);
		ret = -EIO;
		goto out_close;
	} else {
		ret = 0;
	}
out_close:
	sys_close((unsigned int)fd);
open_err:
	set_fs(oldfs);
	return ret;
}

static int judge_header_magic_and_age_valid(
	const struct oeminfo_hdr_page_t *oeminfo_header_ptr, int index)
{
	if (oeminfo_header_ptr == NULL || index <= 0) {
		pr_err("%s: bad parameters, index: %d\n", __func__, index);
		return 0;
	}
	if (oeminfo_header_ptr->oeminfo_age >= OEMINFO_MAX_AGE ||
		(oeminfo_header_ptr->magic_number1 != OEMINFO_MAGIC1) ||
		(oeminfo_header_ptr->magic_number2 != OEMINFO_MAGIC2) ||
		(oeminfo_header_ptr->total_byte == 0) ||
		(oeminfo_header_ptr->info_type != (unsigned int)index)) {
		return 0;
	}
	return 1;
}

/* static func, all parameters have been checked or assured already */
static int get_oeminfo_age(unsigned int offset,
	unsigned int *oeminfo_age, int index)
{
	struct oeminfo_hdr_page_t *oeminfo_hdr = NULL;
	unsigned char oeminfo_hdr_page_buff[OEMINFO_PAGE_SIZE] = {0};

	if (oeminfo_age == NULL) {
		pr_err("%s: param is bad\n", __func__);
		return 0;
	}

	if (oeminfo_read(PART_OEMINFO, offset,
		OEMINFO_PAGE_SIZE, (unsigned char *)oeminfo_hdr_page_buff)) {
		pr_err("%s: can't read oeminfo item header\n", __func__);
		return 0;
	}
	oeminfo_hdr = (struct oeminfo_hdr_page_t *)oeminfo_hdr_page_buff;
	if (judge_header_magic_and_age_valid(oeminfo_hdr, index) == 0)
		*oeminfo_age = 0;
	else
		*oeminfo_age = oeminfo_hdr->oeminfo_age;
	return 1;
}

/* static func, all parameters have been checked or assured already */
static enum oeminfo_region_valid_t get_valid_oeminfo_region(int index)
{
	unsigned int using_region_age = 0;
	unsigned int backup_region_age = 0;
	unsigned int backup_region_offset = 0;

	enum oeminfo_region_valid_t valid_region;
	unsigned int using_region_offset = get_oeminfo_base_offset(index);

	if (using_region_offset != OEMINFO_OFFSET_ERR) {
		backup_region_offset = using_region_offset +
			OEMINFO_BACKUP_REGION_OFFSET;
	} else {
		pr_err("%s: get oeminfo base offset fail\n", __func__);
		return OEMINFO_REGION_ERROR;
	}
	if (get_oeminfo_age(using_region_offset,
		&using_region_age, index) == 1 &&
		get_oeminfo_age(backup_region_offset,
		&backup_region_age, index) == 1) {
		if (using_region_age > backup_region_age + 1 ||
			backup_region_age > using_region_age + 1)
			pr_err("%s: difference is more than one\n", __func__);
	} else {
		pr_err("%s: get oeminfo region age fail\n", __func__);
		return OEMINFO_REGION_ERROR;
	}

	if (using_region_age == 0 || using_region_age >= OEMINFO_MAX_AGE) {
		valid_region = OEMINFO_BACKUP_REGION_VALID;
	} else if (backup_region_age == 0 ||
		backup_region_age >= OEMINFO_MAX_AGE) {
		valid_region = OEMINFO_USING_REGION_VALID;
	} else if (using_region_age > backup_region_age) {
		valid_region = OEMINFO_USING_REGION_VALID;
	} else if (using_region_age < backup_region_age) {
		valid_region = OEMINFO_BACKUP_REGION_VALID;
	} else {
		pr_err("%s: two ages is equal to each other\n", __func__);
		valid_region = OEMINFO_USING_REGION_VALID;
	}
	pr_info("%s: vaild region = %d", __func__, valid_region);

	return valid_region;
}

/* static func, all parameters have been checked or assured already */
static unsigned int get_oeminfo_offset(int index,
	enum oeminfo_operation_t opera)
{
	enum oeminfo_region_valid_t valid_region;
	unsigned int offset = get_oeminfo_base_offset(index);

	if (offset == OEMINFO_OFFSET_ERR)
		return offset;
	valid_region = get_valid_oeminfo_region(index);

	if (valid_region == OEMINFO_REGION_ERROR) {
		pr_err("%s: get valid oeminfo region fail\n", __func__);
		return OEMINFO_OFFSET_ERR;
	}

	switch (opera) {
	case OEMINFO_GET_INVALID_REGION_OFFSET:
		if (valid_region == OEMINFO_USING_REGION_VALID)
			return (offset + OEMINFO_BACKUP_REGION_OFFSET);
		else
			return offset;
	case OEMINFO_GET_VALID_REGION_OFFSET:
		if (valid_region == OEMINFO_USING_REGION_VALID)
			return offset;
		else
			return (offset + OEMINFO_BACKUP_REGION_OFFSET);
	default:
		pr_err("%s: unkown operation\n", __func__);
		break;
	}
	return OEMINFO_OFFSET_ERR;
}

/* static func, all parameters have been checked or assured already */
static int get_oeminfo(int index, int len, char *data)
{
	unsigned int offset;
	unsigned char oeminfo_hdr_page_buff[OEMINFO_PAGE_SIZE] = {0};
	struct oeminfo_hdr_page_t *oeminfo_hdr = NULL;

	if (index < 0 || len <= 0 || data == NULL) {
		pr_err("%s: bad para, index = %d, len = %d\n",
			__func__, index, len);
		return -1;
	}
	if (!oeminfo_index_adapt(&index)) {
		pr_err("%s: invalid index: %d\n", __func__, index);
		return -1;
	}
	offset = get_oeminfo_offset(index,
		OEMINFO_GET_VALID_REGION_OFFSET);
	if (offset == OEMINFO_OFFSET_ERR) {
		pr_err("%s: get_oeminfo_offset fail, index = %d\n",
			__func__, index);
		return -1;
	}
	if (oeminfo_read(PART_OEMINFO, offset,
		OEMINFO_PAGE_SIZE, (unsigned char *)oeminfo_hdr_page_buff)) {
		pr_err("%s: can't read oeminfo item header, index = %d\n",
			__func__, index);
		return -1;
	}
	oeminfo_hdr = (struct oeminfo_hdr_page_t *)oeminfo_hdr_page_buff;
	pr_err("%s: oeminfo read id = %d\n", __func__, index);
	/* check the hdr -- magic number */
	if ((oeminfo_hdr->magic_number1 == OEMINFO_MAGIC1) &&
			(oeminfo_hdr->magic_number2 == OEMINFO_MAGIC2) &&
			(oeminfo_hdr->info_type == index)) {
		if (!is_not_exceed_boundary(index, oeminfo_hdr->total_byte)) {
			pr_err("%s: total_byte larger max_byte\n", __func__);
			return -1;
		}

		if (oeminfo_hdr->total_byte < len)
			len = oeminfo_hdr->total_byte;
		return oeminfo_read(PART_OEMINFO, offset + OEMINFO_PAGE_SIZE,
			len, (unsigned char *)data);
	}
	pr_err("%s: index: %d oeminfo data not find\n", __func__, index);
	return -1;
}

/* static func, all parameters have been checked or assured already */
static int get_oeminfo_age_from_valid_region(int index,
	unsigned int *oeminfo_age)
{
	int ret = 0;
	unsigned int age = 0;
	unsigned int offset;

	if (oeminfo_age == NULL || !judge_index_valid(index)) {
		pr_err("%s: param is bad\n", __func__);
		return ret;
	}
	offset = get_oeminfo_offset(index, OEMINFO_GET_VALID_REGION_OFFSET);
	if (offset == OEMINFO_OFFSET_ERR) {
		pr_err("%s: get offset fail\n", __func__);
		return ret;
	}
	ret = get_oeminfo_age(offset, &age, index);
	if (ret == 1) {
		*oeminfo_age = age;
		return 1;
	}
	return 0;
}
static int write_oeminfo_hdr(int index, int len, unsigned int offset,
	struct oeminfo_hdr_page_t *oeminfo_hdr)
{
	unsigned int oeminfo_age = 0;
	int ret;

	ret = get_oeminfo_age_from_valid_region(index, &oeminfo_age);
	if (ret == 0) {
		pr_err("%s: get valid region age fail, index = %d\n",
			__func__, index);
		return -1;
	}
	oeminfo_hdr->magic_number1  = OEMINFO_MAGIC1;
	oeminfo_hdr->magic_number2  = OEMINFO_MAGIC2;
	oeminfo_hdr->version        = OEMINFO_VERSION;
	oeminfo_hdr->info_type      = index;
	oeminfo_hdr->total_byte     = len;
	oeminfo_hdr->oeminfo_age = (oeminfo_age + 1) % OEMINFO_MAX_AGE;
	if (oeminfo_write(PART_OEMINFO, offset, OEMINFO_PAGE_SIZE,
		(unsigned char *)oeminfo_hdr)) {
		pr_err("%s: oeminfo write header fail, index = %d\n",
			__func__, index);
		return -1;
	}
	return 0;
}

static int set_oeminfo(int index, int len, const char *data)
{
	unsigned int offset;
	int ret;
	unsigned char oeminfo_hdr_page_buff[OEMINFO_PAGE_SIZE] = {0};

	if (data == NULL || len <= 0 || index < 0) {
		pr_err("%s: bad parametes, len = %d, index = %d\n",
			__func__, len, index);
		return -1;
	}
	if (!oeminfo_index_adapt(&index)) {
		pr_err("%s: invalid index: %d\n", __func__, index);
		return -1;
	}
	if (!is_not_exceed_boundary(index, len)) {
		pr_err("%s: oeminfo data is too large\n", __func__);
		return -1;
	}
	offset = get_oeminfo_offset(index, OEMINFO_GET_INVALID_REGION_OFFSET);
	if (offset == OEMINFO_OFFSET_ERR) {
		pr_err("%s: fail to get_oeminfo_offset, index = %d\n",
			__func__, index);
		return -1;
	}
	if (oeminfo_write(PART_OEMINFO, offset + OEMINFO_PAGE_SIZE, len,
		(unsigned char *)data)) {
		pr_err("%s: oeminfo write %d fail\n", __func__, index);
		return -1;
	}
	ret = write_oeminfo_hdr(index, len, offset,
		(struct oeminfo_hdr_page_t *)oeminfo_hdr_page_buff);
	if (ret != 0) {
		pr_err("%s: write oeminfo hdr failed, index = %d\n",
			__func__, index);
		return -1;
	}
	return 0;
}

static int oeminfo_direct_access_for_action(
		struct oeminfo_info_user *user_info)
{
	if (!user_info) {
		pr_err("%s: input parameter is NULL.\n", __func__);
		return -1;
	}

	if (user_info->oeminfo_operation == OEMINFO_READ) {
		return get_oeminfo(user_info->oeminfo_id, user_info->valid_size,
			user_info->oeminfo_data);
	}
	if (user_info->oeminfo_operation == OEMINFO_WRITE) {
		return set_oeminfo(user_info->oeminfo_id, user_info->valid_size,
			user_info->oeminfo_data);
	}
	pr_err("%s: oeminfo_operation parameter is invaild\n", __func__);
	return -1;
}

int oeminfo_direct_access(struct oeminfo_info_user *user_info)
{
	int ret;
	/* ensure only one process can visit oeminfo at the same time in
	 * kernel
	 */
	if (down_interruptible(&oeminfo_sem))
		return -EBUSY;
	ret = oeminfo_direct_access_for_action(user_info);
	if (ret)
		pr_err("%s: access for oeminfo according action failed\n",
			__func__);
	/* release the semaphore */
	up(&oeminfo_sem);
	return ret;
}
EXPORT_SYMBOL(oeminfo_direct_access);

static int do_cmd_for_oeminfo(struct oeminfo_info_user *info, u_int cmd,
	void __user *argp)
{
	int ret = -EFAULT;

	switch (cmd) {
	case OEMINFOACCESSDATA:
		if (copy_from_user(info, argp,
			sizeof(struct oeminfo_info_user))) {
			pr_err("%s: copy_from_user failed\n", __func__);
			break;
		}
		ret = oeminfo_direct_access_for_action(info);
		if (ret) {
			pr_err("%s: oeminfo access failed\n", __func__);
			break;
		}
		if (info->oeminfo_operation == OEMINFO_READ) {
			if (copy_to_user(argp, info,
				sizeof(struct oeminfo_info_user))) {
				pr_err("%s: copy_to_user failed\n", __func__);
				break;
			}
		}
		break;
	default:
		pr_err("%s: Unknow command\n", __func__);
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static long oeminfo_ioctl(struct file *file, u_int cmd, u_long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	u_int size;
	struct oeminfo_info_user *info = NULL;

	info = (struct oeminfo_info_user *)kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	if (!argp) {
		pr_err("%s: The input arg is null\n", __func__);
		kfree(info);
		return -ENOMEM;
	}
	if (down_interruptible(&oeminfo_sem)) {
		pr_err("%s: down_interruptible error\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	size = ((cmd & IOCSIZE_MASK) >> IOCSIZE_SHIFT);
	if (cmd & IOC_IN) {
		if (!access_ok(VERIFY_READ, arg, size)) {
			pr_err("%s: access_in error\n", __func__);
			ret = -EFAULT;
			goto out;
		}
	}
	if (cmd & IOC_OUT) {
		if (!access_ok(VERIFY_WRITE, arg, size)) {
			pr_err("%s: access_out error\n", __func__);
			ret = -EFAULT;
			goto out;
		}
	}
	ret = do_cmd_for_oeminfo(info, cmd, argp);
out:
	up(&oeminfo_sem);
	kfree(info);
	return (long)ret;
}

#ifdef CONFIG_COMPAT
static long oeminfo_compat_ioctl(struct file *file, u_int cmd, u_long arg)
{
	return oeminfo_ioctl(file, cmd,
		(unsigned long)compat_ptr((unsigned int)arg));
}
#endif

static const struct file_operations oeminfo_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = oeminfo_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = oeminfo_compat_ioctl,
#endif
};

static int __init init_oeminfo(void)
{
	int ret = 0;
	struct device *nve_dev = NULL;
	/* semaphore initial */
	sema_init(&oeminfo_sem, 1);
	/* register a device in kernel, return the number of major device */
	dev_major_num = register_chrdev(0, "oeminfo", &oeminfo_fops);
	if (dev_major_num < 0) {
		pr_err("%s: allocate major number failed\n", __func__);
		ret = -1;
	}
	/* register a class, make sure that mdev can create device node in
	 * "/dev"
	 */
	oeminfo_class = class_create(THIS_MODULE, "oeminfo");
	if (IS_ERR(oeminfo_class)) {
		pr_err("%s: Error creating oeminfo class\n", __func__);
		unregister_chrdev(
		(unsigned int)dev_major_num, "oeminfo");
		ret = -1;
	}
	/* create a device node for application */
	nve_dev = device_create(oeminfo_class, NULL, MKDEV(dev_major_num, 0),
		NULL, "oeminfo0");
	if (IS_ERR(nve_dev)) {
		pr_err("%s: failed to create oeminfo device\n",
		__func__);
		return PTR_ERR(nve_dev);
	}
	return ret;
}

static void __exit cleanup_oeminfo(void)
{
	device_destroy(oeminfo_class, MKDEV(dev_major_num, 0));
	class_destroy(oeminfo_class);
	unregister_chrdev((unsigned int)dev_major_num, "oeminfo");
}

module_init(init_oeminfo);
module_exit(cleanup_oeminfo);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("direct character-device access to oeminfo device");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");