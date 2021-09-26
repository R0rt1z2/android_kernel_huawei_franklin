/*
 * hw_f_mass_storage.c
 *
 * mass_storage support for mac system and configfs driver
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

/* usbsdms_read_toc_data1 rsp packet */
static u8 g_usbsdms_read_toc_data1[] = {
	0x00, 0x0A, 0x01, 0x01,
	0x00, 0x14, 0x01, 0x00,
	0x00, 0x00, 0x02, 0x00
};

/* usbsdms_read_toc_data1_format0000 rsp packet */
static u8 g_usbsdms_read_toc_data1_format0000[] = {
	0x00, 0x12, 0x01, 0x01,
	0x00, 0x14, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* the last four bytes:32MB */
	0x00, 0x14, 0xAA, 0x00,
	0x00, 0x00, 0xFF, 0xFF
};

/* usbsdms_read_toc_data1_format0001 rsp packet */
static u8 g_usbsdms_read_toc_data1_format0001[] = {
	0x00, 0x0A, 0x01, 0x01,
	0x00, 0x14, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00
};

/* usbsdms_read_toc_data2 rsp packet */
static u8 g_usbsdms_read_toc_data2[] = {
	0x00, 0x2e, 0x01, 0x01,
	0x01, 0x14, 0x00, 0xa0,
	0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x01,
	0x14, 0x00, 0xa1, 0x00,
	0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x01, 0x14,
	0x00, 0xa2, 0x00, 0x00,
	0x00, 0x00, 0x06, 0x00,
	0x3c, 0x01, 0x14, 0x00,
	/* CDROM size from this byte */
	0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x02, 0x00
};

/* usbsdms_read_toc_data3 rsp packet */
static u8 g_usbsdms_read_toc_data3[] = {
	0x00, 0x12, 0x01, 0x01,
	0x00, 0x14, 0x01, 0x00,
	0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

#define FSG_MAX_LUNS_HUAWEI    2
#define BUF_MAX                20
#define TOC_DATA_LEN           18
#define FIRST_TRK_NUM          1
#define LAST_TRK_NUM           1
#define TOC_DATA_INDEX         1
#define FIRST_TRK_INDEX        2
#define LAST_TRK_INDEX         3
#define DATA_CPY_INDEX         5
#define TRK_NUM_INDEX          6
#define CDROM_IDEX             8
#define DATA_TRACK_INDEX       13
#define OUT_TRACK_INDEX        14
#define CDROM_SEC_IDEX         16
#define DATA_TRACK             0x16
#define LEAD_OUT_TRK_NUM       0xAA
#define MSF_MASK               0x02

#ifdef CONFIG_HUAWEI_USB
static int create_mass_storage_device(struct usb_function_instance *fi);
static int do_read_toc(struct fsg_common *common, struct fsg_buffhd *bh);
static int read_toc_for_meta(struct fsg_common *common, struct fsg_buffhd *bh);
#else
static inline int create_mass_storage_device(struct usb_function_instance *fi)
{
	return 0;
}

static inline int do_read_toc(struct fsg_common *common, struct fsg_buffhd *bh)
{
	return 0;
}

static inline int read_toc_for_meta(struct fsg_common *common,
	struct fsg_buffhd *bh)
{
	return 0;
}
#endif /* CONFIG_HUAWEI_USB */

/* READ_TOC command structure */
struct usbsdms_read_toc_cmd_type {
	u8 op_code;
	/*
	 * bit1 is MSF, 0: address format is LBA form
	 * 1: address format is MSF form
	 */
	u8 msf;
	/*
	 * bit3~bit0,   MSF Field   Track/Session Number
	 * 0000b:       Valid       Valid as a Track Number
	 * 0001b:       Valid       Ignored by Drive
	 * 0010b:       Ignored     Valid as a Session Number
	 * 0011b~0101b: Ignored     Ignored by Drive
	 * 0110b~1111b: Reserved
	 */
	u8 format;
	u8 reserved1;
	u8 reserved2;
	u8 reserved3;
	u8 session_num; /* a specific session or a track */
	u8 allocation_length_msb;
	u8 allocation_length_lsb;
	u8 control;
};

#ifdef CONFIG_HUAWEI_USB
static int read_toc_for_meta(struct fsg_common *common, struct fsg_buffhd *bh)
{
	struct fsg_lun *curlun = common->curlun;
	int msf = (common->cmnd[FIRST_TRK_NUM] & MSF_MASK);
	int start_track = common->cmnd[TRK_NUM_INDEX];
	u8 *buf = NULL;

	if (!curlun)
		return -EINVAL;
	buf = (u8 *)bh->buf;
	if (!buf)
		return -EINVAL;

	if (((common->cmnd[FIRST_TRK_NUM] & (~MSF_MASK)) != 0) ||
		(start_track > FIRST_TRK_NUM)) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	memset(buf, 0, BUF_MAX);
	buf[TOC_DATA_INDEX] = TOC_DATA_LEN;
	buf[FIRST_TRK_INDEX] = FIRST_TRK_NUM;
	buf[LAST_TRK_INDEX] = LAST_TRK_NUM;
	buf[DATA_CPY_INDEX] = DATA_TRACK;
	buf[TRK_NUM_INDEX] = FIRST_TRK_NUM;
	store_cdrom_address(&buf[CDROM_IDEX], msf, 0);

	buf[DATA_TRACK_INDEX] = DATA_TRACK;
	buf[OUT_TRACK_INDEX] = LEAD_OUT_TRK_NUM;
	store_cdrom_address(&buf[CDROM_SEC_IDEX], msf, curlun->num_sectors);
	return BUF_MAX;
}

/* response for command READ TOC */
static int do_read_toc(struct fsg_common *common, struct fsg_buffhd *bh)
{
	u8 *buf = NULL;
	struct usbsdms_read_toc_cmd_type *read_toc_cmd = NULL;
	unsigned long resp_len;
	u8 *resp_ptr = NULL;

	if (!common || !bh)
		return -EINVAL;
	if (get_boot_mode() == META_BOOT) {
		pr_info("Meta mode, use origin read toc\n");
		return read_toc_for_meta(common, bh);
	}

	buf = (u8 *)bh->buf;
	if (!buf)
		return -EINVAL;

	read_toc_cmd = (struct usbsdms_read_toc_cmd_type *)common->cmnd;
	if (!read_toc_cmd)
		return -EINVAL;

	/*
	 * When TIME is set to one, the address fields in some returned
	 * data formats shall be in TIME form.
	 */
	if (read_toc_cmd->msf == 2) { /* 2:time form mask */
		resp_ptr = g_usbsdms_read_toc_data2;
		resp_len = sizeof(g_usbsdms_read_toc_data2);
		goto buf_proc;
	}
	if (read_toc_cmd->allocation_length_msb != 0) {
		resp_ptr = g_usbsdms_read_toc_data3;
		resp_len = sizeof(g_usbsdms_read_toc_data3);
		goto buf_proc;
	}
	/*
	 * When TIME is set to zero, the address fields in some returned
	 * data formats shall be in LBA form.
	 */
	if (read_toc_cmd->format == 0) {
		/* 0 is mean to valid as a Track Number */
		resp_ptr = g_usbsdms_read_toc_data1_format0000;
		resp_len = sizeof(g_usbsdms_read_toc_data1_format0000);
	} else if (read_toc_cmd->format == 1) {
		/* 1 is mean to ignored by Logical Unit */
		resp_ptr = g_usbsdms_read_toc_data1_format0001;
		resp_len = sizeof(g_usbsdms_read_toc_data1_format0001);
	} else {
		/* Valid as a Session Number */
		resp_ptr = g_usbsdms_read_toc_data1;
		resp_len = sizeof(g_usbsdms_read_toc_data1);
	}
buf_proc:
	memcpy(buf, resp_ptr, resp_len);

	if (resp_len < common->data_size_from_cmnd)
		common->data_size_from_cmnd = resp_len;

	common->data_size = common->data_size_from_cmnd;
	common->residue = common->data_size;
	common->usb_amount_left = common->data_size;
	return resp_len;
}
#endif /* CONFIG_HUAWEI_USB */

static ssize_t mass_storage_inquiry_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct usb_function_instance *fi = NULL;
	struct fsg_opts *opts = NULL;

	if (!dev || !attr || !buf) {
		pr_err("inquiry store error\n");
		return -EINVAL;
	}

	fi = dev_get_drvdata(dev);
	if (!fi) {
		pr_err("fi is null\n");
		return -EINVAL;
	}
	opts = fsg_opts_from_func_inst(fi);
	if (!opts || !opts->common) {
		pr_err("opts is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", opts->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct usb_function_instance *fi = NULL;
	struct fsg_opts *opts = NULL;
	int len;

	if (!dev || !attr || !buf) {
		pr_err("inquiry store error\n");
		return -EINVAL;
	}

	fi = dev_get_drvdata(dev);
	if (!fi) {
		pr_err("fi is null\n");
		return -EINVAL;
	}
	opts = fsg_opts_from_func_inst(fi);
	if (!opts || !opts->common) {
		pr_err("opts is null\n");
		return -EINVAL;
	}

	len = min(size, sizeof(opts->common->inquiry_string) - 1);
	strncpy(opts->common->inquiry_string, buf, len);
	opts->common->inquiry_string[len] = 0;
	return len;
}

static DEVICE_ATTR(inquiry_string, 0644,
	mass_storage_inquiry_show, mass_storage_inquiry_store);

static struct device_attribute *g_mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

#ifdef CONFIG_HUAWEI_USB
static int create_mass_storage_device(struct usb_function_instance *fi)
{
	struct device *dev = NULL;
	struct device_attribute **attrs = NULL;
	struct device_attribute *attr = NULL;
	int err = 0;

	dev = create_function_device("f_mass_storage");
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	attrs = g_mass_storage_function_attributes;
	if (attrs) {
		do {
			attr = *attrs++;
			if (!attr)
				break;
			err = device_create_file(dev, attr);
		} while (!err);
		if (err) {
			device_destroy(dev->class, dev->devt);
			return -EINVAL;
		}
	}

	dev_set_drvdata(dev, fi);
	return 0;
}
#endif /* CONFIG_HUAWEI_USB */
