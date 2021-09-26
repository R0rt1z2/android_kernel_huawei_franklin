/* Copyright (C) 2020-2023 huawei Software Operations
 * Description: ufs-inline driver file
 */
#include "ufs-inline.h"
#include <linux/debugfs.h>
#include <uni/fbe/fbe_ctrl.h>
#include <linux/of.h>
#include "ufs.h"
#include "securec.h"

#ifdef CONFIG_MAS_BOOTDEVICE
#include <linux/bootdevice.h>
#endif
 /* workaround for York only by dts,
 *  it`s not need hw-inline2.0 before VN1 */
static int g_inline = DEBUG_DTS_INLINE_ON;

static u32 bkdrhash_alg(const u8 *str, int len)
{
	static u32 seed = UFS_INLINE_HASH_SEED;
	u32 hash = UFS_VOLATE_DEFAULT_VALUE;
	int i;

	for (i = 0; i < len; i++)
		hash = hash * seed + str[i];

	return (hash & UFS_INLINE_HASH_MASK);
}

static void test_generate_cci_dun_use_bkdrhash(const u8 *key, int key_len)
{
	u32 crypto_cci;
	u64 dun;
	u32 hash_res;

	hash_res = bkdrhash_alg(key, key_len);
	crypto_cci = hash_res % MAX_CRYPTO_KEY_INDEX;
	dun = (u64)hash_res;
	pr_err("%s: ufs crypto key index is %u, dun is 0x%llu\n",
		__func__, crypto_cci, dun);
}

static ssize_t ufs_inline_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int ret_show;

	if (ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES) &
		MASK_INLINE_ENCRYPTO_SUPPORT)
		ret_show = UFS_INLINE_VERSION;

	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%d\n", ret_show);
}

void ufs_inline_crypto_attr(struct ufs_hba *hba)
{
	hba->inline_state.show = ufs_inline_stat_show;
	sysfs_attr_init(&hba->inline_state.attr);
	hba->inline_state.attr.name = "ufs_inline_stat";
	hba->inline_state.attr.mode = S_IRUSR | S_IRGRP;
	if (device_create_file(hba->dev, &hba->inline_state))
		dev_err(hba->dev,
			"Failed to create sysfs for ufs_inline_state\n");
}

#ifdef CONFIG_HUAWEI_KERNEL_DEBUG
static ssize_t ufs_inline_debug_show(
	struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	if (hba->inline_debug_flag == DEBUG_LOG_OFF) {
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s %d\n",
			"log_off", g_inline);
	} else if (hba->inline_debug_flag == DEBUG_LOG_ON) {
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s %d\n",
			"log_on", g_inline);
	} else if (hba->inline_debug_flag == DEBUG_CRYPTO_ON) {
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s %d\n",
			"crypto_on", g_inline);
	} else if (hba->inline_debug_flag == DEBUG_CRYPTO_OFF) {
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s %d\n",
			"crypto_off", g_inline);
	} else {
		return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "%s %d\n",
			"error", g_inline);
	}
}

static ssize_t ufs_inline_debug_store(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "off")) {
		hba->inline_debug_flag = DEBUG_LOG_OFF;
	} else if (sysfs_streq(buf, "on")) {
		hba->inline_debug_flag = DEBUG_LOG_ON;
	} else if (sysfs_streq(buf, "crypto_on")) {
		hba->inline_debug_flag = DEBUG_CRYPTO_ON;
	} else if (sysfs_streq(buf, "crypto_off")) {
		hba->inline_debug_flag = DEBUG_CRYPTO_OFF;
	} else {
		dev_err(hba->dev, "%s: invalid input debug parameter.\n", __func__);
		return -EINVAL;
	}

	return count;
}

static ssize_t ufs_inline_dun_cci_test(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	int i;
	char buf_temp[UFS_CCI_KEY_LEN] = {0};

	if (count != UFS_CCI_KEY_LEN) {
		dev_err(dev, "%s: the input key len is not 64.\n", __func__);
		return count;
	}

	for (i = 0; i < (UFS_CCI_KEY_LEN - 1); i++)
		buf_temp[i] = buf[i];

	buf_temp[UFS_CCI_KEY_LEN - 1] = '\0';
	dev_err(dev, "%s: input key is %s\n", __func__, buf_temp);
	/* bkdr hash length 64 */
	test_generate_cci_dun_use_bkdrhash((u8 *)buf_temp, (UFS_CCI_KEY_LEN - 1));

	return count;
}

void ufs_inline_crypto_debug_init(struct ufs_hba *hba)
{
	hba->inline_debug_flag = DEBUG_LOG_OFF;

	hba->inline_debug_state.show = ufs_inline_debug_show;
	hba->inline_debug_state.store =
		ufs_inline_debug_store;
	sysfs_attr_init(&hba->inline_debug_state.attr);
	hba->inline_debug_state.attr.name = "ufs_inline_debug";
	hba->inline_debug_state.attr.mode = 0640; /* 0640 node attribute mode */
	if (device_create_file(hba->dev, &hba->inline_debug_state))
		dev_err(hba->dev,
			"Failed to create sysfs for inline_debug_state\n");

	hba->inline_dun_cci_test.store =
		ufs_inline_dun_cci_test;
	sysfs_attr_init(&hba->inline_dun_cci_test.attr);
	hba->inline_dun_cci_test.attr.name =
		"ufs_inline_dun_cci_test";
	hba->inline_dun_cci_test.attr.mode = 0200; /* 0200 node attribute mode */
	if (device_create_file(hba->dev, &hba->inline_dun_cci_test))
		dev_err(hba->dev,
			"Failed to create sysfs for inline_dun_cci_test\n");
}

static void ufs_inline_crypto_debug(const struct ufs_hba *hba,
	u32 *hash_res, u32 *crypto_enable, u64 dun)
{
	if (hba->inline_debug_flag == DEBUG_LOG_ON)
		dev_err(hba->dev, "%s: dun is 0x%llx\n", __func__,
			(((u64)(*hash_res)) << UFS_UPPER_LOW_OFFSET) | dun);

	if (hba->inline_debug_flag == DEBUG_CRYPTO_ON)
		*crypto_enable = UTP_REQ_DESC_CRYPTO_ENABLE;
	else if (hba->inline_debug_flag == DEBUG_CRYPTO_OFF)
		*crypto_enable = 0x0;
}
#endif

/*
 * generate the key index use bkdrhash alg,we limit
 * the result in the range of 0~29
 */
static void ufs_crypto_set_keyindex(const struct ufs_hba *hba,
	struct ufshcd_lrb *lrbp, u32 *hash_res, u32 *crypto_cci)
{
	*hash_res = bkdrhash_alg((u8 *)lrbp->cmd->request->mas_req.ci_key,
		lrbp->cmd->request->mas_req.ci_key_len);

	lrbp->cmd->request->mas_req.ci_key_index =
		(int)((uint32_t)lrbp->cmd->request->mas_req.ci_key_index &
			0xff);
	/* valid ci_key_index range 0-31 */
	if ((lrbp->cmd->request->mas_req.ci_key_index < 0) ||
		(lrbp->cmd->request->mas_req.ci_key_index > UFS_SLOT_NUM_MAX)) {
		dev_err(hba->dev, "%s: ci_key index err is 0x%x\n", __func__,
			lrbp->cmd->request->mas_req.ci_key_index);
		BUG();
	}

	*crypto_cci = (uint32_t)(lrbp->cmd->request->mas_req.ci_key_index);

	if (hba->inline_debug_flag == DEBUG_LOG_ON)
		dev_err(hba->dev, "%s: key index is %u\n", __func__,
			*crypto_cci);
}

static int ufs_uie_config_init(const struct ufs_hba *hba)
{
	unsigned int reg_value;
	int err;
	if (hba && hba->dev && hba->dev->of_node) {
		if (of_property_read_bool(hba->dev->of_node, "no-inline")) {
			g_inline = DEBUG_DTS_INLINE_OFF;
			dev_err(hba->dev, "%s: ufs inline not support\n", __func__);
		} else {
			g_inline = DEBUG_DTS_INLINE_ON;
			dev_err(hba->dev, "%s: ufs inline support\n", __func__);
		}
	}
	/* enable UFS cryptographic operations on transactions */
	reg_value = ufshcd_readl(hba, REG_CONTROLLER_ENABLE);
	reg_value |= CRYPTO_GENERAL_ENABLE;
	ufshcd_writel(hba, reg_value, REG_CONTROLLER_ENABLE);

	dev_err(hba->dev, "%s: Use UFS inline crypto V3.0 interface.\n", __func__);
	if (ufshcd_eh_in_progress(hba)) {
		err = huawei_fbex_restore_key();
		if (err)
			BUG();
	}
	return err;
}

/* configure UTRD to enable cryptographic operations for this transaction */
static void ufs_uie_utrd_prepare(const struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct utp_transfer_req_desc *req_desc = NULL;
	u32 dword[TEMP_BUFF_LENGTH] = {0};
	u64 dun;
	u32 crypto_cci;
	u32 hash_res;
	if (!lrbp) {
		pr_err("%s: lrbp addr is NULL\n", __func__);
		return;
	}

	req_desc = lrbp->utr_descriptor_ptr;
	if (lrbp->cmd->request && lrbp->cmd->request->mas_req.ci_key)
		ufs_crypto_set_keyindex(hba,
			lrbp, &hash_res, &crypto_cci);
	else {
		return;
	}
	dun = (u64)lrbp->cmd->request->bio->mas_bio.index;
	if (g_inline) {
		/*
		* According to UFS 2.1 SPEC
		* decrypte incoming payload if the command is SCSI READ operation
		* encrypte outgoing payload if the command is SCSI WRITE operation
		* Just flow with hisi`s config,and don`t know why,which only support:
		* READ_10/WRITE_10/WRITE_16
		*/
		if ((lrbp->cmd->cmnd[0] == READ_10) ||
			(lrbp->cmd->cmnd[0] == WRITE_10) || (lrbp->cmd->cmnd[0] == READ_16))
			dword[UFS_HEADER_DWORD_12] = UTP_REQ_DESC_CRYPTO_ENABLE;
		else
			return;
	}

#ifdef CONFIG_HUAWEI_KERNEL_DEBUG
	ufs_inline_crypto_debug(hba, &hash_res, &dword[UFS_HEADER_DWORD_12], dun);
#endif
	/* set val for dword */
	dword[UFS_HEADER_DWORD_0] = dword[UFS_HEADER_DWORD_12] | crypto_cci;
	dword[UFS_HEADER_DWORD_1] = (u32)(dun & UFS_DUN_MASK);
	dword[UFS_HEADER_DWORD_3] = (u32)((dun >> UFS_UPPER_LOW_OFFSET) | hash_res);

	req_desc->header.dword_0 |= cpu_to_le32(dword[UFS_HEADER_DWORD_0]);
	req_desc->header.dword_1 = cpu_to_le32(dword[UFS_HEADER_DWORD_1]);
	req_desc->header.dword_3 = cpu_to_le32(dword[UFS_HEADER_DWORD_3]);
}

 /**
 * ufshcd_support_inline_encrypt - Check if controller supports
 *                            UFS inline encrypt
 * @hba: per adapter instance
 */
bool ufshcd_support_inline_encrypt(const struct ufs_hba *hba)
{
	if (hba->capabilities & MASK_INLINE_ENCRYPTO_SUPPORT)
		return true;
	else
		return false;
}

int ufshcd_hba_uie_init(const struct ufs_hba *hba)
{
	int err;

	if (!ufshcd_support_inline_encrypt(hba))
		return 0;
	err = ufs_uie_config_init(hba);
	if (err)
		pr_err("ufs inline init fail !!!\n");
	else
		hba->host->crypto_enabled = 1;

	return err;
}

void ufshcd_prepare_req_desc_uie(const struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	if (ufshcd_support_inline_encrypt(hba))
		ufs_uie_utrd_prepare(hba, lrbp);
}
