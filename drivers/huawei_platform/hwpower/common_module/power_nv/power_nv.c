/*
 * power_nv.c
 *
 * nv(non-volatile) interface for power module
 *
 * Copyright (c) 2021-2021 Huawei Technologies Co., Ltd.
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

#include <chipset_common/hwpower/common_module/power_nv.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#ifdef CONFIG_HUAWEI_OEMINFO
#include <huawei_platform/oeminfo/oeminfo_def.h>
#endif
#include <chipset_common/hwpower/common_module/power_printk.h>

#define HWLOG_TAG power_nv
HWLOG_REGIST();

static struct power_nv_data_info g_power_nv_data[] = {
	{ POWER_NV_SCCALCUR, 61, "SCCAL_CUR" },
	{ POWER_NV_CUROFFSET, 62, "CUR_OFFSET" },
	{ POWER_NV_BLIMSW, 10, "BLIMSW" },
	{ POWER_NV_BBINFO, 11, "BBINFO" },
	{ POWER_NV_HWCOUL, 12, "HWCOUL" },
};

static struct power_nv_data_info *power_nv_get_data(enum power_nv_type type)
{
	int i;
	struct power_nv_data_info *p_data = g_power_nv_data;
	int size = ARRAY_SIZE(g_power_nv_data);

	if ((type < POWER_NV_TYPE_BEGIN) || (type >= POWER_NV_TYPE_END)) {
		hwlog_err("nv_type %d check fail\n", type);
		return NULL;
	}

	for (i = 0; i < size; i++) {
		if (type == p_data[i].type)
			break;
	}

	if (i >= size) {
		hwlog_err("nv_type %d find fail\n", type);
		return NULL;
	}

	hwlog_info("nv [%d]=%d,%u,%s\n",
		i, p_data[i].type, p_data[i].id, p_data[i].name);
	return &p_data[i];
}

#ifdef CONFIG_HUAWEI_OEMINFO
static int power_nv_mtk_write(uint32_t nv_number, const char *nv_name,
	const void *data, uint32_t data_len)
{
	struct oeminfo_info_user *nv_info = NULL;

	if (!nv_name || !data) {
		hwlog_err("nv_name or data is null\n");
		return -EINVAL;
	}

	nv_info = kzalloc(sizeof(*nv_info), GFP_KERNEL);
	if (!nv_info)
		return -EINVAL;

	nv_info->oeminfo_operation = OEMINFO_WRITE;
	nv_info->oeminfo_id = nv_number;
	nv_info->valid_size = data_len;
	memcpy(nv_info->oeminfo_data, data,
		(sizeof(nv_info->oeminfo_data) < nv_info->valid_size) ?
		sizeof(nv_info->oeminfo_data) : nv_info->valid_size);

	if (oeminfo_direct_access(nv_info)) {
		hwlog_err("nv %s write fail\n", nv_name);
		kfree(nv_info);
		return -EINVAL;
	}
	kfree(nv_info);

	hwlog_info("nv %s,%u write succ\n", nv_name, data_len);
	return 0;
}

static int power_nv_mtk_read(uint32_t nv_number, const char *nv_name,
	void *data, uint32_t data_len)
{
	struct oeminfo_info_user *nv_info = NULL;

	if (!nv_name || !data) {
		hwlog_err("nv_name or data is null\n");
		return -EINVAL;
	}

	nv_info = kzalloc(sizeof(*nv_info), GFP_KERNEL);
	if (!nv_info)
		return -EINVAL;

	nv_info->oeminfo_operation = OEMINFO_READ;
	nv_info->oeminfo_id = nv_number;
	nv_info->valid_size = data_len;

	if (oeminfo_direct_access(nv_info)) {
		hwlog_err("nv %s read fail\n", nv_name);
		kfree(nv_info);
		return -EINVAL;
	}
	memcpy(data, nv_info->oeminfo_data,
		(sizeof(nv_info->oeminfo_data) < nv_info->valid_size) ?
		sizeof(nv_info->oeminfo_data) : nv_info->valid_size);
	kfree(nv_info);

	hwlog_info("nv %s,%u read succ\n", nv_name, data_len);
	return 0;
}
#else
static int power_nv_mtk_write(uint32_t nv_number, const char *nv_name,
	const void *data, uint32_t data_len)
{
	return 0;
}

static int power_nv_mtk_read(uint32_t nv_number, const char *nv_name,
	void *data, uint32_t data_len)
{
	return 0;
}
#endif /* CONFIG_HUAWEI_OEMINFO */

int power_nv_write(enum power_nv_type type, const void *data, uint32_t data_len)
{
	struct power_nv_data_info *p_data = power_nv_get_data(type);

	if (!p_data)
		return -EINVAL;

	return power_nv_mtk_write(p_data->id, p_data->name, data, data_len);
}

int power_nv_read(enum power_nv_type type, void *data, uint32_t data_len)
{
	struct power_nv_data_info *p_data = power_nv_get_data(type);

	if (!p_data)
		return -EINVAL;

	return power_nv_mtk_read(p_data->id, p_data->name, data, data_len);
}
