/*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
* Description: mtk efuse display source file
* Author: Liujingbo
* Create: 2019-09-19
*/

#include <linux/module.h>
#include "sec_boot_lib.h"
#include "mtk_devinfo.h"

#define SECURE_DEBUG_INDEX 92

static struct kobject *g_efuseinfo_kobj = NULL;

static ssize_t efuse_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return sprintf(buf, "%d\n", sec_schip_enabled()? 1:0);
}

static ssize_t secure_debug_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",
		get_devinfo_with_index(SECURE_DEBUG_INDEX) ? 1 : 0);
}

static struct kobj_attribute sboot_efuse_info_attr = {
    .attr = {
        .name = "hw_efuse_info",
        .mode = 0444,
    },
    .show =&efuse_info_show,
};

static struct kobj_attribute secure_debug_attr = {
	.attr = {
		.name = "hw_secure_debug",
		.mode = 0444,
	},
	.show =&secure_debug_show,
};

static struct attribute * g_sboot_efuse_info_attr[] = {
    &sboot_efuse_info_attr.attr,
    &secure_debug_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = g_sboot_efuse_info_attr,
};

static int __init efuseinfo_init(void)
{
    int ret = -ENOMEM;

    g_efuseinfo_kobj = kobject_create_and_add("sboot_efuse_info", NULL);

    if (g_efuseinfo_kobj == NULL) {
        pr_debug("efuseinfo_init: kobject_create_and_add failed\n");
        goto fail;
    }

    ret = sysfs_create_group(g_efuseinfo_kobj, &attr_group);
    if (ret) {
        pr_debug("efuseinfo_init: sysfs_create_group failed\n");
        goto sys_fail;
    }

    return ret;
sys_fail:
    kobject_del(g_efuseinfo_kobj);
fail:
    return ret;

}

static void __exit efuseinfo_exit(void)
{
    if (g_efuseinfo_kobj) {
        sysfs_remove_group(g_efuseinfo_kobj, &attr_group);
        kobject_del(g_efuseinfo_kobj);
    }
}

module_init(efuseinfo_init);
module_exit(efuseinfo_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Boot information collector");
