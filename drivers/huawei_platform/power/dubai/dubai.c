#include <linux/jiffies.h>
#include <linux/module.h>

#include "dubai_mtk_plat.h"

static int __init dubai_init(void)
{
	dubai_gpu_freq_stats_init();
	dubai_mtk_battery_stats_init();
	dubai_wakeup_stats_init();
	dubai_mtk_ddr_stats_init();
	dubai_mtk_vcore_stats_init();

	return 0;
}

static void __exit dubai_exit(void)
{
	dubai_gpu_freq_stats_exit();
	dubai_mtk_battery_stats_exit();
	dubai_wakeup_stats_exit();
	dubai_mtk_ddr_stats_exit();
	dubai_mtk_vcore_stats_exit();
}

late_initcall(dubai_init);
module_exit(dubai_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_DESCRIPTION("Huawei Device Usage Big-data Analytics Initiative Driver");
