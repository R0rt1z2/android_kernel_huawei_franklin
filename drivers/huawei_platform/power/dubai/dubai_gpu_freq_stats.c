#include <linux/slab.h>

#include <chipset_common/dubai/dubai_plat.h>

#include "ged_dvfs.h"

#define KHZ_TO_HZ	1000

static atomic_t stats_enable = ATOMIC_INIT(0);

extern unsigned int mt_gpufreq_get_dvfs_table_num(void);

static int dubai_set_gpu_enable(bool enable)
{
	atomic_set(&stats_enable, enable ? 1 : 0);
	dubai_info("Gpu stats enable: %d", enable ? 1 : 0);

	return 0;
}

static int dubai_get_gpu_info(struct dubai_gpu_freq_info *data, int num)
{
	int ret, i, freq_num;
	struct GED_DVFS_OPP_STAT *stat_data = NULL;

	if (!atomic_read(&stats_enable))
		return -EPERM;

	freq_num = (int)mt_gpufreq_get_dvfs_table_num();
	if (!data || (num != freq_num)) {
		dubai_err("Invalid param: %d, %d", num, freq_num);
		return -EINVAL;
	}

	stat_data = kzalloc(freq_num * sizeof(struct GED_DVFS_OPP_STAT), GFP_KERNEL);
	if (!stat_data) {
		dubai_err("Failed to alloc memory");
		return -ENOMEM;
	}

	ret = ged_dvfs_query_opp_cost(stat_data, freq_num, true);
	if (!ret) {
		for (i = 0; i < freq_num; i++) {
			data[i].freq = stat_data[i].uMem.ui32Freq * KHZ_TO_HZ;
			data[i].run_time = stat_data[i].ui64Active;
			data[i].idle_time = stat_data[i].ui64Idle;
		}
	} else {
		dubai_err("Failed to get gpu stats");
	}
	kfree(stat_data);

	return ret;
}

static int dubai_get_gpu_freq_num(void)
{
	return (int)mt_gpufreq_get_dvfs_table_num();
}

static struct dubai_gpu_stats_ops gpu_ops = {
	.enable = dubai_set_gpu_enable,
	.get_stats = dubai_get_gpu_info,
	.get_num = dubai_get_gpu_freq_num,
};

void dubai_gpu_freq_stats_init(void)
{
	dubai_register_module_ops(DUBAI_MODULE_GPU, &gpu_ops);
}

void dubai_gpu_freq_stats_exit(void)
{
	dubai_unregister_module_ops(DUBAI_MODULE_GPU);
	atomic_set(&stats_enable, 0);
}
