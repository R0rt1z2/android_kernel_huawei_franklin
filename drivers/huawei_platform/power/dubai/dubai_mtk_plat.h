#ifndef DUBAI_MTK_PLAT_H
#define DUBAI_MTK_PLAT_H

void dubai_update_swpm_stats(void);

#ifdef CONFIG_HUAWEI_DUBAI_GPU_STATS
void dubai_gpu_freq_stats_init(void);
void dubai_gpu_freq_stats_exit(void);
#else // !CONFIG_HUAWEI_DUBAI_GPU_STATS
static inline void dubai_gpu_freq_stats_init(void) {}
static inline void dubai_gpu_freq_stats_exit(void) {}
#endif // CONFIG_HUAWEI_DUBAI_GPU_STATS

#ifdef CONFIG_HUAWEI_DUBAI_BATTERY_STATS
void dubai_mtk_battery_stats_init(void);
void dubai_mtk_battery_stats_exit(void);
#else // !CONFIG_HUAWEI_DUBAI_BATTERY_STATS
static inline void dubai_mtk_battery_stats_init(void) {}
static inline void dubai_mtk_battery_stats_exit(void) {}
#endif // CONFIG_HUAWEI_DUBAI_BATTERY_STATS

void dubai_wakeup_stats_init(void);
void dubai_wakeup_stats_exit(void);

#ifdef CONFIG_HUAWEI_DUBAI_DDR_STATS
void dubai_mtk_ddr_stats_init(void);
void dubai_mtk_ddr_stats_exit(void);
#else // !CONFIG_HUAWEI_DUBAI_DDR_STATS
static inline void dubai_mtk_ddr_stats_init(void) {}
static inline void dubai_mtk_ddr_stats_exit(void) {}
#endif // CONFIG_HUAWEI_DUBAI_DDR_STATS

#ifdef CONFIG_HUAWEI_DUBAI_VCORE_STATS
void dubai_mtk_vcore_stats_init(void);
void dubai_mtk_vcore_stats_exit(void);
#else // !CONFIG_HUAWEI_DUBAI_VCORE_STATS
static inline void dubai_mtk_vcore_stats_init(void) {}
static inline void dubai_mtk_vcore_stats_exit(void) {}
#endif // CONFIG_HUAWEI_DUBAI_VCORE_STATS

#endif // DUBAI_MTK_PLAT_H
