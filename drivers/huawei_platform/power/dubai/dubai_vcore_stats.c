#include <linux/slab.h>
#include <mtk_swpm_sp_interface.h>

#include <chipset_common/dubai/dubai_plat.h>

#include "dubai_mtk_plat.h"

static int32_t g_nr_volt = -1;
static int32_t g_nr_ip = -1;
static struct vol_duration *g_volt_times = NULL;
static struct ip_stats *g_ip_stats = NULL;

static bool dubai_init_vcore_volt_time(void)
{
	g_nr_volt = get_vcore_vol_num();
	if (g_nr_volt <= 0) {
		dubai_err("Failed to get volt cnt: %d", g_nr_volt);
		return false;
	}

	g_volt_times = kzalloc(g_nr_volt * sizeof(struct vol_duration), GFP_KERNEL);
	if (!g_volt_times) {
		dubai_err("Failed to allocate mem for g_volt_times");
		return false;
	}

	return true;
}

static bool dubai_init_vcore_ip_stats(void)
{
	int32_t i;

	g_nr_ip = get_vcore_ip_num();
	if (g_nr_ip <= 0) {
		dubai_err("Failed to get ip cnt: %d", g_nr_ip);
		return false;
	}

	g_ip_stats = kzalloc(g_nr_ip * sizeof(struct ip_stats), GFP_KERNEL);
	if (!g_ip_stats) {
		dubai_err("Failed to allocate mem for g_ip_stats");
		goto init_failed;
	}
	for (i = 0; i < g_nr_ip; i++) {
		g_ip_stats[i].vol_times = kzalloc(g_nr_volt * sizeof(struct ip_vol_times), GFP_KERNEL);
		if (!g_ip_stats[i].vol_times) {
			dubai_err("Failed to allocate mem for vol_times");
			goto free_ip_stats;
		}
	}

	return true;

free_ip_stats:
	for (i = 0; i < g_nr_ip; i++) {
		if (g_ip_stats[i].vol_times)
			kfree(g_ip_stats[i].vol_times);
	}
	kfree(g_ip_stats);
	g_ip_stats = NULL;
init_failed:
	return false;
}

static void dubai_clear_stats(void)
{
	int32_t i;

	g_nr_volt = -1;
	g_nr_ip = -1;

	if (g_volt_times) {
		kfree(g_volt_times);
		g_volt_times = NULL;
	}
	if (g_ip_stats) {
		for (i = 0; i < g_nr_ip; i++) {
			if (g_ip_stats[i].vol_times)
				kfree(g_ip_stats[i].vol_times);
		}
		kfree(g_ip_stats);
		g_ip_stats = NULL;
	}
}

static bool dubai_check_vcore_stats(void)
{
	static bool inited = false;

	if (inited) {
		return true;
	}

	inited = dubai_init_vcore_volt_time();
	if (!inited) {
		dubai_err("Failed to init time in state");
		goto init_failed;
	}

	inited = dubai_init_vcore_ip_stats();
	if (!inited) {
		dubai_err("Failed to init ip stats");
		goto init_failed;
	}

	return true;

init_failed:
	dubai_clear_stats();

	return false;
}

static int32_t dubai_get_vcore_volt_cnt(void)
{
	return get_vcore_vol_num();
}

static int32_t dubai_update_vcore_volt_time(void)
{
	int32_t ret;

	memset(g_volt_times, 0, g_nr_volt * sizeof(struct vol_duration));
	dubai_update_swpm_stats();
	ret = get_vcore_vol_duration(g_nr_volt, g_volt_times);
	if (ret < 0) {
		dubai_err("Failed to get vcore volt times: %d", ret);
		return -1;
	}

	return 0;
}

static int32_t dubai_get_vcore_volt_time(int32_t volt_cnt, struct dubai_peri_volt_time *volt_times)
{
	int32_t i, ret;

	if (!dubai_check_vcore_stats()) {
		dubai_err("Failed to check vcore stats");
		return -1;
	}
	if ((volt_cnt < g_nr_volt) || !volt_times) {
		dubai_err("Invalid parameter");
		return -1;
	}
	ret = dubai_update_vcore_volt_time();
	if (ret < 0) {
		dubai_err("Faild to update vcore volt time");
		return -1;
	}
	for (i = 0; i < g_nr_volt; i++) {
		volt_times[i].volt = g_volt_times[i].vol;
		volt_times[i].time = g_volt_times[i].duration;
	}

	return 0;
}

static int32_t dubai_update_vcore_ip_stats(void)
{
	int32_t i, ret;

	for (i = 0; i < g_nr_ip; i++) {
		memset(g_ip_stats[i].ip_name, 0, sizeof(g_ip_stats[i].ip_name));
		if (g_ip_stats[i].vol_times)
			memset(g_ip_stats[i].vol_times, 0, g_nr_volt * sizeof(struct ip_vol_times));
	};
	dubai_update_swpm_stats();
	ret = get_vcore_ip_vol_stats(g_nr_ip, g_nr_volt, g_ip_stats);
	if (ret < 0) {
		dubai_err("Failed to get vcore ip stats: %d", ret);
		return -1;
	}

	return 0;
}

static int32_t dubai_get_vcore_ip_cnt(void)
{
	return get_vcore_ip_num();
}

static int32_t dubai_get_vcore_ip_stats(int32_t ip_cnt, struct dubai_peri_ip_stats *ip_stats)
{
	int32_t i, j, ret;

	if (!dubai_check_vcore_stats()) {
		dubai_err("Failed to check vcore stats");
		return -1;
	}

	if ((ip_cnt < g_nr_ip) || (g_nr_volt > PERI_VOLT_CNT_MAX) || !ip_stats) {
		dubai_err("Invalid parameter");
		return -1;
	}
	ret = dubai_update_vcore_ip_stats();
	if (ret < 0) {
		dubai_err("Faild to update vcore ip stats");
		return -1;
	}
	for (i = 0; i < g_nr_ip; i++) {
		strncpy(ip_stats[i].ip, g_ip_stats[i].ip_name, PERI_IP_NAME_LEN - 1);
		for (j = 0; j < g_nr_volt; j++) {
			ip_stats[i].volt_time[j].volt = g_ip_stats[i].vol_times[j].vol;
			ip_stats[i].volt_time[j].active_time = g_ip_stats[i].vol_times[j].active_time;
			ip_stats[i].volt_time[j].idle_time = g_ip_stats[i].vol_times[j].idle_time;
			ip_stats[i].volt_time[j].off_time = g_ip_stats[i].vol_times[j].off_time;
		}
	}

	return 0;
}

static struct dubai_peri_stats_ops peri_ops = {
	.get_volt_cnt = dubai_get_vcore_volt_cnt,
	.get_ip_cnt = dubai_get_vcore_ip_cnt,
	.get_volt_time = dubai_get_vcore_volt_time,
	.get_ip_stats = dubai_get_vcore_ip_stats,
};

void dubai_mtk_vcore_stats_init(void)
{
	dubai_register_module_ops(DUBAI_MODULE_PERI, &peri_ops);
}

void dubai_mtk_vcore_stats_exit(void)
{
	dubai_unregister_module_ops(DUBAI_MODULE_PERI);
	dubai_clear_stats();
}
