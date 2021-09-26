#include <linux/slab.h>
#include <mtk_swpm_sp_interface.h>

#include <chipset_common/dubai/dubai_plat.h>

#include "dubai_mtk_plat.h"

static int32_t g_nr_freq = -1;
static int32_t g_nr_ip = -1;
static struct ddr_sr_pd_times g_sr_pd_times;
static struct ddr_act_times *g_act_times = NULL;
static struct ddr_ip_bc_stats *g_ip_stats = NULL;

static bool dubai_init_ddr_time_in_state(void)
{
	g_nr_freq = get_ddr_freq_num();
	if (g_nr_freq <= 0) {
		dubai_err("Failed to get freq cnt: %d", g_nr_freq);
		return false;
	}

	g_act_times = kzalloc(g_nr_freq * sizeof(struct ddr_act_times), GFP_KERNEL);
	if (!g_act_times) {
		dubai_err("Failed to allocate mem for g_act_times");
		return false;
	}

	return true;
}

static bool dubai_init_ddr_ip_stats(void)
{
	int32_t i;

	g_nr_ip = get_ddr_data_ip_num();
	if (g_nr_ip <= 0) {
		dubai_err("Failed to get ip cnt: %d", g_nr_ip);
		return false;
	}

	g_ip_stats = kzalloc(g_nr_ip * sizeof(struct ddr_ip_bc_stats), GFP_KERNEL);
	if (!g_ip_stats) {
		dubai_err("Failed to allocate mem for g_ip_stats");
		goto init_failed;
	}
	for (i = 0; i < g_nr_ip; i++) {
		g_ip_stats[i].bc_stats = kzalloc(g_nr_freq * sizeof(struct ddr_bc_stats), GFP_KERNEL);
		if (!g_ip_stats[i].bc_stats) {
			dubai_err("Failed to allocate mem for bc_stats");
			goto free_ip_stats;
		}
	}

	return true;

free_ip_stats:
	for (i = 0; i < g_nr_ip; i++) {
		if (g_ip_stats[i].bc_stats)
			kfree(g_ip_stats[i].bc_stats);
	}
	kfree(g_ip_stats);
	g_ip_stats = NULL;
init_failed:
	return false;
}

static void dubai_clear_stats(void)
{
	int32_t i;

	g_nr_freq = -1;
	g_nr_ip = -1;

	if (g_act_times) {
		kfree(g_act_times);
		g_act_times = NULL;
	}
	if (g_ip_stats) {
		for (i = 0; i < g_nr_ip; i++) {
			if (g_ip_stats[i].bc_stats)
				kfree(g_ip_stats[i].bc_stats);
		}
		kfree(g_ip_stats);
		g_ip_stats = NULL;
	}
}

static bool dubai_check_ddr_stats(void)
{
	static bool inited = false;

	if (inited) {
		return true;
	}

	inited = dubai_init_ddr_time_in_state();
	if (!inited) {
		dubai_err("Failed to init time in state");
		goto init_failed;
	}

	inited = dubai_init_ddr_ip_stats();
	if (!inited) {
		dubai_err("Failed to init ip stats");
		goto init_failed;
	}

	return true;

init_failed:
	dubai_clear_stats();

	return false;
}

static int32_t dubai_get_ddr_freq_cnt(void)
{
	return get_ddr_freq_num();
}

static int32_t dubai_update_ddr_time_in_state(void)
{
	int32_t ret;

	memset(&g_sr_pd_times, 0, sizeof(struct ddr_sr_pd_times));
	memset(g_act_times, 0, sizeof(struct ddr_act_times) * g_nr_freq);
	dubai_update_swpm_stats();
	ret = get_ddr_act_times(g_nr_freq, g_act_times);
	if (ret < 0) {
		dubai_err("Failed to get ddr active times: %d", ret);
		return -1;
	}
	ret = get_ddr_sr_pd_times(&g_sr_pd_times);
	if (ret < 0) {
		dubai_err("Failed to get ddr sr and pd times: %d", ret);
		return -1;
	}

	return 0;
}

static int32_t dubai_get_ddr_time_in_state(struct dubai_ddr_time_in_state *time_in_state)
{
	int32_t i, ret;

	if (!dubai_check_ddr_stats()) {
		dubai_err("Failed to check ddr stats");
		return -1;
	}
	if ((g_nr_freq > DDR_FREQ_CNT_MAX) || !time_in_state) {
		dubai_err("Invalid parameter");
		return -1;
	}
	ret = dubai_update_ddr_time_in_state();
	if (ret < 0) {
		dubai_err("Faild to update ddr time_in_state stats");
		return -1;
	}
	time_in_state->sr_time = g_sr_pd_times.sr_time;
	time_in_state->pd_time = g_sr_pd_times.pd_time;
	for (i = 0; i < g_nr_freq; i++) {
		time_in_state->freq_time[i].freq = g_act_times[i].freq;
		time_in_state->freq_time[i].time = g_act_times[i].active_time;
	}

	return 0;
}

static int32_t dubai_update_ddr_ip_stats(void)
{
	int32_t i, ret;

	for (i = 0; i < g_nr_ip; i++) {
		memset(g_ip_stats[i].ip_name, 0, sizeof(g_ip_stats[i].ip_name));
		if (g_ip_stats[i].bc_stats)
			memset(g_ip_stats[i].bc_stats, 0, g_nr_freq * sizeof(struct ddr_bc_stats));
	}
	dubai_update_swpm_stats();
	ret = get_ddr_freq_data_ip_stats(g_nr_ip, g_nr_freq, g_ip_stats);
	if (ret < 0) {
		dubai_err("Failed to get ddr ip stats: %d", ret);
		return -1;
	}

	return 0;
}

static int32_t dubai_get_ddr_ip_cnt(void)
{
	return get_ddr_data_ip_num();
}

static int32_t dubai_get_ddr_ip_stats(int32_t ip_cnt, struct dubai_ddr_ip_stats *ip_stats)
{
	int32_t i, j, ret;

	if (!dubai_check_ddr_stats()) {
		dubai_err("Failed to check ddr stats");
		return -1;
	}

	if ((ip_cnt < g_nr_ip) || (g_nr_freq > DDR_FREQ_CNT_MAX) || !ip_stats) {
		dubai_err("Invalid parameter");
		return -1;
	}
	ret = dubai_update_ddr_ip_stats();
	if (ret < 0) {
		dubai_err("Faild to update ddr ip stats");
		return -1;
	}
	for (i = 0; i < g_nr_ip; i++) {
		strncpy(ip_stats[i].ip, g_ip_stats[i].ip_name, DDR_IP_NAME_LEN - 1);
		for (j = 0; j < g_nr_freq; j++) {
			ip_stats[i].freq_data[j].freq = g_ip_stats[i].bc_stats[j].freq;
			ip_stats[i].freq_data[j].data = g_ip_stats[i].bc_stats[j].value;
		}
	}

	return 0;
}

static struct dubai_ddr_stats_ops ddr_ops = {
	.get_freq_cnt = dubai_get_ddr_freq_cnt,
	.get_ip_cnt = dubai_get_ddr_ip_cnt,
	.get_time_in_state = dubai_get_ddr_time_in_state,
	.get_ip_stats = dubai_get_ddr_ip_stats,
};

void dubai_mtk_ddr_stats_init(void)
{
	dubai_register_module_ops(DUBAI_MODULE_DDR, &ddr_ops);
}

void dubai_mtk_ddr_stats_exit(void)
{
	dubai_unregister_module_ops(DUBAI_MODULE_DDR);
	dubai_clear_stats();
}
