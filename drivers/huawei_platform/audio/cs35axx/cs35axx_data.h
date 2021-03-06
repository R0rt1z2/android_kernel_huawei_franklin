/*
 * linux/platform_data/cs35l41.h -- Platform data for CS35L41
 *
 * Copyright (c) 2018 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L41_H
#define __CS35L41_H

struct classh_cfg {
	bool classh_bst_override;
	bool classh_algo_enable;
	int classh_bst_max_limit;
	int classh_mem_depth;
	int classh_release_rate;
	int classh_headroom;
	int classh_wk_fet_delay;
	int classh_wk_fet_thld;
};

struct irq_cfg {
	bool is_present;
	bool irq_pol_inv;
	bool irq_out_en;
	int irq_src_sel;
};

struct asp_cfg {
	int asp_rx_wl;
	int asp_rx_width;
	int asp_fmt;
	int asp_srate;
};

struct cs35l41_platform_data {
	bool sclk_frc;
	bool lrclk_frc;
	bool right_channel;
	bool amp_gain_zc;
	bool ng_enable;
	int bst_ind;
	int bst_vctrl;
	int bst_ipk;
	int bst_cap;
	int temp_warn_thld;
	int ng_pcm_thld;
	int ng_delay;
	int dout_hiz;
	int pll_refclk_sel;
	int pll_refclk_freq;
	struct irq_cfg irq_config1;
	struct irq_cfg irq_config2;
	struct classh_cfg classh_config;
	struct asp_cfg asp_config;
};

#endif /* __CS35L41_H */
