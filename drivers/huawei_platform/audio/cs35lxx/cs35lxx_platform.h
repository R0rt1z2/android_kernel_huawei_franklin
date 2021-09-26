/*
 * linux/sound/cs35lxx.h  --  CS35LXX Platform data
 *
 * Copyright 2020 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __CS35LXX_H
#define __CS35LXX_H

struct cs35lxx_vpbr_cfg {
	bool is_present;
	bool vpbr_en;
	int vpbr_thld;
	int vpbr_atk_rate;
	int vpbr_atk_vol;
	int vpbr_max_attn;
	int vpbr_wait;
	int vpbr_rel_rate;
	int vpbr_mute_en;
};

struct asp_cfg {
	int asp_rx_width;
	int asp_tx_width;
	int asp_fmt;
	int asp_sample_rate;
	int asp_sclk_rate;
};

struct cs35lxx_platform_data {
	bool multi_amp_mode;
	bool dcm_mode;
	bool amp_pcm_inv;
	bool imon_pol_inv;
	bool vmon_pol_inv;
	int boost_ind;
	int bst_vctl;
	int bst_vctl_sel;
	int bst_ipk;
	bool extern_boost;
	int temp_warn_thld;
	int irq_drv_sel;
	int irq_gpio_sel;
	int pll_refclk_sel;
	int pll_refclk_freq;
	struct cs35lxx_vpbr_cfg vpbr_config;
	struct asp_cfg asp_config;
};

#endif /* __CS35LXX_H */
