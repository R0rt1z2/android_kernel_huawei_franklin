/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MTKFB_DEBUG_H
#define __MTKFB_DEBUG_H

#define LOGGER_BUFFER_SIZE (16 * 1024)
#define ERROR_BUFFER_COUNT 4
#define FENCE_BUFFER_COUNT 22
#define DEBUG_BUFFER_COUNT 30
#define DUMP_BUFFER_COUNT 10
#define STATUS_BUFFER_COUNT 1
#if defined(CONFIG_MT_ENG_BUILD) || !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
#define DEBUG_BUFFER_SIZE                                                      \
	(4096 +                                                                \
	 (ERROR_BUFFER_COUNT + FENCE_BUFFER_COUNT + DEBUG_BUFFER_COUNT +       \
	  DUMP_BUFFER_COUNT + STATUS_BUFFER_COUNT) *                           \
		 LOGGER_BUFFER_SIZE)
#else
#define DEBUG_BUFFER_SIZE 10240
#endif

#include <lcd_kit_utils.h>
struct mtk_fb_data_type {
	bool panel_power_on;
	struct mtk_panel_info panel_info;
};

extern int mtk_disp_hrt_bw_dbg(void);
extern bool setCaptureRect(int left, int top, int width, int height);
extern bool setCaptureInterval(int interval);
extern bool enableCapture(int en);

#ifdef _DRM_P_H_
void disp_dbg_probe(void);
void disp_dbg_init(struct drm_device *drm_dev);
void disp_dbg_deinit(void);
int mtk_dprec_mmp_dump_ovl_layer(struct mtk_plane_state *plane_state);
int disp_met_set(void *data, u64 val);
void mtk_drm_idlemgr_kick_ext(const char *source);
#ifdef CONFIG_LCD_KIT_DRIVER
void mtk_disp_read_ddic_switch(bool enable);
void lcm_reset(void);
void mtk_drm_esd_switch_ext(bool esd_enable);
#endif
int notify_dss_tui_request(void *pdata, int secure);
#endif

extern int ssmr_offline(phys_addr_t *pa, unsigned long *size, bool is_64bit,
		unsigned int feat);
extern int ssmr_online(unsigned int feat);
#endif
