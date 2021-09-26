/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef TOUCHSCREEN_INTERFACE_H
#define TOUCHSCREEN_INTERFACE_H

enum tp_fp_event {
	TP_EVENT_FINGER_DOWN = 0,
	TP_EVENT_FINGER_UP = 1,
	TP_EVENT_HOVER_DOWN = 2,
	TP_EVENT_HOVER_UP = 3,
	TP_FP_EVENT_MAX,
};

struct tp_to_udfp_data {
	unsigned int version;
	enum tp_fp_event udfp_event;
	unsigned int mis_touch_count_area;
	unsigned int touch_to_fingerdown_time;
	unsigned int pressure;
	unsigned int mis_touch_count_pressure;
	unsigned int tp_coverage;
	unsigned int tp_fingersize;
	unsigned int tp_x;
	unsigned int tp_y;
	unsigned int tp_major;
	unsigned int tp_minor;
	char tp_ori;
};

struct ud_fp_ops {
	int (*fp_irq_notify)(struct tp_to_udfp_data *tp_data);
};

void fp_ops_register(struct ud_fp_ops *ops);
void fp_ops_unregister(struct ud_fp_ops *ops);
struct ud_fp_ops *fp_get_ops(void);

#endif
