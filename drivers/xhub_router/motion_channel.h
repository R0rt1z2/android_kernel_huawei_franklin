/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: motion channel header file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2020-05-25
 */

#ifndef __MOTION_CHANNEL_H__
#define __MOTION_CHANNEL_H__

extern int stop_auto_motion;
extern int step_ref_cnt;
extern int flag_for_sensor_test;
extern bool really_do_enable_disable(int *ref_cnt, bool enable, int bit);
extern void save_step_count(void);

#endif
