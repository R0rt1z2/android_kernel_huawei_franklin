/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Team:    Huawei DIVS
 * Date:    2020.07.20
 * Description: xhub big data channel module
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef _BIG_DATA_CHANNEL_H_
#define _BIG_DATA_CHANNEL_H_

typedef enum {
	INT_PARAM,
	STR_PARAM,
} big_data_param_type_t;

typedef enum {
	BIG_DATA_STR,
} big_data_str_tag_t;

typedef struct {
	const char *param_name;
	big_data_param_type_t param_type;
	int tag;
} big_data_param_detail_t;

typedef struct {
	int event_id;
	int param_num;
	big_data_param_detail_t *param_data;
} big_data_event_detail_t;

int iomcu_dubai_log_fetch(uint32_t event_type, void *data, uint32_t length);

#endif /* _BIG_DATA_CHANNEL_H_ */
