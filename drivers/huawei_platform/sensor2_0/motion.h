/*
 * motion.h
 *
 * code for motion channel
 *
 * Copyright (c) 2020- Huawei Technologies Co., Ltd.
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

#ifndef __MOTION_H
#define __MOTION_H
#include <huawei_platform/log/hw_log.h>

#define MHBIO                            0xB1
#define MHB_IOCTL_MOTION_START           _IOW(MHBIO, 0x01, short)
#define MHB_IOCTL_MOTION_STOP            _IOW(MHBIO, 0x02, short)
#define MHB_IOCTL_MOTION_ATTR_START      _IOW(MHBIO, 0x03, short)
#define MHB_IOCTL_MOTION_ATTR_STOP       _IOW(MHBIO, 0x04, short)
#define MHB_IOCTL_MOTION_INTERVAL_SET    _IOW(MHBIO, 0x05, short)
#define MHB_IOCTL_MOTION_SUPPORT_QUERY   _IOWR(MHBIO, 0x06, long)

#define HWLOG_TAG hw_motion
HWLOG_REGIST();

struct normal_motion_result {
	uint8_t motion;
	uint8_t result;
	int8_t status;
	uint8_t data_len;
};

enum obj_cmd_t{
	CMD_CMN_CLOSE_REQ,
	CMD_CMN_OPEN_REQ,
	CMD_CMN_INTERVAL_REQ,
	CMD_CMN_CONFIG_REQ,
};

enum obj_sub_cmd_t {
	SUB_CMD_NULL_REQ = 0x0,
	// motion
	SUB_CMD_MOTION_ATTR_ENABLE_REQ = 0x20,
	SUB_CMD_MOTION_ATTR_DISABLE_REQ,
	SUB_CMD_MOTION_REPORT_REQ,
	SUB_CMD_MAX = 0xff,
};

enum motion_type_t {
	MOTION_TYPE_START,
	MOTION_TYPE_PICKUP,
	MOTION_TYPE_FLIP,
	MOTION_TYPE_PROXIMITY,
	MOTION_TYPE_SHAKE,
	MOTION_TYPE_TAP,
	MOTION_TYPE_TILT_LR,
	MOTION_TYPE_ROTATION,
	MOTION_TYPE_POCKET,
	MOTION_TYPE_ACTIVITY,
	MOTION_TYPE_TAKE_OFF,
	MOTION_TYPE_EXTEND_STEP_COUNTER,
	MOTION_TYPE_EXT_LOG,
	MOTION_TYPE_HEAD_DOWN,
	MOTION_TYPE_PUT_DOWN,
	MOTION_TYPE_REMOVE,
	MOTION_TYPE_FALL = 16,
	MOTION_TYPE_TOUCH = 32,
	MOTION_TYPE_SIDEGRIP, // sensorhub internal use, must at bottom;
	MOTION_TYPE_END,
};

#define MOTIONHUB_TYPE_POPUP_CAM 0x1f

void scp_motion_data_report(struct hf_manager_event *data);

#endif
