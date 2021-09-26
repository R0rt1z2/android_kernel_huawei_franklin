

#ifndef __MOTION_H
#define __MOTION_H
#include <huawei_platform/log/hw_log.h>

/*------------------------IOCTRL----------------------*/

#define MHBIO                         0xB1
#define MHB_IOCTL_MOTION_START      _IOW(MHBIO, 0x01, short)
#define MHB_IOCTL_MOTION_STOP    _IOW(MHBIO, 0x02, short)
#define MHB_IOCTL_MOTION_ATTR_START      _IOW(MHBIO, 0x03, short)
#define MHB_IOCTL_MOTION_ATTR_STOP    _IOW(MHBIO, 0x04, short)
#define MHB_IOCTL_MOTION_INTERVAL_SET    _IOW(MHBIO, 0x05, short)
#define MHB_IOCTL_MOTION_SUPPORT_QUERY   _IOWR(MHBIO, 0x06, long)
/*------------------------HWLOG----------------------*/

#define HWLOG_TAG hw_motion
HWLOG_REGIST();


/*------------------------cmd----------------------*/


typedef enum {
    CMD_CMN_CLOSE_REQ,
	CMD_CMN_OPEN_REQ,
	CMD_CMN_INTERVAL_REQ,
	CMD_CMN_CONFIG_REQ,
} obj_cmd_t;

typedef enum{
	SUB_CMD_NULL_REQ = 0x0,

	/*motion*/
	SUB_CMD_MOTION_ATTR_ENABLE_REQ = 0x20,
	SUB_CMD_MOTION_ATTR_DISABLE_REQ,
	SUB_CMD_MOTION_REPORT_REQ,
	SUB_CMD_MOTION_HORIZONTAL_PICKUP_REQ,

    SUB_CMD_MAX = 0xff,
}obj_sub_cmd_t;

#if 0
struct hw_step_counter_t_v2
{
	u_int8_t motion;
	u_int32_t begin_time;//0
	u_int16_t record_count;//2000
	u_int16_t capability;// 1
	u_int32_t total_step_count;//fill count data
	u_int32_t total_floor_ascend;// 0
	u_int32_t total_calorie;// 0
	u_int32_t total_distance;// 0
	u_int16_t step_length;// 0
	u_int16_t step_pace;// 0
	u_int16_t speed;// 0
	u_int16_t touchdown_ratio;// 0
	u_int16_t reserve1;// 0
	u_int16_t reserve2;// 0
};
#endif

typedef enum {
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
	MOTION_TYPE_FALL,
	//
	MOTION_TYPE_SIDEGRIP, //sensorhub internal use, must at bottom;
	/*!!!NOTE:add string in motion_type_str when add type*/
	MOTION_TYPE_END,
} motion_type_t;

#define MOTIONHUB_TYPE_POPUP_CAM 0x1f
#endif
