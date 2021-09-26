
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <motion.h>
#include <SCP_sensorHub.h>
#include <hwmsensor.h>
#include <sensor_event.h>
#include <linux/platform_device.h>
#include <stepsignhub.h>
#include <linux/fb.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include "motion_route.h"
#include "sensor_para.h"
#include "sensor_info.h"

#define USER_WRITE_BUFFER_SIZE            (1 + (2 * (sizeof(int))))
#define MAX_SUPPORTED_MOTIONS_TYPE_CNT    ((MOTION_TYPE_END) + 1)
#define SUPPORTED_MOTIONS_TYPE_NODE_PATH  "/motion"
#define SUPPORTED_MOTIONS_TYPE_PROP       "supported_motions_type"
#define MOTION_SUPPORTED_FLAG             1
#define MOTION_UNSUPPORTED_FLAG           0
#define HF_MANAGER_SENSOR_CONFIG_CALI     4

#define EXT_PEDO_VERSION           2000
#define EXT_PEDO_VERSION_SIZE      36
#define EXT_PEDO_MAX_LEN_SIZE      600
#define EXT_PEDO_DATA_LEN          480
#define GAP_TIME_GAIN              1000
#define CURR_TIME_GAIN             1000000L
#define RTC_TIME_GAIN              1000000

static u32 g_supported_motions_type[MAX_SUPPORTED_MOTIONS_TYPE_CNT];
static bool motion_status[MOTION_TYPE_END]={0};
extern int motion_route_init(void);
extern void motion__route_destroy(void);
extern ssize_t motion_route_read(char __user *buf, size_t count);
extern ssize_t motion_route_write(char *buf, size_t count);


//static struct class *color_sensor_class;


struct normal_motion_result{
        uint8_t motion;
        uint8_t result;      
        int8_t status;     
        uint8_t data_len;
};


struct motions_cmd_map {
	int mhb_ioctl_app_cmd;
	int motion_type;
	obj_cmd_t cmd;
	obj_sub_cmd_t subcmd;
};
static const struct motions_cmd_map motions_cmd_map_tab[] = {
	{MHB_IOCTL_MOTION_START, -1, CMD_CMN_OPEN_REQ, SUB_CMD_NULL_REQ},
	{MHB_IOCTL_MOTION_STOP, -1, CMD_CMN_CLOSE_REQ, SUB_CMD_NULL_REQ},
	{MHB_IOCTL_MOTION_ATTR_START, -1, CMD_CMN_CONFIG_REQ, SUB_CMD_MOTION_ATTR_ENABLE_REQ},
	{MHB_IOCTL_MOTION_ATTR_STOP, -1, CMD_CMN_CONFIG_REQ, SUB_CMD_MOTION_ATTR_DISABLE_REQ},
	{MHB_IOCTL_MOTION_INTERVAL_SET, -1, CMD_CMN_INTERVAL_REQ, SUB_CMD_NULL_REQ},
};

static char *motion_type_str[] = {
	[MOTION_TYPE_START] = "start",
	[MOTION_TYPE_PICKUP] = "pickup",
	[MOTION_TYPE_FLIP] = "flip",
	[MOTION_TYPE_PROXIMITY] = "proximity",
	[MOTION_TYPE_SHAKE] = "shake",
	[MOTION_TYPE_TAP] = "tap",
	[MOTION_TYPE_TILT_LR] = "tilt_lr",
	[MOTION_TYPE_ROTATION] = "rotation",
	[MOTION_TYPE_POCKET] = "pocket",
	[MOTION_TYPE_ACTIVITY] = "activity",
	[MOTION_TYPE_TAKE_OFF] = "take_off",
	[MOTION_TYPE_EXTEND_STEP_COUNTER] = "ext_step_counter",
	[MOTION_TYPE_EXT_LOG] = "ext_log",
	[MOTION_TYPE_HEAD_DOWN] = "head_down",
	[MOTION_TYPE_PUT_DOWN] = "put_down",
	[MOTION_TYPE_REMOVE] = "remove",
 	[MOTION_TYPE_FALL] = "fall",
	[MOTION_TYPE_SIDEGRIP] = "sidegrip",
	[MOTION_TYPE_END] = "end",
};

static struct extend_step_para_t *extend_step;

struct timeval curr_time;

enum ap_scp_system_status_t {
	SCP_SCREEN_ON,
	SCP_SCREEN_OFF,
	SCP_SYSTEM_SLEEP,
};

static void get_extend_step_share_mem_addr(void)
{
	static bool shr_extend_step_mem_ready = false;

	if (shr_extend_step_mem_ready)
		return;
	extend_step = get_sensor_share_mem_addr(SHR_MEM_TYPE_PEDO);
	if (!extend_step) {
		pr_err("extend_step share dram not ready\n");
		return;
	}
	pr_info("extend_step share dram ready\n");
	shr_extend_step_mem_ready = true;
}

static void adapt_exend_step_timestamp(void)
{
	uint64_t exted_occur_rtc_time_us;
	uint64_t exted_recv_rtc_time_us;
	uint64_t gap_time_us;

	gap_time_us = extend_step->gap_time / GAP_TIME_GAIN; // convert to us
	do_gettimeofday(&curr_time);
	exted_recv_rtc_time_us = curr_time.tv_sec * (CURR_TIME_GAIN) + curr_time.tv_usec;
	exted_occur_rtc_time_us = exted_recv_rtc_time_us - gap_time_us;
	extend_step->begin_time = exted_occur_rtc_time_us / RTC_TIME_GAIN;
	pr_debug("gaptime = %lu , rec_rtc_time = %llu, begin_rtc_time = %llu\n",
		gap_time_us, exted_recv_rtc_time_us, extend_step->begin_time);
}

void scp_recv_extend_step_data(void)
{
	char step_counter_data[EXT_PEDO_MAX_LEN_SIZE] = {0};
	uint16_t extend_data_size;

	get_extend_step_share_mem_addr();
	if (!extend_step)
		return;

	adapt_exend_step_timestamp();
	pr_info("%s begin_rtc_time = %u, record_count = %u, total_count = %u\n",
		__func__, extend_step->begin_time, extend_step->record_count,
		extend_step->total_step_count);

	if ((extend_step->record_count <= 0) ||
		(extend_step->record_count == EXT_PEDO_VERSION))
		return;
	extend_data_size = EXT_PEDO_VERSION_SIZE +
		(extend_step->record_count - EXT_PEDO_VERSION) * 4;
	if (extend_data_size > EXT_PEDO_DATA_LEN)
		return;
	pr_info("%s, get ar status = %d, extend_step = %d, size = %d\n",
		__func__, extend_step->action_record[0].ar_record,
		extend_step->action_record[0].step_record, extend_data_size);
	step_counter_data[0] = 11; // 11 means ext step counter
	memcpy(&step_counter_data[1], &extend_step->begin_time,
		extend_data_size);
	motion_route_write(step_counter_data, extend_data_size + 1);
}

static void tell_ap_status_to_scp(enum ap_scp_system_status_t status)
{
	uint8_t app_config[4] = {0};

	app_config[0] = SENSOR_TYPE_STEP_COUNTER;  // Sensor type
	app_config[1] = HF_MANAGER_SENSOR_CONFIG_CALI;  // action
	app_config[2] = status;
	app_config[3] = 0;

	sensor_cfg_to_hub(ID_STEP_COUNTER, app_config, sizeof(app_config));
}

static int scp_fb_notifier(struct notifier_block *nb,
	unsigned long action, void *data)
{
	if (!data)
		return NOTIFY_OK;

	switch (action) {
	case FB_EVENT_BLANK: // change finished
	{
		struct fb_event *event = data;
		int *blank = event->data;
		if (registered_fb[0] != event->info) {
			 // only main screen on/off info send to hub
			pr_info("%s, not main screen info, return\n", __func__);
			return NOTIFY_OK;
		}
		switch (*blank) {
		case FB_BLANK_UNBLANK: // screen on
			pr_info("%s, screen change to SCREEN_ON\n", __func__);
			tell_ap_status_to_scp(SCP_SCREEN_ON);
			get_status_from_ap();
			break;
		case FB_BLANK_POWERDOWN: // screen off
			pr_info("%s, screen change to SCREEN_OFF\n", __func__);
			tell_ap_status_to_scp(SCP_SCREEN_OFF);
			break;
		default:
			pr_err("lcd unknown in %s\n", __func__);
			break;
		}
		break;
	}
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block fb_notify = {
	.notifier_call = scp_fb_notifier,
};

static int scp_pm_notify(struct notifier_block *nb,
	unsigned long mode, void *_unused)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE: // suspend
		pr_info("suspend in %s\n", __func__);
		break;

	case PM_POST_SUSPEND: // resume
		pr_info("resume in %s\n", __func__);
		break;

	case PM_HIBERNATION_PREPARE: // Going to hibernate
	case PM_POST_HIBERNATION: // Hibernation finished
	case PM_RESTORE_PREPARE: // Going to restore a saved image
	case PM_POST_RESTORE: // Restore failed
	default:
		break;
	}

	return 0;
}

static void read_supported_motions_type_from_dts(void)
{
	struct device_node *np = NULL;
	int supported_motions_count;
	int ret;

	memset(g_supported_motions_type, 0,
		sizeof(g_supported_motions_type));

	np = of_find_node_by_path(SUPPORTED_MOTIONS_TYPE_NODE_PATH);
	if (!np) {
		hwlog_info("%s, motion node not exist!\n", __func__);
		return;
	}

	supported_motions_count = of_property_count_u32_elems(np,
		SUPPORTED_MOTIONS_TYPE_PROP);
	if (supported_motions_count < 0) {
		hwlog_info("%s, no valid value exist!\n", __func__);
		return;
	}
	if (supported_motions_count > MAX_SUPPORTED_MOTIONS_TYPE_CNT) {
		hwlog_info("%s, buffer is not large enough!\n",
			__func__);
		return;
	}

	ret = of_property_read_u32_array(np, SUPPORTED_MOTIONS_TYPE_PROP,
		g_supported_motions_type, supported_motions_count);
	if (ret != 0 && ret != -ENODATA)
		hwlog_info("%s, read supported motions prop fail!\n",
			__func__);
}

static bool is_motion_supported(u32 motion_type)
{
	int i;

	for (i = 0; i < MAX_SUPPORTED_MOTIONS_TYPE_CNT; i++) {
		if (motion_type == g_supported_motions_type[i])
			return true;
	}
	return false;
}

static int motion_support_query(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	if (!arg)
		return -EFAULT;
	int motion_type = MOTIONHUB_TYPE_POPUP_CAM;
	int supported = MOTION_SUPPORTED_FLAG;
	int unsupported = MOTION_UNSUPPORTED_FLAG;

	if (copy_from_user(&motion_type, argp, sizeof(motion_type))) {
		hwlog_err("%s, copy motion type fail!\n", __func__);
		return -EFAULT;
	}

	if (is_motion_supported(motion_type)) {
		if (copy_to_user((void __user *)arg,
			(void *)&supported,
			sizeof(supported)) != 0) {
			hwlog_err("%s, supported copy_to_user error\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	if (copy_to_user((void __user *)arg,
		(void *)&unsupported,
		sizeof(unsupported)) != 0) {
		hwlog_err("%s, unsupported copy_to_user error\n",
			__func__);
		return -EFAULT;
	}
	return 0;
}

static void update_motion_info(obj_cmd_t cmd, motion_type_t type)
{
	if (!(MOTION_TYPE_START <= type && type < MOTION_TYPE_END))
		return;

	switch (cmd) {
	case CMD_CMN_OPEN_REQ:
		motion_status[type] = true;
		break;

	case CMD_CMN_CLOSE_REQ:
		motion_status[type] = false;
		break;

	default:
		hwlog_err("unknown cmd type in %s\n", __func__);
		break;
	}
}

static bool need_motion_close(void)
{
	int i =0;
	for(i=0; i< sizeof(motion_status) / sizeof(motion_status[0]); i++){
		if(motion_status[i]) {
			break;
		}
	}
	if( i==sizeof(motion_status) / sizeof(motion_status[0])){
		return true;
	}
	return false;
}
static bool need_motion_open(void)
{
	int i = 0;
	int count = 0;
	for(i=0; i< sizeof(motion_status) / sizeof(motion_status[0]); i++){
		if(motion_status[i]) {
			count++;
		}
	}
	if(1 == count){//only first need open
		return true;
	}
	return false;
}
static int send_motion_cmd_internal(obj_cmd_t cmd, obj_sub_cmd_t subcmd, motion_type_t type)
{
	uint8_t app_config[2] = { 0, };
	int en = 0;
	app_config[0] = type;
	app_config[1] = cmd;

	if (CMD_CMN_OPEN_REQ == cmd) {
		en = 1;
		if(need_motion_open()){
	    	sensor_enable_to_hub(ID_HW_MOTION, en);
			hwlog_info("send_motion_cmd send enable  cmd!");
		}
		sensor_cfg_to_hub(ID_HW_MOTION, app_config, sizeof(app_config));
		hwlog_info("send_motion_cmd config cmd:%d motion: %s !", cmd, motion_type_str[type]);
	} else if (CMD_CMN_CLOSE_REQ == cmd) {
		/*send config cmd to close motion type*/
		en = 0;
		sensor_cfg_to_hub(ID_HW_MOTION, app_config, sizeof(app_config));
		hwlog_info("send_motion_cmd config cmd:%d motion: %s !", cmd, motion_type_str[type]);
		if(need_motion_close()){
			sensor_enable_to_hub(ID_HW_MOTION, en);
			hwlog_info("send_motion_cmd send disable  cmd!");
		}
	} else {
		hwlog_err("send_motion_cmd not support cmd!\n");
		return -EINVAL;
	}

	return 0;
}

static int send_motion_cmd(unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int argvalue = 0;
	int i;

	for (i = 0; i < sizeof(motions_cmd_map_tab) / sizeof(motions_cmd_map_tab[0]); ++i) {
		if (motions_cmd_map_tab[i].mhb_ioctl_app_cmd == cmd) {
			break;
		}
	}

	if (sizeof(motions_cmd_map_tab) / sizeof(motions_cmd_map_tab[0]) == i) {
		hwlog_err("send_motion_cmd unknown cmd %d in parse_motion_cmd!\n", cmd);
		return -EFAULT;
	}

	if (copy_from_user(&argvalue, argp, sizeof(argvalue)))
		return -EFAULT;

	if (!(MOTION_TYPE_START <= argvalue && argvalue < MOTION_TYPE_END)) {
		hwlog_err("error motion type %d in %s\n", argvalue, __func__);
		return -EINVAL;
	}
#ifdef CONFIG_MTK_STEPSIGNHUB
	if(argvalue == MOTION_TYPE_EXTEND_STEP_COUNTER){
		//operate pedo
		hwlog_info("%s operate ext step count cmd = %d\n", __func__, motions_cmd_map_tab[i].cmd);
		ext_step_counter_enable(motions_cmd_map_tab[i].cmd);
	}
#endif
	update_motion_info(motions_cmd_map_tab[i].cmd, argvalue);

	return send_motion_cmd_internal(motions_cmd_map_tab[i].cmd,
		motions_cmd_map_tab[i].subcmd, argvalue);
}

#define MAX_STEP_REPORT_LEN 37
int ext_step_counter_report(struct data_unit_t *event,
	void *reserved)
{
	//struct hw_step_counter_t_v2 ext_hw_step_counter_t;
	char step_counter_data[MAX_STEP_REPORT_LEN]={0};//define max step counter data length
	u_int16_t record_count =2000;//define default 2000
	u_int16_t capability = 1;//capability set to default 1
	 u_int32_t total_step_count =0;
	if(event == NULL){
		return -1;
	}
	memset(step_counter_data, 0, sizeof(step_counter_data));
	if(event->flush_action == FLUSH_ACTION || event->flush_action == DATA_ACTION){
		hwlog_info("%s, step counter recv ext data = %d\n", __func__, event->step_counter_t.accumulated_step_count);
		/*ext_hw_step_counter_t.motion = MOTION_TYPE_EXTEND_STEP_COUNTER;
		ext_hw_step_counter_t.record_count = 2000;
		ext_hw_step_counter_t.capability = 1;
		ext_hw_step_counter_t.total_step_count = event->step_counter_t.accumulated_step_count;*/
		total_step_count =  event->step_counter_t.accumulated_step_count;
		step_counter_data[0]=MOTION_TYPE_EXTEND_STEP_COUNTER;
		memcpy(&step_counter_data[5],&record_count,sizeof(u_int16_t));
		memcpy(&step_counter_data[7],&capability,sizeof(u_int16_t));
		memcpy(&step_counter_data[9],&total_step_count,sizeof(u_int32_t));
		motion_route_write(step_counter_data,sizeof(step_counter_data));
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ext_step_counter_report);

static int motion_channel_recv_data(struct data_unit_t *event,
	void *reserved)
{
	struct normal_motion_result normal_motion_result_t;
	if (event->flush_action == FLUSH_ACTION)
		hwlog_info("stat do not support flush\n");
	else if (event->flush_action == DATA_ACTION){
		hwlog_info("motion recv action data0=%d,data1=%d,data2=%d. \n",event->hw_motion_event.data[0],event->hw_motion_event.data[1],event->hw_motion_event.data[2]);
		if(event->hw_motion_event.data[0] == MOTION_TYPE_PICKUP ||
			event->hw_motion_event.data[0] == MOTION_TYPE_REMOVE ||
			event->hw_motion_event.data[0] == MOTION_TYPE_FALL) {
			normal_motion_result_t.motion = event->hw_motion_event.data[0];
			normal_motion_result_t.result = 1;
			normal_motion_result_t.status = (uint8_t)event->hw_motion_event.data[1];
			normal_motion_result_t.data_len=0;
			motion_route_write((char*)&normal_motion_result_t,sizeof(struct normal_motion_result));
		}else{
			hwlog_info("do not support type=%d. \n",event->hw_motion_event.data[0]);
		}
	}
	return 0;
}


/*******************************************************************************************
Function:       mhb_read
Description:   read /dev/motionhub
Data Accessed:  no
Data Updated:   no
Input:          struct file *file, char __user *buf, size_t count, loff_t *pos
Output:         no
Return:         length of read data
*******************************************************************************************/
static ssize_t mhb_read(struct file *file, char __user *buf, size_t count,
			loff_t *pos)
{

	return motion_route_read(buf,count);
}

/*******************************************************************************************
Function:       mhb_write
Description:   write to /dev/motionhub, do nothing now
Data Accessed:  no
Data Updated:   no
Input:          struct file *file, const char __user *data, size_t len, loff_t *ppos
Output:         no
Return:         length of write data
*******************************************************************************************/
static ssize_t mhb_write(struct file *file, const char __user *data,
			 size_t len, loff_t *ppos)
{
	char user_data[USER_WRITE_BUFFER_SIZE] = {0};
	char motion_type;

	if (len != USER_WRITE_BUFFER_SIZE) {
		hwlog_err("%s length is invalid\n", __func__);
		return len;
	}

	if (copy_from_user(user_data, data, len)) {
		hwlog_err("%s copy_from_user failed\n", __func__);
		return len;
	}

	motion_type = user_data[0];
	if (motion_type == MOTIONHUB_TYPE_POPUP_CAM) {
		if (motion_route_write(user_data, len) == 0)
			hwlog_err("%s route_write failed\n",
				__func__);
	}
	return len;
}

/*******************************************************************************************
Function:       mhb_ioctl
Description:   ioctrl function to /dev/motionhub, do open, close motion, or set interval and attribute to motion
Data Accessed:  no
Data Updated:   no
Input:          struct file *file, unsigned int cmd, unsigned long arg
			cmd indicates command, arg indicates parameter
Output:         no
Return:         result of ioctrl command, 0 successed, -ENOTTY failed
*******************************************************************************************/
static long mhb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case MHB_IOCTL_MOTION_START:
	case MHB_IOCTL_MOTION_STOP:
	case MHB_IOCTL_MOTION_ATTR_START:
	case MHB_IOCTL_MOTION_ATTR_STOP:
		break;
	case MHB_IOCTL_MOTION_SUPPORT_QUERY:
		return motion_support_query(arg);
	default:
		hwlog_err("%s unknown cmd : %d\n", __func__, cmd);
		return -ENOTTY;
	}
	return send_motion_cmd(cmd, arg);
}

/*******************************************************************************************
Function:       mhb_open
Description:   open to /dev/motionhub, do nothing now
Data Accessed:  no
Data Updated:   no
Input:          struct inode *inode, struct file *file
Output:         no
Return:         result of open
*******************************************************************************************/
static int mhb_open(struct inode *inode, struct file *file)
{
	hwlog_info("%s ok!\n", __func__);
	return 0;
}

/*******************************************************************************************
Function:       mhb_release
Description:   releaseto /dev/motionhub, do nothing now
Data Accessed:  no
Data Updated:   no
Input:          struct inode *inode, struct file *file
Output:         no
Return:         result of release
*******************************************************************************************/
static int mhb_release(struct inode *inode, struct file *file)
{
	hwlog_info("%s ok!\n", __func__);
	return 0;
}


/*******************************************************************************************
Description:   file_operations to motion
*******************************************************************************************/
static const struct file_operations mhb_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = mhb_read,
	.write = mhb_write,
	.unlocked_ioctl = mhb_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mhb_ioctl,
#endif
	.open = mhb_open,
	.release = mhb_release,
};

/*******************************************************************************************
Description:   miscdevice to motion
*******************************************************************************************/
static struct miscdevice motionhub_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "motionhub",
	.fops = &mhb_fops,
};

ssize_t motion_debug_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{

	bool state = false;
	if(NULL == dev || NULL == attr || NULL == buf)
	{
		hwlog_err("[%s] input NULL!! \n", __func__);
		return -1;
	}
	hwlog_info("[%s] motion_debug_store in!! \n", __func__);
	strtobool(buf, &state);
	hwlog_info("[%s]  enable = %d.\n", __func__,state);
	if(state){
		update_motion_info(CMD_CMN_OPEN_REQ, MOTION_TYPE_PICKUP);
		send_motion_cmd_internal(CMD_CMN_OPEN_REQ,SUB_CMD_NULL_REQ,MOTION_TYPE_PICKUP);
	}else{
	    update_motion_info(CMD_CMN_CLOSE_REQ, MOTION_TYPE_PICKUP);
		send_motion_cmd_internal(CMD_CMN_CLOSE_REQ,SUB_CMD_NULL_REQ,MOTION_TYPE_PICKUP);
	}		
	return size;
}

#if 0
DEVICE_ATTR(motion_debug, 0660, NULL, motion_debug_store);

static struct attribute *color_sensor_attributes[] = {
	&dev_attr_motion_debug.attr,
	NULL,
};

static const struct attribute_group color_sensor_attr_group = {
	.attrs = color_sensor_attributes
};

static const struct attribute_group color_sensor_attr_groups[] = {
	&color_sensor_attr_group
};
#endif

/*******************************************************************************************
Function:       motionhub_init
Description:   apply kernel buffer, register motionhub_miscdev
Data Accessed:  no
Data Updated:   no
Input:          void
Output:        void
Return:        result of function, 0 successed, else false
*******************************************************************************************/
static int __init motionhub_init(void)
{
	int ret;

	ret = misc_register(&motionhub_miscdev);
	if (ret != 0) {
		hwlog_err("cannot register miscdev err=%d\n", ret);
		goto exit1;
	}
	ret = scp_sensorHub_data_registration(ID_HW_MOTION,
		motion_channel_recv_data);
	if (ret != 0) {
		hwlog_err("cannot register cp_sensorHub_data_registration  err=%d\n", ret);
		goto exit2;
	}
	ret = motion_route_init();
	if (ret != 0) {
		hwlog_err("cannot motion_route_init  err=%d\n", ret);
		goto exit2;
	}

	fb_register_client(&fb_notify);
	pm_notifier(scp_pm_notify, 0);
	/* debuge file node */
	read_supported_motions_type_from_dts();
	return ret;
exit1:
	return -1;
exit2:
	misc_deregister(&motionhub_miscdev);
	return -1;

}

/*******************************************************************************************
Function:       motionhub_exit
Description:   release kernel buffer, deregister motionhub_miscdev
Data Accessed:  no
Data Updated:   no
Input:          void
Output:        void
Return:        void
*******************************************************************************************/
static void __exit motionhub_exit(void)
{
	misc_deregister(&motionhub_miscdev);
	motion__route_destroy();
	hwlog_info("exit %s\n", __func__);
}

late_initcall_sync(motionhub_init);
module_exit(motionhub_exit);

MODULE_AUTHOR("MotionHub <smartphone@huawei.com>");
MODULE_DESCRIPTION("MotionHub driver");
MODULE_LICENSE("GPL");
