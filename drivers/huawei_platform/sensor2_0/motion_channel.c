/*
 * motion_channel.c
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
#include "sensor_scp.h"
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pm_wakeup.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <motion.h>
#include <motion_route.h>
#include <linux/platform_device.h>

#define USER_WRITE_BUFFER_SIZE            ((1) + ((2) * (sizeof(int))))
#define MAX_SUPPORTED_MOTIONS_TYPE_CNT    ((MOTION_TYPE_END) + (1))
#define SUPPORTED_MOTIONS_TYPE_NODE_PATH  "/motion"
#define SUPPORTED_MOTIONS_TYPE_PROP       "supported_motions_type"
#define MOTION_SUPPORTED_FLAG             1
#define MOTION_UNSUPPORTED_FLAG           0

static u32 g_supported_motions_type[MAX_SUPPORTED_MOTIONS_TYPE_CNT];
static bool motion_status[MOTION_TYPE_END] = {0};
static struct wakeup_source wlock;

struct motions_cmd_map {
	int mhb_ioctl_app_cmd;
	int motion_type;
	enum obj_cmd_t cmd;
	enum obj_sub_cmd_t subcmd;
};

static const struct motions_cmd_map motions_cmd_map_tab[] = {
	{ MHB_IOCTL_MOTION_START, -1, CMD_CMN_OPEN_REQ, SUB_CMD_NULL_REQ },
	{ MHB_IOCTL_MOTION_STOP, -1, CMD_CMN_CLOSE_REQ, SUB_CMD_NULL_REQ },
	{ MHB_IOCTL_MOTION_ATTR_START, -1,
		CMD_CMN_CONFIG_REQ, SUB_CMD_MOTION_ATTR_ENABLE_REQ },
	{ MHB_IOCTL_MOTION_ATTR_STOP, -1,
		CMD_CMN_CONFIG_REQ, SUB_CMD_MOTION_ATTR_DISABLE_REQ },
	{ MHB_IOCTL_MOTION_INTERVAL_SET, -1,
		CMD_CMN_INTERVAL_REQ, SUB_CMD_NULL_REQ },
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
	[MOTION_TYPE_TOUCH] = "touch",
	[MOTION_TYPE_SIDEGRIP] = "sidegrip",
	[MOTION_TYPE_END] = "end",
};

static void read_supported_motions_type_from_dts(void)
{
	struct device_node *np = NULL;
	int supported_motions_count;
	int ret;

	memset(g_supported_motions_type, 0,
		sizeof(g_supported_motions_type));

	np = of_find_node_by_path(SUPPORTED_MOTIONS_TYPE_NODE_PATH);
	if (np == NULL) {
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
	} else {
		if (copy_to_user((void __user *)arg,
			(void *)&unsupported,
			sizeof(unsupported)) != 0) {
			hwlog_err("%s, unsupported copy_to_user error\n",
				__func__);
			return -EFAULT;
		}
	}
	return 0;
}

static void update_motion_info(enum obj_cmd_t cmd,
	enum motion_type_t type)
{
	if (!((MOTION_TYPE_START <= type) && (type < MOTION_TYPE_END)))
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
	int i;

	for (i = 0; i < ARRAY_SIZE(motion_status); i++)
		if(motion_status[i])
			break;

	if (i == ARRAY_SIZE(motion_status))
		return true;

	return false;
}

static bool need_motion_open(void)
{
	int i;
	int count = 0;

	for(i = 0; i< ARRAY_SIZE(motion_status); i++)
		if(motion_status[i])
			count++;

	if (count == 1) // only first need open
		return true;

	return false;
}

static int send_motion_cmd_internal(enum obj_cmd_t cmd,
	enum obj_sub_cmd_t subcmd, enum motion_type_t type)
{
	struct hf_manager_cmd cmd_in;

	cmd_in.sensor_type = SENSOR_TYPE_HW_MOTION;
	cmd_in.action = HF_MANAGER_SENSOR_CONFIG_CALI;
	cmd_in.data[0] = type;
	cmd_in.data[1] = cmd;

	if (cmd == CMD_CMN_OPEN_REQ) {
		if (need_motion_open()) {
			scp_sensor_ctrl_enable(SENSOR_TYPE_HW_MOTION, true);
			hwlog_info("send_motion_cmd send enable cmd\n");
		}
		if (type == MOTION_TYPE_EXTEND_STEP_COUNTER)
			scp_sensor_ctrl_enable(SENSOR_TYPE_STEP_COUNTER, true);
		scp_sensor_cfg_data(&cmd_in);
		hwlog_info("%s config cmd:%d motion: %s\n",
			__func__, cmd, motion_type_str[type]);
	} else if (cmd == CMD_CMN_CLOSE_REQ) {
		// send config cmd to close motion type
		if (type == MOTION_TYPE_EXTEND_STEP_COUNTER)
			scp_sensor_ctrl_enable(SENSOR_TYPE_STEP_COUNTER, false);
		scp_sensor_cfg_data(&cmd_in);
		hwlog_info("%s config cmd:%d motion: %s\n",
			__func__, cmd, motion_type_str[type]);
		if (need_motion_close()) {
			scp_sensor_ctrl_enable(SENSOR_TYPE_HW_MOTION, false);
			hwlog_info("send_motion_cmd send disable cmd\n");
		}
	} else {
		hwlog_err("send_motion_cmd not support cmd\n");
		return -EINVAL;
	}

	return 0;
}

static int send_motion_cmd(unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int argvalue = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(motions_cmd_map_tab); ++i)
		if (motions_cmd_map_tab[i].mhb_ioctl_app_cmd == cmd)
			break;

	if (i == ARRAY_SIZE(motions_cmd_map_tab)) {
		hwlog_err("%s unknown cmd %d in parse_motion_cmd!\n",
			__func__, cmd);
		return -EFAULT;
	}

	if (copy_from_user(&argvalue, argp, sizeof(argvalue)))
		return -EFAULT;

	if (!((MOTION_TYPE_START <= argvalue) &&
		(argvalue < MOTION_TYPE_END))) {
		hwlog_err("error motion type %d in %s\n", argvalue, __func__);
		return -EINVAL;
	}
	update_motion_info(motions_cmd_map_tab[i].cmd, argvalue);
	return send_motion_cmd_internal(motions_cmd_map_tab[i].cmd,
		motions_cmd_map_tab[i].subcmd, argvalue);
}

void scp_motion_data_report(struct hf_manager_event *data)
{
	struct normal_motion_result normal_motion_result_t;

	if (!data)
		return;

	normal_motion_result_t.motion = (uint8_t)(data->word[0]);
	normal_motion_result_t.result = 1; // get correct motion status
	normal_motion_result_t.status = (int8_t)(data->word[1]);
	normal_motion_result_t.data_len = 0;
	__pm_wakeup_event(&wlock, jiffies_to_msecs(HZ / 2));
	hwlog_info("%s pm wlock motion_type = %d, result = %d\n", __func__,
		normal_motion_result_t.motion,
		normal_motion_result_t.status);
	motion_route_write((char*)&normal_motion_result_t,
		sizeof(struct normal_motion_result));
}

static ssize_t mhb_read(struct file *file, char __user *buf, size_t count,
	loff_t *pos)
{
	return motion_route_read(buf,count);
}

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

static int mhb_open(struct inode *inode, struct file *file)
{
	hwlog_info("%s ok\n", __func__);
	return 0;
}

static int mhb_release(struct inode *inode, struct file *file)
{
	hwlog_info("%s ok\n", __func__);
	return 0;
}

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

static struct miscdevice motionhub_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "motionhub",
	.fops = &mhb_fops,
};

static int __init motionhub_init(void)
{
	int ret;

	ret = misc_register(&motionhub_miscdev);
	if (ret != 0) {
		hwlog_err("cannot register miscdev err=%d\n", ret);
		return -1;
	}

	ret = motion_route_init();
	if (ret != 0) {
		hwlog_err("cannot motion_route_init  err=%d\n", ret);
		goto exit_init;
	}
	wakeup_source_init(&wlock, "motion_ch");
	read_supported_motions_type_from_dts();
	return ret;

exit_init:
	misc_deregister(&motionhub_miscdev);
	return -1;

}

static void __exit motionhub_exit(void)
{
	misc_deregister(&motionhub_miscdev);
	motion_route_destroy();
	hwlog_info("exit %s\n", __func__);
}

late_initcall_sync(motionhub_init);
module_exit(motionhub_exit);

MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
MODULE_DESCRIPTION("MotionHub driver");
MODULE_LICENSE("GPL");
