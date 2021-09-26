 /*
 * hwcam_cfgdev.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * hwcam cfgdev src file
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

#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/freezer.h>
#include <linux/pid.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <media/media-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <securec.h>

#include "hwcam_intf.h"
#include "hwcam_compat32.h"
#include "hwcam_log.h"

#define HWCAM_MODEL_CFG "hwcam_cfgdev"

typedef struct _tag_hwcam_cfgdev_vo {
	struct v4l2_device v4l2;
	struct video_device *vdev;
	struct dentry *debug_root;
	struct v4l2_fh rq;
	__u8 sbuf[64]; /* same with v4l2_event data size, 64 bytes */
	struct mutex lock;
} hwcam_cfgdev_vo_t;

typedef enum _tag_hwcam_cfgsvr_flags {
	HWCAM_CFGSVR_FLAG_UNPLAYED = 0,
	HWCAM_CFGSVR_FLAG_PLAYING = 1,
} hwcam_cfgsvr_flags_t;

static DEFINE_MUTEX(s_cfgdev_lock);
static hwcam_cfgdev_vo_t s_cfgdev;

static DEFINE_MUTEX(s_cfgsvr_lock);
static DECLARE_WAIT_QUEUE_HEAD(s_wait_cfgsvr);
static struct pid *s_pid_cfgsvr;

static DEFINE_SPINLOCK(s_ack_queue_lock);
static atomic_t s_sequence = ATOMIC_INIT(0);
static hwcam_cfgsvr_flags_t s_cfgsvr_flags = HWCAM_CFGSVR_FLAG_UNPLAYED;
static struct list_head s_ack_queue = LIST_HEAD_INIT(s_ack_queue);
static DECLARE_WAIT_QUEUE_HEAD(s_wait_ack);

static ssize_t guard_thermal_show(struct device_driver *drv, char *buf);
static ssize_t guard_thermal_store(struct device_driver *drv, const char *buf, size_t count);
static DRIVER_ATTR_RW(guard_thermal);

int hwcam_cfgdev_send_req(hwcam_user_intf_t *user, struct v4l2_event *ev,
	struct v4l2_fh *target, int one_way, int *ret);

static int hwcam_cfgdev_guard_thermal(void)
{
	struct v4l2_event ev = {
		.type = HWCAM_V4L2_EVENT_TYPE,
		.id = HWCAM_CFGDEV_REQUEST,
	};
	hwcam_cfgreq2dev_t *req = (hwcam_cfgreq2dev_t *)ev.u.data;

	req->req.intf = NULL;
	req->kind = HWCAM_CFGDEV_REQ_GUARD_THERMAL;
	return hwcam_cfgdev_send_req(NULL, &ev, &s_cfgdev.rq, 1, NULL);
}

static ssize_t guard_thermal_store(struct device_driver *drv, const char *buf, size_t count)
{
	int rc;
	errno_t ret;

	(void)drv;
	if (!buf || count <= 1) { /* buffer count must > 1 */
		cam_warn("%s: buf or count is invalid, count = %zu", __func__, count);
		return count;
	}

	ret = memset_s(s_cfgdev.sbuf, sizeof(s_cfgdev.sbuf), 0, sizeof(s_cfgdev.sbuf));
	if (ret != EOK) {
		cam_err("%s memset_s fail, ret = %d", __func__, ret);
		return count;
	}

	if (count < sizeof(s_cfgdev.sbuf)) {
		ret = memcpy_s(s_cfgdev.sbuf, sizeof(s_cfgdev.sbuf) - 1, buf, count);
		if (ret != EOK) {
			cam_err("%s memcpy fail, ret = %d", __func__, ret);
			return count;
		}
	} else {
		cam_warn("%s count[%zu] is beyond sbuf size[%zu]",
			__func__, count, sizeof(s_cfgdev.sbuf));

		ret = memcpy_s(s_cfgdev.sbuf, sizeof(s_cfgdev.sbuf) - 1,
			buf, sizeof(s_cfgdev.sbuf) - 1);
		if (ret != EOK) {
			cam_err("%s memcpy fail, ret = %d", __func__, ret);
			return count;
		}
	}

	rc = hwcam_cfgdev_guard_thermal();
	cam_info("%s cfgdev guard thermal finished, rc is %d", __func__, rc);

	return count;
}

static ssize_t guard_thermal_show(struct device_driver *drv, char *buf)
{
	(void)drv;
	if (!buf) {
		cam_err("%s: buf is null", __func__);
		return 0;
	}

	cam_info("%s enter", __func__);
	return snprintf_s(buf, PAGE_SIZE, PAGE_SIZE - 1, "guard thermal:[%s]\n", s_cfgdev.sbuf);
}

static void hwcam_cfgdev_release_ack(hwcam_cfgack_t *ack)
{
	kzfree(ack);
}

int hwcam_cfgdev_queue_ack(struct v4l2_event *ev)
{
	hwcam_cfgack_t *ack = NULL;
	hwcam_cfgreq_t *req = NULL;

	cam_info("%s enter", __func__);
	if (!ev) {
		cam_err("%s v4l2_event is null", __func__);
		return -EINVAL;
	}

	req = (hwcam_cfgreq_t *)ev->u.data;
	if (req && req->one_way != 0) {
		cam_err("%s need NOT acknowledge an one way "
			"request(0x%pK, 0x%08x, %u)",
			__func__, req->intf,
			(*(unsigned *)(req + 1)), req->seq);
		return -EINVAL;
	}

	ack = kzalloc(sizeof(hwcam_cfgack_t), GFP_KERNEL);
	if (!ack) {
		cam_err("%s out of memory for ack!", __func__);
		return -ENOMEM;
	}
	ack->ev = *ev;
	ack->release = hwcam_cfgdev_release_ack;

	spin_lock(&s_ack_queue_lock);
	list_add_tail(&ack->node, &s_ack_queue);
	wake_up_all(&s_wait_ack);
	spin_unlock(&s_ack_queue_lock);

	return 0;
}

static bool hwcam_cfgdev_check_ack(
	hwcam_cfgreq_t *req,
	hwcam_cfgack_t **ppack)
{
	bool ret = false;
	hwcam_cfgack_t *ack = NULL;
	hwcam_cfgack_t *tmp = NULL;
	hwcam_cfgreq_t *back = NULL;

	spin_lock(&s_ack_queue_lock);
	ret = s_cfgsvr_flags == HWCAM_CFGSVR_FLAG_UNPLAYED;
	list_for_each_entry_safe(ack, tmp, &s_ack_queue, node) {
		back = (hwcam_cfgreq_t*)ack->ev.u.data;
		if (req->user == back->user &&
			req->intf == back->intf &&
			req->seq == back->seq) {
			ret = true;
			*ppack = ack;
			list_del(&ack->node);
			break;
		}
	}
	spin_unlock(&s_ack_queue_lock);
	return ret;
}

static void hwcam_cfgdev_flush_ack_queue(void)
{
	hwcam_cfgack_t *ack = NULL;
	hwcam_cfgack_t *tmp = NULL;

	spin_lock(&s_ack_queue_lock);
	s_cfgsvr_flags = HWCAM_CFGSVR_FLAG_UNPLAYED;
	list_for_each_entry_safe(ack, tmp, &s_ack_queue, node) {
		list_del(&ack->node);
		ack->release(ack);
	}
	wake_up_all(&s_wait_ack);
	spin_unlock(&s_ack_queue_lock);
}

enum {
	HWCAM_WAIT4ACK_TIME = 10000, /* 10s */
	HWCAM_WAIT4CFGSVR_TIME = 3000, /* 3s */
};

static int hwcam_cfgdev_ack_deal(hwcam_user_intf_t *user,
	hwcam_cfgreq_t *req, int timeout, hwcam_cfgack_t **pack)
{
	int rc = -ETIME;
	int retry = 3;
	hwcam_cfgack_t *ack = NULL;

	while (true) {
		if (user)
			hwcam_user_intf_wait_begin(user);
		rc = wait_event_freezable_timeout(s_wait_ack,
			hwcam_cfgdev_check_ack(req, &ack), timeout);
		if (user)
			hwcam_user_intf_wait_end(user);

		if (ack) {
			rc = 0;
			break;
		}

		if (rc == 0) {
			cam_err("request(0x%pK, 0x%08x, %u) is out of time for ACK!",
				req->intf, (*(unsigned*)(req + 1)), req->seq);
			rc = -ETIME;
			break;
		}

		if (rc == -ERESTARTSYS) {
			cam_info("request(0x%pK, 0x%08x, %u) is interrupted! pid: %d, tgid: %d, pending.signal: 0x%lx",
				req->intf, (*(unsigned*)(req + 1)), req->seq,
				current->pid, current->tgid, current->pending.signal.sig[0]);
			if (retry > 0) {
				retry--;
				continue;
			} else {
				rc = -ERESTART;
				break;
			}
		}
	}
	*pack = ack;

	return rc;
}

static int hwcam_cfgdev_wait_ack(hwcam_user_intf_t *user,
	hwcam_cfgreq_t *req, int timeout, int *ret)
{
	int rc;
	hwcam_cfgack_t *ack = NULL;
	timeout = msecs_to_jiffies(timeout);

	if (ret)
		*ret = -EINVAL;

	rc = hwcam_cfgdev_ack_deal(user, req, timeout, &ack);

	if (req->intf) {
		if (ack)
			hwcam_cfgreq_intf_on_ack(req->intf, ack);
		else
			hwcam_cfgreq_intf_on_cancel(req->intf, rc);
	}

	if (ack) {
		if (ret)
			*ret = hwcam_cfgack_result(ack);
		ack->release(ack);
	}

	return rc;
}

static int hwcam_cfgdev_thermal_guard(struct v4l2_event *ev)
{
	char *buf = NULL;
	errno_t ret;

	if (!ev) {
		cam_err("%s: v4l2 event is null", __func__);
		return 0;
	}

	buf = (char *)ev->u.data;
	if (buf) {
		ret = memcpy_s(buf, sizeof(s_cfgdev.sbuf), s_cfgdev.sbuf, sizeof(s_cfgdev.sbuf));
		if (ret != EOK)
			cam_err("%s memcpy_s fail", __func__);
	}

	return 0;
}

int hwcam_cfgdev_send_req(hwcam_user_intf_t *user,
	struct v4l2_event *ev, struct v4l2_fh *target,
	int one_way, int *ret)
{
	int rc = 0;
	hwcam_cfgreq_t *req = NULL;

	if (!ev) {
		cam_err("%s v4l2 event is null", __func__);
		return 0;
	}

	req = (hwcam_cfgreq_t*)ev->u.data;
	if (!req) {
		cam_err("%s req is null", __func__);
		return 0;
	}

	req->user = user;
	req->seq = atomic_add_return(1, &s_sequence);
	req->one_way = one_way ? 1 : 0;
	mutex_lock(&s_cfgdev_lock);
	if (target->vdev) {
		v4l2_event_queue_fh(target, ev);
		if (req->intf)
			hwcam_cfgreq_intf_get(req->intf);
	} else {
		cam_err("%s: the target vdev is invalid", __func__);
		rc = -ENOENT;
	}
	mutex_unlock(&s_cfgdev_lock);

	if (ret)
		*ret = rc;

	if (rc == 0 && req->one_way == 0)
		rc = hwcam_cfgdev_wait_ack(
			user, req, HWCAM_WAIT4ACK_TIME, ret);

	return rc;
}

char* gen_media_prefix(char *media_ent, hwcam_device_id_constants_t dev_const, size_t dst_size)
{
	if (!media_ent) {
		cam_err("%s media_ent is null", __func__);
		return media_ent;
	}

	if (dst_size < 1) {
		cam_err("%s dst_size %zu invalid", __func__, dst_size);
		return media_ent;
	}

	if (snprintf_s(media_ent, dst_size, dst_size - 1, "%d", dev_const) < 0)
		cam_err("%s: snprintf_s media_ent failed", __func__);

	strlcat(media_ent, "-" , dst_size);

	return media_ent;
}

static void hwcam_cfgdev_subdev_release(struct video_device *vdev)
{
	struct v4l2_subdev *sd = video_get_drvdata(vdev);
	if (!sd)
		return;
	sd->devnode = NULL;
	video_device_release(vdev);

	cam_info("%s: TODO", __func__);
}

static unsigned int hwcam_subdev_poll(struct file *file, poll_table *wait)
{
	struct video_device *vdev = NULL;
	struct v4l2_subdev *sd = NULL;
	struct v4l2_fh *fh = NULL;

	if (!file)
		return POLLERR;

	vdev = video_devdata(file);
	if (!vdev)
		return POLLERR;

	sd = vdev_to_v4l2_subdev(vdev);
	if (!sd)
		return POLLERR;

	fh = file->private_data;
	if (!fh || (sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS) == 0)
		return POLLERR;

	poll_wait(file, &fh->wait, wait);

	if (v4l2_event_pending(fh) != 0)
		return POLLIN;

	return 0;
}

static struct v4l2_file_operations hwcam_v4l2_subdev_fops = {};

static void gen_vname_for_mprefix(struct video_device *vdev,
	const char *media_prefix, const char *cfgdev_name)
{
	int rc;

	rc = snprintf_s(vdev->name, sizeof(vdev->name),
		sizeof(vdev->name) - 1, "%s", media_prefix);
	if (rc < 0)
		cam_err("%s snprintf_s video device name failed", __func__);

	(void)strlcpy(vdev->name + strlen(vdev->name),
		cfgdev_name, sizeof(vdev->name) - strlen(vdev->name));
}

static void gen_vdentity_name_for_mprefix(struct video_device *vdev,
	const char *media_prefix)
{
	int rc;

	rc = snprintf_s(vdev->name + strlen(vdev->name),
		sizeof(vdev->name) - strlen(vdev->name),
		sizeof(vdev->name) - strlen(vdev->name) - 1,
		"%s", video_device_node_name(vdev));
	if (rc < 0) {
		cam_err("%s: Truncation Occurred", __func__);
		(void)snprintf_s(vdev->name, sizeof(vdev->name),
			sizeof(vdev->name) - 1, "%s", media_prefix);
		(void)snprintf_s(vdev->name + strlen(vdev->name),
			sizeof(vdev->name) - strlen(vdev->name),
			sizeof(vdev->name) - strlen(vdev->name) - 1,
			"%s", video_device_node_name(vdev));
	}
}

int hwcam_cfgdev_register_subdev(
	struct v4l2_subdev* sd, hwcam_device_id_constants_t dev_const)
{
	int rc = 0;
	char media_prefix[10] = {0};
	struct video_device* vdev = NULL;
	struct v4l2_device* v4l2 = &s_cfgdev.v4l2;

	if (!sd || !sd->name[0]) {
		rc = -EINVAL;
		goto out;
	}

	rc = v4l2_device_register_subdev(v4l2, sd);
	if (rc < 0) {
		cam_err("%s register subdev failed", __func__);
		goto out;
	}

	if ((sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE) == 0) {
		cam_warn("%s suvdev devnode is not using", __func__);
		goto out;
	}

	vdev = video_device_alloc();
	if (!vdev) {
		cam_err("%s: memory alloc falied for video device", __func__);
		rc = -ENOMEM;
		goto video_alloc_fail;
	}

	video_set_drvdata(vdev, sd);
	gen_media_prefix(media_prefix, dev_const, sizeof(media_prefix));
	gen_vname_for_mprefix(vdev, media_prefix, sd->name);
	vdev->v4l2_dev = v4l2;
	vdev->fops = &hwcam_v4l2_subdev_fops;
	vdev->release = hwcam_cfgdev_subdev_release;
	rc = __video_register_device(vdev, VFL_TYPE_SUBDEV, -1, 1, sd->owner);
	if (rc < 0) {
		cam_err("%s video register device failed", __func__);
		goto video_register_fail;
	}
	cam_info("register video devices %s sucessful", sd->name);
	cam_info("video dev name %s %s", vdev->dev.kobj.name, vdev->name);
	gen_vdentity_name_for_mprefix(vdev, media_prefix);

	rc = 0;
	sd->devnode = vdev;
	goto out;

video_register_fail:
	video_device_release(vdev);
video_alloc_fail:
	if (sd->devnode)
		v4l2_device_unregister_subdev(sd);
out:
	return rc;
}

int hwcam_cfgdev_unregister_subdev(struct v4l2_subdev *sd)
{
	if (!sd)
		return -EINVAL;
	v4l2_device_unregister_subdev(sd);
	return 0;
}

static unsigned int hwcam_cfgdev_vo_poll(
	struct file *filep, struct poll_table_struct *ptbl)
{
	unsigned int rc = 0;
	mutex_lock(&s_cfgdev.lock);
	poll_wait(filep, &s_cfgdev.rq.wait, ptbl);
	if (v4l2_event_pending(&s_cfgdev.rq) != 0)
		rc = POLLIN | POLLRDNORM;
	mutex_unlock(&s_cfgdev.lock);
	return rc;
}

static long hwcam_cfgdev_vo_dqevent(
	struct v4l2_event *ev, int nb)
{
	long rc = 0;
	hwcam_cfgreq_t *req = NULL;

	if (!ev) {
		cam_err("%s: v4l2 event is null", __func__);
		return 0;
	}

	while (true) {
		rc = v4l2_event_dequeue(&s_cfgdev.rq, ev, nb);
		if (rc != 0)
			break;
		req = (hwcam_cfgreq_t*)ev->u.data;
		if (req->intf) {
			rc = hwcam_cfgreq_intf_on_req(req->intf, ev);
			hwcam_cfgreq_intf_put(req->intf);
			if (rc != 0) {
				cam_err("request(0x%pK, 0x%08x, %u) skipped",
					req->intf, (*(unsigned*)(req + 1)), req->seq);
				continue; /* to skip failed/timeouted/interrupted events */
			}
		}
		break;
	}
	return rc;
}

static void hwcam_cfgdev_subscribed_event_ops_merge(
	const struct v4l2_event *old, struct v4l2_event *new)
{
	hwcam_cfgreq2dev_t *req = NULL;

	if (!old) {
		cam_err("%s: v4l2 event is null", __func__);
		return;
	}
	(void)new;
	req = (hwcam_cfgreq2dev_t *)&old->u.data;
	if (req && req->req.intf)
		hwcam_cfgreq_intf_put(req->req.intf);
	cam_err("%s: the event queue overflowed", __func__);
}

static struct v4l2_subscribed_event_ops
s_hwcam_subscribed_event_ops = {
	.merge = hwcam_cfgdev_subscribed_event_ops_merge,
};

static long hwcam_cfgdev_vo_do_ioctl(
	struct file *filep,
	unsigned int cmd,
	void *arg)
{
	long rc = -EINVAL;

	if (!filep || !arg) {
		cam_err("%s filep or arg is invalid", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case VIDIOC_DQEVENT:
		rc = hwcam_cfgdev_vo_dqevent(arg,
			filep->f_flags & O_NONBLOCK);
		break;

	case VIDIOC_SUBSCRIBE_EVENT:
		rc = v4l2_event_subscribe(&s_cfgdev.rq,
			(struct v4l2_event_subscription *)arg,
			8, /* 8, arg len */
			&s_hwcam_subscribed_event_ops);
		break;

	case VIDIOC_UNSUBSCRIBE_EVENT:
		rc = v4l2_event_unsubscribe(&s_cfgdev.rq,
			(struct v4l2_event_subscription *)arg);
		break;

	case HWCAM_V4L2_IOCTL_REQUEST_ACK:
		rc = hwcam_cfgdev_queue_ack((struct v4l2_event *)arg);
		break;

	case HWCAM_V4L2_IOCTL_THERMAL_GUARD:
		rc = hwcam_cfgdev_thermal_guard((struct v4l2_event *)arg);
		break;

	default:
		cam_err("%s: invalid IOCTL CMD %d", __func__, cmd);
		break;
	}
	return rc;
}

static long hwcam_cfgdev_vo_ioctl(
	struct file *filep,
	unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(filep, cmd, arg, hwcam_cfgdev_vo_do_ioctl);
}

#ifdef CONFIG_COMPAT
static long hwcam_cfgdev_vo_ioctl32(
	struct file *filep,
	unsigned int cmd,
	unsigned long arg)
{
	long rc = 0;
	void __user *up_p = NULL;
	void __user *kp = NULL;
	up_p = compat_ptr(arg);

	switch (cmd) {
	case HWCAM_V4L2_IOCTL_REQUEST_ACK32:
		cmd = HWCAM_V4L2_IOCTL_REQUEST_ACK;
		break;
	case HWCAM_V4L2_IOCTL_THERMAL_GUARD32:
		cmd = HWCAM_V4L2_IOCTL_THERMAL_GUARD;
		break;
	default:
		cam_err("%s: case err cmd %u", __func__, cmd);
		break;
	}

	switch (cmd) {
	case HWCAM_V4L2_IOCTL_REQUEST_ACK:
	case HWCAM_V4L2_IOCTL_THERMAL_GUARD:
		kp = compat_alloc_user_space(sizeof(struct v4l2_event));
		if (!kp)
			return -EFAULT;
		rc = compat_get_v4l2_event_data(kp, up_p);
		if (rc != 0)
			return rc;
		rc = hwcam_cfgdev_vo_ioctl(filep, cmd,
			(unsigned long)(uintptr_t)(kp));
		if (rc != 0)
			return rc;
		rc = compat_put_v4l2_event_data(kp, up_p);
		break;
	default:
		rc = hwcam_cfgdev_vo_ioctl(filep, cmd, arg);
		break;
	}
	return rc;
}
#endif

static int hwcam_cfgdev_vo_close(struct file *filep)
{
	void *fpd = NULL;
	struct pid *pid = NULL;

	mutex_lock(&s_cfgdev.lock);
	if (filep)
		swap(filep->private_data, fpd);
	mutex_unlock(&s_cfgdev.lock);
	if (!fpd)
		return 0;

	mutex_lock(&s_cfgsvr_lock);

	swap(s_pid_cfgsvr, pid);
	mutex_lock(&s_cfgdev_lock);
	v4l2_fh_del(&s_cfgdev.rq);
	v4l2_fh_exit(&s_cfgdev.rq);
	mutex_unlock(&s_cfgdev_lock);

	if (pid)
		put_pid(pid);

	mutex_unlock(&s_cfgsvr_lock);
	hwcam_cfgdev_flush_ack_queue();
	cam_info("the server %d detached", current->pid);

	return 0;
}

static int hwcam_cfgdev_vo_open(struct file *filep)
{
	int rc = 0;

	mutex_lock(&s_cfgsvr_lock);
	if (s_pid_cfgsvr) {
		mutex_unlock(&s_cfgsvr_lock);
		cam_info("%s: only one server can attach to cfgdev", __func__);
		rc = -EBUSY;
		goto exit_open;
	}
	s_pid_cfgsvr = get_pid(task_pid(current));

	mutex_lock(&s_cfgdev_lock);
	v4l2_fh_init(&s_cfgdev.rq, s_cfgdev.vdev);
	v4l2_fh_add(&s_cfgdev.rq);
	mutex_unlock(&s_cfgdev_lock);
	mutex_unlock(&s_cfgsvr_lock);

	spin_lock(&s_ack_queue_lock);
	s_cfgsvr_flags = HWCAM_CFGSVR_FLAG_PLAYING;
	spin_unlock(&s_ack_queue_lock);

	if (filep)
		filep->private_data = &s_cfgdev;
	wake_up_all(&s_wait_cfgsvr);

	cam_info("the server %d attached", current->pid);
exit_open:
	return rc;
}

static int hwcam_cfgdev_get_dts(struct platform_device* dev)
{
	if (!dev) {
		cam_err("%s dev NULL", __func__);
		return -ENOMEM;
	}

	/* add dts analyze here */
	return 0;
}

static struct v4l2_file_operations s_fops_cfgdev = {
	.owner = THIS_MODULE,
	.open = hwcam_cfgdev_vo_open,
	.poll = hwcam_cfgdev_vo_poll,
	.unlocked_ioctl = hwcam_cfgdev_vo_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = hwcam_cfgdev_vo_ioctl32,
#endif
	.release = hwcam_cfgdev_vo_close,
};

static void hwcam_cfgdev_vo_subdev_notify(
	struct v4l2_subdev *sd,
	unsigned int notification,
	void *arg)
{
	(void)sd;
	(void)notification;
	(void)arg;
	cam_info("%s TODO", __func__);
}

static int video_device_init(struct video_device *vdev,
	struct v4l2_device *v4l2)
{
	int rc;
	char media_prefix[10] = {0}; /* 10, array len */

	vdev->v4l2_dev = v4l2;
	(void)gen_media_prefix(media_prefix, HWCAM_VNODE_GROUP_ID,
		sizeof(media_prefix));

	gen_vname_for_mprefix(vdev, media_prefix, "hwcam-cfgdev");
	vdev->release = video_device_release_empty;
	vdev->fops = &s_fops_cfgdev;
	vdev->minor = -1;
	vdev->vfl_type = VFL_TYPE_GRABBER;
	rc = video_register_device(vdev, VFL_TYPE_GRABBER, 100); /* /dev/video100 */
	if (rc < 0) {
		cam_err("%s video register device failed", __func__);
		return rc;
	}

	cam_info("%s video dev name %s %s",
		__func__, vdev->dev.kobj.name, vdev->name);

	gen_vdentity_name_for_mprefix(vdev, media_prefix);
	vdev->lock = &s_cfgdev_lock;

	return 0;
}

static int hwcam_cfgdev_vo_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct video_device *vdev = NULL;
	struct v4l2_device *v4l2 = &s_cfgdev.v4l2;

	vdev = video_device_alloc();
	if (!vdev) {
		cam_err("%s video device alloc failed", __func__);
		rc = -ENOMEM;
		goto probe_end;
	}

	rc = hwcam_cfgdev_get_dts(pdev);
	if (rc < 0)
		cam_warn("%s get dts failed", __func__);

	v4l2->notify = hwcam_cfgdev_vo_subdev_notify;
	rc = v4l2_device_register(&(pdev->dev), v4l2);
	if (rc < 0) {
		cam_err("%s v4l2 device register failed", __func__);
		goto v4l2_register_fail;
	}

	rc = video_device_init(vdev, v4l2);
	if (rc < 0) {
		cam_err("%s video device init failed", __func__);
		goto video_register_fail;
	}

	video_set_drvdata(vdev, &s_cfgdev);
	s_cfgdev.vdev = vdev;
	s_cfgdev.debug_root = debugfs_create_dir("hwcam", NULL);
	mutex_init(&s_cfgdev.lock);

	goto probe_end;

video_register_fail:
	v4l2_device_unregister(v4l2);
v4l2_register_fail:
	video_device_release(vdev);
probe_end:
	cam_info("%s exit", __func__);
	return rc;
}

static int hwcam_cfgdev_vo_remove(struct platform_device *pdev)
{
	(void)pdev;
	video_unregister_device(s_cfgdev.vdev);
	v4l2_device_unregister(&s_cfgdev.v4l2);
	video_device_release(s_cfgdev.vdev);
	s_cfgdev.vdev = NULL;
	mutex_destroy(&s_cfgdev.lock);
	return 0;
}

static const struct of_device_id s_cfgdev_devtbl_match[] = {
	{ .compatible = "camcfgdev" },
	{},
}

MODULE_DEVICE_TABLE(of, s_cfgdev_devtbl_match);

static struct platform_driver s_cfgdev_driver = {
	.probe = hwcam_cfgdev_vo_probe,
	.remove = hwcam_cfgdev_vo_remove,
	.driver = {
		.name = "camcfgdev",
		.owner = THIS_MODULE,
		.of_match_table = s_cfgdev_devtbl_match,
	},
};

static int __init hwcam_cfgdev_vo_init(void)
{
	int ret;
	hwcam_v4l2_subdev_fops = v4l2_subdev_fops;
	hwcam_v4l2_subdev_fops.poll = hwcam_subdev_poll;
	ret = platform_driver_register(&s_cfgdev_driver);
	if (ret != 0) {
		cam_err("%s platform driver register failed", __func__);
		return ret;
	}

	if (driver_create_file(&s_cfgdev_driver.driver, &driver_attr_guard_thermal))
		cam_warn("%s create driver attr failed", __func__);

	return ret;
}

static void __exit hwcam_cfgdev_vo_exit(void)
{
	platform_driver_unregister(&s_cfgdev_driver);
}

module_init(hwcam_cfgdev_vo_init);
module_exit(hwcam_cfgdev_vo_exit);
MODULE_DESCRIPTION("Huawei V4L2 Camera");
MODULE_LICENSE("GPL v2");
