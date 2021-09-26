/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2014-2020. All rights reserved.
 * Description: Bastet driver device.
 * Author: zhuweichen@huawei.com
 * Create: 2014-06-21
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/string.h>
#include "securec.h"
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <huawei_platform/net/bastet/bastet.h>
#include <huawei_platform/net/bastet/bastet_utils.h>
#include <huawei_platform/net/bastet/bastet_interface.h>

#define BASTET_NAME "bastet"

#define BST_FIRST_MINOR 0
#define BST_DEVICES_NUMBER 1
#define BST_MAX_REPLY_LEN 1024

/* Current Max Traffic report task number is 32 */
#define bst_traffic_len(len) ((len) > 0XFF ? 0XFF : (len))

#define BST_MAX_PROXY_NUM 32
#define APP_MAX_PID_NUM 500
#define FIXED_CTNT_MAX_LEN 1024
#ifdef CONFIG_HUAWEI_BASTET_COMM
#define BST_ACORE_CORE_MSG_TYPE_DSPP 0
#endif
#define BST_MAX_INDICATE_PACKET_LEN 5000

dev_t bastet_dev;
struct cdev bastet_cdev;
struct class *bastet_class;

struct data_packet {
	struct list_head list;
	struct bst_device_ind data;
};

struct bastet_driver_data {
	wait_queue_head_t read_wait;
	spinlock_t read_lock;
	struct list_head read_queue;
};

static struct bastet_driver_data bastet_data;

struct bst_ind_type_name {
	enum bst_ind_type type;
	const char *name;
};

static struct bst_ind_type_name ind_type_name[] = {
	{ BST_IND_INVALID, "BST_IND_INVALID" },
	{ BST_IND_HISICOM, "BST_IND_HISICOM" },
	{ BST_IND_SOCK_SYNC_REQ, "BST_IND_SOCK_SYNC_REQ" },
	{ BST_IND_SOCK_SYNC_PROP, "BST_IND_SOCK_SYNC_PROP" },
	{ BST_IND_SOCK_CLOSED, "BST_IND_SOCK_CLOSED" },
	{ BST_IND_MODEM_RESET, "BST_IND_MODEM_RESET" },
	{ BST_IND_NETFILTER_SYNC_UID, "BST_IND_NETFILTER_SYNC_UID" },
	{ BST_IND_TRAFFIC_FLOW_REQ, "BST_IND_TRAFFIC_FLOW_REQ" },
	{ BST_IND_PRIORITY_DATA, "BST_IND_PRIORITY_DATA" },
	{ BST_IND_SOCK_DISCONNECT, "BST_IND_SOCK_DISCONNECT" },
	{ BST_IND_SOCK_TIMEDOUT, "BST_IND_SOCK_TIMEDOUT" },
	{ BST_IND_SOCK_EST, "BST_IND_SOCK_EST" },
	{ BST_IND_SOCK_RENEW_CLOSE, "BST_IND_SOCK_RENEW_CLOSE" },
	{ BST_IND_SOCK_NORMAL_CLOSE, "BST_IND_SOCK_NORMAL_CLOSE" },
	{ BST_IND_SOCK_ERR_CLOSE, "BST_IND_SOCK_ERR_CLOSE" },
	{ BST_IND_PRIO_SOCK_CLOSE, "BST_IND_PRIO_SOCK_CLOSE" },
	{ BST_IND_SCREEN_STATE, "BST_IND_SCREEN_STATE" },
	{ BST_IND_UID_SOCK_PROP, "BST_IND_UID_SOCK_PROP" },
	{ BST_IND_TRIGGER_THAW, "BST_IND_TRIGGER_THAW" },
	{ BST_IND_SOCK_STATE_CHANGED, "BST_IND_SOCK_STATE_CHANGED" },
	{ BST_IND_PRIORITY_UID, "BST_IND_PRIORITY_UID" },
	{ BST_IND_FG_KEY_MSG, "BST_IND_FG_KEY_MSG" },
	{ BST_IND_FG_UID_SOCK_CHG, "BST_IND_FG_UID_SOCK_CHG" },
	{ BST_IND_HB_REPLY_RECV, "BST_IND_HB_REPLY_RECV" },
	{ BST_IND_RRC_KEEP, "BST_IND_RRC_KEEP" },
	{ BST_IND_RCVQUEUE_FULL, "BST_IND_RCVQUEUE_FULL" },
	{ BST_IND_SKSTATE_NOT_UPDATING, "BST_IND_SKSTATE_NOT_UPDATING" },
	{ BST_IND_SOCK_SYNC_FAILED, "BST_IND_SOCK_SYNC_FAILED" },
	{ BST_IND_GET_SK_FAILED, "BST_IND_GET_SK_FAILED" },
	{ BST_IND_GET_BSK_FAILED, "BST_IND_GET_BSK_FAILED" },
	{ BST_IND_PENDING_SK_SET, "BST_IND_PENDING_SK_SET" },
	{ BST_IND_NOPENDING_SK_SET, "BST_IND_NOPENDING_SK_SET" },
	{ BST_IND_SOCK_STATE_WAIT, "BST_IND_SOCK_STATE_WAIT" },
	{ BST_IND_SEND_DATA_NOTACK, "BST_IND_SEND_DATA_NOTACK" },
	{ BST_IND_RECV_DATA_INQUEUE, "BST_IND_RECV_DATA_INQUEUE" },
	{ BST_IND_SOCK_SYNC_PROP_IPV6, "BST_IND_SOCK_SYNC_PROP_IPV6" },
	{ BST_IND_SOCK_CLOSED_IPV6, "BST_IND_SOCK_CLOSED_IPV6" },
	{ BST_IND_SOCK_SYNC_REQ_IPV6, "BST_IND_SOCK_SYNC_REQ_IPV6" },
	{ BST_IND_PRIORITY_DATA_IPV6, "BST_IND_PRIORITY_DATA_IPV6" },
	{ BST_IND_SOCK_DISCONNECT_IPV6, "BST_IND_SOCK_DISCONNECT_IPV6" },
	{ BST_IND_SOCK_TIMEDOUT_IPV6, "BST_IND_SOCK_TIMEDOUT_IPV6" },
	{ BST_IND_SOCK_EST_IPV6, "BST_IND_SOCK_EST_IPV6" },
	{ BST_IND_SOCK_NORMAL_CLOSE_IPV6, "BST_IND_SOCK_NORMAL_CLOSE_IPV6" },
	{ BST_IND_SOCK_ERR_CLOSE_IPV6, "BST_IND_SOCK_ERR_CLOSE_IPV6" },
	{ BST_IND_PRIO_SOCK_CLOSE_IPV6, "BST_IND_PRIO_SOCK_CLOSE_IPV6" },
	{ BST_IND_UID_SOCK_PROP_IPV6, "BST_IND_UID_SOCK_PROP_IPV6" },
	{ BST_IND_SN_INVALID, "BST_IND_SN_INVALID" },
	{ BST_IND_RECV_DATA_INQUEUE_IPV6, "BST_IND_RECV_DATA_INQUEUE_IPV6" },
	{ BST_IND_SEND_DATA_NOTACK_IPV6, "BST_IND_SEND_DATA_NOTACK_IPV6" },
	{ BST_IND_SOCK_STATE_WAIT_IPV6, "BST_IND_SOCK_STATE_WAIT_IPV6" },
	{ BST_IND_GET_BSK_FAILED_IPV6, "BST_IND_GET_BSK_FAILED_IPV6" },
	{ BST_IND_PENDING_SK_SET_IPV6, "BST_IND_PENDING_SK_SET_IPV6" },
	{ BST_IND_SKSTATE_NOT_UPDATING_IPV6,
		"BST_IND_SKSTATE_NOT_UPDATING_IPV6" },
	{ BST_IND_SOCK_SYNC_FAILED_IPV6, "BST_IND_SOCK_SYNC_FAILED_IPV6" },
	{ BST_IND_NOPENDING_SK_SET_IPV6, "BST_IND_NOPENDING_SK_SET_IPV6" },
	{ BST_IND_RCVQUEUE_FULL_IPV6, "BST_IND_RCVQUEUE_FULL_IPV6" },
};


static int handle_bst_ioc_fg_io_ctrl(void __user *argp);

typedef int (*bst_ioc_fp)(void __user *argp);

typedef struct {
	int bst_ioc_id;
	bst_ioc_fp bst_ioc_handle;
	const char *id_name;
} bst_ioc_cmd;

/*
 * post_indicate_packet() - post_indicate_packet
 * @bst_ind_type: bastet indication type.
 * @info: the pointer to the packet
 * @len: the length of packet.
 *
 * check the bastet device status. if ok then copy
 * data to pkt, add it to the list.
 *
 * Return: 0 - indicate ok.
 *         ENIENT bastet not opened or len is too long
 *         ENOMEM kmalloc is fail.
 */
int post_indicate_packet(enum bst_ind_type type,
	const void *info, unsigned int len)
{
	struct data_packet *pkt = NULL;
	unsigned int packet_len;
	int err;

	if (!get_bastet_dev_en()) {
		bastet_loge("bastet is not opened");
		return -ENOENT;
	}
	if (len > BST_MAX_INDICATE_PACKET_LEN) {
		bastet_loge("len is too long");
		return -ENOENT;
	}
	packet_len = sizeof(*pkt) + len;
	pkt = kzalloc(packet_len, GFP_ATOMIC);
	if (pkt == NULL) {
		bastet_loge("failed to kmalloc");
		return -ENOMEM;
	}
	err = memset_s((void*)pkt, packet_len, 0, packet_len);
	if (err != EOK) {
		return err;
	}

	pkt->data.cons = 0;
	pkt->data.type = type;
	pkt->data.len = len;
	if (info != NULL) {
		err = memcpy_s((void*)pkt->data.value, len, info, len);
		if (err != EOK)
			return err;
	}

	if (type >= BST_IND_INVALID && type <= BST_IND_RCVQUEUE_FULL_IPV6)
		bastet_logi("bst_ind_type:%s", ind_type_name[type].name);

	spin_lock_bh(&bastet_data.read_lock);
	list_add_tail(&pkt->list, &bastet_data.read_queue);
	spin_unlock_bh(&bastet_data.read_lock);

	wake_up_interruptible_sync_poll(&bastet_data.read_wait,
		POLLIN | POLLRDNORM | POLLRDBAND);

	return 0;
}

static int handle_bst_ioc_fg_io_ctrl(void __user *argp)
{
	bastet_logi("BST_IOC_FAST_GRAB_INFO Entered");
	bst_fg_io_ctrl((uintptr_t)argp);
	return 0;
}

/*
 * bastet_ioctl() - bastet ioctl method.
 * @flip: device descriptor.
 * @cmd: command string.
 * @arg: command args.
 *
 * this is main method to exchange data with user space,
 * including socket sync, get ip and port, adjust kernel flow.
 *
 * return deferent return code.
 *        -EFAULT - fail
 *        other - succ.
 */
static long bastet_ioctl(struct file *flip, unsigned int cmd, unsigned long arg)
{
	int rc = -EFAULT;
	void __user *argp = (void __user *)arg;
	int nr_cmd = _IOC_NR(cmd);
	if (nr_cmd < BST_IOC_SOCK_CMD_MIN || nr_cmd > BST_IOC_SOCK_CMD_MAX) {
		bastet_loge("unknown ioctl: %u", nr_cmd);
		return rc;
	}

	if( nr_cmd == BST_IOC_FAST_GRAB_INFO_ID) {
		bastet_logi("LCF call handle_bst_ioc_fg_io_ctrl");
		rc = handle_bst_ioc_fg_io_ctrl(argp);
	} else {
		bastet_logi("bastet_ioctl nr_cmd not handle, nr_cmd = %d", nr_cmd);
	}

	return rc;
}

/* support of 32bit userspace on 64bit platforms */
#ifdef CONFIG_COMPAT
static long compat_bastet_ioctl(struct file *flip,
	unsigned int cmd, unsigned long arg)
{
	return bastet_ioctl(flip, cmd, (unsigned long) compat_ptr(arg));
}
#endif

static int bastet_open(struct inode *inode, struct file *filp)
{
	spin_lock_bh(&bastet_data.read_lock);

	if (get_bastet_dev_en()) {
		bastet_loge("bastet device has been opened");
		spin_unlock_bh(&bastet_data.read_lock);
		return -EPERM;
	}

	set_bastet_dev_en(true);

	spin_unlock_bh(&bastet_data.read_lock);
	bastet_logi("success");

	return 0;
}

static int bastet_packet_read(char __user *buf, size_t count)
{
	struct data_packet *pkt = NULL;
	struct bst_device_ind *p_bst_device_ind = NULL;
	uint8_t *data = NULL;
	bool isfree = false;
	int len = 0;
	int size = 0;

	if (buf == NULL)
		return -EINVAL;

	spin_lock_bh(&bastet_data.read_lock);
	if (list_empty(&bastet_data.read_queue)) {
		spin_unlock_bh(&bastet_data.read_lock);
		return -EAGAIN;
	}

	pkt = list_first_entry(&bastet_data.read_queue,
		struct data_packet, list);
	len = sizeof(*p_bst_device_ind) - sizeof(pkt->data.cons) +
		pkt->data.len;
	data = (uint8_t *)(&pkt->data) + sizeof(pkt->data.cons);

	if ((pkt->data.cons == 0) && (count > len)) {
		list_del(&pkt->list);
		size = len;
		isfree = true;
	} else if (((pkt->data.cons == 0) && (count <= len)) ||
		((pkt->data.cons != 0) && (pkt->data.cons + count <= len))) {
		size = count;
		isfree = false;
	} else {
		list_del(&pkt->list);
		size = len - pkt->data.cons;
		isfree = true;
	}

	spin_unlock_bh(&bastet_data.read_lock);
	if (copy_to_user(buf, data + pkt->data.cons, size)) {
		pkt->data.cons = 0;
		if (isfree)
			kfree(pkt);

		return -EFAULT;
	}
	pkt->data.cons += size;

	if (isfree)
		kfree(pkt);

	return size;
}

/*
 * bastet_read() - read the data.
 * @filp: file descriptor.
 * @buf: user space buffer.
 * @count: read numbers.
 * @ppos: read offset.
 *
 * blocked read, it will be waiting here until net device state is change.
 * standard arg is "const char __user *buf".
 *
 * Return: read the data size.
 *
 */
/*lint -e666*/
static ssize_t bastet_read(struct file *filp, char __user *buf,
	size_t count, loff_t *ppos)
{
	int ret = 0;

	spin_lock_bh(&bastet_data.read_lock);
	while (list_empty(&bastet_data.read_queue)) {
		spin_unlock_bh(&bastet_data.read_lock);
		ret = wait_event_interruptible(bastet_data.read_wait,
			!list_empty(&bastet_data.read_queue));
		if (ret)
			return ret;

		spin_lock_bh(&bastet_data.read_lock);
	}
	spin_unlock_bh(&bastet_data.read_lock);

	return bastet_packet_read(buf, count);
}
/*lint +e666*/

#ifdef CONFIG_HUAWEI_BASTET_COMM
static ssize_t bastet_write(struct file *filp,
	const char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t *msg = NULL;
	int32_t ret = count;

	msg = kzalloc(BST_MAX_WRITE_PAYLOAD, GFP_KERNEL);

	if (msg == NULL)
		return -ENOMEM;

	if ((count > BST_MAX_WRITE_PAYLOAD) || (count <= 0)) {
		bastet_loge("write length over BST_MAX_WRITE_PAYLOAD!");
		ret = -EINVAL;
		goto write_end;
	}

	if (copy_from_user(msg, buf, count)) {
		bastet_loge("copy_from_user error");
		ret = -EFAULT;
		goto write_end;
	}

write_end:
	kfree(msg);
	return ret;
}
#endif

static unsigned int bastet_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;
	poll_wait(file, &bastet_data.read_wait, wait);
	mask = !list_empty(&bastet_data.read_queue) ? (POLLIN | POLLRDNORM) : 0;

	return mask;
}

static int bastet_release(struct inode *inode, struct file *filp)
{
	struct list_head *p = NULL;
	struct list_head *n = NULL;
	struct data_packet *pkt = NULL;

	spin_lock_bh(&bastet_data.read_lock);

	if (list_empty(&bastet_data.read_queue))
		goto out_release;

	list_for_each_safe(p, n, &bastet_data.read_queue) {
		pkt = list_entry(p, struct data_packet, list);
		list_del(&pkt->list);
		kfree(pkt);
	}

out_release:
	set_bastet_dev_en(false);
	spin_unlock_bh(&bastet_data.read_lock);
	bastet_logi("success");

	return 0;
}

static const struct file_operations bastet_dev_fops = {
	.owner = THIS_MODULE,
	.open = bastet_open,
	.unlocked_ioctl = bastet_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_bastet_ioctl,
#endif
	.read = bastet_read,
#ifdef CONFIG_HUAWEI_BASTET_COMM
	.write = bastet_write,
#endif
	.poll = bastet_poll,
	.release = bastet_release,
};

static void bastet_data_init(void)
{
	spin_lock_init(&bastet_data.read_lock);
	INIT_LIST_HEAD(&bastet_data.read_queue);
	init_waitqueue_head(&bastet_data.read_wait);
}

static int bastet_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = NULL;

	bastet_data_init();
	bst_fg_init();

	ret = alloc_chrdev_region(&bastet_dev,
		BST_FIRST_MINOR, BST_DEVICES_NUMBER, BASTET_NAME);
	if (ret) {
		bastet_loge("alloc_chrdev_region error");
		goto fail_region;
	}

	cdev_init(&bastet_cdev, &bastet_dev_fops);
	bastet_cdev.owner = THIS_MODULE;

	ret = cdev_add(&bastet_cdev, bastet_dev, BST_DEVICES_NUMBER);
	if (ret) {
		bastet_loge("cdev_add error");
		goto fail_cdev_add;
	}

	bastet_class = class_create(THIS_MODULE, BASTET_NAME);
	if (IS_ERR(bastet_class)) {
		bastet_loge("class_create error");
		goto fail_class_create;
	}

	dev = device_create(bastet_class, NULL, bastet_dev, NULL, BASTET_NAME);
	if (IS_ERR(dev)) {
		bastet_loge("device_create error");
		goto fail_device_create;
	}

	return 0;

fail_device_create:
	class_destroy(bastet_class);
fail_class_create:
	cdev_del(&bastet_cdev);
fail_cdev_add:
	unregister_chrdev_region(bastet_dev, BST_DEVICES_NUMBER);
fail_region:
	bastet_loge("device_create SUCCESS, ret = %d \n", ret);
	return ret;
}

static int bastet_remove(struct platform_device *pdev)
{
	if (bastet_class != NULL) {
		device_destroy(bastet_class, bastet_dev);
		class_destroy(bastet_class);
	}
	cdev_del(&bastet_cdev);
	unregister_chrdev_region(bastet_dev, BST_DEVICES_NUMBER);
	bastet_utils_exit();

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id of_bastet_match_tbl[] = {
	{
		.compatible = "huawei,bastet",
	},
	{} // end
};

MODULE_DEVICE_TABLE(of, of_bastet_match_tbl);
#endif

static struct platform_driver bastet_driver = {
	.probe = bastet_probe,
	.remove = bastet_remove,
	.driver = {
		.name = "bastet",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(of_bastet_match_tbl),
#endif
	},
};

module_platform_driver(bastet_driver);

MODULE_AUTHOR("zhuxiaolong@huawei.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Bastet driver");
