/*
 * chr_devs.c
 *
 * chr device node implementation
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define CHR_DEBUG  1
#include "chr_devs.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <stdarg.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/un.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#ifdef CONFIG_LOG_EXCEPTION
#include <log/log_usertype.h>
#endif

static int32_t chr_misc_open(struct inode *fd, struct file *fp);
static ssize_t chr_misc_read(struct file *fp, char __user *buff, size_t count, loff_t *loff);
static long chr_misc_ioctl(struct file *fp, uint32_t cmd, uintptr_t arg);
static int32_t chr_misc_release(struct inode *fd, struct file *fp);

static chr_event g_chr_event;
static int32_t g_chr_enable = CHR_LOG_DISABLE;

static const struct file_operations chr_misc_fops = {
    .owner = THIS_MODULE,
    .open = chr_misc_open,
    .read = chr_misc_read,
    .release = chr_misc_release,
    .unlocked_ioctl = chr_misc_ioctl,
};

static struct miscdevice chr_misc_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = CHR_DEV_KMSG_PLAT,
    .fops = &chr_misc_fops,
};

/*
 * 函 数 名  : chr_misc_open
 * 功能描述  : 打开设备节点接口
 */
static int32_t chr_misc_open(struct inode *fd, struct file *fp)
{
    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_err("chr %s open fail, module is disable\n", chr_misc_dev.name);
        return -EBUSY;
    }
    chr_dbg("chr %s open success\n", chr_misc_dev.name);
    return CHR_SUCC;
}

/*
 * 函 数 名  : chr_misc_read
 * 功能描述  : 读取设备节点接口
 */
static ssize_t chr_misc_read(struct file *fp, char __user *buff, size_t count, loff_t *loff)
{
    int32_t ret;
    uint32_t __user *puser = (uint32_t __user *)buff;
    struct sk_buff *skb = NULL;
    u_int16_t data_len = 0;

    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_err("chr %s read fail, module is disable\n", chr_misc_dev.name);
        return -EBUSY;
    }

    if (count < sizeof(chr_dev_exception_stru_para)) {
        chr_err("The user space buff is too small\n");
        return -CHR_EFAIL;
    }

    if (buff == NULL) {
        chr_err("chr %s read fail, user buff is NULL", chr_misc_dev.name);
        return -EAGAIN;
    }
    skb = skb_dequeue(&g_chr_event.errno_queue);
    if (skb == NULL) {
        if (fp->f_flags & O_NONBLOCK) {
            chr_dbg("Thread read chr with NONBLOCK mode\n");
            return 0; /* for no data with O_NONBOCK mode return 0 */
        } else {
            if (wait_event_interruptible(g_chr_event.errno_wait,
                                         (skb = skb_dequeue(&g_chr_event.errno_queue)) != NULL)) {
                if (skb != NULL) {
                    skb_queue_head(&g_chr_event.errno_queue, skb);
                }
                chr_warning("Thread interrupt with signal\n");
                return -ERESTARTSYS;
            }
        }
    }

    /* event_id(4 bytes) + len(2 bytes) + type(1 byte) + resv(1 byte) + real_errid(16 bytes) = 24 bytes */
    data_len = min_t(size_t, skb->len, count);
    ret = copy_to_user(puser, skb->data, data_len);

    if (ret) {
        chr_warning("copy_to_user err!restore it, len=%d\n", data_len);
        skb_queue_head(&g_chr_event.errno_queue, skb);
        return -EFAULT;
    }

    /* have read count1 byte */
    skb_pull(skb, data_len);

    if (skb->len == 0) { /* if skb->len = 0: read is over, curr skb data have read to user */
        kfree_skb(skb);
    } else { /* if don't read over; restore to skb queue */
        skb_queue_head(&g_chr_event.errno_queue, skb);
    }

    return data_len;
}

/*
 * 函 数 名  : chr_write_errno_to_queue
 * 功能描述  : 将异常码写入队列
 */
static int32_t chr_write_errno_to_queue(uint32_t ul_errno, u_int16_t us_flag, uint8_t *ptr_data, u_int16_t ul_len)
{
    struct sk_buff *skb = NULL;
    u_int16_t sk_len;

    if (skb_queue_len(&g_chr_event.errno_queue) > CHR_ERRNO_QUEUE_MAX_LEN) {
        chr_warning("chr errno queue is full, dispose errno=%x\n", ul_errno);
        return CHR_SUCC;
    }

    /* for code run in interrupt context */
    sk_len = sizeof(chr_dev_exception_stru_para) + ul_len;
    if (in_interrupt() || in_atomic() || irqs_disabled()) {
        skb = alloc_skb(sk_len, GFP_ATOMIC);
    } else {
        skb = alloc_skb(sk_len, GFP_KERNEL);
    }
    if (skb == NULL) {
        chr_err("chr errno alloc skbuff failed! len=%d, errno=%x\n", sk_len, ul_errno);
        return -ENOMEM;
    }

    skb_put(skb, sk_len);
    *(uint32_t *)skb->data = ul_errno;
    *((u_int16_t *)(skb->data + 4)) = ul_len;  /* 偏移存放errno的前4个字节 */
    *((u_int16_t *)(skb->data + 6)) = us_flag; /* 偏移存放errno加长度的前6个字节 */

    if ((ul_len > 0) && (ptr_data != NULL)) {
        memcpy(((uint8_t *)skb->data + sizeof(chr_dev_exception_stru_para)), ptr_data, ul_len);
    }

    skb_queue_tail(&g_chr_event.errno_queue, skb);
    wake_up_interruptible(&g_chr_event.errno_wait);
    return CHR_SUCC;
}

static void *oal_memalloc(uint32_t ul_size)
{
    int32_t l_flags = GFP_KERNEL;
    void *puc_mem_space = NULL;

    if (in_interrupt() || irqs_disabled() || in_atomic()) {
        l_flags = GFP_ATOMIC;
    }

    if (unlikely(ul_size == 0)) {
        return NULL;
    }

    puc_mem_space = kmalloc(ul_size, l_flags);
    if (puc_mem_space == NULL) {
       return NULL;
    }

    return puc_mem_space;
}

static void oal_free(void *p_buf)
{
    kfree(p_buf);
}

static int64_t chr_misc_errno_write(uint32_t __user *puser)
{
    uint32_t ret;
    uint8_t *pst_mem = NULL;
    chr_host_exception_stru chr_rx_data;

    ret = copy_from_user(&chr_rx_data, puser, sizeof(chr_host_exception_stru));
    if (ret) {
        chr_err("chr %s ioctl fail, get data from user fail", chr_misc_dev.name);
        return -EINVAL;
    }

    if (chr_rx_data.chr_len == 0) {
        chr_write_errno_to_queue(chr_rx_data.chr_errno, CHR_HOST, NULL, 0);
    } else {
        pst_mem = oal_memalloc(chr_rx_data.chr_len);
        if (pst_mem == NULL) {
            chr_err("chr mem alloc failed len %u\n", chr_rx_data.chr_len);
            return -EINVAL;
        }

        if (chr_rx_data.chr_ptr == NULL) {
            chr_err("chr input arg is invalid!\n");
            oal_free(pst_mem);
            return -EINVAL;
        }
        ret = copy_from_user(pst_mem, (void __user *)(chr_rx_data.chr_ptr), chr_rx_data.chr_len);
        if (ret) {
            chr_err("chr %s ioctl fail, get data from user fail", chr_misc_dev.name);
            oal_free(pst_mem);
            return -EINVAL;
        }

        chr_write_errno_to_queue(chr_rx_data.chr_errno, CHR_HOST, pst_mem, chr_rx_data.chr_len);
        oal_free(pst_mem);
    }

    return CHR_SUCC;
}

/*
 * 函 数 名  : chr_misc_ioctl
 * 功能描述  : 控制设备节点接口
 */
static long chr_misc_ioctl(struct file *fp, uint32_t cmd, uintptr_t arg)
{
    uint32_t __user *puser = (uint32_t __user *)arg;

    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_err("chr %s ioctl fail, module is disable\n", chr_misc_dev.name);
        return -EBUSY;
    }

    if (_IOC_TYPE(cmd) != CHR_MAGIC) {
        chr_err("chr %s ioctl fail, the type of cmd is error type is %d\n",
                chr_misc_dev.name, _IOC_TYPE(cmd));
        return -EINVAL;
    }

    if (_IOC_NR(cmd) > CHR_MAX_NR) {
        chr_err("chr %s ioctl fail, the nr of cmd is error, nr is %d\n",
                chr_misc_dev.name, _IOC_NR(cmd));
        return -EINVAL;
    }

    switch (cmd) {
        case CHR_ERRNO_WRITE:
            if (chr_misc_errno_write(puser) < 0) {
                return -EINVAL;
            }
            break;
        case CHR_ERRNO_ASK:
            break;
        default:
            chr_warning("chr ioctl not support cmd=0x%x\n", cmd);
            return -EINVAL;
    }
    return CHR_SUCC;
}

/*
 * 函 数 名  : chr_misc_release
 * 功能描述  : 释放节点设备接口
 */
static int32_t chr_misc_release(struct inode *fd, struct file *fp)
{
    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_err("chr %s release fail, module is disable\n", chr_misc_dev.name);
        return -EBUSY;
    }
    chr_dbg("chr %s release success\n", chr_misc_dev.name);
    return CHR_SUCC;
}

/*
 * 函 数 名  : chr_exception
 * 功能描述  : 内核空间抛异常码接口
 */
int32_t chr_exception(uint32_t errno)
{
    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_dbg("chr throw exception fail, module is disable\n");
        return -CHR_EFAIL;
    }

    chr_write_errno_to_queue(errno, CHR_HOST, NULL, 0);
    return CHR_SUCC;
}

int32_t chr_exception_para(uint32_t chr_errno, uint8_t *chr_ptr, u_int16_t chr_len)
{
    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_dbg("chr throw exception fail, module is disable\n");
        return -CHR_EFAIL;
    }

    chr_write_errno_to_queue(chr_errno, CHR_HOST, chr_ptr, chr_len);
    return CHR_SUCC;
}
EXPORT_SYMBOL(chr_exception_para);

int32_t chr_exception_para_q(uint32_t chr_errno, u_int16_t chr_flag, uint8_t *chr_ptr, u_int16_t chr_len)
{
    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_dbg("chr throw exception fail, module is disable\n");
        return -CHR_EFAIL;
    }
    chr_write_errno_to_queue(chr_errno, chr_flag, chr_ptr, chr_len);
    return CHR_SUCC;
}

int32_t chr_miscdevs_init(void)
{
    int32_t ret;
    init_waitqueue_head(&g_chr_event.errno_wait);
    skb_queue_head_init(&g_chr_event.errno_queue);

    ret = misc_register(&chr_misc_dev);
    if (ret != CHR_SUCC) {
        chr_err("chr module init fail\n");
        return -CHR_EFAIL;
    }

    g_chr_enable = CHR_LOG_ENABLE;
    chr_info("chr module init succ\n");
    return CHR_SUCC;
}

void chr_miscdevs_exit(void)
{

    if (g_chr_enable != CHR_LOG_ENABLE) {
        chr_info("chr module is diabled\n");
        return;
    }

    misc_deregister(&chr_misc_dev);
    g_chr_enable = CHR_LOG_DISABLE;
    chr_info("chr module exit succ\n");
}

MODULE_AUTHOR("huawei platform Driver Group");
MODULE_DESCRIPTION("huawei chr log driver");
MODULE_LICENSE("GPL");
