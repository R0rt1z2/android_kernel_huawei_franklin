/*
 * aw882xx_misc.c   aw882xx misc module
 *
 * Version: v0.1.11
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Jianming zhang <zhangjianming@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include "aw882xx.h"
#include "aw882xx_misc.h"

extern int mtk_spk_send_ipi_buf_to_dsp(void *data_buffer, uint32_t data_size);
extern int mtk_spk_recv_ipi_buf_from_dsp(int8_t *buffer, int16_t size, uint32_t *buf_len);

static const uint32_t PARAM_ID_INDEX_TABLE[][INDEX_PARAMS_ID_MAX] = {
        {
            AFE_PARAM_ID_AWDSP_RX_PARAMS,
            AFE_PARAM_ID_AWDSP_RX_SET_ENABLE,
            AFE_PARAM_ID_AWDSP_TX_SET_ENABLE,
            AFE_PARAM_ID_AWDSP_RX_VMAX_L,
            AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L,
            AFE_PARAM_ID_AWDSP_RX_RE_L,
            AFE_PARAM_ID_AWDSP_RX_NOISE_L,
            AFE_PARAM_ID_AWDSP_RX_F0_L,
            AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L,
        },
        {
            AFE_PARAM_ID_AWDSP_RX_PARAMS,
            AFE_PARAM_ID_AWDSP_RX_SET_ENABLE,
            AFE_PARAM_ID_AWDSP_TX_SET_ENABLE,
            AFE_PARAM_ID_AWDSP_RX_VMAX_R,
            AFE_PARAM_ID_AWDSP_RX_CALI_CFG_R,
            AFE_PARAM_ID_AWDSP_RX_RE_R,
            AFE_PARAM_ID_AWDSP_RX_NOISE_R,
            AFE_PARAM_ID_AWDSP_RX_F0_R,
            AFE_PARAM_ID_AWDSP_RX_REAL_DATA_R,
        },
    };

static int aw882xx_misc_file_open(struct inode *inode, struct file *file)
{
    struct miscdevice *device = NULL;
    struct aw882xx *aw882xx = NULL;

    if(try_module_get(THIS_MODULE) == 0)
        return -ENODEV;

    device = (struct miscdevice *) file->private_data;
    aw882xx = container_of(device, struct aw882xx, misc_device);
    file->private_data = (void *)aw882xx;
    aw_dev_dbg(aw882xx->dev, "%s: misc open success\n", __func__);

    return 0;
}

static int aw882xx_misc_file_release(struct inode *inode, struct file *file)
{
    file->private_data = (void *) NULL;
    pr_debug("misc release success\n");
    return 0;
}

static int aw_get_params_id_by_index(int index, int32_t *params_id, int channel)
{
    if (index > INDEX_PARAMS_ID_MAX || channel > 1) {
        pr_err("%s: error: index is %d, channel %d\n",
            __func__, index, channel);
        return -EINVAL;
    }
    *params_id = PARAM_ID_INDEX_TABLE[channel][index];
    return 0;
}

int aw_write_data_to_dsp(int index, void *data, int len, int channel)
{
    int ret;
    int32_t param_id;
    uint32_t real_len;
    mtk_dsp_hdr_t *hdr = (mtk_dsp_hdr_t *)data;

    ret = aw_get_params_id_by_index(index, &param_id, channel);
    if (ret < 0)
        return ret;
    pr_info("%s : param id = 0x%x", __func__, param_id);

    real_len = len + sizeof(mtk_dsp_hdr_t);

    hdr->params_id = param_id;
    hdr->type = MTK_DSP_MSG_TYPE_DATA;

    ret = mtk_spk_send_ipi_buf_to_dsp(data, real_len);
    if (ret < 0)
        return ret;

    return 0;
}

int aw_read_data_to_dsp(int index, void *data, int len, int channel)
{
    int ret;
    int32_t param_id;
    uint32_t real_len = len;
    mtk_dsp_hdr_t hdr ;

    ret = aw_get_params_id_by_index(index, &param_id, channel);
    if (ret < 0)
        return ret;

    pr_info("%s : param id = 0x%x", __func__, param_id);
    hdr.type = MTK_DSP_MSG_TYPE_CMD;
    hdr.params_id = param_id;

    ret = mtk_spk_send_ipi_buf_to_dsp(&hdr, sizeof(mtk_dsp_hdr_t));
    if (ret < 0)
        return ret;

    ret = mtk_spk_recv_ipi_buf_from_dsp(data, len, &real_len);
    if (ret < 0)
        return ret;

    return 0;
}

static int aw882xx_get_index_by_cmd(unsigned int cmd, int32_t *index)
{
    switch (cmd) {
    case AW882XX_IOCTL_GET_CALI_CFG:
    case AW882XX_IOCTL_SET_CALI_CFG:
        *index = INDEX_PARAMS_ID_RX_CALI_CFG;
        break;
    case AW882XX_IOCTL_GET_CALI_DATA:
        *index = INDEX_PARAMS_ID_RX_REAL_DATA;
        break;
    case AW882XX_IOCTL_SET_NOISE:
        *index = INDEX_PARAMS_ID_RX_NOISE;
        break;
    case AW882XX_IOCTL_GET_F0:
        *index = INDEX_PARAMS_ID_RX_F0;
        break;
    case AW882XX_IOCTL_GET_CALI_RE:
    case AW882XX_IOCTL_SET_CALI_RE:
        *index = INDEX_PARAMS_ID_RX_RE;
        break;
    case AW882XX_IOCTL_GET_VMAX:
    case AW882XX_IOCTL_SET_VMAX:
        *index = INDEX_PARAMS_ID_RX_VMAX;
        break;
    case AW882XX_IOCTL_SET_PARAMS:
        *index = INDEX_PARAMS_ID_RX_PARAMS;
        break;
    case AW882XX_IOCTL_SET_RX_BYPASS:
        *index = INDEX_PARAMS_ID_RX_ENBALE;
        break;
    default:
        pr_err("%s: unsupported cmd %d\n", __func__, cmd);
        return -EINVAL;
    }

    return 0;
}

static int aw882xx_misc_ioctl(struct aw882xx *aw882xx, unsigned int cmd, unsigned long arg)
{
    int16_t data_len = _IOC_SIZE(cmd);
    int ret = 0;
    char *data_ptr = NULL;
    uint32_t index = 0;

    struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

    aw_dev_info(aw882xx->dev, "cmd: 0x%x, data_len:%d \n", cmd, data_len);
    if(data_len > 0)
        data_ptr = kmalloc(data_len + sizeof(mtk_dsp_hdr_t), GFP_KERNEL);

    if(data_ptr == NULL) {
        aw_dev_err(aw882xx->dev, "%s:malloc failed\n",__func__);
        return -EFAULT;
    }

    ret = aw882xx_get_index_by_cmd(cmd, &index);
    if (ret < 0)
        goto EXIT;

    switch (cmd) {
        case AW882XX_IOCTL_SET_CALI_CFG:
        case AW882XX_IOCTL_SET_NOISE:
        case AW882XX_IOCTL_SET_CALI_RE:
        case AW882XX_IOCTL_SET_VMAX:
        case AW882XX_IOCTL_SET_PARAMS:
        case AW882XX_IOCTL_SET_RX_BYPASS:
        if (copy_from_user((uintptr_t)data_ptr + sizeof(mtk_dsp_hdr_t),
                (void __user *)arg, data_len)) {
            ret = -EFAULT;
            goto EXIT;
        }
        ret = aw_write_data_to_dsp(index, data_ptr,
                    data_len, chan_info->channel);
        if (ret) {
            aw_dev_err(aw882xx->dev, "%s: dsp_msg_write error: %d\n",
                __func__, index);
            ret =  -EFAULT;
            goto EXIT;
        }
        break;
        case AW882XX_IOCTL_GET_CALI_CFG:
        case AW882XX_IOCTL_GET_CALI_DATA:
        case AW882XX_IOCTL_GET_F0:
        case AW882XX_IOCTL_GET_CALI_RE:
        case AW882XX_IOCTL_GET_VMAX:
        ret = aw_read_data_to_dsp(index, data_ptr,
                    data_len, chan_info->channel);
        if (ret) {
            aw_dev_err(aw882xx->dev, "%s: dsp_msg_read error: %d\n",
                __func__, index);
            ret = -EFAULT;
            goto EXIT;
        }
        if (copy_to_user((void __user *)arg,
            data_ptr, data_len)) {
            ret = -EFAULT;
            goto EXIT;
        }
        break;
        default:
            aw_dev_err(aw882xx->dev, "%s:unsupported cmd %d\n", __func__, cmd);
            break;
    }
EXIT:
    kfree(data_ptr);
    return 0;
}

static ssize_t aw882xx_misc_i2c_write(struct file *file,
        const char __user *buf, size_t nbytes, loff_t *ppos)
{
    unsigned char *kern_buf = NULL;
    int len = nbytes / 3;
    int i = 0, ret = 0;
    struct aw882xx *aw882xx = (struct aw882xx *) file->private_data;

    UNUSED_PARAM(ppos);
    if(aw882xx == NULL || file == NULL ||
        buf == NULL || nbytes > I2C_TRANSFER_MAX_SIZE || nbytes <= 0) {
        pr_err("invalid param:%s aw882xx %p file %p buf %p nbytes %ld",
                __func__, aw882xx, file, buf, nbytes);
        return -EINVAL;
    }

    kern_buf = kzalloc(nbytes, GFP_KERNEL);
    if(kern_buf == NULL) {
        pr_err("failed to allocate buffer\n");
        ret = -ENOMEM;
        goto EXIT;
    }

    if(copy_from_user(kern_buf, (void __user *)buf, nbytes)) {
        ret = -EFAULT;
        goto EXIT;
    }
    /*
    pr_err("%s:",__func__);
    for(i = 0 ; i < nbytes; i++)
    {
        pr_err("0x%x,", kern_buf[i]);
    }
    pr_err("\n");
    */

    if(nbytes < 3) {
        aw882xx->reg_addr = kern_buf[0];
    } else {
        for(i = 0; i < len; i++) {
            ret = aw882xx_i2c_write(aw882xx, kern_buf[3 * i], (kern_buf[3 * i +1] << 8) + kern_buf[3 * i + 2]);
            if(ret < 0) {
                pr_err("%s i2c transfer error", __func__);
                goto EXIT;
            }
        }
    }

EXIT:
    kfree(kern_buf);
    return ret;
}


static ssize_t aw882xx_misc_i2c_read(struct file *file,
        char __user *buf, size_t nbytes, loff_t *ppos)
{
    unsigned char *kern_buf = NULL;
    int ret = 0;
    struct aw882xx *aw882xx = (struct aw882xx *) file->private_data;

    UNUSED_PARAM(ppos);
    if(aw882xx == NULL || file == NULL ||
        buf == NULL || nbytes > I2C_TRANSFER_MAX_SIZE || nbytes < 2) {
        pr_err("invalid param:%s aw882xx %p file %p buf %p nbytes %ld",
                __func__, aw882xx, file, buf, nbytes);
        return -EINVAL;
    }

    kern_buf = kzalloc(nbytes, GFP_KERNEL);
    if(kern_buf == NULL) {
        pr_err("failed to allocate buffer\n");
        ret = -ENOMEM;
        goto EXIT;
    }

    ret = aw882xx_i2c_read_sigle(aw882xx, aw882xx->reg_addr, kern_buf);
    if(ret < 0) {
        pr_err("%s i2c transfer error", __func__);
        goto EXIT;
    }
    pr_err("%s: addr:%x, val:%x\n",__func__, aw882xx->reg_addr, (*kern_buf << 8) + *(kern_buf + 1));

    ret = copy_to_user((void __user *)buf, kern_buf, nbytes);
    if(ret) {
        pr_err("%s copy to user fail %d", __func__, ret);
        goto EXIT;
    }

    ret = nbytes;
    
EXIT:
    kfree(kern_buf);
    return ret;
}

static long aw882xx_misc_file_unlocked_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct aw882xx *aw882xx = NULL;

    aw882xx = (struct aw882xx *) file->private_data;
    pr_info("%s: ioctl enter\n", __func__);
    
    if((_IOC_TYPE(cmd)) != (AW882XX_IOCTL_MAGIC)) {
        aw_dev_err(aw882xx->dev, "%s: cmd magic err\n", __func__);
        return -EINVAL;
    }

    ret = aw882xx_misc_ioctl(aw882xx, cmd, arg);
    if(ret)
        return -EINVAL;

    return 0;
}

#ifdef CONFIG_COMPAT
static long aw882xx_misc_file_compat_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
    if (!file->f_op || !file->f_op->unlocked_ioctl) {
        pr_err("%s: op null\n", __func__);
        return -ENOTTY;
    }
    return file->f_op->unlocked_ioctl(file, cmd, arg);
}
#endif

static const struct file_operations aw882xx_misc_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = aw882xx_misc_file_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = aw882xx_misc_file_compat_ioctl,
#endif
    .open = aw882xx_misc_file_open,
    .release = aw882xx_misc_file_release,
    .read = aw882xx_misc_i2c_read,
    .write = aw882xx_misc_i2c_write,
};

void aw882xx_misc_init(struct miscdevice *misc_device)
{
    int ret;
    struct aw882xx *aw882xx = container_of(misc_device, struct aw882xx, misc_device);
    
    misc_device->minor = MISC_DYNAMIC_MINOR;
    misc_device->fops = &aw882xx_misc_fops;
    
    ret = misc_register(misc_device);
    if(ret) {
        aw_dev_err(aw882xx->dev, "%s: misc register fail :%d \n",__func__, ret);
        return;
    }
    
    aw_dev_dbg(aw882xx->dev, "%s: misc register success\n",__func__);
}

void aw882xx_misc_deinit(struct miscdevice *misc_device)
{
    struct aw882xx *aw882xx = container_of(misc_device, struct aw882xx, misc_device);

    misc_deregister(&aw882xx->misc_device);
    aw_dev_dbg(aw882xx->dev, "%s: misc unregister done\n",__func__);
}
