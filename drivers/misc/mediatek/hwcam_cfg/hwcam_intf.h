 /*
 * hwcam_inf.h
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * hwcam inf header file
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

#ifndef __HW_ALAN_KERNEL_CAMERA_OBJ_MDL_INTERFACE__
#define __HW_ALAN_KERNEL_CAMERA_OBJ_MDL_INTERFACE__

#include <linux/dma-buf.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <hwcam_define.h>

typedef struct _tag_hwcam_vbuf {
	struct vb2_buffer buf;
	struct list_head node;
} hwcam_vbuf_t;

struct _tag_hwcam_cfgstream_intf;
struct _tag_hwcam_cfgpipeline_intf;
struct _tag_hwcam_dev_intf;
struct _tag_hwcam_user_intf;
struct _tag_hwcam_cfgreq_mount_stream_intf;
struct _tag_hwcam_cfgreq_mount_pipeline_intf;
struct _tag_hwcam_cfgreq_intf;
struct _tag_hwcam_cfgack;

/* hwcam_cfgreq interface definition begin */
typedef struct _tag_hwcam_cfgreq_vtbl {
	void (*get)(struct _tag_hwcam_cfgreq_intf *intf);
	int (*put)(struct _tag_hwcam_cfgreq_intf *intf);
	int (*on_req)(struct _tag_hwcam_cfgreq_intf *intf,
		struct v4l2_event *ev);
	int (*on_cancel)(struct _tag_hwcam_cfgreq_intf *intf,
		int reason);
	int (*on_ack)(struct _tag_hwcam_cfgreq_intf *intf,
		struct _tag_hwcam_cfgack *ack);
} hwcam_cfgreq_vtbl_t;

typedef struct _tag_hwcam_cfgreq_intf {
	struct _tag_hwcam_cfgreq_vtbl const *vtbl;
} hwcam_cfgreq_intf_t;

static inline void hwcam_cfgreq_intf_get(hwcam_cfgreq_intf_t *intf)
{
	return intf->vtbl->get(intf);
}

static inline int hwcam_cfgreq_intf_put(hwcam_cfgreq_intf_t *intf)
{
	return intf->vtbl->put(intf);
}

static inline int hwcam_cfgreq_intf_on_req(hwcam_cfgreq_intf_t *intf,
	struct v4l2_event *ev)
{
	return intf->vtbl->on_req(intf, ev);
}

static inline int hwcam_cfgreq_intf_on_cancel(hwcam_cfgreq_intf_t *intf,
	int reason)
{
	return intf->vtbl->on_cancel(intf, reason);
}

static inline int hwcam_cfgreq_intf_on_ack(hwcam_cfgreq_intf_t *intf,
	struct _tag_hwcam_cfgack *ack)
{
	return intf->vtbl->on_ack(intf, ack);
}

static inline int hwcam_cfgreq_on_ack_noop(hwcam_cfgreq_intf_t *pintf,
	struct _tag_hwcam_cfgack *ack)
{
	return 0;
}

typedef void (*pfn_hwcam_cfgdev_release_ack)(struct _tag_hwcam_cfgack *ack);

typedef struct _tag_hwcam_cfgack {
	struct list_head node;
	pfn_hwcam_cfgdev_release_ack release;
	struct v4l2_event ev;
} hwcam_cfgack_t;

static inline int hwcam_cfgack_result(struct _tag_hwcam_cfgack *ack)
{
	return ((hwcam_cfgreq_t *)ack->ev.u.data)->rc;
}

/* hwcam_usr interface definition begin */
typedef struct _tag_hwcam_user_vtbl {
	void (*get)(struct _tag_hwcam_user_intf *intf);
	int (*put)(struct _tag_hwcam_user_intf *intf);
	void (*wait_begin)(struct _tag_hwcam_user_intf *intf);
	void (*wait_end)(struct _tag_hwcam_user_intf *intf);
	void (*notify)(struct _tag_hwcam_user_intf *intf,
		struct v4l2_event *ev);
} hwcam_user_vtbl_t;

typedef struct _tag_hwcam_user_intf {
	hwcam_user_vtbl_t const *vtbl;
} hwcam_user_intf_t;

static inline void hwcam_user_intf_get(hwcam_user_intf_t *intf)
{
	intf->vtbl->get(intf);
}

static inline int hwcam_user_intf_put(hwcam_user_intf_t *intf)
{
	return intf->vtbl->put(intf);
}

static inline void hwcam_user_intf_wait_begin(hwcam_user_intf_t *intf)
{
	intf->vtbl->wait_begin(intf);
}

static inline void hwcam_user_intf_wait_end(hwcam_user_intf_t *intf)
{
	intf->vtbl->wait_end(intf);
}

static inline void hwcam_user_intf_notify(hwcam_user_intf_t *intf,
	struct v4l2_event *ev)
{
	intf->vtbl->notify(intf, ev);
}
/* hwcam_usr interface definition end */

/* hwcam_dev interface definition begin */

typedef struct _tag_hwcam_dev_vtbl {
	void (*notify)(struct _tag_hwcam_dev_intf *intf, struct v4l2_event *ev);
} hwcam_dev_vtbl_t;

typedef struct _tag_hwcam_dev_intf {
	hwcam_dev_vtbl_t const *vtbl;
} hwcam_dev_intf_t;

static inline void hwcam_dev_intf_notify(hwcam_dev_intf_t *intf,
	struct v4l2_event *ev)
{
	intf->vtbl->notify(intf, ev);
}
/* hwcam_dev interface definition end */

#endif /* __HW_ALAN_KERNEL_CAMERA_OBJ_MDL_H__ */