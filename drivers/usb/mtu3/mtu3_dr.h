/*
 * mtu3_dr.h - dual role switch and host glue layer header
 *
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MTU3_DR_H_
#define _MTU3_DR_H_

#define DEFAULT_DEV_VRT_REF 6
#define DEFAULT_DEV_TERM_REF 6
#define DEFAULT_DEV_ENHANCE 1
#define DEFAULT_HOST_VRT_REF 6
#define DEFAULT_HOST_TERM_REF 6
#define DEFAULT_HOST_ENHANCE 1
#define INVALID_PHY_PARA (-1)

#if IS_ENABLED(CONFIG_USB_MTU3_HOST) || IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)

int ssusb_host_init(struct ssusb_mtk *ssusb, struct device_node *parent_dn);
void ssusb_host_exit(struct ssusb_mtk *ssusb);
int ssusb_wakeup_of_property_parse(struct ssusb_mtk *ssusb,
				struct device_node *dn);
int ssusb_host_enable(struct ssusb_mtk *ssusb);
int ssusb_host_disable(struct ssusb_mtk *ssusb, bool suspend);
int ssusb_wakeup_enable(struct ssusb_mtk *ssusb);
void ssusb_wakeup_disable(struct ssusb_mtk *ssusb);

#else

static inline int ssusb_host_init(struct ssusb_mtk *ssusb,

	struct device_node *parent_dn)
{
	return 0;
}

static inline void ssusb_host_exit(struct ssusb_mtk *ssusb)
{}

static inline int ssusb_wakeup_of_property_parse(
	struct ssusb_mtk *ssusb, struct device_node *dn)
{
	return 0;
}

static inline int ssusb_host_enable(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline int ssusb_host_disable(struct ssusb_mtk *ssusb, bool suspend)
{
	return 0;
}

static inline int ssusb_wakeup_enable(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_wakeup_disable(struct ssusb_mtk *ssusb)
{}

#endif

enum mtu3_vbus_id_state {
	MTU3_ID_FLOAT = 1,
	MTU3_ID_GROUND,
	MTU3_VBUS_OFF,
	MTU3_VBUS_VALID,
	MTU3_CMODE_VBUS_VALID,
};

extern bool upmu_is_chr_det(void);
extern u32 upmu_get_rgs_chrdet(void);

#if defined(CONFIG_MACH_MT6873) || defined(CONFIG_MACH_MT6853) || defined(CONFIG_MACH_MT6885)
void phy_eye_diagram_write(u32 u2_vrt_ref, u32 u2_term_ref, u32 u2_enhance);
#else
static inline void phy_eye_diagram_write(u32 u2_vrt_ref, u32 u2_term_ref, u32 u2_enhance)
{
}
#endif /* defined(CONFIG_MACH_MT6873) || defined(CONFIG_MACH_MT6853) || defined(CONFIG_MACH_MT6885) */

void ssusb_set_mailbox(struct otg_switch_mtk *otg_sx,
	enum mtu3_vbus_id_state status);


#if IS_ENABLED(CONFIG_USB_MTU3_GADGET) || IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
int ssusb_gadget_init(struct ssusb_mtk *ssusb);
void ssusb_gadget_exit(struct ssusb_mtk *ssusb);
#else
static inline int ssusb_gadget_init(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_gadget_exit(struct ssusb_mtk *ssusb)
{}
#endif


#if IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
int ssusb_otg_switch_init(struct ssusb_mtk *ssusb);
void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb);
int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on);
#if IS_ENABLED(CONFIG_USB_MTU3_PLAT_PHONE)
extern u32 mtu3_speed;
#endif

#else

static inline int ssusb_otg_switch_init(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb)
{}

static inline int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on)
{
	return 0;
}

#endif

#endif		/* _MTU3_DR_H_ */
