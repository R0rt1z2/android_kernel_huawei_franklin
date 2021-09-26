/*
 * rt8979_bc12.h
 *
 * driver file for switch chip
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
 *
 */

#ifndef _RT8979_BC12_H_
#define _RT8979_BC12_H_

/* MUIC_CTRL1 0x02 */
#define RT8979_SW_OPEN_MASK         BIT(4)
#define RT8979_INT_MASK_MASK        BIT(0)

/* TIMING_SET2 0x0D */
#define RT8979_INTB_WATCHDOG_SHIFT  6
#define RT8979_DCD_TIMEOUT_SHIFT    4
#define RT8979_INTB_WATCHDOG_MASK   GENMASK(7, 6)
#define RT8979_DCD_TIMEOUT_MASK     GENMASK(5, 4)

/* MUIC_CTRL2 0x0E */
#define RT8979_SCP_EN_MASK          BIT(6)
#define RT8979_DCD_TIMEOUT_EN_MASK  BIT(0)

/* MUIC_CTRL3 0x10 */
#define RT8979_OVP_SEL_SHIFT        4
#define RT8979_OVP_SEL_MASK         GENMASK(5, 4)

/* MUIC_STATUS1 0x12 */
#define RT8979_DCDT_MASK            BIT(3)
#define RT8979_VBUS_UVLO_MASK       BIT(2)
#define RT8979_VBUS_OVP_MASK        BIT(0)

/* MUIC_STATUS2 0x13 */
#define RT8979_USB_STATUS_SHIFT     4
#define RT8979_USB_STATUS_MASK      GENMASK(6, 4)

/* RESET 0x19 */
#define RT8979_RESET_MASK           BIT(0)

/* MUIC_OPT1 0xA4 */
#define RT8979_USBCHGEN_MASK        BIT(6)

#endif /* _RT8979_BC12_H_ */
