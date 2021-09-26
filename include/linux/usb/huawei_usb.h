/*
 * huawei_usb.h
 *
 * huawei usb interface
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#ifndef HUAWEI_USB_H
#define HUAWEI_USB_H

#define COUNTRY_US         "us"
#define COUNTRY_JAPAN      "jp"
#define VENDOR_TMOBILE     "t-mobile"
#define VENDOR_EMOBILE     "emobile"
#define VENDOR_TRACFONE    "tracfone"
#define VENDOR_CC          "consumercellular"
#define VENDOR_SOFTBANK    "softbank"

#define USB_DEFAULT_SN     "0123456789ABCDEF"
#define USB_SERIAL_LEN     32
#define VENDOR_NAME_LEN    32
#define COUNTRY_NAME_LEN   32

/* support 3 luns at most, 1 lun for cdrom and 2 luns for udisk */
#define USB_MAX_LUNS               3
#define SC_REWIND                  0x01
#define SC_REWIND_11               0x11
#define ORI_INDEX                  0
#define USB_IF_CLASS_HW_PNP21      0xff
#define USB_IF_SUBCLASS_HW_PNP21   0x11
#define USB_IF_PROTOCOL_HW_MODEM   0x21
#define USB_IF_PROTOCOL_HW_PCUI    0x22
#define USB_IF_PROTOCOL_HW_DIAG    0x23
#define USB_IF_PROTOCOL_NOPNP      0xff
#define HW_CUSTOM_FEATURES_LENTH   32
#define HW_CUSTOM_FEATURES_USB_TETHER_DISABLED   BIT(2)

#ifndef KERNEL_HWFLOW
#define KERNEL_HWFLOW 1
#endif
#define USB_LOGS_ERR  1
#define USB_LOGS_INFO 2
#define USB_LOGS_DBG  3

#endif /* HUAWEI_USB_H */
