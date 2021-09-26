/*
 * hw_f_acm.c
 *
 * USB CDC serial (ACM) function driver
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "hw_u_serial.h"

/*
 * This CDC ACM function support just wraps control functions and
 * notifications around the generic serial-over-usb code.
 *
 * Because CDC ACM is standardized by the USB-IF, many host operating
 * systems have drivers for it.  Accordingly, ACM is the preferred
 * interop solution for serial-port type connections.  The control
 * models are often not necessary, and in any case don't do much in
 * this bare-bones implementation.
 *
 * Note that even MS-Windows has some support for ACM.  However, that
 * support is somewhat broken because when you use ACM in a composite
 * device, having multiple interfaces confuses the poor OS.  It doesn't
 * seem to understand CDC Union descriptors.  The new "association"
 * descriptors (roughly equivalent to CDC Unions) may sometimes help.
 */

struct f_acm {
	struct gserial port;
	u8 ctrl_id;
	u8 data_id;
	u8 port_num;
	u8 pending;
	/*
	 * lock is mostly for pending and notify_req ... they get accessed
	 * by callbacks both from tty (open/close/break) under its spinlock,
	 * and notify_req.complete() which can't use that lock.
	 */
	spinlock_t lock;
	int support_notify;
	struct usb_ep *notify;
	struct usb_request *notify_req;
	int (*pending_notify)(struct f_acm *acm);
	/* cdc vendor flow control notify */
	u32 rx_is_on;
	u32 tx_is_on;
	struct usb_cdc_line_coding port_line_coding; /* 8-N-1 etc */
	/* SetControlLineState request -- CDC 1.1 section 6.2.14 (INPUT) */
	u16 port_handshake_bits;
	/* SerialState notification -- CDC 1.1 section 6.3.5 (OUTPUT) */
	u16 serial_state;
};

#define ACM_CTRL_BRK  (1 << 2)
#define ACM_CTRL_DSR  (1 << 1)
#define ACM_CTRL_DCD  (1 << 0)

struct acm_name_type_tbl g_hw_acm_type_table[HW_MAX_U_SERIAL_PORTS] = {
	/* name type(prot id) */
	{ "hw_acm_modem", USB_IF_PROTOCOL_HW_MODEM }, /* hw modem interface */
	{ "hw_acm_PCUI", USB_IF_PROTOCOL_HW_PCUI }, /* hw PCUI for AT command */
	{ "hw_acm_DIAG", USB_IF_PROTOCOL_HW_DIAG }, /* hw DIAG for USB update */
	{ "unknown", USB_IF_PROTOCOL_NOPNP }
};

static int g_acm_is_single_interface = HW_ACM_IS_SINGLE;

static inline unsigned char acm_get_type(struct f_acm *acm)
{
	return (unsigned char)g_hw_acm_type_table[acm->port_num].type;
}

static inline struct f_acm *func_to_acm(struct usb_function *f)
{
	return container_of(f, struct f_acm, port.func);
}

static inline struct f_acm *port_to_acm(struct gserial *p)
{
	return container_of(p, struct f_acm, port);
}

/* notification endpoint uses smallish and infrequent fixed-size messages */
#define GS_NOTIFY_INTERVAL_MS     32
#define GS_NOTIFY_MAXPACKET       10 /* notification + 2 bytes */
#define NOM_PACKET_SIZE           512
#define MAX_PACKET_SIZE           1024
#define EPLEN                     8
#define SINGE_EP                  1
#define DUAL_EP                   2
#define RX_ON                     0x1
#define TX_ON                     0x2
#define RX_OFF                    0x0
#define TX_OFF                    0x0
#define TWO_EPS                   2
#define THREE_EPS                 3
#define TRUE                      1
#define FALSE                     0
#define HW_ACM                    "hw_acm"

/* interface and class descriptors */
static struct usb_interface_assoc_descriptor g_acm_iad_descriptor = {
	.bLength = sizeof(g_acm_iad_descriptor),
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bInterfaceCount = 2, /* 2:control + data */
	.bFunctionClass = USB_CLASS_COMM,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_ACM_PROTO_AT_V25TER,
};

static struct usb_interface_descriptor g_acm_control_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bNumEndpoints = SINGE_EP,
	.bInterfaceClass = USB_CLASS_COMM,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_ACM_PROTO_AT_V25TER,
};

static struct usb_interface_descriptor g_acm_data_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bNumEndpoints = DUAL_EP,
	.bInterfaceClass = USB_CLASS_CDC_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
};

static struct usb_cdc_header_desc g_acm_header_desc = {
	.bLength = sizeof(g_acm_header_desc),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_HEADER_TYPE,
	.bcdCDC = cpu_to_le16(0x0110), /* cdc version */
};

static struct usb_cdc_call_mgmt_descriptor g_acm_call_mgmt_descriptor = {
	.bLength = sizeof(g_acm_call_mgmt_descriptor),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 0,
};

static struct usb_cdc_acm_descriptor g_acm_descriptor = {
	.bLength = sizeof(g_acm_descriptor),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_ACM_TYPE,
	.bmCapabilities = USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc g_acm_union_desc = {
	.bLength = sizeof(g_acm_union_desc),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_UNION_TYPE,
};

static struct usb_interface_descriptor g_acm_single_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceClass = HW_PNP21_CLASS,
	.bInterfaceSubClass = HW_PNP21_SUBCLASS,
};

/* full speed support */
static struct usb_endpoint_descriptor g_acm_fs_notify_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval = GS_NOTIFY_INTERVAL_MS,
};

static struct usb_endpoint_descriptor g_acm_fs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor g_acm_fs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header **g_acm_fs_cur_function;
static struct usb_descriptor_header *g_acm_fs_function_single[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_fs_in_desc,
	(struct usb_descriptor_header *)&g_acm_fs_out_desc,
	NULL,
};

static struct usb_descriptor_header *g_acm_fs_function_single_notify[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_fs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_fs_in_desc,
	(struct usb_descriptor_header *)&g_acm_fs_out_desc,
	NULL,
};

static struct usb_descriptor_header *g_acm_fs_function[] = {
	(struct usb_descriptor_header *)&g_acm_iad_descriptor,
	(struct usb_descriptor_header *)&g_acm_control_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_fs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_data_interface_desc,
	(struct usb_descriptor_header *)&g_acm_fs_in_desc,
	(struct usb_descriptor_header *)&g_acm_fs_out_desc,
	NULL,
};

/* high speed support */
static struct usb_endpoint_descriptor g_acm_hs_notify_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval = USB_MS_TO_HS_INTERVAL(GS_NOTIFY_INTERVAL_MS),
};

static struct usb_endpoint_descriptor g_acm_hs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(NOM_PACKET_SIZE),
};

static struct usb_endpoint_descriptor g_acm_hs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(NOM_PACKET_SIZE),
};

static struct usb_descriptor_header **g_acm_hs_cur_function;
static struct usb_descriptor_header *g_acm_hs_function_single[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_hs_in_desc,
	(struct usb_descriptor_header *)&g_acm_hs_out_desc,
	NULL,
};

static struct usb_descriptor_header *g_acm_hs_function_single_notify[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_hs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_hs_in_desc,
	(struct usb_descriptor_header *)&g_acm_hs_out_desc,
	NULL,
};

static struct usb_descriptor_header *g_acm_hs_function[] = {
	(struct usb_descriptor_header *)&g_acm_iad_descriptor,
	(struct usb_descriptor_header *)&g_acm_control_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_hs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_data_interface_desc,
	(struct usb_descriptor_header *)&g_acm_hs_in_desc,
	(struct usb_descriptor_header *)&g_acm_hs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor g_acm_ss_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(MAX_PACKET_SIZE),
};

static struct usb_endpoint_descriptor g_acm_ss_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(MAX_PACKET_SIZE),
};

static struct usb_ss_ep_comp_descriptor g_acm_ss_bulk_comp_desc = {
	.bLength = sizeof(g_acm_ss_bulk_comp_desc),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *g_acm_ss_function[] = {
	(struct usb_descriptor_header *)&g_acm_iad_descriptor,
	(struct usb_descriptor_header *)&g_acm_control_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_hs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&g_acm_data_interface_desc,
	(struct usb_descriptor_header *)&g_acm_ss_in_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&g_acm_ss_out_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	NULL,
};

static struct usb_descriptor_header **g_acm_ss_cur_function;
static struct usb_descriptor_header *g_acm_ss_function_single[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_ss_in_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&g_acm_ss_out_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	NULL,
};

static struct usb_descriptor_header *g_acm_ss_function_single_notify[] = {
	(struct usb_descriptor_header *)&g_acm_single_interface_desc,
	(struct usb_descriptor_header *)&g_acm_header_desc,
	(struct usb_descriptor_header *)&g_acm_descriptor,
	(struct usb_descriptor_header *)&g_acm_call_mgmt_descriptor,
	(struct usb_descriptor_header *)&g_acm_union_desc,
	(struct usb_descriptor_header *)&g_acm_hs_notify_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&g_acm_ss_in_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&g_acm_ss_out_desc,
	(struct usb_descriptor_header *)&g_acm_ss_bulk_comp_desc,
	NULL,
};

/* string descriptors */
#define ACM_CTRL_IDX    0
#define ACM_DATA_IDX    1
#define ACM_IAD_IDX     2

/* static strings, in UTF-8 */
static struct usb_string g_acm_string_defs[] = {
	[ACM_CTRL_IDX].s = "CDC Abstract Control Model (ACM)",
	[ACM_DATA_IDX].s = "CDC ACM Data",
	[ACM_IAD_IDX].s = "CDC Serial",
	{} /* end of list */
};

static struct usb_gadget_strings g_acm_string_table = {
	.language = 0x0409, /* en-us */
	.strings = g_acm_string_defs,
};

static struct usb_gadget_strings *g_acm_strings[] = {
	&g_acm_string_table,
	NULL,
};

/*
 * ACM control ... data handling is delegated to tty library code.
 * The main task of this function is to activate and deactivate
 * that code based on device state; track parameters like line
 * speed, handshake state, and so on; and issue notifications.
 */
static void acm_complete_set_line_coding(struct usb_ep *ep,
	struct usb_request *req)
{
	struct f_acm *acm = NULL;
	struct usb_composite_dev *cdev = NULL;
	struct usb_cdc_line_coding *value = NULL;

	if (!ep || !req)
		return;

	acm = ep->driver_data;

	if (!acm || !(acm->port.func.config))
		return;

	cdev = acm->port.func.config->cdev;
	if (!cdev)
		return;

	if (req->status != 0) {
		DBG(cdev, "acm ttyGS%d completion, err %d\n",
			acm->port_num, req->status);
		return;
	}

	/* normal completion */
	if (req->actual != sizeof(acm->port_line_coding)) {
		DBG(cdev, "acm ttyGS%d short resp, len %d\n",
			acm->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		if (req->buf) {
			value = req->buf;
			/*
			 * REVISIT:  we currently just remember this data.
			 * If we change that, (a) validate it first, then
			 * (b) update whatever hardware needs updating,
			 * (c) worry about locking.  This is information on
			 * the order of 9600-8-N-1 ... most of which means
			 * nothing unless we control a real RS232 line.
			 */
			acm->port_line_coding = *value;
		}
	}
}

/* this function do nothing when usb send cmd 21 02 */
static void acm_complete_2102(struct usb_ep *ep, struct usb_request *req)
{
}

/* this function do nothing when usb send cmd 41 a3 */
static void acm_complete_41a3(struct usb_ep *ep, struct usb_request *req)
{
}

static int hd_req(const struct usb_ctrlrequest *ctrl, struct usb_request *req,
	struct usb_composite_dev *cdev, struct f_acm *acm)
{
	int value = -EOPNOTSUPP;
	u16 index = le16_to_cpu(ctrl->wIndex);
	u16 len = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << EPLEN) | ctrl->bRequest) {
	case (((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << EPLEN) |
		USB_CDC_SET_COMM_FEATURE):
		value = len;
		cdev->gadget->ep0->driver_data = acm;
		req->complete = acm_complete_2102;
		break;

	case (((USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE) << EPLEN) |
		USB_CDC_SEND_PORT_STATUS):
		value = len;
		cdev->gadget->ep0->driver_data = acm;
		req->complete = acm_complete_41a3;
		break;

	/* SET_LINE_CODING ... just read and save what the host sends */
	case (((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << EPLEN) |
		USB_CDC_REQ_SET_LINE_CODING):
		if (len != sizeof(acm->port_line_coding) ||
			index != acm->ctrl_id)
			goto invalid_index;

		value = len;
		cdev->gadget->ep0->driver_data = acm;
		req->complete = acm_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case (((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << EPLEN) |
		USB_CDC_REQ_GET_LINE_CODING):
		if (index != acm->ctrl_id)
			goto invalid_index;

		value = min_t(unsigned int, len, sizeof(acm->port_line_coding));
		if (req->buf)
			memcpy(req->buf, &acm->port_line_coding, value);
		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case (((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << EPLEN) |
		USB_CDC_REQ_SET_CONTROL_LINE_STATE):
		if (index != acm->ctrl_id)
			goto invalid_index;

		value = FALSE;
		acm->port_handshake_bits = le16_to_cpu(ctrl->wValue);
		break;

	default:
invalid_index:
		VDBG(cdev, "rq%02x.%02x\n", ctrl->bRequestType, ctrl->bRequest);
		break;
	}

	return value;
}

static int acm_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_acm *acm = NULL;
	struct usb_composite_dev *cdev = NULL;
	struct usb_request *req = NULL;
	int value;

	if (!f || !ctrl)
		return -EINVAL;

	acm = func_to_acm(f);
	if (!(f->config) || !acm)
		return -EINVAL;

	cdev = f->config->cdev;

	if (!cdev || !(cdev->gadget) || !(cdev->gadget->ep0))
		return -EINVAL;

	req = cdev->req;
	if (!req)
		return -EINVAL;

	/*
	 * composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 *
	 * Note CDC spec table 4 lists the ACM request profile.  It requires
	 * encapsulated command support ... we don't handle any, and respond
	 * to them by stalling.  Options include get/set/clear comm features
	 * (not that useful) and SEND_BREAK.
	 */
	value = hd_req(ctrl, req, cdev, acm);
	/* respond with data transfer or status phase */
	if (value < 0)
		return value;
	DBG(cdev, "acm ttyGS%d req%02x.%02x\n",
		acm->port_num, ctrl->bRequestType, ctrl->bRequest);
	req->zero = 0;
	req->length = value;
	value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (value < 0)
		ERROR(cdev, "acm response on ttyGS%d, err %d\n",
			acm->port_num, value);
	/* device either stalls or reports success */
	return value;
}

static int acm_ctrl_interface_setting(struct usb_function *f,
	struct usb_composite_dev *cdev, unsigned int intf,
	struct f_acm *acm, int *setting)
{
	int ret = 0;

	if (intf != acm->ctrl_id)
		return ret;

	*setting = TRUE;
	if (!acm->notify)
		return ret;

	ret = usb_ep_disable(acm->notify);
	if (ret < 0) {
		ERROR(cdev, "disable acm interface ep failed\n");
		return ret;
	}
	if (config_ep_by_speed(cdev->gadget, f, acm->notify))
		return -EINVAL;
	ret = usb_ep_enable(acm->notify);
	if (ret < 0) {
		ERROR(cdev, "Enable acm interface ep failed\n");
		return ret;
	}
	return ret;
}

static int acm_data_interface_setting(struct usb_function *f,
	struct usb_composite_dev *cdev, unsigned int intf,
	struct f_acm *acm, int *setting)
{
	int ret = 0;

	if (intf != acm->data_id)
		return ret;
	if (!(acm->port.in) || !(acm->port.out))
		return -EINVAL;

	*setting = TRUE;
	if (acm->port.in->enabled) {
		DBG(cdev, "reset acm ttyGS%d\n", acm->port_num);
		hw_gserial_disconnect(&acm->port);
	}

	if (!(acm->port.in->desc) || !(acm->port.out->desc)) {
		DBG(cdev, "activate acm ttyGS%d\n", acm->port_num);
		if (config_ep_by_speed(cdev->gadget, f, acm->port.in) ||
			config_ep_by_speed(cdev->gadget, f, acm->port.out)) {
			acm->port.in->desc = NULL;
			acm->port.out->desc = NULL;
			return -EINVAL;
		}
	}
	ret = hw_gserial_connect(&acm->port, acm->port_num);
	if (ret < 0)
		return ret;
	return 0;
}


static int acm_set_alt(struct usb_function *f, unsigned int intf,
	unsigned int alt)
{
	struct f_acm *acm = NULL;
	struct usb_composite_dev *cdev = NULL;
	int is_setting = FALSE;
	int ret;

	if (!f)
		return -EINVAL;

	acm = func_to_acm(f);
	if (!(f->config))
		return -EINVAL;
	cdev = f->config->cdev;

	if (!acm || !cdev)
		return -EINVAL;

	/*
	 * we know alt == 0, so this is an activation or a reset
	 * if it is single interface, intf, acm->ctrl_id and acm->data_id
	 * are the same, so we can setting data and notify interface
	 * in the same time.
	 *
	 * if it is multi interface, acm->ctrl_id and acm->data_id
	 * are different, so the setting is go ahead in different times.
	 */
	ret = acm_ctrl_interface_setting(f, cdev, intf, acm, &is_setting);
	if (ret < 0)
		return ret;

	ret = acm_data_interface_setting(f, cdev, intf, acm, &is_setting);
	if (ret < 0)
		return ret;

	if (is_setting == FALSE)
		return -EINVAL;

	return 0;
}

static void acm_disable(struct usb_function *f)
{
	struct f_acm *acm = NULL;

	if (!f)
		return;

	acm = func_to_acm(f);
	if (!acm)
		return;

	pr_debug("acm ttyGS%d deactivated\n", acm->port_num);
	hw_gserial_disconnect(&acm->port);
	if (acm->notify)
		usb_ep_disable(acm->notify);
}

/*
 * issue CDC notification to host
 *
 * See section 6.3.5 of the CDC 1.1 specification for information
 * about the only notification we issue:  SerialState change.
 */
static int acm_cdc_notify(struct f_acm *acm, u8 type, u16 value, u16 *data,
	unsigned int length)
{
	struct usb_ep *ep = NULL;
	struct usb_request *req = NULL;
	struct usb_cdc_notification *notify = NULL;
	unsigned int len;
	int status;

	if (!acm)
		return -EINVAL;

	ep = acm->notify;
	len = sizeof(*notify) + length;
	req = acm->notify_req;
	acm->notify_req = NULL;
	acm->pending = FALSE;
	if (!ep || !req)
		return -EINVAL;
	req->length = len;
	notify = req->buf;
	if (!notify)
		return -EINVAL;
	notify->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS |
		USB_RECIP_INTERFACE;
	notify->bNotificationType = type;
	notify->wValue = cpu_to_le16(value);
	notify->wIndex = cpu_to_le16(acm->ctrl_id);
	notify->wLength = cpu_to_le16(length);

	/* ep_queue() can complete immediately if it fills the fifo */
	spin_unlock(&acm->lock);
	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	spin_lock(&acm->lock);

	if (status < 0) {
		pr_err("acm ttyGS%d can't notify serial state, %d\n",
			acm->port_num, status);
		acm->notify_req = req;
	}
	return status;
}

static int acm_notify_serial_state(struct f_acm *acm)
{
	struct usb_composite_dev *cdev = NULL;
	int status;

	if (!(acm->port.func.config))
		return -EINVAL;

	cdev = acm->port.func.config->cdev;

	if (!cdev)
		return -EINVAL;

	spin_lock(&acm->lock);
	if (acm->notify_req) {
		DBG(cdev, "acm ttyGS%d serial state %04x\n",
			acm->port_num, acm->serial_state);
		status = acm_cdc_notify(acm, USB_CDC_NOTIFY_SERIAL_STATE,
			0, &acm->serial_state, sizeof(acm->serial_state));
	} else {
		acm->pending = TRUE;
		acm->pending_notify = acm_notify_serial_state;
		status = 0;
	}
	spin_unlock(&acm->lock);
	return status;
}

static int acm_notify_flow_control(struct f_acm *acm)
{
	int status;
	u16 value;

	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return -EINVAL;
	}

	value = (acm->rx_is_on ? RX_ON : RX_OFF) |
		(acm->tx_is_on ? TX_ON : TX_OFF);
	spin_lock(&acm->lock);
	if (acm->notify_req) {
		status = acm_cdc_notify(acm, USB_CDC_VENDOR_NTF_FLOW_CONTROL,
			value, NULL, 0);
	} else {
		acm->pending = TRUE;
		acm->pending_notify = acm_notify_flow_control;
		status = 0;
	}
	spin_unlock(&acm->lock);
	return status;
}

static void acm_cdc_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_acm *acm = NULL;
	u8 pend = FALSE;

	if (!ep || !req) {
		pr_err("%s:NULL pointer\n", __func__);
		return;
	}
	acm = req->context;
	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}

	/*
	 * on this call path we do NOT hold the port spinlock,
	 * which is why ACM needs its own spinlock
	 */
	spin_lock(&acm->lock);
	if (req->status != -ESHUTDOWN)
		pend = acm->pending;
	acm->notify_req = req;
	spin_unlock(&acm->lock);

	if ((pend == TRUE) && acm->pending_notify)
		acm->pending_notify(acm);
}

/* connect the TTY link is open */
static void acm_connect(struct gserial *port)
{
	struct f_acm *acm = NULL;
	int ret;

	if (!port) {
		pr_err("%s:port NULL pointer\n", __func__);
		return;
	}

	acm = port_to_acm(port);

	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}
	acm->serial_state |= (ACM_CTRL_DSR | ACM_CTRL_DCD);
	ret = acm_notify_serial_state(acm);
	if (ret < 0)
		pr_err("%s:notify state fail\n", __func__);
}

static void acm_disconnect(struct gserial *port)
{
	struct f_acm *acm = NULL;
	int ret;

	if (!port) {
		pr_err("%s:port NULL pointer\n", __func__);
		return;
	}

	acm = port_to_acm(port);
	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}

	acm->serial_state &= ~(ACM_CTRL_DSR | ACM_CTRL_DCD);
	ret = acm_notify_serial_state(acm);
	if (ret < 0)
		pr_err("%s:notify state fail\n", __func__);
}

static void acm_notify_state(struct gserial *port, u16 state)
{
	struct f_acm *acm = NULL;
	int ret;

	if (!port) {
		pr_err("%s:port NULL pointer\n", __func__);
		return;
	}

	acm = port_to_acm(port);

	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}
	acm->serial_state = state;
	ret = acm_notify_serial_state(acm);
	if (ret < 0)
		pr_err("%s:notify state fail\n", __func__);
}

static void acm_flow_control(struct gserial *port, u32 rx_is_on, u32 tx_is_on)
{
	struct f_acm *acm = NULL;
	int ret;

	if (!port) {
		pr_err("%s:port NULL pointer\n", __func__);
		return;
	}

	acm = port_to_acm(port);

	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}
	acm->rx_is_on = rx_is_on;
	acm->tx_is_on = tx_is_on;
	ret = acm_notify_flow_control(acm);
	if (ret < 0)
		pr_err("%s:acm notify flow failed\n", __func__);
}

static int acm_send_break(struct gserial *port, int duration)
{
	struct f_acm *acm = NULL;
	u16 state;

	if (!port) {
		pr_err("%s:port NULL pointer\n", __func__);
		return -EINVAL;
	}

	acm = port_to_acm(port);
	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return -EINVAL;
	}

	state = acm->serial_state;
	state &= ~ACM_CTRL_BRK;
	if (duration)
		state |= ACM_CTRL_BRK;

	acm->serial_state = state;
	return acm_notify_serial_state(acm);
}

static int alloc_interface_id(struct usb_configuration *c,
	struct usb_function *f, struct f_acm *acm)
{
	int status;

	status = usb_interface_id(c, f);
	if (status < 0)
		return status;

	if (g_acm_is_single_interface) {
		acm->ctrl_id = status;
		acm->data_id = status;
		g_acm_single_interface_desc.bInterfaceNumber = status;
		g_acm_call_mgmt_descriptor.bDataInterface = status;
	} else {
		acm->ctrl_id = status;
		g_acm_iad_descriptor.bFirstInterface = status;
		g_acm_control_interface_desc.bInterfaceNumber = status;
		g_acm_union_desc.bMasterInterface0 = status;
		status = usb_interface_id(c, f);
		if (status < 0)
			return status;

		acm->data_id = status;
		g_acm_data_interface_desc.bInterfaceNumber = status;
		g_acm_union_desc.bSlaveInterface0 = status;
		g_acm_call_mgmt_descriptor.bDataInterface = status;
	}
	return status;
}

static int alloc_interface_ep(struct usb_composite_dev *cdev, struct f_acm *acm)
{
	struct usb_ep *ep = NULL;

	if (!cdev->gadget)
		return -EINVAL;

	ep = usb_ep_autoconfig(cdev->gadget, &g_acm_fs_in_desc);
	if (!ep)
		return -EINVAL;

	acm->port.in = ep;
	ep = usb_ep_autoconfig(cdev->gadget, &g_acm_fs_out_desc);
	if (!ep)
		return -EINVAL;

	acm->port.out = ep;
	if (!acm->support_notify) {
		acm->notify = NULL;
		acm->notify_req = NULL;
		return 0;
	}
	ep = usb_ep_autoconfig(cdev->gadget, &g_acm_fs_notify_desc);
	if (!ep)
		return -EINVAL;
	acm->notify = ep;
	/* allocate notification */
	acm->notify_req = hw_gs_alloc_req(ep,
		sizeof(struct usb_cdc_notification) + DUAL_EP, GFP_KERNEL);
	if (!acm->notify_req)
		return -EINVAL;

	acm->notify_req->complete = acm_cdc_notify_complete;
	acm->notify_req->context = acm;
	return 0;
}

static int get_acm_ep_addr(struct f_acm *acm, struct usb_function *f)
{
	int status;

	g_acm_hs_in_desc.bEndpointAddress = g_acm_fs_in_desc.bEndpointAddress;
	g_acm_hs_out_desc.bEndpointAddress = g_acm_fs_out_desc.bEndpointAddress;
	if (acm->support_notify)
		g_acm_hs_notify_desc.bEndpointAddress =
			g_acm_fs_notify_desc.bEndpointAddress;

	g_acm_ss_in_desc.bEndpointAddress = g_acm_fs_in_desc.bEndpointAddress;
	g_acm_ss_out_desc.bEndpointAddress = g_acm_fs_out_desc.bEndpointAddress;
	g_acm_single_interface_desc.bInterfaceProtocol = acm_get_type(acm);
	status = usb_assign_descriptors(f, g_acm_fs_cur_function,
		g_acm_hs_cur_function, g_acm_ss_cur_function,
		g_acm_ss_cur_function);
	if (status < 0)
		pr_err("assign descriptors fail\n");
	return status;
}

/* ACM function driver setup/binding */
static int acm_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = NULL;
	struct f_acm *acm = NULL;
	struct usb_string *us = NULL;
	int status;

	if (!c || !f)
		return -EINVAL;

	cdev = c->cdev;
	acm = func_to_acm(f);
	if (!acm || !cdev) {
		pr_err("%s:acm or cdev NULL pointer\n", __func__);
		return -EINVAL;
	}
	/*
	 * REVISIT might want instance-specific strings to help
	 * distinguish instances,maybe allocate device-global string IDs, and
	 * patch descriptors
	 */
	us = usb_gstrings_attach(cdev, g_acm_strings,
		ARRAY_SIZE(g_acm_string_defs));
	if (IS_ERR(us))
		return PTR_ERR(us);
	g_acm_control_interface_desc.iInterface = us[ACM_CTRL_IDX].id;
	g_acm_data_interface_desc.iInterface = us[ACM_DATA_IDX].id;
	g_acm_iad_descriptor.iFunction = us[ACM_IAD_IDX].id;
	/* allocate instance-specific interface IDs, and patch descriptors */
	status = alloc_interface_id(c, f, acm);
	if (status < 0)
		goto alloc_fail;

	/* allocate instance-specific endpoints */
	status = alloc_interface_ep(cdev, acm);
	if (status < 0)
		goto alloc_fail;

	/*
	 * support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	status = get_acm_ep_addr(acm, f);
	if (status)
		goto alloc_fail;

	return 0;

alloc_fail:
	if (acm->notify_req)
		hw_gs_free_req(acm->notify, acm->notify_req);
	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);

	return status;
}

static void acm_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_acm *acm = NULL;

	if (!c || !f)
		return;

	acm = func_to_acm(f);
	if (!acm)
		return;
	g_acm_string_defs[0].id = 0;
	usb_free_all_descriptors(f);
	if (acm->notify_req)
		hw_gs_free_req(acm->notify, acm->notify_req);
}

static void acm_set_config_vendor(struct f_acm *acm)
{
	if (g_acm_is_single_interface) {
		if (acm->support_notify) {
			/* 3:bulk in + bulk out + interrupt in */
			g_acm_single_interface_desc.bNumEndpoints = THREE_EPS;
			g_acm_fs_cur_function = g_acm_fs_function_single_notify;
			g_acm_hs_cur_function = g_acm_hs_function_single_notify;
			g_acm_ss_cur_function = g_acm_ss_function_single_notify;
		} else {
			/* 2:bulk in + bulk out */
			g_acm_single_interface_desc.bNumEndpoints = TWO_EPS;
			g_acm_fs_cur_function = g_acm_fs_function_single;
			g_acm_hs_cur_function = g_acm_hs_function_single;
			g_acm_ss_cur_function = g_acm_ss_function_single;
		}

		g_acm_single_interface_desc.bInterfaceClass = HW_PNP21_CLASS;
		g_acm_single_interface_desc.bInterfaceSubClass =
			HW_PNP21_SUBCLASS;
	} else {
		g_acm_control_interface_desc.bInterfaceClass = HW_PNP21_CLASS;
		g_acm_control_interface_desc.bInterfaceSubClass =
			HW_PNP21_SUBCLASS;
		g_acm_control_interface_desc.bInterfaceProtocol =
			acm_get_type(acm);
		g_acm_fs_cur_function = g_acm_fs_function;
		g_acm_hs_cur_function = g_acm_hs_function;
		g_acm_ss_cur_function = g_acm_ss_function;
	}
}

static void acm_free_func(struct usb_function *f)
{
	struct f_acm *acm = NULL;

	if (!f) {
		pr_err("%s:f NULL pointer\n", __func__);
		return;
	}

	acm = func_to_acm(f);
	if (!acm) {
		pr_err("%s:acm NULL pointer\n", __func__);
		return;
	}
	kfree(acm);
	acm = NULL;
}

static struct usb_function *acm_alloc_func(struct usb_function_instance *fi)
{
	struct f_serial_opts *opts = NULL;
	struct f_acm *acm = NULL;

	if (!fi) {
		pr_err("%s:fi NULL pointer\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	acm = kzalloc(sizeof(*acm), GFP_KERNEL);
	if (!acm)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&acm->lock);
	acm->support_notify = HW_ACM_SUPPORT_NOTIFY;
	if (acm->support_notify) {
		acm->port.connect = acm_connect;
		acm->port.disconnect = acm_disconnect;
		acm->port.notify_state = acm_notify_state;
		acm->port.flow_control = acm_flow_control;
		acm->port.send_break = acm_send_break;
	}

	acm->port.func.name = HW_ACM;
	acm->port.func.strings = g_acm_strings;
	/* descriptors are per-instance copies */
	acm->port.func.bind = acm_bind;
	acm->port.func.set_alt = acm_set_alt;
	acm->port.func.setup = acm_setup;
	acm->port.func.disable = acm_disable;

	opts = container_of(fi, struct f_serial_opts, func_inst);
	if (opts)
		acm->port_num = opts->port_num;
	acm->port.func.unbind = acm_unbind;
	acm->port.func.free_func = acm_free_func;

	/* choose descriptors according to single interface or not */
	acm_set_config_vendor(acm);
	return &acm->port.func;
}

static inline struct f_serial_opts *to_f_serial_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_serial_opts,
		func_inst.group);
}

static void acm_attr_release(struct config_item *item)
{
	struct f_serial_opts *opts = NULL;

	if (!item) {
		pr_err("%s:item NULL pointer\n", __func__);
		return;
	}
	opts = to_f_serial_opts(item);

	if (!opts) {
		pr_err("%s:opts NULL pointer\n", __func__);
		return;
	}
	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations g_acm_item_ops = {
	.release = acm_attr_release,
};

static ssize_t f_acm_port_num_show(struct config_item *item, char *page)
{
	struct f_serial_opts *opts = NULL;

	if (!item || !page) {
		pr_err("%s:item or page NULL pointer\n", __func__);
		return -EINVAL;
	}

	opts = to_f_serial_opts(item);

	if (!opts) {
		pr_err("%s:opts NULL pointer\n", __func__);
		return -EINVAL;
	}
	return sprintf(page, "%u\n", opts->port_num);
}

CONFIGFS_ATTR_RO(f_acm_, port_num);
static struct configfs_attribute *g_acm_attrs[] = {
	&f_acm_attr_port_num,
	NULL,
};

static struct config_item_type g_acm_func_type = {
	.ct_item_ops = &g_acm_item_ops,
	.ct_attrs = g_acm_attrs,
	.ct_owner = THIS_MODULE,
};

static void acm_free_instance(struct usb_function_instance *fi)
{
	struct f_serial_opts *opts = NULL;

	if (!fi) {
		pr_err("%s:fi NULL pointer\n", __func__);
		return;
	}

	opts = container_of(fi, struct f_serial_opts, func_inst);
	if (!opts) {
		pr_err("%s:opts NULL pointer\n", __func__);
		return;
	}

	hw_gserial_free_line(opts->port_num);
	kfree(opts);
	opts = NULL;
}

static struct usb_function_instance *acm_alloc_instance(void)
{
	struct f_serial_opts *opts = NULL;
	int ret;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = acm_free_instance;
	ret = hw_gserial_alloc_line(&opts->port_num);
	if (ret) {
		kfree(opts);
		return ERR_PTR(ret);
	}
	config_group_init_type_name(&opts->func_inst.group, "",
		&g_acm_func_type);
	return &opts->func_inst;
}
DECLARE_USB_FUNCTION_INIT(hw_acm, acm_alloc_instance, acm_alloc_func);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("huawei usb gadget acm module driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
