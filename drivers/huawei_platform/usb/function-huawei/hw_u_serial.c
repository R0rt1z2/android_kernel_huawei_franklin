/*
 * hw_u_serial.c
 *
 * utilities for USB gadget "serial port"/TTY support driver
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

#include "hw_u_serial.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

/*
 * This component encapsulates the TTY layer glue needed to provide basic
 * "serial port" functionality through the USB gadget stack.  Each such
 * port is exposed through a /dev/ttyGS* node.
 *
 * After this module has been loaded, the individual TTY port can be requested
 * (gserial_alloc_line()) and it will stay available until they are removed
 * (gserial_free_line()). Each one may be connected to a USB function
 * (gserial_connect), or disconnected (with gserial_disconnect) when the USB
 * host issues a config change event. Data can only flow when the port is
 * connected to the host.
 *
 * A given TTY port can be made available in multiple configurations.
 * For example, each one might expose a ttyGS0 node which provides a
 * login application.  In one case that might use CDC ACM interface 0,
 * while another configuration might use interface 3 for that.  The
 * work to handle that (including descriptor management) is not part
 * of this component.
 *
 * Configurations may expose more than one TTY port.  For example, if
 * ttyGS0 provides login service, then ttyGS1 might provide dialer access
 * for a telephone or fax link.  And ttyGS2 might be something that just
 * needs a simple byte stream interface for some messaging protocol that
 * is managed in userspace ... OBEX, PTP, and MTP have been mentioned.
 */

#define PREFIX "ttyHwGS"

/*
 * gserial is the lifecycle interface, used by USB functions
 * gs_port is the I/O nexus, used by the tty driver
 * tty_struct links to the tty/filesystem framework
 *
 */

#define QUEUE_SIZE           16
#define ONCE_WRITE_BUF_SIZE  8192 /* once max TX */
#define WRITE_BUF_SIZE       8192 /* TX only */
#define C_SPEED              9600
#define DW_DTE_RATE          9600
#define FORMAT               8
#define NORMAL_COMP          0
#define PORT_ONE             1
#define GS_DRIVER_NAME       "g_serial"
#define TRUE                 1
#define FALSE                0

/* circular buffer */
struct gs_buf {
	unsigned int buf_size;
	char *buf_buf;
	char *buf_get;
	char *buf_put;
};

/*
 * The port structure holds info for each port, one for each minor number
 * (and thus for each /dev/ node).
 */
struct gs_port {
	struct tty_port port;
	spinlock_t port_lock; /* guard port_* access */

	struct gserial *port_usb;

	bool openclose; /* open/close in progress */
	u8 port_num;

	struct list_head read_pool;
	int read_started;
	int read_allocated;
	struct list_head read_queue;
	unsigned int n_read;
	struct tasklet_struct push;

	struct list_head write_pool;
	int write_started;
	int write_allocated;
	struct gs_buf port_write_buf;
	wait_queue_head_t drain_wait; /* wait while writes drain */
	bool write_busy;
	wait_queue_head_t close_wait;

	/* REVISIT this state ... */
	struct usb_cdc_line_coding port_line_coding; /* 8-N-1 etc */
};

static struct portmaster {
	struct mutex lock; /* protect open/close */
	struct gs_port *port;
} ports[HW_MAX_U_SERIAL_PORTS];

#define GS_CLOSE_TIMEOUT    15 /* seconds */

/* Allocate a circular buffer and all associated memory */
static int gs_buf_alloc(struct gs_buf *gb, unsigned int size)
{
	gb->buf_buf = kmalloc(size, GFP_KERNEL);
	if (!gb->buf_buf)
		return -ENOMEM;

	gb->buf_size = size;
	gb->buf_put = gb->buf_buf;
	gb->buf_get = gb->buf_buf;

	return 0;
}

/* Free the buffer and all associated memory */
static void gs_buf_free(struct gs_buf *gb)
{
	kfree(gb->buf_buf);
	gb->buf_buf = NULL;
	/*
	 * Free the buf_put and buf_get
	 * because three variable point to same memory
	 */
	gb->buf_put = NULL;
	gb->buf_get = NULL;
}

/* Clear out all data in the circular buffer */
static void gs_buf_clear(struct gs_buf *gb)
{
	gb->buf_get = gb->buf_put;
}

/* Return the number of bytes of data written into the circular buffer */
static unsigned int gs_buf_data_avail(struct gs_buf *gb)
{
	return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
}

/* Return the number of bytes of space available in the circular buffer */
static unsigned int gs_buf_space_avail(struct gs_buf *gb)
{
	return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
}

/*
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 */
static unsigned int gs_buf_put(struct gs_buf *gb, const char *buf,
	unsigned int count)
{
	unsigned int len;

	len = gs_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf + len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

/*
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 */
static unsigned int gs_buf_get(struct gs_buf *gb, char *buf,
	unsigned int count)
{
	unsigned int len;

	len = gs_buf_data_avail(gb);
	if (count > len)
		count = len;
	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf + len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else
			gb->buf_get = gb->buf_buf;
	}

	return count;
}

/*
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */
struct usb_request *hw_gs_alloc_req(struct usb_ep *ep, unsigned int len,
	gfp_t kmalloc_flags)
{
	struct usb_request *req = NULL;

	if (!ep)
		return NULL;

	req = usb_ep_alloc_request(ep, kmalloc_flags);
	if (!req)
		return NULL;
	req->length = len;
	req->buf = kmalloc(len, kmalloc_flags);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}
EXPORT_SYMBOL_GPL(hw_gs_alloc_req);

/* Free a usb_request and its buffer */
void hw_gs_free_req(struct usb_ep *ep, struct usb_request *req)
{
	if (!ep || !req)
		return;

	kfree(req->buf);
	req->buf = NULL;
	usb_ep_free_request(ep, req);
}
EXPORT_SYMBOL_GPL(hw_gs_free_req);

/*
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.
 *
 * Called with port_lock held.
 */
static unsigned int gs_send_packet(struct gs_port *port, char *packet,
	unsigned int size)
{
	unsigned int len;

	len = gs_buf_data_avail(&port->port_write_buf);
	if (len < size)
		size = len;
	if (size != 0)
		size = gs_buf_get(&port->port_write_buf, packet, size);
	return size;
}

static void print_debug(struct gs_port *port, struct usb_request *req, int len)
{
	int i;

	if (port->port_num != PORT_ONE)
		return;
	pr_info("[gs]%d:len=%d ", port->port_num, req->length);
	for (i = 0; i <= len; i++)
		pr_info("%c", *((u8 *)req->buf + i));
	pr_info("\n");
}

/*
 * This function finds available write requests, calls
 * gs_send_packet to fill these packets with data, and
 * continues until either there are no more write requests
 * available or no more data to send.  This function is
 * run whenever data arrives or write requests are available
 */
static int gs_start_tx(struct gs_port *port)
{
	struct list_head *pool = NULL;
	struct usb_ep *in = NULL;
	int ret = 0;
	bool do_tty_wake = false;
	struct usb_request *req = NULL;
	int len;

	pool = &port->write_pool;
	in = port->port_usb->in;
	while (!port->write_busy && !list_empty(pool)) {
		if (port->write_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		if (!req || !req->buf || !in)
			break;
		len = gs_send_packet(port, req->buf, ONCE_WRITE_BUF_SIZE);
		if (len == 0) {
			wake_up_interruptible(&port->drain_wait);
			break;
		}
		do_tty_wake = true;
		req->length = len;
		list_del(&req->list);
		req->zero = (gs_buf_data_avail(&port->port_write_buf) == 0);

		/*
		 * Drop lock while we call out of driver; completions
		 * could be issued while we do so.  Disconnection may
		 * happen too; maybe immediately before we queue this!
		 *
		 * NOTE that we may keep sending data for a while after
		 * the TTY closed.
		 */
		port->write_busy = true;
		spin_unlock(&port->port_lock);
		ret = usb_ep_queue(in, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);
		port->write_busy = false;

		if (ret) {
			list_add(&req->list, pool);
			break;
		}
		print_debug(port, req, 5); /* 5 buf continuate */
		port->write_started++;

		/* abort immediately after disconnect */
		if (!port->port_usb)
			break;
	}

	if (do_tty_wake && port->port.tty)
		tty_wakeup(port->port.tty);
	return ret;
}

/* Context: caller owns port_lock, and port_usb is set */
static unsigned int gs_start_rx(struct gs_port *port)
{
	struct list_head *pool = NULL;
	struct usb_ep *out = NULL;
	struct usb_request *req = NULL;
	struct tty_struct *tty = NULL;
	int status = 0;

	if (!port->port_usb)
		return status;
	pool = &port->read_pool;
	out = port->port_usb->out;

	while (!list_empty(pool)) {
		/* no more rx if closed */
		tty = port->port.tty;
		if (!tty || !out)
			break;

		if (port->read_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		if (!req || !req->buf)
			break;
		list_del(&req->list);
		req->length = out->maxpacket;

		/*
		 * drop lock while we call out; the controller driver
		 * may need to call us back (e.g. for disconnect)
		 */
		spin_unlock(&port->port_lock);
		status = usb_ep_queue(out, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);

		if (status) {
			pr_debug("%s: %s %s err %d\n",
				__func__, "queue", out->name, status);
			list_add(&req->list, pool);
			break;
		}
		port->read_started++;
	}
	return (unsigned int)port->read_started;
}

static int push_data_to_tty(struct gs_port *port, struct usb_request *req,
	int *push)
{
	char *pkt = NULL;
	unsigned int size;
	unsigned int n;
	int count;

	if (req->actual == 0)
		return 0;

	if (!req->buf)
		return -EINVAL;
	pkt = req->buf;
	size = req->actual;
	/* we may have pushed part of this packet already... */
	n = port->n_read;
	if (n != 0) {
		pkt += n;
		size -= n;
	}
	count = tty_insert_flip_string(&port->port, pkt, size);
	if (count != 0)
		*push = TRUE;
	if (count != size) {
		/* stop pushing; TTY layer can't handle more */
		port->n_read += count;
		return -EINVAL;
	}
	port->n_read = 0;
	return 0;
}

/*
 * RX tasklet takes data out of the RX queue and hands it up to the TTY
 * layer until it refuses to take any more data (or is throttled back).
 * Then it issues reads for any further data.
 *
 * If the RX queue becomes full enough that no usb_request is queued,
 * the OUT endpoint may begin NAKing as soon as its FIFO fills up.
 * So QUEUE_SIZE packets plus however many the FIFO holds (usually two)
 * can be buffered before the TTY layer's buffers (currently 64 KB).
 */
static void gs_rx_push(unsigned long _port)
{
	struct gs_port *port = (void *)_port;
	struct tty_struct *tty = NULL;
	struct list_head *queue = NULL;
	bool disconnect = false;
	int do_push = FALSE;
	int ret;
	struct usb_request *req = NULL;

	queue = &port->read_queue;
	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	tty = port->port.tty;
	while (!list_empty(queue)) {
		req = list_first_entry(queue, struct usb_request, list);

		/* leave data queued if tty was rx throttled */
		if (!req || (tty && tty_throttled(tty)))
			break;
		if (req->status == -ESHUTDOWN)
			disconnect = true;
		print_debug(port, req, 7); /* continuate print 7 buf */

		/* push data to (open) tty */
		ret = push_data_to_tty(port, req, &do_push);
		if (ret < 0)
			break;
		list_move(&req->list, &port->read_pool);
		port->read_started--;
	}

	/*
	 * Push from tty to ldisc; without low_latency set this is handled by
	 * a workqueue, so we won't get callbacks and can hold port_lock
	 */
	if (do_push == TRUE) /* push from tty */
		tty_flip_buffer_push(&port->port);

	/*
	 * We want our data queue to become empty ASAP, keeping data
	 * in the tty and ldisc (not here).  If we couldn't push any
	 * this time around, there may be trouble unless there's an
	 * implicit tty_unthrottle() call on its way...
	 *
	 * REVISIT we should probably add a timer to keep the tasklet
	 * from starving ... but it's not clear that case ever happens.
	 */
	if (!list_empty(queue) && tty) {
		if ((!tty_throttled(tty)) && (do_push == TRUE))
			tasklet_schedule(&port->push);
	}

	/* If we're still connected, refill the USB RX queue. */
	if (!disconnect && port->port_usb) {
		ret = gs_start_rx(port);
		if (ret < 0)
			pr_debug("start rx failed\n");
	}

	spin_unlock_irq(&port->port_lock);
}

static void gs_read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port *port = NULL;

	if (!ep || !req)
		return;

	port = ep->driver_data;
	if (!port)
		return;
	/* Queue all received data until the tty layer is ready for it. */
	spin_lock(&port->port_lock);
	list_add_tail(&req->list, &port->read_queue);
	tasklet_schedule(&port->push);
	spin_unlock(&port->port_lock);
}

static void gs_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port *port = NULL;
	int ret;

	if (!ep || !req)
		return;

	port = ep->driver_data;
	if (!port)
		return;

	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);
	port->write_started--;

	switch (req->status) {
	default:
		/* presumably a transient fault */
		pr_warn("%s: unexpected %s status %d\n",
			__func__, ep->name, req->status);
		/* fall-through */
	case NORMAL_COMP:
		/* normal completion */
		if (port->port_usb) {
			ret = gs_start_tx(port);
			if (ret < 0)
				pr_debug("%s:fail\n", __func__);
		}
		break;

	case -ESHUTDOWN:
		/* disconnect */
		pr_debug("%s: %s shutdown\n", __func__, ep->name);
		break;
	}

	spin_unlock(&port->port_lock);
}

static void gs_free_requests(struct usb_ep *ep, struct list_head *head,
	int *allocated)
{
	struct usb_request *req = NULL;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		if (req) {
			list_del(&req->list);
			hw_gs_free_req(ep, req);
		}
		if (allocated)
			(*allocated)--;
	}
}

static int gs_alloc_requests(struct usb_ep *ep, struct list_head *head,
	void (*fn)(struct usb_ep *, struct usb_request *),
	int *allocated)
{
	int i;
	struct usb_request *req = NULL;
	int n;

	n = allocated ? (QUEUE_SIZE - *allocated) : QUEUE_SIZE;

	/*
	 * Pre-allocate up to QUEUE_SIZE transfers, but if we can't
	 * do quite that many this time, don't fail ... we just won't
	 * be as speedy as we might otherwise be.
	 */
	for (i = 0; i < n; i++) {
		req = hw_gs_alloc_req(ep, ONCE_WRITE_BUF_SIZE, GFP_ATOMIC);
		if (!req)
			return list_empty(head) ? -ENOMEM : 0;
		req->complete = fn;
		list_add_tail(&req->list, head);
		if (allocated)
			(*allocated)++;
	}
	return 0;
}

/*
 * We only start I/O when something is connected to both sides of
 * this port.  If nothing is listening on the host side, we may
 * be pointlessly filling up our TX buffers and FIFO.
 */
static int gs_start_io(struct gs_port *port)
{
	struct list_head *head = NULL;
	struct usb_ep *ep = NULL;
	int status;
	unsigned int started;

	pr_info("Enter %s\n", __func__);
	ep = port->port_usb->out;
	head = &port->read_pool;

	if (!ep || !head)
		return -EINVAL;
	/*
	 * Allocate RX and TX I/O buffers.  We can't easily do this much
	 * earlier (with GFP_KERNEL) because the requests are coupled to
	 * endpoints, as are the packet sizes we'll be using.  Different
	 * configurations may use different endpoints with a given port;
	 * and high speed vs full speed changes packet sizes too.
	 */
	status = gs_alloc_requests(ep, head, gs_read_complete,
		&port->read_allocated);
	if (status) {
		pr_err("%s: status is: %d\n", __func__, status);
		return status;
	}

	if (!port->port_usb->in)
		return -EINVAL;

	status = gs_alloc_requests(port->port_usb->in, &port->write_pool,
		gs_write_complete, &port->write_allocated);
	if (status) {
		gs_free_requests(ep, head, &port->read_allocated);
		return status;
	}

	/* queue read requests */
	port->n_read = 0;
	started = gs_start_rx(port);

	/* unblock any pending writes into our circular buffer */
	if (started) {
		if (port->port.tty)
			tty_wakeup(port->port.tty);
	} else {
		gs_free_requests(ep, head, &port->read_allocated);
		gs_free_requests(port->port_usb->in, &port->write_pool,
			&port->write_allocated);
		status = -EIO;
	}
	pr_info("Exit %s\n", __func__);
	return status;
}

static int get_port_status(struct gs_port *port)
{
	int status;
	/* already open */
	if (port->port.count) {
		status = 0;
		port->port.count++;
	/* currently opening/closing wait ... */
	} else if (port->openclose) {
		status = -EBUSY;
	/* we do the work */
	} else {
		status = -EAGAIN;
		port->openclose = true;
	}
	return status;
}

static int start_io_stream(struct gs_port *port)
{
	struct gserial *gser = NULL;
	int ret;

	gser = port->port_usb;
	if (!gser) {
		pr_err("%s port_usb null, but return 0", __func__);
		return 0;
	}

	pr_debug("gs_open: start ttyGS%d\n", port->port_num);
	ret = gs_start_io(port);
	if (ret < 0)
		pr_err("start io failed\n");

	if (gser->connect)
		gser->connect(gser);
	return 0;
}

/*
 * sets up the link between a gs_port and its associated TTY.
 * That link is broken *only* by TTY close(), and all driver methods
 * know that.
 */
static int gs_open(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port = NULL;
	int status;

	if (!tty || !file)
		return -EINVAL;

	pr_info("Enter %s\n", __func__);
	do {
		mutex_lock(&ports[tty->index].lock);
		port = ports[tty->index].port;
		if (!port) {
			status = -ENODEV;
		} else {
			spin_lock_irq(&port->port_lock);
			status = get_port_status(port);
			spin_unlock_irq(&port->port_lock);
		}
		mutex_unlock(&ports[tty->index].lock);

		if (status == -EBUSY)
			usleep_range(1000, 1100); /* 1000-1100: 1ms */
		else if ((status == -ENODEV) || (status == 0))
			return status;
	} while (status != -EAGAIN);

	/* Do the "real open" */
	spin_lock_irq(&port->port_lock);

	/* allocate circular buffer on first open */
	if (!(port->port_write_buf.buf_buf)) {
		spin_unlock_irq(&port->port_lock);
		status = gs_buf_alloc(&port->port_write_buf, WRITE_BUF_SIZE);
		spin_lock_irq(&port->port_lock);

		if (status) {
			port->openclose = false;
			goto exit_unlock_port;
		}
	}

	/*
	 * REVISIT if REMOVED (ports[].port NULL), abort the open
	 * to let rmmod work faster (but this way isn't wrong)
	 * REVISIT maybe wait for "carrier detect"
	 */
	tty->driver_data = port;
	port->port.tty = tty;
	port->port.count = PORT_ONE;
	port->openclose = false;
	/* if connected, start the I/O stream */
	status = start_io_stream(port);
exit_unlock_port:
	spin_unlock_irq(&port->port_lock);
	return status;
}

static int gs_writes_finished(struct gs_port *p)
{
	int cond;

	/* return true on disconnect or empty buffer */
	spin_lock_irq(&p->port_lock);
	cond = ((p->port_usb == NULL) ||
		!gs_buf_data_avail(&p->port_write_buf));
	spin_unlock_irq(&p->port_lock);

	return cond;
}

static void gs_close(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port = NULL;
	struct gserial *gser = NULL;

	if (!tty || !file)
		return;

	port = tty->driver_data;
	if (!port)
		return;

	spin_lock_irq(&port->port_lock);

	if (port->port.count != PORT_ONE) {
		if (port->port.count == 0)
			WARN_ON(1);
		else
			--port->port.count;
		goto port_count_exit;
	}

	pr_debug("%s: ttyGS%d %p,%p\n", __func__, port->port_num, tty, file);

	/*
	 * mark port as closing but in use; we can drop port lock
	 * and sleep if necessary
	 */
	port->openclose = true;
	port->port.count = 0;

	gser = port->port_usb;
	if (gser && gser->disconnect)
		gser->disconnect(gser);

	/*
	 * wait for circular write buffer to drain, disconnect, or at
	 * most GS_CLOSE_TIMEOUT seconds; then discard the rest
	 */
	if (gs_buf_data_avail(&port->port_write_buf) > 0 && gser) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->drain_wait,
			gs_writes_finished(port), (GS_CLOSE_TIMEOUT * HZ));
		spin_lock_irq(&port->port_lock);
		gser = port->port_usb;
	}

	/*
	 * If we're disconnected, there can be no I/O in flight so it's
	 * ok to free the circular buffer; else just scrub it.  And don't
	 * let the push tasklet fire again until we're re-opened.
	 */
	if (!gser)
		gs_buf_free(&port->port_write_buf);
	else
		gs_buf_clear(&port->port_write_buf);

	port->port.tty = NULL;
	port->openclose = false;
	pr_debug("ttyGS%d %p,%p done\n", port->port_num, tty, file);
	wake_up(&port->close_wait);
port_count_exit:
	spin_unlock_irq(&port->port_lock);
}

static int gs_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int ret;

	if (!tty || !buf)
		return -EINVAL;

	port = tty->driver_data;
	if (!port)
		return -EINVAL;

	pr_debug("%s: ttyGS%d %p writing %d bytes\n", __func__, port->port_num,
		tty, count);

	spin_lock_irqsave(&port->port_lock, flags);
	if (count)
		count = gs_buf_put(&port->port_write_buf, buf, count);
	if (port->port_usb) {
		ret = gs_start_tx(port);
		if (ret < 0)
			pr_debug("%s:fail\n", __func__);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	return count;
}

static int gs_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int status;

	if (!tty)
		return -EINVAL;

	port = tty->driver_data;
	if (!port)
		return -EINVAL;

	pr_debug("%s: %d,%p char=0x%x, called from %pf\n", __func__,
		port->port_num, tty, ch, __builtin_return_address(0));

	spin_lock_irqsave(&port->port_lock, flags);
	status = gs_buf_put(&port->port_write_buf, &ch, PORT_ONE);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;
}

static void gs_flush_chars(struct tty_struct *tty)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int ret;

	if (!tty)
		return;

	port = tty->driver_data;
	if (!port)
		return;

	pr_debug("%s: %d,%p\n", __func__, port->port_num, tty);
	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		ret = gs_start_tx(port);
		if (ret < 0)
			pr_debug("%s:fail\n", __func__);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static int gs_write_room(struct tty_struct *tty)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int room = 0;

	if (!tty)
		return room;

	port = tty->driver_data;
	if (!port)
		return room;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		room = gs_buf_space_avail(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_debug("%s: %d,%p room=%d\n", __func__,
		port->port_num, tty, room);

	return room;
}

static int gs_chars_in_buffer(struct tty_struct *tty)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int chars;

	if (!tty)
		return -EINVAL;

	port = tty->driver_data;
	if (!port)
		return -EINVAL;

	spin_lock_irqsave(&port->port_lock, flags);
	chars = gs_buf_data_avail(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_debug("%s: %d,%p chars=%d\n", __func__,
		port->port_num, tty, chars);

	return chars;
}

/* undo side effects of setting TTY_THROTTLED */
static void gs_unthrottle(struct tty_struct *tty)
{
	struct gs_port *port = NULL;
	unsigned long flags;

	if (!tty)
		return;

	port = tty->driver_data;
	if (!port)
		return;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		/*
		 * Kickstart read queue processing.  We don't do xon/xoff,
		 * rts/cts, or other handshaking with the host, but if the
		 * read queue backs up enough we'll be NAKing OUT packets.
		 */
		tasklet_schedule(&port->push);
		pr_debug("%d: unthrottle\n", port->port_num);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static int gs_break_ctl(struct tty_struct *tty, int duration)
{
	struct gs_port *port = NULL;
	int ret = 0;
	struct gserial *gser = NULL;

	if (!tty)
		return -EINVAL;

	port = tty->driver_data;
	if (!port)
		return -EINVAL;

	pr_debug("%s: ttyGS%d, send break %d\n", __func__,
		port->port_num, duration);

	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (gser && gser->send_break)
		ret = gser->send_break(gser, duration);
	spin_unlock_irq(&port->port_lock);

	return ret;
}

static const struct tty_operations g_gs_tty_ops = {
	.open = gs_open,
	.close = gs_close,
	.write = gs_write,
	.put_char = gs_put_char,
	.flush_chars = gs_flush_chars,
	.write_room = gs_write_room,
	.chars_in_buffer = gs_chars_in_buffer,
	.unthrottle = gs_unthrottle,
	.break_ctl = gs_break_ctl,
};

static struct tty_driver *g_gs_tty_driver;

static int gs_port_alloc(unsigned int port_num,
	struct usb_cdc_line_coding *coding)
{
	struct gs_port *port = NULL;
	int ret = 0;

	mutex_lock(&ports[port_num].lock);
	if (ports[port_num].port) {
		ret = -EBUSY;
		goto port_busy;
	}

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	tty_port_init(&port->port);
	spin_lock_init(&port->port_lock);
	init_waitqueue_head(&port->drain_wait);
	init_waitqueue_head(&port->close_wait);
	tasklet_init(&port->push, gs_rx_push, (unsigned long) port);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_LIST_HEAD(&port->write_pool);

	port->port_num = port_num;
	port->port_line_coding = *coding;
	ports[port_num].port = port;

port_busy:
alloc_failed:
	mutex_unlock(&ports[port_num].lock);
	return ret;
}

static int gs_closed(struct gs_port *port)
{
	int cond;

	spin_lock_irq(&port->port_lock);
	cond = ((port->port.count == 0) && !(port->openclose));
	spin_unlock_irq(&port->port_lock);

	return cond;
}

static void gserial_free_port(struct gs_port *port)
{
	tasklet_kill(&port->push);
	/* wait for old opens to finish */
	wait_event(port->close_wait, gs_closed(port));
	WARN_ON(port->port_usb != NULL);
	tty_port_destroy(&port->port);
	kfree(port);
	port = NULL;
}

void hw_gserial_free_line(unsigned char port_num)
{
	struct gs_port *port = NULL;

	mutex_lock(&ports[port_num].lock);
	if (WARN_ON(!ports[port_num].port)) {
		mutex_unlock(&ports[port_num].lock);
		return;
	}
	port = ports[port_num].port;
	ports[port_num].port = NULL;
	mutex_unlock(&ports[port_num].lock);

	if (port)
		gserial_free_port(port);
	if (g_gs_tty_driver)
		tty_unregister_device(g_gs_tty_driver, port_num);
}
EXPORT_SYMBOL_GPL(hw_gserial_free_line);

int hw_gserial_alloc_line(unsigned char *line_num)
{
	struct usb_cdc_line_coding coding;
	struct device *tty_dev = NULL;
	struct gs_port *port = NULL;

	int ret;
	unsigned int port_num;

	if (!line_num || !g_gs_tty_driver)
		return -EINVAL;

	coding.dwDTERate = cpu_to_le32(DW_DTE_RATE);
	coding.bCharFormat = FORMAT;
	coding.bParityType = USB_CDC_NO_PARITY;
	coding.bDataBits = USB_CDC_1_STOP_BITS;

	for (port_num = 0; port_num < HW_MAX_U_SERIAL_PORTS; port_num++) {
		ret = gs_port_alloc(port_num, &coding);
		if (ret == -EBUSY)
			continue;
		else if (ret != 0)
			return ret;

		break;
	}
	if (ret == -EBUSY)
		return ret;

	/* sysfs class devices, so mdev/udev make /dev/ttyGS* */
	tty_dev = tty_port_register_device(&ports[port_num].port->port,
		g_gs_tty_driver, port_num, NULL);
	if (IS_ERR(tty_dev)) {
		pr_err("%s: failed to register tty for port %d, err %ld\n",
			__func__, port_num, PTR_ERR(tty_dev));

		ret = PTR_ERR(tty_dev);
		port = ports[port_num].port;
		ports[port_num].port = NULL; /* [false alarm]:fortify */
		if (port)
			gserial_free_port(port);
		goto dev_err;
	}
	*line_num = port_num;
dev_err:
	return ret;
}
EXPORT_SYMBOL_GPL(hw_gserial_alloc_line);

/*
 * This is called activate endpoints and let the TTY layer know that
 * the connection is active ... not unlike "carrier detect".  It won't
 * necessarily start I/O queues; unless the TTY is held open by any
 * task, there would be no point.  However, the endpoints will be
 * activated so the USB host can perform I/O, subject to basic USB
 * hardware flow control.
 *
 * Caller needs to have set up the endpoints and USB function in @dev
 * before calling this, as well as the appropriate (speed-specific)
 * endpoint descriptors, and also have allocate @port_num by calling
 * @gserial_alloc_line().
 *
 */
int hw_gserial_connect(struct gserial *gser, u8 port_num)
{
	struct gs_port *port = NULL;
	unsigned long flags;
	int status;

	if (!gser || !gser->in || !gser->out)
		return -EINVAL;

	if (port_num >= HW_MAX_U_SERIAL_PORTS)
		return -ENXIO;

	port = ports[port_num].port;
	if (!port)
		return -EINVAL;

	if (port->port_usb) {
		pr_err("serial line %u is in use\n", port_num);
		return -EBUSY;
	}

	/* activate the endpoints */
	status = usb_ep_enable(gser->in);
	if (status < 0)
		return status;
	gser->in->driver_data = port;
	status = usb_ep_enable(gser->out);
	if (status < 0)
		goto fail_out;
	gser->out->driver_data = port;

	/* then tell the tty glue that I/O can work */
	spin_lock_irqsave(&port->port_lock, flags);
	gser->ioport = port;
	port->port_usb = gser;

	/*
	 * REVISIT unclear how best to handle this state...
	 * we don't really couple it with the Linux TTY.
	 */
	gser->port_line_coding = port->port_line_coding;

	/*
	 * if it's already open, start I/O ... and notify the serial
	 * protocol about open/close status (connect/disconnect).
	 */
	if (port->port.count) {
		pr_debug("gserial_connect: start ttyGS%d\n", port->port_num);
		gs_start_io(port);
		if (gser->connect)
			gser->connect(gser);
	} else {
		if (gser->disconnect)
			gser->disconnect(gser);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
	return status;

fail_out:
	usb_ep_disable(gser->in);
	return status;
}
EXPORT_SYMBOL_GPL(hw_gserial_connect);

/*
 * This is called to deactivate endpoints and let the TTY layer know
 * that the connection went inactive ... not unlike "hangup"
 */
void hw_gserial_disconnect(struct gserial *gser)
{
	struct gs_port *port = NULL;
	unsigned long flags;

	if (!gser)
		return;

	port = gser->ioport;
	if (!port || !(gser->out) || !(gser->in))
		return;

	/* tell the TTY glue not to do I/O here any more */
	spin_lock_irqsave(&port->port_lock, flags);

	/* REVISIT as above: how best to track this? */
	port->port_line_coding = gser->port_line_coding;
	port->port_usb = NULL;
	gser->ioport = NULL;
	if ((port->port.count > 0) || (port->openclose)) {
		wake_up_interruptible(&port->drain_wait);
		if (port->port.tty)
			tty_hangup(port->port.tty);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	/* disable endpoints, aborting down any active I/O */
	usb_ep_disable(gser->out);
	usb_ep_disable(gser->in);

	/* finally, free any unused/unusable I/O buffers */
	spin_lock_irqsave(&port->port_lock, flags);
	if ((port->port.count == 0) && !(port->openclose))
		gs_buf_free(&port->port_write_buf);
	gs_free_requests(gser->out, &port->read_pool, NULL);
	gs_free_requests(gser->out, &port->read_queue, NULL);
	gs_free_requests(gser->in, &port->write_pool, NULL);

	port->read_allocated = 0;
	port->read_started = 0;
	port->write_allocated = 0;
	port->write_started = 0;

	spin_unlock_irqrestore(&port->port_lock, flags);
}
EXPORT_SYMBOL_GPL(hw_gserial_disconnect);

static int userial_init(void)
{
	unsigned int i;
	int status;

	g_gs_tty_driver = alloc_tty_driver(HW_MAX_U_SERIAL_PORTS);
	if (!g_gs_tty_driver)
		return -ENOMEM;

	g_gs_tty_driver->driver_name = GS_DRIVER_NAME;
	g_gs_tty_driver->name = PREFIX;

	/* uses dynamically assigned dev_t values */
	g_gs_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	g_gs_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	g_gs_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	g_gs_tty_driver->init_termios = tty_std_termios;

	/*
	 * 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
	 * MS-Windows.  Otherwise, most of these flags shouldn't affect
	 * anything unless we were to actually hook up to a serial line.
	 */
	g_gs_tty_driver->init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	g_gs_tty_driver->init_termios.c_ispeed = C_SPEED;
	g_gs_tty_driver->init_termios.c_ospeed = C_SPEED;

	tty_set_operations(g_gs_tty_driver, &g_gs_tty_ops);
	for (i = 0; i < HW_MAX_U_SERIAL_PORTS; i++)
		mutex_init(&ports[i].lock);

	/* export the driver ... */
	status = tty_register_driver(g_gs_tty_driver);
	if (status) {
		pr_err("%s: cannot register, err %d\n",
			__func__, status);
		goto sts_fail;
	}

	pr_debug("%s: registered %d ttyGS* device\n", __func__,
		HW_MAX_U_SERIAL_PORTS);

	return status;
sts_fail:
	put_tty_driver(g_gs_tty_driver);
	g_gs_tty_driver = NULL;
	return status;
}
module_init(userial_init);

static void userial_cleanup(void)
{
	if (!g_gs_tty_driver)
		return;

	tty_unregister_driver(g_gs_tty_driver);
	put_tty_driver(g_gs_tty_driver);
	g_gs_tty_driver = NULL;
}
module_exit(userial_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("huawei usb gadget serial_port module driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
