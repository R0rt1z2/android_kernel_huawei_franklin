/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: FG-awared MQ IO scheduler.
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/sbitmap.h>
#include <linux/blk-cgroup.h>
#include <linux/version.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-debugfs.h"
#include "blk-mq-tag.h"
#include "blk-mq-sched.h"

#define VIP_IDLE_EXPIRED	(HZ / 5)
#define FMQ_REQ_EXPIRED	(HZ / 2)
#define FMQ_MIN_LIMIT		1

enum queue_type {
	FMQ_QUEUE_VIP,
	FMQ_QUEUE_FG,
	FMQ_QUEUE_KBG,
	FMQ_QUEUE_SBG,
	FMQ_QUEUE_BG,
	FMQ_QUEUE_ASYNC,
	FMQ_QUEUE_NR
};

enum queue_type blkcg_maps[BLK_THROTL_TYPE_NR] = {
	[BLK_THROTL_TA]		= FMQ_QUEUE_FG,
	[BLK_THROTL_FG]		= FMQ_QUEUE_FG,
	[BLK_THROTL_KBG]	= FMQ_QUEUE_KBG,
	[BLK_THROTL_SBG]	= FMQ_QUEUE_SBG,
	[BLK_THROTL_BG]		= FMQ_QUEUE_BG,
};

struct fmq_io_stat {
	/*
	 * These are debug data so we did not add any locks or use atomic
	 * variables. Just for some kind of hint.
	 */
	unsigned long	all_cnt;
	unsigned long	max_wait;
	unsigned long	max_inq;
	unsigned long	wait1;
	unsigned long	wait2;
	unsigned long	wait3;
	unsigned long	wait4;
};

struct fmq_queue {
	struct list_head	fifo_list;
	spinlock_t		lock;
	atomic_t		inlist;
	atomic_t		inflight;
	int			limit;
	struct fmq_io_stat	stat;
	u64			fifo_time;
	enum queue_type		qtype;
	struct fmq_data		*fd;
	struct request		*last;
};

struct fmq_limits {
	int vip_busy_fg_limit;
	int vip_fg_busy_bg_limit;
	int vip_fg_idle_bg_limit;
	int vip_idle_fg_limit;
	int vip_fg_max_inflight;
	int kbg_min_inflight;
};

struct fmq_data {
	/*
	 * run time data
	 */
	struct fmq_queue queue[FMQ_QUEUE_NR];	/* Different queues */
	bool		vip_busy;		/* is vip busy */
	unsigned long	vip_idle_expire;	/* vip idle expire time */

	bool		vip_fg_busy;		/* is vip & fg busy */
	unsigned long	vip_fg_idle_expire;	/* vip & fg idle expire time */

	atomic_t	inlist;			/* all rqs in our queues */
	atomic_t	inflight;		/* all rqs inflight */

	spinlock_t	lock;			/* lock for the dispatch queue */
	struct list_head dispatch;		/* dispatch queue */
	struct request_queue *q;		/* request queue link */

	atomic_t	tm_cnt;			/* time out count */
	bool		is_last_timeout;	/* is the last rq timeout*/
	struct fmq_limits limits;
};

typedef bool (*fmq_dispatch_throttle)(struct fmq_queue *fq, int busy);

static enum queue_type fmq_get_queue_type(struct bio *bio)
{
	enum queue_type qtype = FMQ_QUEUE_VIP;
	struct blkcg *blkcg = NULL;

	if (!bio)
		return qtype;

#ifdef CONFIG_HUAWEI_QOS_BLKIO
	if (bio->bi_opf & REQ_VIP)
		return FMQ_QUEUE_VIP;
#endif
	if (bio->bi_opf & REQ_META)
		return FMQ_QUEUE_FG;

	if (!op_is_sync(bio->bi_opf))
		return FMQ_QUEUE_ASYNC;

	rcu_read_lock();
	blkcg = bio_blkcg(bio);
	if (!blkcg->css.parent || blkcg->type < BLK_THROTL_TA ||
			blkcg->type >= BLK_THROTL_TYPE_NR)
		qtype = FMQ_QUEUE_FG;
	else
		qtype = blkcg_maps[blkcg->type];
	rcu_read_unlock();

	return qtype;
}

/*
 * We provide this function to smartIO to get number of request
 * issued into driver but not finish.
 */
static int fmq_get_queue_inflight(struct request_queue *q, enum queue_type type)
{
	struct fmq_data *fd = q->elevator->elevator_data;
	struct fmq_queue *fq = NULL;

	if (type >= FMQ_QUEUE_NR)
		return -EINVAL;

	fq = &fd->queue[type];
	return atomic_read(&fq->inflight);
}

/*
 * Since flush request will pass io scheduler and add into
 * hctx->dispatch list directly, fd->inflight is not accurate.
 * At any time, there is only one flush request can get driver tag.
 */
static inline int fmq_get_busy_driver_tag(struct blk_mq_hw_ctx *hctx)
{
	struct fmq_data *fd = hctx->queue->elevator->elevator_data;
	struct request *flush_req = hctx->fq->flush_rq;

	return atomic_read(&fd->inflight) + ((flush_req->tag != -1) ? 1 : 0);
}

/*
 * Control the hw queue depth of foreground process if the vip process is busy.
 * After the vip queue idle for VIP_IDLE_EXPIRED, restore the queue
 * depth of foreground process.
 */
static void fmq_fg_control(struct fmq_data *fd)
{
	struct fmq_queue *vq = &fd->queue[FMQ_QUEUE_VIP];
	struct fmq_queue *fq = &fd->queue[FMQ_QUEUE_FG];

	if (fd->vip_busy) {
		if (atomic_read(&vq->inflight) == 0) {
			fd->vip_busy = false;
			fd->vip_idle_expire = jiffies + VIP_IDLE_EXPIRED;
		}
		return;
	}

	if (atomic_read(&vq->inflight) > 0) {
		fd->vip_busy = true;
		fq->limit = fd->limits.vip_busy_fg_limit;
	} else if (fq->limit != fd->limits.vip_idle_fg_limit) {
		if (time_after(jiffies, fd->vip_idle_expire))
			fq->limit = fd->limits.vip_idle_fg_limit;
	}
}

static void fmq_set_bg_limit(struct fmq_data *fd, int limit)
{
	struct fmq_queue *fq = NULL;
	int i;

	for (i = FMQ_QUEUE_KBG; i < FMQ_QUEUE_NR; i++) {
		fq = &fd->queue[i];
		fq->limit = limit;
	}
}

/*
 * Control the hw queue depth of background process if the vip or foreground
 * process is busy. After the vip queue idle for VIP_IDLE_EXPIRED, restore
 * the queue depth of background process.
 */
static void fmq_bg_control(struct fmq_data *fd)
{
	struct fmq_queue *vq = &fd->queue[FMQ_QUEUE_VIP];
	struct fmq_queue *fq = &fd->queue[FMQ_QUEUE_FG];

#define VIP_FG_INFLIGHT(vq, fq) \
	(atomic_read(&(vq)->inflight) + atomic_read(&(fq)->inflight))

	if (fd->vip_fg_busy) {
		if (VIP_FG_INFLIGHT(vq, fq) == 0) {
			fd->vip_fg_busy = false;
			fd->vip_fg_idle_expire = jiffies + VIP_IDLE_EXPIRED;
		}
		return;
	}

	if (VIP_FG_INFLIGHT(vq, fq) > 0) {
		fd->vip_fg_busy = true;
		fmq_set_bg_limit(fd, fd->limits.vip_fg_busy_bg_limit);
	} else if (time_after(jiffies, fd->vip_fg_idle_expire)) {
		fmq_set_bg_limit(fd, fd->limits.vip_fg_idle_bg_limit);
	}
}

static inline void fmq_queue_control(struct fmq_data *fd)
{
	fmq_fg_control(fd);
	fmq_bg_control(fd);
}

static void fmq_queue_inflight_inc(struct fmq_queue *fq)
{
	atomic_inc(&fq->inflight);
	fmq_queue_control(fq->fd);
}

static void fmq_queue_inflight_dec(struct fmq_queue *fq)
{
	atomic_dec(&fq->inflight);
	fmq_queue_control(fq->fd);
}

static void fmq_prepare_request(struct request *rq, struct bio *bio)
{
	enum queue_type rq_type;

	/* Passthrough requests alloc rq before bio */
	if (bio) {
		rq_type = fmq_get_queue_type(bio);
		rq->elv.priv[0] = (void *)(long)rq_type;
	} else {
		rq->elv.priv[0] = (void *)FMQ_QUEUE_NR;
	}
}

static struct fmq_queue *fmq_close_request(struct request *rq)
{
	struct fmq_data *fd = rq->q->elevator->elevator_data;
	enum queue_type rq_type = (enum queue_type)rq->elv.priv[0];
	struct fmq_queue *fq = NULL;

	if (!(rq->rq_flags & RQF_STARTED))
		return fq;

	rq->rq_flags &= ~RQF_STARTED;
	atomic_dec(&fd->inflight);

	if (rq_type >= FMQ_QUEUE_NR)
		return fq;

	fq = &fd->queue[rq_type];
	fmq_queue_inflight_dec(fq);

	return fq;
}

static unsigned long fmq_request_delay(struct request *rq)
{
	unsigned long delay;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	delay = div_u64(ktime_get_ns() - rq->start_time_ns, NSEC_PER_MSEC);
#else
	delay = jiffies_to_msecs(jiffies - rq->start_time);;
#endif
	return delay;
}

static void fmq_finish_request(struct request *rq)
{
	struct fmq_queue *fq = fmq_close_request(rq);
	unsigned long delay = fmq_request_delay(rq);
	struct fmq_data *fd = rq->q->elevator->elevator_data;

	if (!fq)
		return;

	/*
	* This is only for debugging so we do not add any locks. That is
	 * to say, the data may not be trusty. When the counter is overflow,
	 * data may be not right.
	 */
	if (fq->stat.max_wait < delay)
		fq->stat.max_wait = delay;
	if (delay < 15)
		fq->stat.wait1++;
	else if (delay < 30)
		fq->stat.wait2++;
	else if (delay < 100)
		fq->stat.wait3++;
	else
		fq->stat.wait4++;

	fq->stat.all_cnt++;

	if (atomic_read(&fd->inflight) == 0 && atomic_read(&fd->inlist) > 0)
		blk_mq_run_hw_queues(rq->q, true);
}

static void fmq_started_request(struct request *rq)
{
	struct fmq_data *fd = rq->q->elevator->elevator_data;
	enum queue_type rq_type = (enum queue_type)rq->elv.priv[0];
	unsigned long delay = fmq_request_delay(rq);
	struct fmq_queue *fq = NULL;

	if (rq_type >= FMQ_QUEUE_NR)
		return;

	fq = &fd->queue[rq_type];

	if (fq->stat.max_inq < delay)
		fq->stat.max_inq = delay;
}

static void fmq_requeue_request(struct request *rq)
{
	fmq_close_request(rq);
}

static void fmq_insert_request(struct blk_mq_hw_ctx *hctx, struct request *rq,
			      bool at_head)
{
	enum queue_type rq_type = (enum queue_type)rq->elv.priv[0];
	struct request_queue *q = hctx->queue;
	struct fmq_data *fd = q->elevator->elevator_data;
	struct fmq_queue *fq = NULL;

	blk_mq_sched_request_inserted(rq);

	if (at_head || blk_rq_is_passthrough(rq)) {
		spin_lock(&fd->lock);
		if (at_head)
			list_add(&rq->queuelist, &fd->dispatch);
		else
			list_add_tail(&rq->queuelist, &fd->dispatch);
		spin_unlock(&fd->lock);

		atomic_inc(&fd->inlist);
		return;
	}

	if (rq_type >= FMQ_QUEUE_NR) {
		WARN_ON(1);
		return;
	}

	/*
	 * set expire time and add to different type of queue
	 */
	rq->fifo_time = jiffies + FMQ_REQ_EXPIRED;

	fq = &fd->queue[rq_type];

	spin_lock(&fq->lock);
	list_add_tail(&rq->queuelist, &fq->fifo_list);
	fq->last = rq;

	if (atomic_read(&fq->inlist) == 0 ||
		time_after((unsigned long)fq->fifo_time,
			(unsigned long)rq->fifo_time))
		fq->fifo_time = rq->fifo_time;

	spin_unlock(&fq->lock);

	atomic_inc(&fq->inlist);
	atomic_inc(&fd->inlist);
}

static void fmq_insert_requests(struct blk_mq_hw_ctx *hctx,
				struct list_head *list, bool at_head)
{
	struct request_queue *q = hctx->queue;
	struct fmq_data *fd = q->elevator->elevator_data;

	while (!list_empty(list)) {
		struct request *rq;

		rq = list_first_entry(list, struct request, queuelist);
		list_del_init(&rq->queuelist);
		fmq_insert_request(hctx, rq, at_head);
	}

	if (atomic_read(&fd->queue[FMQ_QUEUE_VIP].inlist) > 0 ||
	    atomic_read(&fd->queue[FMQ_QUEUE_FG].inlist) > 0)
		blk_mq_run_hw_queue(hctx, false);
}

static void fmq_start_request(struct fmq_data *fd, struct request *rq)
{
	atomic_dec(&fd->inlist);
	atomic_inc(&fd->inflight);
	rq->rq_flags |= RQF_STARTED;
}

static struct request *fmq_dispatch_passthrough(struct fmq_data *fd)
{
	struct request *rq = NULL;

	if (list_empty_careful(&fd->dispatch))
		return NULL;

	spin_lock(&fd->lock);

	if (list_empty(&fd->dispatch)) {
		spin_unlock(&fd->lock);
		return NULL;
	}

	rq = list_first_entry(&fd->dispatch, struct request, queuelist);
	list_del_init(&rq->queuelist);

	fmq_start_request(fd, rq);

	spin_unlock(&fd->lock);
	return rq;
}

static struct request *fmq_remove_request(struct fmq_queue *fq, bool timeout)
{
	struct request *rq = NULL;
	struct request *next = NULL;

	spin_lock(&fq->lock);

	if (list_empty(&fq->fifo_list)) {
		spin_unlock(&fq->lock);
		return NULL;
	}

	rq = list_first_entry(&fq->fifo_list, struct request, queuelist);

	/* Don't timeout actually */
	if (timeout && time_is_after_jiffies((unsigned long)rq->fifo_time)) {
		if (time_after((unsigned long)fq->fifo_time,
			       (unsigned long)rq->fifo_time))
			fq->fifo_time = rq->fifo_time;
		spin_unlock(&fq->lock);
		return NULL;
	}

	list_del_init(&rq->queuelist);
	if (rq == fq->last)
		fq->last = NULL;

	/*
	 * When delete request from fifo_list, we also need to
	 * update fq->fifo_time.
	 */
	if (!list_empty(&fq->fifo_list)) {
		next = list_first_entry(&fq->fifo_list, struct request, queuelist);
		fq->fifo_time = next->fifo_time;
	}

	fmq_queue_inflight_inc(fq);
	atomic_dec(&fq->inlist);

	if (timeout) {
		fq->fd->is_last_timeout = true;
		atomic_inc(&fq->fd->tm_cnt);
	} else
		fq->fd->is_last_timeout = false;

	fmq_start_request(fq->fd, rq);

	spin_unlock(&fq->lock);

	return rq;
}

bool fmq_foreground_throttle(struct fmq_queue *fq, int busy)
{
	struct fmq_data *fd = fq->fd;
	int fg_busy;

	fg_busy = atomic_read(&fd->queue[FMQ_QUEUE_VIP].inflight) +
		      atomic_read(&fd->queue[FMQ_QUEUE_FG].inflight);

	if (fg_busy >= fd->limits.vip_fg_max_inflight)
		return true;

	return busy >= fq->limit;
}

bool fmq_kbackground_throttle(struct fmq_queue *fq, int busy)
{
	if (atomic_read(&fq->inflight) < fq->fd->limits.kbg_min_inflight)
		return false;

	return busy >= fq->limit;
}

bool fmq_generic_throttle(struct fmq_queue *fq, int busy)
{
	return busy >= fq->limit;
}

static fmq_dispatch_throttle fmq_throttle[FMQ_QUEUE_NR] = {
	fmq_foreground_throttle,
	fmq_foreground_throttle,
	fmq_kbackground_throttle,
	fmq_generic_throttle,
	fmq_generic_throttle,
	fmq_generic_throttle,
};

static struct request *fmq_dispatch_queue(struct blk_mq_hw_ctx *hctx, int i)
{
	struct fmq_data *fd = hctx->queue->elevator->elevator_data;
	struct fmq_queue *fq = &fd->queue[i];
	int busy = fmq_get_busy_driver_tag(hctx);

	if (fmq_throttle[i](fq, busy))
		return NULL;

	return fmq_remove_request(fq, false);
}

static struct request *fmq_dispatch_timeout(struct fmq_data *fd)
{
	struct request *rq = NULL;
	int i;

	/*
	 * To avoid timeout requests congest the hardware queue, we dispatch
	 * timeout requests and normal requests alternately.
	 */
	if (fd->is_last_timeout)
		return NULL;

	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		struct fmq_queue *fq = &fd->queue[i];
		if (atomic_read(&fq->inlist) == 0)
			continue;
		if (time_is_after_jiffies((unsigned long)fq->fifo_time))
			continue;

		rq = fmq_remove_request(fq, true);
		if (rq)
			return rq;
	}

	return NULL;
}

static struct request *fmq_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
	struct fmq_data *fd = hctx->queue->elevator->elevator_data;
	struct request *rq = NULL;
	int i;

	rq = fmq_dispatch_passthrough(fd);
	if (rq)
		goto out;

	rq = fmq_dispatch_timeout(fd);
	if (rq)
		goto out;

	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		rq = fmq_dispatch_queue(hctx, i);
		if (rq)
			break;
	}
out:
	return rq;
}

static bool fmq_has_work(struct blk_mq_hw_ctx *hctx)
{
	struct fmq_data *fd = hctx->queue->elevator->elevator_data;

	return atomic_read(&fd->inlist) > 0;
}

static void fmq_limit_depth(unsigned int op, struct blk_mq_alloc_data *data)
{
	enum queue_type rq_type = fmq_get_queue_type(data->bio);
	struct blk_mq_tags *tags = data->hctx->sched_tags;
	unsigned int depth = 1U << tags->bitmap_tags.sb.shift;

	/*
	 * We don't limit FMQ_QUEUE_VIP
	 * For nr_requests = 64: shift = 4, depth = 16, map_nr = 4.
	 * For nr_requests = 32: shift = 3, depth = 8, map_nr = 4.
	 */
	switch (rq_type) {
	case FMQ_QUEUE_FG:
		data->shallow_depth = depth * 7 / 8;
		break;
	case FMQ_QUEUE_KBG:
		data->shallow_depth = depth * 3 / 4;
		break;
	case FMQ_QUEUE_SBG:
	case FMQ_QUEUE_BG:
		data->shallow_depth = depth * 5 / 8;
		break;
	case FMQ_QUEUE_ASYNC:
		data->shallow_depth = depth / 2;
		break;
	default:
		break;
	}
}

static bool fmq_try_merge_request(struct request_queue *q, struct request *rq,
		struct bio *bio)
{
	bool merged = false;

	if (!elv_bio_merge_ok(rq, bio))
		return false;

	switch (blk_try_merge(rq, bio)) {
	case ELEVATOR_BACK_MERGE:
		merged = bio_attempt_back_merge(q, rq, bio);
		break;
	case ELEVATOR_FRONT_MERGE:
		merged = bio_attempt_front_merge(q, rq, bio);
		break;
	case ELEVATOR_DISCARD_MERGE:
		merged = bio_attempt_discard_merge(q, rq, bio);
		break;
	default:
		break;
	}

	return merged;
}

static bool fmq_bio_merge(struct blk_mq_hw_ctx *hctx, struct bio *bio)
{
	struct request_queue *q = hctx->queue;
	struct fmq_data *fd = q->elevator->elevator_data;
	enum queue_type rq_type = fmq_get_queue_type(bio);
	struct fmq_queue *fq = &fd->queue[rq_type];
	bool ret;

	spin_lock(&fq->lock);
	if (!fq->last) {
		spin_unlock(&fq->lock);
		return false;
	}
	ret = fmq_try_merge_request(q, fq->last, bio);
	spin_unlock(&fq->lock);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
static int fmq_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int index)
{
	struct blk_mq_tags *tags = hctx->sched_tags;
	unsigned int depth = 1U << tags->bitmap_tags.sb.shift;

	/*
	 * The default value of min_shallow_depth is UINT_MAX. Here,
	 * we try to set min_shallow_depth as depth / 2 to avoid warning
	 * in __sbitmap_queue_get_shallow().
	 */
	sbitmap_queue_min_shallow_depth(&tags->bitmap_tags, depth / 2);
	return 0;
}
#endif

static void fmq_init_limits(struct request_queue *q, struct fmq_data *fd)
{
	struct blk_mq_hw_ctx *hctx = NULL;
	int hw_tags = 0;
	int i;

	/*
	 * If 'q->nr_hw_queues > 1', whole fmq control may be not accurate.
	 */
	queue_for_each_hw_ctx(q, hctx, i)
		hw_tags += hctx->tags->bitmap_tags.sb.depth;

	fd->limits.vip_busy_fg_limit	= max(hw_tags / 4, FMQ_MIN_LIMIT);
	fd->limits.vip_fg_busy_bg_limit	= max(hw_tags / 8, FMQ_MIN_LIMIT);
	fd->limits.vip_fg_idle_bg_limit	= max(hw_tags / 2, FMQ_MIN_LIMIT);
	fd->limits.vip_idle_fg_limit	= max(hw_tags * 3 / 4, FMQ_MIN_LIMIT);
	fd->limits.vip_fg_max_inflight	= max(hw_tags * 15 / 16, FMQ_MIN_LIMIT);
	fd->limits.kbg_min_inflight	= max(hw_tags / 16, FMQ_MIN_LIMIT);

	/* init queue limit */
	fd->queue[FMQ_QUEUE_VIP].limit = fd->limits.vip_fg_max_inflight;
	fd->queue[FMQ_QUEUE_FG].limit = fd->limits.vip_idle_fg_limit;
	for (i = FMQ_QUEUE_KBG; i < FMQ_QUEUE_NR; i++)
		fd->queue[i].limit = fd->limits.vip_fg_idle_bg_limit;
}

static int fmq_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct elevator_queue *eq = NULL;
	struct fmq_data *fd = NULL;
	int i;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	fd = kzalloc_node(sizeof(*fd), GFP_KERNEL, q->node);
	if (!fd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = fd;

	fmq_init_limits(q, fd);
	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		fd->queue[i].qtype = i;
		fd->queue[i].fd = fd;
		INIT_LIST_HEAD(&fd->queue[i].fifo_list);
		spin_lock_init(&fd->queue[i].lock);
	}

	spin_lock_init(&fd->lock);
	INIT_LIST_HEAD(&fd->dispatch);
	fd->q = q;

	q->elevator = eq;
	q->get_inflight_rq_fn = fmq_get_queue_inflight;
	return 0;
}

static void fmq_exit_queue(struct elevator_queue *e)
{
	struct fmq_data *fd = e->elevator_data;

	WARN_ON(atomic_read(&fd->inlist) > 0);
	fd->q->get_inflight_rq_fn = NULL;

	kfree(fd);
}

#ifdef CONFIG_BLK_DEBUG_FS

static int fmq_inlist_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;

	seq_printf(m, "%d\n", atomic_read(&fd->inlist));
	return 0;
}

static int fmq_inflight_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;

	seq_printf(m, "%d\n", atomic_read(&fd->inflight));
	return 0;
}

static int fmq_queue_inflight_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;
	int i;

	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		seq_printf(m, "%d: %d\n", i,
			atomic_read(&fd->queue[i].inflight));
	}

	return 0;
}

static int fmq_queue_inlist_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;
	int i;

	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		seq_printf(m, "%d: %d\n", i,
			atomic_read(&fd->queue[i].inlist));
	}

	return 0;
}

static int fmq_queue_limit_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;
	int i;

	for (i = 0; i < FMQ_QUEUE_NR; i++)
		seq_printf(m, "%d: %d\n", i, fd->queue[i].limit);

	return 0;
}

static int fmq_queue_stat_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;
	struct fmq_io_stat *io_stat;
	int i;

#define PERT_WAIT(wait, all)	((all) == 0 ? 0UL : ((wait) * 100UL / (all)))

	for (i = 0; i < FMQ_QUEUE_NR; i++) {
		io_stat = &fd->queue[i].stat;
		seq_printf(m, "%d : max_wait:  %lu\n", i, io_stat->max_wait);
		seq_printf(m, "  : max_inq:   %lu\n", io_stat->max_inq);
		seq_printf(m, "  : (<15  ):   %lu(%lu%%)\n", io_stat->wait1,
			PERT_WAIT(io_stat->wait1, io_stat->all_cnt));
		seq_printf(m, "  : (<30  ):   %lu(%lu%%)\n", io_stat->wait2,
			PERT_WAIT(io_stat->wait2, io_stat->all_cnt));
		seq_printf(m, "  : (<100 ):   %lu(%lu%%)\n", io_stat->wait3,
			PERT_WAIT(io_stat->wait3, io_stat->all_cnt));
		seq_printf(m, "  : (>100 ):   %lu(%lu%%)\n", io_stat->wait4,
			PERT_WAIT(io_stat->wait4, io_stat->all_cnt));
		seq_printf(m, "  : all_cnt:   %lu\n", io_stat->all_cnt);
	}

	return 0;
}

static ssize_t fmq_queue_stat_store(void *data, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;
	int i;

	for (i = 0; i < FMQ_QUEUE_NR; i++)
		memset(&fd->queue[i].stat, 0, sizeof(fd->queue[i].stat));

	return count;
}

static int fmq_busy_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;

	seq_printf(m, "vip_busy: %u\n", fd->vip_busy);
	seq_printf(m, "vip_idle_expire: %lu\n", fd->vip_idle_expire);
	seq_printf(m, "vip_fg_busy: %u\n", fd->vip_fg_busy);
	seq_printf(m, "vip_fg_idle_expire: %lu\n", fd->vip_fg_idle_expire);
	seq_printf(m, "jiffies: %lu\n", jiffies);

	return 0;
}

static int fmq_time_out_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct fmq_data *fd = q->elevator->elevator_data;

	seq_printf(m, "%d\n", atomic_read(&fd->tm_cnt));

	return 0;
}

static const struct blk_mq_debugfs_attr fmq_queue_debugfs_attrs[] = {
	{"queue_inflight", 0400, fmq_queue_inflight_show},
	{"queue_inlist",   0400, fmq_queue_inlist_show},
	{"queue_limit",    0400, fmq_queue_limit_show},
	{"queue_stat",     0600, fmq_queue_stat_show, fmq_queue_stat_store},
	{"busy",           0400, fmq_busy_show},
	{"inflight",       0400, fmq_inflight_show},
	{"inlist",         0400, fmq_inlist_show},
	{"timeout",        0400, fmq_time_out_show},
	{},
};

#endif // CONFIG_BLK_DEBUG_FS

static struct elevator_type fmq_iosched = {
	.ops.mq = {
		.insert_requests	= fmq_insert_requests,
		.dispatch_request	= fmq_dispatch_request,
		.has_work		= fmq_has_work,
		.init_sched		= fmq_init_queue,
		.exit_sched		= fmq_exit_queue,
		.limit_depth		= fmq_limit_depth,
		.prepare_request	= fmq_prepare_request,
		.finish_request		= fmq_finish_request,
		.requeue_request	= fmq_requeue_request,
		.bio_merge		= fmq_bio_merge,
		.started_request	= fmq_started_request,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
		.init_hctx		= fmq_init_hctx,
#endif
	},

	.uses_mq	= true,
#ifdef CONFIG_BLK_DEBUG_FS
	.queue_debugfs_attrs = fmq_queue_debugfs_attrs,
#endif
	.elevator_name	= "fmq",
	.elevator_owner	= THIS_MODULE,
};
MODULE_ALIAS("fmq-iosched");

static int __init fmq_init(void)
{
	return elv_register(&fmq_iosched);
}

static void __exit fmq_exit(void)
{
	elv_unregister(&fmq_iosched);
}

module_init(fmq_init);
module_exit(fmq_exit);

MODULE_AUTHOR("Jason Yan <yanaijie@huawei.com>");
MODULE_AUTHOR("Yufen Yu <yuyufen@huawei.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FG-awared MQ IO scheduler");
