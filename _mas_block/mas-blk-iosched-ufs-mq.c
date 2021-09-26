/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: MAS MQ ioscheduler
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[BLK-IO]" fmt
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/writeback.h>
#include <linux/mm_inline.h>
#include <linux/mpage.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <trace/events/block.h>
#include <linux/rbtree.h>
#include <linux/types.h>
#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-tag.h"
#include "blk-wbt.h"
#include "mas-blk.h"
#include "mas-blk-iosched-interface.h"
#include "mas-blk-mq-tag.h"
#include "mas-blk-flush-interface.h"
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
#include "mas-blk-ft.h"
#endif

#define MAS_UFS_MQ_PLUG_MERGE_ENABLE 1
#define MAS_BLK_CP_LVL1_MS 100
#define MAS_BLK_CP_LVL2_MS 500

#define MAS_UFS_MQ_PLUG_MERGE_MAX_SIZE (512 * 1024)

#define MAS_MQ_SYNC_DISPATCH_LIMIT 28

#define ASYNC_DISP_LMT_HI_PRESSURE 16
#define ASYNC_DISP_LMT_SB1 2
#define ASYNC_DISP_LMT_SB2 1
#define ASYNC_DISP_LMT_SB1_FWB 3
#define ASYNC_DISP_LMT_SB2_FWB 1

#ifdef CONFIG_MAS_BLK_BW_OPTIMIZE
#define ASYNC_DISP_LMT 14
#define ASYNC_DISP_LMT_SB_LONG 14
#define ASYNC_DISP_LMT_SB_LONG_HP 14
#define SB_CHECK_PERIOD_MS 5
#define SB_LEVEL1_HOLD_MS 5
#define SB_LONG_HOLD_MS 5
#else
#define ASYNC_DISP_LMT 12
#define ASYNC_DISP_LMT_SB_LONG 8
#define ASYNC_DISP_LMT_SB_LONG_HP 12
/* SB short for sync burst */
#define SB_CHECK_PERIOD_MS 10
#define SB_LEVEL1_HOLD_MS 30
#define SB_LONG_HOLD_MS 20
#endif

#define SB_LONG_DUR_MS 5000
#define IO_HW_PENDING_THRESH 8

#define MAS_MQ_IO_JAM_MS 2000

#define MAS_BLK_IO_GUARD_PERIOD_MS 2000

#define MAS_BLK_TURBO_CHECK_PERIOD_MS 500
#define ASYNC_THROT_STAGE_ONE 0
#define ASYNC_STAGE_ONE_LIMIT 5
#define ASYNC_THROT_STAGE_TWO 1
#define ASYNC_STAGE_TWO_LIMIT 20
#define ASYNC_WRITE_CHECK_MS 1000

enum mas_blk_mq_sync_burst_level {
	MAS_UFS_MQ_SB_NORMAL = 0,
	MAS_UFS_MQ_SB_LEVEL1,
	MAS_UFS_MQ_SB_LEVEL2,
	MAS_UFS_MQ_SB_LONG,
};

enum mas_ufs_mq_async_dispatch_mode {
	/* async io will be limited by a fixed value */
	MAS_UFS_MQ_ASYNC_NORMAL_MODE = 0,
	/* async io will be limited by sync io */
	MAS_UFS_MQ_ASYNC_FWB_MODE,
	/* async io will be dispatched a.s.a.p */
	MAS_UFS_MQ_ASYNC_LM_MODE,
};

enum mas_ufs_mq_async_sched_type {
	/* async io will be sort by submitted time */
	MAS_UFS_MQ_SCHED_ASYNC_FIFO = 0,
};

struct mas_ufs_mq_work {
	ktime_t last_queue_tm;
	ktime_t last_enter_tm;
	ktime_t last_exit_tm;
	struct delayed_work io_dispatch_work;
	struct request_queue *queue;
};

/*
 * This struct defines all the variable for ufs mq io-scheduler per lld.
 */
struct mas_ufs_sched_ds_lld {
	struct blk_dev_lld *lld;

#define MAS_MQ_DEFAULT_CP_LIMIT 5
	unsigned int cp_io_limit;
	/* IOs with cp flag in low level driver */
	atomic_t cp_io_inflt_cnt;

	/* MQ tag usage statistic */
	atomic_t mq_tag_used_cnt;
	atomic_t mq_resrvd_tag_used_cnt;
	atomic_t mq_prio_tag_used_cnt;
	/* count of VIP IOs that wait for tags */
	atomic_t vip_wait_cnt;

	/* Sync IOs in low level driver */
	atomic_t sync_io_inflt_cnt;
	/* Async IOs in low level driver */
	atomic_t async_io_inflt_cnt;
	/* VIP IOs in low level driver */
	atomic_t vip_io_inflt_cnt;
	/* FG IOs in low level driver */
	atomic_t fg_io_inflt_cnt;

	spinlock_t sync_disp_lock;
	/* List for all requeued high prio IOs */
	struct list_head hp_sync_disp_list;
	/* IO counts in high_prio_sync_dispatch_list */
	atomic_t hp_io_list_cnt;

	/* List for all requeued sync IOs */
	struct list_head sync_disp_list;
	/* IO counts in sync_dispatch_list */
	atomic_t sync_io_list_cnt;

	spinlock_t async_disp_lock;
	bool async_io_sched_inited;
	struct list_head async_fifo_list;
	/* IO counts in async_dispatch_list */
	atomic_t async_io_list_cnt;

	/* Max sync IOs allowed */
	int sync_io_inflt_lmt;
	/* Max async IOs allowed */
	int async_io_inflt_lmt;
	/* Submit time for latest sync io */
	ktime_t last_sync_io_submit_tm;
	/* Burst mode trigger time */
	ktime_t last_sync_burst_trig_tm;
	/* Complete time for latest sync io */
	ktime_t last_sync_io_compl_tm;
	/* Complete time for latest async io */
	ktime_t last_async_io_compl_tm;
	/* Last async inflight limit update time */
	ktime_t last_async_io_inflt_lmt_update_tm;
	struct timer_list sb_dispatch_check_timer;
	/* Status for sync burst state machine */
	atomic_t sb_dispatch_level;
	/* Dispatch mode for async io */
	enum mas_ufs_mq_async_dispatch_mode async_dispatch_mode;
	struct timer_list write_throttle_check_timer;
	spinlock_t async_limit_update_lock;
	bool async_limit_update_enable;

#ifdef CONFIG_MAS_QOS_MQ
	atomic_t per_cpu_io_inflt[NR_CPUS];
#endif
	atomic_t ref_cnt;

	struct timer_list turbo_timer;
	struct delayed_work datasync_work;
	bool turbo_mode;




};

struct mas_ufs_mq_sched {
	struct request_queue *q;
	/* The work will dispatch all the requeue sync IOs */
	struct mas_ufs_mq_work sync_dispatch_work;
	/* The work will dispatch all the requeue async IOs */
	struct mas_ufs_mq_work async_dispatch_work;
	struct list_head io_guard_list_node;
	struct mas_ufs_sched_ds_lld *sched_ds_lld;
};

/*
 * This struct defines all the variable for ufs mq async io-scheduler.
 */
struct mas_ufs_mq_async_sched {
	enum mas_ufs_mq_async_sched_type type;
	/* async io sched init interface */
	void (*async_sched_init_fn)(struct mas_ufs_sched_ds_lld *ds_lld);
	/* async io insert request interface */
	void (*async_sched_insert_fn)(
		struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld);
	/* async io seek request interface */
	struct request *(*async_sched_seek_fn)(
		const struct mas_ufs_sched_ds_lld *ds_lld);
	/* async io requeue request interface */
	void (*async_sched_requeue_fn)(
		struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld);
	/* async io request merge interface */
	bool (*async_sched_attempt_merge_fn)(
		struct bio *bio, struct request_queue *q);
	/* interface to query async io sched empty or not */
	bool (*async_sched_is_empty_fn)(const struct mas_ufs_sched_ds_lld *ds_lld);
};

struct mas_ufs_mq_priv {
	struct mas_ufs_mq_async_sched *async_io_sched_strategy;
};

DECLARE_PER_CPU(struct list_head, blk_cpu_done);

static struct workqueue_struct *mas_blk_mq_async_disp_wq;
static struct workqueue_struct *mas_blk_mq_sync_disp_wq;
static struct workqueue_struct *mas_blk_io_guard_wq;
static LIST_HEAD(mas_io_guard_queue_list);
static DEFINE_SPINLOCK(io_guard_queue_list_lock);
static struct delayed_work mas_io_guard_work;

static struct request_queue *mas_blk_get_queue_by_lld(struct blk_dev_lld *lld)
{
	struct blk_queue_tag *tag = NULL;
	struct blk_mq_tag_set *tag_set = NULL;

	switch (lld->type) {
	case BLK_LLD_QUEUE_BASE:
		return (struct request_queue *)(lld->data);
	case BLK_LLD_QUEUE_TAG_BASE:
		tag = (struct blk_queue_tag *)(lld->data);
		return list_last_entry(
			&tag->tag_list, struct request_queue, tag_set_list);
	case BLK_LLD_TAGSET_BASE:
		tag_set = (struct blk_mq_tag_set *)(lld->data);
		return list_last_entry(
			&tag_set->tag_list, struct request_queue, tag_set_list);
	default:
		mas_blk_rdr_panic("Unknown lld type");
		return NULL;
	}
	return NULL;
}

static struct mas_ufs_sched_ds_lld *get_sched_ds_lld(
	const struct request_queue *q)
{
	void *queuedata = q->mas_queue.cust_queuedata;

	return queuedata ? ((struct mas_ufs_mq_sched *)queuedata)->sched_ds_lld
		       : NULL;
}

void ufs_mq_inc_vip_wait_cnt(struct blk_mq_alloc_data *data)
{
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(data->q);

	if (unlikely(!ds_lld))
		return;

	atomic_inc(&ds_lld->vip_wait_cnt);
}

void ufs_mq_dec_vip_wait_cnt(struct blk_mq_alloc_data *data)
{
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(data->q);

	atomic_dec_if_positive(&ds_lld->vip_wait_cnt);
}

void reset_vip_wait_cnt(struct blk_mq_alloc_data *data)
{
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(data->q);

	if (atomic_read(&ds_lld->mq_prio_tag_used_cnt))
		return;

	atomic_set(&ds_lld->vip_wait_cnt, 0);
}

int ufs_mq_vip_tag_wait_cnt(struct blk_mq_alloc_data *data)
{
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(data->q);

	return atomic_read(&ds_lld->vip_wait_cnt);
}

static unsigned int ufs_tagset_get_tag(const struct blk_mq_alloc_data *data)
{
	unsigned int tag;

	tag = ufs_tagset_bt_get((struct blk_mq_alloc_data *)data,
		&data->hctx->tags->bitmap_tags, data->hctx);
	if (likely(tag != BLK_MQ_TAG_FAIL))
		atomic_inc(&(get_sched_ds_lld(data->q)->mq_tag_used_cnt));

	return tag;
}

static unsigned int ufs_tagset_get_reserved_tag(
	const struct blk_mq_alloc_data *data)
{
	unsigned int tag;

	if (unlikely(!data->hctx->tags->nr_reserved_tags)) {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		mas_blk_rdr_panic("nr_reserved_tags is zero!");
#else
		return BLK_MQ_TAG_FAIL;
#endif
	}

	tag = ufs_tagset_bt_get((struct blk_mq_alloc_data *)data,
		&data->hctx->tags->breserved_tags, data->hctx);
	if (likely(tag != BLK_MQ_TAG_FAIL)) {
		atomic_inc(
			&(get_sched_ds_lld(data->q)->mq_resrvd_tag_used_cnt));
		tag += data->hctx->tags->reserved_tags_id_offset;
	}

	return tag;
}

static unsigned int ufs_tagset_get_high_prio_tag(
	const struct blk_mq_alloc_data *data)
{
	unsigned int tag;

	if (unlikely(!data->hctx->tags->nr_high_prio_tags)) {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		mas_blk_rdr_panic("nr_high_prio_tags is zero!");
#else
		return BLK_MQ_TAG_FAIL;
#endif
	}

	tag = ufs_tagset_bt_get((struct blk_mq_alloc_data *)data,
		&data->hctx->tags->highprio_tags, data->hctx);
	if (likely(tag != BLK_MQ_TAG_FAIL)) {
		atomic_inc(&(get_sched_ds_lld(data->q)->mq_prio_tag_used_cnt));
		tag += data->hctx->tags->high_prio_tags_id_offset;
	}
	return tag;
}

int get_mq_all_tag_used(struct request_queue *q)
{
	struct mas_ufs_sched_ds_lld *sched_ds_lld = NULL;
	struct blk_dev_lld *lld = NULL;

	if (!q)
		return 0;
	lld = mas_blk_get_lld(q);
	if (!lld)
		return 0;
	sched_ds_lld = (struct mas_ufs_sched_ds_lld *)lld->sched_ds_lld;
	if (!sched_ds_lld)
		return 0;
	return atomic_read(&sched_ds_lld->mq_prio_tag_used_cnt) +
	       atomic_read(&sched_ds_lld->mq_tag_used_cnt) +
	       atomic_read(&sched_ds_lld->mq_resrvd_tag_used_cnt);
}

int get_mq_prio_tag_used(struct request_queue *q)
{
	struct mas_ufs_sched_ds_lld *sched_ds_lld = NULL;
	struct blk_dev_lld *lld = NULL;

	if (!q)
		return 0;
	lld = mas_blk_get_lld(q);
	if (!lld)
		return 0;
	sched_ds_lld = (struct mas_ufs_sched_ds_lld *)lld->sched_ds_lld;
	if (!sched_ds_lld)
		return 0;
	return atomic_read(&sched_ds_lld->mq_prio_tag_used_cnt);
}

unsigned int ufs_mq_tag_get(const struct blk_mq_alloc_data *data)
{
	unsigned long flag = data->io_flag & (REQ_SYNC | REQ_FG_META);
	unsigned int tag;

	if (likely(flag == REQ_SYNC))
		tag = ufs_tagset_get_tag(data);
	else if (flag & REQ_FG_META)
		tag = ufs_tagset_get_high_prio_tag(data);
	else
		tag = ufs_tagset_get_reserved_tag(data);

	return tag;
}

int ufs_mq_tag_put(
	const struct blk_mq_hw_ctx *hctx, unsigned int tag,
	const struct request *rq)
{
	struct blk_mq_tags *tags = hctx->tags;

	if (unlikely(tag >= tags->high_prio_tags_id_offset)) {
		sbitmap_queue_clear(&tags->highprio_tags,
			(tag - tags->high_prio_tags_id_offset),
			rq->mas_req.mq_ctx_generate->cpu);
		atomic_dec(
			&(get_sched_ds_lld(hctx->queue)->mq_prio_tag_used_cnt));
	} else if (unlikely(tag >= tags->reserved_tags_id_offset)) {
		sbitmap_queue_clear(&tags->breserved_tags,
			(tag - tags->reserved_tags_id_offset),
			rq->mas_req.mq_ctx_generate->cpu);
		atomic_dec(&(
			get_sched_ds_lld(hctx->queue)->mq_resrvd_tag_used_cnt));
	} else {
		sbitmap_queue_clear(&tags->bitmap_tags, tag,
			rq->mas_req.mq_ctx_generate->cpu);
		atomic_dec(&(get_sched_ds_lld(hctx->queue)->mq_tag_used_cnt));
	}

	return 0;
}

static inline bool io_submit_intrvl_l_than(
	const struct mas_ufs_sched_ds_lld *ds_lld, const ktime_t base,
	const u64 ms)
{
	return ktime_before(
		base, ktime_add_ms(ds_lld->last_sync_io_submit_tm, ms));
}

static inline bool io_submit_intrvl_g_than(
	const struct mas_ufs_sched_ds_lld *ds_lld, const ktime_t base,
	const u64 ms)
{
	return ktime_after(
		base, ktime_add_ms(ds_lld->last_sync_io_submit_tm, ms));
}

static inline bool last_sync_io_compl_g_than(
	const struct mas_ufs_sched_ds_lld *ds_lld, const ktime_t base,
	const u64 ms)
{
	return ktime_after(
		base, ktime_add_ms(ds_lld->last_sync_io_compl_tm, ms));
}

static inline bool last_async_io_compl_g_than(
	const struct mas_ufs_sched_ds_lld *ds_lld, const ktime_t base,
	const u64 ms)
{
	return ktime_after(
		base, ktime_add_ms(ds_lld->last_async_io_compl_tm, ms));
}

static inline bool burst_mode_dur_g_than(
	const struct mas_ufs_sched_ds_lld *ds_lld, const ktime_t base,
	const u64 ms)
{
	return ktime_after(
		base, ktime_add_ms(ds_lld->last_sync_burst_trig_tm, ms));
}

static void ufs_mq_burst_mode_calc_at_sync_io_continue(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_LEVEL1:
		atomic_set(&ds_lld->sb_dispatch_level, MAS_UFS_MQ_SB_LEVEL2);
		break;
	case MAS_UFS_MQ_SB_LONG:
		if (!atomic_read(&(ds_lld->mq_resrvd_tag_used_cnt)))
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_LEVEL1);
		break;
	default:
		break;
	}
}

static bool ufs_mq_burst_mode_calc_at_sync_io_pause(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	bool burst_mode = true;
	ktime_t now = ktime_get();

	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_LEVEL1:
		if (last_sync_io_compl_g_than(ds_lld, now, SB_LEVEL1_HOLD_MS)) {
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_NORMAL);
			burst_mode = false;
		}
		break;
	case MAS_UFS_MQ_SB_LEVEL2:
		if (io_submit_intrvl_g_than(ds_lld, now, SB_CHECK_PERIOD_MS))
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_LEVEL1);
		break;
	case MAS_UFS_MQ_SB_LONG:
		if (last_sync_io_compl_g_than(ds_lld, now, SB_LONG_HOLD_MS)) {
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_NORMAL);
			burst_mode = false;
		}
		break;
	default:
		break;
	}

	return burst_mode;
}

void ufs_mq_sync_burst_check_timer_expire(unsigned long data)
{
	bool restart_timer = false;
	ktime_t now_ktime = ktime_get();
	struct mas_ufs_sched_ds_lld *ds_lld =
		(struct mas_ufs_sched_ds_lld *)(uintptr_t)data;

	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_LEVEL1:
	case MAS_UFS_MQ_SB_LEVEL2:
		if (burst_mode_dur_g_than(ds_lld, now_ktime, SB_LONG_DUR_MS))
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_LONG);
		break;
	default:
		break;
	}

	if (io_submit_intrvl_l_than(ds_lld, now_ktime, SB_CHECK_PERIOD_MS)) {
		ufs_mq_burst_mode_calc_at_sync_io_continue(ds_lld);
		restart_timer = true;
	} else {
		restart_timer = ufs_mq_burst_mode_calc_at_sync_io_pause(ds_lld);
	}

	if (restart_timer)
		mod_timer(&ds_lld->sb_dispatch_check_timer,
			jiffies + msecs_to_jiffies(SB_CHECK_PERIOD_MS));
}

static void ufs_mq_sync_burst_dispatch_check(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	ktime_t now_ktime;

	now_ktime = ktime_get();
	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_NORMAL:
		if (likely(io_submit_intrvl_l_than(ds_lld, now_ktime,
			    SB_CHECK_PERIOD_MS))) {
			ds_lld->last_sync_burst_trig_tm = now_ktime;
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_LEVEL1);
			mod_timer(&ds_lld->sb_dispatch_check_timer,
				jiffies + msecs_to_jiffies(SB_CHECK_PERIOD_MS));
		}
		break;
	case MAS_UFS_MQ_SB_LEVEL1:
		if (likely(io_submit_intrvl_l_than(ds_lld, now_ktime,
			    SB_CHECK_PERIOD_MS)))
			atomic_set(&ds_lld->sb_dispatch_level,
				MAS_UFS_MQ_SB_LEVEL2);
		break;
	default:
		break;
	}
	ds_lld->last_sync_io_submit_tm = now_ktime;
}

static inline void ufs_mq_dump_dispatch_work_stat(
	const struct mas_ufs_mq_work *work)
{
	pr_err("lst_queue_t: %lld, lst_in_t: %lld, lst_out_t: %lld\n",
		work->last_queue_tm, work->last_enter_tm, work->last_exit_tm);
}

void ufs_mq_dump_request(const struct request_queue *q, enum blk_dump_scene s)
{
	struct request *pos = NULL;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = NULL;

	if (!sched_ds || !(sched_ds->sched_ds_lld))
		return;

	ds_lld = sched_ds->sched_ds_lld;
	if (!list_empty(&ds_lld->hp_sync_disp_list)) {
		pr_err("hp_sync_disp_list:\n");
		list_for_each_entry(pos, &ds_lld->hp_sync_disp_list, queuelist)
			if (pos->q == q)
				mas_blk_dump_request(pos, s);
	}
	if (!list_empty(&ds_lld->sync_disp_list)) {
		pr_err("sync_disp_list:\n");
		list_for_each_entry(pos, &ds_lld->sync_disp_list, queuelist)
			if (pos->q == q)
				mas_blk_dump_request(pos, s);
		ufs_mq_dump_dispatch_work_stat(&sched_ds->sync_dispatch_work);
	}

	if (!list_empty(&ds_lld->async_fifo_list)) {
		pr_err("async_fifo_list:\n");
		list_for_each_entry(
			pos, &ds_lld->async_fifo_list, async_list)
			if (pos->q == q)
				mas_blk_dump_request(pos, s);
		ufs_mq_dump_dispatch_work_stat(&sched_ds->async_dispatch_work);
	}
}

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
#ifdef CONFIG_MAS_MQ_USING_CP
ssize_t mas_queue_cp_enabled_show(struct request_queue *q, char *page)
{
	unsigned long offset = 0;
	struct blk_dev_lld *lld = mas_blk_get_lld(q);

	offset += snprintf(page, PAGE_SIZE, "cp_enabled: %d\n",
			   lld->features & BLK_LLD_UFS_CP_EN ? 1 : 0);

	return (ssize_t)offset;
}

ssize_t mas_queue_cp_enabled_store(
	struct request_queue *q, const char *page, size_t count)
{
	ssize_t ret;
	unsigned long val;
	struct blk_dev_lld *lld = mas_blk_get_lld(q);

	ret = queue_var_store(&val, page, count);
	if (ret < 0)
		return ret;

	if (val)
		lld->features |= BLK_LLD_UFS_CP_EN;
	else
		lld->features &= ~BLK_LLD_UFS_CP_EN;

	return ret;
}

static int cp_debug_en;
int mas_blk_cp_debug_en(void)
{
	return cp_debug_en;
}

ssize_t mas_queue_cp_debug_en_show(struct request_queue *q, char *page)
{
	unsigned long offset = 0;

	offset += snprintf(page, PAGE_SIZE, "cp_debug_en: %d\n", cp_debug_en);

	return (ssize_t)offset;
}

ssize_t mas_queue_cp_debug_en_store(
	struct request_queue *q, const char *page, size_t count)
{
	ssize_t ret;
	unsigned long val;

	ret = queue_var_store(&val, page, count);
	if (ret >= 0)
		cp_debug_en = (int)val;
	else
		cp_debug_en = 0;

	return (ssize_t)count;
}

ssize_t mas_queue_cp_limit_show(struct request_queue *q, char *page)
{
	unsigned long offset = 0;
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(q);

	offset += snprintf(page, PAGE_SIZE, "cp_io_limit: %u\n",
		ds_lld ? ds_lld->cp_io_limit : 0U);

	return (ssize_t)offset;
}

ssize_t mas_queue_cp_limit_store(
	struct request_queue *q, const char *page, size_t count)
{
	ssize_t ret;
	unsigned long val;
	struct mas_ufs_sched_ds_lld *ds_lld = get_sched_ds_lld(q);

	ret = queue_var_store(&val, page, count);
	if (ret < 0)
		return ret;

	if (ds_lld)
		ds_lld->cp_io_limit = val;

	return ret;
}
#endif /* CONFIG_MAS_MQ_USING_CP */
#endif /* CONFIG_MAS_DEBUG_FS */

#ifdef CONFIG_MAS_MQ_USING_CP
static void ufs_mq_add_cp_flag(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	if (req_cp(rq))
		mas_blk_rdr_panic("rq has cp flag set already!");
#endif

	rq->mas_req.mas_featrue_flag |= CUST_REQ_COMMAND_PRIO;
	rq->cmd_flags |= REQ_CP;
	atomic_inc(&ds_lld->cp_io_inflt_cnt);

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	if (unlikely(cp_debug_en)) {
		pr_err("mas_cp: add cp flag, %s\n", rq->mas_req.task_comm);
		ufs_mq_status_dump(rq->q, BLK_DUMP_WARNING);
	}
#endif
}

static void ufs_mq_remove_cp_flag(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (req_cp(rq)) {
		rq->mas_req.mas_featrue_flag &= ~CUST_REQ_COMMAND_PRIO;
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		if (!atomic_read(&ds_lld->cp_io_inflt_cnt))
			mas_blk_rdr_panic("cp_io_inflt_cnt is zero!");
#endif
		atomic_dec(&ds_lld->cp_io_inflt_cnt);
	}
}

static inline bool ufs_mq_rq_is_fg(const struct request *rq)
{
	return (rq->cmd_flags & REQ_FG_META) && !(rq->cmd_flags & REQ_VIP);
}

#define UFS_MQ_VIP_BURST_THRESHOLD 2
static inline bool ufs_mq_vip_inflt_burst(const struct mas_ufs_sched_ds_lld *ds_lld)
{
	return atomic_read(&ds_lld->vip_io_inflt_cnt) > UFS_MQ_VIP_BURST_THRESHOLD;
}

static inline int ufs_mq_get_hp_inflt(const struct mas_ufs_sched_ds_lld *ds_lld)
{
	return atomic_read(&ds_lld->fg_io_inflt_cnt) + atomic_read(&ds_lld->vip_io_inflt_cnt);
}

static bool ufs_mq_could_add_cp_flag(
	const struct request *rq,
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	struct blk_dev_lld *lld = mas_blk_get_lld(rq->q);
	struct bio *oldest_bio = NULL;
	unsigned long flags;
	ktime_t now_ktime;
	ktime_t oldest_bio_ktime = 0;

	if ((rq->cmd_flags & REQ_VIP) && (req_op(rq) != REQ_OP_WRITE))
		return true;

	if (ufs_mq_rq_is_fg(rq) && ufs_mq_vip_inflt_burst(ds_lld))
		return false;

	if (!atomic_read(&ds_lld->sync_io_inflt_cnt) &&
		!atomic_read(&ds_lld->async_io_inflt_cnt))
		return true;

	if (!atomic_read(&ds_lld->cp_io_inflt_cnt))
		return true;

	if (list_empty(&lld->blk_idle.bio_list))
		return true;

	spin_lock_irqsave(&lld->blk_idle.counted_list_lock, flags);
	if (!list_empty(&lld->blk_idle.bio_list)) {
		oldest_bio = list_first_entry(&lld->blk_idle.bio_list,
			struct bio, cnt_list);
		oldest_bio_ktime = oldest_bio->mas_bio.bio_stage_ktime
				[BIO_PROC_STAGE_SUBMIT];
	}
	spin_unlock_irqrestore(&lld->blk_idle.counted_list_lock, flags);

	now_ktime = ktime_get();
	if (ktime_before(now_ktime,
		    ktime_add_ms(oldest_bio_ktime, MAS_BLK_CP_LVL1_MS)))
		return true;
	else if (ktime_before(now_ktime,
			 ktime_add_ms(oldest_bio_ktime, MAS_BLK_CP_LVL2_MS)) &&
		 ((unsigned int)atomic_read(&ds_lld->cp_io_inflt_cnt) <
			 ds_lld->cp_io_limit))
		return true;

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	if (unlikely(mas_blk_cp_debug_en())) {
		if (ktime_before(now_ktime, ktime_add_ms(oldest_bio_ktime,
						    MAS_BLK_CP_LVL2_MS)))
			pr_err("CP limited to %d!\n", MAS_MQ_DEFAULT_CP_LIMIT);
		else
			pr_err("Forbid CP IO!\n");
	}
#endif

	return false;
}

static inline int mas_cp_enabled(const struct request_queue *q)
{
	return mas_blk_get_lld((struct request_queue *)q)->features &
	       BLK_LLD_UFS_CP_EN;
}

static void ufs_mq_cp_io_limit(
	struct request *rq, const struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (!mas_cp_enabled(rq->q))
		return;

	if (ds_lld->turbo_mode)
		return;

	if (ufs_mq_could_add_cp_flag(rq, ds_lld))
		ufs_mq_add_cp_flag(rq, (struct mas_ufs_sched_ds_lld *)ds_lld);

}
#endif /* CONFIG_MAS_MQ_USING_CP */

static int ufs_mq_sync_io_limit(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	int ret = BLK_STS_OK;

	if (unlikely(rq->cmd_flags & REQ_FG_META)) {
		/* Dispatch unconditionally for high priority sync io */
		if (ufs_mq_rq_is_fg(rq))
			atomic_inc(&ds_lld->fg_io_inflt_cnt);
		else
			atomic_inc(&ds_lld->vip_io_inflt_cnt);

#ifdef CONFIG_MAS_MQ_USING_CP
		ufs_mq_cp_io_limit(rq, ds_lld);
#endif
		goto out;
	}

	if (unlikely((atomic_inc_return(&ds_lld->sync_io_inflt_cnt) >
		     ds_lld->sync_io_inflt_lmt)
			&& (!ds_lld->turbo_mode))) {
		ret = BLK_STS_RESOURCE;
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "sync lmt", MAS_IO_TRACE_LEN);
#endif
		rq->mas_req.requeue_reason = REQ_REQUEUE_IO_SW_LIMIT;
	}
#if defined(CONFIG_MAS_BLK_BW_OPTIMIZE) && defined(CONFIG_MAS_MQ_USING_CP)
	else {
		if (!ds_lld->async_limit_update_enable &&
		    mas_cp_enabled(rq->q))
			ufs_mq_add_cp_flag(rq, ds_lld);
	}
#endif

out:
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	if (req_hoq(rq))
		trace_mas_io(__func__, "use Head Of Queue", MAS_IO_TRACE_LEN);
	if (req_cp(rq))
		trace_mas_io(__func__, "use CP", MAS_IO_TRACE_LEN);
#endif

	return ret;
}

static int ufs_mq_async_io_limit(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (unlikely((atomic_inc_return(&ds_lld->async_io_inflt_cnt) >
			ds_lld->async_io_inflt_lmt) &&
			(!ds_lld->turbo_mode))) {
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "async io limit", MAS_IO_TRACE_LEN);
#endif
		rq->mas_req.requeue_reason = REQ_REQUEUE_IO_SW_LIMIT;
		return BLK_STS_RESOURCE;
	}

	return BLK_STS_OK;
}

static int ufs_mq_fs_io_limit(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (likely(rq->cmd_flags & REQ_SYNC))
		return ufs_mq_sync_io_limit(rq, ds_lld);

	return ufs_mq_async_io_limit(rq, ds_lld);
}

static void ufs_mq_rq_inflt_add(
	const struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (unlikely(blk_rq_is_passthrough((struct request *)rq)))
		return;
	if (likely(rq->cmd_flags & REQ_SYNC)) {
		if (unlikely(rq->cmd_flags & REQ_FG_META))
			if (ufs_mq_rq_is_fg(rq))
				atomic_inc(&ds_lld->fg_io_inflt_cnt);
			else
				atomic_inc(&ds_lld->vip_io_inflt_cnt);
		else
			atomic_inc(&ds_lld->sync_io_inflt_cnt);
	} else {
		atomic_inc(&ds_lld->async_io_inflt_cnt);
	}
}

static void ufs_mq_rq_inflt_update(
	const struct request *rq,
	struct mas_ufs_sched_ds_lld *ds_lld, bool update_complete_time)
{
	if (likely(rq->cmd_flags & REQ_SYNC)) {
		if (unlikely(rq->cmd_flags & REQ_FG_META)) {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
			if (unlikely(!ufs_mq_get_hp_inflt(ds_lld)))
				mas_blk_rdr_panic("high_prio_sync_io is 0!");
#endif
			if (ufs_mq_rq_is_fg(rq))
				atomic_dec(&ds_lld->fg_io_inflt_cnt);
			else
				atomic_dec(&ds_lld->vip_io_inflt_cnt);
#ifdef CONFIG_MAS_MQ_USING_CP
			ufs_mq_remove_cp_flag((struct request *)rq, ds_lld);
#endif
		} else {
#if defined(CONFIG_MAS_BLK_BW_OPTIMIZE) && defined(CONFIG_MAS_MQ_USING_CP)
			ufs_mq_remove_cp_flag((struct request *)rq, ds_lld);
#endif
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
			if (!atomic_read(&ds_lld->sync_io_inflt_cnt))
				mas_blk_rdr_panic("sync_io_inflt_cnt is 0!");
#endif
			atomic_dec(&ds_lld->sync_io_inflt_cnt);
		}
		if (likely(update_complete_time))
			ds_lld->last_sync_io_compl_tm = ktime_get();
	} else {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		if (unlikely(!atomic_read(&ds_lld->async_io_inflt_cnt)))
			mas_blk_rdr_panic("async_io_inflt_cnt is 0!");
#endif
		atomic_dec(&ds_lld->async_io_inflt_cnt);
		if (likely(update_complete_time))
			ds_lld->last_async_io_compl_tm = ktime_get();
	}
}

#ifdef CONFIG_MAS_QOS_MQ
static __always_inline void ufs_mq_map_qos(struct request *rq)
{
	if (rq->cmd_flags & REQ_VIP)
		rq->mas_req.mas_rq_qos = MAS_MQ_QOS_7;
	else if (rq->cmd_flags & REQ_FG_META)
		rq->mas_req.mas_rq_qos = MAS_MQ_QOS_6;
	else if (rq->cmd_flags & REQ_SYNC)
		rq->mas_req.mas_rq_qos = MAS_MQ_QOS_5;
	else
		rq->mas_req.mas_rq_qos = MAS_MQ_QOS_0;
}

static __always_inline void inc_inflt_for_cpu(
	unsigned int cpu, struct mas_ufs_sched_ds_lld *ds_lld)
{
	atomic_inc(&(ds_lld->per_cpu_io_inflt[cpu]));
}

static __always_inline void dec_inflt_for_cpu(
	unsigned int cpu, struct mas_ufs_sched_ds_lld *ds_lld)
{
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	if (unlikely(!atomic_read(&(ds_lld->per_cpu_io_inflt[cpu]))))
		mas_blk_rdr_panic("per_cpu_io_inflt is 0!");
#endif

	atomic_dec(&(ds_lld->per_cpu_io_inflt[cpu]));
}

#define SLOT_PER_CPU 8
#define MAX_BUSY_SLOT_SYNC 7
#define MAX_BUSY_SLOT_FG 8
static __always_inline bool borrow_slot(struct request *rq,
	unsigned int start_cpu, const struct mas_ufs_sched_ds_lld *ds_lld)
{
	unsigned int i;
	int max_busy_slot = MAX_BUSY_SLOT_SYNC;
	unsigned int cpu;

	if (!(rq->cmd_flags & REQ_SYNC))
		return false;
	if (rq->cmd_flags & REQ_FG_META)
		max_busy_slot = MAX_BUSY_SLOT_FG;
	for (i = 0; i < num_possible_cpus(); i++) {
		cpu = (start_cpu + i) % num_possible_cpus();
		if (atomic_read(&(ds_lld->per_cpu_io_inflt[cpu])) <
			max_busy_slot) {
			rq->mas_req.slot_cpu = cpu;
			return true;
		}
	}

	return false;
}

static __always_inline bool ufs_mq_set_slot_cpu(
	struct request *rq, const struct mas_ufs_sched_ds_lld *ds_lld)
{
	unsigned int cpu = rq->mas_req.mq_ctx_generate->cpu;

	if (atomic_read(&(ds_lld->per_cpu_io_inflt[cpu])) < SLOT_PER_CPU) {
		rq->mas_req.slot_cpu = cpu;
		return true;
	} else {
		return borrow_slot(rq, cpu + 1, ds_lld);
	}
}

/* queue_rq with cpu inflight in mind */
static blk_status_t __ufs_mq_queue_rq_internal(
	const struct blk_mq_hw_ctx *hctx, const struct request_queue *q,
	const struct blk_mq_queue_data *bd,
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	blk_status_t ret;

	if (!ufs_mq_set_slot_cpu(bd->rq, ds_lld))
		return BLK_STS_RESOURCE;

	ufs_mq_map_qos(bd->rq);
	inc_inflt_for_cpu(bd->rq->mas_req.slot_cpu,
		(struct mas_ufs_sched_ds_lld *)ds_lld);
	ret = q->mq_ops->queue_rq((struct blk_mq_hw_ctx *)hctx, bd);
	if (ret != BLK_STS_OK)
		dec_inflt_for_cpu(bd->rq->mas_req.slot_cpu,
			(struct mas_ufs_sched_ds_lld *)ds_lld);

	mas_blk_latency_req_check(bd->rq, REQ_PROC_STAGE_MQ_QUEUE_RQ);

	return ret;
}
#else
static inline blk_status_t __ufs_mq_queue_rq_internal(
	const struct blk_mq_hw_ctx *hctx, const struct request_queue *q,
	const struct blk_mq_queue_data *bd,
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	blk_status_t ret;

	ret = q->mq_ops->queue_rq((struct blk_mq_hw_ctx *)hctx, bd);
	mas_blk_latency_req_check(bd->rq, REQ_PROC_STAGE_MQ_QUEUE_RQ);

	return ret;
}
#endif /* CONFIG_MAS_QOS_MQ */

/* for all IO except passthrough */
static int __ufs_mq_queue_rq(
	struct request *rq, const struct blk_mq_hw_ctx *hctx,
	const struct mas_ufs_sched_ds_lld *ds_lld,
	const struct blk_mq_queue_data *bd,
	const struct request_queue *q)
{
	int ret;

	ret = ufs_mq_fs_io_limit(rq, (struct mas_ufs_sched_ds_lld *)ds_lld);
	if (unlikely(ret != BLK_STS_OK))
		goto exit;

#ifdef CONFIG_MAS_BLK_DEBUG
	if (unlikely(mas_blk_ft_mq_queue_rq_redirection(rq, q)))
		return BLK_STS_OK;
#endif
	ret = __ufs_mq_queue_rq_internal(hctx, q, bd, ds_lld);
	if (likely(ret == BLK_STS_OK))
		return ret;

	rq->mas_req.requeue_reason = REQ_REQUEUE_IO_HW_LIMIT;
	if (ufs_mq_get_hp_inflt(ds_lld) +
			atomic_read(&ds_lld->sync_io_inflt_cnt) +
			atomic_read(&ds_lld->async_io_inflt_cnt) <=
		IO_HW_PENDING_THRESH)
		rq->mas_req.requeue_reason = REQ_REQUEUE_IO_HW_PENDING;

exit:
	ufs_mq_rq_inflt_update(
		rq, (struct mas_ufs_sched_ds_lld *)ds_lld, false);
	return ret;
}

/* for sync and async dispatch */
static int ufs_mq_queue_rq(
	struct request *rq, const struct blk_mq_hw_ctx *hctx,
	const struct mas_ufs_sched_ds_lld *ds_lld,
	const struct blk_mq_queue_data *bd,
	const struct request_queue *q)
{
	if (unlikely(blk_rq_is_passthrough(rq))) {
		rq->cmd_flags |= REQ_VIP | REQ_FG | REQ_SYNC;
		return __ufs_mq_queue_rq_internal(hctx, q, bd, ds_lld);
	}
	return __ufs_mq_queue_rq(rq, hctx, ds_lld, bd, q);
}

static void ufs_mq_run_delay_sync_list(
	const struct request_queue *q, unsigned long delay_jiffies)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);

	queue_delayed_work(mas_blk_mq_sync_disp_wq,
		&sched_ds->sync_dispatch_work.io_dispatch_work, delay_jiffies);
	sched_ds->sync_dispatch_work.last_queue_tm = ktime_get();
}

static inline void ufs_mq_run_sync_list(const struct request_queue *q)
{
	ufs_mq_run_delay_sync_list(q, 0);
}

static inline bool ufs_mq_nr_before(unsigned int nr1, unsigned int nr2)
{
	return (signed int)(nr1 - nr2) < 0;
}

static void __add_vip_rq_to_list(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	struct request *pos = NULL;

	if (list_empty(&ds_lld->hp_sync_disp_list)) {
		list_add_tail(&rq->queuelist, &ds_lld->hp_sync_disp_list);
		return;
	}

	list_for_each_entry(pos, &ds_lld->hp_sync_disp_list, queuelist) {
		if (!(pos->cmd_flags & REQ_VIP)) {
			list_add_tail(&rq->queuelist, &pos->queuelist);
			return;
		}
		if (list_is_last(&pos->queuelist, &ds_lld->hp_sync_disp_list)) {
			list_add(&rq->queuelist, &pos->queuelist);
			return;
		}
	}
}

static void ufs_mq_order_insert_sync_list(struct request *rq,
					struct mas_ufs_sched_ds_lld *ds_lld)
{
	struct list_head *target_list = &ds_lld->sync_disp_list;

	if (!list_empty(target_list) && rq->mas_req.make_req_nr) {
		struct request *pos = NULL;

		list_for_each_entry(pos, target_list, queuelist) {
			if (!pos->mas_req.make_req_nr)
				continue;
			if (ufs_mq_nr_before(rq->mas_req.make_req_nr,
					pos->mas_req.make_req_nr)) {
				list_add_tail(&rq->queuelist, &pos->queuelist);
				break;
			}
		}
		if (&pos->queuelist == target_list)
			list_add_tail(&rq->queuelist, &pos->queuelist);
	} else {
		list_add_tail(&rq->queuelist, target_list);
	}
}

static void ufs_mq_insert_sync_list(struct request *rq, struct request_queue *q)
{
	unsigned long flags;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	mas_blk_latency_req_check(
		(struct request *)rq, REQ_PROC_STAGE_MQ_ADDTO_SYNC_LIST);
	spin_lock_irqsave(&ds_lld->sync_disp_lock, flags);
	if (rq->cmd_flags & REQ_VIP) {
		__add_vip_rq_to_list(rq, ds_lld);
		atomic_inc(&ds_lld->hp_io_list_cnt);
	} else if (rq->cmd_flags & REQ_FG_META) {
		list_add_tail(&rq->queuelist, &ds_lld->hp_sync_disp_list);
		atomic_inc(&ds_lld->hp_io_list_cnt);
	} else {
		ufs_mq_order_insert_sync_list(rq, ds_lld);
		atomic_inc(&ds_lld->sync_io_list_cnt);
	}
	spin_unlock_irqrestore(&ds_lld->sync_disp_lock, flags);
}

static void ufs_mq_requeue_sync_list(
	struct request *rq, struct request_queue *q)
{
	ufs_mq_insert_sync_list(rq, q);
	ufs_mq_run_sync_list(q);
}

static bool ufs_mq_bio_is_sync(
	const struct request_queue *q, struct bio *bio,
	unsigned int dispatch_op)
{
	if (dispatch_op == REQ_OP_READ || dispatch_op == REQ_OP_FLUSH ||
		(bio->bi_opf & (REQ_SYNC | REQ_FG_META | REQ_FUA))) {
		struct mas_ufs_mq_sched *sched_ds =
			(struct mas_ufs_mq_sched *)(q->mas_queue
							     .cust_queuedata);

		bio->bi_opf |= REQ_SYNC;
		ufs_mq_sync_burst_dispatch_check(sched_ds->sched_ds_lld);
		return true;
	}

	return false;
}

static inline void ufs_mq_bio_to_request(struct request *rq, struct bio *bio)
{
	blk_init_request_from_bio(rq, bio);
	blk_account_io_start(rq, true);
}

static bool ufs_mq_attempt_merge(
	const struct request_queue *q, const struct bio *bio)
{
	unsigned long flags;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	if (unlikely(bio->bi_opf & REQ_SYNC))
		return false;

	if (likely(q->mas_queue_ops &&
		    q->mas_queue_ops->scheduler_priv)) {
		struct mas_ufs_mq_priv *priv = (struct mas_ufs_mq_priv
				*)(q->mas_queue_ops->scheduler_priv);

		if (priv->async_io_sched_strategy
				->async_sched_attempt_merge_fn) {
			bool merged = false;

			spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
			merged = priv->async_io_sched_strategy
					 ->async_sched_attempt_merge_fn(
						 (struct bio *)bio,
						 (struct request_queue *)q);
			spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
			return merged;
		}
	}

	return false;
}

static bool ufs_mq_merge_queue_io(struct blk_mq_hw_ctx *hctx,
	struct blk_mq_ctx *ctx, struct request *rq, struct bio *bio)
{
	struct request_queue *q = hctx->queue;

	if (!bio_mergeable(bio) ||
		blk_rq_is_scsi(rq)) {
		ufs_mq_bio_to_request(rq, bio);
		spin_lock(&ctx->lock);
insert_rq:
		ufs_mq_req_insert(rq, hctx->queue);

		spin_unlock(&ctx->lock);
		return false;
	}

	spin_lock(&ctx->lock);
	if (!ufs_mq_attempt_merge(q, bio)) {
		ufs_mq_bio_to_request(rq, bio);
		goto insert_rq;
	}
	spin_unlock(&ctx->lock);

	blk_mq_free_request(rq);
	return true;
}

static bool ufs_mq_attempt_plug_merge(struct request_queue *q, struct bio *bio,
	const struct blk_plug *plug, struct request **same_queue_rq)
{
	struct request *rq = NULL;
	bool ret = false;
	const struct list_head *plug_list = &plug->mas_blk_list;

	list_for_each_entry_reverse(rq, plug_list, queuelist) {
		int el_ret;

		if (likely(rq->q == q)) {
			/*
			 * Only blk-mq multiple hardware queues case checks the
			 * rq in the same queue, there should be only one such
			 * rq in a queue
			 */
			if (same_queue_rq)
				*same_queue_rq = rq;
		}

		if (rq->q != q || !blk_rq_merge_ok(rq, bio))
			continue;

		el_ret = blk_try_merge(rq, bio);
		if (el_ret == ELEVATOR_BACK_MERGE) {
			ret = bio_attempt_back_merge(q, rq, bio);
			if (ret)
				break;
		} else if (el_ret == ELEVATOR_FRONT_MERGE) {
			ret = bio_attempt_front_merge(q, rq, bio);
			if (ret)
				break;
		}
	}

	return ret;
}

void ufs_mq_flush_plug_list(struct blk_plug *plug, bool from_schedule)
{
	int ret;
	struct request *rq = NULL;
	struct request_queue *q = NULL;
	struct blk_mq_hw_ctx *hctx = NULL;
	struct blk_mq_queue_data bd = {
		.rq = NULL,
		.last = 1
	};
	struct mas_ufs_mq_sched *sched_ds = NULL;
	LIST_HEAD(list);

	list_splice_init(&plug->mas_blk_list, &list);
	do {
		rq = list_entry_rq(list.next);
		q = rq->q;
		sched_ds = (struct mas_ufs_mq_sched *)(q->mas_queue
							     .cust_queuedata);
		list_del_init(&rq->queuelist);
		bd.rq = rq;
		rq->mq_ctx = blk_mq_get_ctx(q);
		hctx = blk_mq_map_queue(q, (int)rq->mq_ctx->cpu);
		ret = __ufs_mq_queue_rq(
			rq, hctx, sched_ds->sched_ds_lld, &bd, q);
		blk_mq_put_ctx(rq->mq_ctx);
		if (unlikely(ret != BLK_STS_OK)) {
			if (ret == BLK_STS_RESOURCE) {
				__blk_mq_requeue_request(rq);
				ufs_mq_insert_sync_list(rq, q);
				ufs_mq_run_delay_sync_list(q, 0);
			} else {
				blk_mq_end_request(rq, BLK_STS_IOERR);
			}
		}
	} while (!list_empty(&list));
}

static struct blk_plug *ufs_mq_get_current_plug(const struct bio *bio)
{
#ifdef MAS_UFS_MQ_PLUG_MERGE_ENABLE
	if (unlikely(!current->plug))
		return NULL;
	if (bio->bi_iter.bi_size > MAS_UFS_MQ_PLUG_MERGE_MAX_SIZE)
		return NULL;
	if (unlikely((current->plug->flush_plug_list_fn) &&
		     (current->plug->flush_plug_list_fn !=
			     __cfi_ufs_mq_flush_plug_list)))
		return NULL;
	current->plug->flush_plug_list_fn = __cfi_ufs_mq_flush_plug_list;
	return current->plug;
#else
	return NULL;
#endif
}

static struct request *ufs_mq_req_add_pluglist(struct blk_plug *plg,
	struct request *disp_rq, const struct request *sameq_rq)
{
	struct request *rq = NULL;

	if (likely(plg)) {
		if (likely(sameq_rq &&
			    !list_empty(&plg->mas_blk_list))) {
			rq = (struct request *)sameq_rq;
			list_del_init(&rq->queuelist);
		}
		list_add_tail(&disp_rq->queuelist, &plg->mas_blk_list);
		mas_blk_latency_req_check(
			disp_rq, REQ_PROC_STAGE_MQ_ADDTO_PLUGLIST);
	} else {
		rq = disp_rq;
	}

	return rq;
}

static __always_inline bool ufs_mq_make_flush_request(
	unsigned int dispatch_op, struct request *rq, const struct bio *bio)
{
	if (unlikely((dispatch_op == REQ_OP_FLUSH) ||
		     (bio->bi_opf & REQ_PREFLUSH))) {
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "flush+fua", MAS_IO_TRACE_LEN);
#endif
		ufs_mq_bio_to_request(rq, (struct bio *)bio);
		blk_insert_flush(rq);
		return true;
	}
	return false;
}

static void ufs_mq_make_sync_request(struct bio *bio, struct blk_plug *plug,
	struct request *rq, struct request *same_queue_rq,
	const struct blk_mq_alloc_data *alloc_data)
{
	int ret;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(rq->q->mas_queue.cust_queuedata);
	struct blk_mq_queue_data bd;

#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "sync-io", MAS_IO_TRACE_LEN);
#endif
	ufs_mq_bio_to_request(rq, bio);
	bd.rq = ufs_mq_req_add_pluglist(plug, rq, same_queue_rq);
	ret = bd.rq ? __ufs_mq_queue_rq(bd.rq, alloc_data->hctx,
		sched_ds->sched_ds_lld, &bd, rq->q) : BLK_STS_OK;
	blk_mq_put_ctx(alloc_data->ctx);
	if (likely(ret == BLK_STS_OK)) {
		return;
	} else if (ret != BLK_STS_RESOURCE) {
		blk_mq_end_request(bd.rq, BLK_STS_IOERR);
		return;
	}
	__blk_mq_requeue_request(bd.rq);
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "sync io requeue!", MAS_IO_TRACE_LEN);
#endif
	ufs_mq_requeue_sync_list(bd.rq, rq->q);
}

static inline struct blk_plug *ufs_mq_get_plug(
	bool is_sync, const struct bio *bio)
{
	unsigned int is_flush_fua = bio->bi_opf & (REQ_PREFLUSH | REQ_FUA);

	return (is_sync && (!is_flush_fua)) ? ufs_mq_get_current_plug(bio)
					    : NULL;
}

blk_qc_t ufs_mq_make_request(struct request_queue *q, struct bio *bio)
{
	struct blk_mq_alloc_data alloc_data = {.flags = 0};
	struct request *rq = NULL;
	struct request *same_queue_rq = NULL;
	blk_qc_t cookie;
	unsigned int wb_acct = false;
	bool is_sync = ufs_mq_bio_is_sync(q, bio, bio_op(bio));
	struct blk_plug *plug = ufs_mq_get_plug(is_sync, bio);

	if (plug && ufs_mq_attempt_plug_merge(q, bio, plug, &same_queue_rq))
		return BLK_QC_T_NONE;

	wb_acct = wbt_wait(q->rq_wb, bio, NULL);
	mas_blk_latency_bio_check(bio, BIO_PROC_STAGE_WBT);

	rq = blk_mq_get_request(q, bio, bio->bi_opf, &alloc_data);
	if (unlikely(!rq)) {
		__wbt_done(q->rq_wb, wb_acct);
		if (bio->bi_opf & REQ_NOWAIT)
			bio_wouldblock_error(bio);
		return BLK_QC_T_NONE;
	}

	if (req_op(rq) == REQ_OP_WRITE || req_op(rq) == REQ_OP_DISCARD)
		blk_req_set_make_req_nr(rq);

	wbt_track(&rq->issue_stat, wb_acct);

	cookie = request_to_qc_t(alloc_data.hctx, rq);

	if (ufs_mq_make_flush_request(bio_op(bio), rq, bio))
		goto run_queue;

	if (likely(rq->cmd_flags & REQ_SYNC)) {
		ufs_mq_make_sync_request(
			bio, plug, rq, same_queue_rq, &alloc_data);
		goto done;
	}

#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "async-io", MAS_IO_TRACE_LEN);
#endif

	if (!ufs_mq_merge_queue_io(alloc_data.hctx, alloc_data.ctx, rq, bio))
run_queue:
		blk_mq_run_hw_queue(alloc_data.hctx, !is_sync);

	blk_mq_put_ctx(alloc_data.ctx);
done:
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "out", MAS_IO_TRACE_LEN);
#endif
	return cookie;
}

static struct request *ufs_mq_pick_sync_rq(
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (unlikely(!list_empty_careful(&ds_lld->hp_sync_disp_list))) {
		atomic_dec((atomic_t *)&ds_lld->hp_io_list_cnt);
		return list_first_entry(
			&ds_lld->hp_sync_disp_list, struct request, queuelist);
	}
	if (!list_empty_careful(&ds_lld->sync_disp_list)) {
		atomic_dec((atomic_t *)&ds_lld->sync_io_list_cnt);
		return list_first_entry(
			&ds_lld->sync_disp_list, struct request, queuelist);
	}

	return NULL;
}

static void ufs_mq_sync_dispatch(const struct request_queue *q)
{
	unsigned long flags;
	int ret;
	struct request *rq = NULL;
	struct blk_mq_hw_ctx *hctx = NULL;
	struct blk_mq_queue_data bd;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	sched_ds->sync_dispatch_work.last_enter_tm = ktime_get();

#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "-", MAS_IO_TRACE_LEN);
#endif
	do {
		spin_lock_irqsave(&ds_lld->sync_disp_lock, flags);
		rq = ufs_mq_pick_sync_rq(ds_lld);
		if (unlikely(!rq)) {
			spin_unlock_irqrestore(&ds_lld->sync_disp_lock, flags);
			break;
		}
		list_del_init(&rq->queuelist);
		spin_unlock_irqrestore(&ds_lld->sync_disp_lock, flags);

		hctx = blk_mq_map_queue(
			(struct request_queue *)q, (int)rq->mq_ctx->cpu);
		bd.rq = rq;
		ret = ufs_mq_queue_rq(rq, hctx, ds_lld, &bd, q);
		if (likely(ret == BLK_STS_OK)) {
			continue;
		} else if (ret == BLK_STS_RESOURCE) {
			__blk_mq_requeue_request(rq);
			ufs_mq_insert_sync_list(rq, (struct request_queue *)q);
			if (last_sync_io_compl_g_than(
				    ds_lld, ktime_get(), MAS_MQ_IO_JAM_MS) &&
				last_async_io_compl_g_than(
					ds_lld, ktime_get(), MAS_MQ_IO_JAM_MS))
				ufs_mq_run_delay_sync_list(q, 1);
			else
				ufs_mq_run_delay_sync_list(q, 0);
			break;
		}
		blk_mq_end_request(rq, BLK_STS_IOERR);
	} while (1);

	sched_ds->sync_dispatch_work.last_exit_tm = ktime_get();
}

void ufs_mq_sync_io_dispatch_work_fn(const struct work_struct *work)
{
	struct mas_ufs_mq_work *mq_work = container_of(
		work, struct mas_ufs_mq_work, io_dispatch_work.work);

	ufs_mq_sync_dispatch(mq_work->queue);
}

static void ufs_mq_async_sched_fifo_init(struct mas_ufs_sched_ds_lld *ds_lld)
{
	if (!ds_lld->async_io_sched_inited) {
		INIT_LIST_HEAD(&ds_lld->async_fifo_list);
		ds_lld->async_io_sched_inited = 1;
	}
}

static void ufs_mq_async_sched_fifo_insert(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	list_add_tail(&rq->async_list, &ds_lld->async_fifo_list);
}

static struct request *ufs_mq_async_sched_fifo_seek(
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	struct request *cur_req = NULL;

	if (!list_empty_careful(&ds_lld->async_fifo_list)) {
		cur_req = list_first_entry(&ds_lld->async_fifo_list,
			struct request, async_list);
		list_del_init(&cur_req->async_list);
		return cur_req;
	}
	return NULL;
}

static void ufs_mq_async_sched_fifo_requeue(
	struct request *rq, struct mas_ufs_sched_ds_lld *ds_lld)
{
	list_add(&rq->async_list, &ds_lld->async_fifo_list);
}

static bool ufs_mq_async_sched_fifo_attempt_merge_bio(
	struct bio *bio, struct request_queue *q)
{
	struct request *rq = NULL;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	list_for_each_entry_reverse(rq, &ds_lld->async_fifo_list, async_list) {
		int el_ret;

		if (!blk_rq_merge_ok(rq, bio))
			continue;
		el_ret = blk_try_merge(rq, bio);
		if (el_ret == ELEVATOR_BACK_MERGE) {
			if (bio_attempt_back_merge(q, rq, bio))
				return true;
		} else if (el_ret == ELEVATOR_FRONT_MERGE) {
			if (bio_attempt_front_merge(q, rq, bio))
				return true;
		}
	}

	return false;
}

static inline bool ufs_mq_async_sched_fifo_empty(
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	return list_empty(&ds_lld->async_fifo_list);
}

static struct mas_ufs_mq_async_sched mas_ufs_mq_async_io_fifo_sched = {
	.type = MAS_UFS_MQ_SCHED_ASYNC_FIFO,
	.async_sched_init_fn = ufs_mq_async_sched_fifo_init,
	.async_sched_insert_fn = ufs_mq_async_sched_fifo_insert,
	.async_sched_seek_fn = ufs_mq_async_sched_fifo_seek,
	.async_sched_requeue_fn = ufs_mq_async_sched_fifo_requeue,
	.async_sched_attempt_merge_fn =
		ufs_mq_async_sched_fifo_attempt_merge_bio,
	.async_sched_is_empty_fn = ufs_mq_async_sched_fifo_empty,
};

static void ufs_mq_run_delay_async_list(
	const struct request_queue *q, unsigned long delay_jiffies)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);

	queue_delayed_work(mas_blk_mq_async_disp_wq,
		&sched_ds->async_dispatch_work.io_dispatch_work, delay_jiffies);

	sched_ds->async_dispatch_work.last_queue_tm = ktime_get();
}

static inline void ufs_mq_run_async_list(const struct request_queue *q)
{
	ufs_mq_run_delay_async_list(q, 0);
}

static void ufs_mq_insert_async_list(
	const struct request *rq, const struct request_queue *q)
{
	unsigned long flags;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;
	struct mas_ufs_mq_priv *priv = NULL;

	if (!q->mas_queue_ops || !q->mas_queue_ops->scheduler_priv)
		return;

	priv = (struct mas_ufs_mq_priv *)(q->mas_queue_ops->scheduler_priv);
	if (!priv->async_io_sched_strategy->async_sched_insert_fn)
		return;

	mas_blk_latency_req_check(
		(struct request *)rq, REQ_PROC_STAGE_MQ_ADDTO_ASYNC_LIST);
	spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
	priv->async_io_sched_strategy->async_sched_insert_fn(
		(struct request *)rq, ds_lld);
	atomic_inc(&ds_lld->async_io_list_cnt);
	spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
}

static void ufs_mq_async_requeue(
	const struct request *rq, const struct request_queue *q)
{
	unsigned long flags;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;
	struct mas_ufs_mq_priv *priv = NULL;

	if (!q->mas_queue_ops || !q->mas_queue_ops->scheduler_priv)
		return;

	priv = (struct mas_ufs_mq_priv *)(q->mas_queue_ops->scheduler_priv);
	if (!priv->async_io_sched_strategy->async_sched_requeue_fn)
		return;

	mas_blk_latency_req_check(
		(struct request *)rq, REQ_PROC_STAGE_MQ_ADDTO_ASYNC_LIST);
	spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
	priv->async_io_sched_strategy->async_sched_requeue_fn(
		(struct request *)rq, ds_lld);
	atomic_inc(&ds_lld->async_io_list_cnt);
	spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
}

static bool ufs_mq_async_dispatch_work_trigger_judgement(
	const struct request_queue *q)
{
	unsigned long flags;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;
	struct mas_ufs_mq_priv *priv = NULL;
	bool result = false;

	if (!q->mas_queue_ops || !q->mas_queue_ops->scheduler_priv)
		goto out;

	priv = (struct mas_ufs_mq_priv *)(q->mas_queue_ops->scheduler_priv);
	if (!priv->async_io_sched_strategy->async_sched_is_empty_fn)
		goto out;

	spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
	result = priv->async_io_sched_strategy->async_sched_is_empty_fn(ds_lld);
	spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
	return !result;
out:
	mas_blk_rdr_panic("async_dispatch_work_trigger_judgement error!");

	return true;
}

static struct request *ufs_mq_seek_request(const struct request_queue *q)
{
	unsigned long flags;
	struct request *cur_req = NULL;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;
	struct mas_ufs_mq_priv *priv = NULL;

	if (!q->mas_queue_ops || !q->mas_queue_ops->scheduler_priv)
		return NULL;

	priv = (struct mas_ufs_mq_priv *)(q->mas_queue_ops->scheduler_priv);
	if (!priv->async_io_sched_strategy->async_sched_seek_fn)
		return NULL;

	spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
	cur_req = priv->async_io_sched_strategy->async_sched_seek_fn(ds_lld);
	if (cur_req)
		atomic_dec(&ds_lld->async_io_list_cnt);
	spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
	return cur_req;
}

static void ufs_mq_async_inflt_lmt_update(
	struct mas_ufs_sched_ds_lld *ds_lld, int limit)
{
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	char log[MAS_IO_TRACE_LEN];

	snprintf(log, MAS_IO_TRACE_LEN,
			"async_io_inflt_lmt = %d", limit);
	trace_mas_io(__func__, log, MAS_IO_TRACE_LEN);
#endif
	if (ds_lld->async_io_inflt_lmt == limit)
		return;

#ifdef CONFIG_MAS_BLK_BW_OPTIMIZE
	spin_lock(&ds_lld->async_limit_update_lock);
	if (ds_lld->async_limit_update_enable)
		ds_lld->async_io_inflt_lmt = limit;
	spin_unlock(&ds_lld->async_limit_update_lock);
#else

	ds_lld->async_io_inflt_lmt = limit;
#endif

	ds_lld->last_async_io_inflt_lmt_update_tm = ktime_get();
}

static inline struct blk_mq_tag_set *get_tag_set_from_ds_lld(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	return container_of(ds_lld->lld, struct blk_mq_tag_set, lld_func);
}

#define UFS_MQ_ASYNC_LMT_CALC_THRESH 4
static void ufs_mq_async_normal_mode_inflt_lmt_calc(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	unsigned int limit;
	unsigned int resrvd_tag_remain =
		get_tag_set_from_ds_lld(ds_lld)->reserved_tags -
		atomic_read(&ds_lld->mq_resrvd_tag_used_cnt);

	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_NORMAL:
		limit = resrvd_tag_remain < UFS_MQ_ASYNC_LMT_CALC_THRESH
				? ASYNC_DISP_LMT_HI_PRESSURE
				: ASYNC_DISP_LMT;
		ufs_mq_async_inflt_lmt_update(ds_lld, limit);
		break;
	case MAS_UFS_MQ_SB_LEVEL1:
		ufs_mq_async_inflt_lmt_update(ds_lld, ASYNC_DISP_LMT_SB1);
		break;
	case MAS_UFS_MQ_SB_LEVEL2:
		ufs_mq_async_inflt_lmt_update(ds_lld, ASYNC_DISP_LMT_SB2);
		break;
	case MAS_UFS_MQ_SB_LONG:
		limit = resrvd_tag_remain < UFS_MQ_ASYNC_LMT_CALC_THRESH
				? ASYNC_DISP_LMT_SB_LONG_HP
				: ASYNC_DISP_LMT_SB_LONG;
		ufs_mq_async_inflt_lmt_update(ds_lld, limit);
		break;
	default:
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		mas_blk_rdr_panic("unknown sb_dispatch_level!");
#else
		limit = resrvd_tag_remain < UFS_MQ_ASYNC_LMT_CALC_THRESH
				? ASYNC_DISP_LMT_HI_PRESSURE
				: ASYNC_DISP_LMT;
		ufs_mq_async_inflt_lmt_update(ds_lld, limit);
#endif
	}
}

static void ufs_mq_async_fwb_mode_inflt_lmt_calc(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	switch (atomic_read(&ds_lld->sb_dispatch_level)) {
	case MAS_UFS_MQ_SB_NORMAL:
		ufs_mq_async_inflt_lmt_update(
			ds_lld, ASYNC_DISP_LMT_HI_PRESSURE);
		break;
	case MAS_UFS_MQ_SB_LEVEL1:
		ufs_mq_async_inflt_lmt_update(ds_lld, ASYNC_DISP_LMT_SB1_FWB);
		break;
	case MAS_UFS_MQ_SB_LEVEL2:
		ufs_mq_async_inflt_lmt_update(ds_lld, ASYNC_DISP_LMT_SB2_FWB);
		break;
	case MAS_UFS_MQ_SB_LONG:
		ufs_mq_async_inflt_lmt_update(
			ds_lld, ASYNC_DISP_LMT_SB_LONG_HP);
		break;
	default:
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		mas_blk_rdr_panic("unknown sb_dispatch_level!");
#else
		ufs_mq_async_inflt_lmt_update(
			ds_lld, ASYNC_DISP_LMT_HI_PRESSURE);
#endif
	}
}

static inline void ufs_mq_async_lm_mode_inflt_lmt_calc(
	const struct mas_ufs_sched_ds_lld *ds_lld)
{
	ufs_mq_async_inflt_lmt_update((struct mas_ufs_sched_ds_lld *)ds_lld,
		ASYNC_DISP_LMT_HI_PRESSURE);
}
#define VM_DIRTY_RATIO_THRESH 50
#define VM_DIRTY_RATIO_ADJUST 10
static enum mas_ufs_mq_async_dispatch_mode ufs_mq_async_dispatch_mode_choose(
	void)
{
#ifdef CONFIG_MAS_PAGECACHE_HELPER
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	char log[MAS_IO_TRACE_LEN];
#endif
	unsigned long fg_thresh;
	unsigned long dirty;

	if (pch_lowmem_check2() > 0)
		return MAS_UFS_MQ_ASYNC_LM_MODE;

#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	snprintf(log, MAS_IO_TRACE_LEN,
		"FILE_DIRTY: %lu UNSTABLE_NFS: %lu WRITEBACK: %lu",
		global_node_page_state(NR_FILE_DIRTY),
		global_node_page_state(NR_UNSTABLE_NFS),
		global_node_page_state(NR_WRITEBACK));
	trace_mas_io(__func__, log, MAS_IO_TRACE_LEN);
	snprintf(log, MAS_IO_TRACE_LEN,
		"global_dirtyable_memory = %lu, vm_dirty_ratio = %d",
		global_dirtyable_memory(), vm_dirty_ratio);
	trace_mas_io(__func__, log, MAS_IO_TRACE_LEN);
#endif

	if (likely(vm_dirty_ratio <= VM_DIRTY_RATIO_THRESH))
		fg_thresh = (((unsigned long)vm_dirty_ratio +
				     VM_DIRTY_RATIO_ADJUST) *
				    global_dirtyable_memory()) / ONE_HUNDRED;
	else
		fg_thresh = ((unsigned long)vm_dirty_ratio *
				    global_dirtyable_memory()) / ONE_HUNDRED;

	dirty = global_node_page_state(NR_FILE_DIRTY) +
	      global_node_page_state(NR_UNSTABLE_NFS) +
	      global_node_page_state(NR_WRITEBACK);
	return (enum mas_ufs_mq_async_dispatch_mode)(
		dirty <= fg_thresh ? MAS_UFS_MQ_ASYNC_NORMAL_MODE
				   : MAS_UFS_MQ_ASYNC_FWB_MODE);
#else
	return MAS_UFS_MQ_ASYNC_NORMAL_MODE;
#endif
}

static void ufs_mq_async_disp_inflt_lmt_decision(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	ds_lld->async_dispatch_mode = ufs_mq_async_dispatch_mode_choose();
	switch (ds_lld->async_dispatch_mode) {
	case MAS_UFS_MQ_ASYNC_NORMAL_MODE:
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "ASYNC_NORMAL_MODE", MAS_IO_TRACE_LEN);
#endif
		ufs_mq_async_normal_mode_inflt_lmt_calc(ds_lld);
		break;
	case MAS_UFS_MQ_ASYNC_FWB_MODE:
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "ASYNC_FWB_MODE", MAS_IO_TRACE_LEN);
#endif
		ufs_mq_async_fwb_mode_inflt_lmt_calc(ds_lld);
		break;
	case MAS_UFS_MQ_ASYNC_LM_MODE:
#ifdef CONFIG_MAS_IO_DEBUG_TRACE
		trace_mas_io(__func__, "ASYNC_LM_MODE", MAS_IO_TRACE_LEN);
#endif
		ufs_mq_async_lm_mode_inflt_lmt_calc(ds_lld);
		break;
	default:
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		mas_blk_rdr_panic("unknown async_dispatch_mode!");
#else
		ufs_mq_async_fwb_mode_inflt_lmt_calc(ds_lld);
#endif
	}
}

void ufs_mq_write_throttle_check_timer_expire(unsigned long data)
{
	struct mas_ufs_sched_ds_lld *sched_ds_lld =
		(struct mas_ufs_sched_ds_lld *)data;
	spin_lock(&sched_ds_lld->async_limit_update_lock);
	sched_ds_lld->async_limit_update_enable = 1;
	spin_unlock(&sched_ds_lld->async_limit_update_lock);
	ufs_mq_async_disp_inflt_lmt_decision(sched_ds_lld);
}

void ufs_mq_write_throttle_handler(struct request_queue *q, bool level)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *sched_ds_lld = sched_ds->sched_ds_lld;

	spin_lock(&sched_ds_lld->async_limit_update_lock);
	sched_ds_lld->async_limit_update_enable = 0;
	switch ((int)level) {
	case ASYNC_THROT_STAGE_ONE:
		sched_ds_lld->async_io_inflt_lmt = ASYNC_STAGE_ONE_LIMIT;
		break;
	case ASYNC_THROT_STAGE_TWO:
		sched_ds_lld->async_io_inflt_lmt = ASYNC_STAGE_TWO_LIMIT;
		break;
	default:
		pr_err("%s: level = %d\n", __func__, level);
		break;
	}
	spin_unlock(&sched_ds_lld->async_limit_update_lock);

	mod_timer(&sched_ds_lld->write_throttle_check_timer,
		  jiffies + msecs_to_jiffies(ASYNC_WRITE_CHECK_MS));
}

static void ufs_mq_async_dispatch(struct request_queue *q)
{
	int ret;
	struct blk_mq_hw_ctx *hctx = NULL;
	struct request *cur_req = NULL;
	struct blk_mq_queue_data bd;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	sched_ds->async_dispatch_work.last_enter_tm = ktime_get();

#ifdef CONFIG_MAS_IO_DEBUG_TRACE
	trace_mas_io(__func__, "-", MAS_IO_TRACE_LEN);
#endif

	do {
		ufs_mq_async_disp_inflt_lmt_decision(ds_lld);
		cur_req = ufs_mq_seek_request(q);
		if (!cur_req)
			break;

		hctx = blk_mq_map_queue(q, (int)cur_req->mq_ctx->cpu);
		bd.rq = cur_req;

		ret = ufs_mq_queue_rq(cur_req, hctx, ds_lld, &bd, q);
		if (likely(ret == BLK_STS_OK)) {
			q->end_sector = rq_end_sector(cur_req);
			continue;
		} else if (ret == BLK_STS_RESOURCE) {
			__blk_mq_requeue_request(cur_req);
			ufs_mq_async_requeue(cur_req, q);
			ufs_mq_run_delay_async_list(q,
						    msecs_to_jiffies(TEN_MS));
			break;
		}

		blk_mq_end_request(cur_req, BLK_STS_IOERR);
	} while (1);

	sched_ds->async_dispatch_work.last_exit_tm = ktime_get();
}

void ufs_mq_async_io_dispatch_work_fn(const struct work_struct *work)
{
	struct mas_ufs_mq_work *mq_work = container_of(
		work, struct mas_ufs_mq_work, io_dispatch_work.work);
	ufs_mq_async_dispatch(mq_work->queue);
}

static void ufs_mq_complete_request(struct request *req)
{
	struct list_head *list = NULL;
	unsigned long flags;

	local_irq_save(flags);
	list = this_cpu_ptr(&blk_cpu_done);
	list_add_tail(&req->ipi_list, list);
	if (likely(list->next == &req->ipi_list))
		raise_softirq_irqoff(BLOCK_SOFTIRQ);
	local_irq_restore(flags);
}

void __ufs_mq_complete_request_remote(const void *data)
{
	struct request *rq = (struct request *)data;

	if (likely(rq->cmd_flags & (REQ_SYNC | REQ_FG))) {
		mas_blk_latency_req_check(rq, REQ_PROC_STAGE_DONE_SFTIRQ);
		rq->q->softirq_done_fn(rq);
	} else {
		ufs_mq_complete_request(rq);
	}
}

static void ufs_mq_rq_requeue(const struct request *req)
{
	blk_mq_requeue_request((struct request *)req, true);
	blk_mq_kick_requeue_list(req->q);
}

static void ufs_mq_io_guard_queue(const struct mas_ufs_mq_sched *sched_ds)
{
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	if ((atomic_read(&ds_lld->mq_prio_tag_used_cnt) +
		    atomic_read(&ds_lld->mq_tag_used_cnt)) &&
		!(ufs_mq_get_hp_inflt(ds_lld) +
			atomic_read(&ds_lld->sync_io_inflt_cnt)))
		ufs_mq_sync_dispatch(sched_ds->q);

	if (atomic_read(&ds_lld->mq_resrvd_tag_used_cnt) &&
		!atomic_read(&ds_lld->async_io_inflt_cnt))
		ufs_mq_run_async_list(sched_ds->q);
}

void ufs_mq_io_guard_work_fn(void)
{
	struct mas_ufs_mq_sched *sched_ds = NULL;

	spin_lock(&io_guard_queue_list_lock);
	list_for_each_entry(
		sched_ds, &mas_io_guard_queue_list, io_guard_list_node) {
		spin_unlock(&io_guard_queue_list_lock);
		ufs_mq_io_guard_queue(sched_ds);
		spin_lock(&io_guard_queue_list_lock);
	}
	spin_unlock(&io_guard_queue_list_lock);

	queue_delayed_work(mas_blk_io_guard_wq, &mas_io_guard_work,
		msecs_to_jiffies(MAS_BLK_IO_GUARD_PERIOD_MS));
}

void ufs_mq_req_alloc_prep(
	struct blk_mq_alloc_data *data, unsigned long ioflag, bool fs_submit)
{
	data->io_flag = fs_submit ? ioflag : (ioflag | REQ_SYNC);
}

void ufs_mq_req_init(const struct blk_mq_ctx *ctx, struct request *rq)
{
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	if (unlikely(atomic_read(&rq->mas_req.req_used)))
		mas_blk_rdr_panic("Reinit unreleased request!");

	atomic_set(&rq->mas_req.req_used, 1);
#endif
	rq->mas_req.mq_ctx_generate = (struct blk_mq_ctx *)ctx;
	rq->mas_req.mas_featrue_flag = 0;
}

void ufs_mq_req_complete(
	struct request *rq, const struct request_queue *q, bool succ_done)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

#ifdef CONFIG_MAS_BLK_DEBUG
	if (unlikely(mas_blk_ft_mq_complete_rq_redirection(rq, succ_done)))
		return;
#endif
	if (likely(succ_done)) {
#ifdef CONFIG_MAS_QOS_MQ
		dec_inflt_for_cpu(rq->mas_req.slot_cpu, ds_lld);
#endif
		if (likely(!blk_rq_is_passthrough(rq)))
			ufs_mq_rq_inflt_update(rq, ds_lld, true);
	}

	if (rq->mq_ctx && (rq->mq_ctx->cpu != raw_smp_processor_id()) &&
		(cpu_online(rq->mq_ctx->cpu))) {
		rq->csd.func = __cfi__ufs_mq_complete_request_remote;
		rq->csd.info = rq;
		rq->csd.flags = 0;
		smp_call_function_single_async((int)rq->mq_ctx->cpu, &rq->csd);
	} else {
		ufs_mq_complete_request(rq);
	}
}

void ufs_mq_req_deinit(struct request *rq)
{
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	atomic_set(&rq->mas_req.req_used, 0);
#endif
}

void ufs_mq_req_insert(struct request *req, struct request_queue *q)
{
	if (likely((req->cmd_flags & REQ_SYNC) ||
		    blk_rq_is_passthrough(req)))
		ufs_mq_insert_sync_list(req, q);
	else
		ufs_mq_insert_async_list(req, q);
}

void ufs_mq_req_requeue(
	struct request *req, const struct request_queue *q)
{
	req->mas_req.protocol_nr = 0;
	ufs_mq_req_insert(req, (struct request_queue *)q);
}

void ufs_mq_req_timeout_handler(struct request *req)
{
	const struct blk_mq_ops *ops = req->q->mq_ops;
	enum blk_eh_timer_return proc_ret = BLK_EH_RESET_TIMER;
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(req->q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = sched_ds->sched_ds_lld;

	if (!test_bit(REQ_ATOM_STARTED, &req->atomic_flags))
		return;

#ifdef CONFIG_MAS_QOS_MQ
	dec_inflt_for_cpu(req->mas_req.slot_cpu, ds_lld);
#endif
	if (!blk_rq_is_passthrough(req))
		ufs_mq_rq_inflt_update(req, ds_lld, true);

#ifdef CONFIG_MAS_BLK_DEBUG
	if (!mas_blk_ft_mq_rq_timeout_redirection(req, &proc_ret) &&
		ops->timeout)
		proc_ret = ops->timeout(req, false);
#else
	if (ops->timeout)
		proc_ret = ops->timeout(req, false);
#endif
	switch (proc_ret) {
	/* Protocol layer has dealt with the timeout event */
	case BLK_EH_HANDLED:
#ifdef CONFIG_WBT
		blk_mq_stat_add(req);
#endif
		ufs_mq_req_complete(req, req->q, false);
		break;
	/* Protocol layer need more time to process the request */
	case BLK_EH_RESET_TIMER:
		ufs_mq_rq_inflt_add(req, ds_lld);
		blk_add_timer(req);
		blk_clear_rq_complete(req);
		break;
	/* Protocol layer would do error-handling itself */
	case BLK_EH_NOT_HANDLED:
		break;
	/* Protocol layer expect block layer can requeue the request */
	case BLK_EH_REQUEUE:
		ufs_mq_rq_requeue(req);
		break;
	default:
		break;
	}
}

void ufs_mq_ctx_put(void)
{
#ifdef CONFIG_PREEMPT_COUNT
	barrier();
	preempt_count_dec();
#endif
}

void ufs_mq_hctx_get_by_req(
	const struct request *rq, struct blk_mq_hw_ctx **hctx)
{
	struct request_queue *q = rq->q;

	*hctx = blk_mq_map_queue(q, rq->mas_req.mq_ctx_generate->cpu);
}

void ufs_mq_exec_queue(const struct request_queue *q)
{
	ufs_mq_sync_dispatch(q);
	if (ufs_mq_async_dispatch_work_trigger_judgement(q))
		ufs_mq_run_async_list(q);
}

void ufs_mq_run_hw_queue(const struct request_queue *q)
{
	ufs_mq_run_sync_list(q);
	if (ufs_mq_async_dispatch_work_trigger_judgement(q))
		ufs_mq_run_async_list(q);
}

void ufs_mq_run_requeue(const struct request_queue *q)
{
	ufs_mq_run_sync_list(q);
	if (ufs_mq_async_dispatch_work_trigger_judgement(q))
		ufs_mq_run_async_list(q);
}

void ufs_mq_poll_enable(bool *enable)
{
	unsigned int i;
	unsigned int count = 0;

	for (i = 0; i < num_possible_cpus(); i++)
		if (cpu_online((int)i))
			count++;

	*enable = count > 1;
}

bool ufs_order_panic_wait_datasync_handle(struct blk_dev_lld *blk_lld)
{
	bool need_flush = false;

	struct mas_ufs_sched_ds_lld *sched_ds_lld =
			(struct mas_ufs_sched_ds_lld *)blk_lld->sched_ds_lld;

	if (atomic_read(&(sched_ds_lld->mq_prio_tag_used_cnt)) +
			atomic_read(&(sched_ds_lld->mq_tag_used_cnt)) > 0) {
		sched_ds_lld->turbo_mode = true;
	} else {
		sched_ds_lld->turbo_mode = false;
		need_flush = true;
	}

	return need_flush;
}

void ufs_order_panic_datasync_handle(struct blk_dev_lld *blk_lld)
{
	struct mas_ufs_sched_ds_lld *sched_ds_lld =
			(struct mas_ufs_sched_ds_lld *)blk_lld->sched_ds_lld;

	sched_ds_lld->turbo_mode = true;
	queue_delayed_work(mas_blk_mq_sync_disp_wq,
					&sched_ds_lld->datasync_work, 0);
}

static void ufs_turbo_check_timer_expire(unsigned long data)
{
	static int check_count = 9000 / MAS_BLK_TURBO_CHECK_PERIOD_MS;
	struct mas_ufs_sched_ds_lld *sched_ds_lld =
		(struct mas_ufs_sched_ds_lld *)data;

	pr_err("%s: check_count = %d turbo_mode = %d prio io = %d, sync io = %d "
		"fg_inflt: %d, vip_inflt: %d, s_inflt: %d a_inflt: %d\n",
		__func__, check_count, sched_ds_lld->turbo_mode,
		atomic_read(&(sched_ds_lld->mq_prio_tag_used_cnt)),
		atomic_read(&(sched_ds_lld->mq_tag_used_cnt)),
		atomic_read(&sched_ds_lld->fg_io_inflt_cnt),
		atomic_read(&sched_ds_lld->vip_io_inflt_cnt),
		atomic_read(&(sched_ds_lld->sync_io_inflt_cnt)),
		atomic_read(&(sched_ds_lld->async_io_inflt_cnt)));
	sched_ds_lld->lld->dump_fn(
		mas_blk_get_queue_by_lld(sched_ds_lld->lld), BLK_DUMP_WARNING);

	if (check_count) {
		if (atomic_read(&(sched_ds_lld->mq_prio_tag_used_cnt)) +
			atomic_read(&(sched_ds_lld->mq_tag_used_cnt)) > 0) {
			sched_ds_lld->turbo_mode = true;
		} else {
			blk_power_off_flush(1);
			sched_ds_lld->turbo_mode = false;
		}
		check_count--;
		mod_timer(&sched_ds_lld->turbo_timer, jiffies +
			msecs_to_jiffies(MAS_BLK_TURBO_CHECK_PERIOD_MS));
	} else {
		sched_ds_lld->turbo_mode = false;
		check_count = 9000 / MAS_BLK_TURBO_CHECK_PERIOD_MS;
	}
}

static void ufs_sync_fs(struct super_block *sb, void *arg)
{
	if (!sb_rdonly(sb) && sb->s_op->sync_fs)
		sb->s_op->sync_fs(sb, *(int *)arg);
}

static void ufs_datasync_work(struct work_struct *work)
{
	int wait = 1;

	pr_err("UFS Sync start\n");
	iterate_supers(ufs_sync_fs, &wait);
	pr_err("UFS Sync complete\n");
}

void ufs_tagset_power_off_proc(struct blk_dev_lld *lld)
{
	struct mas_ufs_sched_ds_lld *sched_ds_lld = lld->sched_ds_lld;

	sched_ds_lld->turbo_mode = true;
	queue_delayed_work(mas_blk_mq_sync_disp_wq,
			&sched_ds_lld->datasync_work, 0);
	mod_timer(&sched_ds_lld->turbo_timer, jiffies +
			msecs_to_jiffies(MAS_BLK_TURBO_CHECK_PERIOD_MS));
}

void ufs_mq_status_dump(const struct request_queue *q, enum blk_dump_scene s)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);
	struct mas_ufs_sched_ds_lld *ds_lld = NULL;
	char *prefix = mas_blk_prefix_str(s);

	if (!sched_ds || !(sched_ds->sched_ds_lld))
		return;

	ds_lld = sched_ds->sched_ds_lld;
	pr_err("%s: vip_wait_cnt: %d\n",
		prefix, atomic_read(&ds_lld->vip_wait_cnt));
	pr_err("%s: h_tag_used_cnt: %d tag_used_cnt: %d r_tag_used_cnt: %d\n",
		prefix, atomic_read(&ds_lld->mq_prio_tag_used_cnt),
		atomic_read(&ds_lld->mq_tag_used_cnt),
		atomic_read(&ds_lld->mq_resrvd_tag_used_cnt));
	pr_err("%s: fg_inflt: %d, vip_inflt: %d, s_inflt: %d a_inflt: %d, cp_inflt: %d\n",
		prefix, atomic_read(&ds_lld->fg_io_inflt_cnt),
		atomic_read(&ds_lld->vip_io_inflt_cnt),
		atomic_read(&ds_lld->sync_io_inflt_cnt),
		atomic_read(&ds_lld->async_io_inflt_cnt),
		atomic_read(&ds_lld->cp_io_inflt_cnt));
}

static void ufs_mq_ktime_init(struct mas_ufs_sched_ds_lld *ds_lld)
{
	ds_lld->last_sync_io_submit_tm = ktime_get();
	ds_lld->last_sync_burst_trig_tm = ktime_get();
	ds_lld->last_sync_io_compl_tm = ktime_get();
	ds_lld->last_async_io_compl_tm = ktime_get();
	ds_lld->last_async_io_inflt_lmt_update_tm = ktime_get();
}

static void ufs_mq_tag_used_cnt_init(struct mas_ufs_sched_ds_lld *ds_lld)
{
	atomic_set(&ds_lld->mq_tag_used_cnt, 0);
	atomic_set(&ds_lld->mq_resrvd_tag_used_cnt, 0);
	atomic_set(&ds_lld->mq_prio_tag_used_cnt, 0);
	atomic_set(&ds_lld->vip_wait_cnt, 0);
}

static void ufs_mq_inflt_cnt_init(struct mas_ufs_sched_ds_lld *ds_lld)
{
	atomic_set(&ds_lld->cp_io_inflt_cnt, 0);
	atomic_set(&ds_lld->async_io_inflt_cnt, 0);
	atomic_set(&ds_lld->fg_io_inflt_cnt, 0);
	atomic_set(&ds_lld->vip_io_inflt_cnt, 0);
	atomic_set(&ds_lld->sync_io_inflt_cnt, 0);
}

static void ufs_mq_inflt_lmt_init(struct mas_ufs_sched_ds_lld *ds_lld)
{
	ds_lld->cp_io_limit = MAS_MQ_DEFAULT_CP_LIMIT;
	ds_lld->sync_io_inflt_lmt = MAS_MQ_SYNC_DISPATCH_LIMIT;
	ds_lld->async_io_inflt_lmt = ASYNC_DISP_LMT;
}

static int ufs_mq_workqueue_init(void)
{
	mas_blk_mq_sync_disp_wq =
		alloc_workqueue("sync_dispatch", WQ_HIGHPRI, 0);
	if (!mas_blk_mq_sync_disp_wq) {
		pr_err("%s %d Failed to alloc sync_dispatch_workqueue\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	mas_blk_mq_async_disp_wq =
		alloc_workqueue("async_dispatch", WQ_UNBOUND, 0);
	if (!mas_blk_mq_async_disp_wq)
		goto destroy_sync_wkq;

	mas_blk_io_guard_wq =
		alloc_workqueue("io_guard", WQ_UNBOUND | WQ_FREEZABLE, 0);
	if (!mas_blk_io_guard_wq)
		goto destroy_async_wkq;

	INIT_DELAYED_WORK(&mas_io_guard_work, __cfi_ufs_mq_io_guard_work_fn);
	return 0;

destroy_async_wkq:
	destroy_workqueue(mas_blk_mq_async_disp_wq);
destroy_sync_wkq:
	destroy_workqueue(mas_blk_mq_sync_disp_wq);
	pr_err("%s init Failed!\n", __func__);
	return -ENOMEM;
}

static void ufs_mq_dispatch_list_init(
	struct mas_ufs_sched_ds_lld *ds_lld)
{
	INIT_LIST_HEAD(&ds_lld->hp_sync_disp_list);
	INIT_LIST_HEAD(&ds_lld->sync_disp_list);

	spin_lock_init(&ds_lld->sync_disp_lock);
	spin_lock_init(&ds_lld->async_disp_lock);

	atomic_set(&ds_lld->hp_io_list_cnt, 0);
	atomic_set(&ds_lld->sync_io_list_cnt, 0);
	atomic_set(&ds_lld->async_io_list_cnt, 0);
}

/* Initial sched_ds_lld per lld */
static struct mas_ufs_sched_ds_lld *ufs_mq_sched_ds_lld_init(
	const struct request_queue *q)
{
	struct blk_dev_lld *lld = mas_blk_get_lld((struct request_queue *)q);
	struct mas_ufs_sched_ds_lld *ds_lld = NULL;
#ifdef CONFIG_MAS_QOS_MQ
	unsigned int i;
#endif

	if (lld->sched_ds_lld_inited) {
		ds_lld = (struct mas_ufs_sched_ds_lld *)lld->sched_ds_lld;
		atomic_inc(&ds_lld->ref_cnt);
		return (struct mas_ufs_sched_ds_lld *)lld->sched_ds_lld;
	}

	ds_lld = kzalloc(sizeof(struct mas_ufs_sched_ds_lld), GFP_KERNEL);
	if (unlikely(!ds_lld))
		return NULL;

	atomic_set(&ds_lld->ref_cnt, 1);
	init_timer(&ds_lld->turbo_timer);
	ds_lld->turbo_timer.function = ufs_turbo_check_timer_expire;
	ds_lld->turbo_timer.data = (uintptr_t)ds_lld;
	ds_lld->turbo_mode = false;
	INIT_DELAYED_WORK(&ds_lld->datasync_work, ufs_datasync_work);
	ufs_mq_tag_used_cnt_init(ds_lld);
	ufs_mq_dispatch_list_init(ds_lld);
	ufs_mq_inflt_lmt_init(ds_lld);
	ufs_mq_inflt_cnt_init(ds_lld);
	ufs_mq_ktime_init(ds_lld);
#ifdef CONFIG_MAS_QOS_MQ
	for (i = 0; i < num_possible_cpus(); i++)
		atomic_set(&(ds_lld->per_cpu_io_inflt[i]), 0);
#endif

	init_timer(&ds_lld->sb_dispatch_check_timer);
	ds_lld->sb_dispatch_check_timer.data = (uintptr_t)ds_lld;
	ds_lld->sb_dispatch_check_timer.function =
		__cfi_ufs_mq_sync_burst_check_timer_expire;

#ifdef CONFIG_MAS_BLK_BW_OPTIMIZE
	init_timer(&ds_lld->write_throttle_check_timer);
	ds_lld->write_throttle_check_timer.data = (uintptr_t)ds_lld;
	ds_lld->write_throttle_check_timer.function =
		__cfi_ufs_mq_write_throttle_check_timer_expire;

	spin_lock_init(&ds_lld->async_limit_update_lock);
	ds_lld->async_limit_update_enable = 1;
#endif

	atomic_set(&ds_lld->sb_dispatch_level, MAS_UFS_MQ_SB_NORMAL);

	ds_lld->async_dispatch_mode = MAS_UFS_MQ_ASYNC_NORMAL_MODE;

	if (ufs_mq_workqueue_init())
		goto free_sched_ds_lld;

	ds_lld->lld = lld;
	lld->sched_ds_lld = ds_lld;
	lld->sched_ds_lld_inited = true;

	if (lld->type == BLK_LLD_TAGSET_BASE)
		mas_blk_flush_list_register(&lld->lld_list);

	return ds_lld;

free_sched_ds_lld:
	kfree(ds_lld);
	return NULL;
}

static void ufs_mq_sched_ds_lld_exit(const struct request_queue *q)
{
	struct blk_dev_lld *lld = mas_blk_get_lld((struct request_queue *)q);
	struct mas_ufs_sched_ds_lld *ds_lld = lld->sched_ds_lld;

	if (lld->type == BLK_LLD_TAGSET_BASE)
		mas_blk_flush_list_unregister(&lld->lld_list);

	if (lld->sched_ds_lld_inited &&
		atomic_dec_and_test(&ds_lld->ref_cnt)) {
		lld->sched_ds_lld_inited = false;
		destroy_workqueue(mas_blk_mq_async_disp_wq);
		destroy_workqueue(mas_blk_mq_sync_disp_wq);
		destroy_workqueue(mas_blk_io_guard_wq);
		kfree(lld->sched_ds_lld);
		lld->sched_ds_lld = NULL;
	}
}

int ufs_mq_iosched_init(struct request_queue *q)
{
	unsigned long flags;
	struct mas_ufs_sched_ds_lld *ds_lld = NULL;
	struct mas_ufs_mq_sched *sched_ds =
		kzalloc(sizeof(struct mas_ufs_mq_sched), GFP_KERNEL);
	struct mas_ufs_mq_priv *priv = NULL;

	if (!sched_ds) {
		pr_err("%s %d Failed to alloc sched_ds!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	ds_lld = ufs_mq_sched_ds_lld_init(q);
	if (!ds_lld)
		goto free_sched_ds;

	if (!q->mas_queue_ops || !q->mas_queue_ops->scheduler_priv)
		goto sched_ds_lld_exit;

	priv = (struct mas_ufs_mq_priv *)(q->mas_queue_ops->scheduler_priv);
	if (priv->async_io_sched_strategy->async_sched_init_fn) {
		spin_lock_irqsave(&ds_lld->async_disp_lock, flags);
		priv->async_io_sched_strategy->async_sched_init_fn(ds_lld);
		spin_unlock_irqrestore(&ds_lld->async_disp_lock, flags);
	}

	INIT_DELAYED_WORK(&sched_ds->sync_dispatch_work.io_dispatch_work,
		__cfi_ufs_mq_sync_io_dispatch_work_fn);
	sched_ds->sync_dispatch_work.queue = q;
	INIT_DELAYED_WORK(&sched_ds->async_dispatch_work.io_dispatch_work,
		__cfi_ufs_mq_async_io_dispatch_work_fn);
	sched_ds->async_dispatch_work.queue = q;

	sched_ds->q = q;
	sched_ds->sched_ds_lld = ds_lld;
	q->mas_queue.cust_queuedata = (void *)sched_ds;

	spin_lock(&io_guard_queue_list_lock);
	list_add_tail(&sched_ds->io_guard_list_node, &mas_io_guard_queue_list);
	spin_unlock(&io_guard_queue_list_lock);
	queue_delayed_work(mas_blk_io_guard_wq, &mas_io_guard_work,
		msecs_to_jiffies(MAS_BLK_IO_GUARD_PERIOD_MS));

	blk_queue_make_request(q, __cfi_ufs_mq_make_request);

	return 0;

sched_ds_lld_exit:
	ufs_mq_sched_ds_lld_exit(q);
free_sched_ds:
	kfree(sched_ds);
	return -ENOMEM;
}

void ufs_mq_iosched_exit(struct request_queue *q)
{
	struct mas_ufs_mq_sched *sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);

	if (!sched_ds)
		return;

	cancel_delayed_work_sync(&mas_io_guard_work);

	spin_lock(&io_guard_queue_list_lock);
	list_del_init(&sched_ds->io_guard_list_node);
	spin_unlock(&io_guard_queue_list_lock);

	/* ufs_mq_async_sched_exit_fn */
	ufs_mq_sched_ds_lld_exit(q);

	kfree(q->mas_queue.cust_queuedata);
	q->mas_queue.cust_queuedata = NULL;
}

void blk_mq_tagset_ufs_mq_iosched_enable(
	struct blk_mq_tag_set *tag_set, int enable)
{
	if (enable)
		tag_set->lld_func.features |= BLK_LLD_IOSCHED_UFS_MQ;
	else
		tag_set->lld_func.features &= ~BLK_LLD_IOSCHED_UFS_MQ;
}

struct mas_ufs_mq_priv mas_ufs_mq = {
	.async_io_sched_strategy = &mas_ufs_mq_async_io_fifo_sched,
};

int blk_mq_get_io_in_list_count(struct block_device *bdev)
{
	struct request_queue *q = NULL;
	struct mas_ufs_mq_sched *sched_ds = NULL;
	struct mas_ufs_sched_ds_lld *sched_ds_lld = NULL;

	if (unlikely(!bdev))
		return -1;

	q = bdev->bd_queue;

	if (unlikely(!q))
		return -1;

	sched_ds =
		(struct mas_ufs_mq_sched *)(q->mas_queue.cust_queuedata);

	if (unlikely(!sched_ds))
		return -1;

	sched_ds_lld = sched_ds->sched_ds_lld;

	if (unlikely(!sched_ds_lld))
		return -1;

	return (atomic_read(&sched_ds_lld->hp_io_list_cnt) +
		atomic_read(&sched_ds_lld->sync_io_list_cnt) +
		atomic_read(&sched_ds_lld->async_io_list_cnt));
}
