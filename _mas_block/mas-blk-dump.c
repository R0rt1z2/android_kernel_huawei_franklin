/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description:  mas block dump
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
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdebug.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-tag.h"
#include "mas-blk-dump-interface.h"
#include "mas-blk-latency-interface.h"

static LIST_HEAD(dump_list);
static DEFINE_SPINLOCK(dump_list_lock);

#define MAS_BLK_DUMP_BUF_LEN 512UL
static int mas_blk_io_type_op(u64 op, char *type_buf, int len)
{
	if (len < 1)
		return 0;

	switch (op) {
	case REQ_OP_READ:
		type_buf[0] = 'R';
		break;
	case REQ_OP_WRITE:
		type_buf[0] = 'W';
		break;
	case REQ_OP_DISCARD:
		type_buf[0] = 'D';
		break;
	case REQ_OP_SECURE_ERASE:
		type_buf[0] = 'E';
		break;
	case REQ_OP_WRITE_SAME:
		type_buf[0] = 'T';
		break;
	case REQ_OP_FLUSH:
		type_buf[0] = 'F';
		break;
	default:
		pr_warn("%s: Unknown OP!\n", __func__);
		return 0;
	}

	return 1;
}

static int mas_blk_io_type_flag(u64 flag, char *type_buf, int len)
{
	int offset = 0;

	if ((flag & REQ_FUA) && offset < len) {
		type_buf[offset] = 'A';
		offset++;
	}
	if ((flag & REQ_SYNC) && offset < len) {
		type_buf[offset] = 'S';
		offset++;
	}
	if ((flag & REQ_META) && offset < len) {
		type_buf[offset] = 'M';
		offset++;
	}
	if ((flag & REQ_PREFLUSH) && offset < len) {
		type_buf[offset] = 'F';
		offset++;
	}

	return offset;
}

static int mas_blk_io_type_priv_flag(u64 flag, char *type_buf, u32 len)
{
	int offset = 0;

	if ((flag & REQ_FG) && offset < (int)len) {
		type_buf[offset] = 'H';
		offset++;
	}
	if ((flag & REQ_CP) && offset < (int)len) {
		type_buf[offset] = 'C';
		offset++;
	}
	if ((flag & REQ_TZ) && offset < (int)len) {
		type_buf[offset] = 'Z';
		offset++;
	}

	return offset;
}

static char *mas_blk_io_type(u64 op, u64 flag, char *type_buf, int len)
{
	int offset = 0;

	offset += mas_blk_io_type_op(op, type_buf, len);
	offset += mas_blk_io_type_flag(flag, type_buf + offset, len - offset);
	offset += mas_blk_io_type_priv_flag(
		flag, type_buf + offset, len - offset);
	type_buf[offset] = '\0';

	return type_buf;
}

/*
 * This function is used to dump the request info
 */
void mas_blk_dump_request(struct request *rq, enum blk_dump_scene s)
{
	int i;
	char log[MAS_BLK_DUMP_BUF_LEN];
	char *cmp_type = NULL;
	unsigned int count = 0;
	char io_type[IO_BUF_LEN];
	char *type = mas_blk_io_type(
		req_op(rq), rq->cmd_flags, io_type, sizeof(io_type));
	char *prefix = mas_blk_prefix_str(s);

	if (blk_rq_is_scsi(rq))
		cmp_type = "TYPE_BLOCK_PC";
	else if (blk_rq_is_private(rq))
		cmp_type = "TYPE_DRV_PRIV";
	else
		cmp_type = "TYPE_FS";

	count += snprintf(log, MAS_BLK_DUMP_BUF_LEN,
			"rq: 0x%pK, type:%s len:%u %s requeue_reason: %d ",
			rq, type, rq->__data_len,
			cmp_type, rq->mas_req.requeue_reason);

	count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"rq_hoq_flag: %llx rq_cp_flag: %llx ",
			req_hoq(rq), req_cp(rq));

	if (rq->mas_req.dispatch_task) {
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"disp_pid: %d %s, task: 0x%pK\n",
			(int)rq->mas_req.task_pid, rq->mas_req.task_comm,
			rq->mas_req.dispatch_task);
#else
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"disp_pid: %d %s\n",
			(int)rq->mas_req.task_pid, rq->mas_req.task_comm);
#endif
	} else {
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"\n");
	}

	for (i = 0; i < REQ_PROC_STAGE_MAX; i++) {
		if (!rq->mas_req.req_stage_ktime[i])
			continue;
		count += snprintf(log + count,
			(size_t)(MAS_BLK_DUMP_BUF_LEN - count),
			"%s: %lld ", req_stage_cfg[i].stage_name,
			rq->mas_req.req_stage_ktime[i]);
	}

	log[MAS_BLK_DUMP_BUF_LEN - 1] = 0;
	pr_err("%s: %s\n", prefix, log);
}

/*
 * This function is used to dump the bio info
 */
void mas_blk_dump_bio(const struct bio *bio, enum blk_dump_scene s)
{
	int i;
	char log[MAS_BLK_DUMP_BUF_LEN];
	int count = 0;
	struct blk_bio_cust *mas_bio = (struct blk_bio_cust *)&bio->mas_bio;
	char io_type[IO_BUF_LEN];
	char *type = mas_blk_io_type(
		bio_op(bio), bio->bi_opf, io_type, sizeof(io_type));
	char *prefix = mas_blk_prefix_str(s);

	count += snprintf(log, MAS_BLK_DUMP_BUF_LEN,
			"bio: 0x%pK, type:%s len:%u ",
			bio, type, bio->bi_iter.bi_size);

	if (mas_bio->dispatch_task) {
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"disp_pid: %d %s",
			mas_bio->task_pid, mas_bio->task_comm);
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			", task: 0x%pK", mas_bio->dispatch_task);
#endif
	}
	count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count, "\n");

	for (i = 0; i < BIO_PROC_STAGE_MAX; i++) {
		if (!(mas_bio->bio_stage_ktime[i]))
			continue;
		count += snprintf(log + count, MAS_BLK_DUMP_BUF_LEN - count,
			"<%s:%lld> ", bio_stage_cfg[i].stage_name,
			mas_bio->bio_stage_ktime[i]);
	}
	log[count] = '\0';
	pr_err("%s %s,current: %lld\n", prefix, log, ktime_get());

	if (bio->mas_bio.io_req)
		mas_blk_dump_request(bio->mas_bio.io_req, s);
}

static int __mas_blk_dump_printf(char *buf, int len, char *str)
{
	int offset = 0;

	if (buf && len > 0)
		offset += snprintf(buf, len, "%s", str);
	else
		pr_err("%s", str);

	return offset;
}

static int mas_blk_q_dump_header(
	struct request_queue *q, char *buf, unsigned long len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "Queue Status:\n");
	return __mas_blk_dump_printf(buf, len, tmp_str);
}

static int mas_blk_q_dump_opt_features(
	struct request_queue *q, char *buf, unsigned long len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];
	int offset = 0;

	if (q->mas_queue.io_latency_enable)
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN,
			"\tIO Latency Waring: enable\tThreshold(ms): %u\n",
			q->mas_queue.io_lat_warning_thresh);
	else
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN,
			"\tIO Latency Waring: disable\n");
	offset += __mas_blk_dump_printf(buf, len, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tFlush Reduce: %s\n",
		q->mas_queue.flush_optimize ? "enable" : "disable");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	return offset;
}

int mas_blk_dump_queue_status2(
	struct request_queue *q, char *buf, int len)
{
	int offset = 0;

	offset += mas_blk_q_dump_header(q, buf, len);
	offset += mas_blk_q_dump_opt_features(q, buf + offset, len - offset);
	if (buf)
		buf[offset] = '\0';

	return offset;
}

/*
 * This function is used to dump request queue status
 */
void mas_blk_dump_queue_status(
	const struct request_queue *q, enum blk_dump_scene s)
{
	char *prefix = mas_blk_prefix_str(s);

	if (!q->mas_queue.queue_disk)
		return;

	pr_err("%s: bdev %s, ptable %s, r_inflt: %d w_inflt: %d",
		prefix, q->mas_queue.queue_disk->disk_name,
		q->mas_queue.blk_part_tbl_exist ? "exist" : "none",
		atomic_read(&q->mas_queue.queue_disk->part0.in_flight[0]),
		atomic_read(&q->mas_queue.queue_disk->part0.in_flight[1]));

#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
	pr_err("q: 0x%pK", q);
#endif

	if (s == BLK_DUMP_PANIC)
		mas_blk_dump_queue_status2((struct request_queue *)q, NULL, 0);

	if (q->mas_queue_ops && q->mas_queue_ops->blk_status_dump_fn)
		q->mas_queue_ops->blk_status_dump_fn(
			(struct request_queue *)q, s);
}

static int mas_blk_lld_dump_latency(
	struct blk_dev_lld *lld, char *buf, int len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];

	if (lld->features & BLK_LLD_LATENCY_SUPPORT)
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN,
			"\tLatency Warning:enable\tThreshold(ms):%u\n",
			lld->latency_warning_threshold_ms);
	else
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN,
			"\tLatency Waring: disable\n");
	return __mas_blk_dump_printf(buf, len, tmp_str);
}

static int mas_blk_lld_dump_ioscheduler(
	struct blk_dev_lld *lld, char *buf, int len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];
	int offset = 0;

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tIO Scheduler: ");
	offset += __mas_blk_dump_printf(buf, len, tmp_str);

	if (lld->features & BLK_LLD_IOSCHED_UFS_MQ)
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "MAS MQ\n");
	else
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "Default Scheduler\n");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	return offset;
}

static int mas_blk_lld_dump_busyidle(
	struct blk_dev_lld *lld, char *buf, int len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];
	int offset = 0;

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tBusy/Idle Notifier: %s\t",
		lld->features & BLK_LLD_BUSYIDLE_SUPPORT ? "enable"
							     : "disable");
	offset += __mas_blk_dump_printf(buf, len, tmp_str);

	switch (lld->blk_idle.idle_state) {
	case BLK_IO_BUSY:
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tState: BUSY\t");
		break;
	case BLK_IO_IDLE:
		snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tState: IDLE\t");
		break;
	default:
		return offset;
	}
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tIO Count: %d\n",
		atomic_read(&lld->blk_idle.io_count));
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	return offset;
}

static int mas_blk_lld_dump_opt_feature(
	struct blk_dev_lld *lld, char *buf, int len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];
	int offset = 0;

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tFlush Reduce: %s\n",
		 lld->features & BLK_LLD_FLUSH_REDUCE_SUPPORT ? "enable" :
								"disable");
	offset += __mas_blk_dump_printf(buf, len, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tInline Crypto: %s\n",
		lld->features & BLK_LLD_INLINE_CRYPTO_SUPPORT ? "enable"
								  : "disable");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tUFS command priority: %s\n",
		lld->features & BLK_LLD_UFS_CP_EN ? "enable" : "disable");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tUFS order preserving: %s\n",
		lld->features & BLK_LLD_UFS_ORDER_EN ? "enable" : "disable");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	return offset;
}



static int mas_blk_lld_dump_header(
	struct blk_dev_lld *lld, char *buf, int len)
{
	unsigned char tmp_str[MAS_BLK_DUMP_BUF_LEN];
	int offset = 0;

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "LLD Status:\n");
	offset += __mas_blk_dump_printf(buf, len, tmp_str);

	snprintf(tmp_str, MAS_BLK_DUMP_BUF_LEN, "\tPanic Dump: %s\n",
		lld->features & BLK_LLD_DUMP_SUPPORT ? "enable" : "disable");
	offset += __mas_blk_dump_printf(buf + offset, len - offset, tmp_str);

	return offset;
}

/*
 * This function is used to dump low level driver object status
 */
int mas_blk_dump_lld_status(struct blk_dev_lld *lld, char *buf, int len)
{
	int offset = 0;

	offset += mas_blk_lld_dump_header(lld, buf, len);
	offset += mas_blk_lld_dump_latency(lld, buf + offset, len - offset);
	offset += mas_blk_lld_dump_busyidle(lld, buf + offset, len - offset);
	offset += mas_blk_lld_dump_ioscheduler(
		lld, buf + offset, len - offset);
	offset += mas_blk_lld_dump_opt_feature(
		lld, buf + offset, len - offset);
	if (buf)
		buf[offset] = '\0';

	return offset;
}

static void mas_blk_dump_counted_io(
	struct blk_dev_lld *lld, struct request_queue *q,
	enum blk_dump_scene s)
{
	if (!list_empty(&lld->blk_idle.bio_list)) {
		struct bio *pos = NULL;

		pr_err("bio_list:\n");
		list_for_each_entry(pos, &lld->blk_idle.bio_list, cnt_list) {
			if (pos->mas_bio.q == q)
				mas_blk_dump_bio(pos, s);
		}
	}
	if (!list_empty(&lld->blk_idle.req_list)) {
		struct request *pos = NULL;

		pr_err("req_list:\n");
		list_for_each_entry(pos, &lld->blk_idle.req_list, cnt_list) {
			if (pos->q == q)
				mas_blk_dump_request(pos, s);
		}
	}
}

static void mas_blk_dump_queue(
	struct request_queue *q, enum blk_dump_scene s)
{
	struct blk_dev_lld *lld = mas_blk_get_lld(q);

	if (lld->dump_fn) {
		mas_blk_dump_lld_status(lld, NULL, 0);
		pr_err("bio_count = %llu req_count = %llu\n",
			lld->blk_idle.bio_count, lld->blk_idle.req_count);
		lld->dump_fn(q, BLK_DUMP_PANIC);
		lld->dump_fn = NULL;
	}

	mas_blk_dump_queue_status(q, BLK_DUMP_PANIC);

	mas_blk_dump_counted_io(lld, q, s);
	if (q->mas_queue_ops && q->mas_queue_ops->blk_req_dump_fn)
		q->mas_queue_ops->blk_req_dump_fn(q, s);
}

/*
 * ecall mas_blk_dump() to test blk panic dump functionality
 * don't change this function name or you'll need to correct
 * the ST case too.
 */
#if defined(CONFIG_MAS_DEBUG_FS) || defined(CONFIG_MAS_BLK_DEBUG)
void mas_blk_painc_dump(void)
#else
static void mas_blk_painc_dump(void)
#endif
{
	struct request_queue *q = NULL;
	struct blk_queue_cust *mas_queue = NULL;

	list_for_each_entry(mas_queue, &dump_list, dump_list) {
		q = (struct request_queue *)container_of(
			mas_queue, struct request_queue, mas_queue);
		mas_blk_dump_queue(q, BLK_DUMP_PANIC);
	}
}

int mas_blk_panic_notify(
	struct notifier_block *nb, unsigned long event, void *buf)
{
	pr_err("%s: ++ current = %lld\n", __func__, ktime_get());
	mas_blk_painc_dump();
	pr_err("%s: --\n", __func__);
	return 0;
}

#if 1
ssize_t mas_queue_panic_dump_store(
	struct request_queue *q, const char *page, size_t count)
{
	ssize_t ret;
	unsigned long val;

	ret = queue_var_store(&val, page, count);
	if (ret < 0)
		return (ssize_t)count;

	if (val == 1)
		mas_blk_painc_dump();
	else
		panic("mas panic dump\n");

	return (ssize_t)count;
}
#endif
/*
 * This function is used to add the request queue into the dump list
 */
void mas_blk_dump_register_queue(struct request_queue *q)
{
	struct blk_dev_lld *blk_lld = mas_blk_get_lld(q);

	if (blk_lld->features & BLK_LLD_DUMP_SUPPORT) {
		if (!list_empty(&q->mas_queue.dump_list))
			return;
		spin_lock(&dump_list_lock);
		list_add_tail(&q->mas_queue.dump_list, &dump_list);
		spin_unlock(&dump_list_lock);
	}
}

/*
 * This function is used to remove the request queue from the dump list
 */
void mas_blk_dump_unregister_queue(struct request_queue *q)
{
	struct request_queue *q_node = NULL;
	struct blk_queue_cust *mas_queue = NULL;
	struct blk_queue_cust *mas_queue_backup = NULL;

	spin_lock(&dump_list_lock);
	list_for_each_entry_safe(
		mas_queue, mas_queue_backup, &dump_list, dump_list) {
		q_node = (struct request_queue *)container_of(
			mas_queue, struct request_queue, mas_queue);
		if (q_node == q) {
			list_del(&q_node->mas_queue.dump_list);
			break;
		}
	}
	spin_unlock(&dump_list_lock);
}

static struct notifier_block mas_blk_panic_nb = {
	.notifier_call = __cfi_mas_blk_panic_notify,
	.next			= NULL,
	.priority		= INT_MAX,
};

void __init mas_blk_dump_init(void)
{
	atomic_notifier_chain_register(&panic_notifier_list, &mas_blk_panic_nb);
	register_die_notifier(&mas_blk_panic_nb);
}

/*
 * This function is used to enable the dump function on MQ tagset
 */
void blk_mq_tagset_dump_register(
	struct blk_mq_tag_set *tag_set, lld_dump_status_fn func)
{
	struct blk_dev_lld *blk_lld = &tag_set->lld_func;

	blk_lld->dump_fn = func;
	blk_lld->features |= BLK_LLD_DUMP_SUPPORT;
}

/*
 * This function is used to enable the dump function on request queue
 */
void blk_queue_dump_register(struct request_queue *q, lld_dump_status_fn func)
{
	struct blk_dev_lld *blk_lld = mas_blk_get_lld(q);

	blk_lld->dump_fn = func;
	blk_lld->features |= BLK_LLD_DUMP_SUPPORT;
	mas_blk_dump_register_queue(q);
}
