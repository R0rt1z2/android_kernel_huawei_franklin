/* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * FileName: vltmm.c
 * Description: This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation;
 * either version 2 of the License,
 * or (at your option) any later version.
 */

#define pr_fmt(fmt) "secmem: " fmt

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/genalloc.h>
#include <linux/mutex.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/cma.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>
#include <linux/memblock.h>
#include <linux/swap.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <securec.h>
#include <ion_priv.h>
#include "secmem_tee_op.h"
#include "vltmm_agent.h"
#include "vltmm.h"
#include "secmem_api.h"


static struct smem_heap smemheaps[SMEM_HEAPS_MAX];
static struct smem_free_task smem_async_free;

static int ta_init;
static TEEC_Context *context;
static TEEC_Session *session;

#ifndef CONFIG_ZONE_MOVABLE_CMA
static u64 smem_parse_dt_node(unsigned long node,
				const char *name, u64 default_value)
{
	u64 *val = NULL;

	val = (u64 *)of_get_flat_dt_prop(node, name, NULL);
	if (val)
		return be64_to_cpu(*val);
	else
		return default_value;
}

static int smemheap_cma_set_up(struct reserved_mem *rmem)
{
	phys_addr_t align;
	phys_addr_t mask;
	int ret;
	struct smem_heap smemheap;
	struct cma *cma = NULL;
	unsigned long node = rmem->fdt_node;

	memset_s(&smemheap, sizeof(smemheap), 0, sizeof(smemheap));

	if (!of_get_flat_dt_prop(node, "reusable", NULL) ||
	    of_get_flat_dt_prop(node, "no-map", NULL)) {
		pr_err("%s err node\n", __func__);
		return -EINVAL;
	}

	smemheap.gran = smem_parse_dt_node(node, "subbit-align", SZ_2M);

	align = smemheap.gran;
	mask = align - 1;
	if ((rmem->base & mask) || (rmem->size & mask) || !rmem->size) {
		pr_err("Reserved memory: incorrect CMA region, %llx %llx\n",
				rmem->base, rmem->size);
		return -EINVAL;
	}

	ret = cma_init_reserved_mem(rmem->base, rmem->size,
				    0, rmem->name, &cma);
	if (ret) {
		pr_err("Reserved memory: unable to setup CMA region\n");
		return -EINVAL;
	}
	smemheap.cma = cma;

	smemheap.min =
		smem_parse_dt_node(node, "water-mark-min", SMEM_DEFAULT_MIN);
	smemheap.low =
		smem_parse_dt_node(node, "water-mark-low", SMEM_DEFAULT_LOW);
	smemheap.high =
		smem_parse_dt_node(node, "water-mark-high", SMEM_DEFAULT_HIGH);

	smemheap.sid = smem_parse_dt_node(node, "sid", SMEM_SECOS_2M);
	smemheap.sid %= SMEM_HEAPS_MAX;

	smemheap.escape_flag = smem_parse_dt_node(node, "escape-flag", 0);

	smemheap.avail = 0;
	smemheap.max = rmem->size / smemheap.gran;

	smemheaps[smemheap.sid] = smemheap;

	pr_err("cma base: %llx, size: %llx\n"
		"\t\t gran: %llx min: %llx low: %llx high: %llx max: %llx\n"
		"\t\t sid: %llx escape-flag: %llu\n",
		rmem->base, rmem->size,
		smemheap.gran, smemheap.min, smemheap.low, smemheap.high,
		smemheap.max, smemheap.sid, smemheap.escape_flag);

	return 0;
}

RESERVEDMEM_OF_DECLARE(smemheap_cma, "smemheap-cma", smemheap_cma_set_up);
#else
static struct smem_heap smemheap = {
	0,
	0,
	0x0, /* min */
	0x8, /* low */
	0x20, /* high */
	0x20, /* max */
	0x400000, /* gran */
	SMEM_SECOS_4M, /* sid */
	0,
	1, /* escape_flag */
};

void set_vltmm_cma(struct cma *cma)
{
	if (!cma)
		return;

	smemheap.cma = cma;
	smemheaps[SMEM_SECOS_4M] = smemheap;

	pr_info("vltmm gran: %llx min: %llx low: %llx high: %llx max: %llx\n"
		"\t\t sid: %llx escape-flag: %llu\n",
		smemheap.gran, smemheap.min, smemheap.low, smemheap.high,
		smemheap.max, smemheap.sid, smemheap.escape_flag);
}
#endif

void tick_the_thread(int async, u64 sid)
{
	pr_info("tick thread %d\n", async);
	if (async == ASYNC_ALLOC) {
		atomic_set(&smemheaps[sid].wait_flag, 1);
		wake_up_interruptible(&smemheaps[sid].alloc_wq);
	} else {
		atomic_set(&smem_async_free.wait_flag, 1);
		wake_up_interruptible(&smem_async_free.free_wq);
	}
}

static void __smem_config_mem_attr(phys_addr_t phy,
				unsigned long addr, unsigned long size)
{
	(void)phy;
	change_secpage_range(addr, size, 0);
}

static int __smem_alloc_size_check(uint32_t sid, uint32_t num)
{
	struct smem_heap *p = NULL;

	p = &smemheaps[sid];

	if (num > p->max)
		return -EINVAL;

	if (num + p->used > p->high) {
		pr_err("alloc check failed, num: %u, used: %llu, high: %llu\n",
				num, p->used, p->high);
		return -EINVAL;
	}

	return 0;
}

static struct smem_heap *__smem_select_heap(uint32_t in_sid, uint32_t num,
				uint32_t *sel_num)
{
	struct smem_heap *p = NULL;
	int i;

	p = &smemheaps[in_sid];
	if (p->init && p->cma) {
		*sel_num = num;
		return p;
	}

	for (i = 0; i < SMEM_HEAPS_MAX; i++) {
		p = &smemheaps[i];
		if (!p->init && !p->cma)
			continue;
		*sel_num = ALIGN(num * SMEM_GRAN_2M, p->gran) / p->gran;
		return p;
	}

	return NULL;
}

int __smem_allocator_msg_alloc(struct vltmm_msg_t *msg, phys_addr_t *data,
				uint32_t len, uint32_t num)
{
	phys_addr_t *pphy = NULL;
	struct page *pg = NULL;
	struct smem_heap *p = NULL;
	int ret = -ENOMEM;
	uint32_t sel_num = 0;
	uint32_t sid;

	if (!msg || !num || !data)
		return -EINVAL;

	p = __smem_select_heap(msg->cid, num, &sel_num);

	pr_info("into smem allocator alloc, num: %u, sel_num: %u\n", num, sel_num);

	if (!p || !p->cma)
		return -ENOMEM;

	sid = p->sid;
	msg->cid = p->sid;
	mutex_lock(&p->mutex);
	pr_info("alloc begin, num: %u\n", sel_num);
	if (__smem_alloc_size_check(sid, sel_num))
		goto end;
	if (sel_num > p->avail) {
		int n = sel_num - p->avail;

		while (n--) {
			pg = cma_alloc(p->cma, p->gran >> PAGE_SHIFT,
					(u32)get_order((u32)p->gran), GFP_KERNEL);
			if (!pg) {
				pr_err("cma alloc failed, avail:%llu used: %llu num: %u\n",
					p->avail, p->used, sel_num);
				goto end;
			}
			__smem_config_mem_attr(page_to_phys(pg),
				(unsigned long)(uintptr_t)page_address(pg), p->gran);
			p->phy_array[p->avail++] = page_to_phys(pg);
		}
	}

	p->avail -= sel_num;
	pphy = &p->phy_array[p->avail];
	p->used += sel_num;
	ret = memcpy_s(data, len, pphy, sel_num * sizeof(phys_addr_t));
	if (ret)
		goto end;
	pr_info("alloc succ\n");

	msg->cid = p->sid;
	msg->num = sel_num;
	if (p->used + p->avail < p->high)
		tick_the_thread(ASYNC_ALLOC, sid);
	ret = EOK;
end:
	mutex_unlock(&p->mutex);
	return ret;
}

void __smem_allocator_msg_free(uint32_t sid, phys_addr_t *data, uint32_t num)
{
	uint32_t loop;
	struct smem_heap *p = NULL;
	uint32_t sel_num = 0;

	if ((sid >= SMEM_HEAPS_MAX) || !num || !data)
		return;

	p = __smem_select_heap(sid, num, &sel_num);
	if (!p || !p->init)
		return;

	mutex_lock(&p->mutex);

	if (p->avail + sel_num > p->max ||
		p->used < sel_num)
		goto overflow;

	for (loop = 0; loop < sel_num; loop++)
		p->phy_array[p->avail++] = data[loop];

	p->used -= sel_num;

overflow:
	mutex_unlock(&p->mutex);
}

static int smem_allocator_async_alloc_thread(void *data)
{
	struct smem_heap *p = (struct smem_heap *)data;
	int ret;

	while (!kthread_should_stop()) {
		unsigned long need;

		ret = wait_event_interruptible(p->alloc_wq, atomic_read(&p->wait_flag));
		if (ret)
			continue;
		atomic_set(&p->wait_flag, 0);
		atomic_inc(&p->alloc_run);
		mutex_lock(&p->mutex);
		if (p->used + p->avail >= p->high) {
			mutex_unlock(&p->mutex);
			goto overflow;
		}
		need = p->high - p->used - p->avail;
		mutex_unlock(&p->mutex);
		pr_err("async alloc, need: %lu\n", need);
		while (need--) {
			struct page *pg = NULL;

			pg = cma_alloc(p->cma, p->gran >> PAGE_SHIFT,
					(u32)get_order((u32)p->gran), GFP_KERNEL);
			if (!pg)
				goto overflow;

			__smem_config_mem_attr(page_to_phys(pg),
				(unsigned long)(uintptr_t)page_address(pg), p->gran);

			mutex_lock(&p->mutex);
			p->phy_array[p->avail++] = page_to_phys(pg);

			if (p->used + p->avail >= p->high) {
				mutex_unlock(&p->mutex);
				goto overflow;
			}

			mutex_unlock(&p->mutex);
		}

overflow:
	atomic_dec(&p->alloc_run);
	}

	return 0;
}

static inline struct task_struct *smem_allocator_create_thread(
				struct smem_heap *data, const char *name, int index)
{
	return kthread_run(smem_allocator_async_alloc_thread, data, "%s-%d", name, index);
}

static void smem_allocator_prealloc(struct smem_heap *p, unsigned int num)
{
	struct page *pg = NULL;

	if (!p || !p->init || !p->cma)
		return;

	pr_err("pre alloc smem, num: %u\n", num);
	mutex_lock(&p->mutex);
	while (num--) {
		pg = cma_alloc(p->cma, p->gran >> PAGE_SHIFT,
				(u32)get_order((u32)p->gran), GFP_KERNEL);
		if (!pg) {
			mutex_unlock(&p->mutex);
			return;
		}
		__smem_config_mem_attr(page_to_phys(pg),
				(unsigned long)(uintptr_t)page_address(pg), p->gran);
		p->phy_array[p->avail++] = page_to_phys(pg);
		pr_info("pre alloc smem, avail: %llu\n", p->avail);
		if (p->avail >= p->max)
			break;
	}

	mutex_unlock(&p->mutex);
}

static int smem_allocator_init(const char *name)
{
	struct task_struct *task = NULL;
	struct smem_heap *p = NULL;
	int i;

	for (i = 0; i < SMEM_HEAPS_MAX; i++) {
		p = &smemheaps[i];
		if (!p->cma)
			continue;

		mutex_init(&p->mutex);
		init_waitqueue_head(&p->alloc_wq);
		atomic_set(&p->alloc_run, 0);

		task = smem_allocator_create_thread(p, name, i);
		if (!task)
			return -1;
		p->alloc_task = task;

		p->phy_array = kzalloc(sizeof(phys_addr_t) * p->max + 1, GFP_KERNEL);
		if (!p->phy_array)
			return -1;

		p->init = 1;

		smem_allocator_prealloc(p, p->min);
	}
	return 0;
}

#define SMEM_SZ_1K   1024
#define SMEM_MINFREE_MIN  (300 * 256)
#define SMEM_MINFREE_LOW  (500 * 256)
#define SMEM_MINFREE_HIGH (800 * 256)
#define SMEM_MAX_ADJ 3
static unsigned long smem_adj[SMEM_MAX_ADJ] = {
	8,
	4,
	2,
};

static unsigned long smem_minfree[SMEM_MAX_ADJ] = {
	SMEM_MINFREE_MIN,
	SMEM_MINFREE_LOW,
	SMEM_MINFREE_HIGH,
};

static unsigned long smem_get_adj(void)
{
	unsigned long other_file;
	unsigned long scan_adj;
	unsigned long minfree;
	int i;

	other_file = global_node_page_state(NR_FILE_PAGES) -
				global_node_page_state(NR_SHMEM) -
				global_node_page_state(NR_UNEVICTABLE) -
				total_swapcache_pages();
	scan_adj = 0;
	for (i = 0; i < SMEM_MAX_ADJ; i++) {
		minfree = smem_minfree[i];
		if (other_file < minfree) {
			scan_adj = smem_adj[i];
			break;
		}
	}

	return scan_adj;
}

static unsigned long smemheaps_scan(struct shrinker *s,
				struct shrink_control *sc)
{
	int i;
	unsigned long sum = 0;
	struct smem_heap *p = NULL;
	unsigned long long num;
	unsigned long max_scan_count;

#ifndef CONFIG_SECMEM_TEST
	if (!current_is_kswapd())
		return sum;
#endif

	max_scan_count = smem_get_adj();
	if (max_scan_count == 0)
		return sum;

	for (i = 0; i < SMEM_HEAPS_MAX; i++) {
		p = &smemheaps[i];

		if (atomic_read(&p->alloc_run))
			continue;

		if (p->escape_flag || !p->init)
			continue;

		mutex_lock(&p->mutex);

		if (p->used > 0)
			tick_the_thread(ASYNC_FREE, i);

		num = p->avail;
		if (max_scan_count != SMEM_SCAN_CNT && num > p->min)
			num -= p->min;

		num = min((unsigned long long)max_scan_count, num);
		while (num--) {
			struct page *page =
				phys_to_page(p->phy_array[--p->avail]);

			change_secpage_range((unsigned long)(uintptr_t)page_address(page),
				p->gran,
				1);

			cma_release(p->cma, page, p->gran >> PAGE_SHIFT);
			sum += p->gran >> PAGE_SHIFT;
		}
		mutex_unlock(&p->mutex);
#ifdef CONFIG_SECMEM_TEST
		pr_err("smem heap[%d] scan: %lu\n", i, sum);
#endif
	}

	return sum;
}

static unsigned long smemheaps_count(struct shrinker *s, struct shrink_control *sc)
{
	int i;
	unsigned long sum = 0;
	unsigned long max_scan_count;

	if (!current_is_kswapd())
		return sum;

	if (smem_get_adj() == 0)
		return sum;

	for (i = 0; i < SMEM_HEAPS_MAX; i++) {
		struct smem_heap *p = &smemheaps[i];

		if (p->escape_flag || !p->init)
			continue;

		mutex_lock(&p->mutex);
		sum += (p->avail + p->used) * (p->gran >> PAGE_SHIFT);
		mutex_unlock(&p->mutex);
	}

	max_scan_count = (unsigned long)SMEM_SCAN_CNT *
		((unsigned long)SMEM_GRAN_2M >> PAGE_SHIFT);
	sum = min(max_scan_count, sum);

	return sum;
}

static struct shrinker smemheaps_shrinker = {
	.scan_objects = smemheaps_scan,
	.count_objects = smemheaps_count,
	.seeks = DEFAULT_SEEKS * SMEM_SHRINKER_NUM
};

static void smem_send_reclaim_msg(uint32_t sid)
{
	struct mem_chunk_list mcl = {0};
	int ret, num;
	u64 addr[SMEM_SCAN_CNT] = {0};
	u32 i;

	if (!ta_init || sid > SMEM_HEAPS_MAX)
		return;

	mcl.protect_id = SUB_CMD_FREE;
	mcl.phys_addr = addr;
	mcl.size = sizeof(addr);

	ret = secmem_tee_exec_cmd(session, &mcl, ION_SEC_CMD_VLTMM);
	pr_info("smem reclaim cmd, result: %d count: %u\n", ret, mcl.nents);
	num = 0;
	for (i = 0; i < mcl.nents; i++) {
		if (addr[i] != 0)
			num++;
		else
			break;
	}
	__smem_allocator_msg_free(sid, addr, num);
}

void smem_send_dump_msg(void)
{
	struct mem_chunk_list mcl = {0};
	int ret;
	u64 addr = 0;

	if (!ta_init)
		return;

	mcl.protect_id = SUB_CMD_DUMP;
	mcl.phys_addr = &addr;
	mcl.size = sizeof(addr);

	ret = secmem_tee_exec_cmd(session, &mcl, ION_SEC_CMD_VLTMM);
	pr_info("smem dump cmd, result: %d\n", ret);
}

static int smemheaps_async_free(void *data)
{
	struct smem_free_task *fp = &smem_async_free;

	while (!kthread_should_stop()) {
		int i, ret;

		ret = wait_event_interruptible(fp->free_wq, atomic_read(&fp->wait_flag));
		if (ret)
			continue;
		atomic_set(&fp->wait_flag, 0);

		for (i = 0; i < SMEM_HEAPS_MAX; i++) {
			struct smem_heap *p = &smemheaps[i];

			if (p->init && !atomic_read(&p->alloc_run))
				smem_send_reclaim_msg((uint32_t)i);
		}
	}
	return 0;
}


static int smem_pool_watermark_get(void *data, uint64_t *val)
{
	struct smem_heap *p = NULL;
	uint32_t i;
	uint64_t used = 0;
	(void)data;

	for (i = 0; i < SMEM_HEAPS_MAX; i++) {
		p = &smemheaps[i];

		if (!p->init || !p->cma)
			continue;

		pr_err("smem heap[%u] show:\n"
			"\t\tmin: %llu\nlow: %llu\nhigh: %llu"
			"\t\tmax: %llu\ngran: %llu\navail: %llu\nused: %llu\n",
			i, p->min, p->low, p->high,
			p->max, p->gran, p->avail, p->used);

		used += p->used;
	}

	if (val)
		*val = used;

	return 0;
}

static int smem_pool_watermark_set(void *data, uint64_t val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(smem_pool_watermark, smem_pool_watermark_get,
				smem_pool_watermark_set, "%llu\n");

void smem_sysfs_init(void)
{
	struct dentry *tmp;

	tmp = debugfs_create_dir("vltmm", NULL);
	if (!tmp)
		return;
	debugfs_create_file("smem_watermark", 0644, tmp, NULL,
				&smem_pool_watermark);
}

int smem_free_task_init(void)
{
	init_waitqueue_head(&smem_async_free.free_wq);
	atomic_set(&smem_async_free.wait_flag, 0);
	smem_async_free.free_task = kthread_run(smemheaps_async_free,
								NULL, "%s", "smem_async_free");
	if (!smem_async_free.free_task)
		return -1;

	return 0;
}

int smem_ta_session_init(void)
{
	int ret;

	if (!ta_init) {
		context = kzalloc(sizeof(TEEC_Context), GFP_KERNEL);
		if (!context)
			return -1;

		session = kzalloc(sizeof(TEEC_Session), GFP_KERNEL);
		if (!session) {
			kfree(context);
			context = NULL;
			return -1;
		}

		ret = secmem_tee_init(context, session, TEE_VLTMM_NAME);
		if (ret) {
			pr_err("TA session init failed\n");
			kfree(context);
			kfree(session);
			context = NULL;
			session = NULL;
			return -1;
		}
		ta_init = 1;
	}

	return 0;
}

int smemheaps_init(void)
{
	pr_err("smemheaps init.\n");

	if (smem_free_task_init())
		return -1;

	register_shrinker(&smemheaps_shrinker);

	if (smem_allocator_init("smem-allocator")) {
		pr_err("smemheaps preallocate fail.\n");
		return -1;
	}

	smem_sysfs_init();

	smem_ta_session_init();

	return 0;
}

device_initcall(smemheaps_init);

#ifdef CONFIG_HW_SECMEM_DEBUG
struct task_struct *vltm_test_task = NULL;
wait_queue_head_t vltm_test_wq;
atomic_t vltm_test_flag;
static u32 vltmm_test_p1;
static u32 vltmm_test_p2;
static u32 vltmm_test_p3;

static void vltmm_test_init(void)
{
	init_waitqueue_head(&vltm_test_wq);
	atomic_set(&vltm_test_flag, 0);
}


static void vltmm_test_sendcmd(void)
{
	struct mem_chunk_list mcl;
	int ret;
	u64 addr = 0;

	if (!ta_init) {
		pr_err("ta no init\n");
		return;
	}

	mcl.phys_addr = &addr;
	mcl.size = sizeof(addr);
	mcl.protect_id = vltmm_test_p1;
	mcl.nents = vltmm_test_p2;
	mcl.va = vltmm_test_p3;

	ret = secmem_tee_exec_cmd(session, &mcl, ION_SEC_CMD_VLTMM);
	if (ret)
		pr_err("vltmm test fail, %d\n", ret);
	else
		pr_err("vltmm test, cmd: %u, p2: 0x%x p3: 0x%x\n",
			mcl.protect_id, mcl.nents, mcl.va);
}


static int vltmm_test_kthread(void *p)
{
	int ret;

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(vltm_test_wq, atomic_read(&vltm_test_flag));
		if (ret)
			continue;
		atomic_set(&vltm_test_flag, 0);

		vltmm_test_sendcmd();

		msleep(1);
	}

	return 0;
}

void vltmm_create_thread(void)
{
	if (!vltm_test_task) {
		pr_err("vlt create test thread\n");
		vltmm_test_init();
		vltm_test_task = kthread_run(vltmm_test_kthread, NULL,
						"%s", "vltmm_test");
	}
}

void vltmm_test_wakeup(u32 p1, u32 p2, u32 p3)
{
	if (!vltm_test_task) {
		vltmm_create_thread();
		msleep(1000);
	}
	vltmm_test_p1 = p1;
	vltmm_test_p2 = p2;
	vltmm_test_p3 = p3;
	atomic_set(&vltm_test_flag, 1);
	wake_up_interruptible(&vltm_test_wq);
}

void vltmm_test_secmem(u64 phy, u32 len)
{
	void *va = NULL;
	int ret;

	va = phys_to_virt(phy);
	if (va) {
		ret = memset_s(va, len, SMEM_TEST_MAGIC, len);
		if (ret) {
			pr_err("vlt mem set failed\n");
			return;
		}
		pr_err("vlt mem write: %x\n", SMEM_TEST_MAGIC);
		pr_err("vlt mem read: %llx\n", *(u64 *)va);
	}
}

void vltmm_set_minfree(unsigned long p1, unsigned long p2, unsigned long p3)
{
	int i;

	i = 0;
	smem_minfree[i++] = p1;
	smem_minfree[i++] = p2;
	smem_minfree[i] = p3;
}

static struct tz_pageinfo *__secmem_pageinfo_trans(struct sg_table *sg_table)
{
	struct tz_pageinfo *pageinfo = NULL;
	uint32_t i;
	uint32_t nents = sg_table->nents;
	struct scatterlist *sg = NULL;

	pageinfo = kcalloc(nents, sizeof(*pageinfo), GFP_KERNEL);
	if (!pageinfo)
		return NULL;

	for_each_sg(sg_table->sgl, sg, sg_table->nents, i) {
		pageinfo[i].addr = page_to_phys(sg_page(sg));
		pageinfo[i].nr_pages = sg->length / PAGE_SIZE;
	}

	return pageinfo;
}

void secmem_alloc_test(int svc_id, unsigned int size)
{
	struct sg_table *sg = NULL;
	struct mem_chunk_list mcl;
	int ret;
	ktime_t time_start;

	if (svc_id >= SEC_SVC_MAX) {
		pr_err("alloc test fail, invalid svc_id\n");
		return;
	}

	time_start = ktime_get();
	sg = cma_secmem_alloc(svc_id, size);
	if (!sg) {
		pr_err("alloc failed, size: %u\n", size);
		return;
	}
	pr_err("secmem alloc cost: %lld us, size: %u\n", ktime_us_delta(ktime_get(), time_start), size);

	if (ta_init) {
		struct tz_pageinfo *pageinfo = NULL;

		pageinfo = __secmem_pageinfo_trans(sg);
		if (!pageinfo)
			goto out;
		mcl.phys_addr = pageinfo;
		mcl.size = sg->nents * sizeof(struct tz_pageinfo);
		mcl.protect_id = SUB_CMA_CMA_ALLOC_TEST;
		mcl.nents = sg->nents;
		mcl.va = (uint32_t)svc_id;

		ret = secmem_tee_exec_cmd(session, &mcl, ION_SEC_CMD_VLTMM);
		if (ret)
			pr_err("secmem test fail, %d\n", ret);
		else
			pr_err("secmem test success\n");
	}

out:
	cma_secmem_free(svc_id, sg);
}

#endif

