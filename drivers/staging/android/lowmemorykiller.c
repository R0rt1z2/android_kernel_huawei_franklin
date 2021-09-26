/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/profile.h>
#include <linux/notifier.h>
#include <linux/freezer.h>
#include <linux/ratelimit.h>
#include <linux/atomic.h>
#include <log/log_usertype.h>

//#define MTK_LMK_USER_EVENT

#ifdef MTK_LMK_USER_EVENT
#include <linux/miscdevice.h>
#endif

#ifdef CONFIG_HUAWEI_KSTATE
#include <huawei_platform/power/hw_kcollect.h>
#endif

#if defined CONFIG_LOG_JANK
#include <huawei_platform/log/log_jank.h>
#include <huawei_platform/log/janklogconstants.h>
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
#endif

#ifdef CONFIG_MEMCG_PROTECT_LRU
#include <linux/protect_lru.h>
#endif

#include "internal.h"

#include "lowmem_dbg.h"

#ifdef CONFIG_HUAWEI_LMK_DBG
#define HW_LMK_CACHED_ADJ 950 /* from adj 950 to print info */
#define HW_LMK_INTERVAL 10 /* interval 10s to print info */
static unsigned long long last_jiffs;
#endif

static DEFINE_SPINLOCK(lowmem_shrink_lock);
static short lowmem_warn_adj, lowmem_no_warn_adj = 200;
static u32 lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};

static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 4;
#if defined CONFIG_LOG_JANK
static int jank_buffer_size = 255;
static bool is_first_read_total_memory = true;
static bool is_killed_log_enable;
static int base_size = 3000000; /* 3G */
#endif

static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Don't bother LMK when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_node_page_state(NR_ACTIVE_ANON) +
		global_node_page_state(NR_ACTIVE_FILE) +
		global_node_page_state(NR_INACTIVE_ANON) +
		global_node_page_state(NR_INACTIVE_FILE);
}

#ifdef CONFIG_LOG_JANK
static void upload_to_jank(struct task_struct *tsk, short oom_adj)
{
	if (is_first_read_total_memory) {
		is_first_read_total_memory = false;
		is_killed_log_enable =
			((totalram_pages << (PAGE_SHIFT - 10)) > base_size) &&
			(get_logusertype_flag() == BETA_USER);
	}
	if (is_killed_log_enable) {
		if (tsk->mm) {
			unsigned long anon;
			unsigned long file;
			unsigned long shmem;
			char jbuffer[jank_buffer_size];

			memset(jbuffer, 0, jank_buffer_size);
			anon = get_mm_counter(tsk->mm, MM_ANONPAGES);
			file = get_mm_counter(tsk->mm, MM_FILEPAGES);
			shmem = get_mm_counter(tsk->mm, MM_SHMEMPAGES);
			snprintf(jbuffer, jank_buffer_size,
				 "lmk,%d,%s,%hd,%luKb,%luKb,%luKb,lmk",
				 tsk->pid, tsk->comm, oom_adj,
				 anon << (PAGE_SHIFT - 10),
				 file << (PAGE_SHIFT - 10),
				 shmem << (PAGE_SHIFT - 10));
			LOG_JANK_D(JLID_PROCESS_KILLED, "%s", jbuffer);
		}
	}
}
#endif

#ifdef MTK_LMK_USER_EVENT
static const struct file_operations mtklmk_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice mtklmk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtklmk",
	.fops = &mtklmk_fops,
};

static struct work_struct mtklmk_work;
static int uevent_adj, uevent_minfree;
static void mtklmk_async_uevent(struct work_struct *work)
{
#define MTKLMK_EVENT_LENGTH	(24)
	char adj[MTKLMK_EVENT_LENGTH], free[MTKLMK_EVENT_LENGTH];
	char *envp[3] = { adj, free, NULL };

	snprintf(adj, MTKLMK_EVENT_LENGTH, "OOM_SCORE_ADJ=%d", uevent_adj);
	snprintf(free, MTKLMK_EVENT_LENGTH, "MINFREE=%d", uevent_minfree);
	kobject_uevent_env(&mtklmk_misc.this_device->kobj, KOBJ_CHANGE, envp);
#undef MTKLMK_EVENT_LENGTH
}

static unsigned int mtklmk_initialized;
static unsigned int mtklmk_uevent_timeout = 10000; /* ms */
module_param_named(uevent_timeout, mtklmk_uevent_timeout, uint, 0644);
static void mtklmk_uevent(int oom_score_adj, int minfree)
{
	static unsigned long last_time;
	unsigned long timeout;

	/* change to use jiffies */
	timeout = msecs_to_jiffies(mtklmk_uevent_timeout);

	if (!last_time)
		last_time = jiffies - timeout;

	if (time_before(jiffies, last_time + timeout))
		return;

	last_time = jiffies;

	uevent_adj = oom_score_adj;
	uevent_minfree = minfree;
	schedule_work(&mtklmk_work);
}
#endif

#ifndef CONFIG_MTK_ENABLE_AGO
/* Check memory status by zone, pgdat */
static int lowmem_check_status_by_zone(enum zone_type high_zoneidx,
				       int *other_free, int *other_file)
{
	struct pglist_data *pgdat = NULL;
	struct zone *z = NULL;
	enum zone_type zoneidx;
	unsigned long accumulated_pages = 0;
	u64 scale = (u64)totalram_pages;
	int new_other_free = 0, new_other_file = 0;
	int memory_pressure = 0;
	int unreclaimable = 0;

	if (high_zoneidx < MAX_NR_ZONES - 1) {
		/* Go through all memory nodes */
		for_each_online_pgdat(pgdat) {
			for (zoneidx = 0; zoneidx <= high_zoneidx; zoneidx++) {
				z = pgdat->node_zones + zoneidx;
				accumulated_pages += z->managed_pages;
				new_other_free +=
					zone_page_state(z, NR_FREE_PAGES);
				new_other_free -= high_wmark_pages(z);
				new_other_file +=
				zone_page_state(z, NR_ZONE_ACTIVE_FILE) +
				zone_page_state(z, NR_ZONE_INACTIVE_FILE);

				/* Compute memory pressure level */
				memory_pressure +=
				zone_page_state(z, NR_ZONE_ACTIVE_FILE) +
				zone_page_state(z, NR_ZONE_INACTIVE_FILE) +
#ifdef CONFIG_SWAP
				zone_page_state(z, NR_ZONE_ACTIVE_ANON) +
				zone_page_state(z, NR_ZONE_INACTIVE_ANON) +
#endif
				new_other_free;
			}

			/*
			 * Consider pgdat as unreclaimable when hitting one of
			 * following two cases,
			 * 1. Memory node is unreclaimable in vmscan.c
			 * 2. Memory node is reclaimable, but nearly no user
			 *    pages(under high wmark)
			 */

			if (!pgdat_reclaimable(pgdat) ||
			    (pgdat_reclaimable(pgdat) && memory_pressure < 0))
				unreclaimable++;
		}

		/*
		 * Update if we go through ONLY lower zone(s) ACTUALLY
		 * and scale in totalram_pages
		 */
		if (totalram_pages > accumulated_pages) {
			do_div(scale, accumulated_pages);
			if ((u64)totalram_pages >
			    (u64)accumulated_pages * scale)
				scale += 1;
			new_other_free *= scale;
			new_other_file *= scale;
		}

		/*
		 * Update if not kswapd or
		 * "being kswapd and high memory pressure"
		 */
		if (!current_is_kswapd() ||
		    (current_is_kswapd() && memory_pressure < 0)) {
			*other_free = new_other_free;
			*other_file = new_other_file;
		}
	}

	return unreclaimable;
}
#else
static int lowmem_check_status_by_zone(enum zone_type high_zoneidx,
				       int *other_free, int *other_file)
{
	return 0;
}
#endif

static void __lowmem_trigger_warning(struct task_struct *selected)
{
#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#define MSG_SIZE_TO_AEE 70
	char msg_to_aee[MSG_SIZE_TO_AEE];

	lowmem_print(1, "low memory trigger kernel warning\n");
	snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
		 "please contact AP/AF memory module owner[pid:%d]\n",
		 selected->pid);

	aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
			       DB_OPT_DUMPSYS_ACTIVITY |
			       DB_OPT_LOW_MEMORY_KILLER |
			       DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
			       DB_OPT_PROCESS_COREDUMP |
			       DB_OPT_DUMPSYS_SURFACEFLINGER |
			       DB_OPT_DUMPSYS_GFXINFO |
			       DB_OPT_DUMPSYS_PROCSTATS,
			       "Framework low memory\nCRDISPATCH_KEY:FLM_APAF",
			       msg_to_aee);
#undef MSG_SIZE_TO_AEE
#else
	pr_info("(%s) no warning triggered for selected(%s)(%d)\n",
		__func__, selected->comm, selected->pid);
#endif
}

/* try to trigger warning to get more information */
static void lowmem_trigger_warning(struct task_struct *selected,
				   short selected_oom_score_adj)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 60 * HZ, 1);

	if (selected_oom_score_adj > lowmem_warn_adj)
		return;

	if (!__ratelimit(&ratelimit))
		return;

	__lowmem_trigger_warning(selected);
}

/* try to dump more memory status */
static void dump_memory_status(short selected_oom_score_adj)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 5 * HZ, 1);
	static DEFINE_RATELIMIT_STATE(ratelimit_urgent, 2 * HZ, 1);

	if (selected_oom_score_adj > lowmem_warn_adj &&
	    !__ratelimit(&ratelimit))
		return;

	if (!__ratelimit(&ratelimit_urgent))
		return;

	show_task_mem();
	show_free_areas(0, NULL);
	oom_dump_extra_info();
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk = NULL;
	struct task_struct *selected = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages;
	int other_file = global_node_page_state(NR_FILE_PAGES) -
				global_node_page_state(NR_SHMEM) -
				global_node_page_state(NR_UNEVICTABLE) -
#ifdef CONFIG_MEMCG_PROTECT_LRU
				get_protected_pages() -
#endif
				total_swapcache_pages();
	enum zone_type high_zoneidx = gfp_zone(sc->gfp_mask);
	int d_state_is_found = 0;
	short other_min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int to_be_aggressive = 0;

#ifdef CONFIG_HUAWEI_LMK_DBG
	unsigned long long jiffs = 0;
#endif

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

	/*
	 * Check whether it is caused by low memory in lower zone(s)!
	 * This will help solve over-reclaiming situation while total number
	 * of free pages is enough, but lower one(s) is(are) under low memory.
	 */
	if (lowmem_check_status_by_zone(high_zoneidx, &other_free, &other_file)
			> 0)
		other_min_score_adj = 0;


	/* Let other_free be positive or zero */
	if (other_free < 0)
		other_free = 0;

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			if (to_be_aggressive != 0 && i > 3) {
				i -= to_be_aggressive;
				if (i < 3)
					i = 3;
			}
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

	/* Compute suitable min_score_adj */
	min_score_adj = min(min_score_adj, other_min_score_adj);

	lowmem_print(3, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
		     sc->nr_to_scan, sc->gfp_mask, other_free,
		     other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);
		spin_unlock(&lowmem_shrink_lock);
		return SHRINK_STOP;
	}

	selected_oom_score_adj = min_score_adj;

	rcu_read_lock();

	for_each_process(tsk) {
		struct task_struct *p = NULL;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (task_lmk_waiting(tsk) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
			rcu_read_unlock();
			spin_unlock(&lowmem_shrink_lock);
			return SHRINK_STOP;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		/* Bypass D-state process */
		if (p->state & TASK_UNINTERRUPTIBLE) {
			lowmem_print(2,
				     "lowmem_scan filter D state process: %d (%s) state:0x%lx\n",
				     p->pid, p->comm, p->state);
			task_unlock(p);
			d_state_is_found = 1;
			continue;
		}

		oom_score_adj = p->signal->oom_score_adj;
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

		tasksize = get_mm_rss(p->mm) +
			get_mm_counter(p->mm, MM_SWAPENTS);
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
	}
	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);

		task_lock(selected);
#ifdef CONFIG_LOG_JANK
		upload_to_jank(selected, selected_oom_score_adj);
#endif
		send_sig(SIGKILL, selected, 0);
		if (selected->mm)
			task_set_lmk_waiting(selected);
		task_unlock(selected);
		lowmem_print(1, "Killing '%s' (%d) (tgid %d), adj %hd,\n"
				 "   to free %ldkB on behalf of '%s' (%d) because\n"
				 "   cache %ldkB is below limit %ldkB for oom_score_adj %hd (%hd)\n"
				 "   Free memory is %ldkB above reserved(decrease %d level)\n",
			     selected->comm, selected->pid, selected->tgid,
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     cache_size, cache_limit,
			     min_score_adj, other_min_score_adj,
			     free, to_be_aggressive);

		lowmem_dbg(selected_oom_score_adj);
		lowmem_deathpending_timeout = jiffies + HZ;

#ifdef CONFIG_HUAWEI_KSTATE
		/*0 stand for low memory kill*/
		hw_kill_cb(selected->tgid, 0);
#endif

		lowmem_trigger_warning(selected, selected_oom_score_adj);

		rem += selected_tasksize;
	} else {
		if (d_state_is_found == 1)
			lowmem_print(2,
				     "No selected (full of D-state processes at %d)\n",
				     (int)min_score_adj);
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();
	spin_unlock(&lowmem_shrink_lock);

	/* dump more memory info outside the lock */
#ifndef CONFIG_HUAWEI_LMK_DBG
	if (selected && selected_oom_score_adj <= lowmem_no_warn_adj &&
	    min_score_adj <= lowmem_warn_adj)
		dump_memory_status(selected_oom_score_adj);
#else
	if (selected_oom_score_adj <= HW_LMK_CACHED_ADJ) {
		jiffs = get_jiffies_64();
		if (time_after64(jiffs, (last_jiffs + HW_LMK_INTERVAL * HZ))) {
			last_jiffs = get_jiffies_64();
			dump_memory_status(selected_oom_score_adj);
		}
	}
#endif

#ifdef MTK_LMK_USER_EVENT
	/* Send uevent if needed */
	if (mtklmk_initialized && current_is_kswapd() && mtklmk_uevent_timeout)
		mtklmk_uevent(min_score_adj, minfree);
#endif

	if (!rem)
		rem = SHRINK_STOP;

	return rem;
}

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	if (IS_ENABLED(CONFIG_ZRAM) &&
	    IS_ENABLED(CONFIG_MTK_GMO_RAM_OPTIMIZE))
		vm_swappiness = 100;

	register_shrinker(&lowmem_shrinker);

#ifdef MTK_LMK_USER_EVENT
	/* initialize work for uevent */
	INIT_WORK(&mtklmk_work, mtklmk_async_uevent);

	/* register as misc device */
	if (!misc_register(&mtklmk_misc)) {
		pr_info("%s: successful to register misc device!\n", __func__);
		mtklmk_initialized = 1;
	}
#endif

	return 0;
}
device_initcall(lowmem_init);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * not really modular, but the easiest way to keep compat with existing
 * bootargs behaviour is to continue using module_param here.
 */
module_param_named(cost, lowmem_shrinker.seeks, int, 0644);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj,
		0644);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size, 0644);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 0644);
module_param_named(debug_level, lowmem_debug_level, uint, 0644);
module_param_named(debug_adj, lowmem_warn_adj, short, 0644);
module_param_named(no_debug_adj, lowmem_no_warn_adj, short, 0644);
