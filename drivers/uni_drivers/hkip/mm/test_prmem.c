/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: test cases of prmem
 * Author: Igor Stoppa <igor.stoppa@huawei.com>
 * Creator: security-ap
 * Date: 2020/04/15
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/stringify.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/irqflags.h>
#include <linux/arm-smccc.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <uni/hkip/prmem.h>
#include <linux/set_memory.h>

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "test_prmem : " fmt

#define test_case(pass_condition, test_type, test_name)			\
do {									\
	if (!(pass_condition)) {					\
		pr_err(test_type ": " test_name " test failed");	\
		return false;						\
	}								\
	pr_info(test_type ": " test_name " test successful");		\
} while(0)

/* ------------------------ Static Allocation ------------------------ */

static __wr int wr_scalar = '0';
static __wr_after_init int wr_after_init_scalar = '0';
static __rw int rw_scalar = '5';
static __wr u8 wr_array[PAGE_SIZE * 3] __aligned(PAGE_SIZE);

static const char zero = '0';
static const char one = '1';
static const char two = '2';

static __init void attempt_init_writes(void)
{
	if (!prmem_active())
		pr_info("Runtime: init wr protection test skipped");
	else
		probe_kernel_write(&wr_scalar, &two, sizeof(two));
	if (wr_scalar == zero)
		wr_assign(wr_scalar, one);
	probe_kernel_write(&wr_after_init_scalar, &one, sizeof(one));
}

static bool test_wr_post_init(void)
{
	test_case(wr_after_init_scalar == one,
		  "Static", "__wr_after_init write during init");
	test_case(wr_scalar == one, "Static", "__wr write during init");
	wr_assign(wr_after_init_scalar, two);
	test_case(wr_after_init_scalar == two,
		  "Static", "__wr_after_init write post init");
	wr_assign(wr_scalar, two);
	test_case(wr_scalar == two, "Static", "__wr write post init");
	wr_assign(wr_after_init_scalar, zero);
	wr_assign(wr_scalar, zero);
	return true;
}

static bool test_section_boundaries(unsigned long start,
				    unsigned long end)
{
	if (start < end)
		return true;
	pr_err("section start: 0x%016lx, end: 0x%016lx", start, end);
	return false;
}

/* The section must occupy a non-zero number of whole pages */
static bool test_section(void *start, void *end)
{
	unsigned long pstart = (unsigned long)(uintptr_t)start;
	unsigned long pend = (unsigned long)(uintptr_t)end;

	test_case(!(pstart & ~PAGE_MASK), "Static",
		  "section start alignment");
	test_case(!(pend & ~PAGE_MASK), "Static", "section end alignment");
	test_case(test_section_boundaries(pstart, pend),
		  "Static", "section start < end");
	return true;
}

static bool test_alignment(void)
{
#ifdef CONFIG_HKIP_PRMEM
	test_case(test_section(__start_data_wr_after_init,
			       __end_data_wr_after_init),
		  "Static", "wr_after_init section");
	test_case(test_section(__start_data_wr, __end_data_wr),
		  "Static", "wr section");
	test_case(test_section(__start_data_rw, __end_data_rw),
		  "Static", "rw section");
#endif
	return true;
}

static bool test_pattern(void)
{
	if (memchr_inv(wr_array, '0', PAGE_SIZE / 2))
		pr_info("Pattern part 1 failed");
	if (memchr_inv(wr_array + PAGE_SIZE / 2, '1', PAGE_SIZE * 3 / 4))
		pr_info("Pattern part 2 failed");
	if (memchr_inv(wr_array + PAGE_SIZE * 5 / 4, '0', PAGE_SIZE / 2))
		pr_info("Pattern part 3 failed");
	if (memchr_inv(wr_array + PAGE_SIZE * 7 / 4, '1',
		       PAGE_SIZE * 3 / 4))
		pr_info("Pattern part 4 failed");
	if (memchr_inv(wr_array + PAGE_SIZE * 5 / 2, '0', PAGE_SIZE / 2))
		pr_info("Pattern part 5 failed");
	return true;
}

static __wr __aligned(sizeof(uint8_t))
uint8_t wr_uint8_var = 0x5A;

static __wr __aligned(sizeof(uint16_t))
uint16_t wr_uint16_var = 0x5A5A;

static __wr __aligned(sizeof(uint32_t))
uint32_t wr_uint32_var = 0x5A5A5A5A;

static __wr __aligned(sizeof(uint64_t))
uint64_t wr_uint64_var = 0x5A5A5A5A5A5A5A5A;

static bool test_wr_memset(void)
{
	int new_val = '1';

	/* Scalar Test */
	wr_memset(&wr_scalar, new_val, sizeof(wr_scalar));
	if (memchr_inv(&wr_scalar, new_val, sizeof(wr_scalar))) {
		pr_err("Static: scalar wr_memset failed - "
		       "expected: 0x%016lx  actual: 0x%016lx",
		       new_val, wr_scalar);
		return false;
	}
	pr_info("Static: scalar wr_memset passed");

	wr_memset((void *)&rw_scalar, new_val, sizeof(rw_scalar));
	if (memchr_inv((void *)&rw_scalar, new_val, sizeof(rw_scalar))) {
		pr_err("Static: scalar wr_memset on rw data failed - "
		       "expected: 0x%016lx  actual: 0x%016lx",
		       new_val, wr_scalar);
		return false;
	}
	pr_info("Static: scalar wr_memset on rw data passed");

	/* Array Test, the numbers below represent different page sizes */
	wr_memset(wr_array, '0', PAGE_SIZE * 3);
	test_case(!memchr_inv(wr_array, '0', PAGE_SIZE * 3),
		  "Static", "array page aligned wr_memset");

	wr_memset(wr_array + PAGE_SIZE / 2, '1', PAGE_SIZE * 2);
	test_case(!memchr_inv(wr_array + PAGE_SIZE / 2, '1', PAGE_SIZE * 2),
		  "Static", "array 1/2 page aligned wr_memset");

	wr_memset(wr_array + PAGE_SIZE * 5 / 4, '0', PAGE_SIZE / 2);
	test_case(!memchr_inv(wr_array + PAGE_SIZE * 5 / 4, '0', PAGE_SIZE / 2),
		  "Static", "array 1/4 page aligned wr_memset");

	test_case(test_pattern(), "Static", "array wr_memset");

	/* Immediates Test */
	wr_assign(wr_uint8_var, ~wr_uint8_var);
	test_case(wr_uint8_var == 0xA5, "Static", "uint8_t wr_assign");

	wr_assign(wr_uint16_var, ~wr_uint16_var);
	test_case(wr_uint16_var == 0xA5A5, "Static", "uint16_t wr_assign");

	wr_assign(wr_uint32_var, ~wr_uint32_var);
	test_case(wr_uint32_var == 0xA5A5A5A5,
		  "Static", "uint32_t wr_assign");

	wr_assign(wr_uint64_var, ~wr_uint64_var);
	test_case(wr_uint64_var == 0xA5A5A5A5A5A5A5A5,
		  "Static", "uint64_t wr_assign");
	pr_info("Static: immediates wr_assign test successful");
	return true;
}

static u8 array_1[PAGE_SIZE * 2];
static u8 array_2[PAGE_SIZE * 2];
static __wr int *dst;
static int reference = 0x54;

static bool test_wr_memcpy(void)
{
	int new_val = 0x12345678;

	/* Scalar Test */
	wr_assign(wr_scalar, new_val);
	test_case(!memcmp(&wr_scalar, &new_val, sizeof(wr_scalar)),
		  "Static", "scalar wr_memcpy");

	/* Array Test */
	wr_memset(wr_array, '0', PAGE_SIZE * 3);
	memset(array_1, '1', PAGE_SIZE * 2);
	memset(array_2, '0', PAGE_SIZE * 2);
	wr_memcpy(wr_array + PAGE_SIZE / 2, array_1, PAGE_SIZE * 2);
	wr_memcpy(wr_array + PAGE_SIZE * 5 / 4, array_2, PAGE_SIZE / 2);
	test_case(test_pattern(), "Static", "array wr_memcopy");

	/* RCU Pointer Test */
	wr_rcu_assign_pointer(dst, &reference);
	test_case(dst == &reference, "Static", "wr_rcu_assign_pointer");

	return true;
}

static bool test_wr_ops(void)
{
	/* The operation below means : wr_scalar = 5 + 5 + 1 - 5 - 1 = 5 */
	wr_assign(wr_scalar, 5);

	wr_add(wr_scalar, 5);
	test_case(wr_scalar == 10, "Static", "wr_add operation");

	wr_inc(wr_scalar);
	test_case(wr_scalar == 11, "Static", "wr_inc operation");

	wr_sub(wr_scalar, 5);
	test_case(wr_scalar == 6, "Static", "wr_sub operation");

	wr_dec(wr_scalar);
	test_case(wr_scalar == 5, "Static", "wr_dec operation");
	return true;

}

/* Static: Test Entry Point */
static bool static_test(void)
{
	test_case(test_alignment(), "Static", "alignment");
	test_case(test_wr_post_init(), "Static", "pre-post init write");
	test_case(test_wr_memset(), "Static", "wr_memset");
	test_case(test_wr_memcpy(), "Static", "wr_memcpy");
	test_case(test_wr_ops(), "Static", "wr ops");
	pr_info("Static: test passed");
	return true;
}

/* ------------------------ Runtime Allocation ------------------------ */
PRMEM_POOL(wr_recl_pool, wr_recl, sizeof(void *), kB(32), PRMEM_NO_CAP);
PRMEM_POOL(start_wr_no_recl_pool, start_wr_no_recl, sizeof(void *), kB(32),
	   PRMEM_NO_CAP);
PRMEM_POOL(ro_recl_pool, ro_recl, sizeof(void *), kB(8), PRMEM_NO_CAP);
PRMEM_POOL(rw_recl_pool, rw_recl, sizeof(void *), kB(8), PRMEM_NO_CAP);
PRMEM_POOL(preload_wr_recl_pool, wr_recl, sizeof(void *), kB(8),
	   PRMEM_NO_CAP);

static bool test_preload(void)
{
	int preload_size;

	prmem_pool_preload(&preload_wr_recl_pool, PAGE_SIZE * 4);
	preload_size = preload_wr_recl_pool.offset;
	test_case(preload_size == PAGE_SIZE * 4,
		  "Runtime", "preload pool");
	return true;
}

static bool test_allocation(void)
{
	void *tmp = NULL;

	tmp = pmalloc(&wr_recl_pool, PAGE_SIZE, PRMEM_NO_FLAGS);
	test_case(tmp != NULL, "Runtime", "pmalloc from test_pool");
	return true;
}

extern char __end_data_start_wr_recl_prmem_pools[];
extern char __end_data_wr_recl_prmem_pools[];
#define _128MB (128 * 1024UL * 1024UL)
unsigned long qstart = PRMEM_START + (PRMEM_WR_RECL * _128MB);
unsigned long qend = PRMEM_START + ((PRMEM_WR_RECL + 1) * _128MB);

static bool test_allocation_properties(void)
{
	struct vmap_area *va = NULL;
	void *tmp = NULL;
	unsigned long zero = 0UL;
	unsigned long five = 5UL;

	/* Verify write capability on rw allocation */
	tmp = pmalloc(&rw_recl_pool, sizeof(long), PRMEM_NO_FLAGS);
	test_case(tmp != NULL, "Runtime", "allocation from rw pool");
	*(long *)tmp = 0x5A;

	prmem_protect_pool(&rw_recl_pool);
	*(long *)tmp = 0xA5;
	test_case(*(long *)tmp == 0xA5,
		  "Runtime", "assignment of protected rw memory");
	wr_assign(*(long *)tmp, 0x5A);
	test_case(*(long *)tmp == 0x5A,
		  "Runtime", "wr_assign on protectes rw memory");

	/* Verify basic protection of prmem vmap_area */
	va = prmem_vmap_area_alloc(&wr_recl_pool, PAGE_SIZE, PRMEM_NO_FLAGS);
	tmp = (void *)va->va_start;
	*(unsigned long *)tmp = 0x5AUL;
	set_prmem_ro((unsigned long)(uintptr_t)tmp, 1);
	probe_kernel_write(tmp, &zero, sizeof(zero));
	if (!prmem_active())
		test_case(*(unsigned long *)tmp == zero,
			  "Runtime", "basic inactive write protect");
	else if (!prmem_enabled())
		test_case(*(unsigned long *)tmp == zero,
			  "Runtime", "basic disabled write protect");
	else
		test_case(*(unsigned long *)tmp == 0x5AUL,
			  "Runtime", "basic write protect");
	set_prmem_rw((unsigned long)(uintptr_t)tmp, 1);
	probe_kernel_write(tmp, &zero, sizeof(zero));
	test_case(*(unsigned long *)tmp == 0x0UL,
		  "Runtime", "basic write unprotect");

	/* Verify write capability on wr allocation */
	tmp = pmalloc(&wr_recl_pool, sizeof(long), PRMEM_NO_FLAGS);
	*(long *)tmp = 0x5A;
	prmem_protect_pool(&wr_recl_pool);

	wr_assign(*(long *)tmp, 0xA5);
	test_case(*(long *)tmp == 0xA5,
		  "Runtime", "wr_assign on protected wr memory");

	if (!prmem_active()) {
		pr_info("Runtime: vmap_area_nodes protection test skipped");
	} else {
		/* Verify that the vmap_area_node is protected */
		probe_kernel_write(wr_recl_pool.pmalloc_list, &five,
				   sizeof(five));
		test_case(*(unsigned long *)wr_recl_pool.pmalloc_list !=
			  five, "Runtime", "vmap_area_nodes protection");
	}
	return true;
}

static bool test_allocations_vs_protection(void)
{
	void *tmp1 = NULL;
	void *tmp2 = NULL;

	tmp1 = pmalloc(&wr_recl_pool, sizeof(void *), PRMEM_NO_FLAGS);
	test_case(!prmem_pool_pmalloc_protected(&wr_recl_pool),
		  "Runtime", "unprotected pool");
	prmem_protect_pool(&wr_recl_pool);
	test_case(prmem_pool_pmalloc_protected(&wr_recl_pool),
		  "Runtime", "protected pool");
	tmp2 = pmalloc(&wr_recl_pool, sizeof(void *), PRMEM_NO_FLAGS);
	test_case(!prmem_pool_pmalloc_protected(&wr_recl_pool),
		  "Runtime", "unprotected pool");
	/*
	 * protecting the pool memory after allocating tmp1 will cause
	 * tmp2 to be allocated from a different page.
	 */
	test_case((((unsigned long)(uintptr_t)tmp1) & PAGE_MASK) !=
		  (((unsigned long)(uintptr_t)tmp2) & PAGE_MASK),
		  "Runtime", "allocation from new page");

	tmp1 = pmalloc(&start_wr_no_recl_pool, sizeof(void *),
		       PRMEM_NO_FLAGS);
	tmp2 = pmalloc(&start_wr_no_recl_pool, sizeof(void *),
		       PRMEM_NO_FLAGS);
	/*
	 * Since the pool is of type "start_wr", it allows for allocating
	 * from protected pages, the user is expected to be aware of the
	 * protection and acting accordingly.
	 * The pool had no previous allocations, to tmp1 and tmp2 will be
	 * sequential.
	 */
	test_case((((unsigned long)(uintptr_t)tmp1) & PAGE_MASK) ==
		  (((unsigned long)(uintptr_t)tmp2) & PAGE_MASK),
		  "Runtime", "allocation from same page");
	return true;
}

static bool test_pmalloc_alignment(void)
{
#ifdef CONFIG_HKIP_PRMEM
	void *tmp1 = NULL;

	tmp1 = pmalloc_aligned(&ro_recl_pool, 8, ilog2(PAGE_SIZE),
			       PRMEM_NO_FLAGS);
	test_case(((~PAGE_MASK) & (unsigned long)(uintptr_t)tmp1) == 0,
		  "Runtime", "allocation alignment request");
#endif /* CONFIG_HKIP_PRMEM */
	return true;
}

void prmem_print_pool(struct prmem_pool *p);

PRMEM_POOL(wr_cap_pool, start_wr_recl, sizeof(void *), PAGE_SIZE,
	   8 * PAGE_SIZE);
static bool test_pool_cap(void)
{
	struct vmap_area *v = NULL;
	int i;

	/* At this point the pool can allocate 8 pages */
	for (i = 0; i < 7; i++)
		if (pmalloc(&wr_cap_pool, PAGE_SIZE, PRMEM_NO_FLAGS) ==
		    NULL) {
			pr_err("Runtime: pool cap test failed");
			return false;
		}
	/* 7 are taken in use */
	v = prmem_vmap_area_alloc(&wr_cap_pool, PAGE_SIZE,
				  PRMEM_FREEABLE_VA);

	/* the last one is allcoated as disposable vmap_area*/
	test_case(v != NULL, "Runtime", "pool cap alloc allow");

	/* a new allocation is expected to fail */
	test_case(pmalloc(&wr_cap_pool, 10, PRMEM_NO_FLAGS) == NULL,
		  "Runtime", "pool cap alloc reject");

	/* release the 1-page vmap_area */
	prmem_vmap_area_free(v);

	/* try - and fail - to allocate more than 1 page */
	test_case(pmalloc(&wr_cap_pool, PAGE_SIZE + 10, PRMEM_NO_FLAGS) ==
		  NULL, "Runtime", "pool cap alloc reject");

	/* try - and succeed - to allocate twice half page */
	test_case(pmalloc(&wr_cap_pool, PAGE_SIZE / 2, PRMEM_NO_FLAGS) !=
		  NULL, "Runtime", "pool cap alloc allow");
	test_case(pmalloc(&wr_cap_pool, PAGE_SIZE / 2, PRMEM_NO_FLAGS) !=
		  NULL, "Runtime", "pool cap alloc allow");

	/* try - and fail - to allocate few more bytes */
	test_case(pmalloc(&wr_cap_pool, 10, PRMEM_NO_FLAGS) == NULL,
		  "Runtime", "pool cap alloc reject");
	return true;
}

static bool test_pstrdup(void)
{
	const char reference[] = "test string";
	char *duplicate;

	duplicate = pstrdup(&wr_recl_pool, reference, PRMEM_NO_FLAGS);
	test_case(memcmp(reference, duplicate, strlen(reference)) == 0,
		  "Runtime", "pstrdup");
	return true;
}

#define CACHE_UNITS 30
PRMEM_POOL(wr_cache_pool, start_wr_no_recl, sizeof(void *), kB(8),
	   PRMEM_NO_CAP);
PRMEM_CACHE(wr_cache, &wr_cache_pool, sizeof(void *), sizeof(void *));
PRMEM_POOL(rw_cache_pool, rw_recl, sizeof(void *), kB(8), PRMEM_NO_CAP);
PRMEM_CACHE(rw_cache, &rw_cache_pool, sizeof(void *), sizeof(void *));

static bool test_prmem_cache_type(struct prmem_cache *cache)
{
	void *addresses[CACHE_UNITS];
	unsigned i;

	test_case(prmem_cache_alloc(cache, PRMEM_NO_FLAGS),
		  "Runtime", "allocation from empty cache");

	test_case(prmem_cache_preload(cache, CACHE_UNITS,
				      PRMEM_NO_FLAGS) == CACHE_UNITS,
		  "Runtime", "preload CACHE_UNITS");

	for (i = 0; i < CACHE_UNITS; i++)
		if (!(addresses[i] =
		      prmem_cache_alloc(cache, PRMEM_NO_FLAGS))) {
			pr_err("Runtime: CACHE_UNITS objects allocation "
			       "test failed");
			return false;
		}
	pr_info("Runtime: CACHE_UNITS objects allocation test successful");

	for (i = 0; i < CACHE_UNITS; i++)
		prmem_cache_free(cache, addresses[i]);
	for (i = 0; i < CACHE_UNITS; i++)
		if (addresses[CACHE_UNITS - 1 - i] !=
		    prmem_cache_alloc(cache, PRMEM_NO_FLAGS)) {
			pr_err("Runtime: allocation pattern test failed");
			return false;
		}
	pr_info("Runtime: allocation pattern test successful");
	return true;
}

static bool test_prmem_cache(void)
{
	test_case(test_prmem_cache_type(&wr_cache), "Runtime", "wr cache");
	test_case(test_prmem_cache_type(&rw_cache), "Runtime", "rw cache");
	return true;
}

static struct vmap_area_node *find_vmap_area_node(
	struct vmap_area_node *node, struct vmap_area *va)
{
	while (node)
		if (node->va == va)
			return node;
		else
			node = node->next;
	return NULL;
}

static bool test_vmap_area(void)
{
	struct vmap_area *area_wr_recl1 = NULL;
	int *p = NULL;

	area_wr_recl1 = prmem_vmap_area_alloc(&wr_recl_pool, PAGE_SIZE,
					      PRMEM_FREEABLE_VA);
	test_case(area_wr_recl1 != NULL, "Runtime", "allocate vmap_area");
	test_case(find_vmap_area_node(wr_recl_pool.recl_va_list,
				      area_wr_recl1),
		  "Runtime", "reclaimable vmap_area in pool");
	test_case(vmap_area_is_wr(area_wr_recl1),
		  "Runtime", "allocate wr vmap_area");
	test_case(vmap_area_is_reclaimable(area_wr_recl1),
		  "Runtime", "allocate reclaimable vmap_area");
	if (prmem_enabled())
		test_case(!vmap_area_is_start_wr(area_wr_recl1),
			  "Runtime", "allocate non pre-protected vmap_area");

	p = (int *)area_wr_recl1->va_start;
	*p = 1;
	test_case(*p == 1, "Runtime", "set allocated vmap_area");

	prmem_vmap_area_protect(area_wr_recl1);
	wr_assign((*p), 3);
	test_case(*p == 3, "Runtime", "alter protected vmap_area");

	prmem_vmap_area_free(area_wr_recl1);
	test_case(!find_vmap_area_node(wr_recl_pool.recl_va_list,
				       area_wr_recl1),
		  "Runtime", "reclaimable vmap_area not in pool");
	return true;
}

static bool test_freeing_pool(void)
{
	prmem_protect_pool(&wr_recl_pool);
	prmem_free_pool(&wr_recl_pool);
	test_case((wr_recl_pool.pmalloc_list == NULL) &&
		  (wr_recl_pool.recl_va_list == NULL) &&
		  (wr_recl_pool.no_recl_va_list == NULL),
		  "Runtime", "free freable pool");
	/* Notice: this *will* trigger a warning */
	prmem_free_pool(&start_wr_no_recl_pool);
	test_case(start_wr_no_recl_pool.pmalloc_list,
		  "Runtime", "free unfreable pool");
	pr_info("Runtime: freeing pool test successful");
	return true;
}

static bool runtime_test(void)
{
	return (test_preload() &&
		test_allocation() &&
		test_allocation_properties() &&
		test_allocations_vs_protection() &&
		test_pmalloc_alignment() &&
		test_pstrdup() &&
		test_prmem_cache() &&
		test_vmap_area() &&
		test_pool_cap() &&
		test_freeing_pool());
}

/* ------------------------------- perf ------------------------------- */
static struct kobject *prmem_perf_kobj;
PRMEM_POOL(perf_pool, start_wr_no_recl, sizeof(struct list_head), kB(32),
	   PRMEM_NO_CAP);

static inline s64 getnstimeofdays64(void)
{
	struct timespec64 ts;

	getnstimeofday64(&ts);
	return timespec64_to_ns(&ts);
}

#define measure_iterations_time_ns(operation, iterations)		\
({									\
	long  long i = 0;						\
	s64 start;							\
	s64 end;							\
	s64 duration;							\
	unsigned long flags;						\
									\
	local_irq_save(flags);						\
	mb();								\
	start = getnstimeofdays64();					\
	mb();								\
	for (i = 0; i < (iterations); i++)				\
		operation(i);						\
	mb();								\
	end = getnstimeofdays64();					\
	mb();								\
	local_irq_restore(flags);					\
	duration = end - start;						\
	duration;							\
})

#define measure_time_ns(operation)					\
	measure_iterations_time_ns(operation, 1)

#define RW_NUM_ITERATIONS 3000000
#define WR_NUM_ITERATIONS 3000000

/* ----------- perf calibration ----------- */
static inline void calibrate_10_s(long long i)
{
	msleep(10000);
}

/* Reference measurement: shows the measured time vs a 10s sleep. */
static s64 calibrate_10_s_perf(void)
{
	return measure_time_ns(calibrate_10_s);
}

/* ----------- perf static ptr ----------- */
volatile void *static_rw_ptr = NULL;
static volatile __wr  __aligned(sizeof(uint64_t))
void *static_wr_ptr __wr = NULL;

static __always_inline void set_static_ptr_rw(long long i)
{
	static_rw_ptr = (void *)i;
}

static s64 set_static_ptr_rw_ns_perf(void)
{
	return measure_iterations_time_ns(set_static_ptr_rw,
					  RW_NUM_ITERATIONS);
}

static __always_inline void set_static_ptr_wr(long long i)
{
	wr_assign(static_wr_ptr, i);
}

static s64 set_static_ptr_wr_ns_perf(void)
{
	return measure_iterations_time_ns(set_static_ptr_wr,
					  WR_NUM_ITERATIONS);
}

/* ----------- perf runtime ptr ----------- */
static void *runtime_rw_ptr = NULL;
static void *runtime_wr_ptr = NULL;

static __always_inline void set_runtime_ptr_rw(long long i)
{
	*(void **)runtime_rw_ptr = (void *)i;
}

static s64 set_runtime_ptr_rw_ns_perf(void)
{
	if (!runtime_rw_ptr)
		runtime_rw_ptr = kmalloc(sizeof(void *), GFP_KERNEL);
	return measure_iterations_time_ns(set_runtime_ptr_rw,
					  RW_NUM_ITERATIONS);
}

static __always_inline void set_runtime_ptr_wr(long long i)
{
	wr_assign(*(unsigned long *)runtime_wr_ptr, (unsigned long)i);
}

static s64 set_runtime_ptr_wr_ns_perf(void)
{
	if (!runtime_wr_ptr)
		runtime_wr_ptr = pmalloc(&perf_pool, sizeof(void *),
					 PRMEM_NO_FLAGS);
	test_case(runtime_wr_ptr, "Perf", "runtime ptr allocation");
	return measure_iterations_time_ns(set_runtime_ptr_wr,
					  WR_NUM_ITERATIONS);
}


static void measure_all_perfs(void);
static bool perf_test(void)
{
	measure_all_perfs();
	return true;
}

/* ---------------------------- Attributes ---------------------------- */

enum test_cases {
	test_static,
	test_runtime,
	test_perf,
	TESTS_NR,
};

struct kobj_attribute *test_attrs[TESTS_NR + 1] = {[TESTS_NR] = NULL,};

struct test_attribute {
	struct kobj_attribute attr;
	bool performed;
	bool passed;
	bool (*evaluate)(void);
};

static ssize_t test_show(struct kobject *kobj,
			 struct kobj_attribute *attribute, char *buf)
{
	struct test_attribute *attr = NULL;

	attr = container_of(attribute, struct test_attribute, attr);
	if (!attr->performed)
		return snprintf(buf, PAGE_SIZE, "Unexecuted\n");
	if (!attr->passed)
		return snprintf(buf, PAGE_SIZE, "Failed\n");
	return snprintf(buf, PAGE_SIZE, "Passed\n");
}

static ssize_t test_store(struct kobject *kobj,
			  struct kobj_attribute *attribute,
			  const char *buf, size_t count)
{
	struct test_attribute *attr = NULL;

	attr = container_of(attribute, struct test_attribute, attr);
	if (!attr->performed) {
		attr->performed = true;
		attr->passed = (*attr->evaluate)();
	}
	return count;
}

enum perf_cases {
	perf_set_static_ptr_rw_ns,
	perf_set_static_ptr_wr_ns,
	perf_set_runtime_ptr_rw_ns,
	perf_set_runtime_ptr_wr_ns,
	perf_calibrate_10_s,
	PERFS_NR,
};

struct kobj_attribute *perf_attrs[PERFS_NR + 1] = {[PERFS_NR] = NULL,};

struct perf_attribute {
	struct kobj_attribute attr;
	s64 time_ns;
	s64 (*evaluate)(void);
};

static ssize_t perf_show(struct kobject *kobj,
			 struct kobj_attribute *attribute, char *buf)
{
	struct perf_attribute *attr = NULL;

	attr = container_of(attribute, struct perf_attribute, attr);
	return snprintf(buf, PAGE_SIZE, "%012lld\n", attr->time_ns);
}

static void measure_perf(struct kobj_attribute *attribute)
{
	struct perf_attribute *attr = NULL;

	attr = container_of(attribute, struct perf_attribute, attr);
	attr->time_ns = (*attr->evaluate)();
}

static ssize_t perf_store(struct kobject *kobj,
			  struct kobj_attribute *attribute,
			  const char *buf, size_t count)
{
	measure_perf(attribute);
	return count;
}

static void measure_all_perfs(void)
{
	int i;

	for (i = 0; i < PERFS_NR; i++) {
		measure_perf(perf_attrs[i]);
		msleep_interruptible(100);
	}
}

#define create_attr(type, attr_name, prefix)				\
({									\
	type *attr;							\
									\
	if ((attr = kmalloc(sizeof(type), GFP_KERNEL))) {		\
		sysfs_attr_init(&(attr->attr.attr));			\
		attr->attr.attr.name = __stringify(attr_name);		\
		attr->attr.attr.mode = 0666;				\
		attr->attr.show = prefix##_show;			\
		attr->attr.store = prefix##_store;			\
	}								\
	attr;								\
})

#define create_test_attr(name)						\
({									\
	struct test_attribute *attr;					\
									\
	attr = create_attr(struct test_attribute, name##_test, test);	\
	if (attr) {							\
		attr->performed = false;				\
		attr->passed = false;					\
		attr->evaluate = name##_test;				\
	}								\
	attr;								\
})

#define create_perf_attr(name)						\
({									\
	struct perf_attribute *attr;					\
									\
	attr = create_attr(struct perf_attribute, name##_perf, perf);	\
	if (attr) {							\
		attr->time_ns = 0;					\
		attr->evaluate = name##_perf;				\
	}								\
	attr;								\
})

#define populate_attr(type, name)					\
({									\
	struct kobj_attribute *attr;					\
									\
	attr = (struct kobj_attribute *)create_##type##_attr(name);	\
	type##_attrs[type##_##name] = (struct kobj_attribute *)		\
					(attr ? &(attr->attr) : NULL);	\
	type##_attrs[type##_##name];					\
})

#define populate_perf_attrs_cluster(name)				\
do {									\
	populate_attr(perf, name##_rw_ns);				\
	populate_attr(perf, name##_wr_ns);				\
} while(0)

static void populate_test_attrs(void)
{
	populate_attr(test, static);
	populate_attr(test, runtime);
	populate_attr(test, perf);
}

static void populate_perf_attrs(void)
{
#ifdef CONFIG_HKIP_PRMEM
	populate_attr(perf, calibrate_10_s);
	populate_perf_attrs_cluster(set_static_ptr);
	populate_perf_attrs_cluster(set_runtime_ptr);
#endif
}

extern struct kobject *prmem_kobj;
static __init int test_prmem_init_module(void)
{
	struct kobject *prmem_test_kobj;

	prmem_test_kobj = kobject_create_and_add("self_test", prmem_kobj);
	prmem_test_kobj->sd->mode |= 0777;
	populate_test_attrs();
	sysfs_create_files(prmem_test_kobj,
			   (const struct attribute **)test_attrs);

	prmem_perf_kobj = kobject_create_and_add("perf", prmem_test_kobj);
	prmem_perf_kobj->sd->mode |= 0777;
	populate_perf_attrs();
	sysfs_create_files(prmem_perf_kobj,
			   (const struct attribute **)perf_attrs);
	attempt_init_writes();
	return 0;
}

late_initcall(test_prmem_init_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Igor Stoppa <igor.stoppa@huawei.com>");
MODULE_DESCRIPTION("Test module for prmem.");
