#ifndef __PAGE_TRACKER_H
#define __PAGE_TRACKER_H

#define PGTRACK_TEST (1)
#define TRACK_INV        0xffu
#define TRACK_BUDDY		 0x5a5a

enum page_tracker_type {
	TRACK_SLAB,
	TRACK_LSLAB,
	TRACK_VMALLOC,
	TRACK_FILE,
	TRACK_ANON,
	TRACK_ION,
	TRACK_PROC,
	TRACK_IRQ,
	TRACK_MAX,
};

#ifdef CONFIG_HW_PAGE_TRACKER
extern struct page_ext_operations page_tracker_ops;

struct page_tracker {
	unsigned char order;
	unsigned char head;
	unsigned short type;
	unsigned int pid;
	unsigned long trace;
};

#ifdef CONFIG_PAGE_EXTENSION
extern struct page_ext_operations page_tracker_ops;
#ifdef CONFIG_FLAT_NODE_MEM_MAP
void alloc_node_tracker_map(void *pgdat);
#else
static inline void alloc_node_tracker_map(void *pgdat) {};
#endif
#else
struct page_ext {
	struct page_tracker page_tracker;
};
static inline void alloc_node_tracker_map(void *pgdat) {};
#endif

void page_tracker_show(struct page *page, unsigned int order);
void page_tracker_set_trace(struct page *page, unsigned long func, unsigned int order);
void page_tracker_set_type(struct page *page, int type, unsigned int order);
void page_tracker_set_tracker(struct page *page, unsigned int order);
void page_tracker_reset_tracker(struct page *page, unsigned int order);
void page_tracker_change_tracker(struct page *new_page, struct page *old_page);
void page_tracker_wake_up(void);

#else
static inline void page_tracker_show(struct page *page, unsigned int order) {};
static inline void page_tracker_set_trace(struct page *page, unsigned long func, unsigned int order) {};
static inline void page_tracker_set_type(struct page *page, int type, unsigned int order) {};
static inline void page_tracker_set_tracker(struct page *page, unsigned int order) {};
static inline void page_tracker_reset_tracker(struct page *page, unsigned int order) {};
static inline void page_tracker_change_tracker(struct page *new_page, struct page *old_page) {};
static inline void page_tracker_wake_up(void) {};
static inline void alloc_node_tracker_map(void *pgdat) {};

#endif

#endif
