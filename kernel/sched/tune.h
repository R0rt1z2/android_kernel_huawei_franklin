
#ifdef CONFIG_SCHED_TUNE

#include <linux/reciprocal_div.h>

/*
 * System energy normalization constants
 */
struct target_nrg {
	unsigned long min_power;
	unsigned long max_power;
	struct reciprocal_value rdiv;
};

int schedtune_cpu_boost(int cpu);
int schedtune_task_boost(struct task_struct *tsk);

int schedtune_prefer_idle(struct task_struct *tsk);

void schedtune_enqueue_task(struct task_struct *p, int cpu);
void schedtune_dequeue_task(struct task_struct *p, int cpu);
extern int stune_task_threshold;

#ifdef CONFIG_UCLAMP_TASK_GROUP
extern struct mutex uclamp_mutex;
extern int opp_capacity_tbl_ready;
extern void init_opp_capacity_tbl(void);
extern unsigned int find_fit_capacity(unsigned int cap);
extern  void uclamp_group_get(struct task_struct *p,
			     struct cgroup_subsys_state *css,
			     struct uclamp_se *uc_se,
			     unsigned int clamp_id, unsigned int clamp_value);
extern void uclamp_group_put(unsigned int clamp_id, unsigned int group_id);
#endif

#ifdef CONFIG_HW_RTG
extern bool same_schedtune(struct task_struct *new, struct task_struct *leader);
#endif
#ifdef CONFIG_HW_TOP_TASK
int schedtune_top_task(struct task_struct *tsk);
#endif

#else /* CONFIG_SCHED_TUNE */

#define schedtune_cpu_boost(cpu)  0
#define schedtune_task_boost(tsk) 0

#define schedtune_prefer_idle(tsk) 0

#define schedtune_enqueue_task(task, cpu) do { } while (0)
#define schedtune_dequeue_task(task, cpu) do { } while (0)
#define stune_task_threshold 0

#ifdef CONFIG_HW_TOP_TASK
static inline int schedtune_top_task(struct task_struct *tsk)
{
	return 0;
}
#endif

#endif /* CONFIG_SCHED_TUNE */

#ifdef CONFIG_HW_CPU_FREQ_GOV_SCHEDUTIL
#ifdef CONFIG_SCHED_TUNE
int schedtune_freq_boost(int cpu);
#else
static inline int schedtune_freq_boost(int cpu) { return 0; }
#endif
#endif
