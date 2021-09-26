#ifndef __HUAWEI_LMK_DBG_H
#define __HUAWEI_LMK_DBG_H

#ifdef CONFIG_HUAWEI_LMK_DBG
void lowmem_dbg(short oom_score_adj);
#else
static inline void lowmem_dbg(short oom_score_adj)
{
}
#endif

#endif /* __HUAWEI_LMK_DBG_H */
