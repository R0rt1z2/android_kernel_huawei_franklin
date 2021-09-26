#ifndef _PD_DBG_INFO_H_
#define _PD_DBG_INFO_H_

#ifdef CONFIG_HI6526_PD_DBG_INFO
int hi6526_pd_dbg_info(const char *fmt, ...);
void hi6526_pd_dbg_info_lock(void);
void hi6526_pd_dbg_info_unlock(void);
#define RT_DBG_INFO hi6526_pd_dbg_info
#define RT_DBG_ERR hi6526_pd_dbg_info
#else
static inline int hi6526_pd_dbg_info(const char *fmt, ...)
{ return 0; }
static inline void hi6526_pd_dbg_info_lock(void) { }
static inline void hi6526_pd_dbg_info_unlock(void) { }
#define RT_DBG_INFO pr_info
#define RT_DBG_ERR pr_info
#endif /* CONFIG_HI6526_PD_DBG_INFO */

/* The switch of log message */
#define TYPEC_INFO_ENABLE	1
#define PE_EVENT_DBG_ENABLE	1
#define PE_STATE_INFO_ENABLE	1
#define PE_INFO_ENABLE		1
#define TCPC_INFO_ENABLE	1
#define TCPC_DBG_ENABLE		0
#define DPM_DBG_ENABLE		1
#define PD_ERR_ENABLE		1
#define PE_DBG_ENABLE		0
#define TYPEC_DBG_ENABLE	1

#define DP_INFO_ENABLE		1
#define DP_DBG_ENABLE		1

#define UVDM_INFO_ENABLE	1

#if TYPEC_DBG_ENABLE
#define TYPEC_DBG(format, args...)		\
	RT_DBG_INFO("[TPC-D]" format, ##args)
#else
#define TYPEC_DBG(format, args...)
#endif /* TYPEC_DBG_ENABLE */

#if TYPEC_INFO_ENABLE
#define TYPEC_INFO(format, args...)	\
	RT_DBG_INFO("TPC-I:" format, ##args)
#else
#define TYPEC_INFO(format, args...)
#endif /* TYPEC_INFO_ENABLE */

#if TCPC_INFO_ENABLE
#define TCPC_INFO(format, args...)	\
	RT_DBG_INFO("[TCPC-I]" format, ##args)
#else
#define TCPC_INFO(foramt, args...)
#endif /* TCPC_INFO_ENABLE */

#if TCPC_DBG_ENABLE
#define TCPC_DBG(format, args...)	\
	RT_DBG_INFO("[TCPC-D]" format, ##args)
#else
#define TCPC_DBG(format, args...)
#endif /* TCPC_DBG_ENABLE */

#define TCPC_ERR(format, args...)	\
	RT_DBG_ERR("[TCPC-E]" format, ##args)

#define DP_ERR(format, args...)	\
	RT_DBG_ERR("[DP-E]" format, ##args)

#if DPM_DBG_ENABLE
#define DPM_DBG(format, args...)	\
	RT_DBG_INFO("DPM-D:" format, ##args)
#else
#define DPM_DBG(format, args...)
#endif /* DPM_DBG_ENABLE */

#if PD_ERR_ENABLE
#define PD_ERR(format, args...) \
	RT_DBG_ERR("PD-E:" format, ##args)
#else
#define PD_ERR(format, args...)
#endif /* PD_ERR_ENABLE */

#if PE_INFO_ENABLE
#define PE_INFO(format, args...)	\
	RT_DBG_INFO("PE:" format, ##args)
#else
#define PE_INFO(format, args...)
#endif /* PE_INFO_ENABLE */

#if PE_EVENT_DBG_ENABLE
#define PE_EVT_INFO(format, args...) \
	RT_DBG_INFO("PE-E:" format, ##args)
#else
#define PE_EVT_INFO(format, args...)
#endif /* PE_EVENT_DBG_ENABLE */

#if PE_DBG_ENABLE
#define PE_DBG(format, args...)	\
	RT_DBG_INFO("PE:" format, ##args)
#else
#define PE_DBG(format, args...)
#endif /* PE_DBG_ENABLE */

#if PE_STATE_INFO_ENABLE
#define PE_STATE_INFO(format, args...) \
	RT_DBG_INFO("PE:" format, ##args)
#else
#define PE_STATE_INFO(format, args...)
#endif /* PE_STATE_IFNO_ENABLE */

#if DP_INFO_ENABLE
#define DP_INFO(format, args...)	\
	RT_DBG_INFO("DP:" format, ##args)
#else
#define DP_INFO(format, args...)
#endif /* DP_INFO_ENABLE */

#if DP_DBG_ENABLE
#define DP_DBG(format, args...)	\
	RT_DBG_INFO("DP:" format, ##args)
#else
#define DP_DBG(format, args...)
#endif /* DP_DBG_ENABLE */

#if UVDM_INFO_ENABLE
#define UVDM_INFO(format, args...)	\
	RT_DBG_INFO("UVDM:" format, ## args)
#else
#define UVDM_INFO(format, args...)
#endif

/*****************************************************************/

#ifndef LOG_TAG
#define LOG_TAG ""
#endif

#ifdef CONFIG_HI6526_TCPC_DEBUG
#define D(format, arg...) RT_DBG_INFO(LOG_TAG "[%s]" format, __func__, ##arg)
#else
#define D(format, arg...) do {} while (0)
#endif
#define I(format, arg...) RT_DBG_INFO(LOG_TAG "[%s]" format, __func__, ##arg)
#define E(format, arg...) pr_err("[ERR]" LOG_TAG "[%s]" format, __func__, ##arg)

#endif /* _PD_DBG_INFO_H_ */
