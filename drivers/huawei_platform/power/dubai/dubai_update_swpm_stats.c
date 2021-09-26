#include <linux/jiffies.h>

extern int32_t sync_latest_data(void);

#define MTK_SWPM_STATS_UPDATE_DEALY		(HZ / 50)

void dubai_update_swpm_stats(void)
{
	static unsigned long last_update = 0;

	if (time_is_before_jiffies(last_update + MTK_SWPM_STATS_UPDATE_DEALY)) {
		sync_latest_data();
		last_update = jiffies;
	}
}

