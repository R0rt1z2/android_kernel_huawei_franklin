#ifndef DUBAI_WAKEUP_STATS_H
#define DUBAI_WAKEUP_STATS_H

#include <linux/types.h>

extern void dubai_log_irq_wakeup(const char *source);
extern void dubai_log_sensorhub_wakeup(int source, int reason);

#endif // DUBAI_WAKEUP_STATS_H
