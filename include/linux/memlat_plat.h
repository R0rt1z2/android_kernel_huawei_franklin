#ifndef __MEMLAT_PLAT_H
#define __MEMLAT_PLAT_H

#include <linux/pm_qos.h>

struct vote_reg {
	int pm_qos_class;
	struct pm_qos_request opp_req;
};

#endif

