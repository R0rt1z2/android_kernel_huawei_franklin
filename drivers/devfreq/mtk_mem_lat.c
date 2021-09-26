#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/memlat_plat.h>

void plat_init_hw_vote(struct vote_reg *hw_vote, const char *name_str)
{
	hw_vote->pm_qos_class = PM_QOS_DDR_OPP;
	pm_qos_add_request(&hw_vote->opp_req,
			   PM_QOS_DDR_OPP, PM_QOS_DDR_OPP_DEFAULT_VALUE);
}
EXPORT_SYMBOL(plat_init_hw_vote);

void plat_set_hw_vote(struct vote_reg *hw_vote,
		      unsigned long freq, int idx)
{
	pm_qos_update_request(&hw_vote->opp_req, idx);
}
EXPORT_SYMBOL(plat_set_hw_vote);
