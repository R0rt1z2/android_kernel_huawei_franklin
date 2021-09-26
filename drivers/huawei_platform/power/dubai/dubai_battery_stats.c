#include <linux/power_supply.h>

#include <chipset_common/dubai/dubai_plat.h>

#define BATTERY_POWER_SUPPLY_NAME	"battery"

static int dubai_get_capacity_rm(void)
{
	struct power_supply *psy = NULL;

	psy = power_supply_get_by_name(BATTERY_POWER_SUPPLY_NAME);
	if (!psy) {
		dubai_err("Failed to get power supply: %s", BATTERY_POWER_SUPPLY_NAME);
		return -1;
	}
	return dubai_get_psy_intprop(psy, POWER_SUPPLY_PROP_CAPACITY_RM, -1);
}

static struct dubai_battery_stats_ops batt_ops = {
	.psy_name = BATTERY_POWER_SUPPLY_NAME,
	.charge_full_amp = 1000,
	.get_capacity_rm = dubai_get_capacity_rm,
};

void dubai_mtk_battery_stats_init(void)
{
	dubai_register_module_ops(DUBAI_MODULE_BATTERY, &batt_ops);
}

void dubai_mtk_battery_stats_exit(void)
{
	dubai_unregister_module_ops(DUBAI_MODULE_BATTERY);
}
