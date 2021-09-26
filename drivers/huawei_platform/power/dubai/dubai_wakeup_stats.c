#include <linux/suspend.h>

#include <chipset_common/dubai/dubai_plat.h>
#include <huawei_platform/power/dubai/dubai_wakeup_stats.h>

#define WAKEUP_SOURCE_SIZE						32

static bool wakeup_state = false;
static atomic_t wakeup_num = ATOMIC_INIT(0);
static struct dubai_wakeup_info wakeup_stats[DUBAI_WAKEUP_NUM_MAX];
static const char *rtc_wakeup_source[] = {"rtc"};
static int sensorhub_wakeup_source = -1;
static int sensorhub_wakeup_reason = -1;

static void dubai_reset_wakeup_stats(void)
{
	wakeup_state = false;
	atomic_set(&wakeup_num, 0);
	sensorhub_wakeup_source = -1;
	sensorhub_wakeup_reason = -1;
	memset(wakeup_stats, 0, DUBAI_WAKEUP_NUM_MAX * sizeof(struct dubai_wakeup_info));
}

static bool is_rtc_wakeup(const char *irq)
{
	size_t i;

	for (i = 0; i < sizeof(rtc_wakeup_source) / sizeof(const char *); i++) {
		if (strstr(irq, rtc_wakeup_source[i]))
			return true;
	}
	return false;
}

static void set_wakeup_info(enum dubai_wakeup_type type, const char *tag, const char *msg)
{
	int curr_num = atomic_read(&wakeup_num);
	if ((curr_num < 0) || (curr_num >= DUBAI_WAKEUP_NUM_MAX)) {
		dubai_err("Invalid wakeup num: %d", curr_num);
		return;
	}

	strncpy(wakeup_stats[curr_num].tag, tag, DUBAI_WAKEUP_TAG_LENGTH - 1);
	strncpy(wakeup_stats[curr_num].msg, msg, DUBAI_WAKEUP_MSG_LENGTH - 1);
	wakeup_stats[curr_num].type = type;
	atomic_set(&wakeup_num, curr_num + 1);
}

/*
 * Caution: It's dangerous to use HWDUBAI_LOG in this function,
 * because it's in the SR process, and the HWDUBAI_LOG will wake up the kworker thread that will open irq
 */
void dubai_log_irq_wakeup(const char *source)
{
	char wakeup_source[WAKEUP_SOURCE_SIZE] = {0};
	char msg[DUBAI_WAKEUP_MSG_LENGTH] = {0};
	enum dubai_wakeup_type wakeup_type = DUBAI_WAKEUP_TYPE_INVALID;

	if (!source) {
		dubai_err("Invalid parameter");
		return;
	}

	if (is_rtc_wakeup(source)) {
		snprintf(wakeup_source, WAKEUP_SOURCE_SIZE - 1, "RTC0");
		wakeup_type = DUBAI_WAKEUP_TYPE_RTC;
	} else {
		snprintf(wakeup_source, WAKEUP_SOURCE_SIZE - 1, "%s", source);
		wakeup_type = DUBAI_WAKEUP_TYPE_OTHERS;
	}

	if (!wakeup_state) {
		dubai_wakeup_notify(wakeup_source);
		wakeup_state = true;
	}

	snprintf(msg, DUBAI_WAKEUP_MSG_LENGTH - 1, "name=%s", wakeup_source);
	set_wakeup_info(wakeup_type, "DUBAI_TAG_AP_WAKE_IRQ", msg);
}
EXPORT_SYMBOL(dubai_log_irq_wakeup); //lint !e580

/*
 * Caution: It's dangerous to use HWDUBAI_LOG in this function,
 * because it's in the SR process, and the HWDUBAI_LOG will wake up the kworker thread that will open irq
 */
void dubai_log_sensorhub_wakeup(int source, int reason)
{
	sensorhub_wakeup_source = source;
	sensorhub_wakeup_reason = reason;
}
EXPORT_SYMBOL(dubai_log_sensorhub_wakeup); //lint !e580

static int dubai_get_wakeup_info(struct dubai_wakeup_info *data, size_t num)
{
	int stats_num, offset, curr_num = atomic_read(&wakeup_num);

	if (!data || (num < curr_num) || (num > DUBAI_WAKEUP_NUM_MAX)) {
		dubai_err("Invalid parameter");
		return -1;
	}

	if ((curr_num > 0) && (curr_num <= DUBAI_WAKEUP_NUM_MAX)) {
		offset = (curr_num == 1) ? 0 : 1; // abandon parent wakeup(e.g PMIC) if child wakeup exist
		stats_num = curr_num - offset;
		memcpy(data, &wakeup_stats[offset], stats_num * sizeof(struct dubai_wakeup_info));
		if ((stats_num < num) && (sensorhub_wakeup_source >= 0)) {
			// add sensorhub wakeup
			snprintf(data[stats_num].tag, DUBAI_WAKEUP_TAG_LENGTH - 1, "DUBAI_TAG_SENSORHUB_WAKEUP");
			snprintf(data[stats_num].msg, DUBAI_WAKEUP_MSG_LENGTH - 1, "source=%d reason=%d",
				sensorhub_wakeup_source, sensorhub_wakeup_reason);
			data[stats_num].type = DUBAI_WAKEUP_TYPE_OTHERS;
		}
	}
	return 0;
}

static int dubai_suspend_notify(unsigned long mode)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE:
		dubai_reset_wakeup_stats();
		break;
	default:
		break;
	}
	return 0;
}

static struct dubai_wakeup_stats_ops wakeup_ops = {
	.suspend_notify = dubai_suspend_notify,
	.get_stats = dubai_get_wakeup_info,
};

int dubai_wakeup_stats_init(void)
{
	dubai_register_module_ops(DUBAI_MODULE_WAKEUP, &wakeup_ops);
	return 0;
}

void dubai_wakeup_stats_exit(void)
{
	dubai_unregister_module_ops(DUBAI_MODULE_WAKEUP);
	dubai_reset_wakeup_stats();
}
