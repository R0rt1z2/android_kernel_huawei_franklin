/*
 * hw_pmic.c
 *
 * debug for pmic sensor
 *
 * Copyright (c) 2019 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <huawei_platform/sensor/hw_comm_pmic.h>
#include <huawei_platform/sensor/hw_pmic.h>
#include <huawei_platform/log/hw_log.h>
#include <securec.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <hwmanufac/dev_detect/dev_detect.h>
#endif
#include "hwcam_hiview.h"

#define HWLOG_TAG huawei_pmic
HWLOG_REGIST();

#define GPIO_IRQ_REGISTER_SUCCESS 1
#define MAX_I2C_EXCEP_TIME 3
#define MAX_PMIC_ERR_INFO_LEN 192
#define MAX_PMIC_REG_INFO_LEN 128
#define PMIC_BOOST_ERROR_SHIELD 1
#define MAX_RETRY_TIMES 5
DEFINE_HW_PMIC_MUTEX(vote_cfg);

static struct pmic_ctrl_vote_t pmic_ctrl_vote[MAX_PMIC_NUM][MAX_SEQ_INDEX];
struct hw_pmic_ctrl_t *hw_pmic_ctrl = NULL;
static int pmic_err_notify_limit;
static RAW_NOTIFIER_HEAD(pmic_fault_notifier);
static DEFINE_SPINLOCK(pmic_chain_lock);

const char *g_pmic_error_info[PMIC_ERROR_TYPE_END] = {
	"(i2c write fail)",
	"(i2c read fail)",
	"(ocp error)",
	"(ovp error)",
	"(uvp error)",
};

int pmic_fault_notifier_register(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	return_error_on_null(nb);
	spin_lock_irqsave(&pmic_chain_lock, flags);
	ret = raw_notifier_chain_register(&pmic_fault_notifier, nb);
	spin_unlock_irqrestore(&pmic_chain_lock, flags);
	return ret;
}

int pmic_fault_notifier_unregister(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	return_error_on_null(nb);
	spin_lock_irqsave(&pmic_chain_lock, flags);
	ret = raw_notifier_chain_unregister(&pmic_fault_notifier, nb);
	spin_unlock_irqrestore(&pmic_chain_lock, flags);
	return ret;
}

void pmic_fault_notify(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	return_on_null(pmic_ctrl);
	hwlog_info("%s, event_type = %d", __func__, pmic_ctrl->pmic_info.event_type);
	raw_notifier_call_chain(&pmic_fault_notifier, pmic_ctrl->pmic_info.event_type, NULL);
	pmic_ctrl->pmic_info.event_type = PMIC_ERROR_NONE;
}

static void pmic_hiview_generate_error_info(pmic_error_type error_type,
	char *error_info, unsigned int error_info_len, const char *reg_info)
{
	errno_t ret;

	if (error_type >= PMIC_ERROR_TYPE_START &&
		error_type < PMIC_ERROR_TYPE_END) {
		ret = strcpy_s(error_info, error_info_len, g_pmic_error_info[error_type]);
		if (ret != EOK)
			hwlog_err("%s, pmic error type = %d, ret = %d\n", __func__, error_type, ret);
	}

	if (reg_info) {
		ret = strcat_s(error_info, error_info_len, reg_info);
		if (ret != EOK)
			hwlog_err("%s, error_info strcat_s fail %d\n", __func__, ret);
	}
}

static void pmic_hiview_handle(int error_no, pmic_error_type error_type,
	const char *reg_info)
{
	errno_t ret;
	struct hwcam_hievent_info pmic_info;
	char error_info[MAX_PMIC_ERR_INFO_LEN] = {0};
	struct hw_pmic_ctrl_t *pmic_ctrl = hw_get_pmic_ctrl();

	ret = memset_s(&pmic_info, sizeof(pmic_info), 0, sizeof(pmic_info));
	if (ret != EOK) {
		hwlog_err("pmic_info memset_s fail, ret = %d\n", ret);
		return;
	}

	pmic_hiview_generate_error_info(error_type, error_info,
		sizeof(error_info), reg_info);
	pmic_info.error_no = error_no;
	if (pmic_ctrl)
		hwcam_hiview_get_ic_name(pmic_ctrl->pmic_info.name, &pmic_info);
	hwcam_hiview_get_content(error_no, error_info, &pmic_info);
	hwcam_hiview_report(&pmic_info);
}

void pmic_fault_check(struct hw_pmic_ctrl_t *pmic_ctrl, u8 stat_reg,
	struct pmic_error_info_t *error_map, unsigned int map_size)
{
	struct hw_pmic_i2c_client *i2c_client = NULL;
	struct hw_pmic_i2c_fn_t *i2c_func = NULL;
	struct hw_pmic_info *pmic_info = NULL;
	char reg_info[MAX_PMIC_REG_INFO_LEN] = {0};
	u8 pmu_status = 0;
	u8 reg_value = 0;
	unsigned int i;
	errno_t ret;

	if (!pmic_ctrl || !error_map || !pmic_ctrl->pmic_i2c_client ||
		!pmic_ctrl->pmic_i2c_client->i2c_func_tbl ||
		!pmic_ctrl->pmic_i2c_client->i2c_func_tbl->i2c_read) {
		hwlog_err("%s invalid params\n", __func__);
		return;
	}

	pmic_info = &pmic_ctrl->pmic_info;
	i2c_client = pmic_ctrl->pmic_i2c_client;
	i2c_func = pmic_ctrl->pmic_i2c_client->i2c_func_tbl;

	/* PMU_STAT */
	i2c_func->i2c_read(i2c_client, stat_reg, &pmu_status);

	for (i = 0; i < map_size; ++i) {
		i2c_func->i2c_read(i2c_client, error_map[i].reg_addr, &reg_value);
		hwlog_info("fault reg = 0x%x, value = 0x%x, pmic_ic_stat = 0x%x, value = 0x%x\n",
			error_map[i].reg_addr, reg_value, stat_reg, pmu_status);
		if (reg_value == error_map[i].reg_normal_value)
			continue;

		/* only exist boost error, not report dmd */
		if ((reg_value & error_map[i].boost_mask) > 0) {
			pmic_info->event_type = PMIC_BOOST_ERROR_EVENT;
			hwlog_info("pmic exist boost error\n");
			if ((pmic_info->boost_shield_dmd == PMIC_BOOST_ERROR_SHIELD) &&
				((reg_value & (~error_map[i].boost_mask)) == 0)) {
				hwlog_info("only exist boost error, not report dsm\n");
				continue;
			}
		}

		ret = memset_s(&reg_info, sizeof(reg_info), 0, sizeof(reg_info));
		if (ret != EOK) {
			hwlog_err("reg_info memset_s fail, ret = %d\n", ret);
			continue;
		}
		if (snprintf_s(reg_info, sizeof(reg_info), sizeof(reg_info) - 1,
			"[error_flag: reg(0x%x),value(0x%x), pmic_ic_status: reg(0x%x),value(0x%x)]",
			error_map[i].reg_addr, reg_value, stat_reg, pmu_status) < 0)
			hwlog_err("%s, reg_info snprintf_s fail\n", __func__);
		pmic_hiview_handle(error_map[i].error_no, error_map[i].type, reg_info);
	}
}

static int hw_pmic_i2c_read(struct hw_pmic_i2c_client *client,
	u8 reg, u8 *data)
{
	int rc;
	struct i2c_msg msgs[2];
	char reg_info[MAX_PMIC_REG_INFO_LEN] = {0};
	int retry_times = MAX_RETRY_TIMES;

	if (!client || !data || !client->client || !client->client->adapter) {
		hwlog_err("%s i2c err\n", __func__);
		return -EFAULT;
	}

	msgs[0].addr = client->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = client->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = data;

	rc = i2c_transfer(client->client->adapter, msgs, 2); /* msg len */
	while ((rc < 0) && (--retry_times)) {
		mdelay(2); /* delay 2ms */
		rc = i2c_transfer(client->client->adapter, msgs, 2); /* msg len */
	}
	if (rc >= 0) {
		hwlog_debug("%s reg=0x%x, data=0x%x\n", __func__, reg, *data);
		return rc;
	}
	hwlog_err("%s, transfer error, reg = 0x%x, data = 0x%x, addr = 0x%x\n",
		__func__, reg, *data, msgs[0].addr);

	if (pmic_err_notify_limit < MAX_I2C_EXCEP_TIME) {
		if (snprintf_s(reg_info, sizeof(reg_info), sizeof(reg_info) - 1,
			"[error_flag: reg(0x%x), value(0x%x)]", reg, *data) < 0)
			hwlog_err("%s reg_info snprintf_s fail\n", __func__);
		pmic_hiview_handle(DSM_CAMPMIC_I2C_ERROR_NO, PMIC_READ_I2C_FAIL, reg_info);
		pmic_err_notify_limit++;
	}

	return rc;
}

static int hw_pmic_i2c_write(struct hw_pmic_i2c_client *client,
	u8 reg, u8 data)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg;
	char reg_info[MAX_PMIC_REG_INFO_LEN] = {0};
	int retry_times = MAX_RETRY_TIMES;

	if (!client || !client->client || !client->client->adapter) {
		hwlog_err("%s i2c err\n", __func__);
		return -EFAULT;
	}
	buf[0] = reg;
	buf[1] = data;
	msg.addr = client->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	rc = i2c_transfer(client->client->adapter, &msg, 1); /* msg len */
	while ((rc < 0) && (--retry_times)) {
		mdelay(2); /* delay 2ms */
		rc = i2c_transfer(client->client->adapter, &msg, 1); /* msg len */
	}
	if (rc >= 0) {
		hwlog_info("%s reg=0x%x, data=0x%x\n", __func__, reg, data);
		return rc;
	}
	hwlog_err("%s, transfer error, reg = 0x%x, data = 0x%x, addr = 0x%x\n",
		__func__, reg, data, msg.addr);

	if (pmic_err_notify_limit < MAX_I2C_EXCEP_TIME) {
		if (snprintf_s(reg_info, sizeof(reg_info), sizeof(reg_info) - 1,
			"[error_flag: reg(0x%x), value(0x%x)]", reg, data) < 0)
			hwlog_err("%s reg_info snprintf_s fail\n", __func__);
		pmic_hiview_handle(DSM_CAMPMIC_I2C_ERROR_NO, PMIC_WRITE_I2C_FAIL, reg_info);
		pmic_err_notify_limit++;
	}

	return rc;
}

struct hw_pmic_i2c_fn_t hw_pmic_i2c_func_tbl = {
	.i2c_read = hw_pmic_i2c_read,
	.i2c_write = hw_pmic_i2c_write,
};

void hw_set_pmic_ctrl(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	hwlog_info("%s enter\n", __func__);
	hw_pmic_ctrl = pmic_ctrl;
}

struct hw_pmic_ctrl_t *hw_get_pmic_ctrl(void)
{
	hwlog_info("%s enter\n", __func__);
	return hw_pmic_ctrl;
}

int pmic_ctl_otg_onoff(bool on_off)
{
	int pmic_ctl_gpio_otg_switch;
	pmic_ctl_gpio_otg_switch = of_get_named_gpio(of_find_compatible_node(
		NULL, NULL, "huawei,vbus_channel_boost_gpio"),
		"gpio_otg_switch", 0);
	hwlog_info("pmic_ctl_otg_onoff,ret = %d\n",pmic_ctl_gpio_otg_switch);
	if (pmic_ctl_gpio_otg_switch > 0)
	{
		hwlog_info("otg_gpio is %d\n",pmic_ctl_gpio_otg_switch);
		gpio_set_value(pmic_ctl_gpio_otg_switch,on_off);
	}
	return pmic_ctl_gpio_otg_switch;
}
int pmic_enable_boost(int value)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = hw_get_pmic_ctrl();
	int rc = -EPERM;
	u32 voltage = 0;

	if((!pmic_ctrl) || (!pmic_ctrl->func_tbl) ||
		(!pmic_ctrl->func_tbl->pmic_seq_config)) {
		hwlog_err("pmic_ctrl is NULL,just return");
		return -EPERM;
	}

	if (value == 0) {
		rc = pmic_ctrl->func_tbl->pmic_seq_config(pmic_ctrl,
			(pmic_seq_index_t)VOUT_BOOST, (u32)voltage, 0);
	} else {
		rc = pmic_ctrl->func_tbl->pmic_seq_config(pmic_ctrl,
			(pmic_seq_index_t)VOUT_BOOST, (u32)voltage, 1);
	}

	hwlog_info("%s rc=%d\n",__func__, rc);
	return rc;
}
EXPORT_SYMBOL(pmic_enable_boost);

static void vote_volt_calc(enum pmic_power_req_src_t pmic_power_src,
	struct hw_comm_pmic_cfg_t *pmic_cfg)
{
	pmic_seq_index_t seq;
	u8 num;
	u32 seq_volt = 0;
	u32 i;

	seq = pmic_cfg->pmic_power_type;
	num = pmic_cfg->pmic_num;

	for (i = 0; i < MAX_PMIC_REQ; i++) {
		hwlog_debug("volt = %u\n", pmic_ctrl_vote[num][seq].pmic_volt[i]);
		if (seq_volt >= pmic_ctrl_vote[num][seq].pmic_volt[i])
			continue;
		seq_volt = pmic_ctrl_vote[num][seq].pmic_volt[i];
		hwlog_debug("%s higher vol = %u\n", __func__, seq_volt);
	}
	pmic_cfg->pmic_power_voltage = seq_volt;
	hwlog_info("calc vote volt = %u\n", pmic_cfg->pmic_power_voltage);
}

/* define a function to claim different vote algo */
static void pmic_volt_vote_set(enum pmic_power_req_src_t src,
	struct hw_comm_pmic_cfg_t *pmic_cfg)
{
	pmic_seq_index_t seq = pmic_cfg->pmic_power_type;
	u8 num = pmic_cfg->pmic_num;

	if (pmic_ctrl_vote[num][seq].power_state == 0)
		return;

	vote_volt_calc(src, pmic_cfg);
}

static void pmic_power_vote_set(enum pmic_power_req_src_t src,
	struct hw_comm_pmic_cfg_t *pmic_cfg)
{
	pmic_seq_index_t seq = pmic_cfg->pmic_power_type;
	u8 num = pmic_cfg->pmic_num;

	if (pmic_cfg->pmic_power_state) {
		pmic_ctrl_vote[num][seq].power_state |= (u8)PMIC_VOTE_ON << (u8)src;
		pmic_ctrl_vote[num][seq].power_cnt[src]++;
		pmic_ctrl_vote[num][seq].pmic_volt[src] =
			pmic_cfg->pmic_power_voltage;
	} else {
		pmic_ctrl_vote[num][seq].power_cnt[src]--;
	}

	if (pmic_ctrl_vote[num][seq].power_cnt[src] < 0) {
		pmic_ctrl_vote[num][seq].power_cnt[src] = 0;
		hwlog_err("%s, err power vote cnt, req_module-%d\n",
			__func__, src);
	}

	if (pmic_ctrl_vote[num][seq].power_cnt[src] == 0) {
		pmic_ctrl_vote[num][seq].power_state &=
			~((u8)PMIC_VOTE_ON << (u8)src);
		pmic_ctrl_vote[num][seq].pmic_volt[src] = 0;
	}

	if (pmic_ctrl_vote[num][seq].power_state > 0) {
		pmic_cfg->pmic_power_state = PMIC_VOTE_ON;
		return;
	}

	pmic_cfg->pmic_power_state = PMIC_VOTE_OFF;
	pmic_ctrl_vote[num][seq].pmic_volt[src] = 0;
}

static void pmic_power_vote_reset(enum pmic_power_req_src_t src,
	struct hw_comm_pmic_cfg_t *pmic_cfg)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = NULL;
	u8 num = pmic_cfg->pmic_num;
	int seq;

	hwlog_info("%s, enter\n", __func__);
	if ((src < TP_PMIC_REQ) || (src >= MAX_PMIC_REQ) ||
		(num < MAIN_PMIC) || (num >= MAX_PMIC_NUM)) {
		hwlog_err("%s, err para, req_module-%d, pmic_type-%d\n",
			__func__, src, num);
		return;
	}
	pmic_ctrl = hw_get_pmic_ctrl();
	return_on_null(pmic_ctrl);

	for (seq = VOUT_LDO_1; seq < MAX_SEQ_INDEX; ++seq) {
		if (pmic_ctrl_vote[num][seq].power_cnt[src] == 0)
			continue;
		pmic_ctrl_vote[num][seq].power_cnt[src] = 0;
		pmic_ctrl_vote[num][seq].power_state &= ~((u8)PMIC_VOTE_ON << (u8)src);
		pmic_ctrl_vote[num][seq].pmic_volt[src] = 0;
		if (pmic_ctrl_vote[num][seq].power_state == 0)
			pmic_ctrl->func_tbl->pmic_seq_config(pmic_ctrl,
				seq, 0, PMIC_POWER_OFF);
		hwlog_info("%s, pmic_typ-%u, req_module-%d, power_type-%d, excute vote reset\n",
			__func__, num, src, seq);
	}
}

void hw_pmic_reset(enum pmic_power_req_src_t src,
	struct hw_comm_pmic_cfg_t *pmic_cfg)
{
	return_on_null(pmic_cfg);
	pmic_power_vote_reset(src, pmic_cfg);
}

int hw_pmic_power_cfg(enum pmic_power_req_src_t pmic_power_src,
	struct hw_comm_pmic_cfg_t *comm_pmic_cfg)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = NULL;
	int ret;

	if (!comm_pmic_cfg) {
		hwlog_err("%s, pmic vol cfg para null\n", __func__);
		return -EFAULT;
	}
	if ((pmic_power_src < TP_PMIC_REQ) || (pmic_power_src >= MAX_PMIC_REQ) ||
		(comm_pmic_cfg->pmic_power_type < VOUT_LDO_1) ||
		(comm_pmic_cfg->pmic_power_type >= MAX_SEQ_INDEX) ||
		(comm_pmic_cfg->pmic_num < MAIN_PMIC) ||
		(comm_pmic_cfg->pmic_num >= MAX_PMIC_NUM)) {
		hwlog_err("%s, err para, req_module-%d, power_type-%d\n", __func__,
			pmic_power_src, comm_pmic_cfg->pmic_power_type);
		return -EFAULT;
	}
	mutex_lock(&pmic_mut_vote_cfg);
	hwlog_info("%s, need pmic_typ-%u, req_module-%d, power_type-%d, volt-%u, on-%d, state-%u\n",
		__func__, comm_pmic_cfg->pmic_num, pmic_power_src,
		comm_pmic_cfg->pmic_power_type,
		comm_pmic_cfg->pmic_power_voltage,
		comm_pmic_cfg->pmic_power_state,
		pmic_ctrl_vote[comm_pmic_cfg->pmic_num][comm_pmic_cfg->pmic_power_type].power_state);

	pmic_power_vote_set(pmic_power_src, comm_pmic_cfg);
	pmic_volt_vote_set(pmic_power_src, comm_pmic_cfg);
	hwlog_info("%s, set pmic_typ-%u, req_module-%d, power_type-%d, vol-%u, on-%d, state-%u\n",
		__func__, comm_pmic_cfg->pmic_num, pmic_power_src,
		comm_pmic_cfg->pmic_power_type,
		comm_pmic_cfg->pmic_power_voltage,
		comm_pmic_cfg->pmic_power_state,
		pmic_ctrl_vote[comm_pmic_cfg->pmic_num][comm_pmic_cfg->pmic_power_type].power_state);

	pmic_ctrl = hw_get_pmic_ctrl();
	if (!pmic_ctrl || !pmic_ctrl->func_tbl ||
		!pmic_ctrl->func_tbl->pmic_seq_config) {
		hwlog_err("%s, pmic_ctrl is null,just return\n", __func__);
		mutex_unlock(&pmic_mut_vote_cfg);
		return -EFAULT;
	}

	ret = pmic_ctrl->func_tbl->pmic_seq_config(pmic_ctrl,
		comm_pmic_cfg->pmic_power_type,
		comm_pmic_cfg->pmic_power_voltage,
		comm_pmic_cfg->pmic_power_state);
	mutex_unlock(&pmic_mut_vote_cfg);
	return ret;
}

static irqreturn_t hw_pmic_interrupt_handler(int vec, void *info)
{
	struct hw_pmic_ctrl_t *pmic_ctrl = NULL;

	if (!info) {
		hwlog_err("info is NULL");
		return IRQ_NONE;
	}

	pmic_ctrl = (struct hw_pmic_ctrl_t *)info;
	if (!pmic_ctrl || !pmic_ctrl->func_tbl ||
		!pmic_ctrl->func_tbl->pmic_check_exception)
		return IRQ_NONE;

	pmic_ctrl->func_tbl->pmic_check_exception(pmic_ctrl);

	return IRQ_HANDLED;
}

void irq_err_time_reset(struct irq_err_monitor *irq_err)
{
	if (!irq_err)
		return;
	irq_err->last_time.tv_sec = irq_err->now.tv_sec;
	irq_err->last_time.tv_usec = irq_err->now.tv_usec;
}

void pmic_wake_lock(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	return_on_null(pmic_ctrl);

	if (pmic_ctrl->pmic_wake_lock.active)
		return;
	__pm_stay_awake(&pmic_ctrl->pmic_wake_lock);
	hwlog_info("%s, pmic wakelock lock\n", __func__);
}

void pmic_wake_unlock(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	return_on_null(pmic_ctrl);

	if (!pmic_ctrl->pmic_wake_lock.active)
		return;
	__pm_relax(&pmic_ctrl->pmic_wake_lock);
	hwlog_info("%s, pmic wakelock unlock\n", __func__);
}

void pmic_fault_reset_check(struct hw_pmic_ctrl_t *pmic_ctrl,
	struct irq_err_monitor *irq_err, unsigned int latch_time,
	const unsigned int sche_work_time)
{
	if (!irq_err || !pmic_ctrl) {
		hwlog_err("irq_err or pimc_ctrl is NULL");
		return;
	}
	if (pmic_ctrl->pmic_info.fault_check == 0) {
		hwlog_err("not support fault err check");
		return;
	}
	do_gettimeofday(&irq_err->now);
	irq_err->irq_time = (irq_err->now.tv_sec - irq_err->last_time.tv_sec) *
		(1000000L) + (irq_err->now.tv_usec - irq_err->last_time.tv_usec);
	hwlog_err("%s err time = %ld", __func__, irq_err->irq_time);
	if (irq_err->irq_time > latch_time) {
		hwlog_err("pmic fault err check after %u", latch_time);
		irq_err_time_reset(irq_err);
		pmic_wake_lock(pmic_ctrl);
		schedule_delayed_work(&pmic_ctrl->pmic_err_work,
			msecs_to_jiffies(sche_work_time));
	}
}

int hw_pmic_gpio_boost_enable(struct hw_pmic_ctrl_t *pmic_ctrl, int state)
{
	int ret = -EPERM;
	struct hw_pmic_info *pmic_info = NULL;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl) {
		hwlog_err("pmic_ctrl is NULL, just return");
		return ret;
	}
	pmic_info = &pmic_ctrl->pmic_info;
	if (pmic_info->boost_en_pin == 0) {
		hwlog_err("boost enable pin not controled here\n");
		return ret;
	}

	if (state)
		gpio_set_value(pmic_info->boost_en_pin, 1);
	else
		gpio_set_value(pmic_info->boost_en_pin, 0);
	return 0;
}

int hw_pmic_setup_intr(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct device_node *dev_node = NULL;
	struct hw_pmic_info *pmic_info = NULL;
	int rc = -EPERM;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->dev) {
		hwlog_err("%s pmic_ctrl is NULL\n", __func__);
		return rc;
	}

	dev_node = pmic_ctrl->dev->of_node;
	pmic_info = &pmic_ctrl->pmic_info;

	/* get intrrupt pin */
	pmic_info->intr = of_get_named_gpio(dev_node, "pmic-intr", 0);
	if (pmic_info->intr <= 0)
		hwlog_err("ERROR %s huawei,pmic-intr %u", __func__, pmic_info->intr);
	if (!gpio_is_valid(pmic_info->intr))
		hwlog_err("not valid %s huawei,pmic-intr %u", __func__, pmic_info->intr);
	hwlog_info("GET %s huawei,pmic-intr %u", __func__, pmic_info->intr);

	pmic_info->pinctrl = devm_pinctrl_get(pmic_ctrl->dev);
	if (IS_ERR(pmic_info->pinctrl)) {
		hwlog_err("%s, cannot find pmic pinctrl", __func__);
		return -1;
	}

	pmic_info->pmic_intr = pinctrl_lookup_state(pmic_info->pinctrl, "pmic_intr");
	if (IS_ERR(pmic_info->pmic_intr)) {
		hwlog_err("%s, no pmic_intr pinstate", __func__);
		rc = -1;
		goto pinctrl_lookup_fail;
	}

	/* setup intrrupt request */
	rc = gpio_request(pmic_info->intr, "pmic-intr");
	if (rc < 0) {
		hwlog_err("%s failed request exp pin rc = %d\n", __func__, rc);
		goto irq_req_fail;
	}

	rc = pinctrl_select_state(pmic_info->pinctrl, pmic_info->pmic_intr);
	if (rc < 0) {
		hwlog_err("fail to configure intr as input %d", rc);
		goto direction_failed;
	}

	rc = gpio_to_irq(pmic_info->intr);
	if (rc < 0) {
		hwlog_err("fail to irq rc:%d", rc);
		goto direction_failed;
	}
	pmic_info->irq = rc;

	rc = request_threaded_irq(pmic_info->irq, NULL,
			hw_pmic_interrupt_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"hw_pmic_interrupt",
			(void *)pmic_ctrl);
	if (rc < 0) {
		hwlog_err("allocate int fail result:%d\n", rc);
		goto irq_failed;
	}
	pmic_info->irq_flag = GPIO_IRQ_REGISTER_SUCCESS;

	return 0;

irq_failed:
	free_irq(pmic_info->irq, (void *)pmic_ctrl);
direction_failed:
	gpio_free(pmic_info->intr);
irq_req_fail:
pinctrl_lookup_fail:
	devm_pinctrl_put(pmic_info->pinctrl);
	return rc;
}
EXPORT_SYMBOL(hw_pmic_setup_intr);

void hw_pmic_release_intr(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct hw_pmic_info *pmic_info = NULL;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl) {
		hwlog_err("%s pmic_ctrl is NULL\n", __func__);
		return;
	}
	pmic_info = &pmic_ctrl->pmic_info;
	if (pmic_info->irq_flag != GPIO_IRQ_REGISTER_SUCCESS) {
		hwlog_err("%s pmic_info->irq failed\n", __func__);
		return;
	}
	hwlog_info("%s free_irq:%d\n", __func__, pmic_info->irq);
	free_irq(pmic_info->irq, (void *)pmic_ctrl);
	hwlog_info("%s gpio_free:%d\n", __func__, pmic_info->intr);
	gpio_free(pmic_info->intr);
}
EXPORT_SYMBOL(hw_pmic_release_intr);

static int hw_pmic_get_dt_data(struct hw_pmic_ctrl_t *pmic_ctrl)
{
	struct device_node *dev_node = NULL;
	struct hw_pmic_info *pmic_info = NULL;
	static bool gpio_req_status = false;
	int rc = -EPERM;

	hwlog_info("%s enter\n", __func__);
	if (!pmic_ctrl || !pmic_ctrl->dev) {
		hwlog_err("%s pmic_ctrl is NULL\n", __func__);
		return rc;
	}

	dev_node = pmic_ctrl->dev->of_node;
	pmic_info = &pmic_ctrl->pmic_info;

	rc = of_property_read_u32(dev_node, "mtk,slave_address",
		&pmic_info->slave_address);
	hwlog_info("%s slave_address %d\n", __func__, pmic_info->slave_address);
	if (rc < 0) {
		hwlog_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	rc = of_property_read_u32(dev_node, "mtk,boost-pin",
		&pmic_info->boost_en_pin);
	if (rc < 0) {
		hwlog_err("%s, get boost enable pin failed\n", __func__);
		pmic_info->boost_en_pin = 0;
		rc = 0;
	} else {
		hwlog_info("enable boost-pin = %d\n", pmic_info->boost_en_pin);
		if (!gpio_req_status) {
			rc = gpio_request(pmic_info->boost_en_pin,
				"hw_pmic_boost_en");
			if (rc < 0) {
				hwlog_err("fail req boost en = %d\n", rc);
				return -EPERM;
			}
			(void)gpio_direction_output(pmic_info->boost_en_pin, 1);
			gpio_req_status = true;
		}
	}

	rc = of_property_read_u32(dev_node, "mtk,fault_check",
		&pmic_info->fault_check);
	if (rc < 0) {
		hwlog_info("%s failed get fault_check, set default\n", __func__);
		pmic_info->fault_check = 0;
		rc = 0;
	}
	hwlog_info("%s fault_check = %u\n", __func__, pmic_info->fault_check);

	rc = of_property_read_string(dev_node, "mtk,pmic_name",
		&pmic_info->name);
	if (rc < 0) {
		hwlog_err("%s failed get pmic name\n", __func__);
		rc = 0;
	}
	hwlog_info("%s pmic_name = %s\n", __func__, pmic_info->name);

	if (of_property_read_u32(dev_node, "boost_shield_dmd",
		&pmic_info->boost_shield_dmd) < 0) {
		hwlog_info("%s failed get boost_shield_dmd, set default\n", __func__);
		pmic_info->boost_shield_dmd = 0;
	}
	hwlog_info("%s boost_shield_dmd = %u\n", __func__, pmic_info->boost_shield_dmd);

	if (of_property_read_u32(dev_node, "boost_shield_irq",
		&pmic_info->boost_shield_irq) < 0) {
		hwlog_info("%s failed get boost_shield_irq, set default\n", __func__);
		pmic_info->boost_shield_irq = 0;
	}
	hwlog_info("%s boost_shield_irq = %u\n", __func__, pmic_info->boost_shield_irq);

	return rc;
}

int hw_pmic_config(struct hw_pmic_ctrl_t *pmic_ctrl, void *argp)
{
	return 0;
}
EXPORT_SYMBOL(hw_pmic_config);

int hw_pmic_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = NULL;
	struct hw_pmic_ctrl_t *pmic_ctrl = NULL;
	static bool hw_pmic_enabled;
	int rc;

	pmic_err_notify_limit = 0;
	if (hw_pmic_enabled) {
		hwlog_info("pmic has already registerd\n");
		return -EFAULT;
	}
	hwlog_info("%s client name = %s\n", __func__, client->name);

	adapter = client->adapter;
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		hwlog_err("%s i2c_check_functionality failed\n", __func__);
		return -EIO;
	}
	if (!id) {
		hwlog_err("%s id null\n", __func__);
		return -EFAULT;
	}
	pmic_ctrl = (struct hw_pmic_ctrl_t *)(uintptr_t)id->driver_data;
	if (!pmic_ctrl || !pmic_ctrl->pmic_i2c_client || !pmic_ctrl->func_tbl ||
		!pmic_ctrl->func_tbl->pmic_get_dt_data ||
		!pmic_ctrl->func_tbl->pmic_init) {
		hwlog_err("%s pmic_ctrl null\n", __func__);
		return -EFAULT;
	}
	pmic_ctrl->pmic_i2c_client->client = client;
	pmic_ctrl->dev = &client->dev;
	pmic_ctrl->pmic_i2c_client->i2c_func_tbl = &hw_pmic_i2c_func_tbl;

	rc = hw_pmic_get_dt_data(pmic_ctrl);
	if (rc < 0) {
		hwlog_err("%s hw_pmic_get_dt_data failed\n", __func__);
		return -EFAULT;
	}
	rc = pmic_ctrl->func_tbl->pmic_get_dt_data(pmic_ctrl);
	if (rc < 0) {
		hwlog_err("%s flash_get_dt_data failed\n", __func__);
		return -EFAULT;
	}

	pmic_ctrl->pmic_info.event_type = PMIC_ERROR_NONE;
	rc = pmic_ctrl->func_tbl->pmic_init(pmic_ctrl);
	if (rc < 0) {
		hwlog_err("%s pmic init failed\n", __func__);
		return -EFAULT;
	}

	hw_set_pmic_ctrl(pmic_ctrl);
	hwlog_info("%s hw pmic register success\n", __func__);
	hw_pmic_enabled = true;
	return rc;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("debug for pmic driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

