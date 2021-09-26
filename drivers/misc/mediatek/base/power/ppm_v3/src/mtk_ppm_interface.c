/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include "mtk_ppm_internal.h"


#define PPM_MODE_NAME_LEN	16

/* procfs dir for policies */
struct proc_dir_entry *policy_dir;
struct proc_dir_entry *profile_dir;
struct proc_dir_entry *cpi_dir;
unsigned int ppm_debug;
unsigned int ppm_func_lv_mask;


char *ppm_copy_from_user_for_proc(const char __user *buffer, size_t count)
{
	char *buf = (char *)__get_free_page(GFP_USER);

	if (!buf)
		return NULL;

	if (count >= PAGE_SIZE)
		goto out;

	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';

	return buf;

out:
	free_page((unsigned long)buf);

	return NULL;
}

static int ppm_func_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "ppm func lv debug mask = 0x%x\n", ppm_func_lv_mask);

	return 0;
}

static ssize_t ppm_func_debug_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int func_dbg_lv = 0;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &func_dbg_lv))
		ppm_func_lv_mask = func_dbg_lv;
	else
		ppm_err("echo func_dbg_lv (dec) > /proc/ppm/func_debug\n");

	free_page((unsigned long)buf);
	return count;
}

static int ppm_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "ppm debug (log level) = %d\n", ppm_debug);

	return 0;
}

static ssize_t ppm_debug_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int dbg_lv = 0;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &dbg_lv))
		ppm_debug = dbg_lv;
	else
		ppm_err("echo dbg_lv (dec) > /proc/ppm/debug\n");

	free_page((unsigned long)buf);
	return count;
}

static int ppm_enabled_proc_show(struct seq_file *m, void *v)
{
	if (ppm_main_info.is_enabled == true)
		seq_puts(m, "ppm is enabled\n");
	else
		seq_puts(m, "ppm is disabled\n");

	return 0;
}

static ssize_t ppm_enabled_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int enabled = 0;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	ppm_lock(&ppm_main_info.lock);

	if (!kstrtouint(buf, 10, &enabled)) {
		ppm_main_info.is_enabled = (enabled) ? true : false;
		if (!ppm_main_info.is_enabled) {
			int i;
			struct ppm_client_req *c_req =
				&(ppm_main_info.client_req);
			struct ppm_client_req *last_req =
				&(ppm_main_info.last_req);

			/* send default limit to client */
			ppm_main_clear_client_req(c_req);
#ifdef PPM_SSPM_SUPPORT
			/* update limit to SSPM first */
			ppm_ipi_update_limit(*c_req);
#endif
			for_each_ppm_clients(i) {
				if (!ppm_main_info.client_info[i].limit_cb)
					continue;

				ppm_main_info.client_info[i].limit_cb(*c_req);
			}
			memcpy(last_req->cpu_limit, c_req->cpu_limit,
				ppm_main_info.cluster_num *
				sizeof(*c_req->cpu_limit));

			ppm_info("PPM disabled, send no limit to clinet!\n");
		}
	} else
		ppm_err("echo [0/1] > /proc/ppm/enabled\n");

	ppm_unlock(&ppm_main_info.lock);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_exclusive_core_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "ppm exclusive core = %*pbl\n",
		cpumask_pr_args(ppm_main_info.exclusive_core));

	return 0;
}

static ssize_t ppm_exclusive_core_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int mask = 0;
	int cpu;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &mask)) {
		ppm_lock(&ppm_main_info.lock);
		cpumask_clear(ppm_main_info.exclusive_core);
		for_each_present_cpu(cpu) {
			if (mask & 0x1)
				cpumask_set_cpu(cpu,
					ppm_main_info.exclusive_core);
			mask >>= 1;
		}
		ppm_unlock(&ppm_main_info.lock);
		ppm_info("update exclusive core = %*pbl\n",
			cpumask_pr_args(ppm_main_info.exclusive_core));
		mt_ppm_main();
	} else
		ppm_err("echo <bitmask> > /proc/ppm/exclusive_core\n");

	free_page((unsigned long)buf);
	return count;
}

static int ppm_dump_power_table_proc_show(struct seq_file *m, void *v)
{
	ppm_cobra_dump_tbl(m);

	return 0;
}

static int ppm_dump_policy_list_proc_show(struct seq_file *m, void *v)
{
	struct ppm_policy_data *pos;
	unsigned int i = 0, j = 0;

	ppm_lock(&ppm_main_info.lock);
	seq_puts(m, "\nFinal limit:\n");
	for_each_ppm_clusters(j) {
		seq_printf(m, "cluster %d: (%d)(%d)(%d)(%d)\n", j,
			ppm_main_info.last_req.cpu_limit[j].min_cpufreq_idx,
			ppm_main_info.last_req.cpu_limit[j].max_cpufreq_idx,
			ppm_main_info.last_req.cpu_limit[j].min_cpu_core,
			ppm_main_info.last_req.cpu_limit[j].max_cpu_core);
	}

	seq_puts(m, "\nPolicy limit:\n");
	list_for_each_entry(pos, &ppm_main_info.policy_list, link) {
		ppm_lock(&pos->lock);

		seq_printf(m, "[%d] %s (priority: %d)\n",
			i, pos->name, pos->priority);
		seq_printf(m, "is_enabled = %d, is_activated = %d\n",
			pos->is_enabled, pos->is_activated);
		seq_printf(m, "req_perf_idx = %d, req_power_budget = %d\n",
			pos->req.perf_idx, pos->req.power_budget);
		for_each_ppm_clusters(j) {
			seq_printf(m, "cluster %d: (%d)(%d)(%d)(%d)\n", j,
				pos->req.limit[j].min_cpufreq_idx,
				pos->req.limit[j].max_cpufreq_idx,
				pos->req.limit[j].min_cpu_core,
				pos->req.limit[j].max_cpu_core);
		}
		seq_puts(m, "\n");
		ppm_unlock(&pos->lock);

		i++;
	}
	ppm_unlock(&ppm_main_info.lock);

	return 0;
}

static int ppm_policy_status_proc_show(struct seq_file *m, void *v)
{
	struct ppm_policy_data *pos;

	list_for_each_entry_reverse(pos, &ppm_main_info.policy_list, link)
		seq_printf(m, "[%d] %s: %s\n", pos->policy, pos->name,
				(pos->is_enabled) ? "enabled" : "disabled");

	seq_puts(m, "\nUsage: echo <idx> <1/0> > /proc/ppm/policy_status\n\n");

	return 0;
}

static ssize_t ppm_policy_status_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	struct ppm_policy_data *l_pos;
	unsigned int policy_idx, enabled;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &policy_idx, &enabled) == 2) {
		if (enabled > 1)
			enabled = 1;

		/* set mode status and notify policy via status change cb */
		list_for_each_entry(l_pos,
			&ppm_main_info.policy_list, link) {
			if (l_pos->policy == policy_idx
				&& l_pos->is_enabled != enabled) {
				ppm_lock(&l_pos->lock);
				l_pos->is_enabled =
					(enabled) ? true : false;
				if (!l_pos->is_enabled)
					l_pos->is_activated = false;
				if (l_pos->status_change_cb)
					l_pos->status_change_cb(
						l_pos->is_enabled);
				ppm_unlock(&l_pos->lock);

				mt_ppm_main();
				break;
			}
		}
	} else
		ppm_err("Usage: echo <idx> <1/0> > /proc/ppm/policy_status\n");

	free_page((unsigned long)buf);
	return count;
}

static int ppm_dump_dvfs_table_proc_show(struct seq_file *m, void *v)
{
	struct ppm_cluster_info *info = (struct ppm_cluster_info *)m->private;
	unsigned int i;

	if (!info->dvfs_tbl) {
		ppm_err("DVFS table for cluster %d is NULL!\n",
			info->cluster_id);
		goto end;
	}

	for (i = 0; i < info->dvfs_opp_num; i++)
		seq_printf(m, "%d ", info->dvfs_tbl[i].frequency);

	seq_puts(m, "\n");

end:
	return 0;
}

static int ppm_cobra_budget_to_limit_proc_show(struct seq_file *m, void *v)
{
	ppm_cobra_lookup_get_result(m, LOOKUP_BY_BUDGET);

	return 0;
}

static ssize_t ppm_cobra_budget_to_limit_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int budget;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtoint(buf, 10, &budget))
		cobra_lookup_data.budget = budget;
	else
		ppm_err("echo <budget> > /proc/ppm/cobra_budget_to_limit\n");

	free_page((unsigned long)buf);
	return count;
}

static int ppm_cobra_limit_to_budget_proc_show(struct seq_file *m, void *v)
{
	ppm_cobra_lookup_get_result(m, LOOKUP_BY_LIMIT);

	return 0;
}

static ssize_t ppm_cobra_limit_to_budget_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	char *tok, *tmp;
	unsigned int i = 0, data;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == NR_PPM_CLUSTERS * 2) {
			ppm_err("@%s: number of arguments > %d!\n",
				__func__, NR_PPM_CLUSTERS * 2);
			goto out;
		}

		if (kstrtoint(tok, 10, &data)) {
			ppm_err("@%s: Invalid input: %s\n", __func__, tok);
			goto out;
		} else {
			if (i % 2) /* OPP */
				cobra_lookup_data.limit[i/2].opp = data;
			else /* core */
				cobra_lookup_data.limit[i/2].core = data;

			i++;
		}
	}

out:
	free_page((unsigned long)buf);
	return count;

}

#ifdef CONFIG_HW_PPM_DVFS_LIMIT
#include <trace/events/cpufreq_schedutil.h>

static int ppm_dvfs_limit_notifier(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	struct ppm_policy_data *pos;
	int max;
	int cid;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_DONE;

	ppm_lock(&ppm_main_info.lock);

	list_for_each_entry(pos, &ppm_main_info.policy_list, link) {
		if (!pos->is_enabled)
			continue;
		if (!pos->is_activated)
			continue;

		ppm_lock(&pos->lock);
		if (pos->policy == PPM_POLICY_DLPT) {
#ifdef SINGLE_CLUSTER
			cid = cpufreq_get_cluster_id(policy->cpu);
#else
			cid = arch_get_cluster_id(policy->cpu);
#endif
			max = mt_cpufreq_get_freq_by_idx(cid,
					pos->req.limit[cid].max_cpufreq_idx);
			if (max > 0 && policy->max > max)
				cpufreq_verify_within_limits(policy, 0, max);

			trace_ppm_dlpt_limit(policy->cpu,
				pos->req.limit[cid].max_cpufreq_idx, max);
		}
		ppm_unlock(&pos->lock);
	}

	ppm_unlock(&ppm_main_info.lock);

	return NOTIFY_OK;
}

static struct notifier_block ppm_dvfs_limit_notifier_block = {
	.notifier_call = ppm_dvfs_limit_notifier,
};

/*
 * dvfs_limit_enabled is used to control dvfs limit of ppm
 *
 * 0: ppm dvfs limit will works, default
 * 1: ppm dvfs limit will not work, user space should take care of dvfs_limit
 */
static int ppm_dvfs_limit_enabled_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", ppm_main_info.is_dvfs_limit_enabled);

	return 0;
}

static ssize_t ppm_dvfs_limit_enabled_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int enabled = 0;
	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &enabled)) {
		if (!enabled) {
			cpufreq_unregister_notifier(&ppm_dvfs_limit_notifier_block,
						    CPUFREQ_POLICY_NOTIFIER);
			ppm_lock(&ppm_main_info.lock);
			ppm_main_info.is_dvfs_limit_enabled = false;
			ppm_unlock(&ppm_main_info.lock);
		} else {
			/* only changing 0 to 1 needs to register notifier */
			ppm_lock(&ppm_main_info.lock);
			if (!ppm_main_info.is_dvfs_limit_enabled) {
				ppm_main_info.is_dvfs_limit_enabled = true;
				ppm_unlock(&ppm_main_info.lock);
				cpufreq_register_notifier(&ppm_dvfs_limit_notifier_block,
						CPUFREQ_POLICY_NOTIFIER);
			} else
				ppm_unlock(&ppm_main_info.lock);
		}
	} else
		ppm_err("echo [0/1] > /proc/ppm/dvfs_limit_enabled\n");

	free_page((unsigned long)buf);
	return count;
}

PROC_FOPS_RW(dvfs_limit_enabled);

#define LIMIT_BUF_SIZE 1024

/*
 * output:
 * 1. PPM's min/max
 * 2. each policy's min/max
 *
 * note: reading dvfs_limit will not reach EOF, use poll to check changes
 */
static ssize_t ppm_dvfs_limit_proc_read(struct file *file, char __user *buf,
					size_t size, loff_t *ppos)
{
	char *limit_buf;
	int offset = 0;
	int i;
	int min, max;
	struct ppm_policy_data *pos;
	int n, err;

	limit_buf = kzalloc(LIMIT_BUF_SIZE, GFP_KERNEL);
	if (!limit_buf)
		return -ENOMEM;

	ppm_lock(&ppm_main_info.lock);

	ppm_main_info.is_dvfs_limit_changed = false;

	offset += sprintf(limit_buf + offset, "PPM:");
	for_each_ppm_clusters(i) {
		min = mt_cpufreq_get_freq_by_idx(i,
			ppm_main_info.last_req.cpu_limit[i].min_cpufreq_idx);
		max = mt_cpufreq_get_freq_by_idx(i,
			ppm_main_info.last_req.cpu_limit[i].max_cpufreq_idx);
		offset += sprintf(limit_buf + offset, " %d %d", min, max);
	}
	offset += sprintf(limit_buf + offset, "\n");

	list_for_each_entry(pos, &ppm_main_info.policy_list, link) {
		ppm_lock(&pos->lock);

		offset += sprintf(limit_buf + offset, "%s:", pos->name);
		for_each_ppm_clusters(i) {
			min = max = -1;
			if (pos->is_enabled && pos->is_activated) {
				min = mt_cpufreq_get_freq_by_idx(i,
					pos->req.limit[i].min_cpufreq_idx);
				max = mt_cpufreq_get_freq_by_idx(i,
					pos->req.limit[i].max_cpufreq_idx);
			}

			offset += sprintf(limit_buf + offset,
					  " %d %d", min, max);
		}

		ppm_unlock(&pos->lock);
		offset += sprintf(limit_buf + offset, "\n");
	}

	ppm_unlock(&ppm_main_info.lock);

	n = min((size_t)offset, size);
	err = copy_to_user(buf, limit_buf, n);
	if (err)
		n = -EFAULT;

	kfree(limit_buf);

	return n;
}

static unsigned int ppm_dvfs_limit_proc_poll(struct file *file,
					     poll_table *wait)
{
	int events = 0;

	ppm_lock(&ppm_main_info.lock);

	poll_wait(file, &ppm_main_info.dvfs_limit_wqh, wait);

	if (ppm_main_info.is_dvfs_limit_changed)
		events = POLLIN;

	ppm_unlock(&ppm_main_info.lock);

	return events;
}

/*
 * dvfs_limit is used to show cpufreq limit of ppm, and polling this interface
 * will be notified that cpufreq limit of ppm is changed
 */
static const struct file_operations ppm_dvfs_limit_proc_fops = {
	.owner          = THIS_MODULE,
	.read           = ppm_dvfs_limit_proc_read,
	.llseek         = no_llseek,
	.poll           = ppm_dvfs_limit_proc_poll,
};

struct ppm_dvfs_limit_entry {
	const char *name;
	const kuid_t uid;
	const kgid_t gid;
	const mode_t mode;
	const struct file_operations *fops;
};

static void ppm_dvfs_limit_entries_init(struct proc_dir_entry *dir)
{
	int i = 0;
	struct proc_dir_entry *entry;

	const static struct ppm_dvfs_limit_entry ppm_dvfs_limit_entries[] = {
		{.name = "dvfs_limit", .uid = KUIDT_INIT(1000),
			.gid = KGIDT_INIT(1000), .mode = 0440,
			.fops = &ppm_dvfs_limit_proc_fops },
		{.name = "dvfs_limit_enabled", .uid = KUIDT_INIT(1000),
			.gid = KGIDT_INIT(1000), .mode = 0660,
			.fops = &ppm_dvfs_limit_enabled_proc_fops },
		/* invalid attr */
		{.name = NULL, .uid = KUIDT_INIT(-1), .gid = KGIDT_INIT(-1),
			.mode = 0000, .fops = NULL },
	};

	while (ppm_dvfs_limit_entries[i].name) {
		entry = proc_create(ppm_dvfs_limit_entries[i].name,
				    ppm_dvfs_limit_entries[i].mode, dir,
				    ppm_dvfs_limit_entries[i].fops);
		if (entry == NULL) {
			ppm_err("%s(), create /proc/ppm/%s failed\n",
				__func__, ppm_dvfs_limit_entries[i].name);
			i++;
			continue;
		}

		proc_set_user(entry, ppm_dvfs_limit_entries[i].uid,
			      ppm_dvfs_limit_entries[i].gid);

		i++;
	}
}
#endif

PROC_FOPS_RW(func_debug);
PROC_FOPS_RW(debug);
PROC_FOPS_RW(enabled);
PROC_FOPS_RW(exclusive_core);
PROC_FOPS_RO(dump_power_table);
PROC_FOPS_RO(dump_policy_list);
PROC_FOPS_RW(policy_status);
PROC_FOPS_RO(dump_dvfs_table);
PROC_FOPS_RW(cobra_budget_to_limit);
PROC_FOPS_RW(cobra_limit_to_budget);

int ppm_procfs_init(void)
{
	struct proc_dir_entry *dir = NULL;
	int i;
	char str[32];

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(func_debug),
		PROC_ENTRY(debug),
		PROC_ENTRY(enabled),
		PROC_ENTRY(exclusive_core),
		PROC_ENTRY(dump_power_table),
		PROC_ENTRY(dump_policy_list),
		PROC_ENTRY(policy_status),
		PROC_ENTRY(cobra_budget_to_limit),
		PROC_ENTRY(cobra_limit_to_budget),
	};

	dir = proc_mkdir("ppm", NULL);
	if (!dir) {
		ppm_err("@%s: fail to create /proc/ppm dir\n", __func__);
		return -ENOMEM;
	}

	/* mkdir for policies */
	policy_dir = proc_mkdir("policy", dir);
	if (!policy_dir) {
		ppm_err("fail to create /proc/ppm/policy dir\n");
		return -ENOMEM;
	}

	profile_dir = proc_mkdir("profile", dir);
	if (!profile_dir) {
		ppm_err("fail to create /proc/ppm/profile dir\n");
		return -ENOMEM;
	}

	cpi_dir = proc_mkdir("cpi", dir);
	if (!cpi_dir) {
		ppm_err("fail to create /proc/ppm/cpi dir\n");
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, 0664,
			dir, entries[i].fops))
			ppm_err("%s(), create /proc/ppm/%s failed\n",
				__func__, entries[i].name);
	}

#ifdef CONFIG_HW_PPM_DVFS_LIMIT
	ppm_dvfs_limit_entries_init(dir);
#endif

	for_each_ppm_clusters(i) {
		sprintf(str, "dump_cluster_%d_dvfs_table", i);

		if (!proc_create_data(str, 0644,
			dir, &ppm_dump_dvfs_table_proc_fops,
			&ppm_main_info.cluster_info[i]))
			ppm_err("%s(), create /proc/ppm/%s failed\n",
				__func__, str);
	}

	return 0;
}

