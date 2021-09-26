/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2017-2021. All rights reserved.
 * Description: the eima_module.c for EMUI Integrity Measurement
 *     Architecture(EIMA) kernel space init and deinit
 * Create: 2017-12-20
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/lsm_hooks.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/oom.h> /* for find_lock_task_mm */
#include <linux/printk.h>
#include <linux/proc_fs.h>
#include <linux/security.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tpm.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <chipset_common/security/hw_kernel_stp_interface.h>
#include <crypto/hash.h>
#include <crypto/hash_info.h>

#include "dkm.h"
#include "eima_agent_api.h"
#include "eima_fs.h"

#include "eima_netlink.h"
#include "eima_queue.h"
#include "eima_utils.h"

#include <linux/sched/mm.h>
#include <linux/sched/signal.h>

static void eima_dim_do_work(struct work_struct *unused);
static void eima_baselining_do_work(struct work_struct *unused);

static DECLARE_WORK(eima_dim_work, eima_dim_do_work);
static DECLARE_WORK(eima_baselining_work, eima_baselining_do_work);

static struct workqueue_struct *g_eima_workqueue;

static struct timer_list g_eima_dim_timer;

/* EIMA Policy Info File:Measure target list with measure type and obj name */
pif_struct_t g_eima_pif;

#define SCAN_CYCLE (10 * 60 * 1000) /* 10min */

#define TIMER_PERIOC (jiffies + msecs_to_jiffies(SCAN_CYCLE))
#define TIMER_FIRST (jiffies + msecs_to_jiffies(SCAN_CYCLE))

int g_eima_used_tpm; /* global flag TPM status */
static int g_has_baselining_trigger;

static inline unsigned char check_status(uint status, uint type)
{
	return (status >> type) & 0x1;
}

#ifdef _EIMA_RUN_ON_QEMU
int send_eima_data(int type, const struct m_list_msg *mlist_msg)
{
	return 0;
}
#endif

/* create eima proc file */
static struct proc_dir_entry *g_proc_entry;

static const umode_t g_file_creat_ro_mode = 0440;
static const kuid_t g_root_uid = KUIDT_INIT((uid_t)0);
static const kgid_t g_system_gid = KGIDT_INIT((gid_t)1000); /* 1000 is the system gid */

static int g_has_trigger_flag;

/*
 * trigger baselining, delay baselining to FWK
 * boot-completed, reuse root-scan trigger.
 */
static int eima_baselining_trigger(void)
{
	if (g_eima_workqueue == NULL)
		return ERROR_CODE;

	if (g_has_baselining_trigger == 0) {
		if (queue_work(g_eima_workqueue, &eima_baselining_work) == 0) {
			eima_error("Baselining work has joined the queued");
			return ERROR_CODE;
		}
		g_has_baselining_trigger = 1;
		eima_info("Baselining has been trigger success");
	}

	return 0;
}

static int eima_proc_show(struct seq_file *handle, void *reserved)
{
	int ret = 0;

	(void)reserved;
	/* EIMA baselineling trigger */
	if (g_has_trigger_flag == 0) {
		ret = eima_baselining_trigger();
		if (ret == 0)
			g_has_trigger_flag = 1;
	}

	seq_printf(handle, "%d", ret);
	return 0;
}

static int eima_proc_open(struct inode *inode, struct file *file)
{
	(void)inode;
	return single_open(file, eima_proc_show, NULL);
}

int eima_proc_init(void)
{
	static const struct file_operations eima_proc_fops = {
		.open = eima_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};

	g_proc_entry = proc_create("root_scan", g_file_creat_ro_mode,
					NULL, &eima_proc_fops);
	if (g_proc_entry == NULL) {
		eima_error("g_proc_entry create is failed");
		return -ENOMEM;
	}

	/* set proc file gid to system gid */
	proc_set_user(g_proc_entry, g_root_uid, g_system_gid);

	eima_info("g_proc_entry init success");
	return 0;
}

void eima_proc_deinit(void)
{
	remove_proc_entry("root_scan", NULL);
	if (g_proc_entry != NULL)
		g_proc_entry = NULL;
	eima_info("g_proc_entry cleanup success");
}

int stp_eima_trigger(void)
{
	int ret;
	unsigned char status;
	unsigned char credible = STP_REFERENCE;
	uint tee_status;
	struct stp_item item;
	char *file_path = NULL;

	eima_dim_do_work(NULL);
	tee_status = get_tee_status();

	status = check_status(tee_status, EIMABIT);
	item.id = item_info[EIMA].id;
	item.status = status;
	item.credible = credible;
	item.version = 0;

	ret = strcpy_s(item.name, STP_ITEM_NAME_LEN, item_info[EIMA].name);
	if (ret != EOK) {
		eima_error("strcpy_s failed, s_ret = %d", ret);
		return -ENOMEM;
	}

	eima_debug("tee status in eima = %x", tee_status);
	file_path = get_hash_error_file_path();
	if ((file_path == NULL) || (file_path[0] == '\0')) {
		ret = kernel_stp_upload(item, NULL);
	} else {
		eima_debug("received tampered file path from ta: %s, uploading", file_path);
		ret = kernel_stp_upload(item, file_path);
	}
	if (ret != 0)
		eima_error("stp eima upload failed");

	return 0;
}

static int eima_comp_proc_filename(const char *file_name, char *target_pn,
				int target_pn_len, struct task_struct *p)
{
	char *target_p = NULL;
	struct mm_struct *mm = NULL;
	struct task_struct *t = NULL;

	if ((target_pn == NULL) || (p == NULL) || (file_name == NULL)) {
		eima_error("the parameter is null");
		return 1;
	}

	t = (struct task_struct *)find_lock_task_mm(p);
	if (t == NULL)
		return 1;

	mm = t->mm;
	if (mm == NULL) {
		task_unlock(t);
		return 1;
	}

	if (!down_read_trylock(&mm->mmap_sem)) {
		task_unlock(t);
		return 1;
	}

	if (mm->exe_file == NULL) {
		up_read(&mm->mmap_sem);
		task_unlock(t);
		return 1;
	}

	target_p = d_path(&mm->exe_file->f_path, target_pn, target_pn_len);
	if (IS_ERR_OR_NULL(target_p) ||
		(strlen(target_p) > EIMA_NAME_STRING_LEN)) {
		eima_error("the target proc is not valid here");
		up_read(&mm->mmap_sem);
		task_unlock(t);
		return 1;
	}

	up_read(&mm->mmap_sem);
	task_unlock(t);
	return strcmp(target_p, file_name);
}

/*
 * Description: This function is called to check if the given process is exist.
 * Return Value:
 *     1: Exist
 *     0: Not exist
 */
static int eima_get_proc_from_path(const char *file_name, int filename_len,
				struct task_struct **proc)
{
	char *target_pn = NULL;
	struct task_struct *p = NULL;
	int ret;

	if (filename_len > EIMA_NAME_STRING_LEN) {
		eima_error("filename length is %d, exceeds max length", filename_len);
		return 0;
	}

	target_pn = kmalloc(PATH_MAX, GFP_ATOMIC);
	if (target_pn == NULL) {
		eima_error("get proc from path failed, memory is not enough");
		return 0;
	}

	read_lock(&tasklist_lock);
	for_each_process(p) {
		ret = eima_comp_proc_filename(file_name, target_pn,
					PATH_MAX, p);
		if (ret == 0) {
			*proc = p;
			kfree(target_pn);
			read_unlock(&tasklist_lock);
			return 1;
		}
	}

	kfree(target_pn);
	read_unlock(&tasklist_lock);
	return 0;
}

static int eima_shash_updata(struct file *file, struct shash_desc *shash,
			loff_t *offset, char *rbuf, int rbuf_len)
{
	int buf_len;
	int rc;
	mm_segment_t old_fs;
	char *buf = rbuf;

	if (buf == NULL) {
		rc = -ENOMEM;
		eima_error("rbuf init failed");
		return rc;
	}

	old_fs = get_fs();
	set_fs(get_ds());

	buf_len = __vfs_read(file, (char __user *)buf, rbuf_len, offset);
	set_fs(old_fs);

	if (buf_len <= 0) {
		eima_error("__vfs_read failed buf_len is %d", buf_len);
		return 0;
	}

	rc = crypto_shash_update(shash, buf, buf_len);
	if (rc < 0) {
		eima_error("crypto_shash_update failed rc is %d", rc);
		return rc;
	}
	return 0;
}

static int eima_file_shash_updata(struct file *file, loff_t i_size,
		struct shash_desc *shash)
{
	int read = 0;
	loff_t offset = 0;
	char *buf = NULL;

	if (!(file->f_mode & FMODE_READ)) {
		file->f_mode |= FMODE_READ;
		read = 1;
	}

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL) {
		eima_error("rbuf init failed");
		if (read)
			file->f_mode &= ~FMODE_READ;
		return -ENOMEM;
	}

	while (i_size > offset) {
		if (eima_shash_updata(file, shash, &offset, buf, PAGE_SIZE) < 0)
			break;
	}
	if (read)
		file->f_mode &= ~FMODE_READ;

	kfree(buf);

	return 0;
}

/*
 * When file hashes are calculated we need to add security hooks to proper
 * kernel functions. Mmap is called before process code is copied from file
 * to memory so we want to hook that.
 */
static int eima_calc_file_hash(struct file *file, char *hash, int hash_len)
{
	loff_t i_size;
	int rc;
	struct crypto_shash *tfm = crypto_alloc_shash(HASH_ALG, 0, 0);

	SHASH_DESC_ON_STACK(shash, tfm);
	(void)hash_len;

	if (IS_ERR(tfm)) {
		rc = PTR_ERR(tfm);
		eima_error("the error tfm value is %d", rc);
		return rc;
	}

	shash->tfm = tfm;
	shash->flags = 0;

	rc = crypto_shash_init(shash);
	if (rc < 0) {
		eima_error("crypto_shash_init error: %d", rc);
		crypto_free_shash(tfm);
		return rc;
	}

	i_size = i_size_read(file_inode(file));
	if (i_size == 0) {
		rc = -EBADF;
		eima_error("the value of i_size is 0");
		goto out;
	}

	rc = eima_file_shash_updata(file, i_size, shash);
	if (rc != 0) {
		eima_error("file shash updata failed");
		return rc;
	}

out:
	if (rc == 0)
		rc = crypto_shash_final(shash, (u8 *)hash);

	crypto_free_shash(tfm);

	return rc;
}

static void eima_get_proc_pids(void)
{
	int i;
	int j;
	int ret;
	integrity_targets_t *tmp_target = NULL;
	struct task_struct *proc = NULL;

	for (i = 0; i < g_eima_pif.policy_count; i++) {
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++) {
			tmp_target = &(g_eima_pif.policy[i].targets[j]);
			if ((tmp_target->type != EIMA_DYNAMIC) ||
				(tmp_target->target_pid != 0))
				continue;

			ret = eima_get_proc_from_path(tmp_target->file,
						strlen(tmp_target->file),
						&proc);
			if (ret != 0)
				tmp_target->target_pid = proc->pid;
			else
				tmp_target->target_pid = 0; /* error pid here */
		}
	}
}

static int eima_get_target_hash_value(char *hash, int *hlen,
				integrity_targets_t *target)
{
	int ret;
	struct task_struct *p = NULL;
	char *target_pn = NULL;
	struct pid *tmp_pid = NULL;

	if (target == NULL) {
		eima_error("the parameter is null");
		return 1;
	}

	rcu_read_lock();
	tmp_pid = find_vpid(target->target_pid);
	rcu_read_unlock();
	if (tmp_pid == NULL) {
		eima_warning("struct of pid [%d] get failed",
			target->target_pid);
		target->target_pid = 0;
		return 1;
	}

	p = get_pid_task(tmp_pid, PIDTYPE_PID);
	if (p == NULL) {
		eima_warning("task of pid [%d] get failed", target->target_pid);
		target->target_pid = 0;
		return 1;
	}

	target_pn = kmalloc(PATH_MAX, GFP_ATOMIC);
	if (target_pn == NULL) {
		eima_error("memory is not enough");
		goto out;
	}

	ret = eima_comp_proc_filename(target->file, target_pn, PATH_MAX, p);
	kfree(target_pn);
	if (ret != 0) {
		eima_error("eima comp proc filename is fail");
		goto out;
	}

	ret = measure_process(p, 0, hash, EIMA_HASH_DIGEST_SIZE, hlen);
	if (ret != 0) {
		eima_error("measure process failed: %s", target->file);
		goto out;
	}

	put_task_struct(p);
	return 0;
out:
	target->target_pid = 0;
	put_task_struct(p);
	return 1;
}

static int eima_encap_measure_entry(struct eima_template_entry *temp_entry,
				uint32_t hash_len, struct m_entry *entry)
{
	int s_ret;

	entry->type = temp_entry->type;
	entry->hash_len = hash_len;
	s_ret = memcpy_s(entry->hash, EIMA_HASH_DIGEST_SIZE,
			temp_entry->digest, hash_len);
	if (s_ret != EOK) {
		eima_error("memcpy_s failed, s_ret = %d", s_ret);
		return s_ret;
	}

	entry->fn_len = strlen(temp_entry->fn) + 1;
	s_ret = strcpy_s(entry->fn, EIMA_NAME_STRING_LEN + 1, temp_entry->fn);
	if (s_ret != EOK) {
		eima_error("strcpy_s failed, s_ret = %d", s_ret);
		return s_ret;
	}

	return s_ret;
}

static int eima_set_entry_contents(struct eima_template_entry *entry,
				const char *hash, int hash_len,
				const integrity_targets_t *tmp_target,
				enum eima_measure_type type)
{
	int ret;

	ret = memset_s((void *)entry, sizeof(*entry), 0, sizeof(*entry));
	if (ret != EOK) {
		eima_error("memset_s failed, s_ret=%d", ret);
		return ret;
	}
	ret = memcpy_s(entry->digest, EIMA_HASH_DIGEST_SIZE, hash, hash_len);
	if (ret != EOK) {
		eima_error("memcpy_s failed, s_ret=%d", ret);
		return ret;
	}

	ret = strcpy_s(entry->fn, EIMA_NAME_STRING_LEN + 1, tmp_target->file);
	if (ret != EOK) {
		eima_error("strcpy_s failed, s_ret = %d", ret);
		return ret;
	}
	entry->type = type;

	return 0;
}

static int get_hash_value(char *hash, int *hlen, integrity_targets_t *tmp_target)
{
	int ret;

	ret = memset_s(hash, EIMA_HASH_DIGEST_SIZE, 0, EIMA_HASH_DIGEST_SIZE);
	if (ret != EOK)
		return ret;

	ret = eima_get_target_hash_value(hash, hlen, tmp_target);
	if (ret != 0) {
		eima_error("eima get target hash value failed: %s", tmp_target->file);
		return ret;
	}

	return 0;
}

/*
 * This function is called when process memory hashes are measured,
 * Teecd hash is measured always. Other processes(full path of the executable)
 * can be given in the PIF file.
 */
static int eima_proc_measure_scan(struct m_list_msg *mlist_msg)
{
	char hash[EIMA_HASH_DIGEST_SIZE];
	integrity_targets_t *tmp_target = NULL;
	struct eima_template_entry entry;
	int i;
	int j;
	int ret;
	int hlen = 0;

	if (mlist_msg == NULL) {
		eima_error("mlist_msg in proc mem scan init err");
		return -EINVAL;
	}

	for (i = 0; i < g_eima_pif.policy_count; i++) {
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++) {
			tmp_target = &(g_eima_pif.policy[i].targets[j]);
			if ((tmp_target->type != EIMA_DYNAMIC) ||
				(tmp_target->target_pid == 0)) {
				continue;
			}

			if (mlist_msg->num >= EIMA_MAX_MEASURE_OBJ_CNT) {
				eima_info("measure data cnt exceed MAX num");
				return 0;
			}

			if (get_hash_value(hash, &hlen, tmp_target) != 0)
				continue;

			eima_debug("process %s measured", tmp_target->file);

			ret = eima_set_entry_contents(&entry, hash, hlen,
						tmp_target, EIMA_DYNAMIC);
			if (ret != EOK) {
				eima_error("set_entry_contents failed");
				return ret;
			}

			ret = eima_add_template_entry(&entry);
			if ((ret != EOK) && (ret != -EEXIST))
				eima_error("dynamic add entry failed:%d", ret);

			ret = eima_encap_measure_entry(&entry, hlen,
					&(mlist_msg->m_list[mlist_msg->num]));
			if (ret != EOK)
				return ret;

			mlist_msg->num++;
		}
	}

	return 0;
}

/*
 * This function is called when process memory hashes are measured,
 * Teecd hash is measured always. Other processes(full path of the executable)
 * can be given in the PIF file
 */
static int eima_proc_mem_scan(struct m_list_msg *mlist_msg)
{
	eima_get_proc_pids();
	return eima_proc_measure_scan(mlist_msg);
}

static int eima_calc_static_file_hash(const integrity_targets_t *targets,
				char *hash, int hash_len, bool is_baseline)
{
	int ret;
	struct file *filp = NULL;
	const char *path = targets->file;

	if (is_baseline && targets->sent_baseline)
		return -1;

	filp = eima_file_open(path, O_RDONLY, 0);
	if (filp == NULL) {
		eima_warning("open file %s failed", path);
		return -1;
	}

	ret = eima_calc_file_hash(filp, hash, hash_len);
	eima_file_close(filp);

	if (ret != 0)
		eima_error("calc file hash err %d", ret);

	return ret;
}

static int eima_file_measurement_scan(struct m_list_msg *mlist_msg,
				bool is_baseline)
{
	int i;
	int j;
	int ret;
	char hash[EIMA_HASH_DIGEST_SIZE];
	struct eima_template_entry entry;
	integrity_targets_t *targets = NULL;

	for (i = 0; i < g_eima_pif.policy_count; i++) {
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++) {
			if (mlist_msg->num >= EIMA_MAX_MEASURE_OBJ_CNT) {
				eima_error("measure data cnt exceed MAX num");
				return 0;
			}

			targets = &g_eima_pif.policy[i].targets[j];
			if (targets->type != EIMA_STATIC)
				continue;

			ret = eima_calc_static_file_hash(targets, hash,
							EIMA_HASH_DIGEST_SIZE,
							is_baseline);
			if (ret != 0)
				continue;

			if (is_baseline)
				targets->sent_baseline = 1;

			eima_debug("file %s measured", targets->file);

			ret = eima_set_entry_contents(&entry,
						hash, sizeof(hash),
						targets, EIMA_STATIC);
			if (ret != 0)
				return ret;

			ret = eima_add_template_entry(&entry);
			if (ret != EOK && ret != -EEXIST)
				eima_error("static eima add template entry failed. fc = %d", ret);

			ret = eima_encap_measure_entry(&entry,
						EIMA_HASH_DIGEST_SIZE,
						&(mlist_msg->m_list[mlist_msg->num]));
			if (ret != EOK)
				return ret;

			mlist_msg->num++;
		}
	}
	return 0;
}

static int send_data_to_ca(int msg_type, struct m_list_msg *mlist_msg)
{
	int ret;

	if (mlist_msg->num > 0) {
		ret = strcpy_s(mlist_msg->usecase, EIMA_NAME_STRING_LEN + 1,
			EIMA_POLICY_USECASE_NAME);
		if (ret != EOK) {
			eima_error("strcpy_s failed, s_ret = %d", ret);
			return ret;
		}

		ret = send_eima_data(msg_type, mlist_msg);
		if (ret != 0) {
			eima_error("eima baseline data send failed, ret = %d", ret);
			return ret;
		}
	}

	return 0;
}

#ifdef EIMA_STATIC_MEASUREMENT_IN_TIMER
static int eima_static_do_work(struct m_list_msg *mlist_msg)
{
	int ret;

	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
			sizeof(struct m_list_msg));
	if (ret != EOK) {
		eima_error("static memset_s failed, s_ret=%d", ret);
		return ret;
	}

	ret = eima_file_measurement_scan(mlist_msg, false);
	if (ret != 0) {
		eima_error("eima static measurement scan error");
		return ret;
	}

	ret = send_data_to_ca(EIMA_MSG_RUNTIME_INFO, mlist_msg);
	if (ret != 0) {
		eima_error("eima static measurement data send failed");
		return ret;
	}

	eima_debug("eima static do work success");
	return 0;
}
#endif

static int eima_dynamic_do_work(struct m_list_msg *mlist_msg)
{
	int ret;

	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
			sizeof(struct m_list_msg));
	if (ret != EOK) {
		eima_error("dynamic memset_s failed, s_ret = %d", ret);
		return ret;
	}

	ret = eima_proc_mem_scan(mlist_msg);
	if (ret != 0) {
		eima_error("eima dynamic measurement scan error, ret = %d",
			ret);
		return ret;
	}

	ret = send_data_to_ca(EIMA_MSG_RUNTIME_INFO, mlist_msg);
	if (ret != 0) {
		eima_error("eima dynamic measurement data send failed");
		return ret;
	}

	return 0;
}

/* lint -save -e578 */
static void eima_dim_do_work(struct work_struct *unused)
{
	struct m_list_msg *mlist_msg = NULL;
	int ret;

	(void)unused;

	mlist_msg = kzalloc(sizeof(struct m_list_msg), GFP_KERNEL);
	if (mlist_msg == NULL) {
		eima_error("eima dim mlist_msg init failed");
		return;
	}
#ifdef EIMA_STATIC_MEASUREMENT_IN_TIMER
	ret = eima_static_do_work(mlist_msg);
	if (ret != 0)
		goto error;
#endif
	ret = eima_dynamic_do_work(mlist_msg);
	if (ret != 0)
		goto error;
	kfree(mlist_msg);

	eima_debug("eima dim do work success");
	return;
error:
	kfree(mlist_msg);
	eima_error("eima dim do work failed");
}

#ifndef EIMA_STATIC_MEASUREMENT_IN_TIMER
static int eima_match_target(const struct file *file, integrity_targets_t **target)
{
	int i;
	int j;
	char *pathname = NULL;
	char *path = NULL;

	pathname = kmalloc(PATH_MAX, GFP_KERNEL);
	if (pathname == NULL) {
		eima_error("alloc pathname mem failed");
		return 0;
	}

	path = d_path(&file->f_path, pathname, PATH_MAX);
	if ((path == NULL) || IS_ERR(path) ||
		(strlen(path) > EIMA_NAME_STRING_LEN)) {
		eima_error("the length of path is overflow ,or path is null or error");
		kfree(pathname);
		return 0;
	}

	for (i = 0; i < g_eima_pif.policy_count; i++) {
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++) {
			if ((g_eima_pif.policy[i].targets[j].type == EIMA_STATIC) &&
				memcmp(path, g_eima_pif.policy[i].targets[j].file,
				strlen(path)) == 0) {
				*target = &(g_eima_pif.policy[i].targets[j]);
				kfree(pathname);
				return 1;
			}
		}
	}
	kfree(pathname);
	return 0;
}
/*lint -restore*/

static void eima_async_send_do_work(struct work_struct *w)
{
	int ret;
	struct eima_async_send_work *work_node =
		container_of(w, struct eima_async_send_work, work);

	ret = send_eima_data(work_node->type, work_node->msg);
	if (ret != 0)
		eima_error("eima measure data async send failed, ret = %d", ret);
	else
		eima_debug("eima async send do work success");

	kfree(work_node->msg);
	kfree(work_node);
}

/* lint -save -e429 */
static int eima_send_data_async(integrity_targets_t *target,
				struct m_list_msg *mlist_msg)
{
	int ret;
	struct eima_async_send_work *work_node = NULL;
	enum eima_msg_type type = EIMA_MSG_RUNTIME_INFO;

	ret = strcpy_s(mlist_msg->usecase, EIMA_NAME_STRING_LEN + 1,
		EIMA_POLICY_USECASE_NAME);
	if (ret != EOK) {
		eima_error("strcpy_s failed, s_ret = %d", ret);
		return ret;
	}

	if (target->sent_baseline == 0) {
		target->sent_baseline = 1;
		eima_debug("static baseline ok");
		type = EIMA_MSG_BASELINE;
	}

	work_node = kmalloc(sizeof(struct eima_async_send_work), GFP_KERNEL);
	if (work_node == NULL) {
		eima_error("alloc work_node mem failed");
		return -ENOMEM;
	}

	work_node->msg = mlist_msg;
	work_node->type = type;
	INIT_WORK(&work_node->work, eima_async_send_do_work);

	if (queue_work(g_eima_workqueue, &work_node->work) == 0) {
		eima_error("send data work has joined the queued");
		kfree(work_node);
		return ERROR_CODE;
	}

	return 0;
}
/*lint -restore*/

static int eima_measure_file(struct file *file, integrity_targets_t *target)
{
	int ret;
	struct m_list_msg *mlist_msg = NULL;
	char hash[EIMA_HASH_DIGEST_SIZE] = { 0 };
	struct eima_template_entry entry = {};

	ret = eima_calc_file_hash(file, hash, EIMA_HASH_DIGEST_SIZE);
	if (ret != 0) {
		eima_error("calc file hash err %d", ret);
		return ret;
	}

	ret = eima_set_entry_contents(&entry, hash, EIMA_HASH_DIGEST_SIZE,
				target, EIMA_STATIC);
	if (ret != EOK) {
		eima_error("eima measure file set entry contents failed");
		return ret;
	}

	ret = eima_add_template_entry(&entry);
	if ((ret != EOK) && (ret != -EEXIST))
		eima_error("static runtime eima add template entry failed ret = %d", ret);

	mlist_msg = kzalloc(sizeof(struct m_list_msg), GFP_KERNEL);
	if (mlist_msg == NULL) {
		eima_error("eima mmap file mlist_msg init failed");
		return -ENOMEM;
	}

	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
		sizeof(struct m_list_msg));
	if (ret != EOK)
		goto out;

	ret = eima_encap_measure_entry(&entry, EIMA_HASH_DIGEST_SIZE,
				&(mlist_msg->m_list[mlist_msg->num]));
	if (ret != 0) {
		eima_error("eima static measure data send failed");
		goto out;
	}

	mlist_msg->num++;

	ret = eima_send_data_async(target, mlist_msg);
	if (ret != 0) {
		eima_error("eima static measure data send failed");
		goto out;
	}

	return ret;
out:
	kfree(mlist_msg);
	return ret;
}

static int eima_mmap_file_hook(struct file *file, unsigned long reqprot,
			unsigned long prot, unsigned long flags)
{
	int ret;
	integrity_targets_t *target = NULL;

	(void)reqprot;
	(void)flags;
	if ((file != NULL) && (prot & PROT_EXEC) &&
		(current->mm->exe_file == file)) {
		if (eima_match_target(file, &target)) {
			eima_debug("static-measure file %s in mmap_file_hook",
				target->file);
			ret = eima_measure_file(file, target);
			if (ret != 0) {
				eima_error("eima measure file failed");
				return ret;
			}
		}
	}

	return 0;
}

static struct security_hook_list eima_ops[] __lsm_ro_after_init = {
	LSM_HOOK_INIT(mmap_file, eima_mmap_file_hook),
};
#endif

static int eima_baselining_static_do_work(struct m_list_msg *mlist_msg)
{
	int ret;

	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
		sizeof(struct m_list_msg));
	if (ret != EOK) {
		eima_error("memset_s failed, s_ret=%d", ret);
		return ret;
	}

	ret = eima_file_measurement_scan(mlist_msg, true);
	if (ret != 0) {
		eima_error("eima baseline file measurement scan err");
		return ret;
	}

	ret = send_data_to_ca(EIMA_MSG_BASELINE, mlist_msg);
	if (ret != 0) {
		eima_error("eima static baseline data send failed");
		return ret;
	}

	return 0;
}

static int eima_baselining_dynamic_do_work(struct m_list_msg *mlist_msg)
{
	int ret;

	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
		sizeof(struct m_list_msg));
	if (ret != EOK) {
		eima_error("memset_s failed, s_ret=%d", ret);
		return ret;
	}

	/* generate dynamic measurement(DIM) baseline */
	ret = eima_proc_mem_scan(mlist_msg);
	if (ret != 0) {
		eima_error("eima baseline proc mem scan err");
		return ret;
	}

	ret = send_data_to_ca(EIMA_MSG_BASELINE, mlist_msg);
	if (ret != 0) {
		eima_error("eima dynamic baseline data send failed");
		return ret;
	}

	return 0;
}

/* lint -save -e578 */
static void eima_baselining_do_work(struct work_struct *unused)
{
	int ret;
	struct m_list_msg *mlist_msg = NULL;

	(void)unused;

	mlist_msg = kzalloc(sizeof(struct m_list_msg), GFP_KERNEL);
	if (mlist_msg == NULL) {
		eima_error("mlist_msg in baseline do work init err");
		return;
	}

	/* generate static measurement baseline */
	ret = eima_baselining_static_do_work(mlist_msg);
	if (ret != 0)
		goto error;

	ret = eima_baselining_dynamic_do_work(mlist_msg);
	if (ret != 0)
		goto error;

	kfree(mlist_msg);

	(void)kernel_stp_scanner_register(stp_eima_trigger);
	/* start DIM */
	mod_timer(&g_eima_dim_timer, TIMER_FIRST);
	eima_info("measurement baselining done");

	return;
error:
	if (mlist_msg != NULL)
		kfree(mlist_msg);

	eima_error("eima baselining do work failed");
}

static int generate_trustlist(struct m_list_msg *mlist_msg)
{
	int i;
	int j;
	int ret;

	for (i = 0; i < g_eima_pif.policy_count; i++) {
		for (j = 0; j < g_eima_pif.policy[i].target_count; j++) {
			char *path = g_eima_pif.policy[i].targets[j].file;

			if (mlist_msg->num >= EIMA_MAX_MEASURE_OBJ_CNT) {
				eima_error("measure data cnt exceed max num in trustlist");
				return 1;
			}

			eima_debug("%s trustlist generated", path);
			ret = strcpy_s((mlist_msg->m_list[mlist_msg->num]).fn,
				EIMA_NAME_STRING_LEN + 1, path);
			if (ret != EOK) {
				eima_error("strcpy_s failed, s_ret = %d", ret);
				return -1;
			}
			(mlist_msg->m_list[mlist_msg->num]).type =
				g_eima_pif.policy[i].targets[j].type;
			(mlist_msg->m_list[mlist_msg->num]).fn_len =
				strlen(path) + 1;

			mlist_msg->num++;
		}
	}

	return 0;
}

/* send trustlist to CA in eima module initializing phase */
static int eima_send_trustlist(void)
{
	struct m_list_msg *mlist_msg = NULL;
	int ret;

	mlist_msg = kzalloc(sizeof(struct m_list_msg), GFP_KERNEL);
	if (mlist_msg == NULL) {
		eima_error("mlist_msg in trustlist send init failed");
		return ERROR_CODE;
	}
	ret = memset_s(mlist_msg, sizeof(struct m_list_msg), 0,
		sizeof(struct m_list_msg));
	if (ret != EOK) {
		eima_error("memset_s failed, s_ret=%d", ret);
		goto error;
	}

	/* generate trustlist */
	ret = generate_trustlist(mlist_msg);
	if (ret == -1) {
		eima_error("generate trustlist failed");
		goto error;
	} else if (ret == 1) {
		kfree(mlist_msg);
		eima_error("measure data cnt exceed max num in \
			trustlist, generate trustlist failed");
		return 0;
	}

	/* send trustlist */
	ret = send_data_to_ca(EIMA_MSG_TRUSTLIST, mlist_msg);
	if (ret != 0) {
		eima_error("eima trustlist data send failed");
		goto error;
	}

	kfree(mlist_msg);

	eima_info("trustlist send done");
	return 0;

error:
	if (mlist_msg != NULL)
		kfree(mlist_msg);

	return ERROR_CODE;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
static void eima_dim_timer_func(struct timer_list *timer)
{
	(void)timer;
#else
static void eima_dim_timer_func(unsigned long data)
{
	(void)data;
#endif
	mod_timer(&g_eima_dim_timer, TIMER_PERIOC); /* 10 seconds */

	if (queue_work(g_eima_workqueue, &eima_dim_work) == 0)
		eima_error("dim Timer work has joined the queued");
	else
		eima_debug("dim Timer add work queue success");
}

static int basic_func_init(void)
{
	int ret;
	u8 pcr_i[EIMA_HASH_DIGEST_SIZE];

	/* init proc file */
	ret = eima_proc_init();
	if (ret != 0) {
		eima_error("eima_proc_init init failed");
		return -1;
	}

	/* init eima netlink */
	ret = eima_netlink_init();
	if (ret != 0) {
		eima_info("eima neltink init failed");
		return -1;
	}

	g_eima_used_tpm = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0)
	ret = tpm_pcr_read(NULL, 0, pcr_i);
#else
	ret = tpm_pcr_read(TPM_ANY_NUM, 0, pcr_i);
#endif
	if (ret == 0)
		g_eima_used_tpm = 1;

	if (!g_eima_used_tpm)
		eima_info("No TPM chip found, activating TPM-bypass");

	/* init workqueue */
	g_eima_workqueue = create_singlethread_workqueue("EIMA");
	if (g_eima_workqueue == NULL) {
		eima_error("EIMA workqueue init failed");
		return -1;
	}

	/* init eima fs nodes */
	ret = eima_fs_init();
	if (ret != 0) {
		eima_error("eima fs init failed");
		return -1;
	}
	eima_info("eima fs init success");

	return 0;
}

static int __init eima_module_init(void)
{
	int ret;

	ret = basic_func_init();
	if (ret != 0) {
		eima_error("eima basic function init failed");
		goto error;
	}

	/* send trustlist to CA */
	ret = eima_send_trustlist();
	if (ret != 0) {
		eima_error("eima send trustlist failed");
		goto error;
	}

/* init dim timer */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(&g_eima_dim_timer, eima_dim_timer_func, 0);
#else
	g_eima_dim_timer.function = eima_dim_timer_func;
	g_eima_dim_timer.data = 0;
	init_timer(&g_eima_dim_timer);
#endif

#ifndef EIMA_STATIC_MEASUREMENT_IN_TIMER
	// register LSM hooks for static ima
	security_add_hooks(eima_ops, ARRAY_SIZE(eima_ops), "eima");
#endif

#ifdef _EIMA_RUN_ON_QEMU
	ret = eima_baselining_trigger();
	if (ret != 0)
		goto error;
#endif
	eima_info("eima module init success");

	return 0;
error:
	g_eima_used_tpm = 0;

	eima_proc_deinit();

	eima_netlink_destroy();

	if (g_eima_workqueue != NULL) {
		destroy_workqueue(g_eima_workqueue);
		g_eima_workqueue = NULL;
	}

	eima_fs_exit();
	eima_error("eima module init failed");

	return ERROR_CODE;
}

static void __exit eima_module_exit(void)
{
	eima_proc_deinit();

	/* release netlink socket */
	eima_netlink_destroy();

	if (g_eima_workqueue != NULL) {
		destroy_workqueue(g_eima_workqueue);
		g_eima_workqueue = NULL;
	}

	eima_fs_exit();
	eima_destroy_measurements_list();
}

/* eima module must be loaded early than root scan due to sehooks */
module_init(eima_module_init);
module_exit(eima_module_exit);

MODULE_DESCRIPTION("EMUI Integrity Measurement(EIMA) Driver");
MODULE_LICENSE("GPL v2");
