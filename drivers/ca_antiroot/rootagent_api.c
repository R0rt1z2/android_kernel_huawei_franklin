/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2016-2020. All rights reserved.
 * Description: rootagent API
 * Create: 2016-04-01
 */

#include "rootagent_api.h"
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <asm/sections.h>
#include <asm/syscall.h>
#include "teek_client_id.h"
#include "teek_client_api.h"
#include "rootagent_mem.h"
#include "rootagent_common.h"
#include "rootagent_crypto.h"
#include "rootagent.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0))
#include <linux/lsm_hooks.h>
#endif

static TEEC_Context g_context;
static TEEC_Session g_session;
static int g_inited; /* init flag, 0:uninitialized, 1:initialized */
static int g_count;  /* init count */
static TEEC_UUID g_uuid = UUID_RM_TA; /* TA service ID */
/* for invokecmd params[0].tmpref.buffer */
static TEEC_TempMemoryReference g_mem;
static DEFINE_MUTEX(root_monitor_lock);

/* for cbc crypto buffer: rm_command_in_out */
static TEEC_TempMemoryReference g_swap_mem;
static uint8_t g_chal_req_key[CHALLENGE_REQ_KEY_LENGTH];
static uint8_t g_chal_key[CHALLENGE_KEY_LENGTH];
static uint8_t g_nounce[CHALLENGE_NOUNCE_LENGTH];

static uint8_t g_eima_nounce[CHALLENGE_NOUNCE_LENGTH];
static int g_root_flag = REV_NOT_ROOT;
static uint32_t g_tee_scan_status = REV_NOT_ROOT;
static u8 g_package_name[] = "antiroot-ca";
static int g_se_hooks_num;

/* only consider rootscan and eima bits form tee(but rootproc) */
static uint32_t g_tee_valid_bits = ((1 << KERNELCODEBIT) |
				(1 << SYSTEMCALLBIT) |
				(1 << SESTATUSBIT) | (1 << SEHOOKBIT) |
				(1 << SETIDBIT) | (1 << EIMABIT) |
				(1 << RODATABIT));

#define MAX_MSGLIST_FILE_LEN 256

/* get the tampered file path when the measurement of one process failed */
#define FNAME_LENGTH 256
static char g_eima_hash_error_file_path[FNAME_LENGTH];
char *get_hash_error_file_path(void)
{
	return g_eima_hash_error_file_path;
}

uint32_t get_tee_status(void)
{
	return (g_tee_scan_status & g_tee_valid_bits);
}

void root_monitor_finalize(void)
{
	mutex_lock(&root_monitor_lock);
	rm_mem_destroy();
	g_mem.buffer = NULL;
	g_mem.size = 0;
	g_swap_mem.buffer = NULL;
	g_swap_mem.size = 0;
	TEEK_CloseSession(&g_session);
	TEEK_FinalizeContext(&g_context);
	g_inited = 0;
	g_count = 0;
	mutex_unlock(&root_monitor_lock);
}

static int not_init_agent(void)
{
	return ((g_inited == 0) || (g_mem.buffer == NULL));
}

static TEEC_Result rootagent_open_session(void)
{
	uint32_t origin = 0;
	u32 root_id = 0;
	TEEC_Result teec_ret;
	TEEC_Operation operation;

	teec_ret = TEEK_InitializeContext(NULL, &g_context);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("rootagent initialize context failed\n");
		return teec_ret;
	}

	operation.started = 1;
	operation.cancel_flag = 0;
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE,
		TEEC_MEMREF_TEMP_INPUT, TEEC_MEMREF_TEMP_INPUT);
	/*
	 * operation.params is a array which saves the message sending to TA,
	 * it's total length is 4, so params[2] or params[3] is third or fourth
	 * params and paramTypes is TEEC_MEMREF_TEMP_INPUT
	 */
	operation.params[2].tmpref.buffer = (void *)(&root_id);
	operation.params[2].tmpref.size = sizeof(root_id);
	operation.params[3].tmpref.buffer = (void *)(g_package_name);
	operation.params[3].tmpref.size = strlen(g_package_name) + 1; /* lint !e64 */
	teec_ret = TEEK_OpenSession(&g_context, &g_session, &g_uuid,
			TEEC_LOGIN_IDENTIFY, NULL, &operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("root agent open session failed\n");
		TEEK_FinalizeContext(&g_context);
	}

	return teec_ret;
}

TEEC_Result root_monitor_tee_init(void)
{
	TEEC_Result teec_ret;

	if ((get_ro_secure() != 0) && (g_root_flag != REV_NOT_ROOT)) {
		antiroot_error("device is rooted! ro secure: %d, root flag: %d\n",
			get_ro_secure(), g_root_flag);
		return TEE_ERROR_ANTIROOT_RSP_FAIL; /* lint !e570 */
	}
	mutex_lock(&root_monitor_lock);
	if (g_inited != 0) {
		antiroot_debug(ROOTAGENT_DEBUG_API, "RootAgent has already initialized");
		mutex_unlock(&root_monitor_lock);
		return TEEC_SUCCESS;
	}
	teec_ret = rm_mem_init();
	if (teec_ret != 0) {
		mutex_unlock(&root_monitor_lock);
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	if (initial_rm_shared_mem(&g_mem, &g_swap_mem) != 0) {
		rm_mem_destroy();
		mutex_unlock(&root_monitor_lock);
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	teec_ret = rootagent_open_session();
	if (teec_ret != TEEC_SUCCESS)
		goto rm_mem_free;

	g_inited = 1;
	g_count++;
	antiroot_debug(ROOTAGENT_DEBUG_API,
		"root_monitor_tee_init ok, initialized count: %d, g_inited: %d\n",
		g_count, g_inited);
	mutex_unlock(&root_monitor_lock);
	return TEEC_SUCCESS;

rm_mem_free:
	rm_mem_destroy();
	mutex_unlock(&root_monitor_lock);
	return teec_ret;
}

#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
static int get_sescan_address(phys_addr_t *phy_addreses, int used_length)
{
	phys_addr_t count = 1;

	if (phy_addreses == NULL) {
		antiroot_error("get sescan address phy_addreses error\n");
		return TEEC_ERROR_GENERIC;
	}

	/*
	 * COLLECT PHY ADDRESS FOR SE_FUNCTIONS
	 * return in DO_GEN_PHY_ADDR macro has no security risk
	 */
	#define DO_GEN_PHY_ADDR(FUNC) \
	do { \
		struct security_hook_list *P = NULL; \
		list_for_each_entry(P, &security_hook_heads.FUNC, list) { \
			phy_addreses[count] = __pa_symbol(&(P->hook.FUNC)); \
			count++; \
			if (((count + used_length) * sizeof(phys_addr_t)) > g_mem.size) \
				return TEEC_ERROR_GENERIC; \
		} \
	} while (0)

	/*
	 * reference initialization in security_hook_heads
	 * in security/security.c
	 */
	DO_GEN_PHY_ADDR(binder_set_context_mgr);
	DO_GEN_PHY_ADDR(binder_transaction);
	DO_GEN_PHY_ADDR(binder_transfer_binder);
	DO_GEN_PHY_ADDR(binder_transfer_file);
	DO_GEN_PHY_ADDR(ptrace_access_check);
	DO_GEN_PHY_ADDR(ptrace_traceme);
	DO_GEN_PHY_ADDR(capget);
	DO_GEN_PHY_ADDR(capset);
	DO_GEN_PHY_ADDR(capable);
	DO_GEN_PHY_ADDR(quotactl);
	DO_GEN_PHY_ADDR(quota_on);
	DO_GEN_PHY_ADDR(syslog);
	DO_GEN_PHY_ADDR(settime);
	DO_GEN_PHY_ADDR(vm_enough_memory);
	DO_GEN_PHY_ADDR(bprm_set_creds);
	DO_GEN_PHY_ADDR(bprm_check_security);
	DO_GEN_PHY_ADDR(bprm_committing_creds);
	DO_GEN_PHY_ADDR(bprm_committed_creds);
	DO_GEN_PHY_ADDR(sb_alloc_security);
	DO_GEN_PHY_ADDR(sb_free_security);
	DO_GEN_PHY_ADDR(sb_copy_data);
	DO_GEN_PHY_ADDR(sb_remount);
	DO_GEN_PHY_ADDR(sb_kern_mount);
	DO_GEN_PHY_ADDR(sb_show_options);
	DO_GEN_PHY_ADDR(sb_statfs);
	DO_GEN_PHY_ADDR(sb_mount);
	DO_GEN_PHY_ADDR(sb_umount);
	DO_GEN_PHY_ADDR(sb_pivotroot);
	DO_GEN_PHY_ADDR(sb_set_mnt_opts);
	DO_GEN_PHY_ADDR(sb_clone_mnt_opts);
	DO_GEN_PHY_ADDR(sb_parse_opts_str);
	DO_GEN_PHY_ADDR(dentry_init_security);
#ifdef CONFIG_SECURITY_PATH
	DO_GEN_PHY_ADDR(path_unlink);
	DO_GEN_PHY_ADDR(path_mkdir);
	DO_GEN_PHY_ADDR(path_rmdir);
	DO_GEN_PHY_ADDR(path_mknod);
	DO_GEN_PHY_ADDR(path_truncate);
	DO_GEN_PHY_ADDR(path_symlink);
	DO_GEN_PHY_ADDR(path_link);
	DO_GEN_PHY_ADDR(path_rename);
	DO_GEN_PHY_ADDR(path_chmod);
	DO_GEN_PHY_ADDR(path_chown);
	DO_GEN_PHY_ADDR(path_chroot);
#endif
	DO_GEN_PHY_ADDR(inode_alloc_security);
	DO_GEN_PHY_ADDR(inode_free_security);
	DO_GEN_PHY_ADDR(inode_init_security);
	DO_GEN_PHY_ADDR(inode_create);
	DO_GEN_PHY_ADDR(inode_link);
	DO_GEN_PHY_ADDR(inode_unlink);
	DO_GEN_PHY_ADDR(inode_symlink);
	DO_GEN_PHY_ADDR(inode_mkdir);
	DO_GEN_PHY_ADDR(inode_rmdir);
	DO_GEN_PHY_ADDR(inode_mknod);
	DO_GEN_PHY_ADDR(inode_rename);
	DO_GEN_PHY_ADDR(inode_readlink);
	DO_GEN_PHY_ADDR(inode_follow_link);
	DO_GEN_PHY_ADDR(inode_permission);
	DO_GEN_PHY_ADDR(inode_setattr);
	DO_GEN_PHY_ADDR(inode_getattr);
	DO_GEN_PHY_ADDR(inode_setxattr);
	DO_GEN_PHY_ADDR(inode_post_setxattr);
	DO_GEN_PHY_ADDR(inode_getxattr);
	DO_GEN_PHY_ADDR(inode_listxattr);
	DO_GEN_PHY_ADDR(inode_removexattr);
	DO_GEN_PHY_ADDR(inode_need_killpriv);
	DO_GEN_PHY_ADDR(inode_killpriv);
	DO_GEN_PHY_ADDR(inode_getsecurity);
	DO_GEN_PHY_ADDR(inode_setsecurity);
	DO_GEN_PHY_ADDR(inode_listsecurity);
	DO_GEN_PHY_ADDR(inode_getsecid);
	DO_GEN_PHY_ADDR(file_permission);
	DO_GEN_PHY_ADDR(file_alloc_security);
	DO_GEN_PHY_ADDR(file_free_security);
	DO_GEN_PHY_ADDR(file_ioctl);
	DO_GEN_PHY_ADDR(mmap_addr);
	DO_GEN_PHY_ADDR(mmap_file);
	DO_GEN_PHY_ADDR(file_mprotect);
	DO_GEN_PHY_ADDR(file_lock);
	DO_GEN_PHY_ADDR(file_fcntl);
	DO_GEN_PHY_ADDR(file_set_fowner);
	DO_GEN_PHY_ADDR(file_send_sigiotask);
	DO_GEN_PHY_ADDR(file_receive);
	DO_GEN_PHY_ADDR(file_open);
	DO_GEN_PHY_ADDR(task_free);
	DO_GEN_PHY_ADDR(cred_alloc_blank);
	DO_GEN_PHY_ADDR(cred_free);
	DO_GEN_PHY_ADDR(cred_prepare);
	DO_GEN_PHY_ADDR(cred_transfer);
	DO_GEN_PHY_ADDR(kernel_act_as);
	DO_GEN_PHY_ADDR(kernel_create_files_as);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	DO_GEN_PHY_ADDR(kernel_fw_from_file);
	DO_GEN_PHY_ADDR(kernel_module_request);
	DO_GEN_PHY_ADDR(kernel_module_from_file);
#endif /* KERNEL_VERSION(4, 9, 0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
	DO_GEN_PHY_ADDR(task_alloc);
	DO_GEN_PHY_ADDR(task_prlimit);
#ifdef CONFIG_SECURITY_INFINIBAND
	DO_GEN_PHY_ADDR(ib_pkey_access);
	DO_GEN_PHY_ADDR(ib_endport_manage_subnet);
	DO_GEN_PHY_ADDR(ib_alloc_security);
	DO_GEN_PHY_ADDR(ib_free_security);
#endif /* CONFIG_SECURITY_INFINIBAND */

#else
	DO_GEN_PHY_ADDR(bprm_secureexec);
	DO_GEN_PHY_ADDR(task_create);
	DO_GEN_PHY_ADDR(task_wait);
#endif

	DO_GEN_PHY_ADDR(task_fix_setuid);
	DO_GEN_PHY_ADDR(task_setpgid);
	DO_GEN_PHY_ADDR(task_getpgid);
	DO_GEN_PHY_ADDR(task_getsid);
	DO_GEN_PHY_ADDR(task_getsecid);
	DO_GEN_PHY_ADDR(task_setnice);
	DO_GEN_PHY_ADDR(task_setioprio);
	DO_GEN_PHY_ADDR(task_getioprio);
	DO_GEN_PHY_ADDR(task_setrlimit);
	DO_GEN_PHY_ADDR(task_setscheduler);
	DO_GEN_PHY_ADDR(task_getscheduler);
	DO_GEN_PHY_ADDR(task_movememory);
	DO_GEN_PHY_ADDR(task_kill);
	DO_GEN_PHY_ADDR(task_prctl);
	DO_GEN_PHY_ADDR(task_to_inode);
	DO_GEN_PHY_ADDR(ipc_permission);
	DO_GEN_PHY_ADDR(ipc_getsecid);
	DO_GEN_PHY_ADDR(msg_msg_alloc_security);
	DO_GEN_PHY_ADDR(msg_msg_free_security);
	DO_GEN_PHY_ADDR(msg_queue_alloc_security);
	DO_GEN_PHY_ADDR(msg_queue_free_security);
	DO_GEN_PHY_ADDR(msg_queue_associate);
	DO_GEN_PHY_ADDR(msg_queue_msgctl);
	DO_GEN_PHY_ADDR(msg_queue_msgsnd);
	DO_GEN_PHY_ADDR(msg_queue_msgrcv);
	DO_GEN_PHY_ADDR(shm_alloc_security);
	DO_GEN_PHY_ADDR(shm_free_security);
	DO_GEN_PHY_ADDR(shm_associate);
	DO_GEN_PHY_ADDR(shm_shmctl);
	DO_GEN_PHY_ADDR(shm_shmat);
	DO_GEN_PHY_ADDR(sem_alloc_security);
	DO_GEN_PHY_ADDR(sem_free_security);
	DO_GEN_PHY_ADDR(sem_associate);
	DO_GEN_PHY_ADDR(sem_semctl);
	DO_GEN_PHY_ADDR(sem_semop);
	DO_GEN_PHY_ADDR(netlink_send);
	DO_GEN_PHY_ADDR(d_instantiate);
	DO_GEN_PHY_ADDR(getprocattr);
	DO_GEN_PHY_ADDR(setprocattr);
	DO_GEN_PHY_ADDR(ismaclabel);
	DO_GEN_PHY_ADDR(secid_to_secctx);
	DO_GEN_PHY_ADDR(secctx_to_secid);
	DO_GEN_PHY_ADDR(release_secctx);
	DO_GEN_PHY_ADDR(inode_notifysecctx);
	DO_GEN_PHY_ADDR(inode_setsecctx);
	DO_GEN_PHY_ADDR(inode_getsecctx);
#ifdef CONFIG_SECURITY_NETWORK
	DO_GEN_PHY_ADDR(unix_stream_connect);
	DO_GEN_PHY_ADDR(unix_may_send);
	DO_GEN_PHY_ADDR(socket_create);
	DO_GEN_PHY_ADDR(socket_post_create);
	DO_GEN_PHY_ADDR(socket_bind);
	DO_GEN_PHY_ADDR(socket_connect);
	DO_GEN_PHY_ADDR(socket_listen);
	DO_GEN_PHY_ADDR(socket_accept);
	DO_GEN_PHY_ADDR(socket_sendmsg);
	DO_GEN_PHY_ADDR(socket_recvmsg);
	DO_GEN_PHY_ADDR(socket_getsockname);
	DO_GEN_PHY_ADDR(socket_getpeername);
	DO_GEN_PHY_ADDR(socket_getsockopt);
	DO_GEN_PHY_ADDR(socket_setsockopt);
	DO_GEN_PHY_ADDR(socket_shutdown);
	DO_GEN_PHY_ADDR(socket_sock_rcv_skb);
	DO_GEN_PHY_ADDR(socket_getpeersec_stream);
	DO_GEN_PHY_ADDR(socket_getpeersec_dgram);
	DO_GEN_PHY_ADDR(sk_alloc_security);
	DO_GEN_PHY_ADDR(sk_free_security);
	DO_GEN_PHY_ADDR(sk_clone_security);
	DO_GEN_PHY_ADDR(sk_getsecid);
	DO_GEN_PHY_ADDR(sock_graft);
	DO_GEN_PHY_ADDR(inet_conn_request);
	DO_GEN_PHY_ADDR(inet_csk_clone);
	DO_GEN_PHY_ADDR(inet_conn_established);
	DO_GEN_PHY_ADDR(secmark_relabel_packet);
	DO_GEN_PHY_ADDR(secmark_refcount_inc);
	DO_GEN_PHY_ADDR(secmark_refcount_dec);
	DO_GEN_PHY_ADDR(req_classify_flow);
	DO_GEN_PHY_ADDR(tun_dev_alloc_security);
	DO_GEN_PHY_ADDR(tun_dev_free_security);
	DO_GEN_PHY_ADDR(tun_dev_create);
	DO_GEN_PHY_ADDR(tun_dev_attach_queue);
	DO_GEN_PHY_ADDR(tun_dev_attach);
	DO_GEN_PHY_ADDR(tun_dev_open);
#endif /* CONFIG_SECURITY_NETWORK */
#ifdef CONFIG_SECURITY_NETWORK_XFRM
	DO_GEN_PHY_ADDR(xfrm_policy_alloc_security);
	DO_GEN_PHY_ADDR(xfrm_policy_clone_security);
	DO_GEN_PHY_ADDR(xfrm_policy_free_security);
	DO_GEN_PHY_ADDR(xfrm_policy_delete_security);
	DO_GEN_PHY_ADDR(xfrm_state_alloc);
	DO_GEN_PHY_ADDR(xfrm_state_alloc_acquire);
	DO_GEN_PHY_ADDR(xfrm_state_free_security);
	DO_GEN_PHY_ADDR(xfrm_state_delete_security);
	DO_GEN_PHY_ADDR(xfrm_policy_lookup);
	DO_GEN_PHY_ADDR(xfrm_state_pol_flow_match);
	DO_GEN_PHY_ADDR(xfrm_decode_session);
#endif /* CONFIG_SECURITY_NETWORK_XFRM */
#ifdef CONFIG_KEYS
	DO_GEN_PHY_ADDR(key_alloc);
	DO_GEN_PHY_ADDR(key_free);
	DO_GEN_PHY_ADDR(key_permission);
	DO_GEN_PHY_ADDR(key_getsecurity);
#endif /* CONFIG_KEYS */
#ifdef CONFIG_AUDIT
	DO_GEN_PHY_ADDR(audit_rule_init);
	DO_GEN_PHY_ADDR(audit_rule_known);
	DO_GEN_PHY_ADDR(audit_rule_match);
	DO_GEN_PHY_ADDR(audit_rule_free);
#endif /* CONFIG_AUDIT */

	/* -1 is because first element is reserved for number of SE hooks */
	phy_addreses[0] = count - 1;
	return TEEC_SUCCESS;
}

/* This assumes that session is already open and operation is valid */
static int prepare_physical_addr_to_ta(TEEC_Operation *operation)
{
	int s_ret;
	int result;
	phys_addr_t *ptr = (phys_addr_t *)g_mem.buffer;
	/*
	 * used_length is the number space used of g_mem.buffer except sehooks,
	 * if define CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API, it will add
	 * two which used for rodata
	 */
#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
	int used_length = OFFSET_SEHOOKS_ADDR_START + RODATA_OCCUPY_LENGTH +
			REAL_OFFSET_LENGTH;
#else
	int used_length = OFFSET_SEHOOKS_ADDR_START + REAL_OFFSET_LENGTH;
#endif

	if (operation == NULL) {
		antiroot_error("prepare physical addr to ta operation error\n");
		return TEEC_ERROR_GENERIC;
	}
	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0, g_mem.size);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	ptr[OFFSET_KCODE_ADDR_START] = __pa_symbol(_stext);
	ptr[OFFSET_KCODE_ADDR_END] = __pa_symbol(_etext);
	ptr[OFFSET_SYSCALL_ADDR_START] =
		(phys_addr_t)__pa_symbol(((void *)sys_call_table));
	ptr[OFFSET_SYSCALL_NUM] = NR_syscalls * sizeof(void *);
	result = get_sescan_address(ptr + OFFSET_SEHOOKS_NUM, used_length);
	if (result != TEEC_SUCCESS) {
		antiroot_error("get sescan address error\n");
		return TEEC_ERROR_GENERIC;
	}
	g_se_hooks_num = ptr[OFFSET_SEHOOKS_NUM];
	/*
	 * if defined CONFIG_DEBUG_RODATA in 4.9 or defined
	 * CONFIG_STRICT_KERNEL_RWX in 4.14, we use __init_begin
	 * than __end_data_ro_after_init to cover EXCEPTION_TABLE.
	 */
#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
	ptr[OFFSET_RODATA_ADDR_START] =
		__pa_symbol((uintptr_t)__start_data_ro_after_init);
#if defined(CONFIG_DEBUG_RODATA) || defined(CONFIG_STRICT_KERNEL_RWX)
	ptr[OFFSET_RODATA_ADDR_END] = __pa_symbol((uintptr_t)__init_begin);
#else
	ptr[OFFSET_RODATA_ADDR_END] =
		__pa_symbol((uintptr_t)__end_data_ro_after_init);
#endif
#endif
	(*operation).started = 1;
	(*operation).paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
		TEEC_VALUE_OUTPUT, TEEC_NONE, TEEC_NONE);
	(*operation).params[0].tmpref.buffer = g_mem.buffer;
	(*operation).params[0].tmpref.size = g_mem.size;
	return TEEC_SUCCESS;
}

static int cmd_measurement(enum root_agent_cmd cmd)
{
	int result;
	int s_ret;
	TEEC_Operation operation;

	if (not_init_agent()) {
		antiroot_error("Agent should be initialized first!\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0, g_mem.size);
	if (s_ret != EOK)
		return s_ret;

	s_ret = memset_s(&operation, sizeof(TEEC_Operation),
			0, sizeof(TEEC_Operation));
	if (s_ret != EOK) {
		antiroot_error("memset_s failed.\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	operation.started = 1;
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
						TEEC_VALUE_OUTPUT,
						TEEC_NONE, TEEC_NONE);
	operation.params[0].tmpref.buffer = g_mem.buffer;
	operation.params[0].tmpref.size = g_mem.size;

	result = TEEK_InvokeCommand(&g_session, cmd, &operation, NULL);
	if (result != TEEC_SUCCESS)
		return TEEC_ERROR_GENERIC;

	g_tee_scan_status = operation.params[1].value.b;
	if (operation.params[1].value.a != REV_NOT_ROOT) {
		g_root_flag = operation.params[1].value.a;
		return TEE_ERROR_ANTIROOT_RSP_FAIL;
	}

	return result;
}

/*
 * Send physical address range of kernel, sys_call
 * table and SE hooks to task_antiroot
 */
static TEEC_Result send_phyaddr(TEEC_Operation *operations, uint32_t *origin)
{
	TEEC_Result teec_ret;

	teec_ret = prepare_physical_addr_to_ta(operations);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("prepare physical addr to ta failed\n");
		return teec_ret;
	}

	teec_ret = TEEK_InvokeCommand(&g_session, ROOTAGENT_KERNEL_ADDR_ID,
				operations, origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("Setting kernel physical address range failed\n");
		return teec_ret;
	}
	g_tee_scan_status = operations->params[1].value.b;
	if (operations->params[1].value.a != REV_NOT_ROOT) {
		g_root_flag = operations->params[1].value.a;
		antiroot_debug(ROOTAGENT_DEBUG_ERROR,
			"Setting kernel physical address range failed due to Rooted\n");
		return TEE_ERROR_ANTIROOT_RSP_FAIL; /* lint !e570 */
	}

	return teec_ret;
}

/* inform TA to pause measurement */
int pause_measurement(void)
{
	int ret = cmd_measurement(ROOTAGENT_PAUSE_ID);

	if (ret != TEEC_SUCCESS)
		antiroot_error("Pause measurement failed, ret = %d\n", ret);

	return ret;
}

/* resend kernel txt physical address to TA */
int resume_measurement(void)
{
	int ret = cmd_measurement(ROOTAGENT_RESUME_ID);

	if (ret != TEEC_SUCCESS)
		antiroot_error("Resume measurement failed, ret = %d\n", ret);

	return ret;
}

#ifdef CONFIG_TEE_ANTIROOT_CLIENT_ENG_DEBUG
int tee_status_check(int cmd_item)
{
	int result;
	int s_ret;
	TEEC_Operation operation;
	int *test_cmd = (int *)g_mem.buffer;

	if (not_init_agent()) {
		antiroot_error("Agent should be initialized first!\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0, g_mem.size);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	s_ret = memset_s(&operation, sizeof(TEEC_Operation), 0,
			sizeof(TEEC_Operation));
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	test_cmd[0] = cmd_item;
	operation.started = 1;
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
						TEEC_VALUE_OUTPUT,
						TEEC_NONE, TEEC_NONE);
	operation.params[0].tmpref.buffer = g_mem.buffer;
	operation.params[0].tmpref.size = g_mem.size;

	result = TEEK_InvokeCommand(&g_session, TEE_STATUS_TEST,
				&operation, NULL);
	if (result != TEEC_SUCCESS) {
		antiroot_error("tee status check failed\n");
		return TEEC_ERROR_GENERIC;
	}
	antiroot_debug(ROOTAGENT_DEBUG_AGENT, "tee status check ok\n");
	return result;
}
#endif
#endif

static inline bool right_proc_len(const struct ragent_rootproc *r_proc)
{
	return ((r_proc->length > 0) &&
		(r_proc->length < ROOTAGENT_RPROCS_MAX_LENGTH) &&
		(r_proc->procid != NULL));
}

#ifndef CONFIG_TEE_KERNEL_MEASUREMENT_API
static int config_measurement_api(const struct ragent_config *cfg,
				struct ragent_command *rm_cmd)
{
	int s_ret;

	s_ret = memcpy_s(rm_cmd->content.config.white_list.kcodes,
			KERNEL_CODE_LENGTH_SHA, cfg->white_list.kcodes,
			KERNEL_CODE_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;

	s_ret = memcpy_s(rm_cmd->content.config.white_list.syscalls,
			SYSTEM_CALL_LENGTH_SHA, cfg->white_list.syscalls,
			SYSTEM_CALL_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;

	s_ret = memcpy_s(rm_cmd->content.config.white_list.sehooks,
			SELINUX_HOOKS_LENGTH_SHA, cfg->white_list.sehooks,
			SELINUX_HOOKS_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;

	return s_ret;
end:
	antiroot_error("memcpy_s failed\n");
	return s_ret;
}
#endif

/*
 * set the aes key cipher of g_chal_req_key and
 * g_chal_key, according to cfg parameter.
 */
static int set_aes_key_cipher(const struct ragent_config *cfg,
			struct ragent_command *rm_cmd)
{
	int s_ret;

	s_ret = memcpy_s(g_chal_req_key, CHALLENGE_REQ_KEY_LENGTH,
			cfg->cipher_key.cha_req_key, CHALLENGE_REQ_KEY_LENGTH);
	if (s_ret != EOK)
		goto end;
	s_ret = memcpy_s(rm_cmd->content.config.cipher_key.cha_req_key,
			CHALLENGE_REQ_KEY_LENGTH, cfg->cipher_key.cha_req_key,
			CHALLENGE_REQ_KEY_LENGTH);
	if (s_ret != EOK)
		goto end;

	s_ret = memcpy_s(g_chal_key, CHALLENGE_KEY_LENGTH,
			cfg->cipher_key.cha_key, CHALLENGE_KEY_LENGTH);
	if (s_ret != EOK)
		goto end;
	s_ret = memcpy_s(rm_cmd->content.config.cipher_key.cha_key,
			CHALLENGE_KEY_LENGTH, cfg->cipher_key.cha_key,
			CHALLENGE_KEY_LENGTH);
	if (s_ret != EOK)
		goto end;

	return s_ret;
end:
	antiroot_error("memcpy_s failed\n");
	return s_ret;
}

static int setting_ragent_config(TEEC_Operation *operation,
				const struct ragent_config *cfg,
				struct ragent_command **rm_cmd,
				const struct ragent_rootproc *r_proc)
{
	int s_ret;

	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0, g_mem.size);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return s_ret;
	}
	s_ret = memset_s(operation, sizeof(TEEC_Operation), 0,
			sizeof(TEEC_Operation));
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return s_ret;
	}

	operation->started = 1;
	operation->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
						TEEC_VALUE_OUTPUT, TEEC_NONE,
						TEEC_NONE);
	operation->params[0].tmpref.buffer = g_mem.buffer;
	operation->params[0].tmpref.size = g_mem.size;
	*rm_cmd = (struct ragent_command *)g_mem.buffer;
	(*rm_cmd)->magic = MAGIC;
	(*rm_cmd)->version = VERSION;
	(*rm_cmd)->interface = ROOTAGENT_CONFIG_ID;
	(*rm_cmd)->content.config.white_list.dstatus = cfg->white_list.dstatus;
	(*rm_cmd)->content.config.white_list.selinux = cfg->white_list.selinux;
	(*rm_cmd)->content.config.white_list.proclength = r_proc->length;
	(*rm_cmd)->content.config.white_list.setid = 0;
	s_ret = set_aes_key_cipher(cfg, *rm_cmd);

	return s_ret;
}

static TEEC_Result setting_proc_length(const struct ragent_command *rm_command,
				const struct ragent_rootproc *r_proc)
{
	int s_ret;

	antiroot_debug(ROOTAGENT_DEBUG_API, "setting_config------proclength: %u\n",
			rm_command->content.config.white_list.proclength);
	if (right_proc_len(r_proc)) {
		s_ret = memcpy_s(g_mem.buffer + sizeof(struct ragent_command),
				RM_PRE_ALLOCATE_SIZE - sizeof(struct ragent_command),
				r_proc->procid, r_proc->length);
		if (s_ret != EOK) {
			antiroot_error("memcpy_s failed\n");
			return TEEC_ERROR_GENERIC; /* lint !e570 */
		}
	} else {
		antiroot_error("root_proc is NULL!\n");
		return TEEC_ERROR_OUT_OF_MEMORY; /* lint !e570 */
	}

	return TEEC_SUCCESS;
}

TEEC_Result setting_config(const struct ragent_config *config,
			const struct ragent_rootproc *root_proc)
{
	uint32_t origin;
	TEEC_Result teec_ret;
	TEEC_Operation operation;
	struct ragent_command *rm_command = NULL;
	int s_ret;

	if (not_init_agent()) {
		antiroot_error("Agent should be initialized first!\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	/* judge the input parameter */
	if ((config == NULL) || (root_proc == NULL) ||
			(root_proc->procid == NULL)) {
		antiroot_error("Bad params!\n");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	s_ret = setting_ragent_config(&operation, config,
				&rm_command, root_proc);
	if (s_ret != EOK)
		return TEEC_ERROR_GENERIC; /* lint !e570 */

#ifndef CONFIG_TEE_KERNEL_MEASUREMENT_API
	s_ret = config_measurement_api(config, rm_command);
	if (s_ret != EOK)
		return TEEC_ERROR_GENERIC; /* lint !e570 */
#endif
	teec_ret = setting_proc_length(rm_command, root_proc);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("Setting_config proclength failed\n");
		return teec_ret;
	}
	teec_ret = TEEK_InvokeCommand(&g_session, ROOTAGENT_CONFIG_ID,
				&operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("Setting_config failed\n");
		return teec_ret;
	}
	g_tee_scan_status = operation.params[1].value.b;

	/*
	 * If in user version TA read rootstatus is already rooted,
	 * then TA will NOT setTrustlist, and return TEEC_SUCCESS directly,
	 * with value.a = REV_ROOTED, value is TEE_ERROR_ANTIROOT_RSP_FAIL.
	 * The setting config and tee init will fail.
	 * But if in eng wersion, the value.a will set REV_NOT_ROOT,
	 * so can continue test in fastboot unlock status.
	 */
	if (operation.params[1].value.a != REV_NOT_ROOT) {
		antiroot_debug(ROOTAGENT_DEBUG_ERROR,
				"Setting_config failed due to Rooted\n");
		return TEE_ERROR_ANTIROOT_RSP_FAIL; /* lint !e570 */
	}
#ifdef CONFIG_TEE_KERNEL_MEASUREMENT_API
	teec_ret = send_phyaddr(&operation, &origin);
	if (teec_ret != TEEC_SUCCESS)
		antiroot_error("send_phyaddr failed\n");
#endif
	return teec_ret;
}

static int setting_operation_config(TEEC_Operation *s_opt)
{
	int s_ret;

	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0, g_mem.size);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return s_ret;
	}
	s_ret = memset_s(g_swap_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0,
			g_swap_mem.size);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return s_ret;
	}

	s_ret = memset_s(s_opt, sizeof(TEEC_Operation), 0,
			sizeof(TEEC_Operation));
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return s_ret;
	}
	s_opt->started = 1;
	s_opt->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
			TEEC_VALUE_OUTPUT, TEEC_NONE, TEEC_NONE);
	s_opt->params[0].tmpref.buffer = g_mem.buffer;
	s_opt->params[0].tmpref.size = g_mem.size;

	return s_ret;
}

static TEEC_Result teec_req_challenge(const char *rm_command_out_in,
				char *rm_command_in_out,
				const struct ragent_command *rm_command,
				struct ragent_challenge *ichallenge)
{
	int ret;
	int s_ret;

	ret = do_aes256_cbc((u8 *)rm_command_in_out,
			(const u8 *)rm_command_out_in,
			g_mem.buffer, g_mem.size, g_chal_key,
			CHALLENGE_KEY_LENGTH, ANTIROOT_SRC_LEN, DECRYPT);
	if (ret != 0) {
		antiroot_error("do_aes256_cbc failed, ret = %d\n", ret);
		return TEEC_ERROR_GENERIC;
	}
	s_ret = memcpy_s(g_nounce, CHALLENGE_NOUNCE_LENGTH,
			rm_command->content.challenge.nounce,
			CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	s_ret = memcpy_s(ichallenge->challengeid,
			CHALLENGE_MAX_LENGTH * sizeof(uint32_t),
			rm_command->content.challenge.challengeid,
			CHALLENGE_MAX_LENGTH * sizeof(uint32_t));
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	return TEEC_SUCCESS;
}

static TEEC_Result challenge_proc(char *rm_command_out_in,
				char *rm_command_in_out,
				struct ragent_command *rm_command,
				TEEC_Operation *operation,
				struct ragent_challenge *ichallenge)
{
	int result;
	int s_ret;
	uint32_t origin;
	TEEC_Result teec_ret;

	s_ret = memset_s(g_nounce, CHALLENGE_NOUNCE_LENGTH, 0,
			CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	result = do_aes256_cbc((u8 *)rm_command_out_in,
			(u8 *)rm_command_in_out,
			g_mem.buffer, g_mem.size, g_chal_req_key,
			CHALLENGE_REQ_KEY_LENGTH, ANTIROOT_SRC_LEN, ENCRYPT);
	if (result) {
		antiroot_error("do_aes256_cbc failed, result = %d\n", result);
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	teec_ret = TEEK_InvokeCommand(&g_session, ROOTAGENT_CHALLENGE_ID,
				operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("Request_challenge failed!\n");
	} else if (operation->params[1].value.a != REV_NOT_ROOT) {
		antiroot_debug(ROOTAGENT_DEBUG_ERROR,
				"Request_challenge failed due to Rooted!\n");
		g_root_flag = operation->params[1].value.a;
		g_tee_scan_status = operation->params[1].value.b;
		teec_ret = teec_req_challenge(rm_command_out_in,
				rm_command_in_out, rm_command, ichallenge);
		if (teec_ret == TEEC_SUCCESS)
			antiroot_debug(ROOTAGENT_DEBUG_API,
				"Request_challenge successful when Rooted\n");
	} else {
		g_tee_scan_status = operation->params[1].value.b;
		teec_ret = teec_req_challenge(rm_command_out_in,
				rm_command_in_out, rm_command, ichallenge);
		if (teec_ret == TEEC_SUCCESS)
			antiroot_debug(ROOTAGENT_DEBUG_API,
				"Request_challenge successful\n");
	}
	return teec_ret;
}

TEEC_Result request_challenge(struct ragent_challenge *ichallenge)
{
	TEEC_Result teec_ret;
	TEEC_Operation operation;
	char *rm_command_out_in = NULL;
	char *rm_command_in_out = NULL;
	struct ragent_command *rm_command = NULL;
	int s_ret;

	if (not_init_agent()) {
		antiroot_error("Agent should be initialized first!\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	if (ichallenge == NULL) {
		antiroot_error("Bad params!\n");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	s_ret = setting_operation_config(&operation);
	if (s_ret != EOK)
		return TEEC_ERROR_GENERIC; /* lint !e570 */

	get_random_bytes(g_mem.buffer, IV_SIZE);
	rm_command_out_in = (char *)(g_mem.buffer + IV_SIZE);
	rm_command_in_out = (char *)g_swap_mem.buffer;
	if (rm_command_in_out == NULL) {
		antiroot_error("malloc failed!\n");
		return TEEC_ERROR_OUT_OF_MEMORY; /*lint !e570*/
	}
	rm_command = (struct ragent_command *)rm_command_in_out;
	rm_command->magic = MAGIC;
	rm_command->version = VERSION;
	rm_command->interface = ROOTAGENT_CHALLENGE_ID;
	rm_command->content.challenge.cpuload = ichallenge->cpuload;
	rm_command->content.challenge.battery = ichallenge->battery;
	rm_command->content.challenge.charging = ichallenge->charging;
	rm_command->content.challenge.time = ichallenge->time;
	rm_command->content.challenge.timezone = ichallenge->timezone;
	s_ret = memset_s(rm_command->content.challenge.nounce,
			CHALLENGE_NOUNCE_LENGTH, 0, CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	teec_ret = challenge_proc(rm_command_out_in, rm_command_in_out,
				rm_command, &operation, ichallenge);

	return teec_ret;
}

#ifndef CONFIG_TEE_KERNEL_MEASUREMENT_API
static int back_response(struct ragent_response *dst_rsp,
			const struct ragent_response *src_rsp)
{
	int s_ret;

	s_ret = memcpy_s(dst_rsp->runtime_white_list.kcodes,
			KERNEL_CODE_LENGTH_SHA,
			src_rsp->runtime_white_list.kcodes,
			KERNEL_CODE_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;
	s_ret = memcpy_s(dst_rsp->runtime_white_list.syscalls,
			SYSTEM_CALL_LENGTH_SHA,
			src_rsp->runtime_white_list.syscalls,
			SYSTEM_CALL_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;
	s_ret = memcpy_s(dst_rsp->runtime_white_list.sehooks,
			SELINUX_HOOKS_LENGTH_SHA,
			src_rsp->runtime_white_list.sehooks,
			SELINUX_HOOKS_LENGTH_SHA);
	if (s_ret != EOK)
		goto end;

	return s_ret;
end:
	antiroot_error("memcpy_s failed\n");
	return s_ret;
}
#endif

static TEEC_Result rootagent_response(const struct ragent_rootproc *root_proc,
				TEEC_Operation *operation,
				struct ragent_command *rm_command,
				u8 *rm_command_out,
				const u8 *rm_command_in)
{
	int size;
	int ret;
	int s_ret;
	uint32_t origin;
	TEEC_Result teec_ret;

	if (root_proc->procid && (root_proc->length > 0) &&
		(root_proc->length < ROOTAGENT_RPROCS_MAX_LENGTH)) {
		size = sizeof(struct ragent_command) + root_proc->length;
		if (size > ANTIROOT_SRC_LEN) {
			antiroot_error("response is oom!\n");
			return TEEC_ERROR_OUT_OF_MEMORY; /* lint !e570 */
		}

		s_ret = memcpy_s((char *)rm_command + sizeof(struct ragent_command),
				RM_PRE_ALLOCATE_SIZE - sizeof(struct ragent_command),
				root_proc->procid, root_proc->length);
		if (s_ret != EOK) {
			antiroot_error("memcpy_s failed\n");
			return TEEC_ERROR_GENERIC; /* lint !e570 */
		}
	}
	ret = do_aes256_cbc(rm_command_out, rm_command_in,
			g_mem.buffer, g_mem.size,
			g_nounce, CHALLENGE_NOUNCE_LENGTH,
			ANTIROOT_SRC_LEN, ENCRYPT);
	if (ret) {
		antiroot_error("do_aes256_cbc failed, ret = %d\n", ret);
		return TEEC_ERROR_GENERIC;
	}
	teec_ret = TEEK_InvokeCommand(&g_session,
				ROOTAGENT_RESPONSE_ID, operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("Feedback_response failed result = 0x%x\n",
				teec_ret);
	} else if (operation->params[1].value.a != REV_NOT_ROOT) {
		antiroot_debug(ROOTAGENT_DEBUG_ERROR,
				"feedback_response failed due to Rooted!\n");
		g_root_flag = operation->params[1].value.a;
		g_tee_scan_status = operation->params[1].value.b;
	} else {
		g_tee_scan_status = operation->params[1].value.b;
	}

	return teec_ret;
}

TEEC_Result feedback_response(const struct ragent_response *response,
			const struct ragent_rootproc *root_proc)
{
	TEEC_Result teec_ret;
	TEEC_Operation operation;
	void *rm_command_out = NULL;
	void *rm_command_in = NULL;
	struct ragent_command *rm_command = NULL;
	struct ragent_response *rsp = NULL;
	int s_ret;

	if (not_init_agent()) {
		antiroot_error("Agent should be initialized first!\n");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}

	if ((response == NULL) || (root_proc == NULL)) {
		antiroot_error("Bad params!\n");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	s_ret = setting_operation_config(&operation);
	if (s_ret != EOK)
		return TEEC_ERROR_GENERIC; /* lint !e570 */

	get_random_bytes(g_mem.buffer, IV_SIZE);
	rm_command_out = (void *)(g_mem.buffer + IV_SIZE);
	rm_command_in = (void *)g_swap_mem.buffer;
	if (rm_command_in == NULL) {
		antiroot_error("response kmalloc failed!\n");
		return TEEC_ERROR_OUT_OF_MEMORY; /* lint !e570 */
	}
	rm_command = rm_command_in;
	rsp = &(rm_command->content.response);
	rm_command->magic = MAGIC;
	rm_command->version = VERSION;
	rm_command->interface = ROOTAGENT_RESPONSE_ID;
	rsp->proc_integrated = response->proc_integrated;
	rsp->noop = response->noop;
	rsp->runtime_white_list.selinux = response->runtime_white_list.selinux;
	rsp->runtime_white_list.proclength = root_proc->length;
	rsp->runtime_white_list.setid = response->runtime_white_list.setid;

#ifndef CONFIG_TEE_KERNEL_MEASUREMENT_API
	s_ret = back_response(rsp, response);
	if (s_ret != EOK)
		return TEEC_ERROR_GENERIC; /* lint !e570 */
#endif
	teec_ret = rootagent_response(root_proc, &operation, rm_command,
				(u8 *)rm_command_out, (u8 *)rm_command_in);
	return teec_ret;
}

#ifdef CONFIG_HW_ROOT_SCAN_RODATA_MEASUREMENT_API
void rodata_stp_upload(void)
{
	int s_ret;
	struct stp_item item;

	antiroot_info("tee_scan_status is 0x%x", g_tee_scan_status);
	item.status = (g_tee_scan_status >> RODATABIT) & 0x1;
	if (item.status == 0) {
		antiroot_info("rodata is normal, no need to upload");
		return;
	}

	item.id = STP_ID_RODATA;
	item.credible = STP_REFERENCE;
	item.version = 0;

	if (strlen(STP_NAME_RODATA) >= STP_ITEM_NAME_LEN) {
		antiroot_error("the name of rodata exceeded the max allowed value");
		return;
	}
	s_ret = strcpy_s(item.name, STP_ITEM_NAME_LEN, STP_NAME_RODATA);
	if (s_ret != EOK) {
		antiroot_error("rodata_stp_upload strcpy_s error, s_ret = %d",
			s_ret);
		return;
	}

	s_ret = kernel_stp_upload(item, NULL);
	if (s_ret != 0)
		antiroot_debug(ROOTAGENT_DEBUG_API, "rodata stp upload failed");
}
#endif

static TEEC_Result eima_meminit(TEEC_Operation *operation)
{
	int s_ret;

	if (not_init_agent()) {
		antiroot_error("eima Agent should be initialized first!");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	if (g_mem.size != RM_PRE_ALLOCATE_SIZE)
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */

	s_ret = memset_s(g_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0,
			RM_PRE_ALLOCATE_SIZE);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	if (g_swap_mem.size != RM_PRE_ALLOCATE_SIZE)
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */

	s_ret = memset_s(g_swap_mem.buffer, RM_PRE_ALLOCATE_SIZE, 0x0,
			RM_PRE_ALLOCATE_SIZE);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	s_ret = memset_s(operation, sizeof(TEEC_Operation), 0,
			sizeof(TEEC_Operation));
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}
	return TEEC_SUCCESS;
}

static void set_operation_val(TEEC_Operation *operation)
{
	operation->started = 1;
	operation->paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
		TEEC_VALUE_OUTPUT, TEEC_NONE, TEEC_NONE);
	operation->params[0].tmpref.buffer = g_mem.buffer;
	operation->params[0].tmpref.size = g_mem.size;
}

/*
 * The parameter nounce is the first address of an array,
 * and the length of the array is CHALLENGE_NOUNCE_LENGTH.
 */
static TEEC_Result eima_challenge(char *cm_in, const char *cm_out,
				const uint8_t *nounce, int nounce_len)
{
	int ret;
	int s_ret;

	ret = do_aes256_cbc((u8 *)cm_in, (const u8 *)cm_out,
			g_mem.buffer, g_mem.size,
			nounce, nounce_len,
			ANTIROOT_SRC_LEN, DECRYPT);
	if (ret != 0) {
		antiroot_error("do_aes256_cbc failed, ret = %d!", ret);
		return TEEC_ERROR_GENERIC;
	}

	s_ret = memset_s(g_eima_nounce, CHALLENGE_NOUNCE_LENGTH, 0x0,
			CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memset_s failed\n");
		return TEEC_ERROR_SECURITY;
	}
	s_ret = memcpy_s(g_eima_nounce, CHALLENGE_NOUNCE_LENGTH,
			((struct ragent_command *)cm_in)->content.eima_challenge.nounce,
			CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_GENERIC;
	}

	return TEEC_SUCCESS;
}

TEEC_Result eima_request_challenge(void)
{
	uint32_t origin;
	int ret;
	int s_ret;
	TEEC_Result teec_ret;
	TEEC_Operation operation;
	char *cm_in = NULL; /* ca */
	char *cm_out = NULL; /* ta */
	struct ragent_command *cm = NULL;
	uint8_t nounce[CHALLENGE_NOUNCE_LENGTH] = {0};

	ret = eima_meminit(&operation);
	if (ret != TEEC_SUCCESS) {
		antiroot_error("eima_meminit faild! ret = %d", ret);
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	set_operation_val(&operation);

	get_random_bytes(g_mem.buffer, IV_SIZE);
	cm_out = (char *)(g_mem.buffer + IV_SIZE);
	cm_in = (char *)(g_swap_mem.buffer);

	cm = (struct ragent_command *)cm_out;
	cm->magic = MAGIC;
	cm->version = VERSION;
	cm->interface = EIMA_CHALLENGE;

	get_random_bytes(nounce, CHALLENGE_NOUNCE_LENGTH);
	s_ret = memcpy_s(cm->content.eima_challenge.nounce,
			CHALLENGE_NOUNCE_LENGTH, nounce,
			CHALLENGE_NOUNCE_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	teec_ret = TEEK_InvokeCommand(&g_session, EIMA_CHALLENGE,
				&operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("eima_request_challenge failed!");
	} else if (operation.params[1].value.a != REV_NOT_ROOT) {
		antiroot_error("eima_request_challenge failed due to rooted!");
		g_root_flag = operation.params[1].value.a;
		g_tee_scan_status = operation.params[1].value.b;
		teec_ret = TEE_ERROR_ANTIROOT_RSP_FAIL; /* lint !e570 */
	} else {
		g_tee_scan_status = operation.params[1].value.b;
		teec_ret = eima_challenge(cm_in, cm_out, nounce,
					CHALLENGE_NOUNCE_LENGTH);
		if (teec_ret == TEEC_SUCCESS)
			antiroot_debug(ROOTAGENT_DEBUG_API,
				"eima_request_challenge successful");
	}
	return teec_ret;
}

static inline bool sanity_check_len(const struct m_entry *entry,
				uint16_t index, uint32_t total)
{
	uint16_t entry_total;

	entry_total = sizeof(struct m_entry) - sizeof(entry->fn) +
		entry->fn_len;
	return ((entry->fn_len >= sizeof(entry->fn)) ||
		(entry_total > (UINT16_MAX - index)) ||
		(total < (index + entry_total)));
}

static int parse_msgentry(char *buf, size_t buf_len,
			uint16_t *index, const struct m_entry *entry)
{
	errno_t s_ret;
	uint16_t tmp_index;
	uint16_t fn_len;

	if ((buf == NULL) || (index == NULL) || (entry == NULL) ||
		buf_len < sizeof(struct m_entry)) {
		antiroot_error("eima parse_msgentry bad params!");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	tmp_index = *index;
	/* judeg the length */
	if (sanity_check_len(entry, tmp_index, ANTIROOT_SRC_LEN)) {
		antiroot_error("parse_msgentry: length is error!");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	/* copy type of struct msglist */
	if ((entry->type != EIMA_DYNAMIC) && (entry->type != EIMA_STATIC)) {
		antiroot_error("parse_msgentry: failed! type is %u",
			entry->type);
		return TEEC_ERROR_BAD_PARAMETERS;
	}
	*(uint8_t *)(buf + tmp_index) = entry->type;
	tmp_index += sizeof(uint8_t);

	/* copy hash len of struct msglist */
	*(uint8_t *)(buf + tmp_index) = entry->hash_len;
	tmp_index += sizeof(uint8_t);

	/* copy hash  of struct msglist */
	s_ret = memcpy_s(buf + tmp_index, sizeof(entry->hash),
			entry->hash, sizeof(entry->hash));
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed.\n");
		return TEEC_ERROR_SECURITY;
	}
	tmp_index += sizeof(entry->hash);

	/* copy file_len len of struct msglist */
	fn_len = entry->fn_len;
	if ((fn_len == 0) || (fn_len > MAX_MSGLIST_FILE_LEN)) {
		antiroot_error("parse_msgentry: fn len error! fn_len = 0x%x",
			fn_len);
		return TEEC_ERROR_SECURITY;
	}
	*(uint16_t *)(buf + tmp_index) = fn_len;
	tmp_index += sizeof(uint16_t);

	/* copy file context of struct msglist */
	s_ret = memcpy_s(buf + tmp_index, MAX_MSGLIST_FILE_LEN,
			entry->fn, fn_len);
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_SECURITY;
	}
	tmp_index += fn_len;
	*index = tmp_index;
	return TEEC_SUCCESS;
}

static TEEC_Result eima_package_data(const struct m_list_msg *rsp, uint8_t *cm,
				size_t cm_len)
{
	TEEC_Result ret;
	uint32_t i;
	errno_t s_ret;
	uint16_t index = sizeof(struct ragent_command);

	if ((rsp == NULL) || (cm == NULL) || (cm_len < index)) {
		antiroot_error("eima package data bad params!");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	if ((rsp->num > EIMA_MAX_MEASURE_OBJ_CNT) || (rsp->num == 0)) {
		antiroot_error("eima package data bad param! num = %d",
				rsp->num);
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	s_ret = memcpy_s(cm + index, UNAME_LENTH,
			rsp->usecase, UNAME_LENTH - 1);
	if (s_ret != EOK) {
		antiroot_error("memcpy_s failed\n");
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	index += UNAME_LENTH;

	*((uint32_t *)(cm + index)) = rsp->num;

	index += sizeof(uint32_t);

	for (i = 0; i < rsp->num; i++) {
		ret = parse_msgentry(cm, cm_len - index, &index,
				rsp->m_list + i);
		if (ret != TEEC_SUCCESS) {
			antiroot_error("eima package data parse the data error, ret = 0x%x", ret);
			return ret;
		}
	}
	return TEEC_SUCCESS;
}

static TEEC_Result send_eima_response_cmd(TEEC_Operation *operation)
{
	TEEC_Result teec_ret;
	uint32_t origin;
	errno_t s_ret;

	if (operation == NULL) {
		antiroot_error("eima respone Bad params!");
		return TEEC_ERROR_BAD_PARAMETERS;
	}

	teec_ret = TEEK_InvokeCommand(&g_session, EIMA_RESPONSE,
				operation, &origin);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("eima_response failed! result = 0x%x", teec_ret);
		return teec_ret;
	}

	s_ret = memcpy_s(g_eima_hash_error_file_path, FNAME_LENGTH,
		operation->params[0].tmpref.buffer, FNAME_LENGTH);
	if (s_ret != EOK) {
		antiroot_error("copy tampered file path returned from ta error!");
		return TEEC_ERROR_GENERIC;
	}
	g_eima_hash_error_file_path[FNAME_LENGTH - 1] = '\0';
	antiroot_debug(ROOTAGENT_DEBUG_ERROR, "tampered file path : %s",
		g_eima_hash_error_file_path);

	g_tee_scan_status = operation->params[1].value.b;
	if (operation->params[1].value.a != REV_NOT_ROOT) {
		antiroot_debug(ROOTAGENT_DEBUG_ERROR,
			"eima_response failed due to rooted!");
		g_root_flag = operation->params[1].value.a;
		g_tee_scan_status = operation->params[1].value.b;
		teec_ret = TEE_ERROR_ANTIROOT_RSP_FAIL; /* lint !e570 */
	}

	return teec_ret;
}

TEEC_Result eima_send_response(int type, const struct m_list_msg *response)
{
	TEEC_Result teec_ret;
	TEEC_Operation operation;
	void *cm_out = NULL;
	void *cm_in = NULL;
	struct ragent_command *cm = NULL;
	int ret;

	if (response == NULL) {
		antiroot_error("eima response Bad params!");
		return TEEC_ERROR_BAD_PARAMETERS; /* lint !e570 */
	}

	ret = eima_meminit(&operation);
	if (ret != TEEC_SUCCESS) {
		antiroot_error("eima_meminit faild! ret=%d", ret);
		return TEEC_ERROR_SECURITY; /* lint !e570 */
	}

	operation.started = 1;
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
						TEEC_VALUE_OUTPUT, TEEC_NONE,
						TEEC_NONE);
	operation.params[0].tmpref.buffer = g_mem.buffer;
	operation.params[0].tmpref.size = g_mem.size;

	get_random_bytes(g_mem.buffer, IV_SIZE);
	cm_out = (void *)(g_mem.buffer + IV_SIZE);
	cm_in = (void *)g_swap_mem.buffer;
	if (cm_in == NULL) {
		antiroot_error("eima response kmalloc failed!");
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	cm = cm_in;
	cm->magic = MAGIC;
	cm->version = VERSION;
	cm->interface = EIMA_RESPONSE;
	cm->content.eima_response.msg_type = type;
	teec_ret = eima_package_data(response, (uint8_t *)cm, g_swap_mem.size);
	if (teec_ret != TEEC_SUCCESS) {
		antiroot_error("eima_response failed!result = 0x%x!", teec_ret);
		return TEEC_ERROR_GENERIC; /* lint !e570 */
	}
	ret = do_aes256_cbc(cm_out, cm_in, g_mem.buffer, g_mem.size,
			g_eima_nounce, CHALLENGE_NOUNCE_LENGTH,
			ANTIROOT_SRC_LEN, ENCRYPT);

	if (ret) {
		antiroot_error("do_aes256_cbc failed,ret = %d", ret);
		return TEEC_ERROR_GENERIC;
	}
	teec_ret = send_eima_response_cmd(&operation);

	return teec_ret;
}
