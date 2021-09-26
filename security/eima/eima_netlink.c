/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2018-2021. All rights reserved.
 * Description: eima_netlink.c for eima netlink communication in kernel
 * Create: 2018-10-24
 */

#include "eima_netlink.h"

#include <linux/netlink.h>
#include <linux/socket.h>
#include <linux/types.h>
#include <linux/version.h>

#include <net/net_namespace.h>
#include <net/netlink.h>
#include <securec.h>

#include "dkm.h"
#include "eima_agent_api.h"
#include "eima_utils.h"

#include <linux/sched/task.h>

#define EIMA_HASH_LEN_ERR 1

static DEFINE_MUTEX(eima_nl_mutex);
static struct sock *g_eima_nl;

static int eima_nl_send_message(const u8 *message, int message_len, u32 pid)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int ret;

	skb = nlmsg_new(message_len, 0);
	if (skb == NULL) {
		eima_error("alloc hash [%d] memory for skb error", message_len);
		return -ENOMEM;
	}

	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, message_len, 0);
	if (nlh == NULL) {
		eima_error("fill nlmsghdr content error");
		nlmsg_free(skb);
		return -ENOMEM;
	}
	NETLINK_CB(skb).dst_group = 0;

	ret = memcpy_s(NLMSG_DATA(nlh), message_len, message, message_len);
	if (ret != EOK) {
		eima_error("memcpy_s failed, s_ret = %d", ret);
		nlmsg_free(skb);
		return ret;
	}
	eima_debug("kernel send message is : %pK", NLMSG_DATA(nlh));

	return nlmsg_unicast(g_eima_nl, skb, pid);
}

static int get_process_task(const struct nlmsghdr *nlh, struct task_struct **task)
{
	struct pid *pid = NULL;
	pid_t measure_pid;

	eima_debug("kernel begin to measure process");
	measure_pid = *(pid_t *)NLMSG_DATA(nlh);

	rcu_read_lock();
	pid = find_vpid(measure_pid);
	if (pid == NULL) {
		eima_warning("struct of pid [%d] get failed", measure_pid);
		rcu_read_unlock();
		return -ENOENT;
	}
	rcu_read_unlock();

	*task = get_pid_task(pid, PIDTYPE_PID);
	if (*task == NULL) {
		eima_warning("task of pid [%d] get failed", measure_pid);
		return -ENOENT;
	}

	return 0;
}

static int eima_nl_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh,
	struct netlink_ext_ack *eack __maybe_unused)
{
	struct task_struct *task = NULL;
	char message[sizeof(int) + EIMA_SHA384_DIGEST_SIZE] = {0};
	char hash[EIMA_SHA384_DIGEST_SIZE] = {0};
	int eima_hash_len = 0;
	int message_len = sizeof(int);
	int ret;

	ret = get_process_task(nlh, &task);
	if (ret != 0) {
		eima_error("eima get process task failed");
		goto end;
	}

	ret = measure_process(task, 0, hash, EIMA_SHA384_DIGEST_SIZE,
			&eima_hash_len);
	put_task_struct(task);
	if (ret != 0) {
		eima_error("eima measure process failed");
		goto end;
	}

	if (eima_hash_len != EIMA_SHA384_DIGEST_SIZE) {
		eima_error("eima measure hash length error");
		ret = EIMA_HASH_LEN_ERR;
		goto end;
	}
	message_len += eima_hash_len;
	ret = memcpy_s(message + sizeof(int), EIMA_SHA384_DIGEST_SIZE,
		hash, eima_hash_len);
	if (ret != EOK) {
		eima_error("memcpy_s failed, s_ret = %d", ret);
		return ret;
	}
end:
	*(int *)message = ret;

	ret = eima_nl_send_message(message, message_len, NETLINK_CB(skb).portid);
	if (ret < 0)
		eima_warning("eima netlink send data failed");
	return ret;
}

static void eima_nl_recv(struct sk_buff *skb)
{
	int ret;

	eima_debug("kernel begin to process sent message");
	mutex_lock(&eima_nl_mutex);
	ret = netlink_rcv_skb(skb, &eima_nl_rcv_msg);
	mutex_unlock(&eima_nl_mutex);
	if (ret)
		eima_error("netlink receive socket buffer error");
}

static struct sock *eima_nl_create(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = eima_nl_recv,
	};

	return netlink_kernel_create(&init_net, NETLINK_EIMA, &cfg);
}

int eima_netlink_init(void)
{
	g_eima_nl = eima_nl_create();
	if (g_eima_nl == NULL) {
		eima_error("eima netlink socket create failed");
		return -ENOMEM;
	}
	eima_info("socket in kernel create success");
	return 0;
}

void eima_netlink_destroy(void)
{
	netlink_kernel_release(g_eima_nl);
}
