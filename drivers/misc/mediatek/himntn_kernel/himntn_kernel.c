/*
 * process for himntn function
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#include "himntn_kernel.h"
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <log/log_usertype.h>

#ifdef CONFIG_FINAL_RELEASE
static unsigned long long g_himntn_data = HIMNTN_DEDAULT_COMMERCIAL;
#else
static unsigned long long g_himntn_data = HIMNTN_DEFAULT_BETA;
#endif

// if item read data before himntn init, return false
static unsigned int g_himntn_data_inited;

unsigned long long get_global_himntn_data(void)
{
	return g_himntn_data;
}

void set_global_himntn_data(unsigned long long value)
{
	g_himntn_data = value;
}

/*
 * Description: because open/read/write interface
 * rely on file system initialization, so should be
 * init ensure file can be access,
 * otherwise open file cause uncertainty
 */
static bool is_partition_init(void)
{
	unsigned int time_tmp = 0;

	while (sys_access(PARTITION_PATH, 0) != 0) {
		time_tmp += POLL_INTERVAL;
		if (time_tmp >= POLL_TIMEOUT)
			return false;
		msleep(POLL_INTERVAL);
	}
	return true;
}

static int partition_common_write(unsigned long long input)
{
	int fd = -1;
	off_t file_size;
	off_t ret = (off_t)PARTITION_FAIL;
	unsigned long long *tmp_data = NULL;
	char dst_buf[DATA_BYTE_LEN + 1] = {0};
	mm_segment_t fs;

	fs = get_fs();
	set_fs(get_ds());

	if (!is_partition_init()) {
		HIMNTN_ERR("sys access partition timeout\n");
		goto out_partition_opr;
	}

	fd = sys_open(PARTITION_PATH, O_RDWR, 0);
	if (fd < 0) {
		HIMNTN_ERR("sys open partition fail\n");
		goto out_partition_opr;
	}

	file_size = sys_lseek(fd, 0, SEEK_END);
	if (file_size <= OFFSET_SIZE_TAIL) {
		HIMNTN_ERR("sys seek file size fail\n");
		goto out_partition_opr;
	}

	ret = sys_lseek(fd, file_size - OFFSET_SIZE_TAIL, SEEK_SET);
	if (ret < 0) {
		HIMNTN_ERR("sys seek data size fail\n");
		ret = PARTITION_FAIL;
		goto out_partition_opr;
	}

	ret = sys_read(fd, (char *)dst_buf, DATA_BYTE_LEN);
	if (ret < 0) {
		HIMNTN_ERR("sys read data fail\n");
		ret = PARTITION_FAIL;
		goto out_partition_opr;
	}

	tmp_data = (unsigned long long *)dst_buf;
	*tmp_data = input;

	ret = sys_write(fd, (char *)dst_buf, DATA_BYTE_LEN);
	if (ret < 0) {
		HIMNTN_ERR("sys write data fail\n");
		ret = PARTITION_FAIL;
		goto out_partition_opr;
	}

	ret = PARTITION_PASS;

out_partition_opr:
	if (fd >= 0)
		sys_close(fd);
	set_fs(fs);
	return (int)ret;
}

static int __init get_himntn_from_cmdline(char *arg)
{
	unsigned long long value;
	char *buf = arg;

	if (!buf) {
		HIMNTN_ERR("cmdline input is null\n");
		return -1;
	}

	/* from hex string to unsigned long long */
	if (kstrtoull(buf, 16, &value) < 0) {
		HIMNTN_ERR("get himntn from cmdline fail\n");
		return -1;
	}
	HIMNTN_INFO("cmdline convert to number 0x%llx\n", value);

	set_global_himntn_data(value);
	g_himntn_data_inited = 1;

	return 0;
}

early_param("HIMNTN", get_himntn_from_cmdline);

/*
 * Description: data format before write
 * used to verify mask before data in partition
 */
static unsigned long long data_format_write(unsigned long long data)
{
	return (data | CORRECT_DATA_MASK | HIMNTN_DATA_MASK);
}

// if first boot, kernel will correct himntn data
static bool is_first_boot(unsigned long long data)
{
	if ((data & CORRECT_DATA_MASK) == CORRECT_DATA_MASK) {
		HIMNTN_INFO("partition has been correct\n");
		return false;
	}
	HIMNTN_INFO("device first boot kernel\n");
	return true;
}

// Description: write and read intermediate interface
static int himntn_partition_write(unsigned long long data)
{
	unsigned long long format_data = data_format_write(data);

	set_global_himntn_data(data);
	return partition_common_write(format_data);
}

// Description: get item index by >> base value, then & global
bool cmd_himntn_item_switch(unsigned int index)
{
	unsigned long long tmp_value;
	unsigned long long tmp_global;

	if (index < HIMNTN_ID_HEAD || index >= HIMNTN_ID_BOTTOM) {
		HIMNTN_ERR("himntn item name invalid\n");
		return false;
	}

	if (!g_himntn_data_inited) {
		HIMNTN_ERR("%d: get fail, himntn not init", index);
		return false;
	}

	tmp_value = (BASE_VALUE_GET_SWITCH >> index);
	tmp_global = get_global_himntn_data();

	if ((tmp_global & tmp_value) == 0) {
		HIMNTN_INFO("%d: status is close", index);
		return false;
	}

	HIMNTN_INFO("%d: status is open", index);
	return true;
}

/*
 * Description: usertype is module init
 * so himntn should be wait usertype init succ
 */
static unsigned int is_usertype_init(void)
{
	unsigned int time_tmp = 0;
	unsigned int log_type = get_logusertype_flag();

	while (log_type == 0) {
		time_tmp += POLL_INTERVAL;
		if (time_tmp >= POLL_TIMEOUT)
			return log_type;
		msleep(POLL_INTERVAL);
		log_type = get_logusertype_flag();
	}
	return log_type;
}

static bool is_commercial_version(void)
{
	unsigned int log_type = is_usertype_init();

	// if usertype init fail, run commercial config
	if (log_type == 0) {
		HIMNTN_ERR("log usertype init timeout\n");
		return true;
	}

	HIMNTN_INFO("log usertype = %d\n", log_type);
	if ((log_type != FANS_USER) && (log_type != BETA_USER) &&
		(log_type != TEST_USER) && (log_type != OVERSEA_USER)) {
		HIMNTN_INFO("is commercial version\n");
		return true;
	}
	HIMNTN_INFO("is beta version\n");
	return false;
}

/*
 * Description: used to correct value for log or nolog version
 * because kernel can get log or nolog flag by usertype
 * this only run in log version
 * log version will correct to beta default value
 */
static int correct_himntn_data(void *arg)
{
	if (is_commercial_version())
		return 0;
	if (!is_first_boot(get_global_himntn_data()))
		return 0;

	if (himntn_partition_write(HIMNTN_DEFAULT_BETA) == PARTITION_FAIL)
		HIMNTN_ERR("correct himntn data fail");
	else
		HIMNTN_ERR("correct himntn data succ");
	return 0;
}

static int __init himntn_init(void)
{
	struct task_struct *handle_thread = NULL;

	HIMNTN_INFO("user himntn function enable\n");

	handle_thread = kthread_create(correct_himntn_data, NULL, "himntn_run");
	if (handle_thread != NULL)
		wake_up_process(handle_thread);

	return 0;
}

module_init(himntn_init);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("process for himntn driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");
