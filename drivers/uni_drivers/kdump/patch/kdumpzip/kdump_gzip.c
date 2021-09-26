/*
 * kdump_gzip.c
 *
 * use tar to compress the kdump file
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
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
#include <dirent.h>
#include <utils/Log.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>


#define  GZIP_TIMEOUT 100
#define  GZIP_SLEEP_TIME 10000

static time_t gettime(void)
{
	struct timespec ts;
	int ret;

	ret = clock_gettime(CLOCK_MONOTONIC, &ts);
	if (ret < 0) {
		ALOGE("clock_gettime(CLOCK_MONOTONIC) failed: %d\n", errno);
		return 0;
	}
	return ts.tv_sec;
}
static int wait_for_file(const char *filename, int timeout)
{
	struct stat info;
	time_t timeout_time = gettime() + timeout;
	int ret = -1;

	ALOGE("wait done child status %s \n", filename);
	while (gettime() < timeout_time && ((ret = stat(filename, &info)) < 0))
		usleep(GZIP_SLEEP_TIME);
	ALOGE("wait done child status %d \n", ret);
	ALOGE("wait done child status %d \n", errno);
	return ret;
}

int main(void)
{
	if (wait_for_file("/data/uni/mdump/done", GZIP_TIMEOUT) < 0) {
		ALOGE("Timed out waiting for memorydump done\n");
		return -1;
	}

	ALOGE("***************kdump_gzip del old file*****************\n");
	system("/system/bin/rm -f /data/uni/mdump/dump.tar.gz");
	system("/system/bin/tar -zcvf /data/uni/dump.tar.gz  /data/uni/mdump");
	system("/system/bin/chmod 0640 /data/uni/dump.tar.gz");
	ALOGE("***************kdump_gzip end*****************\n");

	ALOGE("***************kdump_gzip cp file to dump*****************\n");
	system("/system/bin/cp /data/uni/dump.tar.gz  /data/uni/mdump/");
	ALOGE("***************kdump_cp end*****************\n");

	system("/system/bin/rm -f /data/uni/mdump/done");
	ALOGE("***************rm end*****************\n");
	return 0;
}
