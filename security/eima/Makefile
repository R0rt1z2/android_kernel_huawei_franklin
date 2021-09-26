# Copyright (c) Huawei Technologies Co., Ltd. 2017-2020. All rights reserved.
#
# Makefile for the EMUI Integrity Measurement Architecture(EIMA).
#

obj-$(CONFIG_HUAWEI_EIMA) += eima_module.o eima_queue.o eima_fs.o dkm.o hash_generator.o
obj-$(CONFIG_HUAWEI_EIMA_ACCESS_CONTROL) += eima_netlink.o
ifdef CONFIG_MTK_PLATFORM
ccflags-$(CONFIG_HUAWEI_EIMA) += -Idrivers/ca_antiroot \
				-Ivendor/huawei/chipset_common/modules/libc_sec/include
else
ccflags-$(CONFIG_HUAWEI_EIMA) += -Idrivers/hisi/ca_antiroot \
				-Ivendor/hisi/bsp/libc_sec/include
endif

flags += -Wdate-time -Wfloat-equal
