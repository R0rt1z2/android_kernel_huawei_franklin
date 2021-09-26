#
# Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
# Description: efusestate_info.c makefile
# Author: Liujingbo
# Create: 2019-09-19
#

# Makefile for the linux kernel.
#
# Only aarch32 need to build core/ (need machine descriptor)

#ontim add
obj-y += efusestate_info.o

subdir-ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_export_inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/module

