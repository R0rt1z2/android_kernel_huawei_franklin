#
# Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_DRIVER_PATH  := $(srctree)/drivers/misc/mediatek
IMGSENSOR_DRIVER_PATH  := $(srctree)/drivers/misc/mediatek/imgsensor/src
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
CONFIG_MTK_PLATFORM_SENSOR := $(CONFIG_MTK_PLATFORM)

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6765" "mt6768"))
$(warning "platform" $(CONFIG_MTK_PLATFORM))
COMMON_VERSION := v1
ccflags-y := -DDRIVER_VERSION=1
endif

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6853" "mt6873"))
COMMON_VERSION := v1_1
ccflags-y := -DDRIVER_VERSION=2
endif

subdir-ccflags-y += -I$(IMGSENSOR_DRIVER_PATH)/common/$(COMMON_VERSION) \
					-I$(MTK_DRIVER_PATH)/imgsensor/inc/ \
					-I$(srctree)/drivers/i2c/busses/ \
					-I$(srctree)/drivers/misc/mediatek/ccu/inc \
					-I$(srctree)/drivers/misc/mediatek/mmdvfs \
					-I$(MTK_DRIVER_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM_SENSOR) \
					-I$(MTK_DRIVER_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM_SENSOR)/camera_hw \

obj-y += camkit_io_adapter.o
obj-y += camkit_driver_adapter.o
obj-y += camkit_dts_adapter.o