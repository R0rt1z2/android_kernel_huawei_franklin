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

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6765" "mt6768"))
$(warning "platform" $(CONFIG_MTK_PLATFORM))
COMMON_VERSION := v1
ccflags-y := -DDRIVER_VERSION=1
endif

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6853" "mt6873"))
COMMON_VERSION := v1_1
ccflags-y := -DDRIVER_VERSION=2
endif

MEDIATEK_DRIVER_PATH  := $(srctree)/drivers/misc/mediatek/
subdir-ccflags-y += -I$(MEDIATEK_DRIVER_PATH)/camkit/ \
	-I$(MEDIATEK_DRIVER_PATH)/camkit/customized/ \
	-I$(MEDIATEK_DRIVER_PATH)/camkit/adapter/ \
	-I$(MEDIATEK_DRIVER_PATH)/camkit/core/inc \

obj-y += core/src/camkit_sensor_i2c.o
obj-y += core/src/camkit_driver_impl.o
obj-y += core/src/camkit_driver_3a.o
obj-y += core/src/camkit_driver_interface.o
obj-y += core/src/camkit_probe_sensor.o

obj-y += adapter/
obj-y += customized/