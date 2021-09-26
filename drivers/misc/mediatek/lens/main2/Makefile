#
# Copyright (C) 2015 MediaTek Inc.
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

#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main2/inc

ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/

ifeq ($(CONFIG_MTK_LENS_USE_MODULE), y)

MAIN2_CFILES += main2_lens.c

MAIN2_CFILES  += common/dw9800af/dw9800af.c
MAIN3_CFILES  += common/ak7375af/ak7375af.c

obj-m := main2af.o
main2af-objs := $(MAIN2_CFILES:.c=.o)

else

obj-y += main2_lens.o
obj-y += common/dw9800af/
obj-y += common/ak7375af/

endif