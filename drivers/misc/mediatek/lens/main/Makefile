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


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/

ifeq ($(CONFIG_MTK_LENS_USE_MODULE), y)

MAIN_CFILES += main_lens.c

MAIN_CFILES += common/dw9800af/DW9800AF.c
MAIN_CFILES += common/dw9714af/DW9714AF.c
MAIN_CFILES  += common/cn3938af/CN3928AF.c
MAIN_CFILES  += common/gt9764af/GT9764AF.c
MAIN_CFILES  += common/fp5516af/fp5516af.c
MAIN_CFILES  += common/dw9738af/dw9738af.c
MAIN_CFILES  += common/cn3938af/CN3938AF.c
MAIN_CFILES  += common/ak7375af/ak7375af.c
MAIN_CFILES  += common/gt9764baaf/gt9764baaf.c
MAIN_CFILES  += common/bu64754af/bu64754af.c
MAIN_CFILES  += common/lc898249af/lc898249af.c
MAIN_CFILES  += common/cn3927af/cn3927af.c

obj-m := mainaf.o
mainaf-objs := $(MAIN_CFILES:.c=.o)

else

obj-y  += main_lens.o
obj-y  += common/dw9800af/
obj-y  += common/dw9714af/
obj-y  += common/cn3928af/
obj-y  += common/gt9764af/
obj-y  += common/fp5516af/
obj-y  += common/dw9738af/
obj-y  += common/cn3938af/
obj-y  += common/ak7375af/
obj-y  += common/gt9764baaf/
obj-y  += common/bu64754af/
obj-y  += common/lc898249af/
obj-y  += common/dw9800vaf/
obj-y  += common/cn3927af/
endif
