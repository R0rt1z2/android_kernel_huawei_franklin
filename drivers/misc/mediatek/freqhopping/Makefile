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

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/

obj-y := mtk_freqhopping_drv.o

ifeq ($(CONFIG_MACH_MT6893),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/mt6885/
obj-y += mt6885/
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/$(MTK_PLATFORM)/
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
