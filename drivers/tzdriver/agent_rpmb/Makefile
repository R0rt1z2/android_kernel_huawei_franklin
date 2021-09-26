ifneq ($(TARGET_BUILD_VARIANT),user)
	ccflags-y += -DDEF_ENG
endif

EXTRA_CFLAGS += -Idrivers/tzdriver
EXTRA_CFLAGS += -Idrivers/tzdriver/core
EXTRA_CFLAGS += -Idrivers/tzdriver/agent_rpmb/mtk

ifdef CONFIG_IOCACHE
subdir-ccflags-y += -I$(srctree)/drivers/md/iocache
endif

ifeq ($(CONFIG_MEDIATEK_SOLUTION),y)
	MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
    EXTRA_CFLAGS += -Idrivers/mmc/core
    EXTRA_CFLAGS += -Idrivers/mmc/card
    EXTRA_CFLAGS += -Idrivers/misc/mediatek/include
    EXTRA_CFLAGS += -Idrivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
    EXTRA_CFLAGS += -Idrivers/misc/mediatek/include/mt-plat
    EXTRA_CFLAGS += -Idrivers/misc/mediatek/base/power/$(MTK_PLATFORM)
    EXTRA_CFLAGS += -Idrivers/misc/mediatek/base/power/include
    EXTRA_CFLAGS += -Idrivers/devfreq
    EXTRA_CFLAGS += -Idrivers/scsi/ufs
    EXTRA_CFLAGS += -Idrivers/mmc/host/mediatek/ComboA
    ifeq ($(CONFIG_MTK_PLATFORM), "mt6761")
        EXTRA_CFLAGS+= -Idrivers/mmc/host/mediatek/ComboA/mt6765
    else
        EXTRA_CFLAGS+= -Idrivers/mmc/host/mediatek/ComboA/$(MTK_PLATFORM)
    endif
    EXTRA_CFLAGS += -Idrivers/mmc/host/mediatek/$(MTK_PLATFORM)
    EXTRA_CFLAGS += -Idrivers/mmc/host/mediatek/$(MTK_PLATFORM)/$(MTK_PLATFORM)
endif

obj-y += core/agent_rpmb.o mtk/rpmb_driver.o
