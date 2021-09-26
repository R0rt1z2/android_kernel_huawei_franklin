# mt6893&mt6885 co-platform config
ifeq (y, $(filter y, $(CONFIG_MACH_MT6893)))
MTK_PLATFORM := mt6885
else
MTK_PLATFORM := $(CONFIG_MTK_PLATFORM)
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/mddp/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

obj-y += mddp.o

mddp-y	+= ctrl/mddp_if.o
mddp-y	+= ctrl/mddp_sm.o
mddp-y	+= ctrl/mddpwh_sm.o
mddp-y	+= ctrl/mddp_dev.o
mddp-y	+= ctrl/mddp_ipc.o

ifneq (,$(filter y,$(CONFIG_MTK_MDDP_WH_SUPPORT)))
subdir-ccflags-y += -DMDDP_TETHERING_SUPPORT
mddp-y	+= filter/mddp_filter.o
mddp-y	+= filter/mddp_f_dev.o
# Data usage
mddp-y	+= usage/mddp_usage.o
endif
