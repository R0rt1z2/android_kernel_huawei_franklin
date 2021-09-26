obj-$(CONFIG_HUAWEI_DUBAI) += \
	dubai.o \
	dubai_wakeup_stats.o
obj-$(CONFIG_HUAWEI_DUBAI_BATTERY_STATS) += dubai_battery_stats.o

ifeq ($(CONFIG_HUAWEI_DUBAI_GPU_STATS),y)
obj-y += dubai_gpu_freq_stats.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gpu/ged/include/
endif

obj-$(CONFIG_HUAWEI_DUBAI_DDR_STATS) += dubai_ddr_stats.o
obj-$(CONFIG_HUAWEI_DUBAI_VCORE_STATS) += dubai_vcore_stats.o
ifneq ($(filter y, $(CONFIG_HUAWEI_DUBAI_DDR_STATS) $(CONFIG_HUAWEI_DUBAI_VCORE_STATS)),)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/swpm_v1/
obj-y += dubai_update_swpm_stats.o
endif
