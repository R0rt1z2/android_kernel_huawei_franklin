#
# Makefile for misc devices that really don't fit anywhere else.
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/core
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6853/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/devkit/lcdkit/lcdkit3.0/kernel/common/include/
ccflags-y += -Ivendor/huawei/chipset_common/plugins/kernel/include/
ifeq ($(CONFIG_MTK_PLATFORM), "mt6873")
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/v02
endif
ifeq ($(CONFIG_MTK_PLATFORM), "mt6853")
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/mt6853
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/v03
endif
obj-$(CONFIG_HUAWEI_SENSORS_2_0) += sensor_scp.o sensor_sysfs.o motion_channel.o motion_route.o virt_prox.o als_route.o
obj-$(CONFIG_MTK_ABOV_SAR) += sar/a96t346/
#ifdef CONFIG_HUAWEI_HOST_TS_KIT
ccflags-y += -I$(srctree)/drivers/devkit/tpkit/hostprocessing
#endif
obj-$(CONFIG_STEP_MOTOR) += step_motor/
obj-$(CONFIG_POS_HALL) += pos_hall/
