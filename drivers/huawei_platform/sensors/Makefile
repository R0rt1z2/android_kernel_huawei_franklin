#
# Makefile for misc devices that really don't fit anywhere else.
#
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/v01
ccflags-y += -I$(srctree)/drivers/huawei_platform/sensor
ifeq ($(CONFIG_LCD_KIT_DRIVER),y)
	ccflags-y += -I$(srctree)/drivers/devkit/lcdkit/lcdkit3.0/kernel/common/include/
endif

obj-$(CONFIG_HUAWEI_SENSORS) := sensor_info.o
