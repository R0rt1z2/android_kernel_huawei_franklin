# In case the platform does NOT support this type of sensors

ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/step_counter/stepsignhub
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/v01
ccflags-y += -I$(srctree)/drivers/huawei_platform/sensors/

obj-y	+= motion_channel.o
obj-y	+= motion_route.o
