#
# Makefile for ams TCS3430 device.
#
ifdef CONFIG_SENSORS_COLOR_AP

#
# Feature Options
#

ccflags-$(CONFIG_SENSORS_COLOR_AP)	+= -DCONFIG_AMS_OPTICAL_SENSOR_ALS
ccflags-$(CONFIG_SENSORS_COLOR_AP)	+= -DCONFIG_AMS_OPTICAL_SENSOR_ALS_XYZ
ccflags-$(CONFIG_SENSORS_COLOR_AP)	+= -DCONFIG_AMS_DEBUG_LOG
#
# Include Paths
#
ccflags-$(CONFIG_SENSORS_COLOR_AP)	+= -I$(src)

#
# Object Artifacts
obj-$(CONFIG_SENSORS_COLOR_AP)	+= color_sensor.o ams_as7341.o ams_tcs3707.o

endif
