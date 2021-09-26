# Makefile for the step_motor driver.
#
obj-$(CONFIG_STEP_MOTOR) += drv8846.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6873/include

ifeq ($(TARGET_VERSION_MODE),factory)
ccflags-y += -DSTEP_MOTOR_DEBUG
endif
