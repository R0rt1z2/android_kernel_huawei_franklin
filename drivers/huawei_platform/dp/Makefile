ifeq ($(CONFIG_HUAWEI_DP),y)
ccflags-y += -I$(srctree)/drivers/devkit/audiokit/
obj-$(CONFIG_DP_AUX_SWITCH) += dp_aux_switch.o
obj-y += dp_factory.o
obj-y += dp_debug.o
obj-y += hw_dp.o

ifeq ($(TARGET_VERSION_MODE),factory)
	ccflags-y += -DDP_FACTORY_MODE_ENABLE
endif
ifneq ($(TARGET_BUILD_VARIANT),eng)
	ccflags-y += -DDP_DEBUG_ENABLE
endif

endif
