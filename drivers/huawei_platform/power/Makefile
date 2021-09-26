ifeq ($(HLTHERM_RUNTEST), true)
    CFLAGS_KERNEL += -DCONFIG_HLTHERM_RUNTEST
endif

# power drivers
obj-y += charger/
obj-y += wireless/
obj-y += hardware_ic/
obj-y += battery/
obj-$(CONFIG_HUAWEI_FEATURE_PRINT_PID_NAME) += pid_socket/
obj-$(CONFIG_HUAWEI_DUBAI) += dubai/
