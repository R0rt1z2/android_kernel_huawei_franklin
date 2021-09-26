ifeq ($(strip $(TARGET_PRODUCT)), mdc)
	ccflags-y += -DCONFIG_MDC_PLATFORM
endif

ifeq ($(strip $(TARGET_PRODUCT)), mini)
	ccflags-y += -DCONFIG_MINI_PLATFORM
endif

ifneq ($(TARGET_BUILD_VARIANT),user)
	ccflags-y += -DDEF_ENG
endif

EXTRA_CFLAGS += -Idrivers/tzdriver
EXTRA_CFLAGS += -Idrivers/tzdriver/core

obj-$(CONFIG_TEELOG) += tlogger.o
obj-y += log_pages_cfg.o
