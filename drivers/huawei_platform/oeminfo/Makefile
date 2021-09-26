#
# Makefile for device detect devices
#
obj-$(CONFIG_HUAWEI_OEMINFO) += oeminfo_lite.o

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6853" "mt6768"))
	ifneq (,$(filter full_internal full_oversea full_internal_hyperhold, $(SET_SYSTEM_PARTITION)))
		ccflags-y += -DOEMINFO_WP_FEATURE
	else
		ifeq ($(CONFIG_OEMINFO_WP_FEATURE), y)
			ccflags-y += -DOEMINFO_WP_FEATURE
		else
			ccflags-y += -DOEMINFO_LITE_FEATURE
		endif
	endif
else
	ifeq ($(CONFIG_OEMINFO_WP_FEATURE), y)
		ccflags-y += -DOEMINFO_WP_FEATURE
	else
		ccflags-y += -DOEMINFO_LITE_FEATURE
	endif
endif