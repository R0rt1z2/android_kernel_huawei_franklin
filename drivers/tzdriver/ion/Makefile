ifneq ($(TARGET_BUILD_VARIANT),user)
	ccflags-y += -DDEF_ENG
endif

EXTRA_CFLAGS += -Idrivers/tzdriver
EXTRA_CFLAGS += -Idrivers/tzdriver/core
EXTRA_CFLAGS += -Idrivers/staging/android

ifeq ($(CONFIG_HW_SECMEM), y)
EXTRA_CFLAGS += -Idrivers/uni_drivers/secmem
endif

obj-$(CONFIG_STATIC_ION) += static_ion_mem.o
obj-$(CONFIG_DYNAMIC_ION) += dynamic_ion_mem.o
ifneq ($(CONFIG_MTK_PLATFORM), )
obj-$(CONFIG_STATIC_ION) += mtk/declare_static_ion.o
else
obj-$(CONFIG_STATIC_ION) += hisi/declare_static_ion.o
endif
