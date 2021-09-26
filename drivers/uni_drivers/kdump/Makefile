#
#kernel dump Makefile
#
subdir-ccflags-y              += -DTEXT_OFFSET=$(TEXT_OFFSET)
obj-$(CONFIG_HUAWEI_BB) += kdump.o
obj-$(CONFIG_HUAWEI_BB) += ringbuffer.o
obj-$(CONFIG_HUAWEI_BB) += track.o

ccflags-y += -Wextra \
             -Wdate-time \
             -Wfloat-equal \
             -Wshadow \
             -Wformat=2

KDUMP_REMOVE_CFLAGS := -Wno-format \
                       -Wno-trigraphs \
                       -Wno-pointer-sign \
                       -Wno-unused-value \
                       -Wno-sign-compare \
                       -Wno-uninitialized \
                       -Wno-tautological-compare \
                       -Wno-duplicate-decl-specifier
CFLAGS_REMOVE_kdump.o += $(KDUMP_REMOVE_CFLAGS)
CFLAGS_REMOVE_ringbuffer.o += $(KDUMP_REMOVE_CFLAGS)
CFLAGS_REMOVE_track.o += $(KDUMP_REMOVE_CFLAGS)
