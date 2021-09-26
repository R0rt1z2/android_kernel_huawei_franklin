#
# Makefile for pn547
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
obj-y +=pn547.o
ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
