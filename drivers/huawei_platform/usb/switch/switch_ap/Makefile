#add for MTK platform
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/charger
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc/

# Switch Class Driver
obj-$(CONFIG_SWITCH_FSA9685) += switch_fsa9685.o
obj-$(CONFIG_SWITCH_FSA9685) += usbswitch_fsa9685.o
obj-$(CONFIG_SWITCH_FSA9685) += usbswitch_rt8979.o
obj-$(CONFIG_SWITCH_FSA9685) += rt8979_bc12.o
obj-$(CONFIG_SWITCH_FSA9685) += switch_chip_i2c.o