#add for MTK platform
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/auxadc/
subdir-ccflags-$(CONFIG_USB_MU3D_DRV) += -Werror -I$(srctree)/drivers/misc/mediatek/mu3d/drv
subdir-ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb_c/mt6336
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb_c/tcpc/inc
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6335)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6335
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6355)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6356)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6357)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6357
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6358)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6358
subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6370)+= -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6370
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/charger
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/battery
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/auxadc/

subdir-ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc

#compile huawei files
obj-y      += huawei_charger.o
obj-y      += huawei_mtk_charger.o
obj-y      += huawei_mtk_battery.o
obj-y      += huawei_battery.o
obj-y      += huawei_iscd.o
obj-y      += mtk_battery_temp.o
obj-y      += mtk_usb_temp.o
obj-$(CONFIG_FCP_CHARGER) += huawei_charger_common.o
obj-$(CONFIG_FCP_CHARGER) += fcp_charger.o
