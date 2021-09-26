ifdef CONFIG_MTK_PLATFORM
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/charger
subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6765" "mt6768"))
subdir-ccflags-y += -DCONFIG_CHARGER_DUMP_REG_OPS_SUPPORT=1
subdir-ccflags-y += -DCONFIG_CHARGER_GET_REG_HEAD_OPS_SUPPORT=1
endif

obj-$(CONFIG_SCHARGER_V600) += schargerV600/
