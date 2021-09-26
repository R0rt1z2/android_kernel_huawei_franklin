# SPDX-License-Identifier: GPL-2.0
# UFSHCD makefile
ifeq ($(CONFIG_MTK_GCOV_KERNEL),y)
GCOV_PROFILE_ufs-mtk.o := y
GCOV_PROFILE_ufs-mtk-platform.o := y
GCOV_PROFILE_ufs-mtk-dbg.o := y
GCOV_PROFILE_ufs-mtk-block.o := y
endif
obj-$(CONFIG_SCSI_UFS_DWC_TC_PCI) += tc-dwc-g210-pci.o ufshcd-dwc.o tc-dwc-g210.o
obj-$(CONFIG_SCSI_UFS_DWC_TC_PLATFORM) += tc-dwc-g210-pltfrm.o ufshcd-dwc.o tc-dwc-g210.o

ifeq ($(CONFIG_MTK_UFS_SUPPORT),y)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
MTK_MISC_INC := drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/$(MTK_MISC_INC)/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/$(MTK_MISC_INC)/mt-plat
ccflags-y += -I$(srctree)/$(MTK_MISC_INC)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/scsi/ufs/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/huawei_platform/scsi/ufs
ccflags-y += -I$(srctree)/drivers/bootdevice
endif

obj-$(CONFIG_SCSI_UFS_QCOM) += ufs-qcom.o
obj-$(CONFIG_SCSI_UFSHCD) += ufshcd.o
obj-$(CONFIG_UFSFEATURE) += ufsfeature.o
obj-$(CONFIG_UFSHPB) += ufshpb.o
obj-$(CONFIG_UFSTW) += ufstw.o
obj-$(CONFIG_SCSI_SKHPB) += ufshpb_skh.o
obj-$(CONFIG_SCSI_UFSHCD_PCI) += ufshcd-pci.o
obj-$(CONFIG_SCSI_UFSHCD_PLATFORM) += ufshcd-pltfrm.o
obj-$(CONFIG_MAS_UFSTT) += mas_ufs/ufstt.o
obj-$(CONFIG_MTK_UFS_SUPPORT) += ufs-mtk.o mas_ufs/ufshcd_mas_extend.o
obj-$(CONFIG_MTK_UFS_SUPPORT) += mediatek/$(MTK_PLATFORM)/ufs-mtk-platform.o
obj-$(CONFIG_MTK_UFS_BLOCK_IO_LOG) += ufs-mtk-block.o
obj-$(CONFIG_MTK_UFS_SUPPORT) += ufs-mtk-dbg.o
obj-$(CONFIG_HUAWEI_UFS_DSM)  += dsm_ufs.o
obj-$(CONFIG_MAS_BLK)  += mas_ufs/mas_ufs.o
obj-$(CONFIG_MAS_UFS_MANUAL_BKOPS) += mas_ufs/mas-ufs-bkops.o mas_ufs/mas_ufs_bkops_interface.o
obj-$(CONFIG_HUAWEI_UFS_VENDOR_MODE) += ufs-vendor-mode.o
obj-$(CONFIG_MAS_SCSI_UFS_VCMD) += mas_ufs/ufs_vendor_cmd.o

ifeq ($(TARGET_VERSION_MODE),factory)
    EXTRA_CFLAGS += -DHEALTH_INFO_FACTORY_MODE
endif
