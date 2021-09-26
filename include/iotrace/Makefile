#
## Makefile for io tracing
#
ifeq ($(CONFIG_SCSI_UFSHCD),y)
ccflags-y += -I$(srctree)/drivers/scsi/ufs
endif
obj-$(CONFIG_HUAWEI_IO_TRACING) += iotrace.o
ccflags-$(CONFIG_HUAWEI_IO_TRACING) += -Ifs/
