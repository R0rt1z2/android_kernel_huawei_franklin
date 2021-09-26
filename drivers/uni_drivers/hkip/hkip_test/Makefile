ccflags-$(CONFIG_HUAWEI_HKIP_DEBUG) += -Idrivers/hkip/
ccflags-$(CONFIG_HUAWEI_HKIP_DEBUG) += -I$(srctree)/security/selinux -I$(srctree)/security/selinux/ss -I$(srctree)/security/selinux/include
ccflags-$(CONFIG_HUAWEI_HKIP_DEBUG) += -I$(objtree)/security/selinux -I$(objtree)/security/selinux/ss -I$(objtree)/security/selinux/include

obj-$(CONFIG_HUAWEI_HKIP_DEBUG) += dev.o debugfs.o ioctl.o sandbox.o
