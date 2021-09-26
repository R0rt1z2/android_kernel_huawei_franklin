SDCARDFS_VERSION="0.1"

EXTRA_CFLAGS += -DSDCARDFS_VERSION=\"$(SDCARDFS_VERSION)\"
EXTRA_CFLAGS += -DSDCARDFS_SYSFS_FEATURE -DSDCARDFS_PLUGIN_PRIVACY_SPACE
EXTRA_CFLAGS += -I$(srctree)/lib/libc_sec/securec_v2/include

obj-$(CONFIG_SDCARD_FS) += sdcardfs.o

sdcardfs-y := dentry.o file.o inode.o main.o super.o lookup.o mmap.o packagelist.o derived_perm.o xattr.o
sdcardfs-y += sysfs.o
