LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := kdump_gzip.c

LOCAL_MODULE := kdump_gzip
LOCAL_CFLAGS += -fno-common -pipe
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES += vendor/huawei/chipset_common/modules/libc_sec/include
LOCAL_C_INCLUDES += system/core/init
LOCAL_SHARED_LIBRARIES += libc_secshared

LOCAL_SHARED_LIBRARIES += liblog

include $(BUILD_EXECUTABLE)
