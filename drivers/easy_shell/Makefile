#
#easy-shell Makefile
#
obj-$(CONFIG_HUAWEI_BB_ECALL) += easy_shell.o
ccflags-y += -Wextra \
             -Wdate-time \
             -Wfloat-equal \
             -Wshadow \
             -Wformat=2
CFLAGS_REMOVE_easy_shell.o += -Wno-format \
                              -Wno-trigraphs \
                              -Wno-pointer-sign \
                              -Wno-unused-value \
                              -Wno-sign-compare \
                              -Wno-uninitialized \
                              -Wno-tautological-compare \
                              -Wno-duplicate-decl-specifier