ifneq ($(TARGET_BUILD_VARIANT),user)
	ccflags-y += -DDEF_ENG
endif

EXTRA_CFLAGS += -Idrivers/tzdriver
EXTRA_CFLAGS += -Idrivers/tzdriver/agent_rpmb/core
EXTRA_CFLAGS += -Idrivers/tzdriver/auth
EXTRA_CFLAGS += -Idrivers/tzdriver/ion
EXTRA_CFLAGS += -Idrivers/tzdriver/tlogger
EXTRA_CFLAGS += -Idrivers/tzdriver/tui
EXTRA_CFLAGS += -Idrivers/uni_drivers/secmem

obj-$(CONFIG_TEECD_AUTH) += teec_daemon_auth.o
obj-$(CONFIG_KERNEL_CLIENT) += teek_client_api.o
obj-y += smc_smp.o tc_client_driver.o session_manager.o mailbox_mempool.o
obj-y += agent.o gp_ops.o mem.o cmdmonitor.o tzdebug.o tz_spi_notify.o tz_pm.o
