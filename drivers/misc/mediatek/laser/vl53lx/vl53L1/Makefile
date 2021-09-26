#
# Makefile for the vl53L1 drivers.
#

#
# Kbuild for the vl53L1 drivers.
#
#EXTRA_CFLAGS+= -DVL53L1_FULL_KERNEL
#EXTRA_CFLAGS+= -DVL53L3

# Each configuration option enables a list of files.

ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx/vl53L1/inc
ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx/vl53L1/protected
ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx/vl53L1/protected/inc
ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx/vl53L1/src
ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx/vl53L1
ccflags-y	+= -I$(srctree)/drivers/misc/mediatek/laser/vl53lx

ccflags-y	+= -DVL53L1_FULL_KERNEL

ccflags-y	+= -DVL53L3

ifdef VL53L1_LOG_ENABLE
ccflags-y	+= -DVL53L1_LOG_ENABLE
endif

obj-y	+= stmvl53l1_module.o \
         stmvl53l1_module-i2c.o \
         stmvl53l1_i2c.o \
         stmvl53l1_log.o \
         src/vl53l1_api.o \
         src/vl53l1_api_core.o \
         src/vl53l1_api_strings.o \
         src/vl53l1_error_strings.o \
         src/vl53l1_core.o \
         src/vl53l1_register_funcs.o \
         src/vl53l1_api_preset_modes.o \
         src/vl53l1_api_calibration.o \
         src/vl53l1_silicon_core.o \
         src/vl53l1_zone_presets.o \
         src/vl53l1_nvm.o \
         src/vl53l1_api_debug.o \
         src/vl53l1_core_support.o \
         src/vl53l1_wait.o \
         src/vl53l1_nvm_debug.o \
         vl53l1_platform_ipp.o \
         protected/src/vl53l1_hist_funcs.o \
         protected/src/vl53l1_xtalk.o \
         protected/src/vl53l1_hist_algos_gen3.o \
         protected/src/vl53l1_hist_core.o \
         protected/src/vl53l1_sigma_estimate.o \
         protected/src/vl53l1_dmax.o \
         protected/src/vl53l1_hist_algos_gen4.o