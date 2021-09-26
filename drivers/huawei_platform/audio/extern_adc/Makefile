ifdef CONFIG_HUAWEI_EXTERN_ADC
	ccflags-y += -Idrivers/devkit/audiokit/
endif

ifneq ($(TARGET_BUILD_VARIANT),user)
	obj-$(CONFIG_HUAWEI_EXTERN_ADC)	+= extern_adc_info.o
endif
obj-$(CONFIG_HUAWEI_EXTERN_ADC)	+= extern_adc.o
obj-$(CONFIG_HUAWEI_EXTERN_ADC)	+= extern_adc_i2c_ops.o
