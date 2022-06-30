#
# chipone-tddi.ko
#
# Makefile: for external make invocation
#
# Note:
# - Please refer to modules/sample/Makefile to find out what should be
#   done in this Makefile
#


#KO_MODULE_NAME := chipone-tddi
#KO_MODULE_OUT := $(BSP_MODULES_OUT)/$(KO_MODULE_NAME)
#KO_MODULE_KBUILD := $(CURDIR)/Kbuild

#.PHONY: modules modules_install clean
#modules:
#	@ln -snf $(CURDIR) $(KO_MODULE_OUT)/source
#	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@

#modules_install:
#	$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) $@

## Remove the out directory wholly
#clean:
#	@#$(MAKE) -C $(BSP_KERNEL_PATH) M=$(KO_MODULE_OUT) src=$(CURDIR) $@
#	rm -rf $(KO_MODULE_OUT)




obj-y	+=  cts_charger_detect.o
obj-y	+=  cts_core.o
obj-y   +=  cts_earjack_detect.o

obj-y	+=  cts_firmware.o
obj-y	+=  cts_i2c_driver.o
obj-y	+=  cts_oem_huaqin.o
obj-y	+=  cts_platform.o
obj-y   +=  cts_sfctrlv2.o

obj-y   +=  cts_spi_flash.o
obj-y   +=  cts_strerror.o
obj-y   +=  cts_sysfs.o
obj-y   +=  cts_test.o
obj-y   +=  cts_tool.o
obj-y   +=  cts_proc.o