# Comment/uncomment to allow detection of unconfigured board
#DETECT_UNCONFIGURED_BOARD=y
# Comment/uncomment to hide/show debugging messages in system log
DEBUG=y

ifeq ($(DETECT_UNCONFIGURED_BOARD),y)
  EXTRA_CFLAGS += -DDETECT_UNCONFIGURED_BOARD
endif
ifeq ($(DEBUG),y)
  EXTRA_CFLAGS += -DDEBUG
endif

TARGET = pci_mrfevr pci_mrfevg

PKG_DIR = mrf_pci_driver
PKG_FILES = $(PKG_DIR)/COPYING $(PKG_DIR)/Makefile $(PKG_DIR)/module_load \
        $(PKG_DIR)/module_unload $(PKG_DIR)/mrfcommon.c \
        $(PKG_DIR)/mrfevg.c $(PKG_DIR)/flash.c \
        $(PKG_DIR)/mrfevr.c $(PKG_DIR)/pci_mrfev.h $(PKG_DIR)/README \
        $(PKG_DIR)/eeprom_9056.c $(PKG_DIR)/eeprom.c \
        $(PKG_DIR)/jtag_9056.c $(PKG_DIR)/jtag.c \
        $(PKG_DIR)/60-mrf-pci.rules
PKG_DATE = $(shell date +"%y%m%d")
PKG_NAME = mrf_pci_driver.$(PKG_DATE).tar.gz

ifneq ($(KERNELRELEASE),)

pci_mrfevr-objs := flash.o mrfcommon.o eeprom_9056.o eeprom.o \
                   jtag_9056.o jtag.o mrfevr.o
pci_mrfevg-objs := flash.o mrfcommon.o eeprom_9056.o eeprom.o \
                   jtag_9056.o jtag.o mrfevg.o

obj-m := pci_mrfevr.o pci_mrfevg.o

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

endif

package: $(PKG_NAME)

$(PKG_NAME):
	cd ..; tar zcvf $@ $(PKG_FILES)


