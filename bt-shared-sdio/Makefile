EXTRA_CFLAGS += -I$(SUBDIRS)/include
EXTRA_CFLAGS += -DEXPORT_SYMTAB
EXTRA_CFLAGS += -DDEBUG

USE_POLLING ?= FALSE
ifeq ($(strip $(USE_POLLING)),TRUE)
EXTRA_CFLAGS +=-DBT_SDIO_USE_POLLING=TRUE
$(info ****** POLLING BUILD ****** )
else
$(info ****** INTERRUPT BUILD ****** )
endif

FMAC_BUILD ?= FALSE
ifeq ($(strip $(FMAC_BUILD)),TRUE)
EXTRA_CFLAGS +=-DFMAC_BUILD=TRUE
$(info ****** FMAC_BUILD ****** )
else
$(info ****** DHD_BUILD ****** )
endif
ifeq ($(DESKTOP_SESSION), ubuntu)
ifeq ($(KERNELVERSION), 4.15.0)
GCC_SEARCH_DIRS = $(shell ($(CC) -print-search-dirs | sed -n 1p | sed 's/install: //g'))
EXTRA_CFLAGS +=-I$(GCC_SEARCH_DIRS)include
endif
endif
btsdioobjs :=                   \
    src/btsdio.o                \
    src/uart_sdio.o             \
    src/btdlfw.o                \
    src/btttysdio_common.o      \

btttysdio-objs := $(btsdioobjs)
obj-m += btttysdio.o

KBUILD_MODPOST_WARN := 1

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
	rm -f Module.markers
	rm -f Module.symvers
	rm -f modules.order
	rm -f ./src/*.o
	rm -f ./src/*.mod.c
	rm -f ./src/*.o.cmd
	rm -f ./src/*.ko
	rm -f ./src/.*.ko.cmd
	rm -f ./src/.*.mod.o.cmd
	rm -f ./src/.*.o.cmd
