CROSS=/usr/local/arm/4.2.2-eabi/usr/bin/arm-linux-
CC :=$(CROSS)gcc
LD :=$(CROSS)ld
#AR := $(CROSS)ar -rv

ifneq ($(KERNELRELEASE), )
candrv-objs := mcpcan.o spi_cmd.o spi_control.o
obj-m :=candrv.o
else
PWD := $(shell pwd)
KDIR :=/home/staufen/kernel/kernel_yaffs2/linux-2.6.24
all:
	$(MAKE) -C $(KDIR) M=$(PWD)
clean: 
	rm -rf .*.cmd *.o *.mod *.ko .tmp_versions
endif
