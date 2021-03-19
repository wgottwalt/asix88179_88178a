ccflags-y = -DEXPORT_SYMTAB
obj-m := asix_gbit.o

KDIR = /lib/modules/$(shell uname -r)/build/
PWD = $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

.PHONY: modules clean

-include $(KDIR)/Rules.make
