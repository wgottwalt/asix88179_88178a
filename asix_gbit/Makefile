EXTRA_CFLAGS = -DEXPORT_SYMTAB

obj-m += asix_gbit.o
 
all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

.PHONY: modules clean

-include $(KDIR)/Rules.make
