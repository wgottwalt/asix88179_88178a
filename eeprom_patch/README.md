note: Since kernel 5.8 this patch is obsolete, this kernel now includes eeprom
      writing functionality.

This patch adds the ethtool_ops set-eeprom function to the mainline kernel to be
able to use ethtool to modify the content of the EEPROM. There is also the need
to change a checksum if you modify the VID and/or PID of the device. The patch
provides a descrition how to calculate the checksum.

This patch is made for kernel 4.14.103, but it is tested with 3.16, 4.4 and
4.19. You may need some minor and very simple modifications to get it to apply
to these other kernel version.

The patch is experimental, there are some memory alignment requirements, but I
think I got them all right. I did not encounter any issues for now, but there
may be some.
