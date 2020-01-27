This repository is about the existing ASIX 88179/88178a Linux USB drivers. These
drivers have some performance issues on ARM, the transmit performance is quite
low. It is about 9-13 MiB/s on low-end and middle-end ARMs (Cortex-A7/A53) and
up to 20 MiB/s on really high-end ARMs (Cortex-A15/A72) while hitting full 38 to
39 MiB/s on x86. This issue can be seen in USB 2.0 mode and happens with the
mainline driver and the Vendors own driver.

So I started to write my own driver for this hardware (call asix_gbit), based on
the mainline, vendor and generic usbnet drivers. During my work on this I also
added some missing features like being able to read and write the EEPROM to
change USB ids or the MAC.

This driver is not part of the initial commit. I still have to do a rework, but
a working prototype exists (but does not fix the performance issue, still working
on this).

The directory asix_gbit will hold a working version of the asix_gbit driver and
the directory eeprom_patch hold the patch to add these EEPROM functions to the
mainline kernels driver.

I usually test with some mainline LTS kernels like 3.16, 4.14 and 4.19. So it
all should work fine with these kernels.

The EEPROM patch is experimental and may fail in some situations (but I did not
encounter one yet). I only have an AX88178a based USB 2.0 adapter with the
generic USB id 0x0b95:0x178a, so maybe other adpaters, like the USB 3.0 ones,
could show some issues.
