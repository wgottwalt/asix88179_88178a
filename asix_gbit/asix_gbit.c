/*
 * asix-gbit - ASIX AX88179/AX88178A USB 3.0/2.0 gbit ethernet driver
 * Copyright (C) 2011-2020 ASIX
 * Copyright (C) 2020 Wilken 'Akiko' Gottwalt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
#include <linux/mdio.h>
#endif
#include <linux/netdevice.h>
#include <linux/usb.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
#include <linux/usb/usbnet.h>
#else
#pragma GCC error "minimum required kernel version 2.6.25.x"
#endif
#include "asix_gbit.h"

#define DRIVER_NAME "asix-gbit"
#define DRIVER_VERSION "0.1.0"

static const u8 def_mac179[6] = {0, 0x0E, 0xC6, 0x81, 0x79, 0x01};
static const u8 def_mac178a[6] = {0, 0x0E, 0xC6, 0x81, 0x78, 0x01};

static struct {
	u8 ctrl, timer_low, timer_high, size, ifg;
} asix_gbit_bulkin[] = {
	{0x07, 0x4F, 0x00, 0x12, 0xFF},
	{0x07, 0x20, 0x03, 0x16, 0xFF},
	{0x07, 0xAE, 0x07, 0x18, 0xFF},
	{0x07, 0xCC, 0x4C, 0x18, 0x08},
};

struct asix_gbit_data {
	u16 rx_ctrl;
	u8 checksum;
	u8 reg_monitor;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	u8 eee_enabled;
	u8 eee_active;
#endif
} __attribute__((packed));

struct asix_gbit_intf_data {
	__le16 junk1;
	u8 link;
	__le16 junk2;
	u8 status;
	__le16 junk3;
} __attribute__((packed));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
struct asix_gbit_async_handle {
	struct usb_ctrlrequest *req;
	u8 mcast_filter[8];
	u16 rx_ctrl;
} __attribute__((packed));
#endif

static int msg_level = 0;
module_param(msg_level, int, 0);
MODULE_PARM_DESC(msg_level, "usbnet msg_enable debug message level (default: 0)");

static int bulkin_size = -1;
module_param(bulkin_size, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(bulkin_size, "RX bulk in queue size -1..24 (default: -1)");

static int ifg = -1;
module_param(ifg, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ifg, "RX bulk in inter frame gap -1..255 (default: -1)");

static int eeprom_size = ASIX_EEPROM_LEN;
module_param(eeprom_size, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(eeprom_size, "EEPROM/EFUSE size 0..262144 (default: 256)");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
static int enable_eee = 0;
module_param(enable_eee, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_eee, "enable EEE support 0..1 (default: 0)");

static int enable_green = 0;
module_param(enable_green, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_green, "enable green ethernet support 0..1 (default: 0)");
#endif

//--- register access ----------------------------------------------------------

static inline int
__asix_gbit_reg_r(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index,
                  const u16 size, void *data, const bool in_pm)
{
	int ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	int (*func)(struct usbnet *, u8, u8, u16, u16, void *, u16) = in_pm ?
		usbnet_read_cmd_nopm : usbnet_read_cmd;

	BUG_ON(!dev);

	ret = func(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data,
		size);

	if (unlikely(ret < 0)) {
		printk(KERN_WARNING "%s: unable to read register index 0x%04x (%d)\n", DRIVER_NAME,
			index, ret);
	}
#else
	ret = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0), cmd, USB_DIR_IN |
		USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, size, USB_CTRL_GET_TIMEOUT);
#endif

	return ret;
}

static int
_asix_gbit_reg_r(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index, const u16 size,
                 void *data, const bool in_pm)
{
	int ret = 0;

	if (size == 2) {
		u16 tmp = 0;

		ret = __asix_gbit_reg_r(dev, cmd, value, index, size, &tmp, in_pm);
		le16_to_cpus(&tmp);
		*((u16 *)data) = tmp;
	} else if (size == 4) {
		u32 tmp = 0;

		ret = __asix_gbit_reg_r(dev, cmd, value, index, size, &tmp, in_pm);
		le32_to_cpus(&tmp);
		*((u32 *)data) = tmp;
	} else {
		ret = __asix_gbit_reg_r(dev, cmd, value, index, size, data, in_pm);
	}

	return ret;
}

static int
asix_gbit_reg_r(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index, const u16 size,
                void *data)
{
	return _asix_gbit_reg_r(dev, cmd, value, index, size, data, false);
}

static int
asix_gbit_reg_r_nopm(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index,
                     const u16 size, void *data)
{
	return _asix_gbit_reg_r(dev, cmd, value, index, size, data, true);
}

static inline int
__asix_gbit_reg_w(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index,
                  const u16 size, void *data, const bool in_pm)
{
	int ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	int (*func)(struct usbnet *, u8, u8, u16, u16, const void *, u16) = in_pm ?
		usbnet_write_cmd_nopm : usbnet_write_cmd;

	BUG_ON(!dev);

	ret = func(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data,
		size);

	if (unlikely(ret < 0)) {
		printk(KERN_WARNING "%s: unable to write register index 0x%04x (%d)\n", DRIVER_NAME,
			index, ret);
	}
#else
	ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), cmd, USB_DIR_OUT |
		USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, size, USB_CTRL_SET_TIMEOUT);
#endif

	return ret;
}

static int
_asix_gbit_reg_w(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index, const u16 size,
                 void *data, const bool in_pm)
{
	int ret = 0;

	if (size == 2) {
		u16 tmp = 0;

		tmp = *((u16 *)data);
		cpu_to_le16s(&tmp);
		ret = __asix_gbit_reg_w(dev, cmd, value, index, size, &tmp, in_pm);
	} else {
		ret = __asix_gbit_reg_w(dev, cmd, value, index, size, data, in_pm);
	}

	return ret;
}

static int
asix_gbit_reg_w(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index, const u16 size,
                void *data)
{
	return _asix_gbit_reg_w(dev, cmd, value, index, size, data, false);
}

static int
asix_gbit_reg_w_nopm(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index,
                     const u16 size, void *data)
{
	return _asix_gbit_reg_w(dev, cmd, value, index, size, data, true);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static void
asix_gbit_async_cb(struct urb *urb)
{
	struct asix_gbit_async_handle *adata = (struct asix_gbit_async_handle *)urb->context;

	if (urb->status < 0) {
		printk(KERN_ERR "%s: asix_gbit_async_handle() failed (%d)\n", DRIVER_NAME,
			urb->status);
	}

	kfree(adata->req);
	kfree(adata);
	usb_free_urb(urb);
}
#endif

static void
asix_gbit_reg_w_async(struct usbnet *dev, const u8 cmd, const u16 value, const u16 index,
                      const u16 size, const void *data)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	if (size == 2) {
		u16 tmp16 = *((u16 *)data);

		cpu_to_le16s(&tmp16);
		usbnet_write_cmd_async(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value, index, &tmp16, size);
	} else {
		usbnet_write_cmd_async(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value, index, data, size);
	}
#else
	struct urb *urb = NULL;
	struct usb_ctrlrequest *req = NULL;
	struct asix_gbit_async_handle *adata = NULL;
	void *buf = NULL;
	int status = 0;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		printk(KERN_ERR "%s: unable to allocate write async URB\n", DRIVER_NAME);
		return;
	}

	req = kmalloc(sizeof (struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req) {
		printk(KERN_ERR "%s: unable to allocate control request memory\n", DRIVER_NAME);
		usb_free_urb(urb);
		return;
	}

	adata = (struct asix_gbit_async_handle *)kmalloc(sizeof (struct asix_gbit_async_handle),
		GFP_ATOMIC);
	if (!adata) {
		printk(KERN_ERR "%s: unable to allocate async data memory\n", DRIVER_NAME);
		kfree(req);
		usb_free_urb(urb);
		return;
	}
	adata->req = req;

	if (size == 2) {
		adata->rx_ctrl = *((u16 *)data);
		cpu_to_le16s(&adata->rx_ctrl);
		buf = &adata->rx_ctrl;
	} else {
		memcpy(adata->mcast_filter, data, size);
		buf = adata->mcast_filter;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, dev->udev, usb_sndctrlpipe(dev->udev, 0), (void *)req, buf, size,
		asix_gbit_async_cb, adata);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
		printk(KERN_ERR "%s: failure submitting control message (%d)\n", DRIVER_NAME,
			status);
		kfree(adata);
		kfree(req);
		usb_free_urb(urb);
	}
#endif
}

static int
asix_gbit_mdio_r(struct net_device *netdev, const int phy_id, const int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 ret = 0;

	asix_gbit_reg_r(dev, ASIX_ACCESS_PHY, phy_id, (u16)loc, 2, &ret);

	return ret;
}

static void
asix_gbit_mdio_w(struct net_device *netdev, const int phy_id, const int loc, const int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 tmp16 = (u16)val;

	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, phy_id, (u16)loc, 2, &tmp16);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
static inline int
__asix_gbit_mmd(struct usbnet *dev, const u16 prtad, const u16 devad)
{
	int ret = 0;
	u16 tmp16 = 0;

	tmp16 = devad;
	ret = asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_MMD_CTRL, 2, &tmp16);

	tmp16 = prtad;
	ret = asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_MMD_DATA, 2, &tmp16);

	tmp16 = devad | MII_MMD_CTRL_NOINCR;
	ret = asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_MMD_CTRL, 2, &tmp16);

	return ret;
}

static int
asix_gbit_mmd_r(struct usbnet *dev, const u16 prtad, const u16 devad)
{
	int ret = 0;
	u16 tmp16 = 0;

	__asix_gbit_mmd(dev, prtad, devad);
	ret = asix_gbit_reg_r(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_MMD_DATA, 2, &tmp16);
	if (ret < 0) {
		return ret;
	}

	return tmp16;
}

static int
asix_gbit_mmd_w(struct usbnet *dev, const u16 prtad, const u16 devad, u16 data)
{
	int ret = 0;

	__asix_gbit_mmd(dev, prtad, devad);
	ret = asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_MMD_DATA, 2, &data);

	if (ret < 0) {
		return ret;
	}

	return 0;
}
#endif

//--- eeprom/efuse support functions -------------------------------------------

static int
asix_gbit_eeprom_access_eeprom_mac(struct usbnet *dev, u8 *buf, const u8 offset, const bool wflag)
{
	u16 *buf16 = (u16 *)buf;
	int i = 0;
	int ret = 0;
	u16 tmp16 = 0;

	for (i = 0; i < (ETH_ALEN >> 1); ++i) {
		if (wflag) {
			tmp16 = cpu_to_le16(*(buf16 + i));
			ret = asix_gbit_reg_w(dev, ASIX_ACCESS_EEPROM, offset + i, 1, 2, &tmp16);
			if (ret < 0) {
				break;
			}
			mdelay(15);
		} else {
			ret = asix_gbit_reg_r(dev, ASIX_ACCESS_EEPROM, offset + i, 1, 2, &buf16[i]);
			if (ret < 0) {
				break;
			}
		}
	}

	if (wflag) {
		ret = asix_gbit_reg_w(dev, ASIX_EEPROM_EFUSE_RELOAD, 0, 0, 0, NULL);
		if (ret < 0) {
			return ret;
		}
	} else {
		if (ret < 0) {
			printk(KERN_INFO "%s: unable to read EEPROM MAC (%d)\n", DRIVER_NAME, ret);
			return ret;
		}
		memcpy(dev->net->dev_addr, buf, ETH_ALEN);
	}

	return 0;
}

static int
asix_gbit_eeprom_check_eeprom(struct usbnet *dev)
{
	unsigned long jtimeout = 0;
	u8 eeprom[20] = {0};
	u16 csum = 0;
	u16 delay = HZ / 10;
	u8 i = 0;
	u8 buf = 0;

	for (i = 0; i < 6; ++i) {
		buf = i;
		if (asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_SROM_ADDR, 1, 1, &buf) < 0) {
			return -EINVAL;
		}

		buf = ASIX_SROM_EEP_RD;
		if (asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_SROM_CMD, 1, 1, &buf) < 0) {
			return -EINVAL;
		}

		jtimeout = jiffies + delay;
		do {
			asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_SROM_CMD, 1, 1, &buf);
			if (time_after(jiffies, jtimeout)) {
				return -EINVAL;
			}
		} while (buf & ASIX_SROM_EEP_BUSY);

		__asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_SROM_DATA_LOW, 2, 2, &eeprom[i * 2],
			false);
		if ((i == 0) && (eeprom[0] == 0xFF)) {
			return -EINVAL;
		}
	}

	csum = eeprom[6] + eeprom[7] + eeprom[8] + eeprom[9];
	csum = (csum >> 8) + (csum & 0xFF);
	if ((csum + eeprom[10]) != 0xFF) {
		return -EINVAL;
	}

	return 0;
}

static int
asix_gbit_eeprom_check_efuse(struct usbnet *dev, u16 *ledmode)
{
	u8 efuse[ASIX_EFUSE_LEN] = {0};
	u16 csum = 0;
	u8 i = 0;

	if (asix_gbit_reg_r(dev, ASIX_ACCESS_EFUSE, 0, ASIX_EFUSE_LEN, ASIX_EFUSE_LEN, efuse) < 0) {
		return -EINVAL;
	}

	if (*efuse == 0xFF) {
		return -EINVAL;
	}

	for (i = 0; i < ASIX_EFUSE_LEN; ++i) {
		csum = csum + efuse[i];
	}

	while (csum > 255) {
		csum = (csum & 0x00FF) + ((csum >> 8) & 0x00FF);
	}

	if (csum != 0xFF) {
		return -EINVAL;
	}

	*ledmode = (efuse[51] << 8) | efuse[52];

	return 0;
}

//--- support functions --------------------------------------------------------

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int
asix_gbit_support_set_checksums(struct usbnet *dev)
{
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	u8 checksum = 0;

	checksum = (priv->checksum & ASIX_RX_CHECKSUM) ? ASIX_RXCOE_DEF_CSUM : 0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RXCOE_CTRL, 1, 1, &checksum);

	checksum = (priv->checksum & ASIX_TX_CHECKSUM) ? ASIX_TXCOE_DEF_CSUM : 0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &checksum);

	return 0;
}
#endif

static int
asix_gbit_support_check_ether_addr(struct usbnet *dev)
{
	u8 *tmp8 = (u8 *)dev->net->dev_addr;

	if (((*((u8 *)tmp8) == 0) && (*((u8 *)tmp8 + 1) == 0) && (*((u8 *)tmp8 + 2) == 0)) ||
	    !is_valid_ether_addr((u8 *)tmp8) ||
	    !memcmp(dev->net->dev_addr, def_mac179, ETH_ALEN) ||
	    !memcmp(dev->net->dev_addr, def_mac178a, ETH_ALEN)) {
		printk(KERN_INFO "%s: invalid EEPROM MAC %pM\n", DRIVER_NAME, tmp8);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
		eth_hw_addr_random(dev->net);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
		dev->net->addr_assign_type |= NET_ADDR_RANDOM;
#endif
		random_ether_addr(dev->net->dev_addr);
#endif
		tmp8[0] = 0x00;
		tmp8[1] = 0x0E;
		tmp8[2] = 0xC6;
		tmp8[3] = 0x8E;

		return -EADDRNOTAVAIL;
	}

	return 0;
}

static int
asix_gbit_support_get_mac(struct usbnet *dev, u8 *buf)
{
	int ret = 0;

	ret = asix_gbit_eeprom_access_eeprom_mac(dev, buf, 0, false);
	if (ret < 0) {
		return ret;
	}

	if (asix_gbit_support_check_ether_addr(dev)) {
		int i = 0;

		ret = asix_gbit_eeprom_access_eeprom_mac(dev, dev->net->dev_addr, 0, true);
		if (ret < 0) {
			printk(KERN_ERR "%s: unable to write MAC to EEPROM (%d)\n", DRIVER_NAME,
				ret);
			return ret;
		}
		msleep(5);

		ret = asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_NODE_ID, ETH_ALEN, ETH_ALEN, buf);
		if (ret < 0) {
			printk(KERN_ERR "%s: unable to read MAC (%d)\n", DRIVER_NAME, ret);
			return ret;
		}

		for (i = 0; i < ETH_ALEN; ++i) {
			if (*(dev->net->dev_addr + i) != *((u8*)buf + i)) {
				printk(KERN_WARNING "%s: invalid EEPROM or non-EEPROM",
					DRIVER_NAME);
				break;
			}
		}
	}
	memcpy(dev->net->perm_addr, dev->net->dev_addr, ETH_ALEN);

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_NODE_ID, ETH_ALEN, ETH_ALEN, dev->net->dev_addr);
	if (ret < 0) {
		printk(KERN_ERR "%s: unable to write MAC (%d)\n", DRIVER_NAME, ret);
		return ret;
	}

	return 0;
}

static int
asix_gbit_support_convert_old_led(struct usbnet *dev, const u8 efuse, void *ledvalue)
{
	u16 led = 0;
	u16 tmp16 = 0;
	u8 ledmode = 0;

	if (efuse) {
		if (asix_gbit_reg_r(dev, ASIX_ACCESS_EFUSE, 0x18, 1, 2, &tmp16) < 0) {
			return -EINVAL;
		}
		ledmode = tmp16 & 0xFF;
	} else {
		if (asix_gbit_reg_r(dev, ASIX_ACCESS_EEPROM, 0x3C, 1, 2, &tmp16) < 0) {
			return -EINVAL;
		}
		ledmode = tmp16 >> 8;
	}
	printk(KERN_INFO "%s: old led mode = %02X\n", DRIVER_NAME, ledmode);

	switch (ledmode) {
		case 0xFF:
			led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 | LED1_LINK_1000 |
				LED2_ACTIVE | LED2_LINK_10 | LED2_LINK_100 | LED2_LINK_1000 |
				LED_VALID;
			break;
		case 0xFE:
			led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID;
			break;
		case 0xFD:
			led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED2_LINK_10 |
				LED_VALID;
			break;
		case 0xFC:
			led = LED0_ACTIVE | LED1_ACTIVE | LED1_LINK_1000 | LED2_ACTIVE |
				LED2_LINK_100 | LED2_LINK_10 | LED_VALID;
			break;
		default:
			led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 | LED1_LINK_1000 |
				LED2_ACTIVE | LED2_LINK_10 | LED2_LINK_100 | LED2_LINK_1000 |
				LED_VALID;
			break;
	}
	memcpy((u8 *)ledvalue, &led, 2);

	return 0;
}

static int
asix_gbit_support_led_setting(struct usbnet *dev)
{
	unsigned long jtimeout = 0;
	u16 delay = HZ / 10;
	u16 ledvalue = 0;
	u16 ledact = 0;
	u16 ledlink = 0;
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_ACCESS_STATUS, 1, 1, &tmp8);

	if (!(tmp8 & ASIX_SECLD)) {
		tmp8 = ASIX_GPIO_CTRL_GPIO1_ENABLE | ASIX_GPIO_CTRL_GPIO2_ENABLE |
			ASIX_GPIO_CTRL_GPIO3_ENABLE;
		if (asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_GPIO_CTRL, 1, 1, &tmp8) < 0) {
			return -EINVAL;
		}
	}

	if (asix_gbit_eeprom_check_eeprom(dev) == ASIX_EEPROM_EFUSE_CORRECT) {
		tmp8 = 0x42;
		if (asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_SROM_ADDR, 1, 1, &tmp8) < 0) {
			return -EINVAL;
		}

		tmp8 = ASIX_SROM_EEP_RD;
		if (asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_SROM_CMD, 1, 1, &tmp8) < 0) {
			return -EINVAL;
		}

		jtimeout = jiffies + delay;
		do {
			asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_SROM_CMD, 1, 1, &tmp8);
			if (time_after(jiffies, jtimeout)) {
				return -EINVAL;
			}
		} while (tmp8 & ASIX_SROM_EEP_BUSY);

		asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_SROM_DATA_HIGH, 1, 1, &tmp8);
		ledvalue = (tmp8 << 8);
		asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_SROM_DATA_LOW, 1, 1, &tmp8);
		ledvalue |= tmp8;

		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0)) {
			asix_gbit_support_convert_old_led(dev, 0, &ledvalue);
		}
	} else if (asix_gbit_eeprom_check_efuse(dev, &ledvalue) == ASIX_EEPROM_EFUSE_CORRECT) {
		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0)) {
			asix_gbit_support_convert_old_led(dev, 0, &ledvalue);
		}
	} else {
		asix_gbit_support_convert_old_led(dev, 0, &ledvalue);
	}

	tmp16 = GMII_PHY_PAGE_SEL_EXT;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);

	tmp16 = 0x2C;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHYPAGE, 2, &tmp16);
	asix_gbit_reg_r(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_LED_ACT, 2, &ledact);
	asix_gbit_reg_r(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_LED_LINK, 2, &ledlink);

	ledact &= GMII_LED_ACTIVE_MASK;
	ledlink &= GMII_LED_LINK_MASK;

	if (ledvalue & LED0_ACTIVE) {
		ledact |= GMII_LED0_ACTIVE;
	}
	if (ledvalue & LED1_ACTIVE) {
		ledact |= GMII_LED1_ACTIVE;
	}
	if (ledvalue & LED2_ACTIVE) {
		ledact |= GMII_LED2_ACTIVE;
	}
	if (ledvalue & LED0_LINK_10) {
		ledlink |= GMII_LED0_LINK_10;
	}
	if (ledvalue & LED1_LINK_10) {
		ledlink |= GMII_LED1_LINK_10;
	}
	if (ledvalue & LED2_LINK_10) {
		ledlink |= GMII_LED2_LINK_10;
	}
	if (ledvalue & LED0_LINK_100) {
		ledlink |= GMII_LED0_LINK_100;
	}
	if (ledvalue & LED1_LINK_100) {
		ledlink |= GMII_LED1_LINK_100;
	}
	if (ledvalue & LED2_LINK_100) {
		ledlink |= GMII_LED2_LINK_100;
	}
	if (ledvalue & LED0_LINK_1000) {
		ledlink |= GMII_LED0_LINK_1000;
	}
	if (ledvalue & LED1_LINK_1000) {
		ledlink |= GMII_LED1_LINK_1000;
	}
	if (ledvalue & LED2_LINK_1000) {
		ledlink |= GMII_LED2_LINK_1000;
	}
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_LED_ACT, 2, &ledact);
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_LED_LINK, 2, &ledlink);
	tmp16 = GMII_PHY_PAGE_SEL_PAGE0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);

	tmp16 = 0;
	if (ledvalue & LED0_FD) {
		tmp16 |= 0x01;
	} else if ((ledvalue & LED0_USB3_MASK) == 0) {
		tmp16 |= 0x02;
	}
	if (ledvalue & LED1_FD) {
		tmp16 |= 0x04;
	} else if ((ledvalue & LED1_USB3_MASK) == 0) {
		tmp16 |= 0x08;
	}
	if (ledvalue & LED2_FD) {
		tmp16 |= 0x10;
	} else if ((ledvalue & LED2_USB3_MASK) == 0) {
		tmp16 |= 0x20;
	}
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_LED_CTRL, 1, 1, &tmp16);

	return 0;
}

static void
asix_gbit_support_rx_checksum(struct sk_buff *skb, const u32 *header)
{
	skb->ip_summed = CHECKSUM_NONE;

	if ((*header & ASIX_RXHDR_L3CSUM_ERR) || (*header & ASIX_RXHDR_L4CSUM_ERR)) {
		return;
	}

	if (((*header & ASIX_RXHDR_L4_TYPE_MASK) == ASIX_RXHDR_L4_TYPE_TCP ||
	    (*header & ASIX_RXHDR_L4_TYPE_MASK) == ASIX_RXHDR_L4_TYPE_UDP)) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int
asix_gbit_support_autodetach(struct usbnet *dev, const int in_pm)
{
	int (*rfunc)(struct usbnet *, const u8, const u16, const u16, const u16, void *) =
		in_pm ? asix_gbit_reg_r_nopm : asix_gbit_reg_r;
	int (*wfunc)(struct usbnet *, const u8, const u16, const u16, const u16, void *) =
		in_pm ? asix_gbit_reg_w_nopm : asix_gbit_reg_w;
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	if (rfunc(dev, ASIX_ACCESS_EEPROM, 0x43, 1, 2, &tmp16) < 0) {
		return 0;
	}
	if ((tmp16 == 0xFFFF) || (!(tmp16 & 0x0100))) {
		return 0;
	}

	rfunc(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);
	tmp8 |= ASIX_CLK_SELECT_ULR;
	wfunc(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);

	rfunc(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	tmp16 |= ASIX_PHYPWR_RSTCTRL_AUTODETACH;
	wfunc(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);

	return 0;
}

static void
asix_gbit_support_print_mac(const char *mac)
{
	printk(KERN_INFO "%s: MAC = %02X:%02X:%02X:%02X:%02X:%02X\n", DRIVER_NAME, mac[0] & 0xFF,
		mac[1] & 0xFF, mac[2] & 0xFF, mac[3] & 0xFF, mac[4] & 0xFF, mac[5] & 0xFF);
}

//--- netdev ops ---------------------------------------------------------------

static int
asix_gbit_ndo_change_mtu(struct net_device *net, const int new_mtu)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16 = 0;

	if (new_mtu <= ASIX_MIN_MTU || new_mtu > ASIX_MAX_MTU) {
		return -EINVAL;
	}

	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);
	tmp16 = (net->mtu > 1500) ?
		(tmp16 | ASIX_MEDIUM_JUMBO_ENABLE) : (tmp16 & ~ASIX_MEDIUM_JUMBO_ENABLE);
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	return 0;
}

static int
asix_gbit_ndo_do_ioctl(struct net_device *net, struct ifreq *freq, const int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(freq), cmd, NULL);
}

static int
asix_gbit_ndo_set_mac_address(struct net_device *net, void *data)
{
	struct usbnet *dev = netdev_priv(net);
	struct sockaddr *addr = data;
	int ret = 0;

	if (netif_running(net)) {
		return -EBUSY;
	}
	if (!is_valid_ether_addr(addr->sa_data)) {
		return -EADDRNOTAVAIL;
	}

	memcpy(net->dev_addr, addr->sa_data, ETH_ALEN);
	ret = asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_NODE_ID, ETH_ALEN, ETH_ALEN,
		net->dev_addr);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

static void
asix_gbit_ndo_set_multicast_list(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	u8 *mcast_filter = ((u8 *)dev->data + 12);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	int mcast_count = net->mc_count;
#else
	int mcast_count = netdev_mc_count(net);
#endif

	priv->rx_ctrl = ASIX_RX_CTRL_START | ASIX_RX_CTRL_AB;
	if (NET_IP_ALIGN == 0) {
		priv->rx_ctrl |= ASIX_RX_CTRL_IPE;
	}

	if (net->flags & IFF_PROMISC) {
		priv->rx_ctrl |= ASIX_RX_CTRL_PRO;
	} else if ((net->flags & IFF_ALLMULTI) || (mcast_count > ASIX_MAX_MULTICAST)) {
		priv->rx_ctrl |= ASIX_RX_CTRL_AMALL;
	} else if (mcast_count == 0) {
		// nothing to do here
	} else {
		u32 crc_bits = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
		struct dev_mc_list *mcast_list = net->mc_list;
		int i = 0;

		memset(mcast_filter, 0, ASIX_MULTICAST_FILTER_SIZE);
		for (; i < net->mc_count; i++) {
			crc_bits = ether_crc(ETH_ALEN, mcast_list->dmi_addr) >> 26;
			*(mcast_filter + (crc_bits >> 3)) |= 1 << (crc_bits & 7);
			mcast_list = mcast_list->next;
		}
#else
		struct netdev_hw_addr *hwaddr = NULL;

		memset(mcast_filter, 0, ASIX_MULTICAST_FILTER_SIZE);
		netdev_for_each_mc_addr(hwaddr, net) {
			crc_bits = ether_crc(ETH_ALEN, hwaddr->addr) >> 26;
			*(mcast_filter + (crc_bits >> 3)) |= 1 << (crc_bits & 7);
		}
#endif

		asix_gbit_reg_w_async(dev, ASIX_ACCESS_MAC, ASIX_MULTICAST_FILTER_ARRAY,
			ASIX_MULTICAST_FILTER_SIZE, ASIX_MULTICAST_FILTER_SIZE, mcast_filter);
		priv->rx_ctrl |= ASIX_RX_CTRL_AM;
	}
	asix_gbit_reg_w_async(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &priv->rx_ctrl);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)
asix_gbit_ndo_set_features(struct net_device *net, const netdev_features_t features)
#else
asix_gbit_ndo_set_features(struct net_device *net, const u32 features)
#endif
{
	struct usbnet *dev = netdev_priv(net);
	u8 tmp8 = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0)
	netdev_features_t changed = net->features ^ features;
#else
	u32 changed = net->features ^ features;
#endif

	if (changed & NETIF_F_IP_CSUM) {
		asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);
		tmp8 ^= ASIX_TXCOE_TCP | ASIX_TXCOE_UDP;
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);
	}

	if (changed & NETIF_F_IPV6_CSUM) {
		asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);
		tmp8 ^= ASIX_TXCOE_TCPV6 | ASIX_TXCOE_UDPV6;
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);
	}

	if (changed & NETIF_F_RXCSUM) {
		asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_RXCOE_CTRL, 1, 1, &tmp8);
		tmp8 ^= ASIX_RXCOE_IP | ASIX_RXCOE_TCP | ASIX_RXCOE_UDP | ASIX_RXCOE_TCPV6 |
			ASIX_RXCOE_UDPV6;
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RXCOE_CTRL, 1, 1, &tmp8);
	}

	return 0;
}
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)
static int
asix_gbit_ndo_stop(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16 = 0;

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);
	tmp16 &= ~ASIX_MEDIUM_RECEIVE_ENABLE;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	return 0;
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
static const struct net_device_ops asix_gbit_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= asix_gbit_ndo_change_mtu,
	.ndo_do_ioctl		= asix_gbit_ndo_do_ioctl,
	.ndo_set_mac_address	= asix_gbit_ndo_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,2,0)
	.ndo_set_multicast_list	= asix_gbit_ndo_set_multicast_list,
#else
	.ndo_set_rx_mode	= asix_gbit_ndo_set_multicast_list,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	.ndo_set_features	= asix_gbit_ndo_set_features,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0)
	.ndo_get_stats64	= dev_get_tstats64,
#else
	.ndo_get_stats64	= usbnet_get_stats64,
#endif
#endif
};
#endif

//--- energy efficient (EEE) and green ethernet support ------------------------

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
static int
asix_gbit_get_eee(struct usbnet *dev, struct ethtool_eee *eee_data)
{
	int ret = 0;

	ret = asix_gbit_mmd_r(dev, MDIO_PCS_EEE_ABLE, MDIO_MMD_PCS);
	if (ret < 0) {
		return ret;
	}
	eee_data->supported = mmd_eee_cap_to_ethtool_sup_t(ret);

	ret = asix_gbit_mmd_r(dev, MDIO_AN_EEE_ADV, MDIO_MMD_AN);
	if (ret < 0) {
		return ret;
	}
	eee_data->advertised = mmd_eee_adv_to_ethtool_adv_t(ret);

	ret = asix_gbit_mmd_r(dev, MDIO_AN_EEE_LPABLE, MDIO_MMD_AN);
	if (ret < 0) {
		return ret;
	}
	eee_data->lp_advertised = mmd_eee_adv_to_ethtool_adv_t(ret);

	return 0;
}

static int
asix_gbit_set_eee(struct usbnet *dev, struct ethtool_eee *eee_data)
{
	u16 tmp16 = ethtool_adv_to_mmd_eee_adv_t(eee_data->advertised);

	return asix_gbit_mmd_w(dev, MDIO_AN_EEE_ADV, MDIO_MMD_AN, tmp16);
}

static void
asix_gbit_enable_eee(struct usbnet *dev)
{
	u16 tmp16 = 0;

	tmp16 = GMII_PHY_PAGE_SEL_PAGE3;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);

	tmp16 = 0x3247;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_PHYADDR, 2, &tmp16);

	tmp16 = GMII_PHY_PAGE_SEL_PAGE5;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);

	tmp16 = 0x0680;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_BMSR, 2, &tmp16);

	tmp16 = GMII_PHY_PAGE_SEL_PAGE0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);
}

static void
asix_gbit_disable_eee(struct usbnet *dev)
{
	u16 tmp16 = 0;

	tmp16 = GMII_PHY_PAGE_SEL_PAGE3;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);

	tmp16 = 0x3246;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, MII_PHYADDR, 2, &tmp16);

	tmp16 = GMII_PHY_PAGE_SEL_PAGE0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PAGE_SEL, 2, &tmp16);
}

static int
asix_gbit_check_eee(struct usbnet *dev)
{
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	mii_ethtool_gset(&dev->mii, &ecmd);

	if (ecmd.duplex & DUPLEX_FULL) {
		int eee_lp = 0;
		int eee_cap = 0;
		int eee_adv = 0;
		u32 lp = 0;
		u32 cap = 0;
		u32 adv = 0;
		u32 supported = 0;

		eee_cap = asix_gbit_mmd_r(dev, MDIO_PCS_EEE_ABLE, MDIO_MMD_PCS);
		if (eee_cap < 0) {
			priv->eee_active = 0;
			return 0;
		}

		cap = mmd_eee_cap_to_ethtool_sup_t(eee_cap);
		if (!cap) {
			priv->eee_active = 0;
			return 0;
		}

		eee_lp = asix_gbit_mmd_r(dev, MDIO_AN_EEE_LPABLE, MDIO_MMD_AN);
		if (eee_lp < 0) {
			priv->eee_active = 0;
			return 0;
		}

		eee_adv = asix_gbit_mmd_r(dev, MDIO_AN_EEE_ADV, MDIO_MMD_AN);
		if (eee_adv < 0) {
			priv->eee_active = 0;
			return 0;
		}

		adv = mmd_eee_adv_to_ethtool_adv_t(eee_adv);
		lp = mmd_eee_adv_to_ethtool_adv_t(eee_lp);
		supported = (ecmd.speed == SPEED_1000) ?
			SUPPORTED_1000baseT_Full : SUPPORTED_100baseT_Full;
		if (!(lp & adv & supported)) {
			priv->eee_active = 0;
			return 0;
		}

		priv->eee_active = 1;

		return 1;
	}

	priv->eee_active = 0;

	return 0;
}
#else

static void
asix_gbit_eee_setting(struct usbnet *dev)
{
	u16 tmp16 = 0;

	if (enable_eee) {
		tmp16 = 0x07;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x3c;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MAADR, 2, &tmp16);

		tmp16 = 0x4007;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x06;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MAADR, 2, &tmp16);
	} else {
		tmp16 = 0x07;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x3c;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MAADR, 2, &tmp16);

		tmp16 = 0x4007;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x00;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_MAADR, 2, &tmp16);
	}
}

static void
asix_gbit_green_setting(struct usbnet *dev)
{
	u16 tmp16 = 0;

	if (enable_green) {
		tmp16 = 0x03;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 31, 2, &tmp16);

		tmp16 = 0x3247;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 25, 2, &tmp16);

		tmp16 = 0x05;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 31, 2, &tmp16);

		tmp16 = 0x0680;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 1, 2, &tmp16);

		tmp16 = 0;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 31, 2, &tmp16);
	} else {
		tmp16 = 0x03;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 31, 2, &tmp16);

		tmp16 = 0x3246;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 25, 2, &tmp16);

		tmp16 = 0;
		asix_gbit_reg_w(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, 31, 2, &tmp16);
	}
}
#endif



//--- ethtool ops --------------------------------------------------------------

static void
asix_gbit_ethtool_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	usbnet_get_drvinfo(net, info);
	strlcpy(info->version, DRIVER_VERSION, sizeof (info->version));
	info->eedump_len = eeprom_size;
}

static void
asix_gbit_ethtool_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	u8 tmp8 = priv->reg_monitor;

	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	if (tmp8 & ASIX_MONITOR_MODE_RWLC) {
		wolinfo->wolopts |= WAKE_PHY;
	}
	if (tmp8 & ASIX_MONITOR_MODE_RWMP) {
		wolinfo->wolopts |= WAKE_MAGIC;
	}
}

static int
asix_gbit_ethtool_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	u8 tmp8 = 0;

	tmp8 = (wolinfo->wolopts & WAKE_PHY) ?
		(tmp8 | ASIX_MONITOR_MODE_RWLC) : (tmp8 & ~ASIX_MONITOR_MODE_RWLC);
	tmp8 = (wolinfo->wolopts & WAKE_MAGIC) ?
		(tmp8 | ASIX_MONITOR_MODE_RWMP) : (tmp8 & ~ASIX_MONITOR_MODE_RWMP);
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);
	priv->reg_monitor = tmp8;

	return 0;
}

static int
asix_gbit_ethtool_get_eeprom_len(struct net_device *net)
{
	int size = 0;

	if (eeprom_size > ASIX_EEPROM_MAX_LEN) {
		size = ASIX_EEPROM_MAX_LEN;
	} else if (eeprom_size < 0) {
		size = 0;
	} else {
		size = eeprom_size;
	}

	return size;
}

static int
asix_gbit_ethtool_get_eeprom(struct net_device *net, struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	u16 *buf = NULL;
	int fword = 0;
	int lword = 0;
	int i = 0;
	int ret = 0;

	if (eeprom->len == 0) {
		return -EINVAL;
	}
	eeprom->magic = ASIX_EEPROM_MAGIC;

	fword = eeprom->offset >> 1;
	lword = (eeprom->offset + eeprom->len - 1) >> 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,51)
	buf = kmalloc_array(lword - fword + 1, sizeof (u16), GFP_KERNEL);
#else
	buf = kmalloc(sizeof (u16) * (lword - fword + 1), GFP_KERNEL);
#endif
	if (!buf) {
		return -ENOMEM;
	}

	for (i = fword; i <= lword; ++i) {
		ret = __asix_gbit_reg_r(dev, ASIX_ACCESS_EEPROM, i, 1, 2, &buf[i - fword], false);
		if (ret < 0) {
			kfree(buf);
			return -EIO;
		}
	}
	memcpy(data, (u8 *)buf + (eeprom->offset & 1), eeprom->len);
	kfree(buf);

	return 0;
}

static int
asix_gbit_ethtool_set_eeprom(struct net_device *net, struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	u8 *buf = NULL;
	u32 offset = eeprom->offset;
	u32 len = eeprom->len;
	int err = 0;
	int i = 0;

	if (len <= 0) {
		return -EINVAL;
	}

	if ((offset % ASIX_EEPROM_BLOCK) || (len % ASIX_EEPROM_BLOCK)) {
		offset = eeprom->offset / ASIX_EEPROM_BLOCK * ASIX_EEPROM_BLOCK;
		len = eeprom->len + eeprom->offset - offset;
		len = DIV_ROUND_UP(len, ASIX_EEPROM_BLOCK) * ASIX_EEPROM_BLOCK;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,51)
		buf = kmalloc_array(len, sizeof (u8), GFP_KERNEL);
#else
		buf = kmalloc(sizeof (u8) * len, GFP_KERNEL);
#endif
		if (!buf) {
			return -ENOMEM;
		}

		for (i = 0; i < len; i += ASIX_EEPROM_BLOCK) {
			err = asix_gbit_reg_r(dev, ASIX_ACCESS_EEPROM, (offset + i) >> 1,
				ASIX_EEPROM_BLOCK >> 1, ASIX_EEPROM_BLOCK, &buf[i]);
			if (err < 0) {
				kfree(buf);
				return -EIO;
			}
		}
		memcpy(&buf[eeprom->offset - offset], data, eeprom->len);
	} else {
		buf = data;
	}

	for (i = 0; (err >= 0) && (i < len); i += ASIX_EEPROM_BLOCK) {
		err = asix_gbit_reg_r(dev, ASIX_ACCESS_EEPROM, (offset + i) >> 1,
			ASIX_EEPROM_BLOCK >> 1, ASIX_EEPROM_BLOCK, &buf[i]);
	}

	if (buf != data) {
		kfree(buf);
	}

	if (err >= 0) {
		err = asix_gbit_reg_w(dev, ASIX_EEPROM_EFUSE_RELOAD, 0, 0, 0, NULL);
	}

	return (err < 0) ? err : 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
static int
asix_gbit_ethtool_get_eee(struct net_device *net, struct ethtool_eee *edata)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;

	return asix_gbit_get_eee(dev, edata);
}

static int
asix_gbit_ethtool_set_eee(struct net_device *net, struct ethtool_eee *edata)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	int ret = -EOPNOTSUPP;

	priv->eee_enabled = edata->eee_enabled;
	if (!priv->eee_enabled) {
		asix_gbit_disable_eee(dev);
	} else {
		priv->eee_enabled = asix_gbit_check_eee(dev);
		if (!priv->eee_enabled) {
			return -EOPNOTSUPP;
		}
		asix_gbit_enable_eee(dev);
	}

	ret = asix_gbit_set_eee(dev, edata);
	if (ret) {
		return ret;
	}

	mii_nway_restart(&dev->mii);
	usbnet_link_change(dev, 0, 0);

	return ret;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0)
static int
asix_gbit_ethtool_get_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_ethtool_gset(&dev->mii, cmd);
}

static int
asix_gbit_ethtool_set_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_ethtool_sset(&dev->mii, cmd);
}
#else

static int
asix_gbit_ethtool_get_link_ksettings(struct net_device *netdev, struct ethtool_link_ksettings *cmd)
{
	struct usbnet *dev = netdev_priv(netdev);

	if (!dev->mii.mdio_read) {
		return -EOPNOTSUPP;
	}

	mii_ethtool_get_link_ksettings(&dev->mii, cmd);

	return 0;
}

static int
asix_gbit_ethtool_set_link_ksettings(struct net_device *netdev,
                                     const struct ethtool_link_ksettings *cmd)
{
	struct usbnet *dev = netdev_priv(netdev);

	if (!dev->mii.mdio_write) {
		return -EOPNOTSUPP;
	}

	return mii_ethtool_set_link_ksettings(&dev->mii, cmd);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32
asix_gbit_ethtool_get_tx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	return priv->checksum & ASIX_TX_CHECKSUM;
}

static int
asix_gbit_ethtool_set_tx_csum(struct net_device *netdev, const u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	priv->checksum =
		val ? (priv->checksum | ASIX_TX_CHECKSUM) : (priv->checksum & ~ASIX_TX_CHECKSUM);
	ethtool_op_set_tx_csum(netdev, val);

	return asix_gbit_support_set_checksums(dev);
}

static u32
asix_gbit_ethtool_get_rx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	return priv->checksum & ASIX_RX_CHECKSUM;
}

static int
asix_gbit_ethtool_set_rx_csum(struct net_device *netdev, const u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;

	priv->checksum =
		val ? (priv->checksum | ASIX_RX_CHECKSUM) : (priv->checksum & ~ASIX_RX_CHECKSUM);

	return asix_gbit_support_set_checksums(dev);
}

static int
asix_gbit_ethtool_set_tso(struct net_device *netdev, const u32 val)
{
	netdev->features =
		val ? (netdev->features | NETIF_F_TSO) : (netdev->features & ~NETIF_F_TSO);

	return 0;
}
#endif

static struct ethtool_ops asix_gbit_ethtool_ops = {
	.get_drvinfo		= asix_gbit_ethtool_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_gbit_ethtool_get_wol,
	.set_wol		= asix_gbit_ethtool_set_wol,
	.get_eeprom_len		= asix_gbit_ethtool_get_eeprom_len,
	.get_eeprom		= asix_gbit_ethtool_get_eeprom,
	.set_eeprom		= asix_gbit_ethtool_set_eeprom,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	.get_eee		= asix_gbit_ethtool_get_eee,
	.set_eee		= asix_gbit_ethtool_set_eee,
#endif
	.nway_reset		= usbnet_nway_reset,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,12,0)
	.get_settings		= asix_gbit_ethtool_get_settings,
	.set_settings		= asix_gbit_ethtool_set_settings,
#else
	.get_link_ksettings	= asix_gbit_ethtool_get_link_ksettings,
	.set_link_ksettings	= asix_gbit_ethtool_set_link_ksettings,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
	.set_tx_csum		= asix_gbit_ethtool_set_tx_csum,
	.get_tx_csum		= asix_gbit_ethtool_get_tx_csum,
	.get_rx_csum		= asix_gbit_ethtool_get_rx_csum,
	.set_rx_csum		= asix_gbit_ethtool_set_rx_csum,
	.get_tso		= ethtool_op_get_tso,
	.set_tso		= asix_gbit_ethtool_set_tso,
	.get_sg			= ethtool_op_get_sg,
	.set_sg			= ethtool_op_set_sg
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
	.get_ts_info		= ethtool_op_get_ts_info,
#endif
};

//--- driver info --------------------------------------------------------------

static int
asix_gbit_bind(struct usbnet *dev, struct usb_interface *intf)
{
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	struct ethtool_eee eee_data;
#endif
	int ret = 0;
	u32 tmp32 = 0;
	u16 tmp16 = 0;
	u8 tmp8 = 0;
	u8 mac[6] = {0};

	usbnet_get_endpoints(dev, intf);

	if (msg_level != 0) {
		dev->msg_enable = msg_level;
	}

	memset(priv, 0, sizeof (struct asix_gbit_data));

	asix_gbit_reg_w(dev, 0x81, 0x0310, 0, 4, &tmp32);

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	tmp16 = ASIX_PHYPWR_RSTCTRL_IPRL;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	msleep(200);

	tmp8 = ASIX_CLK_SELECT_ACS | ASIX_CLK_SELECT_BCS;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	ret = asix_gbit_support_get_mac(dev, mac);
	if (ret) {
		return ret;
	}
	asix_gbit_support_print_mac(mac);

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_BULKIN_QCTRL, 5, 5, &asix_gbit_bulkin[0]);
	dev->rx_urb_size = 1024 * 20;

	tmp8 = 0x34;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PAUSE_LEVEL_LOW, 1, 1, &tmp8);
	tmp8 = 0x52;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PAUSE_LEVEL_HIGH, 1, 1, &tmp8);

	asix_gbit_reg_w(dev, 0x91, 0, 0, 0, NULL);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	dev->net->do_ioctl = asix_gbit_ndo_do_ioctl;
	dev->net->set_multicast_list = asix_gbit_ndo_set_multicast_list;
	dev->net->set_mac_address = asix_gbit_ndo_set_mac_address;
	dev->net->change_mtu = asix_gbit_ndo_change_mtu;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)
	dev->net->stop = asix_gbit_ndo_stop;
#endif
#else
	dev->net->netdev_ops = &asix_gbit_netdev_ops;
#endif
	dev->net->ethtool_ops = &asix_gbit_ethtool_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
	dev->net->needed_headroom = 8;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
	dev->net->max_mtu = ASIX_MAX_MTU;
#endif

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_gbit_mdio_r;
	dev->mii.mdio_write = asix_gbit_mdio_w;
	dev->mii.phy_id_mask = 0xFF;
	dev->mii.reg_num_mask = 0xFF;
	dev->mii.phy_id = ASIX_PHY_ID;
	dev->mii.supports_gmii = 1;

	dev->net->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
	if (usb_device_no_sg_constraint(dev->udev)) {
		dev->can_dma_sg = 1;
	}
	dev->net->features |= NETIF_F_SG | NETIF_F_TSO;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	dev->net->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_SG | NETIF_F_TSO;
#endif

	tmp8 = ASIX_RXCOE_IP | ASIX_RXCOE_TCP | ASIX_RXCOE_UDP | ASIX_RXCOE_TCPV6 |
		ASIX_RXCOE_UDPV6;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RXCOE_CTRL, 1, 1, &tmp8);

	tmp8 = ASIX_TXCOE_IP | ASIX_TXCOE_TCP | ASIX_TXCOE_UDP | ASIX_TXCOE_TCPV6 |
		ASIX_TXCOE_UDPV6;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);

	priv->checksum |= ASIX_RX_CHECKSUM | ASIX_TX_CHECKSUM;

	tmp16 = ASIX_RX_CTRL_DROPCRCERR | ASIX_RX_CTRL_START | ASIX_RX_CTRL_AP |
		ASIX_RX_CTRL_AMALL | ASIX_RX_CTRL_AB;
	if (NET_IP_ALIGN == 0) {
		tmp16 |= ASIX_RX_CTRL_IPE;
	}
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);

	tmp8 = ASIX_MONITOR_MODE_PMETYPE | ASIX_MONITOR_MODE_PMEPOL | ASIX_MONITOR_MODE_RWMP;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);
	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);
	printk(KERN_INFO "%s: monitor mode = 0x%02x\n", DRIVER_NAME, tmp8);

	tmp16 = ASIX_MEDIUM_RECEIVE_ENABLE | ASIX_MEDIUM_TXFLOW_CTRL_ENABLE |
		ASIX_MEDIUM_RXFLOW_CTRL_ENABLE | ASIX_MEDIUM_FULL_DUPLEX | ASIX_MEDIUM_GIGAMODE;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	asix_gbit_support_led_setting(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	priv->eee_enabled = 0;
	priv->eee_active = 0;
	asix_gbit_disable_eee(dev);
	asix_gbit_get_eee(dev, &eee_data);
	eee_data.advertised = 0;
	asix_gbit_set_eee(dev, &eee_data);
#else
	asix_gbit_eee_setting(dev);
	asix_gbit_green_setting(dev);
#endif

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);
	priv->reg_monitor = tmp8;

	mii_nway_restart(&dev->mii);
	netif_carrier_off(dev->net);
	printk(KERN_INFO "%s: mtu = %d\n", DRIVER_NAME, dev->net->mtu);

	return 0;
}

static void
asix_gbit_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	u16 tmp16 = 0;

	tmp16 = ASIX_RX_CTRL_STOP;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);
	tmp16 = 0;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp16);
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
}

static void
asix_gbit_status(struct usbnet *dev, struct urb *urb)
{
	struct asix_gbit_intf_data *event = NULL;
	int link = 0;

	if (urb->actual_length < 8) {
		return;
	}
	event = urb->transfer_buffer;
	link = event->link & ASIX_INT_PPLS_LINK;

	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			usbnet_defer_kevent(dev, EVENT_LINK_RESET);
		} else {
			netif_carrier_off(dev->net);
			printk(KERN_INFO "%s: link status = %d\n", DRIVER_NAME, link);
		}
	}
}

static int
asix_gbit_reset(struct usbnet *dev)
{
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	struct ethtool_eee eee_data;
#endif
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	tmp16 = ASIX_PHYPWR_RSTCTRL_IPRL;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	msleep(200);

	tmp8 = ASIX_CLK_SELECT_ACS | ASIX_CLK_SELECT_BCS;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	asix_gbit_support_autodetach(dev, 0);

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_NODE_ID, ETH_ALEN, ETH_ALEN, dev->net->dev_addr);
	asix_gbit_support_print_mac(dev->net->dev_addr);

	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_BULKIN_QCTRL, 5, 5, &asix_gbit_bulkin[0]);
	dev->rx_urb_size = 1024 * 20;

	tmp8 = 0x34;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PAUSE_LEVEL_LOW, 1, 1, &tmp8);

	tmp8 = 0x52;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_PAUSE_LEVEL_HIGH, 1, 1, &tmp8);

	dev->net->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	dev->net->features |= NETIF_F_RXCSUM;
	dev->net->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
	if (usb_device_no_sg_constraint(dev->udev)) {
		dev->can_dma_sg = 1;
	}
	dev->net->features |= NETIF_F_SG | NETIF_F_TSO;
	dev->net->hw_features |= NETIF_F_SG | NETIF_F_TSO;
#endif

	tmp8 = ASIX_RXCOE_IP | ASIX_RXCOE_TCP | ASIX_RXCOE_UDP | ASIX_RXCOE_TCPV6 |
		ASIX_RXCOE_UDPV6;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RXCOE_CTRL, 1, 1, &tmp8);

	tmp8 = ASIX_TXCOE_IP | ASIX_TXCOE_TCP | ASIX_TXCOE_UDP | ASIX_TXCOE_TCPV6 |
		ASIX_TXCOE_UDPV6;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_TXCOE_CTRL, 1, 1, &tmp8);

	priv->checksum |= ASIX_RX_CHECKSUM | ASIX_TX_CHECKSUM;

	tmp16 = ASIX_RX_CTRL_DROPCRCERR | ASIX_RX_CTRL_START | ASIX_RX_CTRL_AP |
		ASIX_RX_CTRL_AMALL | ASIX_RX_CTRL_AB;
	if (NET_IP_ALIGN == 0) {
		tmp16 |= ASIX_RX_CTRL_IPE;
	}
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);

	tmp8 = ASIX_MONITOR_MODE_PMETYPE | ASIX_MONITOR_MODE_PMEPOL | ASIX_MONITOR_MODE_RWMP;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);
	printk(KERN_INFO "%s: monitor mode = 0x%02x\n", DRIVER_NAME, tmp8);

	tmp16 = ASIX_MEDIUM_TXFLOW_CTRL_ENABLE | ASIX_MEDIUM_RXFLOW_CTRL_ENABLE |
		ASIX_MEDIUM_FULL_DUPLEX | ASIX_MEDIUM_GIGAMODE;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	asix_gbit_support_led_setting(dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	priv->eee_enabled = 0;
	priv->eee_active = 0;
	asix_gbit_disable_eee(dev);
	asix_gbit_get_eee(dev, &eee_data);
	eee_data.advertised = 0;
	asix_gbit_set_eee(dev, &eee_data);
#else
	asix_gbit_eee_setting(dev);
	asix_gbit_green_setting(dev);
#endif

	mii_nway_restart(&dev->mii);
	netif_carrier_off(dev->net);
	printk(KERN_INFO "%s: mtu = %d\n", DRIVER_NAME, dev->net->mtu);

	return 0;
}

static int
asix_gbit_link_reset(struct usbnet *dev)
{
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	unsigned long jtimeout = 0;
	u32 tmp32 = 0;
	u16 delay = 10 * HZ;
	u16 mode = 0;
	u16 tmp16 = 0;
	u8 link_status = 0;
	u8 bulkin[6] = {0};

	mode = ASIX_MEDIUM_TXFLOW_CTRL_ENABLE | ASIX_MEDIUM_RXFLOW_CTRL_ENABLE;
	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_ACCESS_PHY, 1, 1, &link_status);

	jtimeout = jiffies + delay;
	while (time_before(jiffies, jtimeout)) {
		asix_gbit_reg_r(dev, ASIX_ACCESS_PHY, ASIX_PHY_ID, GMII_PHY_PHYSR, 2, &tmp16);
		if (tmp16 & GMII_PHY_PHYSR_LINK) {
			break;
		}
	}

	if (!(tmp16 & GMII_PHY_PHYSR_LINK)) {
		return 0;
	} else if (GMII_PHY_PHYSR_GIGA == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= ASIX_MEDIUM_GIGAMODE;
		if (dev->net->mtu > 1500) {
			mode |= ASIX_MEDIUM_JUMBO_ENABLE;
		}
		if (link_status & ASIX_USB_SS) {
			memcpy(bulkin, &asix_gbit_bulkin[0], 5);
		} else if (link_status & ASIX_USB_HS) {
			memcpy(bulkin, &asix_gbit_bulkin[1], 5);
		} else {
			memcpy(bulkin, &asix_gbit_bulkin[3], 5);
		}
	} else if (GMII_PHY_PHYSR_100 == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= ASIX_MEDIUM_PS;
		if (link_status & (ASIX_USB_SS | ASIX_USB_HS)) {
			memcpy(bulkin, &asix_gbit_bulkin[2], 5);
		} else {
			memcpy(bulkin, &asix_gbit_bulkin[3], 5);
		}
	} else {
		memcpy(bulkin, &asix_gbit_bulkin[3], 5);
	}

	if (bulkin_size != -1) {
		if (bulkin_size > 24) {
			bulkin_size = 24;
		} else if (bulkin_size == 0) {
			bulkin[1] = 0;
			bulkin[2] = 0;
		}
		bulkin[3] = (u8)bulkin_size;
	}
	if (ifg != -1) {
		if (ifg > 255) {
			ifg = 255;
		}
		bulkin[4] = (u8)ifg;
	}
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_BULKIN_QCTRL, 5, 5, bulkin);
	dev->rx_urb_size = (1024 * (bulkin[3] + 2));

	if (tmp16 & GMII_PHY_PHYSR_FULL) {
		mode |= ASIX_MEDIUM_FULL_DUPLEX;
	}
	printk(KERN_INFO "%s: write medium type = 0x%04x\n", DRIVER_NAME, mode);

	delay = HZ / 2;
	asix_gbit_reg_r(dev, 0x81, 0x8C, 0, 4, &tmp32);
	if (tmp32 & 0x40000000) {
		tmp16 = 0;
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &mode);

		jtimeout = jiffies + delay;
		while (time_before(jiffies, jtimeout)) {
			asix_gbit_reg_r(dev, 0x81, 0x8C, 0, 4, &tmp32);
			if (!(tmp32 & 0x40000000)) {
				break;
			}
			tmp32 = 0x80000000;
			asix_gbit_reg_w(dev, 0x81, 0x8C, 0, 4, &tmp32);
		}
		asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &priv->rx_ctrl);
	}
	mode |= ASIX_MEDIUM_RECEIVE_ENABLE;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &mode);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	priv->eee_enabled = asix_gbit_check_eee(dev);
#endif

	mii_check_media(&dev->mii, 1, 1);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
	if (dev->mii.force_media) {
		netif_carrier_on(dev->net);
	}
#endif

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static int
asix_gbit_stop(struct usbnet *dev)
{
	u16 tmp16 = 0;

	asix_gbit_reg_r(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);
	tmp16 &= ~ASIX_MEDIUM_RECEIVE_ENABLE;
	asix_gbit_reg_w(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	return 0;
}
#endif

static int
asix_gbit_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct sk_buff *skb_tmp = NULL;
	u32 *packet_header = NULL;
	int packet_count = 0;
	u32 rx_header = 0;
	u16 header_off = 0;
	u16 packet_len = 0;

	if (skb->len == 0) {
		printk(KERN_ERR "%s: RX skb length is zero\n", DRIVER_NAME);
		++dev->net->stats.rx_errors;
		return 0;
	}

	skb_trim(skb, skb->len - 4);
	memcpy(&rx_header, skb_tail_pointer(skb), sizeof (rx_header));
	le32_to_cpus(&rx_header);

	packet_count = (u16)rx_header;
	header_off = (u16)(rx_header >> 16);
	packet_header = (u32 *)(skb->data + header_off);

	while (packet_count--) {
		le32_to_cpus(packet_header);
		packet_len = (*packet_header >> 16) & 0x1FFF;

		if ((*packet_header & ASIX_RXHDR_CRC_ERR) ||
		    (*packet_header & ASIX_RXHDR_DROP_ERR)) {
			skb_pull(skb, (packet_len + 7) & 0xFFF8);
			++packet_header;
			continue;
		}

		if (packet_count == 0) {
			skb->len = packet_len;

			if (NET_IP_ALIGN == 0) {
				skb_pull(skb, 2);
			}

			skb_set_tail_pointer(skb, skb->len);
			skb->truesize = skb->len + sizeof (struct sk_buff);
			asix_gbit_support_rx_checksum(skb, packet_header);

			return 1;
		}

		skb_tmp = skb_clone(skb, GFP_ATOMIC);

		if (skb_tmp) {
			skb_tmp->len = packet_len;

			if (NET_IP_ALIGN == 0) {
				skb_pull(skb_tmp, 2);
			}

			skb_set_tail_pointer(skb_tmp, skb_tmp->len);
			skb_tmp->truesize = skb_tmp->len + sizeof (struct sk_buff);
			asix_gbit_support_rx_checksum(skb_tmp, packet_header);
			usbnet_skb_return(dev, skb_tmp);
		} else {
			return 0;
		}

		skb_pull(skb, (packet_len + 7) & 0xFFF8);
		++packet_header;
	}

	return 1;
}

static struct sk_buff *
asix_gbit_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	const int fsize = dev->maxpacket;
	const int mss = skb_shinfo(skb)->gso_size;
	u32 tx_header1 = skb->len;
	u32 tx_header2 = mss;
	int headroom = 0;
	int tailroom = 0;

	if (((skb->len + 8) % fsize) == 0) {
		tx_header2 |= 0x80000000;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
	if (!dev->can_dma_sg && (dev->net->features & NETIF_F_SG) && skb_linearize(skb)) {
		return NULL;
	}
#else
	if ((dev->net->features & NETIF_F_SG) && skb_linearize(skb)) {
		return NULL;
	}
#endif

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);
	if ((headroom + tailroom) >= 8) {
		if (headroom < 8) {
			skb->data = memmove(skb->head + 8, skb->data, skb->len);
			skb_set_tail_pointer(skb, skb->len);
		}
	} else {
		struct sk_buff *skb_tmp = NULL;

		skb_tmp = skb_copy_expand(skb, 8, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb_tmp;
		if (!skb) {
			return NULL;
		}
	}

	skb_push(skb, 4);
	cpu_to_le32s(&tx_header2);
	skb_copy_to_linear_data(skb, &tx_header2, 4);

	skb_push(skb, 4);
	cpu_to_le32s(&tx_header1);
	skb_copy_to_linear_data(skb, &tx_header1, 4);

	return skb;
}

static const struct driver_info asix_gbit_generic_usb3_info = {
	.description	= "ASIX AX88179 USB 3.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_generic_usb2_info = {
	.description	= "ASIX AX88178A USB 2.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_belkin_info = {
	.description	= "Belkin B2B128 USB 3.0 Hub/Gigabit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_cypress_info = {
	.description	= "Cypress GX3 SuperSpeed GBit Ethernet Bridge Controller",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_dlink_info = {
	.description	= "DLink DUB-1312/1332 USB 3.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_mct_info = {
	.description	= "MCT USB 3.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_samsung_info = {
	.description	= "Samsung USB GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_sitecom_info = {
	.description	= "Sitecom USB 3.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_thinkpad_info = {
	.description	= "Lenovo ThinkPad OneLinkDock USB GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

static const struct driver_info asix_gbit_toshiba_info = {
	.description	= "Toshiba USB 3.0 GBit Ethernet Adapter",
	.bind		= asix_gbit_bind,
	.unbind		= asix_gbit_unbind,
	.status		= asix_gbit_status,
	.reset		= asix_gbit_reset,
	.link_reset	= asix_gbit_link_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	.stop		= asix_gbit_stop,
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags		= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup	= asix_gbit_rx_fixup,
	.tx_fixup	= asix_gbit_tx_fixup,
};

//--- usb driver ---------------------------------------------------------------

static const struct usb_device_id asix_gbit_ids_table[] = {
	{ USB_DEVICE(0x0B95, 0x1790), // generic USB 3.0 adapter (AX88179)
	  .driver_info = (kernel_ulong_t)&asix_gbit_generic_usb3_info, },
	{ USB_DEVICE(0x0B95, 0x178A), // generic USB 2.0 adapter (AX88178A)
	  .driver_info = (kernel_ulong_t)&asix_gbit_generic_usb2_info, },
	{ USB_DEVICE(0x050d, 0x0128), // Belkin B2B128 USB 3.0 Hub/Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_belkin_info, },
	{ USB_DEVICE(0x04b4, 0x3610), // Cypress GX3 SuperSpeed Controller
	  .driver_info = (kernel_ulong_t)&asix_gbit_cypress_info, },
	{ USB_DEVICE(0x2001, 0x4a00), // DLink DUB-1312/1332
	  .driver_info = (kernel_ulong_t)&asix_gbit_dlink_info, },
	{ USB_DEVICE(0x0711, 0x0179), // MCT USB Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_mct_info, },
	{ USB_DEVICE(0x04e8, 0xa100), // Samsung USB Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_samsung_info, },
	{ USB_DEVICE(0x0df6, 0x0072), // Sitecom USB 3.0 Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_sitecom_info, },
	{ USB_DEVICE(0x17ef, 0x304b), // Lenovo ThinkPad OneLinkDock USB Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_thinkpad_info, },
	{ USB_DEVICE(0x0930, 0x0a13), // Toshiba USB 3.0 Ethernet
	  .driver_info = (kernel_ulong_t)&asix_gbit_toshiba_info, },
	{},
};
MODULE_DEVICE_TABLE(usb, asix_gbit_ids_table);

static int
asix_gbit_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	struct asix_gbit_data *priv = (struct asix_gbit_data *)dev->data;
	u8 wol[38] = {0};
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	usbnet_suspend(intf, message);

	asix_gbit_reg_r_nopm(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);
	tmp16 &= ASIX_MEDIUM_RECEIVE_ENABLE;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_MEDIUM_STATUS_MODE, 2, 2, &tmp16);

	asix_gbit_reg_r_nopm(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	tmp16 |= ASIX_PHYPWR_RSTCTRL_BZ | ASIX_PHYPWR_RSTCTRL_IPRL;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);

	wol[28] = 0x04;
	wol[29] = ASIX_MASK_WAKEUP_EVENT_4_SEC;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_STATUS, 1, 0, 38, wol);
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);

	tmp16 = ASIX_RX_CTRL_STOP;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);

	tmp8 = priv->reg_monitor;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_MONITOR_MODE, 1, 1, &tmp8);

	return 0;
}

static int
asix_gbit_resume(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	netif_carrier_off(dev->net);

	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	msleep(1);

	tmp16 = ASIX_PHYPWR_RSTCTRL_IPRL;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_PHYPWR_RSTCTRL, 2, 2, &tmp16);
	msleep(200);

	asix_gbit_support_autodetach(dev, 1);

	asix_gbit_reg_r_nopm(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);
	tmp8 |= ASIX_CLK_SELECT_ACS | ASIX_CLK_SELECT_BCS;
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	tmp16 = ASIX_RX_CTRL_DROPCRCERR | ASIX_RX_CTRL_START | ASIX_RX_CTRL_AP |
		ASIX_RX_CTRL_AMALL | ASIX_RX_CTRL_AB;
	if (NET_IP_ALIGN == 0) {
		tmp16 |= ASIX_RX_CTRL_IPE;
	}
	asix_gbit_reg_w_nopm(dev, ASIX_ACCESS_MAC, ASIX_RX_CTRL, 2, 2, &tmp16);

	return usbnet_resume(intf);
}

static struct usb_driver asix_gbit_driver = {
	.name			= DRIVER_NAME,
	.id_table		= asix_gbit_ids_table,
	.probe			= usbnet_probe,
	.suspend		= asix_gbit_suspend,
	.resume			= asix_gbit_resume,
	.reset_resume		= asix_gbit_resume,
	.disconnect		= usbnet_disconnect,
	.supports_autosuspend	= 1,
};

static int
__init asix_gbit_init(void)
{
	printk(KERN_INFO "%s: loading version %s\n", DRIVER_NAME, DRIVER_VERSION);

	return usb_register(&asix_gbit_driver);
}
module_init(asix_gbit_init);

static void
__exit asix_gbit_exit(void)
{
	printk(KERN_INFO "%s: unloading\n", DRIVER_NAME);
	usb_deregister(&asix_gbit_driver);
}
module_exit(asix_gbit_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wilken 'Akiko' Gottwalt");
MODULE_DESCRIPTION("ASIX AX88179/AX88178A usbnet based driver");
MODULE_VERSION(DRIVER_VERSION);
