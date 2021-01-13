/*
 * Davicom DM96xx USB 10/100Mbps ethernet devices
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * revision:    2019-12-21, Jason416 <jason416@foxmail.com>
 * Description: porting DM96xx driver to uboot DM_ETH module.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

// #define DEBUG

#include <common.h>
#include <dm.h>
#include <usb.h>
#include <malloc.h>
#include <memalign.h>
#include <errno.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include "usb_ether.h"

#ifdef DEBUG
#undef debug
#define debug(fmt, args...) debug_cond(true, fmt, ##args)
#endif


/* datasheet:
 http://ptm2.cc.utu.fi/ftp/network/cards/DM9601/From_NET/DM9601-DS-P01-930914.pdf
*/

#define DM9601_BASE_NAME "Davicom DM96xx USB 10/100 Ethernet"

/* control requests */
#define DM_READ_REGS	0x00
#define DM_WRITE_REGS	0x01
#define DM_READ_MEMS	0x02
#define DM_WRITE_REG	0x03
#define DM_WRITE_MEMS	0x05
#define DM_WRITE_MEM	0x07

#ifndef CONFIG_DM_ETH 
/* DM_RX_CTRL */
#define DM_RX_DIS_LONG	(1 << 5)
#define DM_RX_DIS_CRC	(1 << 4)
#define DM_RX_ALL	(1 << 3)
#define DM_RX_PRMSC	(1 << 1)
#define DM_RX_RXEN	(1 << 0)

#define AX_RX_URB_SIZE 2048
#define PHY_CONNECT_TIMEOUT 5000
#endif
/* registers */
#define DM_NET_CTRL	    0x00
#define DM_RX_CTRL	    0x05
#define DM_SHARED_CTRL	0x0b
#define DM_SHARED_ADDR	0x0c
#define DM_SHARED_DATA	0x0d	/* low + high */
#define DM_PHY_ADDR	    0x10	/* 6 bytes */
#define DM_MCAST_ADDR	0x16	/* 8 bytes */
#define DM_GPR_CTRL	    0x1e
#define DM_GPR_DATA	    0x1f
#define DM_CHIP_ID	    0x2c
#define DM_MODE_CTRL	0x91	/* only on dm9620 */

/* chip id values */
#define ID_DM9601	    0
#define ID_DM9620	    1

#define DM_MAX_MCAST	64
#define DM_MCAST_SIZE	8
#define DM_EEPROM_LEN	256
#define DM_TX_OVERHEAD	2	    /* 2 byte header */
#define DM_RX_OVERHEAD	7	    /* 3 byte header + 4 byte crc tail */
#define DM_TIMEOUT	    1000

/* local defines */
#define USB_CTRL_SET_TIMEOUT  5000
#define USB_CTRL_GET_TIMEOUT  5000
#define USB_BULK_SEND_TIMEOUT 5000
#define USB_BULK_RECV_TIMEOUT 5000

#define DM9601_RX_URB_SIZE  2048
#define PHY_CONNECT_TIMEOUT 5000

#ifndef CONFIG_DM_ETH
struct dm_dongle {
	unsigned short vendor;
	unsigned short product;
	int flags;
};

static const struct dm_dongle products[] = {
	{0x07aa, 0x9601, 0, }, /* Corega FEther USB-TXC */
	{0x0a46, 0x9601, 0, }, /* Davicom USB-100 */
	{0x0a46, 0x6688, 0, }, /* ZT6688 USB NIC */
	{0x0a46, 0x0268, 0, }, /* ShanTou ST268 USB NIC */
	{0x0a46, 0x8515, 0, }, /* ADMtek ADM8515 USB NIC */
	{0x0a47, 0x9601, 0, }, /* Hirose USB-100 */
	{0x0fe6, 0x8101, 0, }, /* DM9601 USB to Fast Ethernet Adapter */
	{0x0fe6, 0x9700, 0, }, /* DM9601 USB to Fast Ethernet Adapter */
	{0x0a46, 0x9000, 0, }, /* DM9000E */
	{0x0a46, 0x9620, 0, }, /* DM9620 USB to Fast Ethernet Adapter */
	{0x0a46, 0x9621, 0, }, /* DM9621A USB to Fast Ethernet Adapter */
	{0x0a46, 0x9622, 0, }, /* DM9622 USB to Fast Ethernet Adapter */
	{0x0a46, 0x0269, 0, }, /* DM962OA USB to Fast Ethernet Adapter */
	{0x0a46, 0x1269, 0, }, /* DM9621A USB to Fast Ethernet Adapter */
	{},			/* END */
};
#endif
/* driver private */
struct dm9601_private {
    int flags;
#ifdef CONFIG_DM_ETH
    struct ueth_data ueth;
#endif
};


static int dm_read(struct ueth_data *dev, u8 reg, u16 length, void *data)
{
	int err;
    struct usb_device *usb_dev = dev->pusb_dev;

	err = usb_control_msg(usb_dev, usb_rcvctrlpipe(usb_dev, 0),
                    DM_READ_REGS,
                    USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                    0, reg, data, length,
                    USB_CTRL_SET_TIMEOUT);
	if(err != length && err >= 0)
		err = -EINVAL;

	return err;
}

static int dm_read_reg(struct ueth_data *dev, u8 reg, u8 *value)
{
	return dm_read(dev, reg, 1, value);
}

static int dm_write(struct ueth_data *dev, u8 reg, u16 length, void *data)
{
	int err;
    struct usb_device *usb_dev = dev->pusb_dev;

    err = usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
                    DM_WRITE_REGS,
                    USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                    0, reg, data, length,
                    USB_CTRL_SET_TIMEOUT);
    if (err >= 0 && err < length)
        err = -EINVAL;

    return err;
}

static int dm_write_reg(struct ueth_data *dev, u8 reg, u8 value)
{
    struct usb_device *usb_dev = dev->pusb_dev;

    return usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
                    DM_WRITE_REG,
                    USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                    value, reg, NULL, 0,
                    USB_CTRL_SET_TIMEOUT);
}

static int dm_read_shared_word(struct ueth_data *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0xc : 0x4);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s read timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);
	ret = dm_read(dev, DM_SHARED_DATA, 2, value);

	/* debug("read shared %d 0x%02x returned 0x%04x, %d\n", */
	/* 	   phy, reg, *value, ret); */

 out:
	return ret;
}

static int dm_write_shared_word(struct ueth_data *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;

	ret = dm_write(dev, DM_SHARED_DATA, 2, &value);
	if (ret < 0)
		goto out;

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0x1a : 0x12);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s write timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);

out:
	return ret;
}

static int dm9601_mdio_read(struct ueth_data *dev, int phy_id, int loc)
{
    ALLOC_CACHE_ALIGN_BUFFER(__le16, res, 1);

	if (phy_id) {
		printf("Only internal phy supported\n");
		return 0;
	}

	dm_read_shared_word(dev, 1, loc, res);

    /* debug( "dm9601_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n", */
    /*       phy_id, loc, le16_to_cpu(res)); */

	return le16_to_cpu(*res);
}

static void dm9601_mdio_write(struct ueth_data *dev, int phy_id, int loc, int val)
{
    ALLOC_CACHE_ALIGN_BUFFER(__le16, res, 1);
	*res = cpu_to_le16(val);

	if (phy_id) {
		printf("Only internal phy supported\n");
		return;
	}

	/* debug("dm9601_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n", */
	/* 	   phy_id, loc, val); */

	dm_write_shared_word(dev, 1, loc, *res);
}

#ifdef CONFIG_DM_ETH
static void dm9601_set_multicast(struct ueth_data *dev)
{
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, hashes, DM_MCAST_SIZE);
	u8 rx_ctl = 0x31;

	memset(hashes, 0x00, DM_MCAST_SIZE);
	hashes[DM_MCAST_SIZE - 1] |= 0x80;	/* broadcast address */

	dm_write(dev, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	dm_write_reg(dev, DM_RX_CTRL, rx_ctl);
}

static int dm9601_read_mac_address(struct ueth_data *dev, uint8_t *enetaddr)
{
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buf, ETH_ALEN);

    /* read MAC */
    if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, buf) < 0) {
        printf("dm9601: Error reading MAC address\n");
        return -ENODEV;

    }

    memcpy(enetaddr, buf, ETH_ALEN);

    return 0;
}


static int dm9601_set_mac_address(struct ueth_data *dev, uint8_t *enetaddr)
{
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, addr, ETH_ALEN);

	if (!is_valid_ethaddr(enetaddr)) {
		printf("not setting invalid mac address %pM\n", enetaddr);
		return -EINVAL;
	}

	memcpy(addr, enetaddr, ETH_ALEN);
	dm_write(dev, DM_PHY_ADDR, ETH_ALEN, addr);

#ifdef DEBUG
    dm9601_read_mac_address(dev, addr);
    debug("---->after write: <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          addr[0], addr[1], addr[2],
          addr[3], addr[4], addr[5]);
#endif

	return 0;
}
#endif
/*
 * mii_nway_restart - restart NWay (autonegotiation) for this interface
 *
 * Returns 0 on success, negative on error.
 */
static int mii_nway_restart(struct ueth_data *dev)
{
    int bmcr;
    int r = -1;

    /* if autoneg is off, it's an error */
    bmcr = dm9601_mdio_read(dev, dev->phy_id, MII_BMCR);
    debug("%s: bmcr: 0x%x\n", __func__, bmcr);
    if (bmcr & BMCR_ANENABLE) {
        bmcr |= BMCR_ANRESTART;
        dm9601_mdio_write(dev, dev->phy_id, MII_BMCR, bmcr);
        r = 0;
    }

    return r;
}

#ifndef CONFIG_DM_ETH

static void dm9601_set_multicast(struct ueth_data *dev, u8 mcast[DM_MCAST_SIZE])
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, hashes, DM_MCAST_SIZE);
	u8 rx_ctl = (DM_RX_DIS_LONG | DM_RX_DIS_CRC | DM_RX_RXEN);

	memcpy(hashes, mcast, DM_MCAST_SIZE);
	dm_write(dev, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	dm_write_reg(dev, DM_RX_CTRL, rx_ctl);
}

static int dm9601_link_reset(struct ueth_data *dev)
{
	u8 mcast0[DM_MCAST_SIZE] = { 0x0 };
	u8 mcast1[DM_MCAST_SIZE] = { 0, 0x00, 0, 0x80, 0, 0, 0, 0 };
	u8 mcast2[DM_MCAST_SIZE] = { 0, 0x00, 0, 0x84, 0, 0, 0, 0 };
	u8 mcast3[DM_MCAST_SIZE] = { 0, 0x80, 0, 0x84, 0, 0, 0, 0 };

	dm9601_set_multicast(dev, mcast0);
	dm9601_set_multicast(dev, mcast1);
	dm9601_set_multicast(dev, mcast2);
	dm9601_set_multicast(dev, mcast3);

	return 0;
}

struct eth_device;
static int dm9601_set_mac_address(struct eth_device *eth)
{
	struct ueth_data *dev = (struct ueth_data *)eth->priv;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buf, ETH_ALEN);

	if (!is_valid_ethaddr(eth->enetaddr)) {
		printf("not setting invalid mac address %pM\n", eth->enetaddr);
		return -EINVAL;
	}

	memcpy(buf, eth->enetaddr, ETH_ALEN);
	dm_write(dev, DM_PHY_ADDR, ETH_ALEN, buf);

	return 0;
}


static int dm9601_read_mac_address(struct eth_device *eth)
{
	struct ueth_data *dev = (struct ueth_data *)eth->priv;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buf, ETH_ALEN);

	/* read MAC */
	if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, buf) < 0) {
		printf("dm9601: Error reading MAC address\n");
		return -ENODEV;
	}

	memcpy(eth->enetaddr, buf, ETH_ALEN);

	return 0;
}

static int dm9601_init(struct eth_device *eth, struct bd_info *bd)
{
	struct ueth_data	*dev = (struct ueth_data *)eth->priv;
	int timeout = 0;
	int link_detected;

	debug("** %s()\n", __func__);

	mii_nway_restart(dev);
	/*dm_set_autoneg(dev);*/

	dm9601_link_reset(dev);

#define TIMEOUT_RESOLUTION 50	/* ms */
	do {
		link_detected = dm9601_mdio_read(dev, dev->phy_id, MII_BMSR) &
			BMSR_LSTATUS;
		if (!link_detected) {
			if (timeout == 0)
				printf("Waiting for Ethernet connection... ");
			udelay(TIMEOUT_RESOLUTION * 1000);
			timeout += TIMEOUT_RESOLUTION;
		}
	} while (!link_detected && timeout < PHY_CONNECT_TIMEOUT);
	if (link_detected) {
		if (timeout != 0)
			printf("done.\n");
	} else {
		printf("unable to connect.\n");
		goto out_err;
	}
#undef TIMEOUT_RESOLUTION

	return 0;

out_err:
	printf("dm9601: Error: unable to init device.\n");
	return -1;
}


static int dm9601_send(struct eth_device *eth, void *packet, int length)
{
	struct ueth_data *dev = (struct ueth_data *)eth->priv;
	int err;
	u16 packet_len;
	int actual_len;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, msg, PKTSIZE + sizeof(packet_len));

	debug("** %s(), len %d\n", __func__, length);

	/* format:
	   b1: packet length low
	   b2: packet length high
	   b3..n: packet data
	*/

	packet_len = length;
	cpu_to_le16s(&packet_len);

	memcpy(msg, &packet_len, sizeof(packet_len));
	memcpy(msg + sizeof(packet_len), (void *)packet, length);

	err = usb_bulk_msg(dev->pusb_dev,
				usb_sndbulkpipe(dev->pusb_dev, dev->ep_out),
				(void *)msg,
				length + sizeof(packet_len),
				&actual_len,
				USB_BULK_SEND_TIMEOUT);
	debug("Tx: len = %u, actual = %u, err = %d\n",
			length + sizeof(packet_len), actual_len, err);

	return err;
}


static int dm9601_recv(struct eth_device *eth)
{
	struct ueth_data *dev = (struct ueth_data *)eth->priv;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, recv_buf, AX_RX_URB_SIZE);
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, pkt, PKTSIZE);
	unsigned char *buf_ptr;
	int err;
	int actual_len;
	u16 packet_len;
	u8 status;

	debug("** %s()\n", __func__);

	/* format:
	   b1: rx status
	   b2: packet length (incl crc) low
	   b3: packet length (incl crc) high
	   b4..n-4: packet data
	   bn-3..bn: ethernet crc
	 */

	err = usb_bulk_msg(dev->pusb_dev,
				usb_rcvbulkpipe(dev->pusb_dev, dev->ep_in),
				(void *)recv_buf,
				AX_RX_URB_SIZE,
				&actual_len,
				USB_BULK_RECV_TIMEOUT);
	debug("Rx: len = %u, actual = %u, err = %d\n", AX_RX_URB_SIZE,
		actual_len, err);
	if (err != 0) {
		printf("Rx: failed to receive\n");
		return -1;
	}
	if (actual_len > AX_RX_URB_SIZE) {
		printf("Rx: received too many bytes %d\n", actual_len);
		return -1;
	}

	buf_ptr = recv_buf;
	while (actual_len > 0) {
		/*
		 * First byte contains packet status.
		 */
		if (actual_len < sizeof(status)) {
			debug("Rx: incomplete packet length (status)\n");
			return -1;
		}
		status = buf_ptr[0];
		buf_ptr += sizeof(status);
		actual_len -= sizeof(status);

		if (unlikely(status & 0xbf)) {
			printf("Rx: packet status failure: %d\n", (int)status);
			/*
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
			*/
			return -1;
		}

		/*
		 * 2nd and 3rd bytes contain the length of the actual data.
		 * Extract the length of the data.
		 */
		if (actual_len < sizeof(packet_len)) {
			debug("Rx: incomplete packet length (size)\n");
			return -1;
		}
		memcpy(&packet_len, buf_ptr, sizeof(packet_len));
		le16_to_cpus(&packet_len);
		packet_len -= 4;
		buf_ptr += sizeof(packet_len);
		actual_len -= sizeof(packet_len);

		if (packet_len > actual_len) {
			printf("Rx: too large packet: %d, actual: %d\n", packet_len, actual_len);
			return -1;
		}

		/* Notify net stack */
		memcpy(pkt, buf_ptr, packet_len);
		net_process_received_packet(pkt, packet_len);

		/* Adjust for next iteration. Packets are padded to 16-bits */
		/*if (packet_len & 1)
			packet_len++;*/
		actual_len -= (packet_len + 4);
		buf_ptr += packet_len + 4;
	}

	return err;
}


static void dm9601_halt(struct eth_device *eth)
{
	debug("** %s()\n", __func__);
}


void dm9601_eth_before_probe(void)
{
	debug("** %s()\n", __func__);
}


int dm9601_eth_probe(struct usb_device *dev, unsigned int ifnum,
		struct ueth_data* ss)
{
	struct usb_interface *iface;
	struct usb_interface_descriptor *iface_desc;
	int ep_in_found = 0, ep_out_found = 0;
	int i;

	debug("\n****************** %s *******************\n", __func__);

	/* let's examine the device now */
	iface = &dev->config.if_desc[ifnum];
	iface_desc = &dev->config.if_desc[ifnum].desc;

	for (i = 0; products[i].vendor != 0; i++) {
		debug("\n******************dev->descriptor.idVendor = %#04x *******************\n", dev->descriptor.idVendor);
		debug("\n******************dev->descriptor.idProduct = %#04x *******************\n", dev->descriptor.idProduct);
		if (dev->descriptor.idVendor == products[i].vendor &&
		    dev->descriptor.idProduct == products[i].product)
			/* Found a supported dongle */
			break;
	}

	if (products[i].vendor == 0)
		{
		debug("\n****************** products[i].vendor == 0 *******************\n");
		return 0;
		}

	memset(ss, 0, sizeof(struct ueth_data));

	/* At this point, we know we've got a live one */
	debug("\n\nUSB Ethernet device detected: %#04x:%#04x\n",
	      dev->descriptor.idVendor, dev->descriptor.idProduct);

	/* Initialize the ueth_data structure with some useful info */
	ss->ifnum = ifnum;
	ss->pusb_dev = dev;
	ss->subclass = iface_desc->bInterfaceSubClass;
	ss->protocol = iface_desc->bInterfaceProtocol;

	/*
	 * We are expecting a minimum of 3 endpoints - in, out (bulk), and
	 * int. We will ignore any others.
	 */
	for (i = 0; i < iface_desc->bNumEndpoints; i++) {
		/* is it an BULK endpoint? */
		if ((iface->ep_desc[i].bmAttributes &
		     USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) {
			u8 ep_addr = iface->ep_desc[i].bEndpointAddress;
			if (ep_addr & USB_DIR_IN) {
				if (!ep_in_found) {
					ss->ep_in = ep_addr &
						USB_ENDPOINT_NUMBER_MASK;
					ep_in_found = 1;
				}
			} else {
				if (!ep_out_found) {
					ss->ep_out = ep_addr &
						USB_ENDPOINT_NUMBER_MASK;
					ep_out_found = 1;
				}
			}
		}

		/* is it an interrupt endpoint? */
		if ((iface->ep_desc[i].bmAttributes &
		    USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT) {
			ss->ep_int = iface->ep_desc[i].bEndpointAddress &
				USB_ENDPOINT_NUMBER_MASK;
			ss->irqinterval = iface->ep_desc[i].bInterval;
		}
	}
	debug("Endpoints In %d Out %d Int %d\n",
		  ss->ep_in, ss->ep_out, ss->ep_int);

	/* Do some basic sanity checks, and bail if we find a problem */
	if (usb_set_interface(dev, iface_desc->bInterfaceNumber, 0) ||
	    !ss->ep_in || !ss->ep_out || !ss->ep_int) {
		printf("Problems with device\n");
		return 0;
	}

	return 1;
}


int dm9601_eth_get_info(struct usb_device *usb_dev, struct ueth_data *ss,
				struct eth_device *eth)
{
	u8 id = 0xff;
	u8 mcast0[DM_MCAST_SIZE] = { 0x0 };

	debug("\n%s\n", __func__);

	if (!eth) {
		printf("%s: missing parameter.\n", __func__);
		return 0;
	}

	sprintf(eth->name, "%s%d", DM9601_BASE_NAME, 0 /*curr_eth_dev++*/);
	eth->init = dm9601_init;
	eth->send = dm9601_send;
	eth->recv = dm9601_recv;
	eth->halt = dm9601_halt;
#ifdef CONFIG_MCAST_TFTP
	/*
	eth->mcast = dm9601_mcast(struct eth_device *, const u8 *enetaddr, u8 set);
	*/
#endif
	eth->write_hwaddr = dm9601_set_mac_address;
	eth->priv = ss;

	/* reset */
	dm_write_reg(ss, DM_NET_CTRL, 1);
	udelay(20);

	/* read MAC */
	if (dm9601_read_mac_address(eth))
		return 0;
	printf("\nDavicom DM96xx MAC address is %pM\n", eth->enetaddr);

	/*
	 * Overwrite the auto-generated address only with good ones.
	 */
	if (!is_valid_ethaddr(eth->enetaddr)) {
		printf("dm9601: No valid MAC address in EEPROM, using %pM\n",
			eth->enetaddr);
		/*__dm9601_set_mac_address(ss);*/
	}

	if (dm_read_reg(ss, DM_CHIP_ID, &id) < 0) {
		printf("dm9601: Error reading chip ID\n");
		return 0;
	}

	debug("Chip ID = %d\n", id);
	/* put dm9620 devices in dm9601 mode */
	if (id == ID_DM9620) {
		u8 mode;

		if (dm_read_reg(ss, DM_MODE_CTRL, &mode) < 0) {
			printf("dm9601: Error reading MODE_CTRL\n");
			return 0;
		}
		dm_write_reg(ss, DM_MODE_CTRL, mode & 0x7f);
	}

	/* power up phy */
	dm_write_reg(ss, DM_GPR_CTRL, 1);
	dm_write_reg(ss, DM_GPR_DATA, 0);

	/* receive broadcast packets */
	dm9601_set_multicast(ss, mcast0);

	dm9601_mdio_read(ss, ss->phy_id, MII_BMSR);
	dm9601_mdio_write(ss, ss->phy_id, MII_BMCR, BMCR_RESET);
	dm9601_mdio_write(ss, ss->phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(ss);

	return 1;
}
#endif

#ifdef CONFIG_DM_ETH
static int dm9601_init(struct ueth_data *dev)
{
    int timeout = 0;
    int link_detected;

	debug("\n----> %s()\n", __func__);

    /* dm_set_autoneg(dev); */
    /* dm9601_link_reset(dev); */

#define TIMEOUT_RESOLUTION 50   /* ms */
    do {
        link_detected = dm9601_mdio_read(dev, dev->phy_id, MII_BMSR) & BMSR_LSTATUS;
        if (!link_detected) {
            if (timeout == 0)
                printf("Waiting for Ethernet connection... ");

            udelay(TIMEOUT_RESOLUTION * 1000);
            timeout += TIMEOUT_RESOLUTION;
        }
    } while (!link_detected && timeout < PHY_CONNECT_TIMEOUT);

    if (link_detected) {
        if (timeout != 0)
            printf("done.\n");

    } else {
        printf("unable to connect.\n");
        goto out_err;

    }

    mdelay(25);
    return 0;

out_err:
    printf("dm9601: Error: unable to init device.\n");
    return -1;

}

static int dm9601_bind(struct udevice *udev)
{
	int ret = 0;
	u8 mac[ETH_ALEN], id;
	u8 env_enetaddr[ETH_ALEN];

    struct eth_pdata *pdata = dev_get_platdata(udev);
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	/* reset */
	dm_write_reg(dev, DM_NET_CTRL, 1);
	udelay(20);

    /* set env addr */
	eth_env_get_enetaddr_by_index("eth", udev->seq, env_enetaddr);
	if (!is_zero_ethaddr(env_enetaddr)) {
        dm9601_set_mac_address(dev, env_enetaddr);
		memcpy(pdata->enetaddr, env_enetaddr, ETH_ALEN);
    } else {
        /* read MAC */
        if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, mac) < 0) {
            printf("Error reading MAC address\n");
            ret = -ENODEV;
            goto out;
        }

        /*
         * Overwrite the auto-generated address only with good ones.
         */
        if (is_valid_ethaddr(mac))
            memcpy(pdata->enetaddr, mac, ETH_ALEN);
        else {
            /* set a default mac */
            const u8 mac_addr[ETH_ALEN] = {0x00, 0xD8, 0x1C, 0x04, 0x55, 0x60};

            printf("dm9601: No valid MAC address in EEPROM, using %pM\n", mac);
            dm9601_set_mac_address(dev, (u8 *)mac_addr);
            memcpy(pdata->enetaddr, mac_addr, ETH_ALEN);
        }
    }

	if (dm_read_reg(dev, DM_CHIP_ID, &id) < 0) {
		printf("Error reading chip ID\n");
		ret = -ENODEV;
		goto out;
	}

    debug("id: %s\n", ID_DM9601 ? "DM9601" : "DM9621");

	/* put dm9620 devices in dm9601 mode */
	if (id == ID_DM9620) {
		u8 mode;

		if (dm_read_reg(dev, DM_MODE_CTRL, &mode) < 0) {
			printf("Error reading MODE_CTRL\n");
			ret = -ENODEV;
			goto out;
		}
		dm_write_reg(dev, DM_MODE_CTRL, mode & 0x7f);
	}

	/* power up phy */
	dm_write_reg(dev, DM_GPR_CTRL, 1);
	dm_write_reg(dev, DM_GPR_DATA, 0);

	/* receive broadcast packets */
	dm9601_set_multicast(dev);

	dm9601_mdio_write(dev, dev->phy_id, MII_BMCR, BMCR_RESET);
	dm9601_mdio_write(dev, dev->phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(dev);

out:
	return ret;
}


/*
 * dump_msg() - dump content of memory to hex
 * @ptr:    start address of memory to dump
 * @len:    number of bytes to dump
 */
static void dump_msg(uint8_t *ptr, int len)
{
#ifdef DEBUG
    int i = 0, j = 0;

    debug("Dump:\n");
    for (i = 0; i < len; i++) {
        debug("%02x ", ptr[i]);

        if((i + 1) % 8 == 0) {
            debug(" ");
        }
        if ((i + 1) % 16 == 0) {
            debug("\n");
            j = i + 1;
        }
    }

    if(j < i) {
        debug("\n");
    }
#endif
}


static int dm9601_eth_start(struct udevice *udev)
{
    /* struct eth_pdata *pdata = dev_get_platdata(udev); */
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	debug("\n----> %s()\n", __func__);
    return dm9601_init(dev);
}

void dm9601_eth_stop(struct udevice *udev)
{
	debug("\n----> %s()\n", __func__);
}

int dm9601_eth_send(struct udevice *udev, void *packet, int length)
{
    int err;
    u16 packet_len;
    int actual_len;
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;
    struct usb_device *usb_dev = dev->pusb_dev;
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, msg, PKTSIZE + DM_TX_OVERHEAD);

	debug("\n----> %s()\n", __func__);

	/* format:
	   b1: packet length low
	   b2: packet length high
	   b3..n: packet data
	*/

    packet_len = length;
    cpu_to_le16s(&packet_len);

    memcpy(msg, &packet_len, DM_TX_OVERHEAD);
    memcpy(msg + DM_TX_OVERHEAD, (void *)packet, length);

    err = usb_bulk_msg(usb_dev,
                       usb_sndbulkpipe(usb_dev, dev->ep_out),
                       (void *)msg,
                       length + sizeof(packet_len),
                       &actual_len,
                       USB_BULK_SEND_TIMEOUT);
    debug("Tx: len = %u, actual = %u, err = %d\n",
          length + sizeof(packet_len), actual_len, err);

    dump_msg(msg, actual_len);
    dump_msg(msg + 2, actual_len - 2);

    return err;
}

int dm9601_eth_recv(struct udevice *udev, int flags, uchar **packetp)
{
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *ueth = &priv->ueth;
    uint8_t *ptr;
    int ret = 0;
    int len = 0;
    uint8_t status = 0;
    uint16_t packet_len = 0;
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, pkt, PKTSIZE);

	debug("\n----> %s()\n", __func__);

    len = usb_ether_get_rx_bytes(ueth, &ptr);
    debug("----> %s: first try, len=%d\n", __func__, len);
    if (!len) {
        if (!(flags & ETH_RECV_CHECK_DEVICE))
            return -EAGAIN;
        ret = usb_ether_receive(ueth, DM9601_RX_URB_SIZE);
        if (ret == -EAGAIN)
            return ret;

        len = usb_ether_get_rx_bytes(ueth, &ptr);
        debug("---->%s: second try, len=%d\n", __func__, len);
    }

    debug("---->Rx: len = %u, actual = %u, err = %d\n", DM9601_RX_URB_SIZE, len, ret);
    dump_msg(ptr, len);
    dump_msg(ptr + 3, len - 3);

	/* format:
	   b1: rx status
	   b2: packet length (incl crc) low
	   b3: packet length (incl crc) high
	   b4..n-4: packet data
	   bn-3..bn: ethernet crc
	 */

    if (len < DM_RX_OVERHEAD) {
        debug("Rx: incomplete packet length\n");
        goto err;
    }

    status = ptr[0];
    if (unlikely(status & 0xbf)) {
        printf("Rx: packet status failure: %d\n", status);
        goto err;
    }

    /* real data length, need sub 4 bytes crc tail */
    packet_len = (ptr[1] | (ptr[2] << 8)) - 4;
    if (packet_len > len - DM_RX_OVERHEAD) {
        debug("Rx: too large packet: %d\n", packet_len);
        goto err;
    }

    debug("---> packet_len = %d, len = %d\n", packet_len, len);
    memcpy(pkt, ptr + 3, packet_len);   /* 3 bytes header */
    memcpy(ptr, pkt, packet_len);       /* copy to dev->rxbuf */

    /*
     * MUST RETURN ALIGNED MEMORY, because checksum use LDRH !!!
     * DO NOT return `ptr` allocated by ALLOC_CACHE_ALIGN_BUFFER,
     * which expand to an array on stack.
     * Here ptr --> dev->rxbuf (not array on stack).
     */
    *packetp = ptr;
    return packet_len;

err:
    /* drop all in buffer */
    usb_ether_advance_rxbuf(ueth, -1);
    return -EINVAL;
}

static int dm9601_free_pkt(struct udevice *udev, uchar *packet, int packet_len)
{
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	debug("\n----> %s()\n", __func__);

    packet_len = ALIGN(packet_len, 2);
    usb_ether_advance_rxbuf(dev, DM_RX_OVERHEAD + packet_len);

    return 0;
}

int dm9601_write_hwaddr(struct udevice *udev)
{
    struct eth_pdata *pdata = dev_get_platdata(udev);
    struct dm9601_private *priv = dev_get_priv(udev);

    debug("---->set mac addr <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          pdata->enetaddr[0], pdata->enetaddr[1], pdata->enetaddr[2],
          pdata->enetaddr[3], pdata->enetaddr[4], pdata->enetaddr[5]);
    return dm9601_set_mac_address(&priv->ueth, pdata->enetaddr);
}

static int dm9601_eth_probe(struct udevice *udev)
{
    struct eth_pdata *pdata = dev_get_platdata(udev);
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;
    int ret;

	debug("\n----> %s()\n", __func__);

    priv->flags = udev->driver_data;
    ret = usb_ether_register(udev, dev, DM9601_RX_URB_SIZE);
    if (ret) {
        printf("usb ether register failed! ret = %d\n", ret);
        return ret;
    }

    /* Do a reset in order to get the MAC address from HW */
    if(dm9601_bind(udev)) {
        printf("basic init failed!\n");
        goto err;
    }

    /* Get the MAC address */
    dm9601_read_mac_address(dev, pdata->enetaddr);
    debug("---->get mac addr: <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          pdata->enetaddr[0], pdata->enetaddr[1], pdata->enetaddr[2],
          pdata->enetaddr[3], pdata->enetaddr[4], pdata->enetaddr[5]);

    return 0;

err:
    return usb_ether_deregister(dev);
}

static const struct eth_ops dm9601_eth_ops = {
    .start          = dm9601_eth_start,
    .send           = dm9601_eth_send,
    .recv           = dm9601_eth_recv,
    .free_pkt       = dm9601_free_pkt,
    .stop           = dm9601_eth_stop,
    .write_hwaddr   = dm9601_write_hwaddr,
};

U_BOOT_DRIVER(dm9601_eth) = {
    .name = "DM9601",
    .id = UCLASS_ETH,
    .probe = dm9601_eth_probe,
    .ops = &dm9601_eth_ops,
    .priv_auto_alloc_size = sizeof(struct dm9601_private),
    .platdata_auto_alloc_size = sizeof(struct eth_pdata),
};

static const struct usb_device_id dm9601_eth_id_table[] = {
	{ USB_DEVICE(0x07aa, 0x9601), },    /* Corega FEther USB-TXC */
	{ USB_DEVICE(0x0a46, 0x9601), },    /* Davicom USB-100 */
	{ USB_DEVICE(0x0a46, 0x6688), },    /* ZT6688 USB NIC */
	{ USB_DEVICE(0x0a46, 0x0268), },    /* ShanTou ST268 USB NIC */
	{ USB_DEVICE(0x0a46, 0x8515), },    /* ADMtek ADM8515 USB NIC */
	{ USB_DEVICE(0x0a47, 0x9601), },    /* Hirose USB-100 */
	{ USB_DEVICE(0x0fe6, 0x8101), },	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0fe6, 0x9700), },	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9000), },	/* DM9000E */
	{ USB_DEVICE(0x0a46, 0x9620), },	/* DM9620 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9621), },	/* DM9621A USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9622), },	/* DM9622 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x0269), },	/* DM962OA USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x1269), },	/* DM9621A USB to Fast Ethernet Adapter */
	{},			// END
};

U_BOOT_USB_DEVICE(dm9601_eth, dm9601_eth_id_table);
#endif

