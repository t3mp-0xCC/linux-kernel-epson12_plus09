/*
 *  MFP09 Ethernet device driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms and conditions of the GNU General Public License,
 *  version 2, as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *  You should have received a copy of the GNU General Public License along 
with
 *  this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  The full GNU General Public License is included in this distribution in
 *  the file called "COPYING".
 */

/*
 *  MFP09 Ethernet device driver Ver1.02 
 *
 *  COPYRIGHT (C) 2008-2014 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 */
#include <linux/version.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include "mfp09_eth.h"

MODULE_LICENSE("GPL");

#define MPF09_DE_1
#define SOC09S		(1)

//#define MFP09_DEBUG_PRINT	(1)
//#define DEBUG_L1

#if defined(MFP09_DEBUG_PRINT)
  #define DPRINTK(args...)	  printk(KERN_ERR args)
#else
  #define DPRINTK(args...)
#endif

#if defined(DEBUG_L1)
  #define DEBUG_L1PRINT(args...) printk(KERN_ERR args)
#else
  #define DEBUG_L1PRINT(args...)
#endif

#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
#define COPR_EXESIZE 0x20
#define COPR_ADDRESS_MASK (0xffffffff & ~(COPR_EXESIZE - 1))
inline static void eth_mb(__u32 addr, __u32 len)
{
	__u32 oprnum = 0;

	if (len == 0)
		return;

	if ((addr & (COPR_EXESIZE - 1)) != 0) {
		len += (addr & (COPR_EXESIZE - 1));
		addr &= COPR_ADDRESS_MASK;
	}

	for (oprnum = (len / COPR_EXESIZE) + ((len % COPR_EXESIZE) ? 1 : 0); oprnum != 0; oprnum --) {
		__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 1" : : "r" (addr) : "memory","cc");
		addr += COPR_EXESIZE;
	}
}
#endif


static int mdio_bit_read(long ioaddr);
static void mdio_bit_write(long ioaddr, int bit);
static void mdio_bus_release(long ioaddr);
static void mdio_sync(long ioaddr, int bits);
static int mdio_read(struct net_device *dev, int phy_id, int location);
static void mdio_write(struct net_device *dev, int phy_id, int location, int value);
static void mfp09__eth_reset(struct net_device *ndev);
static int mfp09_eth_phy_start(struct net_device *ndev);
static void mfp09_eth_adjust_link(struct net_device *ndev);
static int mfp09_eth_do_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);
static int mfp09_eth_drv_probe(struct platform_device *pdev);
static int mfp09_eth_drv_remove(struct platform_device *pdev);
static int mfp09_eth_open(struct net_device *ndev);
static int mfp09_eth_close(struct net_device *ndev);
static int mfp09_eth_dev_init(struct net_device *ndev);
static struct net_device_stats *mfp09_eth_get_stats(struct net_device *ndev);
static irqreturn_t mfp09_eth_interrupt(int irq, void *netdev);
static void mfp09_eth_error(struct net_device *ndev, int intr_status);
static int mfp09_eth_ring_init(struct net_device *ndev);
static void mfp09_eth_ring_format(struct net_device *ndev);
static void mfp09_eth_ring_free(struct net_device *ndev);
static int mfp09_eth_txfree(struct net_device *ndev);
static int mfp09_eth_start_xmit(struct sk_buff *skb, struct net_device *ndev);
static void mfp09_eth_tx_timeout(struct net_device *ndev);
static int mfp09_eth_rx(struct net_device *ndev);
static void mfp09_eth_set_multicast_list(struct net_device *ndev);
static void mfp09_eth_timer(unsigned long data);
static int __init mfp09_eth_init(void);
static void __exit mfp09_eth_cleanup(void);
static void mii_watchdog(struct net_device *dev);
static void mii_start(struct net_device *dev);
static void mii_initial_setup(struct net_device *dev, unsigned int phy_id);
static int link_wait(struct net_device *dev);
static int set_eth_autonego(struct net_device *dev);
static int mfp09_eth_up(struct net_device *dev);
unsigned long get_link_status(struct net_device *dev);
static u32 mfp09_get_link(struct net_device *dev);
void   mfp09_get_supported_features(struct net_device *dev,u32 * supported);
void  mfp09_get_advertised_features(struct net_device *dev,u32 * advertised);
static int mfp09_ethtool_gset (struct net_device *netdev, struct ethtool_cmd *ecmd);
static int mfp09_ethtool_sset (struct net_device *netdev, struct ethtool_cmd *ecmd);
int mfp09_set_spd_dplx (struct  mfp09_eth_private *adapter, uint16_t spddplx);
void set_ethtool_ops (struct net_device *netdev);

#define NEW_ETH_IO_SIZE (SZ_16K)
/* RXLENGTH_BOUNDARY must be pow(2, n) */
#define RXLENGTH_BOUNDARY	(32)
#define RXADDRESS_BOUNDARY	(32)

//#define DMA_PADDING		(1)

void __iomem *ioremap_area3(
	unsigned long phy_addr,
	size_t size
)
{
	void __iomem *result;

	result = ioremap_nocache(phy_addr,size);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	printk(KERN_INFO
 		"remap_area3 for mfp09_eth phy_addr[x%x] -> map_addr[x%x]\n",
		(unsigned int)phy_addr,(unsigned int)result
		);
#endif
	return result;
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
static void update_mac_address(struct net_device *ndev)
{
       u32 ioaddr = ndev->base_addr;
DPRINTK("update_mac_address\n");
#if !defined(CONFIG_MFP09_DUMMY)
       outl((ndev->dev_addr[0] << 24) | (ndev->dev_addr[1] << 16) |
                 (ndev->dev_addr[2] << 8) | (ndev->dev_addr[3]),
                 ioaddr + CXR24);
       outl((ndev->dev_addr[4] << 8) | (ndev->dev_addr[5]),
                 ioaddr + CXR25);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
}

/*
 * Get MAC address from SuperH MAC address register
 *
 * SuperH's Ethernet device doesn't have 'ROM' to MAC address.
 * This driver get MAC address that use by bootloader(U-boot or sh-ipl+g).
 * When you want use this device, you must set MAC address in bootloader.
 *
 */
static void __init read_mac_address(struct net_device *ndev)
{
       u32 ioaddr = ndev->base_addr;
 DPRINTK("2ndev->dev_addr[0]:before\n");

#if !defined(CONFIG_MFP09_DUMMY)
#if 1
       outl(0x000048D4,ioaddr + CXR24);
       outl(0x00E6,ioaddr + CXR25);
#endif
       ndev->dev_addr[0] = (inl(ioaddr + CXR24) >> 24);;
       ndev->dev_addr[1] = (inl(ioaddr + CXR24) >> 16) & 0xFF;
       ndev->dev_addr[2] = (inl(ioaddr + CXR24) >> 8) & 0xFF;
       ndev->dev_addr[3] = (inl(ioaddr + CXR24) & 0xFF);
       ndev->dev_addr[4] = (inl(ioaddr + CXR25) >> 8) & 0xFF;
       ndev->dev_addr[5] = (inl(ioaddr + CXR25) & 0xFF);
#else
       ndev->dev_addr[0] = 00;
       ndev->dev_addr[1] = 00;
       ndev->dev_addr[2] = 0x48;
       ndev->dev_addr[3] = 0xd4;
       ndev->dev_addr[4] = 0x00;
       ndev->dev_addr[5] = 0xe6;
#endif	/* !defined(CONFIG_MFP09_DUMMY) */
 DPRINTK("2ndev->dev_addr[0]:after\n");
}

#define MII_TIMEOUT	2*HZ

/* Set iff a MII transceiver on any interface requires mdio preamble.
   This only set with the original DP83840 on older 3c905 boards, so the extra
   code size of a per-interface flag is not worthwhile. */

static const char mii_preamble_required = 1;

/* MII transceiver control section.
   Read and write the MII registers using software-generated serial
   MDIO protocol.  See the MII specifications or DP83840A data sheet
   for details. */

/* The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues. */
#if 0
#define mdio_delay() inl(mdio_addr)
#else
/* in original code, this value was 25 */
#define MDIO_MICROSEC_DELAY		(1)
#define mdio_delay() udelay(MDIO_MICROSEC_DELAY)
#endif

#define MDIO_SHIFT_CLK	0x01
#define MDIO_DIR_WRITE	0x04
#define MDIO_DATA_WRITE0	(0x00 | MDIO_DIR_WRITE)
#define MDIO_DATA_WRITE1	(0x02 | MDIO_DIR_WRITE)
#define MDIO_DATA_READ	0x02
#define MDIO_ENB_IN		0x00
#define MDIO_DATA_0		0
#define MDIO_DATA_1		1

#define	PHY_ID		0x01		/* MFP09 */

/* Generate the preamble required for initial synchronization and
   a few older transceivers. */
#if !defined(CONFIG_MFP09_DUMMY)
static int mdio_bit_read(long ioaddr)
{
	long mdio_addr = ioaddr + CXR23;
	int bit;

	outl(CXR23_MDC, mdio_addr);
	mdio_delay();
	bit = inl(mdio_addr);
	outl(0, mdio_addr);
	mdio_delay();
	if((bit & CXR23_MDI) == CXR23_MDI)
		return(MDIO_DATA_1);
	else
		return (MDIO_DATA_0);
}

static void mdio_bit_write(long ioaddr, int bit)
{
	long mdio_addr = ioaddr + CXR23;

    if(bit == MDIO_DATA_1) {	/* "1" */
		outl((CXR23_MDO | CXR23_MMD), mdio_addr);
		mdio_delay();
		outl((CXR23_MDO | CXR23_MMD | CXR23_MDC), mdio_addr);
		mdio_delay();
	} else {					/* "0" */
		outl(CXR23_MMD, mdio_addr);
		mdio_delay();
		outl((CXR23_MMD | CXR23_MDC), mdio_addr);
		mdio_delay();
	}
}

static void mdio_bus_release(long ioaddr)
{
	long mdio_addr = ioaddr + CXR23;

	outl(0, mdio_addr);
	mdio_delay();
	outl(CXR23_MDC, mdio_addr);
	mdio_delay();
	outl(0, mdio_addr);
	mdio_delay();
}

static void mdio_sync(long ioaddr, int bits)
{
	/* Establish sync by sending at least 32 logic ones. */
	while(-- bits >= 0)
		mdio_bit_write(ioaddr, MDIO_DATA_1);
}
#endif /* !defined(CONFIG_MFP09_DUMMY) */

#if defined(CONFIG_MFP09_DUMMY)
static int s_dummy_phy_register[25];
#endif

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
#if !defined(CONFIG_MFP09_DUMMY)
	unsigned long flags;
	struct mfp09_eth_private *mdp =
	       (struct mfp09_eth_private *)netdev_priv(dev);

	int i;
	long ioaddr = dev->base_addr;
	int read_cmd = (0xf6 << 10) | (phy_id << 5) | location;
	unsigned int retval = 0;

	spin_lock_irqsave(&mdp->mdio_lock, flags);

	if(mii_preamble_required)
		mdio_sync(ioaddr, 32);

	/* Shift the read command bits out. */
	for(i = 14; i >= 0; i--) {
		int dataval = (read_cmd&(1<<i)) ? MDIO_DATA_1 : MDIO_DATA_0;
		mdio_bit_write(ioaddr, dataval);
	}
	/* Read the two transition, 16 data, and wire-idle bits. */
	for(i = 19; i > 0; i--) {
		retval = (retval << 1) | (mdio_bit_read (ioaddr));
	}
	mdio_bus_release(ioaddr);
	spin_unlock_irqrestore(&mdp->mdio_lock, flags);
	return retval & 0x20000 ? 0xffff : retval>>1 & 0xffff;
#else
	return s_dummy_phy_register[location] & 0xffff;
#endif
}

static void mdio_write(struct net_device *dev, int phy_id, int location, int value)
{
#if !defined(CONFIG_MFP09_DUMMY)
	unsigned long flags;
	struct mfp09_eth_private *mdp =
	       (struct mfp09_eth_private *)netdev_priv(dev);

	long ioaddr = dev->base_addr;
	int write_cmd = 0x50020000 | (phy_id << 23) | (location << 18) | value;
	int i;
	DPRINTK("%s: mdio_write location(%d) value(%08x)\n", CARDNAME, location, value);
	spin_lock_irqsave(&mdp->mdio_lock, flags);

	if(mii_preamble_required)
		mdio_sync(ioaddr, 32);

	/* Shift the command bits out. */
	for(i = 31; i >= 0; i--) {
		int dataval = (write_cmd&(1<<i)) ? MDIO_DATA_1 : MDIO_DATA_0;
		mdio_bit_write(ioaddr, dataval);
	}

	if(value&BMCR_RESET) {
		/* PHY RESET, wait Register Reset cycle(160ms RTL8201F hardware reset time */ 
		mdelay(160);
	}
	/* Leave the interface idle. */
	mdio_bus_release(ioaddr);
	spin_unlock_irqrestore(&mdp->mdio_lock, flags);
#else
	s_dummy_phy_register[location] = value;
#endif
	return;
}


static void mfp09__eth_reset(struct net_device *ndev)
{
       u32 ioaddr = ndev->base_addr;
#if !defined(CONFIG_MFP09_DUMMY)
       outl(inl(ioaddr + CXR0) | CXR0_SRST, ioaddr + CXR0);
       mdelay(30);
       outl(inl(ioaddr + CXR0) & ~CXR0_SRST, ioaddr + CXR0);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
}

/* PHY control start function */
static int mfp09_eth_phy_start(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
	unsigned long ul;

       mdp->link = 0;
       mdp->speed = 0;
       mdp->duplex = -1;

#if defined(SOC09S)
/*  change to use MAC I/F instead of send data by PHY's port */
	ul = mdio_read(ndev, mdp->mii.phy_id, MII_BMCR);
	ul |= (BMCR_RESET | BMCR_ANRESTART | BMCR_FULLDPLX); 
	mdio_write(ndev, mdp->mii.phy_id, MII_BMCR, (int)ul);
	mdelay(50);
	ul &= ~BMCR_RESET;
	mdio_write(ndev, mdp->mii.phy_id, MII_BMCR, (int)ul);
#else
       /* Initalize PHY board */
       ul = inl(0xF0200000 + PHYCR);
	ul = ul & ~0x00000020;	/// Repeter OFF
       outl((ul | 0x00008003), 0xF0200000 + PHYCR);	// 100Mbps/full
#endif
	set_eth_autonego(ndev);

       return 0;
}

/* PHY state control function */
static void mfp09_eth_adjust_link(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr = ndev->base_addr;
       int new_state = 0;
		int phy_ctrl, phy_stat;
       int speed,duplex,link;
       u32 val;

		phy_ctrl = mdio_read(ndev, PHY_ID, PHY_CTRL);
		phy_stat = mdio_read(ndev, PHY_ID, PHY_STAT);
		
		speed = (phy_ctrl & PHY_C_SPEEDSL) ? 1:0;
		duplex = (phy_ctrl & PHY_C_DUPLEX) ? 1:0;
		link = (phy_stat & PHY_S_LINK) ? 1:0;

       if (link == 1) {
               if (duplex != mdp->duplex) {
                       new_state = 1;
                       mdp->duplex = duplex;
               }

               if (speed != mdp->speed) {
                       new_state = 1;
                       mdp->speed = speed;
               }
               if (mdp->link == 0) {
						val = inl(ioaddr + CXR20) & ~CXR20_TXF;
						val = mdp->duplex ? (val | CXR20_DM) : (val & ~CXR20_DM);
#if !defined(CONFIG_MFP09_DUMMY)
                       outl(val , ioaddr + CXR20);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
                      new_state = 1;
                       mdp->link = link;
                       netif_carrier_on(ndev);
                       netif_start_queue(ndev);
               }
       } else if (mdp->link) {
               new_state = 1;
               mdp->link = 0;
               mdp->speed = 0;
               mdp->duplex = -1;
               netif_stop_queue(ndev);
               netif_carrier_off(ndev);
       }
}

/* ioctl to device funciotn*/
#define SIOCPHYPWOFF    SIOCDEVPRIVATE
#define SIOCPHYPWON     SIOCDEVPRIVATE + 1
static int mfp09_eth_do_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       int ret = 0, value = 0;

        switch (cmd) {
        case SIOCPHYPWOFF:
		if (netif_running(ndev))
			return -EBUSY;
                value = mdio_read(ndev, mdp->mii.phy_id, MII_BMCR);
                value |= BMCR_PDOWN;
                mdio_write(ndev, mdp->mii.phy_id, MII_BMCR, value);
                break;
        case SIOCPHYPWON:
		if (netif_running(ndev))
			return -EBUSY;
                value = mdio_read(ndev, mdp->mii.phy_id, MII_BMCR);
                value &= ~BMCR_PDOWN;
                mdio_write(ndev, mdp->mii.phy_id, MII_BMCR, value);
                break;
        default:
		if (!netif_running(ndev))
			return -EINVAL;
		ret = generic_mii_ioctl(&mdp->mii, if_mii(rq), cmd, NULL);
	}

	return ret;
}

static const struct net_device_ops mfp09_netdev_ops = {
	.ndo_open	= mfp09_eth_open,
	.ndo_start_xmit = mfp09_eth_start_xmit,
	.ndo_stop = mfp09_eth_close,
	.ndo_get_stats = mfp09_eth_get_stats,
	.ndo_set_multicast_list  = mfp09_eth_set_multicast_list,
	.ndo_do_ioctl = mfp09_eth_do_ioctl,
	.ndo_tx_timeout = mfp09_eth_tx_timeout,
	.ndo_set_mac_address = eth_mac_addr,
};

static int mfp09_eth_drv_probe(struct platform_device *pdev)
{
       int ret, i, devno = 0;
       struct resource *res;
       struct net_device *ndev = NULL;
       struct mfp09_eth_private *mdp;
DPRINTK("mfp09_eth_drv_probe 1\n");
       /* get base addr */
       res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
      if (unlikely(res == NULL)) {
              dev_err(&pdev->dev, "invalid resource\n");
               ret = -EINVAL;
               goto out;
       }

        /* request and get I/O region */
        if(!request_mem_region(res->start, NEW_ETH_IO_SIZE, CARDNAME)){
                ret = -EBUSY;
                printk("%s: Couldn't Get IO mem region for\n", CARDNAME);
                goto out;
        }

       ndev = alloc_etherdev(sizeof(struct mfp09_eth_private));
       if (!ndev) {
               DPRINTK(KERN_ERR "%s: could not allocate device.\n", CARDNAME);
               ret = -ENOMEM;
               goto out;
       }
       /* The mfp09 Ether-specific entries in the device structure. */
       ndev->base_addr = res->start;
       devno = pdev->id;
       if (devno < 0)
               devno = 0;

	mdp = netdev_priv(ndev);
	mdp->rx_ring = NULL;
	mdp->tx_ring = NULL;
       ndev->dma = -1;
       ndev->irq = platform_get_irq(pdev, 0);
       if (ndev->irq < 0) {
               ret = -ENODEV;
               goto out_release;
       }

       SET_NETDEV_DEV(ndev, &pdev->dev);

       /* Fill in the fields of the device structure with ethernet values. */
       ether_setup(ndev);

       spin_lock_init(&mdp->lock);
	spin_lock_init(&mdp->mdio_lock);
	/* self pointer for ethtool func */
	mdp->dev = ndev;

       /* set function */
       ndev->netdev_ops = &mfp09_netdev_ops;
       ndev->watchdog_timeo = TX_TIMEOUT;

       mdp->post_rx = POST_RX >> (devno << 1);
       mdp->post_fw = POST_FW >> (devno << 1);

       /* read and set MAC address */
       read_mac_address(ndev);

	/* ethtool interface add */
	set_ethtool_ops (ndev);
	mdp->autoneg = AUTONEG_ENABLE;
	mdp->forced_speed_duplex = mfp09_100_full;

       /* network device register */
       ret = register_netdev(ndev);
       if (ret)
               goto out_release;

	/* Phy init */
DPRINTK(KERN_ERR "mii_init\n");
	mii_initial_setup(ndev, PHY_ID);

DPRINTK(KERN_ERR "mii_start\n");
	/* mii start */
	mii_start(ndev);

       /* pritnt device infomation */
       DPRINTK(KERN_INFO "%s: %s at 0x%x, ",
              ndev->name, CARDNAME, (u32) ndev->base_addr);

       for (i = 0; i < 5; i++)
               DPRINTK(KERN_INFO "%2.2x:", ndev->dev_addr[i]);
       DPRINTK(KERN_INFO "%2.2x, IRQ %d.\n", ndev->dev_addr[i], ndev->irq);

	/* remap rx/tx descriptor table */
#if defined(SOC09S)
	mdp->rx_ring = ioremap_area3(EPSON09_ETH_RXDESC, SZ_8K);
	mdp->tx_ring = ioremap_area3(EPSON09_ETH_TXDESC, SZ_8K);
#else
	mdp->rx_ring = (struct mfp09_eth_rxdesc *)0xf0000000;
	mdp->tx_ring = (struct mfp09_eth_txdesc *)0xf0002000;
#endif
       if (!mdp->rx_ring) {
               DPRINTK(KERN_ERR "%s: Cannot allocate Rx Ring (size %d bytes)\n",
                       ndev->name, SZ_8K);
               ret = -ENOMEM;
               goto out_release;
       }

       if (!mdp->tx_ring) {
               DPRINTK(KERN_ERR "%s: Cannot allocate Tx Ring (size %d bytes)\n",
                       ndev->name, SZ_8K);
               ret = -ENOMEM;
               goto out_release;
       }

       platform_set_drvdata(pdev, ndev);
       return ret;

out_release:
       /* net_dev free */
	if (mdp->rx_ring != NULL) {
		iounmap(mdp->rx_ring);
	}
	if (mdp->tx_ring != NULL) {
		iounmap(mdp->tx_ring);
	}
       if (ndev)
               free_netdev(ndev);
out:

       return ret;
}

static int mfp09_eth_drv_remove(struct platform_device *pdev)
{
       struct net_device *ndev = platform_get_drvdata(pdev);

       unregister_netdev(ndev);
       flush_scheduled_work();

       free_netdev(ndev);
       platform_set_drvdata(pdev, NULL);

       return 0;
}

/* network device open function */
static int mfp09_eth_open(struct net_device *ndev)
{
       int ret = 0;
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
	u32 ioaddr = ndev->base_addr;
#if !defined(CONFIG_MFP09_DUMMY)
	outl(0x0000, ioaddr + CXR6);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
DPRINTK("mfp09_eth_open 1\n");

       ret = request_irq(ndev->irq, &mfp09_eth_interrupt, 0, ndev->name, ndev);
       if (ret) {
               DPRINTK(KERN_ERR "Can not assign IRQ number to %s\n", CARDNAME);
               return ret;
       }

       /* Descriptor set */
       ret = mfp09_eth_ring_init(ndev);
       if (ret)
               goto out_free_irq;

       /* device init */
       ret = mfp09_eth_dev_init(ndev);
       if (ret)
               goto out_free_irq;

       /* PHY control start*/
       ret = mfp09_eth_phy_start(ndev);
       if (ret)
               goto out_free_irq;
       mdelay(100);

       /* Set the timer to check for link beat. */
       init_timer(&mdp->timer);
       mdp->timer.expires = (jiffies + (24 * HZ)) / 10;/* 2.4 sec. */
       mdp->timer.data = (u32) ndev;
       mdp->timer.function = mfp09_eth_timer;     /* timer handler */
       add_timer(&mdp->timer);

DPRINTK(KERN_ERR "mii_start\n");
		/* mii start */
	mod_timer(&(mdp->mii_timer),(jiffies + MII_TIMEOUT ));

       return ret;

out_free_irq:
       free_irq(ndev->irq, ndev);
       return ret;
}

/* device close function */
static int mfp09_eth_close(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr = ndev->base_addr;

       netif_stop_queue(ndev);

       /* Disable interrupts by clearing the interrupt mask. */
#if !defined(CONFIG_MFP09_DUMMY)
       outl(0x0000, ioaddr + CXR6);

	/* Link Down : disable tx and rx */
	outl(inl(ioaddr + CXR20) & ~(CXR20_RE | CXR20_TE), ioaddr + CXR20);

       /* Stop the chip's Tx and Rx processes. */
       outl(0, ioaddr + CXR1);
       outl(0, ioaddr + CXR2);
#endif /* !defined(CONFIG_MFP09_DUMMY) */

       free_irq(ndev->irq, ndev);

       del_timer_sync(&mdp->timer);
       del_timer_sync(&mdp->mii_timer);

       /* Free all the skbuffs in the Rx queue. */
       mfp09_eth_ring_free(ndev);

       return 0;
}

static int mfp09_eth_dev_init(struct net_device *ndev)
{
       int ret = 0;
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr = ndev->base_addr;
       u32 val;

       /* Soft Reset */
#if !defined(CONFIG_MFP09_DUMMY)
       mfp09__eth_reset(ndev);

       /* all mfp09_eth int mask */
       outl(0, ioaddr + CXR6);

       /* FIFO size set */
#ifdef MPF09_DE_1
       outl(CXR0_DE, ioaddr + CXR0);    /* Endian change */
#else
       outl(0, ioaddr + CXR0);  	  /* Endian change */
#endif
       outl((FIFO_SIZE_T | FIFO_SIZE_R), ioaddr + CXR10);
       outl(0, ioaddr + CXR9);

       outl(CXR11_RST, ioaddr + CXR11);

       outl((FIFO_F_D_RFF | FIFO_F_D_RFD), ioaddr + CXR19);
       outl(0, ioaddr + CXR18);
#endif /* !defined(CONFIG_MFP09_DUMMY) */

       /* Descriptor format */
       mfp09_eth_ring_format(ndev);

#if !defined(CONFIG_MFP09_DUMMY)
#ifdef SOC09S
{
       outl(EPSON09_ETH_RXDESC, ioaddr + CXR4);
       outl(EPSON09_ETH_TXDESC, ioaddr + CXR3);

#if defined(DMA_PADDING)
	/* add 2 byte padding to receive buffer */
	outl(0x00020000, ioaddr + CXRS2);
#endif
}
#else
       outl((u32)mdp->rx_ring, ioaddr + CXR4);
       outl((u32)mdp->tx_ring, ioaddr + CXR3);
#endif

       outl(inl(ioaddr + CXR5), ioaddr + CXR5);
       outl((DMAC_M_RFRMER | DMAC_M_ECI | 0x003fffff), ioaddr + CXR6);

       /* PAUSE Prohibition */
       val = (inl(ioaddr + CXR20) & CXR20_DM) |
               CXR20_ZPF | (mdp->duplex ? CXR20_DM : 0) | CXR20_TE | CXR20_RE;
       outl(val, ioaddr + CXR20);
       outl(CXR21_BRCRX | CXR21_PSRTO | CXR21_LCHNG | CXR21_ICD |
                 CXR22_MPDIP, ioaddr + CXR21);
       outl(CXR22_BRCRXIP | CXR22_PSRTOIP | CXR22_LCHNGIP |
                 CXR22_ICDIP | CXR22_MPDIP, ioaddr + CXR22);

       outl(0x00000001, ioaddr + CXR71);
       outl(0x00000001, ioaddr + CXR72);
#endif /* !defined(CONFIG_MFP09_DUMMY) */

       /* Set MAC address */
       update_mac_address(ndev);

       /* Setting the Rx mode will start the Rx process. */
       netif_start_queue(ndev);
#if !defined(CONFIG_MFP09_DUMMY)
       outl(CXR2_R, ioaddr + CXR2);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
DPRINTK("finish mfp09_eth_dev_init \n");

       return ret;
}

static struct net_device_stats *mfp09_eth_get_stats(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr = ndev->base_addr;
DPRINTK(KERN_ERR "mfp09_eth_get_stats \n");
#if !defined(CONFIG_MFP09_DUMMY)
       mdp->stats.tx_dropped += inl(ioaddr + CXR40);
       outl(0, ioaddr + CXR40);   /* (write clear) */
       mdp->stats.collisions += inl(ioaddr + CXR41);
       outl(0, ioaddr + CXR41);    /* (write clear) */
       mdp->stats.tx_carrier_errors += inl(ioaddr + CXR42);
       outl(0, ioaddr + CXR42);    /* (write clear) */
       mdp->stats.tx_carrier_errors += inl(ioaddr + CXR43);
       outl(0, ioaddr + CXR43);   /* (write clear) */
#endif /* !defined(CONFIG_MFP09_DUMMY) */
DPRINTK(KERN_ERR "mfp09_eth_get_stats end \n");

       return &mdp->stats;
}

static irqreturn_t mfp09_eth_interrupt(int irq, void *netdev)
{
       struct net_device *ndev = netdev;
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr, boguscnt = RX_RING_SIZE;
       u32 intr_status = 0;

       ioaddr = ndev->base_addr;
       spin_lock(&mdp->lock);

       intr_status = inl(ioaddr + CXR5);
       /* Clear interrupt */
       outl(intr_status, ioaddr + CXR5);

	DEBUG_L1PRINT("interrupt start irq = %d, CXR5(inttrupt status) = %x", irq, intr_status);

       if (intr_status & (CXR5_FRC | CXR5_RINT8 |
                          CXR5_RINT5 | CXR5_RINT4 | CXR5_RINT3 | CXR5_RINT2 |
                          CXR5_RINT1)) {
		DEBUG_L1PRINT("  rx start\n");
               mfp09_eth_rx(ndev);
       }
       if (intr_status & (CXR5_FTC |
                          CXR5_TINT4 | CXR5_TINT3 | CXR5_TINT2 | CXR5_TINT1)) {
		DEBUG_L1PRINT("  tx ends\n");
               mfp09_eth_txfree(ndev);
               netif_wake_queue(ndev);
       }

       if (intr_status & CXR5_ERR_CHECK) {
		DEBUG_L1PRINT("  err intrrupt start\n");               
               mfp09_eth_error(ndev, intr_status);
       }

       if (--boguscnt < 0) {
               DPRINTK(KERN_WARNING
                      "%s: Too much work at interrupt, status=0x%4.4x.\n",
                      ndev->name, intr_status);
       }

       spin_unlock(&mdp->lock);

       return IRQ_HANDLED;
}

/* error control function */
static void mfp09_eth_error(struct net_device *ndev, int intr_status)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       u32 ioaddr = ndev->base_addr;
       u32 felic_stat;

       if (intr_status & CXR5_ECI) {
               felic_stat = inl(ioaddr + CXR21);
               outl(felic_stat, ioaddr + CXR21);   /* clear int */
               if (felic_stat & CXR21_ICD)
                       mdp->stats.tx_carrier_errors++;
               if (felic_stat & CXR21_LCHNG) {
                       /* Link Changed */
                       u32 link_stat = (inl(ioaddr + CXR2B));
                       if (link_stat & PHY_ST_LINK) {	/* 1 is up, 0 is down.depend on phy chip. */
                               /* Link Down : disable tx and rx */
                               outl(inl(ioaddr + CXR20) &
                                         ~(CXR20_RE | CXR20_TE), ioaddr + CXR20);
                       } else {
                               /* Link Up */
                               outl(inl(ioaddr + CXR6) &
                                         ~DMAC_M_ECI, ioaddr + CXR6);
                               /*clear int */
                               outl(inl(ioaddr + CXR21),
                                         ioaddr + CXR21);
                               outl(inl(ioaddr + CXR6) |
                                         DMAC_M_ECI, ioaddr + CXR6);
                               /* enable tx and rx */
                               outl(inl(ioaddr + CXR20) |
                                         (CXR20_RE | CXR20_TE), ioaddr + CXR20);
                       }
               }
       }

       if (intr_status & CXR5_TWB) {
               /* Write buck end. unused write back interrupt */
               if (intr_status & CXR5_TABT)    /* Transmit Abort int */
                       mdp->stats.tx_aborted_errors++;
       }

       if (intr_status & CXR5_RABT) {
               /* Receive Abort int */
               if (intr_status & CXR5_RFRMER) {
                       /* Receive Frame Overflow int */
                       mdp->stats.rx_frame_errors++;
                       DPRINTK(KERN_ERR "Receive Frame Overflow\n");
               }
       }

       if (intr_status & CXR5_ADE) {
               if (intr_status & CXR5_TDE) {
                       if (intr_status & CXR5_TFE)
                               mdp->stats.tx_fifo_errors++;
               }
       }

       if (intr_status & CXR5_RDE) {
               /* Receive Descriptor Empty int */
               mdp->stats.rx_over_errors++;

               if (inl(ioaddr + CXR2) ^ CXR2_R)
                       outl(CXR2_R, ioaddr + CXR2);
#if defined(DEBUG_L1)
               {
			int entry = (mdp->cur_rx % RX_RING_SIZE);
			struct mfp09_eth_rxdesc * rxdesc = &mdp->rx_ring[entry];
			long descio = inl(ioaddr + CXR4);
			DPRINTK(KERN_ERR "Receive Descriptor Empty entry = %d\n", entry);
			DPRINTK("mdp = %x, rxdesc = %x, CXR4 = %x, rxdesc->status = %x, rxdesc->addr = %x\n", 
					mdp, rxdesc,  descio, rxdesc->status, rxdesc->addr);
		}
#endif
	   }
       if (intr_status & CXR5_RFE) {
               /* Receive FIFO Overflow int */
               mdp->stats.rx_fifo_errors++;
               DPRINTK(KERN_ERR "Receive FIFO Overflow\n");
       }
       if (intr_status &
           (CXR5_TWB | CXR5_TABT | CXR5_ADE | CXR5_TDE | CXR5_TFE)) {
               /* Tx error */
               u32 cxr1 = inl(ndev->base_addr + CXR1);
               /* dmesg */
               DPRINTK(KERN_ERR "%s:TX error. status=%8.8x cur_tx=%8.8x ",
                               ndev->name, intr_status, mdp->cur_tx);
               DPRINTK(KERN_ERR "dirty_tx=%8.8x state=%8.8x CXR1=%8.8x.\n",
                               mdp->dirty_tx, (u32) ndev->state, cxr1);
               /* dirty buffer free */
               mfp09_eth_txfree(ndev);

               if (cxr1 ^ CXR1_TRNS) {
                       /* tx dma start */
                       outl(CXR1_TRNS, ndev->base_addr + CXR1);
               }
		DPRINTK("CXR5 = %08x, CXR6 = %08x\n", inl(ndev->base_addr + CXR5), inl(ndev->base_addr + CXR6));
               /* wakeup */
               netif_wake_queue(ndev);
       }
}

/* Get skb and descriptor buffer */
static int mfp09_eth_ring_init(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       int ret = 0;
       /*
        * +26 gets the maximum ethernet encapsulation, +7 & ~7 because the
        * card needs room to do 8 byte alignment, +2 so we can reserve
        * the first 2 bytes, and +16 gets room for the status word from the
        * card.
        */
	DPRINTK("mfp09_eth_ring_init\n");
       mdp->rx_buf_sz = (ndev->mtu <= 1492 ? PKT_BUF_SZ :
                         (((ndev->mtu + 26 + 7) & ~7) + 2 + 16));

       /* Allocate RX and TX skb rings */
       mdp->rx_skbuff = kmalloc(sizeof(*mdp->rx_skbuff) * RX_RING_SIZE,
                               GFP_KERNEL);
       if (!mdp->rx_skbuff) {
               DPRINTK(KERN_ERR "%s: Cannot allocate Rx skb\n", ndev->name);
               ret = -ENOMEM;
               return ret;
       }

       mdp->tx_skbuff = kmalloc(sizeof(*mdp->tx_skbuff) * TX_RING_SIZE,
                               GFP_KERNEL);
       if (!mdp->tx_skbuff) {
               DPRINTK(KERN_ERR "%s: Cannot allocate Tx skb\n", ndev->name);
               ret = -ENOMEM;
               goto skb_ring_free;
       }

       mdp->dirty_rx = 0;

       return ret;

skb_ring_free:
       /* Free Rx and Tx skb ring buffer */
       mfp09_eth_ring_free(ndev);

       return ret;
}

/* format skb and descriptor buffer */
static void mfp09_eth_ring_format(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       int i;
       struct sk_buff *skb;
       struct mfp09_eth_rxdesc *rxdesc = NULL;
       struct mfp09_eth_txdesc *txdesc = NULL;
       int rx_ringsize = sizeof(*rxdesc) * RX_RING_SIZE;
       int tx_ringsize = sizeof(*txdesc) * TX_RING_SIZE;

DPRINTK("mfp09_eth_ring_format start \n");
       mdp->cur_rx = mdp->cur_tx = 0;
       mdp->dirty_rx = mdp->dirty_tx = 0;

       memset(mdp->rx_ring, 0, rx_ringsize);

       /* build Rx ring buffer */
       for (i = 0; i < RX_RING_SIZE; i++) {
               /* skb */
               mdp->rx_skbuff[i] = NULL;
               skb = dev_alloc_skb(mdp->rx_buf_sz + 64 + RXLENGTH_BOUNDARY);
               mdp->rx_skbuff[i] = skb;
               if (skb == NULL)
                       break;
               skb->dev = ndev;        /* Mark as being used by this device. */
#if defined(SOC09S)
		skb_reserve(skb, ((u32)(skb->data + RXADDRESS_BOUNDARY - 1) & ~(RXADDRESS_BOUNDARY - 1)) - (u32)skb->data);
#else
               skb_reserve(skb, RX_OFFSET);
               skb_reserve(skb, 0xe);
#endif

               /* RX descriptor */
               rxdesc = &mdp->rx_ring[i];
#if	defined(SOC09S)
               rxdesc->addr = virt_to_phys(skb->data);
               rxdesc->status = cpu_to_le32(RD_RACT | RD_RFP);
#else
               rxdesc->addr = (u32)(RX_BUF_BASE + i * RX_BUF_SIZE) & ~0x3UL;
               rxdesc->status = cpu_to_le32(RD_RACT);		// Non RFP
#endif
               /* The size of the buffer must be 32 byte boundary. */
		rxdesc->buffer_length = (mdp->rx_buf_sz + RXLENGTH_BOUNDARY - 1) & ~(RXLENGTH_BOUNDARY - 1);
				rxdesc->pad0 = 0;	// for debug
       }

       mdp->dirty_rx = (u32) (i - RX_RING_SIZE);

       /* Mark the last entry as wrapping the ring. */
       rxdesc->status |= cpu_to_le32(RC_RDEL);
       memset(mdp->tx_ring, 0, tx_ringsize);

       /* build Tx ring buffer */
       for (i = 0; i < TX_RING_SIZE; i++) {
               mdp->tx_skbuff[i] = NULL;
               txdesc = &mdp->tx_ring[i];
               txdesc->status = cpu_to_le32(TD_TFP);
               txdesc->buffer_length = 0;
				txdesc->pad1 = 0;
       }

       txdesc->status |= cpu_to_le32(TD_TDLE);
	mb();
DPRINTK("mfp09_eth_ring_format end \n");
}

/* free skb and descriptor buffer */
static void mfp09_eth_ring_free(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       int i;

       /* Free Rx skb ringbuffer */
       if (mdp->rx_skbuff) {
               for (i = 0; i < RX_RING_SIZE; i++) {
                       if (mdp->rx_skbuff[i])
                               dev_kfree_skb(mdp->rx_skbuff[i]);
               }
       }
       kfree(mdp->rx_skbuff);

       /* Free Tx skb ringbuffer */
       if (mdp->tx_skbuff) {
               for (i = 0; i < TX_RING_SIZE; i++) {
                       if (mdp->tx_skbuff[i])
                               dev_kfree_skb(mdp->tx_skbuff[i]);
               }
       }
       kfree(mdp->tx_skbuff);
	mb();
}

/* free Tx skb function */
static int mfp09_eth_txfree(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       struct mfp09_eth_txdesc *txdesc;
       int freeNum = 0;
       int entry = 0;

       for (; mdp->cur_tx - mdp->dirty_tx > 0; mdp->dirty_tx++) {
               entry = mdp->dirty_tx % TX_RING_SIZE;
               txdesc = &mdp->tx_ring[entry];
               if (txdesc->status & cpu_to_le32(TD_TACT))
				{
					break;
				}
               /* Free the original skb. */
               if (mdp->tx_skbuff[entry]) {
                       dev_kfree_skb_irq(mdp->tx_skbuff[entry]);
                       mdp->tx_skbuff[entry] = NULL;
                       freeNum++;
               }
               txdesc->status = cpu_to_le32(TD_TFP);
              if (entry >= TX_RING_SIZE - 1) {
                       txdesc->status |= cpu_to_le32(TD_TDLE);
               }

               mdp->stats.tx_packets++;
               mdp->stats.tx_bytes += txdesc->buffer_length;
       }
	mb();
       return freeNum;
}

/* Packet transmit function */
static int mfp09_eth_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       struct mfp09_eth_txdesc *txdesc;
       u32 entry;
       unsigned long flags;
       int ret_err;
	DEBUG_L1PRINT("start xmit start\n");

	spin_lock_irqsave(&mdp->lock, flags);
       if ((mdp->cur_tx - mdp->dirty_tx) >= (TX_RING_SIZE - 4)) {
               if (!mfp09_eth_txfree(ndev)) {
                       netif_stop_queue(ndev);
                       spin_unlock_irqrestore(&mdp->lock, flags);
                       return 1;
               }
       }
       spin_unlock_irqrestore(&mdp->lock, flags);

       {
		u8 movelen = 0;
		movelen=((unsigned long)(skb->data) & 0xf);
		movelen=(0xf & ~movelen)+ 1;
		DEBUG_L1PRINT("skb->data move 16byte line trimlen = %d, before skb=%08x, skb->data=%08x\n", movelen, skb, skb->data);
		skb_cow(skb, movelen);
		DEBUG_L1PRINT("skb->data move 16byte line trimlen = %d, after  skb=%08x, skb->data=%08x\n", movelen, skb, skb->data);
	}

       entry = mdp->cur_tx % TX_RING_SIZE;
	/* 60byte short packet, pad area fill 0x00 */
		if ( ETHERSMALL > skb->len ) {
			ret_err = skb_pad( skb,  ETHERSMALL - skb->len );
			if(ret_err != 0){
				printk(KERN_WARNING "short packet fix up faild.(skb_pad return(%d) \n", ret_err);
				return ret_err;
			}
		}
/* 60byte short packet, pad area fill 0x00 */
       mdp->tx_skbuff[entry] = skb;
       txdesc = &mdp->tx_ring[entry];
#if defined(SOC09S)
  #if !defined(MPF09_DE_1)
        swaps((char *)((unsigned long)(skb->data) & ~0x3), skb->len + 2);
  #endif        /* MPF09_DE_1 */
       txdesc->addr = (u32)virt_to_phys(skb->data);
#else   /* !defined(SOC09S) */
		{
			union change
			{
				unsigned long dt32;
				unsigned char dt8[4];
			};
			union change chg;

			unsigned long *dst;
			unsigned char *src;
			int	count;

			dst=(unsigned long *)(TX_BUF_BASE+entry*TX_BUF_SIZE);
			src=(unsigned char *)(skb->data);
			for(count=0;count<skb->len/4+1;count++) {
#ifdef MPF09_DE_1
				chg.dt8[0]=*src;
				chg.dt8[1]=*(src+1);
				chg.dt8[2]=*(src+2);
				chg.dt8[3]=*(src+3);
#else
				chg.dt8[0]=*(src+3);
				chg.dt8[1]=*(src+2);
				chg.dt8[2]=*(src+1);
				chg.dt8[3]=*src;
#endif
				*dst = chg.dt32;

				src+=4;
				dst+=1;
			}
		}

       txdesc->addr = (u32)(TX_BUF_BASE+entry*TX_BUF_SIZE);
#endif  /* !defined(SOC09S) */


#if 1
		{
			unsigned long ul;
			unsigned short us;
			unsigned long *p = (unsigned long *)&txdesc->status;
			p++;
		    if (skb->len < ETHERSMALL){
					us = ETHERSMALL;
					ul = us;
					ul <<= 16;
					*p = ul;

			}
		     else{
					us = skb->len;
					ul = us;
					ul <<= 16;
					*p = ul;
			}
		}
#else
	    if (skb->len < ETHERSMALL)
	            txdesc->buffer_length = ETHERSMALL;
	     else
		       txdesc->buffer_length = skb->len;
#endif
	if (entry >= TX_RING_SIZE - 1)
               txdesc->status |= cpu_to_le32(TD_TACT | TD_TDLE);
       else
               txdesc->status |= cpu_to_le32(TD_TACT);

       mdp->cur_tx++;
DEBUG_L1PRINT("start xmit txdesc->status = %08x txdesc->buff_len = %08x, irq mask = %08x\n",
		 txdesc->status, txdesc->buffer_length, inl(ndev->base_addr + CXR6));

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mb();
#else
	eth_mb((__u32)skb->data, skb->len);
#endif
 
#if !defined(CONFIG_MFP09_DUMMY)
       outl(CXR1_TRNS, ndev->base_addr + CXR1);
#endif /* !defined(CONFIG_MFP09_DUMMY) */
       ndev->trans_start = jiffies;

       return 0;
}

/* Timeout function */
static void mfp09_eth_tx_timeout(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       struct mfp09_eth_rxdesc *rxdesc;
       int i;
#if defined(DEBUG_PRINT)
       u32 ioaddr = ndev->base_addr;
#endif

DPRINTK("start timeout 1\n");
       netif_stop_queue(ndev);

       /* worning message out. */
//       DPRINTK(KERN_WARNING "%s: transmit timed out, status %8.8x,"
//             " resetting...\n", ndev->name, (int)inl(ioaddr + CXR5));

       /* tx_errors count up */
       mdp->stats.tx_errors++;

       /* timer off */
       del_timer_sync(&mdp->timer);

       /* Free all the skbuffs in the Rx queue. */
       for (i = 0; i < RX_RING_SIZE; i++) {
               rxdesc = &mdp->rx_ring[i];
               rxdesc->status = 0;
               rxdesc->addr = 0xBADF00D0;
               if (mdp->rx_skbuff[i])
                       dev_kfree_skb(mdp->rx_skbuff[i]);
               mdp->rx_skbuff[i] = NULL;
       }
       for (i = 0; i < TX_RING_SIZE; i++) {
               if (mdp->tx_skbuff[i])
                       dev_kfree_skb(mdp->tx_skbuff[i]);
               mdp->tx_skbuff[i] = NULL;
       }

       /* device init */
       mfp09_eth_dev_init(ndev);

       /* timer on */
       mdp->timer.expires = (jiffies + (24 * HZ)) / 10;/* 2.4 sec. */
       add_timer(&mdp->timer);
}

/* Packet receive function */
static int mfp09_eth_rx(struct net_device *ndev)
{
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       struct mfp09_eth_rxdesc *rxdesc;

       int entry = mdp->cur_rx % RX_RING_SIZE;
       int boguscnt = (mdp->dirty_rx + RX_RING_SIZE) - mdp->cur_rx;
       struct sk_buff *skb;
       u16 pkt_len = 0;
       u32 desc_status;

	mb();
       rxdesc = &mdp->rx_ring[entry];
	DEBUG_L1PRINT("start eth_rx (rxdesc=%x) \n", rxdesc);
       while (!(rxdesc->status & cpu_to_le32(RD_RACT))) {
               desc_status = le32_to_cpu(rxdesc->status);
               pkt_len = rxdesc->frame_length;

               if (--boguscnt < 0)
                       break;

               if (!(desc_status & RDFEND))
                       mdp->stats.rx_length_errors++;

               if (desc_status & (RD_RFS1 | RD_RFS2 | RD_RFS3 | RD_RFS4 |
                                  RD_RFS5 | RD_RFS6 | RD_RFS10)) {
			DEBUG_L1PRINT(" rx error detect \n");
                       mdp->stats.rx_errors++;
                       if (desc_status & RD_RFS1)
                               mdp->stats.rx_crc_errors++;
                       if (desc_status & RD_RFS2)
                               mdp->stats.rx_frame_errors++;
                       if (desc_status & RD_RFS3)
                               mdp->stats.rx_length_errors++;
                       if (desc_status & RD_RFS4)
                               mdp->stats.rx_length_errors++;
                       if (desc_status & RD_RFS6)
                               mdp->stats.rx_missed_errors++;
                       if (desc_status & RD_RFS10)
                               mdp->stats.rx_over_errors++;
               } else {
                       skb = mdp->rx_skbuff[entry];
                       mdp->rx_skbuff[entry] = NULL;
#if defined(SOC09S)
  #if !defined(MPF09_DE_1)
                                                swaps((char *)((unsigned long)(skb->data) & ~0x3), pkt_len + 2);
  #endif
#else

						{
							union change
							{
								unsigned long dt32;
								unsigned char dt8[4];
							};
							union change chg;

							unsigned char *dst;
							unsigned long *src;

							src=(unsigned long *)(RX_BUF_BASE+entry*RX_BUF_SIZE);
							dst=(unsigned char *)(skb->data);
							{
								int i,k;
								for(i=0;i<pkt_len/4;i++) {
									chg.dt32=*src;
#ifdef MPF09_DE_1
									*(dst+0) = chg.dt8[0];
									*(dst+1) = chg.dt8[1];
									*(dst+2) = chg.dt8[2];
									*(dst+3) = chg.dt8[3];
#else
									*(dst+0) = chg.dt8[3];
									*(dst+1) = chg.dt8[2];
									*(dst+2) = chg.dt8[1];
									*(dst+3) = chg.dt8[0];
#endif
									src+=1;
									dst+=4;
								}
								if((k=pkt_len%4) > 0) {	
									chg.dt32=*src;
									for(i=0;i<k;i++) {
#ifdef MPF09_DE_1
										*(dst+i) = chg.dt8[i];
#else
										*(dst+i) = chg.dt8[3-i];
#endif
									}
								}
							}
						}
#endif
                       skb_put(skb, pkt_len);
#if defined(SOC09S)
#if !defined(DMA_PADDING)
			memmove(skb->data + RX_OFFSET, skb->data, skb->len);
#endif
			/* DMA add 2 byte padding for received buffer. see CXRS2 */
			skb_reserve(skb, RX_OFFSET);
#endif
#ifdef DEBUG_L1
DPRINTK("skb_put skb=%x, , skb-len=%d\n", skb, skb->len);
#endif
                       skb->protocol = eth_type_trans(skb, ndev);
                       netif_rx(skb);
                       ndev->last_rx = jiffies;
                       mdp->stats.rx_packets++;
                       mdp->stats.rx_bytes += pkt_len;
               }

               entry = (++mdp->cur_rx) % RX_RING_SIZE;

#if 1 //rxdesc is not changed in this while loop. So next code is added.
       	       rxdesc = &mdp->rx_ring[entry];
#endif

       }

/* DPRINTK("Refill ring  buffer\n"); */
       /* Refill the Rx ring buffers. */
       for (; mdp->cur_rx - mdp->dirty_rx > 0; mdp->dirty_rx++) {
               entry = mdp->dirty_rx % RX_RING_SIZE;
               rxdesc = &mdp->rx_ring[entry];
               if (mdp->rx_skbuff[entry] == NULL) {
                       skb = dev_alloc_skb(mdp->rx_buf_sz + 64 + RXLENGTH_BOUNDARY);
                       mdp->rx_skbuff[entry] = skb;
                       if (skb == NULL)
                               break;  /* Better luck next round. */
                       skb->dev = ndev;
#if defined(SOC09S)
                       /* because skb data might be 16 byte boundary, fix it to 32 byte boundary. */
			skb_reserve(skb, ((u32)(skb->data + RXADDRESS_BOUNDARY - 1) & ~(RXADDRESS_BOUNDARY - 1)) - (u32)skb->data);
                       rxdesc->addr = virt_to_phys(skb->data);
			DEBUG_L1PRINT("Refill ring  buffer alloc addr virt = %x, phys = %x\n",  skb->data, rxdesc->addr);
#else
                       skb_reserve(skb, RX_OFFSET);
                       rxdesc->addr = (u32)(RX_BUF_BASE + entry * RX_BUF_SIZE) & ~0x3UL;	// Use SSRAM
#endif
               }
               /* The size of the buffer is 32 byte boundary. */
		rxdesc->buffer_length = (mdp->rx_buf_sz + RXLENGTH_BOUNDARY - 1) & ~(RXLENGTH_BOUNDARY - 1);
               if (entry >= RX_RING_SIZE - 1)
                       rxdesc->status |=
                       cpu_to_le32(RD_RACT | RD_RFP | RC_RDEL);
               else
                       rxdesc->status |=
                       cpu_to_le32(RD_RACT | RD_RFP);
       }

DEBUG_L1PRINT("eth_rx end. (and next rx waiting start) \n");
	mb();
       /* Restart Rx engine if stopped. */
       /* If we don't need to check status, don't. -KDU */
#if !defined(CONFIG_MFP09_DUMMY)
       outl(CXR2_R, ndev->base_addr + CXR2);
#endif /* !defined(CONFIG_MFP09_DUMMY) */

       return 0;
}

/* Multicast reception directions set */
static void mfp09_eth_set_multicast_list(struct net_device *ndev)
{
       u32 ioaddr = ndev->base_addr;
#if !defined(CONFIG_MFP09_DUMMY)
       if (ndev->flags & IFF_PROMISC) {
               /* Set promiscuous. */
               outl((inl(ioaddr + CXR20) & ~CXR20_MCT) | CXR20_PRM,
                         ioaddr + CXR20);
       } else {
               /* Normal, unicast/broadcast-only mode. */
               outl((inl(ioaddr + CXR20) & ~CXR20_PRM) | CXR20_MCT,
                         ioaddr + CXR20);
       }
#endif /* !defined(CONFIG_MFP09_DUMMY) */
}


static struct platform_driver mfp09_eth_driver = {
       .probe = mfp09_eth_drv_probe,
       .remove = mfp09_eth_drv_remove,
       .driver = {
                  .name = CARDNAME,
       },
};

static void mfp09_eth_timer(unsigned long data)
{
       struct net_device *ndev = (struct net_device *)data;
       struct mfp09_eth_private *mdp = netdev_priv(ndev);
       int next_tick = 10 * HZ;

       /* We could do something here... nah. */
       mdp->timer.expires = jiffies + next_tick;
       add_timer(&mdp->timer);
}

static int __init mfp09_eth_init(void)
{
DPRINTK("mfp09_eth_init\n");	
	return platform_driver_register(&mfp09_eth_driver);
}

static void __exit mfp09_eth_cleanup(void)
{
       platform_driver_unregister(&mfp09_eth_driver);
}

/*
 * mdio_watchdog - ethernet link check function.
 * @dev : net device
 *
 * this function is linux software timer handler for ethernet link checking.
 * timer handler interval is 2 sec.
 */
static void mii_watchdog(struct net_device *dev)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);

    mfp09_eth_adjust_link(dev);

	if(priv->mii.force_media == 1){
		mii_check_link(&(priv->mii));
	}
	mod_timer(&(priv->mii_timer),(jiffies + MII_TIMEOUT));
}

/*
 * mii_start - register kernel timer list
 * @dev : net device
 *
 * register linux kernel timer list for mii check
 */
static void mii_start(struct net_device *dev)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);

#if !defined(CONFIG_MFP09_DUMMY)
	init_timer( &priv->mii_timer );
	priv->mii_timer.data = (unsigned long)dev;
	priv->mii_timer.function = (void *) &mii_watchdog;
	
	mod_timer(&(priv->mii_timer),(jiffies + MII_TIMEOUT ));
#endif	/* !defined(CONFIG_MFP09_DUMMY) */
}


/*
 * mii_initial_setup - initialize linux mii_if_info structure
 * @dev : net device
 * @phy_id : phy id
 *
 * initialize linux kernel mii_if_info structure. mdio_read, mdio_write functions....
 */
static void mii_initial_setup(struct net_device *dev, unsigned int phy_id)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);
	unsigned long status;
	DPRINTK("mii_initial_setup\n");

	DPRINTK("mii_initial_setup phy_id = %d\n", phy_id);
	
	priv->mii.dev         = dev;
	priv->mii.full_duplex = 1;
	priv->mii.mdio_read   = mdio_read;
	priv->mii.mdio_write  = mdio_write;
	priv->mii.phy_id      = phy_id;
	priv->mii.reg_num_mask= 0x1f;
	priv->mii.phy_id_mask = 0x3f;
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	/* In case of Normal PHY chip */
	printk("%s: Auto-negotiation\n", dev->name);
#endif
#if !defined(CONFIG_MFP09_DUMMY)
    if (priv->autoneg == AUTONEG_ENABLE) {
                /* set Autonego */
                /* if Autonego is Enabele */
                status = mdio_read (dev, priv->mii.phy_id, MII_BMCR);
		status &= BMCR_ANENABLE;
                mdio_write (dev, priv->mii.phy_id, MII_BMCR, status);
    }
#endif /* !defined(CONFIG_MFP09_DUMMY) */
}

/* wait PHY linked */
/* if Autonegotiation mode, wait L1 link while seconds (max 4s) */
static int link_wait(struct net_device *dev)
{
	/* we change where we control wait time, from kernel to application. */

	return 0;
}

/* auto negotiation param set to PHY*/
static int set_eth_autonego(struct net_device *dev)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);
	unsigned long  status;
	unsigned long  autonego_flag = 0;

	/* EPI Ether PHY setting */
	DPRINTK(KERN_DEBUG "\n <DEBUG> set_eth_autonego() enter (priv-autoneg=%x\n", priv->autoneg);

	/* status reserve */
	status = mdio_read (dev, priv->mii.phy_id, MII_BMCR);
	/* save autonego flag before reset PHYi */
	if (priv->autoneg == AUTONEG_ENABLE) {
		autonego_flag = AUTONEG_ENABLE;
	}

	/* reset  PHY  */
	status |= BMCR_RESET;
	mdio_write(dev, priv->mii.phy_id, MII_BMCR, status);
	status &= ~BMCR_RESET;

	/* PHY reset is clear priv->autonego, set */
	if (autonego_flag == AUTONEG_ENABLE){
		priv->autoneg = AUTONEG_ENABLE;
	}

	if (priv->autoneg == AUTONEG_ENABLE) {
		/* set Autonego */
		/* if local cache of autoneg is Enabele, set PHY's register Autonego enable */
		status = mdio_read (dev, priv->mii.phy_id, MII_BMCR);
		DPRINTK(KERN_INFO "auto neg enabled in set_eth_autonego\n");
		priv->mii.force_media = 0;
		status |= BMCR_ANENABLE;
		mdio_write (dev, priv->mii.phy_id, MII_BMCR, status);
DPRINTK(KERN_INFO "auto neg enabled in set_eth_autonego1 \n");
		link_wait(dev);
DPRINTK(KERN_INFO "auto neg enabled in set_eth_autonego2\n");
	}
	else{  /* disable autonegotition */
		/* only reaches here only when priv->autoneg is set to AUTONEG_DISABLE via ethtool I/F */
		/* if local cache of autoneg is disable, set PHY's register Autonego disable */
		status = mdio_read (dev, priv->mii.phy_id, MII_BMCR);

		if (status & BMCR_ANENABLE) {
			priv->mii.force_media = 1;
			DPRINTK(KERN_INFO "auto neg disabled in set_eth_autonego\n");
			status &= ~BMCR_ANENABLE;
		}
			/* Are we forcing 100Mbps??? */
		if (priv->forced_speed_duplex == mfp09_100_full
			|| priv->forced_speed_duplex == mfp09_100_half) {
			/* Set the 100Mb bit and turn off the 1000Mb and 10Mb bits. */
			status |= PHY_C_SPEEDSL;
			DPRINTK ("Forcing 100MB ");
		}   
		else{
			/* Set the 10Mb bit and turn off the 1000Mb and 100Mb bits. */
			status &= ~PHY_C_SPEEDSL;
			DPRINTK ("Forcing 10MB ");
		}

		/* Are we forcing Full or Half Duplex? */
		if (priv->forced_speed_duplex == mfp09_100_full
                  || priv->forced_speed_duplex == mfp09_10_full) {
                    /* We want to force full duplex so we SET the full duplex bits in the
                     * Device and MII Control Registers.
                     */
			priv->mii.full_duplex = 1;
			status |= PHY_C_DUPLEX;
			DPRINTK ("Full Duplex\n");
		}
		else {
                    /* We want to force half duplex so we CLEAR the full duplex bits in
                     * the Device and MII Control Registers.
                     */
			priv->mii.full_duplex = 0;
			status &= ~PHY_C_DUPLEX;
			DPRINTK ("Half Duplex\n");
		}
		/* write setting */
		mdio_write (dev, priv->mii.phy_id, MII_BMCR, status);
		link_wait(dev);

	}
	DPRINTK ("\n <DEBUG> exit set_eth_autonego() \n");
	return 0;
}


/*
 * mfp09_eth_up - restart ethernet interface (ethtool sset)
 * @dev : net device
 *
 * net device member function for ethernet interface restart
 */
static int mfp09_eth_up(struct net_device *dev)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);

	DPRINTK("\n <DEBG> mfp09_eth_up() enter \n");
	DPRINTK("%s: ethernet start\n", dev->name);

	spin_lock(&priv->lock);

	/* valid ethernet address? */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		printk(KERN_ERR "%s: invalid ethernet MAC address\n",dev->name);
		spin_unlock(&priv->lock);
		return -EINVAL;
	}
	/* EPI Ether PHY setting */
	set_eth_autonego(dev);
	link_wait(dev);

	/* EPI Ethernet enable */
//	hw_start(dev->base_addr);

	spin_unlock(&priv->lock);

	mdelay(10);
	/* net interface queue start */
	netif_wake_queue(dev);

	DPRINTK("\n <DEBG> mfp09_eth_up() exit \n");
	return 0;
}

/* =============---ethtool ------================== */

/****************************************************************************/
/* NAME:  get_autoneg_status                                            */
/*--------------------------------------------------------------------------*/
/* DESCRIPTION:                                                             */
/*   Starts Transmission of data                                           */
/* PARAMETERS:                                                              */
/* IN  : net_device*                                               */
/* OUT : None                                                              */
/* RETURN:  int                                                             */
/*--------------------------------------------------------------------------*/
/* REENTRANCY: NA                                                           */
/****************************************************************************/
int
get_autoneg_status (struct net_device *dev)
{
    int data; 
	struct mfp09_eth_private *priv = netdev_priv(dev);

    data =
        mdio_read (dev, priv->mii.phy_id, MII_BMCR);
    return (data & BMCR_ANENABLE) ? 1 : 0;
}
/*********************************************************************/
/* NAME:  mfp09_get_speed_and_duplex                                   */
/*-------------------------------------------------------------------*/
/* DESCRIPTION:                                                      */
/*   Detects the current speed and duplex settings of the hardware.  */
/* PARAMETERS:                                                       */
/*   IN  : net_device*                                               */
/*          speed - Speed of the connection			*/
/*          duplex - Duplex setting of the connection		*/
/*   OUT :  None                                                */
/* RETURN: int                                                  */
/*--------------------------------------------------------------*/
/* REENTRANCY: NA                                               */
/****************************************************************/
long 
mfp09_get_speed_and_duplex (struct net_device *dev, unsigned long * speed,
                            unsigned long * duplex)
{
    unsigned long status;
	struct mfp09_eth_private *priv = netdev_priv(dev);

  if(get_autoneg_status(dev))
  {
     status = mdio_read (dev, priv->mii.phy_id, MII_LPA); /* temp phy_id */
     if(status & LPA_100FULL )
      {
          *speed = SPEED_100;
          *duplex = DUPLEX_FULL;
         DPRINTK ("100 MB & Full Duplex\r\n");
      }
      else if(status & LPA_100HALF)
      {
          *speed = SPEED_100;
          *duplex = DUPLEX_HALF;
         DPRINTK ("100 MB & HALF Duplex\r\n");      }
      else if(status & LPA_10FULL)
       {
          *speed = SPEED_10;
          *duplex = DUPLEX_FULL;
         DPRINTK ("10 MB & FULL Duplex\r\n");

        }
      else if(status & LPA_10HALF)
       {
          *speed = SPEED_10;
          *duplex = DUPLEX_HALF;
         DPRINTK ("10 MB & HALF Duplex\r\n");

        }
      else
       {
           DPRINTK("No speed(10/100) found from the link partner");
        }
  }
  else
  {
     status =   mdio_read (dev, priv->mii.phy_id, MII_BMCR); /* temp phy_id */
    if (status & BMCR_FULLDPLX )
      {
          *duplex = DUPLEX_FULL;
          DPRINTK ("Full Duplex\r\n");
      }
    else
      {
          *duplex = DUPLEX_HALF;
          DPRINTK (" Half Duplex\r\n");
      }
   if (status & BMCR_SPEED100 )
      {
          *speed = SPEED_100;
          DPRINTK ("100 Mbs, ");
      }
    else
      {
          *speed = SPEED_10;
          DPRINTK ("10 Mbs, ");
      }
 }

    return 0;
}
/*******************************************************************/
/* NAME:  get_link_status                                          */
/*-----------------------------------------------------------------*/
/* DESCRIPTION:get the link status                                 */
/* PARAMETERS:                                                     */
/* 	IN  : net_device* dev                                      */
/* 	OUT : None                                                 */
/* RETURN: void                                                    */
/*-----------------------------------------------------------------*/
/* REENTRANCY: NA                                                  */
/*******************************************************************/

unsigned long get_link_status(struct net_device *dev)
{
    unsigned long mii_status;
	struct mfp09_eth_private *priv = netdev_priv(dev);

    mii_status=mdio_read (dev, priv->mii.phy_id, MII_BMSR); /* temp phy_id */

   return (mii_status & BMSR_LSTATUS);

}

static u32 mfp09_get_link(struct net_device *dev)
{
	struct mfp09_eth_private *priv = netdev_priv(dev);
        u32 ret;

	DPRINTK("\n <DEBG> call mfp09_get_link()\n");
	spin_lock_irq(&priv->lock);
        ret= (u32)get_link_status(dev);
        spin_unlock_irq(&priv->lock);
        return ret;
}

/*******************************************************************/
/* NAME:  mfp09_get_supported_features                             */
/*-----------------------------------------------------------------*/
/* DESCRIPTION:                                                    */
/*    Gets the supported features for ethernet                     */
/* PARAMETERS:                                                     */
/* IN  : net_device* dev,u32* supported:data is written in it      */
/* OUT : None                                                      */
/* RETURN: void                                                    */
/*-----------------------------------------------------------------*/
/* REENTRANCY: NA                                                  */
/*******************************************************************/

void   mfp09_get_supported_features(struct net_device *dev,u32 * supported)
{
  unsigned long status;
	struct mfp09_eth_private *priv = netdev_priv(dev);

  status = mdio_read (dev, priv->mii.phy_id, MII_BMSR); /* temp phy_id */

 *supported = (status& BMSR_100FULL)? SUPPORTED_100baseT_Full:0 ;
 *supported |= (status& BMSR_100HALF)? SUPPORTED_100baseT_Half:0 ;
 *supported |= (status& BMSR_10FULL)? SUPPORTED_10baseT_Full:0 ;
 *supported |= (status& BMSR_10HALF)? SUPPORTED_10baseT_Half:0 ;
 *supported |= (status &  BMSR_ANEGCAPABLE)?SUPPORTED_Autoneg:0 ;

}

/************************************************************/
/* NAME:  mfp09_get_advertised_features                     */
/*----------------------------------------------------------*/
/* DESCRIPTION:                                             */
/*    Gets the supported features for ethernet              */
/*                                                          */
/* PARAMETERS:                                              */
/*    IN  : net_device* dev,u32* advertised                 */
/*    OUT : None                                            */
/* RETURN: void                                             */
/*----------------------------------------------------------*/
/* REENTRANCY: NA                                           */
/************************************************************/

void  mfp09_get_advertised_features(struct net_device *dev,u32 * advertised)
{
  unsigned long status;
	struct mfp09_eth_private *priv = netdev_priv(dev);

  status = mdio_read (dev, priv->mii.phy_id, MII_ADVERTISE); /* temp phy_id */

 *advertised = (status& ADVERTISE_100FULL)? ADVERTISED_100baseT_Full:0 ;
 *advertised |= (status& ADVERTISE_100HALF)?ADVERTISED_100baseT_Half:0 ;
 *advertised |= (status& ADVERTISE_10FULL)?ADVERTISED_10baseT_Full:0 ;
 *advertised |= (status& ADVERTISE_10HALF)?ADVERTISED_10baseT_Half:0 ;
 *advertised |= get_autoneg_status(dev)?ADVERTISED_Autoneg:0 ;
}
/*******************************************************************/
/* NAME:  mfp09_ethtool_gset                                         */
/*-----------------------------------------------------------------*/
/* DESCRIPTION:                                                    */
/*    returns the ethernet settings to the user                    */
/* PARAMETERS:                                                     */
/*    IN  : net_device*, ethtool_cmd*                              */
/*    OUT : None                                                   */
/* RETURN: int                                                     */
/*-----------------------------------------------------------------*/
/* REENTRANCY: NA                                                  */
/*******************************************************************/

static int mfp09_ethtool_gset (struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	int ret;
    struct mfp09_eth_private *priv = netdev_priv(netdev);

/* ecmd->supported =  
        (SUPPORTED_10baseT_Full | SUPPORTED_10baseT_Half |
         SUPPORTED_100baseT_Full | SUPPORTED_100baseT_Half |
         SUPPORTED_Autoneg);*/

	ret =netif_carrier_ok(priv->dev);
    if(ret)
    {
      mfp09_get_supported_features(netdev,&ecmd->supported);
  /*ecmd->advertising =
        (SUPPORTED_100baseT_Full | SUPPORTED_100baseT_Half |
         ADVERTISED_Autoneg);*/
     mfp09_get_advertised_features(netdev,&ecmd->advertising);
     priv->supported_features=ecmd->supported;
     priv->advertised_features=ecmd->advertising;

    }
    else
    {
     ecmd->supported=priv->supported_features;
     ecmd->advertising=priv->advertised_features;
                       
    }
	ecmd->phy_address = PHY_ID;
    ecmd->port = PORT_MII;
    ecmd->transceiver = XCVR_EXTERNAL;
DPRINTK ("inside gset\n");
    if (netif_carrier_ok (priv->dev))
      {
          mfp09_get_speed_and_duplex (netdev, &priv->link_speed,
                                       &priv->link_duplex);
          ecmd->speed = priv->link_speed;

              /* unfortunatly FULL_DUPLEX != DUPLEX_FULL
               *          and HALF_DUPLEX != DUPLEX_HALF */

/*              if(priv->link_duplex == FULL_DUPLEX)
                        ecmd->duplex = DUPLEX_FULL;
                else
                        ecmd->duplex = DUPLEX_HALF;
*/
              ecmd->duplex = priv->link_duplex;
      }

    else
      {
          ecmd->speed = -1;
          ecmd->duplex = -1;
      }
    ecmd->autoneg = get_autoneg_status (netdev);
    return 0;
}

/*****************************************************/
/* NAME: mfp09_ethtool_sset                          */
/*---------------------------------------------------*/
/* DESCRIPTION:                                      */
/*    Sets the eeprom with the user settings         */
/* PARAMETERS:                                       */
/* 	IN  : net_device*, ethtool_cmd*              */
/* 	OUT : None                                   */
/* RETURN:  int                                      */
/*---------------------------------------------------*/
/* REENTRANCY: NA                                    */
/*****************************************************/
static int mfp09_ethtool_sset (struct net_device *netdev, struct ethtool_cmd *ecmd)
{
    struct mfp09_eth_private *priv = netdev_priv(netdev);

    DPRINTK(KERN_INFO "mfp09_ethtool_set, cmd->auroneg=%x\n", ecmd->autoneg);
    if (ecmd->autoneg == AUTONEG_ENABLE)
      {
          DPRINTK(KERN_INFO "autoneg enabled\n");
          priv->autoneg = AUTONEG_ENABLE;
	  priv->mii.force_media = 0;

/*               adapter->fc_autoneg=1; */
          priv->autoneg_advertised = 0x004F;
          ecmd->advertising = 0x004F;
      }
    else
    {
          priv->autoneg = AUTONEG_DISABLE;
	  priv->mii.force_media = 1;
       if(ecmd->duplex > 1  )
           ecmd->duplex = priv->link_duplex;
       if(ecmd->speed >=65535 )
           ecmd->speed = priv->link_speed;
      if (mfp09_set_spd_dplx (priv, ecmd->speed + ecmd->duplex))
        return -EINVAL;
    }
        /* reset the link */
#if 1
     mfp09_eth_up (priv->dev);
#else
    if (netif_running (priv->dev))
    {
	/* stop net interface queue */
          netif_stop_queue(priv->dev);
 		  mdelay(10);
 //          hw_stop(priv->base_addr);
 			  mdelay(10);
 	         netif_start_queue(priv->dev);
 	         mfp09_eth_up (priv->dev);
        }
   	else{
	/* stop net interface queue */
 	      netif_stop_queue(priv->dev);
 		  mdelay(10);
 //         hw_stop(priv->base_addr);
 		  mdelay(10);
          netif_start_queue(priv->dev);
          mfp09_eth_up (priv->dev);
	}
#endif
	mod_timer(&(priv->mii_timer),(jiffies + MII_TIMEOUT));

        return 0;
}

/*************************************************************/
/* NAME:   mfp09_set_spd_dplx                                */
/*-----------------------------------------------------------*/
/* DESCRIPTION:                                              */
/*   Starts Transmission of data                             */
/* PARAMETERS:                                               */
/* IN  : eth_priv*, uint16_t                                 */
/* OUT : None                                                */
/* RETURN: int                                               */
/*-----------------------------------------------------------*/
/* REENTRANCY: NA                                            */
/*************************************************************/
int mfp09_set_spd_dplx (struct  mfp09_eth_private *adapter, uint16_t spddplx)
{
    DPRINTK (" inside set_spd_dplx\n");
    adapter->autoneg = AUTONEG_DISABLE;
   DPRINTK("\nspd & dplx =%d",spddplx);
      switch (spddplx)
      {
      case SPEED_10 + DUPLEX_HALF:
          adapter->forced_speed_duplex = mfp09_10_half;
          break;
      case SPEED_10 + DUPLEX_FULL:
          adapter->forced_speed_duplex = mfp09_10_full;
          break;
      case SPEED_100 + DUPLEX_HALF:
          adapter->forced_speed_duplex = mfp09_100_half;
          break;
      case SPEED_100 + DUPLEX_FULL:
          adapter->forced_speed_duplex = mfp09_100_full;
          break;
      case SPEED_1000 + DUPLEX_FULL:
      case SPEED_1000 + DUPLEX_HALF:    /* not supported */
DPRINTK ("Unsupported Speed/Duplexity configuration\n");
          return -EINVAL;
      default:
          adapter->forced_speed_duplex = mfp09_100_full;
          break;
      }
    return 0;
}

/* ethtool struct */

struct ethtool_ops mfp09_ethtool_ops =
{
	.get_settings =      mfp09_ethtool_gset,
	.set_settings =      mfp09_ethtool_sset,
	.get_link =  mfp09_get_link, 
};

void set_ethtool_ops (struct net_device *netdev)
{
	SET_ETHTOOL_OPS (netdev, &mfp09_ethtool_ops);
}
/* =============---ethtool ------================== */

module_init(mfp09_eth_init);
module_exit(mfp09_eth_cleanup);

