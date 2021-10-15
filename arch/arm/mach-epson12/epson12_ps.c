/*

Copyright (c)SEIKO EPSON CORPORATION 2012-2013. All rights reserved.

Module Name:
	Inter Domain Communication Driver.

Abstract:

Author:
	IJP.LP Division (ijplpd) 08/09/2011

Revision History:

Date		Who	What
--------	-------	------------------------------------------------
08/09/2011	ijplpd	birth

*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/if_vlan.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "mach/epson12_ps.h"
#include "epson12_eth.h"
#include "epson12_pcie.h"

#include "../../../drivers/net/stmmac/stmmac.h"

#define EPS_STMMAC_DISC_CLEAN
#define EPS_STMMAC_STOP_CHK	/* at restart */

/* POWER_SAVING_SWITCH_L1 */
int g_PowerSavingSwiting = PSS_CPU_NORMAL_MODE;
EXPORT_SYMBOL(g_PowerSavingSwiting);

/* for register saveing */
__u32 g_reg_ether_op	= 0;
EXPORT_SYMBOL(g_reg_ether_op);
__u32 g_reg_ether_int	= 0;
EXPORT_SYMBOL(g_reg_ether_int);
__u32 g_ps_ether_status	= PSS_ETHSTA_IFDOWN;
EXPORT_SYMBOL(g_ps_ether_status);
__u32 g_ps_ethernet_restarted	= 0;
EXPORT_SYMBOL(g_ps_ethernet_restarted);

__u32 g_reg_timer_ctl	= 0;	/* Timer register save */
__u16 g_reg_uart_ctl	= 0;	/* UART register save */

/* for LED on/off */
int g_PowerSaveMode = EPPSM_NORMAL;
EXPORT_SYMBOL(g_PowerSaveMode);

struct net_device *g_stmmac_dev = NULL;
EXPORT_SYMBOL(g_stmmac_dev);

__u32 g_Receive_List_Base	= 0;
EXPORT_SYMBOL(g_Receive_List_Base);

__u32 g_Transmit_List_Base	= 0;
EXPORT_SYMBOL(g_Transmit_List_Base);

EP_INTERFACE_SKB_SET *g_descripter_data_top = NULL;
EXPORT_SYMBOL(g_descripter_data_top);

extern int stmmac_mdio_reset(struct mii_bus *bus);

/***********************************************************
	for Stop/Restart
************************************************************/
static int stmmac_dma_stop(struct net_device *ndev, int enforcement)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	__u32 status = 0;
	__u32 eth_base	= IO_ADDRESS(EPSON12_GMAC_BASE);
#ifdef EPS_STMMAC_DISC_CLEAN
	unsigned int txsize = priv->dma_tx_size;
	unsigned int rxsize = priv->dma_rx_size;
#endif
	int dis_ic = 0;

	PSDBG("<EP> %s enter.\n", __FUNCTION__);

	if (g_ps_ether_status == PSS_ETHSTA_IFDOWN) {
		/* Can execute only from interface active state. */
		PSDBG("<EP> stmmac_dma_stop: not stopped (%d)\n", g_ps_ether_status);
		return -ENODEV;
	} else if (enforcement) {
		/* Enforcement */
		PSDBG("<EP> stmmac_dma_stop: enforcement (%d)\n", g_ps_ether_status);
	} else if (g_ps_ether_status == PSS_ETHSTA_STOP_API || g_ps_ether_status == PSS_ETHSTA_STOP_KERN) {
		/* Can execute only stop from interface active state. */
		PSDBG("<EP> stmmac_dma_stop: not stopped (%d)\n", g_ps_ether_status);
		return -EBUSY;
	} else {
		/* Normal Operation */
		PSDBG("<EP> stmmac_dma_stop: normal operation (%d)\n", g_ps_ether_status);
		if (!ndev || !netif_running(ndev)) {
			PSDBG("<EP> stmmac_dma_stop: not active interface (%d)\n", g_ps_ether_status);
			return -ENODEV;
		}
	}
	PSDBG("<EP> stmmac_dma_stop: operation start.\n");

#ifdef STMMAC_DEBUG
	PSDBG("<EP> Ethernet register dump\n");
	pr_info("------------------------------\n");
	stmmac_mdio_dump(priv->mii);
	priv->hw->mac->dump_regs(priv->ioaddr);
	priv->hw->dma->dump_regs(priv->ioaddr);
	pr_info("\n");
#endif

#ifdef CONFIG_STMMAC_TIMER
	/* Disable interrupts on completion for the reception if timer is on */
	if (likely(priv->tm->enable))
		dis_ic = 1;
#endif

	local_irq_disable();

	/* Register 6 (Operation Mode Register) */
	g_reg_ether_op = readl(eth_base + DMA_CONTROL);
	PSDBG("<EP> stmmac_dma_stop : Register 6 (Operation Mode Register) CNT=0x%08x\n",
		(unsigned int)g_reg_ether_op);
	g_reg_ether_op = g_reg_ether_op & ~(GMAC_ST | GMAC_SR);
	writel(g_reg_ether_op, eth_base + DMA_CONTROL);

	/* Receive / Trasmit Descriptor List Address Register */
	g_Receive_List_Base = readl(eth_base + DMA_RCV_BASE_ADDR);
	PSDBG("<EP> stmmac_dma_stop : Register 3 (Rx Descriptor List Address Register) RxBASE=0x%08x\n",
		(unsigned int)g_Receive_List_Base);
	g_Transmit_List_Base = readl(eth_base + DMA_TX_BASE_ADDR);
	PSDBG("<EP> stmmac_dma_stop : Register 4 (Tx Descriptor List Address Register) TxBASE=0x%08x\n",
		(unsigned int)g_Transmit_List_Base);

	/* Register 7 (Interrupt Enable Register) */
	g_reg_ether_int = readl(eth_base + DMA_INTR_ENA);
	PSDBG("<EP> stmmac_dma_stop : Register 7 (Interrupt Enable Register) INT=0x%08x\n",
		(unsigned int)g_reg_ether_int);
	writel(0, eth_base + DMA_INTR_ENA);

	/* DMA Stop waiting */
	do {
		status = readl(eth_base + DMA_STATUS) & (GMAC_TS_MASK | GMAC_RS_MASK);
		PSDBG("<EP> stmmac_dma_stop : ether transport enable status=0x%08x\n", (unsigned int)status);
	} while (status);

	g_ps_ether_status = PSS_ETHSTA_STOP_API;	/* interface stopped */
	g_ps_ethernet_restarted = 1;			/* for watchdog */

	local_irq_enable();

	napi_disable(&priv->napi);
	disable_irq(ndev->irq);

#ifdef EPS_STMMAC_DISC_CLEAN
	/* Clear the Rx/Tx descriptors */
	priv->hw->desc->init_rx_desc(priv->dma_rx, rxsize, dis_ic);
	priv->hw->desc->init_tx_desc(priv->dma_tx, txsize);
#endif

	return 0;
}


int eps_stmmac_dma_stop(struct net_device *ndev, int enforcement)
{
	if (ndev != NULL) {
		return stmmac_dma_stop(ndev, enforcement);
	}

	if (g_stmmac_dev == NULL) {
		printk("<EP> eps_stmmac_dma_stop: not saved stmmac net_device\n");
		return -ENODEV;
	}

	return stmmac_dma_stop(g_stmmac_dev, enforcement);
}
EXPORT_SYMBOL(eps_stmmac_dma_stop);


static int stmmac_dma_restart(struct net_device *ndev, int enforcement)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	__u32 eth_base	= IO_ADDRESS(EPSON12_GMAC_BASE);
	unsigned int tmp = 0;
#ifdef EPS_STMMAC_STOP_CHK
	__u32 status = 0;
#endif

	if (g_ps_ether_status == PSS_ETHSTA_IFDOWN) {
		/* Can execute only from interface active state. */
		PSDBG("<EP> stmmac_dma_restart: not stopped (%d)\n", g_ps_ether_status);
		return -ENODEV;
	} else if (enforcement) {
		/* Enforcement */
		PSDBG("<EP> stmmac_dma_restart: enforcement (%d)\n", g_ps_ether_status);
	} else if (g_ps_ether_status == PSS_ETHSTA_STOP_API || g_ps_ether_status == PSS_ETHSTA_STOP_KERN) {
		/* Normal Operation */
		PSDBG("<EP> stmmac_dma_restart: normal operation (%d)\n", g_ps_ether_status);
		if (!netif_running(ndev)) {
			return -ENODEV;
		}
	} else  {
		/* Can execute only resume from stopped state. */
		PSDBG("<EP> stmmac_dma_restart: not stopped (%d)\n", g_ps_ether_status);
		return -EBUSY;
	}
	PSDBG("<EP> stmmac_dma_restart: operation start.\n");

#ifdef EPS_STMMAC_STOP_CHK
	/* Register 6 (Operation Mode Register) */
	g_reg_ether_op = readl(eth_base + DMA_CONTROL);
	PSDBG("<EP> stmmac_dma_stop : Register 6 (Operation Mode Register) CNT=0x%08x\n",
		(unsigned int)g_reg_ether_op);
	g_reg_ether_op = g_reg_ether_op & ~(GMAC_ST | GMAC_SR);
	writel(g_reg_ether_op, eth_base + DMA_CONTROL);

	/* DMA Stop waiting */
	do {
		status = readl(eth_base + DMA_STATUS) & (GMAC_TS_MASK | GMAC_RS_MASK);
		PSDBG("<EP> stmmac_dma_stop : ether transport enable status=0x%08x\n", (unsigned int)status);
	} while (status);
#endif

	if (!enforcement) local_irq_disable();

	/* RX Descriptor INITIALIZATION */
	priv->cur_rx = 0;
	priv->dirty_rx = 0;

	/* TX Descriptor INITIALIZATION */
	priv->cur_tx = 0;
	priv->dirty_tx = 0;

#if 0	/* Debug PHY status */
	printk("<EP> PHY RESET\n");
	stmmac_mdio_reset(priv->mii);
#endif

	/* Power Down bit, into the PM register, is cleared
	 * automatically as soon as a magic packet or a Wake-up frame
	 * is received. Anyway, it's better to manually clear
	 * this bit because it can generate problems while resuming
	 * from another devices (e.g. serial console). */

	/* Receive / Trasmit Descriptor List Address Register */
	PSDBG("<EP> stmmac_dma_restart : Register 3 (Rx Descriptor List Address Register) RxBASE=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_RCV_BASE_ADDR));
        writel(g_Receive_List_Base, eth_base + DMA_RCV_BASE_ADDR);
	PSDBG("<EP> stmmac_dma_restart : Register 3 (Rx Descriptor List Address Register) RxBASE=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_RCV_BASE_ADDR));

	PSDBG("<EP> stmmac_dma_restart : Register 4 (Tx Descriptor List Address Register) TxBASE=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_TX_BASE_ADDR));
	writel(g_Transmit_List_Base, eth_base + DMA_TX_BASE_ADDR);
	PSDBG("<EP> stmmac_dma_restart : Register 3 (Tx Descriptor List Address Register) TxBASE=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_TX_BASE_ADDR));

	/* Register 6 (Operation Mode Register) Import */
	PSDBG("<EP> stmmac_dma_restart : Register 6-1 (Operation Mode Register) CNT=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_CONTROL));
	PSDBG("<EP> stmmac_dma_restart : g_reg_ether_op=0x%08x\n", g_reg_ether_op);
	writel(g_reg_ether_op, eth_base + DMA_CONTROL);
	PSDBG("<EP> stmmac_dma_restart : Register 6-1 (Operation Mode Register) CNT=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_CONTROL));

	/* Register 6 (Operation Mode Register) Start Tx/Rx */
	tmp = (readl(eth_base + DMA_CONTROL) | (GMAC_ST | GMAC_SR));
	PSDBG("<EP> stmmac_dma_restart : Start Tx/Rx tmp=0x%08x\n", tmp);
	writel(tmp, eth_base + DMA_CONTROL);
	PSDBG("<EP> stmmac_dma_restart : Register 6-2 (Operation Mode Register) CNT=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_CONTROL));

	napi_enable(&priv->napi);

	/* Register 7 (Interrupt Enable Register) */
	PSDBG("<EP> stmmac_dma_restart : Register 7 (Interrupt Enable Register) INT=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_INTR_ENA));
	PSDBG("<EP> stmmac_dma_restart : g_reg_ether_int=0x%08x\n", g_reg_ether_int);
	writel(g_reg_ether_int, eth_base + DMA_INTR_ENA);
	PSDBG("<EP> stmmac_dma_restart : Register 7 (Interrupt Enable Register) INT=0x%08x\n",
		(unsigned int)readl(eth_base + DMA_INTR_ENA));

	g_ps_ether_status = PSS_ETHSTA_IFUP;	/* interface enabled */

	if (!enforcement) {
		local_irq_enable();
		printk("<EP> stmmac_dma_restart discripter_send\n");
		discripter_send(ndev);
		msleep(20);
		}
	enable_irq(ndev->irq);

	printk("<EP> stmmac_dma_restart end\n");
	return 0;
}


int eps_stmmac_dma_restart(struct net_device *ndev, int enforcement)
{
	if (ndev != NULL) {
		return stmmac_dma_restart(ndev, enforcement);
	}

	if (g_stmmac_dev == NULL) {
		printk("<EP> eps_stmmac_dma_restart: not saved stmmac net_device\n");
		return -ENODEV;
	}

	return stmmac_dma_restart(g_stmmac_dev, enforcement);
}
EXPORT_SYMBOL(eps_stmmac_dma_restart);


void dump_data(unsigned char *buffer, unsigned long len)
{
#ifdef POWERSAVE_DEBUG
	int i;
	unsigned char *pos;
	if (buffer == NULL)
		return;
	pos = buffer;
	for (i = 0; i < len; i++, pos++) {
		if (i % 16 == 0) printk("\t");
		printk("0x%02x ", *pos);
		if (i % 16 == 15) printk("\n");
	}
	printk("\n");
#else
#endif
}


void discripter_send(struct net_device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	EP_INTERFACE_SKB_SET *pos, *old;

	if (g_descripter_data_top == NULL) {
		return;
	}

	if (dev == NULL) {
		return;
	}

	PSDBG("<EP> discripter_send enter\n");

	pos = g_descripter_data_top;
	while (pos != NULL) {
		skb = netdev_alloc_skb_ip_align(dev, BUF_SIZE_2KiB);

		if (skb == NULL) {
			printk("<EP> %s: Rx init fails; skb is NULL\n", __func__);
		} else if (pos->len > 1024) {
			printk("<EP> discripter size is too big [%d]byte\n", pos->len);
		} else {
			PSDBG("<EP> discripter_send set packet data\n");

			skb_put(skb, pos->len);
			memcpy(skb->data, pos->descriptor, pos->len);
			skb->protocol = eth_type_trans(skb, priv->dev);
			dump_data(skb->data, pos->len);
			skb_checksum_none_assert(skb);

			napi_gro_receive(&priv->napi, skb);
		}

		old = pos;
		pos = pos->next;
		g_descripter_data_top = pos;

		kfree(old);
	}

	g_descripter_data_top = NULL;

	PSDBG("<EP> discripter_send leave\n");
}
EXPORT_SYMBOL(discripter_send);


/****************************************************************************
	gic_set_disable()
	
	note
		disable irq and gic
	param
		gic_base:	gic register base address
		irq:		irq number
****************************************************************************/
inline void gic_set_disable(unsigned long gic_base, int irq)
{
	int irq_group_top;
	unsigned long mask = 0, tmp = 0;
	int mod = 0;

	mod = irq % 4;
	irq_group_top = irq - mod;

	switch (mod) {
	case 0:
		mask = ~0x0000000F;
		break;
	case 1:
		mask = ~0x00000F00;
		break;
	case 2:
		mask = ~0x000F0000;
		break;
	case 3:
		mask = ~0x0F000000;
		break;
	}

	disable_irq(irq);
	PSDBG("<EP> GIC%d[%d]: addr=0x%08x val=0x%08x\n", 
		irq, irq_group_top, (unsigned int)(gic_base + GIC_DIST_TARGET + irq_group_top),  (unsigned int)(readl(gic_base + GIC_DIST_TARGET + irq_group_top)));
	tmp = readl(gic_base + GIC_DIST_TARGET + irq_group_top);
	tmp &= mask;
	writel(tmp, gic_base + GIC_DIST_TARGET + irq_group_top);
	PSDBG("<EP> GIC%d[%d]: addr=0x%08x val=0x%08x\n", 
		irq, irq_group_top, (unsigned int)(gic_base + GIC_DIST_TARGET + irq_group_top),  (unsigned int)(readl(gic_base + GIC_DIST_TARGET + irq_group_top)));
}


/****************************************************************************
	gic_set_enable()
	
	note
		enable irq and gic
	param
		gic_base:	gic register base address
		irq:		irq number
****************************************************************************/
inline void gic_set_enable(unsigned long gic_base, int irq, int other_recover)
{
	int irq_group_top;
	unsigned long mask = 0, tmp = 0;
	int mod = 0;

	if (other_recover != 0) {
		PSDBG("<EP> GIC%d: other_recover[%d]\n", irq, other_recover);
		enable_irq(irq);
		return;
	}

	mod = irq % 4;
	irq_group_top = irq - mod;

	switch (mod) {
	case 0:
		mask = 0x00000002;
		break;
	case 1:
		mask = 0x00000200;
		break;
	case 2:
		mask = 0x00020000;
		break;
	case 3:
		mask = 0x02000000;
		break;
	}

	PSDBG("<EP> GIC%d[%d]: addr=0x%08x val=0x%08x\n", 
		irq, irq_group_top, (unsigned int)(gic_base + GIC_DIST_TARGET + irq_group_top),  (unsigned int)(readl(gic_base + GIC_DIST_TARGET + irq_group_top)));
	tmp = readl(gic_base + GIC_DIST_TARGET + irq_group_top);
	tmp |= mask;
	writel(tmp, gic_base + GIC_DIST_TARGET + irq_group_top);
	PSDBG("<EP> GIC%d[%d]: addr=0x%08x val=0x%08x\n", 
		irq, irq_group_top, (unsigned int)(gic_base + GIC_DIST_TARGET + irq_group_top),  (unsigned int)(readl(gic_base + GIC_DIST_TARGET + irq_group_top)));

	enable_irq(irq);
}


/****************************************************************************
	eps_device_stop()
	
	note
		device stop
	param
		target:	target device to stop
	retval
		0 is success
****************************************************************************/
int eps_device_stop(unsigned long target)
{
	int status = 0, tmp = 0;
	__u32 gic_base	= IO_ADDRESS(CORE_PM_BASE);
	__u32 eth_base	= IO_ADDRESS(EPSON12_GMAC_BASE);
	__u32 uart_base	= IO_ADDRESS(EPSON12_UART0_BASE);
#if defined(CONFIG_MACH_EPSON12_H)
	__u32 pe_ring_ctrl = 0;
#endif

	PSDBG("<EP> eps_device_stop enter target=0x%x\n", target);
	PSDBG("<EP> gic_base = [0x%x]=0x%x\n", (unsigned int)gic_base, (unsigned int)readl(gic_base));
	PSDBG("<EP> eth_base = [0x%x]=0x%x\n", (unsigned int)eth_base, (unsigned int)readl(eth_base));
	PSDBG("<EP> uart_base = [0x%x]=0x%x\n", (unsigned int)uart_base, (unsigned int)readl(uart_base));

#if defined(CONFIG_EPSON12_PCIE)
	if (target & EPSTG_PCIE) {
#if defined(CONFIG_EPSON12_PCIE_PORT1)
		PSDBG("<EP> eps_device_stop : CONFIG_EPSON12_PCIE_PORT1\n", g_reg_ether_op);
		epson12_pcie1_stop();
#endif
#if defined(CONFIG_EPSON12_PCIE_PORT0)
		PSDBG("<EP> eps_device_stop : CONFIG_EPSON12_PCIE_PORT0\n", g_reg_ether_op);
		epson12_pcie0_stop();
#endif
	}
#endif

	if (target & EPSTG_ETHER) {
		g_reg_ether_op = readl(eth_base + DMA_CONTROL);
		PSDBG("<EP> eps_device_stop : Ethernet Register 6 (Operation Mode Register) CNT=0x%x\n", g_reg_ether_op);
		tmp = (g_reg_ether_op & ~(GMAC_ST | GMAC_SR));
		writel(tmp, eth_base + DMA_CONTROL);

		g_reg_ether_int = readl(eth_base + DMA_INTR_ENA);
		PSDBG("<EP> eps_device_stop : Ethernet  Register 7 (Interrupt Enable Register) INT=0x%x\n", (unsigned int)g_reg_ether_int);
		writel(0, eth_base + DMA_INTR_ENA);

		g_ps_ether_status = PSS_ETHSTA_STOP_KERN;
	}

	if ((target & EPSTG_ETHER_L2) && g_ps_ether_status == PSS_ETHSTA_IFUP) {
		tmp = eps_stmmac_dma_stop(NULL, 1);
		PSDBG("<EP> eps_device_stop Ethernet(L2) Stop : eps_stmmac_dma_stop=0x%x\n", (unsigned int)tmp);
		g_ps_ether_status = PSS_ETHSTA_STOP_KERN;
	}

#ifdef CONFIG_USB_SEC_EHCI_HCD
	if (target & EPSTG_WIFI) {	/* USB Stop */
		PSDBG("<EP> eps_device_stop : USB Stop\n");
		qtd_ctrl(NULL, 0);
	}
#endif

	if (target & EPSTG_TIMER) {
		g_reg_timer_ctl = readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
		tmp = (g_reg_timer_ctl & ~CONTROL_RUN);
		writel(tmp, TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
		PSDBG("<EP> eps_device_stop : Timer Stop 0x%x\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4)));
	}

	if (target & EPSTG_UART) {
		g_reg_uart_ctl = readw(uart_base + UART_IMSC);
		PSDBG("<EP> eps_device_stop : UART INT Stop [0x%x]=0x%x\n", (unsigned int)uart_base + UART_IMSC, (unsigned int)g_reg_uart_ctl);
		writew(0, uart_base + UART_IMSC);
	}

#if defined(CONFIG_MACH_EPSON12_H)
	if (target & EPSTG_IPSEC) {
		__u32 eip94_base = IO_ADDRESS(EPSON12_EIP94_BASE);
		PSDBG("<EP> eps_device_stop : IPSec\n");
		pe_ring_ctrl = readl(eip94_base + EPSON12_EIP94_PE_RING_CTRL_OFFSET);
		pe_ring_ctrl &= ~(EPSON12_EIP94_PE_RING_CTRL_RETRYMASK | EPSON12_EIP94_PE_RING_CTRL_POLLMASK);
		writel(pe_ring_ctrl, eip94_base + EPSON12_EIP94_PE_RING_CTRL_OFFSET);
	}
#endif

	if ((target & EPSTG_ETHER) || g_ps_ether_status == PSS_ETHSTA_STOP_KERN) {
		/* ethernet wait */
		do {
			status = readl(eth_base + DMA_STATUS) & (GMAC_TS_MASK | GMAC_RS_MASK);
			PSDBG("<EP> ether transport enable status=0x%x\n", (unsigned int)status);
		} while (status);
	}

	if (target & EPSTG_GIC) {

		if (target & EPSTG_SFAX) {
			PSDBG("<EP> eps_device_stop : GIC(SFAX)\n");
			/* GIC38: FAX SPI */
			gic_set_disable(gic_base + INTD_BASE, IRQ_SPIU_FAX);

			/* GIC127: SFAX */
			gic_set_disable(gic_base + INTD_BASE, IRQ_SFAX);

			/* GIC144: FAX */
			gic_set_disable(gic_base + INTD_BASE, IRQ_UART1);
		}

#if defined(CONFIG_EPSON12_PCIE)
		if (target & EPSTG_PCIE) {
#if defined(CONFIG_EPSON12_PCIE_PORT0)
			PSDBG("<EP> eps_device_stop : GIC(CONFIG_EPSON12_PCIE_PORT0)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_PCIE0);
#endif
#if defined(CONFIG_EPSON12_PCIE_PORT1)
			PSDBG("<EP> eps_device_stop : GIC(CONFIG_EPSON12_PCIE_PORT1)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_PCIE1);
#endif
		}
#endif

		if (target & EPSTG_IDC) {
#if defined(__EPSON12_IRQSM_H)
			PSDBG("<EP> eps_device_stop : GIC(__EPSON12_IRQSM_H)\n");
			/* GIC68: IDC CA9-0->CA9-1 */
			gic_set_disable(gic_base + INTD_BASE, IRQ_MISC_BK16);
#elif defined(__EPSON12_IRQSH_H)
			PSDBG("<EP> eps_device_stop : GIC(__EPSON12_IRQSH_H)\n");
			/* GIC48: IDC CA9-0->CA9-1 */
			gic_set_disable(gic_base + INTD_BASE, IRQ_MISC_BK36);
#endif
		}

		if (target & EPSTG_TIMER) {
			PSDBG("<EP> eps_device_stop : GIC(EPSTG_TIMER)\n");
			/* GIC100: Timer */
			gic_set_disable(gic_base + INTD_BASE, IRQ_TIMERU4);
			gic_set_disable(gic_base + INTD_BASE, IRQ_TIMERU5);
			gic_set_disable(gic_base + INTD_BASE, IRQ_TIMERU6);
			gic_set_disable(gic_base + INTD_BASE, IRQ_TIMERU7);
		}

		if (((target & EPSTG_ETHER) || (target & EPSTG_ETHER_L2)) &&
		    (g_ps_ether_status == PSS_ETHSTA_IFUP || g_ps_ether_status == PSS_ETHSTA_STOP_KERN)) {
			PSDBG("<EP> eps_device_stop : GIC(ethernet)\n");
			/* GIC135: Ethernet Power Down Mode */
			gic_set_disable(gic_base + INTD_BASE, IRQ_GETH_LPI);

			/* GIC136: GETH_LPI */
			gic_set_disable(gic_base + INTD_BASE, IRQ_GETH_PMT);

			/* GIC137: Ethernet MAC link */
			gic_set_disable(gic_base + INTD_BASE, IRQ_GETH_SBD);
		}

		if (target & EPSTG_WIFI) {
#if defined(CONFIG_USE_EHCIHOST_0)	/* GIC138: WiFi */
			PSDBG("<EP> eps_device_stop : GIC(CONFIG_USE_EHCIHOST_0)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_USBHOST0);
#elif defined(CONFIG_USE_EHCIHOST_1)	/* GIC139: WiFi */
			PSDBG("<EP> eps_device_stop : GIC(CONFIG_USE_EHCIHOST_1)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_USBHOST1);
#elif defined(CONFIG_USE_EHCIHOST_2)	/* GIC140: WiFi */
			PSDBG("<EP> eps_device_stop : GIC(CONFIG_USE_EHCIHOST_2)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_USBHOST2);
#endif
		}

		if (target & EPSTG_UART) {
			/* GIC143: UART */
			PSDBG("<EP> eps_device_stop : GIC(EPSTG_UART)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_UART0);
		}

#if defined(CONFIG_MACH_EPSON12_H)
		if (target & EPSTG_IPSEC) {
			PSDBG("<EP> eps_device_stop : GIC(EPSTG_IPSEC)\n");
			gic_set_disable(gic_base + INTD_BASE, IRQ_EIP_94);
			gic_set_disable(gic_base + INTD_BASE, IRQ_EIP_150_TRNG);
			gic_set_disable(gic_base + INTD_BASE, IRQ_EIP_AIC_OUT);
		}
#endif

	}

	return 0;
}
EXPORT_SYMBOL(eps_device_stop);


/****************************************************************************
	eps_device_recovery()
	
	note
		device restart
	param
		target:	target device to stop
	retval
		0 is success
****************************************************************************/
int eps_device_recovery(unsigned long target)
{
	/* power save message ! */
	__u32 gic_base	= IO_ADDRESS(CORE_PM_BASE);
	__u32 eth_base	= IO_ADDRESS(EPSON12_GMAC_BASE);
	__u32 uart_base	= IO_ADDRESS(EPSON12_UART0_BASE);
#if defined(CONFIG_MACH_EPSON12_H)
	__u32 pe_ring_ctrl = 0;
#endif
	unsigned long other_recover_gic = target & EPSTG_GIC_ROOT;
	int tmp = 0;

	PSDBG("<EP> eps_device_recovery enter target=0x%x\n", target);
	PSDBG("<EP> gic_base = [0x%x]=0x%x\n", (unsigned int)gic_base, (unsigned int)readl(gic_base));
	PSDBG("<EP> eth_base = [0x%x]=0x%x\n", (unsigned int)eth_base, (unsigned int)readl(eth_base));
	PSDBG("<EP> uart_base = [0x%x]=0x%x\n", (unsigned int)uart_base, (unsigned int)readl(uart_base));

	local_irq_disable();

#if defined(CONFIG_EPSON12_PCIE)
	if (target & EPSTG_PCIE) {
#if defined(CONFIG_EPSON12_PCIE_PORT1)
		PSDBG("<EP> eps_device_recovery : CONFIG_EPSON12_PCIE_PORT1\n");
		epson12_pcie1_recovery();
#endif
#if defined(CONFIG_EPSON12_PCIE_PORT0)
		PSDBG("<EP> eps_device_recovery : CONFIG_EPSON12_PCIE_PORT0\n");
		epson12_pcie0_recovery();
#endif
	}
#endif
	if (target & EPSTG_ETHER) {
		/* Ether Restart : Register 6 (Operation Mode Register) */
		writel(g_reg_ether_op, eth_base + DMA_CONTROL);
		PSDBG("<EP> eps_device_recovery : Ethernet Register 6 (Operation Mode Register) CNT=0x%x\n", (unsigned int)readl(eth_base + DMA_CONTROL));

		writel(g_reg_ether_int, eth_base + DMA_INTR_ENA);
		PSDBG("<EP> eps_device_recovery : Ethernet Register 7 (Interrupt Enable Register) INT=0x%x\n", (unsigned int)readl(eth_base + DMA_INTR_ENA));

		g_ps_ether_status = PSS_ETHSTA_IFUP;

#if defined(CONFIG_POWER_SAVING_SWITCH_L1)
		gmac_switch_led_on_off();
#endif	/* CONFIG_POWER_SAVING_SWITCH_L1 */

	}

	if ((target & EPSTG_ETHER_L2) && g_ps_ether_status == PSS_ETHSTA_STOP_KERN) {
		tmp = eps_stmmac_dma_restart(NULL, 1);
		PSDBG("<EP> eps_device_recovery : Ethernet(L2) eps_stmmac_dma_restart=0x%x\n", tmp);

		g_ps_ether_status = PSS_ETHSTA_IFUP;
	} else {
		PSDBG("<EP> eps_device_recovery : Ethernet(L2) g_ps_ether_status=0x%x (%d)\n", (unsigned int)g_ps_ether_status, (target & EPSTG_ETHER));
	}

#ifdef CONFIG_USB_SEC_EHCI_HCD
	if (target & EPSTG_WIFI) {
		PSDBG("<EP> eps_device_recovery : USB\n");
		qtd_ctrl(NULL, 1);
	}
#endif

	if (target & EPSTG_TIMER) {
		writel(g_reg_timer_ctl, TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
		PSDBG("<EP> eps_device_recovery : Timer 0x%x\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4)));
	}

	if (target & EPSTG_UART) {
		writew(g_reg_uart_ctl, uart_base + UART_IMSC);
		PSDBG("<EP> eps_device_recovery : UART [0x%x]=0x%x\n", 
			(unsigned int)(uart_base + UART_IMSC), (unsigned int)readw(uart_base + UART_IMSC));
	}

#if defined(CONFIG_MACH_EPSON12_H)
	if (target & EPSTG_IPSEC) {
		__u32 eip94_base = IO_ADDRESS(EPSON12_EIP94_BASE);
		PSDBG("<EP> eps_device_recovery : EPSTG_IPSEC\n");
		pe_ring_ctrl = readl(eip94_base + EPSON12_EIP94_PE_RING_CTRL_OFFSET);
		pe_ring_ctrl &= ~(EPSON12_EIP94_PE_RING_CTRL_RETRYMASK | EPSON12_EIP94_PE_RING_CTRL_POLLMASK);
		/* IPSec PE-DMA won't start if CONTINIOUS is off */
		pe_ring_ctrl |= EPSON12_EIP94_PE_RING_CTRL_CONTINUOUS | EPSON12_EIP94_PE_RING_CTRL_DIVISORVALUE;
		writel(pe_ring_ctrl, eip94_base + EPSON12_EIP94_PE_RING_CTRL_OFFSET);
		/* soon writeback normal value */
		pe_ring_ctrl &= ~EPSON12_EIP94_PE_RING_CTRL_CONTINUOUS;
		writel(pe_ring_ctrl, eip94_base + EPSON12_EIP94_PE_RING_CTRL_OFFSET);
	}
#endif

	if (target & EPSTG_GIC) {
		PSDBG("<EP> eps_device_recovery : EPSTG_GIC\n");
		if (target & EPSTG_SFAX) {
			PSDBG("<EP> eps_device_recovery : GIC(EPSTG_SFAX)\n");
			/* GIC38: FAX SPI */
			gic_set_enable(gic_base + INTD_BASE, IRQ_SPIU_FAX, other_recover_gic);

			/* GIC127: SFAX */
			gic_set_enable(gic_base + INTD_BASE, IRQ_SFAX, other_recover_gic);

			/* GIC144: FAX */
			gic_set_enable(gic_base + INTD_BASE, IRQ_UART1, other_recover_gic);
		}


#if defined(CONFIG_EPSON12_PCIE)
		if (target & EPSTG_PCIE) {
#if defined(CONFIG_EPSON12_PCIE_PORT0)
			PSDBG("<EP> eps_device_recovery : GIC(CONFIG_EPSON12_PCIE_PORT0)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_PCIE0, other_recover_gic);
#endif
#if defined(CONFIG_EPSON12_PCIE_PORT1)
			PSDBG("<EP> eps_device_recovery : GIC(CONFIG_EPSON12_PCIE_PORT1)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_PCIE1, other_recover_gic);
#endif
		}
#endif

		if (target & EPSTG_IDC) {
#if defined(__EPSON12_IRQSM_H)
			PSDBG("<EP> eps_device_recovery : GIC(__EPSON12_IRQSM_H)\n");
			/* GIC68: IDC CA9-0->CA9-1 */
			gic_set_enable(gic_base + INTD_BASE, IRQ_MISC_BK16, other_recover_gic);
#elif defined(__EPSON12_IRQSH_H)
			PSDBG("<EP> eps_device_recovery : GIC(__EPSON12_IRQSH_H)\n");
			/* GIC48: IDC CA9-0->CA9-1 */
			gic_set_enable(gic_base + INTD_BASE, IRQ_MISC_BK36, other_recover_gic);
#endif
		}

		if (target & EPSTG_TIMER) {
			PSDBG("<EP> eps_device_recovery : GIC(EPSTG_TIMER)\n");
			/* GIC100: Timer */
			gic_set_enable(gic_base + INTD_BASE, IRQ_TIMERU4, other_recover_gic);
			gic_set_enable(gic_base + INTD_BASE, IRQ_TIMERU5, other_recover_gic);
			gic_set_enable(gic_base + INTD_BASE, IRQ_TIMERU6, other_recover_gic);
			gic_set_enable(gic_base + INTD_BASE, IRQ_TIMERU7, other_recover_gic);
		}

		if (target & EPSTG_WIFI) {
#if defined(CONFIG_USE_EHCIHOST_0)	/* GIC138: WiFi */
			PSDBG("<EP> eps_device_recovery : GIC(CONFIG_USE_EHCIHOST_0)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_USBHOST0, other_recover_gic);
#elif defined(CONFIG_USE_EHCIHOST_1)	/* GIC139: WiFi */
			PSDBG("<EP> eps_device_recovery : GIC(CONFIG_USE_EHCIHOST_1)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_USBHOST1, other_recover_gic);
#elif defined(CONFIG_USE_EHCIHOST_2)	/* GIC140: WiFi */
			PSDBG("<EP> eps_device_recovery : GIC(CONFIG_USE_EHCIHOST_2)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_USBHOST2, other_recover_gic);
#endif
		}

		if (((target & EPSTG_ETHER) || (target & EPSTG_ETHER_L2)) &&
		    (g_ps_ether_status == PSS_ETHSTA_IFUP || g_ps_ether_status == PSS_ETHSTA_STOP_KERN)) {
			PSDBG("<EP> eps_device_recovery : GIC(ethernet)\n");
			/* GIC135: Ethernet Power Down Mode */
			gic_set_enable(gic_base + INTD_BASE, IRQ_GETH_LPI, other_recover_gic);

			/* GIC136: GETH_LPI */
			gic_set_enable(gic_base + INTD_BASE, IRQ_GETH_PMT, other_recover_gic);

			/* GIC137: Ethernet MAC link */
			gic_set_enable(gic_base + INTD_BASE, IRQ_GETH_SBD, other_recover_gic);

			printk("<EP> Ether Restart : GIC\n");
		}

		if (target & EPSTG_UART) {
			PSDBG("<EP> eps_device_recovery : GIC(EPSTG_UART)\n");
			/* GIC143: UART */
			gic_set_enable(gic_base + INTD_BASE, IRQ_UART0, other_recover_gic);
		}

#if defined(CONFIG_MACH_EPSON12_H)
		if (target & EPSTG_IPSEC) {
			PSDBG("<EP> eps_device_recovery : GIC(EPSTG_IPSEC)\n");
			gic_set_enable(gic_base + INTD_BASE, IRQ_EIP_94, other_recover_gic);
			gic_set_enable(gic_base + INTD_BASE, IRQ_EIP_150_TRNG, other_recover_gic);
			gic_set_enable(gic_base + INTD_BASE, IRQ_EIP_AIC_OUT, other_recover_gic);
		}
#endif

	}

	if (target & EPSTG_GIC_ROOT) {
		__u32 cpu_gic_base = gic_base + INTI_BASE;

		PSDBG("<EP> eps_device_recovery : base + GIC_CPU_PRIMASK [0x%x]=0x%x\n", (unsigned int)(cpu_gic_base + GIC_CPU_PRIMASK), (unsigned int)readl(cpu_gic_base + GIC_CPU_PRIMASK));
		writel(0xf0, cpu_gic_base + GIC_CPU_PRIMASK);
		PSDBG("<EP> eps_device_recovery : base + GIC_CPU_PRIMASK [0x%x]=0x%x\n", (unsigned int)(cpu_gic_base + GIC_CPU_PRIMASK), (unsigned int)readl(cpu_gic_base + GIC_CPU_PRIMASK));

		PSDBG("<EP> eps_device_recovery : base + GIC_CPU_CTRL [0x%x]=0x%x\n", (unsigned int)(cpu_gic_base + GIC_CPU_CTRL), (unsigned int)readl(cpu_gic_base + GIC_CPU_CTRL));
		writel(1, cpu_gic_base + GIC_CPU_CTRL);
		PSDBG("<EP> eps_device_recovery : base + GIC_CPU_CTRL [0x%x]=0x%x\n", (unsigned int)(cpu_gic_base + GIC_CPU_CTRL), (unsigned int)readl(cpu_gic_base + GIC_CPU_CTRL));
	}

	local_irq_enable();

	return 0;
}
EXPORT_SYMBOL(eps_device_recovery);

#ifdef _EXPERIMENT_CODE
extern void setup_mm_for_reboot(char mode);

void jump_to_hpl(void)
{
	unsigned long tmp, cpumode, reg_timer_ctl;
	unsigned long tmp1, tmp2;
	__u32 gic_base	= IO_ADDRESS(CORE_PM_BASE);

	/* Disable interrupts first */
	local_irq_disable();
	local_fiq_disable();

	/* stop timer device */
	reg_timer_ctl = readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
	tmp = (reg_timer_ctl & ~CONTROL_RUN);
	writel(tmp, TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
	gic_set_disable(gic_base + INTD_BASE, 100);

	__asm__ __volatile__ ("mrs %0, cpsr;" :"=r" (cpumode));
	tmp = cpumode & ~(PSR_A_BIT | PSR_I_BIT | PSR_F_BIT);
	__asm__ __volatile__ ("msr cpsr_c, %0;" ::"r" (tmp));

	__asm__ __volatile__ (
		"mrc p15, 0, %0, c1, c0"
		:"=r" (tmp));
	//tmp1 = SCTLR_INIT;
	tmp1 = tmp & ~0x0000100D;
	tmp2 = 0;

	/*
	 * Tell the mm system that we are going to reboot -
	 * we may need it to insert some 1:1 mappings so that
	 * soft boot works.
	 */
	setup_mm_for_reboot(0);

	/* Clean and invalidate caches */
	flush_cache_all();

	/* Turn off caching */
	cpu_proc_fin();

	/* Push out any further dirty data, and ensure cache is empty */
	flush_cache_all();

	/* cache flush */
	__flush_icache_all();

	__asm__ __volatile__ (
		"mov r1, %0\n\t"
		"mov r2, %1\n\t"
		"dsb\n\t"
		"mcr p15, 0, r1, c1, c0, 0\n\t"
		"mov pc, r2"
		:: "r" (tmp1), "r" (tmp2));
}
EXPORT_SYMBOL(jump_to_hpl);
#endif /* _EXPERIMENT_CODE */
