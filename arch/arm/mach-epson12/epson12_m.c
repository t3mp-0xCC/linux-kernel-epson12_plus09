/*
 *  Copyright (C) 2009 Samsung Electronics TLD.
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
 * Base on linux/arch/arm/mach-realview/core.c
 *
 * History
 *  12/15/09	jiseong oh		source code create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>

#include <asm/mach-types.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/hardware/arm_timer.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>

#include "epson12_eth.h"
#include "epson12_plat.h"

#if defined(CONFIG_STMMAC_ETH) || defined(CONFIG_STMMAC_ETH_MODULE)
/* Gigabit Ethernet Controller default settings */
#if defined(CONFIG_EPSON12_HAS_GIGABIT_PHY)
#define MAX_PBL_VALUE			63
#else
#define MAX_PBL_VALUE			4
#endif
#define DEFAULT_MDC_VALUE		1
#define HAS_GMAC			1
#define HAS_ENHANCED_DISCRIPTOR		1
#define HAS_TX_COE			1
#define BUGGED_JUMBO_FRAME		0
#define PMT_SUPPORTE			0
#endif /* defined(CONFIG_STMMAC_ETH) || defined(CONFIG_STMMAC_ETH_MODULE) */

#if defined(CONFIG_MACH_EPSON12_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

void __iomem *gic_cpu_base_addr;
extern void __init epson12_gic_init_irq(void);

#if defined(CONFIG_STMMAC_ETH) || defined(CONFIG_STMMAC_ETH_MODULE)
struct plat_stmmacenet_data stmmac_data = {
	.bus_id = 0,
	.pbl = MAX_PBL_VALUE, /* 8xPBL by default */
	.clk_csr = DEFAULT_MDC_VALUE,
	.has_gmac = HAS_GMAC,
	.enh_desc = HAS_ENHANCED_DISCRIPTOR,
	.tx_coe = HAS_TX_COE,
	.bugged_jumbo = BUGGED_JUMBO_FRAME,
	.pmt = PMT_SUPPORTE,
#if defined(CONFIG_EPSON12_ETHPHY_AUTO_SWITCH)
#if defined(CONFIG_EPSON12_HAS_GIGABIT_PHY)
	.fix_mac_speed = fix_gmac_speed,
#endif /* defined(CONFIG_EPSON12_HAS_GIGABIT_PHY) */
	.init = gmac_additional_init,
#else
#if defined(CONFIG_EPSON12_HAS_GIGABIT_PHY)
	.fix_mac_speed = fix_gmac_speed,
	.init = gmac_gigabit_init,
#endif /* defined(CONFIG_EPSON12_HAS_GIGABIT_PHY) */
#endif /* defined(CONFIG_EPSON12_ETHPHY_AUTO_SWITCH) */
};

struct plat_stmmacphy_data stmmacphy_data = {
	.bus_id = 0,
	.phy_mask = ETHERNET_PHY_MASK,
#if defined(CONFIG_EPSON12_HAS_BCM54610)
	.phy_id = PHYID_BCM54610,
	.phy_addr = ETHERNET_PHY_ADDR(0),
	.interface = PHY_INTERFACE_MODE_RGMII,
	.phy_reset = bcm54610_phy_reset,
#elif defined(CONFIG_EPSON12_HAS_RTL8201F)
	.phy_id = PHYID_RTL8201F,
	.phy_addr = ETHERNET_PHY_ADDR(1),
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_reset = rtl8201f_phy_reset,
#elif defined(CONFIG_EPSON12_HAS_AR8035)
	.phy_id = PHYID_AR8035,
	.phy_addr = ETHERNET_PHY_ADDR(4),
	.interface = PHY_INTERFACE_MODE_RGMII,
	.phy_reset = ar8035_phy_reset,
#endif
};

PLATFORM_DEVICE("stmmaceth", 0, GMAC, SZ_16K, IRQ_GETH_SBD, 1, &stmmac_data, NULL);
PLATFORM_DEVICE("stmmacphy", 0, EPHY, SZ_4K, NO_IRQ, 0, &stmmacphy_data, NULL);
#endif /* defined(CONFIG_STMMAC_ETH) || defined(CONFIG_STMMAC_ETH_MODULE) */

/* epson12m board io description */
static struct map_desc epson12_m_io_desc[] __initdata = {
/*	{
		.virtual	= IO_ADDRESS(),
		.pfn		= __phys_to_pfn(),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, */
};

/* Initialize epson12m board io map */
static void __init epson12_m_map_io(void)
{
	epson12_map_io();
	iotable_init(epson12_m_io_desc, ARRAY_SIZE(epson12_m_io_desc));
}

/* Platform device list */
static struct platform_device *platform_devs[] __initdata = {
#if defined (CONFIG_STMMAC_ETH) || defined (CONFIG_STMMAC_ETH_MODULE)
	&GMAC_device,
	&EPHY_device,
#endif
};

static void __init epson12_m_init(void)
{
	int i;

	DBG("<EP> %s\n", __FUNCTION__);
	epson12_init();

	for (i = 0; i < ARRAY_SIZE(platform_devs); i++) {
		struct platform_device *d = platform_devs[i];
		platform_device_register(d);
	}
}

MACHINE_START(EPSON12, "ARM-EPSON12M")
	.phys_io       = EPSON12_UART0_BASE,
	.io_pg_offst   = (IO_ADDRESS(EPSON12_UART0_BASE) >> 18) & 0xfffc,
	.boot_params   = 0x40000100, // DDR Offset 0x0
	.map_io        = epson12_m_map_io,
	.init_irq      = epson12_gic_init_irq,
	.timer         = &epson12_timer,
	.init_machine  = epson12_m_init,
MACHINE_END
