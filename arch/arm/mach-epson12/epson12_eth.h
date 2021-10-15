/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#ifndef __EPSON12_ETH_H
#define __EPSON12_ETH_H

#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

/* Ethernet PHY default settings */
#define ETHERNET_PHY_MASK		0
#define ETHERNET_PHY_ADDR(x)		(x)
#define RTL8201F_POWER_SAVING_MODE	24
#define RTL8201F_LED_MASK		0xffcf
#define RTL8201F_LED_SELECT		0x0010
#define RTL8201F_PAGE_SELECT		31

/* GMAC MII interface */
#define GMAC_MII_ADDR		0x00000010 /* MII Address */
#define GMAC_MII_DATA		0x00000014 /* MII Data */
#define MII_BUSY		0x00000001
#define MII_WRITE		0x00000002

/* SAMSUNG MISC register */
#define	GMAC_SEL_MII_RGMII(x)	(x << 4) /* Selection of interface |[ 6: 4]| R_W| 0 */
#define	GMAC_SEL_MII_GMII	0	 /* GMII/MII */
#define	GMAC_SEL_RGMII		1	 /* RGMII */

/* SAMSUNG GMAC Clock Divider */
#define GMAC_CLK_DIVIDER_MASK	0xfffffffc
#define GMAC_CLK_DIVIDER_1000	0x0
#define GMAC_CLK_DIVIDER_100	0x3
#define GMAC_CLK_DIVIDER_10	0x2

/* Ethernet PHY H/W reset mode */
#if defined(CONFIG_ETHPHY_HWRESET_12M_BCM54610)
#define phy_hwreset	phy_hwreset_soc12m_bcm54610
#elif defined(CONFIG_ETHPHY_HWRESET_12M_AR8035)
#define phy_hwreset	phy_hwreset_soc12m_ar8035
#elif defined(CONFIG_ETHPHY_HWRESET_12H_BCM54610_DIVF)
#define phy_hwreset	phy_hwreset_soc12h_bcm54610_divf
#elif defined(CONFIG_ETHPHY_HWRESET_12H_AR8035_DIVL)
#define phy_hwreset	phy_hwreset_soc12h_ar8035_divl
#elif defined(CONFIG_ETHPHY_HWRESET_12H_AR8035_DIVF)
#define phy_hwreset	phy_hwreset_soc12h_ar8035_divf
#elif defined(CONFIG_ETHPHY_HWRESET_12H_AR8035_DIVI)
#define phy_hwreset	phy_hwreset_soc12h_ar8035_divi
#endif

/* AR8035 */
#define AR8035_PHY_SPECIFIC_STATUS	0x11
#define AR8035_LED_CTRL			0x18
#define AR8035_MANUAL_LED_OVERRIDE	0x19

extern void fix_gmac_speed(void *priv, unsigned int speed);
extern int gmac_gigabit_init(struct platform_device *pdev);
extern int gmac_additional_init(struct platform_device *pdev);

extern int rtl8201f_phy_reset(void *priv);
extern int bcm54610_phy_reset(void *priv);
extern int ar8035_phy_reset(void *priv);

int epson12_mdio_write(int phyaddr, int phyreg, u16 phydata);
int epson12_mdio_read(int phyaddr, int phyreg);

int gmac_switch_led_on_off();

#endif
