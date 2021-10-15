/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */
 
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/netdevice.h>

#include <asm/io.h>

#include <linux/phy.h>

#include "epson12_eth.h"
#include "mach/epson12_ps.h"

#include "../../../drivers/net/stmmac/stmmac.h"

#if defined(CONFIG_MACH_EPSON12_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

extern struct plat_stmmacenet_data stmmac_data;
extern struct plat_stmmacphy_data stmmacphy_data;

int epson12_mdio_read(int phyaddr, int phyreg);
int epson12_mdio_write(int phyaddr, int phyreg, u16 phydata);

extern int g_PowerSaveMode;


#if defined(CONFIG_MACH_EPSON12_M)
void phy_hwreset_soc12m_bcm54610(void)
{
	DBG("%s\n", __FUNCTION__);

	writel(0x00000100, __io_address(EPSON12_GPIO_BASE) + 0xE8); /* GPIO_DO_RESET */
	writel(0x00000100, __io_address(EPSON12_GPIO_BASE) + 0xC4); /* GPIO_DIR_SET */

	mdelay(1);

	writel(0x00000100, __io_address(EPSON12_GPIO_BASE) + 0xE4); /* GPIO_DO_SET */

	mdelay(10);
}

void phy_hwreset_soc12m_ar8035(void)
{
	u32 tmp;

	DBG("%s\n", __FUNCTION__);

	tmp = readl(__io_address(EPSON12_GPIO_BASE) + 0x0C); /* GPIO_HD_A_SEL */
	DBG("\t[PHY] GPIO_HD_A_SEL-1=0x%08x\n", tmp);
	tmp &= ~0x00100000;
	DBG("\t[PHY] GPIO_HD_A_SEL-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_GPIO_BASE) + 0x0C); /* GPIO_HD_A_SEL */

	writel(0x00100000, __io_address(EPSON12_GPIO_BASE) + 0x28); /* GPIO_HD_A_DO_RESET */
	tmp = readl(__io_address(EPSON12_GPIO_BASE) + 0x04); /* GPIO_HD_A_DIR_SET */
	DBG("\t[PHY] GPIO_HD_A_DIR_SET-1=0x%08x\n", tmp);
	tmp |= 0x00100000;
	DBG("\t[PHY] GPIO_HD_A_DIR_SET-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_GPIO_BASE) + 0x04); /* GPIO_HD_A_DIR_SET */

	mdelay(1);

	writel(0x00100000, __io_address(EPSON12_GPIO_BASE) + 0x24); /* GPIO_HD_A_DO_SET */

	mdelay(10);
}
#endif /* defined(CONFIG_MACH_EPSON12_M) */

#if defined(CONFIG_MACH_EPSON12_H)
void phy_hwreset_soc12h_bcm54610_divf(void)
{
	DBG("%s\n", __FUNCTION__);

	writel(0x00000080, __io_address(EPSON12_LFP_IOU) + 0x3044); /* OUT_LOW */
	writel(0x00000080, __io_address(EPSON12_LFP_IOU) + 0x3080); /* OUT_SEL */

	mdelay(1);

	writel(0x00000080, __io_address(EPSON12_LFP_IOU) + 0x3084); /* OUT_HIGH */

	mdelay(10);
}

void phy_hwreset_soc12h_ar8035_divl(void)
{
	u32 tmp;

	DBG("%s\n", __FUNCTION__);

	tmp = readl(__io_address(EPSON12_LP_MISC) + 0x8C);
	DBG("\t[PHY] LP_MISC_IOP_OUT1-1=0x%08x\n", tmp);
	tmp &= 0xfffffffd;
	DBG("\t[PHY] LP_MISC_IOP_OUT1-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LP_MISC) + 0x8C);

	tmp = readl(__io_address(EPSON12_LP_MISC) + 0x94);
	DBG("\t[PHY] LP_MISC_IOP_ENB1-1=0x%08x\n", tmp);
	tmp |= 0x2;
	DBG("\t[PHY] LP_MISC_IOP_ENB1-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LP_MISC) + 0x94);

	mdelay(1);

	tmp = readl(__io_address(EPSON12_LP_MISC) + 0x8C);
	DBG("\t[PHY] LP_MISC_IOP_OUT1-1=0x%08x\n", tmp);
	tmp |= 0x2;
	DBG("\t[PHY] LP_MISC_IOP_OUT1-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LP_MISC) + 0x8C);

	mdelay(10);
}

void phy_hwreset_soc12h_ar8035_divf(void)
{
	u32 tmp;

	DBG("%s\n", __FUNCTION__);

	tmp = readl(__io_address(EPSON12_LFP_IOU) + 0x2204);
	DBG("\t[PHY] POUT_PB_2-1=0x%08x\n", tmp);
	tmp &= ~0x00000004;
	DBG("\t[PHY] POUT_PB_2-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LFP_IOU) + 0x2204);

	tmp = readl(__io_address(EPSON12_LFP_IOU) + 0x2200);
	DBG("\t[PHY] IOSEL_PB_2-1=0x%08x\n", tmp);
	tmp |= 0x00000004;
	DBG("\t[PHY] IOSEL_PB_2-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LFP_IOU) + 0x2200);

	mdelay(1);

	tmp = readl(__io_address(EPSON12_LFP_IOU) + 0x2204);
	DBG("\t[PHY] POUT_PB_2-1=0x%08x\n", tmp);
	tmp |= 0x00000004;
	DBG("\t[PHY] POUT_PB_2-2=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LFP_IOU) + 0x2204);

	mdelay(10);
}

void phy_hwreset_soc12h_ar8035_divi(void)
{
	u32 tmp;

	DBG("%s\n", __FUNCTION__);

        tmp = ~0x00000010; /* PB_100 = 'L' */
        DBG("\t[PHY] PORT_PB_3_AND=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LFP_IOU) + 0x2344);

        mdelay(1);

	tmp = 0x00000010; /* PB_100 = 'H' */
	DBG("\t[PHY] PORT_PB_3_OR=0x%08x\n", tmp);
	writel(tmp, __io_address(EPSON12_LFP_IOU) + 0x2384);

	mdelay(10);
}
#endif /* defined(CONFIG_MACH_EPSON12_H) */

void fix_gmac_speed(void *priv, unsigned int speed)
{
	u32 tmp;

	/* Set SAMSUNG MISC_IPMODE12 */
	tmp = readl(__io_address(EPSON12_MISC_BASE) + 0x30);
	tmp &= GMAC_CLK_DIVIDER_MASK;

	switch (speed) {
		case 1000:
			tmp |= GMAC_CLK_DIVIDER_1000;
			break;
		case 100:
			tmp |= GMAC_CLK_DIVIDER_100;
			break;
		case 10:
			tmp |= GMAC_CLK_DIVIDER_10;
			break;
		default:
			printk(KERN_WARNING "Speed (%d) is not 10"
				" or 100, 1000!\n", speed);
			return;
	}

	writel(tmp, __io_address(EPSON12_MISC_BASE) + 0x30);
}

int gmac_gigabit_init(struct platform_device *pdev)
{
	u32 gmac_cfg = readl(__io_address(EPSON12_GMAC_CFG_BASE));

#ifndef CONFIG_EPSON12_WITH_RTOS /* Core1(Linux) doesn't need to set */
#if defined(CONFIG_MACH_EPSON12_M)
	/* Drivability adjustment SAMSUNG MISC DS0/1 */
	writel(0x0059f802, __io_address(EPSON12_MISC_BASE) + 0xB8); /* MISC_IOMODE14 */
	writel(0x0059f807, __io_address(EPSON12_MISC_BASE) + 0xC0); /* MISC_IOMODE16 */
#endif
        /* PHY hardware reset */
	phy_hwreset();
#endif
	/* Change GMAC RGMII mode clock */
	writel(gmac_cfg | GMAC_SEL_MII_RGMII(GMAC_SEL_RGMII),
		__io_address(EPSON12_GMAC_CFG_BASE));

	return 0;
}

int gmac_additional_init(struct platform_device *pdev)
{
	int ret = 0, phy_addr = stmmacphy_data.phy_addr;
	u32 phy_id;

	phy_id = epson12_mdio_read(phy_addr, MII_PHYSID1);
	phy_id = (phy_id << 16) | epson12_mdio_read(phy_addr, MII_PHYSID2);
	DBG("\t[PHY] phy_id=0x%08x\n", phy_id);
	
	if (PHYID_MASK(phy_id) == PHYID_MASK(stmmacphy_data.phy_id))
	{
	        return (PHYID_MASK(phy_id) == PHYID_MASK(PHYID_BCM54610) ||
			PHYID_MASK(phy_id) == PHYID_MASK(PHYID_AR8035)) ?
			gmac_gigabit_init(pdev) : ret;
	}

	pr_warn("Ethernet PHY different from the setting value has been detected!\n"
		"Switch to the valid PHY...\n");
		
	for (phy_addr = 0; phy_addr < 16; phy_addr++) {
		phy_id = epson12_mdio_read(phy_addr, MII_PHYSID1);

		if (phy_id != 0xffff) {
			phy_id = (phy_id << 16) | epson12_mdio_read(phy_addr, MII_PHYSID2);
			DBG("\t[PHY] ADDR=%d, PHY ID=0x%08x\n", phy_addr, phy_id);
			break;
		}
	}

	if (phy_addr < 16) {
		stmmacphy_data.phy_addr = phy_addr;
		stmmacphy_data.phy_id = phy_id;

		switch (PHYID_MASK(phy_id)) {
		case PHYID_MASK(PHYID_RTL8201F):
			DBG("\t[PHY] device is RTL8201F\n");

			stmmac_data.pbl = 4; /* 8xPBL by default */
			stmmac_data.fix_mac_speed = NULL;
			stmmacphy_data.interface = PHY_INTERFACE_MODE_MII;
			stmmacphy_data.phy_reset = rtl8201f_phy_reset;
			break;
		case PHYID_MASK(PHYID_BCM54610):
			DBG("\t[PHY] device is BCM54610\n");

			stmmac_data.pbl = 63; /* 8xPBL by default */
			stmmac_data.fix_mac_speed = fix_gmac_speed;
			stmmacphy_data.interface = PHY_INTERFACE_MODE_RGMII;
			stmmacphy_data.phy_reset = bcm54610_phy_reset;

			gmac_gigabit_init(pdev);
			break;
		case PHYID_MASK(PHYID_AR8035):
			DBG("\t[PHY] device is AR8035\n");

			stmmac_data.pbl = 63; /* 8xPBL by default */
			stmmac_data.fix_mac_speed = fix_gmac_speed;
			stmmacphy_data.interface = PHY_INTERFACE_MODE_RGMII;
			stmmacphy_data.phy_reset = ar8035_phy_reset;

			gmac_gigabit_init(pdev);
			break;
		default:
			DBG("\t[PHY] device is unknown\n");
			stmmacphy_data.phy_addr = -1;
			ret = -ENODEV;
		}
	} else {
		pr_err("\t[PHY] We don't have a PHY, so do nothing\n");
		stmmacphy_data.phy_addr = -1;
		ret = -ENODEV;
	}

	return ret;
}


int gmac_switch_led_on_off()
{
	int ret = 0, phy_addr = stmmacphy_data.phy_addr;
	u32 phy_id;
	unsigned long reg_tmp, flag_giga = 0;

	struct stmmac_priv *priv;

#if defined(CONFIG_POWER_SAVING_SWITCH_L1)
	phy_id = stmmacphy_data.phy_id;

	switch (PHYID_MASK(phy_id)) {
	case PHYID_MASK(PHYID_RTL8201F):
		PSDBG("\t[PHY] device is RTL8201F\n");

		epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 7); /* page reset */
		reg_tmp = epson12_mdio_read(phy_addr, 19);
		if (g_PowerSaveMode == EPPSM_L1) {
			reg_tmp |= (0x1 << 6);		/* 6bit */
		} else {
			reg_tmp &= ~(0x1 << 6);		/* 6bit */
		}
		PSDBG("\t[PHY] Basic mode control register 19-after[0x%x]\n", reg_tmp);
		epson12_mdio_write(phy_addr, 19, reg_tmp);

		epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 0); /* page reset */
		break;
	case PHYID_MASK(PHYID_BCM54610):
		PSDBG("\t[PHY] device is BCM54610\n");
		break;
	case PHYID_MASK(PHYID_AR8035):
		PSDBG("\t[PHY] device is AR8035\n");

		flag_giga = 0;

		if (g_stmmac_dev != NULL) {
			priv = netdev_priv(g_stmmac_dev);
			PSDBG("\t[PHY] link speed=%d\n", (int)(priv->speed));
			if (priv->speed == 1000)
				flag_giga = 1;
		} else {
			reg_tmp = epson12_mdio_read(phy_addr, MII_BMCR);
			if (reg_tmp & BMCR_ANENABLE) {
				/* Regiter Offset 0x11 : PHY specific status register */
				/* 15:14 SPEED	10=1000Mbps,01=100Mbps,00=10Mbps */
				reg_tmp = epson12_mdio_read(phy_addr, AR8035_PHY_SPECIFIC_STATUS);
				if ((reg_tmp & (0x1 << 15)) && !(reg_tmp & (0x1 << 14)))
					flag_giga = 1;
			}
		}

		/* Regiter Offset 0x19 : Manual LED override register */
		/* 7:6 LED_LINK10_100 CONTROL */
		/* 3:2 LED_RX */
		/* 1:0 LED_TX */
		reg_tmp = epson12_mdio_read(phy_addr, AR8035_MANUAL_LED_OVERRIDE);
		if (flag_giga) {				/* 1000 Base-T */
			if (g_PowerSaveMode == EPPSM_L1) {
				reg_tmp |= 0x000000ca;		/* 7,6,3,1bit on */
				reg_tmp &= 0xfffffffa;		/* 2,0bit off */
			} else {
				reg_tmp &= 0xffffff30;		/* 7,6,3,2,1,0bit off */
			}
		} else {					/* 100 Base-T or 10Base-T */
			reg_tmp &= 0xffffff30;
		}
		epson12_mdio_write(phy_addr, AR8035_MANUAL_LED_OVERRIDE, reg_tmp);

		/* Regiter Offset 0x18 : LED control register */
		/* 15 DISABLE_LED */
		reg_tmp = epson12_mdio_read(phy_addr, AR8035_LED_CTRL);
		if (flag_giga) {				/* 1000 Base-T */
			reg_tmp &= ~(0x1 << 15);
		} else {					/* 100 Base-T or 10Base-T */
			if (g_PowerSaveMode == EPPSM_L1) {
				reg_tmp |= (0x1 << 15);
			} else {
				reg_tmp &= ~(0x1 << 15);
			}
		}
		epson12_mdio_write(phy_addr, AR8035_LED_CTRL, reg_tmp);

		break;
	default:
		DBG("\t[PHY] device is unknown\n");

	}
#endif	/* CONFIG_POWER_SAVING_SWITCH_L1 */

	return ret;
}
EXPORT_SYMBOL(gmac_switch_led_on_off);


/**
 * epson12_mdio_read
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * Description: it reads data from the MII register from within the phy device.
 *
 * Base on linux/drivers/net/stmmac/stmmac_mdio.c
 */
int epson12_mdio_read(int phyaddr, int phyreg)
{
	int data, clk_csr = stmmac_data.clk_csr;
	void __iomem *ioaddr = __io_address(EPSON12_GMAC_BASE);

	u16 regValue = (((phyaddr << 11) & (0x0000F800)) |
			((phyreg << 6) & (0x000007C0)));
	regValue |= MII_BUSY | ((clk_csr & 7) << 2);

	do {} while (((readl(ioaddr + GMAC_MII_ADDR)) & MII_BUSY) == 1);
	writel(regValue, ioaddr + GMAC_MII_ADDR);
	do {} while (((readl(ioaddr + GMAC_MII_ADDR)) & MII_BUSY) == 1);

	/* Read the data from the MII data register */
	data = (int)readl(ioaddr + GMAC_MII_DATA);

	return data;
}

/**
 * epson12_mdio_write
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 *
 * Base on linux/drivers/net/stmmac/stmmac_mdio.c
 */
int epson12_mdio_write(int phyaddr, int phyreg, u16 phydata)
{
	int clk_csr = stmmac_data.clk_csr;
	void __iomem *ioaddr = __io_address(EPSON12_GMAC_BASE);

	u16 value =
	    (((phyaddr << 11) & (0x0000F800)) | ((phyreg << 6) & (0x000007C0)))
	    | MII_WRITE;

	value |= MII_BUSY | ((clk_csr & 7) << 2);


	/* Wait until any existing MII operation is complete */
	do {} while (((readl(ioaddr + GMAC_MII_ADDR)) & MII_BUSY) == 1);

	/* Set the MII address register to write */
	writel(phydata, ioaddr + GMAC_MII_DATA);
	writel(value, ioaddr + GMAC_MII_ADDR);

	/* Wait until any existing MII operation is complete */
	do {} while (((readl(ioaddr + GMAC_MII_ADDR)) & MII_BUSY) == 1);

	return 0;
}


int rtl8201f_phy_reset(void *priv)
{
        int phy_addr = stmmacphy_data.phy_addr;
        u16 tmp;
	unsigned long reg_tmp;

	DBG("%s\n", __FUNCTION__);

	tmp = epson12_mdio_read(phy_addr, MII_BMCR);
	DBG("\t[PHY] Basic mode control register tmp-1[0x%04x]\n", tmp);
	tmp |= BMCR_RESET;
	DBG("\t[PHY] Basic mode control register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, MII_BMCR, tmp);

	mdelay(250);

	tmp = epson12_mdio_read(phy_addr, MII_BMCR);
	DBG("\t[PHY] Basic mode control register tmp-3[0x%04x]\n", tmp);

	if (epson12_mdio_read(phy_addr, MII_BMCR) & BMCR_RESET)
		return -EBUSY;

	/* CRS will be asserted when TXEN is asserted in Half-duplex mode */
	DBG("\t[PHY] CRS asserted in Half-duplex mode\n");
	epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 5); /* set page 5 */

	tmp = epson12_mdio_read(phy_addr, 16);
	DBG("\t[PHY] Page5 Register16 tmp2-1[0x%04x]\n", tmp);
	tmp |= 0x1;
	DBG("\t[PHY] Page5 Register16 tmp2-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 16, tmp);
	tmp = epson12_mdio_read(phy_addr, 16);
	DBG("\t[PHY] Page5 Register16 tmp2-3[0x%04x]\n", tmp);

	/* Traditional LED function selection */
	DBG("\t[PHY] Select LED function\n");
        epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 7); /* set page 7 */

	tmp = epson12_mdio_read(phy_addr, 19);
	DBG("\t[PHY] Page7 Register19 tmp2-1[0x%04x]\n", tmp);
	tmp &= RTL8201F_LED_MASK;
	tmp |= RTL8201F_LED_SELECT;
	DBG("\t[PHY] Page7 Register19 tmp2-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 19, tmp);
	tmp = epson12_mdio_read(phy_addr, 19);
	DBG("\t[PHY] Page7 Register19 tmp2-3[0x%04x]\n", tmp);

	epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 0); /* page reset */

#ifdef CONFIG_POWER_SAVING_SWITCH_L1
	if (g_PowerSaveMode == EPPSM_L1) {
		epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 7); /* page reset */
		reg_tmp = epson12_mdio_read(phy_addr, 19);
		reg_tmp |= (0x1 << 6);	// 6bit
		DBG("\t[PHY] rtl8201f_phy_reset LED off [0x%x]\n", reg_tmp);
		epson12_mdio_write(phy_addr, 19, reg_tmp);

		epson12_mdio_write(phy_addr, RTL8201F_PAGE_SELECT, 0); /* page reset */
	}
#endif

	/* Disable power saving mode */
	tmp = epson12_mdio_read(phy_addr, RTL8201F_POWER_SAVING_MODE);
	DBG("\t[PHY] Power saving mode register tmp3-1[0x%04x]\n", tmp);
	tmp &= ~0x8000;
	DBG("\t[PHY] Power saving mode register tmp3-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, RTL8201F_POWER_SAVING_MODE, tmp);
	tmp = epson12_mdio_read(phy_addr, RTL8201F_POWER_SAVING_MODE);
	DBG("\t[PHY] Power saving mode register tmp3-3[0x%04x]\n", tmp);

	return 0;
}

int bcm54610_phy_reset(void *priv)
{
        int phy_addr = stmmacphy_data.phy_addr;
        u16 tmp;

	DBG("%s\n", __FUNCTION__);

	tmp = epson12_mdio_read(phy_addr, MII_BMCR);
	DBG("\t[PHY] Basic mode control register tmp-1[0x%04x]\n", tmp);
	tmp |= BMCR_RESET;
	DBG("\t[PHY] Basic mode control register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, MII_BMCR, tmp);

	mdelay(10);

	tmp = epson12_mdio_read(phy_addr, MII_BMCR);
	DBG("\t[PHY] Basic mode control register tmp-3[0x%04x]\n", tmp);

	if (epson12_mdio_read(phy_addr, MII_BMCR) & BMCR_RESET)
		return -EBUSY;

        DBG("\t[PHY] Set LED register\n");
        epson12_mdio_write(phy_addr, MII_NCONFIG, 0xa410); /* 1010 0100 0001 0000 */
        epson12_mdio_write(phy_addr, MII_NCONFIG, 0x4406); /* 1000 1000 0000 0110 */
        epson12_mdio_write(phy_addr, MII_NCONFIG, 0xb463); /* 1011 0100 0110 0011 */
        epson12_mdio_write(phy_addr, MII_NCONFIG, 0xb810); /* 1011 1000 0001 0000 */

	return 0;
}

int ar8035_phy_reset(void *priv)
{
	int phy_addr = stmmacphy_data.phy_addr;
	u16 tmp;
	unsigned long reg_tmp;
	
	/* PHY hardware reset */
	phy_hwreset();

#ifdef CONFIG_POWER_SAVING_SWITCH_L1
	if (g_PowerSaveMode == EPPSM_L1) {
		reg_tmp = epson12_mdio_read(phy_addr, 0x18);
		reg_tmp |= (0x1 << 15);	// 15bit
		PSDBG("\t[PHY] ar8035_phy_reset LED off [0x%x]\n", reg_tmp);
		epson12_mdio_write(phy_addr, 0x18, reg_tmp);
	}
#endif

	/* Set the RGMII delay */
	tmp = epson12_mdio_read(phy_addr, 0x1D);
	DBG("\t[PHY] Debug port address offset register tmp-1[0x%04x]\n", tmp);
	tmp = 0x05;
	DBG("\t[PHY] Debug port address offset register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x1D, tmp);

	tmp = epson12_mdio_read(phy_addr, 0x1E);
	DBG("\t[PHY] RGMII tx clock delay control register tmp-1[0x%04x]\n", tmp);
	tmp |= (0x1 << 8);
	DBG("\t[PHY] RGMII tx clock delay control register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x1E, tmp);

	/* Disable hibernate */
	tmp = epson12_mdio_read(phy_addr, 0x1D);
	DBG("\t[PHY] Debug port address offset register tmp-1[0x%04x]\n", tmp);
	tmp = 0x0b;
	DBG("\t[PHY] Debug port address offset register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x1D, tmp);

	tmp = epson12_mdio_read(phy_addr, 0x1E);
	DBG("\t[PHY] Hib ctrl and rgmii gtx clock delay register tmp-1[0x%04x]\n", tmp);
	tmp &= 0x7fff;
	DBG("\t[PHY] Hib ctrl and rgmii gtx clock delay register tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x1E, tmp);

	epson12_mdio_write(phy_addr, 0x1D, 0); /* debug port reset */

#if defined(CONFIG_ETHPHY_HWRESET_12H_AR8035_DIVI)
	/* MMD7 turn off CLK_25MHz */
	epson12_mdio_write(phy_addr, 0x0D, 0x7); /* set the device address */
	epson12_mdio_write(phy_addr, 0x0E, 0x8017); /* set the register offset address */
	epson12_mdio_write(phy_addr, 0x0D, 0x4007); /* set the function(keep the device address) */

	tmp = epson12_mdio_read(phy_addr, 0x0E); /* data from register 0x8017 of MMD7 */
	DBG("\t[PHY] MMD7 offset 0x8017 tmp-1[0x%04x]\n", tmp);
	tmp |= 0x0800;
	DBG("\t[PHY] MMD7 offset 0x8017 tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x0E, tmp); /* write to register 0x8017 of MMD7 */
	tmp = epson12_mdio_read(phy_addr, 0x0E); /* data from register 0x8017 of MMD7 */
	DBG("\t[PHY] MMD7 offset 0x8017 tmp-3[0x%04x]\n", tmp);

	epson12_mdio_write(phy_addr, 0x0D, 0x0); /* reset the device address */
#endif /* defined(CONFIG_ETHPHY_HWRESET_12H_AR8035_DIVI) */

#if defined(CONFIG_ETHPHY_HWRESET_12M_AR8035)
	/* MMD7 turn on CLK_125MHz & driver strength */
	epson12_mdio_write(phy_addr, 0x0D, 0x7); /* set the device address */
	epson12_mdio_write(phy_addr, 0x0E, 0x8016); /* set the register offset address */
	epson12_mdio_write(phy_addr, 0x0D, 0x4007); /* set the function(keep the device address) */

	tmp = epson12_mdio_read(phy_addr, 0x0E); /* data from register 0x8016 of MMD7 */
	DBG("\t[PHY] MMD7 offset 0x8016 tmp-1[0x%04x]\n", tmp);
	tmp &= ~0x0198;
	tmp |= 0x0018;
	DBG("\t[PHY] MMD7 offset 0x8016 tmp-2[0x%04x]\n", tmp);
	epson12_mdio_write(phy_addr, 0x0E, tmp); /* write to register 0x8016 of MMD7 */
	tmp = epson12_mdio_read(phy_addr, 0x0E); /* data from register 0x8016 of MMD7 */
	DBG("\t[PHY] MMD7 offset 0x8016 tmp-3[0x%04x]\n", tmp);

	epson12_mdio_write(phy_addr, 0x0D, 0x0); /* reset the device address */
#endif /* defined(CONFIG_ETHPHY_HWRESET_12M_AR8035) */

	tmp = epson12_mdio_read(phy_addr, 0x11);
	DBG("\t[PHY] PHY-Specific Status register tmp-1[0x%04x]\n", tmp);

	return 0;
}
