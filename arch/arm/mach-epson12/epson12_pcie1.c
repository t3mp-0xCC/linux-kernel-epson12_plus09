/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/mbus.h>
#include <asm/irq.h>
#include <asm/signal.h>
#include <asm/mach/pci.h>
#include <mach/epson12.h>
#include <linux/delay.h>

#include "epson12_pcie_regs.h"
#include "epson12_pcie.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EPSON12_PCIE_LNK_BASE	(EPSON12_PCIE1_BASE)
#define EPSON12_PCIE_DBI_BASE	(EPSON12_PCIE_DBI1_BASE)
#define EPSON12_PCIE_AREA_SIZE	(256*1024*1024)
#define IRQ_PCIE		(IRQ_PCIE1)
#define PCIE_PORT_NUM		(1)


/*
 ** PCIe Area Information
 **
 ** 0x08000000	 ~ 0x0BFFFFFF		PCIe Memory BASE boundary (64MB)
 ** 0x08000000	 ~ 0x0807FFFF		PCIe CFG0 Area	(512KB)
 ** 0x08080000	 ~ 0x080FFFFF		PCIe CFG1 Area	(512KB)
 ** 0x08100000	 ~ 0x081FFFFF		PCIe IO Area	(1MB)
 ** 0x08200000	 ~ 0x082FFFFF		PCIe ATU Area	(1MB)
 ** 0x08300000	 ~ 0x083FFFFF		PCIe MSI Area	(1MB)
 ** 0x08400000	 ~ 0x0BFFFFFF		PCIe MEM Area	(60MB)
 **
 **/

#define PCIE_AREA_BASE	EPSON12_PCIE_LNK_BASE		// PCIe Space (0xC0000000 ~ 0xD7FFFFFF :64MB)
#define PCIE_AREA_SIZE	EPSON12_PCIE_AREA_SIZE
#define PCIE_CFG_BASE	PCIE_AREA_BASE			// Config Space (0x08000000 ~ 0x080FFFFF :1MB)
#define PCIE_CFG_SIZE	1*1024*1024
#define PCIE_CFG0_BASE	PCIE_CFG_BASE			// Cnfg0 Space: 0x08000000 ~ 0x0807FFFF
#define PCIE_CFG0_SIZE	512*1024
#define PCIE_CFG1_BASE	PCIE_CFG0_BASE+PCIE_CFG0_SIZE	// Cnfg1 Space: 0x08080000 ~ 0x080FFFFF
#define PCIE_CFG1_SIZE	512*1024
#define PCIE_IO_BASE	PCIE_CFG_BASE+PCIE_CFG_SIZE	// IO Space (0x80100000 ~ 0x801FFFFF :1MB)
#define PCIE_IO_SIZE	1*1024*1024
#define PCIE_MSI_BASE	PCIE_IO_BASE+PCIE_IO_SIZE	// MSI Test Space (0x80200000 ~ 0x802FFFFF :1MB)
#define PCIE_MSI_SIZE	1*1024*1024
#define PCIE_ATU_BASE	PCIE_MSI_BASE+PCIE_MSI_SIZE	// iATU Test Space (0x80300000 ~ 0x803FFFFF :1MB)
#define PCIE_ATU_SIZE	1*1024*1024
#define PCIE_MEM_BASE	PCIE_ATU_BASE+PCIE_ATU_SIZE	// Memory Space (0x80400000 ~ 0x83FFFFFF :60MB)
#define PCIE_MEM_SIZE	(PCIE_AREA_SIZE-PCIE_CFG_SIZE-PCIE_IO_SIZE-PCIE_MSI_SIZE-PCIE_ATU_SIZE)
#define PCIE_CFG0_LIMIT	PCIE_CFG0_BASE+PCIE_CFG0_SIZE-1
#define PCIE_CFG1_LIMIT	PCIE_CFG1_BASE+PCIE_CFG1_SIZE-1
#define PCIE_IO_LIMIT	PCIE_IO_BASE+PCIE_IO_SIZE-1
#define PCIE_MSI_LIMIT	PCIE_MSI_BASE+PCIE_MSI_SIZE-1
#define PCIE_ATU_LIMIT	PCIE_ATU_BASE+PCIE_ATU_SIZE-1
#define PCIE_MEM_LIMIT	PCIE_MEM_BASE+PCIE_MEM_SIZE-1
#define MSICTR_ADDR	PCIE_AREA_BASE

#define dbi_readb(off)		__raw_readb(dbi + (off))
#define dbi_readw(off)		__raw_readw(dbi + (off))
#define dbi_readl(off)		__raw_readl(dbi + (off))
#define dbi_writeb(val, off)	__raw_writeb(val, dbi + (off))
#define dbi_writew(val, off)	__raw_writew(val, dbi + (off))
#define dbi_writel(val, off)	__raw_writel(val, dbi + (off))

/* The CFG space of RC starts with DBI */
#define rccfg_readb(off)	dbi_readb(PCIE_RCCFG_OFF + (off))
#define rccfg_readw(off)	dbi_readw(PCIE_RCCFG_OFF + (off))
#define rccfg_readl(off)	dbi_readl(PCIE_RCCFG_OFF + (off))
#define rccfg_writeb(val, off)	dbi_writeb(val, PCIE_RCCFG_OFF + (off))
#define rccfg_writew(val, off)	dbi_writew(val, PCIE_RCCFG_OFF + (off))
#define rccfg_writel(val, off)	dbi_writel(val, PCIE_RCCFG_OFF + (off))

/* The ATU is located at offset 0x4000 above DBI */
#define atu_readb(off)		dbi_readb(PCIE_ATU_OFF + (off))
#define atu_readw(off)		dbi_readw(PCIE_ATU_OFF + (off))
#define atu_readl(off)		dbi_readl(PCIE_ATU_OFF + (off))
#define atu_writeb(val, off)	dbi_writeb(val, PCIE_ATU_OFF + (off))
#define atu_writew(val, off)	dbi_writew(val, PCIE_ATU_OFF + (off))
#define atu_writel(val, off)	dbi_writel(val, PCIE_ATU_OFF + (off))

#define APP_LTSSM_EN()		atu_writel(atu_readl(PCIE_MODE) | APP_LTSSM_ENABLE, PCIE_MODE)
#define APP_LTSSM_DIS()		atu_writel(atu_readl(PCIE_MODE) & ~APP_LTSSM_ENABLE, PCIE_MODE)
////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void __iomem *dbi, *cfg0_va, *cfg1_va;
static u32 pcie_linkon=0;

#define PCIe_CFG_READ(size) \
	static u32 pcie_cfg_read##size(u8 busnr, u32 devfn, int where)	\
{							\
	u32 val;					\
							\
	if (busnr == 0)	{	/* RC */		\
		val = dbi_read##size(where);		\
	} else if (busnr == 1) {/* Slot */		\
		val = read##size(cfg0_va + where);	\
	} else {					\
		val = read##size(cfg1_va + where);	\
	}						\
	return val;					\
}

#define PCIe_CFG_WRITE(size) \
	static void pcie_cfg_write##size		\
(u8 busnr, u32 devfn, u32 data, int where)		\
{							\
	if (busnr == 0)	{	/* RC */		\
		dbi_write##size(data, where);		\
	} else if (busnr == 1) {/* Slot */		\
		write##size(data, cfg0_va + (where));	\
	} else {					\
		write##size(data, cfg1_va + (where));	\
	}						\
}

	PCIe_CFG_READ(b)
	PCIe_CFG_READ(w)
	PCIe_CFG_READ(l)
	PCIe_CFG_WRITE(b)
	PCIe_CFG_WRITE(w)
	PCIe_CFG_WRITE(l)

static inline bool link_up(void)
{
	if (pcie_linkon)
		return true;
	else
		return false;
}

static int pcie_valid_config(int bus, int dev)
{
	/* Root Port is always present */
	if (bus == 0 && dev == 0)
		return 1;

	/* If the slot has some device indeed */
	if (bus == 1 && dev == 0 && link_up())
		return 1;

	if (bus > 1)
		return 1;

	return 0;
}

/*
 * PCIe config cycles are done by programming the PCIE_CONF_ADDR register
 * and then reading the PCIE_CONF_DATA register. Need to make sure these
 * transactions are atomic.
 */
static DEFINE_SPINLOCK(epson12_pcie_lock);

static int sil_dumped = 0;
static void dump_sil(void)
{
	u32 val;

	val = pcie_cfg_readw(1, 0, 0x0);
	printk("VendorID=%x\n", val);

	val = pcie_cfg_readw(1, 0, 0x2);
	printk("DeviceID=%x\n", val);

	val = pcie_cfg_readw(1, 0, 0x4);
	printk("Command=%x\n", val);

	val = pcie_cfg_readw(1, 0, 0x6);
	printk("Status=%x\n", val);

	val = pcie_cfg_readl(1, 0, 0x8);
	printk("RevID=%x ClassCode=%x\n", val & 0xff, (val >> 8) & 0xffffff);

	val = pcie_cfg_readl(1, 0, 0xc);
	printk("BIST=%x HdrType=%x\n", (val >> 24) & 0xff, (val >> 16) & 0xff);

	val = pcie_cfg_readl(1, 0, 0x10);
	printk("BAR0=%x\n", val);
	val = pcie_cfg_readl(1, 0, 0x14);
	printk("BAR1=%x\n", val);
	val = pcie_cfg_readl(1, 0, 0x18);
	printk("BAR2=%x\n", val);
	val = pcie_cfg_readl(1, 0, 0x1c);
	printk("BAR3=%x\n", val);
	val = pcie_cfg_readl(1, 0, 0x20);
	printk("BAR4=%x\n", val);
	val = pcie_cfg_readl(1, 0, 0x24);
	printk("BAR5=%x\n", val);

	val = pcie_cfg_readl(1, 0, 0x28);
	printk("CIS Pntr=%x\n", val);

	val = pcie_cfg_readl(1, 0, 0x2c);
	printk("SubSysID=%x SubSysVendorId=%x\n", (val >> 16) & 0xffff, (val >> 0) & 0xffff);

	val = pcie_cfg_readl(1, 0, 0x30);
	printk("ROM BASE=%x\n", val);

	val = pcie_cfg_readb(1, 0, 0x34);
	printk("CAP Ptr=%x\n", val);
}

static int pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
	int size, u32 *val)
{
	unsigned long flags;
	int ret = PCIBIOS_SUCCESSFUL;

	if (pcie_valid_config(bus->number, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	//	printk("%s\n", __func__);
	spin_lock_irqsave(&epson12_pcie_lock, flags);

	if (bus->number == 1 && where == 0 && !sil_dumped) {
		sil_dumped = 1;
		dump_sil();
	}

	if (size == 4) {
		*val = pcie_cfg_readl(bus->number, devfn, where);
	} else if (size == 2) {
		*val = pcie_cfg_readw(bus->number, devfn, where);
	} else if (size == 1) {
		*val = pcie_cfg_readb(bus->number, devfn, where);
	} else {
		ret = PCIBIOS_BAD_REGISTER_NUMBER;
	}

	spin_unlock_irqrestore(&epson12_pcie_lock, flags);
#if defined(EPSON12_PCIE_CFG_DEBUG)
	printk("%s: Bus=%d Dev=%d %d@%x => %x\n", __func__, bus->number, PCI_SLOT(devfn), size, where, *val);
#endif

	//	printk("%x\n", *val);
	return ret;
}

static int pcie_wr_conf(struct pci_bus *bus, u32 devfn,
	int where, int size, u32 val)
{
	unsigned long flags;
	int ret = PCIBIOS_SUCCESSFUL;

	if (pcie_valid_config(bus->number, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	//	printk("%s\n", __func__);
#if defined(EPSON12_PCIE_CFG_DEBUG)
	printk("%s: Bus=%d Dev=%d %d@%x => %x\n", __func__, bus->number, PCI_SLOT(devfn), size, where, val);
#endif
	spin_lock_irqsave(&epson12_pcie_lock, flags);

	if (size == 4) {
		pcie_cfg_writel(bus->number, devfn, val, where);
	} else if (size == 2) {
		pcie_cfg_writew(bus->number, devfn, val, where);
	} else if (size == 1) {
		pcie_cfg_writeb(bus->number, devfn, val, where);
	} else {
		ret = PCIBIOS_BAD_REGISTER_NUMBER;
	}

	spin_unlock_irqrestore(&epson12_pcie_lock, flags);

	//	printk("%s:%d Bus=%d Dev=%d %d@%x <= %x\n", __func__, __LINE__, bus->number, PCI_SLOT(devfn), size, where, val);
	return ret;
}

static struct pci_ops pcie_ops = {
	.read = pcie_rd_conf,
	.write = pcie_wr_conf,
};

static void __init epson12_pcie_preinit(void)
{
	///hook_fault_code(16+6, abort_handler, SIGBUS, "imprecise external abort");

	APP_LTSSM_EN();
}

static int __init epson12_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct resource *res;

	/*
	 * Request resources.
	 */
	res = kzalloc(sizeof(struct resource) * 2, GFP_KERNEL);
	if (!res)
		panic("pcie_setup unable to alloc resources");

	/*
	 * IORESOURCE_IO
	 */
	res[0].name = "PCIe I/O Space";
	res[0].flags = IORESOURCE_IO;
	res[0].start = PCIE_IO_BASE;
	res[0].end = res[0].start + PCIE_IO_SIZE - 1;
	if (request_resource(&ioport_resource, &res[0]))
		panic("Request PCIe IO resource failed\n");
	sys->resource[0] = &res[0];

	/*
	 * IORESOURCE_MEM
	 */
	res[1].name = "PCIe Memory Space";
	res[1].flags = IORESOURCE_MEM;
	res[1].start = PCIE_MEM_BASE;
	res[1].end = res[1].start + PCIE_MEM_SIZE - 1;
	if (request_resource(&iomem_resource, &res[1]))
		panic("Request PCIe Memory resource failed\n");
	sys->resource[1] = &res[1];

	sys->resource[2] = NULL;
	sys->io_offset=0;

	return 1;
}

static void __devinit rc_pci_fixup(struct pci_dev *dev)
{
//	printk("%s:%d\n", __func__, __LINE__);
	/*
	 * Prevent enumeration of root complex.
	 */
	if (dev->bus->parent == NULL && dev->devfn == 0) {
		int i;

		for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
			dev->resource[i].start = 0;
			dev->resource[i].end   = 0;
			dev->resource[i].flags = 0;
		}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_EPSON12, PCI_DEVICE_ID_EPSON12, rc_pci_fixup);

static struct pci_bus __init *
	epson12_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;

	if (nr == 0) {
		bus = pci_scan_bus(sys->busnr, &pcie_ops, sys);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int __init epson12_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	return IRQ_PCIE;
}

static void __init epson12_pcie_postinit(void)
{
}

static struct hw_pci epson12_pci __initdata = {
#if defined(CONFIG_PCI_DOMAINS)
	.domain = PCIE_PORT_NUM,
#endif
	.nr_controllers	= 1,
	.swizzle	= pci_std_swizzle,
	.setup		= epson12_pcie_setup,
	.scan		= epson12_pcie_scan_bus,
	.map_irq	= epson12_pcie_map_irq,
	.preinit	= epson12_pcie_preinit,
	.postinit	= epson12_pcie_postinit,
};

#if 0	// no use
static void epson12_pcie_check_link_status(void)
{
	u32 ltssm_state, link_state;

	/* current state of the LTSSM */
	ltssm_state = atu_readl(PCIE_STATE_INFO)&0x1f;
	printk("PCIE> Current state of LTSSM (ATU) : 0x%x(%s)\n", ltssm_state, LINK_STATE_DISP(ltssm_state));

	/* Read port link register to get the link state */
	link_state = (dbi_readl(PL_PT_LNK_R)&(0x3f<<16))>>16;

	if(link_state != ltssm_state)
	{
		DEBUGMSG(DBG_LINK,("PCIe> Port_Force_Link Reg.: 0x%x(%s)\n", link_state, LINK_STATE_DISP(link_state)));
	}
}
#endif	// no use

static u32 epson12_pcie_check_link(void)
{
	int i=0;

	do{
		if( (atu_readl(PCIE_INT0_SIGNAL) & both_link_up) != both_link_up)
		{
			mdelay(10);
			if( ((i++)%30) == 29 ) {
				DEBUGMSG(DBG_LINK, ("PCIe> link off.\n"));
				return false;
			}
		}
		else
		{
			printk("PCIe> Link on.\n");
			return true;
		}

	}while(1);
}
static void epson12_pcie_set_ltssm(int mode)
{
	u32 cur_set, new_set;

	printk("PCIe> app_ltssm_enable will be %s\n", (mode?"enable":"disable"));
	cur_set = atu_readl(PCIE_MODE);

	if(mode) {
		if(cur_set&app_ltssm_enable)
			printk("PCIe> LTSSM is already enabled!\n");
		else {
			printk("PCIe> set app_ltssm_enabled!\n");
			new_set = cur_set | app_ltssm_enable;	//0x00000800
			atu_writel(new_set, PCIE_MODE);
		}
	}else {
		printk("PCIe> Disable LTSSM!\n");
		new_set = cur_set & (~app_ltssm_enable);	//0x00000100
		atu_writel(new_set, PCIE_MODE);
	}

}
static u32 epson12_pcie_get_device_type(void)
{
	u32 device_type;

	device_type = (atu_readl(PCIE_MODE) & 0xF);
	return device_type;

}

static void epson12_pcie_set_iATU_region_indexNdirection(int index, int direction)
{
	u32 value;

	value = dbi_readl(PORTLOG_OFFSET+PL_iATU_VIEWPORT);
	value &= ~(iATU_REGION_INDEX);	// clear Region Index
	value |= index;

	if(direction)
		value |= iATU_REGION_DIRECTION;		// inbound
	else
		value &= ~(iATU_REGION_DIRECTION);	// outbound
	dbi_writel(value, PORTLOG_OFFSET+PL_iATU_VIEWPORT);
}


static void epson12_pcie_set_iATU(void)
{
	printk("PCIe> RC set Own iATU for Transaction\n");

	/*
	 * *  setting for outbound transaction ////////////////////////////////
	 * *
	 * *  Memory Map of EPSON12
	 * *  PCI MEM: 0x0800_0000 ~ 0x0BFF_FFFF 64MB
	 * */

	/* outbound region 0 (cfg type 0) */
	epson12_pcie_set_iATU_region_indexNdirection(0, REGION_OUTBOUND);

	dbi_writel(PCIE_CFG0_BASE, PORTLOG_OFFSET+PL_iATU_LOWERBASE_ADDR);		// set lower base address
	dbi_writel(0x0, PORTLOG_OFFSET+PL_iATU_UPPERBASE_ADDR);				// set upper base address
	dbi_writel(PCIE_CFG0_LIMIT, PORTLOG_OFFSET+PL_iATU_LIMIT_ADDR);			// set limit
	dbi_writel( ((0x1<< 24) | (0x0 << 16)), PORTLOG_OFFSET+PL_iATU_LOWERTGT_ADDR);	// set lower target address
	dbi_writel(0x0, PORTLOG_OFFSET+PL_iATU_UPPERTGT_ADDR);				// set upper target address
	dbi_writel(0x4, PORTLOG_OFFSET+PL_iATU_REGION_CNT1);				// set transaction type
	dbi_writel(iATU_REGION_ENABLE, PORTLOG_OFFSET+PL_iATU_REGION_CNT2);		// enable region


	/* outbound region 1 (cfg type 1) */
	epson12_pcie_set_iATU_region_indexNdirection(1, REGION_OUTBOUND);

	dbi_writel(PCIE_CFG1_BASE, PORTLOG_OFFSET+PL_iATU_LOWERBASE_ADDR);		// set lower base address
	dbi_writel(0x0, PORTLOG_OFFSET+PL_iATU_UPPERBASE_ADDR);				// set upper base address
	dbi_writel(PCIE_CFG1_LIMIT, PORTLOG_OFFSET+PL_iATU_LIMIT_ADDR);			// set limit
	dbi_writel( ((2 << 24)|(0 << 16)), PORTLOG_OFFSET+PL_iATU_LOWERTGT_ADDR);	// set lower target address
	dbi_writel(0x0, PORTLOG_OFFSET+PL_iATU_UPPERTGT_ADDR);				// set upper target address
	dbi_writel(0x5, PORTLOG_OFFSET+PL_iATU_REGION_CNT1);				// set transaction type
	dbi_writel(iATU_REGION_ENABLE, PORTLOG_OFFSET+PL_iATU_REGION_CNT2);		// enable region
}


static void epson12_pcie_init_interrupt(u32 int_mode)		// int_mode: interrupt mode, 0:INTx, 1:MSI
{
	u32 CMD_STS_Reg, INTx_Ass_Disable;
	u32 value;

	/* Add interrupt here... */
	/*    IntSet(NUM_PCIE, ISR_PCIe, 0); */

	if(int_mode)
	{
		// set address for MSI Controller Address register
		dbi_writel(MSICTR_ADDR, PL_MSI_CNT_ADDR);
		dbi_writel(0xffffffff, PL_MSI_CNT_INT_EN(0));
		printk(" MSI Controller Address: 0x%x\n", dbi_readl(PL_MSI_CNT_ADDR));
	}
	if(1)
	{

		/* we must set the EP's interrupt mode */
		CMD_STS_Reg=readl(cfg0_va+PCI_COMMAND);

		if(int_mode)
			CMD_STS_Reg |= (0x1<<10);	// 1: MSI mode
		else
			CMD_STS_Reg &= ~(0x1<<10);	// 0: INTx mode

		writel(CMD_STS_Reg, cfg0_va+PCI_COMMAND);

		/* check the value */
		CMD_STS_Reg=readl(cfg0_va + PCI_COMMAND);
		INTx_Ass_Disable=CMD_STS_Reg&(0x1<<10); // extract Interrupt Disable bit
		if(INTx_Ass_Disable)
			printk(" Enable 'Interrupt Disable bit' --> MSI mode\n");
		else
			printk(" Disable 'Interrupt Disable bit' --> INTx mode\n");


		if(int_mode) {
			writel(  (readl(cfg0_va+MSI_CTR) | (MSI_CTR_Enbale_MSI|MSI_CTR_Enbale_MultiMSI)), cfg0_va+MSI_CTR);
			writel( MSICTR_ADDR, cfg0_va+MSI_L32); // added on 2010/10/28
		}
		else
		{
			writel( (readl(cfg0_va + MSI_CTR) & (~(MSI_CTR_Enbale_MSI|(0x3f<<17)))) , cfg0_va+MSI_CTR);
		}
		value = (readl( cfg0_va + MSI_CTR) & MSI_CTR_Enbale_MSI);
		printk("PCIe> %s 'Enable_MSI bit of EP'\n", (value?"Enable":"Disable"));
	}

	value = (0x1<<24);			// enable legacy_inta
	atu_writel(value, PCIE_INT0_ENABLE);

}

static u32 dwcpcie_init(u32 on_ltssm, u32 ip_mode)
{
	u32 PCIeIP_ID;
	u32 PCIeIP_vendorID, PCIeIP_deviceID;

	PCIeIP_ID = dbi_readl(PCI_VENDOR_ID);
	PCIeIP_vendorID=PCIeIP_ID&0xffff;
	PCIeIP_deviceID=(PCIeIP_ID>>16)&0xffff;
	if(PCIeIP_vendorID==0xffff || PCIeIP_deviceID==0) {
		printk(" Fail Reading PCIe IP Vendor ID\n");
		return false;
	}
	printk("PCIe> Vendor ID:0x%04x, Device ID:0x%04x\n", PCIeIP_vendorID, PCIeIP_deviceID);

	if(on_ltssm) {
		epson12_pcie_set_ltssm(LINK_ENABLE);
		pcie_linkon=epson12_pcie_check_link();
	}

	printk("PCIe> Device Mode : %d\n", epson12_pcie_get_device_type());
	///
	//
	// pm state ??
	////

	/* Bus Master & Memory Space Enable */
	dbi_writel( (dbi_readl(PCI_COMMAND) | 0x6) , PCI_COMMAND);

	/* Change Class ID to Bridge support */
	dbi_writew(PCI_CLASS_BRIDGE_PCI, PCI_CLASS_DEVICE);

	/* 32-bit IO Space */
	//  dbi_writeb(0x1, PCI_IO_BASE);
	// dbi_writeb(0x1 ,PCI_IO_LIMIT);

	/* 32-bit MEM Space */
	//dbi_writeb(0x0, PCI_PREF_MEMORY_BASE);
	//dbi_writeb(0x0, PCI_PREF_MEMORY_LIMIT);


	if(pcie_linkon && (ip_mode == PCIe_RC)) {
		epson12_pcie_set_iATU();
		//dwcpice_Dump_iATU();		// echo iATU setting
		epson12_pcie_init_interrupt(0);// 0: INTx, 1:MSI
	}

#if 0

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable MSI Enable"));

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable PME Enable bit"));
	//	dbi_readl(CFG_PM_CAP+PCI_PM_CTRL) |= PCI_PM_CTRL_PME_ENABLE;

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable Device Control Register"));
	dbi_writel( (dbi_readl(DEV_STS_CTRL) | 0xf), DEV_STS_CTRL);

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable Slot Control"));
	dbi_writel( (dbi_readl(SLT_STS_CTRL) | 0x1fff), SLT_STS_CTRL);

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable Root Control PME Interrupt Enable"));
	dbi_writel( (dbi_readl(RC_CAP_CTRL) | (0x1<<3)), RC_CAP_CTRL);

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable Root Control for Error Enable"));
	dbi_writel( 0xf, RC_CAP_CTRL);

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable Root Error Command for Error Enable\n"));
	dbi_writel( 0x7, PCIE_XCAP_ROOTERR_CMD);

	DEBUGMSG(DBG_REG, ("\nPCIe> Enable UR reporting enable of [device control register]"));
	dbi_writel(0xf, DEV_STS_CTRL);
#endif
	return PCIeIP_ID;
}

static int __init epson12_pcie_init(void)
{
	printk("\nepson12_pcie_init port:%d!!!\n", PCIE_PORT_NUM);

	dbi = ioremap(EPSON12_PCIE_DBI_BASE, EPSON12_PCIE_DBI_LEN);
	if (!dbi)
		printk("DBI IOREMAP FAILED !!!\n");

	cfg0_va = ioremap(PCIE_CFG0_BASE, PCIE_CFG0_SIZE+PCIE_CFG1_SIZE);
	if (!cfg0_va)
		printk("PCIe CFG IOREMAP FAILED !!!\n");
	cfg1_va = cfg0_va + PCIE_CFG0_SIZE;

	printk("PCIE_DBI 0x%x -> 0x%p [%x]\n", EPSON12_PCIE_DBI_BASE, dbi, EPSON12_PCIE_DBI_LEN);
	printk("PCIE_CFG0 0x%x -> 0x%p [%x]\n", PCIE_CFG0_BASE, cfg0_va, PCIE_CFG0_SIZE);
	printk("PCIE_CFG1 0x%x -> 0x%p [%x]\n", PCIE_CFG1_BASE, cfg1_va, PCIE_CFG1_SIZE);
	printk("PCIE_MEM_BASE 0x%x [%x]\n", PCIE_MEM_BASE, PCIE_MEM_SIZE);

	dwcpcie_init(1, PCIe_RC);
	pci_common_init(&epson12_pci);

	return 0;
}

subsys_initcall(epson12_pcie_init);

////////////////////////////////////////////////////////////////////////////////////////////
static unsigned long psave_flags;

static u32 bus0_pci_command;		//16	04
static u32 bus0_pci_status;		//16	06
static u32 bus0_pci_cache_line_size;	//8	0c
static u32 bus0_pci_base_address_0;	//32	10
static u32 bus0_pci_base_address_1;	//32	14
static u32 bus0_pci_base_address_2;	//32	18
static u32 bus0_pci_subordinate_bus;	//8	1a
static u32 bus0_pci_base_address_3;	//32	1c
static u32 bus0_pci_base_address_4;	//32	20
static u32 bus0_pci_base_address_5;	//32	24
static u32 bus0_pci_io_upper;		//32	30
static u32 bus0_pci_rom_address1;	//32	38
static u32 bus0_pci_interrupt_line;	//8	3c
static u32 bus0_pci_bridge_control;	//16	3e
static u32 bus0_pci_class_device;	//16	44

static u32 bus1_pci_command;		//16	04
static u32 bus1_pci_cache_line_size;	//8	0c
static u32 bus1_pci_base_address_0;	//32	10
static u32 bus1_pci_base_address_1;	//32	14
static u32 bus1_pci_base_address_2;	//32	18
static u32 bus1_pci_base_address_3;	//32	1c
static u32 bus1_pci_base_address_4;	//32	20
static u32 bus1_pci_base_address_5;	//32	24
static u32 bus1_pci_rom_address;	//32	30
static u32 bus1_pci_interrupt_line;	//8	3c
static u32 bus1_pci_class_device;	//16	44

static int epson12_pcie_read_conf(u8 busnr, int where, int size, u32 *val)
{
	int ret = 0;

	if (size == 4) {
		*val = pcie_cfg_readl(busnr, 0, where);
	} else if (size == 2) {
		*val = pcie_cfg_readw(busnr, 0, where);
	} else if (size == 1) {
		*val = pcie_cfg_readb(busnr, 0, where);
	} else {
		ret = 1;
	}

	return ret;
}

static int epson12_pcie_write_conf(u8 busnr, int where, int size, u32 val)
{
	int ret = 0;

	if (size == 4) {
		pcie_cfg_writel(busnr, 0, val, where);
	} else if (size == 2) {
		pcie_cfg_writew(busnr, 0, val, where);
	} else if (size == 1) {
		pcie_cfg_writeb(busnr, 0, val, where);
	} else {
		ret = PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return ret;
}

static void epson12_pcie_config_save(void)
{
//	epson12_pcie_read_conf(u8 busnr, int where, int size, u32 *val)

	spin_lock_irqsave(&epson12_pcie_lock, psave_flags);

	epson12_pcie_read_conf(0, PCI_COMMAND, 2, &bus0_pci_command);			//16	04
	epson12_pcie_read_conf(0, PCI_STATUS, 2, &bus0_pci_status);			//16	06
	epson12_pcie_read_conf(0, PCI_CACHE_LINE_SIZE, 1, &bus0_pci_cache_line_size);	//8	0c
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_0, 4, &bus0_pci_base_address_0);	//32	10
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_1, 4, &bus0_pci_base_address_1);	//32	14
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_2, 4, &bus0_pci_base_address_2);	//32	18
	epson12_pcie_read_conf(0, PCI_SUBORDINATE_BUS, 1, &bus0_pci_subordinate_bus);	//8	1a
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_3, 4, &bus0_pci_base_address_3);	//32	1c
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_4, 4, &bus0_pci_base_address_4);	//32	20
	epson12_pcie_read_conf(0, PCI_BASE_ADDRESS_5, 4, &bus0_pci_base_address_5);	//32	24
	epson12_pcie_read_conf(0, PCI_IO_BASE_UPPER16, 4, &bus0_pci_io_upper);	//32	30
	epson12_pcie_read_conf(0, PCI_ROM_ADDRESS1, 4, &bus0_pci_rom_address1);	//32	38
	epson12_pcie_read_conf(0, PCI_INTERRUPT_LINE, 1, &bus0_pci_interrupt_line);	//8	3c
	epson12_pcie_read_conf(0, PCI_BRIDGE_CONTROL, 2, &bus0_pci_bridge_control);	//16	3e
	epson12_pcie_read_conf(0, PCI_CLASS_DEVICE, 2, &bus0_pci_class_device);	//16	44

	epson12_pcie_read_conf(1, PCI_COMMAND, 2, &bus1_pci_command);			//16	04
	epson12_pcie_read_conf(1, PCI_CACHE_LINE_SIZE, 1, &bus1_pci_cache_line_size);	//8	0c
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_0, 4, &bus1_pci_base_address_0);	//32	10
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_1, 4, &bus1_pci_base_address_1);	//32	14
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_2, 4, &bus1_pci_base_address_2);	//32	18
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_3, 4, &bus1_pci_base_address_3);	//32	1c
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_4, 4, &bus1_pci_base_address_4);	//32	20
	epson12_pcie_read_conf(1, PCI_BASE_ADDRESS_5, 4, &bus1_pci_base_address_5);	//32	24
	epson12_pcie_read_conf(1, PCI_ROM_ADDRESS, 4, &bus1_pci_rom_address);		//32	30
	epson12_pcie_read_conf(1, PCI_INTERRUPT_LINE, 1, &bus1_pci_interrupt_line);	//8	3c
	epson12_pcie_read_conf(1, PCI_CLASS_DEVICE, 2, &bus1_pci_class_device);	//16	44
}

static void epson12_pcie_config_restore(void)
{
	epson12_pcie_write_conf(0, PCI_COMMAND, 2, bus0_pci_command);			//16	04
	epson12_pcie_write_conf(0, PCI_STATUS, 2, bus0_pci_status);			//16	06
	epson12_pcie_write_conf(0, PCI_CACHE_LINE_SIZE, 1, bus0_pci_cache_line_size);	//8	0c
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_0, 4, bus0_pci_base_address_0);	//32	10
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_1, 4, bus0_pci_base_address_1);	//32	14
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_2, 4, bus0_pci_base_address_2);	//32	18
	epson12_pcie_write_conf(0, PCI_SUBORDINATE_BUS, 1, bus0_pci_subordinate_bus);	//8	1a
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_3, 4, bus0_pci_base_address_3);	//32	1c
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_4, 4, bus0_pci_base_address_4);	//32	20
	epson12_pcie_write_conf(0, PCI_BASE_ADDRESS_5, 4, bus0_pci_base_address_5);	//32	24
	epson12_pcie_write_conf(0, PCI_IO_BASE_UPPER16, 4, bus0_pci_io_upper);		//32	30
	epson12_pcie_write_conf(0, PCI_ROM_ADDRESS1, 4, bus0_pci_rom_address1);	//32	38
	epson12_pcie_write_conf(0, PCI_INTERRUPT_LINE, 1, bus0_pci_interrupt_line);	//8	3c
	epson12_pcie_write_conf(0, PCI_BRIDGE_CONTROL, 2, bus0_pci_bridge_control);	//16	3e
	epson12_pcie_write_conf(0, PCI_CLASS_DEVICE, 2, bus0_pci_class_device);	//16	44

	epson12_pcie_write_conf(1, PCI_COMMAND, 2, bus1_pci_command);			//16	04
	epson12_pcie_write_conf(1, PCI_CACHE_LINE_SIZE, 1, bus1_pci_cache_line_size);	//8	0c
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_0, 4, bus1_pci_base_address_0);	//32	10
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_1, 4, bus1_pci_base_address_1);	//32	14
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_2, 4, bus1_pci_base_address_2);	//32	18
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_3, 4, bus1_pci_base_address_3);	//32	1c
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_4, 4, bus1_pci_base_address_4);	//32	20
	epson12_pcie_write_conf(1, PCI_BASE_ADDRESS_5, 4, bus1_pci_base_address_5);	//32	24
	epson12_pcie_write_conf(1, PCI_ROM_ADDRESS, 4, bus1_pci_rom_address);		//32	30
	epson12_pcie_write_conf(1, PCI_INTERRUPT_LINE, 1, bus1_pci_interrupt_line);	//8	3c
	epson12_pcie_write_conf(1, PCI_CLASS_DEVICE, 2, bus1_pci_class_device);	//16	44

	spin_unlock_irqrestore(&epson12_pcie_lock, psave_flags);
}

int epson12_pcie1_recovery(void)
{
	printk("%s ... link %s\n", __func__, pcie_linkon ? "on" : "off");

	printk("PCIE_DBI 0x%x -> 0x%p [%x]\n", EPSON12_PCIE_DBI_BASE, dbi, EPSON12_PCIE_DBI_LEN);
	printk("PCIE_CFG0 0x%x -> 0x%p [%x]\n", PCIE_CFG0_BASE, cfg0_va, PCIE_CFG0_SIZE);
	printk("PCIE_CFG1 0x%x -> 0x%p [%x]\n", PCIE_CFG1_BASE, cfg1_va, PCIE_CFG1_SIZE);
	printk("PCIE_MEM_BASE 0x%x [%x]\n", PCIE_MEM_BASE, PCIE_MEM_SIZE);

	dwcpcie_init(1, PCIe_RC);
	if (pcie_linkon) {
		epson12_pcie_config_restore();
	}

	return 0;
}

int epson12_pcie1_stop(void)
{
	printk("%s ... link %s\n", __func__, pcie_linkon ? "on" : "off");

	if (pcie_linkon) {
		epson12_pcie_config_save();
	}

	return 0;
}

EXPORT_SYMBOL(epson12_pcie1_recovery);
EXPORT_SYMBOL(epson12_pcie1_stop);

