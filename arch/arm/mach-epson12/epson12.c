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
 * Base on linux/arch/arm/mach-s3c6000/s3c600_eb.c
 *
 * History
 *  12/15/09	Jiseong oh		source code create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/spi/spi.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <asm/hardware/gic.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>

#include <mach/irqs.h>
#include <mach/gic.h>
#include <mach/clock.h>
#include <mach/hardware.h>

#include "epson12_plat.h"

#if defined(CONFIG_MACH_EPSON12_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

#ifdef CONFIG_USB_SEC_EHCI_HCD
#if defined(CONFIG_USE_EHCIHOST_0)
#define IRQ_EHCIHOST			IRQ_USBHOST0
#define EPSON12_EHCI_BASE		EPSON12_EHCI0_BASE	/* USBH_EHCI0_SFR */
#define EPSON12_EHCIH_CFG_BASE		EPSON12_USBH_CFG0_BASE	/* USBH_CFG0 */
#elif defined(CONFIG_USE_EHCIHOST_1)
#define IRQ_EHCIHOST			IRQ_USBHOST1
#define EPSON12_EHCI_BASE		EPSON12_EHCI1_BASE	/* USBH_EHCI1_SFR */
#define EPSON12_EHCIH_CFG_BASE		EPSON12_USBH_CFG1_BASE	/* USBH_CFG1 */
#elif defined(CONFIG_USE_EHCIHOST_2)
#define IRQ_EHCIHOST			IRQ_USBHOST2
#define EPSON12_EHCI_BASE		EPSON12_EHCI2_BASE	/* USBH_EHCI2_SFR */
#define EPSON12_EHCIH_CFG_BASE		EPSON12_USBH_CFG2_BASE	/* USBH_CFG2 */
#endif
#endif /* CONFIG_USB_SEC_EHCI_HCD */

#ifdef CONFIG_USB_SEC_OHCI_HCD
#if defined(CONFIG_USE_OHCIHOST_0)
#define IRQ_OHCIHOST			IRQ_USBHOST0
#define EPSON12_OHCI_BASE		EPSON12_OHCI0_BASE	/* USBH_OHCI0_SFR */
#define EPSON12_OHCIH_CFG_BASE		EPSON12_USBH_CFG0_BASE	/* USBH_CFG0 */
#elif defined(CONFIG_USE_OHCIHOST_1)
#define IRQ_OHCIHOST			IRQ_USBHOST1
#define EPSON12_OHCI_BASE		EPSON12_OHCI1_BASE	/* USBH_OHCI1_SFR */
#define EPSON12_OHCIH_CFG_BASE		EPSON12_USBH_CFG1_BASE	/* USBH_CFG1 */
#elif defined(CONFIG_USE_OHCIHOST_2)
#define IRQ_OHCIHOST			IRQ_USBHOST2
#define EPSON12_OHCI_BASE		EPSON12_OHCI2_BASE	/* USBH_OHCI2_SFR */
#define EPSON12_OHCIH_CFG_BASE		EPSON12_USBH_CFG2_BASE	/* USBH_CFG2 */
#endif
#endif /* CONFIG_USB_SEC_OHCI_HCD */

#ifdef CONFIG_SPI_EPSON_SQROM
extern u32 sqrom_chip_mode;
#endif

/* static struct dmac_info sdmmc0_info[] = {
        {
    	.modalias   = "sdmmc0",
        .dma_port   = 0,
        .conn_num   = 11,
        },
}; */

/* struct dmac_conn_info sdmmc0_dmac_info = {
	.connection_num = sdmmc0_info,
	.array_size = ARRAY_SIZE(sdmmc0_info),
}; */

PLATFORM_DEVICE("uart", 0, UART0, SZ_4K, IRQ_UART0, 1, NULL, NULL);
PLATFORM_DEVICE("timer",0, TIMER, SZ_4K, IRQ_TIMERU4, 1, NULL, NULL);

#ifdef CONFIG_USB_SEC_EHCI_HCD
PLATFORM_DEVICE("sec_ehci", 0, EHCI, SZ_16K, IRQ_EHCIHOST, 1, NULL, NULL);
#endif

#ifdef CONFIG_USB_SEC_OHCI_HCD
PLATFORM_DEVICE("sec_ohci", 0, OHCI, SZ_4K, IRQ_OHCIHOST, 1, NULL, NULL);
#endif

#ifdef CONFIG_SPI_EPSON_SFLU3
PLATFORM_DEVICE("sflu3", 0, SFLU3, SZ_4K, NO_IRQ, 0, NULL, NULL);
#endif

#ifdef CONFIG_MTD_M25P80
PLATFORM_DEVICE("m25p80",0, SFLASH, SZ_32M, NO_IRQ, 0, &epson12_flash_data, &spi_bus_type);
/* PLATFORM_DEVICE("m25p80",0, SFLASH, SZ_4K, NO_IRQ, 0, &epson12_flash_data, &spi_bus_type); */
#endif

/* PLATFORM_DEVICE("sdmmc", 0, SDMMC0, SZ_128K, IRQ_SDMMC, 1,&sdmmc0_dmac_info, NULL); */

/* EPSON12 board io description */
static struct map_desc epson12_io_desc[] __initdata = {
 	{
		.virtual	= IO_ADDRESS(CORE_PM_BASE),
		.pfn		= __phys_to_pfn(CORE_PM_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EPSON12_SRAM_BASE),
		.pfn		= __phys_to_pfn(EPSON12_SRAM_BASE),
		.length		= SZ_16K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(MISC_WH_BASE),
		.pfn		= __phys_to_pfn(MISC_WH_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(MISC_BK_BASE),
		.pfn		= __phys_to_pfn(MISC_BK_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EPSON12_SPI_BASE),
		.pfn		= __phys_to_pfn(EPSON12_SPI_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EPSON12_UART0_BASE),
		.pfn		= __phys_to_pfn(EPSON12_UART0_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
 	{
		.virtual	= IO_ADDRESS(EPSON12_TIMER_BASE),
		.pfn		= __phys_to_pfn(EPSON12_TIMER_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#ifdef CONFIG_USB_SEC_EHCI_HCD
	{
		.virtual	= IO_ADDRESS(EPSON12_EHCI_BASE),
		.pfn		= __phys_to_pfn(EPSON12_EHCI_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
#ifdef CONFIG_USB_SEC_OHCI_HCD
	{
		.virtual	= IO_ADDRESS(EPSON12_OHCI_BASE),
		.pfn		= __phys_to_pfn(EPSON12_OHCI_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
/* #ifdef CONFIG_USB_SEC_EHCI_HCD
	{
		.virtual	= IO_ADDRESS(EPSON12_EHCIH_CFG_BASE),
		.pfn		= __phys_to_pfn(EPSON12_EHCIH_CFG_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#endif */
/* #ifdef CONFIG_USB_SEC_OHCI_HCD
	{
		.virtual	= IO_ADDRESS(EPSON12_OHCIH_CFG_BASE),
		.pfn		= __phys_to_pfn(EPSON12_OHCIH_CFG_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#endif */
#if defined (CONFIG_STMMAC_ETH) || defined (CONFIG_STMMAC_ETH_MODULE)
	{
		.virtual	= IO_ADDRESS(EPSON12_GMAC_BASE),
		.pfn		= __phys_to_pfn(EPSON12_GMAC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EPSON12_GMAC_CFG_BASE),
		.pfn		= __phys_to_pfn(EPSON12_GMAC_CFG_BASE - MMU_OFFSET_4K),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
	{
		.virtual	= IO_ADDRESS(EPSON12_GPIO_BASE),
		.pfn		= __phys_to_pfn(EPSON12_GPIO_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EPSON12_MISC_BASE),
		.pfn		= __phys_to_pfn(EPSON12_MISC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
/*	{
		.virtual	= IO_ADDRESS(EPSON12_DMU_BASE),
		.pfn		= __phys_to_pfn(EPSON12_DMU_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, */

};

void __init epson12_map_io(void)
{
	iotable_init(epson12_io_desc, ARRAY_SIZE(epson12_io_desc));
}

#ifdef CONFIG_MTD_M25P80
static int epson12_flash_init(void)
{
	return 0;
}

static void epson12_flash_exit(void)
{
}

static void epson12_flash_set_vpp(int on)
{
}

struct flash_platform_data epson12_flash_data = {
	.map_name		= "cfi_probe",
	.width			= 2,
	.init			= epson12_flash_init,
	.exit			= epson12_flash_exit,
	.set_vpp		= epson12_flash_set_vpp,
};

struct flash_platform_data * get_board_flash_info (void)
{
	return &epson12_flash_data;
}

/* The command line chip select */
static u16 chip_no = 0;

static __init int chip_no_setup(char *s)
{
	chip_no = (u16)simple_strtoul(s, NULL, 10);
	return 1;
}

__setup("cn=", chip_no_setup);

/* The command line chip device */
#define CHIP_DEVICE_DEFAULT_VALUE "n25q064"
static char *chip_device = CHIP_DEVICE_DEFAULT_VALUE; /* numonyx 64KB sector */

static __init int chip_device_setup(char *s)
{
	chip_device = s;
	return 1;
}

__setup("cd=", chip_device_setup);

/* Copied from include/linux/spi/flash.h and change name flash_platform_data to spi_flash_platform_data */
struct spi_flash_platform_data {
	char		*name;
	struct mtd_partition *parts;
	unsigned int	nr_parts;
	char		*type;
	/* We'll likely add more ... use JEDEC IDs, etc */
} spi_flash_platform_data = {
	.name = "m25p80",
};

static struct spi_board_info epson12_serial_board_info[] = {
	{
		.modalias = "m25p80",
		.platform_data = &spi_flash_platform_data,
		.controller_data = NULL,
		.irq = -1,
		.max_speed_hz = 120000,
		.bus_num = 0,
		.mode = SPI_MODE_3,
	},
};
#endif /* defined CONFIG_MTD_M25P80 */

/* Platform device list */
static struct platform_device *platform_devs[] __initdata = {

	&UART0_device,
	&TIMER_device,
#ifdef  CONFIG_USB_SEC_EHCI_HCD
	&EHCI_device,
#endif
#ifdef  CONFIG_USB_SEC_OHCI_HCD
	&OHCI_device,
#endif
#ifdef CONFIG_SPI_EPSON_SFLU3
	&SFLU3_device,
#endif
#ifdef CONFIG_MTD_M25P80
	&SFLASH_device,
#endif
/*	&SDMMC0_device, */
};


#ifdef CONFIG_POWER_SAVING_SWITCH_L1
#include <asm/cacheflush.h>
#include <asm/leds.h>
#include <asm/processor.h>
#include "mach/epson12_ps.h"

void *g_psmode1_scrachpad_mapaddr = NULL;
EXPORT_SYMBOL(g_psmode1_scrachpad_mapaddr);

void add_map_for_psmode1()
{
	g_psmode1_scrachpad_mapaddr = (void *) ioremap_nocache(EPSON12_PSMODE1_SCRATCHPAD, 0x4);
	DBG("<EP> g_psmode1_scrachpad_mapaddr=0x%x\n", (unsigned int)g_psmode1_scrachpad_mapaddr);
}
#endif	/* CONFIG_POWER_SAVING_SWITCH_L1 */

#ifdef CONFIG_MACH_EPSON12_EMG_SYSTEM_STOP
static irqreturn_t emg_stop_handler(int irq, void *dev, struct pt_regs *regs){
	while(1);
	return IRQ_HANDLED;
}
#endif

void __init epson12_init(void)
{
	int i;
	char *serial_chip;

	for (i = 0; i < ARRAY_SIZE(platform_devs); i++) {
		struct platform_device *d = platform_devs[i];
		platform_device_register(d);
	}

#ifdef CONFIG_MTD_M25P80
	/* Set the command line parameter */
	epson12_serial_board_info[0].chip_select = chip_no;
#ifdef CONFIG_SPI_EPSON_SQROM
	if (!strcmp(chip_device, CHIP_DEVICE_DEFAULT_VALUE)) {
		if (sqrom_chip_mode == 1) {
			chip_device = "w25q128c1";
			DBG("SQROM: select 1chip mode\n");
		} else if (sqrom_chip_mode == 2) {
			chip_device = "w25q128c2";
			DBG("SQROM: select 2chip mode\n");
		}
	}
#endif
	((struct spi_flash_platform_data *)epson12_serial_board_info[0].platform_data)->type = chip_device;

	spi_register_board_info(epson12_serial_board_info, ARRAY_SIZE(epson12_serial_board_info));
#endif
#ifdef CONFIG_POWER_SAVING_SWITCH_L1
	add_map_for_psmode1();
#endif

#ifdef CONFIG_MACH_EPSON12_EMG_SYSTEM_STOP
	/* open irq of EMG_STOP_FROM_CORE0 */
	int ret;
	ret = request_irq(IRQ_EMG_STOP_FROM_CORE0, &emg_stop_handler, 0, "EMG_STOP_FROM_CORE0", NULL);
	if (ret) {
		panic ("Can not assign IRQ number IRQ_EMG_STOP_FROM_CORE0\n" );
	}
#endif
}

void __init epson12_gic_init_irq(void)
{
	DBG("--------------START-EP-GIC-DIST-INIT----------------\n");
	DBG("<EP> %s\n", __FUNCTION__);

	gic_cpu_base_addr = __io_address(CORE_PM_BASE) + INTI_BASE;
	gic_dist_init(0, __io_address(CORE_PM_BASE) + INTD_BASE, 1);
#ifndef CONFIG_EPSON12_WITH_RTOS
	epson12_misc_init();
#endif
	gic_cpu_init(0, gic_cpu_base_addr);
	clk_init();
#ifdef MISC_TEST /* test only */
	epson12_misc_test();
#endif
	DBG("----------------END-EP-GIC-DIST-INIT----------------\n");
}
