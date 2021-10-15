/*
 *  linux/arch/arm/mach-epson09/core.c
 *
 *  Copyright (c) SEIKO EPSON 2008-2010, All right reserved.
 *
 *  License : GPL2.0
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/vic.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/serial_sci.h>
#include <asm/mach/flash.h>

#include <mach/hardware.h>

#include "core.h"
#include "gicu.h"
#include "epson09_plat.h"

#include <linux/spi/spi.h>

#define VA_GICU_9_BASE		__io_address(EPSON09_GICU_9_BASE)

void __init epson09_init_irq(void)
{
	epson09_gicu_dist_init(VA_GICU_9_BASE);
}

static struct map_desc epson09_io_desc[] __initdata = {
	{
		.virtual	=  IO_ADDRESS(EPSON09_GOBI_9_BASE),
		.pfn		= __phys_to_pfn(EPSON09_GOBI_9_BASE),
		.length		= SZ_4M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(EPSON09_ETH_BASE),
		.pfn		= __phys_to_pfn(EPSON09_ETH_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},

/*
	{
		.virtual	= __phys_to_virt(EPSON09_ETH_TXDESC),
		.pfn		= __phys_to_pfn(EPSON09_ETH_TXDESC),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= __phys_to_virt(EPSON09_ETH_RXDESC),
		.pfn		= __phys_to_pfn(EPSON09_ETH_RXDESC),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
*/

	{
		.virtual	=  IO_ADDRESS(EPSON09_CPUIF_BASE),
		.pfn		= __phys_to_pfn(EPSON09_CPUIF_BASE),
		.length		= SZ_4M,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_UART0_BASE),
		.pfn		= __phys_to_pfn(EPSON09_UART0_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_UART1_BASE),
		.pfn		= __phys_to_pfn(EPSON09_UART1_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_UART2_BASE),
		.pfn		= __phys_to_pfn(EPSON09_UART2_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_IDC_BASE),
		.pfn		= __phys_to_pfn(EPSON09_IDC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_ASIC_SYSCU_BASE),
		.pfn		= __phys_to_pfn(EPSON09_ASIC_SYSCU_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(EPSON09_ASIC_SFLU_BASE),
		.pfn		= __phys_to_pfn(EPSON09_ASIC_SFLU_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
 	{
		.virtual	=  IO_ADDRESS(EPSON09_SRAM_BASE),
		.pfn		= __phys_to_pfn(EPSON09_SRAM_BASE),
		.length		= SZ_16K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(EPSON09_ASICIOU_BASE),
		.pfn		= __phys_to_pfn(EPSON09_ASICIOU_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(EPSON09_SPIU_BASE),
		.pfn		= __phys_to_pfn(EPSON09_SPIU_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= IO_ADDRESS(EPSON09_MYUSB_BASE),
		.pfn		= __phys_to_pfn(EPSON09_MYUSB_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
};

void __init epson09_map_io(void)
{
	iotable_init(epson09_io_desc, ARRAY_SIZE(epson09_io_desc));
}

static struct plat_sci_port scif_platform_data = {
	.mapbase	= 0xE0816000,
	.flags		= UPF_BOOT_AUTOCONF,
	.type		= PORT_SCIF,
	.irqs		= { IRQ_SCIF0_RERR, IRQ_SCIF0_RXI, IRQ_SCIF0_TXI, IRQ_SCIF0_BRK },
};

static struct platform_device scif_device = {
	.name	= "sh-sci",
	.id		= 0,
	.dev	= {
		.platform_data	= &scif_platform_data,
	},
};

#if defined(CONFIG_MFP09) || defined(CONFIG_MFP09_MODULE)
static struct resource mfp09_resources[] = {
	[0] = {
		.start		= EPSON09_ETH_BASE,
		.end		= EPSON09_ETH_BASE + 0xA000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_INT_ETHER,
		.end		= IRQ_INT_ETHER,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mfp09_device = {
	.name			= "mfp09",
	.id				= 0,
	.num_resources	= ARRAY_SIZE(mfp09_resources),
	.resource		= mfp09_resources,
};
#endif

#if 0
static struct resource epson09_flash_resource = {
	.start			= EPSON09_FLASH_BASE,
	.end			= EPSON09_FLASH_BASE + EPSON09_FLASH_SIZE - 1,
	.flags			= IORESOURCE_MEM,
};

static struct platform_device epson09_flash_device = {
	.name			= "armflash",
	.id				= 0,
	.dev			= {
			.platform_data = &epson09_flash_data,
	},
	.num_resources	= 1,
	.resource		= &epson09_flash_resource,
};
#endif

#ifdef CONFIG_SPI_EPSON_SFLU3
PLATFORM_DEVICE("sflu3", 0, ASIC_SFLU, SZ_4K, NO_IRQ, 0, NULL, NULL);
#endif

#ifdef CONFIG_MTD_M25P80
PLATFORM_DEVICE("m25p80",0, FLASH, EPSON09_FLASH_SIZE, NO_IRQ, 0, &epson09_flash_data, &spi_bus_type);
#endif

#if 0
#ifdef CONFIG_SPI_EPSON_SFLU3
struct resource sflu3_resource[] = {
	{
		.start			= EPSON09_ASIC_SYSCU_BASE,
		.end			= EPSON09_ASIC_SYSCU_BASE + SZ_8K - 1,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= EPSON09_ASIC_TSYSU_BASE,
		.end			= EPSON09_ASIC_TSYSU_BASE + SZ_8K - 1,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= EPSON09_ASIC_SFLU_BASE,
		.end			= EPSON09_ASIC_SFLU_BASE + EPSON09_ASIC_SFLU_SIZE - 1,
		.flags			= IORESOURCE_MEM,
	},
};

struct platform_device sflu3_device = {
	.name = "sflu3",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
	.resource		= sflu3_resource,
	.num_resources	= ARRAY_SIZE(sflu3_resource),
};
#endif

#ifdef CONFIG_MTD_M25P80
struct platform_device m25p80device = {
	.name = "m25p80",
	.id = 0,
	.dev = {
		.platform_data = & epson09_flash_data,
		.bus = & spi_bus_type
	},
	.resource		= NULL /*sflu2_resource */,
	.num_resources	= 0 /* ARRAY_SIZE(sflu2_resource) */,
};
#endif
#endif

#ifdef CONFIG_MTD_M25P80
static int epson09_flash_init(void)
{
	return 0;
}

static void epson09_flash_exit(void)
{
}

static void epson09_flash_set_vpp(int on)
{
}

struct flash_platform_data epson09_flash_data = {
	.map_name		= "cfi_probe",
	.width			= 2,
	.init			= epson09_flash_init,
	.exit			= epson09_flash_exit,
	.set_vpp		= epson09_flash_set_vpp,
};

struct flash_platform_data * get_board_flash_info (void)
{
	return &epson09_flash_data;
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
static char *chip_device = "n25q064"; /* numonyx 64KB sector */

static __init int chip_device_setup(char *s)
{
	chip_device = s;
	return 1;
}

__setup("cd=", chip_device_setup);

/* copied from include/linux/spi/flash.h and change name flash_platform_data to spi_flash_platform_data */
struct spi_flash_platform_data {
	char		*name;
	struct mtd_partition *parts;
	unsigned int	nr_parts;
	char		*type;
	/* we'll likely add more ... use JEDEC IDs, etc */
} spi_flash_platform_data = {
	.name = "m25p80",
	.type = "n25q064",
};

static struct spi_board_info epson09_serial_board_info[] = {
	{
		.modalias = "m25p80",
		.platform_data = &spi_flash_platform_data,
		.controller_data = NULL,
		.irq = -1,
		.max_speed_hz = 120000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_3,
	},
};
#endif /* defined CONFIG_MTD_M25P80 */

#ifdef CONFIG_USB_SEC09_EHCI_HCD
/* USB-Host9 */
static struct resource epson09_ehci_resources[] = {
	[0] = {
		.start	= REG_START(EHCI),
		.end	= REG_END(EHCI),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= EHCI_IRQ,
		.end	= EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 usb_dmamask = DMA_32BIT_MASK;

static struct platform_device epson09_ehci_device = {
	.name	= EPSON09_EHCI_DEVICE_NAME,
	.id		= 0,
	.dev = {
//		.release			= usb_release,
		.dma_mask			= &usb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data = NULL,
		.bus = NULL
	},
	.num_resources	= ARRAY_SIZE(epson09_ehci_resources),
	.resource		= epson09_ehci_resources,
};
#endif

static irqreturn_t emg_stop_handler(int irq, void *dev){
	printk("emg stop intrrupt recv from core0\n");
	while(1);
	return IRQ_HANDLED;
}

void __init epson09_init(void)
{
	int ret = 0;

#if 0
	platform_device_register(&epson09_flash_device);
#endif
	platform_device_register(&scif_device);
#ifdef CONFIG_SPI_EPSON_SFLU3
	platform_device_register(&ASIC_SFLU_device);
#endif
#if defined(CONFIG_MTD_M25P80)
	((struct spi_flash_platform_data *)epson09_serial_board_info[0].platform_data)->type = chip_device;
	platform_device_register(&FLASH_device);
	spi_register_board_info(epson09_serial_board_info, ARRAY_SIZE(epson09_serial_board_info));
#endif

#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	printk(KERN_INFO "registed epson09-ehci devices, \n");
#endif
#ifdef CONFIG_USB_SEC09_EHCI_HCD
	platform_device_register(&epson09_ehci_device);
#endif

#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	printk(KERN_INFO "registed mfp09_devices, \n");
#endif
#if defined(CONFIG_MFP09) || defined(CONFIG_MFP09_MODULE)
	platform_device_register(&mfp09_device);
#endif

	/* open irq of EMG_STOP_FROM_CORE0 */
	ret = request_irq(IRQ_EMG_STOP_FROM_CORE0, &emg_stop_handler, 0, "EMG_STOP_FROM_CORE0", NULL);
	if (ret) {
		printk(KERN_ERR "Can not assign IRQ number IRQ_EMG_STOP_FROM_CORE0\n" );
	}
}
