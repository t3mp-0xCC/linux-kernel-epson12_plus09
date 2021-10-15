/**************************************************************************** 
 *
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2009 myoung-taek Lim <mt2007.lim@samsung>, Samsung Electronics Co., Ltd., 
 * 
 * Bus Glue for Samsung EHCI driver
 *
 * Base on "ohci-ppc-soc.c" by Stefan Roese <sr@denx.de> 
 * This file is licenced under the GPL.
****************************************************************************/

#include<linux/platform_device.h>

#define SEC_OHCI_DEBUG 1 

#ifdef SEC_OHCI_DEBUG
#define OHCI_DEBUG(fmt, args...) printk(fmt, ##args)
#else
#define OHCI_DEBUG(fmt, args...) do {}while(0)
#endif

/*
#ifndef CONFIG_USB_SAMSUNG_EHCI_HCD
#define PHY_REG_FAIL -1
#define PHY_REG_SUCCESS 0

struct PHY_OFFSET{
	u32 PHYCON;
	u32 RESERVED[2];
	u32 PHYOPT;
	u32 RESERVED10;
	u32 PHYMON;
	u32 HOSTCON;
};

volatile struct PHY_OFFSET *gphy_regs;

static int sec_ohci_phy_init(void)
{
	gphy_regs = ioremap(S3C6000_SAMSUNG_HOST_MISC_BASE, SZ_1M);
	if(!gphy_regs)
		return PHY_REG_FAIL;

	OHCI_DEBUG("%s : PHY REGISTER Virtual Address : %p\n", __func__, (void *)gphy_regs);
	return PHY_REG_SUCCESS;
}
#else
static int sec_ohci_phy_init(void) {return 0;}
#endif
*/
#if 0
static int sec_ohci_start(struct usb_hcd *hcd)
{
	int retvalue;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	ohci->fminterval &= (unsigned long)0;

	if((retvalue = ohci_init(ohci)) < 0)
		return retvalue;

//	printk("ohci->regs : %p\n",ohci->regs);
//	printk("hcd->regs : %p\n", hcd->regs);
	printk("3.ohci assigned...\n");
	if((retvalue = ohci_run(ohci)) < 0){
		err("can't start %s: error %d\n", ohci_to_hcd(ohci)->self.bus_name, retvalue);
		ohci_stop(hcd);
		return retvalue;
	}

	printk("4.ohci assigned...\n");
	return 0;
}	
#endif

int sec_ohci_reset(struct usb_hcd *hcd)
{
	int retvalue=0;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd); 

	ohci->fminterval &= (unsigned long)0;
	
	if((retvalue = ohci_init(ohci)) < 0)
		return retvalue;

	return retvalue;
}

int sec_ohci_start(struct usb_hcd *hcd)
{
	int retvalue=0;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	if( (retvalue = ohci_run(ohci)) < 0)
	{
		err("can't start %s\n", hcd->self.bus_name);
		ohci_stop(hcd);
	}
	return 0;
}

static const struct hc_driver sec_ohci_hc_driver={
	.description = "Samsung OHCI",
//	.description = hcd_name,
	.product_desc = "Samsung Electronics Co.,LTD.",
	.hcd_priv_size = sizeof(struct ohci_hcd),

	.irq = ohci_irq, 

	.flags = (HCD_MEMORY | HCD_USB11),

	.reset = sec_ohci_reset,
	.start = sec_ohci_start,
	.stop = ohci_stop, 
	.shutdown = ohci_shutdown, 
	
	.get_frame_number = ohci_get_frame,
	
	.urb_enqueue = ohci_urb_enqueue,
	.urb_dequeue = ohci_urb_dequeue,
	.endpoint_disable = ohci_endpoint_disable,

	.hub_status_data = ohci_hub_status_data,
	.hub_control = ohci_hub_control,

#ifdef CONFIG_PM
	.bus_suspend =ohci_bus_suspend,
	.bus_resume = ohci_bus_resume,
#endif	
	.start_port_reset = ohci_start_port_reset,
};

static u64 dma_mask = ~(u32)0;

static int sec_ohci_drv_probe(struct platform_device *pdev)
{

	int retvalue=0;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci;

	pdev->dev.dma_mask = &dma_mask;
/*	retvalue = sec_ohci_phy_init();
	if(retvalue)
		return -ENXIO;
*/
	hcd = usb_create_hcd(&sec_ohci_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if(!hcd)
	{
		return -ENOMEM;
//		retvalue = -ENOMEM;
//		goto no_create_hcd;
	}

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if(!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, pdev->name))
	{
		retvalue = -EBUSY;
		goto no_mem_region;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if(!hcd->regs)
	{
		retvalue = -ENOMEM;
		goto no_ioremap;
	}

//	ohci = kzalloc(sizeof(struct ohci_hcd), GFP_KERNEL);
	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);
	
	retvalue = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED|IRQF_DISABLED);
	
	if(!retvalue)
	{
		platform_set_drvdata(pdev, hcd);
		return retvalue;
	}
	ohci_stop(hcd);
	iounmap(hcd->regs);
no_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
no_mem_region:
	
	usb_put_hcd(hcd);
/*
no_create_hcd:

#ifndef CONFIG_USB_SAMSUNG_EHCI_HCD
	iounmap(gphy_regs);	
#endif
*/
	return retvalue;
}


static int sec_ohci_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}


MODULE_ALIAS("sec_ohci");

static struct platform_driver sec_ohci_driver = {
	.probe = sec_ohci_drv_probe,
	.remove = sec_ohci_drv_remove,
	.shutdown = usb_hcd_platform_shutdown, 
	.driver = {
		.name = "sec_ohci",
		.bus = &platform_bus_type,
	}
};
