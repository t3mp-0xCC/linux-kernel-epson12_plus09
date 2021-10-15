/**************************************************************************** 
 *
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2009 myoung-taek Lim <mt2007.lim@samsung>, Samsung Electronics Co., Ltd., 
 * 
 * Bus Glue for Samsung EHCI driver
 *
 * Base on "ehci-ppc-soc.c" by Stefan Roese <sr@denx.de> 
 * This file is licenced under the GPL.
****************************************************************************/

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

/* use for platform_driver */
#include<linux/platform_device.h>
#include "ehci-sec.h"

#define INSNREG(x) (0x80+(x*0x4))

#if defined(CONFIG_MACH_EPSON12_DBG)
#define DBG(args...)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#endif

static struct usb_hcd *__hcd = NULL;

extern int usb_disabled(void);

int sec_ehci_link_init(struct ehci_hcd *ehci)
{
	unsigned int insreg[6];
	unsigned int cnt;
	int retvalue=0;
	volatile u8 *address_base;

	address_base = (u8 *)ehci->regs;

	for(cnt = 0 ; cnt < 6; cnt++)
		insreg[cnt] = ioread32((void *)(u8 *)(address_base + INSNREG(cnt))); 
	
	insreg[1] &= ~0xFFFFFFFF;
	insreg[1] = 0x00800080;	//OUT/IN buffer thresholds value is 96 bytes.
	iowrite32((u32)insreg[1], (void *)(u8 *)(address_base + INSNREG(1)));
	
	insreg[2] &= 0x0;
	insreg[2] = 0x100;
	iowrite32((u32)insreg[2], (void *)(u8 *)(address_base + INSNREG(2)));
	
	return retvalue;
}

static int sec_ehci_hc_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	retval = ehci_halt(ehci);
	
	if(retval)
		return retval;

	retval = ehci_init(hcd);

	if(retval)
		return retval;

	ehci->sbrn = 0x20;

	return ehci_reset(ehci);
}


static const struct hc_driver sec_ehci_hc_driver={
	.description = "Samsung EHCI",
	.product_desc = "Samsung Electronics Co.,LTD.",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,
	
	.reset = sec_ehci_hc_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	
	.endpoint_disable = ehci_endpoint_disable,

	.get_frame_number = ehci_get_frame,

	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef CONFIG_PM	
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
	.relinquish_port = ehci_relinquish_port, 
	.port_handed_over = ehci_port_handed_over,
};

static u64 dma_mask = (u32)~0;

int qtd_ctrl(struct usb_hcd *hcd, int is_on)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(__hcd);

	if (__hcd == NULL) {
		printk(KERN_ALERT "<EP> Not initialized\n");
		return -1;
	}

	DBG("<EP> qtd_ctrl(is_on==%d) : __hcd=0x%x\n", is_on, __hcd);
	if(is_on){
		ehci_activate (ehci);
		__hcd->state = HC_STATE_RUNNING;
	}else{
		ehci_quiesce (ehci);
		__hcd->state = HC_STATE_QUIESCING;
	}
	return 0;
}
EXPORT_SYMBOL(qtd_ctrl);

static int sec_ehci_drv_probe(struct platform_device *pdev)
{
	int retvalue;
	struct usb_hcd *hcd = NULL;
	struct ehci_hcd *ehci;

	if(usb_disabled())
		return -ENODEV;

	pdev->dev.dma_mask = &dma_mask;

	hcd = usb_create_hcd(&sec_ehci_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	__hcd = hcd;
	if(!hcd)
	{
		return -ENOMEM;
	}

	__hcd = hcd;

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

	ehci = hcd_to_ehci(hcd);

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	sec_ehci_link_init(ehci);

	retvalue = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_SHARED|IRQF_DISABLED);

	if(!retvalue)
	{
		platform_set_drvdata(pdev, hcd);
		return retvalue;
	}
	ehci_stop(hcd);
	iounmap(hcd->regs);

no_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

no_mem_region:
	usb_put_hcd(hcd);

	return retvalue;
}

static int vbus_ctrl(struct ehci_hcd *ehci, int is_on)
{
	ehci_writel(ehci, (is_on ? FLAG_CF : 0), &ehci->regs->configured_flag);
	msleep(200);
	return 0;
}

static int sec_ehci_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct ehci_hcd	*ehci;

	if (__hcd == NULL) {
		pr_alert(KERN_ALERT "Not initialized\n");
		return -1;
	}

	ehci = hcd_to_ehci(__hcd);

	switch (cmd) {
		case SEC_EHCI_PORT_OFF:
			ehci_dbg(ehci, "EHCI port power off");
			vbus_ctrl(ehci, 0);
			break;

		case SEC_EHCI_PORT_ON:
			ehci_dbg(ehci, "EHCI port power on");
			vbus_ctrl(ehci, 1);
			break;

		default:
			ehci_err(ehci, "Unhandled CMD:0x%x\n", cmd);
			return -1;
	}

	return 0;
}

static int sec_ehci_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	
	return 0;
}

MODULE_ALIAS("sec_ehci");

static struct platform_driver sec_ehci_driver={
	.probe = sec_ehci_drv_probe,
	.remove = sec_ehci_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver={
		.name = "sec_ehci",
		.bus = &platform_bus_type
	}
};

static struct file_operations sec_ehci_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= sec_ehci_ioctl
};

static int __init sec_ehci_init(void)
{
	int r;

    	pr_info(SEC_EHCI_CHRDEV_NAME ": Initializing\n");

	r = register_chrdev(SEC_EHCI_MAJOR_NUM,
		SEC_EHCI_CHRDEV_NAME, &sec_ehci_fops);
	if (r < 0) {
		pr_err(SEC_EHCI_CHRDEV_NAME ": Unable to register character device\n");
		return r;
	}

	return 0;
}

static void __exit sec_ehci_cleanup(void)
{
	unregister_chrdev(SEC_EHCI_MAJOR_NUM, SEC_EHCI_CHRDEV_NAME);
}

module_init(sec_ehci_init);
module_exit(sec_ehci_cleanup);