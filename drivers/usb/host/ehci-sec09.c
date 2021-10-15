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

/* Copyright (c)SEIKO EPSON CORPORATION 2013. All rights reserved. */

/* use for platform_driver */
#include<linux/platform_device.h>
#include "ehci-sec09.h"
#include "ehci-epson09.h"

#define INSNREG(x) (0x80+(x*0x4))

#include "ehci-epson09-init.c"

static struct usb_hcd *__hcd = NULL;

extern int usb_disabled(void);

static int sec09_ehci_link_init(struct ehci_hcd *ehci)
{
	int retvalue=0;
	
	return retvalue;
}

static int sec09_ehci_hc_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call epson09_ehci_init\n");
#endif
	retval = epson09_ehci_init(hcd);
	if (retval != 0) {
		goto fail;
	}

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs +
		HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ehci_halt\n");
#endif
	retval = ehci_halt(ehci);
	if (retval != 0) {
		goto fail;
	}

	/*
	 * data structure init
	 */
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ehci_init\n");
#endif
	retval = ehci_init(hcd);
	if (retval != 0) {
		goto fail;
	}

	return ehci_reset(ehci);

fail:
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> failed (%d)\n", retval);
#endif
	return retval;
}

static const struct hc_driver sec09_ehci_hc_driver={
	.description = "Epson09 EHCI",
	.product_desc = "EPSON09 EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,
	
	.reset = sec09_ehci_hc_setup,
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

static int sec09_ehci_drv_probe(struct platform_device *pdev)
{
	int retvalue;
	struct usb_hcd *hcd = NULL;
	struct ehci_hcd *ehci;

	if(usb_disabled())
		return -ENODEV;

	pdev->dev.dma_mask = &dma_mask;

	hcd = usb_create_hcd(&sec09_ehci_hc_driver, &pdev->dev, dev_name(&pdev->dev));
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

	sec09_ehci_link_init(ehci);

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

static int sec09_ehci_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	
	return 0;
}

/* Device I/O Control */
int qtd_ctrl(struct usb_hcd *hcd, int is_on)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(__hcd);

	if (__hcd == NULL) {
		printk(KERN_ALERT "<EP> Not initialized\n");
		return -1;
	}

#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> qtd_ctrl(is_on==%d) : __hcd=0x%x\n", is_on, (unsigned int)__hcd);
#endif
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

static int vbus_ctrl(struct ehci_hcd *ehci, int is_on)
{
	void __iomem *regs = epson09_usb_test_regs();
	u16 value;

	if (regs == NULL) {
		return -1;
	}

	if (is_on) {
		/* bit9: PE     = 0 */
		/* bit8: PE_TES = 0 */
		value = READW(USB_TEST, UHTR) & ~((1<<9)|(1<<8));
	}
	else {
		/* bit9: PE     = 1 */
		/* bit8: PE_TES = 1 */
		value = READW(USB_TEST, UHTR) | ((1<<9)|(1<<8));
	}

	WRITEW(value, USB_TEST, UHTR);

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

MODULE_ALIAS(EPSON09_EHCI_DEVICE_NAME);

static struct platform_driver sec09_ehci_driver={
	.probe = sec09_ehci_drv_probe,
	.remove = sec09_ehci_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver={
		.name = EPSON09_EHCI_DEVICE_NAME,
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