/*
 *  EPSON09 EHCI Host Controller initialization
 *
 *  Copyright (C) 2008-2009 SEIKO EPSON Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define CONFIG_MACH_EPSON12_DBG
#if defined(CONFIG_MACH_EPSON12_DBG)
struct regs_default_u32 {
	unsigned long offset;
	u32 value;
};

struct regs_default_u16 {
	unsigned long offset;
	u16 value;
};

static struct regs_default_u32 ehci_defaults[] = {
	{EHCI_OFFSET_HCCAPBASE,			0x01000010},
	{EHCI_OFFSET_HCSPARAMS,			0x00001212},
	{EHCI_OFFSET_HCCPARAMS,			0x0000A016},
	{EHCI_OFFSET_USBCMD,			0x00080B00},
	{EHCI_OFFSET_USBSTS,			0x00001000},
	{EHCI_OFFSET_USBINTR,			0x00000000},
	{EHCI_OFFSET_FRINDEX,			0x00000000},
	{EHCI_OFFSET_CTRLDSSEGMENT,		0x00000000},
	{EHCI_OFFSET_PERIODICLISTBASE,	0x00000000},
	{EHCI_OFFSET_ASYNCLISTADDR,		0x00000000},
	{EHCI_OFFSET_CONFIGFLAG,		0x00000000},
	{EHCI_OFFSET_PORTSC1,			0x00002000},
	{EHCI_OFFSET_EIIBC1,			0x00400040},
	{EHCI_OFFSET_EIIBC2,			0x00000000},
};

static struct regs_default_u16 test_defaults[] = {
	{USB_TEST_OFFSET_RESET,			0x0100},
	{USB_TEST_OFFSET_PWEN,			0x1001},
	{USB_TEST_OFFSET_UHTR,			0x1022},
	{USB_TEST_OFFSET_OC_MSK_TIM,	0x0000},
	{USB_TEST_OFFSET_STBY_INT,		0x0000},
	{USB_TEST_OFFSET_INT_CLS,		0x0000},
};

static inline void check_ehci_default_values(void __iomem *regs)
{
	int idx;
	u32 value;

	for (idx = 0; idx < sizeof(ehci_defaults)/sizeof(*ehci_defaults); idx++) {
		value = readl(regs + ehci_defaults[idx].offset);
/*
		DEBUG_PRINT("<EP> R: %08x = %08x (%s)\n",
					(u32)(REG_START(EHCI) + ehci_defaults[idx].offset),
					value, (value == ehci_defaults[idx].value) ? "OK" : "NG");
*/
	}
}

static inline void check_usb_test_default_values(void __iomem *regs)
{
	int idx;
	u16 value;

	for (idx = 0; idx < sizeof(test_defaults)/sizeof(*test_defaults); idx++) {
		value = readw(regs + test_defaults[idx].offset);
/*
		DEBUG_PRINT("<EP> R: %08x = %04x (%s)\n",
					(u32)(REG_START(USB_TEST) + test_defaults[idx].offset),
					value, (value == test_defaults[idx].value) ? "OK" : "NG");
*/
	}
}

#else

#define check_ehci_default_values(r)
#define check_usb_test_default_values(r)

#endif

static void __iomem *usb_test_regs = NULL;

static inline void __iomem *epson09_usb_test_regs(void)
{
	return usb_test_regs;
}

static inline int init_usb_test(struct usb_hcd *hcd)
{
	resource_size_t start = REG_START(USB_TEST);
	resource_size_t len = REG_LENGTH(USB_TEST);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call request_mem_region(0x%08x, 0x%x, \"%s\")\n",
				start, len, hcd->driver->description);
#endif
	if (request_mem_region(start, len,
						   hcd->driver->description) == NULL) {
		return -1;
	}
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ioremap_nocache(0x%08x, 0x%x)\n", start, len);
#endif
	usb_test_regs = ioremap_nocache(start, len);
	if (usb_test_regs == NULL) {
		release_mem_region(start, len);
		return -1;
	}

	return 0;
}

static inline int fin_usb_test(void)
{
	if (usb_test_regs != NULL) {
		iounmap(usb_test_regs);
		release_mem_region(REG_START(USB_TEST), REG_LENGTH(USB_TEST));
		usb_test_regs = NULL;
	}

	return 0;
}

static inline int init_phy_and_power_enable(struct usb_hcd *hcd)
{
	void __iomem *regs = epson09_usb_test_regs();
	u16 value;

	if (regs == NULL) {
		return -1;
	}

	check_usb_test_default_values(regs);

#if !defined(CONFIG_USB_EPSON09SS_REGISTERSET)
	/* bit8: PLLENIN     = 1 */
	/* bit0: RESET10_N_0 = 0 */
	value = (READW(USB_TEST, RESET) & ~(1<<0)) | (1<<8);
#else
	/* bit8: PLLENIN     = 1 */
	/* bit4: RESET10_N_0 = 0 */
	value = (READW(USB_TEST, RESET) & ~(1<<4)) | (1<<8);
#endif
	WRITEW(value, USB_TEST, RESET);

#if !defined(CONFIG_USB_EPSON09SS_REGISTERSET)
	/* bit12: OVERCRNT_MASK = 0 */
	/* bit8 : AUTO_OVER_EN  = 1 */
	/* bit4 : PWR_POL       = 1 */
	/* bit0 : PWR_CNTL      = 0 */
	value = (READW(USB_TEST, PWEN) & ~((1<<12)|(1<<0))) | ((1<<8)|(1<<4));
#else
	/* bit4 : PWR_POL       = 1 */
	/* bit0 : PWR_CNTL      = 0 */
	value = (READW(USB_TEST, PWEN) &  ~(1<<0))          | (1<<4);
#endif
	WRITEW(value, USB_TEST, PWEN);

#if !defined(CONFIG_USB_EPSON09SS_REGISTERSET)
	/* bit8: PLLENIN     = 1 */
	/* bit0: RESET10_N_0 = 1 */
	value = READW(USB_TEST, RESET) | ((1<<8)|(1<<0));
#else
	/* bit8: PLLENIN     = 1 */
	/* bit4: RESET10_N_0 = 1 */
	value = READW(USB_TEST, RESET) | ((1<<8)|(1<<4));
#endif
	WRITEW(value, USB_TEST, RESET);

	return 0;
}

static inline int init_usb_host_module(struct usb_hcd *hcd)
{
	void __iomem *regs = NULL;
	resource_size_t start = REG_START(SYSCU);
	resource_size_t len = REG_LENGTH(SYSCU);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call request_mem_region(0x%08x, 0x%x, \"%s\")\n",
				start, len, hcd->driver->description);
#endif
	if (request_mem_region(start, len,
						   hcd->driver->description) == NULL) {
		return -1;
	}
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ioremap_nocache(0x%08x, 0x%x)\n", start, len);
#endif
	regs = ioremap_nocache(start, len);
	if (regs == NULL) {
		release_mem_region(start, len);
		return -1;
	}

	/* write to SYCSRSTS5, but read from SYCSRST5 */
	/* bit3: USBHD9 = 1 */
	writel(1<<3, regs + REG_OFFSET(SYSCU, SYCSRSTS5));
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> W: %08x = %08x (%s)\n",
				REG_ADDRESS(SYSCU, SYCSRST5),
				READL(SYSCU, SYCSRST5),
				((READL(SYSCU, SYCSRST5) & (1<<3)) ? "OK" : "NG"));
#endif
	iounmap(regs);
	release_mem_region(start, len);

	return 0;
}

static inline int init_endian_setting(struct usb_hcd *hcd)
{
	void __iomem *regs = NULL;
	u32 value;
	resource_size_t start = REG_START(USB_MST_BRG);
	resource_size_t len = REG_LENGTH(USB_MST_BRG);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call request_mem_region(0x%08x, 0x%x, \"%s\")\n",
				start, len, hcd->driver->description);
#endif
	if (request_mem_region(start, len,
						   hcd->driver->description) == NULL) {
		return -1;
	}
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ioremap_nocache(0x%08x, 0x%x)\n", start, len);
#endif
	regs = ioremap_nocache(start, len);
	if (regs == NULL) {
		release_mem_region(start, len);
		return -1;
	}

	/* bit1: DT  = 0 */
	/* bit0: NDT = 0 */
	value = READL(USB_MST_BRG, AMDR9) & ~((1<<1)|(1<<0));
	WRITEL(value, USB_MST_BRG, AMDR9);

	/* bit0: CDA = 0 */
	value = READL(USB_MST_BRG, CDAR9) & ~(1<<0);
	WRITEL(value, USB_MST_BRG, CDAR9);

	iounmap(regs);
	release_mem_region(start, len);

	return 0;
}

static inline int init_internal_buffer_setting(struct usb_hcd *hcd)
{
	void __iomem *regs = hcd->regs; /* WRITEL() needs "regs" */
	u32 value;

	check_ehci_default_values(regs);

	/* bit23-16: OUT_THRESHOLD = FFh */
	/* bit7-0  : IN_THRESHOLD  = 40h */
	value = (READL(EHCI, EIIBC1) & (0xFF00FF00)) | 0x00FF0040;
	WRITEL(value, EHCI, EIIBC1);

	/* bit0: BUFFER_ENABLE = 1 */
	value = READL(EHCI, EIIBC2) | (1<<0);
	WRITEL(value, EHCI, EIIBC2);

	return 0;
}

static inline int init_interrupt_setting(struct usb_hcd *hcd)
{
	void __iomem *regs = NULL;
	u32 value;
	resource_size_t start = REG_START(GLUE);
	resource_size_t len = REG_LENGTH(GLUE);
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call request_mem_region(0x%08x, 0x%x, \"%s\")\n",
				start, len, hcd->driver->description);
#endif
	if (request_mem_region(start, len,
						   hcd->driver->description) == NULL) {
		return -1;
	}
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	DEBUG_PRINT("<EP> call ioremap_nocache(0x%08x, 0x%x)\n", start, len);
#endif
	regs = ioremap_nocache(start, len);
	if (regs == NULL) {
		release_mem_region(start, len);
		return -1;
	}

	/* bit2: STBY = 1 */
	/* bit1: EHCI = 0 */
	/* bit0: OHCI = 1 */
	value = (READL(GLUE, UHIMR9) & ~(1<<1)) | ((1<<2)|(1<<0));
	WRITEL(value, GLUE, UHIMR9);

	iounmap(regs);
	release_mem_region(start, len);

	return 0;
}

static inline int epson09_ehci_init(struct usb_hcd *hcd)
{
	if ((init_usb_test(hcd) != 0) ||
		(init_phy_and_power_enable(hcd) != 0) ||
		(init_usb_host_module(hcd) != 0) ||
		(init_endian_setting(hcd) != 0) ||
		(init_internal_buffer_setting(hcd) != 0) ||
		(init_interrupt_setting(hcd) != 0)) {
		DEBUG_PRINT("<EP> failed\n");
		return -1;
	}

	return 0;
}
