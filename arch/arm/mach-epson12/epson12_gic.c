/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>

#include <linux/irq.h>

#include <mach/gic.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/hardware/gic.h>

#include "epson12_plat.h"

#if defined(CONFIG_MACH_EPSON12_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

/* EPSON12 gic distributor configuration */
void arch_gic_dist_config(void __iomem *base)
{
	/* TIMERU4 is edge */
#if defined(CONFIG_MACH_EPSON12_M)
	writel(0x00000200, base + GIC_DIST_CONFIG + 112 * 4 / 16);
#elif defined(CONFIG_MACH_EPSON12_H)
	/* */
#endif
}

/* EPSON12 gic distributor initialize */
void arch_gic_dist_init(void __iomem *base)
{
	/* list up all interrupts */
	unsigned int i;

	for (i = 32; i < 160; i += 4)
		writel(0x02020202, base + GIC_DIST_TARGET + i * 4 / 4);
}

/* EPSON12 MISC initialize */
void epson12_misc_init(void)
{
	u32 readmask = 0;
	u32 readmask2 = 0;
	void __iomem *base;
	
	DBG("----------------START-EP-MISC-INIT------------------\n");
	
	/* MISC HW initialize */
	base = __io_address(MISC_WH_BASE);

#if defined(CONFIG_MACH_EPSON12_M)
	readmask = readl(base + MISC_WH_ISET_MSK1);
	DBG("<EP> read iset_mask1(WH63-32)=%08x\n", readmask);
	readmask &= 0xFFFEFFFF; // WH48(SPI68) TIMERU4
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_WH_INT_MSK1);
	readmask2 = readl(base + MISC_WH_ISET_MSK1);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#elif defined(CONFIG_MACH_EPSON12_H)
        readmask = readl(base + MISC_WH_ISET_MSK0);
	DBG("<EP> read iset_mask0(WH31-0)=%08x\n", readmask);
	readmask &= 0xFFFFFDFF; // WH9(SPI49) TIMERU4
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_WH_INT_MSK0);
	readmask2 = readl(base + MISC_WH_ISET_MSK0);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#endif

#if 0 /* Not used in FY12, has left for the test */
#ifdef CONFIG_MACH_EPSON12_M
	readmask = readl(base + MISC_WH_ISET_MSK2);
	DBG("<EP> read iset_mask2(WH78-64)=%08x\n", readmask);
	readmask &= 0xFFFFAFFF; // WH78(SPI98) CPU2->1
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_WH_INT_MSK2);
	readmask2 = readl(base + MISC_WH_ISET_MSK2);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#endif
	
	readmask = readl(base + MISC_WH_IMASK_CPU21);
	DBG("<EP> read imask_cpu21=%08x\n", readmask);
	readmask &= 0x00000000; // CPU2->1 interrupt
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_WH_IMASK_CPU21);
	readmask2 = readl(base + MISC_WH_IMASK_CPU21);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#endif
	
	/* EPSON12 BK initialize */
	base = __io_address(MISC_BK_BASE);
	
#ifdef CONFIG_MACH_EPSON12_M
	readmask = readl(base + MISC_BK_ISET_MSK);
	DBG("<EP> read iset_mask(BK19-0)=%08x\n", readmask);
	readmask &= 0xFFFEFFFF; // BK16(SPI16) CPU0->1
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_BK_INT_MSK);
	readmask2 = readl(base + MISC_BK_ISET_MSK);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#endif

#ifdef MISC_TEST /* The mask set be done on the IDC driver */
	readmask = readl(base + MISC_BK_IMASK_CPU01);
	DBG("<EP> read imask_cpu01=%08x\n", readmask);
	readmask &= 0x00000000; // CPU0->1 interrupt
	DBG("<EP> set  mask       =%08x\n", readmask);
	writel(readmask, base + MISC_BK_IMASK_CPU01);
	readmask2 = readl(base + MISC_BK_IMASK_CPU01);
	DBG("<EP> setedmask       =%08x\n", readmask2);
#endif
	
	DBG("-----------------END-EP-MISC-INIT-------------------\n");
}

#ifdef MISC_TEST
/* EPSON12 MISC BK/WH test code */
static irqreturn_t
misc_bk_interrupt(int irq, void *dev_id)
{
	DBG("<EP> IRQ=%d MISC BK CPU 0->1 interrupt OK\n", irq);
	writel(0x00, IO_ADDRESS(MISC_BK_BASE) + MISC_BK_INT_CPU01);
	return IRQ_HANDLED;
}

static struct irqaction misc_bk_irq = {
	.name		= "SoC12 Misc BK",
	.flags		= IRQF_DISABLED,	/* IRQF_DISABLED = SA_INTERRUPT */
	.handler	= misc_bk_interrupt,
};

#if 0 /* Not used in FY12, has left for the test */
static irqreturn_t
misc_wh_interrupt(int irq, void *dev_id)
{
	DBG("<EP> IRQ=%d MISC WH CPU 2->1 interrupt OK\n", irq);
	writel(0x00, IO_ADDRESS(MISC_WH_BASE) + MISC_WH_INT_CPU21);
	return IRQ_HANDLED;
}

static struct irqaction misc_wh_irq = {
	.name		= "SoC12 Misc WH",
	.flags		= IRQF_DISABLED,	/* IRQF_DISABLED = SA_INTERRUPT */
	.handler	= misc_wh_interrupt,
};
#endif

void epson12_misc_test(void)
{
	/* Setup IRQs */
#if defined(CONFIG_MACH_EPSON12_M)
	setup_irq(IRQ_MISC_BK16, &misc_bk_irq);
#elif defined(CONFIG_MACH_EPSON12_H)
        setup_irq(IRQ_MISC_BK36, &misc_bk_irq);
#endif

	DBG("<EP> misc bk core interrupt state =%08x\n",	readl(IO_ADDRESS(MISC_BK_BASE) + MISC_BK_ISET_CPU01));
#ifdef CONFIG_MACH_EPSON12_M
	DBG("<EP> misc bk interrupt state =%08x\n",		readl(IO_ADDRESS(MISC_BK_BASE) + MISC_BK_INT_ST));
#endif
	DBG("<EP> mics bk core interrupt mask state =%08x\n",	readl(IO_ADDRESS(MISC_BK_BASE) + MISC_BK_IMASK_CPU01));
	writel(0x01,						IO_ADDRESS(MISC_BK_BASE) + MISC_BK_INT_CPU01);
#ifdef CONFIG_MACH_EPSON12_M
	DBG("<EP> misc bk interrupt state =%08x\n",		readl(IO_ADDRESS(MISC_BK_BASE) + MISC_BK_INT_ST));
#endif
	DBG("<EP> mics bk core interrupt state =%08x\n",	readl(IO_ADDRESS(MISC_BK_BASE) + MISC_BK_ISET_CPU01));

#if 0 /* Not used in FY12, has left for the test */
#if defined(CONFIG_MACH_EPSON12_M)
        setup_irq(IRQ_MISC_WH78, &misc_wh_irq);
#elif defined(CONFIG_MACH_EPSON12_H)
	setup_irq(IRQ_MISC_WH93, &misc_wh_irq);
#endif
        
        DBG("<EP> misc wh core interrupt state =%08x\n",	readl(IO_ADDRESS(MISC_WH_BASE) + MISC_WH_ISET_CPU21));
#ifdef CONFIG_MACH_EPSON12_M
        DBG("<EP> misc wh interrupt state =%08x\n",		readl(IO_ADDRESS(MISC_WH_BASE) + MISC_WH_INT_ST2));
#endif
        DBG("<EP> mics wh core interrupt mask state =%08x\n",	readl(IO_ADDRESS(MISC_WH_BASE) + MISC_WH_IMASK_CPU21));
        writel(0x01,						IO_ADDRESS(MISC_WH_BASE) + MISC_WH_INT_CPU21);
#ifdef CONFIG_MACH_EPSON12_M
        DBG("<EP> misc wh interrupt state =%08x\n",		readl(IO_ADDRESS(MISC_WH_BASE) + MISC_WH_INT_ST2));
#endif
        DBG("<EP> mics wh core interrupt state =%08x\n",	readl(IO_ADDRESS(MISC_WH_BASE) + MISC_WH_ISET_CPU21));
#endif
}
#endif
