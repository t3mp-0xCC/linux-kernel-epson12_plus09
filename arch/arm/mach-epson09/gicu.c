/*
 *  linux/arch/arm/mach-epson09/gicu.c
 *
 *   COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 *  License : GPL2.0
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/cpumask.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>

#include "gicu.h"

static unsigned long epson09_gicu_dist_base;
static DEFINE_SPINLOCK(irq_controller_lock);


/*
 * Routines to acknowledge, disable and enable interrupts
 *
 * Linux assumes that when we're done with an interrupt we need to
 * unmask it, in the same way we need to unmask an interrupt when
 * we first enable it.
 *
 * The GIC has a seperate notion of "end of interrupt" to re-enable
 * an interrupt after handling, in order to support hardware
 * prioritisation.
 *
 * We can make the GIC behave in the way that Linux expects by making
 * our "acknowledge" routine disable the interrupt, then mark it as
 * complete.
 */


static void epson09_gicu_mask_ack_irq(unsigned int irq)
{
	spin_lock(&irq_controller_lock);

	switch(irq) {
	case IRQ_EMG_STOP_FROM_CORE0:
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT100);
#else
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT59);
#endif
		writel(0xdeaddead, 0xe0c027f0);
		while(1);
		break;
	case IRQ_INT_ITIM_9_0:
		writel(ICCNT_INIT_ITIM_9_0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT73);
		break;
	case IRQ_SCIF0_RERR:
		writel(ICCNT_INIT_SCIF0_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT113);
		break;
	case IRQ_SCIF0_BRK:
		writel(ICCNT_INIT_SCIF0_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT114);
		break;
	case IRQ_SCIF0_RXI:
		writel(ICCNT_INIT_SCIF0_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT115);
		break;
	case IRQ_SCIF0_TXI:
		writel(ICCNT_INIT_SCIF0_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT116);
		break;
	case IRQ_SCIF1_RERR:
		writel(ICCNT_INIT_SCIF1_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT117);
		break;
	case IRQ_SCIF1_BRK:
		writel(ICCNT_INIT_SCIF1_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT118);
		break;
	case IRQ_SCIF1_RXI:
		writel(ICCNT_INIT_SCIF1_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT119);
		break;
	case IRQ_SCIF1_TXI:
		writel(ICCNT_INIT_SCIF1_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT120);
		break;
	case IRQ_SCIF2_RERR:
		writel(ICCNT_INIT_SCIF2_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT121);
		break;
	case IRQ_SCIF2_BRK:
		writel(ICCNT_INIT_SCIF2_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT122);
		break;
	case IRQ_SCIF2_RXI:
		writel(ICCNT_INIT_SCIF2_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT123);
		break;
	case IRQ_SCIF2_TXI:
		writel(ICCNT_INIT_SCIF2_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT124);
		break;
	case IRQ_XINT_PC0: /* XINT_PC0 */
		writel(ICCNT_INIT_XINT_PC0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT19);
		break;
	case IRQ_XINT_USW1: /* IDC */
		writel(ICCNT_INIT_XINT_USW1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT32);
		break;
	case IRQ_XINT_USBH1: /* XINT_USBH1 51 USB_Host_9 interrupt */
		writel(ICCNT_INIT_XINT_USBH1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT51);
		break;
	case IRQ_INT_ETHER: /* INT_ETHER 65 Ether DMAC interrupt */
		writel(ICCNT_INIT_INT_ETHER | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT65);
		break;
	case IRQ_INT_IPSEC: /* INT_IPSEC 66 IPsec interrupt */
		writel(ICCNT_INIT_INT_IPSEC | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT66);
		break;
	case IRQ_INT_WDT_9: /* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
		writel(ICCNT_INIT_INT_WDT_9 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT70);
		break;
	case IRQ_INT_ITIM_9_1: /* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
		writel(ICCNT_INIT_INT_ITIM_9_1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT74);
		break;
	case IRQ_GOBI_9_VOIDINT: /* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */
		writel(ICCNT_INIT_GOBI_9_VOIDINT | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT126);
		break;
	default:
		/* All register default */
		printk("<EP> epson09_gicu_ack_irq ICCNT(n=%d)\n", irq);
	}

	spin_unlock(&irq_controller_lock);
}


static void epson09_gicu_unmask_irq(unsigned int irq)
{
	spin_lock(&irq_controller_lock);

	switch(irq) {
	case IRQ_EMG_STOP_FROM_CORE0:
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT100);
#else
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT59);
#endif
		break;
	case IRQ_INT_ITIM_9_0:
		writel(ICCNT_INIT_ITIM_9_0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT73);
		break;
	case IRQ_SCIF0_RERR:
		writel(ICCNT_INIT_SCIF0_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT113);
		break;
	case IRQ_SCIF0_BRK:
		writel(ICCNT_INIT_SCIF0_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT114);
		break;
	case IRQ_SCIF0_RXI:
		writel(ICCNT_INIT_SCIF0_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT115);
		break;
	case IRQ_SCIF0_TXI:
		writel(ICCNT_INIT_SCIF0_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT116);
		break;
	case IRQ_SCIF1_RERR:
		writel(ICCNT_INIT_SCIF1_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT117);
		break;
	case IRQ_SCIF1_BRK:
		writel(ICCNT_INIT_SCIF1_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT118);
		break;
	case IRQ_SCIF1_RXI:
		writel(ICCNT_INIT_SCIF1_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT119);
		break;
	case IRQ_SCIF1_TXI:
		writel(ICCNT_INIT_SCIF1_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT120);
		break;
	case IRQ_SCIF2_RERR:
		writel(ICCNT_INIT_SCIF2_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT121);
		break;
	case IRQ_SCIF2_BRK:
		writel(ICCNT_INIT_SCIF2_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT122);
		break;
	case IRQ_SCIF2_RXI:
		writel(ICCNT_INIT_SCIF2_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT123);
		break;
	case IRQ_SCIF2_TXI:
		writel(ICCNT_INIT_SCIF2_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT124);
		break;
	case IRQ_XINT_PC0: /* XINT_PC0 */
		writel(ICCNT_INIT_XINT_PC0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT19);
		break;
	case IRQ_XINT_USW1: /* IDC */
		writel(ICCNT_INIT_XINT_USW1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT32);
		break;
	case IRQ_XINT_USBH1: /* XINT_USBH1 51 USB_Host_9 interrupt */
		writel(ICCNT_INIT_XINT_USBH1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT51);
		break;
	case IRQ_INT_ETHER: /* INT_ETHER 65 Ether DMAC interrupt */
		writel(ICCNT_INIT_INT_ETHER, epson09_gicu_dist_base + EPSON09_GICU_ICCNT65);
		break;
	case IRQ_INT_IPSEC: /* INT_IPSEC 66 IPsec interrupt */
		writel(ICCNT_INIT_INT_IPSEC, epson09_gicu_dist_base + EPSON09_GICU_ICCNT66);
		break;
	case IRQ_INT_WDT_9: /* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
		writel(ICCNT_INIT_INT_WDT_9, epson09_gicu_dist_base + EPSON09_GICU_ICCNT70);
		break;
	case IRQ_INT_ITIM_9_1: /* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
		writel(ICCNT_INIT_INT_ITIM_9_1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT74);
		break;
	case IRQ_GOBI_9_VOIDINT: /* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */
		writel(ICCNT_INIT_GOBI_9_VOIDINT, epson09_gicu_dist_base + EPSON09_GICU_ICCNT126);
		break;
	default:
		/* All register default */
		printk("<EP> epson09_gicu_ack_irq ICCNT(n=%d)\n", irq);
	}

	spin_unlock(&irq_controller_lock);
}


static void epson09_gicu_enable_irq(unsigned int irq)
{
	spin_lock(&irq_controller_lock);

	switch(irq) {
	case IRQ_EMG_STOP_FROM_CORE0:
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT100);
#else
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT59);
#endif
		break;
	case IRQ_INT_ITIM_9_0:
		writel(ICCNT_INIT_ITIM_9_0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT73);
		break;
	case IRQ_SCIF0_RERR:
		writel(ICCNT_INIT_SCIF0_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT113);
		break;
	case IRQ_SCIF0_BRK:
		writel(ICCNT_INIT_SCIF0_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT114);
		break;
	case IRQ_SCIF0_RXI:
		writel(ICCNT_INIT_SCIF0_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT115);
		break;
	case IRQ_SCIF0_TXI:
		writel(ICCNT_INIT_SCIF0_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT116);
		break;
	case IRQ_SCIF1_RERR:
		writel(ICCNT_INIT_SCIF1_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT117);
		break;
	case IRQ_SCIF1_BRK:
		writel(ICCNT_INIT_SCIF1_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT118);
		break;
	case IRQ_SCIF1_RXI:
		writel(ICCNT_INIT_SCIF1_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT119);
		break;
	case IRQ_SCIF1_TXI:
		writel(ICCNT_INIT_SCIF1_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT120);
		break;
	case IRQ_SCIF2_RERR:
		writel(ICCNT_INIT_SCIF2_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT121);
		break;
	case IRQ_SCIF2_BRK:
		writel(ICCNT_INIT_SCIF2_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT122);
		break;
	case IRQ_SCIF2_RXI:
		writel(ICCNT_INIT_SCIF2_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT123);
		break;
	case IRQ_SCIF2_TXI:
		writel(ICCNT_INIT_SCIF2_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT124);
		break;
	case IRQ_XINT_PC0: /* XINT_PC0 */
		writel(ICCNT_INIT_XINT_PC0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT19);
		break;
	case IRQ_XINT_USW1: /* IDC */
		writel(ICCNT_INIT_XINT_USW1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT32);
		break;
	case IRQ_XINT_USBH1: /* XINT_USBH1 51 USB_Host_9 interrupt */
		writel(ICCNT_INIT_XINT_USBH1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT51);
		break;
	case IRQ_INT_ETHER: /* INT_ETHER 65 Ether DMAC interrupt */
		writel(ICCNT_INIT_INT_ETHER, epson09_gicu_dist_base + EPSON09_GICU_ICCNT65);
		break;
	case IRQ_INT_IPSEC: /* INT_IPSEC 66 IPsec interrupt */
		writel(ICCNT_INIT_INT_IPSEC, epson09_gicu_dist_base + EPSON09_GICU_ICCNT66);
		break;
	case IRQ_INT_WDT_9: /* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
		writel(ICCNT_INIT_INT_WDT_9, epson09_gicu_dist_base + EPSON09_GICU_ICCNT70);
		break;
	case IRQ_INT_ITIM_9_1: /* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
		writel(ICCNT_INIT_INT_ITIM_9_1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT74);
		break;
	case IRQ_GOBI_9_VOIDINT: /* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */
		writel(ICCNT_INIT_GOBI_9_VOIDINT, epson09_gicu_dist_base + EPSON09_GICU_ICCNT126);
		break;
	default:
		/* All register default */
		printk("<EP> epson09_gicu_ack_irq ICCNT(n=%d)\n", irq);
	}
	
	spin_unlock(&irq_controller_lock);
}


static void epson09_gicu_disable_irq(unsigned int irq)
{
	spin_lock(&irq_controller_lock);

	switch(irq) {
	case IRQ_EMG_STOP_FROM_CORE0:
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT100);
#else
		writel(ICCNT_INIT_EMG_STOP_FROM_CORE0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT59);
#endif
		break;
	case IRQ_INT_ITIM_9_0:
		writel(ICCNT_INIT_ITIM_9_0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT73);
		break;
	case IRQ_SCIF0_RERR:
		writel(ICCNT_INIT_SCIF0_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT113);
		break;
	case IRQ_SCIF0_BRK:
		writel(ICCNT_INIT_SCIF0_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT114);
		break;
	case IRQ_SCIF0_RXI:
		writel(ICCNT_INIT_SCIF0_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT115);
		break;
	case IRQ_SCIF0_TXI:
		writel(ICCNT_INIT_SCIF0_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT116);
		break;
	case IRQ_SCIF1_RERR:
		writel(ICCNT_INIT_SCIF1_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT117);
		break;
	case IRQ_SCIF1_BRK:
		writel(ICCNT_INIT_SCIF1_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT118);
		break;
	case IRQ_SCIF1_RXI:
		writel(ICCNT_INIT_SCIF1_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT119);
		break;
	case IRQ_SCIF1_TXI:
		writel(ICCNT_INIT_SCIF1_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT120);
		break;
	case IRQ_SCIF2_RERR:
		writel(ICCNT_INIT_SCIF2_RERR | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT121);
		break;
	case IRQ_SCIF2_BRK:
		writel(ICCNT_INIT_SCIF2_BRK | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT122);
		break;
	case IRQ_SCIF2_RXI:
		writel(ICCNT_INIT_SCIF2_RXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT123);
		break;
	case IRQ_SCIF2_TXI:
		writel(ICCNT_INIT_SCIF2_TXI | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT124);
		break;
	case IRQ_XINT_PC0: /* XINT_PC0 */
		writel(ICCNT_INIT_XINT_PC0 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT19);
		break;
	case IRQ_XINT_USW1: /* IDC */
		writel(ICCNT_INIT_XINT_USW1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT32);
		break;
	case IRQ_XINT_USBH1: /* XINT_USBH1 51 USB_Host_9 interrupt */
		writel(ICCNT_INIT_XINT_USBH1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT51);
		break;
	case IRQ_INT_ETHER: /* INT_ETHER 65 Ether DMAC interrupt */
		writel(ICCNT_INIT_INT_ETHER | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT65);
		break;
	case IRQ_INT_IPSEC: /* INT_IPSEC 66 IPsec interrupt */
		writel(ICCNT_INIT_INT_IPSEC | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT66);
		break;
	case IRQ_INT_WDT_9: /* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
		writel(ICCNT_INIT_INT_WDT_9 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT70);
		break;
	case IRQ_INT_ITIM_9_1: /* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
		writel(ICCNT_INIT_INT_ITIM_9_1 | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT74);
		break;
	case IRQ_GOBI_9_VOIDINT: /* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */
		writel(ICCNT_INIT_GOBI_9_VOIDINT | ICCNT_INT_STOP, epson09_gicu_dist_base + EPSON09_GICU_ICCNT126);
		break;
	default:
		/* All register default */
		printk("<EP> epson09_gicu_ack_irq ICCNT(n=%d)\n", irq);
	}
	
	spin_unlock(&irq_controller_lock);
}



static struct irq_chip epson09_gicu_chip = {
	.name		= "GICU",
	.mask_ack	= epson09_gicu_mask_ack_irq,
	.unmask		= epson09_gicu_unmask_irq,
	.enable		= epson09_gicu_enable_irq,
	.disable	= epson09_gicu_disable_irq,
};


void __init epson09_gicu_dist_init(void __iomem *base)
{
	unsigned int max_irq, i;
#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
	printk("<EP> epson09_gicu_dist_init base = 0x%x\n", base);
#endif
	epson09_gicu_dist_base = base;

	/* mask level - all stop */
	writel(0x00000000, epson09_gicu_dist_base + EPSON09_GICU_ICIMSK);

	/* mask level - all start */
	for (i = 0; i < NR_IRQS; i++) {
		writel(ICCNT_INIT_DEFAULT, epson09_gicu_dist_base + EPSON09_GICU_ICCNT0 + (i * 4));
		writel(ICCNT_INT_CLEAR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT0 + (i * 4));
	}
	
	/* INT_EMG_STOP 59 Gobi_9 from core0 interrupt */
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
	writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT100);
#else
	writel(ICCNT_INIT_EMG_STOP_FROM_CORE0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT59);
#endif
	set_irq_chip(IRQ_EMG_STOP_FROM_CORE0, &epson09_gicu_chip);
	set_irq_handler(IRQ_EMG_STOP_FROM_CORE0, do_level_IRQ);
	set_irq_flags(IRQ_EMG_STOP_FROM_CORE0, IRQF_VALID | IRQF_PROBE);

	/* INT_ITIM_9_0 73 Gobi_9 ITIM2(ch0) interrupt */
	writel(ICCNT_INIT_ITIM_9_0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT73);
	set_irq_chip(IRQ_INT_ITIM_9_0, &epson09_gicu_chip);
	set_irq_handler(IRQ_INT_ITIM_9_0, do_level_IRQ);
	set_irq_flags(IRQ_INT_ITIM_9_0, IRQF_VALID | IRQF_PROBE);

	/* SCIF0_RERR 113 Receive Error Interrupt0 (SCIF ch0) */
	writel(ICCNT_INIT_SCIF0_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT113);	/* Low Level */
	set_irq_chip(IRQ_SCIF0_RERR, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF0_RERR, do_level_IRQ);
	set_irq_flags(IRQ_SCIF0_RERR, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_BRK 114 Break Interrupt0 (SCIF ch0) */
	writel(ICCNT_INIT_SCIF0_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT114);	/* Low Level */
	set_irq_chip(IRQ_SCIF0_BRK, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF0_BRK, do_level_IRQ);
	set_irq_flags(IRQ_SCIF0_BRK, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_RXI 115 Receive-FIFO Data-full Interrupt0 (SCIF ch0) */
	writel(ICCNT_INIT_SCIF0_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT115);	/* Low Level */
	set_irq_chip(IRQ_SCIF0_RXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF0_RXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF0_RXI, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_TXI 116 Transmitt-FIFO Data-empty Interrupt0 (SCIF ch0) */
	writel(ICCNT_INIT_SCIF0_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT116);	/* Low Level */
	set_irq_chip(IRQ_SCIF0_TXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF0_TXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF0_TXI, IRQF_VALID | IRQF_PROBE);

	/* SCIF0_RERR 117 Receive Error Interrupt1 (SCIF ch1) */
	writel(ICCNT_INIT_SCIF1_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT117);	/* Low Level */
	set_irq_chip(IRQ_SCIF1_RERR, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF1_RERR, do_level_IRQ);
	set_irq_flags(IRQ_SCIF1_RERR, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_BRK 118 Break Interrupt1 (SCIF ch1) */
	writel(ICCNT_INIT_SCIF1_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT118);	/* Low Level */
	set_irq_chip(IRQ_SCIF1_BRK, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF1_BRK, do_level_IRQ);
	set_irq_flags(IRQ_SCIF1_BRK, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_RXI 119 Receive-FIFO Data-full Interrupt1 (SCIF ch1) */
	writel(ICCNT_INIT_SCIF1_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT119);	/* Low Level */
	set_irq_chip(IRQ_SCIF1_RXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF1_RXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF1_RXI, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_TXI 120 Transmitt-FIFO Data-empty Interrupt1 (SCIF ch1) */
	writel(ICCNT_INIT_SCIF1_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT120);	/* Low Level */
	set_irq_chip(IRQ_SCIF1_TXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF1_TXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF1_TXI, IRQF_VALID | IRQF_PROBE);

	/* SCIF0_RERR 121 Receive Error Interrupt0 (SCIF ch2) */
	writel(ICCNT_INIT_SCIF2_RERR, epson09_gicu_dist_base + EPSON09_GICU_ICCNT121);	/* Low Level */
	set_irq_chip(IRQ_SCIF2_RERR, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF2_RERR, do_level_IRQ);
	set_irq_flags(IRQ_SCIF2_RERR, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_BRK 122 Break Interrupt0 (SCIF ch2) */
	writel(ICCNT_INIT_SCIF2_BRK, epson09_gicu_dist_base + EPSON09_GICU_ICCNT122);	/* Low Level */
	set_irq_chip(IRQ_SCIF2_BRK, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF2_BRK, do_level_IRQ);
	set_irq_flags(IRQ_SCIF2_BRK, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_RXI 123 Receive-FIFO Data-full Interrupt0 (SCIF ch2) */
	writel(ICCNT_INIT_SCIF2_RXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT123);	/* Low Level */
	set_irq_chip(IRQ_SCIF2_RXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF2_RXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF2_RXI, IRQF_VALID | IRQF_PROBE);
	
	/* SCIF0_TXI 124 Transmitt-FIFO Data-empty Interrupt0 (SCIF ch2) */
	writel(ICCNT_INIT_SCIF2_TXI, epson09_gicu_dist_base + EPSON09_GICU_ICCNT124);	/* Low Level */
	set_irq_chip(IRQ_SCIF2_TXI, &epson09_gicu_chip);
	set_irq_handler(IRQ_SCIF2_TXI, do_level_IRQ);
	set_irq_flags(IRQ_SCIF2_TXI, IRQF_VALID | IRQF_PROBE);

	/* IRQ_XINT_PC0 19 */
	writel(ICCNT_INIT_XINT_PC0, epson09_gicu_dist_base + EPSON09_GICU_ICCNT19);	/* Low Level */
	set_irq_chip(IRQ_XINT_PC0, &epson09_gicu_chip);
	set_irq_handler(IRQ_XINT_PC0, do_level_IRQ);
	set_irq_flags(IRQ_XINT_PC0, IRQF_VALID | IRQF_PROBE);

	/* IRQ_XINT_USW1 32 */
	writel(ICCNT_INIT_XINT_USW1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT32);	/* Low Level */
	set_irq_chip(IRQ_XINT_USW1, &epson09_gicu_chip);
	set_irq_handler(IRQ_XINT_USW1, do_level_IRQ);
	set_irq_flags(IRQ_XINT_USW1, IRQF_VALID | IRQF_PROBE);
#if 0
	/* ARM9_ACQCOMP 0 ARM11/9 core debug (ETM trace data trigger) */
	writel(0x00300000, epson09_gicu_dist_base + EPSON09_GICU_ICCNT0);	/* High Level */
	set_irq_chip(0, &epson09_gicu_chip);
	set_irq_handler(0, do_level_IRQ);
	set_irq_flags(0, IRQF_VALID | IRQF_PROBE);

	/* ARM9_FULL 1 ARM11/9 core debug (ETM FIFO Full) */
	writel(0x00300000, epson09_gicu_dist_base + EPSON09_GICU_ICCNT1);/* High Level */
	set_irq_chip(1, &epson09_gicu_chip);
	set_irq_handler(1, do_level_IRQ);
	set_irq_flags(1, IRQF_VALID | IRQF_PROBE);

	/* ARM9_COMMRX 2 ARM11/9 core debug (channel recive buffer empty) */
	writel(0x00300000, epson09_gicu_dist_base + EPSON09_GICU_ICCNT2);	/* High Level */
	set_irq_chip(2, &epson09_gicu_chip);
	set_irq_handler(2, do_level_IRQ);
	set_irq_flags(2, IRQF_VALID | IRQF_PROBE);

	/* ARM9_COMMTX 3 ARM11/9 core debug (channel send buffer empty) */
	writel(0x00300000, epson09_gicu_dist_base + EPSON09_GICU_ICCNT3);	/* High Level */
	set_irq_chip(3, &epson09_gicu_chip);
	set_irq_handler(3, do_level_IRQ);
	set_irq_flags(3, IRQF_VALID | IRQF_PROBE);
#endif
	/* XINT_USBH1 51 USB_Host_9 interrupt */
	writel(ICCNT_INIT_XINT_USBH1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT51);	/* Low Level */
	set_irq_chip(IRQ_XINT_USBH1, &epson09_gicu_chip);
	set_irq_handler(IRQ_XINT_USBH1, do_level_IRQ);
	set_irq_flags(IRQ_XINT_USBH1, IRQF_VALID | IRQF_PROBE);

	/* INT_ETHER 65 Ether DMAC interrupt */
	writel(ICCNT_INIT_INT_ETHER, epson09_gicu_dist_base + EPSON09_GICU_ICCNT65);	/* Low Level */
	set_irq_chip(IRQ_INT_ETHER, &epson09_gicu_chip);
	set_irq_handler(IRQ_INT_ETHER, do_level_IRQ);
	set_irq_flags(IRQ_INT_ETHER, IRQF_VALID | IRQF_PROBE);

	/* INT_IPSEC 66 IPsec interrupt */
	writel(ICCNT_INIT_INT_IPSEC, epson09_gicu_dist_base + EPSON09_GICU_ICCNT66);	/* Low Level */
	set_irq_chip(IRQ_INT_IPSEC, &epson09_gicu_chip);
	set_irq_handler(IRQ_INT_IPSEC, do_level_IRQ);
	set_irq_flags(IRQ_INT_IPSEC, IRQF_VALID | IRQF_PROBE);

	/* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
	writel(ICCNT_INIT_INT_WDT_9, epson09_gicu_dist_base + EPSON09_GICU_ICCNT70);	/* Low Level */
	set_irq_chip(IRQ_INT_WDT_9, &epson09_gicu_chip);
	set_irq_handler(IRQ_INT_WDT_9, do_level_IRQ);
	set_irq_flags(IRQ_INT_WDT_9, IRQF_VALID | IRQF_PROBE);
#if 0
	/* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
	writel(ICCNT_INIT_INT_ITIM_9_1, epson09_gicu_dist_base + EPSON09_GICU_ICCNT74);	/* Edge Down */
	set_irq_chip(IRQ_INT_ITIM_9_1, &epson09_gicu_chip);
	set_irq_handler(IRQ_INT_ITIM_9_1, do_level_IRQ);
	set_irq_flags(IRQ_INT_ITIM_9_1, IRQF_VALID | IRQF_PROBE);
	/* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */
	writel(ICCNT_INIT_GOBI_9_VOIDINT, epson09_gicu_dist_base + EPSON09_GICU_ICCNT126);	/* Edge Down */
	set_irq_chip(IRQ_GOBI_9_VOIDINT, &epson09_gicu_chip);
	set_irq_handler(IRQ_GOBI_9_VOIDINT, do_level_IRQ);
	set_irq_flags(IRQ_GOBI_9_VOIDINT, IRQF_VALID | IRQF_PROBE);
#endif
	writel(0x0F000000, base + EPSON09_GICU_ICIMSK);


	/* write scrachpad ready IRQ/FIQ status */
	writel(0xbeafbeaf, 0xe0c027f0);

	return;
}
