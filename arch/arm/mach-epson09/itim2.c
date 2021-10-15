/*
 * linux/arch/arm/mach/itim2.c
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 *  License : GPL2.0
 */
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/timex.h>

#include "itim2.h"

extern void timer_tick(void);

static unsigned long timer_reload;
static unsigned long timer_dummy_counter;

#if defined(CONFIG_EPSON09_TIMER_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

static unsigned long epson09_itim2_gettimeoffset(void)
{
	return timer_dummy_counter++;
}

static irqreturn_t
epson09_itim2_timer_interrupt(int irq, void *dev_id)
{
	for (; timer_dummy_counter >= 10000; timer_dummy_counter -= 10000) {
		DBG("interrupt %ld\n", timer_dummy_counter);
		timer_tick();
	}

	timer_tick();
	timer_dummy_counter = 0;

	return IRQ_HANDLED;
}


static struct irqaction epson09_itim2_timer_irq = {
	.name           = "SoC09S Timer Tick",
	.flags          = IRQF_DISABLED | IRQF_TIMER,	/* IRQF_DISABLED = SA_INTERRUPT */
	.handler        = epson09_itim2_timer_interrupt
};


/*
 * Set up timer interrupt
 */
void __init epson09_itim2_time_init (void)
{
	struct timespec tvi;

	/*Code for RTC*/
	tvi.tv_nsec = 0;
	tvi.tv_sec = 1200000000;

	DBG("<EP> %s:start\n",__FUNCTION__);

	do_settimeofday(&tvi);

	timer_reload = (CLOCK_TICK_RATE) / (HZ);
	DBG("<EP> timer_reload=%lu\n", timer_reload);
	timer_dummy_counter = 0;

	/* stop timer */
	writel(0x00000000, ITIM2_BASE_ADDR + ITIM2_ITCNT);

	writel(0x00030000, ITIM2_BASE_ADDR + ITIM2_ITSRC);
	writel(timer_reload, ITIM2_BASE_ADDR + ITIM2_ITRLD);

	setup_irq(IRQ_INT_ITIM_9_0, &epson09_itim2_timer_irq);

	/* start timer */
	writel(0x00010000, ITIM2_BASE_ADDR + ITIM2_ITCNT);
	DBG("<EP> %s:complete\n",__FUNCTION__);
}


struct sys_timer epson09_itim2_timer = {
	.init		= epson09_itim2_time_init,
	.offset		= epson09_itim2_gettimeoffset,
};

