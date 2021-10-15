/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 *  License : GPL2.0
 *
 * Base on linux/arch/arm/mach-omap2/timer-gp.c
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/time.h>
#include <asm/uaccess.h>
#include <asm/smp_twd.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/timex.h>

#include <linux/clockchips.h>

#include "epson12_timer.h"

extern void timer_tick(void);

static struct clock_event_device clockevent_gpt;

#if defined(CONFIG_EPSON12_TIMER_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

static irqreturn_t
epson12_timer_interrupt(int irq, void *dev_id)
{
        struct clock_event_device *evt = &clockevent_gpt;

#ifndef CONFIG_GENERIC_CLOCKEVENTS
	timer_tick();
#endif
	evt->event_handler(evt);
	
	/* Set CH4 Compare-A Register */
	writel(readl(TIMER_REGISTER_ADDRESS(TIMER_CNT_CH4)) + (CLOCK_TICK_RATE) / (HZ),
		TIMER_REGISTER_ADDRESS(TIMER_CMPA_CH4));
	/* Timer interrupt clear */
	writel(INTERRUPT_STATUS_INT_CLR, TIMER_REGISTER_ADDRESS(TIMER_INTCLR));

	return IRQ_HANDLED;
}

static struct irqaction epson12_timer_irq = {
	.name		= "SoC12 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,	/* IRQF_DISABLED = SA_INTERRUPT */
	.handler	= epson12_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 *
 * Currently we only use timer1, as it is the only timer which has no
 * other function that can be exploited externally
 */
static void epson12_timer_setup (void)
{
	/* Code for RTC 2013 */
	struct timespec tvi = { .tv_nsec = 0, .tv_sec = 1357023600 };
			    
        DBG("<EP> %s:start...\n", __FUNCTION__);

	do_settimeofday(&tvi);

        /* CH4 control clear */
	writel(0x00000000, TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
	/* CH4 counter clear */
	writel(0x00000000, TIMER_REGISTER_ADDRESS(TIMER_CNT_CH4));
	/* Timer interrupt clear */
	writel(INTERRUPT_STATUS_INT_CLR, TIMER_REGISTER_ADDRESS(TIMER_INTCLR));
#ifndef  CONFIG_EPSON12_WITH_RTOS
	/* Timer clock Control */
	writel(0x00000001, TIMER_CLKCTL);
#endif
	setup_irq(IRQ_TIMERU4, &epson12_timer_irq);

	/* Set CH4 Compare-A Register */
	writel((CLOCK_TICK_RATE) / (HZ), TIMER_REGISTER_ADDRESS(TIMER_CMPA_CH4));
	DBG("<EP> Set CMP %lu\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CMPA_CH4)));

	/* Start timer */
	DBG("<EP> CTL 0x%08x\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4)));
        writel(CONTROL_RUN | CONTROL_PSCSEL | CONTROL_CMPA_INTEN,
		TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
	DBG("<EP> CTL 0x%08x\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4)));
	DBG("<EP> %s:complete...\n", __FUNCTION__);
}

static cycle_t epson12_read_cycle_count(struct clocksource *cs)
{
	return (cycle_t)readl(TIMER_REGISTER_ADDRESS(TIMER_CNT_CH4));
}

static struct clocksource clocksource_gpt = {
	.name       = "gpt",
	.rating     = 300,
	.read       = epson12_read_cycle_count,
	.mask       = CLOCKSOURCE_MASK(32),
	.shift      = 24,
	.flags      = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init epson12_clocksource_init(void)
{
        DBG("<EP> %s\n", __FUNCTION__);

	clocksource_gpt.mult =
		clocksource_khz2mult(CLOCK_TICK_RATE / 1000, clocksource_gpt.shift);
	clocksource_register(&clocksource_gpt);
}

// static int epson12_timer_set_next_event(unsigned long cycles, struct clock_event_device *evt)
// {
//	return 0;
// }

static void epson12_timer_set_mode(enum clock_event_mode mode, struct clock_event_device *clk)
{
	DBG("<EP> %s\n  -> ", __FUNCTION__);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
	        DBG("CLOCK_EVT_MODE_PERIODIC\n");
		epson12_timer_setup();
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	        DBG("CLOCK_EVT_MODE_ONESHOT\n");
	        writel(CONTROL_RUN | CONTROL_PSCSEL, TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4));
	        DBG("<EP> CTL 0x%08x\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CTL_CH4)));
	        writel(0x00000000, TIMER_REGISTER_ADDRESS(TIMER_CMPA_CH4));
	        DBG("<EP> Set CMP %lu\n", readl(TIMER_REGISTER_ADDRESS(TIMER_CMPA_CH4)));
	        break;
	case CLOCK_EVT_MODE_UNUSED:
	        DBG("CLOCK_EVT_MODE_UNUSED\n");
	        break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	        DBG("CLOCK_EVT_MODE_SHUTDOWN\n");
	        break;
	case CLOCK_EVT_MODE_RESUME:
                DBG("CLOCK_EVT_MODE_RESUME\n");
	        break;
	}
}

static struct clock_event_device clockevent_gpt = {
	.name	  	= "gpt",
	.rating		= 300,
	.shift	  	= 32,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= epson12_timer_set_mode,
//	.set_next_event = epson12_timer_set_next_event,
	.cpumask	= cpu_all_mask,
};

static void __init epson12_clockevents_init(void)
{
        DBG("<EP> %s\n", __FUNCTION__);

	clockevent_gpt.irq = IRQ_TIMERU4;
	clockevent_gpt.mult =
		div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC, clockevent_gpt.shift);
	clockevent_gpt.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &clockevent_gpt);
	clockevent_gpt.min_delta_ns =
		clockevent_delta2ns(3, &clockevent_gpt);
	
	clockevents_register_device(&clockevent_gpt);
}

/*
 * Set up timer1 interrupt
 */
void __init epson12_timer_init (void)
{
	DBG("<EP> %s\n", __FUNCTION__);

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = __io_address(CORE_PM_BASE) + PTIMER_BASE;
#endif
        epson12_timer_setup();

        epson12_clocksource_init();
        epson12_clockevents_init();
}

struct sys_timer epson12_timer = {
	.init		= epson12_timer_init,
};
